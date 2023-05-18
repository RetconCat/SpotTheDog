using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Animation;

namespace SpotTheDog.MovingLightModel
{
    internal class MovingLight
    {
        const double Nintey = Math.PI / 2;
        //Defining P/T parameters
        double panAngle;
        double tiltAngle;
        Dictionary<string, int> patch;
        double panRange;
        double tiltRange;
        double PanMax { get => panRange / 2; }
        double PanMin { get => -panRange / 2; }
        double TiltMax { get => tiltRange / 2; }
        double TiltMin { get => -tiltRange / 2; }
        UInt16 panRaw;
        UInt16 tiltRaw;
        ForwardDirection ForwardDirection;
        //Defining spatial stuff
        Vector3 Position;
        Quaternion Rotation;

        public MovingLight(Vector3 pos, Quaternion rot) 
        { 
            Position = pos;
            Rotation = rot;
            ForwardDirection = ForwardDirection.Front;
        }

        void CalcRawValuesFromAngles()
        {
            tiltRaw = (UInt16)(((tiltAngle + TiltMax)/tiltRange) * UInt16.MaxValue);
            panRaw = (UInt16)(((panAngle + PanMax)/panRange) * UInt16.MaxValue);
        }
        
        public (double U, double V) PtToUV(double pan, double tilt)
        {
            double u = (Math.Sin(pan) * Math.Sin(tilt)) / Math.Cos(tilt);
            double v = (Math.Cos(pan)  * Math.Sin(tilt)) / Math.Cos(tilt);
            return (u, v);
        }

        public (double pan, double tilt) Target3DPoint(Vector3 TargetPoint)
        {
            //Get vector from position to target and normalize
            Vector3 orientation = TargetPoint - Position;
            Debug.WriteLine($"Target     : {orientation.X},{orientation.Y},{orientation.Z}");
            orientation = Vector3.Normalize(orientation);
            Debug.WriteLine($"Normalized : {orientation.X},{orientation.Y},{orientation.Z}");
            //rotate orientation by the opposite of current fixture rotation (negate W component of quaternion)
            Quaternion rot = Rotation;
            orientation = AngleMathHelpers.RotatePoint(orientation, rot);
            Debug.WriteLine($"Rotated    : {orientation.X},{orientation.Y},{orientation.Z}\n");
            //find pan angle by tan of x,y coord (pan is around Z axis)
            double theta = 0;
            double phi = 0;
            switch (ForwardDirection)
            {
                case ForwardDirection.Back:
                    theta = Math.Atan2(orientation.X, -orientation.Y);
                    phi = Nintey-Math.Asin(orientation.Z);
                    break;
                case ForwardDirection.Front://-Y axis represents theta = 0
                    theta = Math.Atan2(-orientation.X, orientation.Y);
                    phi = -(Nintey + Math.Asin(orientation.Z));
                    break;
                case ForwardDirection.Left:
                    theta = -Math.Atan2(-orientation.Y, -orientation.X);
                    theta += Nintey;
                    phi = -(Nintey + Math.Asin(orientation.Z));
                    break;
                case ForwardDirection.Right:
                    theta = -Math.Atan2(orientation.Y, orientation.X);
                    theta -= Nintey;
                    phi = -(Nintey + Math.Asin(orientation.Z));
                    break;
            }          
            return (theta, phi);
        }

        public (Vector3 pos, Quaternion rot) Calibrate(List<Vector3> targetPoints, List<double> pans, List<double> tilts)
        {
            //Notes on geometry:
            //OpenCV expects +Z to be + distance from image plane, so our default orientation for MLs is "upside down"
            // bark bark
            Vector3 pos = new Vector3();
            Quaternion rot = new Quaternion();
            double[,] cam = AngleMathHelpers.GuessCameraMatrix(); 
            List<Point3f> points = new List<Point3f>();
            List<Point2f> coords = new List<Point2f>();
            for(int i = 0; i < pans.Count; i++)
            {
                points.Add(new((float)targetPoints[i].X, (float)targetPoints[i].Y, (float)targetPoints[i].Z));
                (double u, double v) = PtToUV(pans[i], tilts[i]);
                coords.Add(new Point2f((float)u,(float)v));
                Debug.WriteLine($"point: {targetPoints[i].X:F4},{targetPoints[i].Y:F4},{targetPoints[i].Z:F4}   P/T:{pans[i]*180/Math.PI:F4},{tilts[i] * 180 / Math.PI:F4}    U,V: {u:F4},{v:F4}");
            }
            double[] rvecs = new double[3];
            double[] tvecs = new double[3];
            //Data set up, now do the actual calculation
            Cv2.SolvePnP(points, coords, cam, null, ref rvecs, ref tvecs);
            //Should user RANSAC method for reliability but right now it's fucky
            //Cv2.SolvePnPRansac(points, coords, cam, new double[0], out rvecs, out tvecs);
            
            //Convert weird rotation vector (cause wheeee open CV doesn't fucking document the order) to a rotation matrix
            double[,] rmatrix = new double[3, 3];
            double[,] jacobian = new double[3, 3];
            Cv2.Rodrigues(rvecs,out rmatrix, out jacobian);
            Matrix4x4 rotMat = new Matrix4x4((float)rmatrix[0, 0], (float)rmatrix[0, 1], (float)rmatrix[0, 2], 0, (float)rmatrix[1, 0], (float)rmatrix[1, 1], (float)rmatrix[1, 2], 0, (float)rmatrix[2, 0], (float)rmatrix[2, 1], (float)rmatrix[2, 2], 0,0,0,0,1);
            
            //turn that matrix into a quaternion
            Quaternion EndRotation = Quaternion.CreateFromRotationMatrix(rotMat);
            //Rotate that quaternion 180 to match our world coords
            //Cv2.Rodrigues(new double[] { 0, 0, Math.PI }, out rmatrix, out jacobian);
            //rotMat = new Matrix4x4((float)rmatrix[0, 0], (float)rmatrix[0, 1], (float)rmatrix[0, 2], 0, (float)rmatrix[1, 0], (float)rmatrix[1, 1], (float)rmatrix[1, 2], 0, (float)rmatrix[2, 0], (float)rmatrix[2, 1], (float)rmatrix[2, 2], 0,0,0,0,1);
            //Quaternion Flip180 = Quaternion.CreateFromRotationMatrix(rotMat);
            //EndRotation = Flip180 * EndRotation * Quaternion.Inverse(Flip180);
            
            Debug.WriteLine($"{EndRotation.ToString()}");
            Vector3 calibratedLocation = new Vector3((float)tvecs[0], (float)tvecs[1], (float)tvecs[2]);
            calibratedLocation = AngleMathHelpers.RotatePoint(calibratedLocation, Quaternion.Inverse(EndRotation));
            calibratedLocation.X = -calibratedLocation.X;
            calibratedLocation.Y = -calibratedLocation.Y;
            calibratedLocation.Z = -calibratedLocation.Z;
            Debug.WriteLine($"Location = {calibratedLocation.X:F4},{calibratedLocation.Y:F4},{calibratedLocation.Z:F4}\nRotation = {rvecs[0]:F4},{rvecs[1]:F4},{rvecs[2]:F4}");
            return (pos,rot);
        }

    }

    public enum ForwardDirection
    {
        Front,
        Back,
        Left,
        Right,
    }

}
