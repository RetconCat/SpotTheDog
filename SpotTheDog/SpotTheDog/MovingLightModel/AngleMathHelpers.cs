using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using System.Xaml.Schema;
using OpenCvSharp;

namespace SpotTheDog.MovingLightModel
{
    internal class AngleMathHelpers
    {

        public static double[,] GuessCameraMatrix()
        {
            double[,] output =  //if we use a plane 1m from the light as the focal plane and just mirror the coordinates, that should give us a good X,Y matrix
            {                   //And that should give us an identiy matrix as our intrinsic matrix
                {1,0,0},
                {0,1,0},
                {0,0,1}
            };
            return output;
        }

        public static void TestPointRotation(Vector3 point, float yaw, float pitch, float roll)
        {

        }

        


        public static void TestCalibrationMethod(float z, float thetaScale)
        {
            double theta = Math.PI * thetaScale;


            MovingLight light = new MovingLight(new Vector3(50, -15, 25), Quaternion.CreateFromAxisAngle(new Vector3(0,1,0),(float)theta));
            List<Vector3> TargetPoints = new List<Vector3>();
            List<double> pans = new List<double>();
            List<double> tilts = new List<double>();
            TargetPoints.Add(new Vector3(-8,12,z));
            TargetPoints.Add(new Vector3(8,12,z));
            TargetPoints.Add(new Vector3(-8,0,z));
            TargetPoints.Add(new Vector3(8,0,z));
            foreach(var point in TargetPoints)
            {
                (double p, double t) = light.Target3DPoint(point);
                pans.Add(p);
                tilts.Add(t);
            }
            light.Calibrate(TargetPoints, pans,tilts);
        }

        private static List<Vector3> GeneratePointList(int column, int row, float spacing, Vector3 offset, Quaternion rotation)
        {
            List<Vector3> points = new List<Vector3>();
            for(int i = 0; i < column; i++)
            {
                for(int j = 0; j < row; j++)
                {
                    Vector3 point = new Vector3(i * spacing, j * spacing, 0);
                    RotatePointInternal(ref point, rotation);
                    point += offset;
                    points.Add(point);
                }
            }
            return points;
        }

        public static Vector3 RotatePoint(Vector3 point, Quaternion rotation) 
        {   
            Quaternion answer = Quaternion.Inverse(rotation) * new Quaternion(point,0) * rotation;
            return new Vector3(answer.X, answer.Y, answer.Z);
        }
        private static void RotatePointInternal(ref Vector3 point, Quaternion rotation)
        {
            Quaternion answer = rotation * new Quaternion(point, 0) * Quaternion.Inverse(rotation);
            point.X = answer.X;
            point.Y = answer.Y;
            point.Z = answer.Z;
        }
    }
}
