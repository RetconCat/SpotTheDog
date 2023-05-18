using SpotTheDog.MovingLightModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Numerics;

namespace SpotTheDog
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        MovingLight light;
        public MainWindow()
        {
            InitializeComponent();
            light = new MovingLight(new Vector3(0,0,0), new Quaternion(0,0,0,1));
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            AngleMathHelpers.TestCalibrationMethod(int.Parse(Status.Text), (float)ZSlide.Value);
        }

        private void ZSlide_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            ZSlide.Value = 0;
        }
    }
}
