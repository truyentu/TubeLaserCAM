using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using TubeLaserCAM.UI.ViewModels;
// XÓA dòng: using TubeLaserCAM.UI.Controls;

namespace TubeLaserCAM.UI
{
    public partial class MainWindow : Window
    {
        private Point lastMousePosition;
        private bool isRotating = false;

        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainViewModel();

            // Set default camera position
            SetFrontView();
        }

        private void SetFrontView_Click(object sender, RoutedEventArgs e)
        {
            SetFrontView();
        }

        private void SetFrontView()
        {
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                // Thử isometric view trước
                camera.Position = new Point3D(300, 300, 300);
                camera.LookDirection = new Vector3D(-1, -1, -1);
                camera.UpDirection = new Vector3D(0, 0, 1); // Z up
                camera.FieldOfView = 45;
            }
        }

        private void SetTopView_Click(object sender, RoutedEventArgs e)
        {
            // Camera nhìn từ trên xuống (dọc theo Z)
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                camera.Position = new Point3D(0, 0, 300);
                camera.LookDirection = new Vector3D(0, 0, -1);
                camera.UpDirection = new Vector3D(0, 1, 0); // Y forward
                camera.FieldOfView = 45;
            }
        }

        private void SetSideView_Click(object sender, RoutedEventArgs e)
        {
            // Camera nhìn từ bên (dọc theo Y)
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                camera.Position = new Point3D(0, 300, 0);
                camera.LookDirection = new Vector3D(0, -1, 0);
                camera.UpDirection = new Vector3D(0, 0, 1); // Z up
                camera.FieldOfView = 45;
            }
        }

        private void Viewport3D_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            var vm = DataContext as MainViewModel;
            if (vm == null || !vm.IsCameraLocked) return;

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                var currentPos = e.GetPosition(viewport3D);

                if (!isRotating)
                {
                    lastMousePosition = currentPos;
                    isRotating = true;
                    return;
                }

                // Calculate rotation angle around Z axis (thay vì Y)
                double deltaX = currentPos.X - lastMousePosition.X;
                double rotationAngle = deltaX * 0.5; // Sensitivity

                // Rotate camera around Z axis
                RotateCameraAroundZAxis(rotationAngle); // ĐỔI TÊN FUNCTION

                lastMousePosition = currentPos;
                e.Handled = true; // Prevent default camera control
            }
            else
            {
                isRotating = false;
            }
        }

        private void RotateCameraAroundZAxis(double angle)
        {
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera == null) return;

            // Rotation around Z axis tại origin
            var axisAngleRotation = new AxisAngleRotation3D(
                new Vector3D(0, 0, 1), // Z axis
                angle
            );
            var rotateTransform = new RotateTransform3D(axisAngleRotation);

            // Apply rotation
            camera.Position = rotateTransform.Transform(camera.Position);

            // Look at origin
            camera.LookDirection = new Point3D(0, 0, 0) - camera.Position;

            // QUAN TRỌNG: Giữ Z up
            camera.UpDirection = new Vector3D(0, 0, 1);
        }
    }
}