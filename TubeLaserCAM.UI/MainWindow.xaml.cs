using System;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using TubeLaserCAM.UI.ViewModels;
// XÓA dòng: using TubeLaserCAM.UI.Controls;

namespace TubeLaserCAM.UI
{
    public partial class MainWindow : Window
    {
        private MainViewModel viewModel;
        private Point lastMousePosition;
        private bool isRotating = false;

        public MainWindow()
        {
            InitializeComponent();
            viewModel = new MainViewModel();
            DataContext = viewModel;

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
            // Run on background thread to avoid blocking UI
            Task.Run(() =>
            {
                Application.Current.Dispatcher.BeginInvoke(new Action(() =>
                {
                    if (viewModel?.MouseMoveCommand?.CanExecute(e) == true)
                    {
                        viewModel.MouseMoveCommand.Execute(e);
                    }
                }), System.Windows.Threading.DispatcherPriority.Background);
            });
        }

        private void Viewport3D_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                // Check for modifier keys
                bool hasModifier = Keyboard.IsKeyDown(Key.LeftCtrl) ||
                                  Keyboard.IsKeyDown(Key.RightCtrl) ||
                                  Keyboard.IsKeyDown(Key.LeftShift) ||
                                  Keyboard.IsKeyDown(Key.RightShift) ||
                                  Keyboard.IsKeyDown(Key.LeftAlt) ||
                                  Keyboard.IsKeyDown(Key.RightAlt);

                if (!hasModifier)
                {
                    // Handle edge selection
                    if (viewModel?.MouseDownCommand?.CanExecute(e) == true)
                    {
                        viewModel.MouseDownCommand.Execute(e);
                        e.Handled = true; // Prevent camera movement
                    }
                }
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