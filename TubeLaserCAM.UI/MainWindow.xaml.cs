using System;
using System.ComponentModel;
using System.Collections.Specialized;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using TubeLaserCAM.UI.ViewModels;
using System.Windows.Media;

namespace TubeLaserCAM.UI
{
    public partial class MainWindow : Window
    {
        private MainViewModel viewModel;
        // Removed unused fields

        // Store references for add/remove
        private Model3DGroup staticModelsGroup;
        private ModelVisual3D staticModelsVisual;
        private ModelVisual3D simulationModelsVisual;

        public MainWindow()
        {
            InitializeComponent();
            viewModel = new MainViewModel();
            DataContext = viewModel;

            // Initialize visual containers
            InitializeVisualContainers();

            // Subscribe to property changes
            viewModel.PropertyChanged += ViewModel_PropertyChanged;

            // Set default camera position
            SetFrontView();
        }

        private void InitializeVisualContainers()
        {
            // Create static models visual
            staticModelsGroup = new Model3DGroup();
            staticModelsVisual = new ModelVisual3D
            {
                Content = staticModelsGroup
            };

            // Create simulation models visual
            simulationModelsVisual = new ModelVisual3D();

            // Initially show static models
            viewport3D.Children.Add(staticModelsVisual);
        }

        private void ViewModel_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(MainViewModel.IsSimulation3DActive))
            {
                UpdateViewportContent();
            }
            else if (e.PropertyName == nameof(MainViewModel.WireframeModel) ||
                     e.PropertyName == nameof(MainViewModel.AxisModel))
            {
                UpdateStaticModels();
            }
        }

        private void UpdateStaticModels()
        {
            staticModelsGroup.Children.Clear();

            if (viewModel.WireframeModel != null)
            {
                foreach (var child in viewModel.WireframeModel.Children)
                {
                    staticModelsGroup.Children.Add(child);
                }
            }

            if (viewModel.AxisModel != null && viewModel.ShowAxis)
            {
                foreach (var child in viewModel.AxisModel.Children)
                {
                    staticModelsGroup.Children.Add(child);
                }
            }
        }

        private void UpdateViewportContent()
        {
            if (viewModel.IsSimulation3DActive)
            {
                SwitchToSimulationMode();
            }
            else
            {
                SwitchToStaticMode();
            }
        }

        private void SwitchToSimulationMode()
        {
            System.Diagnostics.Debug.WriteLine("\n=== SWITCH TO SIMULATION MODE ===");

            // Remove static models
            if (viewport3D.Children.Contains(staticModelsVisual))
            {
                viewport3D.Children.Remove(staticModelsVisual);
                System.Diagnostics.Debug.WriteLine("Removed static models visual");
            }

            // Clear and setup simulation container
            System.Diagnostics.Debug.WriteLine($"Clearing simulation visual. Current children: {simulationModelsVisual.Children.Count}");
            simulationModelsVisual.Children.Clear();

            // QUAN TRỌNG: Load wireframe lên tube TRƯỚC KHI add visual
            if (viewModel.WireframeModel != null && viewModel.Sim3DViewModel != null)
            {
                System.Diagnostics.Debug.WriteLine("Loading wireframe on tube...");
                System.Diagnostics.Debug.WriteLine($"Original wireframe has {viewModel.WireframeModel.Children.Count} children");

                viewModel.Sim3DViewModel.LoadWireframeOnTube(viewModel.WireframeModel);
                viewModel.Sim3DViewModel.DebugWireframeDirection();
            }

            // Create a single container for tube và wireframe để chúng transform cùng nhau
            var tubeAndWireframeContainer = new ModelVisual3D();
            var containerGroup = new Model3DGroup();
            int itemsInContainer = 0;

            // Add tube geometry to container
            if (viewModel.Sim3DViewModel?.TubeGeometry != null)
            {
                containerGroup.Children.Add(viewModel.Sim3DViewModel.TubeGeometry);
                itemsInContainer++;
                System.Diagnostics.Debug.WriteLine($"Added tube geometry to container. Total items: {itemsInContainer}");

                // Debug tube material
                var tubeMat = viewModel.Sim3DViewModel.TubeGeometry.Material;
                System.Diagnostics.Debug.WriteLine($"Tube material: {tubeMat?.GetType().Name}");
            }

            // Add wireframe to THE SAME container
            if (viewModel.Sim3DViewModel?.WireframeOnTubeGroup != null)
            {
                // QUAN TRỌNG: Đảm bảo wireframe không có transform riêng
                viewModel.Sim3DViewModel.WireframeOnTubeGroup.Transform = Transform3D.Identity;

                // Debug wireframe group
                System.Diagnostics.Debug.WriteLine($"WireframeOnTubeGroup has {viewModel.Sim3DViewModel.WireframeOnTubeGroup.Children.Count} children");

                // Add tất cả children của wireframe vào container group
                foreach (var child in viewModel.Sim3DViewModel.WireframeOnTubeGroup.Children)
                {
                    containerGroup.Children.Add(child);
                    itemsInContainer++;

                    // Debug each wireframe element
                    if (child is GeometryModel3D gm)
                    {
                        var mat = gm.Material;
                        System.Diagnostics.Debug.WriteLine($"  Added wireframe element. Material: {mat?.GetType().Name}");

                        // Check material color
                        if (mat is MaterialGroup mg)
                        {
                            foreach (var subMat in mg.Children)
                            {
                                if (subMat is DiffuseMaterial dm && dm.Brush is SolidColorBrush scb)
                                {
                                    System.Diagnostics.Debug.WriteLine($"    Material color: {scb.Color}");
                                }
                            }
                        }
                        else if (mat is DiffuseMaterial dm && dm.Brush is SolidColorBrush scb)
                        {
                            System.Diagnostics.Debug.WriteLine($"    Material color: {scb.Color}");
                        }
                    }
                }

                System.Diagnostics.Debug.WriteLine($"Added {viewModel.Sim3DViewModel.WireframeOnTubeGroup.Children.Count} wireframe elements. Total in container: {itemsInContainer}");
            }

            // Set content và transform cho container
            tubeAndWireframeContainer.Content = containerGroup;
            System.Diagnostics.Debug.WriteLine($"Container group total children: {containerGroup.Children.Count}");

            // QUAN TRỌNG: Bind transform để tự động update
            var transformBinding = new System.Windows.Data.Binding("TubeTransform")
            {
                Source = viewModel.Sim3DViewModel,
                Mode = System.Windows.Data.BindingMode.OneWay
            };
            System.Windows.Data.BindingOperations.SetBinding(
                tubeAndWireframeContainer,
                ModelVisual3D.TransformProperty,
                transformBinding
            );

            // Add container to simulation visual
            simulationModelsVisual.Children.Add(tubeAndWireframeContainer);
            System.Diagnostics.Debug.WriteLine("Added tube/wireframe container to simulation visual");

            // Add cutter model (riêng biệt, không trong container)
            if (viewModel.Sim3DViewModel?.CutterGeometry != null)
            {
                var cutterVisual = new ModelVisual3D
                {
                    Content = viewModel.Sim3DViewModel.CutterGeometry
                };

                // Bind cutter transform
                var cutterTransformBinding = new System.Windows.Data.Binding("CutterTransform")
                {
                    Source = viewModel.Sim3DViewModel,
                    Mode = System.Windows.Data.BindingMode.OneWay
                };
                System.Windows.Data.BindingOperations.SetBinding(
                    cutterVisual,
                    ModelVisual3D.TransformProperty,
                    cutterTransformBinding
                );

                simulationModelsVisual.Children.Add(cutterVisual);
                System.Diagnostics.Debug.WriteLine("Added cutter visual to simulation");
            }

            // Add toolpath visuals
            int toolpathCount = 0;
            if (viewModel.Sim3DViewModel?.ToolpathVisuals != null)
            {
                foreach (var visual in viewModel.Sim3DViewModel.ToolpathVisuals)
                {
                    simulationModelsVisual.Children.Add(visual);
                    toolpathCount++;

                    // Debug toolpath visual
                    if (visual is LinesVisual3D lines)
                    {
                        System.Diagnostics.Debug.WriteLine($"  Added toolpath visual: Color={lines.Color}, Thickness={lines.Thickness}");
                    }
                }
                System.Diagnostics.Debug.WriteLine($"Added {toolpathCount} toolpath visuals");

                // Subscribe to collection changes
                viewModel.Sim3DViewModel.ToolpathVisuals.CollectionChanged -= ToolpathVisuals_CollectionChanged;
                viewModel.Sim3DViewModel.ToolpathVisuals.CollectionChanged += ToolpathVisuals_CollectionChanged;
            }

            // Subscribe to property changes
            if (viewModel.Sim3DViewModel != null)
            {
                viewModel.Sim3DViewModel.PropertyChanged -= Sim3DViewModel_PropertyChanged;
                viewModel.Sim3DViewModel.PropertyChanged += Sim3DViewModel_PropertyChanged;
            }

            // Add simulation visual to viewport
            if (!viewport3D.Children.Contains(simulationModelsVisual))
            {
                viewport3D.Children.Add(simulationModelsVisual);
                System.Diagnostics.Debug.WriteLine("Added simulation visual to viewport");
            }

            // Final debug summary
            System.Diagnostics.Debug.WriteLine($"\n=== SIMULATION MODE SUMMARY ===");
            System.Diagnostics.Debug.WriteLine($"Total visuals in simulationModelsVisual: {simulationModelsVisual.Children.Count}");
            System.Diagnostics.Debug.WriteLine($"1. Tube/Wireframe container: 1 visual with {containerGroup.Children.Count} models");
            System.Diagnostics.Debug.WriteLine($"2. Cutter: 1 visual");
            System.Diagnostics.Debug.WriteLine($"3. Toolpath visuals: {toolpathCount}");
            System.Diagnostics.Debug.WriteLine($"Viewport3D total children: {viewport3D.Children.Count}");

            // List all children in viewport
            System.Diagnostics.Debug.WriteLine("\nAll viewport children:");
            for (int i = 0; i < viewport3D.Children.Count; i++)
            {
                var child = viewport3D.Children[i];
                System.Diagnostics.Debug.WriteLine($"  [{i}] {child.GetType().Name}");

                if (child is ModelVisual3D mv && mv.Content is Model3DGroup grp)
                {
                    System.Diagnostics.Debug.WriteLine($"      Contains {grp.Children.Count} models");
                }
            }

            System.Diagnostics.Debug.WriteLine("=== END SWITCH TO SIMULATION MODE ===\n");

            viewModel.Sim3DViewModel?.DebugCoordinateSystems();
            viewModel.Sim3DViewModel?.DebugTubeAndWireframeDimensions();
        }

        private void SwitchToStaticMode()
        {
            // Remove simulation models
            if (viewport3D.Children.Contains(simulationModelsVisual))
            {
                viewport3D.Children.Remove(simulationModelsVisual);
            }

            // Clear simulation container
            simulationModelsVisual.Children.Clear();

            // Unsubscribe from events
            if (viewModel.Sim3DViewModel != null)
            {
                viewModel.Sim3DViewModel.PropertyChanged -= Sim3DViewModel_PropertyChanged;

                if (viewModel.Sim3DViewModel.ToolpathVisuals != null)
                {
                    viewModel.Sim3DViewModel.ToolpathVisuals.CollectionChanged -= ToolpathVisuals_CollectionChanged;
                }
            }

            // Add static models back
            if (!viewport3D.Children.Contains(staticModelsVisual))
            {
                viewport3D.Children.Add(staticModelsVisual);
            }

            // Update static models content
            UpdateStaticModels();
        }

        private void Sim3DViewModel_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (!viewModel.IsSimulation3DActive)
                return;

            // Enhanced debug logging
            System.Diagnostics.Debug.WriteLine($"[MainWindow] Sim3D property changed: {e.PropertyName}");

            if (e.PropertyName == nameof(Simulation3DViewModel.TubeTransform))
            {
                System.Diagnostics.Debug.WriteLine("[MainWindow] TubeTransform updated via binding");
            }
            else if (e.PropertyName == nameof(Simulation3DViewModel.CutterTransform))
            {
                System.Diagnostics.Debug.WriteLine("[MainWindow] CutterTransform updated via binding");
            }
            else if (e.PropertyName == nameof(Simulation3DViewModel.CurrentCRotationSimulation))
            {
                System.Diagnostics.Debug.WriteLine(
                    $"[MainWindow] C rotation changed to: {viewModel.Sim3DViewModel.CurrentCRotationSimulation:F3}°"
                );
            }
            else if (e.PropertyName == nameof(Simulation3DViewModel.CurrentYSimulation))
            {
                System.Diagnostics.Debug.WriteLine(
                    $"[MainWindow] Y position changed to: {viewModel.Sim3DViewModel.CurrentYSimulation:F3}mm"
                );
            }
        }

        private void ToolpathVisuals_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (!viewModel.IsSimulation3DActive)
                return;

            // Remove old toolpath visuals (keep tube and cutter)
            var toRemove = simulationModelsVisual.Children
                .Where(c => c is LinesVisual3D)
                .ToList();

            foreach (var item in toRemove)
            {
                simulationModelsVisual.Children.Remove(item);
            }

            // Add all current toolpath visuals
            if (viewModel.Sim3DViewModel?.ToolpathVisuals != null)
            {
                foreach (var visual in viewModel.Sim3DViewModel.ToolpathVisuals)
                {
                    simulationModelsVisual.Children.Add(visual);
                }
            }
        }

        // Camera view methods
        private void SetFrontView_Click(object sender, RoutedEventArgs e)
        {
            SetFrontView();
        }

        private void SetFrontView()
        {
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                camera.Position = new Point3D(300, 300, 300);
                camera.LookDirection = new Vector3D(-1, -1, -1);
                camera.UpDirection = new Vector3D(0, 0, 1);
                camera.FieldOfView = 45;
            }
        }

        private void SetTopView_Click(object sender, RoutedEventArgs e)
        {
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                camera.Position = new Point3D(0, 0, 500);
                camera.LookDirection = new Vector3D(0, 0, -1);
                camera.UpDirection = new Vector3D(0, 1, 0);
                camera.FieldOfView = 45;
            }
        }

        private void SetSideView_Click(object sender, RoutedEventArgs e)
        {
            var camera = viewport3D.Camera as PerspectiveCamera;
            if (camera != null)
            {
                camera.Position = new Point3D(0, 500, 0);
                camera.LookDirection = new Vector3D(0, -1, 0);
                camera.UpDirection = new Vector3D(0, 0, 1);
                camera.FieldOfView = 45;
            }
        }

        // Mouse interaction methods
        private void Viewport3D_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (viewModel.IsSimulation3DActive)
                return;

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
            if (viewModel.IsSimulation3DActive)
                return;

            if (e.LeftButton == MouseButtonState.Pressed)
            {
                bool hasModifier = Keyboard.IsKeyDown(Key.LeftCtrl) ||
                                  Keyboard.IsKeyDown(Key.RightCtrl) ||
                                  Keyboard.IsKeyDown(Key.LeftShift) ||
                                  Keyboard.IsKeyDown(Key.RightShift) ||
                                  Keyboard.IsKeyDown(Key.LeftAlt) ||
                                  Keyboard.IsKeyDown(Key.RightAlt);

                if (!hasModifier)
                {
                    if (viewModel?.MouseDownCommand?.CanExecute(e) == true)
                    {
                        viewModel.MouseDownCommand.Execute(e);
                        e.Handled = true;
                    }
                }
            }
        }

        // Cleanup
        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);

            if (viewModel != null)
            {
                viewModel.PropertyChanged -= ViewModel_PropertyChanged;

                if (viewModel.Sim3DViewModel != null)
                {
                    viewModel.Sim3DViewModel.PropertyChanged -= Sim3DViewModel_PropertyChanged;

                    if (viewModel.Sim3DViewModel.ToolpathVisuals != null)
                    {
                        viewModel.Sim3DViewModel.ToolpathVisuals.CollectionChanged -= ToolpathVisuals_CollectionChanged;
                    }
                }
            }
        }
    }
}