using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using System.Linq;
using TubeLaserCAM.UI.Models;
using System.Windows;
using TubeLaserCAM.Models;
using System.Windows.Media;
using TubeLaserCAM.UI.Helpers;

namespace TubeLaserCAM.UI.ViewModels
{
    public class Simulation3DViewModel : ObservableObject
    {
        private AnimationController3D _animationController;
        private GCodeParser3D _gcodeParser;
        private LinesVisual3D _completedPathVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Green, Thickness = 1.5 };
        private LinesVisual3D _futurePathVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.LightGray, Thickness = 1 };
        private LinesVisual3D _currentAnimatingSegmentVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Red, Thickness = 2 };
        private Model3DGroup _wireframeOnTubeGroup;
        private bool _isUpdatingAnimation = false;
        private DateTime _lastToolpathUpdate = DateTime.MinValue;
        private const int TOOLPATH_UPDATE_INTERVAL_MS = 50;
        private bool _isWireframeFlipped = false;
        private int _transformUpdateCount = 0;
        private DateTime _lastTransformLog = DateTime.MinValue;
        private double _lastLoggedY = 0;
        private double _lastLoggedC = 0;
        private int _toolpathUpdateCount = 0;
        private int _lastCompletedSegmentIndex = -1;
        private Point3DCollection _completedPoints = new Point3DCollection();
        private Point3DCollection _futurePoints = new Point3DCollection();
        private bool _showToolpath = false; // Để control hiển thị toolpath

        public string SimulationSpeedText => $"{SimulationSpeed:F1}x";

        public double SimulationSpeed
        {
            get => _animationController?.SpeedRatio ?? 1.0;
            set
            {
                if (_animationController != null && value > 0)
                {
                    _animationController.SpeedRatio = value;
                    OnPropertyChanged();
                    System.Diagnostics.Debug.WriteLine($"[SIM3D] Simulation speed changed to: {value:F2}x");
                }
            }
        }
        public bool IsWireframeFlipped
        {
            get => _isWireframeFlipped;
            private set => SetProperty(ref _isWireframeFlipped, value);
        }
        public Simulation3DViewModel()
        {
            _animationController = new AnimationController3D();
            _gcodeParser = new GCodeParser3D();

            _animationController.PropertyChanged += AnimationController_PropertyChanged;
            OnPropertyChanged(nameof(SimulationSpeed));
            LoadGCodeCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(LoadGCodeFile);
            PlayCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(PlaySimulation, CanPlaySimulation);
            PauseCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(PauseSimulation, CanPauseSimulation);
            StopCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(StopSimulation, CanStopSimulation);
            ResetCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(ResetSimulation, CanResetSimulation);

            InitializeDefaultModels();
        }
        public Model3DGroup WireframeOnTubeGroup
        {
            get => _wireframeOnTubeGroup;
            set => SetProperty(ref _wireframeOnTubeGroup, value);
        }


        private void InitializeDefaultModels()
        {
            TubeDisplayRadius = 25;
            TubeDisplayLength = 300;
            CurrentZSimulation = 20;

            System.Diagnostics.Debug.WriteLine("=== INITIALIZE DEFAULT MODELS DEBUG ===");
            System.Diagnostics.Debug.WriteLine($"Initial TubeDisplayRadius: {TubeDisplayRadius}mm (Diameter: {TubeDisplayRadius * 2}mm)");
            System.Diagnostics.Debug.WriteLine($"Initial TubeDisplayLength: {TubeDisplayLength}mm");

            var tubeMeshBuilder = new MeshBuilder(false, false);

            var startPoint = new Point3D(0, 0, 0);
            var endPoint = new Point3D(0, TubeDisplayLength, 0);

            System.Diagnostics.Debug.WriteLine($"Creating tube from Y={startPoint.Y} to Y={endPoint.Y}");
            System.Diagnostics.Debug.WriteLine($"Tube center will be at Y={TubeDisplayLength / 2.0}");

            tubeMeshBuilder.AddCylinder(
                startPoint,
                endPoint,
                TubeDisplayRadius * 2,
                36
            );

            var tubeHelixMesh = tubeMeshBuilder.ToMesh(true);
            var tubeWpfMesh = new MeshGeometry3D
            {
                Positions = tubeHelixMesh.Positions,
                TriangleIndices = tubeHelixMesh.TriangleIndices,
                Normals = tubeHelixMesh.Normals,
                TextureCoordinates = tubeHelixMesh.TextureCoordinates
            };

            // QUAN TRỌNG: Tạo material cho cả mặt ngoài và mặt trong
            var tubeFrontMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Blue));
            var tubeBackMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.DarkBlue)); // Màu tối hơn cho mặt trong

            // Hoặc dùng MaterialGroup để có hiệu ứng tốt hơn
            var frontMaterialGroup = new MaterialGroup();
            frontMaterialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.DarkOrange)));
            frontMaterialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.LightYellow), 20));


            var backMaterialGroup = new MaterialGroup();
            backMaterialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(184, 134, 11)))); 
            backMaterialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.DarkGoldenrod), 10));

            TubeGeometry = new GeometryModel3D
            {
                Geometry = tubeWpfMesh,
                Material = frontMaterialGroup,
                BackMaterial = backMaterialGroup  // QUAN TRỌNG: Thêm BackMaterial
            };

            // Cutter initialization
            var cutterMeshBuilder = new MeshBuilder(false, false);
            double cutterYPosition = 0;

            System.Diagnostics.Debug.WriteLine($"Placing cutter at Y={cutterYPosition} (tube center)");

            cutterMeshBuilder.AddArrow(
                new Point3D(0, cutterYPosition, CurrentZSimulation + 15),
                new Point3D(0, cutterYPosition, CurrentZSimulation),
                3, 10, 4
            );

            var cutterHelixMesh = cutterMeshBuilder.ToMesh(true);
            var cutterWpfMesh = new MeshGeometry3D
            {
                Positions = cutterHelixMesh.Positions,
                TriangleIndices = cutterHelixMesh.TriangleIndices,
                Normals = cutterHelixMesh.Normals,
                TextureCoordinates = cutterHelixMesh.TextureCoordinates
            };

            CutterGeometry = new GeometryModel3D(cutterWpfMesh, Materials.Red);

            System.Diagnostics.Debug.WriteLine("=== END INITIALIZE DEFAULT MODELS ===\n");
        }



        private GeometryModel3D _tubeGeometry;
        public GeometryModel3D TubeGeometry
        {
            get => _tubeGeometry;
            set => SetProperty(ref _tubeGeometry, value); // SetProperty sẽ giải quyết CS0103
        }

        private GeometryModel3D _cutterGeometry;
        public GeometryModel3D CutterGeometry
        {
            get => _cutterGeometry;
            set => SetProperty(ref _cutterGeometry, value);
        }

        private Transform3D _tubeTransform = new Transform3DGroup();
        public Transform3D TubeTransform
        {
            get => _tubeTransform;
            set => SetProperty(ref _tubeTransform, value);
        }

        private void AnimationController_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine($"[SIM3D-RECV] Property changed: {e.PropertyName}");
            if (sender == _animationController)
            {
                System.Diagnostics.Debug.WriteLine($"[SIM3D-RECV] From AnimationController");
                System.Diagnostics.Debug.WriteLine($"[SIM3D-RECV] Current values: Y={_animationController.CurrentYPosition:F3}, C={_animationController.CurrentCRotation:F3}");
            }
            System.Diagnostics.Debug.WriteLine($"[SIM3D] AnimationController property changed: {e.PropertyName}");

            Application.Current.Dispatcher.BeginInvoke(new Action(() =>
            {
                switch (e.PropertyName)
                {
                    case nameof(AnimationController3D.CurrentYPosition):
                        var oldY = CurrentYSimulation;
                        CurrentYSimulation = _animationController.CurrentYPosition;
                        System.Diagnostics.Debug.WriteLine(
                            $"[SIM3D] Y updated: {oldY:F3} → {CurrentYSimulation:F3}"
                        );
                        break;

                    case nameof(AnimationController3D.CurrentCRotation):
                        var oldC = CurrentCRotationSimulation;
                        CurrentCRotationSimulation = _animationController.CurrentCRotation;
                        System.Diagnostics.Debug.WriteLine(
                            $"[SIM3D] C updated: {oldC:F3}° → {CurrentCRotationSimulation:F3}°"
                        );
                        break;

                    case nameof(AnimationController3D.CurrentZPosition):
                        CurrentZSimulation = _animationController.CurrentZPosition;
                        UpdateCutterTransform();
                        break;

                    case nameof(AnimationController3D.IsCurrentLaserOn):
                        System.Diagnostics.Debug.WriteLine($"[SIM3D] Laser state changed: {_animationController.IsCurrentLaserOn}");
                        UpdateToolpathVisualsOptimized();
                        break;

                    case nameof(AnimationController3D.SimulationProgress):
                        UpdateToolpathVisualsOptimized();
                        break;

                    case nameof(AnimationController3D.SpeedRatio):
                        OnPropertyChanged(nameof(SimulationSpeed));
                        OnPropertyChanged(nameof(SimulationSpeedText));
                        System.Diagnostics.Debug.WriteLine($"[SIM3D] Speed ratio changed: {_animationController.SpeedRatio:F2}x");
                        break;
                }
            }), System.Windows.Threading.DispatcherPriority.Normal);
        }

        private void UpdateToolpathVisualsOptimized()
        {
            var now = DateTime.Now;
            if ((now - _lastToolpathUpdate).TotalMilliseconds < TOOLPATH_UPDATE_INTERVAL_MS)
                return;

            _lastToolpathUpdate = now;

            // Tắt toolpath nếu không muốn hiển thị
            if (!_showToolpath) return;

            int currentSegmentIndex = _animationController.CurrentCommandIndex;

            // Chỉ update nếu segment thay đổi
            if (currentSegmentIndex == _lastCompletedSegmentIndex)
                return;

            _toolpathUpdateCount++;

            // Debug ít hơn
            if (_toolpathUpdateCount % 50 == 0)
            {
                System.Diagnostics.Debug.WriteLine($"[TOOLPATH] Update #{_toolpathUpdateCount}, Segment: {currentSegmentIndex}");
            }

            // Chỉ thêm segments mới vào completed path
            if (currentSegmentIndex > _lastCompletedSegmentIndex)
            {
                for (int i = _lastCompletedSegmentIndex + 1; i < currentSegmentIndex && i < _allLocalToolpathPoints.Count - 1; i++)
                {
                    if (_animationController.Commands[i + 1].IsLaserOn)
                    {
                        _completedPoints.Add(_allLocalToolpathPoints[i]);
                        _completedPoints.Add(_allLocalToolpathPoints[i + 1]);
                    }
                }

                // Cập nhật visual một lần
                _completedPathVisual.Points = _completedPoints;
                _lastCompletedSegmentIndex = currentSegmentIndex;
            }

            // Current segment - giảm subdivisions
            UpdateCurrentSegmentOptimized(currentSegmentIndex);
        }

        private void UpdateCurrentSegmentOptimized(int currentSegmentIndex)
        {
            _currentAnimatingSegmentVisual.Points.Clear();

            if (currentSegmentIndex >= _animationController.Commands.Count || currentSegmentIndex <= 0)
                return;

            var targetCmd = _animationController.Commands[currentSegmentIndex];
            if (!targetCmd.IsLaserOn) return;

            var startCmd = _animationController.Commands[currentSegmentIndex - 1];

            // Giảm subdivisions từ 20 xuống 5-10
            int subdivisions = AngleHelper.CrossesSeam(startCmd.C, targetCmd.C) ? 10 : 5;

            for (int i = 0; i < subdivisions; i++)
            {
                double t1 = (double)i / subdivisions;
                double t2 = (double)(i + 1) / subdivisions;

                double y1 = startCmd.Y + (targetCmd.Y - startCmd.Y) * t1;
                double c1 = AngleHelper.InterpolateAngle(startCmd.C, targetCmd.C, t1);

                double y2 = startCmd.Y + (targetCmd.Y - startCmd.Y) * t2;
                double c2 = AngleHelper.InterpolateAngle(startCmd.C, targetCmd.C, t2);

                Point3D p1 = ConvertGCodeCoordToLocalPointOnTubeSurface(y1, c1, TubeDisplayRadius);
                Point3D p2 = ConvertGCodeCoordToLocalPointOnTubeSurface(y2, c2, TubeDisplayRadius);

                _currentAnimatingSegmentVisual.Points.Add(p1);
                _currentAnimatingSegmentVisual.Points.Add(p2);
            }
        }

        private Transform3D _cutterTransform = new TranslateTransform3D(0, 0, 20);
        public Transform3D CutterTransform
        {
            get => _cutterTransform;
            set => SetProperty(ref _cutterTransform, value);
        }


        private double _tubeDisplayRadius = 25;
        public double TubeDisplayRadius
        {
            get => _tubeDisplayRadius;
            set
            {
                if (SetProperty(ref _tubeDisplayRadius, value))
                {
                    UpdateTubeGeometry();
                }
            }
        }

        private double _tubeDisplayLength = 300;
        public double TubeDisplayLength
        {
            get => _tubeDisplayLength;
            set
            {
                if (SetProperty(ref _tubeDisplayLength, value))
                {
                    UpdateTubeGeometry();
                }
            }
        }

        private void UpdateTubeGeometry()
        {
            System.Diagnostics.Debug.WriteLine($"UpdateTubeGeometry: Creating tube with R={TubeDisplayRadius:F2}, L={TubeDisplayLength:F2}");

            var tubeMeshBuilder = new MeshBuilder(false, false);

            tubeMeshBuilder.AddCylinder(
                new Point3D(0, 0, 0),
                new Point3D(0, TubeDisplayLength, 0),
                TubeDisplayRadius * 2,
                36
            );

            var tubeHelixMesh = tubeMeshBuilder.ToMesh(true);
            var tubeWpfMesh = new MeshGeometry3D
            {
                Positions = tubeHelixMesh.Positions,
                TriangleIndices = tubeHelixMesh.TriangleIndices,
                Normals = tubeHelixMesh.Normals,
                TextureCoordinates = tubeHelixMesh.TextureCoordinates
            };

            // Tạo materials cho cả hai mặt
            var frontMaterialGroup = new MaterialGroup();
            frontMaterialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.DarkOrange)));
            frontMaterialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.DarkOrange), 20));

            var backMaterialGroup = new MaterialGroup();
            backMaterialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(184, 134, 11))));
            backMaterialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.DarkGoldenrod), 10));

            // Hoặc nếu muốn mặt trong có màu khác biệt rõ hơn
            // backMaterialGroup.Children.Add(new DiffuseMaterial(new SolidColorBrush(Colors.Gray)));

            TubeGeometry = new GeometryModel3D
            {
                Geometry = tubeWpfMesh,
                Material = frontMaterialGroup,
                BackMaterial = backMaterialGroup
            };
        }

        private void UpdateCutterTransform()
        {
            // Cutter luôn ở vị trí cố định Y=0 (đầu ống)
            // Z di chuyển theo CurrentZSimulation từ G-code
            double cutterY = 0;  // Luôn ở đầu ống

            CutterTransform = new TranslateTransform3D(
                0,                      // X không đổi
                cutterY,                // Y cố định ở 0
                CurrentZSimulation      // Z từ G-code
            );
        }


        private double _currentYSimulation;
        public double CurrentYSimulation
        {
            get => _currentYSimulation;
            set
            {
                if (SetProperty(ref _currentYSimulation, value))
                    UpdateTubeTransform();
            }
        }

        private double _currentCRotationSimulation;
        public double CurrentCRotationSimulation
        {
            get => _currentCRotationSimulation;
            set
            {
                if (SetProperty(ref _currentCRotationSimulation, value))
                {
                    System.Diagnostics.Debug.WriteLine($"[SIM3D-PROP] CurrentCRotationSimulation set to: {value:F3}°");
                    UpdateTubeTransform();
                }
            }
        }

        private double _currentZSimulation; // Z này là Z của đầu cắt từ G-code
        public double CurrentZSimulation
        {
            get => _currentZSimulation;
            // Z này nên được AnimationController cập nhật trực tiếp, không cần setter phức tạp ở đây
            // trừ khi có logic phụ thuộc vào nó.
            set => SetProperty(ref _currentZSimulation, value);
        }

        private void UpdateTubeTransform()
        {
            _transformUpdateCount++;

            // Debug current values BEFORE creating transform
            System.Diagnostics.Debug.WriteLine(
                $"[TRANSFORM-PRE] Update #{_transformUpdateCount}: Y={CurrentYSimulation:F3}, C={CurrentCRotationSimulation:F3}°"
            );

            var group = new Transform3DGroup();

            // Get the center of tube for rotation
            double centerY = TubeDisplayLength / 2.0;

            // QUAN TRỌNG: Create rotation transform
            var axisAngleRotation = new AxisAngleRotation3D(new Vector3D(0, 1, 0), CurrentCRotationSimulation);
            var rotateTransform = new RotateTransform3D(axisAngleRotation, new Point3D(0, centerY, 0));

            // Debug rotation details
            System.Diagnostics.Debug.WriteLine(
                $"[TRANSFORM-ROT] Creating rotation: Angle={CurrentCRotationSimulation:F3}°, " +
                $"Axis=(0,1,0), Center=(0,{centerY:F3},0)"
            );

            group.Children.Add(rotateTransform);

            // Then translate
            double yTranslation = IsWireframeFlipped ? CurrentYSimulation : -CurrentYSimulation;
            var translateTransform = new TranslateTransform3D(0, yTranslation, 0);
            group.Children.Add(translateTransform);

            // Apply to both tube and wireframe
            TubeTransform = group;

            if (_wireframeOnTubeGroup != null)
            {
                _wireframeOnTubeGroup.Transform = group;
            }

            // Update toolpath transforms
            UpdateToolpathTransforms();

            // Enhanced debug logging
            bool shouldLog = false;
            if ((DateTime.Now - _lastTransformLog).TotalMilliseconds > 100)
            {
                shouldLog = true;
            }
            else if (Math.Abs(_lastLoggedY - CurrentYSimulation) > 0.5 ||
                     Math.Abs(_lastLoggedC - CurrentCRotationSimulation) > 1.0)
            {
                shouldLog = true;
            }

            if (shouldLog)
            {
                System.Diagnostics.Debug.WriteLine(
                    $"[TRANSFORM-POST] Update #{_transformUpdateCount}: " +
                    $"Y={CurrentYSimulation:F3} (Δ={CurrentYSimulation - _lastLoggedY:F3}), " +
                    $"C={CurrentCRotationSimulation:F3}° (Δ={CurrentCRotationSimulation - _lastLoggedC:F3}°), " +
                    $"Flipped: {IsWireframeFlipped}"
                );
                _lastTransformLog = DateTime.Now;
                _lastLoggedY = CurrentYSimulation;
                _lastLoggedC = CurrentCRotationSimulation;
            }
        }

        private double _lastDebuggedCRotation = 0; // Thêm field


        private Point3D ProjectPointToCylinderSurface(Point3D point, double cylinderRadius)
        {
            // Assuming cylinder axis is Y and centered at (0, Y, 0)
            // Project point to cylinder surface
            double x = point.X;
            double z = point.Z;
            double radius = Math.Sqrt(x * x + z * z);

            if (radius < 0.001) // Point on axis
            {
                return new Point3D(cylinderRadius, point.Y, 0);
            }

            // Normalize and scale to cylinder radius
            double scale = cylinderRadius / radius;
            return new Point3D(x * scale, point.Y, z * scale);
        }

        public void DebugCoordinateSystems()
        {
            System.Diagnostics.Debug.WriteLine("=== COORDINATE SYSTEM DEBUG ===");
            System.Diagnostics.Debug.WriteLine($"Tube Center: (0, {TubeDisplayLength / 2.0:F2}, 0)");
            System.Diagnostics.Debug.WriteLine($"Tube Radius: {TubeDisplayRadius}");
            System.Diagnostics.Debug.WriteLine($"Current Transform: Y={CurrentYSimulation:F2}, C={CurrentCRotationSimulation:F2}°");

            if (_wireframeOnTubeGroup != null && _wireframeOnTubeGroup.Bounds != Rect3D.Empty)
            {
                var bounds = _wireframeOnTubeGroup.Bounds;
                System.Diagnostics.Debug.WriteLine($"Wireframe bounds: Center=({bounds.X + bounds.SizeX / 2:F2}, {bounds.Y + bounds.SizeY / 2:F2}, {bounds.Z + bounds.SizeZ / 2:F2})");
                System.Diagnostics.Debug.WriteLine($"Wireframe size: ({bounds.SizeX:F2}, {bounds.SizeY:F2}, {bounds.SizeZ:F2})");
            }
            System.Diagnostics.Debug.WriteLine("=== END DEBUG ===");
        }

        public void LoadWireframeOnTube(Model3DGroup originalWireframe)
        {
            if (originalWireframe == null) return;
            // Reset flip flag khi load wireframe mới
            IsWireframeFlipped = false;

            _wireframeOnTubeGroup = new Model3DGroup();

            System.Diagnostics.Debug.WriteLine("=== ENHANCED WIREFRAME LOADING WITH Y-ALIGNMENT ===");
            System.Diagnostics.Debug.WriteLine($"Loading {originalWireframe.Children.Count} wireframe elements");

            // Get wireframe bounds
            var bounds = originalWireframe.Bounds;
            System.Diagnostics.Debug.WriteLine($"\nORIGINAL WIREFRAME INFO:");
            System.Diagnostics.Debug.WriteLine($"Bounds: {bounds}");
            System.Diagnostics.Debug.WriteLine($"  X: [{bounds.X:F2}, {bounds.X + bounds.SizeX:F2}] Size: {bounds.SizeX:F2}");
            System.Diagnostics.Debug.WriteLine($"  Y: [{bounds.Y:F2}, {bounds.Y + bounds.SizeY:F2}] Size: {bounds.SizeY:F2}");
            System.Diagnostics.Debug.WriteLine($"  Z: [{bounds.Z:F2}, {bounds.Z + bounds.SizeZ:F2}] Size: {bounds.SizeZ:F2}");

            // Calculate centers
            double wireframeCenterX = bounds.X + bounds.SizeX / 2.0;
            double wireframeCenterY = bounds.Y + bounds.SizeY / 2.0;
            double wireframeCenterZ = bounds.Z + bounds.SizeZ / 2.0;
            System.Diagnostics.Debug.WriteLine($"Wireframe center: X={wireframeCenterX:F2}, Y={wireframeCenterY:F2}, Z={wireframeCenterZ:F2}");

            // Detect alignment
            bool isZAligned = bounds.SizeZ > bounds.SizeY && bounds.SizeZ > bounds.SizeX;
            System.Diagnostics.Debug.WriteLine($"Wireframe alignment: {(isZAligned ? "Z-aligned (need rotation)" : "Y-aligned (correct)")}");

            // Get actual cylinder length
            double actualCylinderLength = isZAligned ? bounds.SizeZ : bounds.SizeY;
            System.Diagnostics.Debug.WriteLine($"Detected cylinder length: {actualCylinderLength:F2}mm");

            // Check and adjust tube radius if needed
            double maxWireframeRadius = Math.Max(bounds.SizeX / 2.0, isZAligned ? bounds.SizeY / 2.0 : bounds.SizeZ / 2.0);
            System.Diagnostics.Debug.WriteLine($"Max wireframe radius: {maxWireframeRadius:F2}mm");

            if (maxWireframeRadius > TubeDisplayRadius * 0.95) // Allow 5% tolerance
            {
                System.Diagnostics.Debug.WriteLine($"⚠️ Adjusting tube radius from {TubeDisplayRadius:F2} to {maxWireframeRadius * 1.01:F2}");
                TubeDisplayRadius = maxWireframeRadius * 1.01; // Add 1% margin
            }

            // Update tube geometry
            TubeDisplayLength = actualCylinderLength;
            UpdateTubeGeometry();

            // Calculate transforms
            double tubeCenterY = TubeDisplayLength / 2.0;
            System.Diagnostics.Debug.WriteLine($"\nTRANSFORM CALCULATIONS:");
            System.Diagnostics.Debug.WriteLine($"Tube Y range: [0, {TubeDisplayLength:F2}], Center: {tubeCenterY:F2}");

            // Calculate offsets
            double xOffset = -wireframeCenterX;  // Center X to 0
            double yOffset = 0;
            double zOffset = -wireframeCenterZ;  // Center Z to 0

            if (isZAligned)
            {
                // For Z-aligned: after rotation, the Z-center becomes Y-center
                yOffset = tubeCenterY - wireframeCenterZ;
                System.Diagnostics.Debug.WriteLine($"Z-aligned: Y offset after rotation = {yOffset:F2}");
            }
            else
            {
                // For Y-aligned: direct Y centering
                yOffset = tubeCenterY - wireframeCenterY;
                System.Diagnostics.Debug.WriteLine($"Y-aligned: Y offset = {yOffset:F2}");
            }

            System.Diagnostics.Debug.WriteLine($"Final offsets: X={xOffset:F2}, Y={yOffset:F2}, Z={zOffset:F2}");

            // Transform each child
            int transformedCount = 0;
            int projectedCount = 0;

            foreach (var child in originalWireframe.Children)
            {
                if (child is GeometryModel3D model && model.Geometry is MeshGeometry3D originalMesh)
                {
                    var newMesh = new MeshGeometry3D();

                    foreach (var pos in originalMesh.Positions)
                    {
                        Point3D transformedPos;

                        if (isZAligned)
                        {
                            // First apply offsets, then rotate
                            double tempX = pos.X + xOffset;
                            double tempY = pos.Y;  // Will become -Z
                            double tempZ = pos.Z + wireframeCenterZ; // Shift so rotation center is at origin

                            // Rotate: Z->Y, Y->-Z
                            transformedPos = new Point3D(
                                tempX,
                                tempZ + yOffset,      // Z becomes Y, then apply Y offset
                                -tempY                // Y becomes -Z
                            );
                        }
                        else
                        {
                            // Y-aligned: just apply offsets
                            transformedPos = new Point3D(
                                pos.X + xOffset,
                                pos.Y + yOffset,
                                pos.Z + zOffset
                            );
                        }

                        // Project to cylinder surface if needed
                        double radius = Math.Sqrt(transformedPos.X * transformedPos.X + transformedPos.Z * transformedPos.Z);

                        // Only project if point is significantly off the cylinder surface
                        if (Math.Abs(radius - TubeDisplayRadius) > 1.0 && radius > 0.1)
                        {
                            double offsetDistance = 0.3; 
                            double scale = (TubeDisplayRadius + offsetDistance) / radius;
                            transformedPos = new Point3D(
                                transformedPos.X * scale,
                                transformedPos.Y,  
                                transformedPos.Z * scale
                            );
                            projectedCount++;
                        }

                        newMesh.Positions.Add(transformedPos);

                        // Debug first few positions
                        if (newMesh.Positions.Count <= 3)
                        {
                            System.Diagnostics.Debug.WriteLine($"\nPoint {newMesh.Positions.Count}:");
                            System.Diagnostics.Debug.WriteLine($"  Original: ({pos.X:F2}, {pos.Y:F2}, {pos.Z:F2})");
                            System.Diagnostics.Debug.WriteLine($"  Transformed: ({transformedPos.X:F2}, {transformedPos.Y:F2}, {transformedPos.Z:F2})");
                            System.Diagnostics.Debug.WriteLine($"  Radius: {radius:F2} -> {Math.Sqrt(transformedPos.X * transformedPos.X + transformedPos.Z * transformedPos.Z):F2}");
                        }
                    }

                    // Copy triangle indices
                    foreach (var idx in originalMesh.TriangleIndices)
                    {
                        newMesh.TriangleIndices.Add(idx);
                    }

                    // Transform normals
                    if (originalMesh.Normals != null && originalMesh.Normals.Count > 0)
                    {
                        foreach (var normal in originalMesh.Normals)
                        {
                            Vector3D transformedNormal;
                            if (isZAligned)
                            {
                                transformedNormal = new Vector3D(normal.X, normal.Z, -normal.Y);
                            }
                            else
                            {
                                transformedNormal = normal;
                            }
                            transformedNormal.Normalize();
                            newMesh.Normals.Add(transformedNormal);
                        }
                    }

                    // Create material
                    var wireframeMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Black))
                    {
                        AmbientColor = Color.FromRgb(50, 50, 50)
                    };

                    var materialGroup = new MaterialGroup();
                    materialGroup.Children.Add(wireframeMaterial);
                    materialGroup.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.DarkGray), 10));

                    var newModel = new GeometryModel3D
                    {
                        Geometry = newMesh,
                        Material = materialGroup,
                        BackMaterial = materialGroup
                    };

                    _wireframeOnTubeGroup.Children.Add(newModel);
                    transformedCount++;
                }
            }

            // Set initial transform
            if (TubeTransform != null)
            {
                _wireframeOnTubeGroup.Transform = TubeTransform;
            }

            // Verify final alignment
            var newBounds = _wireframeOnTubeGroup.Bounds;
            double minY = newBounds.Y;
            double maxY = newBounds.Y + newBounds.SizeY;

            System.Diagnostics.Debug.WriteLine($"\nFINAL VERIFICATION:");
            System.Diagnostics.Debug.WriteLine($"Transformed {transformedCount} models, projected {projectedCount} points");
            System.Diagnostics.Debug.WriteLine($"New wireframe bounds:");
            System.Diagnostics.Debug.WriteLine($"  X: [{newBounds.X:F2}, {newBounds.X + newBounds.SizeX:F2}] Size: {newBounds.SizeX:F2}");
            System.Diagnostics.Debug.WriteLine($"  Y: [{newBounds.Y:F2}, {newBounds.Y + newBounds.SizeY:F2}] Size: {newBounds.SizeY:F2}");
            System.Diagnostics.Debug.WriteLine($"  Z: [{newBounds.Z:F2}, {newBounds.Z + newBounds.SizeZ:F2}] Size: {newBounds.SizeZ:F2}");

            double newWireframeCenterY = newBounds.Y + newBounds.SizeY / 2.0;
            double alignmentError = Math.Abs(newWireframeCenterY - tubeCenterY);

            System.Diagnostics.Debug.WriteLine($"\nALIGNMENT RESULT:");
            System.Diagnostics.Debug.WriteLine($"Tube center Y: {tubeCenterY:F2}");
            System.Diagnostics.Debug.WriteLine($"Wireframe center Y: {newWireframeCenterY:F2}");
            System.Diagnostics.Debug.WriteLine($"Alignment error: {alignmentError:F2}mm {(alignmentError < 1.0 ? "✅ GOOD" : "⚠️ CHECK ALIGNMENT")}");

            // Check radius fit
            double maxNewRadius = Math.Max(
                Math.Max(Math.Abs(newBounds.X), Math.Abs(newBounds.X + newBounds.SizeX)),
                Math.Max(Math.Abs(newBounds.Z), Math.Abs(newBounds.Z + newBounds.SizeZ))
            );

            System.Diagnostics.Debug.WriteLine($"\nRADIUS CHECK:");
            System.Diagnostics.Debug.WriteLine($"Max wireframe radius: {maxNewRadius:F2}mm");
            System.Diagnostics.Debug.WriteLine($"Tube radius: {TubeDisplayRadius:F2}mm");
            System.Diagnostics.Debug.WriteLine($"Fits in tube: {(maxNewRadius <= TubeDisplayRadius ? "✅ YES" : "❌ NO")}");

            System.Diagnostics.Debug.WriteLine("=== END WIREFRAME LOADING ===\n");
            if (minY < TubeDisplayLength / 3.0)
            {
                System.Diagnostics.Debug.WriteLine("⚠️ Auto-flipping wireframe - starts at same end as cutter");
                FlipWireframeYDirection();
            }

            OnPropertyChanged(nameof(WireframeOnTubeGroup));
        }


        public ObservableCollection<Visual3D> ToolpathVisuals { get; } = new ObservableCollection<Visual3D>();
        private List<Point3D> _allLocalToolpathPoints = new List<Point3D>();


        public ICommand LoadGCodeCommand { get; }
        public ICommand PlayCommand { get; }
        public ICommand PauseCommand { get; }
        public ICommand StopCommand { get; }
        public ICommand ResetCommand { get; }

        private string _loadedGCodeFilePath;
        public string LoadedGCodeFilePath
        {
            get => _loadedGCodeFilePath;
            set => SetProperty(ref _loadedGCodeFilePath, value);
        }

        private void LoadGCodeFile()
        {
            OpenFileDialog openFileDialog = new OpenFileDialog
            {
                Filter = "G-code files (*.gcode;*.nc;*.tap)|*.gcode;*.nc;*.tap|All files (*.*)|*.*",
                Title = "Load G-Code File for 3D Simulation"
            };

            if (openFileDialog.ShowDialog() == true)
            {
                LoadedGCodeFilePath = openFileDialog.FileName;
                try
                {
                    List<GCodeCommand3D> commands = _gcodeParser.Parse(LoadedGCodeFilePath);
                    if (commands == null || !commands.Any())
                    {
                        MessageBox.Show("G-code file is empty or could not be parsed.", "Parsing Error", MessageBoxButton.OK, MessageBoxImage.Error);
                        return;
                    }

                    double radiusForSim = TubeDisplayRadius;
                    _animationController.LoadCommands(commands, radiusForSim);

                    PrepareToolpathVisuals(commands, radiusForSim);
                    // Thông báo cho các lệnh rằng CanExecute có thể đã thay đổi
                    (PlayCommand as CommunityToolkit.Mvvm.Input.RelayCommand)?.NotifyCanExecuteChanged();
                    (ResetCommand as CommunityToolkit.Mvvm.Input.RelayCommand)?.NotifyCanExecuteChanged();
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Error loading or parsing G-code file: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
        }


        private Point3D ConvertYCtoXYZ(double y_gcode, double c_gcode_deg, double radius)
        {
            double c_rad = c_gcode_deg * Math.PI / 180.0;
            // Giả sử ống nằm dọc theo trục Y của scene, tâm tại (0,Y,0) và xoay quanh trục Y của scene
            // Đầu cắt cố định tại X=radius, Z=0 (trong hệ tọa độ của ống) và di chuyển lên xuống theo Z_gcode
            // Khi ống xoay, điểm cắt trên ống sẽ di chuyển
            // Y_scene = y_gcode
            // X_scene = radius * cos(c_rad)
            // Z_scene = radius * sin(c_rad)
            return new Point3D(radius * Math.Cos(c_rad), y_gcode, radius * Math.Sin(c_rad));
        }

        private void PrepareToolpathVisuals(List<GCodeCommand3D> commands, double radius)
        {
            _allLocalToolpathPoints.Clear();

            if (commands == null || !commands.Any())
            {
                // Xóa visual nếu không có lệnh
                _completedPathVisual.Points.Clear();
                _futurePathVisual.Points.Clear();
                _currentAnimatingSegmentVisual.Points.Clear();
                return;
            }

            foreach (var cmd in commands)
            {
                Point3D localPoint = ConvertGCodeCoordToLocalPointOnTubeSurface(cmd.Y, cmd.C, radius);
                _allLocalToolpathPoints.Add(localPoint);
            }

            // Thêm các visual vào collection nếu chưa có (thực hiện một lần, ví dụ trong constructor hoặc khi collection trống)
            /*if (!ToolpathVisuals.Contains(_completedPathVisual)) ToolpathVisuals.Add(_completedPathVisual);
            if (!ToolpathVisuals.Contains(_futurePathVisual)) ToolpathVisuals.Add(_futurePathVisual);
            if (!ToolpathVisuals.Contains(_currentAnimatingSegmentVisual)) ToolpathVisuals.Add(_currentAnimatingSegmentVisual);

            // Vẽ toàn bộ toolpath ban đầu với màu "future"
            _futurePathVisual.Points.Clear();
            if (_allLocalToolpathPoints.Count > 1)
            {
                for (int i = 0; i < _allLocalToolpathPoints.Count - 1; i++)
                {
                    _futurePathVisual.Points.Add(_allLocalToolpathPoints[i]);
                    _futurePathVisual.Points.Add(_allLocalToolpathPoints[i + 1]);
                }
            }*/
            _completedPathVisual.Points.Clear();
            _currentAnimatingSegmentVisual.Points.Clear();
        }

        private LinesVisual3D _currentCutSegmentVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Red, Thickness = 2 };

        private void UpdateToolpathVisuals(bool isLaserOn, double progress)
        {
            _toolpathUpdateCount++;

            System.Diagnostics.Debug.WriteLine($"\n[TOOLPATH] Update #{_toolpathUpdateCount}");
            System.Diagnostics.Debug.WriteLine($"  IsLaserOn: {isLaserOn}, Progress: {progress:P}");
            System.Diagnostics.Debug.WriteLine($"  Current segment: {_animationController.CurrentCommandIndex}");
            System.Diagnostics.Debug.WriteLine($"  Current Y: {_animationController.CurrentYPosition:F3}, C: {_animationController.CurrentCRotation:F3}");

            int totalLocalPoints = _allLocalToolpathPoints.Count;
            if (totalLocalPoints < 1) return;

            _completedPathVisual.Points.Clear();
            _futurePathVisual.Points.Clear();
            _currentAnimatingSegmentVisual.Points.Clear();

            var currentTransform = TubeTransform ?? Transform3D.Identity;
            _completedPathVisual.Transform = currentTransform;
            _futurePathVisual.Transform = currentTransform;
            _currentAnimatingSegmentVisual.Transform = currentTransform;

            int currentSegmentOverallProgressIndex = (int)(_animationController.SimulationProgress * (_animationController.Commands?.Count ?? 0));
            if (currentSegmentOverallProgressIndex >= _animationController.Commands.Count && _animationController.Commands.Any())
            {
                currentSegmentOverallProgressIndex = _animationController.Commands.Count - 1;
            }

            // Vẽ completed path
            int completedSegments = 0;
            for (int i = 0; i < currentSegmentOverallProgressIndex && i < totalLocalPoints - 1; i++)
            {
                if (_animationController.Commands.Count > i + 1)
                {
                    var cmd1 = _animationController.Commands[i];
                    var cmd2 = _animationController.Commands[i + 1];

                    if (cmd2.IsLaserOn && !cmd2.IsRapidMove)
                    {
                        completedSegments++;
                        if (AngleHelper.CrossesSeam(cmd1.C, cmd2.C))
                        {
                            DrawSegmentAcrossSeam(i, cmd1, cmd2, _completedPathVisual);
                        }
                        else
                        {
                            _completedPathVisual.Points.Add(_allLocalToolpathPoints[i]);
                            _completedPathVisual.Points.Add(_allLocalToolpathPoints[i + 1]);
                        }
                    }
                }
            }

            // Vẽ current animating segment với nhiều subdivision
            int currentTargetCommandIndex = _animationController.CurrentCommandIndex;
            if (isLaserOn && currentTargetCommandIndex < _animationController.Commands.Count && currentTargetCommandIndex > 0)
            {
                var targetCmd = _animationController.Commands[currentTargetCommandIndex];
                var startCmd = _animationController.Commands[currentTargetCommandIndex - 1];

                // Calculate segment progress
                double segmentProgress = 0;
                if (_animationController.CurrentCommandIndex == currentTargetCommandIndex)
                {
                    // Get progress within current segment
                    var duration = AngleHelper.CrossesSeam(startCmd.C, targetCmd.C) ?
                        TimeSpan.FromSeconds(1.0) : TimeSpan.FromSeconds(0.5); // Adjust as needed

                    segmentProgress = Math.Min(1.0, _elapsedTimeInCurrentSegment.TotalSeconds / duration.TotalSeconds);
                }

                System.Diagnostics.Debug.WriteLine(
                    $"  Drawing current segment: {currentTargetCommandIndex - 1} → {currentTargetCommandIndex}, " +
                    $"Progress: {segmentProgress:P}"
                );

                // Draw with subdivisions for smooth curve
                int subdivisions = 20;
                for (int i = 0; i < subdivisions; i++)
                {
                    double t1 = (double)i / subdivisions * segmentProgress;
                    double t2 = (double)(i + 1) / subdivisions * segmentProgress;

                    if (t2 > 0 && t1 < 1)
                    {
                        double y1 = startCmd.Y + (targetCmd.Y - startCmd.Y) * t1;
                        double c1 = AngleHelper.InterpolateAngle(startCmd.C, targetCmd.C, t1);

                        double y2 = startCmd.Y + (targetCmd.Y - startCmd.Y) * t2;
                        double c2 = AngleHelper.InterpolateAngle(startCmd.C, targetCmd.C, t2);

                        Point3D p1 = ConvertGCodeCoordToLocalPointOnTubeSurface(y1, c1, TubeDisplayRadius);
                        Point3D p2 = ConvertGCodeCoordToLocalPointOnTubeSurface(y2, c2, TubeDisplayRadius);

                        _currentAnimatingSegmentVisual.Points.Add(p1);
                        _currentAnimatingSegmentVisual.Points.Add(p2);
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"  Completed segments: {completedSegments}");
            System.Diagnostics.Debug.WriteLine($"  Completed path points: {_completedPathVisual.Points.Count}");
            System.Diagnostics.Debug.WriteLine($"  Current segment points: {_currentAnimatingSegmentVisual.Points.Count}");
            System.Diagnostics.Debug.WriteLine($"  Future path points: {_futurePathVisual.Points.Count}");
        }

        private TimeSpan _elapsedTimeInCurrentSegment =>
            _animationController?.GetType()
            .GetField("_elapsedTimeInCurrentSegment", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
            ?.GetValue(_animationController) as TimeSpan? ?? TimeSpan.Zero;

        private void DrawSegmentAcrossSeam(int index, GCodeCommand3D cmd1, GCodeCommand3D cmd2, LinesVisual3D targetVisual)
        {
            // Chia segment thành nhiều phần nhỏ để vẽ mượt qua seam
            int subdivisions = 5;
            Point3D prevPoint = _allLocalToolpathPoints[index];

            for (int sub = 1; sub <= subdivisions; sub++)
            {
                double t = (double)sub / subdivisions;

                // Interpolate Y tuyến tính
                double interpY = cmd1.Y + (cmd2.Y - cmd1.Y) * t;

                // Interpolate C với xử lý seam
                double interpC = AngleHelper.InterpolateAngle(cmd1.C, cmd2.C, t);

                // Convert to 3D point
                Point3D currentPoint = ConvertGCodeCoordToLocalPointOnTubeSurface(
                    interpY, interpC, TubeDisplayRadius);

                // Add line segment
                targetVisual.Points.Add(prevPoint);
                targetVisual.Points.Add(currentPoint);

                prevPoint = currentPoint;
            }
        }

        public void ValidateGCodeForSeamCrossing()
        {
            if (_animationController.Commands == null) return;

            System.Diagnostics.Debug.WriteLine("\n=== G-CODE SEAM CROSSING ANALYSIS ===");
            int seamCrossingCount = 0;

            for (int i = 1; i < _animationController.Commands.Count; i++)
            {
                var cmd1 = _animationController.Commands[i - 1];
                var cmd2 = _animationController.Commands[i];

                if (AngleHelper.CrossesSeam(cmd1.C, cmd2.C))
                {
                    double delta = AngleHelper.ShortestAngleDelta(cmd1.C, cmd2.C);
                    seamCrossingCount++;

                    System.Diagnostics.Debug.WriteLine(
                        $"Line {i}: Seam crossing detected! " +
                        $"C{cmd1.C:F1} → C{cmd2.C:F1} " +
                        $"Delta: {delta:F1}° ({(delta > 0 ? "CCW" : "CW")})"
                    );
                }
            }

            System.Diagnostics.Debug.WriteLine($"Total seam crossings: {seamCrossingCount}");
            System.Diagnostics.Debug.WriteLine("=== END ANALYSIS ===\n");
        }

        private Point3D ConvertGCodeCoordToLocalPointOnTubeSurface(double y_gcodeValue, double c_gcodeDegrees, double tubeRadius)
        {
            double c_radians = c_gcodeDegrees * Math.PI / 180.0;
            double x_local = tubeRadius * Math.Cos(c_radians);
            double y_local = y_gcodeValue;
            double z_local = tubeRadius * Math.Sin(c_radians);
            return new Point3D(x_local, y_local, z_local);
        }

        private bool CanPlaySimulation() => _animationController.Commands != null && _animationController.Commands.Any() && !_animationController.IsPlaying;
        private void PlaySimulation() => _animationController.Play();

        private bool CanPauseSimulation() => _animationController.IsPlaying;
        private void PauseSimulation() => _animationController.Pause();

        private bool CanStopSimulation() => _animationController.IsPlaying || (_animationController.Commands != null && _animationController.Commands.Any());
        private void StopSimulation()
        {
            _animationController.Stop();

            // THÊM PHẦN NÀY: Clear toolpath để giải phóng memory
            _completedPoints.Clear();
            _completedPathVisual.Points.Clear();
            _currentAnimatingSegmentVisual.Points.Clear();
            _futurePathVisual.Points.Clear();
            _lastCompletedSegmentIndex = -1;
        }

        private bool CanResetSimulation() => _animationController.Commands != null && _animationController.Commands.Any();
        private void ResetSimulation()
        {
            _animationController.Reset();
            // Sau khi reset controller, nó sẽ tự cập nhật các thuộc tính,
            // điều này sẽ kích hoạt AnimationController_PropertyChanged và gọi UpdateToolpathVisuals
            // Hoặc gọi PrepareToolpathVisuals để vẽ lại trạng thái ban đầu hoàn toàn
            if (_animationController.Commands != null && _animationController.Commands.Any())
            {
                PrepareToolpathVisuals(new List<GCodeCommand3D>(_animationController.Commands), TubeDisplayRadius);
            }
            else
            {
                PrepareToolpathVisuals(new List<GCodeCommand3D>(), TubeDisplayRadius);
            }
        }

        public void LoadGCodeFromFilePathInternal(string filePath, double tubeRadius)
        {
            if (string.IsNullOrEmpty(filePath) || !System.IO.File.Exists(filePath))
            {
                MessageBox.Show($"G-code file not found: {filePath}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            LoadedGCodeFilePath = filePath; // Giả sử bạn có thuộc tính này
            try
            {
                List<GCodeCommand3D> commands = _gcodeParser.Parse(LoadedGCodeFilePath);
                if (commands == null || !commands.Any())
                {
                    MessageBox.Show("G-code file is empty or could not be parsed.", "Parsing Error", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                // Cập nhật TubeDisplayRadius nếu cần, hoặc nó đã được thiết lập
                if (tubeRadius > 0) TubeDisplayRadius = tubeRadius;

                _animationController.LoadCommands(commands, TubeDisplayRadius);
                PrepareToolpathVisuals(commands, TubeDisplayRadius);

                (PlayCommand as CommunityToolkit.Mvvm.Input.RelayCommand)?.NotifyCanExecuteChanged();
                (ResetCommand as CommunityToolkit.Mvvm.Input.RelayCommand)?.NotifyCanExecuteChanged();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error loading or parsing G-code file: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }

        }

        public void DebugWireframeStatus()
        {
            System.Diagnostics.Debug.WriteLine("=== WIREFRAME DEBUG INFO ===");
            System.Diagnostics.Debug.WriteLine($"WireframeOnTubeGroup null? {_wireframeOnTubeGroup == null}");

            if (_wireframeOnTubeGroup != null)
            {
                System.Diagnostics.Debug.WriteLine($"Children count: {_wireframeOnTubeGroup.Children.Count}");
                System.Diagnostics.Debug.WriteLine($"Transform: {_wireframeOnTubeGroup.Transform}");

                foreach (var child in _wireframeOnTubeGroup.Children)
                {
                    if (child is GeometryModel3D gm && gm.Geometry is MeshGeometry3D mesh)
                    {
                        System.Diagnostics.Debug.WriteLine($"  Mesh: {mesh.Positions.Count} positions");
                        if (mesh.Positions.Count > 0)
                        {
                            var firstPos = mesh.Positions[0];
                            System.Diagnostics.Debug.WriteLine($"    First position: ({firstPos.X:F2}, {firstPos.Y:F2}, {firstPos.Z:F2})");
                        }
                    }
                }
            }
            System.Diagnostics.Debug.WriteLine("=== END DEBUG ===");
        }

        public void DebugTubeAndWireframeDimensions()
        {
            System.Diagnostics.Debug.WriteLine("\n=== TUBE VS WIREFRAME DIMENSIONS ===");

            // Tube info
            System.Diagnostics.Debug.WriteLine($"TUBE:");
            System.Diagnostics.Debug.WriteLine($"  Radius: {TubeDisplayRadius:F2}mm (Diameter: {TubeDisplayRadius * 2:F2}mm)");
            System.Diagnostics.Debug.WriteLine($"  Length: {TubeDisplayLength:F2}mm");
            System.Diagnostics.Debug.WriteLine($"  Y Range: [0, {TubeDisplayLength:F2}]");

            // Wireframe info
            if (_wireframeOnTubeGroup != null && _wireframeOnTubeGroup.Bounds != Rect3D.Empty)
            {
                var bounds = _wireframeOnTubeGroup.Bounds;
                System.Diagnostics.Debug.WriteLine($"\nWIREFRAME ON TUBE:");
                System.Diagnostics.Debug.WriteLine($"  X Range: [{bounds.X:F2}, {bounds.X + bounds.SizeX:F2}] (Size: {bounds.SizeX:F2})");
                System.Diagnostics.Debug.WriteLine($"  Y Range: [{bounds.Y:F2}, {bounds.Y + bounds.SizeY:F2}] (Size: {bounds.SizeY:F2})");
                System.Diagnostics.Debug.WriteLine($"  Z Range: [{bounds.Z:F2}, {bounds.Z + bounds.SizeZ:F2}] (Size: {bounds.SizeZ:F2})");

                // Check alignment
                double wireframeCenterY = bounds.Y + bounds.SizeY / 2.0;
                double tubeCenterY = TubeDisplayLength / 2.0;
                double yMisalignment = Math.Abs(wireframeCenterY - tubeCenterY);

                System.Diagnostics.Debug.WriteLine($"\nALIGNMENT CHECK:");
                System.Diagnostics.Debug.WriteLine($"  Tube center Y: {tubeCenterY:F2}");
                System.Diagnostics.Debug.WriteLine($"  Wireframe center Y: {wireframeCenterY:F2}");
                System.Diagnostics.Debug.WriteLine($"  Y Misalignment: {yMisalignment:F2}mm");

                // Check if wireframe fits in tube
                double maxWireframeRadiusXZ = Math.Max(
                    Math.Max(Math.Abs(bounds.X), Math.Abs(bounds.X + bounds.SizeX)),
                    Math.Max(Math.Abs(bounds.Z), Math.Abs(bounds.Z + bounds.SizeZ))
                );

                System.Diagnostics.Debug.WriteLine($"\nSIZE CHECK:");
                System.Diagnostics.Debug.WriteLine($"  Max wireframe radius (X-Z): {maxWireframeRadiusXZ:F2}mm");
                System.Diagnostics.Debug.WriteLine($"  Tube radius: {TubeDisplayRadius:F2}mm");
                System.Diagnostics.Debug.WriteLine($"  Wireframe fits in tube: {(maxWireframeRadiusXZ <= TubeDisplayRadius ? "YES" : "NO")}");
            }

            System.Diagnostics.Debug.WriteLine("=== END DIMENSIONS DEBUG ===\n");
        }
        public void FlipWireframeYDirection()
        {
            if (_wireframeOnTubeGroup == null) return;

            System.Diagnostics.Debug.WriteLine("\n=== FLIPPING WIREFRAME Y DIRECTION ===");
            System.Diagnostics.Debug.WriteLine($"Tube length: {TubeDisplayLength}");

            // Tạo một Model3DGroup mới để chứa wireframe đã flip
            var flippedGroup = new Model3DGroup();

            foreach (var child in _wireframeOnTubeGroup.Children)
            {
                if (child is GeometryModel3D model && model.Geometry is MeshGeometry3D originalMesh)
                {
                    var newMesh = new MeshGeometry3D();

                    // Debug first few points before flip
                    System.Diagnostics.Debug.WriteLine($"\nFlipping mesh with {originalMesh.Positions.Count} positions");
                    for (int i = 0; i < Math.Min(3, originalMesh.Positions.Count); i++)
                    {
                        var oldPos = originalMesh.Positions[i];
                        System.Diagnostics.Debug.WriteLine($"  Point {i}: Y={oldPos.Y:F2} -> Y={TubeDisplayLength - oldPos.Y:F2}");
                    }

                    // Transform positions - flip Y coordinate
                    foreach (var pos in originalMesh.Positions)
                    {
                        var flippedPos = new Point3D(
                            pos.X,
                            TubeDisplayLength - pos.Y,  // Đảo ngược Y
                            pos.Z
                        );
                        newMesh.Positions.Add(flippedPos);
                    }

                    // Copy triangle indices
                    foreach (var idx in originalMesh.TriangleIndices)
                    {
                        newMesh.TriangleIndices.Add(idx);
                    }

                    // Transform normals - cũng cần đảo ngược Y component
                    if (originalMesh.Normals != null && originalMesh.Normals.Count > 0)
                    {
                        foreach (var normal in originalMesh.Normals)
                        {
                            var flippedNormal = new Vector3D(normal.X, -normal.Y, normal.Z);
                            flippedNormal.Normalize();
                            newMesh.Normals.Add(flippedNormal);
                        }
                    }

                    // Create new model with flipped mesh
                    var newModel = new GeometryModel3D
                    {
                        Geometry = newMesh,
                        Material = model.Material,
                        BackMaterial = model.BackMaterial
                    };

                    flippedGroup.Children.Add(newModel);
                }
            }

            // Replace the old group with flipped group
            _wireframeOnTubeGroup = flippedGroup;

            // Set flag
            IsWireframeFlipped = true;

            // Reapply transform if exists
            if (TubeTransform != null)
            {
                _wireframeOnTubeGroup.Transform = TubeTransform;
            }

            System.Diagnostics.Debug.WriteLine("=== WIREFRAME FLIPPED SUCCESSFULLY ===\n");

            // QUAN TRỌNG: Force update bounds
            OnPropertyChanged(nameof(WireframeOnTubeGroup));

            // Debug after flip với bounds mới
            System.Windows.Application.Current.Dispatcher.BeginInvoke(new Action(() =>
            {
                DebugWireframeDirection();
            }), System.Windows.Threading.DispatcherPriority.Background);
        }
        public void DebugWireframeDirection()
        {
            System.Diagnostics.Debug.WriteLine("\n=== WIREFRAME DIRECTION DEBUG ===");

            if (_wireframeOnTubeGroup == null)
            {
                System.Diagnostics.Debug.WriteLine("ERROR: WireframeOnTubeGroup is NULL!");
                return;
            }

            System.Diagnostics.Debug.WriteLine($"Cutter starts at Y=0");
            System.Diagnostics.Debug.WriteLine($"Tube length: {TubeDisplayLength:F2}");
            System.Diagnostics.Debug.WriteLine($"Wireframe children count: {_wireframeOnTubeGroup.Children.Count}");

            // Tìm min và max Y của wireframe
            double minY = double.MaxValue;
            double maxY = double.MinValue;
            int totalPoints = 0;

            foreach (var child in _wireframeOnTubeGroup.Children)
            {
                if (child is GeometryModel3D model && model.Geometry is MeshGeometry3D mesh)
                {
                    totalPoints += mesh.Positions.Count;
                    foreach (var pos in mesh.Positions)
                    {
                        minY = Math.Min(minY, pos.Y);
                        maxY = Math.Max(maxY, pos.Y);
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"Total wireframe points: {totalPoints}");
            System.Diagnostics.Debug.WriteLine($"Wireframe Y range: [{minY:F2}, {maxY:F2}]");

            // Kiểm tra xem wireframe có bắt đầu từ đúng đầu không
            if (minY < TubeDisplayLength / 3.0)
            {
                System.Diagnostics.Debug.WriteLine("⚠️ Wireframe starts near Y=0 (same as cutter)");
                System.Diagnostics.Debug.WriteLine("   Need to flip wireframe direction!");
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("✅ Wireframe starts from opposite end");
            }

            System.Diagnostics.Debug.WriteLine("=== END DEBUG ===\n");
        }
        private void UpdateToolpathTransforms()
        {
            var transform = TubeTransform ?? Transform3D.Identity;

            // Update transform cho tất cả toolpath visuals
            if (_completedPathVisual != null)
                _completedPathVisual.Transform = transform;

            if (_futurePathVisual != null)
                _futurePathVisual.Transform = transform;

            if (_currentAnimatingSegmentVisual != null)
                _currentAnimatingSegmentVisual.Transform = transform;

            // Debug
            System.Diagnostics.Debug.WriteLine($"Updated toolpath transforms. Transform: {transform}");
        }

        public void DebugCurrentTransformState()
        {
            System.Diagnostics.Debug.WriteLine("\n=== TRANSFORM STATE DEBUG ===");
            System.Diagnostics.Debug.WriteLine($"AnimationController.CurrentCRotation: {_animationController?.CurrentCRotation ?? 0:F3}°");
            System.Diagnostics.Debug.WriteLine($"CurrentCRotationSimulation: {CurrentCRotationSimulation:F3}°");
            System.Diagnostics.Debug.WriteLine($"TubeTransform: {TubeTransform}");

            if (TubeTransform is Transform3DGroup group)
            {
                System.Diagnostics.Debug.WriteLine($"Transform group has {group.Children.Count} children:");
                foreach (var child in group.Children)
                {
                    if (child is RotateTransform3D rot)
                    {
                        var axis = rot.Rotation as AxisAngleRotation3D;
                        System.Diagnostics.Debug.WriteLine($"  - Rotation: {axis?.Angle ?? 0:F3}° around {axis?.Axis}");
                    }
                    else if (child is TranslateTransform3D trans)
                    {
                        System.Diagnostics.Debug.WriteLine($"  - Translation: ({trans.OffsetX:F3}, {trans.OffsetY:F3}, {trans.OffsetZ:F3})");
                    }
                }
            }
            System.Diagnostics.Debug.WriteLine("=== END DEBUG ===\n");
        }
    }
}