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

namespace TubeLaserCAM.UI.ViewModels
{
    public class Simulation3DViewModel : ObservableObject
    {
        private AnimationController3D _animationController;
        private GCodeParser3D _gcodeParser;
        private LinesVisual3D _completedPathVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Green, Thickness = 1.5 };
        private LinesVisual3D _futurePathVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.LightGray, Thickness = 1 };
        private LinesVisual3D _currentAnimatingSegmentVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Red, Thickness = 2 };
        public Simulation3DViewModel()
        {
            _animationController = new AnimationController3D();
            _gcodeParser = new GCodeParser3D();

            _animationController.PropertyChanged += AnimationController_PropertyChanged;

            LoadGCodeCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(LoadGCodeFile);
            PlayCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(PlaySimulation, CanPlaySimulation);
            PauseCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(PauseSimulation, CanPauseSimulation);
            StopCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(StopSimulation, CanStopSimulation);
            ResetCommand = new CommunityToolkit.Mvvm.Input.RelayCommand(ResetSimulation, CanResetSimulation);

            InitializeDefaultModels();
        }


        private void InitializeDefaultModels()
        {
            TubeDisplayRadius = 25;
            TubeDisplayLength = 300;
            CurrentZSimulation = 20;

            var tubeMeshBuilder = new MeshBuilder(false, false);
            tubeMeshBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, TubeDisplayLength, 0), TubeDisplayRadius, 36);

            // SỬA LỖI CS1061: Thay thế ToMeshGeometry3D()
            var tubeHelixMesh = tubeMeshBuilder.ToMesh(true); // true để freeze (đóng băng)
            var tubeWpfMesh = new MeshGeometry3D
            {
                Positions = tubeHelixMesh.Positions,
                TriangleIndices = tubeHelixMesh.TriangleIndices,
                Normals = tubeHelixMesh.Normals,
                TextureCoordinates = tubeHelixMesh.TextureCoordinates
            };
            TubeGeometry = new GeometryModel3D(tubeWpfMesh, Materials.Blue); // Sử dụng cùng Material như trên hoặc một Material phù hợp


            var cutterMeshBuilder = new MeshBuilder(false, false);
            cutterMeshBuilder.AddArrow(new Point3D(0, 0, CurrentZSimulation + 15), new Point3D(0, 0, CurrentZSimulation), 3, 10, 4);

            // SỬA LỖI CS1061: Thay thế ToMeshGeometry3D()
            var cutterHelixMesh = cutterMeshBuilder.ToMesh(true);
            var cutterWpfMesh = new MeshGeometry3D
            {
                Positions = cutterHelixMesh.Positions,
                TriangleIndices = cutterHelixMesh.TriangleIndices,
                Normals = cutterHelixMesh.Normals,
                TextureCoordinates = cutterHelixMesh.TextureCoordinates
            };
            CutterGeometry = new GeometryModel3D(cutterWpfMesh, Materials.Red); // Hoặc một Material khác bạn muốn

        }

        private void AnimationController_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(AnimationController3D.CurrentYPosition):
                    CurrentYSimulation = _animationController.CurrentYPosition;
                    break;
                case nameof(AnimationController3D.CurrentCRotation):
                    CurrentCRotationSimulation = _animationController.CurrentCRotation;
                    break;
                case nameof(AnimationController3D.CurrentZPosition):
                    CurrentZSimulation = _animationController.CurrentZPosition;
                    UpdateCutterTransform();
                    break;
                case nameof(AnimationController3D.IsCurrentLaserOn):
                case nameof(AnimationController3D.SimulationProgress):
                    UpdateToolpathVisuals(_animationController.IsCurrentLaserOn, _animationController.SimulationProgress);
                    break;
                case nameof(AnimationController3D.IsPlaying):
                    // RelayCommand tự động xử lý việc cập nhật CanExecute khi các thuộc tính thay đổi nếu bạn dùng [RelayCommand(CanExecute = nameof(CanPlaySimulation))]
                    // Hoặc bạn có thể gọi NotifyCanExecuteChanged() thủ công nếu cần.
                    // Với CommunityToolkit.Mvvm, các thuộc tính công khai sẽ tự động kích hoạt RequerySuggested cho CommandManager.
                    // Nếu bạn dùng phiên bản cũ hơn hoặc custom RelayCommand, bạn có thể cần:
                    // (PlayCommand as RelayCommand)?.NotifyCanExecuteChanged();
                    // (PauseCommand as RelayCommand)?.NotifyCanExecuteChanged();
                    // (StopCommand as RelayCommand)?.NotifyCanExecuteChanged();
                    // (ResetCommand as RelayCommand)?.NotifyCanExecuteChanged();
                    break;
            }
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
            var tubeMeshBuilder = new MeshBuilder(false, false);
            tubeMeshBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, TubeDisplayLength, 0), TubeDisplayRadius, 36);

            // SỬA LỖI CS1061: Thay thế ToMeshGeometry3D()
            var tubeHelixMesh = tubeMeshBuilder.ToMesh(true); // true để freeze
            var tubeWpfMesh = new MeshGeometry3D
            {
                Positions = tubeHelixMesh.Positions,
                TriangleIndices = tubeHelixMesh.TriangleIndices,
                Normals = tubeHelixMesh.Normals,
                TextureCoordinates = tubeHelixMesh.TextureCoordinates
            };
            TubeGeometry = new GeometryModel3D(tubeWpfMesh, Materials.Blue); // Hoặc một Material khác bạn muốn

        }

        private void UpdateCutterTransform()
        {
            // Giả sử đầu cắt chỉ di chuyển theo Z trong hệ tọa độ thế giới
            CutterTransform = new TranslateTransform3D(0, 0, CurrentZSimulation + TubeDisplayRadius); // Điều chỉnh vị trí Z của đầu cắt cho phù hợp
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
                    UpdateTubeTransform();
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
            var group = new Transform3DGroup();
            group.Children.Add(new TranslateTransform3D(0, CurrentYSimulation, 0));
            group.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), CurrentCRotationSimulation), new Point3D(0, CurrentYSimulation, 0)));
            TubeTransform = group;
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
            if (!ToolpathVisuals.Contains(_completedPathVisual)) ToolpathVisuals.Add(_completedPathVisual);
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
            }
            _completedPathVisual.Points.Clear();
            _currentAnimatingSegmentVisual.Points.Clear();
        }

        private LinesVisual3D _currentCutSegmentVisual = new LinesVisual3D { Color = System.Windows.Media.Colors.Red, Thickness = 2 };

        private void UpdateToolpathVisuals(bool isLaserOn, double progress) // progress là từ AnimationController (0.0 đến 1.0)
        {
            int totalLocalPoints = _allLocalToolpathPoints.Count;
            if (totalLocalPoints < 1) return;

            _completedPathVisual.Points.Clear();
            _futurePathVisual.Points.Clear();
            _currentAnimatingSegmentVisual.Points.Clear();

            // currentSegmentOverallProgressIndex là chỉ số của điểm cuối của segment đang được xử lý hoặc vừa hoàn thành
            // Dựa trên progress tổng của animation controller
            int currentSegmentOverallProgressIndex = (int)(_animationController.SimulationProgress * (_animationController.Commands?.Count ?? 0));
            if (currentSegmentOverallProgressIndex >= _animationController.Commands.Count && _animationController.Commands.Any())
            {
                currentSegmentOverallProgressIndex = _animationController.Commands.Count - 1;
            }


            // 1. Vẽ các đoạn đã hoàn toàn cắt (laser on trong quá khứ)
            for (int i = 0; i < currentSegmentOverallProgressIndex && i < totalLocalPoints - 1; i++)
            {
                // Lệnh commands[i] là lệnh để đi từ điểm _allLocalToolpathPoints[i-1] tới _allLocalToolpathPoints[i]
                // Đoạn nối _allLocalToolpathPoints[i] và _allLocalToolpathPoints[i+1] được quyết định bởi lệnh commands[i+1]
                if (_animationController.Commands.Count > i + 1)
                { // Đảm bảo có lệnh tiếp theo để kiểm tra
                    var cmdForSegmentEnd = _animationController.Commands[i + 1];
                    if (cmdForSegmentEnd.IsLaserOn && !cmdForSegmentEnd.IsRapidMove)
                    {
                        _completedPathVisual.Points.Add(_allLocalToolpathPoints[i]);
                        _completedPathVisual.Points.Add(_allLocalToolpathPoints[i + 1]);
                    }
                }
            }

            // 2. Vẽ đoạn đang được cắt (nếu laser on)
            // currentCommandIndex của AnimationController là index của lệnh *target* hiện tại
            int currentTargetCommandIndex = _animationController.CurrentCommandIndex;
            if (isLaserOn && currentTargetCommandIndex < _animationController.Commands.Count && currentTargetCommandIndex < totalLocalPoints)
            {
                Point3D segmentStartPoint;
                if (currentTargetCommandIndex == 0)
                { // Nếu là lệnh đầu tiên, điểm bắt đầu là chính nó (đoạn nội suy từ chính nó đến nó)
                    segmentStartPoint = _allLocalToolpathPoints[0];
                }
                else if (currentTargetCommandIndex > 0 && (currentTargetCommandIndex - 1) < totalLocalPoints)
                {
                    segmentStartPoint = _allLocalToolpathPoints[currentTargetCommandIndex - 1];
                }
                else
                { // Fallback, không nên xảy ra nếu logic đúng
                    segmentStartPoint = _allLocalToolpathPoints[0];
                }

                Point3D animatedCurrentPoint = ConvertGCodeCoordToLocalPointOnTubeSurface(
                                                    _animationController.CurrentYPosition,
                                                    _animationController.CurrentCRotation,
                                                    TubeDisplayRadius);

                // Chỉ vẽ nếu điểm bắt đầu và điểm hiện tại khác nhau (có sự di chuyển)
                if (!segmentStartPoint.Equals(animatedCurrentPoint))
                {
                    _currentAnimatingSegmentVisual.Points.Add(segmentStartPoint);
                    _currentAnimatingSegmentVisual.Points.Add(animatedCurrentPoint);
                }
            }

            // 3. Vẽ các đoạn tương lai (chưa được cắt)
            // Bắt đầu từ điểm cuối của đoạn đang cắt, hoặc điểm cuối của các đoạn đã hoàn thành
            int futurePathStartIndex = currentSegmentOverallProgressIndex;
            if (isLaserOn && currentTargetCommandIndex < totalLocalPoints) // Nếu đang cắt, thì tương lai bắt đầu từ điểm đích của lệnh hiện tại
            {
                futurePathStartIndex = currentTargetCommandIndex;
            }


            for (int i = futurePathStartIndex; i < totalLocalPoints - 1; i++)
            {
                _futurePathVisual.Points.Add(_allLocalToolpathPoints[i]);
                _futurePathVisual.Points.Add(_allLocalToolpathPoints[i + 1]);
            }
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
        private void StopSimulation() => _animationController.Stop();

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
    }
}