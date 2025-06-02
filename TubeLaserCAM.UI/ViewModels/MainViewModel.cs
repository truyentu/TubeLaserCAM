// TubeLaserCAM.UI/ViewModels/MainViewModel.cs
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows.Controls;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using GeometryWrapper;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using TubeLaserCAM.UI.Models;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.IO;
using static TubeLaserCAM.UI.Models.GeometryModel;
using TubeLaserCAM.UI.Views;
using Point3D = System.Windows.Media.Media3D.Point3D;
using Vector3D = System.Windows.Media.Media3D.Vector3D;

namespace TubeLaserCAM.UI.ViewModels
{
    public class RelayCommand : ICommand
    {
        private Action<object> execute;
        private Func<object, bool> canExecute;

        public RelayCommand(Action<object> execute, Func<object, bool> canExecute = null)
        {
            this.execute = execute;
            this.canExecute = canExecute;
        }

        public event EventHandler CanExecuteChanged
        {
            add { CommandManager.RequerySuggested += value; }
            remove { CommandManager.RequerySuggested -= value; }
        }

        public bool CanExecute(object parameter)
        {
            return canExecute == null || canExecute(parameter);
        }

        public void Execute(object parameter)
        {
            execute(parameter);
        }
    }

    public enum SelectionMode
    {
        Single,
        Multiple,
        Chain
    }

    public class MainViewModel : INotifyPropertyChanged
    {
        private GeometryModel geometryModel;
        private Model3DGroup wireframeModel;
        private string statusText;
        private bool showSolidMode = false;
        private ObservableCollection<EdgeInfoWrapper> edgeList;
        private CylinderData cylinderInfo;
        private bool showAxis = true;
        private Model3DGroup axisModel;
        private SelectionMode currentSelectionMode = SelectionMode.Single;
        private ObservableCollection<EdgeSelectionInfo> selectedToolpaths;
        private GCodeSettings gcodeSettings = new GCodeSettings();
        private int? hoveredEdgeId;
        private bool isCameraLocked = false;
        private ObservableCollection<ToolpathSuggestion> suggestedToolpaths;
        public ObservableCollection<ToolpathSuggestion> SuggestedToolpaths
        {
            get => suggestedToolpaths;
            set
            {
                suggestedToolpaths = value;
                OnPropertyChanged();
            }
        }

        private ObservableCollection<UnrolledToolpath> unrolledToolpaths;
        public ObservableCollection<UnrolledToolpath> UnrolledToolpaths
        {
            get => unrolledToolpaths;
            set
            {
                unrolledToolpaths = value;
                OnPropertyChanged();
            }
        }
        public bool IsCameraLocked
        {
            get => isCameraLocked;
            set
            {
                isCameraLocked = value;
                OnPropertyChanged();
            }
        }
        public SelectionMode CurrentSelectionMode
        {
            get => currentSelectionMode;
            set
            {
                currentSelectionMode = value;
                OnPropertyChanged();
            }
        }

        public ObservableCollection<EdgeSelectionInfo> SelectedToolpaths
        {
            get => selectedToolpaths;
            set
            {
                selectedToolpaths = value;
                OnPropertyChanged();
            }
        }
        public ICommand MouseMoveCommand { get; }
        public ICommand MouseDownCommand { get; }
        public ICommand ClearSelectionCommand { get; }
        public ICommand DeleteSelectedCommand { get; }
        public ICommand MoveToolpathUpCommand { get; }
        public ICommand MoveToolpathDownCommand { get; }
        public ICommand SaveConfigurationCommand { get; }
        public ICommand LoadConfigurationCommand { get; }
        public ICommand ApplyFilterCommand { get; }
        public ICommand ClearFilterCommand { get; }
        public ICommand AutoSuggestToolpathsCommand { get; }
        public ICommand AcceptSuggestionCommand { get; }
        public ICommand RecognizeShapesCommand { get; }
        public ICommand UnrollToolpathsCommand { get; }
        public ICommand GenerateGCodeCommand { get; }
        public ICommand ToggleCameraLockCommand { get; }
        public ICommand SetFrontViewCommand { get; }
        public ICommand SetTopViewCommand { get; }
        public ICommand SetSideViewCommand { get; }



        public MainViewModel()
        {
            geometryModel = new GeometryModel();
            EdgeList = new ObservableCollection<EdgeInfoWrapper>();
            SelectedToolpaths = new ObservableCollection<EdgeSelectionInfo>();
            UnrolledToolpaths = new ObservableCollection<UnrolledToolpath>();

            LoadFileCommand = new RelayCommand(ExecuteLoadFile);
            MouseMoveCommand = new RelayCommand(ExecuteMouseMove);
            MouseDownCommand = new RelayCommand(ExecuteMouseDown);
            ClearSelectionCommand = new RelayCommand(ExecuteClearSelection);
            DeleteSelectedCommand = new RelayCommand(ExecuteDeleteSelected);

            StatusText = "Ready. Click 'Load STEP File' to begin.";
            processedInChain = new HashSet<int>();
            MoveToolpathUpCommand = new RelayCommand(ExecuteMoveToolpathUp, CanMoveToolpathUp);
            MoveToolpathDownCommand = new RelayCommand(ExecuteMoveToolpathDown, CanMoveToolpathDown);
            SaveConfigurationCommand = new RelayCommand(ExecuteSaveConfiguration);
            LoadConfigurationCommand = new RelayCommand(ExecuteLoadConfiguration);
            ApplyFilterCommand = new RelayCommand(ExecuteApplyFilter);
            ClearFilterCommand = new RelayCommand(ExecuteClearFilter);
            SuggestedToolpaths = new ObservableCollection<ToolpathSuggestion>();
            AutoSuggestToolpathsCommand = new RelayCommand(ExecuteAutoSuggestToolpaths);
            AcceptSuggestionCommand = new RelayCommand(ExecuteAcceptSuggestion);
            RecognizeShapesCommand = new RelayCommand(ExecuteRecognizeShapes);
            UnrollToolpathsCommand = new RelayCommand(ExecuteUnrollToolpaths);
            GenerateGCodeCommand = new RelayCommand(ExecuteGenerateGCode);
            ToggleCameraLockCommand = new RelayCommand(ExecuteToggleCameraLock);


        }

        public Model3DGroup WireframeModel
        {
            get => wireframeModel;
            set
            {
                wireframeModel = value;
                OnPropertyChanged();
            }
        }

        public string StatusText
        {
            get => statusText;
            set
            {
                statusText = value;
                OnPropertyChanged();
            }
        }

        public ObservableCollection<EdgeInfoWrapper> EdgeList
        {
            get => edgeList;
            set
            {
                edgeList = value;
                OnPropertyChanged();
            }
        }

        public ICommand LoadFileCommand { get; }

        private void ExecuteLoadFile(object parameter)
        {
            var openFileDialog = new OpenFileDialog
            {
                Filter = "STEP Files (*.step;*.stp)|*.step;*.stp|All Files (*.*)|*.*",
                Title = "Select STEP File"
            };

            if (openFileDialog.ShowDialog() == true)
            {
                try
                {
                    StatusText = "Loading file...";

                    if (geometryModel.LoadStepFile(openFileDialog.FileName))
                    {
                        WireframeModel = geometryModel.CreateWireframeModel();
                        CylinderInfo = geometryModel.GetCylinderInfo();

                        if (CylinderInfo.IsValid)
                        {
                            AxisModel = null;
                            AxisModel = geometryModel.CreateAxisVisualization();
                            StatusText = $"File loaded. Found cylinder: R={CylinderInfo.Radius:F2}mm, L={CylinderInfo.Length:F2}mm, {geometryModel.GetEdgeCount()} edges.";
                        }
                        else
                        {
                            StatusText = $"File loaded. No cylinder detected. Found {geometryModel.GetEdgeCount()} edges.";
                        }

                        EdgeList.Clear();
                        // geometryModel.GetEdgeInfo() returns List<ManagedEdgeInfo>
                        // We need to get the EdgeSelectionInfo from geometryModel which now contains classification
                        var allEdgeSelectionInfos = geometryModel.GetAllEdgeInfos();
                        System.Diagnostics.Debug.WriteLine($"Got {allEdgeSelectionInfos?.Count ?? 0} edge infos");

                        if (allEdgeSelectionInfos != null)
                        {
                            foreach (var kvp in allEdgeSelectionInfos.OrderBy(e => e.Key)) // Order by ID for consistency
                            {
                                var edgeSelectionInfo = kvp.Value;
                                var wrapper = new EdgeInfoWrapper(edgeSelectionInfo.EdgeInfo); // Contains Id, Type, Length
                                if (edgeSelectionInfo.Classification != null)
                                {
                                    // Manually map EdgeClassificationData to the wrapper's Classification property
                                    // This assumes EdgeInfoWrapper has a property 'Classification' of type Models.EdgeClassificationData
                                    // and Models.EdgeClassificationData has matching enums/properties.
                                    wrapper.Classification = new Models.EdgeClassificationData
                                    {
                                        Location = (Models.EdgeLocation)edgeSelectionInfo.Classification.Location, // Enum cast
                                        ShapeType = (Models.EdgeShapeType)edgeSelectionInfo.Classification.ShapeType, // Enum cast
                                        IsOnCylinderSurface = edgeSelectionInfo.Classification.IsOnCylinderSurface,
                                        OriginalEdgeId = edgeSelectionInfo.Classification.OriginalEdgeId
                                    };
                                }
                                EdgeList.Add(wrapper);
                            }
                        }
                        UpdateVisualization();
                    }
                    else
                    {
                        StatusText = "Failed to load file.";
                    }
                }
                catch (Exception ex)
                {
                    StatusText = $"Error: {ex.Message}";
                }
            }
        }

        private void ExecuteGenerateGCode(object parameter)
        {
            if (UnrolledToolpaths.Count == 0)
            {
                StatusText = "No unrolled toolpaths to generate G-Code";
                return;
            }

            try
            {
                var settingsDialog = new Views.GCodeSettingsDialog(gcodeSettings);

                if (settingsDialog.ShowDialog() == true)
                {
                    gcodeSettings = settingsDialog.Settings;

                    // Generate G-Code
                    var generator = new GCodeGenerator(gcodeSettings);
                    System.Diagnostics.Debug.WriteLine("=== G-Code Generation Debug ===");
                    System.Diagnostics.Debug.WriteLine($"Total toolpaths: {UnrolledToolpaths.Count}");
                    System.Diagnostics.Debug.WriteLine($"Optimize sequence: {gcodeSettings.OptimizeSequence}");
                    System.Diagnostics.Debug.WriteLine($"Complete profile: {gcodeSettings.CuttingStrategy.CompleteProfileBeforeMoving}");
                    System.Diagnostics.Debug.WriteLine($"Y Direction: {gcodeSettings.CuttingStrategy.YDirection}");
                    var gcode = generator.GenerateGCode(
                        UnrolledToolpaths.ToList(),
                        CylinderInfo
                    );
                    System.Diagnostics.Debug.WriteLine("=== Generated G-Code (first 50 lines) ===");
                    var lines = gcode.Split('\n').Take(50);
                    foreach (var line in lines)
                    {
                        System.Diagnostics.Debug.WriteLine(line.TrimEnd());
                    }
                    System.Diagnostics.Debug.WriteLine("\n=== All Movement Commands ===");
                    var moveLines = gcode.Split('\n')
                        .Where(l => l.TrimStart().StartsWith("G0") || l.TrimStart().StartsWith("G1"))
                        .Take(20);
                    foreach (var line in moveLines)
                    {
                        System.Diagnostics.Debug.WriteLine(line.TrimEnd());
                    }

                    // Save dialog
                    var saveDialog = new SaveFileDialog
                    {
                        Filter = "G-Code Files (*.nc;*.gcode)|*.nc;*.gcode|All Files (*.*)|*.*",
                        DefaultExt = ".nc",
                        FileName = $"{gcodeSettings.ProgramName}.nc"
                    };

                    if (saveDialog.ShowDialog() == true)
                    {
                        System.IO.File.WriteAllText(saveDialog.FileName, gcode);
                        StatusText = $"G-Code saved to {System.IO.Path.GetFileName(saveDialog.FileName)}";

                        // Ask if user wants to preview
                        var result = MessageBox.Show(
                            "G-Code generated successfully. Do you want to preview the toolpath?",
                            "Preview Toolpath",
                            MessageBoxButton.YesNo,
                            MessageBoxImage.Question);

                        if (result == MessageBoxResult.Yes)
                        {
                            // Find existing 2D view window or use the current one
                            var existingWindow = Application.Current.Windows
                                .OfType<Unrolled2DView>()
                                .FirstOrDefault();

                            if (existingWindow != null)
                            {
                                existingWindow.LoadGCodeForVisualization(gcode);
                                existingWindow.Activate();
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                StatusText = $"Error generating G-Code: {ex.Message}";
                MessageBox.Show(ex.Message, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void UpdateVisualization()
        {
            if (geometryModel != null)
            {
                geometryModel.CurrentRenderMode = showSolidMode
                    ? GeometryModel.RenderMode.SolidWithCuts
                    : GeometryModel.RenderMode.Wireframe;

                // Force refresh
                WireframeModel = null;
                OnPropertyChanged(nameof(WireframeModel));

                WireframeModel = geometryModel.CreateVisualization();
                OnPropertyChanged(nameof(WireframeModel));
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
        public CylinderData CylinderInfo
        {
            get => cylinderInfo;
            set
            {
                cylinderInfo = value;
                OnPropertyChanged();
            }
        }

        public bool ShowAxis
        {
            get => showAxis;
            set
            {
                showAxis = value;
                OnPropertyChanged();
                if (value && cylinderInfo != null && cylinderInfo.IsValid)
                {
                    AxisModel = null;
                    AxisModel = geometryModel.CreateAxisVisualization();
                }
                else if (!value)
                {
                    AxisModel = null; // Clear khi tắt
                }
            }
        }
        public bool ShowSolidMode
        {
            get => showSolidMode;
            set
            {
                showSolidMode = value;
                OnPropertyChanged();
                UpdateVisualization();
            }
        }

        public Model3DGroup AxisModel
        {
            get => axisModel;
            set
            {
                axisModel = value;
                OnPropertyChanged();
            }
        }

        private void ExecuteMouseMove(object parameter)
        {
            if (parameter is MouseEventArgs args && geometryModel != null && WireframeModel != null)
            {
                var viewport = FindViewport3D(args.Source);
                if (viewport == null) return;

                var mousePos = args.GetPosition(viewport);
                var hitResult = GetHitTestResult(viewport, mousePos);

                if (hitResult != null)
                {
                    var nearestEdgeId = FindEdgeIdFromHitResult(hitResult);
                    if (hoveredEdgeId != nearestEdgeId)
                    {
                        if (hoveredEdgeId.HasValue)
                            geometryModel.SetEdgeHovered(hoveredEdgeId.Value, false);
                        if (nearestEdgeId.HasValue)
                            geometryModel.SetEdgeHovered(nearestEdgeId.Value, true);
                        hoveredEdgeId = nearestEdgeId;
                        if (nearestEdgeId.HasValue)
                        {
                            var edgeInfo = geometryModel.GetEdgeInfo(nearestEdgeId.Value);
                            StatusText = $"Hovering Edge #{nearestEdgeId.Value} - Type: {edgeInfo?.EdgeInfo.Type}";
                        }
                    }
                }
                else if (hoveredEdgeId.HasValue)
                {
                    geometryModel.SetEdgeHovered(hoveredEdgeId.Value, false);
                    hoveredEdgeId = null;
                    UpdateStatusText();
                }
            }
        }

        private void ExecuteSetFrontView(object parameter)
        {
            // Camera nhìn dọc theo Z axis
            RequestCameraPosition?.Invoke(
                new Point3D(0, 0, 500),      // Position
                new Vector3D(0, 0, -1),      // LookDirection
                new Vector3D(0, 1, 0)        // UpDirection
            );
        }

        private void ExecuteSetTopView(object parameter)
        {
            // Camera nhìn từ trên xuống (dọc theo Y axis)
            RequestCameraPosition?.Invoke(
                new Point3D(0, 500, 0),      // Position
                new Vector3D(0, -1, 0),      // LookDirection
                new Vector3D(0, 0, -1)       // UpDirection
            );
        }

        // Event để MainWindow handle
        public event Action<Point3D, Vector3D, Vector3D> RequestCameraPosition;

        private Viewport3D FindViewport3D(object source)
        {
            if (source is Viewport3D viewport) return viewport;
            if (source is HelixViewport3D helixViewport) return helixViewport.Viewport;
            if (source is DependencyObject obj)
            {
                var parent = VisualTreeHelper.GetParent(obj);
                while (parent != null)
                {
                    if (parent is HelixViewport3D hvp) return hvp.Viewport;
                    if (parent is Viewport3D vp) return vp;
                    parent = VisualTreeHelper.GetParent(parent);
                }
            }
            return null;
        }

        private RayMeshGeometry3DHitTestResult GetHitTestResult(Viewport3D viewport, Point mousePos)
        {
            var hitParams = new PointHitTestParameters(mousePos);
            RayMeshGeometry3DHitTestResult closestHit = null;
            double closestDistance = double.MaxValue;
            VisualTreeHelper.HitTest(viewport, null,
                result =>
                {
                    if (result is RayMeshGeometry3DHitTestResult meshResult)
                    {
                        if (meshResult.DistanceToRayOrigin < closestDistance)
                        {
                            closestDistance = meshResult.DistanceToRayOrigin;
                            closestHit = meshResult;
                        }
                    }
                    return HitTestResultBehavior.Continue;
                },
                hitParams);
            return closestHit;
        }

        private int? FindEdgeIdFromHitResult(RayMeshGeometry3DHitTestResult hitResult)
        {
            if (hitResult?.ModelHit is GeometryModel3D hitModel)
            {
                foreach (var kvp in geometryModel.GetAllEdgeInfos())
                {
                    if (ReferenceEquals(kvp.Value.Model3D, hitModel)) return kvp.Key;
                }
            }
            return null;
        }
        private void ExecuteToggleCameraLock(object parameter)
        {
            IsCameraLocked = !IsCameraLocked;
            StatusText = IsCameraLocked ? "Camera locked - Rotation around Y axis only" : "Camera unlocked";
        }
        private void UpdateCameraConstraints()
        {
            // Sẽ implement trong XAML với Binding
        }


        private void ExecuteMouseDown(object parameter)
        {
            if (parameter is MouseButtonEventArgs args && args.LeftButton == MouseButtonState.Pressed)
            {
                var viewport = FindViewport3D(args.Source);
                if (viewport == null) return;
                var mousePos = args.GetPosition(viewport);
                var hitResult = GetHitTestResult(viewport, mousePos);
                if (hitResult != null)
                {
                    var edgeId = FindEdgeIdFromHitResult(hitResult);
                    if (edgeId.HasValue) HandleEdgeSelection(edgeId.Value);
                }
            }
        }

        private void HandleEdgeSelection(int edgeId)
        {
            var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
            if (edgeInfo == null) return;
            switch (CurrentSelectionMode)
            {
                case SelectionMode.Single: HandleSingleSelection(edgeId, edgeInfo); break;
                case SelectionMode.Multiple: HandleMultipleSelection(edgeId, edgeInfo); break;
                case SelectionMode.Chain: HandleChainSelection(edgeId, edgeInfo); break;
            }
            UpdateStatusText();
        }

        private void HandleSingleSelection(int edgeId, EdgeSelectionInfo edgeInfo)
        {
            foreach (var selected in SelectedToolpaths.ToList())
                geometryModel.SetEdgeSelected(selected.EdgeId, false);
            SelectedToolpaths.Clear();
            geometryModel.SetEdgeSelected(edgeId, true);
            SelectedToolpaths.Add(edgeInfo);
        }

        private void HandleMultipleSelection(int edgeId, EdgeSelectionInfo edgeInfo)
        {
            if (edgeInfo.IsSelected)
            {
                geometryModel.SetEdgeSelected(edgeId, false);
                var toRemove = SelectedToolpaths.FirstOrDefault(e => e.EdgeId == edgeId);
                if (toRemove != null) SelectedToolpaths.Remove(toRemove);
            }
            else
            {
                geometryModel.SetEdgeSelected(edgeId, true);
                SelectedToolpaths.Add(edgeInfo);
            }
        }

        private void HandleChainSelection(int startEdgeId, EdgeSelectionInfo startEdge)
        {
            // Clear previous selection
            foreach (var selected in SelectedToolpaths.ToList())
            {
                geometryModel.SetEdgeSelected(selected.EdgeId, false);
            }
            SelectedToolpaths.Clear();
            processedInChain.Clear(); // Clear for new chain operation

            try
            {
                var edgeGroups = geometryModel.GetEdgeGroups();
                System.Diagnostics.Debug.WriteLine($"Found {edgeGroups.Count} edge groups");

                foreach (var group in edgeGroups)
                {
                    if (group.Contains(startEdgeId))
                    {
                        System.Diagnostics.Debug.WriteLine($"Found group containing edge {startEdgeId}");

                        foreach (var id in group)
                        {
                            var info = geometryModel.GetEdgeInfo(id);
                            if (info != null)
                            {
                                geometryModel.SetEdgeSelected(id, true);
                                SelectedToolpaths.Add(info);
                            }
                        }
                        return;
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error in GetEdgeGroups: {ex.Message}");
            }

            List<int> edgesToSelect = new List<int>();
            const double tangentAngleTolerance = 175.0;
            

            // Prioritize selection based on C++ classification
            if (startEdge.EdgeInfo.Type == GeometryWrapper.ManagedEdgeInfo.EdgeType.Line)
            {
                var allLineLoops = FindAllClosedLineLoops(startEdgeId);
                List<List<int>> candidateSurfaceLoops = new List<List<int>>();

                if (allLineLoops.Any())
                {
                    foreach (var loop in allLineLoops)
                    {
                        bool allOnSurface = true;
                        foreach (var edgeIdInLoop in loop)
                        {
                            var edgeInLoopInfo = geometryModel.GetEdgeInfo(edgeIdInLoop);
                            if (edgeInLoopInfo?.Classification == null || 
                                edgeInLoopInfo.Classification.Location != Models.EdgeLocation.OnCylinderSurface)
                            {
                                allOnSurface = false;
                                break;
                            }
                        }
                        if (allOnSurface)
                        {
                            candidateSurfaceLoops.Add(loop);
                        }
                    }
                }

                if (candidateSurfaceLoops.Any())
                {
                    // Select the longest loop among those entirely on the surface
                    var bestLoop = candidateSurfaceLoops
                        .OrderByDescending(loop => CalculateLoopTotalLength(loop))
                        .FirstOrDefault();
                    if (bestLoop != null) edgesToSelect.AddRange(bestLoop);
                }
                else
                {

                    if (startEdge.Classification.Location == Models.EdgeLocation.OnCylinderSurface)
                    {
                        edgesToSelect.Add(startEdgeId);
                    }

                    else if (!edgesToSelect.Any()) // If nothing selected yet
                    {

                        var legacyLoops = FindAllClosedLineLoops(startEdgeId)
                                          .Where(loop => IsLoopApproximatelyOnCylinderSurface(loop, CylinderInfo, 1.0))
                                          .OrderByDescending(loop => CalculateLoopTotalLength(loop))
                                          .FirstOrDefault();
                        if (legacyLoops != null && legacyLoops.Any())
                        {
                            edgesToSelect.AddRange(legacyLoops);
                        }
                        else
                        {
                             edgesToSelect.Add(startEdgeId); // Select just the clicked one
                        }
                    }
                }
            }
            else // Curves (Circle, BSpline, Bezier, Other)
            {
                if (startEdge.Classification.Location == Models.EdgeLocation.OnCylinderSurface)
                {
                    var points = startEdge.Points;
                    if (points != null && points.Count > 1 && ArePointsEqual(points.First(), points.Last(), 0.01))
                    {
                        // It's a self-closed curve on the surface, select just itself.
                        edgesToSelect.Add(startEdgeId);
                    }
                    else
                    {
                        // It's an open curve on the surface, find a tangent chain of *other surface curves*.
                        var tangentChain = BuildChainRecursive(startEdgeId, id => 
                            FindContinuouslyTangentEdges(id, tangentAngleTolerance)
                            .Where(connectedId => {
                                var connectedEdgeInfo = geometryModel.GetEdgeInfo(connectedId);
                                return connectedEdgeInfo?.Classification?.Location == Models.EdgeLocation.OnCylinderSurface;
                            }).ToList() 
                        );

                        if (tangentChain != null && tangentChain.Any())
                        {
                            edgesToSelect.AddRange(tangentChain);
                        }
                        else // Should include at least startEdgeId if it's on surface
                        {
                            edgesToSelect.Add(startEdgeId);
                        }
                    }
                }
                else
                {
                    // Clicked curve is not on the surface, select only the clicked curve.
                    edgesToSelect.Add(startEdgeId);
                }
            }
            
            // Add selected edges to the toolpath list
            foreach (var edgeId_toSelect in edgesToSelect.Distinct()) // Ensure distinct before adding
            {
                var edgeInfoToSelect = geometryModel.GetEdgeInfo(edgeId_toSelect);
                if (edgeInfoToSelect != null)
                {
                    geometryModel.SetEdgeSelected(edgeId_toSelect, true);
                    SelectedToolpaths.Add(edgeInfoToSelect);
                }
            }
        }

        private HashSet<int> processedInChain;

        private List<int> BuildChainRecursive(int startEdgeId, Func<int, List<int>> getConnectedEdgesFunc)
        {
            var chain = new List<int>();
            var toProcess = new Queue<int>();
            processedInChain.Clear();
            toProcess.Enqueue(startEdgeId);
            processedInChain.Add(startEdgeId);
            while (toProcess.Count > 0)
            {
                var currentId = toProcess.Dequeue();
                chain.Add(currentId);
                var connectedEdges = getConnectedEdgesFunc(currentId);
                foreach (var connectedId in connectedEdges)
                {
                    if (!processedInChain.Contains(connectedId))
                    {
                        processedInChain.Add(connectedId);
                        toProcess.Enqueue(connectedId);
                    }
                }
            }
            return chain;
        }

        private List<int> FindConnectedEdgesBySharedPoints(int edgeId)
        {
            var connected = new List<int>();
            var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
            if (edgeInfo == null || edgeInfo.Points.Count < 2) return connected;
            var startPoint = edgeInfo.Points.First();
            var endPoint = edgeInfo.Points.Last();
            foreach (var kvp in geometryModel.GetAllEdgeInfos())
            {
                if (kvp.Key == edgeId) continue;
                var otherEdge = kvp.Value;
                if (otherEdge.Points.Count < 2) continue;
                var otherStart = otherEdge.Points.First();
                var otherEnd = otherEdge.Points.Last();
                const double tolerance = 0.01;
                if (ArePointsEqual(startPoint, otherStart, tolerance) ||
                    ArePointsEqual(startPoint, otherEnd, tolerance) ||
                    ArePointsEqual(endPoint, otherStart, tolerance) ||
                    ArePointsEqual(endPoint, otherEnd, tolerance))
                {
                    connected.Add(kvp.Key);
                }
            }
            return connected;
        }

        private bool ArePointsEqual(System.Windows.Media.Media3D.Point3D p1, System.Windows.Media.Media3D.Point3D p2, double tolerance)
        {
            return (p1 - p2).Length < tolerance;
        }

        private Vector3D? GetApproximateTangentAtStart(EdgeSelectionInfo edge)
        {
            if (edge?.Points == null || edge.Points.Count < 2) return null;
            Vector3D tangent = edge.Points[1] - edge.Points[0];
            if (tangent.LengthSquared < 1e-9) return null;
            tangent.Normalize();
            return tangent;
        }

        private Vector3D? GetApproximateTangentAtEnd(EdgeSelectionInfo edge)
        {
            if (edge?.Points == null || edge.Points.Count < 2) return null;
            Vector3D tangent = edge.Points[edge.Points.Count - 1] - edge.Points[edge.Points.Count - 2];
            if (tangent.LengthSquared < 1e-9) return null;
            tangent.Normalize();
            return tangent;
        }

        private List<int> FindContinuouslyTangentEdges(int currentEdgeId, double maxAngleDeviationDegrees = 15.0)
        {
            var tangentChain = new List<int>();
            var currentEdgeInfo = geometryModel.GetEdgeInfo(currentEdgeId);
            if (currentEdgeInfo == null || currentEdgeInfo.Points.Count < 2) return tangentChain;
            var currentStartPoint = currentEdgeInfo.Points.First();
            var currentEndPoint = currentEdgeInfo.Points.Last();
            var currentTangentAtStart = GetApproximateTangentAtStart(currentEdgeInfo);
            var currentTangentAtEnd = GetApproximateTangentAtEnd(currentEdgeInfo);
            const double pointEqTolerance = 0.01;
            foreach (var kvp in geometryModel.GetAllEdgeInfos())
            {
                if (kvp.Key == currentEdgeId) continue;
                var otherEdgeInfo = kvp.Value;
                if (otherEdgeInfo == null || otherEdgeInfo.Points.Count < 2) continue;
                var otherStartPoint = otherEdgeInfo.Points.First();
                var otherEndPoint = otherEdgeInfo.Points.Last();
                var otherTangentAtStart = GetApproximateTangentAtStart(otherEdgeInfo);
                var otherTangentAtEnd = GetApproximateTangentAtEnd(otherEdgeInfo);
                Vector3D? t1 = null, t2 = null;
                bool connected = false;
                if (ArePointsEqual(currentEndPoint, otherStartPoint, pointEqTolerance))
                {
                    connected = true; t1 = currentTangentAtEnd; t2 = otherTangentAtStart;
                }
                else if (ArePointsEqual(currentEndPoint, otherEndPoint, pointEqTolerance))
                {
                    connected = true; t1 = currentTangentAtEnd; if (otherTangentAtEnd.HasValue) t2 = -otherTangentAtEnd.Value;
                }
                else if (ArePointsEqual(currentStartPoint, otherStartPoint, pointEqTolerance))
                {
                    connected = true; if (currentTangentAtStart.HasValue) t1 = -currentTangentAtStart.Value; t2 = otherTangentAtStart;
                }
                else if (ArePointsEqual(currentStartPoint, otherEndPoint, pointEqTolerance))
                {
                    connected = true; if (currentTangentAtStart.HasValue) t1 = -currentTangentAtStart.Value; if (otherTangentAtEnd.HasValue) t2 = -otherTangentAtEnd.Value;
                }
                if (connected && t1.HasValue && t2.HasValue)
                {
                    double angleDeg = Vector3D.AngleBetween(t1.Value, t2.Value);
                    if (angleDeg < maxAngleDeviationDegrees) tangentChain.Add(kvp.Key);
                }
            }
            return tangentChain;
        }

        private List<int> FindConnectedLines(int lineId)
        {
            var connectedLines = new List<int>();
            var edgeInfo = geometryModel.GetEdgeInfo(lineId);
            if (edgeInfo == null || edgeInfo.EdgeInfo.Type != GeometryWrapper.ManagedEdgeInfo.EdgeType.Line || edgeInfo.Points.Count < 2) return connectedLines;
            var startPoint = edgeInfo.Points.First();
            var endPoint = edgeInfo.Points.Last();
            foreach (var kvp in geometryModel.GetAllEdgeInfos())
            {
                if (kvp.Key == lineId) continue;
                var otherEdge = kvp.Value;
                if (otherEdge.EdgeInfo.Type != GeometryWrapper.ManagedEdgeInfo.EdgeType.Line || otherEdge.Points.Count < 2) continue;
                var otherStart = otherEdge.Points.First();
                var otherEnd = otherEdge.Points.Last();
                const double tolerance = 0.01;
                if (ArePointsEqual(startPoint, otherStart, tolerance) || ArePointsEqual(startPoint, otherEnd, tolerance) || ArePointsEqual(endPoint, otherStart, tolerance) || ArePointsEqual(endPoint, otherEnd, tolerance))
                {
                    connectedLines.Add(kvp.Key);
                }
            }
            return connectedLines;
        }

        private void DFS_CollectAllLineLoops(int u, int startNode, List<int> currentPath, HashSet<int> visitedOnPath, Dictionary<int, List<int>> adjLines, List<List<int>> allFoundLoops, int maxDepth = 12)
        {
            if (currentPath.Count >= maxDepth) return;
            visitedOnPath.Add(u);
            currentPath.Add(u);
            if (adjLines.ContainsKey(u))
            {
                foreach (int v in adjLines[u])
                {
                    if (v == startNode && currentPath.Count > 2)
                    {
                        allFoundLoops.Add(new List<int>(currentPath));
                    }
                    else if (!visitedOnPath.Contains(v))
                    {
                        DFS_CollectAllLineLoops(v, startNode, currentPath, visitedOnPath, adjLines, allFoundLoops, maxDepth);
                    }
                }
            }
            currentPath.RemoveAt(currentPath.Count - 1);
            visitedOnPath.Remove(u);
        }

        private List<List<int>> FindAllClosedLineLoops(int startLineId)
        {
            var allLoops = new List<List<int>>();
            var allEdges = geometryModel.GetAllEdgeInfos();
            var lineAdjacency = new Dictionary<int, List<int>>();
            foreach (var edgeKvp in allEdges)
            {
                if (edgeKvp.Value.EdgeInfo.Type == GeometryWrapper.ManagedEdgeInfo.EdgeType.Line)
                {
                    var connectedLines = FindConnectedLines(edgeKvp.Key);
                    if (connectedLines.Any()) lineAdjacency[edgeKvp.Key] = connectedLines;
                }
            }
            if (!lineAdjacency.ContainsKey(startLineId)) return allLoops;
            DFS_CollectAllLineLoops(startLineId, startLineId, new List<int>(), new HashSet<int>(), lineAdjacency, allLoops);
            var uniqueLoops = new List<List<int>>();
            var seenLoopSignatures = new HashSet<string>();
            foreach (var loop in allLoops)
            {
                if (loop == null || !loop.Contains(startLineId) || loop.Count < 3) continue;
                var sortedLoopIds = loop.OrderBy(id => id).ToList();
                string signature = string.Join(",", sortedLoopIds);
                if (!seenLoopSignatures.Contains(signature))
                {
                    seenLoopSignatures.Add(signature);
                    uniqueLoops.Add(loop);
                }
            }
            return uniqueLoops;
        }

        private double CalculateLoopTotalLength(List<int> loopEdgeIds)
        {
            double totalLength = 0;
            if (loopEdgeIds == null || !loopEdgeIds.Any()) return 0;
            foreach (var edgeId in loopEdgeIds)
            {
                var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
                if (edgeInfo != null) totalLength += edgeInfo.EdgeInfo.Length;
            }
            return totalLength;
        }

        private bool IsPointOnCylinderSurface(System.Windows.Media.Media3D.Point3D point, CylinderData cylinder, double tolerance)
        {
            if (cylinder == null || !cylinder.IsValid) return false;
            Vector3D p0_p1 = point - cylinder.Center;
            Vector3D v = cylinder.Axis;
            if (v.LengthSquared < 1e-9) return false;
            double v_length = v.Length;
            Vector3D crossProduct = Vector3D.CrossProduct(p0_p1, v);
            double distanceToAxis = crossProduct.Length / v_length;
            return Math.Abs(distanceToAxis - cylinder.Radius) < tolerance;
        }

        private bool IsLoopApproximatelyOnCylinderSurface(List<int> loopEdgeIds, CylinderData cylinder, double surfaceTolerance, double pointThresholdRatio = 0.75)
        {
            if (cylinder == null || !cylinder.IsValid || loopEdgeIds == null || !loopEdgeIds.Any()) return false;
            int pointsOnSurface = 0;
            int totalPointsChecked = 0;
            foreach (var edgeId in loopEdgeIds)
            {
                var edgeSelectionInfo = geometryModel.GetEdgeInfo(edgeId);
                if (edgeSelectionInfo != null && edgeSelectionInfo.Points.Any())
                {
                    var pointsToTestOnEdge = new List<System.Windows.Media.Media3D.Point3D>();
                    if (edgeSelectionInfo.Points.Count > 0) pointsToTestOnEdge.Add(edgeSelectionInfo.Points.First());
                    if (edgeSelectionInfo.Points.Count > 1 && !ArePointsEqual(edgeSelectionInfo.Points.First(), edgeSelectionInfo.Points.Last(), 0.001))
                        pointsToTestOnEdge.Add(edgeSelectionInfo.Points.Last());
                    if (edgeSelectionInfo.Points.Count > 2)
                    {
                        var midPt = edgeSelectionInfo.Points[edgeSelectionInfo.Points.Count / 2];
                        if (!ArePointsEqual(edgeSelectionInfo.Points.First(), midPt, 0.001) && !ArePointsEqual(edgeSelectionInfo.Points.Last(), midPt, 0.001))
                            pointsToTestOnEdge.Add(midPt);
                    }
                    var distinctPointsToTest = pointsToTestOnEdge.Distinct().ToList();
                    foreach(var point in distinctPointsToTest)
                    {
                        totalPointsChecked++;
                        if (IsPointOnCylinderSurface(point, cylinder, surfaceTolerance)) pointsOnSurface++;
                    }
                }
            }
            if (totalPointsChecked == 0) return false;
            return (double)pointsOnSurface / totalPointsChecked >= pointThresholdRatio;
        }
        
        private void ExecuteClearSelection(object parameter)
        {
            foreach (var selected in SelectedToolpaths.ToList())
            {
                geometryModel.SetEdgeSelected(selected.EdgeId, false);
            }
            SelectedToolpaths.Clear();
            UpdateStatusText();
        }

        private void ExecuteDeleteSelected(object parameter)
        {
            if (parameter is EdgeSelectionInfo edgeInfo)
            {
                geometryModel.SetEdgeSelected(edgeInfo.EdgeId, false);
                SelectedToolpaths.Remove(edgeInfo);
                UpdateStatusText();
            }
        }

        private bool CanMoveToolpathUp(object parameter)
        {
            if (parameter is EdgeSelectionInfo edge)
            {
                var index = SelectedToolpaths.IndexOf(edge);
                return index > 0;
            }
            return false;
        }

        private void ExecuteMoveToolpathUp(object parameter)
        {
            if (parameter is EdgeSelectionInfo edge)
            {
                var index = SelectedToolpaths.IndexOf(edge);
                if (index > 0)
                {
                    SelectedToolpaths.Move(index, index - 1);
                }
            }
        }

        private bool CanMoveToolpathDown(object parameter)
        {
            if (parameter is EdgeSelectionInfo edge)
            {
                var index = SelectedToolpaths.IndexOf(edge);
                return index >= 0 && index < SelectedToolpaths.Count - 1;
            }
            return false;
        }

        private void ExecuteMoveToolpathDown(object parameter)
        {
            if (parameter is EdgeSelectionInfo edge)
            {
                var index = SelectedToolpaths.IndexOf(edge);
                if (index < SelectedToolpaths.Count - 1)
                {
                    SelectedToolpaths.Move(index, index + 1);
                }
            }
        }

        private void ExecuteSaveConfiguration(object parameter)
        {
            var saveDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "Toolpath Configuration (*.json)|*.json|All Files (*.*)|*.*",
                DefaultExt = ".json",
                FileName = "toolpath_config"
            };

            if (saveDialog.ShowDialog() == true)
            {
                try
                {
                    var config = new ToolpathConfiguration
                    {
                        Name = Path.GetFileNameWithoutExtension(saveDialog.FileName),
                        FilePath = saveDialog.FileName,
                        CylinderInfo = CylinderInfo
                    };
                    int order = 0;
                    foreach (var edge in SelectedToolpaths)
                    {
                        config.Edges.Add(new SavedEdgeInfo
                        {
                            EdgeId = edge.EdgeId,
                            EdgeType = edge.EdgeInfo.Type.ToString(),
                            Length = edge.EdgeInfo.Length,
                            Order = order++
                        });
                    }
                    var json = JsonConvert.SerializeObject(config, Formatting.Indented);
                    File.WriteAllText(saveDialog.FileName, json);
                    StatusText = $"Configuration saved to {Path.GetFileName(saveDialog.FileName)}";
                }
                catch (Exception ex)
                {
                    StatusText = $"Error saving configuration: {ex.Message}";
                }
            }
        }

        private void ExecuteLoadConfiguration(object parameter)
        {
            var openDialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "Toolpath Configuration (*.json)|*.json|All Files (*.*)|*.*",
                DefaultExt = ".json"
            };

            if (openDialog.ShowDialog() == true)
            {
                try
                {
                    var json = File.ReadAllText(openDialog.FileName);
                    var config = JsonConvert.DeserializeObject<ToolpathConfiguration>(json);
                    ExecuteClearSelection(null);
                    var sortedEdges = config.Edges.OrderBy(e => e.Order).ToList();
                    foreach (var savedEdge in sortedEdges)
                    {
                        var edgeInfo = geometryModel.GetEdgeInfo(savedEdge.EdgeId);
                        if (edgeInfo != null)
                        {
                            geometryModel.SetEdgeSelected(savedEdge.EdgeId, true);
                            SelectedToolpaths.Add(edgeInfo);
                        }
                    }
                    StatusText = $"Configuration loaded from {Path.GetFileName(openDialog.FileName)}. {SelectedToolpaths.Count} edges selected.";
                }
                catch (Exception ex)
                {
                    StatusText = $"Error loading configuration: {ex.Message}";
                }
            }
        }

        private void ExecuteAutoSuggestToolpaths(object parameter)
        {
            var suggestions = geometryModel.GetToolpathSuggestions();

            // Xóa selection cũ
            ExecuteClearSelection(null);

            // Tìm suggestion "all_surface_features" và chọn tất cả edges trong đó
            var allSurfaceFeatures = suggestions.FirstOrDefault(s => s.Type == "all_surface_features");
            if (allSurfaceFeatures != null)
            {
                foreach (int edgeId in allSurfaceFeatures.EdgeIds)
                {
                    var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
                    if (edgeInfo != null)
                    {
                        geometryModel.SetEdgeSelected(edgeId, true);
                        SelectedToolpaths.Add(edgeInfo);
                    }
                }
                StatusText = $"Auto-selected {allSurfaceFeatures.EdgeIds.Count} surface features.";

                // Chuyển sang Multiple Selection mode để có thể click bỏ từng edge
                CurrentSelectionMode = SelectionMode.Multiple;
            }
            else
            {
                // Fallback: hiển thị suggestions như cũ
                SuggestedToolpaths.Clear();
                foreach (var suggestion in suggestions)
                {
                    SuggestedToolpaths.Add(suggestion);
                }
                StatusText = $"Found {suggestions.Count} toolpath suggestions.";
            }
        }

        private void ExecuteAcceptSuggestion(object parameter)
        {
            if (parameter is ToolpathSuggestion suggestion)
            {
                foreach (int edgeId in suggestion.EdgeIds)
                {
                    var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
                    if (edgeInfo != null && !SelectedToolpaths.Any(t => t.EdgeId == edgeId))
                    {
                        geometryModel.SetEdgeSelected(edgeId, true);
                        SelectedToolpaths.Add(edgeInfo);
                    }
                }

                StatusText = $"Added {suggestion.Type} with {suggestion.EdgeIds.Count} edges to toolpath.";
            }
        }

        private void ExecuteRecognizeShapes(object parameter)
        {
            var shapes = ShapeRecognition.RecognizeShapes(
                geometryModel.GetAllEdgeInfos(),
                CylinderInfo);

            // Hiển thị kết quả
            var shapeGroups = shapes.GroupBy(s => s.Type);
            var message = "Recognized shapes:\n";

            foreach (var group in shapeGroups)
            {
                message += $"{group.Key}: {group.Count()}\n";
            }

            StatusText = message;

            // Tự động chọn shapes có confidence cao
            foreach (var shape in shapes.Where(s => s.Confidence > 0.8))
            {
                foreach (var edgeId in shape.EdgeIds)
                {
                    geometryModel.SetEdgeSelected(edgeId, true);
                    var edgeInfo = geometryModel.GetEdgeInfo(edgeId);
                    if (edgeInfo != null && !SelectedToolpaths.Contains(edgeInfo))
                    {
                        SelectedToolpaths.Add(edgeInfo);
                    }
                }
            }
        }

        private void ExecuteUnrollToolpaths(object parameter)
        {
            if (SelectedToolpaths.Count == 0)
            {
                StatusText = "No toolpaths selected to unroll";
                return;
            }

            try
            {
                StatusText = "Unrolling toolpaths...";
                System.Diagnostics.Debug.WriteLine($"Unrolling {SelectedToolpaths.Count} selected toolpaths");

                var unrolled = geometryModel.UnrollSelectedToolpaths();
                System.Diagnostics.Debug.WriteLine($"Unrolled result: {unrolled?.Count ?? 0} toolpaths");

                UnrolledToolpaths.Clear();
                foreach (var toolpath in unrolled)
                {
                    System.Diagnostics.Debug.WriteLine($"  Toolpath {toolpath.EdgeId}: {toolpath.Points?.Count ?? 0} points");
                    UnrolledToolpaths.Add(toolpath);
                }

                StatusText = $"Unrolled {unrolled.Count()} toolpaths";

                // Mở 2D View window
                var view2D = new Views.Unrolled2DView(unrolled, CylinderInfo);
                view2D.Show();
            }
            catch (Exception ex)
            {
                StatusText = $"Error unrolling: {ex.Message}";
                System.Diagnostics.Debug.WriteLine($"Unroll error: {ex}");
            }
        }


        private void UpdateStatusText()
        {
            var count = SelectedToolpaths.Count;
            if (count == 0)
                StatusText = "No edges selected.";
            else if (count == 1)
                StatusText = $"1 edge selected. Type: {SelectedToolpaths[0].EdgeInfo.Type}";
            else
                StatusText = $"{count} edges selected.";
        }

        private void ExecuteApplyFilter(object parameter)
        {
            StatusText = "Filter applied (not implemented yet)";
        }

        private void ExecuteClearFilter(object parameter)
        {
            StatusText = "Filter cleared (not implemented yet)";
        }
    }
}
