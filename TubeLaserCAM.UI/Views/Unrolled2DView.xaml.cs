using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using System.Windows.Threading;
using TubeLaserCAM.UI.Models;
using System.IO;


namespace TubeLaserCAM.UI.Views
{
    public partial class Unrolled2DView : Window
    {
        private List<UnrolledToolpath> toolpaths;
        private double zoomLevel = 1.0;
        private Point? lastMousePosition;
        private bool isPanning = false;
        private double scale = 1.0; 
        private GCodeParser.ParseResult parsedGCode;
        private int currentAnimationIndex = 0;
        private bool isAnimating = false;
        private double cylinderRadius = 50; 
        private double cylinderLength = 200; 
        private DispatcherTimer animationTimer;
        private bool isInitialized = false;
        private EnhancedAnimationController enhancedAnimationController;
        private bool useEnhancedAnimation = true; // Flag để bật/tắt enhanced mode
        private double yOffsetTransform = 0;
        private bool hasGCodeTransformation = false;


        // Public properties để EnhancedAnimationController có thể access
        public double YOffsetTransform
        {
            get { return yOffsetTransform; }
        }

        public bool HasGCodeTransformation
        {
            get { return hasGCodeTransformation; }
        }
        // THÊM: Public properties để EnhancedAnimationController có thể access



        public Unrolled2DView(List<UnrolledToolpath> unrolledToolpaths, CylinderData cylinderInfo)
        {
            InitializeComponent();

            try
            {
                // Validate input
                if (unrolledToolpaths == null)
                {
                    System.Diagnostics.Debug.WriteLine("Warning: Null toolpaths provided");
                    unrolledToolpaths = new List<UnrolledToolpath>();
                }

                if (cylinderInfo == null)
                {
                    System.Diagnostics.Debug.WriteLine("Warning: Null cylinder info, using defaults");
                    cylinderInfo = new CylinderData
                    {
                        Radius = 50,
                        Length = 200,
                        IsValid = true
                    };
                }

                // Store data with validation
                this.toolpaths = unrolledToolpaths;
                this.cylinderRadius = cylinderInfo.Radius > 0 ? cylinderInfo.Radius : 50;
                this.cylinderLength = cylinderInfo.Length > 0 ? cylinderInfo.Length : 200;

                // Debug output
                System.Diagnostics.Debug.WriteLine($"2D View initialized with {toolpaths.Count} toolpaths");
                System.Diagnostics.Debug.WriteLine($"Cylinder: R={cylinderRadius}, L={cylinderLength}");

                // Calculate scale với bounds checking
                double circumference = 2 * Math.PI * cylinderRadius;

                // Ensure minimum canvas size
                double effectiveCanvasWidth = Math.Max(drawingCanvas.Width, 800);
                double effectiveCanvasHeight = Math.Max(drawingCanvas.Height, 600);

                scale = Math.Min(
                    effectiveCanvasWidth / circumference,
                    effectiveCanvasHeight / cylinderLength
                ) * 0.8; // 80% để có margin

                // Ensure minimum scale
                scale = Math.Max(scale, 0.1);
                enhancedAnimationController = new EnhancedAnimationController(
                    drawingCanvas,
                    cylinderRadius,
                    cylinderLength,
                    scale
                 );
                enhancedAnimationController.ParentView = this;

                System.Diagnostics.Debug.WriteLine($"Scale calculated: {scale}, Circumference: {circumference}");

                // Initialize UI elements
                txtSpeed.Text = "1.0x";
                txtZoom.Text = "Zoom: 100%";

                // Draw initial view với error handling
                try
                {
                    DrawUnrolledView(cylinderInfo);
                }
                catch (Exception drawEx)
                {
                    System.Diagnostics.Debug.WriteLine($"Error in initial draw: {drawEx.Message}");
                    MessageBox.Show("Error drawing initial view. Some features may not display correctly.",
                                   "Warning", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Critical error in constructor: {ex}");
                MessageBox.Show($"Error initializing 2D view: {ex.Message}",
                               "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void DrawUnrolledView(CylinderData cylinderInfo)
        {
            drawingCanvas.Children.Clear();

            double circumference = 2 * Math.PI * cylinderInfo.Radius;
            double length = cylinderInfo.Length;

            // Draw cylinder boundary
            var boundary = new Rectangle
            {
                Width = circumference * scale,
                Height = length * scale,
                Stroke = Brushes.Black,
                StrokeThickness = 2,
                Fill = Brushes.Transparent
            };
            Canvas.SetLeft(boundary, 50);
            Canvas.SetTop(boundary, 50);
            drawingCanvas.Children.Add(boundary);

            // Draw grid if enabled
            if (chkShowGrid.IsChecked == true)
            {
                DrawGrid(50, 50, circumference * scale, length * scale);
            }

            // Draw toolpaths
            foreach (var toolpath in toolpaths)
            {
                DrawToolpath(toolpath, cylinderInfo);
            }

            // Draw dimensions if enabled
            if (chkShowDimensions.IsChecked == true)
            {
                DrawDimensions(50, 50, circumference * scale, length * scale,
                              circumference, length);
            }

            UpdateStatus($"Displayed {toolpaths.Count} toolpaths");
        }

        private void CalculateGCodeTransformation()
        {
            if (parsedGCode == null || parsedGCode.Moves.Count == 0 ||
                toolpaths == null || toolpaths.Count == 0)
            {
                hasGCodeTransformation = false;
                yOffsetTransform = 0;
                System.Diagnostics.Debug.WriteLine("❌ Cannot calculate transformation - missing data");
                return;
            }

            System.Diagnostics.Debug.WriteLine("\n🔧 === CALCULATING G-CODE TRANSFORMATION ===");

            // Tìm Y bounds từ G-code (bỏ qua pierce points)
            var validMoves = parsedGCode.Moves.Where(m => m.Type != GCodeParser.GCodeMove.MoveType.Pierce).ToList();
            if (!validMoves.Any())
            {
                hasGCodeTransformation = false;
                return;
            }

            double gCodeYMin = validMoves.Min(m => m.Y);
            double gCodeYMax = validMoves.Max(m => m.Y);

            // Tìm Y bounds từ Toolpaths
            double toolpathYMin = toolpaths.Min(tp => tp.MinY);
            double toolpathYMax = toolpaths.Max(tp => tp.MaxY);

            System.Diagnostics.Debug.WriteLine($"G-Code Y range: [{gCodeYMin:F3}, {gCodeYMax:F3}]");
            System.Diagnostics.Debug.WriteLine($"Toolpath Y range: [{toolpathYMin:F3}, {toolpathYMax:F3}]");

            // Tính offset dựa trên Y max (top của cylinder)
            // Logic: Top của cylinder trong cả 2 hệ phải khớp nhau
            yOffsetTransform = toolpathYMax - gCodeYMax;

            System.Diagnostics.Debug.WriteLine($"\nTransform calculation:");
            System.Diagnostics.Debug.WriteLine($"  Toolpath Y max (top): {toolpathYMax:F3}");
            System.Diagnostics.Debug.WriteLine($"  G-Code Y max (top): {gCodeYMax:F3}");
            System.Diagnostics.Debug.WriteLine($"  Required offset: {yOffsetTransform:F3}");

            hasGCodeTransformation = true;

            // Verify
            System.Diagnostics.Debug.WriteLine($"\nVerification:");
            System.Diagnostics.Debug.WriteLine($"  G-Code Y={gCodeYMax:F3} + offset → {gCodeYMax + yOffsetTransform:F3} (should ≈ {toolpathYMax:F3})");
            System.Diagnostics.Debug.WriteLine($"  G-Code Y={gCodeYMin:F3} + offset → {gCodeYMin + yOffsetTransform:F3} (should ≈ {toolpathYMin:F3})");

            System.Diagnostics.Debug.WriteLine("🔧 === END TRANSFORMATION ===\n");
        }

        // Helper method để transform Y coordinate
        private double TransformGCodeY(double gCodeY)
        {
            return hasGCodeTransformation ? gCodeY + yOffsetTransform : gCodeY;
        }

        private void DrawToolpath(UnrolledToolpath toolpath, CylinderData cylinderInfo)
        {
            try
            {
                // Validate input
                if (toolpath == null)
                {
                    System.Diagnostics.Debug.WriteLine("Warning: Null toolpath provided");
                    return;
                }

                if (toolpath.Points == null || toolpath.Points.Count == 0)
                {
                    System.Diagnostics.Debug.WriteLine($"Warning: Toolpath {toolpath.EdgeId} has no points");
                    return;
                }

                // ========== THÊM DEBUG CHI TIẾT ==========
                System.Diagnostics.Debug.WriteLine($"\n🖌️ === DRAWTOOLPATH DEBUG START - EdgeId: {toolpath.EdgeId} ===");
                System.Diagnostics.Debug.WriteLine($"📊 Toolpath Info:");
                System.Diagnostics.Debug.WriteLine($"   - EdgeId: {toolpath.EdgeId}");
                System.Diagnostics.Debug.WriteLine($"   - Type: {toolpath.EdgeInfo?.Type ?? "Unknown"}");
                System.Diagnostics.Debug.WriteLine($"   - Points count: {toolpath.Points.Count}");
                System.Diagnostics.Debug.WriteLine($"   - Y range: [{toolpath.MinY:F3}, {toolpath.MaxY:F3}]");
                System.Diagnostics.Debug.WriteLine($"   - Total rotation: {toolpath.TotalRotation:F1}°");

                if (toolpath.Points.Count < 2)
                {
                    System.Diagnostics.Debug.WriteLine($"Warning: Toolpath {toolpath.EdgeId} has only {toolpath.Points.Count} point(s)");
                    if (toolpath.Points.Count == 1)
                    {
                        DrawSinglePoint(toolpath.Points[0], toolpath);
                    }
                    return;
                }

                // Create polyline
                var polyline = new Polyline
                {
                    StrokeLineJoin = PenLineJoin.Round,
                    StrokeStartLineCap = PenLineCap.Round,
                    StrokeEndLineCap = PenLineCap.Round
                };

                // Set color
                try
                {
                    polyline.Stroke = GetColorForEdgeType(toolpath.EdgeInfo?.Type ?? "Unknown");
                }
                catch
                {
                    polyline.Stroke = Brushes.Black;
                }
                polyline.StrokeThickness = 2;

                // Validate cylinder info
                double validRadius = (cylinderInfo?.Radius ?? cylinderRadius) > 0 ?
                                    (cylinderInfo?.Radius ?? cylinderRadius) : 50;
                double validLength = (cylinderInfo?.Length ?? cylinderLength) > 0 ?
                                    (cylinderInfo?.Length ?? cylinderLength) : 200;
                double centerY = validLength / 2;

                // ========== DEBUG CYLINDER & COORDINATE SYSTEM ==========
                System.Diagnostics.Debug.WriteLine($"\n📐 Cylinder & Coordinate Info:");
                System.Diagnostics.Debug.WriteLine($"   - cylinderRadius (class): {cylinderRadius:F1}mm");
                System.Diagnostics.Debug.WriteLine($"   - cylinderLength (class): {cylinderLength:F1}mm");
                System.Diagnostics.Debug.WriteLine($"   - validRadius (used): {validRadius:F1}mm");
                System.Diagnostics.Debug.WriteLine($"   - validLength (used): {validLength:F1}mm");
                System.Diagnostics.Debug.WriteLine($"   - centerY: {centerY:F1}mm");
                System.Diagnostics.Debug.WriteLine($"   - scale: {scale:F3}");

                // ========== DEBUG FIRST AND LAST POINTS ==========
                if (toolpath.Points.Count > 0)
                {
                    var firstPt = toolpath.Points[0];
                    var lastPt = toolpath.Points[toolpath.Points.Count - 1];

                    System.Diagnostics.Debug.WriteLine($"\n🔹 First Point:");
                    System.Diagnostics.Debug.WriteLine($"   - Y: {firstPt.Y:F3}mm, C: {firstPt.C:F3}°");

                    System.Diagnostics.Debug.WriteLine($"\n🔸 Last Point:");
                    System.Diagnostics.Debug.WriteLine($"   - Y: {lastPt.Y:F3}mm, C: {lastPt.C:F3}°");
                }

                // Process points
                int skippedPoints = 0;
                Point? lastValidPoint = null;
                int validPointCount = 0;

                // ========== DEBUG TRANSFORMATION FOR KEY POINTS ==========
                System.Diagnostics.Debug.WriteLine($"\n🔄 Point Transformation Debug:");

                for (int i = 0; i < toolpath.Points.Count; i++)
                {
                    var point = toolpath.Points[i];

                    try
                    {
                        // Validate point values
                        if (double.IsNaN(point.Y) || double.IsNaN(point.C) ||
                            double.IsInfinity(point.Y) || double.IsInfinity(point.C))
                        {
                            System.Diagnostics.Debug.WriteLine($"   Point[{i}] INVALID: Y={point.Y}, C={point.C}");
                            skippedPoints++;
                            continue;
                        }

                        // Clamp C value to valid range
                        double normalizedC = point.C;
                        while (normalizedC < 0) normalizedC += 360;
                        while (normalizedC > 360) normalizedC -= 360;

                        // Convert to canvas coordinates
                        double x = 50 + (normalizedC / 360.0) * 2 * Math.PI * validRadius * scale;
                        double y = 50 + (centerY + point.Y) * scale;

                        // ========== DEBUG KEY POINTS TRANSFORMATION ==========
                        if (i == 0 || i == toolpath.Points.Count - 1 || i % 50 == 0)
                        {
                            System.Diagnostics.Debug.WriteLine($"\n   Point[{i}] Transformation:");
                            System.Diagnostics.Debug.WriteLine($"      Input: Y={point.Y:F3}, C={point.C:F3}");
                            System.Diagnostics.Debug.WriteLine($"      NormalizedC: {normalizedC:F3}°");
                            System.Diagnostics.Debug.WriteLine($"      X formula: 50 + ({normalizedC:F3}/360) * 2π * {validRadius:F1} * {scale:F3}");
                            System.Diagnostics.Debug.WriteLine($"      Y formula: 50 + ({centerY:F1} + {point.Y:F3}) * {scale:F3}");
                            System.Diagnostics.Debug.WriteLine($"      Canvas result: X={x:F1}, Y={y:F1}");
                        }

                        // Validate canvas coordinates
                        if (double.IsNaN(x) || double.IsNaN(y) ||
                            double.IsInfinity(x) || double.IsInfinity(y))
                        {
                            System.Diagnostics.Debug.WriteLine($"   Point[{i}] Canvas coords INVALID: x={x}, y={y}");
                            skippedPoints++;
                            continue;
                        }

                        // Check for reasonable bounds
                        if (x < -10000 || x > 10000 || y < -10000 || y > 10000)
                        {
                            System.Diagnostics.Debug.WriteLine($"   Point[{i}] OUT OF BOUNDS: x={x}, y={y}");
                            skippedPoints++;
                            continue;
                        }

                        var canvasPoint = new Point(x, y);

                        // Check for duplicate points
                        if (lastValidPoint.HasValue &&
                            Math.Abs(lastValidPoint.Value.X - canvasPoint.X) < 0.01 &&
                            Math.Abs(lastValidPoint.Value.Y - canvasPoint.Y) < 0.01)
                        {
                            continue;
                        }

                        polyline.Points.Add(canvasPoint);
                        lastValidPoint = canvasPoint;
                        validPointCount++;
                    }
                    catch (Exception ptEx)
                    {
                        System.Diagnostics.Debug.WriteLine($"   Point[{i}] ERROR: {ptEx.Message}");
                        skippedPoints++;
                    }
                }

                // ========== DEBUG SUMMARY ==========
                System.Diagnostics.Debug.WriteLine($"\n📈 Processing Summary:");
                System.Diagnostics.Debug.WriteLine($"   - Total points: {toolpath.Points.Count}");
                System.Diagnostics.Debug.WriteLine($"   - Valid points: {validPointCount}");
                System.Diagnostics.Debug.WriteLine($"   - Skipped points: {skippedPoints}");
                System.Diagnostics.Debug.WriteLine($"   - Polyline points: {polyline.Points.Count}");

                // Only add polyline if it has valid points
                if (polyline.Points.Count > 0)
                {
                    // ========== DEBUG CANVAS BOUNDS ==========
                    double minX = polyline.Points.Min(p => p.X);
                    double maxX = polyline.Points.Max(p => p.X);
                    double minY = polyline.Points.Min(p => p.Y);
                    double maxY = polyline.Points.Max(p => p.Y);

                    System.Diagnostics.Debug.WriteLine($"\n📍 Canvas Bounds:");
                    System.Diagnostics.Debug.WriteLine($"   - X range: [{minX:F1}, {maxX:F1}]");
                    System.Diagnostics.Debug.WriteLine($"   - Y range: [{minY:F1}, {maxY:F1}]");
                    System.Diagnostics.Debug.WriteLine($"   - Width: {maxX - minX:F1}");
                    System.Diagnostics.Debug.WriteLine($"   - Height: {maxY - minY:F1}");

                    drawingCanvas.Children.Add(polyline);

                    // Add tooltip
                    try
                    {
                        var tooltip = new ToolTip
                        {
                            Content = $"Edge #{toolpath.EdgeId}\n" +
                                     $"Type: {toolpath.EdgeInfo?.Type ?? "Unknown"}\n" +
                                     $"Length: {toolpath.EdgeInfo?.Length ?? 0:F2}mm\n" +
                                     $"Points: {polyline.Points.Count} (Total: {toolpath.Points.Count})\n" +
                                     $"Y: [{toolpath.MinY:F2}, {toolpath.MaxY:F2}]\n" +
                                     $"Rotation: {toolpath.TotalRotation:F1}°"
                        };
                        polyline.ToolTip = tooltip;
                    }
                    catch (Exception ttEx)
                    {
                        System.Diagnostics.Debug.WriteLine($"Error creating tooltip: {ttEx.Message}");
                    }

                    System.Diagnostics.Debug.WriteLine($"\n✅ Toolpath {toolpath.EdgeId} drawn successfully");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine($"\n❌ No valid points to draw for toolpath {toolpath.EdgeId}");
                }

                System.Diagnostics.Debug.WriteLine($"🖌️ === DRAWTOOLPATH DEBUG END - EdgeId: {toolpath.EdgeId} ===\n");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"\n❌ CRITICAL ERROR in DrawToolpath {toolpath?.EdgeId}:");
                System.Diagnostics.Debug.WriteLine($"   Message: {ex.Message}");
                System.Diagnostics.Debug.WriteLine($"   Stack: {ex.StackTrace}");
            }
        }

        /// <summary>
        /// Debug method để so sánh data với G-Code
        /// </summary>
        public void DebugCompareWithGCode(string generatedGCode)
        {
            System.Diagnostics.Debug.WriteLine("\n=== COMPARE 2D VIEW vs G-CODE ===");

            // Parse G-Code
            var parser = new GCodeParser();
            var parsed = parser.ParseGCode(generatedGCode);

            System.Diagnostics.Debug.WriteLine($"2D View: {toolpaths.Count} toolpaths");
            System.Diagnostics.Debug.WriteLine($"G-Code: {parsed.Moves.Count} moves, {parsed.PierceCount} pierces");

            // So sánh từng toolpath với G-Code moves
            foreach (var toolpath in toolpaths.Take(3)) // First 3 toolpaths
            {
                System.Diagnostics.Debug.WriteLine($"\nToolpath #{toolpath.EdgeId}:");
                System.Diagnostics.Debug.WriteLine($"  2D View first point: Y={toolpath.Points[0].Y:F3}, C={toolpath.Points[0].C:F3}");
                System.Diagnostics.Debug.WriteLine($"  2D View last point: Y={toolpath.Points.Last().Y:F3}, C={toolpath.Points.Last().C:F3}");

                // Tìm corresponding moves trong G-Code
                var correspondingMoves = FindCorrespondingMoves(parsed, toolpath);
                if (correspondingMoves.Any())
                {
                    System.Diagnostics.Debug.WriteLine($"  G-Code moves found: {correspondingMoves.Count}");
                    var firstMove = correspondingMoves.First();
                    var lastMove = correspondingMoves.Last();
                    System.Diagnostics.Debug.WriteLine($"  G-Code first: Y={firstMove.Y:F3}, C={firstMove.C:F3}");
                    System.Diagnostics.Debug.WriteLine($"  G-Code last: Y={lastMove.Y:F3}, C={lastMove.C:F3}");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine("  WARNING: No corresponding G-Code moves found!");
                }
            }

            // Check coordinate system
            System.Diagnostics.Debug.WriteLine($"\n2D View settings:");
            System.Diagnostics.Debug.WriteLine($"  Cylinder radius: {cylinderRadius}");
            System.Diagnostics.Debug.WriteLine($"  Cylinder length: {cylinderLength}");
            System.Diagnostics.Debug.WriteLine($"  Scale: {scale}");
            System.Diagnostics.Debug.WriteLine($"  Canvas offset: X=50, Y=50");

            System.Diagnostics.Debug.WriteLine("=== END COMPARE ===\n");
        }

        private List<GCodeParser.GCodeMove> FindCorrespondingMoves(
    GCodeParser.ParseResult parsed,
    UnrolledToolpath toolpath)
        {
            var moves = new List<GCodeParser.GCodeMove>();
            var tolerance = 0.1; // 0.1mm tolerance

            // Find moves that match toolpath start point
            var startY = toolpath.Points[0].Y;
            var startC = toolpath.Points[0].C;

            bool foundStart = false;
            foreach (var move in parsed.Moves)
            {
                if (!foundStart &&
                    Math.Abs(move.Y - startY) < tolerance &&
                    Math.Abs(move.C - startC) < tolerance)
                {
                    foundStart = true;
                }

                if (foundStart)
                {
                    moves.Add(move);

                    // Check if we reached the end
                    var endY = toolpath.Points.Last().Y;
                    var endC = toolpath.Points.Last().C;
                    if (Math.Abs(move.Y - endY) < tolerance &&
                        Math.Abs(move.C - endC) < tolerance)
                    {
                        break;
                    }
                }
            }

            return moves;
        }

        private void DrawSinglePoint(UnrolledPoint point, UnrolledToolpath toolpath)
        {
            try
            {
                double x = 50 + (point.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
                double y = 50 + (cylinderLength / 2 + point.Y) * scale;

                var marker = new Ellipse
                {
                    Width = 6,
                    Height = 6,
                    Fill = GetColorForEdgeType(toolpath.EdgeInfo?.Type ?? "Unknown"),
                    Stroke = Brushes.Black,
                    StrokeThickness = 1
                };

                Canvas.SetLeft(marker, x - 3);
                Canvas.SetTop(marker, y - 3);
                drawingCanvas.Children.Add(marker);

                marker.ToolTip = $"Single Point - Edge #{toolpath.EdgeId}";
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error drawing single point: {ex.Message}");
            }
        }

        private Brush GetColorForEdgeType(string type)
        {
            switch (type)
            {
                case "Line": return Brushes.Blue;
                case "Circle": return Brushes.Red;
                case "BSpline": return Brushes.Green;
                case "Bezier": return Brushes.Purple;
                default: return Brushes.Black;
            }
        }

        private void DrawGrid(double x, double y, double width, double height)
        {
            // Vertical lines (C axis)
            for (int i = 0; i <= 360; i += 30)
            {
                double xPos = x + (i / 360.0) * width;
                var line = new Line
                {
                    X1 = xPos,
                    Y1 = y,
                    X2 = xPos,
                    Y2 = y + height,
                    Stroke = Brushes.LightGray,
                    StrokeThickness = 1
                };

                if (i % 90 == 0)
                {
                    line.Stroke = Brushes.Gray;
                    line.StrokeDashArray = new DoubleCollection { 5, 5 };
                }

                drawingCanvas.Children.Add(line);

                // Add angle labels
                var label = new TextBlock
                {
                    Text = $"{i}°",
                    FontSize = 10,
                    Foreground = Brushes.Gray
                };
                Canvas.SetLeft(label, xPos - 10);
                Canvas.SetTop(label, y - 20);
                drawingCanvas.Children.Add(label);
            }

            // Horizontal lines (Y axis)
            int step = 50; // 50mm
            for (double yVal = -height / 2; yVal <= height / 2; yVal += step * scale)
            {
                double yPos = y + height / 2 + yVal;
                var line = new Line
                {
                    X1 = x,
                    Y1 = yPos,
                    X2 = x + width,
                    Y2 = yPos,
                    Stroke = Brushes.LightGray,
                    StrokeThickness = 1
                };

                if (Math.Abs(yVal) < 0.1)
                {
                    line.Stroke = Brushes.Gray;
                    line.StrokeThickness = 2;
                }

                drawingCanvas.Children.Add(line);
            }
        }

        private void DrawDimensions(double x, double y, double width, double height,
                                   double realWidth, double realHeight)
        {
            // Width dimension
            var widthText = new TextBlock
            {
                Text = $"C: 0° - 360° ({realWidth:F1}mm)",
                FontSize = 12,
                FontWeight = FontWeights.Bold
            };
            Canvas.SetLeft(widthText, x + width / 2 - 50);
            Canvas.SetTop(widthText, y + height + 10);
            drawingCanvas.Children.Add(widthText);

            // Height dimension
            var heightText = new TextBlock
            {
                Text = $"Y: {realHeight:F1}mm",
                FontSize = 12,
                FontWeight = FontWeights.Bold,
                RenderTransform = new RotateTransform(-90)
            };
            Canvas.SetLeft(heightText, x - 30);
            Canvas.SetTop(heightText, y + height / 2);
            drawingCanvas.Children.Add(heightText);
        }

        // Event handlers
        private void ZoomIn_Click(object sender, RoutedEventArgs e)
        {
            zoomLevel *= 1.2;
            ApplyZoom();
        }

        private void ZoomOut_Click(object sender, RoutedEventArgs e)
        {
            zoomLevel /= 1.2;
            ApplyZoom();
        }

        private void FitToWindow_Click(object sender, RoutedEventArgs e)
        {
            zoomLevel = 1.0;
            ApplyZoom();
            scrollViewer.ScrollToHome();
        }

        private void ApplyZoom()
        {
            scaleTransform.ScaleX = zoomLevel;
            scaleTransform.ScaleY = zoomLevel;
            txtZoom.Text = $"Zoom: {zoomLevel:P0}";
        }

        private void Canvas_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (Keyboard.IsKeyDown(Key.LeftCtrl))
            {
                zoomLevel *= e.Delta > 0 ? 1.1 : 0.9;
                ApplyZoom();
                e.Handled = true;
            }
        }

        private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            lastMousePosition = e.GetPosition(scrollViewer);
            isPanning = true;
            drawingCanvas.CaptureMouse();
        }

        private void Canvas_MouseMove(object sender, MouseEventArgs e)
        {
            var pos = e.GetPosition(drawingCanvas);
            txtCoordinates.Text = $"Y: {(pos.Y - 50) / scale:F2}, " +
                                 $"C: {(pos.X - 50) / scale * 360 / (2 * Math.PI * 50):F1}°";

            if (isPanning && lastMousePosition.HasValue)
            {
                var currentPos = e.GetPosition(scrollViewer);
                var deltaX = currentPos.X - lastMousePosition.Value.X;
                var deltaY = currentPos.Y - lastMousePosition.Value.Y;

                scrollViewer.ScrollToHorizontalOffset(scrollViewer.HorizontalOffset - deltaX);
                scrollViewer.ScrollToVerticalOffset(scrollViewer.VerticalOffset - deltaY);

                lastMousePosition = currentPos;
            }
        }

        private void Canvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            isPanning = false;
            drawingCanvas.ReleaseMouseCapture();
        }

        private void ShowGrid_Changed(object sender, RoutedEventArgs e)
        {
            // Redraw
            if (toolpaths != null)
            {
                var cylinderInfo = new CylinderData { Radius = 50, Length = 200 }; // Get from somewhere
                DrawUnrolledView(cylinderInfo);
            }
        }

        private void ShowDimensions_Changed(object sender, RoutedEventArgs e)
        {
            // Redraw
            ShowGrid_Changed(sender, e);
        }

        private void UpdateStatus(string message)
        {
            txtStatus.Text = message;
        }
        public void LoadGCodeForVisualization(string gcode)
        {
            var parser = new GCodeParser();
            parsedGCode = parser.ParseGCode(gcode);

            // Calculate coordinate transformation
            CalculateGCodeTransformation();

            UpdateStatus($"Loaded G-Code: {parsedGCode.Moves.Count} moves, " +
                        $"{parsedGCode.PierceCount} pierces");

            DrawGCodeVisualization();
        }

        private void AnalyzeGCodeCoordinateSystem()
        {
            if (parsedGCode == null || parsedGCode.Moves.Count == 0) return;

            System.Diagnostics.Debug.WriteLine("\n🔍 === G-CODE COORDINATE ANALYSIS ===");

            // Tìm Y bounds
            double yMin = double.MaxValue;
            double yMax = double.MinValue;

            foreach (var move in parsedGCode.Moves)
            {
                if (move.Y < yMin) yMin = move.Y;
                if (move.Y > yMax) yMax = move.Y;
            }

            System.Diagnostics.Debug.WriteLine($"G-Code Y range: [{yMin:F3}, {yMax:F3}]");
            System.Diagnostics.Debug.WriteLine($"G-Code Y span: {yMax - yMin:F3}mm");

            // Phân tích xem Y=0 ở đâu
            if (Math.Abs(yMin) < 0.1 && yMax > 0)
            {
                System.Diagnostics.Debug.WriteLine("Y=0 appears to be at TOP (yMin ≈ 0, yMax > 0)");
            }
            else if (yMin < 0 && yMax > 0)
            {
                System.Diagnostics.Debug.WriteLine("Y=0 appears to be at CENTER (yMin < 0, yMax > 0)");
            }
            else if (yMax < 0.1 && yMin < 0)
            {
                System.Diagnostics.Debug.WriteLine("Y=0 appears to be at BOTTOM (yMax ≈ 0, yMin < 0)");
            }

            // So sánh với toolpath bounds
            if (toolpaths != null && toolpaths.Count > 0)
            {
                double tpYMin = double.MaxValue;
                double tpYMax = double.MinValue;

                foreach (var tp in toolpaths)
                {
                    if (tp.MinY < tpYMin) tpYMin = tp.MinY;
                    if (tp.MaxY > tpYMax) tpYMax = tp.MaxY;
                }

                System.Diagnostics.Debug.WriteLine($"\nToolpath Y range: [{tpYMin:F3}, {tpYMax:F3}]");
                System.Diagnostics.Debug.WriteLine($"Toolpath Y span: {tpYMax - tpYMin:F3}mm");

                // Kiểm tra offset
                double yOffset = tpYMin - yMin;
                System.Diagnostics.Debug.WriteLine($"Potential Y offset: {yOffset:F3}mm");
            }

            System.Diagnostics.Debug.WriteLine("=== END ANALYSIS ===\n");
        }

        private void DrawGCodeVisualization()
        {
            // Clear previous G-Code layer
            ClearGCodeLayer();

            // Draw move sequence
            DrawMoveSequence();

            // Draw start/end markers
            DrawStartEndMarkers();

            // Draw pierce points
            DrawPiercePoints();

            // Update statistics
            DisplayStatistics();
        }
        private void DrawMoveSequence()
        {
            if (parsedGCode == null || parsedGCode.Moves.Count < 2) return;

            for (int i = 1; i < parsedGCode.Moves.Count; i++)
            {
                var from = parsedGCode.Moves[i - 1];
                var to = parsedGCode.Moves[i];

                // Skip non-movement commands
                if (to.Type == GCodeParser.GCodeMove.MoveType.Pierce ||
                    to.Type == GCodeParser.GCodeMove.MoveType.LaserOff)
                    continue;

                DrawMove(from, to, i);
            }
        }

        private void DrawMove(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to, int index)
        {
            double centerY = cylinderLength / 2;

            // Apply transformation
            double fromY = TransformGCodeY(from.Y);
            double toY = TransformGCodeY(to.Y);

            if (index <= 3) // Debug first few moves
            {
                System.Diagnostics.Debug.WriteLine($"DrawMove #{index}:");
                System.Diagnostics.Debug.WriteLine($"  Original: Y {from.Y:F3} → {to.Y:F3}");
                System.Diagnostics.Debug.WriteLine($"  Transformed: Y {fromY:F3} → {toY:F3}");
            }

            var line = new Line
            {
                X1 = 50 + (from.C / 360.0) * 2 * Math.PI * cylinderRadius * scale,
                Y1 = 50 + (centerY + fromY) * scale,
                X2 = 50 + (to.C / 360.0) * 2 * Math.PI * cylinderRadius * scale,
                Y2 = 50 + (centerY + toY) * scale,
                Tag = "gcode"
            };

            // Style based on move type
            switch (to.Type)
            {
                case GCodeParser.GCodeMove.MoveType.Cut:
                    line.Stroke = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
                    line.StrokeThickness = 3;
                    break;

                case GCodeParser.GCodeMove.MoveType.Rapid:
                    line.Stroke = new SolidColorBrush(Color.FromArgb(128, 128, 128, 128));
                    line.StrokeThickness = 1;
                    line.StrokeDashArray = new DoubleCollection { 5, 5 };
                    break;
            }

            drawingCanvas.Children.Add(line);

            if (index % 5 == 0 && to.Type == GCodeParser.GCodeMove.MoveType.Cut)
            {
                DrawDirectionArrow(from, to);
            }
        }

        private void DrawPiercePoints()
        {
            var piercePoints = parsedGCode.Moves
                .Where(m => m.Type == GCodeParser.GCodeMove.MoveType.Pierce)
                .ToList();

            double centerY = cylinderLength / 2;

            foreach (var pierce in piercePoints)
            {
                var marker = new Ellipse
                {
                    Width = 8,
                    Height = 8,
                    Fill = Brushes.Orange,
                    Stroke = Brushes.DarkOrange,
                    StrokeThickness = 1,
                    Tag = "gcode"
                };

                double x = 50 + (pierce.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
                double y = 50 + (centerY + TransformGCodeY(pierce.Y)) * scale;

                Canvas.SetLeft(marker, x - 4);
                Canvas.SetTop(marker, y - 4);
                drawingCanvas.Children.Add(marker);

                marker.ToolTip = $"Pierce #{piercePoints.IndexOf(pierce) + 1}\n" +
                                $"Y: {pierce.Y:F2}, C: {pierce.C:F1}°";
            }
        }

        private void DrawStartEndMarkers()
        {
            if (parsedGCode == null || parsedGCode.Moves.Count == 0) return;

            var start = parsedGCode.Moves.First();
            var end = parsedGCode.Moves.Last();

            double centerY = cylinderLength / 2;

            System.Diagnostics.Debug.WriteLine("\n🎯 === START/END MARKERS ===");
            System.Diagnostics.Debug.WriteLine($"Start: Y={start.Y:F3} → {TransformGCodeY(start.Y):F3}, C={start.C:F3}°");
            System.Diagnostics.Debug.WriteLine($"End: Y={end.Y:F3} → {TransformGCodeY(end.Y):F3}, C={end.C:F3}°");

            // START marker
            var startMarker = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = Brushes.LimeGreen,
                Stroke = Brushes.DarkGreen,
                StrokeThickness = 2,
                Tag = "gcode"
            };

            double startX = 50 + (start.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double startY = 50 + (centerY + TransformGCodeY(start.Y)) * scale;

            Canvas.SetLeft(startMarker, startX - 10);
            Canvas.SetTop(startMarker, startY - 10);
            drawingCanvas.Children.Add(startMarker);

            var startText = new TextBlock
            {
                Text = "START",
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.DarkGreen,
                Tag = "gcode"
            };
            Canvas.SetLeft(startText, startX - 20);
            Canvas.SetTop(startText, startY - 30);
            drawingCanvas.Children.Add(startText);

            // END marker
            var endMarker = new Rectangle
            {
                Width = 20,
                Height = 20,
                Fill = Brushes.Red,
                Stroke = Brushes.DarkRed,
                StrokeThickness = 2,
                Tag = "gcode"
            };

            double endX = 50 + (end.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double endY = 50 + (centerY + TransformGCodeY(end.Y)) * scale;

            Canvas.SetLeft(endMarker, endX - 10);
            Canvas.SetTop(endMarker, endY - 10);
            drawingCanvas.Children.Add(endMarker);

            var endText = new TextBlock
            {
                Text = "END",
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.DarkRed,
                Tag = "gcode"
            };
            Canvas.SetLeft(endText, endX - 15);
            Canvas.SetTop(endText, endY + 25);
            drawingCanvas.Children.Add(endText);
        }

        private void DrawDirectionArrow(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to)
        {
            double centerY = cylinderLength / 2;

            double x1 = 50 + (from.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y1 = 50 + (centerY + TransformGCodeY(from.Y)) * scale;
            double x2 = 50 + (to.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y2 = 50 + (centerY + TransformGCodeY(to.Y)) * scale;

            double angle = Math.Atan2(y2 - y1, x2 - x1) * 180 / Math.PI;
            double midX = (x1 + x2) / 2;
            double midY = (y1 + y2) / 2;

            var arrow = new Polygon
            {
                Points = new PointCollection
        {
            new Point(0, 0),
            new Point(-8, -4),
            new Point(-8, 4)
        },
                Fill = Brushes.DarkBlue,
                Tag = "gcode",
                RenderTransform = new TransformGroup
                {
                    Children =
            {
                new RotateTransform(angle),
                new TranslateTransform(midX, midY)
            }
                }
            };

            drawingCanvas.Children.Add(arrow);
        }

        private void ClearGCodeLayer()
        {
            var toRemove = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .Where(e => e.Tag?.ToString() == "gcode" ||
                           e.Tag?.ToString() == "animation")
                .ToList();

            foreach (var element in toRemove)
            {
                drawingCanvas.Children.Remove(element);
            }
        }
        private void StartAnimation_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!ValidateGCodeData())
                {
                    return;
                }

                currentAnimationIndex = 0;
                isAnimating = true;

                // Update UI
                Dispatcher.Invoke(() =>
                {
                    btnPlay.IsEnabled = false;
                    btnPause.IsEnabled = true;
                    btnStop.IsEnabled = true;
                });

                // THÊM MỚI: Clear animation layers based on mode
                if (useEnhancedAnimation && enhancedAnimationController != null)
                {
                    // Enhanced mode: clear current but preserve completed cuts
                    enhancedAnimationController.ClearCurrentAnimation();
                }
                else
                {
                    // Standard mode: clear all
                    ClearAnimationLayer();
                }

                // Setup timer
                if (animationTimer != null)
                {
                    animationTimer.Stop();
                    animationTimer.Tick -= AnimationTimer_Tick;
                    animationTimer.Tick -= AnimationTimer_Tick_Enhanced; // Remove enhanced handler if exists
                }

                animationTimer = new DispatcherTimer
                {
                    Interval = TimeSpan.FromMilliseconds(Math.Max(20, 100 / speedSlider.Value))
                };

                // THÊM MỚI: Choose animation handler based on mode
                if (useEnhancedAnimation && enhancedAnimationController != null)
                {
                    animationTimer.Tick += AnimationTimer_Tick_Enhanced;
                }
                else
                {
                    animationTimer.Tick += AnimationTimer_Tick_Safe;
                }

                animationTimer.Start();
            }
            catch (Exception ex)
            {
                HandleAnimationError(ex);
            }
        }

        private void CompareToolpathWithGCode()
        {
            if (toolpaths?.Count > 0 && parsedGCode?.Moves?.Count > 0)
            {
                System.Diagnostics.Debug.WriteLine("=== Comparing Original vs Parsed ===");

                // Original toolpath
                var firstToolpath = toolpaths[0];
                System.Diagnostics.Debug.WriteLine($"Original toolpath {firstToolpath.EdgeId}:");
                System.Diagnostics.Debug.WriteLine($"  Points: {firstToolpath.Points.Count}");
                System.Diagnostics.Debug.WriteLine($"  First: Y={firstToolpath.Points[0].Y:F3}, " +
                                                 $"C={firstToolpath.Points[0].C:F3}");

                // Parsed G-Code
                var firstMove = parsedGCode.Moves.FirstOrDefault(m => m.Type == GCodeParser.GCodeMove.MoveType.Cut);
                if (firstMove != null)
                {
                    System.Diagnostics.Debug.WriteLine($"First G-Code cut:");
                    System.Diagnostics.Debug.WriteLine($"  Y={firstMove.Y:F3}, C={firstMove.C:F3}");
                }
            }
        }

        private void AnimationTimer_Tick_Safe(object sender, EventArgs e)
        {
            try
            {
                AnimationTimer_Tick(sender, e);
            }
            catch (Exception ex)
            {
                HandleAnimationError(ex);
            }
        }

        /// <summary>
        /// Enhanced animation timer tick với completed cuts tracking
        /// </summary>
        private void AnimationTimer_Tick_Enhanced(object sender, EventArgs e)
        {
            try
            {
                if (!isAnimating || currentAnimationIndex >= parsedGCode.Moves.Count - 1)
                {
                    StopAnimation();
                    return;
                }

                var currentMove = parsedGCode.Moves[currentAnimationIndex];
                var nextMove = parsedGCode.Moves[currentAnimationIndex + 1];

                // Skip non-movement commands quickly
                if (nextMove.Type == GCodeParser.GCodeMove.MoveType.Pierce ||
                    nextMove.Type == GCodeParser.GCodeMove.MoveType.LaserOff)
                {
                    currentAnimationIndex++;
                    return;
                }



                // ENHANCED: Use EnhancedAnimationController
                enhancedAnimationController.AnimateMoveEnhanced(currentMove, nextMove, currentAnimationIndex);
                enhancedAnimationController.UpdateToolPosition(nextMove);

                // Update progress với enhanced statistics
                double progress = (double)currentAnimationIndex / (parsedGCode.Moves.Count - 1) * 100;
                animationProgress.Value = progress;

                // Enhanced status với completed cuts info
                var stats = enhancedAnimationController.GetStats();
                txtAnimationStatus.Text = $"Move {currentAnimationIndex + 1}/{parsedGCode.Moves.Count} | " +
                                         $"Completed: {stats.CompletedCuts} cuts ({stats.TotalCutLength:F1}mm) | " +
                                         $"Y: {nextMove.Y:F2}, C: {nextMove.C:F1}°";
                if (txtCutStats != null)
                {
                    txtCutStats.Text = $"Cuts: {stats.CompletedCuts} | Length: {stats.TotalCutLength:F1}mm";
                }

                currentAnimationIndex++;
            }
            catch (Exception ex)
            {
                HandleAnimationError(ex);
            }
        }


        private void PauseAnimation_Click(object sender, RoutedEventArgs e)
        {
            if (animationTimer != null && animationTimer.IsEnabled)
            {
                animationTimer.Stop();
                btnPlay.IsEnabled = true;
                btnPause.IsEnabled = false;
            }
        }

        private void StopAnimation_Click(object sender, RoutedEventArgs e)
        {
            StopAnimation();
        }

        private void StopAnimation()
        {
            if (animationTimer != null)
            {
                animationTimer.Stop();
                animationTimer.Tick -= AnimationTimer_Tick;
                animationTimer.Tick -= AnimationTimer_Tick_Enhanced; // Remove enhanced handler
            }

            isAnimating = false;
            currentAnimationIndex = 0;

            // Update UI
            btnPlay.IsEnabled = true;
            btnPause.IsEnabled = false;
            btnStop.IsEnabled = false;

            // THÊM MỚI: Clear animation based on mode
            if (useEnhancedAnimation && enhancedAnimationController != null)
            {
                // Optional: Ask user if they want to clear completed cuts
                var result = MessageBox.Show(
                    "Do you want to clear completed cuts visualization?",
                    "Clear Completed Cuts",
                    MessageBoxButton.YesNo,
                    MessageBoxImage.Question);

                if (result == MessageBoxResult.Yes)
                {
                    enhancedAnimationController.ClearAllAnimation();
                }
                else
                {
                    enhancedAnimationController.ClearCurrentAnimation();
                }
            }
            else
            {
                ClearAnimationLayer();
            }

            // Reset progress
            animationProgress.Value = 0;
            txtAnimationStatus.Text = "Ready";
        }

        private void AnimationTimer_Tick(object sender, EventArgs e)
        {
            if (!isAnimating || currentAnimationIndex >= parsedGCode.Moves.Count - 1)
            {
                StopAnimation();
                return;
            }

            var currentMove = parsedGCode.Moves[currentAnimationIndex];
            var nextMove = parsedGCode.Moves[currentAnimationIndex + 1];

            // Skip non-movement commands quickly
            if (nextMove.Type == GCodeParser.GCodeMove.MoveType.Pierce ||
                nextMove.Type == GCodeParser.GCodeMove.MoveType.LaserOff)
            {
                currentAnimationIndex++;
                return;
            }

            // Animate the move
            AnimateMove(currentMove, nextMove);

            // Update progress
            double progress = (double)currentAnimationIndex / (parsedGCode.Moves.Count - 1) * 100;
            animationProgress.Value = progress;

            // Update status
            UpdateAnimationStatus(currentMove, nextMove);

            currentAnimationIndex++;
        }

        private void AnimateMove(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to)
        {
            double centerY = cylinderLength / 2;

            // Apply transformation
            double fromY = TransformGCodeY(from.Y);
            double toY = TransformGCodeY(to.Y);

            var animLine = new Line
            {
                X1 = 50 + (from.C / 360.0) * 2 * Math.PI * cylinderRadius * scale,
                Y1 = 50 + (centerY + fromY) * scale,
                X2 = 50 + (to.C / 360.0) * 2 * Math.PI * cylinderRadius * scale,
                Y2 = 50 + (centerY + toY) * scale,
                Tag = "animation"
            };

            // Style based on move type
            if (to.Type == GCodeParser.GCodeMove.MoveType.Cut)
            {
                animLine.Stroke = Brushes.Red;
                animLine.StrokeThickness = 4;
                animLine.Effect = new DropShadowEffect
                {
                    Color = Colors.Red,
                    BlurRadius = 10,
                    ShadowDepth = 0,
                    Opacity = 0.8
                };
            }
            else
            {
                animLine.Stroke = Brushes.Yellow;
                animLine.StrokeThickness = 2;
                animLine.StrokeDashArray = new DoubleCollection { 5, 5 };
            }

            drawingCanvas.Children.Add(animLine);
            UpdateToolIndicator(to);
            FadeOldAnimationLines();
        }

        private void EnhancedMode_Changed(object sender, RoutedEventArgs e)
        {
            useEnhancedAnimation = chkEnhancedMode.IsChecked ?? false;

            // Clear all animations when switching modes
            if (enhancedAnimationController != null)
            {
                enhancedAnimationController.ClearAllAnimation();
            }
            ClearAnimationLayer();

            // Update status
            txtAnimationStatus.Text = useEnhancedAnimation ?
                "Enhanced Mode: Completed cuts will remain visible" :
                "Standard Mode: All cuts fade after animation";
        }
        private void ClearCompleted_Click(object sender, RoutedEventArgs e)
        {
            if (enhancedAnimationController != null)
            {
                enhancedAnimationController.ClearAllAnimation();
                txtAnimationStatus.Text = "Cleared all completed cuts";
            }
        }
        private void UpdateToolIndicator(GCodeParser.GCodeMove position)
        {
            // Remove old tool
            var oldTool = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .FirstOrDefault(e => e.Tag?.ToString() == "tool");
            if (oldTool != null)
                drawingCanvas.Children.Remove(oldTool);

            double centerY = cylinderLength / 2;
            double transformedY = TransformGCodeY(position.Y);

            var toolGroup = new Canvas { Tag = "tool" };

            var outerCircle = new Ellipse
            {
                Width = 20,
                Height = 20,
                Stroke = Brushes.Black,
                StrokeThickness = 2,
                Fill = position.LaserOn ? Brushes.Red : Brushes.Blue
            };

            var innerDot = new Ellipse
            {
                Width = 6,
                Height = 6,
                Fill = Brushes.White
            };

            Canvas.SetLeft(innerDot, 7);
            Canvas.SetTop(innerDot, 7);

            toolGroup.Children.Add(outerCircle);
            toolGroup.Children.Add(innerDot);

            double x = 50 + (position.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y = 50 + (centerY + transformedY) * scale;

            Canvas.SetLeft(toolGroup, x - 10);
            Canvas.SetTop(toolGroup, y - 10);

            var animation = new DoubleAnimation
            {
                From = 0.5,
                To = 1.0,
                Duration = TimeSpan.FromMilliseconds(500),
                AutoReverse = true,
                RepeatBehavior = RepeatBehavior.Forever
            };
            outerCircle.BeginAnimation(OpacityProperty, animation);

            drawingCanvas.Children.Add(toolGroup);
            ScrollToPosition(x, y);
        }

        private void ScrollToPosition(double x, double y)
        {
            // Get viewport dimensions
            double viewWidth = scrollViewer.ViewportWidth;
            double viewHeight = scrollViewer.ViewportHeight;

            // Calculate if position is outside current view
            double currentX = scrollViewer.HorizontalOffset;
            double currentY = scrollViewer.VerticalOffset;

            // Add margin
            double margin = 50;

            if (x < currentX + margin || x > currentX + viewWidth - margin)
            {
                double newX = x - viewWidth / 2;
                scrollViewer.ScrollToHorizontalOffset(Math.Max(0, newX));
            }

            if (y < currentY + margin || y > currentY + viewHeight - margin)
            {
                double newY = y - viewHeight / 2;
                scrollViewer.ScrollToVerticalOffset(Math.Max(0, newY));
            }
        }

        private void FadeOldAnimationLines()
        {
            var animLines = drawingCanvas.Children
                .OfType<Line>()
                .Where(l => l.Tag?.ToString() == "animation")
                .OrderBy(l => drawingCanvas.Children.IndexOf(l))
                .ToList();

            // Keep only last N lines
            int maxLines = 20;
            if (animLines.Count > maxLines)
            {
                for (int i = 0; i < animLines.Count - maxLines; i++)
                {
                    drawingCanvas.Children.Remove(animLines[i]);
                }
            }

            // Fade older lines
            for (int i = 0; i < Math.Min(animLines.Count, maxLines); i++)
            {
                double opacity = 0.3 + (0.7 * i / maxLines);
                animLines[i].Opacity = opacity;
            }
        }

        private void UpdateAnimationStatus(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to)
        {
            string status = $"Move {currentAnimationIndex + 1}/{parsedGCode.Moves.Count} | ";
            status += $"Y: {to.Y:F2} C: {to.C:F1}° | ";
            status += to.Type == GCodeParser.GCodeMove.MoveType.Cut ? "CUTTING" : "RAPID";

            if (to.LaserOn)
                status += $" | Power: {to.LaserPower}%";

            txtAnimationStatus.Text = status;
        }

        private void SpeedSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!isInitialized) return;

            if (animationTimer != null)
            {
                animationTimer.Interval = TimeSpan.FromMilliseconds(100 / e.NewValue);
            }

            if (txtSpeed != null)
            {
                txtSpeed.Text = $"{e.NewValue:F1}x";
            }
        }
        private void LoadGCode_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog
            {
                Filter = "G-Code Files (*.nc;*.gcode)|*.nc;*.gcode|All Files (*.*)|*.*",
                Title = "Load G-Code File"
            };

            if (dialog.ShowDialog() == true)
            {
                try
                {
                    string gcode = File.ReadAllText(dialog.FileName);
                    LoadGCodeForVisualization(gcode);
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Error loading G-Code: {ex.Message}", "Error",
                                  MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
        }

        private void TestSimpleGCode()
        {
            string testGCode = @"; Test Circle
        G21 ; Metric
        G90 ; Absolute
        G0 Y0 C0 ; Start at center
        G0 Z0
         M3 S80 ; Laser on
        G1 Y0 C90 ; Quarter circle
        G1 Y0 C180 ; Half circle  
        G1 Y0 C270 ; Three quarters
        G1 Y0 C360 ; Full circle
        M5 ; Laser off
        M30";

            System.Diagnostics.Debug.WriteLine("=== Testing with simple G-Code ===");
            LoadGCodeForVisualization(testGCode);
        }

        private void ScrollViewer_PreviewMouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (Keyboard.IsKeyDown(Key.LeftCtrl))
            {
                e.Handled = true;
                // Let canvas handle zoom
            }
        }
        private void ShowRapids_Changed(object sender, RoutedEventArgs e)
        {
            // Re-draw with/without rapids
            if (parsedGCode != null)
            {
                DrawGCodeVisualization();
            }
        }

        private void ShowNumbers_Changed(object sender, RoutedEventArgs e)
        {
            if (chkShowNumbers.IsChecked == true && parsedGCode != null)
            {
                DrawMoveNumbers();
            }
            else
            {
                // Remove numbers
                var numbers = drawingCanvas.Children
                    .OfType<TextBlock>()
                    .Where(t => t.Tag?.ToString() == "movenumber")
                    .ToList();

                foreach (var num in numbers)
                {
                    drawingCanvas.Children.Remove(num);
                }
            }
        }

        private void DrawMoveNumbers()
        {
            int moveCount = 0;

            for (int i = 0; i < parsedGCode.Moves.Count; i++)
            {
                var move = parsedGCode.Moves[i];

                // Only number cut moves
                if (move.Type != GCodeParser.GCodeMove.MoveType.Cut) continue;

                moveCount++;

                // Show every 5th number to avoid clutter
                if (moveCount % 5 != 0) continue;

                var number = new TextBlock
                {
                    Text = moveCount.ToString(),
                    FontSize = 10,
                    FontWeight = FontWeights.Bold,
                    Foreground = Brushes.DarkBlue,
                    Tag = "movenumber"
                };

                double x = 50 + (move.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
                double y = 50 + (cylinderLength / 2 + move.Y) * scale;

                Canvas.SetLeft(number, x + 5);
                Canvas.SetTop(number, y - 15);
                drawingCanvas.Children.Add(number);
            }
        }

        private void DisplayStatistics()
        {
            if (parsedGCode == null) return;

            var stats = new TextBlock
            {
                Text = $"Total Moves: {parsedGCode.Moves.Count} | " +
                       $"Cut Length: {parsedGCode.TotalCutLength:F1}mm | " +
                       $"Rapid Length: {parsedGCode.TotalRapidLength:F1}mm | " +
                       $"Pierce Count: {parsedGCode.PierceCount}",
                FontSize = 12,
                Margin = new Thickness(5),
                Tag = "stats"
            };

            // Update status bar
            txtStatus.Text = stats.Text;
        }

        private void ClearAnimationLayer()
        {
            // THÊM NULL CHECK
            if (drawingCanvas == null)
            {
                System.Diagnostics.Debug.WriteLine("Warning: drawingCanvas is null in ClearAnimationLayer");
                return;
            }

            var toRemove = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .Where(e => e.Tag?.ToString() == "animation" ||
                           e.Tag?.ToString() == "tool")
                .ToList();

            foreach (var element in toRemove)
            {
                drawingCanvas.Children.Remove(element);
            }
        }

        // Helper to save current view as image
        private void SaveAsImage_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new SaveFileDialog
            {
                Filter = "PNG Image|*.png|JPEG Image|*.jpg",
                FileName = "toolpath_visualization.png"
            };

            if (dialog.ShowDialog() == true)
            {
                var renderBitmap = new RenderTargetBitmap(
                    (int)drawingCanvas.ActualWidth,
                    (int)drawingCanvas.ActualHeight,
                    96, 96, PixelFormats.Pbgra32);

                renderBitmap.Render(drawingCanvas);

                BitmapEncoder encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(renderBitmap));

                using (var stream = File.Create(dialog.FileName))
                {
                    encoder.Save(stream);
                }

                MessageBox.Show("Image saved successfully", "Success",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
        }

        /// <summary>
        /// Calculate coordinate transformation để sync animation với G-code machine coordinates
        /// </summary>

        #region Error Handling Helpers

        private void ShowErrorMessage(string message, string title = "Error")
        {
            Dispatcher.Invoke(() =>
            {
                MessageBox.Show(this, message, title, MessageBoxButton.OK, MessageBoxImage.Error);
            });
        }

        private void ShowWarningMessage(string message, string title = "Warning")
        {
            Dispatcher.Invoke(() =>
            {
                MessageBox.Show(this, message, title, MessageBoxButton.OK, MessageBoxImage.Warning);
            });
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            isInitialized = true;
            // Any initialization that needs controls to be ready
        }

        private bool ValidateGCodeData()
        {
            if (parsedGCode == null)
            {
                ShowWarningMessage("No G-Code data loaded", "No Data");
                return false;
            }

            if (parsedGCode.Moves == null || parsedGCode.Moves.Count == 0)
            {
                ShowWarningMessage("G-Code contains no moves", "Invalid Data");
                return false;
            }

            return true;
        }

        private void HandleAnimationError(Exception ex)
        {
            StopAnimation();
            System.Diagnostics.Debug.WriteLine($"Animation error: {ex}");
            ShowErrorMessage($"Animation error: {ex.Message}", "Animation Failed");
        }

        #endregion

    }
}
