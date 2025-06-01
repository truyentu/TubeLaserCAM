using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using TubeLaserCAM.UI.Models;

namespace TubeLaserCAM.UI.Views
{
    public partial class Unrolled2DView : Window
    {
        private List<UnrolledToolpath> toolpaths;
        private double zoomLevel = 1.0;
        private Point? lastMousePosition;
        private bool isPanning = false;
        private double scale = 1.0; // mm to pixel scale

        public Unrolled2DView(List<UnrolledToolpath> unrolledToolpaths, CylinderData cylinderInfo)
        {
            InitializeComponent();

            // THÊM DEBUG
            System.Diagnostics.Debug.WriteLine($"2D View received {unrolledToolpaths?.Count ?? 0} toolpaths");
            System.Diagnostics.Debug.WriteLine($"Cylinder: R={cylinderInfo?.Radius}, L={cylinderInfo?.Length}");

            this.toolpaths = unrolledToolpaths;

            // Calculate scale
            double circumference = 2 * Math.PI * cylinderInfo.Radius;
            scale = Math.Min(
                drawingCanvas.Width / circumference,
                drawingCanvas.Height / cylinderInfo.Length
            ) * 0.8;

            // THÊM DEBUG
            System.Diagnostics.Debug.WriteLine($"Scale calculated: {scale}, Circumference: {circumference}");

            DrawUnrolledView(cylinderInfo);
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

        private void DrawToolpath(UnrolledToolpath toolpath, CylinderData cylinderInfo)
        {
            System.Diagnostics.Debug.WriteLine($"Drawing toolpath {toolpath.EdgeId} with {toolpath.Points?.Count ?? 0} points");

            if (toolpath.Points.Count < 2) return;

            var polyline = new Polyline
            {
                Stroke = GetColorForEdgeType(toolpath.EdgeInfo.Type),
                StrokeThickness = 2,
                StrokeLineJoin = PenLineJoin.Round
            };

            double centerY = cylinderInfo.Length / 2;

            foreach (var point in toolpath.Points)
            {
                // Convert (Y,C) to canvas coordinates
                double x = 50 + (point.C / 360.0) * 2 * Math.PI * cylinderInfo.Radius * scale;
                double y = 50 + (centerY + point.Y) * scale;

                polyline.Points.Add(new Point(x, y));
            }

            drawingCanvas.Children.Add(polyline);

            // Add tooltip
            var tooltip = new ToolTip
            {
                Content = $"Edge #{toolpath.EdgeId}\n" +
                         $"Type: {toolpath.EdgeInfo.Type}\n" +
                         $"Length: {toolpath.EdgeInfo.Length:F2}mm\n" +
                         $"Y: [{toolpath.MinY:F2}, {toolpath.MaxY:F2}]\n" +
                         $"Rotation: {toolpath.TotalRotation:F1}°"
            };
            polyline.ToolTip = tooltip;
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
    }
}