// TubeLaserCAM.UI/Models/EnhancedAnimationController.cs
// ENHANCED ANIMATION với COMPLETED CUTS VISUALIZATION

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Effects;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace TubeLaserCAM.UI.Models
{
    /// <summary>
    /// Enhanced Animation Controller với Completed Cuts Tracking
    /// - Red glowing lines for current cutting (fade after completion)
    /// - Blue permanent lines for completed cuts
    /// - Enhanced legend và visual feedback
    /// </summary>
    public class EnhancedAnimationController
    {
        // Canvas reference
        private Canvas drawingCanvas;
        private double cylinderRadius;
        private double cylinderLength;
        private double scale;

        // Animation tracking
        private List<Line> completedCuts = new List<Line>();
        private HashSet<int> completedMoveIndices = new HashSet<int>();
        private int totalCompletedCuts = 0;

        // Visual settings
        public bool EnableCompletedCutsVisualization { get; set; } = true;
        public Color CompletedCutColor { get; set; } = Color.FromRgb(74, 144, 226); // Sky Blue #4A90E2
        public Color CurrentCuttingColor { get; set; } = Colors.Red;
        public Color RapidMoveColor { get; set; } = Colors.Yellow;
        public Color ToolColor { get; set; } = Colors.Orange;

        public EnhancedAnimationController(Canvas canvas, double radius, double length, double canvasScale)
        {
            drawingCanvas = canvas;
            cylinderRadius = radius;
            cylinderLength = length;
            scale = canvasScale;
        }

        /// <summary>
        /// MAIN METHOD: Enhanced animation của một G-Code move
        /// </summary>
        public void AnimateMoveEnhanced(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to, int moveIndex)
        {
            System.Diagnostics.Debug.WriteLine($"🎬 Animating move #{moveIndex}: {from.Y:F2},{from.C:F1}° → {to.Y:F2},{to.C:F1}° ({to.Type})");

            // Calculate canvas coordinates
            double x1 = 50 + (from.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y1 = 50 + (cylinderLength / 2 + from.Y) * scale;
            double x2 = 50 + (to.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y2 = 50 + (cylinderLength / 2 + to.Y) * scale;

            if (to.Type == GCodeParser.GCodeMove.MoveType.Cut)
            {
                // CUTTING MOVE: Red glow → Blue permanent
                AnimateCuttingMove(x1, y1, x2, y2, from, to, moveIndex);
            }
            else if (to.Type == GCodeParser.GCodeMove.MoveType.Rapid)
            {
                // RAPID MOVE: Yellow dashed → fade out
                AnimateRapidMove(x1, y1, x2, y2, from, to);
            }
        }

        /// <summary>
        /// Animate cutting move với red glow → blue permanent transition
        /// </summary>
        private void AnimateCuttingMove(double x1, double y1, double x2, double y2, 
                                       GCodeParser.GCodeMove from, GCodeParser.GCodeMove to, int moveIndex)
        {
            // STEP 1: Create red glowing cutting line
            var cuttingLine = new Line
            {
                X1 = x1, Y1 = y1, X2 = x2, Y2 = y2,
                Stroke = new SolidColorBrush(CurrentCuttingColor),
                StrokeThickness = 4,
                Tag = "current_cutting",
                Opacity = 1.0
            };

            // Add intense glow effect
            cuttingLine.Effect = new DropShadowEffect
            {
                Color = CurrentCuttingColor,
                BlurRadius = 10,
                ShadowDepth = 0,
                Opacity = 0.9
            };

            // Add to canvas
            drawingCanvas.Children.Add(cuttingLine);

            System.Diagnostics.Debug.WriteLine($"🔥 Added red cutting line: {cuttingLine.X1:F1},{cuttingLine.Y1:F1} → {cuttingLine.X2:F1},{cuttingLine.Y2:F1}");

            // STEP 2: Schedule conversion to completed cut (blue permanent)
            var conversionTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(800) // Show red for 800ms
            };

            conversionTimer.Tick += (s, e) =>
            {
                conversionTimer.Stop();
                ConvertToCompletedCut(cuttingLine, from, to, moveIndex);
            };

            conversionTimer.Start();
        }

        /// <summary>
        /// Animate rapid move với yellow dashed line
        /// </summary>
        private void AnimateRapidMove(double x1, double y1, double x2, double y2, 
                                     GCodeParser.GCodeMove from, GCodeParser.GCodeMove to)
        {
            var rapidLine = new Line
            {
                X1 = x1, Y1 = y1, X2 = x2, Y2 = y2,
                Stroke = new SolidColorBrush(RapidMoveColor),
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 5, 5 },
                Tag = "rapid_move",
                Opacity = 0.7
            };

            drawingCanvas.Children.Add(rapidLine);

            System.Diagnostics.Debug.WriteLine($"⚡ Added yellow rapid line: {rapidLine.X1:F1},{rapidLine.Y1:F1} → {rapidLine.X2:F1},{rapidLine.Y2:F1}");

            // Fade out rapid move after delay
            var fadeTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(1200)
            };

            fadeTimer.Tick += (s, e) =>
            {
                fadeTimer.Stop();
                drawingCanvas.Children.Remove(rapidLine);
                System.Diagnostics.Debug.WriteLine($"💨 Removed rapid line (faded)");
            };

            fadeTimer.Start();
        }

        /// <summary>
        /// Convert red cutting line to permanent blue completed cut
        /// </summary>
        private void ConvertToCompletedCut(Line cuttingLine, GCodeParser.GCodeMove from, 
                                         GCodeParser.GCodeMove to, int moveIndex)
        {
            if (!EnableCompletedCutsVisualization)
            {
                drawingCanvas.Children.Remove(cuttingLine);
                return;
            }

            try
            {
                // Remove red cutting line
                drawingCanvas.Children.Remove(cuttingLine);

                // Create PERMANENT blue completed cut line
                var completedLine = new Line
                {
                    X1 = cuttingLine.X1,
                    Y1 = cuttingLine.Y1,
                    X2 = cuttingLine.X2,
                    Y2 = cuttingLine.Y2,
                    Stroke = new SolidColorBrush(CompletedCutColor), // Sky Blue #4A90E2
                    StrokeThickness = 3,
                    Opacity = 0.85,
                    Tag = "completed_cut" // PERMANENT TAG
                };

                // Add subtle blue glow
                completedLine.Effect = new DropShadowEffect
                {
                    Color = CompletedCutColor,
                    BlurRadius = 2,
                    ShadowDepth = 1,
                    Opacity = 0.4
                };

                // Calculate cut length for tooltip
                double cutLength = CalculateLineLength(from, to);
                totalCompletedCuts++;

                // Enhanced tooltip
                completedLine.ToolTip = $"✅ COMPLETED CUT #{totalCompletedCuts}\n" +
                                       $"From: Y={from.Y:F2}mm, C={from.C:F1}°\n" +
                                       $"To: Y={to.Y:F2}mm, C={to.C:F1}°\n" +
                                       $"Length: {cutLength:F2}mm\n" +
                                       $"Move Index: {moveIndex}\n" +
                                       $"Status: PERMANENT (won't fade)";

                // Add to canvas và tracking
                drawingCanvas.Children.Add(completedLine);
                completedCuts.Add(completedLine);
                completedMoveIndices.Add(moveIndex);

                System.Diagnostics.Debug.WriteLine($"✅ CONVERTED TO COMPLETED CUT #{totalCompletedCuts}: " +
                                                 $"Y={to.Y:F2}, C={to.C:F1}° (PERMANENT BLUE)");
                System.Diagnostics.Debug.WriteLine($"   Total completed cuts: {completedCuts.Count}");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"❌ Error converting to completed cut: {ex.Message}");
            }
        }

        /// <summary>
        /// Update tool position indicator
        /// </summary>
        public void UpdateToolPosition(GCodeParser.GCodeMove position)
        {
            // Remove old tool indicator
            var oldTools = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .Where(e => e.Tag?.ToString() == "tool_indicator")
                .ToList();

            foreach (var oldTool in oldTools)
            {
                drawingCanvas.Children.Remove(oldTool);
            }

            // Create new tool indicator
            double x = 50 + (position.C / 360.0) * 2 * Math.PI * cylinderRadius * scale;
            double y = 50 + (cylinderLength / 2 + position.Y) * scale;

            var toolGroup = new Canvas
            {
                Tag = "tool_indicator"
            };

            // Outer circle với laser status color
            var outerCircle = new Ellipse
            {
                Width = 18,
                Height = 18,
                Stroke = Brushes.Black,
                StrokeThickness = 2,
                Fill = position.LaserOn ? 
                       new SolidColorBrush(CurrentCuttingColor) : 
                       new SolidColorBrush(ToolColor)
            };

            // Inner dot
            var innerDot = new Ellipse
            {
                Width = 6,
                Height = 6,
                Fill = Brushes.White
            };

            Canvas.SetLeft(innerDot, 6);
            Canvas.SetTop(innerDot, 6);

            toolGroup.Children.Add(outerCircle);
            toolGroup.Children.Add(innerDot);

            Canvas.SetLeft(toolGroup, x - 9);
            Canvas.SetTop(toolGroup, y - 9);

            // Enhanced tooltip
            var toolTip = $"🔧 TOOL POSITION\n" +
                         $"Y: {position.Y:F2}mm\n" +
                         $"C: {position.C:F1}°\n" +
                         $"Laser: {(position.LaserOn ? "ON" : "OFF")}\n" +
                         $"Power: {position.LaserPower}%\n" +
                         $"Feed: {position.FeedRate:F0}mm/min";

            toolGroup.ToolTip = toolTip;

            drawingCanvas.Children.Add(toolGroup);

            System.Diagnostics.Debug.WriteLine($"🔧 Tool updated: Y={position.Y:F2}, C={position.C:F1}°, Laser={(position.LaserOn ? "ON" : "OFF")}");
        }

        /// <summary>
        /// Get animation statistics
        /// </summary>
        public AnimationStats GetStats()
        {
            return new AnimationStats
            {
                CompletedCuts = completedCuts.Count,
                TotalCutLength = completedCuts.Sum(c => CalculateLineLength(c)),
                CompletedMoveIndices = completedMoveIndices.ToList()
            };
        }

        /// <summary>
        /// Clear all current animation elements but PRESERVE completed cuts
        /// </summary>
        public void ClearCurrentAnimation()
        {
            var toRemove = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .Where(e => e.Tag?.ToString() == "current_cutting" ||
                           e.Tag?.ToString() == "rapid_move" ||
                           e.Tag?.ToString() == "tool_indicator")
                .ToList();

            foreach (var element in toRemove)
            {
                drawingCanvas.Children.Remove(element);
            }

            System.Diagnostics.Debug.WriteLine($"🧹 Cleared current animation, preserved {completedCuts.Count} completed cuts");
        }

        /// <summary>
        /// Clear ALL animation including completed cuts
        /// </summary>
        public void ClearAllAnimation()
        {
            var toRemove = drawingCanvas.Children
                .OfType<FrameworkElement>()
                .Where(e => e.Tag?.ToString() == "current_cutting" ||
                           e.Tag?.ToString() == "rapid_move" ||
                           e.Tag?.ToString() == "tool_indicator" ||
                           e.Tag?.ToString() == "completed_cut")
                .ToList();

            foreach (var element in toRemove)
            {
                drawingCanvas.Children.Remove(element);
            }

            // Reset tracking
            completedCuts.Clear();
            completedMoveIndices.Clear();
            totalCompletedCuts = 0;

            System.Diagnostics.Debug.WriteLine($"🧹 Cleared ALL animation including completed cuts");
        }

        /// <summary>
        /// Calculate line length
        /// </summary>
        private double CalculateLineLength(GCodeParser.GCodeMove from, GCodeParser.GCodeMove to)
        {
            double deltaY = to.Y - from.Y;
            double deltaC = (to.C - from.C) * Math.PI * cylinderRadius / 180;
            return Math.Sqrt(deltaY * deltaY + deltaC * deltaC);
        }

        /// <summary>
        /// Calculate line length from canvas line
        /// </summary>
        private double CalculateLineLength(Line line)
        {
            double deltaX = line.X2 - line.X1;
            double deltaY = line.Y2 - line.Y1;
            return Math.Sqrt(deltaX * deltaX + deltaY * deltaY) / scale;
        }

        /// <summary>
        /// Generate legend information for UI
        /// </summary>
        public LegendInfo GetLegendInfo()
        {
            return new LegendInfo
            {
                CompletedCutColor = CompletedCutColor,
                CompletedCutDescription = "Completed cuts - remain visible permanently",
                CurrentCuttingColor = CurrentCuttingColor,
                CurrentCuttingDescription = "Currently cutting - shows active laser path with glow",
                RapidMoveColor = RapidMoveColor,
                RapidMoveDescription = "Rapid positioning moves - fade after completion",
                ToolColor = ToolColor,
                ToolDescription = "Current tool position - red when cutting, orange when moving"
            };
        }
    }

    /// <summary>
    /// Animation statistics
    /// </summary>
    public class AnimationStats
    {
        public int CompletedCuts { get; set; }
        public double TotalCutLength { get; set; }
        public List<int> CompletedMoveIndices { get; set; }
    }

    /// <summary>
    /// Legend information for UI
    /// </summary>
    public class LegendInfo
    {
        public Color CompletedCutColor { get; set; }
        public string CompletedCutDescription { get; set; }
        public Color CurrentCuttingColor { get; set; }
        public string CurrentCuttingDescription { get; set; }
        public Color RapidMoveColor { get; set; }
        public string RapidMoveDescription { get; set; }
        public Color ToolColor { get; set; }
        public string ToolDescription { get; set; }
    }
}
