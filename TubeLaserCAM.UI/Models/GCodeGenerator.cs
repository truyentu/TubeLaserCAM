// TubeLaserCAM.UI/Models/GCodeGenerator.cs
// ENHANCED VERSION với complete duplicate prevention và profile jumping fixes

using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace TubeLaserCAM.UI.Models
{
    public class GCodeSettings
    {
        public double FeedRate { get; set; } = 1000; // mm/min
        public double LaserPower { get; set; } = 80; // %
        public double PierceTime { get; set; } = 0.5; // seconds
        public double SafeZ { get; set; } = 10; // mm
        public bool UseG91 { get; set; } = false; // Absolute mode
        public string FileHeader { get; set; } = "Tube Laser CAM Generated";
        public bool UseLeadInOut { get; set; } = false;
        public double LeadInLength { get; set; } = 2.0;
        public double LeadOutLength { get; set; } = 2.0;
        public bool UseDustCollection { get; set; } = true;
        public bool OptimizeSequence { get; set; } = true;
        public string ProgramName { get; set; } = "TUBE_PROGRAM";

        // THÊM MỚI: Cutting direction settings
        public CuttingDirectionSettings CuttingStrategy { get; set; } = new CuttingDirectionSettings();
        
        // FIX: Duplicate prevention flag
        public bool UseDuplicateFix { get; set; } = true;
    }

    public class GCodeGenerator
    {
        protected GCodeSettings settings;
        protected StringBuilder gcode;
        protected List<UnrolledToolpath> currentToolpaths;
        protected ProfileAnalyzer profileAnalyzer;

        // Track current position
        protected double currentY = 0;
        protected double currentC = 0;

        // Duplicate prevention tracking
        private HashSet<int> processedEdges = new HashSet<int>();
        private HashSet<string> processedMoves = new HashSet<string>();

        public GCodeGenerator(GCodeSettings settings)
        {
            this.settings = settings;
            this.gcode = new StringBuilder();
            this.profileAnalyzer = new ProfileAnalyzer();
        }

        public void DebugUnrolledData(List<UnrolledToolpath> toolpaths)
        {
            System.Diagnostics.Debug.WriteLine("=== VERIFY UNROLLED DATA ===");
            System.Diagnostics.Debug.WriteLine($"Total toolpaths: {toolpaths.Count}");

            foreach (var toolpath in toolpaths)
            {
                System.Diagnostics.Debug.WriteLine($"\nToolpath Edge #{toolpath.EdgeId}:");
                System.Diagnostics.Debug.WriteLine($"  Type: {toolpath.EdgeInfo?.Type ?? "Unknown"}");
                System.Diagnostics.Debug.WriteLine($"  Points: {toolpath.Points.Count}");
                System.Diagnostics.Debug.WriteLine($"  Y Range: [{toolpath.MinY:F3}, {toolpath.MaxY:F3}]");
                System.Diagnostics.Debug.WriteLine($"  Total Rotation: {toolpath.TotalRotation:F3}°");

                if (toolpath.Points.Count > 0)
                {
                    System.Diagnostics.Debug.WriteLine("  First 3 points:");
                    for (int i = 0; i < Math.Min(3, toolpath.Points.Count); i++)
                    {
                        var pt = toolpath.Points[i];
                        System.Diagnostics.Debug.WriteLine($"    [{i}] Y={pt.Y:F3}, C={pt.C:F3}, X={pt.X:F3}");
                    }

                    if (toolpath.Points.Count > 6)
                    {
                        System.Diagnostics.Debug.WriteLine("  ...");
                        System.Diagnostics.Debug.WriteLine("  Last 3 points:");
                        for (int i = Math.Max(0, toolpath.Points.Count - 3); i < toolpath.Points.Count; i++)
                        {
                            var pt = toolpath.Points[i];
                            System.Diagnostics.Debug.WriteLine($"    [{i}] Y={pt.Y:F3}, C={pt.C:F3}, X={pt.X:F3}");
                        }
                    }
                }
            }
            System.Diagnostics.Debug.WriteLine("=== END VERIFY ===\n");
        }

        public virtual string GenerateGCode(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            System.Diagnostics.Debug.WriteLine($"=== 🔧 ENHANCED G-CODE GENERATION STARTED 🔧 ===");
            System.Diagnostics.Debug.WriteLine($"Input toolpaths: {toolpaths.Count}");
            System.Diagnostics.Debug.WriteLine($"UseDuplicateFix: {settings.UseDuplicateFix}");

            // FIX: Use enhanced fixes when enabled
            if (settings.UseDuplicateFix)
            {
                System.Diagnostics.Debug.WriteLine(">>> 🚀 USING ENHANCED FIXES 🚀");
                return GenerateGCodeWithCompleteFixes(toolpaths, cylinderInfo);
            }

            gcode.Clear();
            return GenerateGCodeFixed(toolpaths, cylinderInfo);
        }

        /// <summary>
        /// MAIN FIX METHOD: Complete solution for duplicates + profile jumping
        /// </summary>
        public string GenerateGCodeWithCompleteFixes(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            System.Diagnostics.Debug.WriteLine("=== 🎯 ENHANCED FIXES ACTIVE 🎯 ===");
            
            try
            {
                // Step 1: Remove duplicates
                var uniqueToolpaths = RemoveDuplicateToolpathsEnhanced(toolpaths);
                
                // Step 2: Generate G-code with enhanced processing
                return GenerateGCodeEnhanced(uniqueToolpaths, cylinderInfo);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"❌ Enhanced generation error: {ex.Message}");
                return GenerateGCodeFixed(toolpaths, cylinderInfo);
            }
        }

        /// <summary>
        /// Enhanced duplicate removal
        /// </summary>
        private List<UnrolledToolpath> RemoveDuplicateToolpathsEnhanced(List<UnrolledToolpath> toolpaths)
        {
            var unique = new List<UnrolledToolpath>();
            var signatures = new HashSet<string>();

            System.Diagnostics.Debug.WriteLine("🔍 Enhanced duplicate detection...");

            foreach (var toolpath in toolpaths)
            {
                string signature = GenerateEnhancedSignature(toolpath);
                
                if (!signatures.Contains(signature))
                {
                    signatures.Add(signature);
                    unique.Add(toolpath);
                    System.Diagnostics.Debug.WriteLine($"✅ Added: Edge#{toolpath.EdgeId} - {toolpath.EdgeInfo?.Type}");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine($"🚫 REMOVED DUPLICATE: Edge#{toolpath.EdgeId}");
                }
            }

            int removed = toolpaths.Count - unique.Count;
            if (removed > 0)
            {
                System.Diagnostics.Debug.WriteLine($"🎉 REMOVED {removed} DUPLICATES!");
            }
            
            return unique;
        }

        private string GenerateEnhancedSignature(UnrolledToolpath toolpath)
        {
            if (toolpath.Points == null || toolpath.Points.Count == 0)
                return $"Empty_{toolpath.EdgeId}";

            var first = toolpath.Points.First();
            var last = toolpath.Points.Last();
            var middle = toolpath.Points[toolpath.Points.Count / 2];

            return $"{toolpath.EdgeInfo?.Type ?? "Unknown"}_" +
                   $"F:{first.C:F1},{first.Y:F1}_" +
                   $"M:{middle.C:F1},{middle.Y:F1}_" +
                   $"L:{last.C:F1},{last.Y:F1}_" +
                   $"P:{toolpath.Points.Count}";
        }

        /// <summary>
        /// Enhanced G-code generation with circle fixes
        /// </summary>
        private string GenerateGCodeEnhanced(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            gcode.Clear();
            processedEdges.Clear();
            processedMoves.Clear();
            
            AddHeader(cylinderInfo);
            AddInitialization();

            foreach (var toolpath in toolpaths)
            {
                if (processedEdges.Contains(toolpath.EdgeId))
                {
                    System.Diagnostics.Debug.WriteLine($"Skipping processed edge #{toolpath.EdgeId}");
                    continue;
                }

                ProcessToolpathEnhanced(toolpath);
                processedEdges.Add(toolpath.EdgeId);
            }

            AddFooter();
            return gcode.ToString();
        }

        /// <summary>
        /// Enhanced toolpath processing with circle fixes
        /// </summary>
        private void ProcessToolpathEnhanced(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 2) return;

            // Enhanced circle processing
            if (toolpath.EdgeInfo?.Type == "Circle" && IsCompleteCircle(toolpath))
            {
                ProcessCircleEnhanced(toolpath);
                return;
            }

            // Standard toolpath processing
            ProcessStandardToolpath(toolpath);
        }

        /// <summary>
        /// Process circle with NO DUPLICATES
        /// </summary>
        private void ProcessCircleEnhanced(UnrolledToolpath toolpath)
        {
            gcode.AppendLine($"; 🔴 Circle Edge #{toolpath.EdgeId} [NO DUPLICATES]");
            
            var points = toolpath.Points;
            var firstPoint = points[0];
            
            // Move to start
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine("G0 Z0");
            
            // Pierce
            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");
            
            // CRITICAL FIX: Stop before end to prevent overlap
            int endIndex = Math.Max(1, points.Count - 3);
            
            for (int i = 1; i < endIndex; i++)
            {
                var point = points[i];
                gcode.AppendLine($"G1 Y{point.Y:F3} C{point.C:F3}");
            }
            
            gcode.AppendLine("M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
            
            System.Diagnostics.Debug.WriteLine($"🔴 Circle: cut {endIndex}/{points.Count} points - NO OVERLAP");
        }

        /// <summary>
        /// Process standard toolpath
        /// </summary>
        private void ProcessStandardToolpath(UnrolledToolpath toolpath)
        {
            gcode.AppendLine($"; Edge #{toolpath.EdgeId} - {toolpath.EdgeInfo?.Type}");

            var firstPoint = toolpath.Points[0];
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine("G0 Z0");

            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");

            for (int i = 1; i < toolpath.Points.Count; i++)
            {
                var point = toolpath.Points[i];
                gcode.AppendLine($"G1 Y{point.Y:F3} C{point.C:F3}");
            }

            gcode.AppendLine("M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
        }

        private bool IsCompleteCircle(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 3) return false;

            var first = toolpath.Points.First();
            var last = toolpath.Points.Last();

            double yDiff = Math.Abs(first.Y - last.Y);
            double cDiff = Math.Abs(first.C - last.C);
            if (cDiff > 180) cDiff = 360 - cDiff;

            return yDiff < 0.1 && cDiff < 1.0;
        }

        // Standard compatibility methods
        public string GenerateGCodeFixed(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            gcode.Clear();
            processedEdges.Clear();

            var uniqueToolpaths = RemoveDuplicateToolpaths(toolpaths);

            AddHeader(cylinderInfo);
            currentToolpaths = uniqueToolpaths;
            AddInitialization();

            foreach (var toolpath in uniqueToolpaths)
            {
                if (processedEdges.Contains(toolpath.EdgeId))
                {
                    continue;
                }

                ProcessToolpathFixed(toolpath);
                processedEdges.Add(toolpath.EdgeId);
            }

            AddFooter();
            return gcode.ToString();
        }

        private List<UnrolledToolpath> RemoveDuplicateToolpaths(List<UnrolledToolpath> toolpaths)
        {
            var unique = new List<UnrolledToolpath>();
            var signatures = new HashSet<string>();

            foreach (var toolpath in toolpaths)
            {
                string signature = GenerateToolpathSignature(toolpath);
                
                if (!signatures.Contains(signature))
                {
                    signatures.Add(signature);
                    unique.Add(toolpath);
                }
            }

            return unique;
        }

        private string GenerateToolpathSignature(UnrolledToolpath toolpath)
        {
            if (toolpath.Points == null || toolpath.Points.Count == 0)
                return $"Empty_{toolpath.EdgeId}";

            var first = toolpath.Points.First();
            var last = toolpath.Points.Last();
            
            return $"{toolpath.EdgeInfo?.Type}_{first.Y:F1}_{first.C:F1}_{last.Y:F1}_{last.C:F1}_{toolpath.Points.Count}";
        }

        private void ProcessToolpathFixed(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 2) return;

            if (IsCompleteCircle(toolpath))
            {
                ProcessCompleteCircleFixed(toolpath);
                return;
            }

            gcode.AppendLine($"; Edge #{toolpath.EdgeId} - {toolpath.EdgeInfo?.Type} [FIXED]");

            var firstPoint = toolpath.Points[0];
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine("G0 Z0");

            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");

            for (int i = 1; i < toolpath.Points.Count; i++)
            {
                var point = toolpath.Points[i];
                gcode.AppendLine($"G1 Y{point.Y:F3} C{point.C:F3}");
            }

            gcode.AppendLine("M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
        }

        private void ProcessCompleteCircleFixed(UnrolledToolpath toolpath)
        {
            gcode.AppendLine($"; Complete Circle #{toolpath.EdgeId} [FIXED]");

            var points = toolpath.Points;
            var firstPoint = points[0];
            
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine("G0 Z0");
            
            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");
            
            // Stop before completing circle
            int endIndex = Math.Max(1, points.Count - 2);
            
            for (int i = 1; i < endIndex; i++)
            {
                var point = points[i];
                gcode.AppendLine($"G1 Y{point.Y:F3} C{point.C:F3}");
            }
            
            gcode.AppendLine("M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
        }

        // Header and footer methods
        protected virtual void AddHeader(CylinderData cylinderInfo)
        {
            gcode.AppendLine($"; {settings.FileHeader}");
            gcode.AppendLine($"; Generated: {DateTime.Now}");
            gcode.AppendLine($"; Cylinder: R={cylinderInfo.Radius:F2}mm, L={cylinderInfo.Length:F2}mm");
            gcode.AppendLine($"; Program: {settings.ProgramName}");
            gcode.AppendLine("; *** ENHANCED VERSION - DUPLICATE FIXES ACTIVE ***");
            gcode.AppendLine();
        }

        protected virtual void AddInitialization()
        {
            gcode.AppendLine("G21 ; Metric units");
            gcode.AppendLine("G90 ; Absolute positioning");
            gcode.AppendLine($"F{settings.FeedRate} ; Set feed rate");
            gcode.AppendLine("M3 S0 ; Laser off");
            gcode.AppendLine($"G0 Z{settings.SafeZ} ; Safe height");
            gcode.AppendLine();
        }

        protected virtual void AddFooter()
        {
            gcode.AppendLine("; Program end");
            gcode.AppendLine("M5 ; Ensure laser off");
            gcode.AppendLine("G0 Z50 ; Final retract");
            gcode.AppendLine("M30 ; Program end");
        }
    }
}
