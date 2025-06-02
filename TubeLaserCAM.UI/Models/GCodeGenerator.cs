// TubeLaserCAM.UI/Models/GCodeGenerator.cs
// ENHANCED VERSION với CORRECTED START POINT LOGIC và C-AXIS UNROLL CONSISTENCY

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
            System.Diagnostics.Debug.WriteLine($"=== 🔧 ENHANCED G-CODE WITH UNROLL CONSISTENCY 🔧 ===");
            System.Diagnostics.Debug.WriteLine($"Input toolpaths: {toolpaths.Count}");
            System.Diagnostics.Debug.WriteLine($"UseDuplicateFix: {settings.UseDuplicateFix}");

            // FIX: Always use enhanced profile grouping
            if (settings.UseDuplicateFix)
            {
                System.Diagnostics.Debug.WriteLine(">>> 🎯 USING COMPLETE PROFILE GROUPING + UNROLL CONSISTENCY 🎯");
                return GenerateGCodeWithCompleteProfiles(toolpaths, cylinderInfo);
            }

            gcode.Clear();
            return GenerateGCodeFixed(toolpaths, cylinderInfo);
        }

        /// <summary>
        /// MAIN METHOD: Generate G-code with complete profile grouping
        /// </summary>
        public string GenerateGCodeWithCompleteProfiles(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            System.Diagnostics.Debug.WriteLine("=== 🎯 COMPLETE PROFILE GROUPING + UNROLL CONSISTENCY 🎯 ===");
            System.Diagnostics.Debug.WriteLine("✅ Group connected edges into complete profiles");
            System.Diagnostics.Debug.WriteLine("✅ Single pierce per complete profile");
            System.Diagnostics.Debug.WriteLine("✅ C-axis unroll consistency [0°, 360°]");
            System.Diagnostics.Debug.WriteLine("✅ Natural start points for profiles");
            
            try
            {
                // Step 1: Remove duplicates
                var uniqueToolpaths = RemoveDuplicateToolpathsEnhanced(toolpaths);
                
                // Step 2: Group into complete profiles
                var completeProfiles = GroupEdgesIntoCompleteProfiles(uniqueToolpaths);
                
                // Step 3: Generate G-code by complete profiles
                return GenerateGCodeByCompleteProfiles(completeProfiles, cylinderInfo);
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
        /// Group edges into complete profiles (rectangles, circles, etc.)
        /// </summary>
        private List<CompleteProfile> GroupEdgesIntoCompleteProfiles(List<UnrolledToolpath> toolpaths)
        {
            var profiles = new List<CompleteProfile>();
            var usedEdges = new HashSet<int>();

            System.Diagnostics.Debug.WriteLine($"🔗 Grouping {toolpaths.Count} edges into complete profiles...");

            foreach (var startEdge in toolpaths)
            {
                if (usedEdges.Contains(startEdge.EdgeId))
                    continue;

                var profile = new CompleteProfile
                {
                    ProfileId = profiles.Count + 1,
                    Edges = new List<UnrolledToolpath>()
                };

                // Build connected chain of edges
                var currentEdge = startEdge;
                var edgeChain = new List<UnrolledToolpath>();

                do
                {
                    edgeChain.Add(currentEdge);
                    usedEdges.Add(currentEdge.EdgeId);

                    // Find next connected edge
                    var nextEdge = FindNextConnectedEdge(currentEdge, toolpaths, usedEdges);
                    currentEdge = nextEdge;

                } while (currentEdge != null && !IsProfileClosed(edgeChain));

                profile.Edges = edgeChain;
                profile.IsClosed = IsProfileClosed(edgeChain);
                profile.ProfileType = DetermineProfileType(edgeChain);

                // Calculate bounds
                var allPoints = edgeChain.SelectMany(e => e.Points).ToList();
                profile.MinY = allPoints.Min(p => p.Y);
                profile.MaxY = allPoints.Max(p => p.Y);
                profile.MinC = allPoints.Min(p => p.C);
                profile.MaxC = allPoints.Max(p => p.C);

                profiles.Add(profile);

                System.Diagnostics.Debug.WriteLine($"📋 Profile #{profile.ProfileId}: {profile.ProfileType}, {edgeChain.Count} edges, Closed={profile.IsClosed}");
                System.Diagnostics.Debug.WriteLine($"   Bounds: Y[{profile.MinY:F1}, {profile.MaxY:F1}], C[{profile.MinC:F1}, {profile.MaxC:F1}]");
                System.Diagnostics.Debug.WriteLine($"   Edges: {string.Join(", ", edgeChain.Select(e => $"#{e.EdgeId}({e.EdgeInfo?.Type})"))}");
            }

            return profiles;
        }

        /// <summary>
        /// Find next edge connected to current edge endpoint
        /// </summary>
        private UnrolledToolpath FindNextConnectedEdge(UnrolledToolpath currentEdge, List<UnrolledToolpath> allEdges, HashSet<int> usedEdges)
        {
            var currentEndPoint = currentEdge.Points.Last();
            const double tolerance = 2.0; // mm tolerance for connection

            foreach (var candidate in allEdges)
            {
                if (usedEdges.Contains(candidate.EdgeId))
                    continue;

                var candidateStartPoint = candidate.Points.First();
                var candidateEndPoint = candidate.Points.Last();

                // Check if candidate starts where current ends
                if (ArePointsConnected(currentEndPoint, candidateStartPoint, tolerance))
                {
                    System.Diagnostics.Debug.WriteLine($"🔗 Connected Edge#{candidate.EdgeId} to Edge#{currentEdge.EdgeId} (normal)");
                    return candidate;
                }

                // Check if candidate ends where current ends (reverse needed)
                if (ArePointsConnected(currentEndPoint, candidateEndPoint, tolerance))
                {
                    // Reverse the candidate edge
                    candidate.Points.Reverse();
                    System.Diagnostics.Debug.WriteLine($"🔄 Reversed and connected Edge#{candidate.EdgeId} to Edge#{currentEdge.EdgeId}");
                    return candidate;
                }
            }

            return null;
        }

        /// <summary>
        /// Check if two points are connected within tolerance
        /// </summary>
        private bool ArePointsConnected(UnrolledPoint p1, UnrolledPoint p2, double tolerance)
        {
            double yDiff = Math.Abs(p1.Y - p2.Y);
            double cDiff = Math.Abs(p1.C - p2.C);

            // Handle C-axis wrap around
            if (cDiff > 180)
                cDiff = 360 - cDiff;

            return yDiff < tolerance && cDiff < tolerance;
        }

        /// <summary>
        /// Check if profile forms a closed loop
        /// </summary>
        private bool IsProfileClosed(List<UnrolledToolpath> edges)
        {
            if (edges.Count < 2)
                return false;

            var firstPoint = edges.First().Points.First();
            var lastPoint = edges.Last().Points.Last();

            bool isClosed = ArePointsConnected(firstPoint, lastPoint, 2.0);
            System.Diagnostics.Debug.WriteLine($"🔍 Profile closure check: {(isClosed ? "CLOSED" : "OPEN")} (dist: Y={Math.Abs(firstPoint.Y - lastPoint.Y):F2}, C={Math.Abs(firstPoint.C - lastPoint.C):F2})");
            
            return isClosed;
        }

        /// <summary>
        /// Determine profile type (Rectangle, Circle, etc.)
        /// </summary>
        private string DetermineProfileType(List<UnrolledToolpath> edges)
        {
            if (edges.Count == 1 && edges[0].EdgeInfo?.Type == "Circle")
                return "Circle";

            if (edges.Count == 4)
            {
                int lineCount = edges.Count(e => e.EdgeInfo?.Type == "Line");
                int bsplineCount = edges.Count(e => e.EdgeInfo?.Type == "BSpline");

                if (lineCount == 2 && bsplineCount == 2)
                    return "Rectangle";
            }

            return $"Complex_{edges.Count}edges";
        }

        /// <summary>
        /// Generate G-code by complete profiles
        /// </summary>
        private string GenerateGCodeByCompleteProfiles(List<CompleteProfile> profiles, CylinderData cylinderInfo)
        {
            gcode.Clear();
            processedEdges.Clear();
            processedMoves.Clear();
            
            AddHeader(cylinderInfo);
            AddInitialization();

            System.Diagnostics.Debug.WriteLine($"🚀 Processing {profiles.Count} complete profiles...");

            foreach (var profile in profiles)
            {
                ProcessCompleteProfileWithUnrollConsistency(profile);
            }

            AddFooter();
            
            System.Diagnostics.Debug.WriteLine("🎉 Complete profile G-code generation finished!");
            
            return gcode.ToString();
        }

        /// <summary>
        /// CORRECTED: Choose optimal start point for profile - UNROLL CONSISTENCY
        /// </summary>
        private UnrolledPoint ChooseOptimalStartPointForProfile(CompleteProfile profile)
        {
            var allPoints = profile.Edges.SelectMany(e => e.Points).ToList();
            if (allPoints.Count == 0) return null;

            var firstPoint = allPoints.First();
            var lastPoint = allPoints.Last();

            System.Diagnostics.Debug.WriteLine($"🎯 Analyzing profile start point strategy...");
            System.Diagnostics.Debug.WriteLine($"   Profile type: {profile.ProfileType}, Closed: {profile.IsClosed}");
            System.Diagnostics.Debug.WriteLine($"   First point: Y={firstPoint.Y:F1}, C={firstPoint.C:F1}");
            System.Diagnostics.Debug.WriteLine($"   Last point: Y={lastPoint.Y:F1}, C={lastPoint.C:F1}");

            // OPEN profile: PHẢI start từ điểm đầu tự nhiên
            if (!profile.IsClosed)
            {
                System.Diagnostics.Debug.WriteLine($"📍 Open profile: MUST use natural start point");
                return firstPoint;
            }

            // CLOSED profile: chọn endpoint gần 0° hoặc 360° nhất để optimize C-axis
            System.Diagnostics.Debug.WriteLine($"🔄 Closed profile: Selecting endpoint closest to 0°/360°");
            
            // Calculate distances to 0°/360°
            double firstDistanceTo0 = Math.Min(firstPoint.C, 360 - firstPoint.C);
            double lastDistanceTo0 = Math.Min(lastPoint.C, 360 - lastPoint.C);

            System.Diagnostics.Debug.WriteLine($"   First point distance to 0°/360°: {firstDistanceTo0:F1}°");
            System.Diagnostics.Debug.WriteLine($"   Last point distance to 0°/360°: {lastDistanceTo0:F1}°");

            if (firstDistanceTo0 <= lastDistanceTo0)
            {
                System.Diagnostics.Debug.WriteLine($"✅ SELECTED: First point (closer to boundary)");
                return firstPoint;
            }
            else
            {
                System.Diagnostics.Debug.WriteLine($"✅ SELECTED: Last point (closer to boundary) - Reordering profile");
                // Reorder profile để start từ last point
                ReorderProfileToStartFromEnd(profile);
                return profile.Edges[0].Points[0]; // New first point after reorder
            }
        }

        /// <summary>
        /// Reorder profile to start from end (for closed profiles)
        /// </summary>
        private void ReorderProfileToStartFromEnd(CompleteProfile profile)
        {
            var reversedEdges = profile.Edges.AsEnumerable().Reverse().ToList();
            
            // Reverse each edge's points as well
            foreach (var edge in reversedEdges)
            {
                edge.Points.Reverse();
            }
            
            profile.Edges = reversedEdges;
            System.Diagnostics.Debug.WriteLine($"🔄 Profile reordered to start from previous end point");
        }

        /// <summary>
        /// Normalize C value to unroll range [0, 360°] - CONSISTENT với unroll logic
        /// </summary>
        private double NormalizeToUnrollRange(double cValue)
        {
            while (cValue < 0) cValue += 360;
            while (cValue >= 360) cValue -= 360;
            return cValue;
        }

        /// <summary>
        /// Enhanced C-axis handling for unroll consistency
        /// </summary>
        private double HandleCAxisForUnrollConsistency(double currentC, double targetC)
        {
            // Always normalize to [0, 360) để match unroll logic
            double normalizedCurrent = NormalizeToUnrollRange(currentC);
            double normalizedTarget = NormalizeToUnrollRange(targetC);
            
            // Calculate shortest path movement
            double directMove = normalizedTarget - normalizedCurrent;
            double wrapMove = directMove > 0 ? directMove - 360 : directMove + 360;
            
            // Choose shortest path but keep result in [0, 360)
            double chosenMove = Math.Abs(directMove) <= Math.Abs(wrapMove) ? directMove : wrapMove;
            double resultC = normalizedCurrent + chosenMove;
            
            // Final normalization to [0, 360) - ALWAYS trong unroll range
            resultC = NormalizeToUnrollRange(resultC);
            
            System.Diagnostics.Debug.WriteLine($"🎯 C-axis unroll: {currentC:F1}° → {targetC:F1}° = {resultC:F1}° (move: {chosenMove:F1}°, normalized)");
            
            return resultC;
        }

        /// <summary>
        /// Process complete profile với UNROLL CONSISTENCY
        /// </summary>
        private void ProcessCompleteProfileWithUnrollConsistency(CompleteProfile profile)
        {
            if (profile.Edges.Count == 0)
                return;

            // STEP 1: Choose optimal start point based on unroll logic
            var optimalStartPoint = ChooseOptimalStartPointForProfile(profile);
            
            gcode.AppendLine($"; 🎯 COMPLETE PROFILE #{profile.ProfileId} - {profile.ProfileType} [UNROLL CONSISTENT]");
            gcode.AppendLine($"; Edges: {string.Join(", ", profile.Edges.Select(e => $"#{e.EdgeId}({e.EdgeInfo?.Type})"))}");
            gcode.AppendLine($"; Bounds: Y[{profile.MinY:F1}, {profile.MaxY:F1}], C[{profile.MinC:F1}, {profile.MaxC:F1}]");
            gcode.AppendLine($"; Closed: {profile.IsClosed}");
            gcode.AppendLine($"; Start strategy: {(profile.IsClosed ? "Optimal boundary selection" : "Natural start point")}");

            // Move to chosen start point với normalized C trong [0, 360°]
            double normalizedC = NormalizeToUnrollRange(optimalStartPoint.C);
            currentC = normalizedC;
            currentY = optimalStartPoint.Y;
            
            gcode.AppendLine($"G0 Y{optimalStartPoint.Y:F3} C{normalizedC:F3} ; Profile start (unroll consistent)");
            gcode.AppendLine("G0 Z0");

            // SINGLE PIERCE for entire profile
            gcode.AppendLine($"M3 S{settings.LaserPower} ; SINGLE PIERCE for complete {profile.ProfileType}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");

            // Cut ALL edges với C-axis unroll consistency
            bool isFirstEdge = true;
            foreach (var edge in profile.Edges)
            {
                System.Diagnostics.Debug.WriteLine($"🔥 Cutting Edge#{edge.EdgeId} ({edge.EdgeInfo?.Type}) with unroll consistency");

                var points = edge.Points;
                int startIndex = isFirstEdge ? 1 : 1;

                // Special handling for circles
                if (edge.EdgeInfo?.Type == "Circle" && IsCompleteCircle(edge))
                {
                    int endIndex = Math.Max(startIndex, points.Count - 3);
                    for (int i = startIndex; i < endIndex; i++)
                    {
                        var point = points[i];
                        double normalizedTargetC = HandleCAxisForUnrollConsistency(currentC, point.C);
                        
                        if (Math.Abs(point.Y - currentY) > 0.001 || Math.Abs(normalizedTargetC - currentC) > 0.001)
                        {
                            gcode.AppendLine($"G1 Y{point.Y:F3} C{normalizedTargetC:F3}");
                            currentY = point.Y;
                            currentC = normalizedTargetC;
                        }
                    }
                }
                else
                {
                    // Lines and bsplines với unroll consistency
                    for (int i = startIndex; i < points.Count; i++)
                    {
                        var point = points[i];
                        double normalizedTargetC = HandleCAxisForUnrollConsistency(currentC, point.C);
                        
                        if (Math.Abs(point.Y - currentY) > 0.001 || Math.Abs(normalizedTargetC - currentC) > 0.001)
                        {
                            gcode.AppendLine($"G1 Y{point.Y:F3} C{normalizedTargetC:F3}");
                            currentY = point.Y;
                            currentC = normalizedTargetC;
                        }
                    }
                }

                isFirstEdge = false;
            }

            // SINGLE M5 for entire profile
            gcode.AppendLine($"M5 ; {profile.ProfileType} profile complete");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();

            System.Diagnostics.Debug.WriteLine($"✅ Profile #{profile.ProfileId} completed with unroll consistency - C range: [0°, 360°]");
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
            gcode.AppendLine("; *** ENHANCED VERSION - UNROLL CONSISTENCY + PROFILE GROUPING ***");
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

        /// <summary>
        /// Complete profile structure
        /// </summary>
        public class CompleteProfile
        {
            public int ProfileId { get; set; }
            public List<UnrolledToolpath> Edges { get; set; } = new List<UnrolledToolpath>();
            public bool IsClosed { get; set; }
            public string ProfileType { get; set; } = "Unknown";
            public double MinY { get; set; }
            public double MaxY { get; set; }
            public double MinC { get; set; }
            public double MaxC { get; set; }
        }
    }
}
