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
        
        // Material Allowance Settings
        public double MaterialAllowanceStart { get; set; } = 5.0; // mm - phần phôi dư ở điểm bắt đầu
        public double MaterialAllowanceEnd { get; set; } = 5.0; // mm - phần phôi dư ở điểm kết thúc
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
        protected double currentZ = 0;
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
                    return candidate;
                }

                // Check if candidate ends where current ends (reverse needed)
                if (ArePointsConnected(currentEndPoint, candidateEndPoint, tolerance))
                {
                    // Reverse the candidate edge
                    candidate.Points.Reverse();
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
            System.Diagnostics.Debug.WriteLine($"Y Direction Preference: {settings.CuttingStrategy.YDirection}");
            
            // Apply Y Direction sorting based on user preference
            // CRITICAL: Transform unroll coordinates to machine coordinates
            var transformedProfiles = TransformUnrollToMachineCoordinates(profiles);
            
            var sortedProfiles = SortProfilesByYDirection(transformedProfiles);

            foreach (var profile in sortedProfiles)
            {
                // Apply Y Direction to individual profile cutting
                ApplyYDirectionToProfile(profile);
                
                ProcessCompleteProfileWithUnrollConsistency(profile);
            }

            AddFooter();
            
            System.Diagnostics.Debug.WriteLine("🎉 Complete profile G-code generation finished!");
            
            return gcode.ToString();
        }

        /// <summary>
        /// Sort profiles by Y Direction preference
        /// </summary>
        private List<CompleteProfile> SortProfilesByYDirection(List<CompleteProfile> profiles)
        {
            System.Diagnostics.Debug.WriteLine($"🎯 Sorting profiles by Y Direction: {settings.CuttingStrategy.YDirection}");
            
            var sortedProfiles = new List<CompleteProfile>(profiles);
            
            switch (settings.CuttingStrategy.YDirection)
            {
                case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                    sortedProfiles.Sort((p1, p2) => p1.MinY.CompareTo(p2.MinY));
                    System.Diagnostics.Debug.WriteLine("✅ Always Positive: Cắt theo thứ tự Y âm -> Y dương (1->2->3->4)");
                    break;
                    
                case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                    sortedProfiles.Sort((p1, p2) => p2.MinY.CompareTo(p1.MinY));
                    System.Diagnostics.Debug.WriteLine("✅ Always Negative: ĐẢO NGƯỢC TẤT CẢ profiles Y dương -> Y âm (4->3->2->1)");
                    break;
            }
            
            for (int i = 0; i < sortedProfiles.Count; i++)
            {
                var profile = sortedProfiles[i];
                System.Diagnostics.Debug.WriteLine($"   [{i+1}] Profile #{profile.ProfileId}: Y[{profile.MinY:F1}, {profile.MaxY:F1}]");
            }
            
            return sortedProfiles;
        }
        
        /// <summary>
        /// Apply Y Direction to individual profile cutting
        /// </summary>
        private void ApplyYDirectionToProfile(CompleteProfile profile)
        {
            switch (settings.CuttingStrategy.YDirection)
            {
                case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                    ReverseProfileDirection(profile);
                    System.Diagnostics.Debug.WriteLine($"🔄 Profile #{profile.ProfileId}: Reversed for AlwaysNegative");
                    break;
                    
                case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                default:
                    System.Diagnostics.Debug.WriteLine($"➡️ Profile #{profile.ProfileId}: Natural direction for AlwaysPositive");
                    break;
            }
        }
        
        /// <summary>
        /// Reverse profile cutting direction
        /// </summary>
        private void ReverseProfileDirection(CompleteProfile profile)
        {
            profile.Edges.Reverse();
            
            foreach (var edge in profile.Edges)
            {
                edge.Points.Reverse();
            }
        }

        /// <summary>
        /// Process complete profile với UNROLL CONSISTENCY
        /// </summary>
        private void ProcessCompleteProfileWithUnrollConsistency(CompleteProfile profile)
        {
            if (profile.Edges.Count == 0)
                return;

            var optimalStartPoint = ChooseOptimalStartPointForProfile(profile);

            // Move to chosen start point
            double normalizedC = NormalizeToUnrollRange(optimalStartPoint.C);
            // THÊM: Kiểm tra xem có cần di chuyển xa không
            double yDistance = Math.Abs(currentY - optimalStartPoint.Y);
            double cDistance = Math.Abs(currentC - normalizedC);

            // Nếu di chuyển xa, nâng Z trước
            if (yDistance > 10.0 || cDistance > 30.0) // Thresholds có thể điều chỉnh
            {
                // Ensure Z is up before long rapid move
                if (Math.Abs(currentZ) < settings.SafeZ - 0.1)
                {
                    gcode.AppendLine($"G0 Z{settings.SafeZ}");
                    currentZ = settings.SafeZ;
                }
            }

            // THÊM: Debug log
            System.Diagnostics.Debug.WriteLine($"🎯 Moving to profile start: Y={optimalStartPoint.Y:F3}, C={normalizedC:F3}");

            // SỬA: Di chuyển trực tiếp đến vị trí start point thực tế thay vì Y=0
            gcode.AppendLine($"G0 Y{optimalStartPoint.Y:F3} C{normalizedC:F3}");
            gcode.AppendLine("G0 Z0");

            // Update current position tracking
            currentC = normalizedC;
            currentY = optimalStartPoint.Y;
            currentZ = 0;

            // SINGLE PIERCE for entire profile
            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");

            // Cut ALL edges với C-axis unroll consistency
            bool isFirstEdge = true;
            foreach (var edge in profile.Edges)
            {
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
                    // Lines and bsplines
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

            gcode.AppendLine($"M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
        }

        /// <summary>
        /// Choose optimal start point for profile based on Y Direction preference
        /// </summary>
        private UnrolledPoint ChooseOptimalStartPointForProfile(CompleteProfile profile)
        {
            var allPoints = profile.Edges.SelectMany(e => e.Points).ToList();
            if (allPoints.Count == 0) return null;

            var firstPoint = allPoints.First();
            var lastPoint = allPoints.Last();

            // OPEN profile: chọn endpoint phù hợp với Y Direction preference
            if (!profile.IsClosed)
            {
                switch (settings.CuttingStrategy.YDirection)
                {
                    case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                        // Start từ Y nhỏ nhất (bottom)
                        var result = firstPoint.Y <= lastPoint.Y ? firstPoint : lastPoint;
                        System.Diagnostics.Debug.WriteLine($"✅ Open Profile AlwaysPositive: Start Y={result.Y:F3}");
                        return result;
                        
                    case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                        // Start từ Y lớn nhất (top)
                        var result2 = firstPoint.Y >= lastPoint.Y ? firstPoint : lastPoint;
                        System.Diagnostics.Debug.WriteLine($"✅ Open Profile AlwaysNegative: Start Y={result2.Y:F3}");
                        return result2;
                        
                    default:
                        return firstPoint;
                }
            }

            // CLOSED profile: chọn start point theo Y Direction preference
            UnrolledPoint optimalPoint = null;
            
            switch (settings.CuttingStrategy.YDirection)
            {
                case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                    // Start từ điểm có Y nhỏ nhất (bottom of cylinder)
                    optimalPoint = allPoints.OrderBy(p => p.Y).First();
                    System.Diagnostics.Debug.WriteLine($"✅ Closed Profile AlwaysPositive: Start Y={optimalPoint.Y:F3} (bottom)");
                    break;
                    
                case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                    // Start từ điểm có Y lớn nhất (top of cylinder)
                    optimalPoint = allPoints.OrderByDescending(p => p.Y).First();
                    System.Diagnostics.Debug.WriteLine($"✅ Closed Profile AlwaysNegative: Start Y={optimalPoint.Y:F3} (top)");
                    break;
                    
                default:
                    // Fallback: gần 0°/360° nhất
                    double firstDistanceTo0 = Math.Min(firstPoint.C, 360 - firstPoint.C);
                    double lastDistanceTo0 = Math.Min(lastPoint.C, 360 - lastPoint.C);
                    optimalPoint = firstDistanceTo0 <= lastDistanceTo0 ? firstPoint : lastPoint;
                    break;
            }
            
            // Reorder profile để start từ optimal point nếu cần
            ReorderProfileToStartFromPoint(profile, optimalPoint);
            
            return profile.Edges[0].Points[0]; // Return first point after reorder
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
        }

        /// <summary>
        /// Reorder profile to start from specific point
        /// </summary>
        private void ReorderProfileToStartFromPoint(CompleteProfile profile, UnrolledPoint targetPoint)
        {
            if (!profile.IsClosed || profile.Edges.Count == 0)
                return;
                
            // Find edge and point index that contains target point
            int targetEdgeIndex = -1;
            int targetPointIndex = -1;
            
            for (int edgeIdx = 0; edgeIdx < profile.Edges.Count; edgeIdx++)
            {
                var edge = profile.Edges[edgeIdx];
                for (int pointIdx = 0; pointIdx < edge.Points.Count; pointIdx++)
                {
                    var point = edge.Points[pointIdx];
                    if (Math.Abs(point.Y - targetPoint.Y) < 0.1 && Math.Abs(point.C - targetPoint.C) < 1.0)
                    {
                        targetEdgeIndex = edgeIdx;
                        targetPointIndex = pointIdx;
                        break;
                    }
                }
                if (targetEdgeIndex >= 0) break;
            }
            
            if (targetEdgeIndex < 0)
            {
                System.Diagnostics.Debug.WriteLine($"⚠️ Target point Y={targetPoint.Y:F3}, C={targetPoint.C:F3} not found in profile");
                return;
            }
            
            System.Diagnostics.Debug.WriteLine($"🎯 Reordering profile to start from Edge#{targetEdgeIndex}, Point#{targetPointIndex}");
            
            // Split and reorder edges
            var newEdges = new List<UnrolledToolpath>();
            
            // Add edges from target edge to end
            for (int i = targetEdgeIndex; i < profile.Edges.Count; i++)
            {
                newEdges.Add(profile.Edges[i]);
            }
            
            // Add edges from start to target edge
            for (int i = 0; i < targetEdgeIndex; i++)
            {
                newEdges.Add(profile.Edges[i]);
            }
            
            profile.Edges = newEdges;
        }

        /// <summary>
        /// Normalize C value to unroll range [0, 360°]
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
            // Always normalize to [0, 360)
            double normalizedCurrent = NormalizeToUnrollRange(currentC);
            double normalizedTarget = NormalizeToUnrollRange(targetC);
            
            // Calculate shortest path movement
            double directMove = normalizedTarget - normalizedCurrent;
            double wrapMove = directMove > 0 ? directMove - 360 : directMove + 360;
            
            // Choose shortest path but keep result in [0, 360)
            double chosenMove = Math.Abs(directMove) <= Math.Abs(wrapMove) ? directMove : wrapMove;
            double resultC = normalizedCurrent + chosenMove;
            
            // Final normalization to [0, 360)
            resultC = NormalizeToUnrollRange(resultC);
            
            return resultC;
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
            var points = toolpath.Points;
            var firstPoint = points[0];
            
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine("G0 Z0");
            
            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");
            
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

        protected virtual void AddHeader(CylinderData cylinderInfo)
        {
            gcode.AppendLine($"{settings.FileHeader} by T2N Industries");
            gcode.AppendLine($"(Generated: {DateTime.Now})");
            gcode.AppendLine($"(Cylinder: R={cylinderInfo.Radius:F2}mm, L={cylinderInfo.Length:F2}mm)");
            gcode.AppendLine($"( Program: {settings.ProgramName})");
            gcode.AppendLine();
        }

        protected virtual void AddInitialization()
        {
            gcode.AppendLine("G21");
            gcode.AppendLine("G90");
            gcode.AppendLine($"F{settings.FeedRate}");
            gcode.AppendLine("M3 S0");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
            gcode.AppendLine();
        }

        protected virtual void AddFooter()
        {
            gcode.AppendLine("M5");
            gcode.AppendLine("G0 Z50");
            gcode.AppendLine("M30");
        }

        /// <summary>
        /// Transform unroll coordinates to machine coordinates
        /// Machine coordinate system: Y=0 is start position, all cutting moves go to Y negative
        /// </summary>
        private List<CompleteProfile> TransformUnrollToMachineCoordinates(List<CompleteProfile> profiles)
        {
            System.Diagnostics.Debug.WriteLine("🔧 COORDINATE TRANSFORMATION: Unroll → Machine");
            
            // Step 1: Find Y bounds of all unrolled geometry
            double yMax = double.MinValue;
            double yMin = double.MaxValue;
            
            foreach (var profile in profiles)
            {
                foreach (var edge in profile.Edges)
                {
                    foreach (var point in edge.Points)
                    {
                        yMax = Math.Max(yMax, point.Y);
                        yMin = Math.Min(yMin, point.Y);
                    }
                }
            }
            
            System.Diagnostics.Debug.WriteLine($"Unroll Y bounds: [{yMin:F2}, {yMax:F2}]mm");
            
            // Step 2: Calculate offset to map Y_max → Y=0 (machine start position)
            double yOffset = yMax;
            System.Diagnostics.Debug.WriteLine($"Y offset applied: {yOffset:F2}mm (Y_max becomes Y=0)");
            
            // Step 3: Transform all points
            foreach (var profile in profiles)
            {
                foreach (var edge in profile.Edges)
                {
                    for (int i = 0; i < edge.Points.Count; i++)
                    {
                        var point = edge.Points[i];
                        point.Y = point.Y - yOffset;  // Transform: Y_machine = Y_unroll - Y_max
                        edge.Points[i] = point;
                    }
                }
                
                // Update profile bounds
                profile.MinY = profile.MinY - yOffset;
                profile.MaxY = profile.MaxY - yOffset;
            }
            
            // Step 4: Verify transformation
            double newYMax = double.MinValue;
            double newYMin = double.MaxValue;
            
            foreach (var profile in profiles)
            {
                newYMax = Math.Max(newYMax, profile.MaxY);
                newYMin = Math.Min(newYMin, profile.MinY);
            }
            
            System.Diagnostics.Debug.WriteLine($"✅ Machine Y bounds: [{newYMin:F2}, {newYMax:F2}]mm");
            System.Diagnostics.Debug.WriteLine($"✅ Y=0 at machine start position, all cuts go to Y negative");
            
            if (newYMax > 0.1)
            {
                System.Diagnostics.Debug.WriteLine($"⚠️ WARNING: Some Y coordinates > 0 after transformation!");
            }
            
            return profiles;
        }

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
