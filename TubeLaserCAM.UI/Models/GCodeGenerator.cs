// TubeLaserCAM.UI/Models/GCodeGenerator.cs
// THAY THẾ TOÀN BỘ FILE

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
    }

    public class GCodeGenerator
    {
        protected GCodeSettings settings;
        protected StringBuilder gcode;
        protected List<UnrolledToolpath> currentToolpaths;
        protected ProfileAnalyzer profileAnalyzer;

        // THÊM MỚI: Track current position
        protected double currentY = 0;
        protected double currentC = 0;

        public GCodeGenerator(GCodeSettings settings)
        {
            this.settings = settings;
            this.gcode = new StringBuilder();
            this.profileAnalyzer = new ProfileAnalyzer();
        }

        public virtual string GenerateGCode(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            gcode.Clear();

            // THAY ĐỔI: Sử dụng ProfileAnalyzer
            if (settings.OptimizeSequence && settings.CuttingStrategy.CompleteProfileBeforeMoving)
            {
                return GenerateOptimizedGCode(toolpaths, cylinderInfo);
            }
            else
            {
                // Legacy mode - giữ nguyên như cũ
                return GenerateLegacyGCode(toolpaths, cylinderInfo);
            }
        }

        /// <summary>
        /// Generate G-Code với profile optimization
        /// </summary>
        private string GenerateOptimizedGCode(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            System.Diagnostics.Debug.WriteLine("Generating optimized G-Code with ProfileAnalyzer");

            // Analyze profiles
            var profiles = profileAnalyzer.AnalyzeProfiles(toolpaths, settings.CuttingStrategy);

            System.Diagnostics.Debug.WriteLine($"Found {profiles.Count} profiles to cut");

            // Header
            AddHeader(cylinderInfo);
            AddInitialization();

            // Sort profiles by cutting order
            var sortedProfiles = profiles.OrderBy(p => p.CuttingOrder).ToList();

            // Process each profile
            foreach (var profile in sortedProfiles)
            {
                ProcessProfile(profile);
            }

            // Footer
            AddFooter();

            return gcode.ToString();
        }

        /// <summary>
        /// Process một profile hoàn chỉnh
        /// </summary>
        private void ProcessProfile(ProfileAnalyzer.CuttingProfile profile)
        {
            gcode.AppendLine();
            gcode.AppendLine($"; PROFILE #{profile.ProfileId}");
            gcode.AppendLine($"; Type: {profile.Type}, Closed: {profile.IsClosed}");
            gcode.AppendLine($"; Edges: {profile.Edges.Count}");

            if (profile.IsClosed)
            {
                ProcessClosedProfile(profile);
            }
            else
            {
                ProcessOpenProfile(profile);
            }
        }

        /// <summary>
        /// Process closed profile với lead-in/out
        /// </summary>
        private void ProcessClosedProfile(ProfileAnalyzer.CuttingProfile profile)
        {
            // Find optimal start point
            var startPoint = FindBestStartPoint(profile);

            // Reorder edges để cắt continuous từ start point
            var orderedEdges = ReorderEdgesForContinuousCut(profile, startPoint);

            if (orderedEdges.Count == 0) return;

            // Move to start position
            var firstToolpath = orderedEdges.First().Toolpath;
            var firstPoint = GetStartPointForDirection(firstToolpath, profile.PreferredDirection);

            gcode.AppendLine($"; Move to profile start");
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine($"G0 Z0 ; To surface");

            // Lead-in if enabled
            if (settings.UseLeadInOut)
            {
                AddLeadIn(firstPoint, profile);
            }

            // Pierce
            gcode.AppendLine($"M3 S{settings.LaserPower} ; Laser on");
            gcode.AppendLine($"G4 P{settings.PierceTime} ; Pierce");

            // Cut all edges in profile
            bool firstEdge = true;
            foreach (var edge in orderedEdges)
            {
                CutEdgeContinuous(edge.Toolpath, profile.PreferredDirection, firstEdge);
                firstEdge = false;
            }

            // Lead-out if enabled
            if (settings.UseLeadInOut && profile.IsClosed)
            {
                var lastPoint = GetLastCutPoint();
                AddLeadOut(lastPoint, profile);
            }

            // Laser off and retract
            gcode.AppendLine("M5 ; Laser off");
            gcode.AppendLine($"G0 Z{settings.SafeZ} ; Retract");
        }

        /// <summary>
        /// Process open profile
        /// </summary>
        private void ProcessOpenProfile(ProfileAnalyzer.CuttingProfile profile)
        {
            if (profile.Edges.Count == 0) return;

            // Determine start direction based on settings
            var shouldReverse = ShouldReverseOpenProfile(profile);
            var edges = shouldReverse ?
                profile.Edges.AsEnumerable().Reverse().ToList() :
                profile.Edges;

            // Move to start
            var firstToolpath = edges.First().Toolpath;
            var firstPoint = shouldReverse ?
                firstToolpath.Points.Last() :
                firstToolpath.Points.First();

            gcode.AppendLine($"; Open profile start");
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3}");
            gcode.AppendLine($"G0 Z0");

            // Pierce
            gcode.AppendLine($"M3 S{settings.LaserPower}");
            gcode.AppendLine($"G4 P{settings.PierceTime}");

            // Cut edges
            foreach (var edge in edges)
            {
                var points = shouldReverse ?
                    edge.Toolpath.Points.AsEnumerable().Reverse().ToList() :
                    edge.Toolpath.Points;

                foreach (var point in points.Skip(1))
                {
                    double cValue = HandleCAxisWrap(currentC, point.C);
                    gcode.AppendLine($"G1 Y{point.Y:F3} C{cValue:F3}");
                    currentY = point.Y;
                    currentC = cValue;
                }
            }

            // Laser off
            gcode.AppendLine("M5");
            gcode.AppendLine($"G0 Z{settings.SafeZ}");
        }

        /// <summary>
        /// Find best start point cho closed profile
        /// </summary>
        private UnrolledPoint FindBestStartPoint(ProfileAnalyzer.CuttingProfile profile)
        {
            // Use optimal start point từ analyzer
            if (profile.OptimalStartPoint != null)
            {
                // Convert từ Point3D về UnrolledPoint
                return new UnrolledPoint
                {
                    Y = profile.OptimalStartPoint.Y,
                    C = profile.OptimalStartPoint.X, // X trong 3D space = C
                    X = 0
                };
            }

            // Fallback: use first point
            return profile.Edges.First().Toolpath.Points.First();
        }

        /// <summary>
        /// Reorder edges để cắt continuous
        /// </summary>
        private List<ProfileAnalyzer.EdgeNode> ReorderEdgesForContinuousCut(
            ProfileAnalyzer.CuttingProfile profile,
            UnrolledPoint startPoint)
        {
            // TODO: Implement edge reordering algorithm
            // For now, return as-is
            return profile.Edges;
        }

        /// <summary>
        /// Cut edge với direction control
        /// </summary>
        private void CutEdgeContinuous(
            UnrolledToolpath toolpath,
            ProfileAnalyzer.CuttingDirection direction,
            bool skipFirst)
        {
            var points = toolpath.Points;

            // Reverse nếu cần
            if (NeedToReverseEdge(toolpath, direction))
            {
                points = points.AsEnumerable().Reverse().ToList();
            }

            // Skip first point nếu đã connected từ edge trước
            var pointsToCut = skipFirst ? points.Skip(1) : points;

            foreach (var point in pointsToCut)
            {
                double cValue = HandleCAxisWrap(currentC, point.C);
                gcode.AppendLine($"G1 Y{point.Y:F3} C{cValue:F3}");
                currentY = point.Y;
                currentC = cValue;
            }
        }

        /// <summary>
        /// Add lead-in motion
        /// </summary>
        private void AddLeadIn(UnrolledPoint startPoint, ProfileAnalyzer.CuttingProfile profile)
        {
            // Calculate lead-in point (offset from start)
            double leadInOffset = settings.LeadInLength;

            // Simple perpendicular lead-in
            gcode.AppendLine($"; Lead-in");
            gcode.AppendLine($"G1 Y{startPoint.Y - leadInOffset:F3} C{startPoint.C:F3}");
            gcode.AppendLine($"G1 Y{startPoint.Y:F3} C{startPoint.C:F3}");
        }

        /// <summary>
        /// Add lead-out motion
        /// </summary>
        private void AddLeadOut(UnrolledPoint endPoint, ProfileAnalyzer.CuttingProfile profile)
        {
            double leadOutOffset = settings.LeadOutLength;

            gcode.AppendLine($"; Lead-out");
            gcode.AppendLine($"G1 Y{endPoint.Y + leadOutOffset:F3} C{endPoint.C:F3}");
        }

        // ================ HELPER METHODS ================

        private bool ShouldReverseOpenProfile(ProfileAnalyzer.CuttingProfile profile)
        {
            switch (settings.CuttingStrategy.YDirection)
            {
                case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                    return profile.Edges.First().StartPoint.Y > profile.Edges.Last().EndPoint.Y;

                case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                    return profile.Edges.First().StartPoint.Y < profile.Edges.Last().EndPoint.Y;

                default:
                    return false;
            }
        }

        private bool NeedToReverseEdge(UnrolledToolpath toolpath, ProfileAnalyzer.CuttingDirection direction)
        {
            var firstY = toolpath.Points.First().Y;
            var lastY = toolpath.Points.Last().Y;

            switch (direction)
            {
                case ProfileAnalyzer.CuttingDirection.YPositive:
                    return firstY > lastY;

                case ProfileAnalyzer.CuttingDirection.YNegative:
                    return firstY < lastY;

                default:
                    return false;
            }
        }

        private UnrolledPoint GetStartPointForDirection(
            UnrolledToolpath toolpath,
            ProfileAnalyzer.CuttingDirection direction)
        {
            if (NeedToReverseEdge(toolpath, direction))
                return toolpath.Points.Last();
            else
                return toolpath.Points.First();
        }

        private UnrolledPoint GetLastCutPoint()
        {
            return new UnrolledPoint { Y = currentY, C = currentC, X = 0 };
        }

        // ================ LEGACY METHODS (giữ nguyên) ================

        private string GenerateLegacyGCode(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            AddHeader(cylinderInfo);
            currentToolpaths = toolpaths;
            AddInitialization();

            foreach (var toolpath in toolpaths)
            {
                ProcessToolpath(toolpath);
            }

            AddFooter();
            return gcode.ToString();
        }

        // ... (giữ nguyên các methods cũ: AddHeader, AddInitialization, ProcessToolpath, etc.)

        private void AddHeader(CylinderData cylinderInfo)
        {
            gcode.AppendLine($"; {settings.FileHeader}");
            gcode.AppendLine($"; Generated: {DateTime.Now}");
            gcode.AppendLine($"; Cylinder: R={cylinderInfo.Radius:F2}, L={cylinderInfo.Length:F2}");
            gcode.AppendLine($"; Toolpaths: {currentToolpaths?.Count ?? 0}");
            gcode.AppendLine();
        }

        private void AddInitialization()
        {
            gcode.AppendLine("G21 ; Metric units");
            gcode.AppendLine(settings.UseG91 ? "G91 ; Relative mode" : "G90 ; Absolute mode");
            gcode.AppendLine("G94 ; Feed rate mode");
            gcode.AppendLine($"F{settings.FeedRate} ; Set feed rate");
            gcode.AppendLine("M3 S0 ; Laser off");
            gcode.AppendLine($"G0 Z{settings.SafeZ} ; Safe Z");
            gcode.AppendLine();
        }

        private void ProcessToolpath(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 2) return;

            gcode.AppendLine($"; Edge #{toolpath.EdgeId} - {toolpath.EdgeInfo.Type}");

            var firstPoint = toolpath.Points[0];
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3} ; Rapid to start");
            gcode.AppendLine($"G0 Z0 ; Move to surface");

            gcode.AppendLine($"M3 S{settings.LaserPower} ; Laser on");
            gcode.AppendLine($"G4 P{settings.PierceTime} ; Pierce delay");

            for (int i = 1; i < toolpath.Points.Count; i++)
            {
                var point = toolpath.Points[i];
                double cValue = HandleCAxisWrap(toolpath.Points[i - 1].C, point.C);
                gcode.AppendLine($"G1 Y{point.Y:F3} C{cValue:F3}");
            }

            gcode.AppendLine("M5 ; Laser off");
            gcode.AppendLine($"G0 Z{settings.SafeZ} ; Retract");
            gcode.AppendLine();
        }

        private double HandleCAxisWrap(double prevC, double currentC)
        {
            double diff = currentC - prevC;

            if (Math.Abs(diff) > 180)
            {
                if (diff > 0)
                    return currentC - 360;
                else
                    return currentC + 360;
            }

            return currentC;
        }

        private void AddFooter()
        {
            gcode.AppendLine("; Program end");
            gcode.AppendLine("M5 ; Ensure laser off");
            gcode.AppendLine("G0 Z50 ; Final retract");
            gcode.AppendLine("M30 ; Program end");
        }
    }
}