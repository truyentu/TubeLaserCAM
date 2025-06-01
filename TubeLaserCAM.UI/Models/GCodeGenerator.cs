// TubeLaserCAM.UI/Models/GCodeGenerator.cs
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
    }

    public class GCodeGenerator
    {
        private GCodeSettings settings;
        private StringBuilder gcode;
        private List<UnrolledToolpath> currentToolpaths;

        public GCodeGenerator(GCodeSettings settings)
        {
            this.settings = settings;
            gcode = new StringBuilder();
        }

        public string GenerateGCode(List<UnrolledToolpath> toolpaths, CylinderData cylinderInfo)
        {
            gcode.Clear();

            // Header
            AddHeader(cylinderInfo);
            currentToolpaths = toolpaths;

            // Initial setup
            AddInitialization();

            // Process each toolpath
            foreach (var toolpath in toolpaths)
            {
                ProcessToolpath(toolpath);
            }

            // Footer
            AddFooter();

            return gcode.ToString();
        }

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

            // Move to start position
            var firstPoint = toolpath.Points[0];
            gcode.AppendLine($"G0 Y{firstPoint.Y:F3} C{firstPoint.C:F3} ; Rapid to start");
            gcode.AppendLine($"G0 Z0 ; Move to surface");

            // Pierce
            gcode.AppendLine($"M3 S{settings.LaserPower} ; Laser on");
            gcode.AppendLine($"G4 P{settings.PierceTime} ; Pierce delay");

            // Cut path
            for (int i = 1; i < toolpath.Points.Count; i++)
            {
                var point = toolpath.Points[i];

                // Handle C axis wrap-around
                double cValue = HandleCAxisWrap(
                    toolpath.Points[i - 1].C,
                    point.C
                );

                gcode.AppendLine($"G1 Y{point.Y:F3} C{cValue:F3}");
            }

            // Laser off and retract
            gcode.AppendLine("M5 ; Laser off");
            gcode.AppendLine($"G0 Z{settings.SafeZ} ; Retract");
            gcode.AppendLine();
        }

        private double HandleCAxisWrap(double prevC, double currentC)
        {
            // Handle 360° wrap-around
            double diff = currentC - prevC;

            if (Math.Abs(diff) > 180)
            {
                // Wrapped around
                if (diff > 0)
                    return currentC - 360; // Go negative
                else
                    return currentC + 360; // Go over 360
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