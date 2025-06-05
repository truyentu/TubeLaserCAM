using System.Windows.Media.Media3D;

namespace TubeLaserCAM.Models
{
    public class GCodeCommand3D
    {
        public Point3D TargetPosition { get; set; }
        public double Y { get; set; }
        public double C { get; set; }
        public double Z { get; set; }
        public double FeedRate { get; set; }
        public bool IsLaserOn { get; set; }
        public bool IsRapidMove { get; set; }
        public string OriginalLine { get; set; }
        public GCodeCommandType CommandType { get; set; }


        public GCodeCommand3D(double y, double c, double z, double feedRate, bool isLaserOn, bool isRapidMove, GCodeCommandType commandType, string originalLine = "")
        {
            Y = y;
            C = c;
            Z = z;
            FeedRate = feedRate;
            IsLaserOn = isLaserOn;
            IsRapidMove = isRapidMove;
            CommandType = commandType;
            OriginalLine = originalLine;
            TargetPosition = new Point3D(0, 0, 0);
        }
    }

    public enum GCodeCommandType
    {
        G00, // Rapid move
        G01, // Linear interpolation
        G02, // Circular interpolation CW
        G03, // Circular interpolation CCW
        M03, // Spindle on (Laser on)
        M04, // Spindle on CCW (Laser on, alternative)
        M05, // Spindle off (Laser off)
        M02, // Program end
        M30, // Program end and reset
        Other
    }
}