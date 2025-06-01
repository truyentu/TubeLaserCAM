// TubeLaserCAM.UI/Models/UnrollingModels.cs
using System;
using System.Collections.Generic;

namespace TubeLaserCAM.UI.Models
{
    public class UnrolledPoint
    {
        public double Y { get; set; }  // Position along cylinder axis
        public double C { get; set; }  // Rotation angle (degrees)
        public double X { get; set; }  // Radial depth
    }

    public class UnrollingSettings
    {
        public double ChordTolerance { get; set; } = 0.1;
        public double AngleTolerance { get; set; } = 5.0;
        public int MinPoints { get; set; } = 10;
        public int MaxPoints { get; set; } = 1000;
        public bool UnwrapAngles { get; set; } = true;
    }

    public class UnrolledToolpath
    {
        public int EdgeId { get; set; }
        public List<UnrolledPoint> Points { get; set; }
        public EdgeInfoWrapper EdgeInfo { get; set; }
        public bool RequiresSeamHandling { get; set; }
        public double MinY { get; set; }
        public double MaxY { get; set; }
        public double TotalRotation { get; set; }
    }
}