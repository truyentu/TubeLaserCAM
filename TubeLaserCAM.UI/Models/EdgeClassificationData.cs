namespace TubeLaserCAM.UI.Models
{
    public enum EdgeLocation
    {
        OnCylinderSurface,
        OnEndFace,
        Internal,
        Unknown
    }

    public enum EdgeShapeType
    {
        Line,
        Circle,
        Ellipse,
        Parabola,
        Hyperbola,
        BSpline,
        Bezier,
        Other
    }

    public class EdgeClassificationData
    {
        public EdgeLocation Location { get; set; }
        public EdgeShapeType ShapeType { get; set; }
        public bool IsOnCylinderSurface { get; set; }
        public int OriginalEdgeId { get; set; }

        public EdgeClassificationData()
        {
            Location = EdgeLocation.Unknown;
            ShapeType = EdgeShapeType.Other;
            IsOnCylinderSurface = false;
            OriginalEdgeId = -1;
        }
    }
}
