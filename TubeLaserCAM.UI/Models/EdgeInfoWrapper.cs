using GeometryWrapper;

namespace TubeLaserCAM.UI.Models
{
    /// <summary>
    /// C# Wrapper for C++/CLI ManagedEdgeInfo to fix WPF binding issues
    /// </summary>
    public class EdgeInfoWrapper
    {
        public int Id { get; set; }
        public string Type { get; set; }
        public double Length { get; set; }
        public EdgeClassificationData Classification { get; set; }

        public EdgeInfoWrapper(ManagedEdgeInfo managedEdge)
        {
            Id = managedEdge.Id;
            Type = managedEdge.Type.ToString();
            Length = managedEdge.Length;
            // Classification will be set separately after creation, or initialized to a default
            Classification = new EdgeClassificationData(); 
        }

        // Optional: A constructor that takes classification data directly
        public EdgeInfoWrapper(ManagedEdgeInfo managedEdge, EdgeClassificationData classification)
        {
            Id = managedEdge.Id;
            Type = managedEdge.Type.ToString();
            Length = managedEdge.Length;
            Classification = classification;
        }
    }
}
