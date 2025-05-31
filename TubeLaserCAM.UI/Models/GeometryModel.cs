// TubeLaserCAM.UI/Models/GeometryModel.cs
using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using GeometryWrapper;
using System.Linq;
using HelixToolkit.Wpf;

namespace TubeLaserCAM.UI.Models
{
    public class CylinderData
    {
        public bool IsValid { get; set; }
        public double Radius { get; set; }
        public double Length { get; set; }
        public Vector3D Axis { get; set; }
        public System.Windows.Media.Media3D.Point3D Center { get; set; }

    }

    public class EdgeSelectionInfo
    {
        public int EdgeId { get; set; }
        public GeometryWrapper.ManagedEdgeInfo EdgeInfo { get; set; } // Sử dụng kiểu từ GeometryWrapper
        public List<System.Windows.Media.Media3D.Point3D> Points { get; set; } // Sử dụng kiểu của WPF
        public GeometryModel3D Model3D { get; set; } // System.Windows.Media.Media3D.GeometryModel3D
        public Material OriginalMaterial { get; set; } // System.Windows.Media.Media3D.Material
        public bool IsSelected { get; set; }
        public bool IsHovered { get; set; }
        public EdgeClassificationData Classification { get; set; } // New property
    }

    public class GeometryModel
    {
        private ManagedStepReader stepReader;
        private CylinderData cylinderInfo;
        private Dictionary<int, EdgeSelectionInfo> edgeSelectionMap;
        private Dictionary<int, EdgeClassificationData> edgeClassifications; // New field
        private List<int> selectedEdgeIds;

        public GeometryModel()
        {
            stepReader = new ManagedStepReader();
            edgeSelectionMap = new Dictionary<int, EdgeSelectionInfo>();
            edgeClassifications = new Dictionary<int, EdgeClassificationData>(); // Initialize
            selectedEdgeIds = new List<int>();
        }

        public bool LoadStepFile(string filePath)
        {
            return stepReader.LoadFile(filePath);
        }

        public int GetEdgeCount()
        {
            return stepReader.GetEdgeCount();
        }

        public List<ManagedEdgeInfo> GetEdgeInfo()
        {
            return stepReader.GetEdgeInfoList();
        }

        // Helper methods for enum conversion
        private EdgeLocation ConvertManagedLocation(GeometryWrapper.ManagedEdgeClassification.Location managedLocation)
        {
            switch (managedLocation)
            {
                case GeometryWrapper.ManagedEdgeClassification.Location.OnCylinderSurface:
                    return EdgeLocation.OnCylinderSurface;
                case GeometryWrapper.ManagedEdgeClassification.Location.OnEndFace:
                    return EdgeLocation.OnEndFace;
                case GeometryWrapper.ManagedEdgeClassification.Location.Internal:
                    return EdgeLocation.Internal;
                default:
                    return EdgeLocation.Unknown;
            }
        }

        private EdgeShapeType ConvertManagedShapeType(GeometryWrapper.ManagedEdgeClassification.ShapeType managedShapeType)
        {
            switch (managedShapeType)
            {
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Line:
                    return EdgeShapeType.Line;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Circle:
                    return EdgeShapeType.Circle;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Ellipse:
                    return EdgeShapeType.Ellipse;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Parabola:
                    return EdgeShapeType.Parabola;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Hyperbola:
                    return EdgeShapeType.Hyperbola;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.BSpline:
                    return EdgeShapeType.BSpline;
                case GeometryWrapper.ManagedEdgeClassification.ShapeType.Bezier:
                    return EdgeShapeType.Bezier;
                default:
                    return EdgeShapeType.Other;
            }
        }

        public void FetchAndStoreEdgeClassifications()
        {
            edgeClassifications.Clear();
            if (cylinderInfo == null || !cylinderInfo.IsValid || stepReader == null)
            {
                return;
            }

            var managedCylInfo = new GeometryWrapper.ManagedCylinderInfo
            {
                IsValid = cylinderInfo.IsValid,
                Radius = cylinderInfo.Radius,
                Length = cylinderInfo.Length,
                AxisX = cylinderInfo.Axis.X,
                AxisY = cylinderInfo.Axis.Y,
                AxisZ = cylinderInfo.Axis.Z,
                CenterX = cylinderInfo.Center.X,
                CenterY = cylinderInfo.Center.Y,
                CenterZ = cylinderInfo.Center.Z
            };

            var classificationsFromWrapper = stepReader.GetEdgeClassifications(managedCylInfo);

            if (classificationsFromWrapper != null)
            {
                foreach (var kvp in classificationsFromWrapper)
                {
                    var cSharpClassification = new EdgeClassificationData
                    {
                        OriginalEdgeId = kvp.Value.OriginalEdgeId,
                        IsOnCylinderSurface = kvp.Value.IsOnCylinderSurface,
                        Location = ConvertManagedLocation(kvp.Value.ClassificationLocation),
                        ShapeType = ConvertManagedShapeType(kvp.Value.TypeOfShape)
                    };
                    edgeClassifications[kvp.Key] = cSharpClassification;
                }
            }
        }


        public Model3DGroup CreateWireframeModel()
        {
            var modelGroup = new Model3DGroup();
            edgeSelectionMap.Clear();
            
            // Get cylinder info first, as it's needed for classification
            // GetCylinderInfo() already populates the this.cylinderInfo field
            if (this.cylinderInfo == null) GetCylinderInfo(); // Ensure cylinderInfo is populated

            // Fetch and store classifications if cylinder is valid
            if (this.cylinderInfo != null && this.cylinderInfo.IsValid)
            {
                FetchAndStoreEdgeClassifications();
            }


            // Lấy dữ liệu wireframe từ native code
            List<GeometryWrapper.Point3D> wrapperVertices = null;
            List<Tuple<int, int>> lineIndices = null;
            stepReader.GetWireframeData(ref wrapperVertices, ref lineIndices);

            if (wrapperVertices == null || lineIndices == null)
                return modelGroup;

            // Convert vertices
            var vertices = new List<System.Windows.Media.Media3D.Point3D>();
            foreach (var wrapperPoint in wrapperVertices)
            {
                vertices.Add(new System.Windows.Media.Media3D.Point3D(
                    wrapperPoint.X, wrapperPoint.Y, wrapperPoint.Z));
            }

            // Lấy edge info list
            var edgeInfoList = stepReader.GetEdgeInfoList();

            // Group line indices by edge (giả định các indices liên tiếp thuộc cùng edge)
            int edgeIndex = 0;
            // int indicesPerEdge = lineIndices.Count / edgeInfoList.Count; // This logic might be flawed if edgeInfoList is empty or counts don't match

            foreach (var edgeInfo in edgeInfoList)
            {
                if (edgeClassifications.ContainsKey(edgeInfo.Id))
                {
                    var classification = edgeClassifications[edgeInfo.Id];
                    // Bỏ qua edges INTERNAL
                    if (classification.Location == EdgeLocation.Internal)
                    {
                        continue;
                    }
                }
                var edgeGroup = new Model3DGroup();
                var edgeSelectionInfo = new EdgeSelectionInfo
                {
                    EdgeId = edgeInfo.Id,
                    EdgeInfo = edgeInfo,
                    Points = new List<System.Windows.Media.Media3D.Point3D>(), // Will be populated by GetEdgePoints
                    IsSelected = false,
                    IsHovered = false,
                    Classification = edgeClassifications.ContainsKey(edgeInfo.Id) ? edgeClassifications[edgeInfo.Id] : new EdgeClassificationData() // Assign classification
                };

                // Get ordered points for the edge from the StepReader
                var managedEdgePoints = stepReader.GetEdgePoints(edgeInfo.Id);
                if (managedEdgePoints != null)
                {
                    foreach (var mp in managedEdgePoints)
                    {
                        edgeSelectionInfo.Points.Add(new System.Windows.Media.Media3D.Point3D(mp.X, mp.Y, mp.Z));
                    }
                }

                // Default material
                var defaultMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Blue));
                edgeSelectionInfo.OriginalMaterial = defaultMaterial;

                // Tạo geometry cho edge này dựa trên edgeSelectionInfo.Points (đã được lấy từ GetEdgePoints)
                // Điều này đảm bảo hình học của mỗi edgeModel là độc lập và chính xác.
                if (edgeSelectionInfo.Points.Count >= 2)
                {
                    for (int i = 0; i < edgeSelectionInfo.Points.Count - 1; i++)
                    {
                        var p1 = edgeSelectionInfo.Points[i];
                        var p2 = edgeSelectionInfo.Points[i + 1];

                        // Tạo cylinder cho line segment (visual representation)
                        var segmentVisual = CreateCylinderBetweenPoints(p1, p2, 0.5); // Bán kính 0.5, có thể điều chỉnh
                        var segmentGeometryModel = new GeometryModel3D
                        {
                            Geometry = segmentVisual,
                            Material = defaultMaterial, // defaultMaterial được tạo mới cho mỗi edge
                            BackMaterial = defaultMaterial
                        };
                        edgeGroup.Children.Add(segmentGeometryModel); // Thêm segment vào group của cạnh hiện tại
                    }
                }
                
                // Nếu không có điểm nào hoặc chỉ có 1 điểm, edgeGroup sẽ rỗng.
                // CreateMeshFromGroup sẽ trả về một MeshGeometry3D rỗng, điều này là chấp nhận được.

                // Wrap edge group trong một GeometryModel3D để dễ thao tác
                var edgeModel = new GeometryModel3D
                {
                    Geometry = CreateMeshFromGroup(edgeGroup), // edgeGroup giờ chỉ chứa các segment của đúng cạnh này
                    Material = defaultMaterial,
                    BackMaterial = defaultMaterial
                    // No Tag here, GeometryModel3D does not have it.
                };

                edgeSelectionInfo.Model3D = edgeModel; // This Model3D is for material changes and hit-testing comparison
                edgeSelectionMap[edgeInfo.Id] = edgeSelectionInfo;

                modelGroup.Children.Add(edgeModel); // Add the GeometryModel3D directly
                edgeIndex++;
            }

            return modelGroup;
        }

        private MeshGeometry3D CreateCylinderBetweenPoints(
            System.Windows.Media.Media3D.Point3D point1,
            System.Windows.Media.Media3D.Point3D point2,
            double radius)
        {
            var mesh = new MeshGeometry3D();

            // Vector từ point1 đến point2
            var direction = point2 - point1;
            var length = direction.Length;
            direction.Normalize();

            // Tạo hệ tọa độ local
            var up = new Vector3D(0, 1, 0);
            if (Math.Abs(Vector3D.DotProduct(direction, up)) > 0.999)
                up = new Vector3D(1, 0, 0);

            var right = Vector3D.CrossProduct(direction, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, direction);
            up.Normalize();

            // Tạo vertices cho cylinder
            int segments = 6; // Số cạnh của cylinder
            for (int i = 0; i <= segments; i++)
            {
                double angle = 2 * Math.PI * i / segments;
                var offset = Math.Cos(angle) * radius * right + Math.Sin(angle) * radius * up;

                // Bottom circle
                mesh.Positions.Add(point1 + offset);
                // Top circle  
                mesh.Positions.Add(point2 + offset);
            }

            // Tạo triangles
            for (int i = 0; i < segments; i++)
            {
                int i0 = i * 2;
                int i1 = i * 2 + 1;
                int i2 = ((i + 1) % segments) * 2;
                int i3 = ((i + 1) % segments) * 2 + 1;

                // Side face
                mesh.TriangleIndices.Add(i0);
                mesh.TriangleIndices.Add(i2);
                mesh.TriangleIndices.Add(i1);

                mesh.TriangleIndices.Add(i2);
                mesh.TriangleIndices.Add(i3);
                mesh.TriangleIndices.Add(i1);
            }

            return mesh;
        }
        public CylinderData GetCylinderInfo()
        {
            var info = stepReader.DetectCylinder();
            cylinderInfo = new CylinderData
            {
                IsValid = info.IsValid,
                Radius = info.Radius,
                Length = info.Length,
                Axis = new Vector3D(info.AxisX, info.AxisY, info.AxisZ),
                Center = new System.Windows.Media.Media3D.Point3D(info.CenterX, info.CenterY, info.CenterZ)
            };
            return cylinderInfo;
        }
        public Model3DGroup CreateAxisVisualization()
        {
            var group = new Model3DGroup();
            if (cylinderInfo == null || !cylinderInfo.IsValid) return group;

            // Tạo arrow để show axis direction
            var axisLength = cylinderInfo.Length * 1.2;
            var startPoint = cylinderInfo.Center - (cylinderInfo.Axis * axisLength / 2);
            var endPoint = cylinderInfo.Center + (cylinderInfo.Axis * axisLength / 2);

            // Cylinder cho axis line
            var axisCylinder = CreateCylinderBetweenPoints(startPoint, endPoint, 2.0);
            var axisMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Red));

            group.Children.Add(new GeometryModel3D
            {
                Geometry = axisCylinder,
                Material = axisMaterial
            });

            // Arrow head
            var arrowLength = 10.0;
            var arrowRadius = 5.0;
            var arrowBase = endPoint - (cylinderInfo.Axis * arrowLength);
            var arrowMesh = CreateCone(arrowBase, endPoint, arrowRadius);

            group.Children.Add(new GeometryModel3D
            {
                Geometry = arrowMesh,
                Material = axisMaterial
            });

            return group;
        }

        private MeshGeometry3D CreateCone(System.Windows.Media.Media3D.Point3D baseCenter, System.Windows.Media.Media3D.Point3D tip, double radius)
        {
            var mesh = new MeshGeometry3D();
            var direction = tip - baseCenter;
            direction.Normalize();

            var up = new Vector3D(0, 1, 0);
            if (Math.Abs(Vector3D.DotProduct(direction, up)) > 0.999)
                up = new Vector3D(1, 0, 0);

            var right = Vector3D.CrossProduct(direction, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, direction);
            up.Normalize();

            // Tip vertex
            mesh.Positions.Add(tip);

            // Base vertices
            int segments = 12;
            for (int i = 0; i < segments; i++)
            {
                double angle = 2 * Math.PI * i / segments;
                var offset = Math.Cos(angle) * radius * right + Math.Sin(angle) * radius * up;
                mesh.Positions.Add(baseCenter + offset);
            }

            // Create triangles
            for (int i = 0; i < segments; i++)
            {
                int next = (i + 1) % segments;

                // Side triangle
                mesh.TriangleIndices.Add(0); // tip
                mesh.TriangleIndices.Add(i + 1);
                mesh.TriangleIndices.Add(next + 1);
            }

            return mesh;
        }
        private MeshGeometry3D CreateMeshFromGroup(Model3DGroup group)
        {
            var combinedMesh = new MeshGeometry3D();
            int positionOffset = 0;

            foreach (GeometryModel3D model in group.Children.OfType<GeometryModel3D>())
            {
                var mesh = model.Geometry as MeshGeometry3D;
                if (mesh != null)
                {
                    // Add positions
                    foreach (var position in mesh.Positions)
                    {
                        combinedMesh.Positions.Add(position);
                    }

                    // Add triangle indices với offset
                    foreach (var index in mesh.TriangleIndices)
                    {
                        combinedMesh.TriangleIndices.Add(index + positionOffset);
                    }

                    positionOffset += mesh.Positions.Count;
                }
            }

            return combinedMesh;
        }
        public void SetEdgeHovered(int edgeId, bool isHovered)
        {
            if (edgeSelectionMap.ContainsKey(edgeId))
            {
                var edgeInfo = edgeSelectionMap[edgeId];
                edgeInfo.IsHovered = isHovered;
                UpdateEdgeMaterial(edgeId);
            }
        }

        public void SetEdgeSelected(int edgeId, bool isSelected)
        {
            if (edgeSelectionMap.ContainsKey(edgeId))
            {
                var edgeInfo = edgeSelectionMap[edgeId];
                edgeInfo.IsSelected = isSelected;

                if (isSelected && !selectedEdgeIds.Contains(edgeId))
                    selectedEdgeIds.Add(edgeId);
                else if (!isSelected)
                    selectedEdgeIds.Remove(edgeId);

                UpdateEdgeMaterial(edgeId);
            }
        }

        private void UpdateEdgeMaterial(int edgeId)
        {
            if (!edgeSelectionMap.ContainsKey(edgeId)) return;

            var edgeSelection = edgeSelectionMap[edgeId]; // Renamed for clarity
            Material newMaterial;

            if (edgeSelection.IsSelected)
                newMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Green));
            else if (edgeSelection.IsHovered)
                newMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Yellow));
            else
                newMaterial = edgeSelection.OriginalMaterial;

            // edgeSelection.Model3D is the GeometryModel3D whose material needs to change.
            if (edgeSelection.Model3D != null)
            {
                edgeSelection.Model3D.Material = newMaterial;
                edgeSelection.Model3D.BackMaterial = newMaterial;
            }
        }

        public List<int> GetSelectedEdgeIds()
        {
            return new List<int>(selectedEdgeIds);
        }

        public EdgeSelectionInfo GetEdgeInfo(int edgeId)
        {
            return edgeSelectionMap.ContainsKey(edgeId) ? edgeSelectionMap[edgeId] : null;
        }
        public int? FindNearestEdge(System.Windows.Media.Media3D.Point3D rayOrigin, Vector3D rayDirection)
        {
            double minDistance = double.MaxValue;
            int? nearestEdgeId = null;

            foreach (var kvp in edgeSelectionMap)
            {
                var edgeInfo = kvp.Value;

                // Check ray intersection với bounding sphere của edge
                foreach (var point in edgeInfo.Points)
                {
                    // Tính khoảng cách từ ray đến point
                    var toPoint = point - rayOrigin;
                    var projectedLength = Vector3D.DotProduct(toPoint, rayDirection);

                    if (projectedLength < 0) continue; // Point behind ray origin

                    var projectedPoint = rayOrigin + rayDirection * projectedLength;
                    var distance = (point - projectedPoint).Length;

                    if (distance < 5.0 && distance < minDistance) // Threshold 5 units
                    {
                        minDistance = distance;
                        nearestEdgeId = kvp.Key;
                    }
                }
            }

            return nearestEdgeId;
        }
        public Dictionary<int, EdgeSelectionInfo> GetAllEdgeInfos()
        {
            return new Dictionary<int, EdgeSelectionInfo>(edgeSelectionMap);
        }
        public List<ToolpathSuggestion> GetToolpathSuggestions()
        {
            var suggestions = new List<ToolpathSuggestion>();

            if (stepReader == null) return suggestions;

            var candidates = stepReader.GetToolpathCandidates();
            if (candidates != null)
            {
                foreach (var candidate in candidates)
                {
                    var suggestion = new ToolpathSuggestion
                    {
                        EdgeIds = candidate.EdgeIds.ToList(),
                        Type = candidate.Type,
                        Priority = candidate.Priority,
                        TotalLength = candidate.TotalLength
                    };
                    suggestions.Add(suggestion);
                }
            }

            return suggestions;
        }

        // Thêm class
        public class ToolpathSuggestion
        {
            public List<int> EdgeIds { get; set; }
            public string Type { get; set; }
            public double Priority { get; set; }
            public double TotalLength { get; set; }
        }
        // Thêm class để nhận diện pattern
        public class ShapeRecognition
        {
            public enum ShapeType
            {
                Unknown,
                Circle,
                Rectangle,
                Square,
                Slot,
                Triangle,
                Polygon,
                FreeForm
            }

            public class RecognizedShape
            {
                public ShapeType Type { get; set; }
                public List<int> EdgeIds { get; set; }
                public double Confidence { get; set; }
                public Dictionary<string, double> Parameters { get; set; }
            }

            public static List<RecognizedShape> RecognizeShapes(
                Dictionary<int, EdgeSelectionInfo> edgeMap,
                CylinderData cylinderInfo)
            {
                var shapes = new List<RecognizedShape>();

                // Tìm các closed loops
                var closedLoops = FindClosedLoops(edgeMap);

                foreach (var loop in closedLoops)
                {
                    var shape = AnalyzeLoop(loop, edgeMap, cylinderInfo);
                    if (shape != null && shape.Confidence > 0.7)
                    {
                        shapes.Add(shape);
                    }
                }

                return shapes;
            }

            private static List<List<int>> FindClosedLoops(Dictionary<int, EdgeSelectionInfo> edgeMap)
            {
                var loops = new List<List<int>>();
                var visited = new HashSet<int>();

                foreach (var kvp in edgeMap)
                {
                    if (visited.Contains(kvp.Key)) continue;

                    var loop = new List<int>();
                    if (TraceLoop(kvp.Key, kvp.Key, edgeMap, visited, loop))
                    {
                        loops.Add(loop);
                    }
                }

                return loops;
            }

            private static bool TraceLoop(int startId, int currentId,
    Dictionary<int, EdgeSelectionInfo> edgeMap,
    HashSet<int> visited,
    List<int> loop)
            {
                if (visited.Contains(currentId))
                    return false;

                visited.Add(currentId);
                loop.Add(currentId);

                var currentEdge = edgeMap[currentId];
                if (currentEdge.Points == null || currentEdge.Points.Count < 2)
                    return false;

                // Tìm edge kế tiếp
                var endPoint = currentEdge.Points.Last();

                foreach (var kvp in edgeMap)
                {
                    if (kvp.Key == currentId || visited.Contains(kvp.Key))
                        continue;

                    var nextEdge = kvp.Value;
                    if (nextEdge.Points == null || nextEdge.Points.Count < 2)
                        continue;

                    var startPoint = nextEdge.Points.First();

                    // Check if endpoints connect (với tolerance)
                    if ((endPoint - startPoint).Length < 0.1)
                    {
                        if (kvp.Key == startId && loop.Count > 2)
                        {
                            // Found closed loop
                            return true;
                        }

                        if (TraceLoop(startId, kvp.Key, edgeMap, visited, loop))
                            return true;
                    }
                }

                // Backtrack
                loop.RemoveAt(loop.Count - 1);
                visited.Remove(currentId);
                return false;
            }

            private static RecognizedShape AnalyzeLoop(
                List<int> loop,
                Dictionary<int, EdgeSelectionInfo> edgeMap,
                CylinderData cylinderInfo)
            {
                if (loop.Count == 0) return null;

                var shape = new RecognizedShape
                {
                    EdgeIds = new List<int>(loop),
                    Parameters = new Dictionary<string, double>()
                };

                // Phân tích dựa trên số edge và loại edge
                var edgeTypes = loop.Select(id => edgeMap[id].EdgeInfo.Type).ToList();

                // Circle - 1 edge duy nhất là Circle
                if (loop.Count == 1 && edgeTypes[0] == ManagedEdgeInfo.EdgeType.Circle)
                {
                    shape.Type = ShapeType.Circle;
                    shape.Confidence = 1.0;

                    // Tính bán kính
                    var edge = edgeMap[loop[0]];
                    if (edge.Points.Count >= 3)
                    {
                        var center = CalculateCircleCenter(edge.Points);
                        var radius = (edge.Points[0] - center).Length;
                        shape.Parameters["Radius"] = radius;
                    }
                }
                // Rectangle/Square - 4 edges, all lines
                else if (loop.Count == 4 && edgeTypes.All(t => t == ManagedEdgeInfo.EdgeType.Line))
                {
                    var lengths = loop.Select(id => edgeMap[id].EdgeInfo.Length).OrderBy(l => l).ToList();

                    // Check if opposite sides are equal
                    if (Math.Abs(lengths[0] - lengths[1]) < 0.1 &&
                        Math.Abs(lengths[2] - lengths[3]) < 0.1)
                    {
                        if (Math.Abs(lengths[0] - lengths[2]) < 0.1)
                        {
                            shape.Type = ShapeType.Square;
                            shape.Parameters["Side"] = lengths[0];
                        }
                        else
                        {
                            shape.Type = ShapeType.Rectangle;
                            shape.Parameters["Width"] = lengths[0];
                            shape.Parameters["Height"] = lengths[2];
                        }
                        shape.Confidence = 0.95;
                    }
                }
                // Slot - 2 lines + 2 semicircles
                else if (loop.Count == 4 &&
                         edgeTypes.Count(t => t == ManagedEdgeInfo.EdgeType.Line) == 2 &&
                         edgeTypes.Count(t => t == ManagedEdgeInfo.EdgeType.Circle) == 2)
                {
                    shape.Type = ShapeType.Slot;
                    shape.Confidence = 0.9;
                }
                // Polygon
                else if (loop.Count >= 3 && edgeTypes.All(t => t == ManagedEdgeInfo.EdgeType.Line))
                {
                    shape.Type = ShapeType.Polygon;
                    shape.Parameters["Sides"] = loop.Count;
                    shape.Confidence = 0.85;
                }
                // Freeform
                else
                {
                    shape.Type = ShapeType.FreeForm;
                    shape.Confidence = 0.6;
                }

                return shape;
            }

            private static System.Windows.Media.Media3D.Point3D CalculateCircleCenter(List<System.Windows.Media.Media3D.Point3D> points)
            {
                // Simplified - use first 3 points
                if (points.Count < 3) return new System.Windows.Media.Media3D.Point3D(0, 0, 0);

                var p1 = points[0];
                var p2 = points[points.Count / 2];
                var p3 = points[points.Count - 1];

                // Calculate center using 3 points
                // (Implementation of circle center calculation)
                return new System.Windows.Media.Media3D.Point3D(
                    (p1.X + p2.X + p3.X) / 3,
                    (p1.Y + p2.Y + p3.Y) / 3,
                    (p1.Z + p2.Z + p3.Z) / 3
                );
            }
        }

    }
}
