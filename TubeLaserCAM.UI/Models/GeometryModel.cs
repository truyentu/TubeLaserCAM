using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using GeometryWrapper;
using System.Linq;
using HelixToolkit.Wpf;
using static TubeLaserCAM.UI.Models.ProfileInfo;
using System.Threading.Tasks;

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
        public GeometryWrapper.ManagedEdgeInfo EdgeInfo { get; set; }
        public List<System.Windows.Media.Media3D.Point3D> Points { get; set; }
        public GeometryModel3D Model3D { get; set; }
        public Material OriginalMaterial { get; set; }
        public bool IsSelected { get; set; }
        public bool IsHovered { get; set; }
        public EdgeClassificationData Classification { get; set; }
    }

    public class GeometryModel
    {
        private ManagedStepReader stepReader;
        private CylinderData cylinderInfo;
        private Dictionary<int, EdgeSelectionInfo> edgeSelectionMap;
        private Dictionary<int, EdgeClassificationData> edgeClassifications;
        private List<int> selectedEdgeIds;
        private Dictionary<int, ProfileInfo> profileCache = new Dictionary<int, ProfileInfo>();
        private bool profileCacheValid = false;
        private Transform3D cylinderToYTransform = Transform3D.Identity;
        private readonly Dictionary<string, Material> materialCache = new Dictionary<string, Material>();
        private readonly object materialCacheLock = new object();
        private MeshGeometry3D cachedCylinderMesh;
        private Material cachedCylinderMaterial;
        private bool cylinderMeshDirty = true;




        // Thêm property cho rendering mode
        public enum RenderMode { Wireframe, SolidWithCuts }
        public RenderMode CurrentRenderMode { get; set; } = RenderMode.Wireframe;

        public GeometryModel()
        {
            stepReader = new ManagedStepReader();
            edgeSelectionMap = new Dictionary<int, EdgeSelectionInfo>();
            edgeClassifications = new Dictionary<int, EdgeClassificationData>();
            selectedEdgeIds = new List<int>();
        }

        public bool LoadStepFile(string filePath)
        {
            var result = stepReader.LoadFile(filePath);

            if (result)
            {
                // Clear all caches when loading new file
                ClearAllCaches();

                // Optionally build profile cache immediately
                // BuildProfileCache(); // Uncomment if you want to pre-build cache
            }

            return result;
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

        private Material GetCachedMaterial(string key, Color color)
        {
            lock (materialCacheLock)
            {
                if (!materialCache.ContainsKey(key))
                {
                    materialCache[key] = new DiffuseMaterial(new SolidColorBrush(color));
                }
                return materialCache[key];
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

        public Model3DGroup CreateVisualization()
        {
            return CurrentRenderMode == RenderMode.SolidWithCuts
                ? CreateSolidCylinderWithCuts()
                : CreateWireframeModel();
        }

        public Model3DGroup CreateSolidCylinderWithCuts()
        {
            var modelGroup = new Model3DGroup();

            if (cylinderInfo == null || !cylinderInfo.IsValid)
            {
                System.Diagnostics.Debug.WriteLine("CreateSolidCylinderWithCuts: Invalid cylinder info");
                return CreateWireframeModel();
            }

            // Ensure edgeSelectionMap is populated
            if (edgeSelectionMap.Count == 0)
            {
                CreateWireframeModel();
            }

            // 1. Lấy thông tin cylinder GỐC (trước transform)
            var originalInfo = stepReader.DetectMainCylinder();

            System.Diagnostics.Debug.WriteLine($"=== CYLINDER INFO ===");
            System.Diagnostics.Debug.WriteLine($"Original Radius: {originalInfo.Radius:F2} mm");
            System.Diagnostics.Debug.WriteLine($"Original Length: {originalInfo.Length:F2} mm");

            // 2. Tạo hoặc reuse cached cylinder mesh
            if (cachedCylinderMesh == null || cylinderMeshDirty)
            {
                var center = new System.Windows.Media.Media3D.Point3D(originalInfo.CenterX, originalInfo.CenterY, originalInfo.CenterZ);
                var axis = new Vector3D(originalInfo.AxisX, originalInfo.AxisY, originalInfo.AxisZ);
                axis.Normalize();

                // Sử dụng custom method để có control hoàn toàn
                cachedCylinderMesh = CreateCustomCylinderMesh(center, axis, originalInfo.Radius, originalInfo.Length, 24);

                var bounds = cachedCylinderMesh.Bounds;
                System.Diagnostics.Debug.WriteLine($"Mesh bounds - Size: ({bounds.SizeX:F2}, {bounds.SizeY:F2}, {bounds.SizeZ:F2})");

                cachedCylinderMesh.Freeze();
                cylinderMeshDirty = false;
            }

            // 3. Material cache
            if (cachedCylinderMaterial == null)
            {
                var brush = new SolidColorBrush(Color.FromArgb(255, 100, 150, 200));
                brush.Freeze();
                cachedCylinderMaterial = new DiffuseMaterial(brush);
                cachedCylinderMaterial.Freeze();
            }

            // 4. Tạo GeometryModel3D
            var cylinderModel = new GeometryModel3D
            {
                Geometry = cachedCylinderMesh,
                Material = cachedCylinderMaterial,
                BackMaterial = cachedCylinderMaterial
            };

            modelGroup.Children.Add(cylinderModel);

            // 5. Vẽ edges với MeshBuilder (batched)
            AddBatchedEdges(modelGroup);

            // 6. APPLY TRANSFORM cho toàn bộ group
            if (cylinderToYTransform != Transform3D.Identity)
            {
                var transformedGroup = new Model3DGroup();
                transformedGroup.Transform = cylinderToYTransform;

                while (modelGroup.Children.Count > 0)
                {
                    var child = modelGroup.Children[0];
                    modelGroup.Children.RemoveAt(0);
                    transformedGroup.Children.Add(child);
                }

                return transformedGroup;
            }

            return modelGroup;
        }

        private MeshGeometry3D CreateCustomCylinderMesh(System.Windows.Media.Media3D.Point3D center, Vector3D axis, double radius, double length, int segments)
        {
            var mesh = new MeshGeometry3D();
            axis.Normalize();

            // Create coordinate system
            var up = Math.Abs(axis.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0);
            var right = Vector3D.CrossProduct(axis, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, axis);
            up.Normalize();

            var start = center - axis * (length / 2);
            var end = center + axis * (length / 2);

            // Generate vertices cho cylinder
            for (int i = 0; i <= segments; i++)
            {
                double angle = 2 * Math.PI * i / segments;
                double cos = Math.Cos(angle);
                double sin = Math.Sin(angle);
                var offset = right * (cos * radius) + up * (sin * radius);

                mesh.Positions.Add(start + offset);
                mesh.Positions.Add(end + offset);
            }

            // Generate triangles
            for (int i = 0; i < segments; i++)
            {
                int current = i * 2;
                int next = (i + 1) * 2;

                // First triangle
                mesh.TriangleIndices.Add(current);
                mesh.TriangleIndices.Add(current + 1);
                mesh.TriangleIndices.Add(next);

                // Second triangle
                mesh.TriangleIndices.Add(next);
                mesh.TriangleIndices.Add(current + 1);
                mesh.TriangleIndices.Add(next + 1);
            }

            return mesh;
        }


        private void AddBatchedEdges(Model3DGroup modelGroup)
        {
            // Tạo 2 mesh builders: một cho edges thường, một cho selected
            var normalEdgeBuilder = new MeshBuilder(false, false);
            var selectedEdgeBuilder = new MeshBuilder(false, false);

            int edgeCount = 0;
            int maxEdgesToRender = 100;

            // Lấy thông tin cylinder gốc để tính toán
            var originalInfo = stepReader.DetectMainCylinder();
            var cylinderRadius = originalInfo.Radius;
            var cylinderCenter = new System.Windows.Media.Media3D.Point3D(originalInfo.CenterX, originalInfo.CenterY, originalInfo.CenterZ);
            var cylinderAxis = new Vector3D(originalInfo.AxisX, originalInfo.AxisY, originalInfo.AxisZ);
            cylinderAxis.Normalize();

            // Lấy inverse transform
            var inverseTransform = cylinderToYTransform.Inverse;

            foreach (var kvp in edgeSelectionMap)
            {
                if (edgeCount >= maxEdgesToRender) break;

                var edgeInfo = kvp.Value;

                // Skip internal edges
                if (edgeInfo.Classification.Location == EdgeLocation.Internal)
                    continue;

                if (edgeInfo.Points.Count >= 2)
                {
                    edgeCount++;

                    // Simplify points
                    var simplifiedPoints = SimplifyPoints(edgeInfo.Points, 35);

                    // Transform ngược và project lên bề mặt cylinder
                    var projectedPoints = new Point3DCollection();

                    foreach (var p in simplifiedPoints)
                    {
                        // Transform ngược về tọa độ gốc
                        var originalPoint = inverseTransform.Transform(p);

                        // Project point lên bề mặt cylinder nếu edge nằm trên surface
                        if (edgeInfo.Classification.Location == EdgeLocation.OnCylinderSurface)
                        {
                            originalPoint = ProjectPointToCylinderSurface(
                                originalPoint,
                                cylinderCenter,
                                cylinderAxis,
                                cylinderRadius,
                                0.1  // KHÔNG OFFSET - edges nằm đúng trên bề mặt
                            );
                        }

                        projectedPoints.Add(originalPoint);
                    }

                    // KÍCH THƯỚC EDGES
                    double normalEdgeDiameter = 0.3;    // 0.3mm diameter
                    double selectedEdgeDiameter = 0.5;  // 0.5mm diameter

                    // Add tube với diameter nhỏ và ít segments
                    if (edgeInfo.IsSelected)
                    {
                        selectedEdgeBuilder.AddTube(projectedPoints, selectedEdgeDiameter, 4, false);
                    }
                    else
                    {
                        normalEdgeBuilder.AddTube(projectedPoints, normalEdgeDiameter, 4, false);
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"Total edges rendered: {edgeCount}");

            // Tạo model cho normal edges
            if (normalEdgeBuilder.Positions.Count > 0)
            {
                var normalMesh = normalEdgeBuilder.ToMesh();
                normalMesh.Freeze();

                var normalMaterial = GetCachedMaterial("Edges", Colors.DarkBlue);

                var normalModel = new GeometryModel3D
                {
                    Geometry = normalMesh,
                    Material = normalMaterial,
                    BackMaterial = normalMaterial
                };

                modelGroup.Children.Add(normalModel);
            }

            // Tạo model cho selected edges  
            if (selectedEdgeBuilder.Positions.Count > 0)
            {
                var selectedMesh = selectedEdgeBuilder.ToMesh();
                selectedMesh.Freeze();

                var selectedMaterial = GetCachedMaterial("Selected", Colors.Lime);

                var selectedModel = new GeometryModel3D
                {
                    Geometry = selectedMesh,
                    Material = selectedMaterial,
                    BackMaterial = selectedMaterial
                };

                modelGroup.Children.Add(selectedModel);
            }
        }


        private System.Windows.Media.Media3D.Point3D ProjectPointToCylinderSurface(
    System.Windows.Media.Media3D.Point3D point,
    System.Windows.Media.Media3D.Point3D cylinderCenter,
    Vector3D cylinderAxis,
    double cylinderRadius,
    double offsetDistance = 0.0)  // Default offset = 0
        {
            // Vector từ cylinder center đến point
            var toPoint = point - cylinderCenter;

            // Project vector lên cylinder axis để tìm điểm gần nhất trên axis
            var axisProjection = Vector3D.DotProduct(toPoint, cylinderAxis);
            var pointOnAxis = cylinderCenter + cylinderAxis * axisProjection;

            // Vector từ axis đến point (radial direction)
            var radialVector = point - pointOnAxis;
            var currentRadius = radialVector.Length;

            if (currentRadius < 0.001) // Point nằm trên axis
            {
                // Tạo một vector perpendicular bất kỳ
                var perp = Math.Abs(cylinderAxis.X) < 0.9 ? new Vector3D(1, 0, 0) : new Vector3D(0, 1, 0);
                radialVector = Vector3D.CrossProduct(cylinderAxis, perp);
            }

            radialVector.Normalize();

            // Point mới trên bề mặt cylinder
            var projectedRadius = cylinderRadius + offsetDistance;

            return pointOnAxis + radialVector * projectedRadius;
        }
        private List<System.Windows.Media.Media3D.Point3D> SimplifyPoints(List<System.Windows.Media.Media3D.Point3D> points, int maxPoints)
        {
            if (points.Count <= maxPoints)
                return points;

            var simplified = new List<System.Windows.Media.Media3D.Point3D>();

            // Luôn giữ điểm đầu
            simplified.Add(points[0]);

            // Lấy mẫu đều và smooth
            double step = (double)(points.Count - 1) / (maxPoints - 1);

            for (int i = 1; i < maxPoints - 1; i++)
            {
                int index = (int)(i * step);

                // Thêm smoothing bằng cách lấy trung bình với điểm lân cận
                if (index > 0 && index < points.Count - 1)
                {
                    var prev = points[index - 1];
                    var curr = points[index];
                    var next = points[index + 1];

                    // Weighted average cho smoothing
                    var smoothedPoint = new System.Windows.Media.Media3D.Point3D(
                        curr.X * 0.5 + prev.X * 0.25 + next.X * 0.25,
                        curr.Y * 0.5 + prev.Y * 0.25 + next.Y * 0.25,
                        curr.Z * 0.5 + prev.Z * 0.25 + next.Z * 0.25
                    );

                    simplified.Add(smoothedPoint);
                }
                else
                {
                    simplified.Add(points[index]);
                }
            }

            // Luôn giữ điểm cuối
            simplified.Add(points[points.Count - 1]);

            return simplified;
        }

        // Method để invalidate cache





        private Model3DGroup CreateOptimizedWireframeCuts()
        {
            var group = new Model3DGroup();
            var edgeMaterial = GetCachedMaterial("WireframeCuts", Colors.DarkBlue);

            int edgeCount = 0;
            int maxEdgesToRender = 100; // Limit number of edges to render

            foreach (var kvp in edgeSelectionMap)
            {
                if (edgeCount >= maxEdgesToRender) break;

                var edgeInfo = kvp.Value;

                if (edgeInfo.Classification.Location == EdgeLocation.Internal)
                    continue;

                if (edgeInfo.Points.Count >= 2)
                {
                    edgeCount++;

                    // Use simpler geometry for edges
                    var edgeMesh = CreateSimpleEdgeMesh(edgeInfo.Points, 0.3);

                    if (edgeMesh != null && edgeMesh.Positions.Count > 0)
                    {
                        var edgeModel = new GeometryModel3D
                        {
                            Geometry = edgeMesh,
                            Material = edgeMaterial,
                            BackMaterial = edgeMaterial
                        };

                        group.Children.Add(edgeModel);
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"CreateOptimizedWireframeCuts: Created {edgeCount} edges");

            return group;
        }

        private MeshGeometry3D CreateSimpleEdgeMesh(List<System.Windows.Media.Media3D.Point3D> points, double radius)
        {
            if (points.Count < 2) return null;

            var mesh = new MeshGeometry3D();
            int segments = 4; // Very low segment count for edges

            // Create simple tube along points
            for (int i = 0; i < points.Count - 1; i++)
            {
                AddCylinderSegment(mesh, points[i], points[i + 1], radius, segments);
            }

            return mesh;
        }

        private GeometryModel3D CombineModelsIntoSingleMesh(Model3DGroup group)
        {
            var combinedMesh = new MeshGeometry3D();
            var positions = new Point3DCollection();
            var indices = new Int32Collection();
            int vertexOffset = 0;

            foreach (GeometryModel3D model in group.Children.OfType<GeometryModel3D>())
            {
                var mesh = model.Geometry as MeshGeometry3D;
                if (mesh != null)
                {
                    // Add positions
                    foreach (var pos in mesh.Positions)
                    {
                        positions.Add(pos);
                    }

                    // Add indices with offset
                    foreach (var idx in mesh.TriangleIndices)
                    {
                        indices.Add(idx + vertexOffset);
                    }

                    vertexOffset += mesh.Positions.Count;
                }
            }

            if (positions.Count == 0) return null;

            combinedMesh.Positions = positions;
            combinedMesh.TriangleIndices = indices;

            if (combinedMesh.CanFreeze)
            {
                combinedMesh.Freeze();
            }

            var material = GetCachedMaterial("WireframeCuts", Colors.DarkBlue);

            return new GeometryModel3D
            {
                Geometry = combinedMesh,
                Material = material,
                BackMaterial = material
            };
        }

        private Model3DGroup CreateOptimizedSelectedEdgeHighlights()
        {
            var group = new Model3DGroup();
            var selectedMaterial = GetCachedMaterial("Selected", Colors.Lime);

            foreach (var edgeId in selectedEdgeIds)
            {
                if (edgeSelectionMap.ContainsKey(edgeId))
                {
                    var edgeInfo = edgeSelectionMap[edgeId];

                    if (edgeInfo.Points.Count >= 2)
                    {
                        // Use simple line instead of dashed for performance
                        var edgeMesh = CreateSimpleEdgeMesh(edgeInfo.Points, 0.5);

                        if (edgeMesh != null)
                        {
                            var model = new GeometryModel3D
                            {
                                Geometry = edgeMesh,
                                Material = selectedMaterial,
                                BackMaterial = selectedMaterial
                            };

                            group.Children.Add(model);
                        }
                    }
                }
            }

            return group;
        }

        public void InvalidateCylinderCache()
        {
            cylinderMeshDirty = true;
            cachedCylinderMesh = null;
            cachedCylinderMaterial = null;
        }

        

        private async Task<Model3DGroup> CreateSolidCylinderWithCutsAsync()
        {
            return await Task.Run(() =>
            {
                var modelGroup = CreateSolidCylinderWithCuts();

                // Freeze entire group
                if (modelGroup.CanFreeze)
                {
                    modelGroup.Freeze();
                }

                return modelGroup;
            });
        }
        private Material GetOptimizedCylinderMaterial()
        {
            // Dùng DiffuseMaterial thay vì SpecularMaterial
            var brush = new SolidColorBrush(Color.FromArgb(255, 200, 200, 200));
            brush.Freeze(); // Freeze brush

            var material = new DiffuseMaterial(brush);
            material.Freeze(); // Freeze material

            return material;
        }

        private int GetOptimalSegmentCount(double cameraDistance)
        {
            // Adjust segments based on camera distance
            if (cameraDistance > 1000) return 8;   // Far: low detail
            if (cameraDistance > 500) return 12;   // Medium distance
            if (cameraDistance > 200) return 16;   // Close
            return 24;                              // Very close: high detail
        }
        private Model3DGroup CreateWireframeCuts()
        {
            var group = new Model3DGroup();
            var edgeMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.DarkBlue));

            System.Diagnostics.Debug.WriteLine($"CreateWireframeCuts: Total edges = {edgeSelectionMap.Count}");

            // Lấy thông tin cylinder GỐC
            var originalInfo = stepReader.DetectMainCylinder();

            int edgeCount = 0;
            foreach (var kvp in edgeSelectionMap)
            {
                var edgeInfo = kvp.Value;

                if (edgeInfo.Classification.Location == EdgeLocation.Internal)
                    continue;

                if (edgeInfo.Points.Count >= 2)
                {
                    edgeCount++;

                    // QUAN TRỌNG: Tạo points GỐC bằng cách ĐẢO NGƯỢC transform
                    var inverseTransform = cylinderToYTransform.Inverse;

                    for (int i = 0; i < edgeInfo.Points.Count - 1; i++)
                    {
                        // Apply inverse transform để có points gốc
                        var p1 = inverseTransform.Transform(edgeInfo.Points[i]);
                        var p2 = inverseTransform.Transform(edgeInfo.Points[i + 1]);

                        var segment = CreateCylinderBetweenPoints(p1, p2, 0.4);

                        group.Children.Add(new GeometryModel3D
                        {
                            Geometry = segment,
                            Material = edgeMaterial,
                            BackMaterial = edgeMaterial
                        });
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine($"CreateWireframeCuts: Created {edgeCount} edges");

            return group;
        }

        private Model3DGroup CreateSelectedEdgeHighlights()
        {
            var group = new Model3DGroup();
            var selectedMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Lime));

            // Lấy inverse transform
            var inverseTransform = cylinderToYTransform.Inverse;

            foreach (var kvp in edgeSelectionMap)
            {
                var edgeInfo = kvp.Value;

                if (!edgeInfo.IsSelected || edgeInfo.Classification.Location == EdgeLocation.Internal)
                    continue;

                if (edgeInfo.Points.Count >= 2)
                {
                    // Transform ngược lại points
                    var originalPoints = new List<System.Windows.Media.Media3D.Point3D>();
                    foreach (var p in edgeInfo.Points)
                    {
                        originalPoints.Add(inverseTransform.Transform(p));
                    }

                    DrawDashedEdge(group, originalPoints, selectedMaterial, 0.6);
                }
            }

            return group;
        }

        private MeshGeometry3D CreateCylinderMesh(System.Windows.Media.Media3D.Point3D center, Vector3D axis, double radius, double length, int segments)
        {
            var mesh = new MeshGeometry3D();
            axis.Normalize();

            // Pre-calculate to avoid repeated calculations
            var angleStep = 2 * Math.PI / segments;
            var cosAngles = new double[segments + 1];
            var sinAngles = new double[segments + 1];

            for (int i = 0; i <= segments; i++)
            {
                double angle = angleStep * i;
                cosAngles[i] = Math.Cos(angle);
                sinAngles[i] = Math.Sin(angle);
            }

            // Create coordinate system once
            var up = Math.Abs(axis.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0);
            var right = Vector3D.CrossProduct(axis, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, axis);

            var start = center - axis * length / 2;
            var end = center + axis * length / 2;

            // Add vertices using pre-calculated values
            for (int i = 0; i <= segments; i++)
            {
                var offset = cosAngles[i] * radius * right + sinAngles[i] * radius * up;
                mesh.Positions.Add(start + offset);
                mesh.Positions.Add(end + offset);
            }

            // Add triangles
            for (int i = 0; i < segments; i++)
            {
                int i0 = i * 2;
                int i1 = i * 2 + 1;
                int i2 = ((i + 1) % segments) * 2;
                int i3 = ((i + 1) % segments) * 2 + 1;

                mesh.TriangleIndices.Add(i0);
                mesh.TriangleIndices.Add(i2);
                mesh.TriangleIndices.Add(i1);

                mesh.TriangleIndices.Add(i2);
                mesh.TriangleIndices.Add(i3);
                mesh.TriangleIndices.Add(i1);
            }

            return mesh;
        }

        private void DrawDashedEdge(Model3DGroup group, List<System.Windows.Media.Media3D.Point3D> points, Material material, double thickness)
        {
            // Cài đặt dash pattern
            double dashLength = 5.0;  // Chiều dài mỗi nét
            double gapLength = 3.0;   // Khoảng cách giữa các nét

            double currentDistance = 0;
            bool isDash = true;

            for (int i = 0; i < points.Count - 1; i++)
            {
                System.Windows.Media.Media3D.Point3D p1 = points[i];
                System.Windows.Media.Media3D.Point3D p2 = points[i + 1];
                Vector3D direction = p2 - p1;
                double segmentLength = direction.Length;
                direction.Normalize();

                double segmentProgress = 0;

                while (segmentProgress < segmentLength)
                {
                    double remainingInSegment = segmentLength - segmentProgress;
                    double currentPatternLength = isDash ? dashLength : gapLength;
                    double actualLength = Math.Min(currentPatternLength, remainingInSegment);

                    System.Windows.Media.Media3D.Point3D dashStart = p1 + direction * segmentProgress;
                    System.Windows.Media.Media3D.Point3D dashEnd = dashStart + direction * actualLength;

                    if (isDash)
                    {
                        // Vẽ dash
                        var dashCylinder = CreateCylinderBetweenPoints(dashStart, dashEnd, thickness);

                        group.Children.Add(new GeometryModel3D
                        {
                            Geometry = dashCylinder,
                            Material = material,
                            BackMaterial = material
                        });
                    }

                    segmentProgress += actualLength;
                    currentDistance += actualLength;

                    // Chuyển dash/gap
                    if (actualLength >= currentPatternLength - 0.001)
                    {
                        isDash = !isDash;
                    }
                }
            }
        }

        private Model3DGroup CreateDashedToolpaths()
        {
            var group = new Model3DGroup();
            var dashMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.DarkGreen));

            foreach (var kvp in edgeSelectionMap)
            {
                if (kvp.Value.IsSelected && kvp.Value.Points.Count >= 2)
                {
                    // Tạo dashed line bằng cách chia nhỏ thành segments
                    for (int i = 0; i < kvp.Value.Points.Count - 1; i += 2)
                    {
                        if (i + 1 < kvp.Value.Points.Count)
                        {
                            var dash = CreateCylinderBetweenPoints(
                                kvp.Value.Points[i],
                                kvp.Value.Points[i + 1],
                                0.3 // Thinner for dashed effect
                            );

                            group.Children.Add(new GeometryModel3D
                            {
                                Geometry = dash,
                                Material = dashMaterial
                            });
                        }
                    }
                }
            }

            return group;
        }

        private void CreateCutoutVisuals(Model3DGroup group)
        {
            var cutMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(255, 50, 50, 50)));

            foreach (var kvp in edgeSelectionMap)
            {
                if (!kvp.Value.IsSelected) continue;

                var edgeInfo = kvp.Value;

                // Tạo visual cho cuts dựa trên loại edge
                if (edgeInfo.EdgeInfo.Type == ManagedEdgeInfo.EdgeType.Circle)
                {
                    // Tạo lỗ tròn
                    var cutMesh = CreateHoleCutout(edgeInfo.Points);
                    if (cutMesh != null)
                    {
                        group.Children.Add(new GeometryModel3D
                        {
                            Geometry = cutMesh,
                            Material = cutMaterial
                        });
                    }
                }
                else
                {
                    // Tạo rãnh cho các loại edge khác
                    var cutMesh = CreateSlotCutout(edgeInfo.Points, 2.0); // 2mm width
                    if (cutMesh != null)
                    {
                        group.Children.Add(new GeometryModel3D
                        {
                            Geometry = cutMesh,
                            Material = cutMaterial
                        });
                    }
                }
            }
        }

        private MeshGeometry3D CreateHoleCutout(List<System.Windows.Media.Media3D.Point3D> circlePoints)
        {
            if (circlePoints.Count < 3) return null;

            var mesh = new MeshGeometry3D();

            // Tính center và radius
            var center = new System.Windows.Media.Media3D.Point3D(
                circlePoints.Average(p => p.X),
                circlePoints.Average(p => p.Y),
                circlePoints.Average(p => p.Z)
            );

            var radius = circlePoints[0].DistanceTo(center);

            // Tạo disc mesh
            mesh.Positions.Add(center);

            for (int i = 0; i < circlePoints.Count; i++)
            {
                mesh.Positions.Add(circlePoints[i]);

                if (i > 0)
                {
                    mesh.TriangleIndices.Add(0);
                    mesh.TriangleIndices.Add(i);
                    mesh.TriangleIndices.Add(i + 1 < circlePoints.Count ? i + 1 : 1);
                }
            }

            return mesh;
        }

        private MeshGeometry3D CreateSlotCutout(List<System.Windows.Media.Media3D.Point3D> points, double width)
        {
            if (points.Count < 2) return null;

            var mesh = new MeshGeometry3D();

            // Tạo ribbon mesh dọc theo path
            for (int i = 0; i < points.Count - 1; i++)
            {
                var p1 = points[i];
                var p2 = points[i + 1];
                var dir = p2 - p1;
                dir.Normalize();

                // Perpendicular vector
                var perp = Vector3D.CrossProduct(dir, cylinderInfo.Axis);
                if (perp.Length < 0.001)
                    perp = Vector3D.CrossProduct(dir, new Vector3D(1, 0, 0));
                perp.Normalize();
                perp *= width / 2;

                // Add quad vertices
                int baseIndex = mesh.Positions.Count;
                mesh.Positions.Add(p1 + perp);
                mesh.Positions.Add(p1 - perp);
                mesh.Positions.Add(p2 + perp);
                mesh.Positions.Add(p2 - perp);

                // Add triangles
                mesh.TriangleIndices.Add(baseIndex);
                mesh.TriangleIndices.Add(baseIndex + 2);
                mesh.TriangleIndices.Add(baseIndex + 1);

                mesh.TriangleIndices.Add(baseIndex + 1);
                mesh.TriangleIndices.Add(baseIndex + 2);
                mesh.TriangleIndices.Add(baseIndex + 3);
            }

            return mesh;
        }

        public Model3DGroup CreateWireframeModel()
        {
            var modelGroup = new Model3DGroup();
            edgeSelectionMap.Clear();

            System.Diagnostics.Debug.WriteLine($"Starting CreateWireframeModel - Optimized Version");

            if (this.cylinderInfo == null) GetCylinderInfo();

            // Fetch and store classifications if cylinder is valid
            if (this.cylinderInfo != null && this.cylinderInfo.IsValid)
            {
                FetchAndStoreEdgeClassifications();
            }

            // Calculate transform
            Transform3D cylinderTransform = CalculateCylinderTransform();
            this.cylinderToYTransform = cylinderTransform;

            // Get edge info list
            var edgeInfoList = stepReader.GetEdgeInfoList();

            // Create individual model for each edge (for hit testing)
            foreach (var edgeInfo in edgeInfoList)
            {
                // Skip internal edges
                if (edgeClassifications.ContainsKey(edgeInfo.Id))
                {
                    var classification = edgeClassifications[edgeInfo.Id];
                    if (classification.Location == EdgeLocation.Internal)
                    {
                        continue;
                    }
                }

                // Get edge points
                var managedEdgePoints = stepReader.GetEdgePoints(edgeInfo.Id);
                if (managedEdgePoints == null || managedEdgePoints.Count < 2) continue;

                var edgeSelectionInfo = new EdgeSelectionInfo
                {
                    EdgeId = edgeInfo.Id,
                    EdgeInfo = edgeInfo,
                    Points = new List<System.Windows.Media.Media3D.Point3D>(),
                    IsSelected = false,
                    IsHovered = false,
                    Classification = edgeClassifications.ContainsKey(edgeInfo.Id) ?
                        edgeClassifications[edgeInfo.Id] : new EdgeClassificationData()
                };

                // Transform points
                foreach (var mp in managedEdgePoints)
                {
                    var point = new System.Windows.Media.Media3D.Point3D(mp.X, mp.Y, mp.Z);
                    point = cylinderTransform.Transform(point);
                    edgeSelectionInfo.Points.Add(point);
                }

                // Create mesh for this edge
                var edgeMesh = CreateEdgeMesh(edgeSelectionInfo.Points, 0.5);

                // Get material from cache
                var material = GetCachedMaterial($"Default_{edgeInfo.Type}", Colors.Blue);

                var edgeModel = new GeometryModel3D
                {
                    Geometry = edgeMesh,
                    Material = material,
                    BackMaterial = material
                };

                edgeSelectionInfo.Model3D = edgeModel;
                edgeSelectionInfo.OriginalMaterial = material;
                edgeSelectionMap[edgeInfo.Id] = edgeSelectionInfo;

                modelGroup.Children.Add(edgeModel);
            }

            return modelGroup;
        }

        private MeshGeometry3D CreateEdgeMesh(List<System.Windows.Media.Media3D.Point3D> points, double radius)
        {
            var mesh = new MeshGeometry3D();

            if (points.Count < 2) return mesh;

            // Use lower segment count for performance
            int segments = 4; // Reduced from 6

            for (int i = 0; i < points.Count - 1; i++)
            {
                var p1 = points[i];
                var p2 = points[i + 1];

                // Add cylinder segment
                AddCylinderSegment(mesh, p1, p2, radius, segments);
            }

            return mesh;
        }

        private void AddCylinderSegment(MeshGeometry3D mesh, System.Windows.Media.Media3D.Point3D p1, System.Windows.Media.Media3D.Point3D p2, double radius, int segments)
        {
            var direction = p2 - p1;
            var length = direction.Length;
            if (length < 0.001) return;

            direction.Normalize();

            // Create perpendicular vectors
            var up = Math.Abs(direction.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0);
            var right = Vector3D.CrossProduct(direction, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, direction);
            up.Normalize();

            int baseIndex = mesh.Positions.Count;

            // Add vertices
            for (int i = 0; i <= segments; i++)
            {
                double angle = 2 * Math.PI * i / segments;
                var offset = Math.Cos(angle) * radius * right + Math.Sin(angle) * radius * up;

                mesh.Positions.Add(p1 + offset);
                mesh.Positions.Add(p2 + offset);
            }

            // Add triangles
            for (int i = 0; i < segments; i++)
            {
                int idx = baseIndex + i * 2;
                int nextIdx = baseIndex + ((i + 1) % segments) * 2;

                // First triangle
                mesh.TriangleIndices.Add(idx);
                mesh.TriangleIndices.Add(nextIdx);
                mesh.TriangleIndices.Add(idx + 1);

                // Second triangle
                mesh.TriangleIndices.Add(nextIdx);
                mesh.TriangleIndices.Add(nextIdx + 1);
                mesh.TriangleIndices.Add(idx + 1);
            }
        }




        private GeometryModel3D CreateBatchedEdgeGeometry(List<EdgeSelectionInfo> edges, string edgeType)
        {
            var combinedMesh = new MeshGeometry3D();
            int vertexOffset = 0;

            foreach (var edgeInfo in edges)
            {
                // OPTIMIZATION: Use tube mesh builder từ HelixToolkit
                var tubeBuilder = new HelixToolkit.Wpf.MeshBuilder(false, false);

                // Create path từ edge points
                var path = new Point3DCollection(edgeInfo.Points);

                // Add tube với diameter nhỏ và ít segments
                tubeBuilder.AddTube(path, 0.3, 4, false); // 4 segments thay vì 6

                // Merge vào combined mesh
                var tubeMesh = tubeBuilder.ToMesh();
                foreach (var pos in tubeMesh.Positions)
                {
                    combinedMesh.Positions.Add(pos);
                }

                foreach (var idx in tubeMesh.TriangleIndices)
                {
                    combinedMesh.TriangleIndices.Add(idx + vertexOffset);
                }

                vertexOffset = combinedMesh.Positions.Count;

                // Store reference cho selection
                edgeInfo.Model3D = new GeometryModel3D { Geometry = tubeMesh };
            }

            // Create material based on edge type
            var material = GetMaterialForEdgeType(edgeType);

            return new GeometryModel3D
            {
                Geometry = combinedMesh,
                Material = material,
                BackMaterial = material
            };
        }

        private Transform3D CalculateCylinderTransform()
        {
            Transform3D cylinderTransform = Transform3D.Identity;

            if (this.cylinderInfo != null && this.cylinderInfo.IsValid)
            {
                Vector3D currentAxis = this.cylinderInfo.Axis;
                currentAxis.Normalize();
                Vector3D targetAxis = new Vector3D(0, 1, 0); // Y axis

                double dotProduct = Vector3D.DotProduct(currentAxis, targetAxis);

                if (Math.Abs(dotProduct) < 0.999)
                {
                    Vector3D rotationAxis = Vector3D.CrossProduct(currentAxis, targetAxis);
                    if (rotationAxis.Length > 0.001)
                    {
                        rotationAxis.Normalize();
                        double angle = Math.Acos(Math.Max(-1, Math.Min(1, dotProduct))) * 180 / Math.PI;

                        cylinderTransform = new RotateTransform3D(
                            new AxisAngleRotation3D(rotationAxis, angle),
                            this.cylinderInfo.Center
                        );
                    }
                }
            }

            return cylinderTransform;
        }

        private Material GetMaterialForEdgeType(string edgeType)
        {
            switch (edgeType)
            {
                case "Line":
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Blue));
                case "Circle":
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Red));
                case "BSpline":
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Green));
                default:
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Gray));
            }
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
            var info = stepReader.DetectMainCylinder();

            cylinderInfo = new CylinderData
            {
                IsValid = info.IsValid,
                Radius = info.Radius,
                Length = info.Length,
                Axis = new Vector3D(info.AxisX, info.AxisY, info.AxisZ),
                Center = new System.Windows.Media.Media3D.Point3D(info.CenterX, info.CenterY, info.CenterZ)
            };
            System.Diagnostics.Debug.WriteLine($"Original Cylinder Axis: ({info.AxisX:F2}, {info.AxisY:F2}, {info.AxisZ:F2})");
            // Nếu axis là Z, không cần transform
            if (Math.Abs(info.AxisZ - 1.0) < 0.01)
            {
                System.Diagnostics.Debug.WriteLine("Cylinder already aligned with Z axis, no transform needed");
                cylinderToYTransform = Transform3D.Identity;
            }

            // QUAN TRỌNG: Transform center theo cùng cách
            if (cylinderInfo.IsValid && cylinderToYTransform != Transform3D.Identity)
            {
                cylinderInfo.Center = cylinderToYTransform.Transform(cylinderInfo.Center);
                // Axis sau transform luôn là Y
                cylinderInfo.Axis = new Vector3D(0, 1, 0);
            }

            return cylinderInfo;
        }

        public Model3DGroup CreateAxisVisualization()
        {
            var group = new Model3DGroup();
            if (cylinderInfo == null || !cylinderInfo.IsValid) return group;

            var axisLength = cylinderInfo.Length * 1.2; // Dài hơn cylinder một chút

            // Tạo cylinder cho axis thay vì LineVisual3D
            var axisCylinder = CreateCylinderBetweenPoints(
                new System.Windows.Media.Media3D.Point3D(
                    cylinderInfo.Center.X,
                    cylinderInfo.Center.Y - (axisLength / 2),
                    cylinderInfo.Center.Z
                ),
                new System.Windows.Media.Media3D.Point3D(
                    cylinderInfo.Center.X,
                    cylinderInfo.Center.Y + (axisLength / 2),
                    cylinderInfo.Center.Z
                ),
                0.2  // Radius của axis line
            );

            // Material màu đỏ cho axis
            var axisMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Red));

            var axisModel = new GeometryModel3D
            {
                Geometry = axisCylinder,
                Material = axisMaterial,
                BackMaterial = axisMaterial
            };

            group.Children.Add(axisModel);

            // Thêm mũi tên ở đầu axis (optional)
            var arrowTip = new System.Windows.Media.Media3D.Point3D(
                cylinderInfo.Center.X,
                cylinderInfo.Center.Y + (axisLength / 2) + 10,
                cylinderInfo.Center.Z
            );

            var arrowBase = new System.Windows.Media.Media3D.Point3D(
                cylinderInfo.Center.X,
                cylinderInfo.Center.Y + (axisLength / 2),
                cylinderInfo.Center.Z
            );

            var arrowCone = CreateCone(arrowBase, arrowTip, 5.0);

            var arrowModel = new GeometryModel3D
            {
                Geometry = arrowCone,
                Material = axisMaterial,
                BackMaterial = axisMaterial
            };

            group.Children.Add(arrowModel);

            // Apply transform nếu cần
            if (cylinderToYTransform != Transform3D.Identity)
            {
                group.Transform = cylinderToYTransform;
            }

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

            var edgeSelection = edgeSelectionMap[edgeId];
            Material newMaterial;

            // Use cached materials
            if (edgeSelection.IsSelected)
            {
                newMaterial = GetCachedMaterial("Selected", Colors.Lime);
            }
            else if (edgeSelection.IsHovered)
            {
                newMaterial = GetCachedMaterial("Hovered", Colors.Yellow);
            }
            else
            {
                newMaterial = edgeSelection.OriginalMaterial;
            }

            if (edgeSelection.Model3D != null && edgeSelection.Model3D.Material != newMaterial)
            {
                edgeSelection.Model3D.Material = newMaterial;
                edgeSelection.Model3D.BackMaterial = newMaterial;
            }
        }

        private string GetMaterialKey(EdgeSelectionInfo edgeSelection)
        {
            if (edgeSelection.IsSelected) return "Selected";
            if (edgeSelection.IsHovered) return "Hovered_" + edgeSelection.EdgeInfo.Type;
            return "Default_" + edgeSelection.EdgeInfo.Type;
        }

        private Material CreateMaterialForState(EdgeSelectionInfo edgeSelection)
        {
            if (edgeSelection.IsSelected)
                return new DiffuseMaterial(new SolidColorBrush(Colors.Green));

            if (edgeSelection.IsHovered)
            {
                var profile = GetProfileContainingEdge(edgeSelection.EdgeId);
                if (profile != null && profile.IsClosed && profile.OrderedEdgeIds.Count > 1)
                {
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Orange));
                }
                else
                {
                    return new DiffuseMaterial(new SolidColorBrush(Colors.Yellow));
                }
            }

            return GetMaterialForEdgeType(edgeSelection.EdgeInfo.Type.ToString());
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

        public ProfileInfo GetProfileContainingEdge(int edgeId)
        {
            try
            {
                // 1. Check cache first
                if (profileCacheValid)
                {
                    foreach (var profile in profileCache.Values)
                    {
                        if (profile.ContainsEdge(edgeId))
                        {
                            System.Diagnostics.Debug.WriteLine($"Found edge {edgeId} in cached profile {profile.ProfileId}");
                            return profile;
                        }
                    }
                    // Not in any profile
                    return null;
                }

                // 2. If cache invalid, detect profile for this edge
                if (stepReader == null)
                {
                    System.Diagnostics.Debug.WriteLine("StepReader is null");
                    return null;
                }

                var managedProfile = stepReader.DetectCompleteProfile(edgeId);

                // 3. Check if valid profile found
                if (!managedProfile.IsValid || managedProfile.OrderedEdgeIds == null ||
                    managedProfile.OrderedEdgeIds.Length == 0)
                {
                    System.Diagnostics.Debug.WriteLine($"No valid profile found for edge {edgeId}");
                    return null;
                }

                // 4. Convert ManagedProfileInfo to ProfileInfo with gap information
                var detectedProfile = new ProfileInfo
                {
                    ProfileId = edgeId, // Use starting edge as profile ID
                    OrderedEdgeIds = new List<int>(managedProfile.OrderedEdgeIds),
                    IsClosed = managedProfile.IsClosed,
                    ProfileType = managedProfile.ProfileType,
                    TotalLength = managedProfile.TotalLength,

                    // THÊM: Gap-related properties
                    TotalGapLength = managedProfile.TotalGapLength,
                    ProfileConfidence = managedProfile.ProfileConfidence,
                    HasVirtualEdges = managedProfile.HasVirtualEdges
                };

                // Convert gaps if present
                if (managedProfile.Gaps != null && managedProfile.Gaps.Length > 0)
                {
                    foreach (var gap in managedProfile.Gaps)
                    {
                        detectedProfile.Gaps.Add(new GapInfo
                        {
                            FromEdgeId = gap.FromEdgeId,
                            ToEdgeId = gap.ToEdgeId,
                            GapDistance = gap.GapDistance,
                            SuggestedMethod = gap.SuggestedMethod,
                            Confidence = gap.Confidence
                        });
                    }
                }

                // 5. Update cache for all edges in this profile
                foreach (int id in detectedProfile.OrderedEdgeIds)
                {
                    profileCache[id] = detectedProfile;
                }

                // Enhanced debug output with gap information
                System.Diagnostics.Debug.WriteLine(
                    $"Detected {detectedProfile.ProfileType} profile with {detectedProfile.OrderedEdgeIds.Count} edges, " +
                    $"Total length: {detectedProfile.TotalLength:F2}mm");

                if (detectedProfile.HasVirtualEdges)
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"  Profile has {detectedProfile.Gaps.Count} gaps, " +
                        $"Total gap length: {detectedProfile.TotalGapLength:F2}mm, " +
                        $"Confidence: {detectedProfile.ProfileConfidence:P}");
                }

                return detectedProfile;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error in GetProfileContainingEdge: {ex.Message}");
                return null;
            }
        }


        private void InvalidateProfileCache()
        {
            profileCache.Clear();
            profileCacheValid = false;
            System.Diagnostics.Debug.WriteLine("Profile cache invalidated");
        }

        // Call this when loading new file
        public void ClearAllCaches()
        {
            InvalidateProfileCache();
            edgeSelectionMap.Clear();
            edgeClassifications.Clear();
            selectedEdgeIds.Clear();
        }

        // Bulk detection for performance
        public void BuildProfileCache()
        {
            try
            {
                if (stepReader == null)
                {
                    System.Diagnostics.Debug.WriteLine("Cannot build profile cache: StepReader is null");
                    return;
                }

                System.Diagnostics.Debug.WriteLine("Building profile cache...");

                // Clear existing cache
                profileCache.Clear();

                // Call DetectAllProfiles() once
                var allProfiles = stepReader.DetectAllProfiles();

                if (allProfiles == null)
                {
                    System.Diagnostics.Debug.WriteLine("DetectAllProfiles returned null");
                    profileCacheValid = false;
                    return;
                }

                System.Diagnostics.Debug.WriteLine($"Found {allProfiles.Count} profiles");

                // Populate cache with all profiles
                int profileIdCounter = 0;
                foreach (var managedProfile in allProfiles)
                {
                    if (managedProfile.IsValid && managedProfile.OrderedEdgeIds != null)
                    {
                        var profile = new ProfileInfo
                        {
                            ProfileId = profileIdCounter++,
                            OrderedEdgeIds = new List<int>(managedProfile.OrderedEdgeIds),
                            IsClosed = managedProfile.IsClosed,
                            ProfileType = managedProfile.ProfileType,
                            TotalLength = managedProfile.TotalLength
                        };

                        // Add to cache for each edge in profile
                        foreach (int edgeId in profile.OrderedEdgeIds)
                        {
                            profileCache[edgeId] = profile;
                        }

                        System.Diagnostics.Debug.WriteLine(
                            $"  Profile {profile.ProfileId}: {profile.ProfileType} with " +
                            $"{profile.OrderedEdgeIds.Count} edges");
                    }
                }

                profileCacheValid = true;
                System.Diagnostics.Debug.WriteLine($"Profile cache built with {profileCache.Count} edge mappings");
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error building profile cache: {ex.Message}");
                profileCacheValid = false;
            }
        }

        // Method to detect all profiles (wrapper)
        public List<ProfileInfo> DetectAllProfiles()
        {
            BuildProfileCache();

            // Return unique profiles
            var uniqueProfiles = profileCache.Values
                .GroupBy(p => p.ProfileId)
                .Select(g => g.First())
                .ToList();

            return uniqueProfiles;
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

        public List<List<int>> GetEdgeGroups()
        {
            var groups = stepReader.GetEdgeGroups();
            var result = new List<List<int>>();
            foreach (var group in groups)
            {
                result.Add(group.ToList());
            }
            return result;
        }

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
        // Thêm vào GeometryModel.cs

        private UnrollingSettings defaultUnrollingSettings = new UnrollingSettings();

        public List<UnrolledToolpath> UnrollSelectedToolpaths()
        {
            var selectedIds = GetSelectedEdgeIds();
            return UnrollSelectedEdges(selectedIds);
        }

        public List<UnrolledToolpath> UnrollSelectedEdges(List<int> edgeIds)
        {
            var unrolledToolpaths = new List<UnrolledToolpath>();

            if (cylinderInfo == null || !cylinderInfo.IsValid)
            {
                System.Diagnostics.Debug.WriteLine("Cannot unroll: No valid cylinder detected");
                return unrolledToolpaths;
            }

            // Convert settings
            var managedParams = new GeometryWrapper.ManagedUnrollingParams
            {
                ChordTolerance = defaultUnrollingSettings.ChordTolerance,
                AngleTolerance = defaultUnrollingSettings.AngleTolerance,
                MinPoints = defaultUnrollingSettings.MinPoints,
                MaxPoints = defaultUnrollingSettings.MaxPoints,
                UnwrapAngles = defaultUnrollingSettings.UnwrapAngles
            };

            // Convert cylinder info
            var managedCylinder = new GeometryWrapper.ManagedCylinderInfo
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

            foreach (var edgeId in edgeIds)
            {
                try
                {
                    // Get edge info
                    var edgeInfo = edgeSelectionMap.ContainsKey(edgeId)
                        ? edgeSelectionMap[edgeId]
                        : null;

                    if (edgeInfo == null) continue;

                    // Unroll edge
                    var managedPoints = stepReader.UnrollEdge(edgeId, managedCylinder, managedParams);

                    if (managedPoints != null && managedPoints.Count > 0)
                    {
                        // Convert to C# points
                        var points = new List<UnrolledPoint>();
                        double minY = double.MaxValue, maxY = double.MinValue;
                        double minC = double.MaxValue, maxC = double.MinValue;

                        foreach (var mp in managedPoints)
                        {
                            var point = new UnrolledPoint
                            {
                                Y = mp.Y,
                                C = mp.C,
                                X = mp.X
                            };
                            points.Add(point);

                            minY = Math.Min(minY, point.Y);
                            maxY = Math.Max(maxY, point.Y);
                            minC = Math.Min(minC, point.C);
                            maxC = Math.Max(maxC, point.C);
                        }

                        var unrolledToolpath = new UnrolledToolpath
                        {
                            EdgeId = edgeId,
                            Points = points,
                            EdgeInfo = new EdgeInfoWrapper(edgeInfo.EdgeInfo),
                            MinY = minY,
                            MaxY = maxY,
                            TotalRotation = maxC - minC,
                            RequiresSeamHandling = (maxC - minC) > 350 // Crosses 360 boundary
                        };

                        unrolledToolpaths.Add(unrolledToolpath);

                        System.Diagnostics.Debug.WriteLine($"Unrolled edge {edgeId}: " +
                            $"{points.Count} points, Y:[{minY:F2},{maxY:F2}], C:[{minC:F2},{maxC:F2}]");
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"Error unrolling edge {edgeId}: {ex.Message}");
                }
            }

            return unrolledToolpaths;
        }
        public List<ProfileAnalyzer.CuttingProfile> AnalyzeUnrolledProfiles(
            List<UnrolledToolpath> unrolledToolpaths)
        {
            var analyzer = new ProfileAnalyzer();

            // Analyze với gap closing
            var settings = new CuttingDirectionSettings
            {
                ProfileConnectionTolerance = 0.5,  // 0.5mm gap tolerance
                CompleteProfileBeforeMoving = true,
                YDirection = CuttingDirectionSettings.YDirectionPreference.AlwaysPositive,
                OptimizeStartPoint = true,
                UseLeadInOut = true,
                LeadInLength = 2.0,
                LeadOutLength = 2.0
            };

            var profiles = analyzer.AnalyzeProfiles(unrolledToolpaths, settings);

            // Handle virtual edges cho gaps
            foreach (var profile in profiles)
            {
                if (profile.TotalGapLength > 0)
                {
                    System.Diagnostics.Debug.WriteLine(
                        $"Profile {profile.ProfileId} has gaps: " +
                        $"{profile.NumGaps} gaps, {profile.TotalGapLength:F2}mm total");

                    // Log chi tiết từng virtual edge nếu cần
                    foreach (var virtualEdge in profile.VirtualEdges)
                    {
                        System.Diagnostics.Debug.WriteLine(
                            $"  Virtual edge: {virtualEdge.Type} from " +
                            $"({virtualEdge.StartPoint.X:F2}, {virtualEdge.StartPoint.Y:F2}) to " +
                            $"({virtualEdge.EndPoint.X:F2}, {virtualEdge.EndPoint.Y:F2}), " +
                            $"Confidence: {virtualEdge.Confidence:P}");
                    }
                }
            }

            System.Diagnostics.Debug.WriteLine(
                $"Analyzed {unrolledToolpaths.Count} toolpaths into {profiles.Count} profiles");

            return profiles;
        }

    }

    public class ProfileInfo
    {
        public List<int> OrderedEdgeIds { get; set; }
        public bool IsClosed { get; set; }
        public string ProfileType { get; set; }
        public double TotalLength { get; set; }
        public int ProfileId { get; set; }
        public List<GapInfo> Gaps { get; set; } = new List<GapInfo>();
        public double TotalGapLength { get; set; }
        public double ProfileConfidence { get; set; } = 1.0;
        public bool HasVirtualEdges { get; set; }
        public bool NeedsGapClosing => Gaps != null && Gaps.Count > 0;

        public ProfileInfo()
        {
            OrderedEdgeIds = new List<int>();
            IsClosed = false;
            ProfileType = "UNKNOWN";
            TotalLength = 0.0;
            ProfileId = -1;
        }
        public class GapInfo
        {
            public int FromEdgeId { get; set; }
            public int ToEdgeId { get; set; }
            public double GapDistance { get; set; }
            public string SuggestedMethod { get; set; }
            public double Confidence { get; set; }
        }

        // Helper method
        public bool ContainsEdge(int edgeId)
        {
            return OrderedEdgeIds != null && OrderedEdgeIds.Contains(edgeId);
        }

        // Helper to get display name
        public string GetDisplayName()
        {
            if (ProfileType == "CIRCLE") return "Circle";
            if (ProfileType == "RECTANGLE") return "Rectangle";
            if (ProfileType == "SLOT") return "Slot";
            if (ProfileType == "POLYGON") return $"Polygon ({OrderedEdgeIds.Count} sides)";
            if (ProfileType == "COMPLEX_CLOSED") return "Complex Shape";
            if (ProfileType == "OPEN_CHAIN") return "Open Chain";
            return "Unknown";
        }
    }

}
