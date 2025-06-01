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
        private Transform3D cylinderToYTransform = Transform3D.Identity;


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

            System.Diagnostics.Debug.WriteLine($"Creating solid cylinder: Center=({cylinderInfo.Center.X:F2},{cylinderInfo.Center.Y:F2},{cylinderInfo.Center.Z:F2})");
            System.Diagnostics.Debug.WriteLine($"Axis=({cylinderInfo.Axis.X:F2},{cylinderInfo.Axis.Y:F2},{cylinderInfo.Axis.Z:F2})");
            System.Diagnostics.Debug.WriteLine($"Radius={cylinderInfo.Radius:F2}, Length={cylinderInfo.Length:F2}");

            // Ensure edgeSelectionMap is populated
            if (edgeSelectionMap.Count == 0)
            {
                CreateWireframeModel(); // This populates edgeSelectionMap
            }

            // 1. Lấy thông tin cylinder GỐC (trước transform)
            var originalInfo = stepReader.DetectMainCylinder();

            // 2. Tạo cylinder mesh với thông tin GỐC
            var cylinderMesh = CreateCylinderMesh(
                new System.Windows.Media.Media3D.Point3D(originalInfo.CenterX, originalInfo.CenterY, originalInfo.CenterZ),
                new Vector3D(originalInfo.AxisX, originalInfo.AxisY, originalInfo.AxisZ),
                originalInfo.Radius,
                originalInfo.Length,
                32
            );

            // Debug mesh
            System.Diagnostics.Debug.WriteLine($"Mesh created: Positions={cylinderMesh.Positions.Count}, Triangles={cylinderMesh.TriangleIndices.Count}");

            // 3. Material
            var cylinderMaterial = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(255, 200, 200, 200)));

            // 4. Tạo GeometryModel3D
            var cylinderModel = new GeometryModel3D
            {
                Geometry = cylinderMesh,
                Material = cylinderMaterial,
                BackMaterial = cylinderMaterial
            };

            // 5. QUAN TRỌNG: THÊM cylinderModel vào modelGroup
            modelGroup.Children.Add(cylinderModel);

            // 6. THÊM MỚI: Vẽ wireframe cuts màu xanh
            var wireframeCuts = CreateWireframeCuts();
            foreach (var cut in wireframeCuts.Children)
            {
                modelGroup.Children.Add(cut);
            }

            // 7. Hiển thị selected edges dạng nét đứt (màu xanh lá)
            var selectedEdges = CreateSelectedEdgeHighlights();
            foreach (var edge in selectedEdges.Children)
            {
                modelGroup.Children.Add(edge);
            }

            // 8. Debug trước transform
            System.Diagnostics.Debug.WriteLine($"Model group has {modelGroup.Children.Count} children before transform");

            // 9. APPLY TRANSFORM cho toàn bộ group
            if (cylinderToYTransform != Transform3D.Identity)
            {
                System.Diagnostics.Debug.WriteLine("Applying cylinderToYTransform");

                var transformedGroup = new Model3DGroup();
                transformedGroup.Transform = cylinderToYTransform;

                // Move tất cả children vào transformed group
                while (modelGroup.Children.Count > 0)
                {
                    var child = modelGroup.Children[0];
                    modelGroup.Children.RemoveAt(0);
                    transformedGroup.Children.Add(child);
                }

                System.Diagnostics.Debug.WriteLine($"Transformed group has {transformedGroup.Children.Count} children");
                return transformedGroup;
            }

            return modelGroup;
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
            System.Diagnostics.Debug.WriteLine($"Creating cylinder: Center={center}, Radius={radius}, Length={length}");
            var mesh = new MeshGeometry3D();
            axis.Normalize();

            // Tạo hệ tọa độ local
            var up = Math.Abs(axis.Y) < 0.9 ? new Vector3D(0, 1, 0) : new Vector3D(1, 0, 0);
            var right = Vector3D.CrossProduct(axis, up);
            right.Normalize();
            up = Vector3D.CrossProduct(right, axis);

            var start = center - axis * length / 2;
            var end = center + axis * length / 2;

            // Vertices cho 2 vòng tròn
            for (int i = 0; i <= segments; i++)
            {
                double angle = 2 * Math.PI * i / segments;
                var offset = Math.Cos(angle) * radius * right + Math.Sin(angle) * radius * up;

                mesh.Positions.Add(start + offset);
                mesh.Positions.Add(end + offset);
            }

            // Triangles cho mặt ngoài
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

            System.Diagnostics.Debug.WriteLine($"Starting CreateWireframeModel");

            if (this.cylinderInfo == null) GetCylinderInfo();

            // Fetch and store classifications if cylinder is valid
            if (this.cylinderInfo != null && this.cylinderInfo.IsValid)
            {
                FetchAndStoreEdgeClassifications();
            }

            // QUAN TRỌNG: Tính transform để xoay cylinder axis về trục Y
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

                        // Tạo rotation transform quanh center của cylinder
                        cylinderTransform = new RotateTransform3D(
                            new AxisAngleRotation3D(rotationAxis, angle),
                            this.cylinderInfo.Center
                        );
                        this.cylinderToYTransform = cylinderTransform;
                        System.Diagnostics.Debug.WriteLine($"Rotating cylinder by {angle:F1}° around axis ({rotationAxis.X:F2},{rotationAxis.Y:F2},{rotationAxis.Z:F2})");
                    }
                }
            }

            // Lấy dữ liệu wireframe từ native code
            List<GeometryWrapper.Point3D> wrapperVertices = null;
            List<Tuple<int, int>> lineIndices = null;
            stepReader.GetWireframeData(ref wrapperVertices, ref lineIndices);

            if (wrapperVertices == null || lineIndices == null)
                return modelGroup;

            // Convert và TRANSFORM vertices
            var vertices = new List<System.Windows.Media.Media3D.Point3D>();
            foreach (var wrapperPoint in wrapperVertices)
            {
                var point = new System.Windows.Media.Media3D.Point3D(
                    wrapperPoint.X, wrapperPoint.Y, wrapperPoint.Z);

                // APPLY TRANSFORM
                point = cylinderTransform.Transform(point);

                vertices.Add(point);
            }

            // Lấy edge info list
            var edgeInfoList = stepReader.GetEdgeInfoList();
            int edgeIndex = 0;

            foreach (var edgeInfo in edgeInfoList)
            {
                if (edgeClassifications.ContainsKey(edgeInfo.Id))
                {
                    var classification = edgeClassifications[edgeInfo.Id];
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
                    Points = new List<System.Windows.Media.Media3D.Point3D>(),
                    IsSelected = false,
                    IsHovered = false,
                    Classification = edgeClassifications.ContainsKey(edgeInfo.Id) ?
                        edgeClassifications[edgeInfo.Id] : new EdgeClassificationData()
                };

                // Get edge points và TRANSFORM chúng
                var managedEdgePoints = stepReader.GetEdgePoints(edgeInfo.Id);
                if (managedEdgePoints != null)
                {
                    foreach (var mp in managedEdgePoints)
                    {
                        var point = new System.Windows.Media.Media3D.Point3D(mp.X, mp.Y, mp.Z);
                        // APPLY TRANSFORM
                        point = cylinderTransform.Transform(point);
                        edgeSelectionInfo.Points.Add(point);
                    }
                }

                // Default material
                var defaultMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Blue));
                edgeSelectionInfo.OriginalMaterial = defaultMaterial;

                if (edgeSelectionInfo.Points.Count >= 2)
                {
                    for (int i = 0; i < edgeSelectionInfo.Points.Count - 1; i++)
                    {
                        var p1 = edgeSelectionInfo.Points[i];
                        var p2 = edgeSelectionInfo.Points[i + 1];

                        var segmentVisual = CreateCylinderBetweenPoints(p1, p2, 0.5);
                        var segmentGeometryModel = new GeometryModel3D
                        {
                            Geometry = segmentVisual,
                            Material = defaultMaterial,
                            BackMaterial = defaultMaterial
                        };
                        edgeGroup.Children.Add(segmentGeometryModel);
                    }
                }

                var edgeModel = new GeometryModel3D
                {
                    Geometry = CreateMeshFromGroup(edgeGroup),
                    Material = defaultMaterial,
                    BackMaterial = defaultMaterial
                };

                edgeSelectionInfo.Model3D = edgeModel;
                edgeSelectionMap[edgeInfo.Id] = edgeSelectionInfo;

                modelGroup.Children.Add(edgeModel);
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

            var axisLength = cylinderInfo.Length;

            // Axis theo Y (sau khi đã transform)
            var startPoint = new System.Windows.Media.Media3D.Point3D(
                cylinderInfo.Center.X,
                cylinderInfo.Center.Y - (axisLength / 2),
                cylinderInfo.Center.Z
            );

            var endPoint = new System.Windows.Media.Media3D.Point3D(
                cylinderInfo.Center.X,
                cylinderInfo.Center.Y + (axisLength / 2),
                cylinderInfo.Center.Z
            );

            // APPLY CÙNG TRANSFORM NHƯ WIREFRAME
            startPoint = cylinderToYTransform.Transform(startPoint);
            endPoint = cylinderToYTransform.Transform(endPoint);

            // Vẽ cylinder màu đỏ
            var axisCylinder = CreateCylinderBetweenPoints(startPoint, endPoint, 2.0);
            var axisMaterial = new DiffuseMaterial(new SolidColorBrush(Colors.Red));

            group.Children.Add(new GeometryModel3D
            {
                Geometry = axisCylinder,
                Material = axisMaterial,
                BackMaterial = axisMaterial
            });

            // XÓA PHẦN ARROW HEAD - có thể đây là nguyên nhân tạo ra trục lạ
            /* 
            // Arrow head
            var arrowLength = 10.0;
            var arrowRadius = 5.0;
            var arrowBase = endPoint - (new Vector3D(0, 1, 0) * arrowLength);
            arrowBase = cylinderToYTransform.Transform(arrowBase);  // SAI: transform 2 lần

            var arrowMesh = CreateCone(arrowBase, endPoint, arrowRadius);
            group.Children.Add(new GeometryModel3D
            {
                Geometry = arrowMesh,
                Material = axisMaterial,
                BackMaterial = axisMaterial
            });
            */

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
    }
}