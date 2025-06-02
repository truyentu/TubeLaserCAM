// TubeLaserCAM.UI/Models/ProfileAnalyzer.cs
// ĐÂY LÀ FILE MỚI - Tạo trong folder Models/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace TubeLaserCAM.UI.Models
{
    /// <summary>
    /// Phân tích và nhóm edges thành profiles để tối ưu G-Code generation
    /// </summary>
    public class ProfileAnalyzer
    {
        #region Constants
        private const double POSITION_TOLERANCE = 0.1; //
        private const double TANGENT_ANGLE_TOLERANCE = 15.0; // 15 degrees
        private const double PARALLEL_TOLERANCE = 1.0; // 1 degree cho parallel checking
        #endregion

        #region Data Structures

        /// <summary>
        /// Node trong edge graph
        /// </summary>
        /// 

        public class VirtualEdge
        {
            public string Type { get; set; } // "LINE", "ARC"
            public Point3D StartPoint { get; set; }
            public Point3D EndPoint { get; set; }
            public double Confidence { get; set; }
            public string Reason { get; set; } // "GAP_CLOSING", "MISSING_PATTERN"
            public double Length { get; set; }

            public VirtualEdge()
            {
                Type = "LINE";
                Confidence = 1.0;
                Reason = "GAP_CLOSING";
            }
        }

        public class EdgeNode
        {
            public int EdgeId { get; set; }
            public Point3D StartPoint { get; set; }
            public Point3D EndPoint { get; set; }
            public Vector3D StartTangent { get; set; }
            public Vector3D EndTangent { get; set; }
            public UnrolledToolpath Toolpath { get; set; }
            public EdgeInfoWrapper EdgeInfo { get; set; }
        }

        /// <summary>
        /// Connection giữa 2 edges
        /// </summary>
        public class EdgeConnection
        {
            public int FromEdgeId { get; set; }
            public int ToEdgeId { get; set; }
            public ConnectionType Type { get; set; }
            public double TangentAngle { get; set; }
            public bool IsSmooth { get; set; } // True nếu tangent continuous
        }

        /// <summary>
        /// Loại connection
        /// </summary>
        public enum ConnectionType
        {
            EndToStart,   // End của edge1 -> Start của edge2 (normal)
            EndToEnd,     // End của edge1 -> End của edge2 (reversed)
            StartToStart, // Start của edge1 -> Start của edge2 (both reversed)
            StartToEnd    // Start của edge1 -> End của edge2 (edge1 reversed)
        }

        /// <summary>
        /// Profile đã phân tích
        /// </summary>
        public class CuttingProfile
        {
            public int ProfileId { get; set; }
            public List<EdgeNode> Edges { get; set; } = new List<EdgeNode>();
            public List<EdgeConnection> Connections { get; set; } = new List<EdgeConnection>();
            public bool IsClosed { get; set; }
            public ProfileType Type { get; set; }
            public double MinY { get; set; }
            public double MaxY { get; set; }
            public double CenterY { get; set; }
            public Point3D BoundingBoxMin { get; set; }
            public Point3D BoundingBoxMax { get; set; }

            // Hierarchy
            public int? ParentProfileId { get; set; }
            public List<int> ChildProfileIds { get; set; } = new List<int>();

            // Optimization
            public Point3D OptimalStartPoint { get; set; }
            public CuttingDirection PreferredDirection { get; set; }
            public int CuttingOrder { get; set; } // Thứ tự cắt (0 = first)
            public double TotalGapLength { get; set; }
            public int NumGaps { get; set; }
            public List<VirtualEdge> VirtualEdges { get; set; } = new List<VirtualEdge>();
            public double Confidence { get; set; } = 1.0;
            public bool IsCompleteWithGaps { get; set; }
        }

        /// <summary>
        /// Loại profile
        /// </summary>
        public enum ProfileType
        {
            Unknown,
            Circle,
            Rectangle,
            RoundedRectangle,
            Slot,
            Polygon,
            OpenChain,
            Complex
        }

        /// <summary>
        /// Hướng cắt
        /// </summary>
        public enum CuttingDirection
        {
            YPositive,  // -Y to +Y
            YNegative,  // +Y to -Y
            Clockwise,
            CounterClockwise
        }

        #endregion

        #region Main Analysis Method

        /// <summary>
        /// Phân tích và nhóm toolpaths thành profiles
        /// </summary>
        public List<CuttingProfile> AnalyzeProfiles(
            List<UnrolledToolpath> toolpaths,
            CuttingDirectionSettings settings = null)
        {
            if (toolpaths == null || toolpaths.Count == 0)
                return new List<CuttingProfile>();

            settings = settings ?? new CuttingDirectionSettings();

            try
            {
                System.Diagnostics.Debug.WriteLine($"ProfileAnalyzer: Analyzing {toolpaths.Count} toolpaths");

                // STEP 1: Build Edge Graph
                var nodes = BuildEdgeNodes(toolpaths);

                // THÊM: Validate nodes
                if (nodes.Count == 0)
                {
                    System.Diagnostics.Debug.WriteLine("No valid nodes created from toolpaths");
                    return new List<CuttingProfile>();
                }

                var connections = FindConnections(nodes);

                System.Diagnostics.Debug.WriteLine($"Found {connections.Count} connections between edges");

                // STEP 2: Find Connected Components
                var components = FindConnectedComponents(nodes, connections);

                System.Diagnostics.Debug.WriteLine($"Found {components.Count} connected components");

                // STEP 3: Create Profiles from Components
                var profiles = new List<CuttingProfile>();
                int profileId = 0;

                foreach (var component in components)
                {
                    try
                    {
                        var componentProfiles = AnalyzeComponent(component, connections, profileId);
                        profiles.AddRange(componentProfiles);
                        profileId += componentProfiles.Count;
                    }
                    catch (Exception ex)
                    {
                        System.Diagnostics.Debug.WriteLine($"Error analyzing component: {ex.Message}");
                        // Continue with other components
                    }
                }

                System.Diagnostics.Debug.WriteLine($"Created {profiles.Count} profiles");

                // STEP 4: Analyze Profile Types
                foreach (var profile in profiles)
                {
                    profile.Type = DetermineProfileType(profile);
                    CalculateBoundingBox(profile);
                    System.Diagnostics.Debug.WriteLine($"Profile {profile.ProfileId}: Type={profile.Type}, Closed={profile.IsClosed}");
                }

                // STEP 5: Analyze Hierarchy (nested profiles)
                AnalyzeProfileHierarchy(profiles);

                // STEP 6: Optimize Cutting Order
                OptimizeCuttingOrder(profiles, settings);

                return profiles;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"ProfileAnalyzer error: {ex}");
                return new List<CuttingProfile>();
            }
        }

        #endregion

        #region Step 1: Build Edge Graph

        /// <summary>
        /// Tạo edge nodes từ toolpaths
        /// </summary>
        private Dictionary<int, EdgeNode> BuildEdgeNodes(List<UnrolledToolpath> toolpaths)
        {
            var nodes = new Dictionary<int, EdgeNode>();

            foreach (var toolpath in toolpaths)
            {
                if (toolpath.Points == null || toolpath.Points.Count < 2)
                    continue;

                var node = new EdgeNode
                {
                    EdgeId = toolpath.EdgeId,
                    Toolpath = toolpath,
                    EdgeInfo = toolpath.EdgeInfo
                };

                // Convert 2D points to 3D (Y,C -> X,Y in 3D space for analysis)
                // Y remains Y, C converts to X for planar analysis
                var firstPoint = toolpath.Points.First();
                var lastPoint = toolpath.Points.Last();

                node.StartPoint = new Point3D(firstPoint.C, firstPoint.Y, 0);
                node.EndPoint = new Point3D(lastPoint.C, lastPoint.Y, 0);
                // Debug cho circles
                if (toolpath.EdgeInfo?.Type == "Circle" && toolpath.Points.Count < 20)
                {
                    System.Diagnostics.Debug.WriteLine($"  Small Circle Edge#{toolpath.EdgeId}:");
                    System.Diagnostics.Debug.WriteLine($"    Points: {toolpath.Points.Count}");
                    System.Diagnostics.Debug.WriteLine($"    Start: C={firstPoint.C:F3} -> End: C={lastPoint.C:F3}");
                    System.Diagnostics.Debug.WriteLine($"    Start==End? {firstPoint.C == lastPoint.C && firstPoint.Y == lastPoint.Y}");
                }
                // Calculate tangents
                node.StartTangent = CalculateTangentAtStart(toolpath);
                node.EndTangent = CalculateTangentAtEnd(toolpath);


                nodes[toolpath.EdgeId] = node;
            }

            return nodes;
        }

        /// <summary>
        /// Tính tangent vector tại điểm đầu
        /// </summary>
        private Vector3D CalculateTangentAtStart(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 2)
                return new Vector3D(0, 0, 0);

            var p1 = toolpath.Points[0];
            var p2 = toolpath.Points[Math.Min(1, toolpath.Points.Count - 1)];

            // Tìm điểm đủ xa để tính tangent chính xác
            int index = 1;
            double minDist = 0.5; // Minimum 0.5mm

            while (index < toolpath.Points.Count - 1)
            {
                var dist = Math.Sqrt(
                    Math.Pow(p2.Y - p1.Y, 2) +
                    Math.Pow(p2.C - p1.C, 2)
                );

                if (dist >= minDist) break;
                index++;
                p2 = toolpath.Points[index];
            }

            var tangent = new Vector3D(p2.C - p1.C, p2.Y - p1.Y, 0);
            tangent.Normalize();
            return tangent;
        }

        /// <summary>
        /// Tính tangent vector tại điểm cuối
        /// </summary>
        private Vector3D CalculateTangentAtEnd(UnrolledToolpath toolpath)
        {
            if (toolpath.Points.Count < 2)
                return new Vector3D(0, 0, 0);

            var p1 = toolpath.Points[toolpath.Points.Count - 1];
            var p2 = toolpath.Points[Math.Max(0, toolpath.Points.Count - 2)];

            // Tìm điểm đủ xa
            int index = toolpath.Points.Count - 2;
            double minDist = 0.5;

            while (index > 0)
            {
                var dist = Math.Sqrt(
                    Math.Pow(p1.Y - p2.Y, 2) +
                    Math.Pow(p1.C - p2.C, 2)
                );

                if (dist >= minDist) break;
                index--;
                p2 = toolpath.Points[index];
            }

            var tangent = new Vector3D(p1.C - p2.C, p1.Y - p2.Y, 0);
            tangent.Normalize();
            return tangent;
        }

        #endregion

        #region Step 2: Find Connections

        /// <summary>
        /// Tìm tất cả connections giữa edges
        /// </summary>
        private List<EdgeConnection> FindConnections(Dictionary<int, EdgeNode> nodes)
        {
            var connections = new List<EdgeConnection>();

            var nodeList = nodes.Values.ToList();

            for (int i = 0; i < nodeList.Count; i++)
            {
                for (int j = i + 1; j < nodeList.Count; j++)
                {
                    var possibleConnections = CheckPossibleConnections(
                        nodeList[i], nodeList[j]);

                    connections.AddRange(possibleConnections);
                }
            }

            return connections;
        }

        /// <summary>
        /// Kiểm tra tất cả possible connections giữa 2 edges
        /// </summary>
        private List<EdgeConnection> CheckPossibleConnections(EdgeNode node1, EdgeNode node2)
        {
            var connections = new List<EdgeConnection>();

            System.Diagnostics.Debug.WriteLine($"Checking connections between Edge#{node1.EdgeId} and Edge#{node2.EdgeId}");
            System.Diagnostics.Debug.WriteLine($"  Edge#{node1.EdgeId}: Start({node1.StartPoint.X:F3},{node1.StartPoint.Y:F3}) End({node1.EndPoint.X:F3},{node1.EndPoint.Y:F3})");
            System.Diagnostics.Debug.WriteLine($"  Edge#{node2.EdgeId}: Start({node2.StartPoint.X:F3},{node2.StartPoint.Y:F3}) End({node2.EndPoint.X:F3},{node2.EndPoint.Y:F3})");

            // Check 4 possible connections với C-axis wrap support

            // 1. End1 -> Start2 (normal continuation)
            if (ArePointsEqualWithWrap(node1.EndPoint, node2.StartPoint))
            {
                System.Diagnostics.Debug.WriteLine($"  Found End1->Start2 connection (with wrap check)");
                double angle = Vector3D.AngleBetween(node1.EndTangent, node2.StartTangent);
                connections.Add(new EdgeConnection
                {
                    FromEdgeId = node1.EdgeId,
                    ToEdgeId = node2.EdgeId,
                    Type = ConnectionType.EndToStart,
                    TangentAngle = angle,
                    IsSmooth = angle < TANGENT_ANGLE_TOLERANCE ||
                              angle > (180 - TANGENT_ANGLE_TOLERANCE)
                });
            }

            // 2. End1 -> End2 (node2 reversed)
            if (ArePointsEqualWithWrap(node1.EndPoint, node2.EndPoint))
            {
                System.Diagnostics.Debug.WriteLine($"  Found End1->End2 connection (with wrap check)");
                double angle = Vector3D.AngleBetween(node1.EndTangent, -node2.EndTangent);
                connections.Add(new EdgeConnection
                {
                    FromEdgeId = node1.EdgeId,
                    ToEdgeId = node2.EdgeId,
                    Type = ConnectionType.EndToEnd,
                    TangentAngle = angle,
                    IsSmooth = angle < TANGENT_ANGLE_TOLERANCE ||
                              angle > (180 - TANGENT_ANGLE_TOLERANCE)
                });
            }

            // 3. Start1 -> Start2 (both reversed)
            if (ArePointsEqualWithWrap(node1.StartPoint, node2.StartPoint))
            {
                System.Diagnostics.Debug.WriteLine($"  Found Start1->Start2 connection (with wrap check)");
                double angle = Vector3D.AngleBetween(-node1.StartTangent, node2.StartTangent);
                connections.Add(new EdgeConnection
                {
                    FromEdgeId = node1.EdgeId,
                    ToEdgeId = node2.EdgeId,
                    Type = ConnectionType.StartToStart,
                    TangentAngle = angle,
                    IsSmooth = angle < TANGENT_ANGLE_TOLERANCE ||
                              angle > (180 - TANGENT_ANGLE_TOLERANCE)
                });
            }

            // 4. Start1 -> End2 (node1 reversed)
            if (ArePointsEqualWithWrap(node1.StartPoint, node2.EndPoint))
            {
                System.Diagnostics.Debug.WriteLine($"  Found Start1->End2 connection (with wrap check)");
                double angle = Vector3D.AngleBetween(-node1.StartTangent, -node2.EndTangent);
                connections.Add(new EdgeConnection
                {
                    FromEdgeId = node1.EdgeId,
                    ToEdgeId = node2.EdgeId,
                    Type = ConnectionType.StartToEnd,
                    TangentAngle = angle,
                    IsSmooth = angle < TANGENT_ANGLE_TOLERANCE ||
                              angle > (180 - TANGENT_ANGLE_TOLERANCE)
                });
            }

            // Debug output nếu tìm thấy connections
            if (connections.Count > 0)
            {
                System.Diagnostics.Debug.WriteLine($"  Total connections found: {connections.Count}");
                foreach (var conn in connections)
                {
                    System.Diagnostics.Debug.WriteLine($"    {conn.Type}: {conn.FromEdgeId}->{conn.ToEdgeId}, Smooth={conn.IsSmooth}, Angle={conn.TangentAngle:F1}°");
                }
            }

            return connections;
        }

        /// <summary>
        /// Kiểm tra 2 điểm có bằng nhau (với tolerance)
        /// </summary>
        private bool ArePointsEqual(Point3D p1, Point3D p2)
        {
            double distance = (p1 - p2).Length;
            bool equal = distance < POSITION_TOLERANCE;

            // Debug chi tiết
            if (distance < 1.0) // Trong khoảng 1mm
            {
                System.Diagnostics.Debug.WriteLine($"Point comparison: ({p1.X:F3},{p1.Y:F3}) vs ({p2.X:F3},{p2.Y:F3}) = {distance:F6}mm, Equal={equal}");
            }

            return equal;
        }

        #endregion

        #region Step 3: Find Connected Components

        /// <summary>
        /// Tìm các connected components trong graph
        /// </summary>
        /// 
        /// <summary>
        /// Kiểm tra 2 điểm có bằng nhau với C-axis wrap
        /// </summary>
        /// <summary>
        /// Kiểm tra 2 điểm có bằng nhau với C-axis wrap-around support
        /// </summary>
        private bool ArePointsEqualWithWrap(Point3D p1, Point3D p2)
        {
            // Check Y first (no wrap needed)
            if (Math.Abs(p1.Y - p2.Y) > POSITION_TOLERANCE)
                return false;

            // Check C (X in 3D space) with wrap-around at 0°/360°
            double c1 = p1.X; // X represents C angle
            double c2 = p2.X;

            // Direct comparison
            if (Math.Abs(c1 - c2) < POSITION_TOLERANCE)
            {
                System.Diagnostics.Debug.WriteLine($"    Points equal directly: C1={c1:F3} ≈ C2={c2:F3}");
                return true;
            }

            // Check with +360° wrap (c2 wrapped forward)
            if (Math.Abs(c1 - (c2 + 360)) < POSITION_TOLERANCE)
            {
                System.Diagnostics.Debug.WriteLine($"    Points equal with +360 wrap: C1={c1:F3} ≈ C2+360={c2 + 360:F3}");
                return true;
            }

            // Check with -360° wrap (c2 wrapped backward)
            if (Math.Abs(c1 - (c2 - 360)) < POSITION_TOLERANCE)
            {
                System.Diagnostics.Debug.WriteLine($"    Points equal with -360 wrap: C1={c1:F3} ≈ C2-360={c2 - 360:F3}");
                return true;
            }

            // Also check reverse (c1 wrapped)
            if (Math.Abs((c1 + 360) - c2) < POSITION_TOLERANCE)
            {
                System.Diagnostics.Debug.WriteLine($"    Points equal with C1+360 wrap: C1+360={c1 + 360:F3} ≈ C2={c2:F3}");
                return true;
            }

            if (Math.Abs((c1 - 360) - c2) < POSITION_TOLERANCE)
            {
                System.Diagnostics.Debug.WriteLine($"    Points equal with C1-360 wrap: C1-360={c1 - 360:F3} ≈ C2={c2:F3}");
                return true;
            }

            return false;
        }

        private List<List<EdgeNode>> FindConnectedComponents(
            Dictionary<int, EdgeNode> nodes,
            List<EdgeConnection> connections)
        {
            var components = new List<List<EdgeNode>>();
            var visited = new HashSet<int>();

            // Build adjacency list
            var adjacency = new Dictionary<int, List<int>>();
            foreach (var conn in connections)
            {
                if (!adjacency.ContainsKey(conn.FromEdgeId))
                    adjacency[conn.FromEdgeId] = new List<int>();
                if (!adjacency.ContainsKey(conn.ToEdgeId))
                    adjacency[conn.ToEdgeId] = new List<int>();

                adjacency[conn.FromEdgeId].Add(conn.ToEdgeId);
                adjacency[conn.ToEdgeId].Add(conn.FromEdgeId);
            }

            // DFS to find components
            foreach (var node in nodes.Values)
            {
                if (!visited.Contains(node.EdgeId))
                {
                    var component = new List<EdgeNode>();
                    DFSComponent(node.EdgeId, nodes, adjacency, visited, component);

                    if (component.Count > 0)
                        components.Add(component);
                }
            }

            return components;
        }

        /// <summary>
        /// DFS helper để tìm component
        /// </summary>
        private void DFSComponent(
            int edgeId,
            Dictionary<int, EdgeNode> nodes,
            Dictionary<int, List<int>> adjacency,
            HashSet<int> visited,
            List<EdgeNode> component)
        {
            visited.Add(edgeId);
            component.Add(nodes[edgeId]);

            if (adjacency.ContainsKey(edgeId))
            {
                foreach (var neighborId in adjacency[edgeId])
                {
                    if (!visited.Contains(neighborId))
                    {
                        DFSComponent(neighborId, nodes, adjacency, visited, component);
                    }
                }
            }
        }

        #endregion

        #region Step 4: Analyze Component

        /// <summary>
        /// Phân tích một component thành profiles
        /// </summary>
        private List<CuttingProfile> AnalyzeComponent(
            List<EdgeNode> component,
            List<EdgeConnection> allConnections,
            int startProfileId)
        {
            var profiles = new List<CuttingProfile>();

            // Get connections for this component
            var componentEdgeIds = component.Select(n => n.EdgeId).ToHashSet();
            var componentConnections = allConnections
                .Where(c => componentEdgeIds.Contains(c.FromEdgeId) &&
                           componentEdgeIds.Contains(c.ToEdgeId))
                .ToList();

            // THÊM: Validate component
            if (component.Count == 0)
                return profiles;

            // Try to find closed loops first
            var loops = FindClosedLoops(component, componentConnections);

            var usedEdges = new HashSet<int>();
            int profileId = startProfileId;

            // Create profiles for closed loops
            foreach (var loop in loops)
            {
                var profile = CreateProfileFromLoop(loop, componentConnections, profileId++);
                profiles.Add(profile);

                foreach (var edge in loop)
                    usedEdges.Add(edge.EdgeId);
            }

            // Handle remaining edges as open chains
            var remainingEdges = component.Where(e => !usedEdges.Contains(e.EdgeId)).ToList();
            if (remainingEdges.Any())
            {
                var openChains = ExtractOpenChains(remainingEdges, componentConnections);
                foreach (var chain in openChains)
                {
                    var profile = CreateProfileFromChain(chain, componentConnections, profileId++);
                    profiles.Add(profile);
                }
            }

            return profiles;
        }

        /// <summary>
        /// Tìm closed loops trong component
        /// </summary>
        private List<List<EdgeNode>> FindClosedLoops(
            List<EdgeNode> component,
            List<EdgeConnection> connections)
        {
            var loops = new List<List<EdgeNode>>();
            var visited = new HashSet<int>();

            // THÊM: Create node dictionary
            var nodeDict = component.ToDictionary(n => n.EdgeId);

            System.Diagnostics.Debug.WriteLine($"FindClosedLoops: Checking {component.Count} nodes");


            foreach (var startNode in component)
            {
                if (visited.Contains(startNode.EdgeId))
                    continue;

                var path = new List<EdgeNode>();
                var pathSet = new HashSet<int>();

                if (DFSFindLoop(startNode, startNode, connections,
                               visited, path, pathSet, loops, nodeDict))  // THÊM nodeDict
                {
                    // Mark all edges in found loop as visited
                    foreach (var node in loops.Last())
                        visited.Add(node.EdgeId);
                }
            }

            return loops;
        }

        /// <summary>
        /// DFS để tìm closed loop
        /// </summary>
        private bool DFSFindLoop(
            EdgeNode startNode,
            EdgeNode currentNode,
            List<EdgeConnection> connections,
            HashSet<int> globalVisited,
            List<EdgeNode> path,
            HashSet<int> pathSet,
            List<List<EdgeNode>> foundLoops,
            Dictionary<int, EdgeNode> nodeDict)  // THÊM parameter này
        {
            path.Add(currentNode);
            pathSet.Add(currentNode.EdgeId);

            // Find connections from current node
            var outgoingConnections = connections
                .Where(c => c.FromEdgeId == currentNode.EdgeId) // Allow traversing non-smooth connections to find loops with corners
                .ToList();

            foreach (var conn in outgoingConnections)
            {
                // Check if we've returned to start (found loop)
                if (conn.ToEdgeId == startNode.EdgeId && path.Count > 2)
                {
                    foundLoops.Add(new List<EdgeNode>(path));
                    return true;
                }

                // Continue DFS if not visited in current path
                if (!pathSet.Contains(conn.ToEdgeId))
                {
                    // SỬA: Lấy node từ dictionary thay vì placeholder
                    if (nodeDict.TryGetValue(conn.ToEdgeId, out EdgeNode nextNode))
                    {
                        if (DFSFindLoop(startNode, nextNode, connections,
                                       globalVisited, path, pathSet, foundLoops, nodeDict))
                        {
                            return true;
                        }
                    }
                }
            }

            // Backtrack
            path.RemoveAt(path.Count - 1);
            pathSet.Remove(currentNode.EdgeId);
            return false;
        }

        /// <summary>
        /// Extract open chains từ remaining edges
        /// </summary>
        private List<List<EdgeNode>> ExtractOpenChains(
            List<EdgeNode> edges,
            List<EdgeConnection> connections)
        {
            var chains = new List<List<EdgeNode>>();
            var used = new HashSet<int>();

            foreach (var edge in edges)
            {
                if (used.Contains(edge.EdgeId))
                    continue;

                var chain = new List<EdgeNode>();
                BuildChainFromEdge(edge, edges, connections, used, chain);

                if (chain.Count > 0)
                    chains.Add(chain);
            }

            return chains;
        }

        /// <summary>
        /// Build chain từ starting edge
        /// </summary>
        private void BuildChainFromEdge(
            EdgeNode startEdge,
            List<EdgeNode> availableEdges,
            List<EdgeConnection> connections,
            HashSet<int> used,
            List<EdgeNode> chain)
        {
            // THÊM: Giới hạn độ dài chain để tránh infinite loop
            if (chain.Count > availableEdges.Count)
            {
                System.Diagnostics.Debug.WriteLine("Warning: Chain exceeds available edges count");
                return;
            }

            chain.Add(startEdge);
            used.Add(startEdge.EdgeId);

            // Find next connected edge
            var nextConn = connections
                .Where(c => c.FromEdgeId == startEdge.EdgeId &&
                           !used.Contains(c.ToEdgeId) &&
                           c.IsSmooth)
                .OrderBy(c => c.TangentAngle)
                .FirstOrDefault();

            if (nextConn != null)
            {
                var nextEdge = availableEdges.FirstOrDefault(e => e.EdgeId == nextConn.ToEdgeId);
                if (nextEdge != null)
                {
                    BuildChainFromEdge(nextEdge, availableEdges, connections, used, chain);
                }
            }
        }

        #endregion

        #region Step 5: Create Profiles

        /// <summary>
        /// Tạo profile từ closed loop
        /// </summary>
        private CuttingProfile CreateProfileFromLoop(
            List<EdgeNode> loop,
            List<EdgeConnection> connections,
            int profileId)
        {
            var profile = new CuttingProfile
            {
                ProfileId = profileId,
                Edges = loop,
                IsClosed = true,
                Connections = connections.Where(c =>
                    loop.Any(e => e.EdgeId == c.FromEdgeId) &&
                    loop.Any(e => e.EdgeId == c.ToEdgeId)).ToList()
            };

            // Calculate Y bounds
            profile.MinY = loop.SelectMany(e => e.Toolpath.Points).Min(p => p.Y);
            profile.MaxY = loop.SelectMany(e => e.Toolpath.Points).Max(p => p.Y);
            profile.CenterY = (profile.MinY + profile.MaxY) / 2;

            return profile;
        }

        /// <summary>
        /// Tạo profile từ open chain
        /// </summary>
        private CuttingProfile CreateProfileFromChain(
            List<EdgeNode> chain,
            List<EdgeConnection> connections,
            int profileId)
        {
            var profile = new CuttingProfile
            {
                ProfileId = profileId,
                Edges = chain,
                IsClosed = false,
                Connections = connections.Where(c =>
                    chain.Any(e => e.EdgeId == c.FromEdgeId) &&
                    chain.Any(e => e.EdgeId == c.ToEdgeId)).ToList()
            };

            // Calculate Y bounds
            profile.MinY = chain.SelectMany(e => e.Toolpath.Points).Min(p => p.Y);
            profile.MaxY = chain.SelectMany(e => e.Toolpath.Points).Max(p => p.Y);
            profile.CenterY = (profile.MinY + profile.MaxY) / 2;

            return profile;
        }

        #endregion

        #region Step 6: Determine Profile Type

        /// <summary>
        /// Xác định loại profile
        /// </summary>
        private ProfileType DetermineProfileType(CuttingProfile profile)
        {
            if (!profile.IsClosed)
                return ProfileType.OpenChain;

            var edgeCount = profile.Edges.Count;
            var edgeTypes = profile.Edges.Select(e => e.EdgeInfo.Type).ToList();

            // Single circle
            if (edgeCount == 1 && edgeTypes[0] == "Circle")
                return ProfileType.Circle;

            // Rectangle (4 lines)
            if (edgeCount == 4 && edgeTypes.All(t => t == "Line"))
            {
                // Check if angles are ~90 degrees
                bool isRectangle = true;
                foreach (var conn in profile.Connections)
                {
                    if (Math.Abs(conn.TangentAngle - 90) > 5 &&
                        Math.Abs(conn.TangentAngle - 270) > 5)
                    {
                        isRectangle = false;
                        break;
                    }
                }

                if (isRectangle)
                    return ProfileType.Rectangle;
            }

            // Rounded Rectangle (4 lines + 4 arcs)
            if (edgeCount == 8)
            {
                int lineCount = edgeTypes.Count(t => t == "Line");
                int arcCount = edgeTypes.Count(t => t == "Circle" || t == "BSpline");

                if (lineCount == 4 && arcCount == 4)
                    return ProfileType.RoundedRectangle;
            }

            // Slot (2 lines + 2 semicircles)
            if (edgeCount == 4 &&
                edgeTypes.Count(t => t == "Line") == 2 &&
                edgeTypes.Count(t => t == "Circle") == 2)
            {
                return ProfileType.Slot;
            }

            // Polygon (all lines)
            if (edgeTypes.All(t => t == "Line") && edgeCount > 4)
                return ProfileType.Polygon;

            return ProfileType.Complex;
        }

        #endregion

        #region Step 7: Calculate Bounding Box

        /// <summary>
        /// Tính bounding box cho profile
        /// </summary>
        private void CalculateBoundingBox(CuttingProfile profile)
        {
            var allPoints = profile.Edges
                .SelectMany(e => e.Toolpath.Points)
                .ToList();

            if (!allPoints.Any())
                return;

            double minY = allPoints.Min(p => p.Y);
            double maxY = allPoints.Max(p => p.Y);
            double minC = allPoints.Min(p => p.C);
            double maxC = allPoints.Max(p => p.C);

            profile.BoundingBoxMin = new Point3D(minC, minY, 0);
            profile.BoundingBoxMax = new Point3D(maxC, maxY, 0);
        }

        #endregion

        #region Step 8: Analyze Hierarchy

        /// <summary>
        /// Phân tích quan hệ lồng nhau giữa profiles
        /// </summary>
        private void AnalyzeProfileHierarchy(List<CuttingProfile> profiles)
        {
            // Only analyze closed profiles
            var closedProfiles = profiles.Where(p => p.IsClosed).ToList();

            for (int i = 0; i < closedProfiles.Count; i++)
            {
                for (int j = 0; j < closedProfiles.Count; j++)
                {
                    if (i == j) continue;

                    var outer = closedProfiles[i];
                    var inner = closedProfiles[j];

                    if (IsProfileInside(inner, outer))
                    {
                        inner.ParentProfileId = outer.ProfileId;
                        outer.ChildProfileIds.Add(inner.ProfileId);
                    }
                }
            }

            // Remove indirect parent relationships
            foreach (var profile in closedProfiles)
            {
                if (profile.ParentProfileId.HasValue)
                {
                    // Find direct parent (closest parent)
                    var directParent = FindDirectParent(profile, closedProfiles);
                    if (directParent != null)
                    {
                        profile.ParentProfileId = directParent.ProfileId;
                    }
                }
            }
        }

        /// <summary>
        /// Check if inner profile is inside outer profile
        /// </summary>
        private bool IsProfileInside(CuttingProfile inner, CuttingProfile outer)
        {
            // Simple bounding box check first
            if (inner.BoundingBoxMin.X < outer.BoundingBoxMin.X ||
                inner.BoundingBoxMax.X > outer.BoundingBoxMax.X ||
                inner.BoundingBoxMin.Y < outer.BoundingBoxMin.Y ||
                inner.BoundingBoxMax.Y > outer.BoundingBoxMax.Y)
            {
                return false;
            }

            // Check if any point of inner is inside outer
            // Using winding number algorithm
            var innerPoint = inner.Edges.First().Toolpath.Points.First();
            return IsPointInsideProfile(
                new Point3D(innerPoint.C, innerPoint.Y, 0),
                outer);
        }

        /// <summary>
        /// Winding number algorithm để check point in polygon
        /// </summary>
        private bool IsPointInsideProfile(Point3D point, CuttingProfile profile)
        {
            int windingNumber = 0;

            foreach (var edge in profile.Edges)
            {
                var points = edge.Toolpath.Points;
                for (int i = 0; i < points.Count - 1; i++)
                {
                    var p1 = new Point3D(points[i].C, points[i].Y, 0);
                    var p2 = new Point3D(points[i + 1].C, points[i + 1].Y, 0);

                    if (p1.Y <= point.Y)
                    {
                        if (p2.Y > point.Y)
                        {
                            if (IsLeft(p1, p2, point))
                                windingNumber++;
                        }
                    }
                    else
                    {
                        if (p2.Y <= point.Y)
                        {
                            if (!IsLeft(p1, p2, point))
                                windingNumber--;
                        }
                    }
                }
            }

            return windingNumber != 0;
        }

        /// <summary>
        /// Test if point is left of line
        /// </summary>
        private bool IsLeft(Point3D p0, Point3D p1, Point3D p2)
        {
            return ((p1.X - p0.X) * (p2.Y - p0.Y) -
                    (p2.X - p0.X) * (p1.Y - p0.Y)) > 0;
        }

        /// <summary>
        /// Find direct parent (để loại bỏ indirect relationships)
        /// </summary>
        private CuttingProfile FindDirectParent(
            CuttingProfile child,
            List<CuttingProfile> allProfiles)
        {
            CuttingProfile directParent = null;
            double minArea = double.MaxValue;

            foreach (var candidate in allProfiles)
            {
                if (candidate.ProfileId == child.ProfileId)
                    continue;

                if (IsProfileInside(child, candidate))
                {
                    double area = (candidate.BoundingBoxMax.X - candidate.BoundingBoxMin.X) *
                                 (candidate.BoundingBoxMax.Y - candidate.BoundingBoxMin.Y);

                    if (area < minArea)
                    {
                        minArea = area;
                        directParent = candidate;
                    }
                }
            }

            return directParent;
        }

        #endregion

        #region Step 9: Optimize Cutting Order

        /// <summary>
        /// Tối ưu thứ tự cắt
        /// </summary>
        private void OptimizeCuttingOrder(
            List<CuttingProfile> profiles,
            CuttingDirectionSettings settings)
        {
            // Sort by hierarchy first (inner profiles first)
            var sortedProfiles = new List<CuttingProfile>();

            // Add profiles by depth (deepest first)
            int maxDepth = GetMaxDepth(profiles);

            for (int depth = maxDepth; depth >= 0; depth--)
            {
                var profilesAtDepth = GetProfilesAtDepth(profiles, depth);

                // Sort profiles at same depth by Y position
                profilesAtDepth = OptimizeSequenceAtSameLevel(profilesAtDepth, settings);

                sortedProfiles.AddRange(profilesAtDepth);
            }

            // Assign cutting order
            for (int i = 0; i < sortedProfiles.Count; i++)
            {
                sortedProfiles[i].CuttingOrder = i;

                // Determine optimal start point for closed profiles
                if (sortedProfiles[i].IsClosed)
                {
                    sortedProfiles[i].OptimalStartPoint =
                        FindOptimalStartPoint(sortedProfiles[i], settings);
                }

                // Set cutting direction
                sortedProfiles[i].PreferredDirection =
                    DetermineOptimalDirection(sortedProfiles[i], settings);
            }
        }

        /// <summary>
        /// Get max depth trong hierarchy
        /// </summary>
        private int GetMaxDepth(List<CuttingProfile> profiles)
        {
            int maxDepth = 0;

            foreach (var profile in profiles)
            {
                int depth = GetProfileDepth(profile, profiles);
                maxDepth = Math.Max(maxDepth, depth);
            }

            return maxDepth;
        }

        /// <summary>
        /// Get depth của một profile
        /// </summary>
        private int GetProfileDepth(CuttingProfile profile, List<CuttingProfile> allProfiles)
        {
            if (!profile.ParentProfileId.HasValue)
                return 0;

            var parent = allProfiles.FirstOrDefault(p => p.ProfileId == profile.ParentProfileId);
            if (parent == null)
                return 0;

            return 1 + GetProfileDepth(parent, allProfiles);
        }

        /// <summary>
        /// Get profiles at specific depth
        /// </summary>
        private List<CuttingProfile> GetProfilesAtDepth(
            List<CuttingProfile> profiles,
            int targetDepth)
        {
            return profiles
                .Where(p => GetProfileDepth(p, profiles) == targetDepth)
                .ToList();
        }

        /// <summary>
        /// Optimize sequence của profiles ở cùng level
        /// </summary>
        private List<CuttingProfile> OptimizeSequenceAtSameLevel(
            List<CuttingProfile> profiles,
            CuttingDirectionSettings settings)
        {
            if (profiles.Count <= 1)
                return profiles;

            var optimized = new List<CuttingProfile>();
            var remaining = new List<CuttingProfile>(profiles);

            // Start with profile có Y nhỏ nhất
            var current = remaining.OrderBy(p => p.MinY).First();
            optimized.Add(current);
            remaining.Remove(current);

            // Nearest neighbor
            while (remaining.Any())
            {
                var lastPoint = GetLastPointOfProfile(current);

                current = remaining
                    .OrderBy(p => DistanceToProfile(lastPoint, p))
                    .First();

                optimized.Add(current);
                remaining.Remove(current);
            }

            return optimized;
        }

        /// <summary>
        /// Get last point của profile
        /// </summary>
        private Point3D GetLastPointOfProfile(CuttingProfile profile)
        {
            var lastEdge = profile.Edges.Last();
            var lastPoint = lastEdge.Toolpath.Points.Last();
            return new Point3D(lastPoint.C, lastPoint.Y, 0);
        }

        /// <summary>
        /// Calculate distance từ point đến profile
        /// </summary>
        private double DistanceToProfile(Point3D point, CuttingProfile profile)
        {
            var firstEdge = profile.Edges.First();
            var firstPoint = firstEdge.Toolpath.Points.First();
            var profileStart = new Point3D(firstPoint.C, firstPoint.Y, 0);

            return (point - profileStart).Length;
        }

        /// <summary>
        /// Find optimal start point cho closed profile
        /// </summary>
        private Point3D FindOptimalStartPoint(
            CuttingProfile profile,
            CuttingDirectionSettings settings)
        {
            // Strategy 1: Start at lowest Y
            if (settings.YDirection == CuttingDirectionSettings.YDirectionPreference.AlwaysPositive)
            {
                var lowestPoint = profile.Edges
                    .SelectMany(e => e.Toolpath.Points)
                    .OrderBy(p => p.Y)
                    .First();

                return new Point3D(lowestPoint.C, lowestPoint.Y, 0);
            }

            // Strategy 2: Start at highest Y
            if (settings.YDirection == CuttingDirectionSettings.YDirectionPreference.AlwaysNegative)
            {
                var highestPoint = profile.Edges
                    .SelectMany(e => e.Toolpath.Points)
                    .OrderByDescending(p => p.Y)
                    .First();

                return new Point3D(highestPoint.C, highestPoint.Y, 0);
            }

            // Strategy 3: Start at longest straight section (for better lead-in)
            EdgeNode longestLine = null;
            double maxLength = 0;

            foreach (var edge in profile.Edges)
            {
                if (edge.EdgeInfo.Type == "Line" && edge.EdgeInfo.Length > maxLength)
                {
                    maxLength = edge.EdgeInfo.Length;
                    longestLine = edge;
                }
            }

            if (longestLine != null)
            {
                var midPoint = longestLine.Toolpath.Points[longestLine.Toolpath.Points.Count / 2];
                return new Point3D(midPoint.C, midPoint.Y, 0);
            }

            // Default: start at first edge
            var defaultPoint = profile.Edges.First().Toolpath.Points.First();
            return new Point3D(defaultPoint.C, defaultPoint.Y, 0);
        }

        /// <summary>
        /// Determine optimal cutting direction
        /// </summary>
        private CuttingDirection DetermineOptimalDirection(
            CuttingProfile profile,
            CuttingDirectionSettings settings)
        {
            switch (settings.YDirection)
            {
                case CuttingDirectionSettings.YDirectionPreference.AlwaysPositive:
                    return CuttingDirection.YPositive;

                case CuttingDirectionSettings.YDirectionPreference.AlwaysNegative:
                    return CuttingDirection.YNegative;

                case CuttingDirectionSettings.YDirectionPreference.Alternating:
                    // Alternate based on cutting order
                    return (profile.CuttingOrder % 2 == 0) ?
                        CuttingDirection.YPositive :
                        CuttingDirection.YNegative;

                default: // Auto
                    // Choose based on next profile position
                    return CuttingDirection.YPositive;
            }
        }

        #endregion
    }

    /// <summary>
    /// Settings cho cutting direction
    /// </summary>
    public class CuttingDirectionSettings
    {
        public enum YDirectionPreference
        {
            Auto,           // Tự động chọn dựa trên tối ưu
            AlwaysPositive, // Luôn cắt từ -Y đến +Y
            AlwaysNegative, // Luôn cắt từ +Y đến -Y
            Alternating     // Xen kẽ để giảm di chuyển
        }

        public YDirectionPreference YDirection { get; set; } = YDirectionPreference.Auto;
        public bool CompleteProfileBeforeMoving { get; set; } = true;
        public bool OptimizeStartPoint { get; set; } = true;
        public double ProfileConnectionTolerance { get; set; } = 0.1; // mm
        public bool UseLeadInOut { get; set; } = true;
        public double LeadInLength { get; set; } = 2.0; // mm
        public double LeadOutLength { get; set; } = 2.0; // mm
    }
}
