#include "StepReader.h"
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>
#include <iostream>
#include <limits>
#include <cmath>
#include <Geom_BezierCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <GeomAPI_IntCS.hxx>
#include <Geom_Plane.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

// Additional includes needed
#include <set>
#include <stack>
#include <queue>
#include <algorithm>
#include <sstream>
#include <iomanip>

// Additional OpenCASCADE includes
#include <ElCLib.hxx>
#include <BRepTools.hxx>
#include <Precision.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <TopoDS_Face.hxx>
#include <Geom_Surface.hxx>

#define _USE_MATH_DEFINES
#include <cmath>

// Custom comparator for gp_Pnt to be used in std::map
struct GpPntComparator {
    bool operator()(const gp_Pnt& a, const gp_Pnt& b) const {
        if (std::abs(a.X() - b.X()) > Precision::Confusion()) return a.X() < b.X();
        if (std::abs(a.Y() - b.Y()) > Precision::Confusion()) return a.Y() < b.Y();
        if (std::abs(a.Z() - b.Z()) > Precision::Confusion()) return a.Z() < b.Z();
        return false;
    }
};

// Helper function to convert EdgeType to ShapeType
GeometryKernel::EdgeClassification::ShapeType ConvertEdgeType(GeometryKernel::EdgeInfo::EdgeType type) {
    switch (type) {
    case GeometryKernel::EdgeInfo::LINE:
        return GeometryKernel::EdgeClassification::ShapeType::LINE_SEGMENT;
    case GeometryKernel::EdgeInfo::CIRCLE:
        return GeometryKernel::EdgeClassification::ShapeType::CIRCLE_SHAPE;
    case GeometryKernel::EdgeInfo::BSPLINE:
        return GeometryKernel::EdgeClassification::ShapeType::FREEFORM_SHAPE;
    case GeometryKernel::EdgeInfo::BEZIER:
        return GeometryKernel::EdgeClassification::ShapeType::FREEFORM_SHAPE;
    default:
        return GeometryKernel::EdgeClassification::ShapeType::UNKNOWN_SHAPE;
    }
}

namespace GeometryKernel {

    StepReader::StepReader() : m_shape(nullptr) {}

    StepReader::~StepReader() {}

    bool StepReader::LoadFile(const std::string& filePath) {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str());

        if (status != IFSelect_RetDone) {
            return false;
        }

        reader.TransferRoots();
        m_shape = std::make_unique<TopoDS_Shape>(reader.OneShape());

        m_edges.clear();
        m_topoEdgeMap.clear();
        TopExp_Explorer edgeExplorer;
        int edgeIdCounter = 0;
        for (edgeExplorer.Init(*m_shape, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
            TopoDS_Edge currentEdge = TopoDS::Edge(edgeExplorer.Current());
            m_topoEdgeMap[edgeIdCounter] = currentEdge;
            BRepAdaptor_Curve curveAdaptor(currentEdge);
            EdgeInfo edgeInfo;
            edgeInfo.id = edgeIdCounter++;
            edgeInfo.length = GCPnts_AbscissaPoint::Length(curveAdaptor);

            switch (curveAdaptor.GetType()) {
            case GeomAbs_Line:
                edgeInfo.type = EdgeInfo::LINE;
                break;
            case GeomAbs_Circle:
                edgeInfo.type = EdgeInfo::CIRCLE;
                break;
            case GeomAbs_BSplineCurve:
                edgeInfo.type = EdgeInfo::BSPLINE;
                break;
            case GeomAbs_BezierCurve:
                edgeInfo.type = EdgeInfo::BEZIER;
                break;
            default:
                edgeInfo.type = EdgeInfo::OTHER;
                break;
            }
            m_edges.push_back(edgeInfo);
        }
        return true;
    }

    int StepReader::GetEdgeCount() const {
        if (!m_shape || m_shape->IsNull()) {
            return 0;
        }
        return static_cast<int>(m_edges.size());
    }

    std::vector<EdgeInfo> StepReader::GetEdgeInfoList() const {
        // LỌC edges INTERNAL trước khi trả về
        std::vector<EdgeInfo> filteredEdges;
        auto classifications = ClassifyEdges();


        for (const auto& edge : m_edges) {
            auto it = classifications.find(edge.id);
            if (it != classifications.end() && it->second.location == EdgeClassification::INTERNAL) {
                continue; // Bỏ qua edges INTERNAL
            }
            filteredEdges.push_back(edge);
        }

        return filteredEdges;
    }

    void StepReader::ExtractWireframeData(std::vector<double>& vertices, std::vector<int>& indices) const {
        if (!m_shape || m_shape->IsNull()) {
            return;
        }

        vertices.clear();
        indices.clear();
        std::map<gp_Pnt, int, GpPntComparator> vertexMap;
        int vertexIndexCounter = 0;

        // QUAN TRỌNG: Lọc edges trước khi tạo wireframe
        auto classifications = ClassifyEdges();

        TopExp_Explorer edgeExplorer;
        int edgeId = 0;

        for (edgeExplorer.Init(*m_shape, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next(), edgeId++) {
            // Kiểm tra classification
            auto classIt = classifications.find(edgeId);
            if (classIt != classifications.end()) {
                // CHỈ LẤY edges trên bề mặt ngoài hoặc không phải INTERNAL
                // INTERNAL bao gồm cả edges bên trong VÀ axial lines
                if (classIt->second.location == EdgeClassification::INTERNAL) {
                    continue; // Bỏ qua edges bên trong
                }
            }

            TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
            Standard_Real firstParam, lastParam;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, firstParam, lastParam);

            if (curve.IsNull()) continue;

            gp_Pnt startPoint = curve->Value(firstParam);
            gp_Pnt endPoint = curve->Value(lastParam);

            int startIndex, endIndex;

            auto itStart = vertexMap.find(startPoint);
            if (itStart == vertexMap.end()) {
                vertexMap[startPoint] = vertexIndexCounter;
                vertices.push_back(startPoint.X());
                vertices.push_back(startPoint.Y());
                vertices.push_back(startPoint.Z());
                startIndex = vertexIndexCounter++;
            }
            else {
                startIndex = itStart->second;
            }

            auto itEnd = vertexMap.find(endPoint);
            if (itEnd == vertexMap.end()) {
                vertexMap[endPoint] = vertexIndexCounter;
                vertices.push_back(endPoint.X());
                vertices.push_back(endPoint.Y());
                vertices.push_back(endPoint.Z());
                endIndex = vertexIndexCounter++;
            }
            else {
                endIndex = itEnd->second;
            }

            indices.push_back(startIndex);
            indices.push_back(endIndex);
        }
    }

    // Hàm tìm axis của ống dựa trên circles hoặc phân tích hình học
    TubeAxisInfo StepReader::AnalyzeTubeAxis() const {
        TubeAxisInfo axisInfo;
        if (!m_shape || m_shape->IsNull()) return axisInfo;

        // Bước 1: Tìm tất cả circles và phân loại theo vị trí Z (hoặc hướng chính)
        std::vector<std::pair<gp_Pnt, double>> circleData; // center, radius
        std::map<double, std::vector<std::pair<gp_Pnt, double>>> circlesByZ;

        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

            if (!curve.IsNull() && curve->IsKind(STANDARD_TYPE(Geom_Circle))) {
                Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
                gp_Pnt center = circle->Location();
                double radius = circle->Radius();

                // Phân loại theo Z (hoặc tọa độ dominan)
                double zCoord = center.Z();
                circlesByZ[zCoord].push_back(std::make_pair(center, radius));
                circleData.push_back(std::make_pair(center, radius));
            }
        }

        // Bước 2: Tìm 2 nhóm circles ở 2 đầu
        if (circlesByZ.size() >= 2) {
            // Lấy nhóm đầu và cuối
            auto itFirst = circlesByZ.begin();
            auto itLast = std::prev(circlesByZ.end());

            // Tính tâm trung bình cho mỗi nhóm
            gp_Pnt startCenter(0, 0, 0);
            gp_Pnt endCenter(0, 0, 0);
            int countStart = 0, countEnd = 0;

            for (const auto& circle : itFirst->second) {
                startCenter.SetX(startCenter.X() + circle.first.X());
                startCenter.SetY(startCenter.Y() + circle.first.Y());
                startCenter.SetZ(startCenter.Z() + circle.first.Z());
                countStart++;
            }

            for (const auto& circle : itLast->second) {
                endCenter.SetX(endCenter.X() + circle.first.X());
                endCenter.SetY(endCenter.Y() + circle.first.Y());
                endCenter.SetZ(endCenter.Z() + circle.first.Z());
                countEnd++;
            }

            if (countStart > 0 && countEnd > 0) {
                startCenter.SetX(startCenter.X() / countStart);
                startCenter.SetY(startCenter.Y() / countStart);
                startCenter.SetZ(startCenter.Z() / countStart);

                endCenter.SetX(endCenter.X() / countEnd);
                endCenter.SetY(endCenter.Y() / countEnd);
                endCenter.SetZ(endCenter.Z() / countEnd);

                // Tính axis direction
                gp_Vec axisVec(startCenter, endCenter);
                if (axisVec.Magnitude() > Precision::Confusion()) {
                    axisInfo.isValid = true;
                    axisInfo.startCenter = startCenter;
                    axisInfo.endCenter = endCenter;
                    axisInfo.axisDirection = gp_Dir(axisVec);
                    std::cout << "Axis from "
                        << "(" << startCenter.X() << "," << startCenter.Y() << "," << startCenter.Z() << ") to "
                        << "(" << endCenter.X() << "," << endCenter.Y() << "," << endCenter.Z() << ")"
                        << std::endl;
                }
            }
        }

        // Bước 3: Nếu không tìm được từ circles, thử phương pháp khác
        if (!axisInfo.isValid && !circleData.empty()) {
            // Phân tích Principal Component Analysis (PCA) đơn giản
            // Tìm hướng chính của tất cả circle centers
            gp_Pnt centroid(0, 0, 0);
            for (const auto& circle : circleData) {
                centroid.SetX(centroid.X() + circle.first.X());
                centroid.SetY(centroid.Y() + circle.first.Y());
                centroid.SetZ(centroid.Z() + circle.first.Z());
            }
            centroid.SetX(centroid.X() / circleData.size());
            centroid.SetY(centroid.Y() / circleData.size());
            centroid.SetZ(centroid.Z() / circleData.size());

            // Tính ma trận hiệp phương sai
            double cov[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
            for (const auto& circle : circleData) {
                double dx = circle.first.X() - centroid.X();
                double dy = circle.first.Y() - centroid.Y();
                double dz = circle.first.Z() - centroid.Z();

                cov[0][0] += dx * dx;
                cov[0][1] += dx * dy;
                cov[0][2] += dx * dz;
                cov[1][1] += dy * dy;
                cov[1][2] += dy * dz;
                cov[2][2] += dz * dz;
            }

            // Ma trận đối xứng
            cov[1][0] = cov[0][1];
            cov[2][0] = cov[0][2];
            cov[2][1] = cov[1][2];

            // Tìm eigenvector lớn nhất (phương pháp power iteration đơn giản)
            double v[3] = { 1, 0, 0 };
            for (int iter = 0; iter < 10; iter++) {
                double new_v[3] = { 0, 0, 0 };
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        new_v[i] += cov[i][j] * v[j];
                    }
                }

                double norm = sqrt(new_v[0] * new_v[0] + new_v[1] * new_v[1] + new_v[2] * new_v[2]);
                if (norm > Precision::Confusion()) {
                    v[0] = new_v[0] / norm;
                    v[1] = new_v[1] / norm;
                    v[2] = new_v[2] / norm;
                }
            }

            axisInfo.isValid = true;
            axisInfo.axisDirection = gp_Dir(v[0], v[1], v[2]);
            axisInfo.startCenter = centroid;
            axisInfo.endCenter = centroid;
        }

        return axisInfo;
    }

    CylinderInfo StepReader::DetectCylinder() const {
        CylinderInfo info;
        if (!m_shape || m_shape->IsNull()) return info;

        TopExp_Explorer faceExplorer;
        for (faceExplorer.Init(*m_shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
            TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)) {
                Handle(Geom_CylindricalSurface) cylSurface = Handle(Geom_CylindricalSurface)::DownCast(surface);
                info.isValid = true;
                info.radius = cylSurface->Radius();

                gp_Ax3 axis = cylSurface->Position();
                gp_Dir dir = axis.Direction();
                gp_Pnt loc = axis.Location();

                info.axisX = dir.X();
                info.axisY = dir.Y();
                info.axisZ = dir.Z();
                info.centerX = loc.X();
                info.centerY = loc.Y();
                info.centerZ = loc.Z();

                Standard_Real uMin, uMax, vMin, vMax;
                BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
                info.length = std::abs(vMax - vMin);

                return info;
            }
        }
        return info;
    }

    ThickCylinderInfo StepReader::DetectThickCylinder() const {
        ThickCylinderInfo info;
        if (!m_shape || m_shape->IsNull()) return info;

        std::vector<double> cylinderRadii;
        gp_Ax1 commonAxis;
        bool firstCylinder = true;

        // Tìm tất cả cylindrical surfaces
        TopExp_Explorer faceExplorer;
        for (faceExplorer.Init(*m_shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
            TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

            if (surface->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)) {
                Handle(Geom_CylindricalSurface) cylSurface = Handle(Geom_CylindricalSurface)::DownCast(surface);
                double radius = cylSurface->Radius();

                gp_Ax3 axis = cylSurface->Position();
                gp_Ax1 currentAxis(axis.Location(), axis.Direction());

                // Kiểm tra xem các cylinder có cùng trục không
                if (firstCylinder) {
                    commonAxis = currentAxis;
                    firstCylinder = false;
                    cylinderRadii.push_back(radius);
                }
                else {
                    // So sánh trục (với tolerance)
                    if (currentAxis.Direction().IsParallel(commonAxis.Direction(), 0.001) &&
                        currentAxis.Location().Distance(commonAxis.Location()) < 0.1) {
                        cylinderRadii.push_back(radius);
                    }
                }
            }
        }

        // Nếu tìm thấy 2 cylinder cùng trục
        if (cylinderRadii.size() >= 2) {
            std::sort(cylinderRadii.begin(), cylinderRadii.end());
            info.hasThickness = true;
            info.innerRadius = cylinderRadii[0];
            info.outerRadius = cylinderRadii[cylinderRadii.size() - 1];

            // Lấy thông tin trục và tâm
            gp_Dir dir = commonAxis.Direction();
            gp_Pnt loc = commonAxis.Location();
            info.axisX = dir.X();
            info.axisY = dir.Y();
            info.axisZ = dir.Z();
            info.centerX = loc.X();
            info.centerY = loc.Y();
            info.centerZ = loc.Z();

            // Tính chiều dài (giống như DetectCylinder)
            TopExp_Explorer faceExp(*m_shape, TopAbs_FACE);
            for (; faceExp.More(); faceExp.Next()) {
                TopoDS_Face face = TopoDS::Face(faceExp.Current());
                Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
                if (surface->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)) {
                    Standard_Real uMin, uMax, vMin, vMax;
                    BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
                    info.length = std::abs(vMax - vMin);
                    break;
                }
            }
        }

        return info;
    }

    bool StepReader::GetEdgeGeometry(int edgeId, std::vector<double>& points) const {
        if (!m_shape || m_shape->IsNull() || edgeId < 0) {
            return false;
        }

        TopExp_Explorer edgeExplorer;
        int currentId = 0;
        for (edgeExplorer.Init(*m_shape, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
            if (currentId == edgeId) {
                TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
                BRepAdaptor_Curve curveAdaptor(edge);

                Standard_Real firstParam = curveAdaptor.FirstParameter();
                Standard_Real lastParam = curveAdaptor.LastParameter();

                const int numSamples = 60;
                points.clear();

                for (int i = 0; i <= numSamples; ++i) {
                    Standard_Real param = firstParam + (lastParam - firstParam) * (Standard_Real)i / numSamples;
                    gp_Pnt pt = curveAdaptor.Value(param);
                    points.push_back(pt.X());
                    points.push_back(pt.Y());
                    points.push_back(pt.Z());
                }
                return true;
            }
            currentId++;
        }
        return false;
    }

    // Hàm tính khoảng cách từ edge đến axis
    double StepReader::ComputeDistanceToAxis(const TopoDS_Edge& edge,
        const gp_Ax1& axis) const {
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull()) return -1.0;

        // Sample nhiều điểm trên edge
        const int numSamples = 60;
        double totalDistance = 0.0;
        int validSamples = 0;

        for (int i = 0; i <= numSamples; i++) {
            double param = first + (last - first) * i / numSamples;
            gp_Pnt pt = curve->Value(param);

            // Tính khoảng cách từ điểm đến axis
            double dist = pt.Distance(axis.Location());
            gp_Vec v(axis.Location(), pt);
            double projLength = v.Dot(gp_Vec(axis.Direction()));
            gp_Pnt projectedPt = axis.Location();
            projectedPt.Translate(projLength * gp_Vec(axis.Direction()));

            double perpDist = pt.Distance(projectedPt);
            totalDistance += perpDist;
            validSamples++;
        }

        return validSamples > 0 ? totalDistance / validSamples : -1.0;
    }

    std::map<int, EdgeClassification> StepReader::ClassifyEdges() const {

        std::map<int, EdgeClassification> classifications;

        if (!m_shape || m_shape->IsNull()) return classifications;

        // Phân tích axis của ống
        TubeAxisInfo tubeAxis = AnalyzeTubeAxis();

        if (!tubeAxis.isValid) {
            // Fallback về phương pháp cũ
            CylinderInfo cylInfo = DetectCylinder();
            if (!cylInfo.isValid) return classifications;

            // Sử dụng cylinder info để tạo axis
            gp_Pnt center(cylInfo.centerX, cylInfo.centerY, cylInfo.centerZ);
            gp_Dir dir(cylInfo.axisX, cylInfo.axisY, cylInfo.axisZ);
            tubeAxis.isValid = true;
            tubeAxis.axisDirection = dir;
            tubeAxis.startCenter = center;
        }

        // Tạo axis từ tube info
        gp_Ax1 centralAxis(tubeAxis.startCenter, tubeAxis.axisDirection);

        // Bước 1: Tính khoảng cách của tất cả edges đến axis
        std::map<int, double> edgeDistances;
        std::vector<double> allDistances;

        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int edgeId = 0;

        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            double distance = ComputeDistanceToAxis(edge, centralAxis);

            if (distance > 0) {
                edgeDistances[edgeId] = distance;
                allDistances.push_back(distance);
            }
        }

        // Bước 2: Phân tích phân bố khoảng cách để tìm ngưỡng
        if (!allDistances.empty()) {
            std::sort(allDistances.begin(), allDistances.end());

            std::vector<std::pair<double, int>> radiusGroups; // radius, count
            double currentRadius = allDistances[0];
            int currentCount = 1;

            const double GROUPING_TOLERANCE = 0.5; // 0.5mm cho nhóm

            for (size_t i = 1; i < allDistances.size(); i++) {
                if (std::abs(allDistances[i] - currentRadius) < GROUPING_TOLERANCE) {
                    currentRadius = (currentRadius * currentCount + allDistances[i]) / (currentCount + 1); // Tính trung bình
                    currentCount++;
                }
                else {
                    radiusGroups.push_back(std::make_pair(currentRadius, currentCount));
                    currentRadius = allDistances[i];
                    currentCount = 1;
                }
            }
            radiusGroups.push_back(std::make_pair(currentRadius, currentCount));

            // Debug output
            std::cout << "Debug - Found radius groups:" << std::endl;
            for (const auto& group : radiusGroups) {
                std::cout << "  Radius: " << group.first << " mm, Count: " << group.second << std::endl;
            }

            // Tìm bán kính lớn nhất (mặt ngoài)
            double maxRadius = 0.0;
            double secondMaxRadius = 0.0;

            // Sắp xếp theo bán kính
            std::sort(radiusGroups.begin(), radiusGroups.end(),
                [](const auto& a, const auto& b) { return a.first > b.first; });

            if (!radiusGroups.empty()) {
                maxRadius = radiusGroups[0].first;
                if (radiusGroups.size() > 1) {
                    secondMaxRadius = radiusGroups[1].first;
                }
            }

            std::cout << "Max radius (outer surface): " << maxRadius << " mm" << std::endl;
            if (secondMaxRadius > 0) {
                std::cout << "Inner radius: " << secondMaxRadius << " mm" << std::endl;
                std::cout << "Wall thickness: " << (maxRadius - secondMaxRadius) << " mm" << std::endl;
            }

            // Tính tolerance động dựa trên độ dày thành ống
            double classificationTolerance = 0.5; // Mặc định
            if (secondMaxRadius > 0 && (maxRadius - secondMaxRadius) < 5.0) {
                // Nếu độ dày < 5mm, dùng tolerance nhỏ hơn
                classificationTolerance = (maxRadius - secondMaxRadius) * 0.2; // 20% độ dày
                classificationTolerance = std::max(0.1, classificationTolerance); // Tối thiểu 0.1mm
            }
            std::cout << "Using classification tolerance: " << classificationTolerance << " mm" << std::endl;


            // Bước 3: Phân loại edges
            edgeId = 0;
            edgeExp.Init(*m_shape, TopAbs_EDGE);

            for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
                EdgeClassification classification;
                classification.location = EdgeClassification::UNKNOWN;
                classification.shapeType = EdgeClassification::UNKNOWN_SHAPE;
                classification.groupId = -1;
                classification.originalEdgeId = edgeId;

                auto distIt = edgeDistances.find(edgeId);
                if (distIt != edgeDistances.end()) {
                    classification.distanceFromAxis = distIt->second;
                    // Debug output
                    if (edgeId < 5) { // In ra 5 edges đầu tiên
                        std::cout << "Edge " << edgeId << ": distance=" << distIt->second
                            << ", maxRadius=" << maxRadius
                            << ", diff=" << std::abs(distIt->second - maxRadius) << std::endl;
                    }

                    // Kiểm tra xem edge có thuộc nhóm bán kính lớn nhất không
                    if (std::abs(distIt->second - maxRadius) < classificationTolerance) {
                        classification.location = EdgeClassification::ON_CYLINDER_SURFACE;
                        classification.isOnCylinderSurface = true;
                    }
                    else {
                        classification.location = EdgeClassification::INTERNAL;
                        classification.isOnCylinderSurface = false;
                    }

                    // Phân loại shape type
                    TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
                    Standard_Real first, last;
                    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

                    if (!curve.IsNull()) {
                        if (curve->IsKind(STANDARD_TYPE(Geom_Circle))) {
                            classification.shapeType = EdgeClassification::CIRCLE_SHAPE;
                        }
                        else if (curve->IsKind(STANDARD_TYPE(Geom_Line))) {
                            classification.shapeType = EdgeClassification::LINE_SEGMENT;
                        }
                        else if (curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) {
                            classification.shapeType = EdgeClassification::FREEFORM_SHAPE;
                        }
                        else if (curve->IsKind(STANDARD_TYPE(Geom_BezierCurve))) {
                            classification.shapeType = EdgeClassification::FREEFORM_SHAPE;
                        }
                    }
                }

                classifications[edgeId] = classification;
                if (m_edges[edgeId].type == EdgeInfo::BSPLINE) {
                    std::cout << "Debug BSpline #" << edgeId
                        << ": distance=" << distIt->second
                        << ", location=" << (classification.location == EdgeClassification::INTERNAL ? "INTERNAL" : "ON_SURFACE")
                        << ", length=" << m_edges[edgeId].length << " mm" << std::endl;
                }


            }
        }
        // THÊM: Phát hiện và đánh dấu axial lines
        std::vector<int> axialLines = DetectAxialLines();
        if (!axialLines.empty()) {
            std::cout << "Debug - Found " << axialLines.size() << " axial lines" << std::endl;

            // Nhóm các lines đồng tâm
            auto collinearGroups = GroupCollinearLines(axialLines);
            std::cout << "Debug - Grouped into " << collinearGroups.size() << " collinear groups" << std::endl;

            // Đánh dấu tất cả axial lines là INTERNAL để ẩn chúng
            for (int lineId : axialLines) {
                if (classifications.find(lineId) != classifications.end()) {
                    classifications[lineId].location = EdgeClassification::INTERNAL;
                    std::cout << "Debug - Marked edge " << lineId << " as INTERNAL (axial line)" << std::endl;
                }
            }
        }
        // Phát hiện BSpline đồng trục
        auto collinearBSplines = DetectCollinearBSplines();
        for (int bsplineId : collinearBSplines) {
            classifications[bsplineId].location = EdgeClassification::INTERNAL;
            std::cout << "Marked collinear BSpline #" << bsplineId << " as INTERNAL" << std::endl;
        }
        // Ẩn BSpline thẳng dọc trục (generator lines)
        edgeId = 0;
        edgeExp.Init(*m_shape, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            if (edgeId >= m_edges.size()) break;
            if (m_edges[edgeId].type != EdgeInfo::BSPLINE) continue;

            auto it = classifications.find(edgeId);
            if (it == classifications.end()) continue;

            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            if (!IsBSplineStraight(edge, 0.1)) continue;

            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
            gp_Pnt p1 = curve->Value(first);
            gp_Pnt p2 = curve->Value(last);

            if (std::abs(p1.X() - p2.X()) < 0.1 &&
                std::abs(p1.Y() - p2.Y()) < 0.1 &&
                std::abs(p1.Z() - p2.Z()) > 100.0) {

                it->second.location = EdgeClassification::INTERNAL;
                std::cout << "Marked vertical BSpline #" << edgeId
                    << " as INTERNAL (Z length=" << std::abs(p1.Z() - p2.Z()) << ")" << std::endl;
            }
        }
        return classifications;
    }

    std::vector<int> StepReader::DetectAxialLines() const {
        std::vector<int> axialLines;
        if (!m_shape || m_shape->IsNull()) return axialLines;

        CylinderInfo cylInfo = DetectCylinder();
        if (!cylInfo.isValid) return axialLines;

        gp_Dir cylinderAxis(cylInfo.axisX, cylInfo.axisY, cylInfo.axisZ);

        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int edgeId = 0;

        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());

            // Xét cả LINE và BSPLINE
            if (m_edges[edgeId].type == EdgeInfo::LINE) {
                Standard_Real first, last;
                Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

                if (!curve.IsNull() && curve->IsKind(STANDARD_TYPE(Geom_Line))) {
                    Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve);
                    gp_Dir lineDir = line->Position().Direction();

                    double dotProduct = std::abs(lineDir.Dot(cylinderAxis));
                    if (dotProduct > 0.99) {
                        axialLines.push_back(edgeId);
                        std::cout << "Found axial LINE edge: " << edgeId << std::endl;
                    }
                }
            }
            else if (m_edges[edgeId].type == EdgeInfo::BSPLINE) {
                std::cout << "Checking BSpline #" << edgeId << ": ";
                if (IsBSplineStraight(edge, 0.1)) {
                    gp_Dir bsplineDir;
                    if (GetBSplineDirection(edge, bsplineDir)) {
                        double dotProduct = std::abs(bsplineDir.Dot(cylinderAxis));
                        std::cout << "STRAIGHT, dot=" << dotProduct;
                        if (dotProduct > 0.9) {
                            std::cout << " -> AXIAL";
                        }
                    }
                }
                else {
                    std::cout << "NOT STRAIGHT";
                }
                std::cout << std::endl;


                // Kiểm tra BSpline có phải đường thẳng không
                if (IsBSplineStraight(edge)) {
                    gp_Dir bsplineDir;
                    if (GetBSplineDirection(edge, bsplineDir)) {
                        double dotProduct = std::abs(bsplineDir.Dot(cylinderAxis));
                        if (dotProduct > 0.99) {
                            axialLines.push_back(edgeId);
                            std::cout << "Found axial BSPLINE edge: " << edgeId << std::endl;
                        }
                    }
                }
            }
        }

        std::cout << "Total axial edges found: " << axialLines.size() << std::endl;
        return axialLines;
    }

    std::vector<int> StepReader::DetectCollinearBSplines() const {
        std::vector<int> collinearBSplines;
        std::map<std::string, std::vector<int>> axisGroups;

        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int edgeId = 0;

        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            if (m_edges[edgeId].type != EdgeInfo::BSPLINE) continue;

            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());
            if (!IsBSplineStraight(edge, 0.1)) continue;

            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
            gp_Pnt p1 = curve->Value(first);
            gp_Pnt p2 = curve->Value(last);

            // Tạo key
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << round(p1.X() * 10) / 10
                << "," << round(p1.Y() * 10) / 10;
            std::string key = ss.str();

            axisGroups[key].push_back(edgeId);  // Sửa lỗi ở đây
        }

        for (const auto& group : axisGroups) {
            if (group.second.size() >= 3) {
                std::cout << "Found " << group.second.size()
                    << " collinear BSplines at " << group.first << std::endl;
                collinearBSplines.insert(collinearBSplines.end(),
                    group.second.begin(), group.second.end());
            }
        }

        return collinearBSplines;
    }



    bool StepReader::AreLinesCollinear(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2,
        double tolerance) const {
        Standard_Real first1, last1, first2, last2;
        Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, first1, last1);
        Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, first2, last2);

        if (curve1.IsNull() || curve2.IsNull()) return false;
        if (!curve1->IsKind(STANDARD_TYPE(Geom_Line)) ||
            !curve2->IsKind(STANDARD_TYPE(Geom_Line))) return false;

        Handle(Geom_Line) line1 = Handle(Geom_Line)::DownCast(curve1);
        Handle(Geom_Line) line2 = Handle(Geom_Line)::DownCast(curve2);

        // Kiểm tra hướng
        gp_Dir dir1 = line1->Position().Direction();
        gp_Dir dir2 = line2->Position().Direction();

        if (!dir1.IsParallel(dir2, tolerance)) return false;

        // Kiểm tra điểm trên line1 có nằm trên line2 không
        gp_Pnt p1 = curve1->Value(first1);
        gp_Lin lin2(line2->Position());

        return lin2.Distance(p1) < tolerance;
    }

    bool StepReader::IsBSplineStraight(const TopoDS_Edge& edge, double tolerance) const {
        BSplineStraightInfo info;
        return IsBSplineStraightDetailed(edge, tolerance, info);
    }
    bool StepReader::IsBSplineStraightDetailed(const TopoDS_Edge& edge,
        double tolerance,
        BSplineStraightInfo& info) const {

        // Initialize info
        info.isStaight = false;
        info.maxDeviation = 0.0;
        info.maxCurvature = 0.0;
        info.straightnessScore = 0.0;
        info.length = 0.0;

        try {
            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

            if (curve.IsNull() || !curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) {
                info.reason = "Not a BSpline curve";
                return false;
            }

            Handle(Geom_BSplineCurve) bspline = Handle(Geom_BSplineCurve)::DownCast(curve);

            // THÊM: Kiểm tra validity của BSpline handle
            if (bspline.IsNull()) {
                info.reason = "Invalid BSpline handle";
                return false;
            }

            // THÊM: Kiểm tra parameter range
            if (first >= last || !std::isfinite(first) || !std::isfinite(last)) {
                info.reason = "Invalid parameter range";
                return false;
            }

            // 1. Check control points collinearity
            int numPoles = 0;
            try {
                numPoles = bspline->NbPoles();
            }
            catch (...) {
                info.reason = "Cannot get number of poles";
                return false;
            }

            if (numPoles < 2) {
                info.reason = "Insufficient poles";
                return false;
            }

            std::vector<gp_Pnt> poles;
            poles.reserve(numPoles);

            for (int i = 1; i <= numPoles; i++) {
                try {
                    poles.push_back(bspline->Pole(i));
                }
                catch (...) {
                    info.reason = "Cannot access pole";
                    return false;
                }
            }

            // Compute best-fit line through control points
            gp_Pnt startPt, endPt;
            try {
                startPt = bspline->Value(first);
                endPt = bspline->Value(last);
            }
            catch (...) {
                info.reason = "Cannot evaluate curve endpoints";
                return false;
            }

            // THÊM: Kiểm tra endpoints hợp lệ
            gp_Vec lineVec(startPt, endPt);
            if (lineVec.Magnitude() < Precision::Confusion()) {
                info.reason = "Zero-length curve";
                return false;
            }

            gp_Lin referenceLine(startPt, gp_Dir(lineVec));

            double maxControlDeviation = 0.0;
            for (const auto& pole : poles) {
                try {
                    double dist = referenceLine.Distance(pole);
                    maxControlDeviation = std::max(maxControlDeviation, dist);
                }
                catch (...) {
                    // Skip problematic pole
                    continue;
                }
            }

            if (maxControlDeviation > tolerance * 5) {  // Loose check for control points
                info.reason = "Control points not collinear";
                info.maxDeviation = maxControlDeviation;
                return false;
            }

            // 2. Check curvature
            const int CURVATURE_SAMPLES = 20;
            double maxCurvature = 0.0;

            for (int i = 0; i <= CURVATURE_SAMPLES; i++) {
                try {
                    double param = first + (last - first) * i / CURVATURE_SAMPLES;

                    // THÊM: Validate parameter
                    if (!std::isfinite(param) || param < first || param > last) {
                        continue;
                    }

                    gp_Pnt pt;
                    gp_Vec d1, d2;

                    // THÊM: Use try-catch for D2 evaluation
                    try {
                        bspline->D2(param, pt, d1, d2);
                    }
                    catch (...) {
                        // Skip this sample point
                        continue;
                    }

                    double d1Mag = d1.Magnitude();
                    if (d1Mag > Precision::Confusion()) {
                        gp_Vec cross = d1.Crossed(d2);
                        double curvature = cross.Magnitude() / pow(d1Mag, 3);

                        // THÊM: Check for valid curvature value
                        if (std::isfinite(curvature)) {
                            maxCurvature = std::max(maxCurvature, curvature);
                        }
                    }
                }
                catch (...) {
                    // Continue with next sample
                    continue;
                }
            }

            const double MAX_CURVATURE = 0.001;
            if (maxCurvature > MAX_CURVATURE) {
                info.reason = "Curvature too high";
                info.maxCurvature = maxCurvature;
                return false;
            }

            // 3. Sample point deviation
            const int NUM_SAMPLES = 50;
            double maxSampleDeviation = 0.0;
            int validSamples = 0;

            for (int i = 0; i <= NUM_SAMPLES; i++) {
                try {
                    double param = first + (last - first) * i / NUM_SAMPLES;

                    // THÊM: Validate parameter
                    if (!std::isfinite(param) || param < first || param > last) {
                        continue;
                    }

                    gp_Pnt pt = bspline->Value(param);
                    double dist = referenceLine.Distance(pt);

                    // THÊM: Check for valid distance
                    if (std::isfinite(dist)) {
                        maxSampleDeviation = std::max(maxSampleDeviation, dist);
                        validSamples++;
                    }
                }
                catch (...) {
                    // Skip problematic sample
                    continue;
                }
            }

            // THÊM: Ensure we have enough valid samples
            if (validSamples < NUM_SAMPLES / 2) {
                info.reason = "Too few valid samples";
                return false;
            }

            if (maxSampleDeviation > tolerance) {
                info.reason = "Sample points deviate";
                info.maxDeviation = maxSampleDeviation;
                return false;
            }

            // Fill success info
            info.isStaight = true;
            info.startPoint = startPt;
            info.endPoint = endPt;
            info.direction = gp_Dir(lineVec);
            info.length = startPt.Distance(endPt);
            info.maxDeviation = maxSampleDeviation;
            info.maxCurvature = maxCurvature;
            info.straightnessScore = 1.0 - (maxSampleDeviation / tolerance);

            return true;

        }
        catch (const Standard_Failure& e) {
            info.reason = "OpenCASCADE exception: ";
            if (e.GetMessageString() != NULL) {
                info.reason += e.GetMessageString();
            }
            return false;
        }
        catch (const std::exception& e) {
            info.reason = "Standard exception: ";
            info.reason += e.what();
            return false;
        }
        catch (...) {
            info.reason = "Unknown exception";
            return false;
        }
    }


    bool StepReader::GetBSplineDirection(const TopoDS_Edge& edge, gp_Dir& direction) const {
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull()) return false;

        gp_Pnt p1 = curve->Value(first);
        gp_Pnt p2 = curve->Value(last);

        gp_Vec v(p1, p2);
        if (v.Magnitude() < Precision::Confusion()) return false;

        direction = gp_Dir(v);

        // DEBUG: In ra hướng thực tế
        std::cout << "  Direction: (" << direction.X() << ", "
            << direction.Y() << ", " << direction.Z() << ")" << std::endl;

        return true;
    }

    std::vector<std::vector<int>> StepReader::GroupCollinearLines(
        const std::vector<int>& lineIds) const {
        std::vector<std::vector<int>> groups;
        std::vector<bool> processed(lineIds.size(), false);

        // Map edge ID to TopoDS_Edge
        std::map<int, TopoDS_Edge> edgeMap;
        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int currentId = 0;
        for (; edgeExp.More(); edgeExp.Next(), currentId++) {
            edgeMap[currentId] = TopoDS::Edge(edgeExp.Current());
        }

        for (size_t i = 0; i < lineIds.size(); i++) {
            if (processed[i]) continue;

            std::vector<int> currentGroup;
            currentGroup.push_back(lineIds[i]);
            processed[i] = true;

            auto it1 = edgeMap.find(lineIds[i]);
            if (it1 == edgeMap.end()) continue;

            // Tìm tất cả lines đồng tâm với line hiện tại
            for (size_t j = i + 1; j < lineIds.size(); j++) {
                if (processed[j]) continue;

                auto it2 = edgeMap.find(lineIds[j]);
                if (it2 == edgeMap.end()) continue;

                if (AreLinesCollinear(it1->second, it2->second)) {
                    currentGroup.push_back(lineIds[j]);
                    processed[j] = true;
                }
            }

            groups.push_back(currentGroup);
        }

        return groups;
    }

    std::vector<int> StepReader::FilterPotentialEdges(const EdgeFilterCriteria& criteria) const {
        std::vector<int> filteredEdges;
        if (!m_shape || m_shape->IsNull()) return filteredEdges;

        auto classifications = ClassifyEdges();
        for (auto& edge : m_edges) {
            auto it = classifications.find(edge.id);
            if (it != classifications.end() && it->second.location == EdgeClassification::INTERNAL) {
                // Đánh dấu edge này để không hiển thị
                // (cần thêm field isInternal vào EdgeInfo nếu cần)
            }
        }

        for (const auto& edgeInfo : m_edges) {
            // Kiểm tra độ dài
            if (edgeInfo.length < criteria.minLength) continue;

            // Kiểm tra classification
            auto it = classifications.find(edgeInfo.id);
            if (it != classifications.end()) {
                // QUAN TRỌNG: Loại bỏ edges INTERNAL
                if (criteria.excludeInternalEdges &&
                    it->second.location == EdgeClassification::Location::INTERNAL) {
                    continue;
                }

                // Chỉ giữ lại edges trên bề mặt ngoài
                if (it->second.isOnCylinderSurface &&
                    it->second.location == EdgeClassification::Location::ON_CYLINDER_SURFACE) {
                    filteredEdges.push_back(edgeInfo.id);
                }
            }
        }

        return filteredEdges;
    }

    std::vector<std::vector<int>> StepReader::DetectClosedLoops() const {
        std::vector<std::vector<int>> loops;

        if (!m_shape || m_shape->IsNull()) return loops;

        // Build connectivity map
        std::map<int, std::vector<int>> connectivity;
        std::map<int, TopoDS_Edge> edgeMap;

        // Thu thập tất cả edges
        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int edgeId = 0;
        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            edgeMap[edgeId] = TopoDS::Edge(edgeExp.Current());
        }

        // Xây dựng connectivity dựa trên vertices chung
        for (auto& pair1 : edgeMap) {
            TopoDS_Vertex v1Start, v1End;
            TopExp::Vertices(pair1.second, v1Start, v1End);

            for (auto& pair2 : edgeMap) {
                if (pair1.first == pair2.first) continue;

                TopoDS_Vertex v2Start, v2End;
                TopExp::Vertices(pair2.second, v2Start, v2End);

                // Kiểm tra vertices chung
                if (v1Start.IsSame(v2Start) || v1Start.IsSame(v2End) ||
                    v1End.IsSame(v2Start) || v1End.IsSame(v2End)) {
                    connectivity[pair1.first].push_back(pair2.first);
                }
            }
        }

        // Tìm closed loops bằng DFS
        std::set<int> visited;

        for (auto& pair : connectivity) {
            if (visited.find(pair.first) != visited.end()) continue;

            std::vector<int> currentLoop;
            std::stack<int> stack;
            stack.push(pair.first);

            while (!stack.empty()) {
                int current = stack.top();
                stack.pop();

                if (visited.find(current) != visited.end()) continue;

                visited.insert(current);
                currentLoop.push_back(current);

                // Thêm neighbors
                for (int neighbor : connectivity[current]) {
                    if (visited.find(neighbor) == visited.end()) {
                        stack.push(neighbor);
                    }
                }
            }

            // Kiểm tra xem loop có khép kín không
            if (currentLoop.size() >= 3) {
                loops.push_back(currentLoop);
            }
        }

        return loops;
    }

    std::vector<ToolpathCandidate> StepReader::GenerateToolpathCandidates() const {
        std::vector<ToolpathCandidate> candidates;

        EdgeFilterCriteria criteria;
        criteria.minLength = 0.5; // Giảm ngưỡng để không bỏ sót circles nhỏ
        auto filteredEdges = FilterPotentialEdges(criteria);
        auto classifications = ClassifyEdges();

        // Tạo một candidate chứa TẤT CẢ edges trên surface
        ToolpathCandidate allSurfaceEdges;
        allSurfaceEdges.type = "all_surface_features";
        allSurfaceEdges.priority = 2.0;
        allSurfaceEdges.totalLength = 0;

        // Thu thập tất cả edges không phải INTERNAL
        for (const auto& edge : m_edges) {
            auto classIt = classifications.find(edge.id);
            if (classIt != classifications.end() &&
                classIt->second.location != EdgeClassification::INTERNAL) {
                allSurfaceEdges.edgeIds.push_back(edge.id);
                allSurfaceEdges.totalLength += edge.length;
            }
        }

        if (!allSurfaceEdges.edgeIds.empty()) {
            candidates.push_back(allSurfaceEdges);
        }

        // Vẫn giữ các closed loops riêng lẻ để tham khảo
        auto closedLoops = DetectClosedLoops();
        for (const auto& loop : closedLoops) {
            ToolpathCandidate candidate;
            candidate.edgeIds = loop;
            candidate.type = "closed_loop";
            candidate.priority = 1.0;

            double totalLength = 0;
            for (int edgeId : loop) {
                for (const auto& edge : m_edges) {
                    if (edge.id == edgeId) {
                        totalLength += edge.length;
                        break;
                    }
                }
            }
            candidate.totalLength = totalLength;
            candidates.push_back(candidate);
        }

        // Thêm các circles đơn lẻ không thuộc loop nào
        std::set<int> usedEdges;
        for (const auto& candidate : candidates) {
            for (int id : candidate.edgeIds) {
                usedEdges.insert(id);
            }
        }

        for (const auto& edge : m_edges) {
            if (edge.type == EdgeInfo::CIRCLE &&
                usedEdges.find(edge.id) == usedEdges.end()) {
                auto classIt = classifications.find(edge.id);
                if (classIt != classifications.end() &&
                    classIt->second.location != EdgeClassification::INTERNAL) {
                    ToolpathCandidate circleCandidate;
                    circleCandidate.edgeIds.push_back(edge.id);
                    circleCandidate.type = "single_circle";
                    circleCandidate.priority = 0.8;
                    circleCandidate.totalLength = edge.length;
                    candidates.push_back(circleCandidate);
                }
            }
        }

        // Sort by priority
        std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.priority > b.priority; });

        return candidates;
    }
    bool StepReader::EdgesConnected(int edge1Id, int edge2Id, double tolerance) const {
        TopExp_Explorer exp(*m_shape, TopAbs_EDGE);
        TopoDS_Edge e1, e2;
        int currentId = 0;

        for (; exp.More(); exp.Next(), currentId++) {
            if (currentId == edge1Id) e1 = TopoDS::Edge(exp.Current());
            if (currentId == edge2Id) e2 = TopoDS::Edge(exp.Current());
        }

        if (e1.IsNull() || e2.IsNull()) return false;

        TopoDS_Vertex v1Start, v1End, v2Start, v2End;
        TopExp::Vertices(e1, v1Start, v1End);
        TopExp::Vertices(e2, v2Start, v2End);

        gp_Pnt p1s = BRep_Tool::Pnt(v1Start);
        gp_Pnt p1e = BRep_Tool::Pnt(v1End);
        gp_Pnt p2s = BRep_Tool::Pnt(v2Start);
        gp_Pnt p2e = BRep_Tool::Pnt(v2End);

        return (p1s.Distance(p2s) < tolerance || p1s.Distance(p2e) < tolerance ||
            p1e.Distance(p2s) < tolerance || p1e.Distance(p2e) < tolerance);
    }

    bool StepReader::IsClosedLoop(const std::vector<int>& edgeIds) const {
        if (edgeIds.size() < 3) return false;

        // Simplified check - kiểm tra endpoints
        std::map<std::string, int> vertexCount;

        for (int id : edgeIds) {
            // Get endpoints and count occurrences
            // Real implementation would track actual vertices
        }

        return true; // Simplified
    }

    bool StepReader::HasOnlyCircles(const std::vector<int>& edgeIds) const {
        for (int id : edgeIds) {
            if (m_edges[id].type != EdgeInfo::CIRCLE) return false;
        }
        return true;
    }

    bool StepReader::HasRectanglePattern(const std::vector<int>& edgeIds) const {
        if (edgeIds.size() != 4) return false;

        int lineCount = 0;
        for (int id : edgeIds) {
            if (m_edges[id].type == EdgeInfo::LINE) lineCount++;
        }

        return lineCount == 4;
    }


    std::vector<EdgeGroup> StepReader::GroupConnectedEdges(double tolerance) const {
        std::vector<EdgeGroup> groups;
        std::set<int> processed;

        // Lấy danh sách edges đã lọc (không INTERNAL)
        auto classifications = ClassifyEdges();
        std::vector<int> visibleEdges;

        for (const auto& kvp : classifications) {
            if (kvp.second.location != EdgeClassification::INTERNAL) {
                visibleEdges.push_back(kvp.first);
            }
        }

        // Dùng Union-Find để gom nhóm
        std::map<int, int> parent;
        for (int id : visibleEdges) parent[id] = id;

        std::function<int(int)> find = [&](int x) {
            if (parent[x] != x) parent[x] = find(parent[x]);
            return parent[x];
            };

        auto unite = [&](int x, int y) {
            parent[find(x)] = find(y);
            };

        // Gom nhóm dựa trên endpoints và tiếp tuyến
        for (size_t i = 0; i < visibleEdges.size(); i++) {
            std::cout << "Debug - Checking " << visibleEdges.size() << " visible edges for grouping" << std::endl;
            // Debug chi tiết
            for (size_t i = 0; i < visibleEdges.size(); i++) {
                int id = visibleEdges[i];
                std::cout << "Visible edge " << id << ": type=" << m_edges[id].type
                    << ", length=" << m_edges[id].length << std::endl;
            }

            for (size_t j = i + 1; j < visibleEdges.size(); j++) {
                int id1 = visibleEdges[i];
                int id2 = visibleEdges[j];

                bool connected = EdgesConnected(id1, id2, tolerance);
                if (connected) {
                    std::cout << "Edge " << id1 << "-" << id2 << " connected, checking tangent..." << std::endl;
                    bool tangent = AreEdgesTangent(id1, id2, 15.0);
                    bool perpendicular = AreEdgesPerpendicular(id1, id2, 15.0);
                    if (tangent || perpendicular) {
                        unite(id1, id2);
                    }
                    std::cout << "Edge " << id1 << " - " << id2
                        << ": connected=" << connected
                        << ", tangent=" << tangent << std::endl;

                    if (tangent) {
                        unite(id1, id2);
                    }
                }

                // Kiểm tra endpoints
                if (EdgesConnected(id1, id2, tolerance)) {
                    // Kiểm tra tiếp tuyến
                    if (AreEdgesTangent(id1, id2, 175.0)) { // 15 độ tolerance
                        unite(id1, id2);
                    }
                }
            }
        }

        // Tạo groups từ Union-Find
        std::map<int, std::vector<int>> groupMap;
        for (int id : visibleEdges) {
            groupMap[find(id)].push_back(id);
        }

        // Chuyển thành EdgeGroup
        for (const auto& kvp : groupMap) {
            std::cout << "Debug - Found " << groupMap.size() << " groups:" << std::endl;
            for (const auto& kvp : groupMap) {
                std::cout << "  Group root " << kvp.first << " has "
                    << kvp.second.size() << " edges: ";
                for (int id : kvp.second) std::cout << id << " ";
                std::cout << std::endl;
            }
            if (kvp.second.size() >= 2) { // Ít nhất 2 edges
                EdgeGroup group;
                group.edgeIds = kvp.second;

                // Phân loại group
                if (IsClosedLoop(group.edgeIds)) {
                    if (HasOnlyCircles(group.edgeIds)) {
                        group.groupType = "circle";
                    }
                    else if (HasRectanglePattern(group.edgeIds)) {
                        group.groupType = "rectangle";
                    }
                    else {
                        group.groupType = "closed_profile";
                    }
                    group.confidence = 0.9;
                }
                else {
                    group.groupType = "open_chain";
                    group.confidence = 0.7;
                }

                groups.push_back(group);
            }
        }

        return groups;
    }

    bool StepReader::AreEdgesTangent(int edge1Id, int edge2Id, double angleTolerance) const {
        TopExp_Explorer exp(*m_shape, TopAbs_EDGE);
        TopoDS_Edge e1, e2;
        int currentId = 0;

        for (; exp.More(); exp.Next(), currentId++) {
            if (currentId == edge1Id) e1 = TopoDS::Edge(exp.Current());
            if (currentId == edge2Id) e2 = TopoDS::Edge(exp.Current());
        }

        if (e1.IsNull() || e2.IsNull()) return false;

        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) c1 = BRep_Tool::Curve(e1, f1, l1);
        Handle(Geom_Curve) c2 = BRep_Tool::Curve(e2, f2, l2);

        if (c1.IsNull() || c2.IsNull()) return false;

        // Kiểm tra tại điểm nối
        gp_Pnt p1e = c1->Value(l1);
        gp_Pnt p2s = c2->Value(f2);

        if (p1e.Distance(p2s) < 0.1) {
            gp_Vec t1, t2;
            c1->D1(l1, p1e, t1);
            c2->D1(f2, p2s, t2);

            double angle = t1.Angle(t2) * 180.0 / M_PI;
            return angle < angleTolerance || angle >(180.0 - angleTolerance);
        }

        return false;
    }
    bool StepReader::AreEdgesPerpendicular(int edge1Id, int edge2Id, double angleTolerance) const {
        TopExp_Explorer exp(*m_shape, TopAbs_EDGE);
        TopoDS_Edge e1, e2;
        int currentId = 0;

        for (; exp.More(); exp.Next(), currentId++) {
            if (currentId == edge1Id) e1 = TopoDS::Edge(exp.Current());
            if (currentId == edge2Id) e2 = TopoDS::Edge(exp.Current());
        }

        if (e1.IsNull() || e2.IsNull()) return false;

        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) c1 = BRep_Tool::Curve(e1, f1, l1);
        Handle(Geom_Curve) c2 = BRep_Tool::Curve(e2, f2, l2);

        if (c1.IsNull() || c2.IsNull()) return false;

        // Lấy endpoints
        gp_Pnt p1s = c1->Value(f1);
        gp_Pnt p1e = c1->Value(l1);
        gp_Pnt p2s = c2->Value(f2);
        gp_Pnt p2e = c2->Value(l2);

        gp_Vec t1, t2;
        gp_Pnt p;

        // Kiểm tra 4 trường hợp kết nối
        if (p1e.Distance(p2s) < 0.1) {
            c1->D1(l1, p, t1);
            c2->D1(f2, p, t2);
        }
        else if (p1e.Distance(p2e) < 0.1) {
            c1->D1(l1, p, t1);
            c2->D1(l2, p, t2);
            t2.Reverse();
        }
        else if (p1s.Distance(p2s) < 0.1) {
            c1->D1(f1, p, t1);
            t1.Reverse();
            c2->D1(f2, p, t2);
        }
        else if (p1s.Distance(p2e) < 0.1) {
            c1->D1(f1, p, t1);
            t1.Reverse();
            c2->D1(l2, p, t2);
            t2.Reverse();
        }
        else {
            return false; // Không kết nối
        }

        double angle = t1.Angle(t2) * 180.0 / M_PI;
        return (angle > 90 - angleTolerance && angle < 90 + angleTolerance);
    }
    std::vector<UnrolledPoint> StepReader::UnrollEdge(
        int edgeId,
        const CylinderInfo& cylinderInfo,
        const UnrollingParams& params) {

        // Kiểm tra edge có trong map không
        auto it = m_topoEdgeMap.find(edgeId);
        if (it == m_topoEdgeMap.end()) {
            return std::vector<UnrolledPoint>();
        }

        // Setup params
        UnrollingParams engineParams = params;
        engineParams.cylinderRadius = cylinderInfo.radius;
        engineParams.cylinderCenter = gp_Pnt(
            cylinderInfo.centerX,
            cylinderInfo.centerY,
            cylinderInfo.centerZ);
        engineParams.cylinderAxis = gp_Vec(
            cylinderInfo.axisX,
            cylinderInfo.axisY,
            cylinderInfo.axisZ);

        // Unroll
        UnrollingEngine engine(engineParams);
        return engine.UnrollEdge(it->second);
    }
    // Tìm cylinder chính (outer surface) dựa trên radius lớn nhất
    CylinderInfo StepReader::DetectMainCylinder() const {
        CylinderInfo mainCylinder;
        std::cout << "=== DetectMainCylinder Debug ===" << std::endl;

        if (!m_shape || m_shape->IsNull()) return mainCylinder;

        // Bước 1: Phân tích axis của ống (giống AnalyzeTubeAxis)
        TubeAxisInfo tubeAxis = AnalyzeTubeAxis();
        if (!tubeAxis.isValid) {
            std::cout << "Tube axis detected: ("
                << tubeAxis.axisDirection.X() << ", "
                << tubeAxis.axisDirection.Y() << ", "
                << tubeAxis.axisDirection.Z() << ")" << std::endl;

            // Fallback về phương pháp cũ
            CylinderInfo cylInfo = DetectCylinder();
            if (!cylInfo.isValid)
            {
                std::cout << "Original cylinder axis: ("
                    << cylInfo.axisX << ", "
                    << cylInfo.axisY << ", "
                    << cylInfo.axisZ << ")" << std::endl;
                std::cout << "Center: ("
                    << cylInfo.centerX << ", "
                    << cylInfo.centerY << ", "
                    << cylInfo.centerZ << ")" << std::endl;
                return mainCylinder;
            }

            // Sử dụng cylinder info để tạo axis
            gp_Pnt center(cylInfo.centerX, cylInfo.centerY, cylInfo.centerZ);
            gp_Dir dir(cylInfo.axisX, cylInfo.axisY, cylInfo.axisZ);
            tubeAxis.isValid = true;
            tubeAxis.axisDirection = dir;
            tubeAxis.startCenter = center;
        }

        // Tạo axis từ tube info
        gp_Ax1 centralAxis(tubeAxis.startCenter, tubeAxis.axisDirection);

        // Bước 2: COPY CHÍNH XÁC từ ClassifyEdges - Tính khoảng cách của tất cả edges đến axis
        std::map<int, double> edgeDistances;
        std::vector<double> allDistances;

        TopExp_Explorer edgeExp(*m_shape, TopAbs_EDGE);
        int edgeId = 0;

        for (; edgeExp.More(); edgeExp.Next(), edgeId++) {
            TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());

            // COPY CHÍNH XÁC từ ComputeDistanceToAxis
            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

            if (curve.IsNull()) continue;

            // Sample nhiều điểm trên edge
            const int numSamples = 60;
            double totalDistance = 0.0;
            int validSamples = 0;

            for (int i = 0; i <= numSamples; i++) {
                double param = first + (last - first) * i / numSamples;
                gp_Pnt pt = curve->Value(param);

                // Tính khoảng cách từ điểm đến axis
                double dist = pt.Distance(centralAxis.Location());
                gp_Vec v(centralAxis.Location(), pt);
                double projLength = v.Dot(gp_Vec(centralAxis.Direction()));
                gp_Pnt projectedPt = centralAxis.Location();
                projectedPt.Translate(projLength * gp_Vec(centralAxis.Direction()));

                double perpDist = pt.Distance(projectedPt);
                totalDistance += perpDist;
                validSamples++;
            }

            if (validSamples > 0) {
                double avgDistance = totalDistance / validSamples;
                edgeDistances[edgeId] = avgDistance;
                allDistances.push_back(avgDistance);
            }
        }

        // Bước 3: COPY CHÍNH XÁC logic phân nhóm từ ClassifyEdges
        if (!allDistances.empty()) {
            std::sort(allDistances.begin(), allDistances.end());

            std::vector<std::pair<double, int>> radiusGroups;
            double currentRadius = allDistances[0];
            int currentCount = 1;

            const double GROUPING_TOLERANCE = 0.5; // 0.5mm cho nhóm

            for (size_t i = 1; i < allDistances.size(); i++) {
                if (std::abs(allDistances[i] - currentRadius) < GROUPING_TOLERANCE) {
                    currentRadius = (currentRadius * currentCount + allDistances[i]) / (currentCount + 1);
                    currentCount++;
                }
                else {
                    radiusGroups.push_back(std::make_pair(currentRadius, currentCount));
                    currentRadius = allDistances[i];
                    currentCount = 1;
                }
            }
            radiusGroups.push_back(std::make_pair(currentRadius, currentCount));

            // Debug output
            std::cout << "Debug - Found radius groups:" << std::endl;
            for (const auto& group : radiusGroups) {
                std::cout << "  Radius: " << group.first << " mm, Count: " << group.second << std::endl;
            }

            // Sắp xếp theo bán kính
            std::sort(radiusGroups.begin(), radiusGroups.end(),
                [](const auto& a, const auto& b) { return a.first > b.first; });

            if (!radiusGroups.empty()) {
                mainCylinder.radius = radiusGroups[0].first;
                std::cout << "Max radius (outer surface): " << mainCylinder.radius << " mm" << std::endl;
            }
        }

        // Bước 4: Set axis info
        mainCylinder.isValid = true;
        mainCylinder.axisX = tubeAxis.axisDirection.X();
        mainCylinder.axisY = tubeAxis.axisDirection.Y();
        mainCylinder.axisZ = tubeAxis.axisDirection.Z();
        mainCylinder.centerX = tubeAxis.startCenter.X();
        mainCylinder.centerY = tubeAxis.startCenter.Y();
        mainCylinder.centerZ = tubeAxis.startCenter.Z();

        // Bước 5: Tính chiều dài thực
        double minProj = std::numeric_limits<double>::max();
        double maxProj = std::numeric_limits<double>::min();

        TopExp_Explorer vertexExp(*m_shape, TopAbs_VERTEX);
        for (; vertexExp.More(); vertexExp.Next()) {
            TopoDS_Vertex vertex = TopoDS::Vertex(vertexExp.Current());
            gp_Pnt point = BRep_Tool::Pnt(vertex);

            gp_Vec vecFromCenter(tubeAxis.startCenter, point);
            double projection = vecFromCenter.Dot(gp_Vec(tubeAxis.axisDirection));

            minProj = std::min(minProj, projection);
            maxProj = std::max(maxProj, projection);
        }

        mainCylinder.length = maxProj - minProj;

        // Cập nhật center về giữa
        double midProjection = (minProj + maxProj) / 2.0;
        gp_Vec offset = gp_Vec(tubeAxis.axisDirection) * midProjection;
        mainCylinder.centerX = tubeAxis.startCenter.X() + offset.X();
        mainCylinder.centerY = tubeAxis.startCenter.Y() + offset.Y();
        mainCylinder.centerZ = tubeAxis.startCenter.Z() + offset.Z();

        std::cout << "Main cylinder: R=" << mainCylinder.radius
            << ", L=" << mainCylinder.length << " mm" << std::endl;

        return mainCylinder;
    }

    // Main profile detection method
    ProfileInfo StepReader::DetectCompleteProfile(int startEdgeId) const {
        ProfileInfo profile;
        profile.profileId = startEdgeId;

        try {
            // Get all edges với enhanced gap tolerance
            std::vector<int> allEdgeIds;
            for (const auto& kvp : m_topoEdgeMap) {
                allEdgeIds.push_back(kvp.first);
            }

            // Find connections với multiple tolerance levels
            auto connections = FindEdgeConnectionsWithGaps(allEdgeIds);

            // Build profile với gap handling
            profile = BuildProfileWithGaps(startEdgeId, connections);

            // Classify profile type
            profile.profileType = ClassifyProfileType(profile);

            // Calculate confidence dựa trên gaps
            CalculateProfileConfidence(profile);

            profile.isValid = true;

            std::cout << "Profile detected: Type=" << profile.profileType
                << ", Edges=" << profile.orderedEdgeIds.size()
                << ", Gaps=" << profile.gaps.size()
                << ", Confidence=" << profile.profileConfidence << std::endl;

        }
        catch (const std::exception& e) {
            std::cerr << "Error in DetectCompleteProfile: " << e.what() << std::endl;
            profile.isValid = false;
        }

        return profile;
    }

    std::vector<EdgeConnection> StepReader::FindEdgeConnectionsWithGaps(
        const std::vector<int>& edgeIds) const {

        std::vector<EdgeConnection> connections;

        // Multiple tolerance levels
        const double EXACT_TOL = 0.01;      // 0.01mm
        const double SMALL_GAP_TOL = 0.5;   // 0.5mm
        const double MEDIUM_GAP_TOL = 5.0;  // 5mm

        for (size_t i = 0; i < edgeIds.size(); i++) {
            for (size_t j = i + 1; j < edgeIds.size(); j++) {
                int id1 = edgeIds[i];
                int id2 = edgeIds[j];

                auto it1 = m_topoEdgeMap.find(id1);
                auto it2 = m_topoEdgeMap.find(id2);

                if (it1 == m_topoEdgeMap.end() || it2 == m_topoEdgeMap.end())
                    continue;

                // Check all 4 connection types
                CheckConnectionWithGap(it1->second, it2->second, id1, id2,
                    connections, MEDIUM_GAP_TOL);
            }
        }

        return connections;
    }

    // Find all edge connections with tangent checking
    std::vector<EdgeConnection> StepReader::FindEdgeConnections(
        const std::vector<int>& edgeIds,
        double positionTolerance,
        double tangentTolerance) const {

        std::vector<EdgeConnection> connections;

        // Check all edge pairs
        for (size_t i = 0; i < edgeIds.size(); i++) {
            for (size_t j = 0; j < edgeIds.size(); j++) {
                if (i == j) continue;

                int id1 = edgeIds[i];
                int id2 = edgeIds[j];

                auto it1 = m_topoEdgeMap.find(id1);
                auto it2 = m_topoEdgeMap.find(id2);

                if (it1 == m_topoEdgeMap.end() || it2 == m_topoEdgeMap.end())
                    continue;

                gp_Pnt connectionPt;
                EdgeConnection::ConnectionType connType;
                double tangentAngle;

                if (CheckEdgeConnection(it1->second, it2->second,
                    connectionPt, connType, tangentAngle, positionTolerance)) {

                    EdgeConnection conn;
                    conn.fromEdgeId = id1;
                    conn.toEdgeId = id2;
                    conn.connectionType = connType;
                    conn.connectionPoint = connectionPt;
                    conn.tangentAngle = tangentAngle;
                    conn.isSmooth = (tangentAngle < tangentTolerance ||
                        tangentAngle >(180.0 - tangentTolerance));

                    connections.push_back(conn);
                }
            }
        }

        return connections;
    }

    // Check if two edges connect and calculate tangent angle
    bool StepReader::CheckEdgeConnection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2,
        gp_Pnt& connectionPoint,
        EdgeConnection::ConnectionType& connType,
        double& tangentAngle, double posTol) const {

        // Get vertices
        TopoDS_Vertex v1Start, v1End, v2Start, v2End;
        TopExp::Vertices(edge1, v1Start, v1End);
        TopExp::Vertices(edge2, v2Start, v2End);

        gp_Pnt p1s = BRep_Tool::Pnt(v1Start);
        gp_Pnt p1e = BRep_Tool::Pnt(v1End);
        gp_Pnt p2s = BRep_Tool::Pnt(v2Start);
        gp_Pnt p2e = BRep_Tool::Pnt(v2End);

        // Get curve parameters
        Standard_Real f1, l1, f2, l2;
        Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, f1, l1);
        Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, f2, l2);

        if (curve1.IsNull() || curve2.IsNull())
            return false;

        // Check 4 possible connections
        // 1. End1 -> Start2
        if (p1e.Distance(p2s) < posTol) {
            connectionPoint = p1e;
            connType = EdgeConnection::END_TO_START;

            gp_Vec t1 = GetTangentAtParameter(edge1, l1, true);
            gp_Vec t2 = GetTangentAtParameter(edge2, f2, true);
            tangentAngle = t1.Angle(t2) * 180.0 / M_PI;
            return true;
        }

        // 2. End1 -> End2
        if (p1e.Distance(p2e) < posTol) {
            connectionPoint = p1e;
            connType = EdgeConnection::END_TO_END;

            gp_Vec t1 = GetTangentAtParameter(edge1, l1, true);
            gp_Vec t2 = GetTangentAtParameter(edge2, l2, false); // Reverse
            tangentAngle = t1.Angle(t2) * 180.0 / M_PI;
            return true;
        }

        // 3. Start1 -> Start2
        if (p1s.Distance(p2s) < posTol) {
            connectionPoint = p1s;
            connType = EdgeConnection::START_TO_START;

            gp_Vec t1 = GetTangentAtParameter(edge1, f1, false); // Reverse
            gp_Vec t2 = GetTangentAtParameter(edge2, f2, true);
            tangentAngle = t1.Angle(t2) * 180.0 / M_PI;
            return true;
        }

        // 4. Start1 -> End2
        if (p1s.Distance(p2e) < posTol) {
            connectionPoint = p1s;
            connType = EdgeConnection::START_TO_END;

            gp_Vec t1 = GetTangentAtParameter(edge1, f1, false); // Reverse
            gp_Vec t2 = GetTangentAtParameter(edge2, l2, false); // Reverse
            tangentAngle = t1.Angle(t2) * 180.0 / M_PI;
            return true;
        }

        return false;
    }

    // Get tangent vector at parameter
    gp_Vec StepReader::GetTangentAtParameter(const TopoDS_Edge& edge, double param, bool forward) const {
        BRepAdaptor_Curve adaptor(edge);
        gp_Pnt pt;
        gp_Vec tangent;

        adaptor.D1(param, pt, tangent);

        if (!forward) {
            tangent.Reverse();
        }

        if (tangent.Magnitude() > Precision::Confusion()) {
            tangent.Normalize();
        }

        return tangent;
    }

    // Classify profile type based on edges
    ProfileInfo::ProfileType StepReader::ClassifyProfileType(const ProfileInfo& profile) const {
        if (!profile.isClosed) {
            std::cout << "Profile is OPEN_CHAIN\n";
            return ProfileInfo::OPEN_CHAIN;
        }

        int edgeCount = static_cast<int>(profile.orderedEdgeIds.size());
        std::cout << "Classifying profile with " << edgeCount << " edges\n";


        // Count edge types
        int lineCount = 0;
        int circleCount = 0;
        int bsplineCount = 0;
        int otherCount = 0;

        for (int edgeId : profile.orderedEdgeIds) {
            for (const auto& edge : m_edges) {
                if (edge.id == edgeId) {
                    switch (edge.type) {
                    case EdgeInfo::LINE:
                        lineCount++;
                        std::cout << "  Edge " << edgeId << ": LINE\n";
                        break;
                    case EdgeInfo::CIRCLE:
                        circleCount++;
                        std::cout << "  Edge " << edgeId << ": CIRCLE\n";
                        break;
                    case EdgeInfo::BSPLINE:
                        bsplineCount++;
                        std::cout << "  Edge " << edgeId << ": BSPLINE\n";
                        break;
                    default:
                        otherCount++;
                        std::cout << "  Edge " << edgeId << ": OTHER\n";
                        break;
                    }
                    break;
                }
            }
        }
        std::cout << "Summary: " << lineCount << " lines, " << circleCount << " circles, "
            << bsplineCount << " bsplines\n";


        // Single circle
        if (edgeCount == 1 && circleCount == 1) {
            return ProfileInfo::SINGLE_CIRCLE;
        }

        // Rectangle: 4 lines with 90 degree angles
        if (edgeCount == 4) {
            // Accept 4 lines OR mix of lines and bsplines
            if (lineCount == 4) {
                std::cout << "Detected: RECTANGLE (4 lines)\n";
                return ProfileInfo::RECTANGLE;
            }
            if (lineCount + bsplineCount == 4) {
                std::cout << "Detected: RECTANGLE (lines + bsplines)\n";
                return ProfileInfo::RECTANGLE;
            }
        }

        // Slot: 2 lines + 2 semicircles
        if (edgeCount == 4 && lineCount == 2 && circleCount == 2) {
            return ProfileInfo::SLOT;
        }

        // Polygon: all lines
        if (lineCount == edgeCount && edgeCount > 4) {
            return ProfileInfo::POLYGON;
        }

        return ProfileInfo::COMPLEX_CLOSED;
    }
    // Implementation của CheckConnectionWithGap
    void StepReader::CheckConnectionWithGap(const TopoDS_Edge& edge1,
        const TopoDS_Edge& edge2,
        int id1, int id2,
        std::vector<EdgeConnection>& connections,
        double maxGapTol) const {
        // Get endpoints của 2 edges
        TopoDS_Vertex v1Start, v1End, v2Start, v2End;
        TopExp::Vertices(edge1, v1Start, v1End);
        TopExp::Vertices(edge2, v2Start, v2End);

        gp_Pnt p1s = BRep_Tool::Pnt(v1Start);
        gp_Pnt p1e = BRep_Tool::Pnt(v1End);
        gp_Pnt p2s = BRep_Tool::Pnt(v2Start);
        gp_Pnt p2e = BRep_Tool::Pnt(v2End);

        // Check 4 possible connections với gap tolerance
        struct ConnectionCandidate {
            EdgeConnection::ConnectionType type;
            gp_Pnt p1, p2;
            double distance;
        };

        std::vector<ConnectionCandidate> candidates = {
            {EdgeConnection::END_TO_START, p1e, p2s, p1e.Distance(p2s)},
            {EdgeConnection::END_TO_END, p1e, p2e, p1e.Distance(p2e)},
            {EdgeConnection::START_TO_START, p1s, p2s, p1s.Distance(p2s)},
            {EdgeConnection::START_TO_END, p1s, p2e, p1s.Distance(p2e)}
        };

        // Check each candidate
        for (const auto& cand : candidates) {
            if (cand.distance <= maxGapTol) {
                EdgeConnection conn;
                conn.fromEdgeId = id1;
                conn.toEdgeId = id2;
                conn.connectionType = cand.type;
                conn.connectionPoint = cand.p1;  // Use first point as connection
                conn.gapDistance = cand.distance;
                conn.requiresGapClosing = (cand.distance > 0.01);  // Need gap closing if > 0.01mm

                // Calculate tangent angle
                double tangentAngle = 0.0;
                gp_Pnt dummy;
                CheckEdgeConnection(edge1, edge2, dummy, conn.connectionType, tangentAngle, maxGapTol);
                conn.tangentAngle = tangentAngle;
                conn.isSmooth = (tangentAngle < 15.0 || tangentAngle > 165.0);

                connections.push_back(conn);
            }
        }
    }

    // Implementation của BuildProfileWithGaps
    ProfileInfo StepReader::BuildProfileWithGaps(int startEdgeId,
        const std::vector<EdgeConnection>& connections) const {
        ProfileInfo profile;
        profile.profileId = startEdgeId;
        profile.isValid = false;

        // Build adjacency map từ connections
        std::map<int, std::vector<EdgeConnection>> adjacency;
        for (const auto& conn : connections) {
            adjacency[conn.fromEdgeId].push_back(conn);
        }

        // DFS/BFS để build ordered edge list
        std::set<int> visited;
        std::vector<int> orderedEdges;
        std::vector<EdgeConnection> usedConnections;

        // Start từ startEdgeId
        int currentId = startEdgeId;
        orderedEdges.push_back(currentId);
        visited.insert(currentId);

        // Trace profile
        bool foundClosure = false;
        while (true) {
            bool foundNext = false;

            // Find best connection từ current edge
            if (adjacency.find(currentId) != adjacency.end()) {
                const auto& conns = adjacency[currentId];

                // Prioritize connections với gap nhỏ nhất
                EdgeConnection bestConn;
                double minGap = std::numeric_limits<double>::max();

                for (const auto& conn : conns) {
                    if (visited.find(conn.toEdgeId) == visited.end() ||
                        (conn.toEdgeId == startEdgeId && orderedEdges.size() > 2)) {

                        if (conn.gapDistance < minGap) {
                            minGap = conn.gapDistance;
                            bestConn = conn;
                            foundNext = true;
                        }
                    }
                }

                if (foundNext) {
                    // Check if closing the loop
                    if (bestConn.toEdgeId == startEdgeId) {
                        foundClosure = true;
                        usedConnections.push_back(bestConn);

                        // Add gap info if needed
                        if (bestConn.requiresGapClosing) {
                            GapInfo gap;
                            gap.fromEdgeId = bestConn.fromEdgeId;
                            gap.toEdgeId = bestConn.toEdgeId;
                            gap.gapDistance = bestConn.gapDistance;
                            gap.confidence = 1.0 - (bestConn.gapDistance / 5.0); // Simple confidence
                            gap.suggestedMethod = (bestConn.gapDistance < 0.5) ?
                                GapInfo::VIRTUAL_LINE : GapInfo::EXTEND_EDGES;
                            profile.gaps.push_back(gap);
                            profile.totalGapLength += bestConn.gapDistance;
                        }
                        break;
                    }

                    // Continue to next edge
                    currentId = bestConn.toEdgeId;
                    orderedEdges.push_back(currentId);
                    visited.insert(currentId);
                    usedConnections.push_back(bestConn);

                    // Add gap info if needed
                    if (bestConn.requiresGapClosing) {
                        GapInfo gap;
                        gap.fromEdgeId = bestConn.fromEdgeId;
                        gap.toEdgeId = bestConn.toEdgeId;
                        gap.gapDistance = bestConn.gapDistance;
                        gap.confidence = 1.0 - (bestConn.gapDistance / 5.0);
                        gap.suggestedMethod = (bestConn.gapDistance < 0.5) ?
                            GapInfo::VIRTUAL_LINE : GapInfo::EXTEND_EDGES;
                        profile.gaps.push_back(gap);
                        profile.totalGapLength += bestConn.gapDistance;
                    }
                }
            }

            if (!foundNext) break;
        }

        // Set profile properties
        profile.orderedEdgeIds = orderedEdges;
        profile.connections = usedConnections;
        profile.isClosed = foundClosure;
        profile.hasVirtualEdges = !profile.gaps.empty();

        // Calculate total length
        profile.totalLength = 0.0;
        for (int edgeId : orderedEdges) {
            for (const auto& edge : m_edges) {
                if (edge.id == edgeId) {
                    profile.totalLength += edge.length;
                    break;
                }
            }
        }

        profile.isValid = true;
        return profile;
    }

    // Implementation của CalculateProfileConfidence
    void StepReader::CalculateProfileConfidence(ProfileInfo& profile) const {
        if (profile.gaps.empty()) {
            profile.profileConfidence = 1.0;
            return;
        }

        // Base confidence
        double confidence = 1.0;

        // Reduce confidence based on total gap length
        if (profile.totalLength > 0) {
            double gapRatio = profile.totalGapLength / profile.totalLength;
            confidence *= (1.0 - gapRatio);
        }

        // Reduce confidence based on number of gaps
        confidence *= std::pow(0.95, profile.gaps.size()); // 5% reduction per gap

        // Reduce confidence based on max gap size
        double maxGap = 0.0;
        for (const auto& gap : profile.gaps) {
            maxGap = std::max(maxGap, gap.gapDistance);
        }

        if (maxGap > 2.0) {  // Gaps > 2mm reduce confidence significantly
            confidence *= 0.8;
        }

        profile.profileConfidence = std::max(0.1, confidence); // Minimum 10% confidence
    }




} // namespace GeometryKernel