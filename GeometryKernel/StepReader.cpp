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

// Additional includes needed
#include <set>
#include <stack>
#include <algorithm>

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
        TopExp_Explorer edgeExplorer;
        int edgeIdCounter = 0;
        for (edgeExplorer.Init(*m_shape, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
            TopoDS_Edge currentEdge = TopoDS::Edge(edgeExplorer.Current());
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
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull() || !curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve)))
            return false;

        // Sample points dọc theo BSpline
        const int numSamples = 10;
        std::vector<gp_Pnt> points;

        for (int i = 0; i <= numSamples; i++) {
            double param = first + (last - first) * i / numSamples;
            points.push_back(curve->Value(param));
        }

        // Kiểm tra tất cả points có nằm trên đường thẳng không
        if (points.size() < 3) return false;

        gp_Vec v1(points[0], points[1]);
        if (v1.Magnitude() < Precision::Confusion()) return false;
        v1.Normalize();

        for (size_t i = 2; i < points.size(); i++) {
            gp_Vec vi(points[0], points[i]);
            if (vi.Magnitude() < Precision::Confusion()) continue;

            double cross = v1.Crossed(vi).Magnitude();
            double dist = cross / vi.Magnitude();

            if (dist > tolerance) return false;
        }

        return true;
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
        auto filteredEdges = FilterPotentialEdges(criteria);
        auto closedLoops = DetectClosedLoops();

        // Process closed loops
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

        // Sort by priority
        std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.priority > b.priority; });

        return candidates;
    }

} // namespace GeometryKernel