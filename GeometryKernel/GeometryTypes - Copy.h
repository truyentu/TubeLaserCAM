#pragma once
#include <vector>
#include <string>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>

namespace GeometryKernel {

    // Thêm struct mới để lưu thông tin axis và phân tích bán kính
    struct TubeAxisInfo {
        bool isValid;
        gp_Pnt startCenter;  // Tâm ở đầu ống
        gp_Pnt endCenter;    // Tâm ở cuối ống
        gp_Dir axisDirection; // Hướng trục
        double averageRadius; // Bán kính trung bình
        double outerRadius;   // Bán kính ngoài (lớn nhất)
        double innerRadius;   // Bán kính trong (nhỏ nhất)

        TubeAxisInfo() : isValid(false), averageRadius(0.0),
            outerRadius(0.0), innerRadius(0.0) {}
    };
    struct ThickCylinderInfo {
        bool hasThickness;
        double outerRadius;
        double innerRadius;
        double length;
        double axisX, axisY, axisZ;
        double centerX, centerY, centerZ;

        ThickCylinderInfo() : hasThickness(false), outerRadius(0.0), innerRadius(0.0),
            length(0.0), axisX(0.0), axisY(0.0), axisZ(1.0),
            centerX(0.0), centerY(0.0), centerZ(0.0) {}
    };

    struct CylinderInfo {
        bool isValid;
        double radius;
        double length;
        double axisX, axisY, axisZ;
        double centerX, centerY, centerZ;

        CylinderInfo() : isValid(false), radius(0.0), length(0.0),
            axisX(0.0), axisY(0.0), axisZ(1.0),
            centerX(0.0), centerY(0.0), centerZ(0.0) {}
    };

    struct EdgeInfo {
        enum EdgeType {
            LINE,
            CIRCLE,
            BSPLINE,
            BEZIER,
            OTHER
        };

        EdgeType type;
        double length;
        int id;

        EdgeInfo() : type(OTHER), length(0.0), id(-1) {}
    };

    struct EdgeClassification {
        enum Location {
            ON_CYLINDER_SURFACE,
            ON_END_FACE,
            INTERNAL,
            UNKNOWN
        };

        enum ShapeType {
            CIRCLE_SHAPE,
            RECTANGLE_SHAPE,
            SQUARE_SHAPE,
            SLOT_SHAPE,
            FREEFORM_SHAPE,
            LINE_SEGMENT,
            UNKNOWN_SHAPE
        };

        Location location;
        ShapeType shapeType;
        int groupId;
        double distanceFromAxis;
        bool isOnCylinderSurface;
        int originalEdgeId;

        EdgeClassification() :
            location(Location::UNKNOWN),
            shapeType(ShapeType::UNKNOWN_SHAPE),
            isOnCylinderSurface(false),
            originalEdgeId(-1),
            groupId(-1),
            distanceFromAxis(0.0) {}
    };
}

struct EdgeFilterCriteria {
    double minLength = 1.0;
    double maxDistanceFromSurface = 0.1;
    bool excludeInternalEdges = true;
    bool excludeConstructionGeometry = true;
};

struct ToolpathCandidate {
    std::vector<int> edgeIds;
    double priority;
    std::string type;
    double totalLength;
};