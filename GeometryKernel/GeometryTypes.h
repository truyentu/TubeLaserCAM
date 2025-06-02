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

        // THÊM: Straightness info cho BSpline
        bool isStraightCurve = false;
        double straightnessScore = 0.0;

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

    // Edge connection information for profile detection
    struct EdgeConnection {
        int fromEdgeId;
        int toEdgeId;

        enum ConnectionType {
            END_TO_START,    // Normal: end1 -> start2
            END_TO_END,      // Reversed: end1 -> end2  
            START_TO_START,  // Both reversed: start1 -> start2
            START_TO_END     // First reversed: start1 -> end2
        } connectionType;

        gp_Pnt connectionPoint;      // Điểm kết nối
        double tangentAngle;         // Góc giữa 2 tangent vectors (degrees)
        bool isSmooth;              // True nếu tangent continuous (< 15°)

        // THÊM: Gap information
        double gapDistance = 0.0;
        bool requiresGapClosing = false;

        EdgeConnection() : fromEdgeId(-1), toEdgeId(-1),
            connectionType(END_TO_START),
            tangentAngle(0.0), isSmooth(false) {}
    };

    // QUAN TRỌNG: Di chuyển GapInfo lên TRƯỚC ProfileInfo
    struct GapInfo {
        int fromEdgeId;
        int toEdgeId;
        double gapDistance;
        gp_Pnt gapStartPoint;
        gp_Pnt gapEndPoint;

        enum GapClosingMethod {
            NONE,
            VIRTUAL_LINE,
            VIRTUAL_ARC,
            EXTEND_EDGES
        } suggestedMethod;

        double confidence;  // 0-1, độ tin cậy của gap closing

        // Constructor
        GapInfo() : fromEdgeId(-1), toEdgeId(-1), gapDistance(0.0),
            suggestedMethod(NONE), confidence(0.0) {}
    };

    // BSpline straightness analysis info
    struct BSplineStraightInfo {
        bool isStaight = false;
        gp_Pnt startPoint;
        gp_Pnt endPoint;
        gp_Dir direction;
        double length = 0.0;
        double maxDeviation = 0.0;
        double maxCurvature = 0.0;
        double straightnessScore = 0.0;  // 0-1
        std::string reason;  // Lý do không straight
    };

    // Complete profile information - SAU khi đã define GapInfo
    struct ProfileInfo {
        int profileId;
        std::vector<int> orderedEdgeIds;           // Edges theo thứ tự kết nối
        std::vector<EdgeConnection> connections;    // Connections giữa edges
        std::vector<GapInfo> gaps;                 // Gaps trong profile

        double totalGapLength = 0.0;
        double profileConfidence = 1.0;  // 0-1
        bool hasVirtualEdges = false;
        bool isClosed;                   // Profile khép kín?
        bool isValid;                    // Profile hợp lệ?
        bool isMissingEdgeDetected = false;
        std::string missingEdgeReason;

        enum ProfileType {
            UNKNOWN,
            SINGLE_CIRCLE,      // 1 edge circle duy nhất
            RECTANGLE,          // 4 lines, góc 90°
            SLOT,              // 2 lines + 2 semicircles  
            POLYGON,           // n lines (n>4)
            COMPLEX_CLOSED,    // Hình phức tạp khép kín
            OPEN_CHAIN         // Chuỗi hở
        } profileType;

        // Geometric properties
        double totalLength;            // Tổng chiều dài

        ProfileInfo() : profileId(-1), isClosed(false), isValid(false),
            profileType(UNKNOWN), totalLength(0.0) {}
    };

    // Edge filter criteria
    struct EdgeFilterCriteria {
        double minLength = 1.0;
        double maxDistanceFromSurface = 0.1;
        bool excludeInternalEdges = true;
        bool excludeConstructionGeometry = true;
    };

    // Toolpath candidate
    struct ToolpathCandidate {
        std::vector<int> edgeIds;
        double priority = 0.0;
        std::string type;
        double totalLength = 0.0;
    };

    // Edge group
    struct EdgeGroup {
        std::vector<int> edgeIds;
        std::string groupType;
        double confidence = 0.0;
        gp_Pnt centroid;
        double boundingRadius = 0.0;
    };

} // namespace GeometryKernel

// Global enums (outside namespace)
enum EdgeFilterType {
    FILTER_NONE = 0,
    FILTER_INTERNAL = 1,
    FILTER_AXIAL_LINES = 2  // Các line song song với trục
};