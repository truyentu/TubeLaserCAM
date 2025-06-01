// GeometryWrapper/Header Files/ManagedStepReader.h
#pragma once

using namespace System;
using namespace System::Collections::Generic;

// Forward declaration
namespace GeometryKernel {
    class StepReader;
    struct EdgeInfo;
    struct EdgeClassification;
    struct UnrolledPoint;
    struct UnrollingParams;
}

namespace GeometryWrapper {

    // Value structs
    public value struct ManagedCylinderInfo {
        bool IsValid;
        double Radius;
        double Length;
        double AxisX, AxisY, AxisZ;
        double CenterX, CenterY, CenterZ;
    };

    public value struct ManagedEdgeClassification {
    public:
        enum class Location {
            OnCylinderSurface,
            OnEndFace,
            Internal,
            Unknown
        };

        enum class ShapeType {
            Line,
            Circle,
            Ellipse,
            Parabola,
            Hyperbola,
            BSpline,
            Bezier,
            Other
        };

        Location ClassificationLocation;
        ShapeType TypeOfShape;
        bool IsOnCylinderSurface;
        int OriginalEdgeId;
    };

    public value struct ManagedEdgeInfo {
        enum class EdgeType {
            Line,
            Circle,
            BSpline,
            Bezier,
            Other
        };

        property EdgeType Type;
        property double Length;
        property int Id;
    };

    public value struct Point3D {
        double X;
        double Y;
        double Z;

        Point3D(double x, double y, double z) : X(x), Y(y), Z(z) {}
    };

    public value struct ManagedToolpathCandidate {
        array<int>^ EdgeIds;
        double Priority;
        String^ Type;
        double TotalLength;
    };

    // Ref classes - KHAI BÁO MỘT LẦN
    public ref class ManagedUnrolledPoint {
    public:
        property double Y;
        property double C;
        property double X;
    };

    public ref class ManagedUnrollingParams {
    public:
        property double ChordTolerance;
        property double AngleTolerance;
        property int MinPoints;
        property int MaxPoints;
        property bool UnwrapAngles;

        ManagedUnrollingParams() {
            ChordTolerance = 0.1;
            AngleTolerance = 5.0;
            MinPoints = 10;
            MaxPoints = 1000;
            UnwrapAngles = true;
        }
    };

    // Main managed wrapper class
    public ref class ManagedStepReader {
    private:
        GeometryKernel::StepReader* m_pNativeReader;

    public:
        // Constructor/Destructor
        ManagedStepReader();
        ~ManagedStepReader();
        !ManagedStepReader(); // Finalizer

        // Basic file operations
        bool LoadFile(String^ filePath);
        int GetEdgeCount();
        List<ManagedEdgeInfo>^ GetEdgeInfoList();

        // Wireframe data
        void GetWireframeData(List<Point3D>^% vertices, List<Tuple<int, int>^>^% lineIndices);

        // Cylinder detection
        ManagedCylinderInfo DetectCylinder();

        // Edge operations
        List<Point3D>^ GetEdgePoints(int edgeId);
        Dictionary<int, ManagedEdgeClassification>^ GetEdgeClassifications(ManagedCylinderInfo cylinderInfo);

        // Toolpath operations
        List<ManagedToolpathCandidate>^ GetToolpathCandidates();
        List<List<int>^>^ GetEdgeGroups();

        // Unrolling operations
        List<ManagedUnrolledPoint^>^ UnrollEdge(
            int edgeId,
            ManagedCylinderInfo^ cylinderInfo,
            ManagedUnrollingParams^ params);
    };

} // namespace GeometryWrapper