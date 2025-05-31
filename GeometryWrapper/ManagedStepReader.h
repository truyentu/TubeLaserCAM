// GeometryWrapper/Header Files/ManagedStepReader.h
#pragma once

using namespace System;
using namespace System::Collections::Generic;

// Forward declaration
namespace GeometryKernel {
    class StepReader;
    struct EdgeInfo;
    struct EdgeClassification;
}

namespace GeometryWrapper {

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

    // Managed struct tương ứng với EdgeInfo
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

    // Managed Point3D
    public value struct Point3D {
        double X;
        double Y;
        double Z;

        Point3D(double x, double y, double z) : X(x), Y(y), Z(z) {}
    };

    // Di chuyển ManagedToolpathCandidate ra ngoài class
    public value struct ManagedToolpathCandidate {
        array<int>^ EdgeIds;
        double Priority;
        String^ Type;
        double TotalLength;
    };

    // Managed wrapper class
    public ref class ManagedStepReader {
    private:
        // Pointer to native C++ object
        GeometryKernel::StepReader* m_pNativeReader;

    public:
        ManagedStepReader();
        ~ManagedStepReader();
        !ManagedStepReader(); // Finalizer

        bool LoadFile(String^ filePath);
        int GetEdgeCount();
        List<ManagedEdgeInfo>^ GetEdgeInfoList();

        // Lấy dữ liệu wireframe để hiển thị
        void GetWireframeData(List<Point3D>^% vertices, List<Tuple<int, int>^>^% lineIndices);
        ManagedCylinderInfo DetectCylinder();
        List<Point3D>^ GetEdgePoints(int edgeId);
        Dictionary<int, ManagedEdgeClassification>^ GetEdgeClassifications(ManagedCylinderInfo cylinderInfo);
        List<ManagedToolpathCandidate>^ GetToolpathCandidates();
    };
}