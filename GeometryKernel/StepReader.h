#pragma once
#include <string>
#include <vector>
#include <memory>
#include <map>
#include "GeometryTypes.h"  // Include types

// OpenCASCADE includes
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>

// Forward declarations
class TopoDS_Shape;

namespace GeometryKernel {

    class StepReader {
    private:
        std::unique_ptr<TopoDS_Shape> m_shape;
        std::vector<EdgeInfo> m_edges;

    public:
        StepReader();
        ~StepReader();

        bool LoadFile(const std::string& filePath);
        int GetEdgeCount() const;
        std::vector<EdgeInfo> GetEdgeInfoList() const;

        void ExtractWireframeData(std::vector<double>& vertices,
            std::vector<int>& indices) const;

        CylinderInfo DetectCylinder() const;
        TubeAxisInfo AnalyzeTubeAxis() const;
        ThickCylinderInfo DetectThickCylinder() const;
        double ComputeDistanceToAxis(const TopoDS_Edge& edge, const gp_Ax1& axis) const;
        // Phát hiện các line song song với trục cylinder
        std::vector<int> DetectAxialLines() const;

        // Kiểm tra BSpline có phải là đường thẳng không
        bool IsBSplineStraight(const TopoDS_Edge& edge, double tolerance = 0.01) const;

        // Lấy hướng của BSpline thẳng
        bool GetBSplineDirection(const TopoDS_Edge& edge, gp_Dir& direction) const;

        // Kiểm tra 2 lines có đồng tâm không
        bool AreLinesCollinear(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2,
            double tolerance = 0.01) const;

        // Nhóm các lines đồng tâm
        std::vector<std::vector<int>> GroupCollinearLines(const std::vector<int>& lineIds) const;
        bool GetEdgeGeometry(int edgeId, std::vector<double>& points) const;
        std::map<int, EdgeClassification> ClassifyEdges() const;
        std::vector<int> FilterPotentialEdges(const EdgeFilterCriteria& criteria) const;
        std::vector<std::vector<int>> DetectClosedLoops() const;
        std::vector<ToolpathCandidate> GenerateToolpathCandidates() const;
    };
}
