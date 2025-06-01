// GeometryKernel/UnrollingEngine.h
#pragma once
#include "GeometryTypes.h"
#include <vector>
#include <TopoDS_Edge.hxx>
#include <Geom_Curve.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>

namespace GeometryKernel {

    struct UnrolledPoint {
        double Y;      // Position along cylinder axis (mm)
        double C;      // Rotation angle (degrees)
        double X;      // Radial depth (0 for surface cuts)

        // Original 3D coordinates for validation
        double X3D, Y3D, Z3D;

        // Parameter on original curve
        double curveParameter;
    };

    struct UnrollingParams {
        double cylinderRadius;
        gp_Pnt cylinderCenter;
        gp_Vec cylinderAxis;  // Must be normalized

        // Discretization parameters
        double chordTolerance = 0.1;     // Max deviation from curve (mm)
        double angleTolerance = 5.0;     // Max angle between segments (degrees)
        int minPointsPerCurve = 10;      // Minimum sampling points
        int maxPointsPerCurve = 1000;    // Maximum sampling points

        // Angle handling
        bool unwrapAngles = true;        // true: allow C > 360°
        bool splitAtSeam = false;        // true: split curves at 0/360 boundary

        // Reference direction for C=0°
        gp_Vec referenceDirection;       // Perpendicular to axis
    };

    class UnrollingEngine {
    private:
        UnrollingParams m_params;

        // Helper methods
        gp_Vec GetReferenceDirection() const;
        double CalculateAngle(const gp_Vec& radialVector) const;
        std::vector<double> GetAdaptiveSampleParameters(
            const Handle(Geom_Curve)& curve,
            double startParam,
            double endParam) const;

    public:
        UnrollingEngine(const UnrollingParams& params);

        // Main unrolling method
        std::vector<UnrolledPoint> UnrollEdge(
            const TopoDS_Edge& edge,
            bool isReversed = false);

        // Unroll a single 3D point
        UnrolledPoint UnrollPoint(const gp_Pnt& point3D) const;

        // Angle unwrapping utilities
        static std::vector<UnrolledPoint> UnwrapAngles(
            const std::vector<UnrolledPoint>& points,
            bool allowMultipleRotations = true);

        static std::vector<std::vector<UnrolledPoint>> SplitAtSeam(
            const std::vector<UnrolledPoint>& points,
            double seamTolerance = 180.0);
    };
}