// GeometryKernel/UnrollingEngine.cpp
#include "UnrollingEngine.h"
#include <BRep_Tool.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <cmath>

namespace GeometryKernel {

    UnrollingEngine::UnrollingEngine(const UnrollingParams& params)
        : m_params(params) {
        // Ensure axis is normalized
        m_params.cylinderAxis.Normalize();

        // Set default reference direction if not provided
        if (m_params.referenceDirection.Magnitude() < 1e-6) {
            m_params.referenceDirection = GetReferenceDirection();
        }
        m_params.referenceDirection.Normalize();
    }

    gp_Vec UnrollingEngine::GetReferenceDirection() const {
        // Choose a vector perpendicular to cylinder axis
        gp_Vec ref;

        // If axis is not parallel to Z, use Z cross axis
        if (std::abs(m_params.cylinderAxis.Z()) < 0.9) {
            ref = gp_Vec(0, 0, 1).Crossed(m_params.cylinderAxis);
        }
        else {
            // Otherwise use X cross axis
            ref = gp_Vec(1, 0, 0).Crossed(m_params.cylinderAxis);
        }

        ref.Normalize();
        return ref;
    }

    UnrolledPoint UnrollingEngine::UnrollPoint(const gp_Pnt& point3D) const {
        UnrolledPoint result;

        // Store original 3D coordinates
        result.X3D = point3D.X();
        result.Y3D = point3D.Y();
        result.Z3D = point3D.Z();

        // Vector from cylinder center to point
        gp_Vec toPoint(m_params.cylinderCenter, point3D);

        // Project onto cylinder axis to get Y coordinate
        result.Y = toPoint.Dot(m_params.cylinderAxis);

        // Get radial component
        gp_Vec axialComponent = m_params.cylinderAxis * result.Y;
        gp_Vec radialVector = toPoint - axialComponent;

        // Calculate radial distance (for validation)
        double radialDistance = radialVector.Magnitude();
        result.X = radialDistance - m_params.cylinderRadius;

        // Calculate angle C
        if (radialDistance > 1e-6) {
            radialVector.Normalize();
            result.C = CalculateAngle(radialVector);
        }
        else {
            // Point is on axis
            result.C = 0.0;
        }

        return result;
    }

    double UnrollingEngine::CalculateAngle(const gp_Vec& radialVector) const {
        // Calculate angle from reference direction
        double cosAngle = radialVector.Dot(m_params.referenceDirection);

        // Get perpendicular vector for sin calculation
        gp_Vec perpVector = m_params.cylinderAxis.Crossed(m_params.referenceDirection);
        double sinAngle = radialVector.Dot(perpVector);

        // Calculate angle in radians then convert to degrees
        double angleRad = std::atan2(sinAngle, cosAngle);
        double angleDeg = angleRad * 180.0 / M_PI;

        // Normalize to [0, 360)
        if (angleDeg < 0) {
            angleDeg += 360.0;
        }

        return angleDeg;
    }

    std::vector<UnrolledPoint> UnrollingEngine::UnrollEdge(
        const TopoDS_Edge& edge,
        bool isReversed) {

        std::vector<UnrolledPoint> unrolledPoints;

        // Get curve from edge
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull()) {
            return unrolledPoints;
        }

        // Get sample parameters
        std::vector<double> parameters = GetAdaptiveSampleParameters(
            curve, first, last);

        // Sample points and unroll
        for (double param : parameters) {
            gp_Pnt point3D = curve->Value(param);

            UnrolledPoint unrolled = UnrollPoint(point3D);
            unrolled.curveParameter = param;

            unrolledPoints.push_back(unrolled);
        }

        // Reverse if needed
        if (isReversed) {
            std::reverse(unrolledPoints.begin(), unrolledPoints.end());
        }

        // Apply angle unwrapping if requested
        if (m_params.unwrapAngles) {
            unrolledPoints = UnwrapAngles(unrolledPoints, true);
        }

        return unrolledPoints;
    }

    std::vector<double> UnrollingEngine::GetAdaptiveSampleParameters(
        const Handle(Geom_Curve)& curve,
        double startParam,
        double endParam) const {

        std::vector<double> parameters;

        // Use GeomAdaptor for curve analysis
        GeomAdaptor_Curve adaptor(curve, startParam, endParam);

        // Use uniform deflection algorithm
        GCPnts_UniformDeflection discretizer(
            adaptor,
            m_params.chordTolerance,
            startParam,
            endParam);

        if (discretizer.IsDone()) {
            int nbPoints = discretizer.NbPoints();

            // Ensure minimum points
            if (nbPoints < m_params.minPointsPerCurve) {
                // Use uniform distribution
                double step = (endParam - startParam) /
                    (m_params.minPointsPerCurve - 1);

                for (int i = 0; i < m_params.minPointsPerCurve; ++i) {
                    parameters.push_back(startParam + i * step);
                }
            }
            else {
                // Use computed points
                for (int i = 1; i <= nbPoints; ++i) {
                    parameters.push_back(discretizer.Parameter(i));
                }
            }
        }
        else {
            // Fallback to uniform sampling
            int numPoints = m_params.minPointsPerCurve;
            double step = (endParam - startParam) / (numPoints - 1);

            for (int i = 0; i < numPoints; ++i) {
                parameters.push_back(startParam + i * step);
            }
        }

        // Limit maximum points
        if (parameters.size() > m_params.maxPointsPerCurve) {
            // Resample to max points
            std::vector<double> resampled;
            double step = (parameters.size() - 1.0) /
                (m_params.maxPointsPerCurve - 1.0);

            for (int i = 0; i < m_params.maxPointsPerCurve; ++i) {
                int idx = static_cast<int>(i * step);
                resampled.push_back(parameters[idx]);
            }

            parameters = resampled;
        }

        return parameters;
    }

    std::vector<UnrolledPoint> UnrollingEngine::UnwrapAngles(
        const std::vector<UnrolledPoint>& points,
        bool allowMultipleRotations) {

        if (points.size() < 2) {
            return points;
        }

        std::vector<UnrolledPoint> unwrapped = points;
        double cumulativeRotation = 0.0;

        for (size_t i = 1; i < unwrapped.size(); ++i) {
            double prevC = unwrapped[i - 1].C;
            double currC = points[i].C;  // Use original value

            // Calculate angle difference
            double diff = currC - prevC;

            // Detect seam crossing
            if (diff > 180.0) {
                diff -= 360.0;
            }
            else if (diff < -180.0) {
                diff += 360.0;
            }

            cumulativeRotation += diff;

            // Apply unwrapping
            if (allowMultipleRotations) {
                unwrapped[i].C = unwrapped[0].C + cumulativeRotation;
            }
            else {
                // Keep within [0, 360) but maintain continuity
                unwrapped[i].C = prevC + diff;
                if (unwrapped[i].C < 0) {
                    unwrapped[i].C += 360.0;
                }
                else if (unwrapped[i].C >= 360.0) {
                    unwrapped[i].C -= 360.0;
                }
            }
        }

        return unwrapped;
    }

    std::vector<std::vector<UnrolledPoint>> UnrollingEngine::SplitAtSeam(
        const std::vector<UnrolledPoint>& points,
        double seamTolerance) {

        std::vector<std::vector<UnrolledPoint>> segments;

        if (points.empty()) {
            return segments;
        }

        std::vector<UnrolledPoint> currentSegment;
        currentSegment.push_back(points[0]);

        for (size_t i = 1; i < points.size(); ++i) {
            double angleDiff = std::abs(points[i].C - points[i - 1].C);

            // Check for seam crossing
            if (angleDiff > seamTolerance) {
                // End current segment
                segments.push_back(currentSegment);

                // Start new segment
                currentSegment.clear();
            }

            currentSegment.push_back(points[i]);
        }

        // Add final segment
        if (!currentSegment.empty()) {
            segments.push_back(currentSegment);
        }

        return segments;
    }
}