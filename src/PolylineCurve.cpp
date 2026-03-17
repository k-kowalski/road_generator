#include "PolylineCurve.h"

#include <cmath>

namespace {

float DistanceSquared(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

} // namespace

PolylineCurve MakeDefaultPolylineCurve() {
    PolylineCurve curve;
    curve.controlPoints = {
        {-1.25f, 0.0f, -0.90f},
        {-0.55f, 0.0f, -0.10f},
        {0.00f, 0.0f, 0.70f},
        {0.65f, 0.0f, -0.15f},
        {1.25f, 0.0f, 0.85f},
    };
    return curve;
}

std::optional<PolylineCurveValidationError> ValidatePolylineCurve(
    const PolylineCurve& curve,
    float groundTolerance) {
    if (curve.controlPoints.size() < 2) {
        return PolylineCurveValidationError::TooFewPoints;
    }

    const float duplicateToleranceSquared = groundTolerance * groundTolerance;

    for (std::size_t i = 0; i < curve.controlPoints.size(); ++i) {
        const DirectX::XMFLOAT3& point = curve.controlPoints[i];
        if (std::fabs(point.y) > groundTolerance) {
            return PolylineCurveValidationError::PointOffGroundPlane;
        }

        if (i > 0 && DistanceSquared(curve.controlPoints[i - 1], point) <= duplicateToleranceSquared) {
            return PolylineCurveValidationError::AdjacentDuplicatePoints;
        }
    }

    return std::nullopt;
}

const char* PolylineCurveValidationErrorMessage(PolylineCurveValidationError error) {
    switch (error) {
    case PolylineCurveValidationError::TooFewPoints:
        return "Polyline curve validation failed: at least two control points are required.";
    case PolylineCurveValidationError::PointOffGroundPlane:
        return "Polyline curve validation failed: all control points must lie on the XZ ground plane.";
    case PolylineCurveValidationError::AdjacentDuplicatePoints:
        return "Polyline curve validation failed: adjacent control points must not be duplicates.";
    default:
        return "Polyline curve validation failed.";
    }
}
