#include "PolylineCurve.h"

#include <cmath>

namespace {

float DistanceSquared(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

DirectX::XMFLOAT3 Midpoint(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    return {
        0.5f * (left.x + right.x),
        0.5f * (left.y + right.y),
        0.5f * (left.z + right.z),
    };
}

DirectX::XMFLOAT3 SmoothInteriorPoint(
    const DirectX::XMFLOAT3& left,
    const DirectX::XMFLOAT3& center,
    const DirectX::XMFLOAT3& right) {
    return {
        0.25f * left.x + 0.50f * center.x + 0.25f * right.x,
        0.25f * left.y + 0.50f * center.y + 0.25f * right.y,
        0.25f * left.z + 0.50f * center.z + 0.25f * right.z,
    };
}

} // namespace

PolylineCurve MakeDefaultPolylineCurve() {
    PolylineCurve curve;
    curve.controlPoints = {
        {-1.25f, 0.0f, 1.10f},
        {-1.25f, 0.0f, -1.10f},
        {1.25f, 0.0f, -1.10f},
        {1.25f, 0.0f, 1.10f},
    };
    return curve;
}

PolylineCurve SubdividePolylineCurveTowardsBezierLimit(const PolylineCurve& curve) {
    if (curve.controlPoints.size() < 2) {
        return curve;
    }

    PolylineCurve subdividedCurve;
    subdividedCurve.controlPoints.reserve(curve.controlPoints.size() * 2 - 1);
    subdividedCurve.controlPoints.push_back(curve.controlPoints.front());

    for (std::size_t i = 1; i < curve.controlPoints.size(); ++i) {
        subdividedCurve.controlPoints.push_back(Midpoint(curve.controlPoints[i - 1], curve.controlPoints[i]));
        subdividedCurve.controlPoints.push_back(curve.controlPoints[i]);
    }

    PolylineCurve smoothedCurve = subdividedCurve;

    // A single 1-2-1 smoothing pass preserves the fixed ends and keeps inserted
    // edge points centered while nudging interior vertices toward the limit curve.
    for (std::size_t i = 1; i + 1 < subdividedCurve.controlPoints.size(); ++i) {
        smoothedCurve.controlPoints[i] = SmoothInteriorPoint(
            subdividedCurve.controlPoints[i - 1],
            subdividedCurve.controlPoints[i],
            subdividedCurve.controlPoints[i + 1]);
    }

    return smoothedCurve;
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
