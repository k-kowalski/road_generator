#pragma once

#include <DirectXMath.h>

#include <cstdint>
#include <optional>
#include <vector>

enum class PolylineCurveValidationError : std::uint32_t {
    TooFewPoints = 1,
    PointOffGroundPlane,
    AdjacentDuplicatePoints,
};

struct PolylineCurve {
    std::vector<DirectX::XMFLOAT3> controlPoints;
};

PolylineCurve MakeDefaultPolylineCurve();
PolylineCurve SubdividePolylineCurveTowardsBezierLimit(const PolylineCurve& curve);
std::optional<PolylineCurveValidationError> ValidatePolylineCurve(
    const PolylineCurve& curve,
    float groundTolerance = 1.0e-4f);
const char* PolylineCurveValidationErrorMessage(PolylineCurveValidationError error);
