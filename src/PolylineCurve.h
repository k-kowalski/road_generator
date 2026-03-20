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

struct SegmentIntersection {
    DirectX::XMFLOAT3 position;
    float firstSegmentT;
    float secondSegmentT;
};

struct CurveTrimResult {
    PolylineCurve curve11;
    PolylineCurve curve12;
    PolylineCurve curve21;
    PolylineCurve curve22;
};

struct Vector2f {
    float x;
    float z;
};

std::optional<CurveTrimResult> TrimCurvesCircle(const PolylineCurve& curve1, const PolylineCurve& curve2, const Vector2f& center, float radius);

std::optional<SegmentIntersection> IntersectSegments(
    const DirectX::XMFLOAT3& a,
    const DirectX::XMFLOAT3& b,
    const DirectX::XMFLOAT3& c,
    const DirectX::XMFLOAT3& d,
    float epsilon = 1.0e-6f);

std::vector<DirectX::XMFLOAT3> IntersectCurves(
    const PolylineCurve& curve1,
    const PolylineCurve& curve2,
    float epsilon = 1.0e-6f);

Vector2f ProjectPointXZ(const DirectX::XMFLOAT3& point);

PolylineCurve MakeDefaultPolylineCurve();

PolylineCurve SubdividePolylineCurveTowardsBezierLimit(const PolylineCurve& curve);

std::optional<PolylineCurveValidationError> ValidatePolylineCurve(
    const PolylineCurve& curve,
    float groundTolerance = 1.0e-4f);

const char* PolylineCurveValidationErrorMessage(PolylineCurveValidationError error);
