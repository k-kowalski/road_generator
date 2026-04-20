#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "MathTypes.h"

using CurveIndex = std::size_t;
using SegmentIndex = std::size_t;

enum class PolylineCurveValidationError : std::uint32_t {
    TooFewPoints = 1,
    PointOffGroundPlane,
    AdjacentDuplicatePoints,
};

struct PolylineCurve {
    std::vector<Float3> controlPoints;
};

struct SegmentIntersection {
    Float3 position;
    float firstSegmentT;
    float secondSegmentT;
};

struct TrimmedCurveBranch {
    PolylineCurve curve;
    Float3 trimPoint = {};
    Float3 tangentAwayFromIntersection = {};
};

struct CurveTrimResult {
    TrimmedCurveBranch curve11;
    TrimmedCurveBranch curve12;
    TrimmedCurveBranch curve21;
    TrimmedCurveBranch curve22;
};

struct Vector2f {
    float x;
    float z;
};

std::optional<CurveTrimResult> TrimCurvesCircle(const PolylineCurve& curve1, const PolylineCurve& curve2, const Vector2f& center, float radius);

std::optional<SegmentIntersection> IntersectSegments(
    const Float3& a,
    const Float3& b,
    const Float3& c,
    const Float3& d,
    float epsilon = 1.0e-6f);

std::vector<Float3> IntersectCurves(
    const PolylineCurve& curve1,
    const PolylineCurve& curve2,
    float epsilon = 1.0e-6f);

Vector2f ProjectPointXZ(const Float3& point);

PolylineCurve MakeDefaultPolylineCurve();

PolylineCurve SubdividePolylineCurveTowardsBezierLimit(const PolylineCurve& curve);

std::optional<PolylineCurveValidationError> ValidatePolylineCurve(
    const PolylineCurve& curve,
    float groundTolerance = 1.0e-4f);

const char* PolylineCurveValidationErrorMessage(PolylineCurveValidationError error);
