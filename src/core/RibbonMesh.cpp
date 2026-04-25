#include "RibbonMesh.h"

#include <array>
#include <cmath>

namespace {

constexpr float kMinimumVectorLengthSquared = 1.0e-8f;
constexpr float kMinimumMiterDenominator = 1.0e-4f;
constexpr float kMaximumMiterScale = 4.0f;
constexpr std::size_t kStackRibbonSampleCapacity = 64;
constexpr Float3 kUp = {0.0f, 1.0f, 0.0f};

Float3 Subtract(const Float3& left, const Float3& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Float3 Add(const Float3& left, const Float3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Float3 Scale(const Float3& value, float scalar) {
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

float Dot(const Float3& left, const Float3& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Float3 Cross(const Float3& left, const Float3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float LengthSquared(const Float3& value) {
    return Dot(value, value);
}

float Length(const Float3& value) {
    return std::sqrt(LengthSquared(value));
}

bool NormalizeVector(const Float3& vector, Float3& normalizedVector) {
    const float lengthSquared = LengthSquared(vector);
    if (lengthSquared <= kMinimumVectorLengthSquared) {
        return false;
    }

    const float inverseLength = 1.0f / std::sqrt(lengthSquared);
    normalizedVector = Scale(vector, inverseLength);
    return true;
}

bool ComputeCentralDifferenceTangent(
    const std::vector<Float3>& controlPoints,
    SampleIndex sampleIndex,
    Float3& tangent) {
    Float3 rawTangent = {};
    if (sampleIndex == 0) {
        rawTangent = Subtract(controlPoints[1], controlPoints[0]);
    } else if (sampleIndex + 1 == controlPoints.size()) {
        rawTangent = Subtract(controlPoints[sampleIndex], controlPoints[sampleIndex - 1]);
    } else {
        rawTangent = Subtract(controlPoints[sampleIndex + 1], controlPoints[sampleIndex - 1]);
    }

    return NormalizeVector(rawTangent, tangent);
}

bool ComputeAverageSegmentDirectionsTangent(
    const std::vector<Float3>& controlPoints,
    SampleIndex sampleIndex,
    Float3& tangent) {
    if (sampleIndex == 0) {
        return NormalizeVector(
            Subtract(controlPoints[1], controlPoints[0]),
            tangent);
    }

    if (sampleIndex + 1 == controlPoints.size()) {
        return NormalizeVector(
            Subtract(controlPoints[sampleIndex], controlPoints[sampleIndex - 1]),
            tangent);
    }

    Float3 previousDirection = {};
    if (!NormalizeVector(
            Subtract(controlPoints[sampleIndex], controlPoints[sampleIndex - 1]),
            previousDirection)) {
        return false;
    }

    Float3 nextDirection = {};
    if (!NormalizeVector(
            Subtract(controlPoints[sampleIndex + 1], controlPoints[sampleIndex]),
            nextDirection)) {
        return false;
    }

    return NormalizeVector(Add(previousDirection, nextDirection), tangent);
}

std::optional<RibbonMeshBuildIssue> ComputeSegmentSides(
    const PolylineCurve& curve,
    std::vector<Float3>& segmentSides) {
    segmentSides.clear();

    const SampleIndex segmentCount = curve.controlPoints.size() - 1;
    segmentSides.reserve(segmentCount);

    for (SampleIndex i = 0; i < segmentCount; ++i) {
        Float3 direction = {};
        if (!NormalizeVector(
                Subtract(curve.controlPoints[i + 1], curve.controlPoints[i]),
                direction)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        Float3 side = {};
        if (!NormalizeVector(Cross(kUp, direction), side)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        segmentSides.push_back(side);
    }

    return std::nullopt;
}

std::optional<RibbonMeshBuildIssue> FillSegmentSides(
    const PolylineCurve& curve,
    Float3* segmentSides,
    std::size_t segmentCount) {
    for (SampleIndex i = 0; i < segmentCount; ++i) {
        Float3 direction = {};
        if (!NormalizeVector(
                Subtract(curve.controlPoints[i + 1], curve.controlPoints[i]),
                direction)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        Float3 side = {};
        if (!NormalizeVector(Cross(kUp, direction), side)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        segmentSides[i] = side;
    }

    return std::nullopt;
}

bool ComputeCurveTangent(
    const std::vector<Float3>& controlPoints,
    SampleIndex sampleIndex,
    RibbonTangentMode tangentMode,
    Float3& tangent) {
    switch (tangentMode) {
    case RibbonTangentMode::AverageSegmentDirections:
        return ComputeAverageSegmentDirectionsTangent(controlPoints, sampleIndex, tangent);
    case RibbonTangentMode::CentralDifference:
        return ComputeCentralDifferenceTangent(controlPoints, sampleIndex, tangent);
    default:
        return false;
    }
}

std::optional<RibbonMeshBuildIssue> FillCurveTangents(
    const PolylineCurve& curve,
    RibbonTangentMode tangentMode,
    Float3* tangents,
    std::size_t sampleCount) {
    for (std::size_t i = 0; i < sampleCount; ++i) {
        Float3 tangent = {};
        if (!ComputeCurveTangent(curve.controlPoints, i, tangentMode, tangent)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        tangents[i] = tangent;
    }

    return std::nullopt;
}

float SegmentLength(const Float3& start, const Float3& end) {
    return Length(Subtract(end, start));
}

bool TryComputeMiterOffset(
    const Float3& previousSide,
    const Float3& nextSide,
    float halfWidth,
    Float3& offset) {
    Float3 miter = {};
    if (!NormalizeVector(Add(previousSide, nextSide), miter)) {
        return false;
    }

    const float denominator = Dot(miter, nextSide);
    if (denominator <= kMinimumMiterDenominator) {
        return false;
    }

    const float miterLength = halfWidth / denominator;
    if (!std::isfinite(miterLength) || miterLength > halfWidth * kMaximumMiterScale) {
        return false;
    }

    offset = Scale(miter, miterLength);
    return true;
}

} // namespace

std::optional<RibbonMeshBuildIssue> ComputeCurveTangents(
    const PolylineCurve& curve,
    RibbonTangentMode tangentMode,
    std::vector<Float3>& tangents) {
    tangents.clear();

    const std::size_t sampleCount = curve.controlPoints.size();
    if (sampleCount < 2) {
        return RibbonMeshBuildIssue{RibbonMeshBuildError::TooFewSamples, 0};
    }

    tangents.reserve(sampleCount);

    for (std::size_t i = 0; i < sampleCount; ++i) {
        Float3 tangent = {};
        if (!ComputeCurveTangent(curve.controlPoints, i, tangentMode, tangent)) {
            tangents.clear();
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        tangents.push_back(tangent);
    }

    return std::nullopt;
}

std::optional<RibbonMeshBuildIssue> BuildFlatRibbonMesh(
    const PolylineCurve& curve,
    float halfWidth,
    RibbonMeshData& ribbonMesh,
    RibbonTangentMode tangentMode) {
    ribbonMesh.vertices.clear();
    ribbonMesh.indices.clear();

    const std::size_t sampleCount = curve.controlPoints.size();
    if (sampleCount < 2) {
        return RibbonMeshBuildIssue{RibbonMeshBuildError::TooFewSamples, 0};
    }

    if (halfWidth <= 0.0f) {
        return RibbonMeshBuildIssue{RibbonMeshBuildError::NonPositiveHalfWidth, 0};
    }

    const std::size_t segmentCount = sampleCount - 1;
    std::array<Float3, kStackRibbonSampleCapacity> stackSegmentSides = {};
    std::vector<Float3> heapSegmentSides;
    Float3* segmentSides = stackSegmentSides.data();
    if (segmentCount > stackSegmentSides.size()) {
        heapSegmentSides.resize(segmentCount);
        segmentSides = heapSegmentSides.data();
    }

    if (const auto issue = FillSegmentSides(curve, segmentSides, segmentCount)) {
        return issue;
    }

    std::array<Float3, kStackRibbonSampleCapacity> stackTangents = {};
    std::vector<Float3> heapTangents;
    Float3* tangents = stackTangents.data();
    if (sampleCount > stackTangents.size()) {
        heapTangents.resize(sampleCount);
        tangents = heapTangents.data();
    }

    if (const auto issue = FillCurveTangents(curve, tangentMode, tangents, sampleCount)) {
        return issue;
    }

    ribbonMesh.vertices.reserve(sampleCount * 2);
    ribbonMesh.indices.reserve((sampleCount - 1) * 6);

    float accumulatedLength = 0.0f;

    for (std::size_t i = 0; i < sampleCount; ++i) {
        Float3 offset = {};
        if (i == 0) {
            offset = Scale(segmentSides[0], halfWidth);
        } else if (i + 1 == sampleCount) {
            offset = Scale(segmentSides[segmentCount - 1], halfWidth);
        } else if (!TryComputeMiterOffset(segmentSides[i - 1], segmentSides[i], halfWidth, offset)) {
            Float3 fallbackSide = {};
            if (!NormalizeVector(Cross(kUp, tangents[i]), fallbackSide)) {
                return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
            }
            offset = Scale(fallbackSide, halfWidth);
        }

        if (i > 0) {
            accumulatedLength += SegmentLength(curve.controlPoints[i - 1], curve.controlPoints[i]);
        }

        const Float3 center = curve.controlPoints[i];
        const Float3 left = Subtract(center, offset);
        const Float3 right = Add(center, offset);

        RibbonVertex leftVertex = {};
        leftVertex.position = left;
        leftVertex.uv = {0.0f, accumulatedLength};
        leftVertex.surfaceKind = kRibbonSurfaceRoad;
        leftVertex.color = kRibbonWireframeColor;
        ribbonMesh.vertices.push_back(leftVertex);

        RibbonVertex rightVertex = {};
        rightVertex.position = right;
        rightVertex.uv = {1.0f, accumulatedLength};
        rightVertex.surfaceKind = kRibbonSurfaceRoad;
        rightVertex.color = kRibbonWireframeColor;
        ribbonMesh.vertices.push_back(rightVertex);
    }

    for (SampleIndex sampleIndex = 0; sampleIndex + 1 < sampleCount; ++sampleIndex) {
        const std::uint32_t baseVertex = static_cast<std::uint32_t>(sampleIndex * 2);
        ribbonMesh.indices.push_back(baseVertex + 0);
        ribbonMesh.indices.push_back(baseVertex + 2);
        ribbonMesh.indices.push_back(baseVertex + 1);
        ribbonMesh.indices.push_back(baseVertex + 1);
        ribbonMesh.indices.push_back(baseVertex + 2);
        ribbonMesh.indices.push_back(baseVertex + 3);
    }

    if (ribbonMesh.vertices.size() != sampleCount * 2 ||
        ribbonMesh.indices.size() != (sampleCount - 1) * 6) {
        return RibbonMeshBuildIssue{RibbonMeshBuildError::UnexpectedTopologySize, 0};
    }

    return std::nullopt;
}

std::string RibbonMeshBuildErrorMessage(const RibbonMeshBuildIssue& issue) {
    switch (issue.error) {
    case RibbonMeshBuildError::TooFewSamples:
        return "Ribbon mesh build failed: at least two curve samples are required.";
    case RibbonMeshBuildError::NonPositiveHalfWidth:
        return "Ribbon mesh build failed: ribbon half-width must be positive.";
    case RibbonMeshBuildError::DegenerateTangent:
        return "Ribbon mesh build failed: encountered a degenerate tangent at sample " + std::to_string(issue.sampleIndex) + ".";
    case RibbonMeshBuildError::UnexpectedTopologySize:
        return "Ribbon mesh build failed: generated vertex or index counts did not match the expected ribbon strip formula.";
    default:
        return "Ribbon mesh build failed.";
    }
}
