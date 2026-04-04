#include "RibbonMesh.h"

#include <DirectXMath.h>

#include <cmath>

namespace {

constexpr float kMinimumVectorLengthSquared = 1.0e-8f;
constexpr float kMinimumMiterDenominator = 1.0e-4f;
constexpr float kMaximumMiterScale = 4.0f;

DirectX::XMVECTOR LoadPoint(const DirectX::XMFLOAT3& point) {
    return DirectX::XMLoadFloat3(&point);
}

bool NormalizeVector(DirectX::FXMVECTOR vector, DirectX::XMVECTOR& normalizedVector) {
    const float lengthSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(vector));
    if (lengthSquared <= kMinimumVectorLengthSquared) {
        return false;
    }

    normalizedVector = DirectX::XMVectorScale(vector, 1.0f / std::sqrt(lengthSquared));
    return true;
}

bool ComputeCentralDifferenceTangent(
    const std::vector<DirectX::XMFLOAT3>& controlPoints,
    SampleIndex sampleIndex,
    DirectX::XMVECTOR& tangent) {
    DirectX::XMVECTOR rawTangent;
    if (sampleIndex == 0) {
        rawTangent = DirectX::XMVectorSubtract(LoadPoint(controlPoints[1]), LoadPoint(controlPoints[0]));
    } else if (sampleIndex + 1 == controlPoints.size()) {
        rawTangent = DirectX::XMVectorSubtract(
            LoadPoint(controlPoints[sampleIndex]),
            LoadPoint(controlPoints[sampleIndex - 1]));
    } else {
        rawTangent = DirectX::XMVectorSubtract(
            LoadPoint(controlPoints[sampleIndex + 1]),
            LoadPoint(controlPoints[sampleIndex - 1]));
    }

    return NormalizeVector(rawTangent, tangent);
}

bool ComputeAverageSegmentDirectionsTangent(
    const std::vector<DirectX::XMFLOAT3>& controlPoints,
    SampleIndex sampleIndex,
    DirectX::XMVECTOR& tangent) {
    if (sampleIndex == 0) {
        return NormalizeVector(
            DirectX::XMVectorSubtract(LoadPoint(controlPoints[1]), LoadPoint(controlPoints[0])),
            tangent);
    }

    if (sampleIndex + 1 == controlPoints.size()) {
        return NormalizeVector(
            DirectX::XMVectorSubtract(LoadPoint(controlPoints[sampleIndex]), LoadPoint(controlPoints[sampleIndex - 1])),
            tangent);
    }

    DirectX::XMVECTOR previousDirection;
    if (!NormalizeVector(
            DirectX::XMVectorSubtract(LoadPoint(controlPoints[sampleIndex]), LoadPoint(controlPoints[sampleIndex - 1])),
            previousDirection)) {
        return false;
    }

    DirectX::XMVECTOR nextDirection;
    if (!NormalizeVector(
            DirectX::XMVectorSubtract(LoadPoint(controlPoints[sampleIndex + 1]), LoadPoint(controlPoints[sampleIndex])),
            nextDirection)) {
        return false;
    }

    return NormalizeVector(DirectX::XMVectorAdd(previousDirection, nextDirection), tangent);
}

std::optional<RibbonMeshBuildIssue> ComputeSegmentSides(
    const PolylineCurve& curve,
    std::vector<DirectX::XMFLOAT3>& segmentSides) {
    segmentSides.clear();

    const SampleIndex segmentCount = curve.controlPoints.size() - 1;
    segmentSides.reserve(segmentCount);

    const DirectX::XMVECTOR up = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

    for (SampleIndex i = 0; i < segmentCount; ++i) {
        DirectX::XMVECTOR direction;
        if (!NormalizeVector(
                DirectX::XMVectorSubtract(LoadPoint(curve.controlPoints[i + 1]), LoadPoint(curve.controlPoints[i])),
                direction)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        DirectX::XMVECTOR side;
        if (!NormalizeVector(DirectX::XMVector3Cross(up, direction), side)) {
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        DirectX::XMFLOAT3 sideValue = {};
        DirectX::XMStoreFloat3(&sideValue, side);
        segmentSides.push_back(sideValue);
    }

    return std::nullopt;
}

bool ComputeCurveTangent(
    const std::vector<DirectX::XMFLOAT3>& controlPoints,
    SampleIndex sampleIndex,
    RibbonTangentMode tangentMode,
    DirectX::XMVECTOR& tangent) {
    switch (tangentMode) {
    case RibbonTangentMode::AverageSegmentDirections:
        return ComputeAverageSegmentDirectionsTangent(controlPoints, sampleIndex, tangent);
    case RibbonTangentMode::CentralDifference:
        return ComputeCentralDifferenceTangent(controlPoints, sampleIndex, tangent);
    default:
        return false;
    }
}

float SegmentLength(const DirectX::XMFLOAT3& start, const DirectX::XMFLOAT3& end) {
    const DirectX::XMVECTOR delta = DirectX::XMVectorSubtract(LoadPoint(end), LoadPoint(start));
    return DirectX::XMVectorGetX(DirectX::XMVector3Length(delta));
}

bool TryComputeMiterOffset(
    DirectX::FXMVECTOR previousSide,
    DirectX::FXMVECTOR nextSide,
    float halfWidth,
    DirectX::XMVECTOR& offset) {
    DirectX::XMVECTOR miter;
    if (!NormalizeVector(DirectX::XMVectorAdd(previousSide, nextSide), miter)) {
        return false;
    }

    const float denominator = DirectX::XMVectorGetX(DirectX::XMVector3Dot(miter, nextSide));
    if (denominator <= kMinimumMiterDenominator) {
        return false;
    }

    const float miterLength = halfWidth / denominator;
    if (!std::isfinite(miterLength) || miterLength > halfWidth * kMaximumMiterScale) {
        return false;
    }

    offset = DirectX::XMVectorScale(miter, miterLength);
    return true;
}

} // namespace

std::optional<RibbonMeshBuildIssue> ComputeCurveTangents(
    const PolylineCurve& curve,
    RibbonTangentMode tangentMode,
    std::vector<DirectX::XMFLOAT3>& tangents) {
    tangents.clear();

    const std::size_t sampleCount = curve.controlPoints.size();
    if (sampleCount < 2) {
        return RibbonMeshBuildIssue{RibbonMeshBuildError::TooFewSamples, 0};
    }

    tangents.reserve(sampleCount);

    for (std::size_t i = 0; i < sampleCount; ++i) {
        DirectX::XMVECTOR tangent;
        if (!ComputeCurveTangent(curve.controlPoints, i, tangentMode, tangent)) {
            tangents.clear();
            return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
        }

        DirectX::XMFLOAT3 tangentValue = {};
        DirectX::XMStoreFloat3(&tangentValue, tangent);
        tangents.push_back(tangentValue);
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

    std::vector<DirectX::XMFLOAT3> segmentSides;
    if (const auto issue = ComputeSegmentSides(curve, segmentSides)) {
        return issue;
    }

    std::vector<DirectX::XMFLOAT3> tangents;
    if (const auto issue = ComputeCurveTangents(curve, tangentMode, tangents)) {
        return issue;
    }

    ribbonMesh.vertices.reserve(sampleCount * 2);
    ribbonMesh.indices.reserve((sampleCount - 1) * 6);

    const DirectX::XMVECTOR up = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    float accumulatedLength = 0.0f;

    for (std::size_t i = 0; i < sampleCount; ++i) {
        DirectX::XMVECTOR offset;
        if (i == 0) {
            offset = DirectX::XMVectorScale(LoadPoint(segmentSides.front()), halfWidth);
        } else if (i + 1 == sampleCount) {
            offset = DirectX::XMVectorScale(LoadPoint(segmentSides.back()), halfWidth);
        } else {
            const DirectX::XMVECTOR previousSide = LoadPoint(segmentSides[i - 1]);
            const DirectX::XMVECTOR nextSide = LoadPoint(segmentSides[i]);
            if (!TryComputeMiterOffset(previousSide, nextSide, halfWidth, offset)) {
                const DirectX::XMVECTOR tangent = LoadPoint(tangents[i]);
                DirectX::XMVECTOR fallbackSide;
                if (!NormalizeVector(DirectX::XMVector3Cross(up, tangent), fallbackSide)) {
                    return RibbonMeshBuildIssue{RibbonMeshBuildError::DegenerateTangent, i};
                }
                offset = DirectX::XMVectorScale(fallbackSide, halfWidth);
            }
        }

        if (i > 0) {
            accumulatedLength += SegmentLength(curve.controlPoints[i - 1], curve.controlPoints[i]);
        }

        const DirectX::XMVECTOR center = LoadPoint(curve.controlPoints[i]);
        const DirectX::XMVECTOR left = DirectX::XMVectorSubtract(center, offset);
        const DirectX::XMVECTOR right = DirectX::XMVectorAdd(center, offset);

        RibbonVertex leftVertex = {};
        DirectX::XMStoreFloat3(&leftVertex.position, left);
        leftVertex.uv = {0.0f, accumulatedLength};
        leftVertex.color = kRibbonWireframeColor;
        ribbonMesh.vertices.push_back(leftVertex);

        RibbonVertex rightVertex = {};
        DirectX::XMStoreFloat3(&rightVertex.position, right);
        rightVertex.uv = {1.0f, accumulatedLength};
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
