#include "RoadGeneration.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

struct BoundaryRef {
    std::uint32_t vertexIndex = 0;
    float angle = 0.0f;
};

float DistanceSquared(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

float BoundaryAngle(const DirectX::XMFLOAT3& center, const DirectX::XMFLOAT3& point) {
    return std::atan2(point.z - center.z, point.x - center.x);
}

std::uint32_t AppendRibbonMesh(const RibbonMeshData& source, RibbonMeshData& destination) {
    const std::uint32_t vertexOffset = static_cast<std::uint32_t>(destination.vertices.size());
    destination.vertices.insert(destination.vertices.end(), source.vertices.begin(), source.vertices.end());
    destination.indices.reserve(destination.indices.size() + source.indices.size());

    for (std::uint32_t index : source.indices) {
        destination.indices.push_back(vertexOffset + index);
    }

    return vertexOffset;
}

void CollectBranchBoundary(
    const TrimmedCurveBranch& trimmedBranch,
    const RibbonMeshData& trimmedRibbonMesh,
    std::uint32_t vertexOffset,
    const DirectX::XMFLOAT3& intersectionPoint,
    std::vector<BoundaryRef>& boundaryRefs) {
    if (trimmedBranch.curve.controlPoints.size() < 2 || trimmedRibbonMesh.vertices.size() < 2) {
        return;
    }

    const float frontDistanceSquared =
        DistanceSquared(trimmedBranch.curve.controlPoints.front(), trimmedBranch.trimPoint);
    const float backDistanceSquared =
        DistanceSquared(trimmedBranch.curve.controlPoints.back(), trimmedBranch.trimPoint);
    const std::uint32_t seamStartVertex =
        (frontDistanceSquared <= backDistanceSquared)
            ? 0u
            : static_cast<std::uint32_t>(trimmedRibbonMesh.vertices.size() - 2);

    for (std::uint32_t localIndex = seamStartVertex; localIndex < seamStartVertex + 2; ++localIndex) {
        const DirectX::XMFLOAT3& position = trimmedRibbonMesh.vertices[localIndex].position;
        boundaryRefs.push_back({
            vertexOffset + localIndex,
            BoundaryAngle(intersectionPoint, position),
        });
    }
}

void AppendCenterPatch(
    const DirectX::XMFLOAT3& intersectionPoint,
    const std::vector<BoundaryRef>& boundaryRefs,
    RibbonMeshData& ribbonMesh) {
    std::vector<BoundaryRef> sortedBoundaryRefs = boundaryRefs;

    std::sort(
        sortedBoundaryRefs.begin(),
        sortedBoundaryRefs.end(),
        [](const BoundaryRef& left, const BoundaryRef& right) {
            return left.angle < right.angle;
        });

    const std::uint32_t centerVertexIndex = static_cast<std::uint32_t>(ribbonMesh.vertices.size());
    RibbonVertex centerVertex = {};
    centerVertex.position = {
        intersectionPoint.x,
        intersectionPoint.y,
        intersectionPoint.z,
    };
    centerVertex.uv = {0.5f, 0.5f};
    centerVertex.color = kRibbonWireframeColor;
    ribbonMesh.vertices.push_back(centerVertex);

    ribbonMesh.indices.reserve(ribbonMesh.indices.size() + sortedBoundaryRefs.size() * 3);
    for (std::size_t i = 0; i < sortedBoundaryRefs.size(); ++i) {
        const std::uint32_t current = sortedBoundaryRefs[i].vertexIndex;
        const std::uint32_t next = sortedBoundaryRefs[(i + 1) % sortedBoundaryRefs.size()].vertexIndex;
        ribbonMesh.indices.push_back(centerVertexIndex);
        ribbonMesh.indices.push_back(current);
        ribbonMesh.indices.push_back(next);
    }
}

} // namespace

std::optional<GenerateRoadIssue> GenerateRoad(
    const PolylineCurve& firstSpine,
    const PolylineCurve& secondSpine,
    float cleanupRadius,
    float ribbonHalfWidth,
    GenerateRoadResult& road,
    RibbonTangentMode tangentMode) {
    road = {};

    if (const auto error = ValidatePolylineCurve(firstSpine)) {
        return GenerateRoadIssue{
            GenerateRoadError::InvalidSpineCurve,
            0,
            0,
            0,
            0,
            *error,
            {}};
    }

    if (const auto error = ValidatePolylineCurve(secondSpine)) {
        return GenerateRoadIssue{
            GenerateRoadError::InvalidSpineCurve,
            1,
            0,
            0,
            0,
            *error,
            {}};
    }

    const std::vector<DirectX::XMFLOAT3> intersections = IntersectCurves(firstSpine, secondSpine);
    if (intersections.empty()) {
        return GenerateRoadIssue{
            GenerateRoadError::NoIntersection,
            0,
            0,
            0,
            0,
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    if (intersections.size() > 1) {
        return GenerateRoadIssue{
            GenerateRoadError::MultipleIntersections,
            0,
            intersections.size(),
            0,
            0,
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    road.intersectionPoint = intersections.front();

    const auto trimmedSpines = TrimCurvesCircle(
        firstSpine,
        secondSpine,
        ProjectPointXZ(road.intersectionPoint),
        cleanupRadius);
    if (!trimmedSpines.has_value()) {
        return GenerateRoadIssue{
            GenerateRoadError::TrimFailed,
            0,
            1,
            0,
            0,
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    road.trimmedSpines = *trimmedSpines;

    const std::array<const TrimmedCurveBranch*, 4> trimmedCurves = {
        &road.trimmedSpines.curve11,
        &road.trimmedSpines.curve12,
        &road.trimmedSpines.curve21,
        &road.trimmedSpines.curve22,
    };

    std::vector<BoundaryRef> patchBoundaryRefs;
    patchBoundaryRefs.reserve(trimmedCurves.size() * 2);

    for (std::size_t i = 0; i < trimmedCurves.size(); ++i) {
        const TrimmedCurveBranch& trimmedBranch = *trimmedCurves[i];
        const PolylineCurve& trimmedCurve = trimmedBranch.curve;
        if (trimmedCurve.controlPoints.size() < 2) {
            continue;
        }

        RibbonMeshData trimmedRibbonMesh;
        if (const auto issue = BuildFlatRibbonMesh(
                trimmedCurve,
                ribbonHalfWidth,
                trimmedRibbonMesh,
                tangentMode)) {
            return GenerateRoadIssue{
                GenerateRoadError::RibbonBuildFailed,
                0,
                1,
                i,
                0,
                PolylineCurveValidationError::TooFewPoints,
                *issue};
        }

        const std::uint32_t vertexOffset = AppendRibbonMesh(trimmedRibbonMesh, road.ribbonMesh);
        CollectBranchBoundary(
            trimmedBranch,
            trimmedRibbonMesh,
            vertexOffset,
            road.intersectionPoint,
            patchBoundaryRefs);
    }

    if (road.ribbonMesh.vertices.empty() || road.ribbonMesh.indices.empty()) {
        return GenerateRoadIssue{
            GenerateRoadError::NoRibbonOutput,
            0,
            1,
            0,
            0,
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    if (patchBoundaryRefs.size() < 3) {
        return GenerateRoadIssue{
            GenerateRoadError::CenterPatchFailed,
            0,
            1,
            0,
            patchBoundaryRefs.size(),
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    AppendCenterPatch(
        road.intersectionPoint,
        patchBoundaryRefs,
        road.ribbonMesh);
    return std::nullopt;
}

std::string GenerateRoadErrorMessage(const GenerateRoadIssue& issue) {
    switch (issue.error) {
    case GenerateRoadError::InvalidSpineCurve:
        return "Road generation failed: spine " + std::to_string(issue.spineIndex) +
               " is invalid. " + PolylineCurveValidationErrorMessage(issue.validationError);
    case GenerateRoadError::NoIntersection:
        return "Road generation failed: the input spines do not intersect.";
    case GenerateRoadError::MultipleIntersections:
        return "Road generation failed: expected one intersection point, found " +
               std::to_string(issue.intersectionCount) + ".";
    case GenerateRoadError::TrimFailed:
        return "Road generation failed: trimming the spines against the cleanup circle did not produce a usable split.";
    case GenerateRoadError::CenterPatchFailed:
        return "Road generation failed: the center patch needs at least three distinct boundary points, found " +
               std::to_string(issue.boundaryPointCount) + ".";
    case GenerateRoadError::RibbonBuildFailed:
        return "Road generation failed: ribbon build for trimmed curve " +
               std::to_string(issue.trimmedCurveIndex) + " failed. " +
               RibbonMeshBuildErrorMessage(issue.ribbonIssue);
    case GenerateRoadError::NoRibbonOutput:
        return "Road generation failed: trimming succeeded, but no trimmed curve produced a ribbon mesh.";
    default:
        return "Road generation failed.";
    }
}
