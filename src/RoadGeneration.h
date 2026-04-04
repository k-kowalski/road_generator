#pragma once

#include <DirectXMath.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "PolylineCurve.h"
#include "RibbonMesh.h"

using NodeIndex = std::size_t;
using SpanIndex = std::size_t;
using BoundaryGroupIndex = std::size_t;

enum class GenerateRoadError : std::uint32_t {
    InvalidSpineCurve = 1,
    RibbonBuildFailed,
    NoRibbonOutput,
};

struct GenerateRoadIssue {
    GenerateRoadError error = GenerateRoadError::InvalidSpineCurve;
    CurveIndex spineIndex = 0;
    SpanIndex spanIndex = 0;
    PolylineCurveValidationError validationError = PolylineCurveValidationError::TooFewPoints;
    RibbonMeshBuildIssue ribbonIssue = {};
};

struct RoadJunctionInfo {
    DirectX::XMFLOAT3 position = {};
    std::size_t incidentCount = 0;
};

struct GenerateRoadResult {
    std::vector<RoadJunctionInfo> junctions;
    RibbonMeshData ribbonMesh;
};

std::optional<GenerateRoadIssue> GenerateRoad(
    const std::vector<PolylineCurve>& spines,
    float cleanupRadius,
    float ribbonHalfWidth,
    GenerateRoadResult& road,
    RibbonTangentMode tangentMode = RibbonTangentMode::AverageSegmentDirections);

std::optional<GenerateRoadIssue> GenerateRoad(
    const PolylineCurve& firstSpine,
    const PolylineCurve& secondSpine,
    float cleanupRadius,
    float ribbonHalfWidth,
    GenerateRoadResult& road,
    RibbonTangentMode tangentMode = RibbonTangentMode::AverageSegmentDirections);

std::string GenerateRoadErrorMessage(const GenerateRoadIssue& issue);
