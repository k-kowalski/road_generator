#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "MathTypes.h"
#include "PolylineCurve.h"

using SampleIndex = std::size_t;

enum class RibbonMeshBuildError : std::uint32_t {
    TooFewSamples = 1,
    NonPositiveHalfWidth,
    DegenerateTangent,
    UnexpectedTopologySize,
};

enum class RibbonTangentMode : std::uint32_t {
    AverageSegmentDirections = 1,
    CentralDifference,
};

struct RibbonMeshBuildIssue {
    RibbonMeshBuildError error = RibbonMeshBuildError::TooFewSamples;
    SampleIndex sampleIndex = 0;
};

inline constexpr Float3 kRibbonWireframeColor = {1.0f, 0.10f, 0.70f};
inline constexpr float kRibbonSurfaceRoad = 0.0f;
inline constexpr float kRibbonSurfaceIntersection = 1.0f;

struct RibbonVertex {
    Float3 position;
    Float2 uv;
    float surfaceKind = kRibbonSurfaceRoad;
    Float3 color = kRibbonWireframeColor;
};

struct RibbonMeshData {
    std::vector<RibbonVertex> vertices;
    std::vector<std::uint32_t> indices;
};

std::optional<RibbonMeshBuildIssue> ComputeCurveTangents(
    const PolylineCurve& curve,
    RibbonTangentMode tangentMode,
    std::vector<Float3>& tangents);

std::optional<RibbonMeshBuildIssue> BuildFlatRibbonMesh(
    const PolylineCurve& curve,
    float halfWidth,
    RibbonMeshData& ribbonMesh,
    RibbonTangentMode tangentMode = RibbonTangentMode::AverageSegmentDirections);

std::string RibbonMeshBuildErrorMessage(const RibbonMeshBuildIssue& issue);
