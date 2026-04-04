#pragma once

#include <DirectXMath.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "PolylineCurve.h"
#include "RibbonMesh.h"

enum class GenerateRoadError : std::uint32_t {
    InvalidSpineCurve = 1,
    NoIntersection,
    MultipleIntersections,
    TrimFailed,
    CenterPatchFailed,
    RibbonBuildFailed,
    NoRibbonOutput,
};

struct GenerateRoadIssue {
    GenerateRoadError error = GenerateRoadError::InvalidSpineCurve;
    std::size_t spineIndex = 0;
    std::size_t intersectionCount = 0;
    std::size_t trimmedCurveIndex = 0;
    std::size_t boundaryPointCount = 0;
    PolylineCurveValidationError validationError = PolylineCurveValidationError::TooFewPoints;
    RibbonMeshBuildIssue ribbonIssue = {};
};

struct GenerateRoadResult {
    DirectX::XMFLOAT3 intersectionPoint = {};
    CurveTrimResult trimmedSpines = {};
    RibbonMeshData ribbonMesh;
};

std::optional<GenerateRoadIssue> GenerateRoad(
    const PolylineCurve& firstSpine,
    const PolylineCurve& secondSpine,
    float cleanupRadius,
    float ribbonHalfWidth,
    GenerateRoadResult& road,
    RibbonTangentMode tangentMode = RibbonTangentMode::AverageSegmentDirections);

std::string GenerateRoadErrorMessage(const GenerateRoadIssue& issue);
