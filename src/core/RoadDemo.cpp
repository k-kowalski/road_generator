#include "RoadDemo.h"

#include <cmath>
#include <utility>

#include "RoadGeneration.h"

namespace {

constexpr float kRoadCleanupRadius = 0.45f;
constexpr float kRibbonHalfWidth = 0.20f;
constexpr RibbonTangentMode kRibbonTangentMode = RibbonTangentMode::AverageSegmentDirections;
constexpr float kTangentDebugYOffset = 0.14f;
constexpr float kTangentDebugLength = 0.28f;

struct CurveDebugStyle {
    Float3 segmentColor;
    Float3 markerColor;
    float yOffset = 0.0f;
    float baseHalfSpan = 0.08f;
    float stemHeight = 0.18f;
    float crownHalfSpan = 0.05f;
};

struct ProcessedCurveData {
    CurveIndex authoredIndex = 0;
    PolylineCurve curve;
    std::vector<Float3> tangents;
};

float DistanceSquared(const Float3& left, const Float3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

Float3 OffsetPoint(const Float3& point, float dx, float dy, float dz) {
    return {point.x + dx, point.y + dy, point.z + dz};
}

Float3 LiftPoint(const Float3& point, float yOffset) {
    return OffsetPoint(point, 0.0f, yOffset, 0.0f);
}

CurveDebugStyle RoughCurveStyle(CurveIndex curveIndex) {
    static constexpr CurveDebugStyle kStyles[] = {
        {{0.95f, 0.56f, 0.21f}, {1.00f, 0.76f, 0.46f}, 0.02f, 0.09f, 0.20f, 0.06f},
        {{0.18f, 0.84f, 0.90f}, {0.72f, 0.98f, 1.00f}, 0.02f, 0.06f, 0.14f, 0.03f},
        {{0.44f, 0.82f, 0.28f}, {0.73f, 0.96f, 0.62f}, 0.02f, 0.07f, 0.17f, 0.05f},
        {{0.96f, 0.34f, 0.58f}, {1.00f, 0.70f, 0.82f}, 0.02f, 0.07f, 0.17f, 0.05f},
    };
    return kStyles[curveIndex % (sizeof(kStyles) / sizeof(kStyles[0]))];
}

CurveDebugStyle SubdividedCurveStyle(CurveIndex curveIndex) {
    static constexpr CurveDebugStyle kStyles[] = {
        {{1.00f, 0.78f, 0.42f}, {1.00f, 0.88f, 0.64f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{0.46f, 0.92f, 0.98f}, {0.78f, 1.00f, 1.00f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{0.66f, 0.92f, 0.46f}, {0.84f, 1.00f, 0.72f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{1.00f, 0.56f, 0.76f}, {1.00f, 0.80f, 0.90f}, 0.08f, 0.06f, 0.14f, 0.03f},
    };
    return kStyles[curveIndex % (sizeof(kStyles) / sizeof(kStyles[0]))];
}

void AppendLine(
    std::vector<DebugVertex>& vertices,
    const Float3& start,
    const Float3& end,
    const Float3& color) {
    vertices.push_back({start, color});
    vertices.push_back({end, color});
}

DrawRange BuildGroundGrid(std::vector<DebugVertex>& vertices) {
    DrawRange range = {};
    range.startVertex = static_cast<std::uint32_t>(vertices.size());

    constexpr float kGridExtent = 8.0f;
    constexpr float kGridSpacing = 0.5f;
    constexpr int kGridHalfSteps = 16;
    const Float3 gridColor = {0.24f, 0.29f, 0.36f};
    const Float3 axisColor = {0.36f, 0.43f, 0.52f};

    for (int step = -kGridHalfSteps; step <= kGridHalfSteps; ++step) {
        const float coordinate = static_cast<float>(step) * kGridSpacing;
        const Float3 color = (step == 0) ? axisColor : gridColor;

        AppendLine(vertices, {-kGridExtent, 0.0f, coordinate}, {kGridExtent, 0.0f, coordinate}, color);
        AppendLine(vertices, {coordinate, 0.0f, -kGridExtent}, {coordinate, 0.0f, kGridExtent}, color);
    }

    range.vertexCount = static_cast<std::uint32_t>(vertices.size()) - range.startVertex;
    return range;
}

void BuildCurveSegments(
    const PolylineCurve& curve,
    const CurveDebugStyle& style,
    std::vector<DebugVertex>& vertices) {
    for (std::size_t i = 1; i < curve.controlPoints.size(); ++i) {
        AppendLine(
            vertices,
            LiftPoint(curve.controlPoints[i - 1], style.yOffset),
            LiftPoint(curve.controlPoints[i], style.yOffset),
            style.segmentColor);
    }
}

void BuildControlPointMarkers(
    const PolylineCurve& curve,
    const CurveDebugStyle& style,
    std::vector<DebugVertex>& vertices) {
    for (const Float3& point : curve.controlPoints) {
        const Float3 liftedPoint = LiftPoint(point, style.yOffset);
        const Float3 crown = OffsetPoint(liftedPoint, 0.0f, style.stemHeight, 0.0f);
        AppendLine(
            vertices,
            OffsetPoint(liftedPoint, -style.baseHalfSpan, 0.0f, 0.0f),
            OffsetPoint(liftedPoint, style.baseHalfSpan, 0.0f, 0.0f),
            style.markerColor);
        AppendLine(
            vertices,
            OffsetPoint(liftedPoint, 0.0f, 0.0f, -style.baseHalfSpan),
            OffsetPoint(liftedPoint, 0.0f, 0.0f, style.baseHalfSpan),
            style.markerColor);
        AppendLine(vertices, liftedPoint, crown, style.markerColor);
        AppendLine(
            vertices,
            OffsetPoint(crown, -style.crownHalfSpan, 0.0f, 0.0f),
            OffsetPoint(crown, style.crownHalfSpan, 0.0f, 0.0f),
            style.markerColor);
        AppendLine(
            vertices,
            OffsetPoint(crown, 0.0f, 0.0f, -style.crownHalfSpan),
            OffsetPoint(crown, 0.0f, 0.0f, style.crownHalfSpan),
            style.markerColor);
    }
}

void BuildTangentSegments(
    const PolylineCurve& curve,
    const std::vector<Float3>& tangents,
    float yOffset,
    float lineLength,
    const Float3& color,
    std::vector<DebugVertex>& vertices) {
    if (curve.controlPoints.size() != tangents.size()) {
        return;
    }

    for (std::size_t i = 0; i < curve.controlPoints.size(); ++i) {
        const Float3 start = LiftPoint(curve.controlPoints[i], yOffset);
        const Float3 end = {
            start.x + tangents[i].x * lineLength,
            start.y + tangents[i].y * lineLength,
            start.z + tangents[i].z * lineLength,
        };
        AppendLine(vertices, start, end, color);
    }
}

} // namespace

bool RoadDemoDocument::AppendControlPoint(const Float3& point) {
    if (!currentCurveActive_) {
        authoredCurves_.push_back({});
        currentCurveActive_ = true;
    }

    PolylineCurve& currentCurve = authoredCurves_.back();
    if (!currentCurve.controlPoints.empty() &&
        DistanceSquared(currentCurve.controlPoints.back(), point) <= 1.0e-6f) {
        return false;
    }

    currentCurve.controlPoints.push_back(point);
    return true;
}

bool RoadDemoDocument::FinishCurrentCurve() {
    if (!currentCurveActive_) {
        return false;
    }

    currentCurveActive_ = false;
    return true;
}

void RoadDemoDocument::Clear() {
    authoredCurves_.clear();
    currentCurveActive_ = false;
}

const std::vector<PolylineCurve>& RoadDemoDocument::AuthoredCurves() const {
    return authoredCurves_;
}

bool RoadDemoDocument::CurrentCurveActive() const {
    return currentCurveActive_;
}

RoadDemoSceneGeometry BuildRoadDemoSceneGeometry(const std::vector<PolylineCurve>& authoredCurves) {
    const Float3 tangentColor = {1.00f, 0.18f, 0.48f};
    RoadDemoSceneGeometry geometry = {};

    std::vector<ProcessedCurveData> processedCurves;
    processedCurves.reserve(authoredCurves.size());
    for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
        const PolylineCurve& roughCurve = authoredCurves[curveIndex];
        if (ValidatePolylineCurve(roughCurve).has_value()) {
            continue;
        }

        ProcessedCurveData processedCurve = {};
        processedCurve.authoredIndex = curveIndex;
        processedCurve.curve = SubdividePolylineCurveTowardsBezierLimit(roughCurve);
        if (ValidatePolylineCurve(processedCurve.curve).has_value()) {
            continue;
        }

        if (ComputeCurveTangents(
                processedCurve.curve,
                kRibbonTangentMode,
                processedCurve.tangents)
                .has_value()) {
            continue;
        }

        processedCurves.push_back(std::move(processedCurve));
    }

    geometry.gridRange = BuildGroundGrid(geometry.lineVertices);

    geometry.roughCurveRange.startVertex = static_cast<std::uint32_t>(geometry.lineVertices.size());
    for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
        BuildCurveSegments(authoredCurves[curveIndex], RoughCurveStyle(curveIndex), geometry.lineVertices);
    }
    geometry.roughCurveRange.vertexCount =
        static_cast<std::uint32_t>(geometry.lineVertices.size()) - geometry.roughCurveRange.startVertex;

    geometry.roughControlPointRange.startVertex = static_cast<std::uint32_t>(geometry.lineVertices.size());
    for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
        BuildControlPointMarkers(authoredCurves[curveIndex], RoughCurveStyle(curveIndex), geometry.lineVertices);
    }
    geometry.roughControlPointRange.vertexCount =
        static_cast<std::uint32_t>(geometry.lineVertices.size()) - geometry.roughControlPointRange.startVertex;

    geometry.subdividedCurveRange.startVertex = static_cast<std::uint32_t>(geometry.lineVertices.size());
    for (const ProcessedCurveData& processedCurve : processedCurves) {
        BuildCurveSegments(
            processedCurve.curve,
            SubdividedCurveStyle(processedCurve.authoredIndex),
            geometry.lineVertices);
    }
    geometry.subdividedCurveRange.vertexCount =
        static_cast<std::uint32_t>(geometry.lineVertices.size()) - geometry.subdividedCurveRange.startVertex;

    geometry.subdividedTangentRange.startVertex = static_cast<std::uint32_t>(geometry.lineVertices.size());
    for (const ProcessedCurveData& processedCurve : processedCurves) {
        const CurveDebugStyle style = SubdividedCurveStyle(processedCurve.authoredIndex);
        BuildTangentSegments(
            processedCurve.curve,
            processedCurve.tangents,
            style.yOffset + kTangentDebugYOffset,
            kTangentDebugLength,
            tangentColor,
            geometry.lineVertices);
    }
    geometry.subdividedTangentRange.vertexCount =
        static_cast<std::uint32_t>(geometry.lineVertices.size()) - geometry.subdividedTangentRange.startVertex;

    if (processedCurves.empty()) {
        return geometry;
    }

    std::vector<PolylineCurve> roadSpines;
    roadSpines.reserve(processedCurves.size());
    for (const ProcessedCurveData& processedCurve : processedCurves) {
        roadSpines.push_back(processedCurve.curve);
    }

    GenerateRoadResult generatedRoad = {};
    if (GenerateRoad(
            roadSpines,
            kRoadCleanupRadius,
            kRibbonHalfWidth,
            generatedRoad,
            kRibbonTangentMode)
            .has_value()) {
        return geometry;
    }

    if (generatedRoad.ribbonMesh.vertices.empty() || generatedRoad.ribbonMesh.indices.empty()) {
        return geometry;
    }

    geometry.ribbonVertices = std::move(generatedRoad.ribbonMesh.vertices);
    geometry.ribbonIndices = std::move(generatedRoad.ribbonMesh.indices);
    return geometry;
}
