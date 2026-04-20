#include <cstdint>

#include <emscripten/emscripten.h>

#include "RoadDemo.h"

namespace {

struct RoadDemoContext {
    RoadDemoDocument document;
    RoadDemoSceneGeometry geometry;
};

std::uint32_t RangeStart(const DrawRange& range) {
    return range.startVertex;
}

std::uint32_t RangeCount(const DrawRange& range) {
    return range.vertexCount;
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE RoadDemoContext* road_demo_create() {
    return new RoadDemoContext();
}

EMSCRIPTEN_KEEPALIVE void road_demo_destroy(RoadDemoContext* context) {
    delete context;
}

EMSCRIPTEN_KEEPALIVE void road_demo_clear(RoadDemoContext* context) {
    if (context == nullptr) {
        return;
    }

    context->document.Clear();
    context->geometry = {};
}

EMSCRIPTEN_KEEPALIVE int road_demo_append_point(RoadDemoContext* context, float x, float y, float z) {
    if (context == nullptr) {
        return 0;
    }

    return context->document.AppendControlPoint({x, y, z}) ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE int road_demo_finish_curve(RoadDemoContext* context) {
    if (context == nullptr) {
        return 0;
    }

    return context->document.FinishCurrentCurve() ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE void road_demo_rebuild(RoadDemoContext* context) {
    if (context == nullptr) {
        return;
    }

    context->geometry = BuildRoadDemoSceneGeometry(context->document.AuthoredCurves());
}

EMSCRIPTEN_KEEPALIVE const DebugVertex* road_demo_line_vertices(RoadDemoContext* context) {
    if (context == nullptr || context->geometry.lineVertices.empty()) {
        return nullptr;
    }

    return context->geometry.lineVertices.data();
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_line_vertex_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : static_cast<std::uint32_t>(context->geometry.lineVertices.size());
}

EMSCRIPTEN_KEEPALIVE const RibbonVertex* road_demo_ribbon_vertices(RoadDemoContext* context) {
    if (context == nullptr || context->geometry.ribbonVertices.empty()) {
        return nullptr;
    }

    return context->geometry.ribbonVertices.data();
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_ribbon_vertex_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : static_cast<std::uint32_t>(context->geometry.ribbonVertices.size());
}

EMSCRIPTEN_KEEPALIVE const std::uint32_t* road_demo_ribbon_indices(RoadDemoContext* context) {
    if (context == nullptr || context->geometry.ribbonIndices.empty()) {
        return nullptr;
    }

    return context->geometry.ribbonIndices.data();
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_ribbon_index_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : static_cast<std::uint32_t>(context->geometry.ribbonIndices.size());
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_grid_start(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeStart(context->geometry.gridRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_grid_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeCount(context->geometry.gridRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_rough_curve_start(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeStart(context->geometry.roughCurveRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_rough_curve_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeCount(context->geometry.roughCurveRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_rough_control_start(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeStart(context->geometry.roughControlPointRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_rough_control_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeCount(context->geometry.roughControlPointRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_subdivided_curve_start(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeStart(context->geometry.subdividedCurveRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_subdivided_curve_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeCount(context->geometry.subdividedCurveRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_subdivided_tangent_start(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeStart(context->geometry.subdividedTangentRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_subdivided_tangent_count(RoadDemoContext* context) {
    return (context == nullptr) ? 0u : RangeCount(context->geometry.subdividedTangentRange);
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_debug_vertex_stride() {
    return static_cast<std::uint32_t>(sizeof(DebugVertex));
}

EMSCRIPTEN_KEEPALIVE std::uint32_t road_demo_ribbon_vertex_stride() {
    return static_cast<std::uint32_t>(sizeof(RibbonVertex));
}

} // extern "C"
