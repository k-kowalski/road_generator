#pragma once

#include <cstdint>
#include <vector>

#include "MathTypes.h"
#include "PolylineCurve.h"
#include "RibbonMesh.h"

struct DebugVertex {
    Float3 position;
    Float3 color;
};

struct DrawRange {
    std::uint32_t startVertex = 0;
    std::uint32_t vertexCount = 0;
};

struct RoadDemoSceneGeometry {
    std::vector<DebugVertex> lineVertices;
    DrawRange gridRange = {};
    DrawRange roughCurveRange = {};
    DrawRange roughControlPointRange = {};
    DrawRange subdividedCurveRange = {};
    DrawRange subdividedTangentRange = {};
    std::vector<RibbonVertex> ribbonVertices;
    std::vector<std::uint32_t> ribbonIndices;
};

struct RoadDemoCameraState {
    float yaw = 3.14159265f;
    float pitch = 0.48f;
    float distance = 8.4f;
    Float3 target = {0.0f, 0.15f, 0.0f};
};

class RoadDemoDocument {
public:
    bool AppendControlPoint(const Float3& point);
    bool FinishCurrentCurve();
    void Clear();

    const std::vector<PolylineCurve>& AuthoredCurves() const;
    bool CurrentCurveActive() const;

private:
    std::vector<PolylineCurve> authoredCurves_;
    bool currentCurveActive_ = false;
};

RoadDemoSceneGeometry BuildRoadDemoSceneGeometry(const std::vector<PolylineCurve>& authoredCurves);
