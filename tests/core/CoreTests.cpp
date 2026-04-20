#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "PolylineCurve.h"
#include "RibbonMesh.h"
#include "RoadDemo.h"
#include "RoadGeneration.h"

namespace {

bool ApproximatelyEqual(float left, float right, float epsilon = 1.0e-4f) {
  return std::fabs(left - right) <= epsilon;
}

bool ApproximatelyEqual(const Float3& left, const Float3& right, float epsilon = 1.0e-4f) {
  return ApproximatelyEqual(left.x, right.x, epsilon) &&
         ApproximatelyEqual(left.y, right.y, epsilon) &&
         ApproximatelyEqual(left.z, right.z, epsilon);
}

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    Fail(message);
  }
}

template <typename EnumType>
void ExpectEqual(std::optional<EnumType> actual, EnumType expected, const std::string& message) {
  if (!actual.has_value() || *actual != expected) {
    Fail(message);
  }
}

} // namespace

int main() {
  {
    PolylineCurve tooSmall = {};
    tooSmall.controlPoints.push_back({0.0f, 0.0f, 0.0f});
    ExpectEqual(
        ValidatePolylineCurve(tooSmall),
        PolylineCurveValidationError::TooFewPoints,
        "A single-point curve should be rejected.");

    PolylineCurve duplicate = {};
    duplicate.controlPoints = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
    };
    ExpectEqual(
        ValidatePolylineCurve(duplicate),
        PolylineCurveValidationError::AdjacentDuplicatePoints,
        "Adjacent duplicate control points should be rejected.");

    PolylineCurve offGround = {};
    offGround.controlPoints = {
        {0.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    };
    ExpectEqual(
        ValidatePolylineCurve(offGround),
        PolylineCurveValidationError::PointOffGroundPlane,
        "Off-ground control points should be rejected.");
  }

  {
    const PolylineCurve curve = MakeDefaultPolylineCurve();
    const PolylineCurve subdivided = SubdividePolylineCurveTowardsBezierLimit(curve);
    Expect(subdivided.controlPoints.size() == 7, "One subdivision pass should double interior samples.");
    Expect(
        ApproximatelyEqual(subdivided.controlPoints.front(), curve.controlPoints.front()) &&
            ApproximatelyEqual(subdivided.controlPoints.back(), curve.controlPoints.back()),
        "Subdivision must preserve end points.");
  }

  {
    PolylineCurve curve = {};
    curve.controlPoints = {
        {-1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
    };

    std::vector<Float3> tangents;
    Expect(
        !ComputeCurveTangents(curve, RibbonTangentMode::AverageSegmentDirections, tangents).has_value(),
        "Tangents should build for a simple straight curve.");
    Expect(tangents.size() == curve.controlPoints.size(), "Tangent count should match control point count.");

    RibbonMeshData ribbon;
    Expect(
        !BuildFlatRibbonMesh(curve, 0.2f, ribbon).has_value(),
        "Ribbon mesh should build for a simple straight curve.");
    Expect(ribbon.vertices.size() == 6, "A three-sample curve should emit six ribbon vertices.");
    Expect(ribbon.indices.size() == 12, "A three-sample curve should emit two ribbon quads.");
  }

  {
    std::vector<PolylineCurve> spines(2);
    spines[0].controlPoints = {
        {-2.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},
    };
    spines[1].controlPoints = {
        {0.0f, 0.0f, -2.0f},
        {0.0f, 0.0f, 2.0f},
    };

    GenerateRoadResult road = {};
    const auto issue = GenerateRoad(
        spines,
        0.45f,
        0.20f,
        road,
        RibbonTangentMode::AverageSegmentDirections);
    Expect(!issue.has_value(), "Crossing spines should generate a road mesh.");
    Expect(road.junctions.size() == 1, "Crossing spines should generate one junction.");
    Expect(!road.ribbonMesh.vertices.empty(), "Generated road mesh should contain vertices.");
    Expect(!road.ribbonMesh.indices.empty(), "Generated road mesh should contain indices.");
  }

  {
    RoadDemoDocument document;
    Expect(document.AppendControlPoint({-1.0f, 0.0f, 0.0f}), "First authored point should be accepted.");
    Expect(document.AppendControlPoint({0.0f, 0.0f, 0.0f}), "Second authored point should be accepted.");
    Expect(document.AppendControlPoint({1.0f, 0.0f, 0.5f}), "Third authored point should be accepted.");
    Expect(!document.AppendControlPoint({1.0f, 0.0f, 0.5f}), "Duplicate authored point should be ignored.");
    Expect(document.FinishCurrentCurve(), "The active curve should finish.");
    Expect(!document.FinishCurrentCurve(), "Finishing twice should be a no-op.");

    const RoadDemoSceneGeometry scene = BuildRoadDemoSceneGeometry(document.AuthoredCurves());
    Expect(scene.gridRange.vertexCount > 0, "The demo scene should always include the ground grid.");
    Expect(scene.roughCurveRange.vertexCount > 0, "The demo scene should include rough curve lines.");
    Expect(scene.roughControlPointRange.vertexCount > 0, "The demo scene should include authored control markers.");
    Expect(scene.subdividedCurveRange.vertexCount > 0, "The demo scene should include subdivided curves.");
    Expect(scene.subdividedTangentRange.vertexCount > 0, "The demo scene should include tangent debug lines.");
    Expect(!scene.ribbonVertices.empty(), "The demo scene should include ribbon vertices for a valid curve.");
    Expect(!scene.ribbonIndices.empty(), "The demo scene should include ribbon indices for a valid curve.");
  }

  std::cout << "core tests passed\n";
  return 0;
}
