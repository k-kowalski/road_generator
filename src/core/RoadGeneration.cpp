#include "RoadGeneration.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace {

constexpr float kCenterPatchIntersectionInfluence = 0.25f;
constexpr float kIntersectionTolerance = 1.0e-6f;
constexpr float kCurveLocationTolerance = 1.0e-4f;
constexpr float kBoundaryPointMergeDistanceSquared = 1.0e-6f;

struct CurveMetrics {
    std::vector<float> prefixLengths;
    std::vector<float> segmentLengths;
    float totalLength = 0.0f;
};

struct CurveLocation {
    CurveIndex curveIndex = 0;
    SegmentIndex segmentIndex = 0;
    float segmentT = 0.0f;
    float s = 0.0f;
    Float3 position = {};
};

struct RawIntersectionHit {
    CurveLocation first;
    CurveLocation second;
};

struct JunctionIncident {
    CurveLocation location;
};

struct JunctionNode {
    JunctionNode() = default;

    JunctionNode(NodeIndex nodeIdValue, const Float3& positionValue)
        : nodeId(nodeIdValue), position(positionValue) {}

    NodeIndex nodeId = 0;
    Float3 position = {};
    std::vector<JunctionIncident> incidents;
};

struct BlockedInterval {
    CurveIndex curveIndex = 0;
    NodeIndex nodeId = 0;
    CurveLocation startLocation;
    CurveLocation endLocation;
};

struct MergedBlockedInterval {
    CurveIndex curveIndex = 0;
    NodeIndex startNodeId = 0;
    NodeIndex endNodeId = 0;
    CurveLocation startLocation;
    CurveLocation endLocation;
};

struct SpanBoundary {
    NodeIndex nodeId = 0;
    CurveLocation location;
};

struct CurveSpan {
    CurveIndex curveIndex = 0;
    PolylineCurve curve;
    std::optional<SpanBoundary> startBoundary;
    std::optional<SpanBoundary> endBoundary;
};

struct BoundaryRef {
    BoundaryGroupIndex boundaryGroupIndex = 0;
    RibbonVertex vertex = {};
    float angle = 0.0f;
};

float DistanceSquared(const Float3& left, const Float3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

float DistanceSquaredXZ(const Float3& left, const Float3& right) {
    const float dx = left.x - right.x;
    const float dz = left.z - right.z;
    return dx * dx + dz * dz;
}

float Lerp(float start, float end, float t) {
    return start + (end - start) * t;
}

Float3 LerpPoint(const Float3& start, const Float3& end, float t) {
    return {
        Lerp(start.x, end.x, t),
        Lerp(start.y, end.y, t),
        Lerp(start.z, end.z, t),
    };
}

CurveMetrics BuildCurveMetrics(const PolylineCurve& curve) {
    CurveMetrics metrics = {};
    metrics.prefixLengths.resize(curve.controlPoints.size(), 0.0f);
    if (curve.controlPoints.size() < 2) {
        return metrics;
    }

    metrics.segmentLengths.reserve(curve.controlPoints.size() - 1);
    for (std::size_t i = 0; i + 1 < curve.controlPoints.size(); ++i) {
        const float segmentLength = std::sqrt(DistanceSquared(curve.controlPoints[i], curve.controlPoints[i + 1]));
        metrics.segmentLengths.push_back(segmentLength);
        metrics.prefixLengths[i + 1] = metrics.prefixLengths[i] + segmentLength;
    }

    metrics.totalLength = metrics.prefixLengths.back();
    return metrics;
}

CurveLocation MakeCurveLocation(
    CurveIndex curveIndex,
    SegmentIndex segmentIndex,
    float segmentT,
    const Float3& position,
    const CurveMetrics& metrics) {
    CurveLocation location = {};
    location.curveIndex = curveIndex;
    location.segmentIndex = segmentIndex;
    location.segmentT = segmentT;
    location.position = position;
    location.s = metrics.prefixLengths[segmentIndex] + metrics.segmentLengths[segmentIndex] * segmentT;
    return location;
}

CurveLocation CurveStartLocation(
    CurveIndex curveIndex,
    const PolylineCurve& curve,
    const CurveMetrics&) {
    CurveLocation location = {};
    location.curveIndex = curveIndex;
    location.segmentIndex = 0;
    location.segmentT = 0.0f;
    location.s = 0.0f;
    location.position = curve.controlPoints.front();
    return location;
}

CurveLocation CurveEndLocation(
    CurveIndex curveIndex,
    const PolylineCurve& curve,
    const CurveMetrics& metrics) {
    CurveLocation location = {};
    location.curveIndex = curveIndex;
    location.segmentIndex = curve.controlPoints.size() - 2;
    location.segmentT = 1.0f;
    location.s = metrics.totalLength;
    location.position = curve.controlPoints.back();
    return location;
}

bool SameCurveLocation(const CurveLocation& left, const CurveLocation& right) {
    return left.curveIndex == right.curveIndex &&
           std::fabs(left.s - right.s) <= kCurveLocationTolerance;
}

void AppendDistinctPoint(std::vector<Float3>& points, const Float3& point) {
    if (!points.empty() && DistanceSquared(points.back(), point) <= kBoundaryPointMergeDistanceSquared) {
        return;
    }

    points.push_back(point);
}

float BoundaryAngle(const Float3& center, const Float3& point) {
    return std::atan2(point.z - center.z, point.x - center.x);
}

Float2 LerpUv(const Float2& start, const Float2& end, float t) {
    return {
        Lerp(start.x, end.x, t),
        Lerp(start.y, end.y, t),
    };
}

Float2 AverageUv(const Float2& left, const Float2& right) {
    return LerpUv(left, right, 0.5f);
}

Float3 SmoothTowardIntersection(
    const Float3& left,
    const Float3& center,
    const Float3& right,
    float centerInfluence) {
    const float sideInfluence = 0.5f * (1.0f - centerInfluence);
    return {
        sideInfluence * left.x + centerInfluence * center.x + sideInfluence * right.x,
        sideInfluence * left.y + centerInfluence * center.y + sideInfluence * right.y,
        sideInfluence * left.z + centerInfluence * center.z + sideInfluence * right.z,
    };
}

void AppendRibbonMesh(const RibbonMeshData& source, RibbonMeshData& destination) {
    const std::uint32_t vertexOffset = static_cast<std::uint32_t>(destination.vertices.size());
    destination.vertices.insert(destination.vertices.end(), source.vertices.begin(), source.vertices.end());
    destination.indices.reserve(destination.indices.size() + source.indices.size());

    for (std::uint32_t index : source.indices) {
        destination.indices.push_back(vertexOffset + index);
    }
}

void CollectSpanBoundary(
    const Float3& nodePosition,
    BoundaryGroupIndex boundaryGroupIndex,
    bool seamAtStart,
    const RibbonMeshData& spanRibbonMesh,
    std::vector<BoundaryRef>& boundaryRefs) {
    if (spanRibbonMesh.vertices.size() < 2) {
        return;
    }

    const std::uint32_t seamStartVertex =
        seamAtStart ? 0u : static_cast<std::uint32_t>(spanRibbonMesh.vertices.size() - 2);
    for (std::uint32_t localIndex = seamStartVertex; localIndex < seamStartVertex + 2; ++localIndex) {
        const RibbonVertex& seamVertex = spanRibbonMesh.vertices[localIndex];
        boundaryRefs.push_back({
            boundaryGroupIndex,
            seamVertex,
            BoundaryAngle(nodePosition, seamVertex.position),
        });
    }
}

void AppendCenterPatch(
    const Float3& intersectionPoint,
    const std::vector<BoundaryRef>& boundaryRefs,
    RibbonMeshData& ribbonMesh) {
    std::vector<BoundaryRef> sortedBoundaryRefs = boundaryRefs;
    std::sort(
        sortedBoundaryRefs.begin(),
        sortedBoundaryRefs.end(),
        [](const BoundaryRef& left, const BoundaryRef& right) {
            return left.angle < right.angle;
        });

    std::vector<std::uint32_t> patchRingIndices;
    patchRingIndices.reserve(sortedBoundaryRefs.size() * 2);
    Float2 accumulatedPatchUv = {0.0f, 0.0f};
    std::size_t accumulatedPatchUvCount = 0;

    for (BoundaryGroupIndex i = 0; i < sortedBoundaryRefs.size(); ++i) {
        const BoundaryRef& current = sortedBoundaryRefs[i];
        const BoundaryRef& next = sortedBoundaryRefs[(i + 1) % sortedBoundaryRefs.size()];
        const std::uint32_t duplicatedBoundaryVertexIndex = static_cast<std::uint32_t>(ribbonMesh.vertices.size());
        RibbonVertex duplicatedBoundaryVertex = current.vertex;
        duplicatedBoundaryVertex.surfaceKind = kRibbonSurfaceIntersection;
        ribbonMesh.vertices.push_back(duplicatedBoundaryVertex);
        patchRingIndices.push_back(duplicatedBoundaryVertexIndex);
        accumulatedPatchUv.x += duplicatedBoundaryVertex.uv.x;
        accumulatedPatchUv.y += duplicatedBoundaryVertex.uv.y;
        ++accumulatedPatchUvCount;

        if (current.boundaryGroupIndex == next.boundaryGroupIndex) {
            continue;
        }

        RibbonVertex insertedVertex = {};
        insertedVertex.position = SmoothTowardIntersection(
            current.vertex.position,
            intersectionPoint,
            next.vertex.position,
            kCenterPatchIntersectionInfluence);
        insertedVertex.uv = AverageUv(current.vertex.uv, next.vertex.uv);
        insertedVertex.surfaceKind = kRibbonSurfaceIntersection;
        insertedVertex.color = kRibbonWireframeColor;

        const std::uint32_t insertedVertexIndex = static_cast<std::uint32_t>(ribbonMesh.vertices.size());
        ribbonMesh.vertices.push_back(insertedVertex);
        patchRingIndices.push_back(insertedVertexIndex);
        accumulatedPatchUv.x += insertedVertex.uv.x;
        accumulatedPatchUv.y += insertedVertex.uv.y;
        ++accumulatedPatchUvCount;
    }

    const std::uint32_t centerVertexIndex = static_cast<std::uint32_t>(ribbonMesh.vertices.size());
    RibbonVertex centerVertex = {};
    centerVertex.position = {
        intersectionPoint.x,
        intersectionPoint.y,
        intersectionPoint.z,
    };
    if (accumulatedPatchUvCount > 0) {
        centerVertex.uv = {
            accumulatedPatchUv.x / static_cast<float>(accumulatedPatchUvCount),
            accumulatedPatchUv.y / static_cast<float>(accumulatedPatchUvCount),
        };
    } else {
        centerVertex.uv = {0.5f, 0.5f};
    }
    centerVertex.surfaceKind = kRibbonSurfaceIntersection;
    centerVertex.color = kRibbonWireframeColor;
    ribbonMesh.vertices.push_back(centerVertex);

    ribbonMesh.indices.reserve(ribbonMesh.indices.size() + patchRingIndices.size() * 3);
    for (BoundaryGroupIndex i = 0; i < patchRingIndices.size(); ++i) {
        const std::uint32_t current = patchRingIndices[i];
        const std::uint32_t next = patchRingIndices[(i + 1) % patchRingIndices.size()];
        ribbonMesh.indices.push_back(centerVertexIndex);
        ribbonMesh.indices.push_back(current);
        ribbonMesh.indices.push_back(next);
    }
}

std::vector<RawIntersectionHit> CollectRawIntersections(
    const std::vector<PolylineCurve>& curves,
    const std::vector<CurveMetrics>& curveMetrics) {
    std::vector<RawIntersectionHit> rawHits;

    for (CurveIndex curveIndex = 0; curveIndex < curves.size(); ++curveIndex) {
        const PolylineCurve& curve = curves[curveIndex];
        for (SegmentIndex firstSegment = 0; firstSegment + 1 < curve.controlPoints.size(); ++firstSegment) {
            for (SegmentIndex secondSegment = firstSegment + 2; secondSegment + 1 < curve.controlPoints.size();
                 ++secondSegment) {
                const auto intersection = IntersectSegments(
                    curve.controlPoints[firstSegment],
                    curve.controlPoints[firstSegment + 1],
                    curve.controlPoints[secondSegment],
                    curve.controlPoints[secondSegment + 1],
                    kIntersectionTolerance);
                if (!intersection.has_value()) {
                    continue;
                }

                rawHits.push_back({
                    MakeCurveLocation(
                        curveIndex,
                        firstSegment,
                        intersection->firstSegmentT,
                        intersection->position,
                        curveMetrics[curveIndex]),
                    MakeCurveLocation(
                        curveIndex,
                        secondSegment,
                        intersection->secondSegmentT,
                        intersection->position,
                        curveMetrics[curveIndex]),
                });
            }
        }
    }

    for (CurveIndex firstCurveIndex = 0; firstCurveIndex < curves.size(); ++firstCurveIndex) {
        for (CurveIndex secondCurveIndex = firstCurveIndex + 1; secondCurveIndex < curves.size(); ++secondCurveIndex) {
            const PolylineCurve& firstCurve = curves[firstCurveIndex];
            const PolylineCurve& secondCurve = curves[secondCurveIndex];
            for (SegmentIndex firstSegment = 0; firstSegment + 1 < firstCurve.controlPoints.size(); ++firstSegment) {
                for (SegmentIndex secondSegment = 0; secondSegment + 1 < secondCurve.controlPoints.size();
                     ++secondSegment) {
                    const auto intersection = IntersectSegments(
                        firstCurve.controlPoints[firstSegment],
                        firstCurve.controlPoints[firstSegment + 1],
                        secondCurve.controlPoints[secondSegment],
                        secondCurve.controlPoints[secondSegment + 1],
                        kIntersectionTolerance);
                    if (!intersection.has_value()) {
                        continue;
                    }

                    rawHits.push_back({
                        MakeCurveLocation(
                            firstCurveIndex,
                            firstSegment,
                            intersection->firstSegmentT,
                            intersection->position,
                            curveMetrics[firstCurveIndex]),
                        MakeCurveLocation(
                            secondCurveIndex,
                            secondSegment,
                            intersection->secondSegmentT,
                            intersection->position,
                            curveMetrics[secondCurveIndex]),
                    });
                }
            }
        }
    }

    return rawHits;
}

void AddNodeIncident(JunctionNode& node, const CurveLocation& location) {
    for (const JunctionIncident& existingIncident : node.incidents) {
        if (SameCurveLocation(existingIncident.location, location)) {
            return;
        }
    }

    node.incidents.push_back({location});
}

std::vector<JunctionNode> ClusterIntersectionNodes(const std::vector<RawIntersectionHit>& rawHits) {
    std::vector<JunctionNode> nodes;
    const float mergeDistanceSquared = kCurveLocationTolerance * kCurveLocationTolerance;

    for (const RawIntersectionHit& hit : rawHits) {
        const Float3& hitPosition = hit.first.position;

        JunctionNode* node = nullptr;
        for (JunctionNode& candidate : nodes) {
            if (DistanceSquaredXZ(candidate.position, hitPosition) <= mergeDistanceSquared) {
                node = &candidate;
                break;
            }
        }

        if (node == nullptr) {
            nodes.emplace_back(nodes.size(), hitPosition);
            node = &nodes.back();
        }

        AddNodeIncident(*node, hit.first);
        AddNodeIncident(*node, hit.second);
    }

    return nodes;
}

std::vector<CurveLocation> IntersectCurveCircle(
    const PolylineCurve& curve,
    CurveIndex curveIndex,
    const CurveMetrics& metrics,
    const Float3& center,
    float radius) {
    std::vector<CurveLocation> hits;
    const Vector2f centerGround = ProjectPointXZ(center);
    const float radiusSquared = radius * radius;

    for (SegmentIndex segmentIndex = 0; segmentIndex + 1 < curve.controlPoints.size(); ++segmentIndex) {
        const Float3& start = curve.controlPoints[segmentIndex];
        const Float3& end = curve.controlPoints[segmentIndex + 1];
        const Vector2f startGround = ProjectPointXZ(start);
        const Vector2f endGround = ProjectPointXZ(end);

        const float directionX = endGround.x - startGround.x;
        const float directionZ = endGround.z - startGround.z;
        const float directionLengthSquared = directionX * directionX + directionZ * directionZ;
        if (directionLengthSquared <= kIntersectionTolerance * kIntersectionTolerance) {
            continue;
        }

        const float centerOffsetX = centerGround.x - startGround.x;
        const float centerOffsetZ = centerGround.z - startGround.z;
        const float closestT =
            (centerOffsetX * directionX + centerOffsetZ * directionZ) / directionLengthSquared;
        const float closestPointX = startGround.x + directionX * closestT;
        const float closestPointZ = startGround.z + directionZ * closestT;
        const float centerToClosestX = closestPointX - centerGround.x;
        const float centerToClosestZ = closestPointZ - centerGround.z;
        const float closestDistanceSquared = centerToClosestX * centerToClosestX + centerToClosestZ * centerToClosestZ;
        if (closestDistanceSquared > radiusSquared + kIntersectionTolerance) {
            continue;
        }

        const float offsetDistanceSquared = std::max(0.0f, radiusSquared - closestDistanceSquared);
        const float offsetT = std::sqrt(offsetDistanceSquared / directionLengthSquared);
        const float segmentTs[2] = {
            closestT - offsetT,
            closestT + offsetT,
        };

        for (float segmentT : segmentTs) {
            if (segmentT < -kIntersectionTolerance || segmentT > 1.0f + kIntersectionTolerance) {
                continue;
            }

            const float clampedT = std::clamp(segmentT, 0.0f, 1.0f);
            const Float3 position = LerpPoint(start, end, clampedT);
            hits.push_back(MakeCurveLocation(curveIndex, segmentIndex, clampedT, position, metrics));
        }
    }

    std::sort(
        hits.begin(),
        hits.end(),
        [](const CurveLocation& left, const CurveLocation& right) {
            return left.s < right.s;
        });

    hits.erase(
        std::unique(
            hits.begin(),
            hits.end(),
            [](const CurveLocation& left, const CurveLocation& right) {
                return std::fabs(left.s - right.s) <= kCurveLocationTolerance ||
                       DistanceSquared(left.position, right.position) <= kBoundaryPointMergeDistanceSquared;
            }),
        hits.end());

    return hits;
}

std::optional<BlockedInterval> BuildBlockedInterval(
    const JunctionNode& node,
    const JunctionIncident& incident,
    const PolylineCurve& curve,
    const CurveMetrics& metrics,
    const std::vector<CurveLocation>& circleHits) {
    CurveLocation startLocation = CurveStartLocation(incident.location.curveIndex, curve, metrics);
    CurveLocation endLocation = CurveEndLocation(incident.location.curveIndex, curve, metrics);

    for (const CurveLocation& hit : circleHits) {
        if (hit.s < incident.location.s - kCurveLocationTolerance) {
            startLocation = hit;
            continue;
        }

        if (hit.s > incident.location.s + kCurveLocationTolerance) {
            endLocation = hit;
            break;
        }
    }

    if (endLocation.s <= startLocation.s + kCurveLocationTolerance) {
        return std::nullopt;
    }

    BlockedInterval interval = {};
    interval.curveIndex = incident.location.curveIndex;
    interval.nodeId = node.nodeId;
    interval.startLocation = startLocation;
    interval.endLocation = endLocation;
    return interval;
}

std::vector<BlockedInterval> BuildBlockedIntervals(
    const std::vector<PolylineCurve>& curves,
    const std::vector<CurveMetrics>& curveMetrics,
    const std::vector<JunctionNode>& nodes,
    float cleanupRadius) {
    std::vector<BlockedInterval> intervals;

    for (const JunctionNode& node : nodes) {
        std::vector<std::pair<CurveIndex, std::vector<CurveLocation>>> circleHitsByCurve;

        for (const JunctionIncident& incident : node.incidents) {
            const CurveIndex curveIndex = incident.location.curveIndex;
            auto hitIt = std::find_if(
                circleHitsByCurve.begin(),
                circleHitsByCurve.end(),
                [&](const auto& entry) {
                    return entry.first == curveIndex;
                });

            if (hitIt == circleHitsByCurve.end()) {
                circleHitsByCurve.push_back({
                    curveIndex,
                    IntersectCurveCircle(
                        curves[curveIndex],
                        curveIndex,
                        curveMetrics[curveIndex],
                        node.position,
                        cleanupRadius),
                });
                hitIt = circleHitsByCurve.end() - 1;
            }

            const auto interval = BuildBlockedInterval(
                node,
                incident,
                curves[curveIndex],
                curveMetrics[curveIndex],
                hitIt->second);
            if (interval.has_value()) {
                intervals.push_back(*interval);
            }
        }
    }

    return intervals;
}

std::vector<MergedBlockedInterval> MergeBlockedIntervalsForCurve(std::vector<BlockedInterval> intervals) {
    std::vector<MergedBlockedInterval> merged;
    if (intervals.empty()) {
        return merged;
    }

    std::sort(
        intervals.begin(),
        intervals.end(),
        [](const BlockedInterval& left, const BlockedInterval& right) {
            if (left.startLocation.s != right.startLocation.s) {
                return left.startLocation.s < right.startLocation.s;
            }

            return left.endLocation.s < right.endLocation.s;
        });

    for (const BlockedInterval& interval : intervals) {
        if (merged.empty() ||
            interval.startLocation.s > merged.back().endLocation.s + kCurveLocationTolerance) {
            merged.push_back({
                interval.curveIndex,
                interval.nodeId,
                interval.nodeId,
                interval.startLocation,
                interval.endLocation,
            });
            continue;
        }

        if (interval.endLocation.s > merged.back().endLocation.s + kCurveLocationTolerance) {
            merged.back().endLocation = interval.endLocation;
            merged.back().endNodeId = interval.nodeId;
        }
    }

    return merged;
}

PolylineCurve ExtractCurveSpan(
    const PolylineCurve& curve,
    const CurveLocation& startLocation,
    const CurveLocation& endLocation) {
    PolylineCurve spanCurve;
    if (endLocation.s <= startLocation.s + kCurveLocationTolerance) {
        return spanCurve;
    }

    AppendDistinctPoint(spanCurve.controlPoints, startLocation.position);
    if (startLocation.segmentIndex != endLocation.segmentIndex) {
        for (SegmentIndex pointIndex = startLocation.segmentIndex + 1;
             pointIndex <= endLocation.segmentIndex;
             ++pointIndex) {
            AppendDistinctPoint(spanCurve.controlPoints, curve.controlPoints[pointIndex]);
        }
    }
    AppendDistinctPoint(spanCurve.controlPoints, endLocation.position);
    return spanCurve;
}

std::vector<CurveSpan> BuildCurveSpans(
    const std::vector<PolylineCurve>& curves,
    const std::vector<CurveMetrics>& curveMetrics,
    const std::vector<BlockedInterval>& blockedIntervals) {
    std::vector<CurveSpan> spans;

    for (CurveIndex curveIndex = 0; curveIndex < curves.size(); ++curveIndex) {
        std::vector<BlockedInterval> perCurveIntervals;
        for (const BlockedInterval& interval : blockedIntervals) {
            if (interval.curveIndex == curveIndex) {
                perCurveIntervals.push_back(interval);
            }
        }

        const PolylineCurve& curve = curves[curveIndex];
        const CurveMetrics& metrics = curveMetrics[curveIndex];
        const CurveLocation curveStart = CurveStartLocation(curveIndex, curve, metrics);
        const CurveLocation curveEnd = CurveEndLocation(curveIndex, curve, metrics);

        if (perCurveIntervals.empty()) {
            CurveSpan span = {};
            span.curveIndex = curveIndex;
            span.curve = curve;
            spans.push_back(std::move(span));
            continue;
        }

        const std::vector<MergedBlockedInterval> mergedIntervals = MergeBlockedIntervalsForCurve(std::move(perCurveIntervals));
        CurveLocation cursor = curveStart;
        std::optional<SpanBoundary> startBoundary;

        for (const MergedBlockedInterval& interval : mergedIntervals) {
            if (interval.startLocation.s > cursor.s + kCurveLocationTolerance) {
                CurveSpan span = {};
                span.curveIndex = curveIndex;
                span.curve = ExtractCurveSpan(curve, cursor, interval.startLocation);
                span.startBoundary = startBoundary;
                span.endBoundary = SpanBoundary{interval.startNodeId, interval.startLocation};
                if (span.curve.controlPoints.size() >= 2) {
                    spans.push_back(std::move(span));
                }
            }

            cursor = interval.endLocation;
            startBoundary = SpanBoundary{interval.endNodeId, interval.endLocation};
        }

        if (curveEnd.s > cursor.s + kCurveLocationTolerance) {
            CurveSpan span = {};
            span.curveIndex = curveIndex;
            span.curve = ExtractCurveSpan(curve, cursor, curveEnd);
            span.startBoundary = startBoundary;
            if (span.curve.controlPoints.size() >= 2) {
                spans.push_back(std::move(span));
            }
        }
    }

    return spans;
}

} // namespace

std::optional<GenerateRoadIssue> GenerateRoad(
    const std::vector<PolylineCurve>& spines,
    float cleanupRadius,
    float ribbonHalfWidth,
    GenerateRoadResult& road,
    RibbonTangentMode tangentMode) {
    road = {};

    std::vector<CurveMetrics> curveMetrics;
    curveMetrics.reserve(spines.size());
    for (CurveIndex spineIndex = 0; spineIndex < spines.size(); ++spineIndex) {
        if (const auto error = ValidatePolylineCurve(spines[spineIndex])) {
            return GenerateRoadIssue{
                GenerateRoadError::InvalidSpineCurve,
                spineIndex,
                0,
                *error,
                {}};
        }

        curveMetrics.push_back(BuildCurveMetrics(spines[spineIndex]));
    }

    const std::vector<RawIntersectionHit> rawHits = CollectRawIntersections(spines, curveMetrics);
    const std::vector<JunctionNode> nodes = ClusterIntersectionNodes(rawHits);
    const std::vector<BlockedInterval> blockedIntervals =
        BuildBlockedIntervals(spines, curveMetrics, nodes, cleanupRadius);
    const std::vector<CurveSpan> spans = BuildCurveSpans(spines, curveMetrics, blockedIntervals);

    road.junctions.reserve(nodes.size());
    for (const JunctionNode& node : nodes) {
        road.junctions.push_back({node.position, node.incidents.size()});
    }

    std::vector<std::vector<BoundaryRef>> nodeBoundaryRefs(nodes.size());
    BoundaryGroupIndex nextBoundaryGroupIndex = 0;

    for (SpanIndex spanIndex = 0; spanIndex < spans.size(); ++spanIndex) {
        const CurveSpan& span = spans[spanIndex];
        RibbonMeshData spanRibbonMesh;
        if (const auto issue = BuildFlatRibbonMesh(
                span.curve,
                ribbonHalfWidth,
                spanRibbonMesh,
                tangentMode)) {
            return GenerateRoadIssue{
                GenerateRoadError::RibbonBuildFailed,
                span.curveIndex,
                spanIndex,
                PolylineCurveValidationError::TooFewPoints,
                *issue};
        }

        AppendRibbonMesh(spanRibbonMesh, road.ribbonMesh);
        if (span.startBoundary.has_value()) {
            CollectSpanBoundary(
                nodes[span.startBoundary->nodeId].position,
                nextBoundaryGroupIndex++,
                true,
                spanRibbonMesh,
                nodeBoundaryRefs[span.startBoundary->nodeId]);
        }

        if (span.endBoundary.has_value()) {
            CollectSpanBoundary(
                nodes[span.endBoundary->nodeId].position,
                nextBoundaryGroupIndex++,
                false,
                spanRibbonMesh,
                nodeBoundaryRefs[span.endBoundary->nodeId]);
        }
    }

    if (road.ribbonMesh.vertices.empty() || road.ribbonMesh.indices.empty()) {
        return GenerateRoadIssue{
            GenerateRoadError::NoRibbonOutput,
            0,
            0,
            PolylineCurveValidationError::TooFewPoints,
            {}};
    }

    for (NodeIndex nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
        if (nodeBoundaryRefs[nodeIndex].size() < 3) {
            continue;
        }

        AppendCenterPatch(
            nodes[nodeIndex].position,
            nodeBoundaryRefs[nodeIndex],
            road.ribbonMesh);
    }

    return std::nullopt;
}

std::string GenerateRoadErrorMessage(const GenerateRoadIssue& issue) {
    switch (issue.error) {
    case GenerateRoadError::InvalidSpineCurve:
        return "Road generation failed: spine " + std::to_string(issue.spineIndex) +
               " is invalid. " + PolylineCurveValidationErrorMessage(issue.validationError);
    case GenerateRoadError::RibbonBuildFailed:
        return "Road generation failed: ribbon build for span " +
               std::to_string(issue.spanIndex) + " failed. " +
               RibbonMeshBuildErrorMessage(issue.ribbonIssue);
    case GenerateRoadError::NoRibbonOutput:
        return "Road generation failed: no ribbon mesh could be produced from the current curve network.";
    default:
        return "Road generation failed.";
    }
}
