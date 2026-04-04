#include "PolylineCurve.h"

#include <algorithm>
#include <cmath>

namespace {

struct SegmentCircleHit {
    DirectX::XMFLOAT3 position = {};
    std::size_t segmentIndex = 0;
    float segmentT = 0.0f;
};

float DistanceSquared(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

DirectX::XMFLOAT3 NormalizeDirection(
    const DirectX::XMFLOAT3& start,
    const DirectX::XMFLOAT3& end) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float dz = end.z - start.z;
    const float lengthSquared = dx * dx + dy * dy + dz * dz;
    if (lengthSquared <= 1.0e-8f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float inverseLength = 1.0f / std::sqrt(lengthSquared);
    return {
        dx * inverseLength,
        dy * inverseLength,
        dz * inverseLength,
    };
}

Vector2f Subtract(const Vector2f& end, const Vector2f& start) {
    return {end.x - start.x, end.z - start.z};
}

Vector2f Add(const Vector2f& left, const Vector2f& right) {
    return {left.x + right.x, left.z + right.z};
}

Vector2f Scale(const Vector2f& value, float scalar) {
    return {value.x * scalar, value.z * scalar};
}

float Cross2D(const Vector2f& left, const Vector2f& right) {
    return left.x * right.z - left.z * right.x;
}

float Dot2D(const Vector2f& left, const Vector2f& right) {
    return left.x * right.x + left.z * right.z;
}

float Lerp(float start, float end, float t) {
    return start + (end - start) * t;
}

DirectX::XMFLOAT3 LerpPoint(const DirectX::XMFLOAT3& start, const DirectX::XMFLOAT3& end, float t) {
    return {
        Lerp(start.x, end.x, t),
        Lerp(start.y, end.y, t),
        Lerp(start.z, end.z, t),
    };
}

DirectX::XMFLOAT3 Midpoint(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    return {
        0.5f * (left.x + right.x),
        0.5f * (left.y + right.y),
        0.5f * (left.z + right.z),
    };
}

DirectX::XMFLOAT3 SmoothInteriorPoint(
    const DirectX::XMFLOAT3& left,
    const DirectX::XMFLOAT3& center,
    const DirectX::XMFLOAT3& right) {
    return {
        0.25f * left.x + 0.50f * center.x + 0.25f * right.x,
        0.25f * left.y + 0.50f * center.y + 0.25f * right.y,
        0.25f * left.z + 0.50f * center.z + 0.25f * right.z,
    };
}

bool HitSortsBefore(const SegmentCircleHit& left, const SegmentCircleHit& right) {
    if (left.segmentIndex != right.segmentIndex) {
        return left.segmentIndex < right.segmentIndex;
    }

    return left.segmentT < right.segmentT;
}

void SortAndDeduplicateHits(
    std::vector<SegmentCircleHit>& hits,
    float parameterTolerance,
    float positionToleranceSquared) {
    std::sort(hits.begin(), hits.end(), HitSortsBefore);

    hits.erase(
        std::unique(
            hits.begin(),
            hits.end(),
            [&](const SegmentCircleHit& left, const SegmentCircleHit& right) {
                const bool sameCurveLocation =
                    left.segmentIndex == right.segmentIndex &&
                    std::fabs(left.segmentT - right.segmentT) <= parameterTolerance;
                const bool samePosition =
                    DistanceSquared(left.position, right.position) <= positionToleranceSquared;
                return sameCurveLocation || samePosition;
            }),
        hits.end());
}

void AppendTrimEndpoint(
    std::vector<DirectX::XMFLOAT3>& points,
    const DirectX::XMFLOAT3& trimPoint,
    float mergeDistanceSquared) {
    if (!points.empty() && DistanceSquared(points.back(), trimPoint) <= mergeDistanceSquared) {
        return;
    }

    points.push_back(trimPoint);
}

TrimmedCurveBranch BuildTrimmedPrefix(
    const PolylineCurve& curve,
    const SegmentCircleHit& firstHit,
    float mergeDistanceSquared) {
    TrimmedCurveBranch trimmed = {};
    trimmed.curve.controlPoints.reserve(firstHit.segmentIndex + 2);

    for (std::size_t i = 0; i <= firstHit.segmentIndex; ++i) {
        trimmed.curve.controlPoints.push_back(curve.controlPoints[i]);
    }

    AppendTrimEndpoint(trimmed.curve.controlPoints, firstHit.position, mergeDistanceSquared);
    trimmed.trimPoint = firstHit.position;
    trimmed.tangentAwayFromIntersection = NormalizeDirection(
        curve.controlPoints[firstHit.segmentIndex + 1],
        curve.controlPoints[firstHit.segmentIndex]);
    return trimmed;
}

TrimmedCurveBranch BuildTrimmedSuffix(
    const PolylineCurve& curve,
    const SegmentCircleHit& lastHit,
    float mergeDistanceSquared) {
    TrimmedCurveBranch trimmed = {};
    trimmed.curve.controlPoints.reserve(curve.controlPoints.size() - lastHit.segmentIndex);

    for (std::size_t i = curve.controlPoints.size(); i-- > lastHit.segmentIndex + 1;) {
        trimmed.curve.controlPoints.push_back(curve.controlPoints[i]);
    }

    AppendTrimEndpoint(trimmed.curve.controlPoints, lastHit.position, mergeDistanceSquared);
    std::reverse(trimmed.curve.controlPoints.begin(), trimmed.curve.controlPoints.end());
    trimmed.trimPoint = lastHit.position;
    trimmed.tangentAwayFromIntersection = NormalizeDirection(
        curve.controlPoints[lastHit.segmentIndex],
        curve.controlPoints[lastHit.segmentIndex + 1]);
    return trimmed;
}

} // namespace

Vector2f ProjectPointXZ(const DirectX::XMFLOAT3& point) {
    return {point.x, point.z};
}

std::optional<CurveTrimResult> TrimCurvesCircle(const PolylineCurve& curve1, const PolylineCurve& curve2, const Vector2f& center, float radius)
{
    if (curve1.controlPoints.size() < 2 || curve2.controlPoints.size() < 2) {
        return std::nullopt;
    }

    if (radius <= 0.0f) {
        return std::nullopt;
    }

    const float radiusSquared = radius * radius;
    const float parameterTolerance = 1.0e-6f;
    const float duplicateToleranceSquared = parameterTolerance * parameterTolerance;
    const float trimEndpointMergeDistance = 1.0e-3f;
    const float trimEndpointMergeDistanceSquared =
        trimEndpointMergeDistance * trimEndpointMergeDistance;

    auto intersectSegmentCircle = [&](const DirectX::XMFLOAT3& start, const DirectX::XMFLOAT3& end, std::size_t segmentIndex) {
        std::vector<SegmentCircleHit> hits;
        const Vector2f startGround = ProjectPointXZ(start);
        const Vector2f endGround = ProjectPointXZ(end);
        const Vector2f direction = Subtract(endGround, startGround);
        const float directionLengthSquared = Dot2D(direction, direction);
        if (directionLengthSquared <= parameterTolerance * parameterTolerance) {
            return hits;
        }

        const Vector2f centerFromStart = Subtract(center, startGround);
        const float closestT = Dot2D(centerFromStart, direction) / directionLengthSquared;
        const Vector2f closestPoint = Add(startGround, Scale(direction, closestT));
        const Vector2f centerToClosest = Subtract(closestPoint, center);
        const float closestDistanceSquared = Dot2D(centerToClosest, centerToClosest);
        if (closestDistanceSquared > radiusSquared + parameterTolerance) {
            return hits;
        }

        const float offsetDistanceSquared = std::max(0.0f, radiusSquared - closestDistanceSquared);
        const float offsetT = std::sqrt(offsetDistanceSquared / directionLengthSquared);
        const float firstT = closestT - offsetT;
        const float secondT = closestT + offsetT;

        auto appendHitIfOnSegment = [&](float segmentT) {
            if (segmentT < -parameterTolerance || segmentT > 1.0f + parameterTolerance) {
                return;
            }

            const float clampedT = std::clamp(segmentT, 0.0f, 1.0f);
            SegmentCircleHit hit = {};
            hit.segmentIndex = segmentIndex;
            hit.segmentT = clampedT;
            hit.position = LerpPoint(start, end, clampedT);

            for (const SegmentCircleHit& existingHit : hits) {
                if (DistanceSquared(existingHit.position, hit.position) <= parameterTolerance * parameterTolerance) {
                    return;
                }
            }

            hits.push_back(hit);
        };

        appendHitIfOnSegment(firstT);
        appendHitIfOnSegment(secondT);
        return hits;
    };

    std::vector<SegmentCircleHit> curve1Hits;
    for (std::size_t i = 0; i + 1 < curve1.controlPoints.size(); ++i) {
        const auto hits = intersectSegmentCircle(
            curve1.controlPoints[i],
            curve1.controlPoints[i + 1],
            i);
        curve1Hits.insert(curve1Hits.end(), hits.begin(), hits.end());
    }
    SortAndDeduplicateHits(curve1Hits, parameterTolerance, duplicateToleranceSquared);

    std::vector<SegmentCircleHit> curve2Hits;
    for (std::size_t i = 0; i + 1 < curve2.controlPoints.size(); ++i) {
        const auto hits = intersectSegmentCircle(
            curve2.controlPoints[i],
            curve2.controlPoints[i + 1],
            i);
        curve2Hits.insert(curve2Hits.end(), hits.begin(), hits.end());
    }
    SortAndDeduplicateHits(curve2Hits, parameterTolerance, duplicateToleranceSquared);

    if (curve1Hits.size() != 2 || curve2Hits.size() != 2) {
        return std::nullopt;
    }

    CurveTrimResult result = {};
    result.curve11 = BuildTrimmedPrefix(
        curve1,
        curve1Hits.front(),
        trimEndpointMergeDistanceSquared);
    result.curve12 = BuildTrimmedSuffix(
        curve1,
        curve1Hits.back(),
        trimEndpointMergeDistanceSquared);
    result.curve21 = BuildTrimmedPrefix(
        curve2,
        curve2Hits.front(),
        trimEndpointMergeDistanceSquared);
    result.curve22 = BuildTrimmedSuffix(
        curve2,
        curve2Hits.back(),
        trimEndpointMergeDistanceSquared);
    return result;
}

std::optional<SegmentIntersection> IntersectSegments(
    const DirectX::XMFLOAT3& a,
    const DirectX::XMFLOAT3& b,
    const DirectX::XMFLOAT3& c,
    const DirectX::XMFLOAT3& d,
    float epsilon) {
    const Vector2f aGround = ProjectPointXZ(a);
    const Vector2f bGround = ProjectPointXZ(b);
    const Vector2f cGround = ProjectPointXZ(c);
    const Vector2f dGround = ProjectPointXZ(d);

    const Vector2f firstDirection = Subtract(bGround, aGround);
    const Vector2f secondDirection = Subtract(dGround, cGround);
    const Vector2f offset = Subtract(cGround, aGround);

    const float denominator = Cross2D(firstDirection, secondDirection);
    if (std::fabs(denominator) <= epsilon) {
        return std::nullopt;
    }

    const float firstT = Cross2D(offset, secondDirection) / denominator;
    const float secondT = Cross2D(offset, firstDirection) / denominator;

    if (firstT < -epsilon || firstT > 1.0f + epsilon ||
        secondT < -epsilon || secondT > 1.0f + epsilon) {
        return std::nullopt;
    }

    SegmentIntersection intersection = {};
    intersection.firstSegmentT = firstT;
    intersection.secondSegmentT = secondT;
    intersection.position = {
        Lerp(a.x, b.x, firstT),
        Lerp(a.y, b.y, firstT),
        Lerp(a.z, b.z, firstT),
    };
    return intersection;
}

PolylineCurve MakeDefaultPolylineCurve() {
    PolylineCurve curve;
    curve.controlPoints = {
        {-1.25f, 0.0f, 1.10f},
        {-1.25f, 0.0f, -1.10f},
        {1.25f, 0.0f, -1.10f},
        {1.25f, 0.0f, 1.10f},
    };
    return curve;
}

PolylineCurve SubdividePolylineCurveTowardsBezierLimit(const PolylineCurve& curve) {
    if (curve.controlPoints.size() < 2) {
        return curve;
    }

    PolylineCurve subdividedCurve;
    subdividedCurve.controlPoints.reserve(curve.controlPoints.size() * 2 - 1);
    subdividedCurve.controlPoints.push_back(curve.controlPoints.front());

    for (std::size_t i = 1; i < curve.controlPoints.size(); ++i) {
        subdividedCurve.controlPoints.push_back(Midpoint(curve.controlPoints[i - 1], curve.controlPoints[i]));
        subdividedCurve.controlPoints.push_back(curve.controlPoints[i]);
    }

    PolylineCurve smoothedCurve = subdividedCurve;

    // A single 1-2-1 smoothing pass preserves the fixed ends and keeps inserted
    // edge points centered while nudging interior vertices toward the limit curve.
    for (std::size_t i = 1; i + 1 < subdividedCurve.controlPoints.size(); ++i) {
        smoothedCurve.controlPoints[i] = SmoothInteriorPoint(
            subdividedCurve.controlPoints[i - 1],
            subdividedCurve.controlPoints[i],
            subdividedCurve.controlPoints[i + 1]);
    }

    return smoothedCurve;
}

std::optional<PolylineCurveValidationError> ValidatePolylineCurve(
    const PolylineCurve& curve,
    float groundTolerance) {
    if (curve.controlPoints.size() < 2) {
        return PolylineCurveValidationError::TooFewPoints;
    }

    const float duplicateToleranceSquared = groundTolerance * groundTolerance;

    for (std::size_t i = 0; i < curve.controlPoints.size(); ++i) {
        const DirectX::XMFLOAT3& point = curve.controlPoints[i];
        if (std::fabs(point.y) > groundTolerance) {
            return PolylineCurveValidationError::PointOffGroundPlane;
        }

        if (i > 0 && DistanceSquared(curve.controlPoints[i - 1], point) <= duplicateToleranceSquared) {
            return PolylineCurveValidationError::AdjacentDuplicatePoints;
        }
    }

    return std::nullopt;
}

std::vector<DirectX::XMFLOAT3> IntersectCurves(
    const PolylineCurve& curve1,
    const PolylineCurve& curve2,
    float epsilon) {
    std::vector<DirectX::XMFLOAT3> points;
    const float duplicateToleranceSquared = epsilon * epsilon;

    if (curve1.controlPoints.size() < 2 || curve2.controlPoints.size() < 2) {
        return points;
    }

    for (std::size_t i = 0; i + 1 < curve1.controlPoints.size(); ++i) {
        for (std::size_t j = 0; j + 1 < curve2.controlPoints.size(); ++j) {
            const auto intersection = IntersectSegments(
                curve1.controlPoints[i],
                curve1.controlPoints[i + 1],
                curve2.controlPoints[j],
                curve2.controlPoints[j + 1],
                epsilon);
            if (!intersection.has_value()) {
                continue;
            }

            bool alreadyRecorded = false;
            for (const DirectX::XMFLOAT3& point : points) {
                if (DistanceSquared(point, intersection->position) <= duplicateToleranceSquared) {
                    alreadyRecorded = true;
                    break;
                }
            }

            if (!alreadyRecorded) {
                points.push_back(intersection->position);
            }
        }
    }

    return points;
}

const char* PolylineCurveValidationErrorMessage(PolylineCurveValidationError error) {
    switch (error) {
    case PolylineCurveValidationError::TooFewPoints:
        return "Polyline curve validation failed: at least two control points are required.";
    case PolylineCurveValidationError::PointOffGroundPlane:
        return "Polyline curve validation failed: all control points must lie on the XZ ground plane.";
    case PolylineCurveValidationError::AdjacentDuplicatePoints:
        return "Polyline curve validation failed: adjacent control points must not be duplicates.";
    default:
        return "Polyline curve validation failed.";
    }
}
