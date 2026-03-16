#include "HalfEdgeMesh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <unordered_set>

namespace {

MeshPoint operator+(const MeshPoint& left, const MeshPoint& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

MeshPoint operator-(const MeshPoint& left, const MeshPoint& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

MeshPoint operator*(const MeshPoint& value, float scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

MeshPoint operator/(const MeshPoint& value, float scale) {
    return {value.x / scale, value.y / scale, value.z / scale};
}

float Dot(const MeshPoint& left, const MeshPoint& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

MeshPoint Cross(const MeshPoint& left, const MeshPoint& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

float Length(const MeshPoint& value) {
    return std::sqrt(Dot(value, value));
}

MeshPoint Normalize(const MeshPoint& value) {
    const float length = Length(value);
    if (length <= 1.0e-6f) {
        return {0.0f, 1.0f, 0.0f};
    }
    return value / length;
}

MeshPoint Lerp(const MeshPoint& left, const MeshPoint& right, float t) {
    return left * (1.0f - t) + right * t;
}

MeshColor Lerp(const MeshColor& left, const MeshColor& right, float t) {
    return {
        left.r * (1.0f - t) + right.r * t,
        left.g * (1.0f - t) + right.g * t,
        left.b * (1.0f - t) + right.b * t
    };
}

MeshPoint RotateAroundAxis(const MeshPoint& point, const MeshPoint& axis, float angleRadians) {
    const MeshPoint unitAxis = Normalize(axis);
    const float cosAngle = std::cos(angleRadians);
    const float sinAngle = std::sin(angleRadians);

    return point * cosAngle
        + Cross(unitAxis, point) * sinAngle
        + unitAxis * (Dot(unitAxis, point) * (1.0f - cosAngle));
}

} // namespace

std::size_t HalfEdgeMesh::DirectedEdgeKeyHash::operator()(const DirectedEdgeKey& key) const noexcept {
    const std::size_t seed = std::hash<std::uint32_t>{}(key.from);
    const std::size_t mixed = std::hash<std::uint32_t>{}(key.to) + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u);
    return seed ^ mixed;
}

bool HalfEdgeMesh::DirectedEdgeKeyEqual::operator()(const DirectedEdgeKey& left, const DirectedEdgeKey& right) const noexcept {
    return left.from == right.from && left.to == right.to;
}

void HalfEdgeMesh::Clear() {
    vertices_.clear();
    polygonFaces_.clear();
    polygonFaceColors_.clear();
    halfedges_.clear();
    edges_.clear();
    faces_.clear();
    directedEdges_.clear();
}

std::uint32_t HalfEdgeMesh::AddVertex(const MeshPoint& position, const MeshColor& color) {
    Vertex vertex;
    vertex.position = position;
    vertex.bindPosition = position;
    vertex.color = color;
    vertices_.push_back(vertex);
    return static_cast<std::uint32_t>(vertices_.size() - 1);
}

std::uint32_t HalfEdgeMesh::AddPolygonFace(const std::vector<std::uint32_t>& cycle, const MeshColor& color) {
    polygonFaces_.push_back(cycle);
    polygonFaceColors_.push_back(color);
    return static_cast<std::uint32_t>(polygonFaces_.size() - 1);
}

std::optional<HalfEdgeMesh::ErrorCode> HalfEdgeMesh::Rebuild() {
    auto fail = [](ErrorCode error) -> std::optional<ErrorCode> {
        return error;
    };

    halfedges_.clear();
    edges_.clear();
    faces_.clear();
    directedEdges_.clear();

    for (Vertex& vertex : vertices_) {
        vertex.halfedge = Invalid;
    }

    faces_.reserve(polygonFaces_.size());

    for (std::uint32_t faceIndex = 0; faceIndex < polygonFaces_.size(); ++faceIndex) {
        const std::vector<std::uint32_t>& cycle = polygonFaces_[faceIndex];
        if (cycle.size() < 3) {
            return fail(ErrorCode::FaceTooSmall);
        }

        std::unordered_set<std::uint32_t> uniqueVertices;
        uniqueVertices.reserve(cycle.size());

        for (std::size_t i = 0; i < cycle.size(); ++i) {
            const std::uint32_t from = cycle[i];
            const std::uint32_t to = cycle[(i + 1) % cycle.size()];

            if (from >= vertices_.size() || to >= vertices_.size()) {
                return fail(ErrorCode::FaceVertexOutOfRange);
            }

            if (from == to) {
                return fail(ErrorCode::FaceZeroLengthEdge);
            }

            if (!uniqueVertices.insert(from).second) {
                return fail(ErrorCode::FaceDuplicateVertex);
            }

            const DirectedEdgeKey forward{from, to};
            const DirectedEdgeKey reverse{to, from};

            if (directedEdges_.find(forward) != directedEdges_.end()) {
                return fail(ErrorCode::DuplicateDirectedEdge);
            }

            const auto twinIt = directedEdges_.find(reverse);
            if (twinIt != directedEdges_.end() && halfedges_[twinIt->second].twin != Invalid) {
                return fail(ErrorCode::NonManifoldEdge);
            }
        }

        Face face;
        face.color = polygonFaceColors_[faceIndex];
        faces_.push_back(face);

        std::vector<std::uint32_t> loop(cycle.size(), Invalid);
        for (std::size_t i = 0; i < cycle.size(); ++i) {
            HalfEdge halfedge;
            halfedge.origin = cycle[i];
            halfedge.face = faceIndex;
            halfedges_.push_back(halfedge);
            loop[i] = static_cast<std::uint32_t>(halfedges_.size() - 1);
        }

        faces_[faceIndex].halfedge = loop.front();

        for (std::size_t i = 0; i < loop.size(); ++i) {
            const std::uint32_t current = loop[i];
            const std::uint32_t next = loop[(i + 1) % loop.size()];
            const std::uint32_t prev = loop[(i + loop.size() - 1) % loop.size()];

            halfedges_[current].next = next;
            halfedges_[current].prev = prev;

            const std::uint32_t origin = halfedges_[current].origin;
            if (vertices_[origin].halfedge == Invalid) {
                vertices_[origin].halfedge = current;
            }
        }

        for (std::size_t i = 0; i < cycle.size(); ++i) {
            const std::uint32_t from = cycle[i];
            const std::uint32_t to = cycle[(i + 1) % cycle.size()];
            const std::uint32_t current = loop[i];

            const DirectedEdgeKey reverse{to, from};
            const auto twinIt = directedEdges_.find(reverse);

            if (twinIt != directedEdges_.end()) {
                const std::uint32_t twin = twinIt->second;
                halfedges_[current].twin = twin;
                halfedges_[current].edge = halfedges_[twin].edge;
                halfedges_[twin].twin = current;
            } else {
                Edge edge;
                edge.halfedge = current;
                edges_.push_back(edge);
                halfedges_[current].edge = static_cast<std::uint32_t>(edges_.size() - 1);
            }

            directedEdges_.emplace(DirectedEdgeKey{from, to}, current);
        }
    }

    return Validate();
}

std::optional<HalfEdgeMesh::ErrorCode> HalfEdgeMesh::Validate() const {
    auto fail = [](ErrorCode error) -> std::optional<ErrorCode> {
        return error;
    };

    if (faces_.size() != polygonFaces_.size() || faces_.size() != polygonFaceColors_.size()) {
        return fail(ErrorCode::FaceStorageOutOfSync);
    }

    for (std::uint32_t vertexIndex = 0; vertexIndex < vertices_.size(); ++vertexIndex) {
        const Vertex& vertex = vertices_[vertexIndex];
        if (vertex.halfedge == Invalid) {
            continue;
        }
        if (vertex.halfedge >= halfedges_.size()) {
            return fail(ErrorCode::VertexHalfedgeInvalid);
        }
        if (halfedges_[vertex.halfedge].origin != vertexIndex) {
            return fail(ErrorCode::VertexHalfedgeNotOutgoing);
        }
    }

    for (std::uint32_t faceIndex = 0; faceIndex < faces_.size(); ++faceIndex) {
        std::vector<std::uint32_t> loop;
        if (const auto error = CollectFaceLoopHalfEdges(faceIndex, loop)) {
            return error;
        }
        if (loop.size() < 3) {
            return fail(ErrorCode::FaceLoopTooShort);
        }

        std::vector<std::uint32_t> faceVertices;
        if (const auto error = TryCollectFaceVertices(faceIndex, faceVertices)) {
            return error;
        }
        if (faceVertices.size() != polygonFaces_[faceIndex].size()) {
            return fail(ErrorCode::FaceLoopSizeMismatch);
        }

        for (std::size_t i = 0; i < faceVertices.size(); ++i) {
            if (faceVertices[i] != polygonFaces_[faceIndex][i]) {
                return fail(ErrorCode::FaceLoopOrderMismatch);
            }
        }
    }

    for (std::uint32_t halfedgeIndex = 0; halfedgeIndex < halfedges_.size(); ++halfedgeIndex) {
        const HalfEdge& halfedge = halfedges_[halfedgeIndex];
        if (halfedge.origin >= vertices_.size()) {
            return fail(ErrorCode::HalfedgeOriginInvalid);
        }
        if (halfedge.face >= faces_.size()) {
            return fail(ErrorCode::HalfedgeFaceInvalid);
        }
        if (halfedge.edge >= edges_.size()) {
            return fail(ErrorCode::HalfedgeEdgeInvalid);
        }
        if (halfedge.next >= halfedges_.size() || halfedge.prev >= halfedges_.size()) {
            return fail(ErrorCode::HalfedgeNextPrevInvalid);
        }
        if (halfedges_[halfedge.next].prev != halfedgeIndex || halfedges_[halfedge.prev].next != halfedgeIndex) {
            return fail(ErrorCode::HalfedgeNextPrevInconsistent);
        }

        if (halfedge.twin != Invalid) {
            if (halfedge.twin >= halfedges_.size()) {
                return fail(ErrorCode::HalfedgeTwinInvalid);
            }

            const HalfEdge& twin = halfedges_[halfedge.twin];
            if (twin.twin != halfedgeIndex) {
                return fail(ErrorCode::HalfedgeTwinNotReciprocal);
            }
            if (twin.edge != halfedge.edge) {
                return fail(ErrorCode::HalfedgeTwinEdgeMismatch);
            }
        }
    }

    return std::nullopt;
}

std::string HalfEdgeMesh::Summary() const {
    std::ostringstream stream;
    stream << "Half-edge mesh\n";
    stream << "  vertices: " << vertices_.size() << '\n';
    stream << "  halfedges: " << halfedges_.size() << '\n';
    stream << "  edges: " << edges_.size() << '\n';
    stream << "  faces: " << faces_.size() << '\n';
    return stream.str();
}

std::vector<std::uint32_t> HalfEdgeMesh::CollectFaceVertices(std::uint32_t faceIndex) const {
    std::vector<std::uint32_t> vertices;
    TryCollectFaceVertices(faceIndex, vertices);
    return vertices;
}

std::vector<std::uint32_t> HalfEdgeMesh::CollectVertexNeighbors(std::uint32_t vertexIndex) const {
    std::vector<std::uint32_t> neighbors;
    std::unordered_set<std::uint32_t> uniqueNeighbors;

    for (const std::uint32_t halfedgeIndex : CollectOutgoingHalfEdges(vertexIndex)) {
        const std::uint32_t destination = Destination(halfedgeIndex);
        if (destination == Invalid) {
            continue;
        }
        if (uniqueNeighbors.insert(destination).second) {
            neighbors.push_back(destination);
        }
    }

    return neighbors;
}

MeshPoint HalfEdgeMesh::ComputeFaceCenter(std::uint32_t faceIndex) const {
    const std::vector<std::uint32_t> vertices = CollectFaceVertices(faceIndex);
    MeshPoint center{};
    if (vertices.empty()) {
        return center;
    }

    for (const std::uint32_t vertexIndex : vertices) {
        center = center + vertices_[vertexIndex].position;
    }

    return center / static_cast<float>(vertices.size());
}

MeshPoint HalfEdgeMesh::ComputeFaceNormal(std::uint32_t faceIndex) const {
    const std::vector<std::uint32_t> vertices = CollectFaceVertices(faceIndex);
    if (vertices.size() < 3) {
        return {0.0f, 1.0f, 0.0f};
    }

    const MeshPoint p0 = vertices_[vertices[0]].position;
    MeshPoint normal{};

    for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
        const MeshPoint edgeA = vertices_[vertices[i]].position - p0;
        const MeshPoint edgeB = vertices_[vertices[i + 1]].position - p0;
        normal = normal + Cross(edgeA, edgeB);
    }

    return Normalize(normal);
}

std::uint32_t HalfEdgeMesh::FindBestFace(const MeshPoint& direction) const {
    const MeshPoint unitDirection = Normalize(direction);
    float bestScore = -1.0e9f;
    std::uint32_t bestFace = Invalid;

    for (std::uint32_t faceIndex = 0; faceIndex < faces_.size(); ++faceIndex) {
        const float score = Dot(ComputeFaceNormal(faceIndex), unitDirection);
        if (score > bestScore) {
            bestScore = score;
            bestFace = faceIndex;
        }
    }

    return bestFace;
}

HalfEdgeMesh::ExtrusionResult HalfEdgeMesh::InsetExtrudeFace(std::uint32_t faceIndex, float inset, float distance) {
    // This is intentionally a specialized local topological edit for inset-extruding one face.
    // It is not a general mesh editing framework and should be revisited before broader edit support is added.
    ExtrusionResult result;
    if (faceIndex >= polygonFaces_.size()) {
        result.error = ErrorCode::ExtrudeFaceOutOfRange;
        return result;
    }

    inset = std::clamp(inset, 0.0f, 0.95f);

    std::vector<std::uint32_t> boundaryHalfedges;
    if (const auto error = CollectFaceLoopHalfEdges(faceIndex, boundaryHalfedges)) {
        result.error = error;
        return result;
    }

    std::vector<std::uint32_t> sourceLoop;
    if (const auto error = TryCollectFaceVertices(faceIndex, sourceLoop)) {
        result.error = error;
        return result;
    }

    if (sourceLoop.size() != boundaryHalfedges.size()) {
        result.error = ErrorCode::ExtrudeFaceLoopBookkeepingMismatch;
        return result;
    }

    const MeshColor sourceFaceColor = polygonFaceColors_[faceIndex];
    const MeshColor extrusionAccent = {0.18f, 0.92f, 0.82f};
    const MeshColor topFaceColor = Lerp(sourceFaceColor, extrusionAccent, 0.75f);
    const MeshColor sideFaceColor = Lerp(sourceFaceColor, extrusionAccent, 0.45f);
    const MeshPoint center = ComputeFaceCenter(faceIndex);
    const MeshPoint normal = ComputeFaceNormal(faceIndex);

    std::vector<std::uint32_t> extrudedLoop;
    extrudedLoop.reserve(sourceLoop.size());

    for (const std::uint32_t vertexIndex : sourceLoop) {
        const MeshPoint basePosition = vertices_[vertexIndex].bindPosition;
        const MeshPoint insetPosition = Lerp(basePosition, center, inset);
        const MeshPoint extrudedPosition = insetPosition + normal * distance;
        const MeshColor color = Lerp(vertices_[vertexIndex].color, topFaceColor, 0.75f);
        extrudedLoop.push_back(AddVertex(extrudedPosition, color));
    }

    polygonFaces_[faceIndex] = extrudedLoop;
    polygonFaceColors_[faceIndex] = topFaceColor;

    for (std::size_t i = 0; i < sourceLoop.size(); ++i) {
        const std::uint32_t a0 = sourceLoop[i];
        const std::uint32_t a1 = sourceLoop[(i + 1) % sourceLoop.size()];
        const std::uint32_t b0 = extrudedLoop[i];
        const std::uint32_t b1 = extrudedLoop[(i + 1) % extrudedLoop.size()];

        polygonFaces_.push_back({a0, a1, b1, b0});
        polygonFaceColors_.push_back(sideFaceColor);
    }

    const std::size_t sideCount = sourceLoop.size();
    std::vector<std::uint32_t> sideFaceIndices(sideCount, Invalid);
    std::vector<std::uint32_t> topHalfedges(sideCount, Invalid);
    std::vector<std::uint32_t> sideTopHalfedges(sideCount, Invalid);
    std::vector<std::uint32_t> upHalfedges(sideCount, Invalid);
    std::vector<std::uint32_t> downHalfedges(sideCount, Invalid);
    std::vector<std::uint32_t> topEdgeIndices(sideCount, Invalid);
    std::vector<std::uint32_t> verticalEdgeIndices(sideCount, Invalid);

    auto appendHalfedge = [&](std::uint32_t origin) {
        HalfEdge halfedge;
        halfedge.origin = origin;
        halfedges_.push_back(halfedge);
        return static_cast<std::uint32_t>(halfedges_.size() - 1);
    };

    for (std::size_t i = 0; i < sideCount; ++i) {
        topHalfedges[i] = appendHalfedge(extrudedLoop[i]);
        sideTopHalfedges[i] = appendHalfedge(extrudedLoop[(i + 1) % sideCount]);
        upHalfedges[i] = appendHalfedge(sourceLoop[i]);
        downHalfedges[i] = appendHalfedge(extrudedLoop[i]);
    }

    faces_[faceIndex].halfedge = topHalfedges[0];
    faces_[faceIndex].color = topFaceColor;

    for (std::size_t i = 0; i < sideCount; ++i) {
        Face face;
        face.color = sideFaceColor;
        face.halfedge = boundaryHalfedges[i];
        faces_.push_back(face);
        sideFaceIndices[i] = static_cast<std::uint32_t>(faces_.size() - 1);
    }

    for (std::size_t i = 0; i < sideCount; ++i) {
        Edge topEdge;
        topEdge.halfedge = topHalfedges[i];
        edges_.push_back(topEdge);
        topEdgeIndices[i] = static_cast<std::uint32_t>(edges_.size() - 1);

        Edge verticalEdge;
        verticalEdge.halfedge = upHalfedges[i];
        edges_.push_back(verticalEdge);
        verticalEdgeIndices[i] = static_cast<std::uint32_t>(edges_.size() - 1);
    }

    for (std::size_t i = 0; i < sideCount; ++i) {
        const std::size_t next = (i + 1) % sideCount;
        const std::size_t prev = (i + sideCount - 1) % sideCount;

        HalfEdge& top = halfedges_[topHalfedges[i]];
        top.origin = extrudedLoop[i];
        top.twin = sideTopHalfedges[i];
        top.next = topHalfedges[next];
        top.prev = topHalfedges[prev];
        top.face = faceIndex;
        top.edge = topEdgeIndices[i];

        HalfEdge& sideTop = halfedges_[sideTopHalfedges[i]];
        sideTop.origin = extrudedLoop[next];
        sideTop.twin = topHalfedges[i];
        sideTop.next = downHalfedges[i];
        sideTop.prev = upHalfedges[next];
        sideTop.face = sideFaceIndices[i];
        sideTop.edge = topEdgeIndices[i];

        HalfEdge& up = halfedges_[upHalfedges[i]];
        up.origin = sourceLoop[i];
        up.twin = downHalfedges[i];
        up.face = sideFaceIndices[prev];
        up.edge = verticalEdgeIndices[i];

        HalfEdge& down = halfedges_[downHalfedges[i]];
        down.origin = extrudedLoop[i];
        down.twin = upHalfedges[i];
        down.next = boundaryHalfedges[i];
        down.prev = sideTopHalfedges[i];
        down.face = sideFaceIndices[i];
        down.edge = verticalEdgeIndices[i];
    }

    for (std::size_t i = 0; i < sideCount; ++i) {
        const std::size_t next = (i + 1) % sideCount;

        HalfEdge& bottom = halfedges_[boundaryHalfedges[i]];
        bottom.face = sideFaceIndices[i];
        bottom.next = upHalfedges[next];
        bottom.prev = downHalfedges[i];

        HalfEdge& upForSide = halfedges_[upHalfedges[next]];
        upForSide.next = sideTopHalfedges[i];
        upForSide.prev = boundaryHalfedges[i];
    }

    for (std::size_t i = 0; i < sideCount; ++i) {
        directedEdges_.emplace(DirectedEdgeKey{sourceLoop[i], extrudedLoop[i]}, upHalfedges[i]);
        directedEdges_.emplace(DirectedEdgeKey{extrudedLoop[i], sourceLoop[i]}, downHalfedges[i]);
        directedEdges_.emplace(DirectedEdgeKey{extrudedLoop[i], extrudedLoop[(i + 1) % sideCount]}, topHalfedges[i]);
        directedEdges_.emplace(DirectedEdgeKey{extrudedLoop[(i + 1) % sideCount], extrudedLoop[i]}, sideTopHalfedges[i]);
        vertices_[extrudedLoop[i]].halfedge = topHalfedges[i];
    }

    if (const auto error = Validate()) {
        result.error = error;
        return result;
    }

    result.topFace = faceIndex;
    result.newVertices = extrudedLoop;
    result.error = std::nullopt;
    return result;
}

void HalfEdgeMesh::AnimateFaceRegion(std::uint32_t faceIndex, float timeSeconds) {
    ResetToBindPose();

    const std::vector<std::uint32_t> faceVertices = CollectFaceVertices(faceIndex);
    if (faceVertices.empty()) {
        return;
    }

    std::unordered_set<std::uint32_t> faceVertexSet(faceVertices.begin(), faceVertices.end());
    std::unordered_set<std::uint32_t> neighborSet;

    MeshPoint center{};
    for (const std::uint32_t vertexIndex : faceVertices) {
        center = center + vertices_[vertexIndex].bindPosition;
        for (const std::uint32_t neighborIndex : CollectVertexNeighbors(vertexIndex)) {
            if (faceVertexSet.find(neighborIndex) == faceVertexSet.end()) {
                neighborSet.insert(neighborIndex);
            }
        }
    }
    center = center / static_cast<float>(faceVertices.size());

    MeshPoint normal = ComputeFaceNormal(faceIndex);
    if (Length(normal) <= 1.0e-6f) {
        normal = {0.0f, 1.0f, 0.0f};
    }

    const float lift = 0.18f * std::sin(timeSeconds * 1.75f);
    const float twist = 0.45f * std::sin(timeSeconds * 0.9f);

    for (const std::uint32_t vertexIndex : faceVertices) {
        const MeshPoint local = vertices_[vertexIndex].bindPosition - center;
        const MeshPoint twisted = RotateAroundAxis(local, normal, twist);
        vertices_[vertexIndex].position = center + twisted + normal * lift;
    }

    for (const std::uint32_t vertexIndex : neighborSet) {
        const MeshPoint bindPosition = vertices_[vertexIndex].bindPosition;
        const MeshPoint towardCenter = center - bindPosition;
        vertices_[vertexIndex].position = bindPosition + normal * (lift * 0.28f) + towardCenter * 0.06f;
    }
}

void HalfEdgeMesh::ResetToBindPose() {
    for (Vertex& vertex : vertices_) {
        vertex.position = vertex.bindPosition;
    }
}

void HalfEdgeMesh::BuildRenderBuffers(std::vector<RenderVertex>& vertices, std::vector<std::uint16_t>& indices) const {
    vertices.clear();
    indices.clear();

    std::size_t vertexCount = 0;
    std::size_t indexCount = 0;
    for (const std::vector<std::uint32_t>& face : polygonFaces_) {
        if (face.size() < 3) {
            continue;
        }
        vertexCount += face.size();
        indexCount += (face.size() - 2) * 3;
    }
    vertices.reserve(vertexCount);
    indices.reserve(indexCount);

    for (std::size_t faceIndex = 0; faceIndex < polygonFaces_.size(); ++faceIndex) {
        const std::vector<std::uint32_t>& face = polygonFaces_[faceIndex];
        if (face.size() < 3) {
            continue;
        }

        // Emit per-face vertices so semantic face colors can stay visible even when topology shares positions.
        const std::uint16_t baseVertex = static_cast<std::uint16_t>(vertices.size());
        const MeshColor faceColor = polygonFaceColors_[faceIndex];

        for (const std::uint32_t vertexIndex : face) {
            const Vertex& source = vertices_[vertexIndex];
            vertices.push_back({
                source.position,
                Lerp(source.color, faceColor, 0.7f)
            });
        }

        for (std::size_t i = 1; i + 1 < face.size(); ++i) {
            indices.push_back(baseVertex);
            indices.push_back(static_cast<std::uint16_t>(baseVertex + i));
            indices.push_back(static_cast<std::uint16_t>(baseVertex + i + 1));
        }
    }
}

std::size_t HalfEdgeMesh::VertexCount() const {
    return vertices_.size();
}

std::size_t HalfEdgeMesh::HalfEdgeCount() const {
    return halfedges_.size();
}

std::size_t HalfEdgeMesh::EdgeCount() const {
    return edges_.size();
}

std::size_t HalfEdgeMesh::FaceCount() const {
    return faces_.size();
}

std::uint32_t HalfEdgeMesh::Destination(std::uint32_t halfedgeIndex) const {
    if (halfedgeIndex >= halfedges_.size()) {
        return Invalid;
    }

    const HalfEdge& halfedge = halfedges_[halfedgeIndex];
    if (halfedge.next >= halfedges_.size()) {
        return Invalid;
    }

    return halfedges_[halfedge.next].origin;
}

std::optional<HalfEdgeMesh::ErrorCode> HalfEdgeMesh::CollectFaceLoopHalfEdges(
    std::uint32_t faceIndex,
    std::vector<std::uint32_t>& loop) const {
    loop.clear();

    if (faceIndex >= faces_.size()) {
        return ErrorCode::FaceLoopOutOfRange;
    }

    const Face& face = faces_[faceIndex];
    if (face.halfedge == Invalid || face.halfedge >= halfedges_.size()) {
        return ErrorCode::FaceHalfedgeInvalid;
    }

    std::unordered_set<std::uint32_t> seen;
    std::uint32_t current = face.halfedge;

    for (std::size_t steps = 0; steps <= halfedges_.size(); ++steps) {
        if (current >= halfedges_.size()) {
            loop.clear();
            return ErrorCode::FaceLoopOutsideArray;
        }

        if (!seen.insert(current).second) {
            if (current == face.halfedge) {
                return std::nullopt;
            }
            loop.clear();
            return ErrorCode::FaceLoopRevisitsHalfedge;
        }

        const HalfEdge& halfedge = halfedges_[current];
        if (halfedge.face != faceIndex) {
            loop.clear();
            return ErrorCode::FaceLoopWrongFace;
        }

        if (halfedge.next == Invalid || halfedge.prev == Invalid) {
            loop.clear();
            return ErrorCode::FaceLoopMissingLinks;
        }

        loop.push_back(current);
        current = halfedge.next;

        if (current == face.halfedge) {
            return std::nullopt;
        }
    }

    loop.clear();
    return ErrorCode::FaceLoopDidNotClose;
}

std::optional<HalfEdgeMesh::ErrorCode> HalfEdgeMesh::TryCollectFaceVertices(
    std::uint32_t faceIndex,
    std::vector<std::uint32_t>& vertices) const {
    vertices.clear();

    std::vector<std::uint32_t> loop;
    if (const auto error = CollectFaceLoopHalfEdges(faceIndex, loop)) {
        return error;
    }

    vertices.reserve(loop.size());
    for (const std::uint32_t halfedgeIndex : loop) {
        vertices.push_back(halfedges_[halfedgeIndex].origin);
    }

    return std::nullopt;
}

std::vector<std::uint32_t> HalfEdgeMesh::CollectOutgoingHalfEdges(std::uint32_t vertexIndex) const {
    std::vector<std::uint32_t> fan;
    if (vertexIndex >= vertices_.size()) {
        return fan;
    }

    const Vertex& vertex = vertices_[vertexIndex];
    if (vertex.halfedge == Invalid || vertex.halfedge >= halfedges_.size()) {
        return fan;
    }

    std::unordered_set<std::uint32_t> seen;
    std::uint32_t current = vertex.halfedge;

    while (seen.insert(current).second) {
        fan.push_back(current);

        const HalfEdge& halfedge = halfedges_[current];
        if (halfedge.twin == Invalid || halfedge.twin >= halfedges_.size()) {
            break;
        }

        const HalfEdge& twin = halfedges_[halfedge.twin];
        if (twin.next == Invalid || twin.next >= halfedges_.size()) {
            break;
        }

        current = twin.next;
    }

    current = vertex.halfedge;
    while (true) {
        const HalfEdge& halfedge = halfedges_[current];
        if (halfedge.prev == Invalid || halfedge.prev >= halfedges_.size()) {
            break;
        }

        const HalfEdge& incoming = halfedges_[halfedge.prev];
        if (incoming.twin == Invalid || incoming.twin >= halfedges_.size()) {
            break;
        }

        current = incoming.twin;
        if (!seen.insert(current).second) {
            break;
        }

        fan.insert(fan.begin(), current);
    }

    return fan;
}

HalfEdgeMeshBuildResult MakeColoredCubeMesh() {
    HalfEdgeMeshBuildResult result;
    HalfEdgeMesh& mesh = result.mesh;

    const std::uint32_t v0 = mesh.AddVertex({-1.0f, -1.0f, -1.0f}, {0.95f, 0.20f, 0.20f});
    const std::uint32_t v1 = mesh.AddVertex({-1.0f,  1.0f, -1.0f}, {0.95f, 0.60f, 0.20f});
    const std::uint32_t v2 = mesh.AddVertex({ 1.0f,  1.0f, -1.0f}, {0.95f, 0.90f, 0.20f});
    const std::uint32_t v3 = mesh.AddVertex({ 1.0f, -1.0f, -1.0f}, {0.20f, 0.80f, 0.30f});
    const std::uint32_t v4 = mesh.AddVertex({-1.0f, -1.0f,  1.0f}, {0.20f, 0.70f, 0.95f});
    const std::uint32_t v5 = mesh.AddVertex({-1.0f,  1.0f,  1.0f}, {0.45f, 0.35f, 0.95f});
    const std::uint32_t v6 = mesh.AddVertex({ 1.0f,  1.0f,  1.0f}, {0.85f, 0.30f, 0.95f});
    const std::uint32_t v7 = mesh.AddVertex({ 1.0f, -1.0f,  1.0f}, {0.95f, 0.35f, 0.65f});

    mesh.AddPolygonFace({v0, v1, v2, v3}, {0.75f, 0.45f, 0.20f});
    mesh.AddPolygonFace({v4, v7, v6, v5}, {0.30f, 0.40f, 0.85f});
    mesh.AddPolygonFace({v4, v5, v1, v0}, {0.20f, 0.65f, 0.90f});
    mesh.AddPolygonFace({v3, v2, v6, v7}, {0.25f, 0.85f, 0.35f});
    mesh.AddPolygonFace({v1, v5, v6, v2}, {0.90f, 0.55f, 0.30f});
    mesh.AddPolygonFace({v4, v0, v3, v7}, {0.70f, 0.25f, 0.60f});

    result.error = mesh.Rebuild();
    return result;
}
