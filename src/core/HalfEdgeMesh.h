#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct MeshPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct MeshColor {
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
};

struct RenderVertex {
    MeshPoint position;
    MeshColor color;
};

class HalfEdgeMesh {
public:
    static constexpr std::uint32_t Invalid = std::numeric_limits<std::uint32_t>::max();

    enum class ErrorCode : std::uint32_t {
        FaceTooSmall = 1,
        FaceVertexOutOfRange,
        FaceZeroLengthEdge,
        FaceDuplicateVertex,
        DuplicateDirectedEdge,
        NonManifoldEdge,
        FaceStorageOutOfSync,
        VertexHalfedgeInvalid,
        VertexHalfedgeNotOutgoing,
        FaceLoopOutOfRange,
        FaceHalfedgeInvalid,
        FaceLoopOutsideArray,
        FaceLoopRevisitsHalfedge,
        FaceLoopWrongFace,
        FaceLoopMissingLinks,
        FaceLoopDidNotClose,
        FaceLoopTooShort,
        FaceLoopSizeMismatch,
        FaceLoopOrderMismatch,
        HalfedgeOriginInvalid,
        HalfedgeFaceInvalid,
        HalfedgeEdgeInvalid,
        HalfedgeNextPrevInvalid,
        HalfedgeNextPrevInconsistent,
        HalfedgeTwinInvalid,
        HalfedgeTwinNotReciprocal,
        HalfedgeTwinEdgeMismatch,
        ExtrudeFaceOutOfRange,
        ExtrudeFaceLoopBookkeepingMismatch,
    };

    struct Vertex {
        MeshPoint position{};
        MeshPoint bindPosition{};
        MeshColor color{};
        std::uint32_t halfedge = Invalid;
    };

    struct HalfEdge {
        std::uint32_t origin = Invalid;
        std::uint32_t twin = Invalid;
        std::uint32_t next = Invalid;
        std::uint32_t prev = Invalid;
        std::uint32_t face = Invalid;
        std::uint32_t edge = Invalid;
    };

    struct Edge {
        std::uint32_t halfedge = Invalid;
    };

    struct Face {
        std::uint32_t halfedge = Invalid;
        MeshColor color{};
    };

    struct ExtrusionResult {
        std::uint32_t topFace = Invalid;
        std::vector<std::uint32_t> newVertices;
        std::optional<ErrorCode> error;
    };

    void Clear();
    std::uint32_t AddVertex(const MeshPoint& position, const MeshColor& color);
    std::uint32_t AddPolygonFace(const std::vector<std::uint32_t>& cycle, const MeshColor& color);

    std::optional<ErrorCode> Rebuild();
    std::optional<ErrorCode> Validate() const;
    std::string Summary() const;

    std::vector<std::uint32_t> CollectFaceVertices(std::uint32_t faceIndex) const;
    std::vector<std::uint32_t> CollectVertexNeighbors(std::uint32_t vertexIndex) const;
    MeshPoint ComputeFaceCenter(std::uint32_t faceIndex) const;
    MeshPoint ComputeFaceNormal(std::uint32_t faceIndex) const;
    std::uint32_t FindBestFace(const MeshPoint& direction) const;

    ExtrusionResult InsetExtrudeFace(std::uint32_t faceIndex, float inset, float distance);
    void AnimateFaceRegion(std::uint32_t faceIndex, float timeSeconds);
    void ResetToBindPose();
    void BuildRenderBuffers(std::vector<RenderVertex>& vertices, std::vector<std::uint16_t>& indices) const;

    std::size_t VertexCount() const;
    std::size_t HalfEdgeCount() const;
    std::size_t EdgeCount() const;
    std::size_t FaceCount() const;

private:
    struct DirectedEdgeKey {
        std::uint32_t from = Invalid;
        std::uint32_t to = Invalid;
    };

    struct DirectedEdgeKeyHash {
        std::size_t operator()(const DirectedEdgeKey& key) const noexcept;
    };

    struct DirectedEdgeKeyEqual {
        bool operator()(const DirectedEdgeKey& left, const DirectedEdgeKey& right) const noexcept;
    };

    std::uint32_t Destination(std::uint32_t halfedgeIndex) const;
    std::optional<ErrorCode> CollectFaceLoopHalfEdges(std::uint32_t faceIndex, std::vector<std::uint32_t>& loop) const;
    std::optional<ErrorCode> TryCollectFaceVertices(std::uint32_t faceIndex, std::vector<std::uint32_t>& vertices) const;
    std::vector<std::uint32_t> CollectOutgoingHalfEdges(std::uint32_t vertexIndex) const;

    std::vector<Vertex> vertices_;
    std::vector<std::vector<std::uint32_t>> polygonFaces_;
    std::vector<MeshColor> polygonFaceColors_;

    std::vector<HalfEdge> halfedges_;
    std::vector<Edge> edges_;
    std::vector<Face> faces_;
    std::unordered_map<DirectedEdgeKey, std::uint32_t, DirectedEdgeKeyHash, DirectedEdgeKeyEqual> directedEdges_;
};

struct HalfEdgeMeshBuildResult {
    HalfEdgeMesh mesh;
    std::optional<HalfEdgeMesh::ErrorCode> error;
};

HalfEdgeMeshBuildResult MakeColoredCubeMesh();
