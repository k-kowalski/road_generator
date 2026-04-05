#include <windows.h>
#include <windowsx.h>
#include <wrl/client.h>

#include <d3d12.h>
#include <dxgi1_6.h>

#include <DirectXMath.h>

#include "PolylineCurve.h"
#include "RoadGeneration.h"
#include "RibbonMesh.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using Microsoft::WRL::ComPtr;

namespace {

constexpr UINT kFrameCount = 2;
constexpr UINT kWindowWidth = 1280;
constexpr UINT kWindowHeight = 720;
constexpr wchar_t kWindowClassName[] = L"RoadGenWindow";
constexpr wchar_t kWindowTitle[] = L"RoadGen";
constexpr float kRibbonHalfWidth = 0.20f;
constexpr RibbonTangentMode kRibbonTangentMode = RibbonTangentMode::AverageSegmentDirections;
constexpr float kTangentDebugYOffset = 0.14f;
constexpr float kTangentDebugLength = 0.28f;
struct OrbitCamera {
    float yaw = DirectX::XM_PI;
    float pitch = 0.48f;
    float distance = 8.4f;
    DirectX::XMFLOAT3 target = {0.0f, 0.15f, 0.0f};
};

struct SceneConstants {
    DirectX::XMFLOAT4X4 mvp;
};

struct DebugVertex {
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT3 color;
};

struct DrawRange {
    UINT startVertex = 0;
    UINT vertexCount = 0;
};

struct CurveDebugStyle {
    DirectX::XMFLOAT3 segmentColor;
    DirectX::XMFLOAT3 markerColor;
    float yOffset = 0.0f;
    float baseHalfSpan = 0.08f;
    float stemHeight = 0.18f;
    float crownHalfSpan = 0.05f;
};

struct ProcessedCurveData {
    CurveIndex authoredIndex = 0;
    PolylineCurve curve;
    std::vector<DirectX::XMFLOAT3> tangents;
};

struct SceneGeometryCpuData {
    std::vector<DebugVertex> lineVertices;
    DrawRange gridRange = {};
    DrawRange roughCurveRange = {};
    DrawRange roughControlPointRange = {};
    DrawRange subdividedCurveRange = {};
    DrawRange subdividedTangentRange = {};
    std::vector<RibbonVertex> ribbonVertices;
    std::vector<std::uint32_t> ribbonIndices;
};

struct RenderStateSnapshot {
    OrbitCamera camera = {};
    bool showOriginalCurve = true;
    bool showSubdividedCurve = true;
    bool showRibbon = true;
    bool showRibbonWireframe = false;
};

UINT AlignTo(UINT value, UINT alignment) {
    return (value + alignment - 1U) & ~(alignment - 1U);
}

float ClampFloat(float value, float minValue, float maxValue) {
    return std::clamp(value, minValue, maxValue);
}

float DistanceSquared(const DirectX::XMFLOAT3& left, const DirectX::XMFLOAT3& right) {
    const float dx = left.x - right.x;
    const float dy = left.y - right.y;
    const float dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

D3D12_BLEND_DESC OpaqueBlendDesc() {
    D3D12_BLEND_DESC blendDesc = {};
    blendDesc.RenderTarget[0].BlendEnable = FALSE;
    blendDesc.RenderTarget[0].LogicOpEnable = FALSE;
    blendDesc.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
    blendDesc.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
    blendDesc.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
    blendDesc.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
    blendDesc.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].LogicOp = D3D12_LOGIC_OP_NOOP;
    blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    return blendDesc;
}

D3D12_RASTERIZER_DESC DefaultRasterizerDesc(D3D12_FILL_MODE fillMode = D3D12_FILL_MODE_SOLID) {
    D3D12_RASTERIZER_DESC rasterizerDesc = {};
    rasterizerDesc.FillMode = fillMode;
    rasterizerDesc.CullMode = D3D12_CULL_MODE_NONE;
    rasterizerDesc.FrontCounterClockwise = FALSE;
    rasterizerDesc.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
    rasterizerDesc.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
    rasterizerDesc.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
    rasterizerDesc.DepthClipEnable = TRUE;
    return rasterizerDesc;
}

D3D12_DEPTH_STENCIL_DESC DepthStencilDesc(D3D12_DEPTH_WRITE_MASK depthWriteMask) {
    D3D12_DEPTH_STENCIL_DESC depthStencilDesc = {};
    depthStencilDesc.DepthEnable = TRUE;
    depthStencilDesc.DepthWriteMask = depthWriteMask;
    depthStencilDesc.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    return depthStencilDesc;
}

D3D12_RESOURCE_DESC BufferDesc(UINT64 sizeInBytes) {
    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = sizeInBytes;
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    return desc;
}

D3D12_RESOURCE_BARRIER TransitionBarrier(
    ID3D12Resource* resource,
    D3D12_RESOURCE_STATES before,
    D3D12_RESOURCE_STATES after) {
    D3D12_RESOURCE_BARRIER barrier = {};
    barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    barrier.Transition.pResource = resource;
    barrier.Transition.StateBefore = before;
    barrier.Transition.StateAfter = after;
    barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    return barrier;
}

[[noreturn]] void FatalError(const char* message, HRESULT hr = E_FAIL) {
    char buffer[512] = {};
    if (FAILED(hr)) {
        std::snprintf(buffer, sizeof(buffer), "%s (HRESULT=0x%08X)", message, static_cast<unsigned>(hr));
    } else {
        std::snprintf(buffer, sizeof(buffer), "%s", message);
    }
    OutputDebugStringA(buffer);
    MessageBoxA(nullptr, buffer, "RoadGen", MB_OK | MB_ICONERROR);
    ExitProcess(1);
}

void ThrowIfFailed(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        FatalError(message, hr);
    }
}

std::wstring GetExecutableDirectory() {
    wchar_t path[MAX_PATH] = {};
    GetModuleFileNameW(nullptr, path, MAX_PATH);
    std::wstring fullPath(path);
    const size_t slash = fullPath.find_last_of(L"\\/");
    return slash == std::wstring::npos ? L"." : fullPath.substr(0, slash);
}

std::vector<std::uint8_t> ReadBinaryFile(const std::wstring& path) {
    FILE* file = nullptr;
    if (_wfopen_s(&file, path.c_str(), L"rb") != 0 || file == nullptr) {
        return {};
    }

    fseek(file, 0, SEEK_END);
    const long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (fileSize <= 0) {
        fclose(file);
        return {};
    }

    std::vector<std::uint8_t> data(static_cast<size_t>(fileSize));
    const size_t readSize = fread(data.data(), 1, data.size(), file);
    fclose(file);

    if (readSize != data.size()) {
        return {};
    }

    return data;
}

DirectX::XMFLOAT3 OffsetPoint(const DirectX::XMFLOAT3& point, float dx, float dy, float dz) {
    return {point.x + dx, point.y + dy, point.z + dz};
}

DirectX::XMFLOAT3 LiftPoint(const DirectX::XMFLOAT3& point, float yOffset) {
    return OffsetPoint(point, 0.0f, yOffset, 0.0f);
}

CurveDebugStyle RoughCurveStyle(CurveIndex curveIndex) {
    static constexpr CurveDebugStyle kStyles[] = {
        {{0.95f, 0.56f, 0.21f}, {1.00f, 0.76f, 0.46f}, 0.02f, 0.09f, 0.20f, 0.06f},
        {{0.18f, 0.84f, 0.90f}, {0.72f, 0.98f, 1.00f}, 0.02f, 0.06f, 0.14f, 0.03f},
        {{0.44f, 0.82f, 0.28f}, {0.73f, 0.96f, 0.62f}, 0.02f, 0.07f, 0.17f, 0.05f},
        {{0.96f, 0.34f, 0.58f}, {1.00f, 0.70f, 0.82f}, 0.02f, 0.07f, 0.17f, 0.05f},
    };
    return kStyles[curveIndex % _countof(kStyles)];
}

CurveDebugStyle SubdividedCurveStyle(CurveIndex curveIndex) {
    static constexpr CurveDebugStyle kStyles[] = {
        {{1.00f, 0.78f, 0.42f}, {1.00f, 0.88f, 0.64f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{0.46f, 0.92f, 0.98f}, {0.78f, 1.00f, 1.00f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{0.66f, 0.92f, 0.46f}, {0.84f, 1.00f, 0.72f}, 0.08f, 0.06f, 0.14f, 0.03f},
        {{1.00f, 0.56f, 0.76f}, {1.00f, 0.80f, 0.90f}, 0.08f, 0.06f, 0.14f, 0.03f},
    };
    return kStyles[curveIndex % _countof(kStyles)];
}

void AppendLine(
    std::vector<DebugVertex>& vertices,
    const DirectX::XMFLOAT3& start,
    const DirectX::XMFLOAT3& end,
    const DirectX::XMFLOAT3& color) {
    vertices.push_back({start, color});
    vertices.push_back({end, color});
}

DrawRange BuildGroundGrid(std::vector<DebugVertex>& vertices) {
    DrawRange range = {};
    range.startVertex = static_cast<UINT>(vertices.size());

    constexpr float kGridExtent = 8.0f;
    constexpr float kGridSpacing = 0.5f;
    constexpr int kGridHalfSteps = 16;
    const DirectX::XMFLOAT3 gridColor = {0.24f, 0.29f, 0.36f};
    const DirectX::XMFLOAT3 axisColor = {0.36f, 0.43f, 0.52f};

    for (int step = -kGridHalfSteps; step <= kGridHalfSteps; ++step) {
        const float coordinate = static_cast<float>(step) * kGridSpacing;
        const DirectX::XMFLOAT3 color = (step == 0) ? axisColor : gridColor;

        AppendLine(vertices, {-kGridExtent, 0.0f, coordinate}, {kGridExtent, 0.0f, coordinate}, color);
        AppendLine(vertices, {coordinate, 0.0f, -kGridExtent}, {coordinate, 0.0f, kGridExtent}, color);
    }

    range.vertexCount = static_cast<UINT>(vertices.size()) - range.startVertex;
    return range;
}

DrawRange BuildCurveSegments(
    const PolylineCurve& curve,
    const CurveDebugStyle& style,
    std::vector<DebugVertex>& vertices) {
    DrawRange range = {};
    range.startVertex = static_cast<UINT>(vertices.size());

    for (std::size_t i = 1; i < curve.controlPoints.size(); ++i) {
        AppendLine(
            vertices,
            LiftPoint(curve.controlPoints[i - 1], style.yOffset),
            LiftPoint(curve.controlPoints[i], style.yOffset),
            style.segmentColor);
    }

    range.vertexCount = static_cast<UINT>(vertices.size()) - range.startVertex;
    return range;
}

DrawRange BuildControlPointMarkers(
    const PolylineCurve& curve,
    const CurveDebugStyle& style,
    std::vector<DebugVertex>& vertices) {
    DrawRange range = {};
    range.startVertex = static_cast<UINT>(vertices.size());

    for (const DirectX::XMFLOAT3& point : curve.controlPoints) {
        const DirectX::XMFLOAT3 liftedPoint = LiftPoint(point, style.yOffset);
        const DirectX::XMFLOAT3 crown = OffsetPoint(liftedPoint, 0.0f, style.stemHeight, 0.0f);
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

    range.vertexCount = static_cast<UINT>(vertices.size()) - range.startVertex;
    return range;
}

DrawRange BuildTangentSegments(
    const PolylineCurve& curve,
    const std::vector<DirectX::XMFLOAT3>& tangents,
    float yOffset,
    float lineLength,
    const DirectX::XMFLOAT3& color,
    std::vector<DebugVertex>& vertices) {
    DrawRange range = {};
    range.startVertex = static_cast<UINT>(vertices.size());

    if (curve.controlPoints.size() != tangents.size()) {
        return range;
    }

    for (std::size_t i = 0; i < curve.controlPoints.size(); ++i) {
        const DirectX::XMFLOAT3 start = LiftPoint(curve.controlPoints[i], yOffset);
        const DirectX::XMFLOAT3 end = {
            start.x + tangents[i].x * lineLength,
            start.y + tangents[i].y * lineLength,
            start.z + tangents[i].z * lineLength,
        };
        AppendLine(vertices, start, end, color);
    }

    range.vertexCount = static_cast<UINT>(vertices.size()) - range.startVertex;
    return range;
}

class RoadCurveApp {
public:
    int Run(HINSTANCE instance, int cmdShow) {
        instance_ = instance;
        ResetCamera();
        CreateAppWindow(cmdShow);
        InitializeD3D();
        previousFrameTime_ = std::chrono::steady_clock::now();
        StartThreads();
        RequestGeometryRebuild();
        MainLoop();
        StopThreads();
        Cleanup();
        return 0;
    }

private:
    void ResetCamera() {
        std::lock_guard<std::mutex> lock(renderStateMutex_);
        camera_ = OrbitCamera{};
    }

    DirectX::XMMATRIX ProjectionMatrix() const {
        return DirectX::XMMatrixPerspectiveFovLH(
            DirectX::XMConvertToRadians(60.0f),
            static_cast<float>(width_) / static_cast<float>(height_),
            0.1f,
            100.0f);
    }

    DirectX::XMMATRIX CameraViewMatrix(const OrbitCamera& camera) const {
        const float cosPitch = std::cos(camera.pitch);
        const float sinPitch = std::sin(camera.pitch);
        const float cosYaw = std::cos(camera.yaw);
        const float sinYaw = std::sin(camera.yaw);

        const DirectX::XMFLOAT3 eye = {
            camera.target.x + camera.distance * cosPitch * sinYaw,
            camera.target.y + camera.distance * sinPitch,
            camera.target.z + camera.distance * cosPitch * cosYaw
        };

        return DirectX::XMMatrixLookAtLH(
            DirectX::XMVectorSet(eye.x, eye.y, eye.z, 1.0f),
            DirectX::XMVectorSet(camera.target.x, camera.target.y, camera.target.z, 1.0f),
            DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
    }

    bool ScreenPointToGroundPlane(int screenX, int screenY, DirectX::XMFLOAT3& groundPoint) const {
        const DirectX::XMMATRIX world = DirectX::XMMatrixIdentity();
        const OrbitCamera camera = CaptureRenderStateSnapshot().camera;
        const DirectX::XMMATRIX view = CameraViewMatrix(camera);
        const DirectX::XMMATRIX projection = ProjectionMatrix();

        const DirectX::XMVECTOR nearPoint = DirectX::XMVector3Unproject(
            DirectX::XMVectorSet(static_cast<float>(screenX), static_cast<float>(screenY), 0.0f, 1.0f),
            0.0f,
            0.0f,
            static_cast<float>(width_),
            static_cast<float>(height_),
            0.0f,
            1.0f,
            projection,
            view,
            world);
        const DirectX::XMVECTOR farPoint = DirectX::XMVector3Unproject(
            DirectX::XMVectorSet(static_cast<float>(screenX), static_cast<float>(screenY), 1.0f, 1.0f),
            0.0f,
            0.0f,
            static_cast<float>(width_),
            static_cast<float>(height_),
            0.0f,
            1.0f,
            projection,
            view,
            world);

        DirectX::XMFLOAT3 nearValue = {};
        DirectX::XMFLOAT3 farValue = {};
        DirectX::XMStoreFloat3(&nearValue, nearPoint);
        DirectX::XMStoreFloat3(&farValue, farPoint);

        const float rayY = farValue.y - nearValue.y;
        if (std::fabs(rayY) <= 1.0e-6f) {
            return false;
        }

        const float t = -nearValue.y / rayY;
        if (t < 0.0f) {
            return false;
        }

        groundPoint = {
            nearValue.x + (farValue.x - nearValue.x) * t,
            0.0f,
            nearValue.z + (farValue.z - nearValue.z) * t,
        };
        return std::isfinite(groundPoint.x) && std::isfinite(groundPoint.z);
    }

    void AppendControlPointToCurrentCurve(const DirectX::XMFLOAT3& point) {
        {
            std::lock_guard<std::mutex> lock(authoredCurvesMutex_);
            if (!currentCurveActive_) {
                authoredCurves_.push_back({});
                currentCurveActive_ = true;
            }

            PolylineCurve& currentCurve = authoredCurves_.back();
            if (!currentCurve.controlPoints.empty() &&
                DistanceSquared(currentCurve.controlPoints.back(), point) <= 1.0e-6f) {
                return;
            }

            currentCurve.controlPoints.push_back(point);
        }
        RequestGeometryRebuild();
    }

    void FinishCurrentCurve() {
        if (!currentCurveActive_) {
            return;
        }

        currentCurveActive_ = false;
        RequestGeometryRebuild();
    }

    std::vector<PolylineCurve> CaptureAuthoredCurvesSnapshot() const {
        std::lock_guard<std::mutex> lock(authoredCurvesMutex_);
        return authoredCurves_;
    }

    RenderStateSnapshot CaptureRenderStateSnapshot() const {
        std::lock_guard<std::mutex> lock(renderStateMutex_);
        RenderStateSnapshot snapshot = {};
        snapshot.camera = camera_;
        snapshot.showOriginalCurve = showOriginalCurve_;
        snapshot.showSubdividedCurve = showSubdividedCurve_;
        snapshot.showRibbon = showRibbon_;
        snapshot.showRibbonWireframe = showRibbonWireframe_;
        return snapshot;
    }

    void RequestGeometryRebuild() {
        {
            std::lock_guard<std::mutex> lock(geometryRequestMutex_);
            ++geometryRequestVersion_;
        }
        geometryRequestCondition_.notify_one();
    }

    void UpdateCamera(float deltaSeconds) {
        const float panSpeed = std::max(1.5f, camera_.distance * 0.9f);
        const float cosYaw = std::cos(camera_.yaw);
        const float sinYaw = std::sin(camera_.yaw);

        float moveX = 0.0f;
        float moveZ = 0.0f;
        if (GetAsyncKeyState('W') & 0x8000) {
            moveX += -sinYaw;
            moveZ += -cosYaw;
        }
        if (GetAsyncKeyState('S') & 0x8000) {
            moveX -= -sinYaw;
            moveZ -= -cosYaw;
        }
        if (GetAsyncKeyState('D') & 0x8000) {
            moveX += -cosYaw;
            moveZ += sinYaw;
        }
        if (GetAsyncKeyState('A') & 0x8000) {
            moveX -= -cosYaw;
            moveZ -= sinYaw;
        }

        const float moveLength = std::sqrt(moveX * moveX + moveZ * moveZ);
        if (moveLength > 0.0f) {
            const float moveScale = panSpeed * deltaSeconds / moveLength;
            camera_.target.x += moveX * moveScale;
            camera_.target.z += moveZ * moveScale;
        }
    }

    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        if (message == WM_NCCREATE) {
            const auto* createInfo = reinterpret_cast<CREATESTRUCTW*>(lParam);
            auto* app = static_cast<RoadCurveApp*>(createInfo->lpCreateParams);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
            return TRUE;
        }

        auto* app = reinterpret_cast<RoadCurveApp*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        if (app != nullptr) {
            return app->HandleMessage(hwnd, message, wParam, lParam);
        }

        return DefWindowProcW(hwnd, message, wParam, lParam);
    }

    LRESULT HandleMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        switch (message) {
        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;
        case WM_LBUTTONDOWN: {
            DirectX::XMFLOAT3 groundPoint = {};
            if (ScreenPointToGroundPlane(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), groundPoint)) {
                AppendControlPointToCurrentCurve(groundPoint);
            }
            return 0;
        }
        case WM_KEYDOWN:
            if ((lParam & 0x40000000) == 0) {
                switch (wParam) {
                case VK_HOME:
                    ResetCamera();
                    return 0;
                case '1':
                    {
                        std::lock_guard<std::mutex> lock(renderStateMutex_);
                        showOriginalCurve_ = !showOriginalCurve_;
                    }
                    return 0;
                case '2':
                    {
                        std::lock_guard<std::mutex> lock(renderStateMutex_);
                        showSubdividedCurve_ = !showSubdividedCurve_;
                    }
                    return 0;
                case '3':
                    {
                        std::lock_guard<std::mutex> lock(renderStateMutex_);
                        showRibbon_ = !showRibbon_;
                    }
                    return 0;
                case '4':
                    {
                        std::lock_guard<std::mutex> lock(renderStateMutex_);
                        showRibbonWireframe_ = !showRibbonWireframe_;
                    }
                    return 0;
                case VK_RETURN:
                    FinishCurrentCurve();
                    return 0;
                default:
                    break;
                }
            }
            break;
        case WM_RBUTTONDOWN:
            orbitingCamera_ = true;
            lastMouseX_ = GET_X_LPARAM(lParam);
            lastMouseY_ = GET_Y_LPARAM(lParam);
            SetCapture(hwnd);
            return 0;
        case WM_RBUTTONUP:
            orbitingCamera_ = false;
            ReleaseCapture();
            return 0;
        case WM_CAPTURECHANGED:
            orbitingCamera_ = false;
            return 0;
        case WM_MOUSEMOVE:
            if (orbitingCamera_) {
                const int x = GET_X_LPARAM(lParam);
                const int y = GET_Y_LPARAM(lParam);
                const int dx = x - lastMouseX_;
                const int dy = y - lastMouseY_;
                lastMouseX_ = x;
                lastMouseY_ = y;

                std::lock_guard<std::mutex> lock(renderStateMutex_);
                camera_.yaw += static_cast<float>(dx) * 0.01f;
                camera_.pitch += static_cast<float>(dy) * 0.01f;
                camera_.pitch = ClampFloat(camera_.pitch, -1.35f, 1.35f);
                return 0;
            }
            break;
        case WM_MOUSEWHEEL: {
            const short wheelDelta = GET_WHEEL_DELTA_WPARAM(wParam);
            std::lock_guard<std::mutex> lock(renderStateMutex_);
            camera_.distance *= (wheelDelta > 0) ? 0.9f : 1.1f;
            camera_.distance = ClampFloat(camera_.distance, 2.0f, 20.0f);
            return 0;
        }
        case WM_DESTROY:
            running_.store(false);
            geometryRequestCondition_.notify_one();
            PostQuitMessage(0);
            return 0;
        default:
            break;
        }

        return DefWindowProcW(hwnd, message, wParam, lParam);
    }

    void CreateAppWindow(int cmdShow) {
        WNDCLASSW windowClass = {};
        windowClass.lpfnWndProc = WindowProc;
        windowClass.hInstance = instance_;
        windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
        windowClass.lpszClassName = kWindowClassName;
        RegisterClassW(&windowClass);

        RECT rect = {0, 0, static_cast<LONG>(width_), static_cast<LONG>(height_)};
        const DWORD windowStyle = WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX;
        AdjustWindowRect(&rect, windowStyle, FALSE);

        hwnd_ = CreateWindowExW(
            0,
            kWindowClassName,
            kWindowTitle,
            windowStyle,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            rect.right - rect.left,
            rect.bottom - rect.top,
            nullptr,
            nullptr,
            instance_,
            this);

        if (hwnd_ == nullptr) {
            FatalError("Failed to create the window.");
        }

        ShowWindow(hwnd_, cmdShow);
    }

    void InitializeD3D() {
        EnableDebugLayer();
        CreateFactory();
        CreateDevice();
        CreateCommandQueue();
        CreateSwapChain();
        CreateDescriptorHeaps();
        CreateCommandAllocators();
        CreateRenderTargets();
        CreateDepthBuffer();
        CreateRootSignature();
        LoadCompiledShaders();
        CreatePipelineStates();
        CreateCommandList();
        CreateConstantBuffer();
        CreateFenceObjects();
    }

    void EnableDebugLayer() {
#if defined(_DEBUG)
        ComPtr<ID3D12Debug> debugController;
        if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController)))) {
            debugController->EnableDebugLayer();
            dxgiFactoryFlags_ |= DXGI_CREATE_FACTORY_DEBUG;
        }
#endif
    }

    void CreateFactory() {
        ThrowIfFailed(
            CreateDXGIFactory2(dxgiFactoryFlags_, IID_PPV_ARGS(&factory_)),
            "Failed to create DXGI factory.");
    }

    ComPtr<IDXGIAdapter1> PickAdapter() const {
        ComPtr<IDXGIAdapter1> adapter;
        for (UINT adapterIndex = 0;
             factory_->EnumAdapters1(adapterIndex, &adapter) != DXGI_ERROR_NOT_FOUND;
             ++adapterIndex) {
            DXGI_ADAPTER_DESC1 desc = {};
            adapter->GetDesc1(&desc);

            if ((desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0) {
                continue;
            }

            if (SUCCEEDED(D3D12CreateDevice(
                    adapter.Get(),
                    D3D_FEATURE_LEVEL_11_0,
                    __uuidof(ID3D12Device),
                    nullptr))) {
                return adapter;
            }
        }

        ComPtr<IDXGIAdapter> warpAdapter;
        ThrowIfFailed(
            factory_->EnumWarpAdapter(IID_PPV_ARGS(&warpAdapter)),
            "Failed to enumerate the WARP adapter.");

        ComPtr<IDXGIAdapter1> warpAdapter1;
        ThrowIfFailed(warpAdapter.As(&warpAdapter1), "Failed to query IDXGIAdapter1 for WARP.");
        return warpAdapter1;
    }

    void CreateDevice() {
        ComPtr<IDXGIAdapter1> adapter = PickAdapter();
        ThrowIfFailed(
            D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device_)),
            "Failed to create the D3D12 device.");
    }

    void CreateCommandQueue() {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

        ThrowIfFailed(
            device_->CreateCommandQueue(&desc, IID_PPV_ARGS(&commandQueue_)),
            "Failed to create the command queue.");
    }

    void CreateSwapChain() {
        DXGI_SWAP_CHAIN_DESC1 desc = {};
        desc.BufferCount = kFrameCount;
        desc.Width = width_;
        desc.Height = height_;
        desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        desc.SampleDesc.Count = 1;

        ComPtr<IDXGISwapChain1> swapChain;
        ThrowIfFailed(
            factory_->CreateSwapChainForHwnd(
                commandQueue_.Get(),
                hwnd_,
                &desc,
                nullptr,
                nullptr,
                &swapChain),
            "Failed to create the swap chain.");

        ThrowIfFailed(
            factory_->MakeWindowAssociation(hwnd_, DXGI_MWA_NO_ALT_ENTER),
            "Failed to configure the swap chain window association.");

        ThrowIfFailed(swapChain.As(&swapChain_), "Failed to query IDXGISwapChain3.");
        frameIndex_ = swapChain_->GetCurrentBackBufferIndex();
    }

    void CreateDescriptorHeaps() {
        D3D12_DESCRIPTOR_HEAP_DESC rtvDesc = {};
        rtvDesc.NumDescriptors = kFrameCount;
        rtvDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;

        ThrowIfFailed(
            device_->CreateDescriptorHeap(&rtvDesc, IID_PPV_ARGS(&rtvHeap_)),
            "Failed to create the RTV descriptor heap.");

        D3D12_DESCRIPTOR_HEAP_DESC dsvDesc = {};
        dsvDesc.NumDescriptors = 1;
        dsvDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;

        ThrowIfFailed(
            device_->CreateDescriptorHeap(&dsvDesc, IID_PPV_ARGS(&dsvHeap_)),
            "Failed to create the DSV descriptor heap.");

        rtvDescriptorSize_ = device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    }

    void CreateCommandAllocators() {
        for (UINT i = 0; i < kFrameCount; ++i) {
            ThrowIfFailed(
                device_->CreateCommandAllocator(
                    D3D12_COMMAND_LIST_TYPE_DIRECT,
                    IID_PPV_ARGS(&commandAllocators_[i])),
                "Failed to create a command allocator.");
        }
    }

    void CreateRenderTargets() {
        D3D12_CPU_DESCRIPTOR_HANDLE handle = rtvHeap_->GetCPUDescriptorHandleForHeapStart();
        for (UINT i = 0; i < kFrameCount; ++i) {
            ThrowIfFailed(
                swapChain_->GetBuffer(i, IID_PPV_ARGS(&renderTargets_[i])),
                "Failed to acquire a back buffer.");
            device_->CreateRenderTargetView(renderTargets_[i].Get(), nullptr, handle);
            handle.ptr += static_cast<SIZE_T>(rtvDescriptorSize_);
        }

        viewport_.TopLeftX = 0.0f;
        viewport_.TopLeftY = 0.0f;
        viewport_.Width = static_cast<float>(width_);
        viewport_.Height = static_cast<float>(height_);
        viewport_.MinDepth = 0.0f;
        viewport_.MaxDepth = 1.0f;

        scissorRect_.right = static_cast<LONG>(width_);
        scissorRect_.bottom = static_cast<LONG>(height_);
    }

    void CreateDepthBuffer() {
        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

        D3D12_RESOURCE_DESC depthDesc = {};
        depthDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        depthDesc.Width = width_;
        depthDesc.Height = height_;
        depthDesc.DepthOrArraySize = 1;
        depthDesc.MipLevels = 1;
        depthDesc.Format = DXGI_FORMAT_D32_FLOAT;
        depthDesc.SampleDesc.Count = 1;
        depthDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
        depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

        D3D12_CLEAR_VALUE clearValue = {};
        clearValue.Format = DXGI_FORMAT_D32_FLOAT;
        clearValue.DepthStencil.Depth = 1.0f;

        ThrowIfFailed(
            device_->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &depthDesc,
                D3D12_RESOURCE_STATE_DEPTH_WRITE,
                &clearValue,
                IID_PPV_ARGS(&depthBuffer_)),
            "Failed to create the depth buffer.");

        D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
        dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
        dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;

        device_->CreateDepthStencilView(
            depthBuffer_.Get(),
            &dsvDesc,
            dsvHeap_->GetCPUDescriptorHandleForHeapStart());
    }

    void CreateRootSignature() {
        D3D12_ROOT_PARAMETER rootParameter = {};
        rootParameter.ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameter.Descriptor.ShaderRegister = 0;
        rootParameter.ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;

        D3D12_ROOT_SIGNATURE_DESC rootSignatureDesc = {};
        rootSignatureDesc.NumParameters = 1;
        rootSignatureDesc.pParameters = &rootParameter;
        rootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

        ComPtr<ID3DBlob> serializedRootSignature;
        ComPtr<ID3DBlob> errors;

        HRESULT hr = D3D12SerializeRootSignature(
            &rootSignatureDesc,
            D3D_ROOT_SIGNATURE_VERSION_1,
            &serializedRootSignature,
            &errors);

        if (FAILED(hr)) {
            if (errors != nullptr) {
                OutputDebugStringA(static_cast<const char*>(errors->GetBufferPointer()));
            }
            FatalError("Failed to serialize the root signature.", hr);
        }

        ThrowIfFailed(
            device_->CreateRootSignature(
                0,
                serializedRootSignature->GetBufferPointer(),
                serializedRootSignature->GetBufferSize(),
                IID_PPV_ARGS(&rootSignature_)),
            "Failed to create the root signature.");
    }

    void LoadCompiledShaders() {
        const std::wstring shaderDirectory = GetExecutableDirectory();
        lineVertexShader_ = ReadBinaryFile(shaderDirectory + L"\\cube_vs.cso");
        linePixelShader_ = ReadBinaryFile(shaderDirectory + L"\\cube_ps.cso");
        ribbonVertexShader_ = ReadBinaryFile(shaderDirectory + L"\\ribbon_vs.cso");
        ribbonPixelShader_ = ReadBinaryFile(shaderDirectory + L"\\ribbon_ps.cso");

        if (lineVertexShader_.empty() ||
            linePixelShader_.empty() ||
            ribbonVertexShader_.empty() ||
            ribbonPixelShader_.empty()) {
            FatalError("Failed to load precompiled shaders. Re-run CMake configure to regenerate the .cso files.");
        }
    }

    void CreatePipelineStates() {
        CreateLinePipelineState();
        CreateRibbonPipelineStates();
    }

    void CreateLinePipelineState() {
        static constexpr D3D12_INPUT_ELEMENT_DESC inputElements[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        };

        D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.InputLayout = {inputElements, _countof(inputElements)};
        psoDesc.pRootSignature = rootSignature_.Get();
        psoDesc.VS = {lineVertexShader_.data(), lineVertexShader_.size()};
        psoDesc.PS = {linePixelShader_.data(), linePixelShader_.size()};
        psoDesc.RasterizerState = DefaultRasterizerDesc();
        psoDesc.BlendState = OpaqueBlendDesc();
        psoDesc.DepthStencilState = DepthStencilDesc(D3D12_DEPTH_WRITE_MASK_ZERO);
        psoDesc.SampleMask = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
        psoDesc.NumRenderTargets = 1;
        psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        psoDesc.SampleDesc.Count = 1;

        ThrowIfFailed(
            device_->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&linePipelineState_)),
            "Failed to create the debug line graphics pipeline state.");
    }

    void CreateRibbonPipelineStates() {
        static constexpr D3D12_INPUT_ELEMENT_DESC ribbonInputElements[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"TEXCOORD", 1, DXGI_FORMAT_R32_FLOAT, 0, 20, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        };
        static constexpr D3D12_INPUT_ELEMENT_DESC wireframeInputElements[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        };

        D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.InputLayout = {ribbonInputElements, _countof(ribbonInputElements)};
        psoDesc.pRootSignature = rootSignature_.Get();
        psoDesc.VS = {ribbonVertexShader_.data(), ribbonVertexShader_.size()};
        psoDesc.PS = {ribbonPixelShader_.data(), ribbonPixelShader_.size()};
        psoDesc.RasterizerState = DefaultRasterizerDesc(D3D12_FILL_MODE_SOLID);
        psoDesc.BlendState = OpaqueBlendDesc();
        psoDesc.DepthStencilState = DepthStencilDesc(D3D12_DEPTH_WRITE_MASK_ALL);
        psoDesc.SampleMask = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        psoDesc.NumRenderTargets = 1;
        psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        psoDesc.SampleDesc.Count = 1;

        ThrowIfFailed(
            device_->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&ribbonPipelineState_)),
            "Failed to create the ribbon graphics pipeline state.");

        psoDesc.InputLayout = {wireframeInputElements, _countof(wireframeInputElements)};
        psoDesc.VS = {lineVertexShader_.data(), lineVertexShader_.size()};
        psoDesc.PS = {linePixelShader_.data(), linePixelShader_.size()};
        psoDesc.RasterizerState = DefaultRasterizerDesc(D3D12_FILL_MODE_WIREFRAME);
        ThrowIfFailed(
            device_->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&ribbonWireframePipelineState_)),
            "Failed to create the ribbon wireframe graphics pipeline state.");
    }

    void CreateCommandList() {
        ThrowIfFailed(
            device_->CreateCommandList(
                0,
                D3D12_COMMAND_LIST_TYPE_DIRECT,
                commandAllocators_[frameIndex_].Get(),
                linePipelineState_.Get(),
                IID_PPV_ARGS(&commandList_)),
            "Failed to create the command list.");

        ThrowIfFailed(commandList_->Close(), "Failed to close the initial command list.");
    }

    SceneGeometryCpuData BuildSceneGeometryData(const std::vector<PolylineCurve>& authoredCurves) const {
        constexpr float kRoadCleanupRadius = 0.45f;
        const DirectX::XMFLOAT3 tangentColor = {1.00f, 0.18f, 0.48f};
        SceneGeometryCpuData geometry = {};

        std::vector<ProcessedCurveData> processedCurves;
        processedCurves.reserve(authoredCurves.size());
        for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
            const PolylineCurve& roughCurve = authoredCurves[curveIndex];
            if (const auto error = ValidatePolylineCurve(roughCurve)) {
                continue;
            }

            ProcessedCurveData processedCurve = {};
            processedCurve.authoredIndex = curveIndex;
            processedCurve.curve = SubdividePolylineCurveTowardsBezierLimit(roughCurve);
            if (const auto error = ValidatePolylineCurve(processedCurve.curve)) {
                continue;
            }

            if (const auto issue = ComputeCurveTangents(
                    processedCurve.curve,
                    kRibbonTangentMode,
                    processedCurve.tangents)) {
                continue;
            }

            processedCurves.push_back(std::move(processedCurve));
        }

        geometry.gridRange = BuildGroundGrid(geometry.lineVertices);
        geometry.roughCurveRange.startVertex = static_cast<UINT>(geometry.lineVertices.size());
        for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
            BuildCurveSegments(authoredCurves[curveIndex], RoughCurveStyle(curveIndex), geometry.lineVertices);
        }
        geometry.roughCurveRange.vertexCount =
            static_cast<UINT>(geometry.lineVertices.size()) - geometry.roughCurveRange.startVertex;

        geometry.roughControlPointRange.startVertex = static_cast<UINT>(geometry.lineVertices.size());
        for (CurveIndex curveIndex = 0; curveIndex < authoredCurves.size(); ++curveIndex) {
            BuildControlPointMarkers(authoredCurves[curveIndex], RoughCurveStyle(curveIndex), geometry.lineVertices);
        }
        geometry.roughControlPointRange.vertexCount =
            static_cast<UINT>(geometry.lineVertices.size()) - geometry.roughControlPointRange.startVertex;

        geometry.subdividedCurveRange.startVertex = static_cast<UINT>(geometry.lineVertices.size());
        for (const ProcessedCurveData& processedCurve : processedCurves) {
            BuildCurveSegments(
                processedCurve.curve,
                SubdividedCurveStyle(processedCurve.authoredIndex),
                geometry.lineVertices);
        }
        geometry.subdividedCurveRange.vertexCount =
            static_cast<UINT>(geometry.lineVertices.size()) - geometry.subdividedCurveRange.startVertex;

        geometry.subdividedTangentRange.startVertex = static_cast<UINT>(geometry.lineVertices.size());
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
            static_cast<UINT>(geometry.lineVertices.size()) - geometry.subdividedTangentRange.startVertex;

        if (processedCurves.empty()) {
            return geometry;
        }

        std::vector<PolylineCurve> roadSpines;
        roadSpines.reserve(processedCurves.size());
        for (const ProcessedCurveData& processedCurve : processedCurves) {
            roadSpines.push_back(processedCurve.curve);
        }

        GenerateRoadResult generatedRoad = {};
        const auto issue = GenerateRoad(
            roadSpines,
            kRoadCleanupRadius,
            kRibbonHalfWidth,
            generatedRoad,
            kRibbonTangentMode);
        if (issue.has_value() ||
            generatedRoad.ribbonMesh.vertices.empty() ||
            generatedRoad.ribbonMesh.indices.empty()) {
            return geometry;
        }

        geometry.ribbonVertices = std::move(generatedRoad.ribbonMesh.vertices);
        geometry.ribbonIndices = std::move(generatedRoad.ribbonMesh.indices);
        return geometry;
    }

    void UploadSceneGeometryBuffers(const SceneGeometryCpuData& geometry) {
        gridRange_ = geometry.gridRange;
        roughCurveRange_ = geometry.roughCurveRange;
        roughControlPointRange_ = geometry.roughControlPointRange;
        subdividedCurveRange_ = geometry.subdividedCurveRange;
        subdividedTangentRange_ = geometry.subdividedTangentRange;
        ribbonVertexBuffer_.Reset();
        ribbonIndexBuffer_.Reset();
        ribbonVertexBufferView_ = {};
        ribbonIndexBufferView_ = {};
        ribbonIndexCount_ = 0;

        CreateStaticVertexBuffer(
            geometry.lineVertices,
            lineVertexBuffer_,
            lineVertexBufferView_,
            "Failed to create the debug line vertex buffer.");

        if (geometry.ribbonVertices.empty() || geometry.ribbonIndices.empty()) {
            return;
        }

        CreateStaticVertexBuffer(
            geometry.ribbonVertices,
            ribbonVertexBuffer_,
            ribbonVertexBufferView_,
            "Failed to create the ribbon vertex buffer.");
        CreateStaticIndexBuffer(
            geometry.ribbonIndices,
            ribbonIndexBuffer_,
            ribbonIndexBufferView_,
            "Failed to create the ribbon index buffer.");
        ribbonIndexCount_ = static_cast<UINT>(geometry.ribbonIndices.size());
    }

    void ApplyPendingSceneGeometry() {
        std::shared_ptr<SceneGeometryCpuData> pendingGeometry;
        std::uint64_t pendingVersion = 0;
        {
            std::lock_guard<std::mutex> lock(pendingGeometryMutex_);
            if (pendingGeometry_ == nullptr || pendingGeometryVersion_ <= uploadedGeometryVersion_) {
                return;
            }

            pendingGeometry = pendingGeometry_;
            pendingVersion = pendingGeometryVersion_;
        }

        if (commandQueue_ != nullptr && fence_ != nullptr && fenceEvent_ != nullptr) {
            WaitForGpu();
        }

        UploadSceneGeometryBuffers(*pendingGeometry);
        uploadedGeometryVersion_ = pendingVersion;
    }

    void CreateStaticBuffer(
        const void* sourceData,
        UINT bufferSize,
        ComPtr<ID3D12Resource>& buffer,
        const char* errorMessage) {
        if (sourceData == nullptr || bufferSize == 0) {
            FatalError("Cannot create a GPU buffer from empty source data.");
        }

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

        const D3D12_RESOURCE_DESC desc = BufferDesc(bufferSize);
        ThrowIfFailed(
            device_->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &desc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&buffer)),
            errorMessage);

        void* mappedData = nullptr;
        D3D12_RANGE readRange = {0, 0};
        ThrowIfFailed(buffer->Map(0, &readRange, &mappedData), "Failed to map an upload buffer.");
        std::memcpy(mappedData, sourceData, bufferSize);
        buffer->Unmap(0, nullptr);
    }

    template <typename VertexType>
    void CreateStaticVertexBuffer(
        const std::vector<VertexType>& vertices,
        ComPtr<ID3D12Resource>& buffer,
        D3D12_VERTEX_BUFFER_VIEW& view,
        const char* errorMessage) {
        if (vertices.empty()) {
            FatalError("Cannot create a GPU buffer for an empty vertex array.");
        }

        const UINT bufferSize = static_cast<UINT>(sizeof(VertexType) * vertices.size());
        CreateStaticBuffer(vertices.data(), bufferSize, buffer, errorMessage);

        view.BufferLocation = buffer->GetGPUVirtualAddress();
        view.StrideInBytes = sizeof(VertexType);
        view.SizeInBytes = bufferSize;
    }

    void CreateStaticIndexBuffer(
        const std::vector<std::uint32_t>& indices,
        ComPtr<ID3D12Resource>& buffer,
        D3D12_INDEX_BUFFER_VIEW& view,
        const char* errorMessage) {
        if (indices.empty()) {
            FatalError("Cannot create a GPU buffer for an empty index array.");
        }

        const UINT bufferSize = static_cast<UINT>(sizeof(std::uint32_t) * indices.size());
        CreateStaticBuffer(indices.data(), bufferSize, buffer, errorMessage);

        view.BufferLocation = buffer->GetGPUVirtualAddress();
        view.Format = DXGI_FORMAT_R32_UINT;
        view.SizeInBytes = bufferSize;
    }

    void CreateConstantBuffer() {
        constantBufferStride_ = AlignTo(
            static_cast<UINT>(sizeof(SceneConstants)),
            D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

        const D3D12_RESOURCE_DESC desc = BufferDesc(static_cast<UINT64>(constantBufferStride_) * kFrameCount);
        ThrowIfFailed(
            device_->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &desc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&constantBuffer_)),
            "Failed to create the constant buffer.");

        D3D12_RANGE readRange = {0, 0};
        ThrowIfFailed(
            constantBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&constantBufferDataBegin_)),
            "Failed to map the constant buffer.");

        std::memset(constantBufferDataBegin_, 0, static_cast<size_t>(constantBufferStride_) * kFrameCount);
    }

    void CreateFenceObjects() {
        ThrowIfFailed(
            device_->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence_)),
            "Failed to create the fence.");

        fenceValues_[frameIndex_] = 1;
        fenceEvent_ = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (fenceEvent_ == nullptr) {
            FatalError("Failed to create the fence event.", HRESULT_FROM_WIN32(GetLastError()));
        }
    }

    void StartThreads() {
        geometryWorkerThread_ = std::thread(&RoadCurveApp::GeometryWorkerMain, this);
        renderThread_ = std::thread(&RoadCurveApp::RenderThreadMain, this);
    }

    void StopThreads() {
        running_.store(false);
        geometryRequestCondition_.notify_one();

        if (geometryWorkerThread_.joinable()) {
            geometryWorkerThread_.join();
        }

        if (renderThread_.joinable()) {
            renderThread_.join();
        }
    }

    void MainLoop() {
        MSG message = {};
        while (running_.load()) {
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }

            if (!running_.load()) {
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            const float deltaSeconds = std::chrono::duration<float>(now - previousFrameTime_).count();
            previousFrameTime_ = now;
            {
                std::lock_guard<std::mutex> lock(renderStateMutex_);
                UpdateCamera(deltaSeconds);
            }
            Sleep(1);
        }
    }

    void RenderThreadMain() {
        while (running_.load()) {
            ApplyPendingSceneGeometry();
            RenderFrame(CaptureRenderStateSnapshot());
        }
    }

    void GeometryWorkerMain() {
        std::uint64_t processedRequestVersion = 0;

        while (true) {
            std::uint64_t requestVersion = 0;
            {
                std::unique_lock<std::mutex> lock(geometryRequestMutex_);
                geometryRequestCondition_.wait(lock, [this, &processedRequestVersion]() {
                    return !running_.load() || geometryRequestVersion_ != processedRequestVersion;
                });

                if (!running_.load() && geometryRequestVersion_ == processedRequestVersion) {
                    return;
                }

                requestVersion = geometryRequestVersion_;
            }

            SceneGeometryCpuData geometry = BuildSceneGeometryData(CaptureAuthoredCurvesSnapshot());

            {
                std::lock_guard<std::mutex> lock(geometryRequestMutex_);
                if (requestVersion != geometryRequestVersion_) {
                    processedRequestVersion = requestVersion;
                    continue;
                }
            }

            {
                std::lock_guard<std::mutex> lock(pendingGeometryMutex_);
                pendingGeometry_ = std::make_shared<SceneGeometryCpuData>(std::move(geometry));
                pendingGeometryVersion_ = requestVersion;
            }

            processedRequestVersion = requestVersion;
        }
    }

    void RenderFrame(const RenderStateSnapshot& renderState) {
        UpdateSceneConstants(renderState);
        PopulateCommandList(renderState);

        ID3D12CommandList* commandLists[] = {commandList_.Get()};
        commandQueue_->ExecuteCommandLists(1, commandLists);

        ThrowIfFailed(swapChain_->Present(1, 0), "Failed to present the swap chain.");
        MoveToNextFrame();
    }

    void UpdateSceneConstants(const RenderStateSnapshot& renderState) {
        const DirectX::XMMATRIX view = CameraViewMatrix(renderState.camera);
        const DirectX::XMMATRIX projection = ProjectionMatrix();

        SceneConstants constants = {};
        DirectX::XMStoreFloat4x4(&constants.mvp, DirectX::XMMatrixTranspose(view * projection));

        std::memcpy(
            constantBufferDataBegin_ + static_cast<size_t>(frameIndex_) * constantBufferStride_,
            &constants,
            sizeof(constants));
    }

    void PopulateCommandList(const RenderStateSnapshot& renderState) {
        ThrowIfFailed(commandAllocators_[frameIndex_]->Reset(), "Failed to reset the command allocator.");
        ThrowIfFailed(
            commandList_->Reset(commandAllocators_[frameIndex_].Get(), linePipelineState_.Get()),
            "Failed to reset the command list.");

        commandList_->SetGraphicsRootSignature(rootSignature_.Get());
        commandList_->RSSetViewports(1, &viewport_);
        commandList_->RSSetScissorRects(1, &scissorRect_);

        const D3D12_RESOURCE_BARRIER toRenderTarget = TransitionBarrier(
            renderTargets_[frameIndex_].Get(),
            D3D12_RESOURCE_STATE_PRESENT,
            D3D12_RESOURCE_STATE_RENDER_TARGET);
        commandList_->ResourceBarrier(1, &toRenderTarget);

        const D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = CurrentBackBufferView();
        const D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle = dsvHeap_->GetCPUDescriptorHandleForHeapStart();

        constexpr float clearColor[] = {0.06f, 0.08f, 0.12f, 1.0f};
        commandList_->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
        commandList_->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
        commandList_->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

        const D3D12_GPU_VIRTUAL_ADDRESS cbAddress =
            constantBuffer_->GetGPUVirtualAddress() + static_cast<UINT64>(frameIndex_) * constantBufferStride_;

        commandList_->SetGraphicsRootConstantBufferView(0, cbAddress);
        if (renderState.showRibbon && ribbonIndexCount_ > 0) {
            commandList_->SetPipelineState(
                renderState.showRibbonWireframe ? ribbonWireframePipelineState_.Get() : ribbonPipelineState_.Get());
            commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            commandList_->IASetVertexBuffers(0, 1, &ribbonVertexBufferView_);
            commandList_->IASetIndexBuffer(&ribbonIndexBufferView_);
            commandList_->DrawIndexedInstanced(ribbonIndexCount_, 1, 0, 0, 0);
        }

        commandList_->SetPipelineState(linePipelineState_.Get());
        commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        if (lineVertexBufferView_.SizeInBytes > 0) {
            commandList_->IASetVertexBuffers(0, 1, &lineVertexBufferView_);
            commandList_->DrawInstanced(gridRange_.vertexCount, 1, gridRange_.startVertex, 0);
        }
        if (renderState.showOriginalCurve && lineVertexBufferView_.SizeInBytes > 0) {
            commandList_->DrawInstanced(roughCurveRange_.vertexCount, 1, roughCurveRange_.startVertex, 0);
            commandList_->DrawInstanced(
                roughControlPointRange_.vertexCount,
                1,
                roughControlPointRange_.startVertex,
                0);
        }
        if (renderState.showSubdividedCurve && lineVertexBufferView_.SizeInBytes > 0) {
            commandList_->DrawInstanced(
                subdividedCurveRange_.vertexCount,
                1,
                subdividedCurveRange_.startVertex,
                0);
            commandList_->DrawInstanced(
                subdividedTangentRange_.vertexCount,
                1,
                subdividedTangentRange_.startVertex,
                0);
        }

        const D3D12_RESOURCE_BARRIER toPresent = TransitionBarrier(
            renderTargets_[frameIndex_].Get(),
            D3D12_RESOURCE_STATE_RENDER_TARGET,
            D3D12_RESOURCE_STATE_PRESENT);
        commandList_->ResourceBarrier(1, &toPresent);

        ThrowIfFailed(commandList_->Close(), "Failed to close the command list.");
    }

    void MoveToNextFrame() {
        const UINT64 currentFenceValue = fenceValues_[frameIndex_];
        ThrowIfFailed(
            commandQueue_->Signal(fence_.Get(), currentFenceValue),
            "Failed to signal the fence.");

        frameIndex_ = swapChain_->GetCurrentBackBufferIndex();

        if (fence_->GetCompletedValue() < fenceValues_[frameIndex_]) {
            ThrowIfFailed(
                fence_->SetEventOnCompletion(fenceValues_[frameIndex_], fenceEvent_),
                "Failed to set the fence completion event.");
            WaitForSingleObject(fenceEvent_, INFINITE);
        }

        fenceValues_[frameIndex_] = currentFenceValue + 1;
    }

    void WaitForGpu() {
        ThrowIfFailed(
            commandQueue_->Signal(fence_.Get(), fenceValues_[frameIndex_]),
            "Failed to signal the fence while waiting for the GPU.");
        ThrowIfFailed(
            fence_->SetEventOnCompletion(fenceValues_[frameIndex_], fenceEvent_),
            "Failed to set the fence completion event.");
        WaitForSingleObject(fenceEvent_, INFINITE);
        ++fenceValues_[frameIndex_];
    }

    void Cleanup() {
        if (device_ != nullptr && commandQueue_ != nullptr && fence_ != nullptr && fenceEvent_ != nullptr) {
            WaitForGpu();
        }

        if (constantBuffer_ != nullptr && constantBufferDataBegin_ != nullptr) {
            constantBuffer_->Unmap(0, nullptr);
            constantBufferDataBegin_ = nullptr;
        }

        if (fenceEvent_ != nullptr) {
            CloseHandle(fenceEvent_);
            fenceEvent_ = nullptr;
        }
    }

    D3D12_CPU_DESCRIPTOR_HANDLE CurrentBackBufferView() const {
        D3D12_CPU_DESCRIPTOR_HANDLE handle = rtvHeap_->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += static_cast<SIZE_T>(frameIndex_) * rtvDescriptorSize_;
        return handle;
    }

    HINSTANCE instance_ = nullptr;
    HWND hwnd_ = nullptr;
    std::atomic<bool> running_{true};
    UINT width_ = kWindowWidth;
    UINT height_ = kWindowHeight;
    UINT dxgiFactoryFlags_ = 0;

    ComPtr<IDXGIFactory4> factory_;
    ComPtr<ID3D12Device> device_;
    ComPtr<ID3D12CommandQueue> commandQueue_;
    ComPtr<IDXGISwapChain3> swapChain_;
    ComPtr<ID3D12DescriptorHeap> rtvHeap_;
    ComPtr<ID3D12DescriptorHeap> dsvHeap_;
    ComPtr<ID3D12CommandAllocator> commandAllocators_[kFrameCount];
    ComPtr<ID3D12GraphicsCommandList> commandList_;
    ComPtr<ID3D12Fence> fence_;

    ComPtr<ID3D12Resource> renderTargets_[kFrameCount];
    ComPtr<ID3D12Resource> depthBuffer_;
    ComPtr<ID3D12RootSignature> rootSignature_;
    ComPtr<ID3D12PipelineState> linePipelineState_;
    ComPtr<ID3D12PipelineState> ribbonPipelineState_;
    ComPtr<ID3D12PipelineState> ribbonWireframePipelineState_;
    std::vector<std::uint8_t> lineVertexShader_;
    std::vector<std::uint8_t> linePixelShader_;
    std::vector<std::uint8_t> ribbonVertexShader_;
    std::vector<std::uint8_t> ribbonPixelShader_;

    ComPtr<ID3D12Resource> lineVertexBuffer_;
    ComPtr<ID3D12Resource> ribbonVertexBuffer_;
    ComPtr<ID3D12Resource> ribbonIndexBuffer_;
    ComPtr<ID3D12Resource> constantBuffer_;

    DrawRange gridRange_ = {};
    DrawRange roughCurveRange_ = {};
    DrawRange roughControlPointRange_ = {};
    DrawRange subdividedCurveRange_ = {};
    DrawRange subdividedTangentRange_ = {};

    D3D12_VIEWPORT viewport_ = {};
    D3D12_RECT scissorRect_ = {};
    D3D12_VERTEX_BUFFER_VIEW lineVertexBufferView_ = {};
    D3D12_VERTEX_BUFFER_VIEW ribbonVertexBufferView_ = {};
    D3D12_INDEX_BUFFER_VIEW ribbonIndexBufferView_ = {};

    UINT rtvDescriptorSize_ = 0;
    UINT frameIndex_ = 0;
    UINT constantBufferStride_ = 0;
    UINT ribbonIndexCount_ = 0;
    UINT64 fenceValues_[kFrameCount] = {};
    HANDLE fenceEvent_ = nullptr;
    std::uint8_t* constantBufferDataBegin_ = nullptr;
    mutable std::mutex renderStateMutex_;
    OrbitCamera camera_{};
    bool showOriginalCurve_ = true;
    bool showSubdividedCurve_ = true;
    bool showRibbon_ = true;
    bool showRibbonWireframe_ = false;
    mutable std::mutex authoredCurvesMutex_;
    std::vector<PolylineCurve> authoredCurves_;
    std::mutex geometryRequestMutex_;
    std::condition_variable geometryRequestCondition_;
    std::uint64_t geometryRequestVersion_ = 0;
    std::mutex pendingGeometryMutex_;
    std::shared_ptr<SceneGeometryCpuData> pendingGeometry_;
    std::uint64_t pendingGeometryVersion_ = 0;
    std::uint64_t uploadedGeometryVersion_ = 0;
    std::thread renderThread_;
    std::thread geometryWorkerThread_;
    bool currentCurveActive_ = false;
    bool orbitingCamera_ = false;
    int lastMouseX_ = 0;
    int lastMouseY_ = 0;
    std::chrono::steady_clock::time_point previousFrameTime_;
};

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int cmdShow) {
    RoadCurveApp app;
    return app.Run(instance, cmdShow);
}
