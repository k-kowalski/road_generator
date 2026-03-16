#include <windows.h>
#include <wrl/client.h>

#include <d3d12.h>
#include <dxgi1_6.h>

#include <DirectXMath.h>

#include "HalfEdgeMesh.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using Microsoft::WRL::ComPtr;

namespace {

constexpr UINT kFrameCount = 2;
constexpr UINT kWindowWidth = 1280;
constexpr UINT kWindowHeight = 720;
constexpr wchar_t kWindowClassName[] = L"DX12ColoredCubeWindow";
constexpr wchar_t kWindowTitle[] = L"DX12 Colored Cube";

struct SceneConstants {
    DirectX::XMFLOAT4X4 mvp;
};

UINT AlignTo(UINT value, UINT alignment) {
    return (value + alignment - 1U) & ~(alignment - 1U);
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
    MessageBoxA(nullptr, buffer, "DX12ColoredCube", MB_OK | MB_ICONERROR);
    ExitProcess(1);
}

void ThrowIfFailed(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        FatalError(message, hr);
    }
}

[[noreturn]] void FatalHalfEdgeError(HalfEdgeMesh::ErrorCode error) {
    char buffer[32] = {};
    std::snprintf(buffer, sizeof(buffer), "%u", static_cast<unsigned>(error));
    FatalError(buffer);
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

class CubeApp {
public:
    int Run(HINSTANCE instance, int cmdShow) {
        instance_ = instance;
        CreateAppWindow(cmdShow);
        InitializeD3D();
        startTime_ = std::chrono::steady_clock::now();
        MainLoop();
        Cleanup();
        return 0;
    }

private:
    void UpdateWindowTitle() const {
        SetWindowTextW(hwnd_, showWireframe_ ? L"DX12 Colored Cube [Wireframe]" : kWindowTitle);
    }

    void ToggleWireframe() {
        showWireframe_ = !showWireframe_;
        UpdateWindowTitle();
    }

    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        if (message == WM_NCCREATE) {
            const auto* createInfo = reinterpret_cast<CREATESTRUCTW*>(lParam);
            auto* app = static_cast<CubeApp*>(createInfo->lpCreateParams);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
            return TRUE;
        }

        auto* app = reinterpret_cast<CubeApp*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
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
        case WM_KEYDOWN:
            if ((lParam & 0x40000000) == 0 && (wParam == 'B' || wParam == 'b')) {
                ToggleWireframe();
                return 0;
            }
            break;
        case WM_DESTROY:
            running_ = false;
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
        UpdateWindowTitle();
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
        CreatePipelineState();
        CreateCommandList();
        CreateGeometryBuffers();
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
        vertexShader_ = ReadBinaryFile(shaderDirectory + L"\\cube_vs.cso");
        pixelShader_ = ReadBinaryFile(shaderDirectory + L"\\cube_ps.cso");

        if (vertexShader_.empty() || pixelShader_.empty()) {
            FatalError("Failed to load precompiled shaders. Re-run CMake configure to regenerate the .cso files.");
        }
    }

    void CreatePipelineState() {
        static constexpr D3D12_INPUT_ELEMENT_DESC inputElements[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
            {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        };

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

        D3D12_DEPTH_STENCIL_DESC depthStencilDesc = {};
        depthStencilDesc.DepthEnable = TRUE;
        depthStencilDesc.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
        depthStencilDesc.DepthFunc = D3D12_COMPARISON_FUNC_LESS;

        const auto createMeshPso = [&](D3D12_FILL_MODE fillMode, ComPtr<ID3D12PipelineState>& target, const char* errorMessage) {
            D3D12_RASTERIZER_DESC rasterizerDesc = {};
            rasterizerDesc.FillMode = fillMode;
            rasterizerDesc.CullMode = D3D12_CULL_MODE_NONE;
            rasterizerDesc.FrontCounterClockwise = FALSE;
            rasterizerDesc.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
            rasterizerDesc.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
            rasterizerDesc.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
            rasterizerDesc.DepthClipEnable = TRUE;

            D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
            psoDesc.InputLayout = {inputElements, _countof(inputElements)};
            psoDesc.pRootSignature = rootSignature_.Get();
            psoDesc.VS = {vertexShader_.data(), vertexShader_.size()};
            psoDesc.PS = {pixelShader_.data(), pixelShader_.size()};
            psoDesc.RasterizerState = rasterizerDesc;
            psoDesc.BlendState = blendDesc;
            psoDesc.DepthStencilState = depthStencilDesc;
            psoDesc.SampleMask = UINT_MAX;
            psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            psoDesc.NumRenderTargets = 1;
            psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
            psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
            psoDesc.SampleDesc.Count = 1;

            ThrowIfFailed(
                device_->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&target)),
                errorMessage);
        };

        createMeshPso(D3D12_FILL_MODE_SOLID, pipelineState_, "Failed to create the solid graphics pipeline state.");
        createMeshPso(D3D12_FILL_MODE_WIREFRAME, wireframePipelineState_, "Failed to create the wireframe graphics pipeline state.");
    }

    void CreateCommandList() {
        ThrowIfFailed(
            device_->CreateCommandList(
                0,
                D3D12_COMMAND_LIST_TYPE_DIRECT,
                commandAllocators_[frameIndex_].Get(),
                pipelineState_.Get(),
                IID_PPV_ARGS(&commandList_)),
            "Failed to create the command list.");

        ThrowIfFailed(commandList_->Close(), "Failed to close the initial command list.");
    }

    void CreateGeometryBuffers() {
        HalfEdgeMeshBuildResult buildResult = MakeColoredCubeMesh();
        if (buildResult.error) {
            FatalHalfEdgeError(*buildResult.error);
        }
        editableMesh_ = std::move(buildResult.mesh);

        const std::uint32_t topFace = editableMesh_.FindBestFace({0.0f, 1.0f, 0.0f});
        if (topFace == HalfEdgeMesh::Invalid) {
            FatalError("Failed to locate the cube's top face.");
        }

        const HalfEdgeMesh::ExtrusionResult extrusion = editableMesh_.InsetExtrudeFace(topFace, 0.28f, 0.75f);
        if (extrusion.error) {
            FatalHalfEdgeError(*extrusion.error);
        }

        animatedFace_ = extrusion.topFace;

        if (const auto error = editableMesh_.Validate()) {
            FatalHalfEdgeError(*error);
        }

        const std::string meshSummary = editableMesh_.Summary();
        OutputDebugStringA(meshSummary.c_str());

        editableMesh_.AnimateFaceRegion(animatedFace_, 0.0f);
        editableMesh_.BuildRenderBuffers(cpuVertices_, cpuIndices_);

        vertexBufferSliceSize_ = AlignTo(static_cast<UINT>(sizeof(RenderVertex) * cpuVertices_.size()), 256);
        indexBufferSliceSize_ = AlignTo(static_cast<UINT>(sizeof(std::uint16_t) * cpuIndices_.size()), 256);

        CreateMappedUploadBuffer(
            static_cast<UINT64>(vertexBufferSliceSize_) * kFrameCount,
            vertexBuffer_,
            reinterpret_cast<void**>(&vertexBufferDataBegin_),
            "Failed to create the editable vertex buffer.");

        CreateMappedUploadBuffer(
            static_cast<UINT64>(indexBufferSliceSize_) * kFrameCount,
            indexBuffer_,
            reinterpret_cast<void**>(&indexBufferDataBegin_),
            "Failed to create the editable index buffer.");

        UploadMeshForCurrentFrame();
    }

    void CreateMappedUploadBuffer(
        UINT64 sizeInBytes,
        ComPtr<ID3D12Resource>& buffer,
        void** mappedData,
        const char* errorMessage) {
        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_UPLOAD;

        const D3D12_RESOURCE_DESC desc = BufferDesc(sizeInBytes);
        ThrowIfFailed(
            device_->CreateCommittedResource(
                &heapProps,
                D3D12_HEAP_FLAG_NONE,
                &desc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&buffer)),
            errorMessage);

        D3D12_RANGE readRange = {0, 0};
        ThrowIfFailed(buffer->Map(0, &readRange, mappedData), "Failed to map an upload buffer.");
    }

    void UploadMeshForCurrentFrame() {
        editableMesh_.BuildRenderBuffers(cpuVertices_, cpuIndices_);

        const UINT vertexBytes = static_cast<UINT>(sizeof(RenderVertex) * cpuVertices_.size());
        const UINT indexBytes = static_cast<UINT>(sizeof(std::uint16_t) * cpuIndices_.size());

        if (vertexBytes > vertexBufferSliceSize_ || indexBytes > indexBufferSliceSize_) {
            FatalError("The editable mesh grew beyond the current GPU buffer capacity.");
        }

        const size_t vertexOffset = static_cast<size_t>(frameIndex_) * vertexBufferSliceSize_;
        const size_t indexOffset = static_cast<size_t>(frameIndex_) * indexBufferSliceSize_;

        std::memcpy(vertexBufferDataBegin_ + vertexOffset, cpuVertices_.data(), vertexBytes);
        std::memcpy(indexBufferDataBegin_ + indexOffset, cpuIndices_.data(), indexBytes);

        vertexBufferView_.BufferLocation = vertexBuffer_->GetGPUVirtualAddress() + vertexOffset;
        vertexBufferView_.StrideInBytes = sizeof(RenderVertex);
        vertexBufferView_.SizeInBytes = vertexBytes;

        indexBufferView_.BufferLocation = indexBuffer_->GetGPUVirtualAddress() + indexOffset;
        indexBufferView_.Format = DXGI_FORMAT_R16_UINT;
        indexBufferView_.SizeInBytes = indexBytes;

        indexCount_ = static_cast<UINT>(cpuIndices_.size());
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

    void MainLoop() {
        MSG message = {};
        while (running_) {
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }

            if (!running_) {
                break;
            }

            RenderFrame();
        }
    }

    void RenderFrame() {
        UpdateSceneConstants();
        PopulateCommandList();

        ID3D12CommandList* commandLists[] = {commandList_.Get()};
        commandQueue_->ExecuteCommandLists(1, commandLists);

        ThrowIfFailed(swapChain_->Present(1, 0), "Failed to present the swap chain.");
        MoveToNextFrame();
    }

    void UpdateSceneConstants() {
        const float timeSeconds = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - startTime_).count();

        editableMesh_.AnimateFaceRegion(animatedFace_, timeSeconds);
        UploadMeshForCurrentFrame();

        const DirectX::XMMATRIX world =
            DirectX::XMMatrixRotationRollPitchYaw(timeSeconds * 0.35f, timeSeconds * 0.55f, 0.0f);
        const DirectX::XMMATRIX view = DirectX::XMMatrixLookAtLH(
            DirectX::XMVectorSet(0.0f, 1.25f, -5.0f, 1.0f),
            DirectX::XMVectorZero(),
            DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
        const DirectX::XMMATRIX projection = DirectX::XMMatrixPerspectiveFovLH(
            DirectX::XMConvertToRadians(60.0f),
            static_cast<float>(width_) / static_cast<float>(height_),
            0.1f,
            100.0f);

        SceneConstants constants = {};
        DirectX::XMStoreFloat4x4(&constants.mvp, DirectX::XMMatrixTranspose(world * view * projection));

        std::memcpy(
            constantBufferDataBegin_ + static_cast<size_t>(frameIndex_) * constantBufferStride_,
            &constants,
            sizeof(constants));
    }

    void PopulateCommandList() {
        ID3D12PipelineState* activePipelineState = showWireframe_ ? wireframePipelineState_.Get() : pipelineState_.Get();

        ThrowIfFailed(commandAllocators_[frameIndex_]->Reset(), "Failed to reset the command allocator.");
        ThrowIfFailed(
            commandList_->Reset(commandAllocators_[frameIndex_].Get(), activePipelineState),
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
        commandList_->SetPipelineState(activePipelineState);
        commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList_->IASetVertexBuffers(0, 1, &vertexBufferView_);
        commandList_->IASetIndexBuffer(&indexBufferView_);
        commandList_->DrawIndexedInstanced(indexCount_, 1, 0, 0, 0);

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

        if (vertexBuffer_ != nullptr && vertexBufferDataBegin_ != nullptr) {
            vertexBuffer_->Unmap(0, nullptr);
            vertexBufferDataBegin_ = nullptr;
        }

        if (indexBuffer_ != nullptr && indexBufferDataBegin_ != nullptr) {
            indexBuffer_->Unmap(0, nullptr);
            indexBufferDataBegin_ = nullptr;
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
    bool running_ = true;
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
    ComPtr<ID3D12PipelineState> pipelineState_;
    ComPtr<ID3D12PipelineState> wireframePipelineState_;
    std::vector<std::uint8_t> vertexShader_;
    std::vector<std::uint8_t> pixelShader_;

    ComPtr<ID3D12Resource> vertexBuffer_;
    ComPtr<ID3D12Resource> indexBuffer_;
    ComPtr<ID3D12Resource> constantBuffer_;

    HalfEdgeMesh editableMesh_;
    std::vector<RenderVertex> cpuVertices_;
    std::vector<std::uint16_t> cpuIndices_;

    D3D12_VIEWPORT viewport_ = {};
    D3D12_RECT scissorRect_ = {};
    D3D12_VERTEX_BUFFER_VIEW vertexBufferView_ = {};
    D3D12_INDEX_BUFFER_VIEW indexBufferView_ = {};

    UINT rtvDescriptorSize_ = 0;
    UINT frameIndex_ = 0;
    UINT indexCount_ = 0;
    UINT constantBufferStride_ = 0;
    UINT vertexBufferSliceSize_ = 0;
    UINT indexBufferSliceSize_ = 0;
    UINT64 fenceValues_[kFrameCount] = {};
    HANDLE fenceEvent_ = nullptr;
    std::uint8_t* vertexBufferDataBegin_ = nullptr;
    std::uint8_t* indexBufferDataBegin_ = nullptr;
    std::uint8_t* constantBufferDataBegin_ = nullptr;
    std::uint32_t animatedFace_ = HalfEdgeMesh::Invalid;
    bool showWireframe_ = false;
    std::chrono::steady_clock::time_point startTime_;
};

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int cmdShow) {
    CubeApp app;
    return app.Run(instance, cmdShow);
}
