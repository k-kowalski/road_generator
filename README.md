# RoadGen

Repo layout after the core split:

- `src/core/`: platform-independent road generation, mesh utilities, and the shared demo scene builder.
- `src/native_dx12/`: Win32 + DirectX 12 wrapper over the shared core.
- `web/src/`: wasm bridge and WebGPU site code.
- `web/shaders/`: WGSL shaders for the web demo.
- `tests/core/`: core regression coverage.

## Native DX12 build

Requirements:

- CMake
- MSVC / Visual Studio C++ toolchain
- Windows SDK shader tools (`fxc.exe`)

Commands:

```powershell
cmake -S . -B build
cmake --build build --config Debug
```

Run:

```powershell
.\build\Debug\RoadGen.exe
```

On this machine the verified native toolchain entrypoint is:

```powershell
D:\VisualStudio\2022\Enterprise\Common7\Tools\VsDevCmd.bat
```

Native controls:

- Left click: append a control point to the current curve.
- `Enter`: finish the current curve.
- Right drag: orbit the camera.
- Mouse wheel: zoom.
- `W`, `A`, `S`, `D`: pan.
- `Home`: reset camera.
- `1` / `2` / `3` / `4`: toggle rough curve, subdivided curve, ribbon, ribbon wireframe.

## Core tests

```powershell
ctest --test-dir build --output-on-failure
```

## Web build

Requirements:

- Emscripten (`emcmake`, `emmake`)
- A browser with WebGPU enabled

Commands:

```powershell
emcmake cmake -S . -B build-web
cmake --build build-web
```

The web output is written to `web/dist/`. Serve that folder with any static file server.

The web demo mirrors the native feature set and consumes the same C++ core through wasm.

The generated web bundle is single-file/self-contained on the wasm side and uses classic scripts, so `web/dist/index.html` can also be opened directly from disk without the previous `file://` CORS/module blocking issue.

If you only want the core tests or the web build on a machine without the native DX12 prerequisites, configure with `-DROADGEN_BUILD_NATIVE_DX12=OFF`.

A repo-local `emsdk` can live under `tools/emsdk/`; in this workspace that is the verified wasm toolchain location.

## GitHub Pages

The repository includes a Pages workflow at `.github/workflows/deploy-pages.yml` that builds the web demo with Emscripten on pushes to `master` and deploys `web/dist` using the official GitHub Pages actions.

For this repository, the default project-site URL is expected to be:

```text
https://k-kowalski.github.io/road_generator/
```

If Pages has not been enabled yet in the repository settings, set the Pages source to `GitHub Actions` once and subsequent pushes to `master` will publish the current web build.
