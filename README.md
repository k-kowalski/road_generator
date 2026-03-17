# DX12 Road Curve

Minimal Win32 + DirectX 12 app scaffolded as a standalone CMake project. It now starts with an authored ground-plane polyline road intent, rendered with a lightweight debug line path and a simple reference grid.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Debug
```

## Run

```powershell
.\build\Debug\DX12ColoredCube.exe
```

`CMakeLists.txt` compiles `shaders/cube.hlsl` into `cube_vs.cso` and `cube_ps.cso` during CMake configure, then copies those compiled shader blobs next to the executable during the build.

If you edit the HLSL, re-run the CMake configure step before rebuilding:

```powershell
cmake -S . -B build
```

## Current demo

- The app starts with a hard-coded S-shaped `PolylineCurve` on the `XZ` ground plane.
- A muted ground grid gives the curve spatial context without involving mesh generation.
- Control points are drawn as distinct markers so the scene already reads like authored road input.
