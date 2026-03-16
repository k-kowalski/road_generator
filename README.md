# DX12 Colored Cube

Minimal Win32 + DirectX 12 app scaffolded as a standalone CMake project. It now renders a rotating cube-derived mesh backed by a simple in-project half-edge structure, with a small runtime face-extrusion deformation demo.

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

- The app starts from a colored cube stored as a half-edge mesh.
- It performs one simple inset-extrude on the top face.
- Each frame it uses half-edge face and neighbor traversal to animate that edited region and uploads the updated mesh to the GPU.
