# DX12 Colored Cube

Minimal Win32 + DirectX 12 app scaffolded as a standalone CMake project. It renders a rotating 3D cube with per-vertex colors.

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
