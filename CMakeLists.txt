cmake_minimum_required(VERSION 3.20)

project(DX12ColoredCube LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(DX12ColoredCube WIN32
    src/main.cpp
    shaders/cube.hlsl
)

source_group("Shaders" FILES shaders/cube.hlsl)
set_source_files_properties(shaders/cube.hlsl PROPERTIES HEADER_FILE_ONLY YES)

set(SHADER_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cube.hlsl")
set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
set(VERTEX_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/cube_vs.cso")
set(PIXEL_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/cube_ps.cso")

find_program(FXC_EXE
    NAMES fxc
    HINTS
        "D:/Windows Kits/10/bin/10.0.22000.0/x64"
        "D:/Windows Kits/10/bin/x64"
        "C:/Program Files (x86)/Windows Kits/10/bin/10.0.22000.0/x64"
        "C:/Program Files (x86)/Windows Kits/10/bin/x64"
)

if(NOT FXC_EXE)
    message(FATAL_ERROR "fxc.exe not found. Install the Windows SDK shader tools or update the CMake search hints.")
endif()

file(MAKE_DIRECTORY "${SHADER_OUTPUT_DIR}")

execute_process(
    COMMAND "${FXC_EXE}" /nologo /T vs_5_0 /E VSMain /Fo "${VERTEX_SHADER_OUTPUT}" "${SHADER_SOURCE}"
    RESULT_VARIABLE FXC_VS_RESULT
    OUTPUT_VARIABLE FXC_VS_STDOUT
    ERROR_VARIABLE FXC_VS_STDERR
)

if(NOT FXC_VS_RESULT EQUAL 0)
    message(FATAL_ERROR "Vertex shader compilation failed during CMake configure.\n${FXC_VS_STDOUT}\n${FXC_VS_STDERR}")
endif()

execute_process(
    COMMAND "${FXC_EXE}" /nologo /T ps_5_0 /E PSMain /Fo "${PIXEL_SHADER_OUTPUT}" "${SHADER_SOURCE}"
    RESULT_VARIABLE FXC_PS_RESULT
    OUTPUT_VARIABLE FXC_PS_STDOUT
    ERROR_VARIABLE FXC_PS_STDERR
)

if(NOT FXC_PS_RESULT EQUAL 0)
    message(FATAL_ERROR "Pixel shader compilation failed during CMake configure.\n${FXC_PS_STDOUT}\n${FXC_PS_STDERR}")
endif()

target_compile_definitions(DX12ColoredCube PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    UNICODE
    _UNICODE
)

if(MSVC)
    target_compile_options(DX12ColoredCube PRIVATE /W4 /EHsc)
endif()

target_link_libraries(DX12ColoredCube PRIVATE
    d3d12
    dxgi
    dxguid
    user32
    gdi32
)

add_custom_command(TARGET DX12ColoredCube POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E rm -f
        "$<TARGET_FILE_DIR:DX12ColoredCube>/cube.hlsl"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VERTEX_SHADER_OUTPUT}"
        "$<TARGET_FILE_DIR:DX12ColoredCube>/cube_vs.cso"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PIXEL_SHADER_OUTPUT}"
        "$<TARGET_FILE_DIR:DX12ColoredCube>/cube_ps.cso"
)

set_target_properties(DX12ColoredCube PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:DX12ColoredCube>"
)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT DX12ColoredCube)
