cmake_minimum_required(VERSION 3.20)

project(RoadGen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROADGEN_BUILD_NATIVE_DX12 "Build the native DX12 demo app." ON)
option(ROADGEN_BUILD_CORE_TESTS "Build the core test executable." ON)

set(ROADGEN_CORE_FILES
    src/core/HalfEdgeMesh.cpp
    src/core/HalfEdgeMesh.h
    src/core/MathTypes.h
    src/core/PolylineCurve.cpp
    src/core/PolylineCurve.h
    src/core/RibbonMesh.cpp
    src/core/RibbonMesh.h
    src/core/RoadDemo.cpp
    src/core/RoadDemo.h
    src/core/RoadGeneration.cpp
    src/core/RoadGeneration.h
)

add_library(roadgen_core ${ROADGEN_CORE_FILES})
target_include_directories(roadgen_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/core)

if(MSVC)
    target_compile_options(roadgen_core PRIVATE /W4 /EHsc)
else()
    target_compile_options(roadgen_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32 AND ROADGEN_BUILD_NATIVE_DX12 AND NOT EMSCRIPTEN)
    set(ROADGEN_NATIVE_SHADER_FILES
        src/native_dx12/shaders/cube.hlsl
        src/native_dx12/shaders/ribbon.hlsl
    )

    add_executable(RoadGen WIN32
        src/native_dx12/main.cpp
        ${ROADGEN_NATIVE_SHADER_FILES}
    )
    target_link_libraries(RoadGen PRIVATE
        roadgen_core
        d3d12
        dxgi
        dxguid
        user32
        gdi32
    )
    target_compile_definitions(RoadGen PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        UNICODE
        _UNICODE
    )

    source_group("Shaders" FILES ${ROADGEN_NATIVE_SHADER_FILES})
    set_source_files_properties(${ROADGEN_NATIVE_SHADER_FILES} PROPERTIES HEADER_FILE_ONLY YES)

    if(MSVC)
        target_compile_options(RoadGen PRIVATE /W4 /EHsc)
    endif()

    set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
    set(LINE_SHADER_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/native_dx12/shaders/cube.hlsl")
    set(LINE_VERTEX_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/cube_vs.cso")
    set(LINE_PIXEL_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/cube_ps.cso")
    set(RIBBON_SHADER_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/native_dx12/shaders/ribbon.hlsl")
    set(RIBBON_VERTEX_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/ribbon_vs.cso")
    set(RIBBON_PIXEL_SHADER_OUTPUT "${SHADER_OUTPUT_DIR}/ribbon_ps.cso")

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
        COMMAND "${FXC_EXE}" /nologo /T vs_5_0 /E VSMain /Fo "${LINE_VERTEX_SHADER_OUTPUT}" "${LINE_SHADER_SOURCE}"
        RESULT_VARIABLE FXC_VS_RESULT
        OUTPUT_VARIABLE FXC_VS_STDOUT
        ERROR_VARIABLE FXC_VS_STDERR
    )

    if(NOT FXC_VS_RESULT EQUAL 0)
        message(FATAL_ERROR "Vertex shader compilation failed during CMake configure.\n${FXC_VS_STDOUT}\n${FXC_VS_STDERR}")
    endif()

    execute_process(
        COMMAND "${FXC_EXE}" /nologo /T ps_5_0 /E PSMain /Fo "${LINE_PIXEL_SHADER_OUTPUT}" "${LINE_SHADER_SOURCE}"
        RESULT_VARIABLE FXC_PS_RESULT
        OUTPUT_VARIABLE FXC_PS_STDOUT
        ERROR_VARIABLE FXC_PS_STDERR
    )

    if(NOT FXC_PS_RESULT EQUAL 0)
        message(FATAL_ERROR "Pixel shader compilation failed during CMake configure.\n${FXC_PS_STDOUT}\n${FXC_PS_STDERR}")
    endif()

    execute_process(
        COMMAND "${FXC_EXE}" /nologo /T vs_5_0 /E VSMain /Fo "${RIBBON_VERTEX_SHADER_OUTPUT}" "${RIBBON_SHADER_SOURCE}"
        RESULT_VARIABLE FXC_RIBBON_VS_RESULT
        OUTPUT_VARIABLE FXC_RIBBON_VS_STDOUT
        ERROR_VARIABLE FXC_RIBBON_VS_STDERR
    )

    if(NOT FXC_RIBBON_VS_RESULT EQUAL 0)
        message(FATAL_ERROR "Ribbon vertex shader compilation failed during CMake configure.\n${FXC_RIBBON_VS_STDOUT}\n${FXC_RIBBON_VS_STDERR}")
    endif()

    execute_process(
        COMMAND "${FXC_EXE}" /nologo /T ps_5_0 /E PSMain /Fo "${RIBBON_PIXEL_SHADER_OUTPUT}" "${RIBBON_SHADER_SOURCE}"
        RESULT_VARIABLE FXC_RIBBON_PS_RESULT
        OUTPUT_VARIABLE FXC_RIBBON_PS_STDOUT
        ERROR_VARIABLE FXC_RIBBON_PS_STDERR
    )

    if(NOT FXC_RIBBON_PS_RESULT EQUAL 0)
        message(FATAL_ERROR "Ribbon pixel shader compilation failed during CMake configure.\n${FXC_RIBBON_PS_STDOUT}\n${FXC_RIBBON_PS_STDERR}")
    endif()

    add_custom_command(TARGET RoadGen POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LINE_VERTEX_SHADER_OUTPUT}"
            "$<TARGET_FILE_DIR:RoadGen>/cube_vs.cso"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${LINE_PIXEL_SHADER_OUTPUT}"
            "$<TARGET_FILE_DIR:RoadGen>/cube_ps.cso"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${RIBBON_VERTEX_SHADER_OUTPUT}"
            "$<TARGET_FILE_DIR:RoadGen>/ribbon_vs.cso"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${RIBBON_PIXEL_SHADER_OUTPUT}"
            "$<TARGET_FILE_DIR:RoadGen>/ribbon_ps.cso"
    )

    set_target_properties(RoadGen PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:RoadGen>"
    )

    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT RoadGen)
endif()

if(ROADGEN_BUILD_CORE_TESTS AND NOT EMSCRIPTEN)
    enable_testing()

    add_executable(roadgen_core_tests
        tests/core/CoreTests.cpp
    )
    target_link_libraries(roadgen_core_tests PRIVATE roadgen_core)

    if(MSVC)
        target_compile_options(roadgen_core_tests PRIVATE /W4 /EHsc)
    else()
        target_compile_options(roadgen_core_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME roadgen_core_tests COMMAND roadgen_core_tests)
endif()

if(EMSCRIPTEN)
    add_executable(road_demo_core
        web/src/road_demo_wasm.cpp
    )
    target_link_libraries(road_demo_core PRIVATE roadgen_core)
    set_target_properties(road_demo_core PROPERTIES
        OUTPUT_NAME "road_demo_core"
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/web/dist"
    )
    target_link_options(road_demo_core PRIVATE
        "-sALLOW_MEMORY_GROWTH=1"
        "-sENVIRONMENT=web"
        "-sEXPORT_NAME=createRoadDemoModule"
        "SHELL:-sEXPORTED_RUNTIME_METHODS=['HEAPU8']"
        "-sFILESYSTEM=0"
        "-sMODULARIZE=1"
        "-sSINGLE_FILE=1"
        "-sWASM=1"
    )

    add_custom_target(road_demo_web_assets ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/index.html"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/index.html"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/src/main.js"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/main.js"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/src/math.js"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/math.js"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/src/shaders.js"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/shaders.js"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/src/site.css"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/site.css"
        COMMAND ${CMAKE_COMMAND} -E rm -f
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/road_demo_core.wasm"
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/shaders"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/shaders/lines.wgsl"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/shaders/lines.wgsl"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/web/shaders/ribbon.wgsl"
            "${CMAKE_CURRENT_SOURCE_DIR}/web/dist/shaders/ribbon.wgsl"
    )
    add_dependencies(road_demo_web_assets road_demo_core)
endif()
