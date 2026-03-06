include(FetchContent)

# ── Client-only dependencies ─────────────────────────────────────────────

FetchContent_Declare(webgpu
    GIT_REPOSITORY https://github.com/eliemichel/WebGPU-distribution.git
    GIT_TAG        main-v0.2.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(webgpu)

if(NOT EMSCRIPTEN)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(glfw)
endif()

FetchContent_Declare(glfw3webgpu
    GIT_REPOSITORY https://github.com/eliemichel/glfw3webgpu.git
    GIT_TAG        v1.2.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw3webgpu)

FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(stb)

# ── Embed WGSL shaders as C headers ─────────────────────────────────────

set(SHADER_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/shaders)
file(MAKE_DIRECTORY ${SHADER_GEN_DIR})

set(SHADER_SOURCES
    terrain.wgsl
    surface.wgsl
    highway.wgsl
    wireframe.wgsl
    ui.wgsl
    tree.wgsl
    poi.wgsl
)

set(SHADER_HEADERS)
foreach(SHADER ${SHADER_SOURCES})
    string(REPLACE "." "_" VAR_NAME ${SHADER})   # e.g. terrain_wgsl
    set(INPUT  ${SHADER_DIR}/${SHADER})
    set(OUTPUT ${SHADER_GEN_DIR}/${SHADER}.h)
    add_custom_command(
        OUTPUT  ${OUTPUT}
        COMMAND ${CMAKE_COMMAND}
            -DINPUT=${INPUT}
            -DINPUT_NAME=${SHADER}
            -DOUTPUT=${OUTPUT}
            -DVAR_NAME=${VAR_NAME}
            -P ${CMAKE_SOURCE_DIR}/cmake/embed_shader.cmake
        DEPENDS ${INPUT} ${CMAKE_SOURCE_DIR}/cmake/embed_shader.cmake
        COMMENT "Embedding shader ${SHADER}"
    )
    list(APPEND SHADER_HEADERS ${OUTPUT})
endforeach()

add_custom_target(embed_shaders DEPENDS ${SHADER_HEADERS})

# ── Embed binary assets as C headers ────────────────────────────────────

set(ASSET_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/assets)
set(ASSET_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/assets)
file(MAKE_DIRECTORY ${ASSET_GEN_DIR})

set(FONT_INPUT  ${ASSET_DIR}/Inter-Regular.ttf)
set(FONT_OUTPUT ${ASSET_GEN_DIR}/Inter-Regular.ttf.h)
add_custom_command(
    OUTPUT  ${FONT_OUTPUT}
    COMMAND ${CMAKE_COMMAND}
        -DINPUT=${FONT_INPUT}
        -DINPUT_NAME=Inter-Regular.ttf
        -DOUTPUT=${FONT_OUTPUT}
        -DVAR_NAME=inter_regular_ttf
        -P ${CMAKE_SOURCE_DIR}/cmake/embed_binary.cmake
    DEPENDS ${FONT_INPUT} ${CMAKE_SOURCE_DIR}/cmake/embed_binary.cmake
    COMMENT "Embedding font Inter-Regular.ttf"
)
add_custom_target(embed_assets DEPENDS ${FONT_OUTPUT})

# ── Main executable ──────────────────────────────────────────────────────

add_executable(arpentry_client
    src/main.c
    src/http.c
    src/tile/fetch.c
    src/camera.c
    src/control.c
    src/tile/decode.c
    src/tile/prepare.c
    src/pipeline/core.c
    src/pipeline/mesh.c
    src/pipeline/texture.c
    src/pipeline/extrusion.c
    src/pipeline/instance.c
    src/pipeline/label.c
    src/pipeline/placeholder.c
    src/style.c
    src/tile/visible.c
    src/tile/manager.c
    src/ui.c
    src/font.c
)
add_dependencies(arpentry_client flatcc_generate embed_shaders embed_assets)
target_include_directories(arpentry_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SHADER_GEN_DIR}
    ${ASSET_GEN_DIR}
    ${stb_SOURCE_DIR}
)
target_link_libraries(arpentry_client PRIVATE webgpu glfw3webgpu arpentry_common arpentry_warnings)

# ── Tests (native only) ──────────────────────────────────────────────
if(NOT EMSCRIPTEN)
    find_package(Threads REQUIRED)

    add_executable(test_tile_fetch tests/test_tile_fetch.c src/tile/fetch.c src/http.c
        tests/demo_tile.c)
    add_dependencies(test_tile_fetch flatcc_generate)
    target_include_directories(test_tile_fetch PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_link_libraries(test_tile_fetch PRIVATE unity arpentry_common Threads::Threads)
    add_test(NAME test_tile_fetch COMMAND test_tile_fetch)
endif()

if(EMSCRIPTEN)
    # -sUSE_WEBGPU is provided by the webgpu target's INTERFACE link options
    target_link_options(arpentry_client PRIVATE
        -sUSE_GLFW=3 -sASYNCIFY -sALLOW_MEMORY_GROWTH -sFETCH
        --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html
    )
    set_target_properties(arpentry_client PROPERTIES SUFFIX ".html")
else()
    target_link_libraries(arpentry_client PRIVATE glfw Threads::Threads)
    target_copy_webgpu_binaries(arpentry_client)
endif()

