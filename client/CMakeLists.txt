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

# ── Main executable ──────────────────────────────────────────────────────

add_executable(arpentry_client
    src/main.c
    src/http.c
    src/tile_fetch.c
    src/camera.c
    src/control.c
    src/tile_decode.c
    src/renderer.c
    src/tile_visible.c
    src/tile_manager.c
    src/ui.c
)
add_dependencies(arpentry_client flatcc_generate)
target_include_directories(arpentry_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(arpentry_client PRIVATE webgpu glfw3webgpu arpentry_common)

# ── Tests (native only) ──────────────────────────────────────────────
if(NOT EMSCRIPTEN)
    find_package(Threads REQUIRED)

    add_executable(test_tile_fetch tests/test_tile_fetch.c src/tile_fetch.c src/http.c
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

