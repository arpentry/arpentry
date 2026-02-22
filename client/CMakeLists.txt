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

add_executable(arpentry src/main.c)
target_link_libraries(arpentry PRIVATE webgpu glfw3webgpu)

if(EMSCRIPTEN)
    # -sUSE_WEBGPU is provided by the webgpu target's INTERFACE link options
    target_link_options(arpentry PRIVATE
        -sUSE_GLFW=3 -sASYNCIFY -sALLOW_MEMORY_GROWTH
    )
    set_target_properties(arpentry PROPERTIES SUFFIX ".html")
else()
    target_link_libraries(arpentry PRIVATE glfw)
    target_copy_webgpu_binaries(arpentry)
endif()

