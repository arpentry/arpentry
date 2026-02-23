cmake_minimum_required(VERSION 3.20)
project(arpentry C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)
include(CTest)

# ── Shared dependencies ─────────────────────────────────────────────────

# Brotli (static)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(brotli
    GIT_REPOSITORY https://github.com/google/brotli.git
    GIT_TAG        v1.1.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(brotli)

# FlatCC — FlatBuffers compiler + runtime for C
FetchContent_Declare(flatcc
    GIT_REPOSITORY https://github.com/dvidelabs/flatcc.git
    GIT_TAG        v0.6.1
    GIT_SHALLOW    TRUE
)
set(FLATCC_INSTALL OFF CACHE BOOL "" FORCE)
set(FLATCC_TEST    OFF CACHE BOOL "" FORCE)
if(EMSCRIPTEN)
    set(FLATCC_RTONLY ON CACHE BOOL "" FORCE)
endif()
FetchContent_MakeAvailable(flatcc)

# FlatCC v0.6.1 sets -Werror but triggers warnings on newer Clang.
foreach(_tgt flatccrt flatcc flatcc_cli)
    if(TARGET ${_tgt})
        target_compile_options(${_tgt} PRIVATE
            -Wno-error=c23-extensions
            -Wno-error=unused-but-set-variable
            -Wno-error=implicit-int-conversion
        )
    endif()
endforeach()

# Unity test framework (no upstream CMake — built manually)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(unity)
if(NOT unity_POPULATED)
    FetchContent_Populate(unity)
endif()
add_library(unity STATIC ${unity_SOURCE_DIR}/src/unity.c)
target_include_directories(unity PUBLIC ${unity_SOURCE_DIR}/src)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)

# Civetweb (embedded HTTP server for tile serving)
if(NOT EMSCRIPTEN)
    set(CIVETWEB_ENABLE_SERVER_EXECUTABLE OFF CACHE BOOL "" FORCE)
    set(CIVETWEB_BUILD_TESTING            OFF CACHE BOOL "" FORCE)
    set(CIVETWEB_ENABLE_SSL               OFF CACHE BOOL "" FORCE)
    set(CIVETWEB_ENABLE_CXX               OFF CACHE BOOL "" FORCE)
    set(CIVETWEB_ENABLE_ASAN              OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(civetweb
        GIT_REPOSITORY https://github.com/civetweb/civetweb.git
        GIT_TAG        v1.16
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(civetweb)
endif()

# ── FlatBuffers schema compilation ───────────────────────────────────────

set(FLATCC_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated/flatcc")
file(MAKE_DIRECTORY ${FLATCC_GENERATED_DIR})

if(FLATCC_HOST_COMPILER)
    set(FLATCC_EXE ${FLATCC_HOST_COMPILER})
else()
    set(FLATCC_EXE $<TARGET_FILE:flatcc_cli>)
endif()

set(FLATCC_SCHEMAS ${CMAKE_SOURCE_DIR}/schemas/tile.fbs)

add_custom_command(
    OUTPUT
        ${FLATCC_GENERATED_DIR}/tile_builder.h
        ${FLATCC_GENERATED_DIR}/tile_reader.h
        ${FLATCC_GENERATED_DIR}/tile_verifier.h
    COMMAND ${FLATCC_EXE} -a -o ${FLATCC_GENERATED_DIR} ${FLATCC_SCHEMAS}
    DEPENDS ${FLATCC_SCHEMAS}
    COMMENT "Compiling FlatBuffers schemas"
)
add_custom_target(flatcc_generate DEPENDS
    ${FLATCC_GENERATED_DIR}/tile_builder.h
    ${FLATCC_GENERATED_DIR}/tile_reader.h
    ${FLATCC_GENERATED_DIR}/tile_verifier.h
)

# ── Subdirectories ───────────────────────────────────────────────────────

add_subdirectory(common)
add_subdirectory(client)
add_subdirectory(server)
