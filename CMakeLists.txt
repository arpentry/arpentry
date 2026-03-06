cmake_minimum_required(VERSION 3.20)
project(arpentry C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)
include(CTest)

# ── Project-wide compiler options ─────────────────────────────────────

# Strict warnings for our own code (applied per-target below, not globally)
add_library(arpentry_warnings INTERFACE)
target_compile_options(arpentry_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Werror
    -Wno-unused-parameter          # too noisy for callback-heavy code
    -Wno-overlength-strings        # embedded shaders exceed C99 minimum
    -Wno-c23-extensions            # flatcc headers use [[]] attributes
)

# Optional sanitizers (on by default for Debug builds)
option(ARPENTRY_SANITIZERS "Enable ASan + UBSan" OFF)
if(ARPENTRY_SANITIZERS)
    add_library(arpentry_sanitizers INTERFACE)
    target_compile_options(arpentry_sanitizers INTERFACE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(arpentry_sanitizers INTERFACE
        -fsanitize=address,undefined)
endif()

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

# json.c — lightweight JSON parser (no upstream CMake — built manually)
FetchContent_Declare(json_c
    GIT_REPOSITORY https://github.com/tidwall/json.c.git
    GIT_TAG        v1.1.8
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(json_c)
if(NOT json_c_POPULATED)
    FetchContent_Populate(json_c)
endif()
add_library(json_c STATIC ${json_c_SOURCE_DIR}/json.c)
target_include_directories(json_c PUBLIC ${json_c_SOURCE_DIR})

# ── FlatBuffers schema compilation ───────────────────────────────────────

set(FLATCC_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated/flatcc")
file(MAKE_DIRECTORY ${FLATCC_GENERATED_DIR})

if(FLATCC_HOST_COMPILER)
    set(FLATCC_EXE ${FLATCC_HOST_COMPILER})
else()
    set(FLATCC_EXE $<TARGET_FILE:flatcc_cli>)
endif()

set(FLATCC_SCHEMAS
    ${CMAKE_SOURCE_DIR}/schemas/tile.fbs
    ${CMAKE_SOURCE_DIR}/schemas/tileset.fbs
    ${CMAKE_SOURCE_DIR}/schemas/style.fbs
    ${CMAKE_SOURCE_DIR}/schemas/model.fbs
)

add_custom_command(
    OUTPUT
        ${FLATCC_GENERATED_DIR}/tile_builder.h
        ${FLATCC_GENERATED_DIR}/tile_reader.h
        ${FLATCC_GENERATED_DIR}/tile_verifier.h
        ${FLATCC_GENERATED_DIR}/tileset_builder.h
        ${FLATCC_GENERATED_DIR}/tileset_reader.h
        ${FLATCC_GENERATED_DIR}/tileset_verifier.h
        ${FLATCC_GENERATED_DIR}/style_builder.h
        ${FLATCC_GENERATED_DIR}/style_reader.h
        ${FLATCC_GENERATED_DIR}/style_verifier.h
        ${FLATCC_GENERATED_DIR}/model_builder.h
        ${FLATCC_GENERATED_DIR}/model_reader.h
        ${FLATCC_GENERATED_DIR}/model_verifier.h
    COMMAND ${FLATCC_EXE} -a -o ${FLATCC_GENERATED_DIR} ${FLATCC_SCHEMAS}
    DEPENDS ${FLATCC_SCHEMAS}
    COMMENT "Compiling FlatBuffers schemas"
)
add_custom_target(flatcc_generate DEPENDS
    ${FLATCC_GENERATED_DIR}/tile_builder.h
    ${FLATCC_GENERATED_DIR}/tile_reader.h
    ${FLATCC_GENERATED_DIR}/tile_verifier.h
    ${FLATCC_GENERATED_DIR}/tileset_builder.h
    ${FLATCC_GENERATED_DIR}/tileset_reader.h
    ${FLATCC_GENERATED_DIR}/tileset_verifier.h
    ${FLATCC_GENERATED_DIR}/style_builder.h
    ${FLATCC_GENERATED_DIR}/style_reader.h
    ${FLATCC_GENERATED_DIR}/style_verifier.h
    ${FLATCC_GENERATED_DIR}/model_builder.h
    ${FLATCC_GENERATED_DIR}/model_reader.h
    ${FLATCC_GENERATED_DIR}/model_verifier.h
)

# ── Subdirectories ───────────────────────────────────────────────────────

add_subdirectory(common)
add_subdirectory(client)
add_subdirectory(server)
