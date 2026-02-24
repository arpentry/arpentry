# ── arpentry_common: shared library for client & server ─────────────────

add_library(arpentry_common STATIC
    src/coords.c
    src/tile.c
    src/buf.c
    src/hashmap.c
    src/globe.c
)

target_include_directories(arpentry_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Shared deps from root (FlatCC generated headers + runtime, Brotli)
add_dependencies(arpentry_common flatcc_generate)
target_include_directories(arpentry_common PUBLIC
    ${FLATCC_GENERATED_DIR}
    ${flatcc_SOURCE_DIR}/include
)
target_link_libraries(arpentry_common PUBLIC flatccrt brotlienc brotlidec)

# ── Tests (native only) ────────────────────────────────────────────────

if(NOT EMSCRIPTEN)
    add_executable(test_common tests/test_common.c)
    target_link_libraries(test_common PRIVATE arpentry_common unity)
    add_test(NAME test_common COMMAND test_common)

    add_executable(test_brotli tests/test_brotli.c)
    target_link_libraries(test_brotli PRIVATE arpentry_common unity)
    add_test(NAME test_brotli COMMAND test_brotli)

    add_executable(test_flatbuf tests/test_flatbuf.c)
    target_link_libraries(test_flatbuf PRIVATE arpentry_common unity)
    add_test(NAME test_flatbuf COMMAND test_flatbuf)

    add_executable(test_tile tests/test_tile.c)
    target_link_libraries(test_tile PRIVATE arpentry_common unity)
    add_test(NAME test_tile COMMAND test_tile)

    add_executable(test_math3d tests/test_math3d.c)
    target_link_libraries(test_math3d PRIVATE arpentry_common unity)
    add_test(NAME test_math3d COMMAND test_math3d)

    add_executable(test_globe tests/test_globe.c)
    target_link_libraries(test_globe PRIVATE arpentry_common unity)
    add_test(NAME test_globe COMMAND test_globe)

    add_executable(test_camera tests/test_camera.c
        ${CMAKE_SOURCE_DIR}/client/src/camera.c)
    target_include_directories(test_camera PRIVATE
        ${CMAKE_SOURCE_DIR}/client/src)
    target_link_libraries(test_camera PRIVATE arpentry_common unity)
    add_test(NAME test_camera COMMAND test_camera)

    add_executable(test_tile_decode tests/test_tile_decode.c
        ${CMAKE_SOURCE_DIR}/client/src/tile_decode.c)
    add_dependencies(test_tile_decode flatcc_generate)
    target_include_directories(test_tile_decode PRIVATE
        ${CMAKE_SOURCE_DIR}/client/src)
    target_link_libraries(test_tile_decode PRIVATE arpentry_common unity)
    add_test(NAME test_tile_decode COMMAND test_tile_decode)

    add_executable(test_tile_manager tests/test_tile_manager.c
        ${CMAKE_SOURCE_DIR}/client/src/tile_visible.c
        ${CMAKE_SOURCE_DIR}/client/src/camera.c)
    target_include_directories(test_tile_manager PRIVATE
        ${CMAKE_SOURCE_DIR}/client/src)
    target_link_libraries(test_tile_manager PRIVATE arpentry_common unity)
    add_test(NAME test_tile_manager COMMAND test_tile_manager)
endif()
