# ── arpentry_common: shared library for client & server ─────────────────

add_library(arpentry_common STATIC
    src/coords.c
    src/tile.c
    src/buf.c
    src/hashmap.c
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
endif()
