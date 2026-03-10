# ── arpentry_tiler: tile generation library and CLI ───────────────────

# Native only — not built for Emscripten
if(EMSCRIPTEN)
    return()
endif()

# ── Library ──────────────────────────────────────────────────────────

add_library(arpentry_tiler STATIC
    src/hilbert.c
    src/wkb.c
    src/simplify.c
    src/clip.c
    src/sort.c
    src/tile_build.c
    src/archive.c
    src/pipeline.c
)

target_include_directories(arpentry_tiler PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(arpentry_tiler PRIVATE arpentry_warnings)
if(ARPENTRY_SANITIZERS)
    target_link_libraries(arpentry_tiler PUBLIC arpentry_sanitizers)
endif()

# ── CLI executable ───────────────────────────────────────────────────

add_executable(arpentry_tiler_cli src/main.c)
set_target_properties(arpentry_tiler_cli PROPERTIES OUTPUT_NAME arpentry_tiler)
target_link_libraries(arpentry_tiler_cli PRIVATE arpentry_tiler arpentry_warnings)

# ── Tests ────────────────────────────────────────────────────────────

add_executable(test_hilbert tests/test_hilbert.c)
target_link_libraries(test_hilbert PRIVATE arpentry_tiler unity)
add_test(NAME test_hilbert COMMAND test_hilbert)

add_executable(test_wkb tests/test_wkb.c)
target_link_libraries(test_wkb PRIVATE arpentry_tiler unity)
add_test(NAME test_wkb COMMAND test_wkb)

add_executable(test_simplify tests/test_simplify.c)
target_link_libraries(test_simplify PRIVATE arpentry_tiler unity)
add_test(NAME test_simplify COMMAND test_simplify)

add_executable(test_clip tests/test_clip.c)
target_link_libraries(test_clip PRIVATE arpentry_tiler unity)
add_test(NAME test_clip COMMAND test_clip)

add_executable(test_sort tests/test_sort.c)
target_link_libraries(test_sort PRIVATE arpentry_tiler unity)
add_test(NAME test_sort COMMAND test_sort)

add_executable(test_tile_build tests/test_tile_build.c)
target_link_libraries(test_tile_build PRIVATE arpentry_tiler unity)
add_test(NAME test_tile_build COMMAND test_tile_build)

add_executable(test_archive tests/test_archive.c)
target_link_libraries(test_archive PRIVATE arpentry_tiler unity)
add_test(NAME test_archive COMMAND test_archive)

add_executable(test_pipeline tests/test_pipeline.c)
target_link_libraries(test_pipeline PRIVATE arpentry_tiler unity)
add_test(NAME test_pipeline COMMAND test_pipeline)
