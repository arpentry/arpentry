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
    src/parquet.c
    src/geoparquet.c
    src/overture.c
)

target_include_directories(arpentry_tiler PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_dependencies(arpentry_tiler flatcc_generate)
target_include_directories(arpentry_tiler PUBLIC
    ${FLATCC_GENERATED_DIR}
    ${flatcc_SOURCE_DIR}/include
)
target_link_libraries(arpentry_tiler PRIVATE arpentry_warnings carquet json_c flatccrt brotlienc brotlidec)
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
target_link_libraries(test_tile_build PRIVATE arpentry_tiler arpentry_common unity)
add_test(NAME test_tile_build COMMAND test_tile_build)

add_executable(test_archive tests/test_archive.c)
target_link_libraries(test_archive PRIVATE arpentry_tiler unity)
add_test(NAME test_archive COMMAND test_archive)

add_executable(test_pipeline tests/test_pipeline.c)
target_link_libraries(test_pipeline PRIVATE arpentry_tiler arpentry_common unity)
add_test(NAME test_pipeline COMMAND test_pipeline)

add_executable(test_parquet tests/test_parquet.c)
target_link_libraries(test_parquet PRIVATE arpentry_tiler carquet unity)
add_test(NAME test_parquet COMMAND test_parquet)

add_executable(test_geoparquet tests/test_geoparquet.c)
target_link_libraries(test_geoparquet PRIVATE arpentry_tiler unity)
add_test(NAME test_geoparquet COMMAND test_geoparquet)

add_executable(test_overture tests/test_overture.c)
target_link_libraries(test_overture PRIVATE arpentry_tiler carquet unity)
add_test(NAME test_overture COMMAND test_overture)

add_executable(test_overture_integration tests/test_overture_integration.c)
target_compile_definitions(test_overture_integration PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/overture")
target_link_libraries(test_overture_integration PRIVATE arpentry_tiler carquet unity)
add_test(NAME test_overture_integration COMMAND test_overture_integration)
