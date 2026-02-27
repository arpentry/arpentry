# ── arpentry server: tile server ──────────────────────────────────────────

if(EMSCRIPTEN)
    return()
endif()

# ── Pogocache vendored sources (multi-threaded event loop) ────────────
set(POGOCACHE_SOURCES
    src/net.c
    src/tls.c
    src/uring.c
)

add_executable(arpentry_server
    src/main.c
    src/http.c
    src/tile_path.c
    src/noise.c
    src/terrain_gen.c
    ${POGOCACHE_SOURCES}
)

target_compile_definitions(arpentry_server PRIVATE NOOPENSSL NOURING)

add_dependencies(arpentry_server flatcc_generate)

target_include_directories(arpentry_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(Threads REQUIRED)
target_link_libraries(arpentry_server PRIVATE arpentry_common Threads::Threads)

# ── Tests ──────────────────────────────────────────────────────────────

add_executable(test_server tests/test_server.c src/tile_path.c)
target_include_directories(test_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(test_server PRIVATE unity)
add_test(NAME test_server COMMAND test_server)

add_executable(test_http tests/test_http.c tests/mock_net.c
    src/http.c src/tile_path.c src/noise.c src/terrain_gen.c)
add_dependencies(test_http flatcc_generate)
target_include_directories(test_http PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(test_http PRIVATE arpentry_common unity)
add_test(NAME test_http COMMAND test_http)

add_executable(test_terrain_gen tests/test_terrain_gen.c src/noise.c src/terrain_gen.c)
add_dependencies(test_terrain_gen flatcc_generate)
target_include_directories(test_terrain_gen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(test_terrain_gen PRIVATE arpentry_common unity)
add_test(NAME test_terrain_gen COMMAND test_terrain_gen)
