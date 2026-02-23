# ── arpentry server: tile server with Civetweb ────────────────────────────

if(EMSCRIPTEN)
    return()
endif()

add_executable(arpt_server
    src/main.c
    src/tile_path.c
)

target_include_directories(arpt_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(arpt_server PRIVATE civetweb-c-library)

# ── Tests ──────────────────────────────────────────────────────────────

add_executable(test_server tests/test_server.c src/tile_path.c)
target_include_directories(test_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(test_server PRIVATE unity)
add_test(NAME test_server COMMAND test_server)
