# ── Vendored carquet: Parquet reader (MIT license) ────────────────────
#
# Vendored from https://github.com/Vitruves/carquet (v0.1.3)
# Only the reader is used by the tiler; writer is kept for tests.
# Compression: Snappy and LZ4 are built-in; Brotli and ZSTD use project deps;
# GZIP is stubbed out (returns unsupported).

include(CheckCSourceCompiles)

# Architecture detection
check_c_source_compiles("
    #if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    int main() { return 0; }
    #else
    #error Not x86
    #endif
" CARQUET_ARCH_X86)

check_c_source_compiles("
    #if defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
    int main() { return 0; }
    #else
    #error Not ARM
    #endif
" CARQUET_ARCH_ARM)

# Sources
set(CARQUET_SOURCES
    src/core/arena.c
    src/core/buffer.c
    src/core/bitpack.c
    src/core/endian.c
    src/core/error.c
    src/thrift/thrift_decode.c
    src/thrift/thrift_encode.c
    src/thrift/parquet_types.c
    src/encoding/plain.c
    src/encoding/rle.c
    src/encoding/delta.c
    src/encoding/delta_length.c
    src/encoding/delta_strings.c
    src/encoding/dictionary.c
    src/encoding/byte_stream_split.c
    src/compression/snappy.c
    src/compression/lz4.c
    src/compression/brotli.c
    src/compression/zstd.c
    src/compression/gzip.c
    src/reader/file_reader.c
    src/reader/row_group_reader.c
    src/reader/column_reader.c
    src/reader/page_reader.c
    src/reader/batch_reader.c
    src/reader/statistics.c
    src/reader/mmap_reader.c
    src/writer/file_writer.c
    src/writer/row_group_writer.c
    src/writer/column_writer.c
    src/writer/page_writer.c
    src/metadata/schema.c
    src/metadata/statistics.c
    src/metadata/bloom_filter.c
    src/metadata/page_index.c
    src/util/crc32.c
    src/util/xxhash.c
    src/simd/detect.c
    src/simd/dispatch.c
)

# Platform SIMD sources
if(CARQUET_ARCH_X86)
    list(APPEND CARQUET_SOURCES
        src/simd/x86/sse_ops.c
        src/simd/x86/avx2_ops.c
        src/simd/x86/avx512_ops.c
    )
    set_source_files_properties(src/simd/x86/sse_ops.c PROPERTIES
        COMPILE_FLAGS "-msse4.2")
    set_source_files_properties(src/simd/x86/avx2_ops.c PROPERTIES
        COMPILE_FLAGS "-mavx2 -mbmi2")
    set_source_files_properties(src/simd/x86/avx512_ops.c PROPERTIES
        COMPILE_FLAGS "-mavx512f -mavx512bw -mavx512vl")
endif()

if(CARQUET_ARCH_ARM)
    list(APPEND CARQUET_SOURCES
        src/simd/arm/neon_ops.c
        src/simd/arm/sve_ops.c
        src/simd/arm/crc32_arm.c
    )
    set_source_files_properties(src/simd/arm/crc32_arm.c PROPERTIES
        COMPILE_FLAGS "-march=armv8-a+crc")
endif()

add_library(carquet STATIC ${CARQUET_SOURCES})

target_include_directories(carquet
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Architecture definitions
if(CARQUET_ARCH_X86)
    target_compile_definitions(carquet PRIVATE
        CARQUET_ARCH_X86 CARQUET_ENABLE_SSE CARQUET_ENABLE_AVX2 CARQUET_ENABLE_AVX512)
endif()
if(CARQUET_ARCH_ARM)
    target_compile_definitions(carquet PRIVATE
        CARQUET_ARCH_ARM CARQUET_ENABLE_NEON)
endif()

# Brotli decompression (already fetched in root CMakeLists)
target_link_libraries(carquet PRIVATE brotlidec)

# Zstd compression (already fetched in root CMakeLists)
target_link_libraries(carquet PRIVATE libzstd_static)

# Suppress warnings in vendored code
target_compile_options(carquet PRIVATE -Wno-unused-parameter -Wno-sign-conversion)

# ── Tests ────────────────────────────────────────────────────────────

if(NOT EMSCRIPTEN)
    # Tests that use carquet internal headers
    foreach(_test core thrift encodings compression)
        add_executable(test_carquet_${_test} tests/test_${_test}.c)
        target_link_libraries(test_carquet_${_test} PRIVATE carquet)
        target_include_directories(test_carquet_${_test} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
        add_test(NAME test_carquet_${_test} COMMAND test_carquet_${_test})
    endforeach()

    # Tests that use only the public API
    foreach(_test reader mmap)
        add_executable(test_carquet_${_test} tests/test_${_test}.c)
        target_link_libraries(test_carquet_${_test} PRIVATE carquet)
        add_test(NAME test_carquet_${_test} COMMAND test_carquet_${_test})
    endforeach()
endif()
