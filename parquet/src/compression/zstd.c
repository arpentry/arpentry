/* ZSTD compression for Parquet pages. */

#include <carquet/error.h>
#include <zstd.h>
#include <stdint.h>
#include <stddef.h>

int carquet_zstd_decompress(
    const uint8_t *src, size_t src_size,
    uint8_t *dst, size_t dst_capacity, size_t *dst_size)
{
    if (!dst || !dst_size)
        return CARQUET_ERROR_INVALID_ARGUMENT;

    if (src_size == 0) {
        *dst_size = 0;
        return CARQUET_OK;
    }

    if (!src)
        return CARQUET_ERROR_INVALID_ARGUMENT;

    size_t result = ZSTD_decompress(dst, dst_capacity, src, src_size);
    if (ZSTD_isError(result))
        return CARQUET_ERROR_DECOMPRESSION;

    *dst_size = result;
    return CARQUET_OK;
}

int carquet_zstd_compress(
    const uint8_t *src, size_t src_size,
    uint8_t *dst, size_t dst_capacity, size_t *dst_size, int level)
{
    if (!dst || !dst_size)
        return CARQUET_ERROR_INVALID_ARGUMENT;

    if (src_size == 0) {
        *dst_size = 0;
        return CARQUET_OK;
    }

    if (!src)
        return CARQUET_ERROR_INVALID_ARGUMENT;

    if (level <= 0) level = 3;  /* ZSTD default */

    size_t result = ZSTD_compress(dst, dst_capacity, src, src_size, level);
    if (ZSTD_isError(result))
        return CARQUET_ERROR_COMPRESSION;

    *dst_size = result;
    return CARQUET_OK;
}

size_t carquet_zstd_compress_bound(size_t src_size)
{
    return ZSTD_compressBound(src_size);
}

void carquet_zstd_init_tables(void) {}
