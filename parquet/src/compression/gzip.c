/* GZIP/DEFLATE compression — stub, returns unsupported. */

#include <carquet/error.h>
#include <stdint.h>
#include <stddef.h>

int carquet_gzip_decompress(
    const uint8_t *src, size_t src_size,
    uint8_t *dst, size_t dst_capacity, size_t *dst_size)
{
    (void)src; (void)src_size; (void)dst; (void)dst_capacity; (void)dst_size;
    return CARQUET_ERROR_UNSUPPORTED_CODEC;
}

int carquet_gzip_compress(
    const uint8_t *src, size_t src_size,
    uint8_t *dst, size_t dst_capacity, size_t *dst_size, int level)
{
    (void)src; (void)src_size; (void)dst; (void)dst_capacity; (void)dst_size; (void)level;
    return CARQUET_ERROR_UNSUPPORTED_CODEC;
}

size_t carquet_gzip_compress_bound(size_t src_size)
{
    (void)src_size;
    return 0;
}

void carquet_gzip_init_tables(void) {}
