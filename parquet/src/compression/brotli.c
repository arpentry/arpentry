/* Brotli decompression for Parquet pages. */

#include <carquet/error.h>
#include <brotli/decode.h>
#include <stdint.h>
#include <stddef.h>

int carquet_brotli_decompress(
    const uint8_t *src, size_t src_size,
    uint8_t *dst, size_t dst_capacity, size_t *dst_size)
{
    if (!src || !dst || !dst_size)
        return CARQUET_ERROR_INVALID_ARGUMENT;

    size_t available = dst_capacity;
    BrotliDecoderResult result = BrotliDecoderDecompress(
        src_size, src, &available, dst);

    if (result != BROTLI_DECODER_RESULT_SUCCESS)
        return CARQUET_ERROR_DECOMPRESSION;

    *dst_size = available;
    return CARQUET_OK;
}
