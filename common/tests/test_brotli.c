#include "unity.h"

#include <string.h>
#include <stdlib.h>
#include <brotli/encode.h>
#include <brotli/decode.h>

void setUp(void) {}
void tearDown(void) {}

void test_brotli_roundtrip(void) {
    const char *input = "Hello, Brotli! "
                        "This string will be compressed and decompressed.";
    size_t input_size = strlen(input);

    /* Compress */
    size_t compressed_size = BrotliEncoderMaxCompressedSize(input_size);
    uint8_t *compressed = malloc(compressed_size);
    TEST_ASSERT_NOT_NULL(compressed);

    BROTLI_BOOL ok = BrotliEncoderCompress(
        BROTLI_DEFAULT_QUALITY, BROTLI_DEFAULT_WINDOW, BROTLI_DEFAULT_MODE,
        input_size, (const uint8_t *)input, &compressed_size, compressed);
    TEST_ASSERT_EQUAL(BROTLI_TRUE, ok);

    /* Decompress */
    size_t decompressed_size = input_size * 2;
    uint8_t *decompressed = malloc(decompressed_size);
    TEST_ASSERT_NOT_NULL(decompressed);

    BrotliDecoderResult result = BrotliDecoderDecompress(
        compressed_size, compressed, &decompressed_size, decompressed);
    TEST_ASSERT_EQUAL(BROTLI_DECODER_RESULT_SUCCESS, result);

    /* Verify */
    TEST_ASSERT_EQUAL(input_size, decompressed_size);
    TEST_ASSERT_EQUAL_MEMORY(input, decompressed, input_size);

    free(compressed);
    free(decompressed);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_brotli_roundtrip);
    return UNITY_END();
}
