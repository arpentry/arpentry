#include "arpentry/tile_io.h"

#include <stdlib.h>
#include <string.h>
#include <brotli/encode.h>
#include <brotli/decode.h>

#include "arpentry_tiles_verifier.h"

/* ── Compress ───────────────────────────────────────────────────────────── */

bool arpt_compress(const uint8_t *input, size_t input_size,
                   uint8_t **output, size_t *output_size, int quality) {
    if (!input || !output || !output_size) return false;

    size_t max_size = BrotliEncoderMaxCompressedSize(input_size);
    if (max_size == 0) max_size = input_size + 64;

    uint8_t *buf = malloc(max_size);
    if (!buf) return false;

    size_t encoded_size = max_size;
    BROTLI_BOOL ok = BrotliEncoderCompress(
        quality, BROTLI_DEFAULT_WINDOW, BROTLI_DEFAULT_MODE,
        input_size, input, &encoded_size, buf);

    if (!ok) {
        free(buf);
        return false;
    }

    *output = buf;
    *output_size = encoded_size;
    return true;
}

/* ── Decompress ─────────────────────────────────────────────────────────── */

bool arpt_decompress(const uint8_t *input, size_t input_size,
                     uint8_t **output, size_t *output_size) {
    if (!input || !output || !output_size) return false;

    BrotliDecoderState *state = BrotliDecoderCreateInstance(NULL, NULL, NULL);
    if (!state) return false;

    size_t capacity = input_size * 4;
    if (capacity < 256) capacity = 256;
    uint8_t *buf = malloc(capacity);
    if (!buf) {
        BrotliDecoderDestroyInstance(state);
        return false;
    }

    size_t available_in  = input_size;
    const uint8_t *next_in = input;
    size_t available_out = capacity;
    uint8_t *next_out    = buf;
    size_t total_out     = 0;

    BrotliDecoderResult result;
    for (;;) {
        result = BrotliDecoderDecompressStream(
            state, &available_in, &next_in, &available_out, &next_out, &total_out);

        if (result == BROTLI_DECODER_RESULT_SUCCESS) {
            break;
        }
        if (result == BROTLI_DECODER_RESULT_NEEDS_MORE_OUTPUT) {
            size_t new_capacity = capacity * 2;
            uint8_t *new_buf = realloc(buf, new_capacity);
            if (!new_buf) {
                free(buf);
                BrotliDecoderDestroyInstance(state);
                return false;
            }
            buf = new_buf;
            available_out = new_capacity - total_out;
            next_out = buf + total_out;
            capacity = new_capacity;
            continue;
        }
        /* BROTLI_DECODER_RESULT_ERROR or NEEDS_MORE_INPUT with no input left */
        free(buf);
        BrotliDecoderDestroyInstance(state);
        return false;
    }

    BrotliDecoderDestroyInstance(state);

    *output = buf;
    *output_size = total_out;
    return true;
}

/* ── Verify ─────────────────────────────────────────────────────────────── */

bool arpt_tile_verify(const void *buf, size_t size) {
    if (!buf || size < 8) return false;

    int ret = arpentry_tiles_Tile_verify_as_root_with_identifier(buf, size, "arpt");
    return ret == 0;
}
