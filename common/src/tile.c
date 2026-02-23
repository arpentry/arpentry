#include "tile.h"

#include <stdlib.h>
#include <string.h>
#include <brotli/encode.h>
#include <brotli/decode.h>

#include "tile_verifier.h"

/* ── Internals ─────────────────────────────────────────────────────────── */

static bool compress(const uint8_t *input, size_t input_size,
                     uint8_t **output, size_t *output_size, int quality) {
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

static bool decompress(const uint8_t *input, size_t input_size,
                       uint8_t **output, size_t *output_size) {
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
        free(buf);
        BrotliDecoderDestroyInstance(state);
        return false;
    }

    BrotliDecoderDestroyInstance(state);

    /* Shrink to exact size */
    uint8_t *shrunk = realloc(buf, total_out);
    *output = shrunk ? shrunk : buf;
    *output_size = total_out;
    return true;
}

static bool verify(const void *buf, size_t size) {
    if (!buf || size < 8) return false;
    return arpentry_tiles_Tile_verify_as_root_with_identifier(buf, size, "arpt") == 0;
}

/* ── Public API ────────────────────────────────────────────────────────── */

bool arpt_encode(const void *buf, size_t size,
                 uint8_t **out, size_t *out_size, int quality) {
    if (!buf || !out || !out_size) return false;
    return compress((const uint8_t *)buf, size, out, out_size, quality);
}

bool arpt_decode(const uint8_t *data, size_t size,
                 uint8_t **out, size_t *out_size) {
    if (!data || !out || !out_size) return false;

    uint8_t *buf = NULL;
    size_t buf_size = 0;
    if (!decompress(data, size, &buf, &buf_size)) return false;

    if (!verify(buf, buf_size)) {
        free(buf);
        return false;
    }

    *out = buf;
    *out_size = buf_size;
    return true;
}
