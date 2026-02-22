#ifndef ARPENTRY_TILE_IO_H
#define ARPENTRY_TILE_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Brotli-compress a buffer.
 *
 * @param input       Input data.
 * @param input_size  Size of input in bytes.
 * @param output      Receives malloc'd compressed buffer (caller must free).
 * @param output_size Receives compressed size in bytes.
 * @param quality     Brotli quality level (0-11, higher = smaller but slower).
 * @return true on success.
 */
bool arpt_compress(const uint8_t *input, size_t input_size,
                   uint8_t **output, size_t *output_size, int quality);

/**
 * Brotli-decompress a buffer.
 *
 * Handles unknown output size internally via streaming fallback.
 *
 * @param input       Compressed data.
 * @param input_size  Size of compressed data in bytes.
 * @param output      Receives malloc'd decompressed buffer (caller must free).
 * @param output_size Receives decompressed size in bytes.
 * @return true on success.
 */
bool arpt_decompress(const uint8_t *input, size_t input_size,
                     uint8_t **output, size_t *output_size);

/**
 * Verify that a buffer contains a valid Arpentry Tile.
 *
 * Runs the FlatCC verifier and checks the "arpt" file identifier.
 *
 * @param buf  Buffer containing raw (uncompressed) FlatBuffer data.
 * @param size Size of the buffer in bytes.
 * @return true if the buffer is a valid Tile.
 */
bool arpt_tile_verify(const void *buf, size_t size);

#endif /* ARPENTRY_TILE_IO_H */
