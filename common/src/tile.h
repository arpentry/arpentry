#ifndef ARPENTRY_TILE_H
#define ARPENTRY_TILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Encode a raw FlatBuffer tile into the .arpt wire format.
 *
 * Brotli-compresses the buffer at the given quality level.
 *
 * @param buf      Raw (uncompressed) FlatBuffer data.
 * @param size     Size of the FlatBuffer in bytes.
 * @param out      Receives malloc'd compressed buffer (caller must free).
 * @param out_size Receives compressed size in bytes.
 * @param quality  Brotli quality level (0-11, higher = smaller but slower).
 * @return true on success.
 */
bool arpt_encode(const void *buf, size_t size,
                 uint8_t **out, size_t *out_size, int quality);

/**
 * Decode .arpt wire bytes into a verified FlatBuffer.
 *
 * Decompresses and verifies the "arpt" file identifier in one step.
 * On success the returned buffer is guaranteed valid for FlatCC readers.
 *
 * @param data     Compressed .arpt data.
 * @param size     Size of compressed data in bytes.
 * @param out      Receives malloc'd FlatBuffer (caller must free).
 * @param out_size Receives decompressed size in bytes.
 * @return true on success (decompression and verification both passed).
 */
bool arpt_decode(const uint8_t *data, size_t size,
                 uint8_t **out, size_t *out_size);

#endif /* ARPENTRY_TILE_H */
