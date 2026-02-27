#ifndef ARPENTRY_TILE_H
#define ARPENTRY_TILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Brotli-compress a raw FlatBuffer into .arpt wire format. Caller frees *out.
 */
bool arpt_encode(const void *buf, size_t size, uint8_t **out, size_t *out_size,
                 int quality);

/* Decompress and verify an .arpt tile. Caller frees *out. */
bool arpt_decode(const uint8_t *data, size_t size, uint8_t **out,
                 size_t *out_size);

#endif /* ARPENTRY_TILE_H */
