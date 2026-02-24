#ifndef TILE_FETCH_H
#define TILE_FETCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Called when a tile fetch completes.
 *
 * On success: flatbuf points to a verified FlatBuffer valid only for
 * the duration of the callback. Copy if you need to keep it.
 * On failure: flatbuf is NULL.
 */
typedef void (*arpt_tile_fetch_cb)(bool success,
                                   const void *flatbuf, size_t size,
                                   void *userdata);

/**
 * Fetch a tile asynchronously from base_url/{level}/{x}/{y}.arpt.
 *
 * The callback fires on the main thread. On Emscripten, uses the Fetch API
 * (browser handles Brotli decompression via Content-Encoding: br).
 * Returns false if the request could not be initiated.
 */
bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata);

#endif /* TILE_FETCH_H */
