#ifndef ARPENTRY_TILE_FETCH_H
#define ARPENTRY_TILE_FETCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Called when a tile fetch completes.
 *
 * On success: flatbuf is a malloc'd verified FlatBuffer — caller owns it
 * and must free() it after use.
 * On failure: flatbuf is NULL.
 */
typedef void (*arpt_tile_fetch_cb)(bool success, uint8_t *flatbuf, size_t size,
                                   void *userdata);

/**
 * Initialize the fetch subsystem.
 *
 * On native builds, creates a thread pool with max_concurrent worker threads.
 * On Emscripten, this is a no-op (the browser Fetch API is inherently async).
 * Must be called before arpt_fetch_tile().
 */
bool arpt_fetch_init(int max_concurrent);

/**
 * Fetch a tile asynchronously from base_url/{level}/{x}/{y}.arpt.
 *
 * The callback fires on the main thread. On native builds the callback is
 * delivered via arpt_fetch_drain(). On Emscripten, the browser calls back
 * directly on the main thread.
 *
 * Returns false if the request could not be initiated.
 */
bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata);

/**
 * Poll for completed fetches and invoke their callbacks on the main thread.
 *
 * On native builds, drains the result queue. On Emscripten, this is a no-op.
 * Returns the number of callbacks invoked.
 */
int arpt_fetch_drain(void);

/**
 * Shut down the fetch subsystem.
 *
 * On native builds, signals workers to exit, joins threads, and frees queues.
 * On Emscripten, this is a no-op.
 */
void arpt_fetch_shutdown(void);

#endif /* ARPENTRY_TILE_FETCH_H */
