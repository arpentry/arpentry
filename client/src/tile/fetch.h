#ifndef ARPENTRY_TILE_FETCH_H
#define ARPENTRY_TILE_FETCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Worker-thread hook, called after HTTP + FlatBuffer verification succeed.
 *
 * Runs on a background fetch worker, so this is the place to do CPU-heavy
 * tile processing (decode, triangulation, buffer preparation) without
 * blocking the render loop. flatbuf is malloc'd — ownership transfers in,
 * the callback must free() it before returning. Returns a heap-allocated
 * payload that will be handed to the finish callback on the main thread,
 * or NULL on processing failure (which is reported to finish as !success).
 */
typedef void *(*arpt_tile_prepare_fn)(uint8_t *flatbuf, size_t size,
                                       void *userdata);

/**
 * Main-thread hook, delivered via arpt_fetch_drain.
 *
 * success is true only if HTTP+verify succeeded and (if a prepare callback
 * was provided) prepare returned non-NULL. payload is the prepare return
 * value, or the raw flatbuf if prepare was NULL, or NULL on failure. The
 * callback owns payload and must free it.
 */
typedef void (*arpt_tile_finish_fn)(bool success, void *payload,
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
 * prepare (optional) runs on a worker thread after the HTTP response is
 * verified, so tile decoding and preparation can overlap with HTTP for
 * other tiles and not touch the render thread. Pass NULL to skip the
 * worker-side hook and deliver the raw flatbuf straight to finish.
 *
 * finish runs on the main thread, delivered via arpt_fetch_drain() on
 * native builds or directly on the browser main thread on Emscripten.
 *
 * Returns false if the request could not be initiated.
 */
bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_prepare_fn prepare,
                     arpt_tile_finish_fn finish,
                     void *userdata);

/**
 * Poll for completed fetches and invoke their callbacks on the main thread.
 *
 * `max` caps how many callbacks fire in this call (0 = unlimited); the
 * remaining results stay queued for subsequent drains. This lets the caller
 * bound per-frame tile-processing cost and keep panning smooth when many
 * fetches complete at once.
 *
 * On native builds, drains the result queue. On Emscripten, this is a no-op.
 * Returns the number of callbacks invoked.
 */
int arpt_fetch_drain(int max);

/**
 * Shut down the fetch subsystem.
 *
 * On native builds, signals workers to exit, joins threads, and frees queues.
 * On Emscripten, this is a no-op.
 */
void arpt_fetch_shutdown(void);

#endif /* ARPENTRY_TILE_FETCH_H */
