#ifndef ARPENTRY_TILE_ARCHIVE_H
#define ARPENTRY_TILE_ARCHIVE_H

#ifndef __EMSCRIPTEN__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * A read-only `.arpa` tile archive (docs/TILER.md §2), mapped from disk.
 *
 * The archive is what the tiler emits and what the server serves out of; a
 * client that can read one directly needs no server, no port and no network
 * to draw a scene. Blobs are returned exactly as stored — still Brotli-
 * compressed — because the caller already has to decompress a server's
 * response and there is no reason to have two of that.
 */
typedef struct arpt_archive arpt_archive;

/** Map an archive. Returns NULL if it is missing, truncated or not a `.arpa`. */
arpt_archive *arpt_archive_open(const char *path);

void arpt_archive_free(arpt_archive *a);

/**
 * Locate a tile's stored blob, or return false if the archive has no tile at
 * that address — which is ordinary: an archive is sparse, and a miss is the
 * same answer as a 404.
 */
bool arpt_archive_tile(const arpt_archive *a, int level, int x, int y,
                       const uint8_t **blob, size_t *size);

/**
 * The archive's own metadata blob: the `.arpi` tileset the tiler wrote. This
 * is the authoritative description of what the archive holds, so a client
 * reading it locally sees the same tileset a server would have derived.
 */
bool arpt_archive_metadata(const arpt_archive *a, const uint8_t **blob,
                           size_t *size);

/** Number of tiles in the directory. */
uint64_t arpt_archive_tile_count(const arpt_archive *a);

/**
 * Zoom-prefixed Hilbert tile id (docs/TILER.md §1) — the archive directory's
 * sort key. Exposed for the tests that hold it to the tiler's ordering.
 */
uint64_t arpt_archive_tile_id(int level, int x, int y);

#endif /* !__EMSCRIPTEN__ */
#endif /* ARPENTRY_TILE_ARCHIVE_H */
