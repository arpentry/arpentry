#ifndef ARPENTRY_TILE_SOURCE_H
#define ARPENTRY_TILE_SOURCE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Where the viewer's bytes come from.
 *
 * The client reads four things — tiles, the tileset, the style and the model
 * library — and it should not care whether they arrive over a socket or off a
 * local disk. One locator names either: a base URL, or a path to a `.arpa`.
 * Everything downstream asks for a name (`/12/34/56.arpt`, `/index.arpi`) and
 * gets bytes, decompressed.
 *
 * Reading an archive directly is what lets a headless capture run with no
 * server, no port to bind and no failed-tile retries — see docs/VIEWER.md
 * "Headless capture".
 */

/**
 * Point the source at a local `.arpa`. Pass NULL (or never call this) to keep
 * every request on HTTP. Returns false if the archive cannot be read, which
 * the caller should treat as fatal: silently falling back to the network
 * would answer a question nobody asked.
 */
bool arpt_source_open_archive(const char *path);

void arpt_source_close(void);

/**
 * Whether the open archive's file has changed on disk since it was opened.
 *
 * A stat, no side effect, cheap enough to ask every frame. False when no
 * archive is open, so a served viewer never pays for it.
 *
 * Split from [`arpt_source_reload_archive`] deliberately: asking is always
 * safe, acting is not.
 */
bool arpt_source_archive_changed(void);

/**
 * Reopen the archive, picking up whatever the tiler last wrote.
 *
 * **The caller must guarantee no fetch is in flight.** Workers read the mmap
 * without a lock — that is the whole reason the archive is documented as
 * immutable — so unmapping it under a worker is a use-after-free, not a stale
 * read. `arpt_tile_manager_active_fetches() == 0` is the condition, and the
 * tiler's write-to-temp-then-rename is what makes the old mapping stay valid
 * for as long as anyone still holds it.
 *
 * Returns false and keeps the current archive if the new file cannot be read:
 * a tiler halfway through a run should not blank the viewer.
 */
bool arpt_source_reload_archive(void);

/** True once an archive is open — i.e. no server is involved. */
bool arpt_source_is_archive(void);

/**
 * Fetch one named blob, decompressed. `base` is the HTTP base URL, ignored
 * when an archive is open. `name` is server-relative and begins with '/'.
 *
 * A missing tile is a false return, not an error: archives are sparse and so
 * are servers. Caller frees *body.
 */
bool arpt_source_get(const char *base, const char *name, uint8_t **body,
                     size_t *size);

#endif /* ARPENTRY_TILE_SOURCE_H */
