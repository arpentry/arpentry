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
