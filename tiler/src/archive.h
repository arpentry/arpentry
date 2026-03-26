/* .arpa archive reader and writer. */

#ifndef ARPT_ARCHIVE_H
#define ARPT_ARCHIVE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ── Writer ─────────────────────────────────────────────────────────── */

typedef struct arpt_archive_writer arpt_archive_writer;

/* Configuration for creating a new archive. */
typedef struct {
    const char *path;         /* Output file path */
    uint8_t     min_zoom;
    uint8_t     max_zoom;
    double      bounds[4];    /* west, south, east, north */
    double      root_error;   /* geometric error for LOD (0 = default) */
} arpt_archive_config;

/* Create a writer with the given configuration. */
arpt_archive_writer *arpt_archive_writer_create(const arpt_archive_config *config);

/* Append a compressed tile blob. Returns false on I/O error. */
bool arpt_archive_writer_add_tile(arpt_archive_writer *w,
                                  uint8_t z, uint32_t x, uint32_t y,
                                  const void *data, size_t size);

/* Set metadata blob (Brotli-compressed .arpi). Caller retains ownership. */
void arpt_archive_writer_set_metadata(arpt_archive_writer *w,
                                      const void *data, size_t size);

/* Finalize: write directory and header. Returns false on error. */
bool arpt_archive_writer_finish(arpt_archive_writer *w);

/* Free the writer. */
void arpt_archive_writer_free(arpt_archive_writer *w);

/* ── Reader ─────────────────────────────────────────────────────────── */

typedef struct arpt_archive_reader arpt_archive_reader;

/* Open an .arpa file via mmap. Returns NULL on error. */
arpt_archive_reader *arpt_archive_reader_open(const char *path);

/* Look up a tile. Returns a pointer into the mmap'd data; size is set.
   Returns NULL if not found. Valid until the reader is closed. */
const void *arpt_archive_reader_get_tile(const arpt_archive_reader *r,
                                         uint8_t z, uint32_t x, uint32_t y,
                                         size_t *size);

/* Query header fields. */
uint64_t arpt_archive_reader_tile_count(const arpt_archive_reader *r);
uint8_t  arpt_archive_reader_min_zoom(const arpt_archive_reader *r);
uint8_t  arpt_archive_reader_max_zoom(const arpt_archive_reader *r);

/* Close the reader and unmap the file. */
void arpt_archive_reader_close(arpt_archive_reader *r);

#endif
