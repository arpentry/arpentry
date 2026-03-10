#include "archive.h"

#include <stdlib.h>

struct arpt_archive_writer {
    char *path;
};

struct arpt_archive_reader {
    char *path;
};

/* ── Writer ─────────────────────────────────────────────────────────── */

arpt_archive_writer *arpt_archive_writer_create(const char *path) {
    (void)path;
    return NULL;
}

void arpt_archive_writer_set_zoom(arpt_archive_writer *w,
                                  uint8_t min_zoom, uint8_t max_zoom) {
    (void)w; (void)min_zoom; (void)max_zoom;
}

void arpt_archive_writer_set_bounds(arpt_archive_writer *w,
                                    double west, double south,
                                    double east, double north) {
    (void)w; (void)west; (void)south; (void)east; (void)north;
}

void arpt_archive_writer_set_root_error(arpt_archive_writer *w, double err) {
    (void)w; (void)err;
}

bool arpt_archive_writer_add_tile(arpt_archive_writer *w,
                                  uint8_t z, uint32_t x, uint32_t y,
                                  const void *data, size_t size) {
    (void)w; (void)z; (void)x; (void)y; (void)data; (void)size;
    return false;
}

void arpt_archive_writer_set_metadata(arpt_archive_writer *w,
                                      const void *data, size_t size) {
    (void)w; (void)data; (void)size;
}

bool arpt_archive_writer_finish(arpt_archive_writer *w) {
    (void)w;
    return false;
}

void arpt_archive_writer_free(arpt_archive_writer *w) {
    if (!w) return;
    free(w->path);
    free(w);
}

/* ── Reader ─────────────────────────────────────────────────────────── */

arpt_archive_reader *arpt_archive_reader_open(const char *path) {
    (void)path;
    return NULL;
}

const void *arpt_archive_reader_get_tile(const arpt_archive_reader *r,
                                         uint8_t z, uint32_t x, uint32_t y,
                                         size_t *size) {
    (void)r; (void)z; (void)x; (void)y;
    if (size) *size = 0;
    return NULL;
}

uint64_t arpt_archive_reader_tile_count(const arpt_archive_reader *r) {
    (void)r;
    return 0;
}

void arpt_archive_reader_close(arpt_archive_reader *r) {
    if (!r) return;
    free(r->path);
    free(r);
}
