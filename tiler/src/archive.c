#include "archive.h"
#include "hilbert.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define ARPA_MAGIC     0x61727061u  /* "arpa" */
#define ARPA_VERSION   1u
#define HEADER_SIZE    128
#define DIR_ENTRY_SIZE 40

/* ── Header layout (128 bytes) ─────────────────────────────────── */

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t  min_zoom;
    uint8_t  max_zoom;
    uint8_t  pad1[6];
    double   bounds[4];
    double   root_error;
    uint64_t tile_count;
    uint64_t dir_offset;
    uint64_t meta_offset;
    uint64_t meta_size;
    uint8_t  reserved[16];
} arpa_header;

/* ── Directory entry (40 bytes) ────────────────────────────────── */

typedef struct {
    uint64_t hilbert_id;
    uint64_t offset;
    uint64_t size;
    uint8_t  z;
    uint8_t  pad[3];
    uint32_t x;
    uint32_t y;
} arpa_dir_entry;

/* ── Writer ────────────────────────────────────────────────────── */

/*
 * Directory entries are streamed to a temporary file so we don't
 * accumulate them in memory. On finish(), we mmap the temp file,
 * qsort by hilbert_id, and append the sorted directory to the archive.
 */

struct arpt_archive_writer {
    char *path;
    FILE *fp;

    char *dir_path;
    FILE *dir_fp;

    uint8_t min_zoom, max_zoom;
    double  bounds[4];
    double  root_error;
    uint64_t tile_count;

    uint8_t *meta;
    size_t   meta_size;
};

arpt_archive_writer *arpt_archive_writer_create(const char *path) {
    if (!path) return NULL;
    arpt_archive_writer *w = calloc(1, sizeof(*w));
    if (!w) return NULL;
    w->path = strdup(path);
    if (!w->path) { free(w); return NULL; }

    w->fp = fopen(path, "wb");
    if (!w->fp) goto fail;

    /* Reserve header space */
    uint8_t zeros[HEADER_SIZE] = {0};
    if (fwrite(zeros, 1, HEADER_SIZE, w->fp) != HEADER_SIZE) goto fail;

    /* Temp file for directory entries */
    char dir_tmpl[512];
    snprintf(dir_tmpl, sizeof(dir_tmpl), "%s.dir_XXXXXX", path);
    int dir_fd = mkstemp(dir_tmpl);
    if (dir_fd < 0) goto fail;
    w->dir_fp = fdopen(dir_fd, "w+b");
    if (!w->dir_fp) { close(dir_fd); goto fail; }
    w->dir_path = strdup(dir_tmpl);
    if (!w->dir_path) goto fail;

    return w;

fail:
    if (w->fp) fclose(w->fp);
    if (w->dir_fp) fclose(w->dir_fp);
    free(w->dir_path);
    free(w->path);
    free(w);
    return NULL;
}

void arpt_archive_writer_set_zoom(arpt_archive_writer *w,
                                  uint8_t min_zoom, uint8_t max_zoom) {
    if (!w) return;
    w->min_zoom = min_zoom;
    w->max_zoom = max_zoom;
}

void arpt_archive_writer_set_bounds(arpt_archive_writer *w,
                                    double west, double south,
                                    double east, double north) {
    if (!w) return;
    w->bounds[0] = west;
    w->bounds[1] = south;
    w->bounds[2] = east;
    w->bounds[3] = north;
}

void arpt_archive_writer_set_root_error(arpt_archive_writer *w, double err) {
    if (!w) return;
    w->root_error = err;
}

bool arpt_archive_writer_add_tile(arpt_archive_writer *w,
                                  uint8_t z, uint32_t x, uint32_t y,
                                  const void *data, size_t size) {
    if (!w || !w->fp || !data || size == 0) return false;

    uint64_t offset = (uint64_t)ftell(w->fp);
    if (fwrite(data, 1, size, w->fp) != size) return false;

    arpa_dir_entry e = {0};
    e.hilbert_id = arpt_hilbert_tile_id(z, (int)x, (int)y);
    e.offset = offset;
    e.size = size;
    e.z = z;
    e.x = x;
    e.y = y;
    if (fwrite(&e, DIR_ENTRY_SIZE, 1, w->dir_fp) != 1) return false;
    w->tile_count++;
    return true;
}

void arpt_archive_writer_set_metadata(arpt_archive_writer *w,
                                      const void *data, size_t size) {
    if (!w) return;
    free(w->meta);
    w->meta = NULL;
    w->meta_size = 0;
    if (data && size > 0) {
        w->meta = malloc(size);
        if (w->meta) {
            memcpy(w->meta, data, size);
            w->meta_size = size;
        }
    }
}

static int cmp_dir_entry(const void *a, const void *b) {
    uint64_t ka = ((const arpa_dir_entry *)a)->hilbert_id;
    uint64_t kb = ((const arpa_dir_entry *)b)->hilbert_id;
    if (ka < kb) return -1;
    if (ka > kb) return 1;
    return 0;
}

/* Read directory from temp file into a malloc'd array, sort, return it.
   Caller frees the returned pointer. */
static arpa_dir_entry *load_and_sort_dir(arpt_archive_writer *w) {
    if (w->tile_count == 0) return NULL;

    size_t n = (size_t)w->tile_count;
    arpa_dir_entry *dir = malloc(n * DIR_ENTRY_SIZE);
    if (!dir) return NULL;

    fflush(w->dir_fp);
    fseek(w->dir_fp, 0, SEEK_SET);
    if (fread(dir, DIR_ENTRY_SIZE, n, w->dir_fp) != n) {
        free(dir);
        return NULL;
    }

    qsort(dir, n, DIR_ENTRY_SIZE, cmp_dir_entry);
    return dir;
}

bool arpt_archive_writer_finish(arpt_archive_writer *w) {
    if (!w || !w->fp) return false;

    /* Sort directory */
    arpa_dir_entry *dir = load_and_sort_dir(w);
    /* dir is NULL if tile_count == 0, which is fine */

    /* Append sorted directory to archive */
    uint64_t dir_offset = (uint64_t)ftell(w->fp);
    for (uint64_t i = 0; i < w->tile_count; i++) {
        if (fwrite(&dir[i], DIR_ENTRY_SIZE, 1, w->fp) != 1) {
            free(dir);
            return false;
        }
    }
    free(dir);

    /* Append metadata */
    uint64_t meta_offset = (uint64_t)ftell(w->fp);
    if (w->meta && w->meta_size > 0) {
        if (fwrite(w->meta, 1, w->meta_size, w->fp) != w->meta_size)
            return false;
    }

    /* Write header at offset 0 */
    arpa_header hdr = {0};
    hdr.magic = ARPA_MAGIC;
    hdr.version = ARPA_VERSION;
    hdr.min_zoom = w->min_zoom;
    hdr.max_zoom = w->max_zoom;
    memcpy(hdr.bounds, w->bounds, sizeof(hdr.bounds));
    hdr.root_error = w->root_error;
    hdr.tile_count = w->tile_count;
    hdr.dir_offset = dir_offset;
    hdr.meta_offset = meta_offset;
    hdr.meta_size = w->meta_size;

    fseek(w->fp, 0, SEEK_SET);
    if (fwrite(&hdr, HEADER_SIZE, 1, w->fp) != 1) return false;

    fclose(w->fp);
    w->fp = NULL;

    /* Clean up temp file */
    if (w->dir_fp) { fclose(w->dir_fp); w->dir_fp = NULL; }
    if (w->dir_path) { remove(w->dir_path); }

    return true;
}

void arpt_archive_writer_free(arpt_archive_writer *w) {
    if (!w) return;
    if (w->fp) fclose(w->fp);
    if (w->dir_fp) fclose(w->dir_fp);
    if (w->dir_path) { remove(w->dir_path); free(w->dir_path); }
    free(w->meta);
    free(w->path);
    free(w);
}

/* ── Reader ────────────────────────────────────────────────────── */

struct arpt_archive_reader {
    uint8_t  *map;
    size_t    map_size;
    arpa_header hdr;
    const arpa_dir_entry *dir;
};

arpt_archive_reader *arpt_archive_reader_open(const char *path) {
    if (!path) return NULL;

    int fd = open(path, O_RDONLY);
    if (fd < 0) return NULL;

    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < HEADER_SIZE) {
        close(fd);
        return NULL;
    }

    uint8_t *map = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if (map == MAP_FAILED) return NULL;

    arpa_header hdr;
    memcpy(&hdr, map, sizeof(hdr));
    if (hdr.magic != ARPA_MAGIC || hdr.version != ARPA_VERSION) {
        munmap(map, (size_t)st.st_size);
        return NULL;
    }

    arpt_archive_reader *r = calloc(1, sizeof(*r));
    if (!r) { munmap(map, (size_t)st.st_size); return NULL; }

    r->map = map;
    r->map_size = (size_t)st.st_size;
    r->hdr = hdr;
    r->dir = (const arpa_dir_entry *)(map + hdr.dir_offset);
    return r;
}

const void *arpt_archive_reader_get_tile(const arpt_archive_reader *r,
                                         uint8_t z, uint32_t x, uint32_t y,
                                         size_t *size) {
    if (!r || r->hdr.tile_count == 0) {
        if (size) *size = 0;
        return NULL;
    }

    uint64_t target = arpt_hilbert_tile_id(z, (int)x, (int)y);

    uint64_t lo = 0, hi = r->hdr.tile_count;
    while (lo < hi) {
        uint64_t mid = lo + (hi - lo) / 2;
        if (r->dir[mid].hilbert_id < target)
            lo = mid + 1;
        else
            hi = mid;
    }

    if (lo < r->hdr.tile_count && r->dir[lo].hilbert_id == target) {
        if (size) *size = (size_t)r->dir[lo].size;
        return r->map + r->dir[lo].offset;
    }

    if (size) *size = 0;
    return NULL;
}

uint64_t arpt_archive_reader_tile_count(const arpt_archive_reader *r) {
    if (!r) return 0;
    return r->hdr.tile_count;
}

void arpt_archive_reader_close(arpt_archive_reader *r) {
    if (!r) return;
    if (r->map) munmap(r->map, r->map_size);
    free(r);
}
