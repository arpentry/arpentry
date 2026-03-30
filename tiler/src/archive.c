#include "archive.h"
#include "hilbert.h"

#include <errno.h>
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
    uint8_t  reserved[40];
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

arpt_archive_writer *arpt_archive_writer_create(const arpt_archive_config *config) {
    if (!config || !config->path) return NULL;
    arpt_archive_writer *w = calloc(1, sizeof(*w));
    if (!w) return NULL;
    w->path = strdup(config->path);
    if (!w->path) { free(w); return NULL; }

    w->min_zoom = config->min_zoom;
    w->max_zoom = config->max_zoom;
    memcpy(w->bounds, config->bounds, sizeof(w->bounds));
    w->root_error = config->root_error;

    w->fp = fopen(config->path, "wb");
    if (!w->fp) goto fail;

    /* Reserve header space */
    uint8_t zeros[HEADER_SIZE] = {0};
    if (fwrite(zeros, 1, HEADER_SIZE, w->fp) != HEADER_SIZE) goto fail;

    /* Temp file for directory entries */
    char dir_tmpl[512];
    snprintf(dir_tmpl, sizeof(dir_tmpl), "%s.dir_XXXXXX", config->path);
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

bool arpt_archive_writer_add_tile(arpt_archive_writer *w,
                                  uint8_t z, uint32_t x, uint32_t y,
                                  const void *data, size_t size) {
    if (!w || !w->fp || !data || size == 0) return false;

    uint64_t offset = (uint64_t)ftell(w->fp);
    if (fwrite(data, 1, size, w->fp) != size) {
        fprintf(stderr, "archive: failed to write tile z%u/%u/%u (%zu bytes) at offset %llu: %s\n",
                z, x, y, size, (unsigned long long)offset, strerror(errno));
        return false;
    }

    arpa_dir_entry e = {0};
    e.hilbert_id = arpt_hilbert_tile_id(z, (int)x, (int)y);
    e.offset = offset;
    e.size = size;
    e.z = z;
    e.x = x;
    e.y = y;
    if (fwrite(&e, DIR_ENTRY_SIZE, 1, w->dir_fp) != 1) {
        fprintf(stderr, "archive: failed to write dir entry for z%u/%u/%u: %s\n",
                z, x, y, strerror(errno));
        return false;
    }
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
   Caller frees the returned pointer. Returns NULL on error and sets *ok
   to false. When tile_count == 0, returns NULL with *ok == true. */
static arpa_dir_entry *load_and_sort_dir(arpt_archive_writer *w, bool *ok) {
    *ok = true;
    if (w->tile_count == 0) return NULL;

    size_t n = (size_t)w->tile_count;
    size_t alloc_bytes = n * DIR_ENTRY_SIZE;
    arpa_dir_entry *dir = malloc(alloc_bytes);
    if (!dir) {
        fprintf(stderr, "archive: failed to allocate directory (%llu entries, %.1f MB)\n",
                (unsigned long long)n, (double)alloc_bytes / (1024.0 * 1024.0));
        *ok = false;
        return NULL;
    }

    if (fflush(w->dir_fp) != 0) {
        fprintf(stderr, "archive: fflush dir temp file failed: %s\n", strerror(errno));
        free(dir);
        *ok = false;
        return NULL;
    }
    if (fseek(w->dir_fp, 0, SEEK_SET) != 0) {
        fprintf(stderr, "archive: fseek dir temp file failed: %s\n", strerror(errno));
        free(dir);
        *ok = false;
        return NULL;
    }
    size_t nread = fread(dir, DIR_ENTRY_SIZE, n, w->dir_fp);
    if (nread != n) {
        fprintf(stderr, "archive: fread dir temp file: read %llu of %llu entries: %s\n",
                (unsigned long long)nread, (unsigned long long)n,
                feof(w->dir_fp) ? "unexpected EOF" : strerror(errno));
        free(dir);
        *ok = false;
        return NULL;
    }

    qsort(dir, n, DIR_ENTRY_SIZE, cmp_dir_entry);
    return dir;
}

bool arpt_archive_writer_finish(arpt_archive_writer *w) {
    if (!w || !w->fp) return false;

    fprintf(stderr, "archive: finishing, %llu tiles\n",
            (unsigned long long)w->tile_count);

    /* Sort directory */
    bool dir_ok;
    arpa_dir_entry *dir = load_and_sort_dir(w, &dir_ok);
    if (!dir_ok) {
        fprintf(stderr, "archive: load_and_sort_dir failed\n");
        return false;
    }

    /* Pad to 8-byte alignment before directory for safe struct access */
    uint64_t pos = (uint64_t)ftell(w->fp);
    uint64_t pad_bytes = (8 - (pos & 7)) & 7;
    if (pad_bytes > 0) {
        uint8_t zeros[8] = {0};
        if (fwrite(zeros, 1, (size_t)pad_bytes, w->fp) != (size_t)pad_bytes) {
            fprintf(stderr, "archive: failed to write alignment padding: %s\n",
                    strerror(errno));
            free(dir);
            return false;
        }
    }

    /* Append sorted directory to archive */
    uint64_t dir_offset = (uint64_t)ftell(w->fp);
    if (dir && w->tile_count > 0) {
        size_t written = fwrite(dir, DIR_ENTRY_SIZE, (size_t)w->tile_count, w->fp);
        if (written != (size_t)w->tile_count) {
            fprintf(stderr, "archive: failed to write directory: wrote %llu of %llu entries: %s\n",
                    (unsigned long long)written, (unsigned long long)w->tile_count,
                    strerror(errno));
            free(dir);
            return false;
        }
    }
    free(dir);

    /* Append metadata */
    uint64_t meta_offset = (uint64_t)ftell(w->fp);
    if (w->meta && w->meta_size > 0) {
        if (fwrite(w->meta, 1, w->meta_size, w->fp) != w->meta_size) {
            fprintf(stderr, "archive: failed to write metadata: %s\n",
                    strerror(errno));
            return false;
        }
    }

    /* Flush before seeking back to write the header */
    if (fflush(w->fp) != 0) {
        fprintf(stderr, "archive: fflush failed before header write: %s\n",
                strerror(errno));
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

    if (fseek(w->fp, 0, SEEK_SET) != 0) {
        fprintf(stderr, "archive: fseek to header failed: %s\n",
                strerror(errno));
        return false;
    }
    if (fwrite(&hdr, HEADER_SIZE, 1, w->fp) != 1) {
        fprintf(stderr, "archive: failed to write header: %s\n",
                strerror(errno));
        return false;
    }

    fclose(w->fp);
    w->fp = NULL;

    /* Clean up temp file */
    if (w->dir_fp) { fclose(w->dir_fp); w->dir_fp = NULL; }
    if (w->dir_path) { remove(w->dir_path); }

    fprintf(stderr, "archive: finalized, dir at offset %llu (%.1f MB)\n",
            (unsigned long long)dir_offset,
            (double)(w->tile_count * DIR_ENTRY_SIZE) / (1024.0 * 1024.0));

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

uint8_t arpt_archive_reader_min_zoom(const arpt_archive_reader *r) {
    if (!r) return 0;
    return r->hdr.min_zoom;
}

uint8_t arpt_archive_reader_max_zoom(const arpt_archive_reader *r) {
    if (!r) return 0;
    return r->hdr.max_zoom;
}

void arpt_archive_reader_close(arpt_archive_reader *r) {
    if (!r) return;
    if (r->map) munmap(r->map, r->map_size);
    free(r);
}
