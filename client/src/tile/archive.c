#include "archive.h"

#ifndef __EMSCRIPTEN__

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

/* Header layout (docs/TILER.md §2). Must match the writer in
   server/src/archive.rs — the offsets are the format. */
#define ARCHIVE_MAGIC     0x61727061u /* "arpa" little-endian */
#define ARCHIVE_VERSION   1u
#define HEADER_SIZE       128u
#define DIR_ENTRY_SIZE    40u

#define H_MAGIC        0
#define H_VERSION      4
#define H_TILE_COUNT   56
#define H_DIR_OFFSET   64
#define H_META_OFFSET  72
#define H_META_SIZE    80

/* A directory entry's Hilbert id and blob range. */
#define E_HILBERT_ID   0
#define E_OFFSET       8
#define E_SIZE         16

/* Bits reserved for the Hilbert distance within a tile id; the zoom occupies
   the bits above it. */
#define HILBERT_BITS   42
#define HILBERT_MASK   ((UINT64_C(1) << HILBERT_BITS) - 1)

struct arpt_archive {
    const uint8_t *data;
    size_t size;
    uint64_t tile_count;
    uint64_t dir_offset;
    uint64_t meta_offset;
    uint64_t meta_size;
};

static uint32_t rd_u32(const uint8_t *d, size_t o) {
    uint32_t v;
    memcpy(&v, d + o, sizeof(v));
    return v;
}

static uint64_t rd_u64(const uint8_t *d, size_t o) {
    uint64_t v;
    memcpy(&v, d + o, sizeof(v));
    return v;
}

/* Rotate/reflect a quadrant so the Hilbert curve stays continuous. */
static void rot(uint32_t n, uint32_t *x, uint32_t *y, uint32_t rx,
                uint32_t ry) {
    if (ry != 0) return;
    if (rx == 1) {
        *x = n - 1 - *x;
        *y = n - 1 - *y;
    }
    uint32_t t = *x;
    *x = *y;
    *y = t;
}

/* Hilbert-curve distance of (x, y) on a 2^order grid. */
static uint64_t xy2d(uint32_t order, uint32_t x, uint32_t y) {
    uint32_t n = (order >= 32) ? 0u : (1u << order);
    uint64_t d = 0;
    for (uint32_t s = n / 2; s > 0; s /= 2) {
        uint32_t rx = (x & s) ? 1u : 0u;
        uint32_t ry = (y & s) ? 1u : 0u;
        d += (uint64_t)s * (uint64_t)s * (uint64_t)((3u * rx) ^ ry);
        rot(n, &x, &y, rx, ry);
    }
    return d;
}

uint64_t arpt_archive_tile_id(int level, int x, int y) {
    if (level < 0 || x < 0 || y < 0) return 0;
    uint64_t h = xy2d((uint32_t)level, (uint32_t)x, (uint32_t)y) & HILBERT_MASK;
    return ((uint64_t)level << HILBERT_BITS) | h;
}

arpt_archive *arpt_archive_open(const char *path) {
    if (!path) return NULL;

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "archive: cannot open %s\n", path);
        return NULL;
    }

    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < (off_t)HEADER_SIZE) {
        fprintf(stderr, "archive: %s is truncated\n", path);
        close(fd);
        return NULL;
    }
    size_t size = (size_t)st.st_size;

    void *map = mmap(NULL, size, PROT_READ, MAP_PRIVATE, fd, 0);
    /* The mapping outlives the descriptor, so the archive can be opened once
       and the fd not held for the life of the run. */
    close(fd);
    if (map == MAP_FAILED) {
        fprintf(stderr, "archive: cannot map %s\n", path);
        return NULL;
    }

    const uint8_t *data = map;
    uint32_t magic = rd_u32(data, H_MAGIC);
    uint32_t version = rd_u32(data, H_VERSION);
    if (magic != ARCHIVE_MAGIC) {
        fprintf(stderr, "archive: %s has bad magic 0x%08x\n", path, magic);
        munmap(map, size);
        return NULL;
    }
    if (version != ARCHIVE_VERSION) {
        fprintf(stderr, "archive: %s is version %u, expected %u\n", path,
                version, ARCHIVE_VERSION);
        munmap(map, size);
        return NULL;
    }

    arpt_archive *a = calloc(1, sizeof(*a));
    if (!a) {
        munmap(map, size);
        return NULL;
    }
    a->data = data;
    a->size = size;
    a->tile_count = rd_u64(data, H_TILE_COUNT);
    a->dir_offset = rd_u64(data, H_DIR_OFFSET);
    a->meta_offset = rd_u64(data, H_META_OFFSET);
    a->meta_size = rd_u64(data, H_META_SIZE);

    /* Bound the directory and metadata now, so every later lookup can trust
       its own arithmetic instead of re-checking a hostile file. */
    if (a->tile_count > (UINT64_MAX - a->dir_offset) / DIR_ENTRY_SIZE ||
        a->dir_offset + a->tile_count * DIR_ENTRY_SIZE > size ||
        a->meta_size > UINT64_MAX - a->meta_offset ||
        a->meta_offset + a->meta_size > size) {
        fprintf(stderr, "archive: %s has a directory outside the file\n", path);
        munmap(map, size);
        free(a);
        return NULL;
    }
    return a;
}

void arpt_archive_free(arpt_archive *a) {
    if (!a) return;
    munmap((void *)(uintptr_t)a->data, a->size);
    free(a);
}

bool arpt_archive_metadata(const arpt_archive *a, const uint8_t **blob,
                           size_t *size) {
    if (!a || !blob || !size || a->meta_size == 0) return false;
    *blob = a->data + a->meta_offset;
    *size = (size_t)a->meta_size;
    return true;
}

uint64_t arpt_archive_tile_count(const arpt_archive *a) {
    return a ? a->tile_count : 0;
}

bool arpt_archive_tile(const arpt_archive *a, int level, int x, int y,
                       const uint8_t **blob, size_t *size) {
    if (!a || !blob || !size) return false;

    uint64_t id = arpt_archive_tile_id(level, x, y);

    /* The directory is sorted by tile id, so search it where it lies rather
       than materializing it. */
    uint64_t lo = 0, hi = a->tile_count;
    while (lo < hi) {
        uint64_t mid = lo + (hi - lo) / 2;
        const uint8_t *e = a->data + a->dir_offset + mid * DIR_ENTRY_SIZE;
        uint64_t entry_id = rd_u64(e, E_HILBERT_ID);
        if (entry_id < id) {
            lo = mid + 1;
        } else if (entry_id > id) {
            hi = mid;
        } else {
            uint64_t off = rd_u64(e, E_OFFSET);
            uint64_t len = rd_u64(e, E_SIZE);
            if (len > UINT64_MAX - off || off + len > a->size) return false;
            *blob = a->data + off;
            *size = (size_t)len;
            return true;
        }
    }
    return false;
}

#endif /* !__EMSCRIPTEN__ */
