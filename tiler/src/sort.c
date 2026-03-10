#include "sort.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Record layout in buffer: [key:8][size:4][data:size] */
#define REC_HDR (sizeof(uint64_t) + sizeof(uint32_t))

/* ---- Growable byte buffer ---- */

typedef struct {
    uint8_t *data;
    size_t   len;
    size_t   cap;
} membuf;

static bool membuf_append(membuf *b, const void *src, size_t n) {
    if (b->len + n > b->cap) {
        size_t nc = b->cap ? b->cap * 2 : 4096;
        while (nc < b->len + n) nc *= 2;
        uint8_t *p = realloc(b->data, nc);
        if (!p) return false;
        b->data = p;
        b->cap = nc;
    }
    memcpy(b->data + b->len, src, n);
    b->len += n;
    return true;
}

/* ---- Run file (one sorted spill) ---- */

typedef struct {
    FILE    *fp;
    uint64_t key;
    uint8_t *buf;
    uint32_t buf_cap;
    uint32_t data_size;
    bool     exhausted;
} run_file;

static bool run_read_next(run_file *r) {
    if (fread(&r->key, sizeof(uint64_t), 1, r->fp) != 1) {
        r->exhausted = true;
        return false;
    }
    uint32_t sz;
    if (fread(&sz, sizeof(uint32_t), 1, r->fp) != 1) {
        r->exhausted = true;
        return false;
    }
    if (sz > r->buf_cap) {
        uint8_t *p = realloc(r->buf, sz);
        if (!p) { r->exhausted = true; return false; }
        r->buf = p;
        r->buf_cap = sz;
    }
    r->data_size = sz;
    if (sz > 0 && fread(r->buf, 1, sz, r->fp) != sz) {
        r->exhausted = true;
        return false;
    }
    return true;
}

/* ---- Min-heap for k-way merge ---- */

typedef struct {
    int     *idx;       /* heap of run indices */
    int      size;
    int      cap;
    run_file *runs;     /* pointer to run array (not owned) */
} min_heap;

static void heap_swap(int *a, int *b) {
    int t = *a; *a = *b; *b = t;
}

static void heap_sift_down(min_heap *h, int pos) {
    while (1) {
        int smallest = pos;
        int left = 2 * pos + 1;
        int right = 2 * pos + 2;
        if (left < h->size &&
            h->runs[h->idx[left]].key < h->runs[h->idx[smallest]].key)
            smallest = left;
        if (right < h->size &&
            h->runs[h->idx[right]].key < h->runs[h->idx[smallest]].key)
            smallest = right;
        if (smallest == pos) break;
        heap_swap(&h->idx[pos], &h->idx[smallest]);
        pos = smallest;
    }
}

static void heap_sift_up(min_heap *h, int pos) {
    while (pos > 0) {
        int parent = (pos - 1) / 2;
        if (h->runs[h->idx[pos]].key >= h->runs[h->idx[parent]].key) break;
        heap_swap(&h->idx[pos], &h->idx[parent]);
        pos = parent;
    }
}

static bool heap_init(min_heap *h, run_file *runs, int n_runs) {
    h->idx = malloc((size_t)n_runs * sizeof(int));
    if (!h->idx) return false;
    h->size = 0;
    h->cap = n_runs;
    h->runs = runs;
    /* Insert all non-exhausted runs */
    for (int i = 0; i < n_runs; i++) {
        if (!runs[i].exhausted) {
            h->idx[h->size] = i;
            heap_sift_up(h, h->size);
            h->size++;
        }
    }
    return true;
}

static void heap_free(min_heap *h) {
    free(h->idx);
    h->idx = NULL;
    h->size = 0;
}

/* ---- Sorter ---- */

typedef struct { uint64_t key; size_t offset; } rec_idx;

struct arpt_sorter {
    char   *tmp_dir;
    size_t  mem_budget;

    /* Accumulation */
    membuf  buf;
    size_t  n_records;

    /* In-memory sorted index (after finish, no spills) */
    rec_idx *sorted;
    size_t   sorted_n;
    size_t   iter_pos;

    /* Spill files */
    char  **run_paths;
    int     n_runs;
    int     run_cap;

    /* Merge state */
    run_file *runs;
    min_heap  heap;
    bool      finished;

    /* Stashed record from heap pop (data pointer valid until next call) */
    uint64_t stash_key;
    uint8_t *stash_buf;
    uint32_t stash_size;
    uint32_t stash_cap;
};

static int cmp_rec(const void *a, const void *b) {
    uint64_t ka = ((const rec_idx *)a)->key;
    uint64_t kb = ((const rec_idx *)b)->key;
    if (ka < kb) return -1;
    if (ka > kb) return 1;
    return 0;
}

static rec_idx *build_index(const membuf *buf, size_t n, size_t *out_n) {
    rec_idx *idx = malloc(n * sizeof(*idx));
    if (!idx) return NULL;
    size_t pos = 0;
    for (size_t i = 0; i < n; i++) {
        uint64_t key;
        uint32_t sz;
        memcpy(&key, buf->data + pos, sizeof(key));
        memcpy(&sz, buf->data + pos + sizeof(key), sizeof(sz));
        idx[i].key = key;
        idx[i].offset = pos;
        pos += REC_HDR + sz;
    }
    qsort(idx, n, sizeof(rec_idx), cmp_rec);
    *out_n = n;
    return idx;
}

static bool flush_run(arpt_sorter *s) {
    if (s->n_records == 0) return true;

    size_t n;
    rec_idx *idx = build_index(&s->buf, s->n_records, &n);
    if (!idx) return false;

    char path[512];
    snprintf(path, sizeof(path), "%s/arpt_sort_%d_XXXXXX",
             s->tmp_dir, s->n_runs);
    int fd = mkstemp(path);
    if (fd < 0) { free(idx); return false; }

    FILE *fp = fdopen(fd, "wb");
    if (!fp) { close(fd); free(idx); return false; }

    for (size_t i = 0; i < n; i++) {
        size_t off = idx[i].offset;
        uint64_t key;
        uint32_t sz;
        memcpy(&key, s->buf.data + off, sizeof(key));
        memcpy(&sz, s->buf.data + off + sizeof(key), sizeof(sz));
        fwrite(&key, sizeof(key), 1, fp);
        fwrite(&sz, sizeof(sz), 1, fp);
        if (sz > 0) fwrite(s->buf.data + off + REC_HDR, 1, sz, fp);
    }
    fclose(fp);
    free(idx);

    if (s->n_runs == s->run_cap) {
        int nc = s->run_cap ? s->run_cap * 2 : 4;
        char **p = realloc(s->run_paths, (size_t)nc * sizeof(char *));
        if (!p) return false;
        s->run_paths = p;
        s->run_cap = nc;
    }
    s->run_paths[s->n_runs] = strdup(path);
    if (!s->run_paths[s->n_runs]) return false;
    s->n_runs++;

    s->buf.len = 0;
    s->n_records = 0;
    return true;
}

/* ---- Public API ---- */

arpt_sorter *arpt_sorter_create(const char *tmp_dir, size_t mem_budget) {
    arpt_sorter *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->tmp_dir = strdup(tmp_dir ? tmp_dir : "/tmp");
    if (!s->tmp_dir) { free(s); return NULL; }
    s->mem_budget = mem_budget > 0 ? mem_budget : 64 * 1024 * 1024;
    return s;
}

bool arpt_sorter_add(arpt_sorter *s, uint64_t key,
                     const void *data, size_t size) {
    if (!s || s->finished) return false;

    size_t rec_size = REC_HDR + size;

    if (s->buf.len + rec_size > s->mem_budget && s->n_records > 0) {
        if (!flush_run(s)) return false;
    }

    uint32_t sz32 = (uint32_t)size;
    if (!membuf_append(&s->buf, &key, sizeof(key))) return false;
    if (!membuf_append(&s->buf, &sz32, sizeof(sz32))) return false;
    if (size > 0 && !membuf_append(&s->buf, data, size)) return false;
    s->n_records++;
    return true;
}

bool arpt_sorter_finish(arpt_sorter *s) {
    if (!s) return false;

    if (s->n_runs == 0) {
        /* All in memory */
        if (s->n_records > 0) {
            s->sorted = build_index(&s->buf, s->n_records, &s->sorted_n);
            if (!s->sorted) return false;
        }
        s->iter_pos = 0;
        s->finished = true;
        return true;
    }

    /* Flush remaining records */
    if (s->n_records > 0) {
        if (!flush_run(s)) return false;
    }

    /* Open all run files */
    s->runs = calloc((size_t)s->n_runs, sizeof(run_file));
    if (!s->runs) return false;

    for (int i = 0; i < s->n_runs; i++) {
        s->runs[i].fp = fopen(s->run_paths[i], "rb");
        if (!s->runs[i].fp) return false;
        run_read_next(&s->runs[i]);
    }

    /* Build min-heap for O(log k) merge */
    if (!heap_init(&s->heap, s->runs, s->n_runs)) return false;

    s->finished = true;
    return true;
}

bool arpt_sorter_next(arpt_sorter *s, uint64_t *key,
                      const void **data, size_t *size) {
    if (!s || !s->finished) return false;

    if (!s->runs) {
        /* In-memory path */
        if (s->iter_pos >= s->sorted_n) return false;
        size_t off = s->sorted[s->iter_pos++].offset;
        uint64_t k;
        uint32_t sz;
        memcpy(&k, s->buf.data + off, sizeof(k));
        memcpy(&sz, s->buf.data + off + sizeof(k), sizeof(sz));
        if (key) *key = k;
        if (data) *data = (sz > 0) ? s->buf.data + off + REC_HDR : NULL;
        if (size) *size = sz;
        return true;
    }

    /* Heap-based k-way merge: peek min, copy its data, then advance */
    if (s->heap.size == 0) return false;

    int best = s->heap.idx[0];
    run_file *r = &s->runs[best];

    /* Stash the current record before advancing */
    s->stash_key = r->key;
    if (r->data_size > s->stash_cap) {
        uint8_t *p = realloc(s->stash_buf, r->data_size);
        if (!p) return false;
        s->stash_buf = p;
        s->stash_cap = r->data_size;
    }
    s->stash_size = r->data_size;
    if (r->data_size > 0) {
        memcpy(s->stash_buf, r->buf, r->data_size);
    }

    /* Advance the run and re-heapify */
    run_read_next(r);
    if (r->exhausted) {
        s->heap.size--;
        if (s->heap.size > 0) {
            s->heap.idx[0] = s->heap.idx[s->heap.size];
            heap_sift_down(&s->heap, 0);
        }
    } else {
        heap_sift_down(&s->heap, 0);
    }

    if (key) *key = s->stash_key;
    if (data) *data = s->stash_buf;
    if (size) *size = s->stash_size;
    return true;
}

void arpt_sorter_free(arpt_sorter *s) {
    if (!s) return;

    heap_free(&s->heap);

    if (s->runs) {
        for (int i = 0; i < s->n_runs; i++) {
            if (s->runs[i].fp) fclose(s->runs[i].fp);
            free(s->runs[i].buf);
        }
        free(s->runs);
    }
    if (s->run_paths) {
        for (int i = 0; i < s->n_runs; i++) {
            if (s->run_paths[i]) {
                remove(s->run_paths[i]);
                free(s->run_paths[i]);
            }
        }
        free(s->run_paths);
    }

    free(s->stash_buf);
    free(s->sorted);
    free(s->buf.data);
    free(s->tmp_dir);
    free(s);
}
