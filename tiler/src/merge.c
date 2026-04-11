/* Merge connected transportation segments into longer linestrings.
 *
 * Only major road classes (motorway, trunk, primary and their _link
 * variants) are included.  Segments are merged by spatial proximity
 * of their endpoints — any two segments whose endpoints are within
 * ~10 m are considered connected, regardless of connector IDs or
 * class boundaries.  This produces long continuous linestrings that
 * survive simplification at low zoom levels.
 *
 *   Pass 1 — single-pass read of class + geometry + properties.
 *            Extract endpoint coordinates from WKB.
 *   Pass 2 — spatial endpoint index → build merge chains.
 *   Pass 3 — write output from memory (no second file scan). */

#include "merge.h"
#include "geoparquet.h"
#include "parquet.h"
#include "wkb.h"
#include "geom.h"

#include <carquet/carquet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Merge eligibility ────────────────────────────────────────────────── */

static const char *const MERGE_CLASSES[] = {
    "motorway",
    NULL
};

static bool is_mergeable_class(const uint8_t *data, size_t len) {
    for (int i = 0; MERGE_CLASSES[i]; i++) {
        size_t clen = strlen(MERGE_CLASSES[i]);
        if (len == clen && memcmp(data, MERGE_CLASSES[i], clen) == 0)
            return true;
    }
    return false;
}

static int32_t class_min_zoom(const char *cls) {
    if (!cls) return 0;
    if (strcmp(cls, "motorway") == 0 || strcmp(cls, "motorway_link") == 0) return 4;
    if (strcmp(cls, "trunk") == 0    || strcmp(cls, "trunk_link") == 0)    return 5;
    if (strcmp(cls, "primary") == 0  || strcmp(cls, "primary_link") == 0)  return 7;
    return 0;
}

/* ── Per-segment info ─────────────────────────────────────────────────── */

typedef struct {
    /* Endpoint coordinates (extracted from WKB) */
    double start_x, start_y;
    double end_x,   end_y;

    int32_t  chain_id;
    int32_t  chain_pos;
    bool     reversed;

    /* Stored output data (owned) */
    uint8_t *wkb;
    uint32_t wkb_len;
    char    *type;
    char    *subtype;
    char    *cls;
    float    bbox[4];
    bool     has_bbox;
    int32_t  min_zoom, max_zoom, sort_key;
    bool     has_min_zoom, has_max_zoom, has_sort_key;
} seg_info;

static void seg_info_free(seg_info *s) {
    free(s->wkb);
    free(s->type);
    free(s->subtype);
    free(s->cls);
}

/* Decode WKB type and detect Z coordinates.
 * Returns base type (2=LineString, 5=MultiLineString) and stride per point. */
static bool decode_wkb(const uint8_t *wkb, uint32_t wkb_len,
                       uint32_t *base_type, size_t *point_stride) {
    if (wkb_len < 5) return false;
    uint32_t raw;
    memcpy(&raw, wkb + 1, 4);

    bool has_z = false;
    uint32_t t = raw;
    /* ISO Z: type + 1000 */
    if (raw > 1000 && raw <= 1006) { t = raw - 1000; has_z = true; }
    /* OGC Z flag */
    else if (raw & 0x80000000u)    { t = raw & ~0x80000000u; has_z = true; }

    *base_type = t;
    *point_stride = has_z ? 24 : 16;  /* x,y[,z] as doubles */
    return true;
}

/* Extract first and last point from WKB LineString/MultiLineString.
 * Handles both 2D and 3D geometries. */
static bool extract_endpoints(const uint8_t *wkb, uint32_t wkb_len,
                              double *sx, double *sy,
                              double *ex, double *ey) {
    uint32_t type;
    size_t stride;
    if (!decode_wkb(wkb, wkb_len, &type, &stride)) return false;

    if (type == 2) {
        /* LineString */
        if (wkb_len < 9) return false;
        uint32_t npts;
        memcpy(&npts, wkb + 5, 4);
        if (npts < 2 || wkb_len < 9 + npts * stride) return false;
        memcpy(sx, wkb + 9, 8);
        memcpy(sy, wkb + 9 + 8, 8);
        size_t last_off = 9 + (size_t)(npts - 1) * stride;
        memcpy(ex, wkb + last_off, 8);
        memcpy(ey, wkb + last_off + 8, 8);
        return true;
    } else if (type == 5) {
        /* MultiLineString */
        if (wkb_len < 9) return false;
        uint32_t nlines;
        memcpy(&nlines, wkb + 5, 4);
        if (nlines == 0) return false;

        /* First line's first point */
        size_t off = 9;
        if (off + 9 > wkb_len) return false;
        uint32_t sub_type; size_t sub_stride;
        if (!decode_wkb(wkb + off, wkb_len - (uint32_t)off,
                        &sub_type, &sub_stride)) return false;
        off += 5;
        uint32_t npts1;
        memcpy(&npts1, wkb + off, 4);
        off += 4;
        if (npts1 < 2 || off + npts1 * sub_stride > wkb_len) return false;
        memcpy(sx, wkb + off, 8);
        memcpy(sy, wkb + off + 8, 8);

        /* Navigate to last line's last point */
        off = 9;
        for (uint32_t li = 0; li < nlines; li++) {
            if (off + 9 > wkb_len) return false;
            if (!decode_wkb(wkb + off, wkb_len - (uint32_t)off,
                            &sub_type, &sub_stride)) return false;
            off += 5;
            uint32_t np;
            memcpy(&np, wkb + off, 4);
            off += 4;
            if (li == nlines - 1) {
                if (np < 2 || off + np * sub_stride > wkb_len) return false;
                size_t last_off = off + (size_t)(np - 1) * sub_stride;
                memcpy(ex, wkb + last_off, 8);
                memcpy(ey, wkb + last_off + 8, 8);
                return true;
            }
            off += (size_t)np * sub_stride;
        }
        return false;
    }
    return false;
}

/* ── Spatial endpoint index (R-tree) ──────────────────────────────────── */

#include "rtree/rtree.h"

/* Pack seg_idx (30 bits) + is_start (1 bit) into a void* for the R-tree.
 * Bit 0 = is_start, bits 1..30 = seg_idx. */
static const void *ep_pack(uint32_t seg_idx, uint8_t is_start) {
    uintptr_t v = ((uintptr_t)seg_idx << 1) | (is_start & 1);
    return (const void *)v;
}
static uint32_t ep_seg_idx(const void *data) {
    return (uint32_t)((uintptr_t)data >> 1);
}
static bool ep_is_start(const void *data) {
    return ((uintptr_t)data & 1) != 0;
}

/* Search radius: ~0.0001° ≈ 11 m at equator.
 * Overture segments that form the same road share exact coordinates
 * at their junction points, so a tight radius suffices. */
#define EP_RADIUS 0.0001

/* Context for the R-tree search callback. */
typedef struct {
    const seg_info *segs;
    uint32_t  seg_idx;      /* segment to exclude */
    double    qx, qy;       /* query point */
    uint32_t  best;
    double    best_dist;
    bool      best_is_start;
} ep_search_ctx;

static bool ep_search_cb(const double *min, const double *max,
                         const void *data, void *udata) {
    (void)min; (void)max;
    ep_search_ctx *ctx = (ep_search_ctx *)udata;
    uint32_t si = ep_seg_idx(data);
    if (si == ctx->seg_idx) return true;
    if (ctx->segs[si].chain_id != -1) return true;  /* assigned or temp-marked */

    bool is_start = ep_is_start(data);
    double ex = is_start ? ctx->segs[si].start_x : ctx->segs[si].end_x;
    double ey = is_start ? ctx->segs[si].start_y : ctx->segs[si].end_y;
    double dx = ctx->qx - ex;
    double dy = ctx->qy - ey;
    double dist = dx * dx + dy * dy;
    if (dist < ctx->best_dist) {
        ctx->best = si;
        ctx->best_dist = dist;
        ctx->best_is_start = is_start;
    }
    return true;  /* continue */
}

static uint32_t ep_find_neighbor(const struct rtree *tr, const seg_info *segs,
                                 uint32_t seg_idx, double x, double y,
                                 bool *neighbor_is_start) {
    double qmin[2] = { x - EP_RADIUS, y - EP_RADIUS };
    double qmax[2] = { x + EP_RADIUS, y + EP_RADIUS };

    ep_search_ctx ctx = {
        .segs = segs, .seg_idx = seg_idx,
        .qx = x, .qy = y,
        .best = UINT32_MAX, .best_dist = EP_RADIUS * EP_RADIUS,
        .best_is_start = false,
    };
    rtree_search(tr, qmin, qmax, ep_search_cb, &ctx);

    if (ctx.best != UINT32_MAX) {
        *neighbor_is_start = ctx.best_is_start;
        return ctx.best;
    }
    return UINT32_MAX;
}

/* ── Chain data ───────────────────────────────────────────────────────── */

typedef struct {
    uint32_t *seg_indices;
    bool     *reversed;
    uint32_t  length;
    uint32_t  capacity;
} chain;

static bool chain_push(chain *c, uint32_t seg_idx, bool rev) {
    if (c->length >= c->capacity) {
        uint32_t new_cap = c->capacity ? c->capacity * 2 : 8;
        uint32_t *ni = realloc(c->seg_indices, new_cap * sizeof(*ni));
        bool *nr = realloc(c->reversed, new_cap * sizeof(*nr));
        if (!ni || !nr) { free(ni); free(nr); return false; }
        c->seg_indices = ni;
        c->reversed = nr;
        c->capacity = new_cap;
    }
    c->seg_indices[c->length] = seg_idx;
    c->reversed[c->length] = rev;
    c->length++;
    return true;
}

static void chain_free(chain *c) {
    free(c->seg_indices);
    free(c->reversed);
    memset(c, 0, sizeof(*c));
}

/* ── Byte array copy helpers ──────────────────────────────────────────── */

static char *ba_strdup(const carquet_byte_array_t *ba) {
    if (!ba->data || ba->length <= 0) return NULL;
    return strndup((const char *)ba->data, (size_t)ba->length);
}

static uint8_t *ba_memdup(const carquet_byte_array_t *ba, uint32_t *out_len) {
    if (!ba->data || ba->length <= 0) { *out_len = 0; return NULL; }
    uint8_t *copy = malloc((size_t)ba->length);
    if (!copy) { *out_len = 0; return NULL; }
    memcpy(copy, ba->data, (size_t)ba->length);
    *out_len = (uint32_t)ba->length;
    return copy;
}

/* ── Pass 1: Single-pass read ─────────────────────────────────────────── */

static seg_info *pass1_read(const char *path, uint32_t *n_segs) {
    *n_segs = 0;

    carquet_error_t err = CARQUET_ERROR_INIT;
    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);
    opts.use_mmap = true;
    carquet_reader_t *reader = carquet_reader_open(path, &opts, &err);
    if (!reader) return NULL;

    int64_t total_rows = carquet_reader_num_rows(reader);
    if (total_rows <= 0) { carquet_reader_close(reader); return NULL; }

    /* Find columns — reject repeated (LIST) columns since we read
     * them as flat (one value per row). Use the arpt_parquet wrapper
     * for reliable column discovery including repetition check. */
    arpt_parquet *pq = arpt_parquet_open(path);
    int32_t col_class = -1, col_geom = -1, col_type = -1, col_subtype = -1;
    int32_t col_bxmin = -1, col_bymin = -1, col_bxmax = -1, col_bymax = -1;
    int32_t col_cmin = -1, col_cmax = -1, col_csort = -1;
    if (pq) {
        /* Use the wrapper's find which gives leaf column indices */
        col_geom = arpt_parquet_find_column(pq, "geometry");
        col_type = arpt_parquet_find_column(pq, "type");
        col_subtype = arpt_parquet_find_column(pq, "subtype");
        col_class = arpt_parquet_find_column(pq, "class");
        col_bxmin = arpt_parquet_find_column_path(pq, "bbox.xmin");
        col_bymin = arpt_parquet_find_column_path(pq, "bbox.ymin");
        col_bxmax = arpt_parquet_find_column_path(pq, "bbox.xmax");
        col_bymax = arpt_parquet_find_column_path(pq, "bbox.ymax");
        col_cmin = arpt_parquet_find_column_path(pq, "cartography.min_zoom");
        col_cmax = arpt_parquet_find_column_path(pq, "cartography.max_zoom");
        col_csort = arpt_parquet_find_column_path(pq, "cartography.sort_key");

        /* Reject repeated columns */
        if (col_type >= 0 && arpt_parquet_column_is_repeated(pq, col_type))
            col_type = -1;
        if (col_subtype >= 0 && arpt_parquet_column_is_repeated(pq, col_subtype))
            col_subtype = -1;
        if (col_class >= 0 && arpt_parquet_column_is_repeated(pq, col_class))
            col_class = -1;

        arpt_parquet_close(pq);
    }

    /* Detect bbox column physical type — needed for buffer sizing.
     * Re-open briefly to check (pq was closed above). */
    bool bbox_is_double = false;
    {
        arpt_parquet *pq_chk = arpt_parquet_open(path);
        if (pq_chk) {
            if (col_bxmin >= 0)
                bbox_is_double = (arpt_parquet_column_type(pq_chk, col_bxmin)
                                  == ARPT_PARQUET_DOUBLE);
            arpt_parquet_close(pq_chk);
        }
    }

    fprintf(stderr, "merge: %lld rows, class=%d geom=%d type=%d subtype=%d\n",
            (long long)total_rows, col_class, col_geom, col_type, col_subtype);

    if (col_class < 0 || col_geom < 0) {
        fprintf(stderr, "merge: required columns not found\n");
        carquet_reader_close(reader);
        return NULL;
    }

    uint32_t seg_cap = 1024 * 1024;
    seg_info *segs = calloc(seg_cap, sizeof(*segs));
    if (!segs) { carquet_reader_close(reader); return NULL; }
    uint32_t seg_count = 0;

    int32_t n_rg = carquet_reader_num_row_groups(reader);
    uint64_t rows_scanned = 0;

    for (int32_t rg = 0; rg < n_rg; rg++) {
        /* Step 1: Read class → bitmap of mergeable rows */
        carquet_column_reader_t *cr_cls =
            carquet_reader_get_column(reader, rg, col_class, &err);
        if (!cr_cls) continue;

        int64_t rg_rows = carquet_column_remaining(cr_cls);
        size_t bmap_bytes = ((size_t)rg_rows + 7) / 8;
        uint8_t *bmap = calloc(bmap_bytes, 1);
        if (!bmap) { carquet_column_reader_free(cr_cls); continue; }

        {
            #define CLS_BATCH 8192
            carquet_byte_array_t cls_vals[CLS_BATCH];
            int16_t cls_def[CLS_BATCH];
            int16_t cls_max_def = 0;
            int64_t row = 0;
            for (;;) {
                int64_t count = carquet_column_read_batch(
                    cr_cls, cls_vals, CLS_BATCH, cls_def, NULL);
                if (count <= 0) break;
                if (cls_max_def == 0)
                    for (int64_t d = 0; d < count; d++)
                        if (cls_def[d] > cls_max_def) cls_max_def = cls_def[d];
                int vi = 0;
                for (int64_t i = 0; i < count; i++) {
                    bool present = (cls_def[i] == cls_max_def);
                    if (present && row < rg_rows) {
                        carquet_byte_array_t *ba = &cls_vals[vi];
                        if (ba->data && ba->length > 0 &&
                            is_mergeable_class(ba->data, (size_t)ba->length))
                            bmap[row / 8] |= (uint8_t)(1u << (row % 8));
                    }
                    if (present) vi++;
                    row++;
                }
            }
            #undef CLS_BATCH
        }
        carquet_column_reader_free(cr_cls);

        /* Count mergeable and build row map */
        uint32_t rg_merge_count = 0;
        for (int64_t r = 0; r < rg_rows; r++)
            if (bmap[r / 8] & (1u << (r % 8))) rg_merge_count++;

        /* Skip expensive column reads for RGs with no mergeable rows */
        if (rg_merge_count == 0) {
            free(bmap);
            rows_scanned += (uint64_t)rg_rows;
            if (rg % 50 == 0 || rg == n_rg - 1)
                fprintf(stderr, "merge: rg %d/%d  %llu rows  %u mergeable\n",
                        rg + 1, n_rg, (unsigned long long)rows_scanned, seg_count);
            continue;
        }

        int32_t *row_to_local = malloc((size_t)rg_rows * sizeof(int32_t));
        if (!row_to_local) { free(bmap); continue; }
        {
            int32_t li = 0;
            for (int64_t r = 0; r < rg_rows; r++)
                row_to_local[r] = (bmap[r / 8] & (1u << (r % 8))) ? li++ : -1;
        }

        /* Ensure capacity */
        while (seg_count + rg_merge_count > seg_cap) {
            uint32_t new_cap = seg_cap * 2;
            seg_info *ns = realloc(segs, new_cap * sizeof(*ns));
            if (!ns) goto next_rg;
            segs = ns; seg_cap = new_cap;
        }
        for (uint32_t i = 0; i < rg_merge_count; i++) {
            seg_info *s = &segs[seg_count + i];
            memset(s, 0, sizeof(*s));
            s->chain_id = -1;
        }

        /* Step 2: Read flat columns for mergeable rows */

        /* Macro: read a flat BYTE_ARRAY column */
        #define READ_FLAT_BA(col_idx, field) do { \
            if ((col_idx) >= 0 && rg_merge_count > 0) { \
                carquet_column_reader_t *cr = \
                    carquet_reader_get_column(reader, rg, (col_idx), &err); \
                if (cr) { \
                    carquet_byte_array_t _vals[4096]; \
                    int16_t _def[4096]; \
                    int16_t _max_def = 0; \
                    int64_t _row = 0; \
                    for (;;) { \
                        int64_t _count = carquet_column_read_batch( \
                            cr, _vals, 4096, _def, NULL); \
                        if (_count <= 0) break; \
                        if (_max_def == 0) \
                            for (int64_t _d = 0; _d < _count; _d++) \
                                if (_def[_d] > _max_def) _max_def = _def[_d]; \
                        int _vi = 0; \
                        for (int64_t _i = 0; _i < _count; _i++) { \
                            bool _present = (_def[_i] == _max_def); \
                            if (_present && _row < rg_rows && row_to_local[_row] >= 0) { \
                                uint32_t _li = (uint32_t)row_to_local[_row]; \
                                segs[seg_count + _li].field = ba_strdup(&_vals[_vi]); \
                            } \
                            if (_present) _vi++; \
                            _row++; \
                        } \
                    } \
                    carquet_column_reader_free(cr); \
                } \
            } \
        } while(0)

        /* Geometry: copy raw WKB + extract endpoints */
        if (col_geom >= 0 && rg_merge_count > 0) {
            carquet_column_reader_t *cr =
                carquet_reader_get_column(reader, rg, col_geom, &err);
            if (cr) {
                carquet_byte_array_t _vals[4096];
                int16_t _def[4096];
                int16_t _max_def = 0;
                int64_t _row = 0;
                for (;;) {
                    int64_t _count = carquet_column_read_batch(
                        cr, _vals, 4096, _def, NULL);
                    if (_count <= 0) break;
                    if (_max_def == 0)
                        for (int64_t _d = 0; _d < _count; _d++)
                            if (_def[_d] > _max_def) _max_def = _def[_d];
                    int _vi = 0;
                    for (int64_t _i = 0; _i < _count; _i++) {
                        bool _present = (_def[_i] == _max_def);
                        if (_present && _row < rg_rows && row_to_local[_row] >= 0) {
                            uint32_t _li = (uint32_t)row_to_local[_row];
                            seg_info *s = &segs[seg_count + _li];
                            s->wkb = ba_memdup(&_vals[_vi], &s->wkb_len);
                            extract_endpoints(s->wkb, s->wkb_len,
                                              &s->start_x, &s->start_y,
                                              &s->end_x, &s->end_y);
                        }
                        if (_present) _vi++;
                        _row++;
                    }
                }
                carquet_column_reader_free(cr);
            }
        }

        READ_FLAT_BA(col_type, type);
        READ_FLAT_BA(col_subtype, subtype);
        READ_FLAT_BA(col_class, cls);

        /* Read bbox columns.  Overture bbox may be FLOAT or DOUBLE,
         * so we use a double-sized buffer and read the physical type. */
        {
            int32_t bbox_cols[4] = { col_bxmin, col_bymin, col_bxmax, col_bymax };
            for (int bi = 0; bi < 4; bi++) {
                if (bbox_cols[bi] < 0 || rg_merge_count == 0) continue;
                carquet_column_reader_t *cr =
                    carquet_reader_get_column(reader, rg, bbox_cols[bi], &err);
                if (!cr) continue;

                bool is_dbl = bbox_is_double;
                /* Buffer large enough for either float or double */
                double _dvals[4096];
                float  _fvals[4096];
                void  *buf = is_dbl ? (void *)_dvals : (void *)_fvals;
                int16_t _def[4096];
                int16_t _max_def = 0;
                int64_t _row = 0;
                for (;;) {
                    int64_t _count = carquet_column_read_batch(
                        cr, buf, 4096, _def, NULL);
                    if (_count <= 0) break;
                    if (_max_def == 0)
                        for (int64_t _d = 0; _d < _count; _d++)
                            if (_def[_d] > _max_def) _max_def = _def[_d];
                    int _vi = 0;
                    for (int64_t _i = 0; _i < _count; _i++) {
                        bool _present = (_def[_i] == _max_def);
                        if (_present && _row < rg_rows &&
                            row_to_local[_row] >= 0) {
                            uint32_t _li = (uint32_t)row_to_local[_row];
                            float val = is_dbl ? (float)_dvals[_vi]
                                               : _fvals[_vi];
                            segs[seg_count + _li].bbox[bi] = val;
                            segs[seg_count + _li].has_bbox = true;
                        }
                        if (_present) _vi++;
                        _row++;
                    }
                }
                carquet_column_reader_free(cr);
            }
        }

        #define READ_FLAT_INT32(col_idx, field, has_field) do { \
            if ((col_idx) >= 0 && rg_merge_count > 0) { \
                carquet_column_reader_t *cr = \
                    carquet_reader_get_column(reader, rg, (col_idx), &err); \
                if (cr) { \
                    int32_t _ivals[4096]; \
                    int16_t _def[4096]; \
                    int16_t _max_def = 0; \
                    int64_t _row = 0; \
                    for (;;) { \
                        int64_t _count = carquet_column_read_batch( \
                            cr, _ivals, 4096, _def, NULL); \
                        if (_count <= 0) break; \
                        if (_max_def == 0) \
                            for (int64_t _d = 0; _d < _count; _d++) \
                                if (_def[_d] > _max_def) _max_def = _def[_d]; \
                        int _vi = 0; \
                        for (int64_t _i = 0; _i < _count; _i++) { \
                            bool _present = (_def[_i] == _max_def); \
                            if (_present && _row < rg_rows && row_to_local[_row] >= 0) { \
                                uint32_t _li = (uint32_t)row_to_local[_row]; \
                                segs[seg_count + _li].field = _ivals[_vi]; \
                                segs[seg_count + _li].has_field = true; \
                            } \
                            if (_present) _vi++; \
                            _row++; \
                        } \
                    } \
                    carquet_column_reader_free(cr); \
                } \
            } \
        } while(0)

        READ_FLAT_INT32(col_cmin, min_zoom, has_min_zoom);
        READ_FLAT_INT32(col_cmax, max_zoom, has_max_zoom);
        READ_FLAT_INT32(col_csort, sort_key, has_sort_key);

        #undef READ_FLAT_BA
        #undef READ_FLAT_FLOAT
        #undef READ_FLAT_INT32

        seg_count += rg_merge_count;

next_rg:
        free(bmap);
        free(row_to_local);
        rows_scanned += (uint64_t)rg_rows;

        if (rg % 50 == 0 || rg == n_rg - 1)
            fprintf(stderr, "merge: rg %d/%d  %llu rows  %u mergeable\n",
                    rg + 1, n_rg, (unsigned long long)rows_scanned, seg_count);
    }

    carquet_reader_close(reader);
    *n_segs = seg_count;

    /* Diagnostics: check endpoint extraction quality */
    uint32_t valid_eps = 0, zero_eps = 0, no_wkb = 0;
    for (uint32_t i = 0; i < seg_count; i++) {
        if (segs[i].wkb_len == 0) { no_wkb++; continue; }
        if (segs[i].start_x == 0.0 && segs[i].start_y == 0.0 &&
            segs[i].end_x == 0.0 && segs[i].end_y == 0.0)
            zero_eps++;
        else
            valid_eps++;
    }
    fprintf(stderr, "merge: %u mergeable segments "
            "(endpoints: %u valid, %u zero, %u no-wkb)\n",
            seg_count, valid_eps, zero_eps, no_wkb);

    /* Print first few sample endpoints */
    for (uint32_t i = 0; i < seg_count && i < 5; i++) {
        fprintf(stderr, "merge:   seg[%u] wkb_len=%u "
                "start=(%.6f,%.6f) end=(%.6f,%.6f) cls=%s\n",
                i, segs[i].wkb_len,
                segs[i].start_x, segs[i].start_y,
                segs[i].end_x, segs[i].end_y,
                segs[i].cls ? segs[i].cls : "(null)");
        /* Also show raw WKB type byte */
        if (segs[i].wkb && segs[i].wkb_len >= 5) {
            uint32_t raw_type;
            memcpy(&raw_type, segs[i].wkb + 1, 4);
            fprintf(stderr, "merge:         raw_wkb_type=0x%08X\n", raw_type);
        }
    }

    return segs;
}

/* ── Pass 2: Build spatial index + walk chains ────────────────────────── */

static void pass2_build_chains(seg_info *segs, uint32_t n_segs,
                               chain **out_chains, uint32_t *out_n_chains) {
    *out_chains = NULL;
    *out_n_chains = 0;
    if (n_segs == 0) return;

    /* Build R-tree spatial index */
    fprintf(stderr, "merge: building R-tree for %u endpoints...\n", n_segs * 2);
    struct rtree *tr = rtree_new();
    if (!tr) return;
    rtree_opt_relaxed_atomics(tr);

    for (uint32_t i = 0; i < n_segs; i++) {
        if (segs[i].wkb_len == 0) continue;
        double pt[2];
        pt[0] = segs[i].start_x; pt[1] = segs[i].start_y;
        rtree_insert(tr, pt, NULL, ep_pack(i, 1));
        pt[0] = segs[i].end_x; pt[1] = segs[i].end_y;
        rtree_insert(tr, pt, NULL, ep_pack(i, 0));
    }
    fprintf(stderr, "merge: R-tree: %zu entries\n", rtree_count(tr));

    /* Walk chains */
    uint32_t max_chains = n_segs / 2 + 1;
    chain *chains = calloc(max_chains, sizeof(*chains));
    if (!chains) { rtree_free(tr); return; }

    uint32_t n_chains = 0, total_merged = 0;

    fprintf(stderr, "merge: building chains...\n");
    for (uint32_t i = 0; i < n_segs; i++) {
        if (segs[i].chain_id != -1 || segs[i].wkb_len == 0) continue;

        if ((i & 0xFFFF) == 0 && i > 0)
            fprintf(stderr, "merge: chains %u/%u  %u chains  %u merged\n",
                    i, n_segs, n_chains, total_merged);

        chain c = {0};

        /* Walk backward from start endpoint */
        uint32_t *back_segs = NULL;
        bool *back_rev = NULL;
        uint32_t back_len = 0, back_cap = 0;
        uint32_t cur = i;
        bool cur_use_start = true;  /* which endpoint to follow */
        for (;;) {
            double px = cur_use_start ? segs[cur].start_x : segs[cur].end_x;
            double py = cur_use_start ? segs[cur].start_y : segs[cur].end_y;
            bool nb_is_start;
            uint32_t nb = ep_find_neighbor(tr, segs, cur, px, py, &nb_is_start);
            if (nb == UINT32_MAX || segs[nb].chain_id != -1) break;
            if (back_len >= back_cap) {
                uint32_t new_cap = back_cap ? back_cap * 2 : 8;
                uint32_t *ns = realloc(back_segs, new_cap * sizeof(*ns));
                bool *nr = realloc(back_rev, new_cap * sizeof(*nr));
                if (!ns || !nr) { free(ns); free(nr); break; }
                back_segs = ns; back_rev = nr; back_cap = new_cap;
            }
            back_segs[back_len] = nb;
            back_rev[back_len] = nb_is_start;
            back_len++;
            segs[nb].chain_id = -2;
            cur = nb;
            cur_use_start = !nb_is_start;
            if (back_len >= 50000) break;
        }
        for (uint32_t b = 0; b < back_len; b++)
            segs[back_segs[b]].chain_id = -1;
        for (uint32_t b = back_len; b > 0; b--)
            chain_push(&c, back_segs[b - 1], back_rev[b - 1]);
        free(back_segs); free(back_rev);

        chain_push(&c, i, false);

        /* Walk forward from end endpoint */
        cur = i;
        bool cur_use_end = true;
        for (;;) {
            double px = cur_use_end ? segs[cur].end_x : segs[cur].start_x;
            double py = cur_use_end ? segs[cur].end_y : segs[cur].start_y;
            bool nb_is_start;
            uint32_t nb = ep_find_neighbor(tr, segs, cur, px, py, &nb_is_start);
            if (nb == UINT32_MAX || segs[nb].chain_id != -1) break;
            chain_push(&c, nb, !nb_is_start);
            segs[nb].chain_id = -2;
            cur = nb;
            cur_use_end = nb_is_start ? false : true;
            if (c.length >= 50000) break;
        }

        if (c.length < 2) {
            for (uint32_t ci = 0; ci < c.length; ci++)
                if (segs[c.seg_indices[ci]].chain_id == -2)
                    segs[c.seg_indices[ci]].chain_id = -1;
            chain_free(&c);
            continue;
        }

        int32_t cid = (int32_t)n_chains;
        for (uint32_t ci = 0; ci < c.length; ci++) {
            segs[c.seg_indices[ci]].chain_id = cid;
            segs[c.seg_indices[ci]].chain_pos = (int32_t)ci;
            segs[c.seg_indices[ci]].reversed = c.reversed[ci];
        }
        chains[n_chains++] = c;
        total_merged += c.length;
        if (n_chains >= max_chains) break;
    }

    rtree_free(tr);
    *out_chains = chains;
    *out_n_chains = n_chains;

    /* Chain length histogram */
    uint32_t hist[7] = {0};  /* 1, 2-5, 6-10, 11-50, 51-200, 201-1000, 1001+ */
    for (uint32_t c = 0; c < n_chains; c++) {
        uint32_t len = chains[c].length;
        if (len <= 1)        hist[0]++;
        else if (len <= 5)   hist[1]++;
        else if (len <= 10)  hist[2]++;
        else if (len <= 50)  hist[3]++;
        else if (len <= 200) hist[4]++;
        else if (len <= 1000)hist[5]++;
        else                 hist[6]++;
    }
    fprintf(stderr, "merge: %u chains covering %u segments "
            "(%u standalone, %u output rows)\n",
            n_chains, total_merged,
            n_segs - total_merged,
            n_segs - total_merged + n_chains);
    fprintf(stderr, "merge: chain lengths: "
            "2-5:%u 6-10:%u 11-50:%u 51-200:%u 201-1k:%u 1k+:%u\n",
            hist[1], hist[2], hist[3], hist[4], hist[5], hist[6]);
}

/* ── WKB LineString builder ───────────────────────────────────────────── */

static uint8_t *build_wkb_linestring(const double *xs, const double *ys,
                                     uint32_t n, size_t *out_len) {
    size_t len = 1 + 4 + 4 + (size_t)n * 16;
    uint8_t *buf = malloc(len);
    if (!buf) return NULL;
    size_t off = 0;
    buf[off++] = 1;
    uint32_t type = 2;
    memcpy(buf + off, &type, 4); off += 4;
    memcpy(buf + off, &n, 4); off += 4;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(buf + off, &xs[i], 8); off += 8;
        memcpy(buf + off, &ys[i], 8); off += 8;
    }
    *out_len = len;
    return buf;
}

/* ── Pass 3: Write from memory ────────────────────────────────────────── */

static void write_row(carquet_writer_t *writer,
                      const uint8_t *wkb, int32_t wkb_len,
                      const char *type, const char *subtype, const char *cls,
                      const float bbox[4], bool has_bbox,
                      int32_t min_zoom, bool has_min,
                      int32_t max_zoom, bool has_max,
                      int32_t sort_key, bool has_sort) {
    carquet_byte_array_t ba = { .data = (uint8_t *)wkb, .length = wkb_len };
    (void)carquet_writer_write_batch(writer, 0, &ba, 1, NULL, NULL);
    int16_t def;
    ba.data = type ? (uint8_t *)type : NULL;
    ba.length = type ? (int32_t)strlen(type) : 0;
    def = type ? 1 : 0;
    (void)carquet_writer_write_batch(writer, 1, &ba, 1, &def, NULL);
    ba.data = subtype ? (uint8_t *)subtype : NULL;
    ba.length = subtype ? (int32_t)strlen(subtype) : 0;
    def = subtype ? 1 : 0;
    (void)carquet_writer_write_batch(writer, 2, &ba, 1, &def, NULL);
    ba.data = cls ? (uint8_t *)cls : NULL;
    ba.length = cls ? (int32_t)strlen(cls) : 0;
    def = cls ? 1 : 0;
    (void)carquet_writer_write_batch(writer, 3, &ba, 1, &def, NULL);
    def = has_bbox ? 1 : 0;
    float xmin = has_bbox ? bbox[0] : 0, ymin = has_bbox ? bbox[1] : 0;
    float xmax = has_bbox ? bbox[2] : 0, ymax = has_bbox ? bbox[3] : 0;
    (void)carquet_writer_write_batch(writer, 4, &xmin, 1, &def, NULL);
    (void)carquet_writer_write_batch(writer, 5, &ymin, 1, &def, NULL);
    (void)carquet_writer_write_batch(writer, 6, &xmax, 1, &def, NULL);
    (void)carquet_writer_write_batch(writer, 7, &ymax, 1, &def, NULL);
    int16_t cdef;
    cdef = has_min ? 2 : 0;
    (void)carquet_writer_write_batch(writer, 8, &min_zoom, 1, &cdef, NULL);
    cdef = has_max ? 2 : 0;
    (void)carquet_writer_write_batch(writer, 9, &max_zoom, 1, &cdef, NULL);
    cdef = has_sort ? 2 : 0;
    (void)carquet_writer_write_batch(writer, 10, &sort_key, 1, &cdef, NULL);
}

static bool pass3_write(const char *output_path,
                        seg_info *segs, uint32_t n_segs,
                        chain *chains, uint32_t n_chains) {
    carquet_error_t err = CARQUET_ERROR_INIT;
    carquet_schema_t *schema = carquet_schema_create(&err);
    if (!schema) return false;

    (void)carquet_schema_add_column(schema, "geometry",
        CARQUET_PHYSICAL_BYTE_ARRAY, NULL, CARQUET_REPETITION_REQUIRED, 0, 0);
    (void)carquet_schema_add_column(schema, "type",
        CARQUET_PHYSICAL_BYTE_ARRAY, NULL, CARQUET_REPETITION_OPTIONAL, 0, 0);
    (void)carquet_schema_add_column(schema, "subtype",
        CARQUET_PHYSICAL_BYTE_ARRAY, NULL, CARQUET_REPETITION_OPTIONAL, 0, 0);
    (void)carquet_schema_add_column(schema, "class",
        CARQUET_PHYSICAL_BYTE_ARRAY, NULL, CARQUET_REPETITION_OPTIONAL, 0, 0);
    int32_t bbox_grp = carquet_schema_add_group(schema, "bbox",
        CARQUET_REPETITION_OPTIONAL, 0);
    (void)carquet_schema_add_column(schema, "xmin",
        CARQUET_PHYSICAL_FLOAT, NULL, CARQUET_REPETITION_REQUIRED, 0, bbox_grp);
    (void)carquet_schema_add_column(schema, "ymin",
        CARQUET_PHYSICAL_FLOAT, NULL, CARQUET_REPETITION_REQUIRED, 0, bbox_grp);
    (void)carquet_schema_add_column(schema, "xmax",
        CARQUET_PHYSICAL_FLOAT, NULL, CARQUET_REPETITION_REQUIRED, 0, bbox_grp);
    (void)carquet_schema_add_column(schema, "ymax",
        CARQUET_PHYSICAL_FLOAT, NULL, CARQUET_REPETITION_REQUIRED, 0, bbox_grp);
    int32_t cart_grp = carquet_schema_add_group(schema, "cartography",
        CARQUET_REPETITION_OPTIONAL, 0);
    (void)carquet_schema_add_column(schema, "min_zoom",
        CARQUET_PHYSICAL_INT32, NULL, CARQUET_REPETITION_OPTIONAL, 0, cart_grp);
    (void)carquet_schema_add_column(schema, "max_zoom",
        CARQUET_PHYSICAL_INT32, NULL, CARQUET_REPETITION_OPTIONAL, 0, cart_grp);
    (void)carquet_schema_add_column(schema, "sort_key",
        CARQUET_PHYSICAL_INT32, NULL, CARQUET_REPETITION_OPTIONAL, 0, cart_grp);

    carquet_writer_options_t wopts;
    carquet_writer_options_init(&wopts);
    wopts.compression = CARQUET_COMPRESSION_ZSTD;
    wopts.row_group_size = 16 * 1024 * 1024;  /* 16 MB row groups */
    wopts.created_by = "arpentry_merge";

    carquet_writer_t *writer = carquet_writer_create(
        output_path, schema, &wopts, &err);
    if (!writer) {
        fprintf(stderr, "merge: cannot create writer: %s\n", err.message);
        carquet_schema_free(schema);
        return false;
    }

    typedef struct {
        double *xs, *ys;
        uint32_t n_coords, capacity;
        uint32_t segments_seen;
    } chain_buf;

    chain_buf *cbufs = NULL;
    if (n_chains > 0) {
        cbufs = calloc(n_chains, sizeof(*cbufs));
        if (!cbufs) {
            carquet_writer_abort(writer);
            carquet_schema_free(schema);
            return false;
        }
    }

    uint64_t rows_written = 0;
    uint32_t null_cls = 0, empty_cls = 0, good_cls = 0;
    fprintf(stderr, "merge: writing %u segments (%u chains)...\n",
            n_segs, n_chains);

    for (uint32_t i = 0; i < n_segs; i++) {
        if ((i & 0xFFFFF) == 0 && i > 0)
            fprintf(stderr, "merge: writing %u/%u  %llu rows written\n",
                    i, n_segs, (unsigned long long)rows_written);

        seg_info *s = &segs[i];

        if (s->chain_id < 0) {
            /* Standalone — override min_zoom from class */
            if (s->wkb && s->wkb_len > 0) {
                if (!s->cls) null_cls++;
                else if (s->cls[0] == '\0') empty_cls++;
                else good_cls++;
                int32_t mz = class_min_zoom(s->cls);
                write_row(writer, s->wkb, (int32_t)s->wkb_len,
                          s->type, s->subtype, s->cls,
                          s->bbox, s->has_bbox,
                          mz, true,
                          s->max_zoom, s->has_max_zoom,
                          s->sort_key, s->has_sort_key);
                rows_written++;
            }
        } else {
            uint32_t cid = (uint32_t)s->chain_id;
            chain_buf *cb = &cbufs[cid];

            if (s->wkb && s->wkb_len > 0) {
                arpt_geom geom = {0};
                if (arpt_wkb_parse(s->wkb, (size_t)s->wkb_len, &geom)) {
                    uint32_t needed = cb->n_coords + geom.n_coords;
                    if (needed > cb->capacity) {
                        uint32_t new_cap = cb->capacity ? cb->capacity : 64;
                        while (new_cap < needed) new_cap *= 2;
                        double *nx = realloc(cb->xs, new_cap * sizeof(double));
                        double *ny = realloc(cb->ys, new_cap * sizeof(double));
                        if (nx && ny) {
                            cb->xs = nx; cb->ys = ny; cb->capacity = new_cap;
                        } else {
                            free(nx); free(ny);
                            arpt_geom_free(&geom);
                            continue;
                        }
                    }

                    uint32_t start = 0;
                    if (cb->n_coords > 0 && geom.n_coords > 0) {
                        double fx = s->reversed
                            ? geom.x[geom.n_coords - 1] : geom.x[0];
                        double fy = s->reversed
                            ? geom.y[geom.n_coords - 1] : geom.y[0];
                        double ddx = fx - cb->xs[cb->n_coords - 1];
                        double ddy = fy - cb->ys[cb->n_coords - 1];
                        if (ddx * ddx + ddy * ddy < 1e-10) start = 1;
                    }

                    if (s->reversed) {
                        for (uint32_t k = geom.n_coords - 1 - start; ; k--) {
                            cb->xs[cb->n_coords] = geom.x[k];
                            cb->ys[cb->n_coords] = geom.y[k];
                            cb->n_coords++;
                            if (k == 0) break;
                        }
                    } else {
                        for (uint32_t k = start; k < geom.n_coords; k++) {
                            cb->xs[cb->n_coords] = geom.x[k];
                            cb->ys[cb->n_coords] = geom.y[k];
                            cb->n_coords++;
                        }
                    }
                    arpt_geom_free(&geom);
                }
            }

            cb->segments_seen++;

            if (cb->segments_seen == chains[cid].length) {
                uint32_t first_si = chains[cid].seg_indices[0];
                seg_info *first = &segs[first_si];

                float mbbox[4] = {1e30f, 1e30f, -1e30f, -1e30f};
                for (uint32_t ci = 0; ci < chains[cid].length; ci++) {
                    seg_info *cs = &segs[chains[cid].seg_indices[ci]];
                    if (cs->has_bbox) {
                        if (cs->bbox[0] < mbbox[0]) mbbox[0] = cs->bbox[0];
                        if (cs->bbox[1] < mbbox[1]) mbbox[1] = cs->bbox[1];
                        if (cs->bbox[2] > mbbox[2]) mbbox[2] = cs->bbox[2];
                        if (cs->bbox[3] > mbbox[3]) mbbox[3] = cs->bbox[3];
                    }
                }

                int32_t merged_min_zoom = class_min_zoom(first->cls);

                size_t wkb_len;
                uint8_t *merged_wkb = build_wkb_linestring(
                    cb->xs, cb->ys, cb->n_coords, &wkb_len);
                if (merged_wkb) {
                    write_row(writer, merged_wkb, (int32_t)wkb_len,
                              first->type, first->subtype, first->cls,
                              mbbox, true,
                              merged_min_zoom, true,
                              first->max_zoom, first->has_max_zoom,
                              first->sort_key, first->has_sort_key);
                    free(merged_wkb);
                    rows_written++;
                }
                free(cb->xs); free(cb->ys);
                cb->xs = NULL; cb->ys = NULL;
            }
        }
    }

    free(cbufs);

    (void)carquet_writer_set_key_value(writer, "geo",
        "{\"version\":\"1.0.0\","
        "\"primary_column\":\"geometry\","
        "\"columns\":{\"geometry\":{\"encoding\":\"WKB\","
        "\"geometry_types\":[\"LineString\",\"MultiLineString\"]}}}");

    carquet_status_t st = carquet_writer_close(writer);
    carquet_schema_free(schema);

    fprintf(stderr, "merge: class stats: %u good, %u null, %u empty\n",
            good_cls, null_cls, empty_cls);
    fprintf(stderr, "merge: wrote %llu rows to %s\n",
            (unsigned long long)rows_written, output_path);
    return st == CARQUET_OK;
}

/* ── Public API ───────────────────────────────────────────────────────── */

bool arpt_merge_run(const char *input_path,
                    const char *output_path,
                    const double *bbox) {
    (void)bbox;
    fprintf(stderr, "merge: reading %s\n", input_path);

    /* Pass 1: single-pass read */
    uint32_t n_segs = 0;
    seg_info *segs = pass1_read(input_path, &n_segs);
    if (!segs) n_segs = 0;

    /* Pass 2: spatial merge */
    chain *chains = NULL;
    uint32_t n_chains = 0;
    if (n_segs > 0)
        pass2_build_chains(segs, n_segs, &chains, &n_chains);

    /* Pass 3: write */
    bool ok = pass3_write(output_path, segs, n_segs, chains, n_chains);

    for (uint32_t c = 0; c < n_chains; c++) chain_free(&chains[c]);
    free(chains);
    if (segs) {
        for (uint32_t i = 0; i < n_segs; i++) seg_info_free(&segs[i]);
        free(segs);
    }
    return ok;
}
