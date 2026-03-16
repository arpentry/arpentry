#include "clip.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Clip buffer in pixels.  Polygons and lines are clipped with this
 * much extra beyond the tile bounds, creating overlap between adjacent
 * tiles that prevents visible seams.  Standard in all vector tile
 * systems (tippecanoe, planetiler, geojson-vt). */
#define CLIP_BUFFER_PX  8
#define TILE_PIXELS     256

/* Equirectangular tile bounds in WGS84 degrees for tile (z, x, y).
   Grid: 2^(z+1) columns × 2^z rows, y=0 at south. */
static arpt_bounds tile_bounds(int z, int tx, int ty) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;
    double lon_span = 360.0 / (double)n_cols;
    double lat_span = 180.0 / (double)n_rows;
    double w = -180.0 + (double)tx * lon_span;
    double s = -90.0 + (double)ty * lat_span;
    return (arpt_bounds){w, s, w + lon_span, s + lat_span};
}

/* Tile bounds extended by a clip buffer on each side. */
static arpt_bounds tile_bounds_buffered(int z, int tx, int ty) {
    arpt_bounds b = tile_bounds(z, tx, ty);
    double buf_x = (b.max_x - b.min_x) * ((double)CLIP_BUFFER_PX / TILE_PIXELS);
    double buf_y = (b.max_y - b.min_y) * ((double)CLIP_BUFFER_PX / TILE_PIXELS);
    b.min_x -= buf_x;
    b.max_x += buf_x;
    b.min_y -= buf_y;
    b.max_y += buf_y;
    return b;
}

/* ---------- Dynamic arrays ---------- */

typedef struct {
    double *data;
    uint32_t len;
    uint32_t cap;
} darray;

static void da_init(darray *a) {
    a->data = NULL;
    a->len = 0;
    a->cap = 0;
}

static bool da_push(darray *a, double v) {
    if (a->len == a->cap) {
        uint32_t nc = a->cap ? a->cap * 2 : 16;
        double *p = realloc(a->data, nc * sizeof(double));
        if (!p) return false;
        a->data = p;
        a->cap = nc;
    }
    a->data[a->len++] = v;
    return true;
}

static void da_free(darray *a) {
    free(a->data);
    a->data = NULL;
    a->len = 0;
    a->cap = 0;
}

typedef struct {
    uint32_t *data;
    uint32_t len;
    uint32_t cap;
} u32array;

static void u32a_init(u32array *a) {
    a->data = NULL;
    a->len = 0;
    a->cap = 0;
}

static bool u32a_push(u32array *a, uint32_t v) {
    if (a->len == a->cap) {
        uint32_t nc = a->cap ? a->cap * 2 : 16;
        uint32_t *p = realloc(a->data, nc * sizeof(uint32_t));
        if (!p) return false;
        a->data = p;
        a->cap = nc;
    }
    a->data[a->len++] = v;
    return true;
}

static void u32a_free(u32array *a) {
    free(a->data);
    a->data = NULL;
    a->len = 0;
    a->cap = 0;
}

/* ---------- Point clipping ---------- */

static void clip_points(const arpt_geom *geom, int z,
                        arpt_tile_cb cb, void *ctx) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;

    for (uint32_t i = 0; i < geom->n_coords; i++) {
        double px = geom->x[i];
        double py = geom->y[i];

        /* Clamp to valid range */
        if (px < -180.0) px = -180.0;
        if (px > 180.0)  px = 180.0;
        if (py < -90.0)  py = -90.0;
        if (py > 90.0)   py = 90.0;

        /* Determine which tile this point falls into (equirectangular) */
        int tx = (int)floor((px + 180.0) / 360.0 * (double)n_cols);
        int ty = (int)floor((py + 90.0) / 180.0 * (double)n_rows);

        /* Clamp to valid range */
        if (tx < 0) tx = 0;
        if (tx >= n_cols) tx = n_cols - 1;
        if (ty < 0) ty = 0;
        if (ty >= n_rows) ty = n_rows - 1;

        arpt_geom clipped = {0};
        clipped.type = 1; /* Point */
        clipped.x = malloc(sizeof(double));
        clipped.y = malloc(sizeof(double));
        if (!clipped.x || !clipped.y) {
            free(clipped.x);
            free(clipped.y);
            continue;
        }
        clipped.x[0] = px;
        clipped.y[0] = py;
        clipped.n_coords = 1;

        cb(z, tx, ty, &clipped, ctx);
        arpt_geom_free(&clipped);
    }
}

/* ---------- Unified slab clipper (geojson-vt approach) ----------
 *
 * Clips a coordinate sequence against two parallel axis-aligned lines
 * (a "slab") [k1, k2] along a given axis (0=x, 1=y).
 *
 * For lines (is_polygon=false): breaks into separate segments on exit,
 *   recording each segment start in seg_starts.
 * For polygons (is_polygon=true): keeps one continuous ring, closes it
 *   at the end if needed.
 *
 * Rectangle clipping = two passes: x-slab then y-slab. */

/* Push a vertex, suppressing consecutive duplicates. */
static void da_push_dedup(darray *out_x, darray *out_y, double px, double py) {
    if (out_x->len > 0 &&
        out_x->data[out_x->len - 1] == px &&
        out_y->data[out_y->len - 1] == py)
        return;
    da_push(out_x, px);
    da_push(out_y, py);
}

static void slab_intersect(darray *out_x, darray *out_y,
                           double ax, double ay, double bx, double by,
                           double k, int axis) {
    if (axis) {
        double t = (k - ay) / (by - ay);
        da_push_dedup(out_x, out_y, ax + (bx - ax) * t, k);
    } else {
        double t = (k - ax) / (bx - ax);
        da_push_dedup(out_x, out_y, k, ay + (by - ay) * t);
    }
}

/* Clip n vertices against slab [k1, k2] on the given axis.
 * Processes n-1 edges: (0,1), (1,2), ..., (n-2, n-1).
 * seg_starts is optional (may be NULL for polygon mode). */
static void clip_slab(const double *rx, const double *ry, uint32_t n,
                      double k1, double k2, int axis, bool is_polygon,
                      darray *out_x, darray *out_y,
                      u32array *seg_starts) {
    if (n < 2) return;

    uint32_t slice_start = out_x->len;

    for (uint32_t i = 0; i + 1 < n; i++) {
        double ax = rx[i],     ay = ry[i];
        double bx = rx[i + 1], by = ry[i + 1];
        double a = axis ? ay : ax;
        double b = axis ? by : bx;
        bool exited = false;

        if (a < k1) {
            if (b > k1) {
                slab_intersect(out_x, out_y, ax, ay, bx, by, k1, axis);
            }
        } else if (a > k2) {
            if (b < k2) {
                slab_intersect(out_x, out_y, ax, ay, bx, by, k2, axis);
            }
        } else {
            da_push_dedup(out_x, out_y, ax, ay);
        }

        if (b < k1 && a >= k1) {
            slab_intersect(out_x, out_y, ax, ay, bx, by, k1, axis);
            exited = true;
        }
        if (b > k2 && a <= k2) {
            slab_intersect(out_x, out_y, ax, ay, bx, by, k2, axis);
            exited = true;
        }

        if (!is_polygon && exited) {
            uint32_t len = out_x->len - slice_start;
            if (len >= 2 && seg_starts) {
                u32a_push(seg_starts, slice_start);
            } else {
                out_x->len = slice_start;
                out_y->len = slice_start;
            }
            slice_start = out_x->len;
        }
    }

    /* Add the last point if inside */
    {
        double a = axis ? ry[n - 1] : rx[n - 1];
        if (a >= k1 && a <= k2) {
            da_push_dedup(out_x, out_y, rx[n - 1], ry[n - 1]);
        }
    }

    if (is_polygon) {
        /* Close the ring if endpoints don't match */
        uint32_t len = out_x->len - slice_start;
        if (len >= 3) {
            uint32_t last = out_x->len - 1;
            if (out_x->data[slice_start] != out_x->data[last] ||
                out_y->data[slice_start] != out_y->data[last]) {
                da_push(out_x, out_x->data[slice_start]);
                da_push(out_y, out_y->data[slice_start]);
            }
        } else {
            /* Degenerate — revert */
            out_x->len = slice_start;
            out_y->len = slice_start;
        }
    } else {
        /* Push final line segment */
        uint32_t len = out_x->len - slice_start;
        if (len >= 2 && seg_starts) {
            u32a_push(seg_starts, slice_start);
        } else {
            out_x->len = slice_start;
            out_y->len = slice_start;
        }
    }
}

/* ---------- Helpers ---------- */

/* Compute bounding box of a coordinate range and convert to tile range. */
static void coords_to_tile_range(const double *x, const double *y,
                                  uint32_t start, uint32_t count,
                                  int n_cols, int n_rows,
                                  int *tx_min, int *tx_max,
                                  int *ty_min, int *ty_max) {
    double gmin_x = x[start], gmax_x = x[start];
    double gmin_y = y[start], gmax_y = y[start];
    for (uint32_t i = start + 1; i < start + count; i++) {
        if (x[i] < gmin_x) gmin_x = x[i];
        if (x[i] > gmax_x) gmax_x = x[i];
        if (y[i] < gmin_y) gmin_y = y[i];
        if (y[i] > gmax_y) gmax_y = y[i];
    }

    if (gmin_x < -180.0) gmin_x = -180.0;
    if (gmax_x > 180.0)  gmax_x = 180.0;
    if (gmin_y < -90.0)  gmin_y = -90.0;
    if (gmax_y > 90.0)   gmax_y = 90.0;

    *tx_min = (int)floor((gmin_x + 180.0) / 360.0 * (double)n_cols);
    *tx_max = (int)floor((gmax_x + 180.0) / 360.0 * (double)n_cols);
    *ty_min = (int)floor((gmin_y + 90.0) / 180.0 * (double)n_rows);
    *ty_max = (int)floor((gmax_y + 90.0) / 180.0 * (double)n_rows);

    if (*tx_min < 0) *tx_min = 0;
    if (*tx_max >= n_cols) *tx_max = n_cols - 1;
    if (*ty_min < 0) *ty_min = 0;
    if (*ty_max >= n_rows) *ty_max = n_rows - 1;
}

/* Get the start index of segment si and the end (= start of next). */
static void seg_range(const u32array *segs, uint32_t si, uint32_t total,
                      uint32_t *start, uint32_t *end) {
    *start = segs->data[si];
    *end = (si + 1 < segs->len) ? segs->data[si + 1] : total;
}

/* ---------- Line clipping ---------- */

static void clip_lines(const arpt_geom *geom, int z,
                       arpt_tile_cb cb, void *ctx) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;

    int tx_min, tx_max, ty_min, ty_max;
    coords_to_tile_range(geom->x, geom->y, 0, geom->n_coords,
                         n_cols, n_rows, &tx_min, &tx_max, &ty_min, &ty_max);

    uint32_t n_lines = 1;
    if (geom->type == 5 && geom->n_offsets > 1)
        n_lines = geom->n_offsets - 1;

    for (int tx = tx_min; tx <= tx_max; tx++) {
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds_buffered(z, tx, ty);

            darray out_x, out_y;
            u32array segs;
            da_init(&out_x);
            da_init(&out_y);
            u32a_init(&segs);

            for (uint32_t li = 0; li < n_lines; li++) {
                uint32_t start = 0, end = geom->n_coords;
                if (geom->offsets && geom->n_offsets > 1) {
                    start = geom->offsets[li];
                    end = geom->offsets[li + 1];
                }
                uint32_t ln = end - start;
                if (ln < 2) continue;

                /* Pass 1: clip against x-slab */
                darray mx, my;
                u32array msegs;
                da_init(&mx);
                da_init(&my);
                u32a_init(&msegs);
                clip_slab(geom->x + start, geom->y + start, ln,
                          tb.min_x, tb.max_x, 0, false,
                          &mx, &my, &msegs);

                /* Pass 2: clip each x-clipped segment against y-slab */
                for (uint32_t si = 0; si < msegs.len; si++) {
                    uint32_t ss, se;
                    seg_range(&msegs, si, mx.len, &ss, &se);
                    uint32_t sn = se - ss;
                    if (sn >= 2) {
                        clip_slab(mx.data + ss, my.data + ss, sn,
                                  tb.min_y, tb.max_y, 1, false,
                                  &out_x, &out_y, &segs);
                    }
                }

                da_free(&mx);
                da_free(&my);
                u32a_free(&msegs);
            }

            /* Emit each segment */
            for (uint32_t si = 0; si < segs.len; si++) {
                uint32_t ss, se;
                seg_range(&segs, si, out_x.len, &ss, &se);
                uint32_t sn = se - ss;
                if (sn >= 2) {
                    arpt_geom clipped = {0};
                    clipped.type = 2; /* LineString */
                    clipped.x = out_x.data + ss;
                    clipped.y = out_y.data + ss;
                    clipped.n_coords = sn;
                    cb(z, tx, ty, &clipped, ctx);
                }
            }

            da_free(&out_x);
            da_free(&out_y);
            u32a_free(&segs);
        }
    }
}

/* ---------- Polygon clipping ---------- */

/* --- Boundary walk helpers for polygon ring splitting --- */

/* Tolerance for boundary membership tests.  slab_intersect places one
 * coordinate exactly on k, so this only needs to handle FP rounding
 * on the OTHER coordinate (computed via interpolation). */
#define BNDRY_EPS 1e-10

/* Check if a point is on the clip rect boundary. */
static bool on_boundary(double px, double py, const arpt_bounds *b) {
    return (fabs(px - b->min_x) < BNDRY_EPS ||
            fabs(px - b->max_x) < BNDRY_EPS ||
            fabs(py - b->min_y) < BNDRY_EPS ||
            fabs(py - b->max_y) < BNDRY_EPS);
}

/* CCW perimeter parameter 0..4 for a point on the clip rect boundary.
 * 0-1 = bottom (min_y, left→right), 1-2 = right (max_x, bottom→top),
 * 2-3 = top (max_y, right→left), 3-4 = left (min_x, top→bottom). */
static double perim_param(double px, double py, const arpt_bounds *b) {
    double w = b->max_x - b->min_x;
    double h = b->max_y - b->min_y;
    if (w < 1e-20 || h < 1e-20) return 0.0;

    /* Bottom edge */
    if (fabs(py - b->min_y) < BNDRY_EPS)
        return (px - b->min_x) / w;
    /* Right edge */
    if (fabs(px - b->max_x) < BNDRY_EPS)
        return 1.0 + (py - b->min_y) / h;
    /* Top edge */
    if (fabs(py - b->max_y) < BNDRY_EPS)
        return 2.0 + (b->max_x - px) / w;
    /* Left edge */
    if (fabs(px - b->min_x) < BNDRY_EPS)
        return 3.0 + (b->max_y - py) / h;
    return 0.0; /* shouldn't happen */
}

/* Emit boundary walk vertices (corners) from perimeter position t_from
 * to t_to.  If ccw is true, walks CCW (increasing t); if false, walks
 * CW (decreasing t).  Does NOT emit the start or end points. */
static void walk_boundary(double t_from, double t_to, bool ccw,
                           const arpt_bounds *b,
                           darray *ox, darray *oy) {
    double corner_x[4], corner_y[4];
    corner_x[0] = b->min_x; corner_y[0] = b->min_y; /* BL at t=0 */
    corner_x[1] = b->max_x; corner_y[1] = b->min_y; /* BR at t=1 */
    corner_x[2] = b->max_x; corner_y[2] = b->max_y; /* TR at t=2 */
    corner_x[3] = b->min_x; corner_y[3] = b->max_y; /* TL at t=3 */

    if (ccw) {
        /* Walk CCW (increasing t) from t_from to t_to, emitting corners. */
        double to = t_to;
        if (to <= t_from) to += 4.0;
        /* Start from the first corner AFTER t_from */
        int start_c = (int)floor(t_from) + 1;
        for (int i = 0; i < 4; i++) {
            int c = (start_c + i) % 4;
            double ct = (double)(start_c + i);
            if (ct >= to) break;
            da_push_dedup(ox, oy, corner_x[c], corner_y[c]);
        }
    } else {
        /* Walk CW (decreasing t) from t_from to t_to, emitting corners. */
        double to = t_to;
        if (to >= t_from) to -= 4.0;
        /* Start from the first corner BEFORE t_from */
        int start_c = (int)ceil(t_from) - 1;
        for (int i = 0; i < 4; i++) {
            int c = ((start_c - i) % 4 + 4) % 4;
            double ct = (double)(start_c - i);
            if (ct <= to) break;
            da_push_dedup(ox, oy, corner_x[c], corner_y[c]);
        }
    }
}

/* A boundary crossing: entry or exit point on the clip rectangle. */
typedef struct {
    double x, y;
    double perim;     /* CCW perimeter parameter 0..4 */
    uint32_t idx;     /* vertex index in the SH ring */
    bool is_entry;    /* true = entry, false = exit */
} bcross;

static int bcross_cmp(const void *a, const void *b) {
    double pa = ((const bcross *)a)->perim;
    double pb = ((const bcross *)b)->perim;
    return (pa > pb) - (pa < pb);
}

/* Emit a single ring to the output arrays if it has valid area. */
static void emit_ring(const double *rx, const double *ry, uint32_t n,
                       darray *out_x, darray *out_y,
                       u32array *ring_starts) {
    if (n < 4) return;
    /* Reject zero-area (collinear) rings */
    double area2 = 0.0;
    for (uint32_t i = 0; i + 1 < n; i++) {
        area2 += rx[i] * ry[i + 1] - rx[i + 1] * ry[i];
    }
    if (area2 < 0) area2 = -area2;
    if (area2 <= 1e-20) return;

    uint32_t ring_start = out_x->len;
    for (uint32_t i = 0; i < n; i++) {
        da_push(out_x, rx[i]);
        da_push(out_y, ry[i]);
    }
    u32a_push(ring_starts, ring_start);
}

/* Split a clipped ring at boundary crossings and produce multiple
 * non-self-intersecting rings with proper boundary walks.
 *
 * The SH/slab clip produces a single ring that may self-overlap
 * along the clip boundary when the polygon exits and re-enters
 * the same boundary edge at multiple points.  This function detects
 * the entry/exit crossings, pairs them in CCW order, and assembles
 * separate rings with correct boundary-walk segments. */
static void split_ring(const double *rx, const double *ry, uint32_t n,
                        const arpt_bounds *b,
                        darray *out_x, darray *out_y,
                        u32array *ring_starts) {
    if (n < 4) return;

    /* Step 1: classify vertices and find crossings */
    bool *is_bnd = calloc(n, sizeof(bool));
    if (!is_bnd) return;
    for (uint32_t i = 0; i < n; i++)
        is_bnd[i] = on_boundary(rx[i], ry[i], b);

    /* Step 2: find entry/exit transitions.
     * Interior → boundary = EXIT (the boundary vertex)
     * Boundary → interior = ENTRY (the boundary vertex) */
    bcross *crosses = NULL;
    uint32_t n_crosses = 0, cross_cap = 0;

    uint32_t n_unique = n - 1; /* exclude closing vertex */
    for (uint32_t i = 0; i < n_unique; i++) {
        uint32_t next = (i + 1) % n_unique;
        if (!is_bnd[i] && is_bnd[next]) {
            /* EXIT at next */
            if (n_crosses == cross_cap) {
                cross_cap = cross_cap ? cross_cap * 2 : 8;
                crosses = realloc(crosses, cross_cap * sizeof(bcross));
            }
            crosses[n_crosses++] = (bcross){
                rx[next], ry[next],
                perim_param(rx[next], ry[next], b),
                next, false
            };
        }
        if (is_bnd[i] && !is_bnd[next]) {
            /* ENTRY at i */
            if (n_crosses == cross_cap) {
                cross_cap = cross_cap ? cross_cap * 2 : 8;
                crosses = realloc(crosses, cross_cap * sizeof(bcross));
            }
            crosses[n_crosses++] = (bcross){
                rx[i], ry[i],
                perim_param(rx[i], ry[i], b),
                i, true
            };
        }
    }

    /* If no crossings (all interior or all boundary), emit as single ring */
    if (n_crosses == 0 || n_crosses < 2) {
        emit_ring(rx, ry, n, out_x, out_y, ring_starts);
        free(is_bnd);
        free(crosses);
        return;
    }

    /* Step 3: sort crossings by CCW perimeter parameter */
    qsort(crosses, n_crosses, sizeof(bcross), bcross_cmp);

    /* Verify equal counts of entries and exits. */
    uint32_t n_exits = 0, n_entries = 0;
    for (uint32_t i = 0; i < n_crosses; i++) {
        if (crosses[i].is_entry) n_entries++;
        else n_exits++;
    }
    if (n_exits != n_entries || n_exits == 0) {
        emit_ring(rx, ry, n, out_x, out_y, ring_starts);
        free(is_bnd);
        free(crosses);
        return;
    }

    /* Detect ring winding: positive area = CCW, negative = CW.
     * For CW rings (holes), we pair exits with the PREVIOUS entry
     * in the sorted list instead of the next. */
    double ring_area = 0.0;
    for (uint32_t i = 0; i + 1 < n; i++) {
        ring_area += rx[i] * ry[i + 1] - rx[i + 1] * ry[i];
    }
    bool is_ccw = (ring_area > 0.0);

    /* Step 4: pair each exit with the adjacent entry.
     * CCW rings: exit → next entry in CCW order.
     * CW rings: exit → previous entry in CCW order (= next in CW). */
    uint32_t *exit_to_entry = malloc(n_crosses * sizeof(uint32_t));
    uint32_t *entry_to_exit = malloc(n_crosses * sizeof(uint32_t));
    if (!exit_to_entry || !entry_to_exit) {
        emit_ring(rx, ry, n, out_x, out_y, ring_starts);
        free(is_bnd); free(crosses);
        free(exit_to_entry); free(entry_to_exit);
        return;
    }
    for (uint32_t i = 0; i < n_crosses; i++) {
        exit_to_entry[i] = UINT32_MAX;
        entry_to_exit[i] = UINT32_MAX;
    }
    for (uint32_t i = 0; i < n_crosses; i++) {
        if (crosses[i].is_entry) continue; /* skip entries */
        /* Find adjacent entry in the appropriate direction. */
        int step = is_ccw ? 1 : -1;
        for (uint32_t j = 1; j <= n_crosses; j++) {
            uint32_t ci = (uint32_t)((int)i + step * (int)j +
                          (int)n_crosses * 2) % n_crosses;
            if (crosses[ci].is_entry) {
                exit_to_entry[i] = ci;
                entry_to_exit[ci] = i;
                break;
            }
        }
    }

    /* Step 5: assemble rings.
     * For each unvisited entry, trace: entry → interior → exit → boundary
     * walk → next entry → ... until we return to the starting entry. */
    bool *visited_entry = calloc(n_crosses, sizeof(bool));
    darray ring_x, ring_y;
    da_init(&ring_x);
    da_init(&ring_y);

    for (uint32_t ci = 0; ci < n_crosses; ci++) {
        if (!crosses[ci].is_entry || visited_entry[ci]) continue;

        ring_x.len = 0;
        ring_y.len = 0;

        uint32_t start_ci = ci;
        uint32_t cur_entry_ci = ci;

        uint32_t max_iters = n_crosses + 1;
        uint32_t iter = 0;
        do {
            if (iter++ > max_iters) break;
            visited_entry[cur_entry_ci] = true;

            /* Trace interior arc from this entry to the next exit.
             * The entry vertex index is crosses[cur_entry_ci].idx.
             * Walk the ring from there until we hit an exit crossing. */
            uint32_t vi = crosses[cur_entry_ci].idx;
            /* Emit the entry vertex */
            da_push(&ring_x, rx[vi]);
            da_push(&ring_y, ry[vi]);

            /* Walk forward in the ring, emitting interior vertices,
             * until we reach an exit crossing. */
            bool found_exit = false;
            for (uint32_t step = 0; step < n_unique; step++) {
                vi = (vi + 1) % n_unique;
                da_push(&ring_x, rx[vi]);
                da_push(&ring_y, ry[vi]);

                /* Check if this vertex is an exit crossing */
                uint32_t exit_ci = UINT32_MAX;
                for (uint32_t k = 0; k < n_crosses; k++) {
                    if (!crosses[k].is_entry && crosses[k].idx == vi) {
                        exit_ci = k;
                        break;
                    }
                }
                if (exit_ci != UINT32_MAX) {
                    /* Walk boundary from this exit to the paired entry.
                     * CCW rings walk CCW; CW rings (holes) walk CW. */
                    uint32_t next_entry_ci = exit_to_entry[exit_ci];
                    if (next_entry_ci == UINT32_MAX) break;
                    walk_boundary(crosses[exit_ci].perim,
                                  crosses[next_entry_ci].perim,
                                  is_ccw, b, &ring_x, &ring_y);
                    cur_entry_ci = next_entry_ci;
                    found_exit = true;
                    break;
                }
            }
            if (!found_exit) break; /* safety: couldn't find exit */
        } while (cur_entry_ci != start_ci);

        /* Close the ring */
        if (ring_x.len >= 3) {
            da_push(&ring_x, ring_x.data[0]);
            da_push(&ring_y, ring_y.data[0]);
            emit_ring(ring_x.data, ring_y.data, ring_x.len,
                      out_x, out_y, ring_starts);
        }
    }

    da_free(&ring_x);
    da_free(&ring_y);
    free(visited_entry);
    free(exit_to_entry);
    free(entry_to_exit);
    free(is_bnd);
    free(crosses);
}

/* Clip a single closed ring against a rectangle.
 * Input: n vertices of a closed ring (first == last).
 * Output: one or more closed rings appended to out_x/out_y;
 *         ring_starts receives the start index of each ring produced.
 *
 * Uses two-pass slab clipping (Sutherland-Hodgman) to get the correct
 * vertices, then splits the result at boundary crossings to eliminate
 * self-intersections from re-entrant polygons. */
static void clip_ring_rect(const double *rx, const double *ry, uint32_t n,
                            const arpt_bounds *b,
                            darray *out_x, darray *out_y,
                            u32array *ring_starts) {
    if (n < 4) return; /* need at least 3 unique + closing */

    /* Pass 1: clip against x-slab */
    darray mx, my;
    da_init(&mx);
    da_init(&my);
    clip_slab(rx, ry, n, b->min_x, b->max_x, 0, true, &mx, &my, NULL);

    if (mx.len < 4) {
        da_free(&mx);
        da_free(&my);
        return;
    }

    /* Pass 2: clip against y-slab */
    darray sx, sy;
    da_init(&sx);
    da_init(&sy);
    clip_slab(mx.data, my.data, mx.len, b->min_y, b->max_y, 1, true,
              &sx, &sy, NULL);

    da_free(&mx);
    da_free(&my);

    if (sx.len < 4) {
        da_free(&sx);
        da_free(&sy);
        return;
    }

    /* Split the SH result at boundary crossings to produce
     * non-self-intersecting output rings. */
    split_ring(sx.data, sy.data, sx.len, b, out_x, out_y, ring_starts);

    da_free(&sx);
    da_free(&sy);
}

/* Clip a set of rings (one polygon part) to tiles at the given zoom.
 * first_ring is the index into geom->offsets; n_rings is the count. */
static void clip_polygon_part(const arpt_geom *geom,
                               uint32_t first_ring, uint32_t n_rings,
                               int z, arpt_tile_cb cb, void *ctx) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;

    /* Compute bounding box of this polygon part */
    uint32_t coord_start, coord_end;
    if (geom->offsets && geom->n_offsets > 1) {
        coord_start = geom->offsets[first_ring];
        coord_end = geom->offsets[first_ring + n_rings];
    } else {
        coord_start = 0;
        coord_end = geom->n_coords;
    }
    uint32_t coord_count = coord_end - coord_start;
    if (coord_count < 3) return;

    int tx_min, tx_max, ty_min, ty_max;
    coords_to_tile_range(geom->x, geom->y, coord_start, coord_count,
                         n_cols, n_rows, &tx_min, &tx_max, &ty_min, &ty_max);

    for (int tx = tx_min; tx <= tx_max; tx++) {
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds_buffered(z, tx, ty);

            darray out_x, out_y;
            u32array ring_starts;
            da_init(&out_x);
            da_init(&out_y);
            u32a_init(&ring_starts);

            for (uint32_t ri = 0; ri < n_rings; ri++) {
                uint32_t rstart, rend;
                if (geom->offsets && geom->n_offsets > 1) {
                    rstart = geom->offsets[first_ring + ri];
                    rend = geom->offsets[first_ring + ri + 1];
                } else {
                    rstart = 0;
                    rend = geom->n_coords;
                }
                uint32_t ring_n = rend - rstart;
                if (ring_n < 4) continue;

                clip_ring_rect(geom->x + rstart, geom->y + rstart,
                               ring_n, &tb, &out_x, &out_y, &ring_starts);
            }

            if (ring_starts.len > 0 && out_x.len >= 4) {
                uint32_t n_clipped_rings = ring_starts.len;
                uint32_t *offsets =
                    malloc((n_clipped_rings + 1) * sizeof(*offsets));
                if (offsets) {
                    for (uint32_t i = 0; i < n_clipped_rings; i++) {
                        offsets[i] = ring_starts.data[i];
                    }
                    offsets[n_clipped_rings] = out_x.len;

                    arpt_geom clipped = {0};
                    clipped.type = 3; /* Polygon */
                    clipped.x = out_x.data;
                    clipped.y = out_y.data;
                    clipped.n_coords = out_x.len;
                    clipped.offsets = offsets;
                    clipped.n_offsets = n_clipped_rings + 1;

                    cb(z, tx, ty, &clipped, ctx);
                    free(offsets);
                }
                free(out_x.data);
                free(out_y.data);
            } else {
                da_free(&out_x);
                da_free(&out_y);
            }
            u32a_free(&ring_starts);
        }
    }
}

static void clip_polygons(const arpt_geom *geom, int z,
                          arpt_tile_cb cb, void *ctx) {
    if (geom->type == 6 && geom->parts && geom->n_parts > 0) {
        /* MultiPolygon: clip each polygon part independently so that
         * rings from different polygons are not mixed together. */
        uint32_t total_rings = geom->n_offsets > 0 ? geom->n_offsets - 1 : 0;
        for (uint32_t pi = 0; pi < geom->n_parts; pi++) {
            uint32_t first_ring = geom->parts[pi];
            uint32_t last_ring = (pi + 1 < geom->n_parts)
                ? geom->parts[pi + 1] : total_rings;
            uint32_t n_rings = last_ring - first_ring;
            if (n_rings > 0) {
                clip_polygon_part(geom, first_ring, n_rings, z, cb, ctx);
            }
        }
    } else {
        /* Single Polygon: all rings belong to one polygon */
        uint32_t n_rings = geom->n_offsets > 0 ? geom->n_offsets - 1 : 1;
        clip_polygon_part(geom, 0, n_rings, z, cb, ctx);
    }
}

void arpt_assign_tiles(const arpt_geom *geom, int zoom,
                       arpt_tile_cb cb, void *ctx) {
    if (!geom || !cb) return;
    if (geom->n_coords == 0) return;
    if (zoom < 0) return;

    switch (geom->type) {
    case 1: /* Point */
    case 4: /* MultiPoint */
        clip_points(geom, zoom, cb, ctx);
        break;
    case 2: /* LineString */
    case 5: /* MultiLineString */
        clip_lines(geom, zoom, cb, ctx);
        break;
    case 3: /* Polygon */
    case 6: /* MultiPolygon */
        clip_polygons(geom, zoom, cb, ctx);
        break;
    default:
        break;
    }
}
