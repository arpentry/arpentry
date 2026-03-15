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

/* ---------- Liang-Barsky line clipping ---------- */

static bool liang_barsky(double x0, double y0, double x1, double y1,
                         const arpt_bounds *b,
                         double *cx0, double *cy0, double *cx1, double *cy1) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - b->min_x, b->max_x - x0, y0 - b->min_y, b->max_y - y0};
    double t0 = 0.0, t1 = 1.0;

    for (int i = 0; i < 4; i++) {
        if (p[i] == 0.0) {
            if (q[i] < 0.0) return false;
        } else {
            double r = q[i] / p[i];
            if (p[i] < 0.0) {
                if (r > t1) return false;
                if (r > t0) t0 = r;
            } else {
                if (r < t0) return false;
                if (r < t1) t1 = r;
            }
        }
    }

    *cx0 = x0 + t0 * dx;
    *cy0 = y0 + t0 * dy;
    *cx1 = x0 + t1 * dx;
    *cy1 = y0 + t1 * dy;
    return true;
}

/* Dynamic double array for building clipped coordinates. */
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

/* Dynamic uint32 array for tracking segment/ring offsets. */
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

/* ---------- Line clipping ---------- */

/* Clip line segments to left+right edges of a column strip using
 * Liang-Barsky.  Tracks segment boundaries in out_seg so callers
 * can distinguish disconnected groups. */
static void clip_lines_to_strip(const arpt_geom *geom, uint32_t n_lines,
                                const arpt_bounds *strip,
                                darray *out_x, darray *out_y,
                                u32array *out_seg) {
    /* Use a strip that is unbounded in y for the column pass */
    arpt_bounds col = {strip->min_x, -90.0, strip->max_x, 90.0};
    for (uint32_t li = 0; li < n_lines; li++) {
        uint32_t start = 0, end = geom->n_coords;
        if (geom->offsets && geom->n_offsets > 1) {
            start = geom->offsets[li];
            end = geom->offsets[li + 1];
        }
        for (uint32_t i = start; i + 1 < end; i++) {
            double cx0, cy0, cx1, cy1;
            if (liang_barsky(geom->x[i], geom->y[i],
                             geom->x[i + 1], geom->y[i + 1],
                             &col, &cx0, &cy0, &cx1, &cy1)) {
                bool connected = (out_x->len > 0 &&
                                  out_x->data[out_x->len - 1] == cx0 &&
                                  out_y->data[out_y->len - 1] == cy0);
                if (!connected) {
                    u32a_push(out_seg, out_x->len);
                    da_push(out_x, cx0);
                    da_push(out_y, cy0);
                }
                da_push(out_x, cx1);
                da_push(out_y, cy1);
            }
        }
    }
}

static void clip_lines(const arpt_geom *geom, int z,
                       arpt_tile_cb cb, void *ctx) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;

    /* Compute the bounding box of the geometry to limit tile iteration */
    double gmin_x = geom->x[0], gmax_x = geom->x[0];
    double gmin_y = geom->y[0], gmax_y = geom->y[0];
    for (uint32_t i = 1; i < geom->n_coords; i++) {
        if (geom->x[i] < gmin_x) gmin_x = geom->x[i];
        if (geom->x[i] > gmax_x) gmax_x = geom->x[i];
        if (geom->y[i] < gmin_y) gmin_y = geom->y[i];
        if (geom->y[i] > gmax_y) gmax_y = geom->y[i];
    }

    /* Clamp to valid range */
    if (gmin_x < -180.0) gmin_x = -180.0;
    if (gmax_x > 180.0)  gmax_x = 180.0;
    if (gmin_y < -90.0)  gmin_y = -90.0;
    if (gmax_y > 90.0)   gmax_y = 90.0;

    /* Convert geom bbox to tile range (equirectangular) */
    int tx_min = (int)floor((gmin_x + 180.0) / 360.0 * (double)n_cols);
    int tx_max = (int)floor((gmax_x + 180.0) / 360.0 * (double)n_cols);
    int ty_min = (int)floor((gmin_y + 90.0) / 180.0 * (double)n_rows);
    int ty_max = (int)floor((gmax_y + 90.0) / 180.0 * (double)n_rows);

    if (tx_min < 0) tx_min = 0;
    if (tx_max >= n_cols) tx_max = n_cols - 1;
    if (ty_min < 0) ty_min = 0;
    if (ty_max >= n_rows) ty_max = n_rows - 1;

    /* Determine number of linestrings */
    uint32_t n_lines = 1;
    if (geom->type == 5) { /* MultiLineString */
        n_lines = geom->n_offsets > 0 ? geom->n_offsets - 1 : 0;
    }

    /* Strip-based clipping: clip to column strip (left+right) once,
     * then for each row, clip the strip result to top+bottom. */
    for (int tx = tx_min; tx <= tx_max; tx++) {
        /* Buffered column strip — extends left/right by clip buffer */
        arpt_bounds strip = tile_bounds_buffered(z, tx, 0);

        /* Clip all segments to this column strip */
        darray sx, sy;
        u32array sseg;
        da_init(&sx);
        da_init(&sy);
        u32a_init(&sseg);
        clip_lines_to_strip(geom, n_lines, &strip, &sx, &sy, &sseg);

        if (sx.len < 2) {
            da_free(&sx);
            da_free(&sy);
            u32a_free(&sseg);
            continue;
        }

        /* For each row, clip the strip-clipped segments to the row bounds.
         * Iterate within each segment group to avoid phantom segments
         * between disconnected parts. */
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds_buffered(z, tx, ty);
            /* Clip only top+bottom: use full x range so only y clips */
            arpt_bounds row = {-180.0, tb.min_y, 180.0, tb.max_y};

            darray cx, cy;
            u32array cseg;
            da_init(&cx);
            da_init(&cy);
            u32a_init(&cseg);

            for (uint32_t si = 0; si < sseg.len; si++) {
                uint32_t seg_start = sseg.data[si];
                uint32_t seg_end = (si + 1 < sseg.len)
                    ? sseg.data[si + 1] : sx.len;

                for (uint32_t i = seg_start; i + 1 < seg_end; i++) {
                    double cx0, cy0, cx1, cy1;
                    if (liang_barsky(sx.data[i], sy.data[i],
                                     sx.data[i + 1], sy.data[i + 1],
                                     &row, &cx0, &cy0, &cx1, &cy1)) {
                        bool connected = (cx.len > 0 &&
                                          cx.data[cx.len - 1] == cx0 &&
                                          cy.data[cy.len - 1] == cy0);
                        if (!connected) {
                            u32a_push(&cseg, cx.len);
                            da_push(&cx, cx0);
                            da_push(&cy, cy0);
                        }
                        da_push(&cx, cx1);
                        da_push(&cy, cy1);
                    }
                }
            }

            /* Emit each connected segment group as a separate callback
             * to avoid phantom lines between disconnected parts. */
            for (uint32_t si = 0; si < cseg.len; si++) {
                uint32_t start = cseg.data[si];
                uint32_t end = (si + 1 < cseg.len)
                    ? cseg.data[si + 1] : cx.len;
                uint32_t n = end - start;
                if (n >= 2) {
                    arpt_geom clipped = {0};
                    clipped.type = 2; /* LineString */
                    clipped.x = cx.data + start;
                    clipped.y = cy.data + start;
                    clipped.n_coords = n;
                    cb(z, tx, ty, &clipped, ctx);
                }
            }

            da_free(&cx);
            da_free(&cy);
            u32a_free(&cseg);
        }

        da_free(&sx);
        da_free(&sy);
        u32a_free(&sseg);
    }
}

/* ---------- Sutherland-Hodgman slab clipping ----------
 *
 * Clip a polygon ring against two parallel axis-aligned lines (a "slab")
 * in a single pass, following the geojson-vt approach.  For rectangle
 * clipping, two slab passes are used: x then y.
 *
 * This produces a single output ring with implicit boundary edges along
 * the clip lines.  Re-entrant polygons that exit and re-enter the slab
 * produce overlapping boundary edges, which the ear-clipping triangulator
 * handles correctly.
 */

/* Clip an open ring (n unique vertices, no closing dup) against the slab
 * [k1, k2] along the given axis (0=x, 1=y).
 * Appends vertices to out_x/out_y.  Returns the number of vertices added. */
static uint32_t clip_ring_slab(const double *rx, const double *ry, uint32_t n,
                                double k1, double k2, int axis,
                                darray *out_x, darray *out_y) {
    uint32_t start = out_x->len;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t j = (i + 1) % n;
        double a = axis ? ry[i] : rx[i];
        double b = axis ? ry[j] : rx[j];

        if (a < k1) {
            /* Current point is left of / below k1 */
            if (b > k1) {
                /* Segment enters the slab from the left/bottom */
                double t = (k1 - a) / (b - a);
                if (axis) {
                    da_push(out_x, rx[i] + t * (rx[j] - rx[i]));
                    da_push(out_y, k1);
                } else {
                    da_push(out_x, k1);
                    da_push(out_y, ry[i] + t * (ry[j] - ry[i]));
                }
                if (b > k2) {
                    /* Passes all the way through — exits on the right/top */
                    double t2 = (k2 - a) / (b - a);
                    if (axis) {
                        da_push(out_x, rx[i] + t2 * (rx[j] - rx[i]));
                        da_push(out_y, k2);
                    } else {
                        da_push(out_x, k2);
                        da_push(out_y, ry[i] + t2 * (ry[j] - ry[i]));
                    }
                }
            }
        } else if (a > k2) {
            /* Current point is right of / above k2 */
            if (b < k2) {
                /* Segment enters the slab from the right/top */
                double t = (k2 - a) / (b - a);
                if (axis) {
                    da_push(out_x, rx[i] + t * (rx[j] - rx[i]));
                    da_push(out_y, k2);
                } else {
                    da_push(out_x, k2);
                    da_push(out_y, ry[i] + t * (ry[j] - ry[i]));
                }
                if (b < k1) {
                    /* Passes all the way through — exits on the left/bottom */
                    double t2 = (k1 - a) / (b - a);
                    if (axis) {
                        da_push(out_x, rx[i] + t2 * (rx[j] - rx[i]));
                        da_push(out_y, k1);
                    } else {
                        da_push(out_x, k1);
                        da_push(out_y, ry[i] + t2 * (ry[j] - ry[i]));
                    }
                }
            }
        } else {
            /* Current point is inside the slab */
            da_push(out_x, rx[i]);
            da_push(out_y, ry[i]);
        }

        /* Check exits from inside the slab */
        if (b < k1 && a >= k1) {
            /* Exits on the left/bottom */
            double t = (k1 - a) / (b - a);
            if (axis) {
                da_push(out_x, rx[i] + t * (rx[j] - rx[i]));
                da_push(out_y, k1);
            } else {
                da_push(out_x, k1);
                da_push(out_y, ry[i] + t * (ry[j] - ry[i]));
            }
        }
        if (b > k2 && a <= k2) {
            /* Exits on the right/top */
            double t = (k2 - a) / (b - a);
            if (axis) {
                da_push(out_x, rx[i] + t * (rx[j] - rx[i]));
                da_push(out_y, k2);
            } else {
                da_push(out_x, k2);
                da_push(out_y, ry[i] + t * (ry[j] - ry[i]));
            }
        }
    }

    /* Close the ring if endpoints don't match */
    uint32_t count = out_x->len - start;
    if (count >= 3) {
        uint32_t last = out_x->len - 1;
        if (out_x->data[start] != out_x->data[last] ||
            out_y->data[start] != out_y->data[last]) {
            da_push(out_x, out_x->data[start]);
            da_push(out_y, out_y->data[start]);
            count++;
        }
    }

    return count;
}

/* Strip a closing duplicate vertex (first == last) from a ring.
 * Returns the number of unique vertices (ring_n or ring_n - 1). */
static uint32_t strip_closing(const double *rx, const double *ry,
                               uint32_t ring_n) {
    if (ring_n >= 2 &&
        rx[0] == rx[ring_n - 1] && ry[0] == ry[ring_n - 1]) {
        return ring_n - 1;
    }
    return ring_n;
}

/* Clip a single closed ring against a rectangle using two-pass
 * Sutherland-Hodgman slab clipping (x then y).
 *
 * Input: n unique vertices (open ring, no closing dup).
 * Output: zero or more closed rings (first == last) appended to out_x/out_y.
 *         ring_starts receives the start index of each output ring. */
static void clip_ring_rect(const double *rx, const double *ry, uint32_t n,
                            const arpt_bounds *b,
                            darray *out_x, darray *out_y,
                            u32array *ring_starts) {
    if (n < 3) return;

    /* Pass 1: clip against x-slab [min_x, max_x] */
    darray mid_x, mid_y;
    da_init(&mid_x);
    da_init(&mid_y);
    uint32_t mid_n = clip_ring_slab(rx, ry, n,
                                     b->min_x, b->max_x, 0,
                                     &mid_x, &mid_y);

    if (mid_n < 4) { /* need at least 3 unique + closing */
        da_free(&mid_x);
        da_free(&mid_y);
        return;
    }

    /* Strip closing vertex for pass 2 input */
    uint32_t open_n = strip_closing(mid_x.data, mid_y.data, mid_n);
    if (open_n < 3) {
        da_free(&mid_x);
        da_free(&mid_y);
        return;
    }

    /* Pass 2: clip against y-slab [min_y, max_y] */
    uint32_t ring_start = out_x->len;
    uint32_t out_n = clip_ring_slab(mid_x.data, mid_y.data, open_n,
                                     b->min_y, b->max_y, 1,
                                     out_x, out_y);

    da_free(&mid_x);
    da_free(&mid_y);

    if (out_n >= 4) { /* at least 3 unique + closing */
        u32a_push(ring_starts, ring_start);
    } else {
        /* Degenerate — revert */
        out_x->len = ring_start;
        out_y->len = ring_start;
    }
}

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

    /* Clip each ring directly to each tile's full rectangle bounds
     * using Weiler-Atherton.  This correctly handles re-entrant polygons
     * that exit and re-enter the clip boundary multiple times. */
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
                if (ring_n < 3) continue;

                /* Strip closing duplicate if present */
                uint32_t open_n = strip_closing(geom->x + rstart,
                                                geom->y + rstart, ring_n);

                clip_ring_rect(geom->x + rstart, geom->y + rstart,
                               open_n, &tb, &out_x, &out_y, &ring_starts);
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
