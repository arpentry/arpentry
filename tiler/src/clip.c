#include "clip.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

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

/* Clip line segments to left+right edges of a column strip using
 * Liang-Barsky. Only the x-bounds of strip are used. */
static void clip_lines_to_strip(const arpt_geom *geom, uint32_t n_lines,
                                const arpt_bounds *strip,
                                darray *out_x, darray *out_y) {
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
                if (out_x->len == 0 || out_x->data[out_x->len - 1] != cx0 ||
                    out_y->data[out_y->len - 1] != cy0) {
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
        arpt_bounds strip = tile_bounds(z, tx, ty_min);
        arpt_bounds last_row = tile_bounds(z, tx, ty_max);
        strip.min_y = last_row.min_y;

        /* Clip all segments to this column strip */
        darray sx, sy;
        da_init(&sx);
        da_init(&sy);
        clip_lines_to_strip(geom, n_lines, &strip, &sx, &sy);

        if (sx.len < 2) {
            da_free(&sx);
            da_free(&sy);
            continue;
        }

        /* For each row, clip the strip-clipped segments to the row bounds */
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds(z, tx, ty);
            /* Clip only top+bottom: use full x range so only y clips */
            arpt_bounds row = {-180.0, tb.min_y, 180.0, tb.max_y};

            darray cx, cy;
            da_init(&cx);
            da_init(&cy);

            for (uint32_t i = 0; i + 1 < sx.len; i++) {
                double cx0, cy0, cx1, cy1;
                if (liang_barsky(sx.data[i], sy.data[i],
                                 sx.data[i + 1], sy.data[i + 1],
                                 &row, &cx0, &cy0, &cx1, &cy1)) {
                    if (cx.len == 0 || cx.data[cx.len - 1] != cx0 ||
                        cy.data[cy.len - 1] != cy0) {
                        da_push(&cx, cx0);
                        da_push(&cy, cy0);
                    }
                    da_push(&cx, cx1);
                    da_push(&cy, cy1);
                }
            }

            if (cx.len >= 2) {
                arpt_geom clipped = {0};
                clipped.type = 2; /* LineString */
                clipped.x = cx.data;
                clipped.y = cy.data;
                clipped.n_coords = cx.len;

                cb(z, tx, ty, &clipped, ctx);

                free(cx.data);
                free(cy.data);
            } else {
                da_free(&cx);
                da_free(&cy);
            }
        }

        da_free(&sx);
        da_free(&sy);
    }
}

/* ---------- Sutherland-Hodgman polygon clipping ---------- */

typedef enum { EDGE_LEFT, EDGE_RIGHT, EDGE_BOTTOM, EDGE_TOP } edge_t;

static bool inside(double px, double py, edge_t edge, const arpt_bounds *b) {
    switch (edge) {
    case EDGE_LEFT:   return px >= b->min_x;
    case EDGE_RIGHT:  return px <= b->max_x;
    case EDGE_BOTTOM: return py >= b->min_y;
    case EDGE_TOP:    return py <= b->max_y;
    }
    return false;
}

static void intersect(double sx, double sy, double ex, double ey,
                      edge_t edge, const arpt_bounds *b,
                      double *ix, double *iy) {
    double dx = ex - sx;
    double dy = ey - sy;
    double t = 0.0;
    switch (edge) {
    case EDGE_LEFT:   t = (b->min_x - sx) / dx; break;
    case EDGE_RIGHT:  t = (b->max_x - sx) / dx; break;
    case EDGE_BOTTOM: t = (b->min_y - sy) / dy; break;
    case EDGE_TOP:    t = (b->max_y - sy) / dy; break;
    }
    *ix = sx + t * dx;
    *iy = sy + t * dy;
}

static bool clip_polygon_edge(const double *in_x, const double *in_y, uint32_t in_n,
                               darray *out_x, darray *out_y,
                               edge_t edge, const arpt_bounds *b) {
    if (in_n == 0) return true;

    for (uint32_t i = 0; i < in_n; i++) {
        uint32_t j = (i + 1) % in_n;
        bool s_in = inside(in_x[i], in_y[i], edge, b);
        bool e_in = inside(in_x[j], in_y[j], edge, b);

        if (s_in && e_in) {
            da_push(out_x, in_x[j]);
            da_push(out_y, in_y[j]);
        } else if (s_in) {
            double ix, iy;
            intersect(in_x[i], in_y[i], in_x[j], in_y[j], edge, b, &ix, &iy);
            da_push(out_x, ix);
            da_push(out_y, iy);
        } else if (e_in) {
            double ix, iy;
            intersect(in_x[i], in_y[i], in_x[j], in_y[j], edge, b, &ix, &iy);
            da_push(out_x, ix);
            da_push(out_y, iy);
            da_push(out_x, in_x[j]);
            da_push(out_y, in_y[j]);
        }
    }
    return true;
}

/* Per-ring strip data: coordinates clipped to a column strip. */
typedef struct {
    double *x, *y;
    uint32_t len;
} ring_strip;

/* Clip a single ring to left+right edges of a column strip bounds.
 * Returns the clipped ring in *out (caller must free x/y). */
static void clip_ring_to_strip(const double *rx, const double *ry,
                                uint32_t ring_n, const arpt_bounds *strip,
                                ring_strip *out) {
    darray a_x, a_y, b_x, b_y;
    da_init(&a_x); da_init(&a_y);
    da_init(&b_x); da_init(&b_y);

    clip_polygon_edge(rx, ry, ring_n, &a_x, &a_y, EDGE_LEFT, strip);
    clip_polygon_edge(a_x.data, a_y.data, a_x.len, &b_x, &b_y, EDGE_RIGHT, strip);
    da_free(&a_x); da_free(&a_y);

    out->x = b_x.data;
    out->y = b_y.data;
    out->len = b_x.len;
}

/* Clip a strip-clipped ring to bottom+top edges of a row tile bounds. */
static void clip_strip_ring_to_row(const ring_strip *strip_ring,
                                    const arpt_bounds *tb,
                                    darray *out_x, darray *out_y) {
    if (strip_ring->len < 3) return;

    darray a_x, a_y, b_x, b_y;
    da_init(&a_x); da_init(&a_y);
    da_init(&b_x); da_init(&b_y);

    clip_polygon_edge(strip_ring->x, strip_ring->y, strip_ring->len,
                      &a_x, &a_y, EDGE_BOTTOM, tb);
    clip_polygon_edge(a_x.data, a_y.data, a_x.len,
                      &b_x, &b_y, EDGE_TOP, tb);
    da_free(&a_x); da_free(&a_y);

    if (b_x.len >= 3) {
        for (uint32_t i = 0; i < b_x.len; i++) {
            da_push(out_x, b_x.data[i]);
            da_push(out_y, b_y.data[i]);
        }
    }
    da_free(&b_x); da_free(&b_y);
}

static void clip_polygons(const arpt_geom *geom, int z,
                          arpt_tile_cb cb, void *ctx) {
    int n_cols = 1 << (z + 1);
    int n_rows = 1 << z;

    /* Bounding box of geometry */
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

    int tx_min = (int)floor((gmin_x + 180.0) / 360.0 * (double)n_cols);
    int tx_max = (int)floor((gmax_x + 180.0) / 360.0 * (double)n_cols);
    int ty_min = (int)floor((gmin_y + 90.0) / 180.0 * (double)n_rows);
    int ty_max = (int)floor((gmax_y + 90.0) / 180.0 * (double)n_rows);

    if (tx_min < 0) tx_min = 0;
    if (tx_max >= n_cols) tx_max = n_cols - 1;
    if (ty_min < 0) ty_min = 0;
    if (ty_max >= n_rows) ty_max = n_rows - 1;

    /* Determine rings: offsets give ring boundaries */
    uint32_t n_rings = geom->n_offsets > 0 ? geom->n_offsets - 1 : 1;

    /* Strip-based clipping: clip each ring to the column strip (left+right)
     * once, then for each row within the strip, clip only bottom+top.
     * This reduces work from O(cols * rows * vertices * 4_edges) to
     * O(cols * vertices * 2 + cols * rows * strip_vertices * 2). */
    ring_strip *strips = malloc(n_rings * sizeof(ring_strip));
    if (!strips) return;

    for (int tx = tx_min; tx <= tx_max; tx++) {
        /* Build strip bounds: full latitude range, single column width */
        arpt_bounds strip = tile_bounds(z, tx, ty_min);
        arpt_bounds last_row = tile_bounds(z, tx, ty_max);
        strip.min_y = last_row.min_y;  /* extend to bottom of tile range */

        /* Clip each ring to the column strip (left+right only) */
        bool any_strip_nonempty = false;
        for (uint32_t ri = 0; ri < n_rings; ri++) {
            uint32_t start = 0, end = geom->n_coords;
            if (geom->offsets && geom->n_offsets > 1) {
                start = geom->offsets[ri];
                end = geom->offsets[ri + 1];
            }
            uint32_t ring_n = end - start;
            if (ring_n < 3) {
                strips[ri] = (ring_strip){NULL, NULL, 0};
                continue;
            }
            clip_ring_to_strip(geom->x + start, geom->y + start,
                               ring_n, &strip, &strips[ri]);
            if (strips[ri].len >= 3) any_strip_nonempty = true;
        }

        if (!any_strip_nonempty) {
            for (uint32_t ri = 0; ri < n_rings; ri++) {
                free(strips[ri].x);
                free(strips[ri].y);
            }
            continue;
        }

        /* For each row in this column, clip strip rings to the row */
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds(z, tx, ty);

            darray out_x, out_y, off;
            da_init(&out_x);
            da_init(&out_y);
            da_init(&off);

            for (uint32_t ri = 0; ri < n_rings; ri++) {
                if (strips[ri].len < 3) continue;

                uint32_t before = out_x.len;
                clip_strip_ring_to_row(&strips[ri], &tb, &out_x, &out_y);

                if (out_x.len - before >= 3) {
                    da_push(&off, (double)before);
                }
            }

            if (out_x.len >= 3) {
                uint32_t n_clipped_rings = off.len;
                uint32_t *offsets = malloc((n_clipped_rings + 1) * sizeof(*offsets));
                if (offsets) {
                    for (uint32_t i = 0; i < n_clipped_rings; i++) {
                        offsets[i] = (uint32_t)off.data[i];
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
            da_free(&off);
        }

        /* Free strip data for this column */
        for (uint32_t ri = 0; ri < n_rings; ri++) {
            free(strips[ri].x);
            free(strips[ri].y);
        }
    }

    free(strips);
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
