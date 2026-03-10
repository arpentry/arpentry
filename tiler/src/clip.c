#include "clip.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Tile bounds in WGS84 degrees for tile (z, x, y). */
static arpt_bounds tile_bounds(int z, int tx, int ty) {
    double n = (double)(1 << z);
    double w = (double)tx / n * 360.0 - 180.0;
    double e = (double)(tx + 1) / n * 360.0 - 180.0;
    /* TMS-style: y=0 is top (north) */
    double n_lat = atan(sinh(M_PI * (1.0 - 2.0 * (double)ty / n))) * 180.0 / M_PI;
    double s_lat = atan(sinh(M_PI * (1.0 - 2.0 * (double)(ty + 1) / n))) * 180.0 / M_PI;
    return (arpt_bounds){w, s_lat, e, n_lat};
}

/* ---------- Point clipping ---------- */

static void clip_points(const arpt_geom *geom, int z,
                        arpt_tile_cb cb, void *ctx) {
    for (uint32_t i = 0; i < geom->n_coords; i++) {
        double px = geom->x[i];
        double py = geom->y[i];

        /* Determine which tile this point falls into */
        double n = (double)(1 << z);
        int tx = (int)floor((px + 180.0) / 360.0 * n);
        double lat_rad = py * M_PI / 180.0;
        int ty = (int)floor((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n);

        /* Clamp to valid range */
        if (tx < 0) tx = 0;
        if (tx >= (int)n) tx = (int)n - 1;
        if (ty < 0) ty = 0;
        if (ty >= (int)n) ty = (int)n - 1;

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

static void clip_lines(const arpt_geom *geom, int z,
                       arpt_tile_cb cb, void *ctx) {
    int n_tiles = 1 << z;

    /* Compute the bounding box of the geometry to limit tile iteration */
    double gmin_x = geom->x[0], gmax_x = geom->x[0];
    double gmin_y = geom->y[0], gmax_y = geom->y[0];
    for (uint32_t i = 1; i < geom->n_coords; i++) {
        if (geom->x[i] < gmin_x) gmin_x = geom->x[i];
        if (geom->x[i] > gmax_x) gmax_x = geom->x[i];
        if (geom->y[i] < gmin_y) gmin_y = geom->y[i];
        if (geom->y[i] > gmax_y) gmax_y = geom->y[i];
    }

    /* Convert geom bbox to tile range */
    double nd = (double)n_tiles;
    int tx_min = (int)floor((gmin_x + 180.0) / 360.0 * nd);
    int tx_max = (int)floor((gmax_x + 180.0) / 360.0 * nd);

    /* y is inverted in web mercator tile grid */
    double lat_rad;
    lat_rad = gmax_y * M_PI / 180.0;
    int ty_min = (int)floor((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * nd);
    lat_rad = gmin_y * M_PI / 180.0;
    int ty_max = (int)floor((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * nd);

    if (tx_min < 0) tx_min = 0;
    if (tx_max >= n_tiles) tx_max = n_tiles - 1;
    if (ty_min < 0) ty_min = 0;
    if (ty_max >= n_tiles) ty_max = n_tiles - 1;

    /* Determine number of linestrings */
    uint32_t n_lines = 1;
    if (geom->type == 5) { /* MultiLineString */
        n_lines = geom->n_offsets > 0 ? geom->n_offsets - 1 : 0;
    }

    for (int tx = tx_min; tx <= tx_max; tx++) {
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds(z, tx, ty);

            darray cx, cy;
            da_init(&cx);
            da_init(&cy);

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
                                     &tb, &cx0, &cy0, &cx1, &cy1)) {
                        if (cx.len == 0 || cx.data[cx.len - 1] != cx0 ||
                            cy.data[cy.len - 1] != cy0) {
                            da_push(&cx, cx0);
                            da_push(&cy, cy0);
                        }
                        da_push(&cx, cx1);
                        da_push(&cy, cy1);
                    }
                }
            }

            if (cx.len >= 2) {
                arpt_geom clipped = {0};
                clipped.type = 2; /* LineString */
                clipped.x = cx.data;
                clipped.y = cy.data;
                clipped.n_coords = cx.len;

                cb(z, tx, ty, &clipped, ctx);

                /* Don't free cx/cy data, just the struct's ownership */
                free(cx.data);
                free(cy.data);
            } else {
                da_free(&cx);
                da_free(&cy);
            }
        }
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

static void clip_polygons(const arpt_geom *geom, int z,
                          arpt_tile_cb cb, void *ctx) {
    int n_tiles = 1 << z;

    /* Bounding box of geometry */
    double gmin_x = geom->x[0], gmax_x = geom->x[0];
    double gmin_y = geom->y[0], gmax_y = geom->y[0];
    for (uint32_t i = 1; i < geom->n_coords; i++) {
        if (geom->x[i] < gmin_x) gmin_x = geom->x[i];
        if (geom->x[i] > gmax_x) gmax_x = geom->x[i];
        if (geom->y[i] < gmin_y) gmin_y = geom->y[i];
        if (geom->y[i] > gmax_y) gmax_y = geom->y[i];
    }

    double nd = (double)n_tiles;
    int tx_min = (int)floor((gmin_x + 180.0) / 360.0 * nd);
    int tx_max = (int)floor((gmax_x + 180.0) / 360.0 * nd);
    double lat_rad;
    lat_rad = gmax_y * M_PI / 180.0;
    int ty_min = (int)floor((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * nd);
    lat_rad = gmin_y * M_PI / 180.0;
    int ty_max = (int)floor((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * nd);

    if (tx_min < 0) tx_min = 0;
    if (tx_max >= n_tiles) tx_max = n_tiles - 1;
    if (ty_min < 0) ty_min = 0;
    if (ty_max >= n_tiles) ty_max = n_tiles - 1;

    /* Determine rings: offsets give ring boundaries */
    uint32_t n_rings = geom->n_offsets > 0 ? geom->n_offsets - 1 : 1;

    for (int tx = tx_min; tx <= tx_max; tx++) {
        for (int ty = ty_min; ty <= ty_max; ty++) {
            arpt_bounds tb = tile_bounds(z, tx, ty);

            darray out_x, out_y;
            da_init(&out_x);
            da_init(&out_y);
            darray off;
            da_init(&off);

            for (uint32_t ri = 0; ri < n_rings; ri++) {
                uint32_t start = 0, end = geom->n_coords;
                if (geom->offsets && geom->n_offsets > 1) {
                    start = geom->offsets[ri];
                    end = geom->offsets[ri + 1];
                }
                uint32_t ring_n = end - start;
                if (ring_n < 3) continue;

                const double *rx = geom->x + start;
                const double *ry = geom->y + start;

                /* Four-edge clip: left, right, bottom, top */
                darray a_x, a_y, b_x, b_y;
                da_init(&a_x); da_init(&a_y);
                da_init(&b_x); da_init(&b_y);

                clip_polygon_edge(rx, ry, ring_n, &a_x, &a_y, EDGE_LEFT, &tb);
                clip_polygon_edge(a_x.data, a_y.data, a_x.len, &b_x, &b_y, EDGE_RIGHT, &tb);
                da_free(&a_x); da_free(&a_y);
                da_init(&a_x); da_init(&a_y);
                clip_polygon_edge(b_x.data, b_y.data, b_x.len, &a_x, &a_y, EDGE_BOTTOM, &tb);
                da_free(&b_x); da_free(&b_y);
                da_init(&b_x); da_init(&b_y);
                clip_polygon_edge(a_x.data, a_y.data, a_x.len, &b_x, &b_y, EDGE_TOP, &tb);
                da_free(&a_x); da_free(&a_y);

                if (b_x.len >= 3) {
                    /* Record ring offset */
                    /* Use a union-compatible cast: store as double */
                    da_push(&off, (double)out_x.len);
                    for (uint32_t i = 0; i < b_x.len; i++) {
                        da_push(&out_x, b_x.data[i]);
                        da_push(&out_y, b_y.data[i]);
                    }
                }
                da_free(&b_x);
                da_free(&b_y);
            }

            if (out_x.len >= 3) {
                /* Build offset array */
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
