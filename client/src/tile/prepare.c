#include "prepare.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Polygon and line tessellation for offscreen texture rasterization */

static void count_polygon_geom(const arpt_surface_data *data,
                                size_t *out_verts, size_t *out_indices) {
    *out_verts = 0;
    *out_indices = 0;
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        size_t vc = data->polygons[i].vertex_count;
        if (vc < 3) continue;
        *out_verts += vc;
        *out_indices += (vc - 2) * 3;
    }
}

/* --- Ear-clipping triangulation for concave polygons --- */

static int64_t earclip_cross(int32_t ax, int32_t ay, int32_t bx, int32_t by,
                              int32_t cx, int32_t cy) {
    return (int64_t)(bx - ax) * (cy - ay) - (int64_t)(by - ay) * (cx - ax);
}

static bool earclip_pt_in_tri(int32_t px, int32_t py,
                               int32_t ax, int32_t ay,
                               int32_t bx, int32_t by,
                               int32_t cx, int32_t cy) {
    int64_t d1 = earclip_cross(ax, ay, bx, by, px, py);
    int64_t d2 = earclip_cross(bx, by, cx, cy, px, py);
    int64_t d3 = earclip_cross(cx, cy, ax, ay, px, py);
    /* Strict containment: exclude boundary (on-edge) points */
    return (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0);
}

/* Triangulate a simple polygon (n vertices, implicitly closed) using
   ear-clipping.  Writes triangle indices at out_indices as offsets from base.
   Returns number of indices written. */
static size_t earclip_triangulate(const uint16_t *x, const uint16_t *y,
                                   size_t n, uint32_t base,
                                   uint32_t *out_indices) {
    if (n < 3) return 0;
    if (n == 3) {
        out_indices[0] = base;
        out_indices[1] = base + 1;
        out_indices[2] = base + 2;
        return 3;
    }

    /* Determine winding from signed area (positive = CCW) */
    int64_t area2 = 0;
    for (size_t i = 0; i < n; i++) {
        size_t j = (i + 1) % n;
        area2 += (int64_t)x[i] * y[j] - (int64_t)x[j] * y[i];
    }
    bool ccw = area2 > 0;

    /* Doubly-linked list of active vertex indices */
    uint32_t *prv = malloc(n * sizeof(uint32_t));
    uint32_t *nxt = malloc(n * sizeof(uint32_t));
    if (!prv || !nxt) { free(prv); free(nxt); return 0; }

    for (size_t i = 0; i < n; i++) {
        prv[i] = (uint32_t)((i + n - 1) % n);
        nxt[i] = (uint32_t)((i + 1) % n);
    }

    size_t remaining = n;
    size_t idx = 0;
    uint32_t ear = 0;
    size_t attempts = 0;

    while (remaining > 3 && attempts < remaining * remaining) {
        uint32_t p = prv[ear], c = ear, nx = nxt[ear];

        int64_t cross = earclip_cross((int32_t)x[p], (int32_t)y[p],
                                       (int32_t)x[c], (int32_t)y[c],
                                       (int32_t)x[nx], (int32_t)y[nx]);
        /* Treat collinear (cross==0) as convex to ensure progress */
        bool convex = ccw ? (cross >= 0) : (cross <= 0);

        if (convex) {
            /* Check no other vertex lies inside this triangle */
            bool blocked = false;
            uint32_t test = nxt[nx];
            while (test != p) {
                if (earclip_pt_in_tri((int32_t)x[test], (int32_t)y[test],
                                       (int32_t)x[p], (int32_t)y[p],
                                       (int32_t)x[c], (int32_t)y[c],
                                       (int32_t)x[nx], (int32_t)y[nx])) {
                    blocked = true;
                    break;
                }
                test = nxt[test];
            }

            if (!blocked) {
                out_indices[idx++] = base + p;
                out_indices[idx++] = base + c;
                out_indices[idx++] = base + nx;
                nxt[p] = nx;
                prv[nx] = p;
                remaining--;
                attempts = 0;
                ear = nx;
                continue;
            }
        }

        ear = nxt[ear];
        attempts++;
    }

    /* Last triangle */
    if (remaining == 3) {
        uint32_t a = ear, b = nxt[a], c = nxt[b];
        out_indices[idx++] = base + a;
        out_indices[idx++] = base + b;
        out_indices[idx++] = base + c;
    } else if (remaining > 3) {
        /* Earclip exhausted attempts — force-emit remaining vertices as a
           triangle fan from the current ear.  This produces slightly wrong
           triangles for concave regions but avoids the much worse artifact
           of silently dropping large polygon sections. */
        uint32_t a = ear;
        uint32_t b = nxt[a];
        while (remaining > 2) {
            uint32_t c = nxt[b];
            out_indices[idx++] = base + a;
            out_indices[idx++] = base + b;
            out_indices[idx++] = base + c;
            b = c;
            remaining--;
        }
    }

    free(prv);
    free(nxt);
    return idx;
}

static void emit_polygons(const arpt_surface_data *data,
                           const arpt_style *style,
                           arpt_poly_vertex *verts, uint32_t *idxs,
                           size_t *vi, size_t *ii,
                           arpt_poly_group *groups, size_t *gi) {
    if (!data) return;
    uint8_t cur_cls = 0;
    uint16_t cur_poly_id = 0;
    for (size_t i = 0; i < data->count; i++) {
        const arpt_surface_polygon *p = &data->polygons[i];
        if (p->vertex_count < 3) continue;
        if (p->cls == 0) continue; /* skip unmatched class (background) */

        /* Start a new group when class or polygon changes.  Each polygon
           (exterior + holes) gets its own stencil pass so that overlapping
           polygons of the same class don't cancel via even-odd invert. */
        if (p->cls != cur_cls || p->poly_id != cur_poly_id) {
            if (cur_cls != 0 && groups) {
                groups[*gi - 1].index_count =
                    (uint32_t)*ii - groups[*gi - 1].first_index;
            }
            cur_cls = p->cls;
            cur_poly_id = p->poly_id;
            if (groups) {
                groups[*gi] = (arpt_poly_group){(uint32_t)*ii, 0};
            }
            (*gi)++;
        }

        const float *c = style->colors[p->cls];
        uint32_t base = (uint32_t)*vi;

        for (size_t v = 0; v < p->vertex_count; v++) {
            verts[*vi] = (arpt_poly_vertex){p->x[v], p->y[v],
                                             c[0], c[1], c[2], c[3]};
            (*vi)++;
        }

        size_t written = earclip_triangulate(p->x, p->y, p->vertex_count,
                                              base, idxs + *ii);
        *ii += written;
    }
    /* Close the last group. */
    if (cur_cls != 0 && groups && *gi > 0) {
        groups[*gi - 1].index_count =
            (uint32_t)*ii - groups[*gi - 1].first_index;
    }
}

/* Stroke widths in the style are authored for this zoom level; above it
   roads widen each level (like every slippy-map style), below it they
   narrow, down to a floor so low-zoom strokes keep a readable weight. */
#define LINE_WIDTH_REF_LEVEL 12
#define LINE_WIDTH_GROWTH 1.35
#define LINE_WIDTH_SCALE_MIN 0.55
#define LINE_WIDTH_SCALE_MAX 2.0

/* Split centerline segments longer than this (tile quantized units) before
   draping, so a road crossing relief follows the terrain instead of cutting
   a flat chord through a hill.  Short segments (the common case) are kept. */
#define LINE_MAX_SEG 768.0

static double line_zoom_scale(int level) {
    double s = pow(LINE_WIDTH_GROWTH, level - LINE_WIDTH_REF_LEVEL);
    if (s < LINE_WIDTH_SCALE_MIN) s = LINE_WIDTH_SCALE_MIN;
    if (s > LINE_WIDTH_SCALE_MAX) s = LINE_WIDTH_SCALE_MAX;
    return s;
}

/* --- Terrain elevation lookup for draping road vertices ---

   Roads arrive as flat 2D polylines; to sit on the ground each vertex needs
   the terrain elevation at its (qx,qy).  The terrain is an irregular mesh, so
   bin its triangles into a uniform grid over the quantized space and, per
   query, barycentrically interpolate z within the containing triangle.  This
   matches the rendered terrain plane exactly, minimizing z-fighting. */

#define TGRID_N 64        /* cells per axis */
#define TGRID_SHIFT 10    /* 65536 / 64 = 1024 = 1 << 10 */

typedef struct {
    const arpt_terrain_mesh *mesh;
    uint32_t *cell_start;  /* CSR offsets, length TGRID_N*TGRID_N + 1 */
    uint32_t *tri_idx;     /* triangle indices grouped by cell */
    uint16_t xmin, xmax, ymin, ymax; /* meshed-area bounds (quantized) */
} terrain_grid;

/* Visit every grid cell a triangle's bounding box overlaps. */
#define TGRID_FOR_CELLS(m, t, cellvar)                                        \
    uint32_t _i0 = (m)->indices[(t) * 3 + 0];                                 \
    uint32_t _i1 = (m)->indices[(t) * 3 + 1];                                 \
    uint32_t _i2 = (m)->indices[(t) * 3 + 2];                                 \
    uint16_t _xmin = (m)->x[_i0], _xmax = (m)->x[_i0];                        \
    uint16_t _ymin = (m)->y[_i0], _ymax = (m)->y[_i0];                        \
    if ((m)->x[_i1] < _xmin) _xmin = (m)->x[_i1];                             \
    if ((m)->x[_i1] > _xmax) _xmax = (m)->x[_i1];                             \
    if ((m)->x[_i2] < _xmin) _xmin = (m)->x[_i2];                             \
    if ((m)->x[_i2] > _xmax) _xmax = (m)->x[_i2];                             \
    if ((m)->y[_i1] < _ymin) _ymin = (m)->y[_i1];                             \
    if ((m)->y[_i1] > _ymax) _ymax = (m)->y[_i1];                             \
    if ((m)->y[_i2] < _ymin) _ymin = (m)->y[_i2];                             \
    if ((m)->y[_i2] > _ymax) _ymax = (m)->y[_i2];                             \
    int _cx0 = _xmin >> TGRID_SHIFT, _cx1 = _xmax >> TGRID_SHIFT;             \
    int _cy0 = _ymin >> TGRID_SHIFT, _cy1 = _ymax >> TGRID_SHIFT;             \
    for (int _cy = _cy0; _cy <= _cy1; _cy++)                                  \
        for (int _cx = _cx0; _cx <= _cx1; _cx++)                              \
            for (size_t cellvar = (size_t)_cy * TGRID_N + _cx, _once = 1;     \
                 _once; _once = 0)

static bool tgrid_build(terrain_grid *g, const arpt_terrain_mesh *m) {
    memset(g, 0, sizeof(*g));
    if (!m || m->vertex_count == 0 || m->index_count < 3 || !m->z ||
        !m->indices || !m->x || !m->y)
        return false;
    g->mesh = m;
    size_t cells = (size_t)TGRID_N * TGRID_N;
    size_t tri_count = m->index_count / 3;

    /* Meshed-area bounds: queries outside these (road vertices in the tile
       buffer beyond the terrain) clamp to the edge instead of extrapolating. */
    g->xmin = g->xmax = m->x[0];
    g->ymin = g->ymax = m->y[0];
    for (size_t i = 1; i < m->vertex_count; i++) {
        if (m->x[i] < g->xmin) g->xmin = m->x[i];
        if (m->x[i] > g->xmax) g->xmax = m->x[i];
        if (m->y[i] < g->ymin) g->ymin = m->y[i];
        if (m->y[i] > g->ymax) g->ymax = m->y[i];
    }

    g->cell_start = calloc(cells + 1, sizeof(uint32_t));
    if (!g->cell_start) return false;

    /* Pass 1: count triangles per cell. */
    for (size_t t = 0; t < tri_count; t++) {
        TGRID_FOR_CELLS(m, t, cell) { g->cell_start[cell + 1]++; }
    }
    for (size_t i = 0; i < cells; i++)
        g->cell_start[i + 1] += g->cell_start[i];

    size_t total = g->cell_start[cells];
    g->tri_idx = malloc((total ? total : 1) * sizeof(uint32_t));
    uint32_t *cursor = malloc(cells * sizeof(uint32_t));
    if (!g->tri_idx || !cursor) {
        free(g->tri_idx); free(cursor); free(g->cell_start);
        memset(g, 0, sizeof(*g));
        return false;
    }
    memcpy(cursor, g->cell_start, cells * sizeof(uint32_t));

    /* Pass 2: scatter triangle indices into their cells. */
    for (size_t t = 0; t < tri_count; t++) {
        TGRID_FOR_CELLS(m, t, cell) { g->tri_idx[cursor[cell]++] = (uint32_t)t; }
    }
    free(cursor);
    return true;
}

static void tgrid_free(terrain_grid *g) {
    if (!g) return;
    free(g->cell_start);
    free(g->tri_idx);
    memset(g, 0, sizeof(*g));
}

/* Nearest-vertex elevation, used only when a point falls in no triangle
   (tile-margin queries). */
static int32_t tgrid_nearest_z(const terrain_grid *g, uint16_t qx, uint16_t qy) {
    const arpt_terrain_mesh *m = g->mesh;
    size_t best = 0;
    int64_t best_d2 = INT64_MAX;
    for (size_t i = 0; i < m->vertex_count; i++) {
        int64_t dx = (int64_t)m->x[i] - qx;
        int64_t dy = (int64_t)m->y[i] - qy;
        int64_t d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return m->z[best];
}

/* Terrain elevation (mm) at quantized position (qx,qy).  Interpolates within
   the containing terrain triangle.  If the point falls in no triangle (it sits
   in the tile buffer, just outside the meshed area), it uses the *nearest*
   triangle in the cell and clamps to it, rather than a global nearest vertex:
   on steep terrain a far vertex's elevation would stretch the road geometry
   into long spikes, so the fallback must stay local. */
static int32_t tgrid_z(const terrain_grid *g, uint16_t qx, uint16_t qy) {
    if (!g || !g->cell_start) return 0;
    const arpt_terrain_mesh *m = g->mesh;
    /* Clamp into the meshed area so buffer-zone road vertices sample the edge
       elevation rather than a far cell with no triangles. */
    if (qx < g->xmin) qx = g->xmin; else if (qx > g->xmax) qx = g->xmax;
    if (qy < g->ymin) qy = g->ymin; else if (qy > g->ymax) qy = g->ymax;
    size_t cell = (size_t)(qy >> TGRID_SHIFT) * TGRID_N + (qx >> TGRID_SHIFT);
    uint32_t s = g->cell_start[cell], e = g->cell_start[cell + 1];
    double best_min = -1e30, best_z = 0.0;
    bool found = false;
    for (uint32_t k = s; k < e; k++) {
        uint32_t t = g->tri_idx[k];
        uint32_t i0 = m->indices[t * 3], i1 = m->indices[t * 3 + 1],
                 i2 = m->indices[t * 3 + 2];
        double ax = m->x[i0], ay = m->y[i0];
        double bx = m->x[i1], by = m->y[i1];
        double cx = m->x[i2], cy = m->y[i2];
        double d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if (fabs(d) < 1e-9) continue;
        double w0 = ((by - cy) * (qx - cx) + (cx - bx) * (qy - cy)) / d;
        double w1 = ((cy - ay) * (qx - cx) + (ax - cx) * (qy - cy)) / d;
        double w2 = 1.0 - w0 - w1;
        double z = w0 * m->z[i0] + w1 * m->z[i1] + w2 * m->z[i2];
        double mn = w0 < w1 ? (w0 < w2 ? w0 : w2) : (w1 < w2 ? w1 : w2);
        if (mn >= -1e-6) return (int32_t)llround(z); /* inside this triangle */
        if (mn > best_min) { best_min = mn; best_z = z; found = true; }
    }
    if (found) return (int32_t)llround(best_z); /* nearest triangle in cell */
    return tgrid_nearest_z(g, qx, qy);           /* cell empty: far from mesh */
}

/* Number of sub-segments a polyline densifies to (upper bound on emitted
   quads), splitting any segment longer than LINE_MAX_SEG. */
static size_t densified_seg_count(const uint16_t *x, const uint16_t *y,
                                  size_t vc) {
    size_t total = 0;
    for (size_t s = 0; s + 1 < vc; s++) {
        double dx = (double)x[s + 1] - x[s], dy = (double)y[s + 1] - y[s];
        double len = sqrt(dx * dx + dy * dy);
        size_t splits = (size_t)ceil(len / LINE_MAX_SEG);
        total += splits < 1 ? 1 : splits;
    }
    return total;
}

/* Write the densified polyline into ox/oy (capacity cap); returns point count. */
static size_t densify_polyline(const uint16_t *x, const uint16_t *y, size_t vc,
                               uint16_t *ox, uint16_t *oy, size_t cap) {
    if (vc == 0 || cap == 0) return 0;
    size_t n = 0;
    ox[n] = x[0]; oy[n] = y[0]; n++;
    for (size_t s = 0; s + 1 < vc; s++) {
        double dx = (double)x[s + 1] - x[s], dy = (double)y[s + 1] - y[s];
        double len = sqrt(dx * dx + dy * dy);
        int splits = (int)ceil(len / LINE_MAX_SEG);
        if (splits < 1) splits = 1;
        for (int k = 1; k <= splits && n < cap; k++) {
            double tt = (double)k / splits;
            ox[n] = (uint16_t)lround(x[s] + dx * tt);
            oy[n] = (uint16_t)lround(y[s] + dy * tt);
            n++;
        }
    }
    return n;
}

/* Liang-Barsky: clip segment (x0,y0)-(x1,y1) to the rect, returning the
   visible parameter range [t0,t1] (false if fully outside). */
static bool clip_segment(double x0, double y0, double x1, double y1,
                         double xmin, double ymin, double xmax, double ymax,
                         double *t0, double *t1) {
    double dx = x1 - x0, dy = y1 - y0;
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};
    double u0 = 0.0, u1 = 1.0;
    for (int i = 0; i < 4; i++) {
        if (fabs(p[i]) < 1e-12) {
            if (q[i] < 0) return false; /* parallel and outside this edge */
        } else {
            double r = q[i] / p[i];
            if (p[i] < 0) { if (r > u1) return false; if (r > u0) u0 = r; }
            else          { if (r < u0) return false; if (r < u1) u1 = r; }
        }
    }
    *t0 = u0; *t1 = u1;
    return true;
}

static void emit_subpolyline(const uint16_t *px, const uint16_t *py, size_t vc,
                             double hw, const float color[4],
                             const terrain_grid *grid,
                             arpt_line_vertex *verts, uint32_t *idxs,
                             size_t *vi, size_t *ii);

/* Width of the overlap kept on each side of the tile when clipping roads, in
   quantized units.  Adjacent tiles each extend this far past the shared edge so
   their roads overlap and join with no seam; small enough that the flat
   edge-elevation extrapolation over it is negligible. */
#define LINE_TILE_OVERLAP 192.0

/* Emit a draped road for one polyline.  The polyline is first clipped to the
   tile-proper bounds (plus a small overlap): the tiler keeps a half-tile line
   buffer, but the terrain mesh covers only the tile proper, so road vertices
   far out in the buffer would sample a clamped edge elevation and stretch the
   geometry into spikes on steep terrain.  Clipping to the *canonical* tile
   bounds (identical for adjacent tiles, unlike the terrain vertex bbox) makes
   neighbouring tiles' roads meet on exactly the same world line; the overlap
   margin hides the seam.  The overhang is redundant — neighbours draw it draped
   on their own terrain. */
static void emit_polyline(const arpt_line_feature *line, double hw,
                          const float color[4], const terrain_grid *grid,
                          arpt_line_vertex *verts, uint32_t *idxs,
                          size_t *vi, size_t *ii) {
    size_t vc = line->vertex_count;
    if (vc < 2) return;
    /* Drop closing vertex of a closed loop; the renderer draws open polylines. */
    if (vc >= 3 && line->x[0] == line->x[vc - 1] &&
        line->y[0] == line->y[vc - 1])
        vc--;

    double xmin = ARPT_BUFFER - LINE_TILE_OVERLAP;
    double xmax = ARPT_BUFFER + ARPT_EXTENT + LINE_TILE_OVERLAP;
    double ymin = xmin, ymax = xmax;

    /* Accumulate clipped runs; flush a sub-polyline whenever the line leaves
       the rect (a segment is fully out, or enters/exits partway). */
    uint16_t *sx = malloc((vc + 1) * sizeof(uint16_t));
    uint16_t *sy = malloc((vc + 1) * sizeof(uint16_t));
    if (!sx || !sy) { free(sx); free(sy); return; }
    size_t sn = 0;

#define FLUSH_RUN()                                                            \
    do {                                                                        \
        if (sn >= 2)                                                            \
            emit_subpolyline(sx, sy, sn, hw, color, grid, verts, idxs, vi, ii); \
        sn = 0;                                                                  \
    } while (0)

    for (size_t s = 0; s + 1 < vc; s++) {
        double ax = line->x[s], ay = line->y[s];
        double bx = line->x[s + 1], by = line->y[s + 1];
        double t0, t1;
        if (!clip_segment(ax, ay, bx, by, xmin, ymin, xmax, ymax, &t0, &t1)) {
            FLUSH_RUN();
            continue;
        }
        double cax = ax + (bx - ax) * t0, cay = ay + (by - ay) * t0;
        double cbx = ax + (bx - ax) * t1, cby = ay + (by - ay) * t1;
        if (sn == 0) {
            sx[sn] = (uint16_t)lround(cax); sy[sn] = (uint16_t)lround(cay); sn++;
        } else if (t0 > 1e-9) {
            /* Segment re-entered the rect: break the run and restart here. */
            FLUSH_RUN();
            sx[sn] = (uint16_t)lround(cax); sy[sn] = (uint16_t)lround(cay); sn++;
        }
        sx[sn] = (uint16_t)lround(cbx); sy[sn] = (uint16_t)lround(cby); sn++;
        if (t1 < 1.0 - 1e-9) FLUSH_RUN(); /* segment exited the rect */
    }
    FLUSH_RUN();

#undef FLUSH_RUN
    free(sx); free(sy);
}

/* Emit terrain-draped SDF quads for one (already clipped) sub-polyline at the
   given half-width and color.  Each vertex carries the terrain elevation at the
   segment centerline so the ribbon drapes the ground. */
static void emit_subpolyline(const uint16_t *px, const uint16_t *py, size_t vc,
                             double hw, const float color[4],
                             const terrain_grid *grid,
                             arpt_line_vertex *verts, uint32_t *idxs,
                             size_t *vi, size_t *ii) {
    const float *c = color;
    if (vc < 2) return;

    /* Densify so long segments follow terrain relief once draped. */
    size_t dcap = densified_seg_count(px, py, vc) + 1;
    uint16_t *lx = malloc(dcap * sizeof(uint16_t));
    uint16_t *ly = malloc(dcap * sizeof(uint16_t));
    if (!lx || !ly) { free(lx); free(ly); return; }
    size_t dn = densify_polyline(px, py, vc, lx, ly, dcap);
    if (dn < 2) { free(lx); free(ly); return; }
    size_t n_segs = dn - 1;

    /* Pre-compute per-segment direction and normal */
    double *seg_nx = malloc(n_segs * sizeof(double));
    double *seg_ny = malloc(n_segs * sizeof(double));
    double *seg_ux = malloc(n_segs * sizeof(double));
    double *seg_uy = malloc(n_segs * sizeof(double));
    double *seg_len = malloc(n_segs * sizeof(double));
    if (!seg_nx || !seg_ny || !seg_ux || !seg_uy || !seg_len) {
        free(seg_nx); free(seg_ny); free(seg_ux); free(seg_uy);
        free(seg_len); free(lx); free(ly);
        return;
    }

    bool any_valid = false;
    for (size_t s = 0; s < n_segs; s++) {
        double dx = (double)lx[s + 1] - lx[s];
        double dy = (double)ly[s + 1] - ly[s];
        double len = sqrt(dx * dx + dy * dy);
        if (len < 0.001) len = 0.001;
        seg_ux[s] = dx / len;
        seg_uy[s] = dy / len;
        seg_nx[s] = -seg_uy[s];
        seg_ny[s] = seg_ux[s];
        seg_len[s] = len;
        if (len >= 1.0) any_valid = true;
    }

    if (!any_valid) {
        free(seg_nx); free(seg_ny); free(seg_ux); free(seg_uy);
        free(seg_len); free(lx); free(ly);
        return;
    }

#define CLAMP16(v) ((uint16_t)((v) < 0 ? 0 : (v) > 65535 ? 65535 : (v)))
    /* Emit one draped vertex.  The elevation `vz` is sampled at the segment
       centerline, not at this offset corner, so both edges of a cross-section
       (and the casing + fill strokes that share it) sit at the same height.
       That keeps the road a flat ribbon on the terrain instead of tilting wide
       strokes to different heights, which read as a doubled, broken road. */
#define EMIT_V(px, py, vz, lu, lv)                                             \
    do {                                                                        \
        verts[*vi] = (arpt_line_vertex){CLAMP16(px), CLAMP16(py), (vz),        \
                                        c[0], c[1], c[2], c[3],                \
                                        (float)(lu), (float)(lv),              \
                                        (float)hw, (float)len};                \
        (*vi)++;                                                                \
    } while (0)

    for (size_t s = 0; s < n_segs; s++) {
        double len = seg_len[s];
        if (len < 1.0) continue;

        double x1 = lx[s], y1 = ly[s];
        double x2 = lx[s + 1], y2 = ly[s + 1];

        /* Offset vectors at start and end of this segment.
         * At interior vertices, use a miter between adjacent
         * segment normals so consecutive quads share edges. */
        double m1x, m1y, m2x, m2y;

        if (s > 0 && seg_len[s - 1] >= 1.0) {
            /* Miter at start vertex */
            double mx = seg_nx[s - 1] + seg_nx[s];
            double my = seg_ny[s - 1] + seg_ny[s];
            double d = mx * seg_nx[s] + my * seg_ny[s];
            if (d < 0.25) d = 0.25;  /* miter limit: cap at 4x */
            m1x = mx / d;
            m1y = my / d;
        } else {
            m1x = seg_nx[s];
            m1y = seg_ny[s];
        }

        if (s + 1 < n_segs && seg_len[s + 1] >= 1.0) {
            /* Miter at end vertex */
            double mx = seg_nx[s] + seg_nx[s + 1];
            double my = seg_ny[s] + seg_ny[s + 1];
            double d = mx * seg_nx[s] + my * seg_ny[s];
            if (d < 0.25) d = 0.25;
            m2x = mx / d;
            m2y = my / d;
        } else {
            m2x = seg_nx[s];
            m2y = seg_ny[s];
        }

        /* Extend endpoints along tangent for caps at polyline ends */
        double ex1 = x1, ey1 = y1, ex2 = x2, ey2 = y2;
        double cap1 = 0.0, cap2 = 0.0;
        if (s == 0 || seg_len[s - 1] < 1.0) {
            ex1 -= seg_ux[s] * hw;
            ey1 -= seg_uy[s] * hw;
            cap1 = hw;
        }
        if (s + 1 == n_segs || seg_len[s + 1] < 1.0) {
            ex2 += seg_ux[s] * hw;
            ey2 += seg_uy[s] * hw;
            cap2 = hw;
        }

        /* Centerline elevation at each end, shared by both edge corners so the
           cross-section stays horizontal and the ribbon hugs the road, not the
           cross-slope.  Interior vertices sample the same point in adjacent
           segments, so the draped ribbon stays continuous. */
        int32_t z1 = tgrid_z(grid, CLAMP16(ex1), CLAMP16(ey1));
        int32_t z2 = tgrid_z(grid, CLAMP16(ex2), CLAMP16(ey2));

        uint32_t base = (uint32_t)*vi;
        EMIT_V(ex1 - m1x * hw, ey1 - m1y * hw, z1, -cap1, -hw);
        EMIT_V(ex1 + m1x * hw, ey1 + m1y * hw, z1, -cap1, hw);
        EMIT_V(ex2 + m2x * hw, ey2 + m2y * hw, z2, len + cap2, hw);
        EMIT_V(ex2 - m2x * hw, ey2 - m2y * hw, z2, len + cap2, -hw);

        idxs[(*ii)++] = base;
        idxs[(*ii)++] = base + 1;
        idxs[(*ii)++] = base + 2;
        idxs[(*ii)++] = base;
        idxs[(*ii)++] = base + 2;
        idxs[(*ii)++] = base + 3;
    }

#undef CLAMP16
#undef EMIT_V

    free(seg_nx); free(seg_ny); free(seg_ux); free(seg_uy);
    free(seg_len); free(lx); free(ly);
}

/* Resolve the rendered half-width for one stroke: just the zoom scale.  Road
   widths are now true geographic widths drawn as geometry, so there is no
   one-texel floor (that existed only for the fixed-resolution texture). */
static double resolve_half_width(double hw, double zoom_scale) {
    return hw * zoom_scale;
}

/* Emit every line feature as a single filled stroke (no casing outline).
   Features arrive sorted by class, so later style entries draw on top. */
static void emit_line_sdf_quads(const arpt_line_data *data,
                                const arpt_style *style, int level,
                                const terrain_grid *grid,
                                arpt_line_vertex *verts, uint32_t *idxs,
                                size_t *vi, size_t *ii) {
    if (!data) return;
    double zs = line_zoom_scale(level);
    for (size_t i = 0; i < data->count; i++) {
        const arpt_line_feature *line = &data->lines[i];
        if (style->stroke_widths[line->cls] <= 0.0f) continue;
        double hw = resolve_half_width(style->stroke_widths[line->cls], zs);
        float color[4];
        memcpy(color, style->colors[line->cls], sizeof(color));
        emit_polyline(line, hw, color, grid, verts, idxs, vi, ii);
    }
}

void arpt_prepare_polygons(const arpt_surface_data *surface,
                           const arpt_style *style, arpt_polygon_prim *out) {
    memset(out, 0, sizeof(*out));

    size_t nv = 0, ni = 0;
    count_polygon_geom(surface, &nv, &ni);
    if (nv == 0 || ni == 0) return;

    out->verts = malloc(nv * sizeof(arpt_poly_vertex));
    out->indices = malloc(ni * sizeof(uint32_t));
    /* Upper bound for groups: one per polygon (worst case). */
    size_t max_groups = surface ? surface->count : 0;
    out->groups = max_groups > 0
        ? malloc(max_groups * sizeof(arpt_poly_group)) : NULL;
    if (out->verts && out->indices) {
        size_t vi = 0, ii = 0, gi = 0;
        emit_polygons(surface, style, out->verts, out->indices,
                      &vi, &ii, out->groups, &gi);
        out->vert_count = vi;
        out->index_count = ii;
        out->group_count = gi;
    } else {
        free(out->verts);
        free(out->indices);
        free(out->groups);
        memset(out, 0, sizeof(*out));
    }
}

void arpt_prepare_lines(const arpt_line_data *line_data,
                        const arpt_style *style, int level,
                        const arpt_terrain_mesh *terrain,
                        arpt_line_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!line_data) return;

    /* Upper bound: every feature emits one filled stroke.  Segments are
       densified before tessellation, so count those sub-segments. */
    size_t nv = 0, ni = 0;
    for (size_t i = 0; i < line_data->count; i++) {
        const arpt_line_feature *line = &line_data->lines[i];
        if (line->vertex_count < 2) continue;
        size_t segs = densified_seg_count(line->x, line->y,
                                          line->vertex_count);
        nv += segs * 4;
        ni += segs * 6;
    }
    if (nv == 0 || ni == 0) return;

    terrain_grid grid;
    tgrid_build(&grid, terrain); /* on failure, tgrid_z returns z=0 (flat) */

    out->verts = malloc(nv * sizeof(arpt_line_vertex));
    out->indices = malloc(ni * sizeof(uint32_t));
    if (out->verts && out->indices) {
        size_t vi = 0, ii = 0;
        emit_line_sdf_quads(line_data, style, level, &grid, out->verts,
                            out->indices, &vi, &ii);
        out->vert_count = vi;
        out->index_count = ii;
    } else {
        free(out->verts);
        free(out->indices);
        memset(out, 0, sizeof(*out));
    }

    tgrid_free(&grid);
}

/* Instances — pack tree points into per-model batches */

void arpt_prepare_instances(const arpt_tree_data *trees, int model_count,
                            arpt_instance_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!trees || trees->count == 0 || model_count == 0) return;

    /* Count instances per model */
    size_t counts[ARPT_MAX_MODELS] = {0};
    for (size_t i = 0; i < trees->count; i++) {
        int mi = trees->points[i].model_index;
        if (mi >= 0 && mi < model_count) counts[mi]++;
    }

    /* Count non-empty batches */
    int batch_count = 0;
    for (int mi = 0; mi < model_count; mi++) {
        if (counts[mi] > 0) batch_count++;
    }
    if (batch_count == 0) return;

    out->batches = calloc((size_t)batch_count, sizeof(arpt_instance_batch));
    if (!out->batches) return;
    out->batch_count = batch_count;

    int bi = 0;
    for (int mi = 0; mi < model_count; mi++) {
        if (counts[mi] == 0) continue;

        arpt_instance_batch *batch = &out->batches[bi++];
        batch->model_index = mi;
        batch->count = counts[mi];
        batch->instances = malloc(counts[mi] * sizeof(arpt_instance_pt));
        if (!batch->instances) { batch->count = 0; continue; }

        size_t idx = 0;
        for (size_t i = 0; i < trees->count; i++) {
            if (trees->points[i].model_index != mi) continue;

            batch->instances[idx].qx = trees->points[i].qx;
            batch->instances[idx].qy = trees->points[i].qy;
            batch->instances[idx].qz = trees->points[i].z;

            uint32_t hash = trees->points[i].id * 2654435761u;
            float yaw_01 = (float)(hash & 0xFF) / 255.0f;
            float scale_01 = (float)((hash >> 8) & 0xFF) / 255.0f;
            batch->instances[idx].yaw_scale =
                (float)((int)(yaw_01 * 256.0f)) + scale_01;
            idx++;
        }
    }
}

/* Labels — lay out POI glyph instances */

void arpt_prepare_labels(const arpt_poi_data *pois, const font_glyph *glyphs,
                         float font_height, const icon_glyph *icon_glyphs,
                         int num_icons, float icon_height,
                         arpt_label_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!pois || pois->count == 0 || !glyphs) return;

    float font_size = font_height;
    if (font_size < 1.0f) font_size = 40.0f;
    float icon_size = icon_height;
    if (icon_size < 1.0f) icon_size = 64.0f;

    /* Count total renderable glyphs */
    size_t total_glyphs = 0;
    for (size_t i = 0; i < pois->count; i++) {
        const char *p = pois->points[i].name;
        while (*p) {
            uint32_t cp = font_utf8_decode(&p);
            if (cp < (uint32_t)FONT_FIRST_CHAR || cp > (uint32_t)FONT_LAST_CHAR)
                continue;
            int gi = (int)(cp - FONT_FIRST_CHAR);
            if (glyphs[gi].width > 0) total_glyphs++;
        }
    }
    if (total_glyphs == 0) return;

    out->glyphs = malloc(total_glyphs * sizeof(arpt_glyph_inst));
    out->labels = malloc(pois->count * sizeof(arpt_label_meta));
    out->icons = malloc(pois->count * sizeof(arpt_icon_inst));
    if (!out->glyphs || !out->labels || !out->icons) {
        free(out->glyphs);
        free(out->labels);
        free(out->icons);
        memset(out, 0, sizeof(*out));
        return;
    }

    size_t idx = 0;
    int label_count = 0;
    size_t icon_idx = 0;

    for (size_t i = 0; i < pois->count; i++) {
        const arpt_poi_point *p = &pois->points[i];
        const char *name = p->name;

        /* Compute total string width in pixels */
        float total_w = 0;
        float max_h = 0;
        const char *sp = name;
        while (*sp) {
            uint32_t cp = font_utf8_decode(&sp);
            if (cp < (uint32_t)FONT_FIRST_CHAR || cp > (uint32_t)FONT_LAST_CHAR)
                cp = FONT_FIRST_CHAR;
            const font_glyph *g = &glyphs[cp - FONT_FIRST_CHAR];
            total_w += g->advance;
            if (g->height > max_h) max_h = g->height;
        }
        float half_w = total_w * 0.5f;

        uint32_t first_inst = (uint32_t)idx;

        /* Emit glyph instances */
        float cursor = 0;
        sp = name;
        while (*sp) {
            uint32_t cp = font_utf8_decode(&sp);
            if (cp < (uint32_t)FONT_FIRST_CHAR || cp > (uint32_t)FONT_LAST_CHAR)
                cp = FONT_FIRST_CHAR;
            int gi = (int)(cp - FONT_FIRST_CHAR);
            const font_glyph *g = &glyphs[gi];

            if (g->width > 0) {
                out->glyphs[idx].qx = p->qx;
                out->glyphs[idx].qy = p->qy;
                out->glyphs[idx].qz = p->z;
                out->glyphs[idx].u0 = g->u0;
                out->glyphs[idx].v0 = g->v0;
                out->glyphs[idx].u1 = g->u1;
                out->glyphs[idx].v1 = g->v1;
                out->glyphs[idx].ox =
                    (cursor + g->bearing_x - half_w) / font_size;
                out->glyphs[idx].oy = 0.8f - g->bearing_y / font_size;
                idx++;
            }
            cursor += g->advance;
        }

        uint32_t glyph_count = (uint32_t)idx - first_inst;
        if (glyph_count > 0) {
            arpt_label_meta *lm = &out->labels[label_count++];
            lm->qx = p->qx;
            lm->qy = p->qy;
            lm->qz = p->z;
            lm->w_px = total_w;
            lm->h_px = max_h;
            lm->first = first_inst;
            lm->count = glyph_count;
        }

        /* Emit icon instance (one per POI, centered above the text) */
        int ii = icon_find(p->icon);
        if (ii >= 0 && ii < num_icons && icon_glyphs[ii].width > 0) {
            const icon_glyph *ig = &icon_glyphs[ii];
            out->icons[icon_idx].qx = p->qx;
            out->icons[icon_idx].qy = p->qy;
            out->icons[icon_idx].qz = p->z;
            out->icons[icon_idx].u0 = ig->u0;
            out->icons[icon_idx].v0 = ig->v0;
            out->icons[icon_idx].u1 = ig->u1;
            out->icons[icon_idx].v1 = ig->v1;
            /* Center the icon on the POI location */
            out->icons[icon_idx].ox = -ig->width * 0.5f / icon_size;
            out->icons[icon_idx].oy = ig->height * 0.5f / icon_size;
            icon_idx++;
        }
    }

    out->glyph_count = idx;
    out->label_count = label_count;
    out->icon_count = icon_idx;
}

/* Line labels — copy named polylines and pre-measure their text */

/* Keep at most this many street-label candidates per tile; the tiler
   orders features most-important first, so the head of the list wins. */
#define ARPT_MAX_LINE_LABELS_PER_TILE 64

/* Total advance of `name` in pixels at the atlas font size; 0 when no
   glyph is renderable. */
static float measure_text_width(const char *name, const font_glyph *glyphs) {
    float total_w = 0;
    bool any = false;
    while (*name) {
        uint32_t cp = font_utf8_decode(&name);
        if (cp < (uint32_t)FONT_FIRST_CHAR || cp > (uint32_t)FONT_LAST_CHAR)
            cp = FONT_FIRST_CHAR;
        const font_glyph *g = &glyphs[cp - FONT_FIRST_CHAR];
        total_w += g->advance;
        if (g->width > 0) any = true;
    }
    return any ? total_w : 0.0f;
}

/* Terrain elevation (mm) at quantized tile position (qx, qy): the z of the
   nearest mesh vertex. Streets follow the terrain surface, so anchoring
   their labels at the local elevation keeps them on the road in hilly
   tiles instead of floating at the ellipsoid. */
static int32_t terrain_elevation_at(const arpt_terrain_mesh *terrain,
                                    uint16_t qx, uint16_t qy) {
    if (!terrain || terrain->vertex_count == 0 || !terrain->z) return 0;
    size_t best = 0;
    int64_t best_d2 = INT64_MAX;
    for (size_t i = 0; i < terrain->vertex_count; i++) {
        int64_t dx = (int64_t)terrain->x[i] - qx;
        int64_t dy = (int64_t)terrain->y[i] - qy;
        int64_t d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return terrain->z[best];
}

void arpt_prepare_line_labels(const arpt_line_label_data *data,
                              const arpt_terrain_mesh *terrain,
                              const font_glyph *glyphs,
                              arpt_line_label_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!data || data->count == 0 || !glyphs) return;

    size_t max = data->count;
    if (max > ARPT_MAX_LINE_LABELS_PER_TILE)
        max = ARPT_MAX_LINE_LABELS_PER_TILE;

    out->labels = calloc(max, sizeof(arpt_line_label));
    if (!out->labels) return;

    int count = 0;
    for (size_t i = 0; i < data->count && count < (int)max; i++) {
        const arpt_line_label_feature *f = &data->features[i];
        if (f->vertex_count < 2 ||
            f->vertex_count > ARPT_MAX_LINE_LABEL_POINTS)
            continue;

        float text_w = measure_text_width(f->name, glyphs);
        if (text_w <= 0.0f) continue;

        arpt_line_label *ll = &out->labels[count];
        ll->x = malloc(f->vertex_count * sizeof(uint16_t));
        ll->y = malloc(f->vertex_count * sizeof(uint16_t));
        if (!ll->x || !ll->y) {
            free(ll->x);
            free(ll->y);
            ll->x = ll->y = NULL;
            break;
        }
        memcpy(ll->x, f->x, f->vertex_count * sizeof(uint16_t));
        memcpy(ll->y, f->y, f->vertex_count * sizeof(uint16_t));
        ll->vertex_count = (uint32_t)f->vertex_count;
        size_t mid = f->vertex_count / 2;
        ll->qz = terrain_elevation_at(terrain, f->x[mid], f->y[mid]);
        memcpy(ll->name, f->name, sizeof(ll->name));
        ll->text_w_px = text_w;
        count++;
    }

    out->count = count;
    if (count == 0) {
        free(out->labels);
        out->labels = NULL;
    }
}

/* Cleanup */

void arpt_tile_prims_free(arpt_tile_prims *p) {
    if (!p) return;
    /* terrain: zero-copy, nothing to free */

    /* polygons */
    free(p->polygons.verts);
    free(p->polygons.indices);
    free(p->polygons.groups);

    /* lines */
    free(p->lines.verts);
    free(p->lines.indices);

    /* buildings */
    free(p->buildings.xy);
    free(p->buildings.z);
    free(p->buildings.normals);
    free(p->buildings.indices);

    /* instances */
    if (p->instances.batches) {
        for (int i = 0; i < p->instances.batch_count; i++)
            free(p->instances.batches[i].instances);
        free(p->instances.batches);
    }

    /* labels */
    free(p->labels.glyphs);
    free(p->labels.labels);
    free(p->labels.icons);

    /* line labels */
    if (p->line_labels.labels) {
        for (int i = 0; i < p->line_labels.count; i++) {
            free(p->line_labels.labels[i].x);
            free(p->line_labels.labels[i].y);
        }
        free(p->line_labels.labels);
    }
}
