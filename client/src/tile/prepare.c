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

/* Tiles from this level up stroke drivable roads at the physical `width_m`
   the tiler baked from its engineering priors — the same numbers that size
   the bridge decks — so paint and structures meet edge-to-edge. Coarser
   tiles keep the cartographic style widths. Matches the tiler's
   STRUCTURE_DETAIL_MIN_ZOOM so widths turn physical alongside the piers. */
#define LINE_PHYSICAL_WIDTH_MIN_LEVEL 13

/* Metres per degree matching the tiler's ENU frame (server building_mesh),
   so a stroked width in metres agrees with the swept structure boxes. */
#define LINE_M_PER_DEG_LAT 110540.0
#define LINE_M_PER_DEG_LON_EQ 111320.0

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

/* The road network arrives as a consistent 3D model: the tiler reconstructs each
   road's elevation from the terrain surface and bakes it per vertex (see server
   `structures`/`terrain::surface_height`), so the client just strokes the road
   at the supplied heights — no terrain sampling, no draping. */

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

static void emit_subpolyline(const uint16_t *px, const uint16_t *py,
                             const int32_t *pz, size_t vc,
                             double hw, double ax, double ay,
                             const float color[4],
                             arpt_line_vertex *verts, uint32_t *idxs,
                             size_t *vi, size_t *ii);

/* Width of the overlap kept on each side of the tile when clipping roads, in
   quantized units.  Adjacent tiles each extend this far past the shared edge so
   their roads overlap and join with no seam; small enough that the flat
   edge-elevation extrapolation over it is negligible. */
#define LINE_TILE_OVERLAP 192.0

/* Emit a road stroke for one polyline at its baked elevations.  The polyline is
   first clipped to the tile-proper bounds (plus a small overlap): the tiler keeps
   a half-tile line buffer, but the overhang is redundant — the neighbouring tile
   strokes it from its own copy.  Clipping to the *canonical* tile bounds
   (identical for adjacent tiles) makes neighbouring tiles' roads meet on exactly
   the same world line; the overlap margin hides the seam. */
static void emit_polyline(const arpt_line_feature *line, double hw,
                          double ax, double ay, const float color[4],
                          arpt_line_vertex *verts, uint32_t *idxs,
                          size_t *vi, size_t *ii) {
    size_t vc = line->vertex_count;
    if (vc < 2) return;
    /* Drop closing vertex of a closed loop; the renderer draws open polylines. */
    if (vc >= 3 && line->x[0] == line->x[vc - 1] &&
        line->y[0] == line->y[vc - 1])
        vc--;

    /* Per-vertex road elevation the tiler baked from the terrain surface; NULL on
       the DEM-less path, where the road lies flat at z = 0. */
    const int32_t *lz = line->z;

    double xmin = ARPT_BUFFER - LINE_TILE_OVERLAP;
    double xmax = ARPT_BUFFER + ARPT_EXTENT + LINE_TILE_OVERLAP;
    double ymin = xmin, ymax = xmax;

    /* Accumulate clipped runs; flush a sub-polyline whenever the line leaves the
       rect (a segment is fully out, or enters/exits partway).  `sz` interpolates
       the baked elevation through each clip cut; the centerline is densely
       sampled by the tiler, so linear interpolation there matches. */
    uint16_t *sx = malloc((vc + 1) * sizeof(uint16_t));
    uint16_t *sy = malloc((vc + 1) * sizeof(uint16_t));
    int32_t *sz = malloc((vc + 1) * sizeof(int32_t));
    if (!sx || !sy || !sz) { free(sx); free(sy); free(sz); return; }
    size_t sn = 0;

#define FLUSH_RUN()                                                            \
    do {                                                                        \
        if (sn >= 2)                                                            \
            emit_subpolyline(sx, sy, sz, sn, hw, ax, ay, color, verts,         \
                             idxs, vi, ii);                                     \
        sn = 0;                                                                  \
    } while (0)

    for (size_t s = 0; s + 1 < vc; s++) {
        /* Named x0/y0 (not ax/ay): the metre-scale params `ax`/`ay` are what
           FLUSH_RUN passes to emit_subpolyline, and this loop calls FLUSH_RUN.
           Shadowing them with vertex coords here would feed the emitter garbage
           scales and collapse every clipped run to zero width. */
        double x0 = line->x[s], y0 = line->y[s];
        double bx = line->x[s + 1], by = line->y[s + 1];
        double za = lz ? (double)lz[s] : 0.0;
        double zb = lz ? (double)lz[s + 1] : 0.0;
        double t0, t1;
        if (!clip_segment(x0, y0, bx, by, xmin, ymin, xmax, ymax, &t0, &t1)) {
            FLUSH_RUN();
            continue;
        }
        double cax = x0 + (bx - x0) * t0, cay = y0 + (by - y0) * t0;
        double cbx = x0 + (bx - x0) * t1, cby = y0 + (by - y0) * t1;
        if (sn == 0) {
            sx[sn] = (uint16_t)lround(cax); sy[sn] = (uint16_t)lround(cay);
            sz[sn] = (int32_t)lround(za + (zb - za) * t0); sn++;
        } else if (t0 > 1e-9) {
            /* Segment re-entered the rect: break the run and restart here. */
            FLUSH_RUN();
            sx[sn] = (uint16_t)lround(cax); sy[sn] = (uint16_t)lround(cay);
            sz[sn] = (int32_t)lround(za + (zb - za) * t0); sn++;
        }
        sx[sn] = (uint16_t)lround(cbx); sy[sn] = (uint16_t)lround(cby);
        sz[sn] = (int32_t)lround(za + (zb - za) * t1); sn++;
        if (t1 < 1.0 - 1e-9) FLUSH_RUN(); /* segment exited the rect */
    }
    FLUSH_RUN();

#undef FLUSH_RUN
    free(sx); free(sy); free(sz);
}

/* Emit SDF quads for one (already clipped) sub-polyline at the given half-width
   and color.  Each vertex carries the road elevation the tiler reconstructed from
   the terrain surface (`pz`, the same source the bridge/tunnel solids ride), so
   the stroke follows the consistent 3D road model.  The polyline is already
   densely sampled by the tiler, so the vertices are used as-is.

   All stroke math runs in tile-local METRES: positions are scaled by the
   per-axis metres-per-unit (`ax`, `ay` — plate-carrée tiles are anisotropic,
   a lon unit shrinking with cos(lat)) before directions, normals, miters and
   offsets are computed, and scaled back on emission.  The half-width `hw` is
   in metres, so a stroke holds the same ground width whatever its bearing —
   and matches the swept structure boxes, which are sized in metres too. */
static void emit_subpolyline(const uint16_t *px, const uint16_t *py,
                             const int32_t *pz, size_t vc,
                             double hw, double ax, double ay,
                             const float color[4],
                             arpt_line_vertex *verts, uint32_t *idxs,
                             size_t *vi, size_t *ii) {
    const float *c = color;
    if (vc < 2) return;
    size_t n_segs = vc - 1;
    /* Degenerate-segment floor, in metres: one y-unit, matching the old
       one-quantized-unit threshold. */
    double min_seg = ay;

    /* Pre-compute per-segment direction and normal (metres) */
    double *seg_nx = malloc(n_segs * sizeof(double));
    double *seg_ny = malloc(n_segs * sizeof(double));
    double *seg_ux = malloc(n_segs * sizeof(double));
    double *seg_uy = malloc(n_segs * sizeof(double));
    double *seg_len = malloc(n_segs * sizeof(double));
    if (!seg_nx || !seg_ny || !seg_ux || !seg_uy || !seg_len) {
        free(seg_nx); free(seg_ny); free(seg_ux); free(seg_uy);
        free(seg_len);
        return;
    }

    bool any_valid = false;
    for (size_t s = 0; s < n_segs; s++) {
        double dx = ((double)px[s + 1] - px[s]) * ax;
        double dy = ((double)py[s + 1] - py[s]) * ay;
        double len = sqrt(dx * dx + dy * dy);
        if (len < 1e-6) len = 1e-6;
        seg_ux[s] = dx / len;
        seg_uy[s] = dy / len;
        seg_nx[s] = -seg_uy[s];
        seg_ny[s] = seg_ux[s];
        seg_len[s] = len;
        if (len >= min_seg) any_valid = true;
    }

    if (!any_valid) {
        free(seg_nx); free(seg_ny); free(seg_ux); free(seg_uy);
        free(seg_len);
        return;
    }

#define CLAMP16(v) ((uint16_t)((v) < 0 ? 0 : (v) > 65535 ? 65535 : (v)))
    /* Emit one road vertex from tile-local metre coordinates.  The elevation
       `vz` is the baked centerline height at this segment end, shared by both
       edge corners so the cross-section stays horizontal and the ribbon hugs
       the road, not the cross-slope. */
    /* `cmx, cmy` are the centerline (metre) coords this vertex is offset from;
       the shader projects both to floor the stroke's screen-space width. */
#define EMIT_V(mx, my, cmx, cmy, vz, lu, lv)                                   \
    do {                                                                        \
        verts[*vi] = (arpt_line_vertex){CLAMP16(lround((mx) / ax)),            \
                                        CLAMP16(lround((my) / ay)), (vz),      \
                                        c[0], c[1], c[2], c[3],                \
                                        (float)(lu), (float)(lv),              \
                                        (float)hw, (float)len,                 \
                                        CLAMP16(lround((cmx) / ax)),           \
                                        CLAMP16(lround((cmy) / ay))};          \
        (*vi)++;                                                                \
    } while (0)

    for (size_t s = 0; s < n_segs; s++) {
        double len = seg_len[s];
        if (len < min_seg) continue;

        double x1 = px[s] * ax, y1 = py[s] * ay;
        double x2 = px[s + 1] * ax, y2 = py[s + 1] * ay;

        /* Offset vectors at start and end of this segment.
         * At interior vertices, use a miter between adjacent
         * segment normals so consecutive quads share edges. */
        double m1x, m1y, m2x, m2y;

        if (s > 0 && seg_len[s - 1] >= min_seg) {
            /* Miter at start vertex */
            double mx = seg_nx[s - 1] + seg_nx[s];
            double my = seg_ny[s - 1] + seg_ny[s];
            double d = mx * seg_nx[s] + my * seg_ny[s];
            /* Miter limit: cap at 2x. Both edge corners of a quad carry the
               centerline's height (a horizontal cross-section), so a miter tip
               offset far from the centerline hangs at the wrong elevation over a
               steep flank and is eaten by the terrain despite the road's depth
               margin. Capping the spike at 2x half-width keeps that burial within
               the margin — a tighter cap than the visual 4x, but sharp turns are
               rare on the tiler's densified centerlines. */
            if (d < 0.5) d = 0.5;
            m1x = mx / d;
            m1y = my / d;
        } else {
            m1x = seg_nx[s];
            m1y = seg_ny[s];
        }

        if (s + 1 < n_segs && seg_len[s + 1] >= min_seg) {
            /* Miter at end vertex */
            double mx = seg_nx[s] + seg_nx[s + 1];
            double my = seg_ny[s] + seg_ny[s + 1];
            double d = mx * seg_nx[s] + my * seg_ny[s];
            if (d < 0.5) d = 0.5;  /* miter limit: cap at 2x (see start miter) */
            m2x = mx / d;
            m2y = my / d;
        } else {
            m2x = seg_nx[s];
            m2y = seg_ny[s];
        }

        /* Extend endpoints along tangent for caps at polyline ends */
        double ex1 = x1, ey1 = y1, ex2 = x2, ey2 = y2;
        double cap1 = 0.0, cap2 = 0.0;
        if (s == 0 || seg_len[s - 1] < min_seg) {
            ex1 -= seg_ux[s] * hw;
            ey1 -= seg_uy[s] * hw;
            cap1 = hw;
        }
        if (s + 1 == n_segs || seg_len[s + 1] < min_seg) {
            ex2 += seg_ux[s] * hw;
            ey2 += seg_uy[s] * hw;
            cap2 = hw;
        }

        /* Baked road elevation at each segment end (cap extension reuses the end
           vertex's height — caps are short). Shared by both edge corners. */
        int32_t z1 = pz[s];
        int32_t z2 = pz[s + 1];

        uint32_t base = (uint32_t)*vi;
        EMIT_V(ex1 - m1x * hw, ey1 - m1y * hw, ex1, ey1, z1, -cap1, -hw);
        EMIT_V(ex1 + m1x * hw, ey1 + m1y * hw, ex1, ey1, z1, -cap1, hw);
        EMIT_V(ex2 + m2x * hw, ey2 + m2y * hw, ex2, ey2, z2, len + cap2, hw);
        EMIT_V(ex2 - m2x * hw, ey2 - m2y * hw, ex2, ey2, z2, len + cap2, -hw);

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
    free(seg_len);
}

/* Resolve the rendered half-width for one stroke, in METRES.  A drivable road
   on a close-zoom tile takes its physical `width_m` (the tiler's engineering
   prior, the same number that sizes its bridge decks) so paint and structures
   meet edge-to-edge; everything else takes the cartographic style width,
   zoom-scaled and converted from quantized units at the y-axis metre scale. */
static double resolve_half_width_m(const arpt_line_feature *line,
                                   const arpt_style *style, int level,
                                   double zoom_scale, double ay) {
    if (line->width_m > 0.0f && level >= LINE_PHYSICAL_WIDTH_MIN_LEVEL)
        return line->width_m * 0.5;
    return style->stroke_widths[line->cls] * zoom_scale * ay;
}

/* Emit every line feature as a single filled stroke (no casing outline).
   Features arrive sorted by class, so later style entries draw on top. */
static void emit_line_sdf_quads(const arpt_line_data *data,
                                const arpt_style *style, int level,
                                double ax, double ay,
                                arpt_line_vertex *verts, uint32_t *idxs,
                                size_t *vi, size_t *ii) {
    if (!data) return;
    double zs = line_zoom_scale(level);
    for (size_t i = 0; i < data->count; i++) {
        const arpt_line_feature *line = &data->lines[i];
        if (style->stroke_widths[line->cls] <= 0.0f) continue;
        double hw = resolve_half_width_m(line, style, level, zs, ay);
        float color[4];
        memcpy(color, style->colors[line->cls], sizeof(color));
        emit_polyline(line, hw, ax, ay, color, verts, idxs, vi, ii);
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
                        arpt_bounds bounds, arpt_line_prim *out) {
    memset(out, 0, sizeof(*out));
    if (!line_data) return;

    /* Metres per quantized unit along each tile axis: plate-carrée tiles are
       anisotropic (a lon unit shrinks with cos(lat)), so the stroke emitter
       works in metres and converts back per axis. */
    double lat_c = 0.5 * (bounds.south + bounds.north);
    double ay = (bounds.north - bounds.south) * LINE_M_PER_DEG_LAT
                / (double)ARPT_EXTENT;
    double ax = (bounds.east - bounds.west) * LINE_M_PER_DEG_LON_EQ
                * cos(lat_c * M_PI / 180.0) / (double)ARPT_EXTENT;
    if (ax <= 0.0 || ay <= 0.0) return;

    /* Upper bound: every feature emits one filled stroke, one quad per segment.
       The tiler already densified the centerline, so the vertices are used as-is. */
    size_t nv = 0, ni = 0;
    for (size_t i = 0; i < line_data->count; i++) {
        const arpt_line_feature *line = &line_data->lines[i];
        if (line->vertex_count < 2) continue;
        size_t segs = line->vertex_count - 1;
        nv += segs * 4;
        ni += segs * 6;
    }
    if (nv == 0 || ni == 0) return;

    out->verts = malloc(nv * sizeof(arpt_line_vertex));
    out->indices = malloc(ni * sizeof(uint32_t));
    if (out->verts && out->indices) {
        size_t vi = 0, ii = 0;
        emit_line_sdf_quads(line_data, style, level, ax, ay, out->verts,
                            out->indices, &vi, &ii);
        out->vert_count = vi;
        out->index_count = ii;
    } else {
        free(out->verts);
        free(out->indices);
        memset(out, 0, sizeof(*out));
    }
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
    free(p->buildings.color);

    /* road structures — bridges + tunnels (same form as buildings) */
    free(p->bridges.xy);
    free(p->bridges.z);
    free(p->bridges.normals);
    free(p->bridges.indices);
    free(p->bridges.color);
    free(p->tunnels.xy);
    free(p->tunnels.z);
    free(p->tunnels.normals);
    free(p->tunnels.indices);
    free(p->tunnels.color);

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
