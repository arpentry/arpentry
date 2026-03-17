#include "simplify.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Perpendicular distance from point (px,py) to line (ax,ay)-(bx,by). */
static double perp_dist(double px, double py,
                        double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    double len_sq = dx * dx + dy * dy;
    if (len_sq == 0.0) {
        dx = px - ax;
        dy = py - ay;
        return sqrt(dx * dx + dy * dy);
    }
    double area2 = fabs(dy * px - dx * py + bx * ay - by * ax);
    return area2 / sqrt(len_sq);
}

/* Run Douglas-Peucker on a contiguous segment [start..end] within
 * arrays x, y.  Marks surviving interior vertices in keep[]. */
static void dp_segment(const double *x, const double *y, bool *keep,
                        uint32_t start, uint32_t end, double tolerance,
                        uint32_t *stack, uint32_t *sp) {
    if (end <= start + 1) return;

    stack[(*sp)++] = start;
    stack[(*sp)++] = end;

    while (*sp >= 2) {
        uint32_t e = stack[--(*sp)];
        uint32_t s = stack[--(*sp)];

        double max_dist = 0.0;
        uint32_t max_idx = s;
        for (uint32_t i = s + 1; i < e; i++) {
            double d = perp_dist(x[i], y[i],
                                 x[s], y[s], x[e], y[e]);
            if (d > max_dist) {
                max_dist = d;
                max_idx = i;
            }
        }

        if (max_dist > tolerance) {
            keep[max_idx] = true;
            stack[(*sp)++] = s;
            stack[(*sp)++] = max_idx;
            stack[(*sp)++] = max_idx;
            stack[(*sp)++] = e;
        }
    }
}

/* Anchor-based Douglas-Peucker for open polylines.
 *
 * Standard DP uses the polyline's first and last vertex as the only
 * fixed points, then recursively splits.  Two polylines sharing a
 * sub-sequence but with different endpoints simplify the shared part
 * differently, creating gaps at junction points.
 *
 * Fix: identify "natural anchors" — interior vertices whose
 * perpendicular distance to the line between their two immediate
 * neighbors is >= tolerance.  These are significant bends that DP
 * would keep anyway.  Because anchor selection depends only on a
 * vertex and its two neighbors, shared sub-sequences in different
 * polylines produce the same anchors.  DP on the segments between
 * anchors is symmetric (works identically on reversed input), so
 * the simplification is consistent for shared edges. */
uint32_t arpt_simplify(double *x, double *y, uint32_t count, double tolerance) {
    if (count <= 2) return count;
    if (tolerance <= 0.0) return count;

    bool *keep = calloc(count, sizeof(*keep));
    if (!keep) return count;

    uint32_t *stack = malloc(count * 2 * sizeof(*stack));
    if (!stack) { free(keep); return count; }

    /* Endpoints are always kept */
    keep[0] = true;
    keep[count - 1] = true;

    /* Find natural anchors: interior vertices with high local curvature */
    for (uint32_t i = 1; i < count - 1; i++) {
        double d = perp_dist(x[i], y[i],
                             x[i - 1], y[i - 1],
                             x[i + 1], y[i + 1]);
        if (d >= tolerance) {
            keep[i] = true;
        }
    }

    /* Run DP on each segment between consecutive anchors */
    uint32_t prev_anchor = 0;
    for (uint32_t i = 1; i < count; i++) {
        if (keep[i]) {
            uint32_t sp = 0;
            dp_segment(x, y, keep, prev_anchor, i, tolerance, stack, &sp);
            prev_anchor = i;
        }
    }

    /* Compact in-place */
    uint32_t out = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (keep[i]) {
            x[out] = x[i];
            y[out] = y[i];
            out++;
        }
    }

    free(stack);
    free(keep);
    return out;
}

/* ---- Topology-preserving ring simplification ----
 *
 * Based on JTS TopologyPreservingSimplifier (Martin Davis, Vivid Solutions).
 *
 * Standard DP can create self-intersecting rings because it doesn't check
 * whether the flattened segment crosses other parts of the ring.  This
 * causes artifacts like land bridges across straits (Spain-Africa).
 *
 * Fix: before flattening a DP section, verify the proposed segment does
 * not have an interior intersection with any ring segment outside the
 * section.  If it would intersect, recurse deeper instead of flattening.
 *
 * The ring is split at natural anchors (significant bends) into arcs,
 * each processed with topology-aware DP.  Intersection checks run against
 * ALL original ring segments (conservative but safe). */

/* Orientation of triangle (ax,ay)-(bx,by)-(cx,cy).
 * Returns +1 (CCW), -1 (CW), or 0 (collinear). */
static int orient2d(double ax, double ay, double bx, double by,
                     double cx, double cy) {
    double d = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (d > 0.0) return 1;
    if (d < 0.0) return -1;
    return 0;
}

/* Check if point P is strictly in the interior of segment AB
 * (on the segment but not equal to either endpoint).
 * Assumes P is collinear with A-B. */
static bool on_seg_interior(double ax, double ay, double bx, double by,
                             double px, double py) {
    if ((px == ax && py == ay) || (px == bx && py == by))
        return false;
    double min_x = ax < bx ? ax : bx;
    double max_x = ax > bx ? ax : bx;
    double min_y = ay < by ? ay : by;
    double max_y = ay > by ? ay : by;
    return px >= min_x && px <= max_x && py >= min_y && py <= max_y;
}

/* Check if segments AB and CD have an interior intersection
 * (they cross, or one endpoint lies strictly inside the other segment).
 * Shared endpoints are NOT counted as interior intersections. */
static bool segs_interior_intersect(double ax, double ay, double bx, double by,
                                     double cx, double cy, double dx, double dy) {
    int o1 = orient2d(ax, ay, bx, by, cx, cy);
    int o2 = orient2d(ax, ay, bx, by, dx, dy);
    int o3 = orient2d(cx, cy, dx, dy, ax, ay);
    int o4 = orient2d(cx, cy, dx, dy, bx, by);

    /* Proper crossing: all orientations non-zero and opposing */
    if (o1 != 0 && o2 != 0 && o1 != o2 &&
        o3 != 0 && o4 != 0 && o3 != o4)
        return true;

    /* Degenerate: one endpoint on the other segment's interior */
    if (o1 == 0 && on_seg_interior(ax, ay, bx, by, cx, cy)) return true;
    if (o2 == 0 && on_seg_interior(ax, ay, bx, by, dx, dy)) return true;
    if (o3 == 0 && on_seg_interior(cx, cy, dx, dy, ax, ay)) return true;
    if (o4 == 0 && on_seg_interior(cx, cy, dx, dy, bx, by)) return true;

    return false;
}

/* Check if ring segment index k is in the section [start..end) mod n.
 * The section covers segments start, start+1, ..., end-1  (wrapping). */
static bool in_ring_section(uint32_t k, uint32_t start, uint32_t end,
                             uint32_t n) {
    if (start <= end) {
        return k >= start && k < end;
    }
    /* Wrapping: section goes start..n-1, 0..end-1 */
    return k >= start || k < end;
}

/* Check if flattening the arc [start→end] in a ring of n unique vertices
 * would cause the flat segment to have an interior intersection with
 * any ring segment outside the flattened section.
 *
 * Uses bounding-box pre-filter: only tests segments whose bbox overlaps
 * the proposed flat segment's bbox.  For large rings this skips the vast
 * majority of segments, turning the common case from O(n) to O(nearby). */
static bool creates_intersection(const double *x, const double *y,
                                  uint32_t n,
                                  uint32_t start, uint32_t end) {
    double ax = x[start], ay = y[start];
    double bx = x[end],   by = y[end];

    /* Bounding box of the proposed flat segment */
    double lo_x = ax < bx ? ax : bx;
    double hi_x = ax > bx ? ax : bx;
    double lo_y = ay < by ? ay : by;
    double hi_y = ay > by ? ay : by;

    for (uint32_t k = 0; k < n; k++) {
        if (in_ring_section(k, start, end, n)) continue;
        uint32_t k1 = (k + 1) % n;

        /* Quick bbox rejection: skip segments entirely outside the
         * flat segment's bounding box. */
        double sx0 = x[k], sy0 = y[k], sx1 = x[k1], sy1 = y[k1];
        double seg_lo_x = sx0 < sx1 ? sx0 : sx1;
        double seg_hi_x = sx0 > sx1 ? sx0 : sx1;
        double seg_lo_y = sy0 < sy1 ? sy0 : sy1;
        double seg_hi_y = sy0 > sy1 ? sy0 : sy1;
        if (seg_hi_x < lo_x || seg_lo_x > hi_x ||
            seg_hi_y < lo_y || seg_lo_y > hi_y)
            continue;

        if (segs_interior_intersect(ax, ay, bx, by, sx0, sy0, sx1, sy1))
            return true;
    }
    return false;
}

/* Topology-preserving DP for an arc of a ring.
 *
 * The arc goes from vertex start to vertex end, forward through a ring
 * of n unique vertices.  Marks surviving interior vertices in keep[].
 *
 * Before flattening any section, checks that the flat segment does not
 * intersect any ring segment outside the section.  If it would, the
 * farthest vertex is kept and the algorithm recurses. */
static void tp_dp_arc(const double *x, const double *y, uint32_t n,
                       bool *keep, uint32_t start, uint32_t end,
                       double tolerance,
                       uint32_t *stack, uint32_t *sp) {
    /* Compute arc length (number of vertices from start to end) */
    uint32_t arc_len;
    if (end >= start) {
        arc_len = end - start + 1;
    } else {
        arc_len = (n - start) + end + 1;
    }
    if (arc_len <= 2) return;

    /* Push initial section */
    stack[(*sp)++] = start;
    stack[(*sp)++] = end;

    while (*sp >= 2) {
        uint32_t sec_end   = stack[--(*sp)];
        uint32_t sec_start = stack[--(*sp)];

        /* Section vertex count */
        uint32_t sec_len;
        if (sec_end >= sec_start) {
            sec_len = sec_end - sec_start + 1;
        } else {
            sec_len = (n - sec_start) + sec_end + 1;
        }
        if (sec_len <= 2) continue;

        /* Find farthest vertex from line sec_start → sec_end.
         * Initialize max_idx to the first interior vertex so that
         * if all interior vertices are collinear (distance 0), the
         * split still makes progress. */
        double max_dist = 0.0;
        uint32_t max_idx = (sec_start + 1) % n;
        for (uint32_t i = 1; i < sec_len - 1; i++) {
            uint32_t vi = (sec_start + i) % n;
            double d = perp_dist(x[vi], y[vi],
                                  x[sec_start], y[sec_start],
                                  x[sec_end], y[sec_end]);
            if (d > max_dist) {
                max_dist = d;
                max_idx = vi;
            }
        }

        bool must_recurse = (max_dist > tolerance);

        if (!must_recurse) {
            /* Distance is within tolerance — check topology */
            if (creates_intersection(x, y, n, sec_start, sec_end))
                must_recurse = true;
        }

        if (must_recurse) {
            keep[max_idx] = true;
            stack[(*sp)++] = sec_start;
            stack[(*sp)++] = max_idx;
            stack[(*sp)++] = max_idx;
            stack[(*sp)++] = sec_end;
        }
        /* else: flatten — intermediate vertices stay unmarked */
    }
}

/* Shared-edge-consistent ring simplification.
 *
 * Key insight: adjacent polygons sharing an edge produce gaps when
 * Douglas-Peucker simplifies each ring independently, because the
 * pivot/split selection differs.  To fix this:
 *
 * 1. Identify "natural anchors" — vertices whose perpendicular distance
 *    to the line between their two original neighbors is >= tolerance.
 *    These are significant bends that DP would keep anyway.  Because
 *    the anchor decision depends ONLY on a vertex and its two immediate
 *    neighbors, shared edges between adjacent polygons produce the
 *    same anchors.
 *
 * 2. Split the ring into arcs at the natural anchors.
 *
 * 3. Run topology-preserving DP on each arc.  Since shared edges have
 *    the same anchors, the arcs along shared edges are identical (or
 *    reversed).  DP is symmetric (perp_dist doesn't depend on line
 *    direction), so reversed arcs produce identical results.
 *
 * 4. If fewer than 2 natural anchors exist (very smooth ring at this
 *    zoom), fall back to the classic pivot approach (vertex 0 +
 *    farthest). This fallback is feature-dependent but only triggers
 *    for very small or smooth features. */
uint32_t arpt_simplify_ring(double *x, double *y, uint32_t count,
                             double tolerance) {
    if (count <= 4) return count;   /* 3 unique + closing = minimum */
    if (tolerance <= 0.0) return count;

    /* Check if ring is closed (first == last vertex) */
    bool closed = (x[0] == x[count - 1] && y[0] == y[count - 1]);
    if (!closed) {
        return arpt_simplify(x, y, count, tolerance);
    }

    uint32_t n = count - 1;   /* unique vertices */
    if (n <= 3) return count;  /* triangle — nothing to simplify */

    bool *keep = calloc(n, sizeof(*keep));
    uint32_t *stack = malloc(n * 4 * sizeof(*stack));
    uint32_t *anchors = malloc(n * sizeof(*anchors));
    if (!keep || !stack || !anchors) {
        free(keep); free(stack); free(anchors);
        return count;
    }

    /* Find natural anchors: vertices with perp_dist >= tolerance */
    uint32_t n_anchors = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t prev = (i + n - 1) % n;
        uint32_t next = (i + 1) % n;
        double d = perp_dist(x[i], y[i],
                             x[prev], y[prev],
                             x[next], y[next]);
        if (d >= tolerance) {
            keep[i] = true;
            anchors[n_anchors++] = i;
        }
    }

    if (n_anchors >= 2) {
        /* Run topology-preserving DP on each arc between consecutive
         * anchors.  Each arc's endpoints (anchors) are already marked
         * in keep[]. */
        for (uint32_t a = 0; a < n_anchors; a++) {
            uint32_t start = anchors[a];
            uint32_t end = anchors[(a + 1) % n_anchors];
            uint32_t sp = 0;
            tp_dp_arc(x, y, n, keep, start, end, tolerance, stack, &sp);
        }
    } else {
        /* Fallback: classic pivot approach for very smooth rings.
         * This is feature-dependent but only triggers when the entire
         * ring is sub-tolerance (very small or smooth features). */
        memset(keep, 0, n * sizeof(*keep));

        /* Find pivot: vertex farthest from vertex 0 */
        uint32_t pivot = 1;
        double max_dsq = 0.0;
        for (uint32_t i = 1; i < n; i++) {
            double dx = x[i] - x[0];
            double dy = y[i] - y[0];
            double dsq = dx * dx + dy * dy;
            if (dsq > max_dsq) {
                max_dsq = dsq;
                pivot = i;
            }
        }

        keep[0] = true;
        keep[pivot] = true;

        /* DP on first arc: [0 → pivot] */
        uint32_t sp = 0;
        tp_dp_arc(x, y, n, keep, 0, pivot, tolerance, stack, &sp);

        /* DP on second arc: [pivot → 0] wrapping through end of ring */
        sp = 0;
        tp_dp_arc(x, y, n, keep, pivot, 0, tolerance, stack, &sp);
    }

    /* Compact in-place */
    uint32_t out = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (keep[i]) {
            x[out] = x[i];
            y[out] = y[i];
            out++;
        }
    }
    /* Re-close the ring */
    x[out] = x[0];
    y[out] = y[0];
    out++;

    free(anchors);
    free(stack);
    free(keep);
    return out;
}
