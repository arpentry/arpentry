#include "town.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * Procedural town layout centered at (0, 0).
 *
 * An 8x8 jittered grid of intersections produces ~180 road segments
 * with visible bends.  Buildings line both sides of each road.
 * All arrays are file-scope static (~35 KB), generated lazily on
 * first API call via ensure_generated().
 *
 * At the equator 1 degree ~ 111 320 m, so 0.001 deg ~ 111 m.
 */

#define DEG_PER_M (1.0 / 111319.5)

/* Town bounding box (~1800 m to cover node jitter + building setbacks) */
#define TOWN_WEST  (-0.0080)
#define TOWN_EAST    0.0080
#define TOWN_SOUTH (-0.0080)
#define TOWN_NORTH   0.0080

/* Grid parameters */
#define GRID_N       8
#define GRID_SPAN_M  1492.0
#define GRID_CELL_M  (GRID_SPAN_M / (GRID_N - 1))  /* ~213 m */
#define JITTER_FRAC  0.25                            /* +/-25% of cell */

/* Road parameters */
#define PRIMARY_IDX  3       /* row/col index for primary roads */
#define SKIP_FRAC    0.20    /* fraction of non-primary edges randomly dropped */
#define BEND_FRAC    0.15    /* perpendicular offset as fraction of edge length */

/* Building placement */
#define BLDG_STEP_M  30.0   /* walk interval along each road segment */
#define SETBACK_MIN   3.0   /* min distance from building edge to road */
#define SETBACK_MAX   8.0
#define BLDG_W_MIN   10.0   /* east-west footprint range (meters) */
#define BLDG_W_MAX   28.0
#define BLDG_D_MIN    8.0   /* north-south footprint range (meters) */
#define BLDG_D_MAX   22.0
#define TOWN_HALF_M  750.0  /* building centers must stay within this radius */

/* Static array capacities */
#define MAX_ROADS     256
#define MAX_BUILDINGS 512

/* ── PRNG (xorshift32) ─────────────────────────────────────────────── */

static uint32_t rng_state;

static uint32_t xorshift32(void) {
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

static double rng_01(void) {
    return (double)(xorshift32() & 0x7FFFFFFF) / 2147483648.0;
}

static double rng_range(double lo, double hi) {
    return lo + rng_01() * (hi - lo);
}

/* ── Static storage ────────────────────────────────────────────────── */

static bool generated;
static town_road road_buf[MAX_ROADS];
static int n_roads;
static town_building bldg_buf[MAX_BUILDINGS];
static int n_bldgs;

static double node_x[GRID_N][GRID_N]; /* meters, east-west */
static double node_y[GRID_N][GRID_N]; /* meters, north-south */

/* ── 1. Node generation ────────────────────────────────────────────── */

static void gen_nodes(void) {
    double half = GRID_SPAN_M * 0.5;
    double jitter = GRID_CELL_M * JITTER_FRAC;
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N; c++) {
            node_x[r][c] = -half + c * GRID_CELL_M + rng_range(-jitter, jitter);
            node_y[r][c] = -half + r * GRID_CELL_M + rng_range(-jitter, jitter);
        }
}

/* ── 2. Road generation ────────────────────────────────────────────── */

static void push_road(double x1, double y1, double x2, double y2, int cls) {
    if (n_roads >= MAX_ROADS) return;
    town_road *r = &road_buf[n_roads++];
    r->lon1 = x1 * DEG_PER_M;
    r->lat1 = y1 * DEG_PER_M;
    r->lon2 = x2 * DEG_PER_M;
    r->lat2 = y2 * DEG_PER_M;
    r->cls = cls;
}

/* Connect two nodes with a bend at the midpoint. */
static void add_edge(double ax, double ay, double bx, double by, int cls) {
    double mx = (ax + bx) * 0.5;
    double my = (ay + by) * 0.5;
    double dx = bx - ax, dy = by - ay;
    double len = sqrt(dx * dx + dy * dy);
    if (len > 0.01) {
        double off = rng_range(-BEND_FRAC, BEND_FRAC) * len;
        mx += (-dy / len) * off;
        my += (dx / len) * off;
    }
    push_road(ax, ay, mx, my, cls);
    push_road(mx, my, bx, by, cls);
}

static void gen_roads(void) {
    n_roads = 0;
    /* Horizontal edges: connect (r, c) to (r, c+1) */
    for (int r = 0; r < GRID_N; r++)
        for (int c = 0; c < GRID_N - 1; c++) {
            bool pri = (r == PRIMARY_IDX);
            if (!pri && rng_01() < SKIP_FRAC) continue;
            add_edge(node_x[r][c], node_y[r][c],
                     node_x[r][c + 1], node_y[r][c + 1],
                     pri ? TOWN_VAL_PRIMARY : TOWN_VAL_RESIDENTIAL);
        }
    /* Vertical edges: connect (r, c) to (r+1, c) */
    for (int r = 0; r < GRID_N - 1; r++)
        for (int c = 0; c < GRID_N; c++) {
            bool pri = (c == PRIMARY_IDX);
            if (!pri && rng_01() < SKIP_FRAC) continue;
            add_edge(node_x[r][c], node_y[r][c],
                     node_x[r + 1][c], node_y[r + 1][c],
                     pri ? TOWN_VAL_PRIMARY : TOWN_VAL_RESIDENTIAL);
        }
}

/* ── 3. Building placement ─────────────────────────────────────────── */

static bool overlaps_any(double cx, double cy, double w, double h) {
    double hw = w * 0.5, hh = h * 0.5;
    for (int i = 0; i < n_bldgs; i++) {
        double ex = bldg_buf[i].lon / DEG_PER_M;
        double ey = bldg_buf[i].lat / DEG_PER_M;
        double ehw = bldg_buf[i].w_m * 0.5;
        double ehh = bldg_buf[i].h_m * 0.5;
        if (cx - hw < ex + ehw && cx + hw > ex - ehw &&
            cy - hh < ey + ehh && cy + hh > ey - ehh)
            return true;
    }
    return false;
}

/* Choose building height value based on distance from town center.
 *
 * Available height values:
 *   TOWN_VAL_H5  (5 m)   TOWN_VAL_H8  (8 m)   TOWN_VAL_H10 (10 m)
 *   TOWN_VAL_H12 (12 m)  TOWN_VAL_H15 (15 m)
 *
 * dist_m: distance from (0,0) in meters, range [0, ~1060].
 * Returns: one of the TOWN_VAL_H* constants.
 */
static int choose_height(double dist_m) {
    if (dist_m < 150.0) return TOWN_VAL_H15;
    if (dist_m < 300.0) return TOWN_VAL_H12;
    if (dist_m < 450.0) return TOWN_VAL_H10;
    if (dist_m < 600.0) return TOWN_VAL_H8;
    return TOWN_VAL_H5;
}

static void place_along_road(int ri) {
    double ax = road_buf[ri].lon1 / DEG_PER_M;
    double ay = road_buf[ri].lat1 / DEG_PER_M;
    double bx = road_buf[ri].lon2 / DEG_PER_M;
    double by = road_buf[ri].lat2 / DEG_PER_M;
    double dx = bx - ax, dy = by - ay;
    double len = sqrt(dx * dx + dy * dy);
    if (len < 1.0) return;

    /* Perpendicular to road direction */
    double px = -dy / len, py = dx / len;
    int steps = (int)(len / BLDG_STEP_M);

    for (int s = 1; s < steps; s++) {
        double t = (double)s / (double)steps;
        double rx = ax + dx * t;
        double ry = ay + dy * t;

        /* Try both sides of the road */
        for (int side = -1; side <= 1; side += 2) {
            if (n_bldgs >= MAX_BUILDINGS) return;

            double w = rng_range(BLDG_W_MIN, BLDG_W_MAX);
            double d = rng_range(BLDG_D_MIN, BLDG_D_MAX);
            double setback = rng_range(SETBACK_MIN, SETBACK_MAX);
            double off = setback + fmax(w, d) * 0.5;

            double cx = rx + side * px * off;
            double cy = ry + side * py * off;

            /* Stay within town bounds */
            double hw = w * 0.5, hd = d * 0.5;
            if (cx - hw < -TOWN_HALF_M || cx + hw > TOWN_HALF_M) continue;
            if (cy - hd < -TOWN_HALF_M || cy + hd > TOWN_HALF_M) continue;

            if (overlaps_any(cx, cy, w, d)) continue;

            double dist = sqrt(cx * cx + cy * cy);
            town_building *b = &bldg_buf[n_bldgs++];
            b->lon = cx * DEG_PER_M;
            b->lat = cy * DEG_PER_M;
            b->w_m = w;
            b->h_m = d;
            b->cls = TOWN_VAL_BUILDING;
            b->height_val = choose_height(dist);
        }
    }
}

static void gen_buildings(void) {
    n_bldgs = 0;
    for (int i = 0; i < n_roads; i++)
        place_along_road(i);
}

/* ── Lazy initialization ───────────────────────────────────────────── */

static void ensure_generated(void) {
    if (generated) return;
    rng_state = 0xBEEF1234;
    gen_nodes();
    gen_roads();
    gen_buildings();
    generated = true;
}

/* ── Public API ────────────────────────────────────────────────────── */

bool town_overlaps(arpt_bounds bounds) {
    return bounds.east > TOWN_WEST && bounds.west < TOWN_EAST &&
           bounds.north > TOWN_SOUTH && bounds.south < TOWN_NORTH;
}

int town_road_count(void) {
    ensure_generated();
    return n_roads;
}

const town_road *town_get_roads(void) {
    ensure_generated();
    return road_buf;
}

int town_building_count(void) {
    ensure_generated();
    return n_bldgs;
}

const town_building *town_get_buildings(void) {
    ensure_generated();
    return bldg_buf;
}
