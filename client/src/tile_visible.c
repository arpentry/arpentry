#include "tile_manager.h"
#include "globe.h"
#include "coords.h"
#include "camera.h"
#include "math3d.h"
#include <math.h>

int arpt_enumerate_visible_tiles(const arpt_camera *cam, int level,
                                  arpt_tile_key *out, int max_count) {
    if (!cam || !out || max_count <= 0 || level < 0) return 0;

    int n_cols = 1 << (level + 1);
    int n_rows = 1 << level;

    /* At low zoom levels the total tile count is small enough to
       enumerate every tile.  This sidesteps all ray-casting edge cases
       (limb under-sampling, antimeridian, polar convergence) and
       guarantees no visible tile is ever missed. */
    int total_tiles = n_cols * n_rows;
    if (total_tiles <= max_count) {
        int count = 0;
        for (int y = 0; y < n_rows; y++)
            for (int x = 0; x < n_cols; x++)
                out[count++] = (arpt_tile_key){ level, x, y };
        return count;
    }

    /* At higher zoom levels the globe fills most of the viewport,
       so a coarse screen-space ray grid samples the visible region
       reliably.  Cast a 7x7 grid and build a geodetic bounding box. */
    int vp_w = arpt_camera_vp_width(cam);
    int vp_h = arpt_camera_vp_height(cam);
    if (vp_w <= 0 || vp_h <= 0) return 0;

    #define GRID_N 7
    #define GRID_COUNT (GRID_N * GRID_N)

    double lons[GRID_COUNT], lats[GRID_COUNT];
    int hit_count = 0;

    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            double sx = vp_w * (double)c / (GRID_N - 1);
            double sy = vp_h * (double)r / (GRID_N - 1);

            arpt_dvec3 origin, dir;
            if (!arpt_camera_screen_to_ray(cam, sx, sy, &origin, &dir))
                continue;

            double t;
            if (!arpt_ray_ellipsoid(origin, dir, &t))
                continue;

            arpt_dvec3 hit = arpt_dvec3_add(origin, arpt_dvec3_scale(dir, t));
            double lon, lat, alt;
            arpt_ecef_to_geodetic(hit, &lon, &lat, &alt);

            lons[hit_count] = lon * 180.0 / M_PI;
            lats[hit_count] = lat * 180.0 / M_PI;
            hit_count++;
        }
    }

    if (hit_count == 0) return 0;

    /* Detect antimeridian crossing: >180° gap means the shorter
       path wraps across the ±180° meridian. */
    bool crosses_antimeridian = false;
    for (int i = 0; i < hit_count && !crosses_antimeridian; i++) {
        for (int j = i + 1; j < hit_count; j++) {
            if (fabs(lons[i] - lons[j]) > 180.0) {
                crosses_antimeridian = true;
                break;
            }
        }
    }

    /* Shift to [0, 360) if crossing antimeridian */
    if (crosses_antimeridian) {
        for (int i = 0; i < hit_count; i++) {
            if (lons[i] < 0.0) lons[i] += 360.0;
        }
    }

    /* Compute bounding box */
    double min_lon = lons[0], max_lon = lons[0];
    double min_lat = lats[0], max_lat = lats[0];
    for (int i = 1; i < hit_count; i++) {
        if (lons[i] < min_lon) min_lon = lons[i];
        if (lons[i] > max_lon) max_lon = lons[i];
        if (lats[i] < min_lat) min_lat = lats[i];
        if (lats[i] > max_lat) max_lat = lats[i];
    }

    /* If the longitude span in [0,360) space exceeds 180°, the view covers
       more than half the globe (e.g. near a pole).  The antimeridian wrapping
       logic would invert the range, so fall back to the full longitude span. */
    if (crosses_antimeridian && (max_lon - min_lon) > 180.0) {
        crosses_antimeridian = false;
        min_lon = -180.0;
        max_lon = 180.0;
    }

    /* Shift back if we were in [0, 360) space */
    if (crosses_antimeridian) {
        if (min_lon > 180.0) min_lon -= 360.0;
        if (max_lon > 180.0) max_lon -= 360.0;
    }

    /* Clamp latitude */
    if (min_lat < -90.0) min_lat = -90.0;
    if (max_lat > 90.0) max_lat = 90.0;

    /* Convert bbox to tile indices */
    double tile_w = 360.0 / n_cols;
    double tile_h = 180.0 / n_rows;

    int x_min = (int)floor((min_lon + 180.0) / tile_w);
    int x_max = (int)floor((max_lon + 180.0) / tile_w);
    int y_min = (int)floor((min_lat + 90.0) / tile_h);
    int y_max = (int)floor((max_lat + 90.0) / tile_h);

    /* Pad by 1 tile: covers terrain that protrudes into the viewport
       even though its ellipsoid footprint is off-screen. */
    x_min -= 1;
    x_max += 1;
    y_min -= 1;
    y_max += 1;

    /* Clamp y to valid range */
    if (y_min < 0) y_min = 0;
    if (y_max >= n_rows) y_max = n_rows - 1;

    /* Generate tile list.
       When the x range wraps past the grid boundary (either from
       antimeridian crossing or from padding near lon ±180°), emit
       two sub-ranges: [x_min, n_cols-1] and [0, x_max]. */
    int count = 0;
    bool x_wraps = (x_min < 0 || x_max >= n_cols ||
                    (crosses_antimeridian && x_min > x_max));

    if (x_wraps) {
        int lo = ((x_min % n_cols) + n_cols) % n_cols;
        int hi = ((x_max % n_cols) + n_cols) % n_cols;

        for (int y = y_min; y <= y_max && count < max_count; y++) {
            for (int x = lo; x < n_cols && count < max_count; x++)
                out[count++] = (arpt_tile_key){ level, x, y };
            for (int x = 0; x <= hi && count < max_count; x++)
                out[count++] = (arpt_tile_key){ level, x, y };
        }
    } else {
        for (int y = y_min; y <= y_max && count < max_count; y++)
            for (int x = x_min; x <= x_max && count < max_count; x++)
                out[count++] = (arpt_tile_key){ level, x, y };
    }

    return count;
}
