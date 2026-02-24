#include "tile_manager.h"
#include "globe.h"
#include "coords.h"
#include "camera.h"
#include "math3d.h"
#include <math.h>

int arpt_enumerate_visible_tiles(const arpt_camera *cam, int level,
                                  arpt_tile_key *out, int max_count) {
    if (!cam || !out || max_count <= 0 || level < 0) return 0;

    int vp_w = arpt_camera_vp_width(cam);
    int vp_h = arpt_camera_vp_height(cam);
    if (vp_w <= 0 || vp_h <= 0) return 0;

    /* Sample screen points: corners + center + edge midpoints */
    #define SAMPLE_COUNT 9
    double pts[SAMPLE_COUNT][2] = {
        {0, 0}, {vp_w, 0}, {vp_w, vp_h}, {0, vp_h},
        {vp_w / 2.0, vp_h / 2.0},
        {vp_w / 2.0, 0}, {vp_w, vp_h / 2.0},
        {vp_w / 2.0, vp_h}, {0, vp_h / 2.0},
    };

    /* Cast rays and collect geodetic hit points */
    double lons[SAMPLE_COUNT], lats[SAMPLE_COUNT];
    int hit_count = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        arpt_dvec3 origin, dir;
        if (!arpt_camera_screen_to_ray(cam, pts[i][0], pts[i][1],
                                        &origin, &dir))
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

    /* Shift back if we were in [0, 360) space */
    if (crosses_antimeridian) {
        if (min_lon > 180.0) min_lon -= 360.0;
        if (max_lon > 180.0) max_lon -= 360.0;
    }

    /* Clamp latitude */
    if (min_lat < -90.0) min_lat = -90.0;
    if (max_lat > 90.0) max_lat = 90.0;

    /* Convert bbox to tile indices at the given level */
    int n_cols = 1 << (level + 1);
    int n_rows = 1 << level;
    double tile_w = 360.0 / n_cols;
    double tile_h = 180.0 / n_rows;

    int x_min = (int)floor((min_lon + 180.0) / tile_w);
    int x_max = (int)floor((max_lon + 180.0) / tile_w);
    int y_min = (int)floor((min_lat + 90.0) / tile_h);
    int y_max = (int)floor((max_lat + 90.0) / tile_h);

    /* Clamp y to valid range */
    if (y_min < 0) y_min = 0;
    if (y_max >= n_rows) y_max = n_rows - 1;

    /* Generate tile list */
    int count = 0;

    if (crosses_antimeridian && x_min > x_max) {
        /* Wrapping: two ranges [x_min, n_cols-1] and [0, x_max] */
        for (int y = y_min; y <= y_max && count < max_count; y++) {
            for (int x = x_min; x < n_cols && count < max_count; x++) {
                out[count++] = (arpt_tile_key){ level, x, y };
            }
            for (int x = 0; x <= x_max && count < max_count; x++) {
                out[count++] = (arpt_tile_key){ level, x, y };
            }
        }
    } else {
        /* Clamp x to valid range */
        if (x_min < 0) x_min = 0;
        if (x_max >= n_cols) x_max = n_cols - 1;

        for (int y = y_min; y <= y_max && count < max_count; y++) {
            for (int x = x_min; x <= x_max && count < max_count; x++) {
                out[count++] = (arpt_tile_key){ level, x, y };
            }
        }
    }

    return count;
}
