#ifndef ARPENTRY_CAMERA_H
#define ARPENTRY_CAMERA_H

#include "math3d.h"
#include <stdbool.h>

typedef struct arpt_camera arpt_camera;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

arpt_camera *arpt_camera_create(void);
void arpt_camera_free(arpt_camera *cam);

/* ── Setters ───────────────────────────────────────────────────────────── */

void arpt_camera_set_position(arpt_camera *cam, double lon_rad, double lat_rad,
                               double altitude);
void arpt_camera_set_tilt(arpt_camera *cam, double tilt_rad);
void arpt_camera_set_bearing(arpt_camera *cam, double bearing_rad);
void arpt_camera_set_viewport(arpt_camera *cam, int width, int height);

/* ── Getters ───────────────────────────────────────────────────────────── */

double arpt_camera_lon(const arpt_camera *cam);
double arpt_camera_lat(const arpt_camera *cam);
double arpt_camera_altitude(const arpt_camera *cam);
double arpt_camera_tilt(const arpt_camera *cam);
double arpt_camera_bearing(const arpt_camera *cam);
int    arpt_camera_vp_width(const arpt_camera *cam);
int    arpt_camera_vp_height(const arpt_camera *cam);

/* ── Computed matrices ─────────────────────────────────────────────────── */

/** Perspective projection matrix (float32 for GPU). */
arpt_mat4 arpt_camera_projection(const arpt_camera *cam);

/**
 * Per-tile model matrix: transforms tile-local ECEF to camera space.
 *
 * center_lon, center_lat in radians; center_alt in meters.
 * This computes: M_tile = mat4(R_tilt * R_globe, tile_pos_cam) per VIEWER.md.
 */
arpt_mat4 arpt_camera_tile_model(const arpt_camera *cam,
                                  double center_lon, double center_lat,
                                  double center_alt);

/* ── Tile management helpers ───────────────────────────────────────────── */

/**
 * Compute the uniform zoom level for the current view.
 * L = floor(log2(root_error * vp_height / (2 * altitude * tan(fov/2) * 8)))
 * Clamped to [min_level, max_level].
 */
int arpt_camera_zoom_level(const arpt_camera *cam, double root_error,
                            int min_level, int max_level);

/**
 * Cast a ray from screen coordinates (sx, sy) in pixels through the camera.
 * Outputs origin and direction in ECEF (after inverse globe + tilt rotation).
 * Returns false if the screen point is outside the viewport.
 */
bool arpt_camera_screen_to_ray(const arpt_camera *cam, double sx, double sy,
                                arpt_dvec3 *origin, arpt_dvec3 *dir);

#endif /* ARPENTRY_CAMERA_H */
