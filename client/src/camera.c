#include "camera.h"
#include "globe.h"
#include <math.h>
#include <stdlib.h>

#define CAM_FOV      (M_PI / 4.0)  /* 45 degrees */
#define CAM_MIN_ALT  100.0
#define CAM_MAX_ALT  30000000.0    /* 30,000 km */
#define CAM_MAX_TILT (60.0 * M_PI / 180.0)
#define MAX_LAT_RAD  (89.0 * M_PI / 180.0)
#define TWO_PI       (2.0 * M_PI)

struct arpt_camera {
    double lon_rad, lat_rad, altitude;
    double tilt_rad, bearing_rad;
    int vp_width, vp_height;
};

static void clamp_position(arpt_camera *cam) {
    if (cam->lat_rad > MAX_LAT_RAD) cam->lat_rad = MAX_LAT_RAD;
    if (cam->lat_rad < -MAX_LAT_RAD) cam->lat_rad = -MAX_LAT_RAD;
    while (cam->lon_rad > M_PI) cam->lon_rad -= TWO_PI;
    while (cam->lon_rad < -M_PI) cam->lon_rad += TWO_PI;
}

arpt_camera *arpt_camera_create(void) {
    arpt_camera *cam = calloc(1, sizeof(*cam));
    if (!cam) return NULL;
    cam->altitude = 10000000.0;  /* 10,000 km default */
    cam->vp_width = 800;
    cam->vp_height = 600;
    return cam;
}

void arpt_camera_free(arpt_camera *cam) { free(cam); }

/* ── Setters ───────────────────────────────────────────────────────────── */

void arpt_camera_set_position(arpt_camera *cam, double lon_rad, double lat_rad,
                               double altitude) {
    cam->lon_rad = lon_rad;
    cam->lat_rad = lat_rad;
    clamp_position(cam);
    cam->altitude = fmax(CAM_MIN_ALT, fmin(CAM_MAX_ALT, altitude));
}

void arpt_camera_set_tilt(arpt_camera *cam, double tilt_rad) {
    cam->tilt_rad = fmax(0.0, fmin(CAM_MAX_TILT, tilt_rad));
}

void arpt_camera_set_bearing(arpt_camera *cam, double bearing_rad) {
    cam->bearing_rad = fmod(bearing_rad, TWO_PI);
    if (cam->bearing_rad < 0.0) cam->bearing_rad += TWO_PI;
}

void arpt_camera_set_viewport(arpt_camera *cam, int width, int height) {
    cam->vp_width = width > 1 ? width : 1;
    cam->vp_height = height > 1 ? height : 1;
}

/* ── Getters ───────────────────────────────────────────────────────────── */

double arpt_camera_lon(const arpt_camera *cam) { return cam->lon_rad; }
double arpt_camera_lat(const arpt_camera *cam) { return cam->lat_rad; }
double arpt_camera_altitude(const arpt_camera *cam) { return cam->altitude; }
double arpt_camera_tilt(const arpt_camera *cam) { return cam->tilt_rad; }
double arpt_camera_bearing(const arpt_camera *cam) { return cam->bearing_rad; }
int arpt_camera_vp_width(const arpt_camera *cam) { return cam->vp_width; }
int arpt_camera_vp_height(const arpt_camera *cam) { return cam->vp_height; }

/* ── Internal helpers ──────────────────────────────────────────────────── */

/* R_tilt: applies bearing rotation (around -Z) then tilt (around X). */
static arpt_dmat4 compute_tilt_matrix(double tilt, double bearing) {
    /* Bearing: rotate around Z by -bearing (clockwise from north = +Y) */
    double cb = cos(-bearing), sb = sin(-bearing);
    arpt_dmat4 Rb = arpt_dmat4_from_cols(
        (arpt_dvec3){cb, sb, 0}, (arpt_dvec3){-sb, cb, 0},
        (arpt_dvec3){0, 0, 1}, (arpt_dvec3){0, 0, 0});

    /* Tilt: rotate around X by -tilt (tilt 0 = nadir = looking along -Z) */
    double ct = cos(-tilt), st = sin(-tilt);
    arpt_dmat4 Rt = arpt_dmat4_from_cols(
        (arpt_dvec3){1, 0, 0}, (arpt_dvec3){0, ct, st},
        (arpt_dvec3){0, -st, ct}, (arpt_dvec3){0, 0, 0});

    return arpt_dmat4_mul(Rt, Rb);
}

/* ── Input actions ─────────────────────────────────────────────────────── */

void arpt_camera_pan(arpt_camera *cam, double dx, double dy) {
    /* Sensitivity: at the current altitude, how many radians per pixel?
       Using the vertical FOV and viewport height. */
    double rad_per_px = 2.0 * cam->altitude * tan(CAM_FOV * 0.5) /
                        (cam->vp_height * ARPT_WGS84_A);

    /* Rotate pan direction by bearing */
    double cb = cos(cam->bearing_rad), sb = sin(cam->bearing_rad);
    double pdx = dx * cb - dy * sb;
    double pdy = dx * sb + dy * cb;

    cam->lon_rad -= pdx * rad_per_px / cos(cam->lat_rad);
    cam->lat_rad += pdy * rad_per_px;
    clamp_position(cam);
}

void arpt_camera_zoom(arpt_camera *cam, double factor) {
    cam->altitude *= factor;
    cam->altitude = fmax(CAM_MIN_ALT, fmin(CAM_MAX_ALT, cam->altitude));
}

void arpt_camera_tilt_bearing(arpt_camera *cam, double dtilt, double dbearing) {
    arpt_camera_set_tilt(cam, cam->tilt_rad + dtilt);
    arpt_camera_set_bearing(cam, cam->bearing_rad + dbearing);
}

/* ── Computed matrices ─────────────────────────────────────────────────── */

arpt_mat4 arpt_camera_projection(const arpt_camera *cam) {
    float aspect = (float)cam->vp_width / (float)cam->vp_height;
    float near = (float)fmax(1.0, cam->altitude * 0.01);
    float far = (float)(cam->altitude * 10.0);
    return arpt_mat4_perspective((float)CAM_FOV, aspect, near, far);
}

arpt_mat4 arpt_camera_tile_model(const arpt_camera *cam,
                                  double center_lon, double center_lat,
                                  double center_alt) {
    arpt_dmat4 R_globe = arpt_globe_rotation(cam->lon_rad, cam->lat_rad);
    arpt_dmat4 R_tilt = compute_tilt_matrix(cam->tilt_rad, cam->bearing_rad);

    arpt_dvec3 tile_center_ecef = arpt_geodetic_to_ecef(center_lon, center_lat,
                                                         center_alt);
    arpt_dvec3 interest_ecef = arpt_geodetic_to_ecef(cam->lon_rad, cam->lat_rad,
                                                      0.0);
    arpt_dvec3 delta = arpt_dvec3_sub(tile_center_ecef, interest_ecef);

    arpt_dmat4 R = arpt_dmat4_mul(R_tilt, R_globe);
    arpt_dvec3 tile_pos_cam = arpt_dmat4_rotate(R, delta);
    tile_pos_cam.z -= cam->altitude;

    /* Build final 4×4: upper-3×3 from R, translation from tile_pos_cam */
    arpt_dmat4 M = R;
    M.m[12] = tile_pos_cam.x;
    M.m[13] = tile_pos_cam.y;
    M.m[14] = tile_pos_cam.z;
    M.m[15] = 1.0;

    return arpt_dmat4_to_mat4(M);
}

/* ── Tile management helpers ───────────────────────────────────────────── */

int arpt_camera_zoom_level(const arpt_camera *cam, double root_error,
                            int min_level, int max_level) {
    /* L = floor(log2(root_error * vp_height / (2 * altitude * tan(fov/2) * 8))) */
    double val = root_error * cam->vp_height /
                 (2.0 * cam->altitude * tan(CAM_FOV * 0.5) * 8.0);
    if (val <= 1.0) return min_level;
    int L = (int)floor(log2(val));
    if (L < min_level) return min_level;
    if (L > max_level) return max_level;
    return L;
}

bool arpt_camera_screen_to_ray(const arpt_camera *cam, double sx, double sy,
                                arpt_dvec3 *origin, arpt_dvec3 *dir) {
    /* Convert screen (pixels) to NDC [-1, 1] */
    double ndc_x = (2.0 * sx / cam->vp_width - 1.0);
    double ndc_y = (1.0 - 2.0 * sy / cam->vp_height);

    /* Camera space ray direction (perspective, camera at origin looking -Z) */
    double aspect = (double)cam->vp_width / (double)cam->vp_height;
    double half_h = tan(CAM_FOV * 0.5);
    arpt_dvec3 cam_dir = arpt_dvec3_normalize((arpt_dvec3){
        ndc_x * half_h * aspect,
        ndc_y * half_h,
        -1.0,
    });

    /* Transform ray from camera space to ECEF:
       camera_pos is at interest_ecef + R * (0, 0, altitude) in ECEF,
       but since camera is at origin in camera space, we need the inverse
       of the globe+tilt rotation. */
    arpt_dmat4 R_globe = arpt_globe_rotation(cam->lon_rad, cam->lat_rad);
    arpt_dmat4 R_tilt = compute_tilt_matrix(cam->tilt_rad, cam->bearing_rad);
    arpt_dmat4 R = arpt_dmat4_mul(R_tilt, R_globe);

    /* Inverse of rotation = transpose */
    arpt_dmat4 R_inv;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R_inv.m[c * 4 + r] = R.m[r * 4 + c];
    R_inv.m[3] = R_inv.m[7] = R_inv.m[11] = 0.0;
    R_inv.m[12] = R_inv.m[13] = R_inv.m[14] = 0.0;
    R_inv.m[15] = 1.0;

    /* Camera position in ECEF:
       In camera space, the interest point is at (0, 0, -altitude).
       Camera is at origin. So the camera in "rotated ECEF" space is at
       the position where interest_ecef would be plus the offset.
       cam_ecef = interest_ecef + R_inv * (0, 0, altitude) */
    arpt_dvec3 interest_ecef = arpt_geodetic_to_ecef(cam->lon_rad, cam->lat_rad,
                                                      0.0);
    arpt_dvec3 cam_offset = arpt_dmat4_rotate(R_inv, (arpt_dvec3){0, 0, cam->altitude});
    *origin = arpt_dvec3_add(interest_ecef, cam_offset);
    *dir = arpt_dmat4_rotate(R_inv, cam_dir);

    return true;
}
