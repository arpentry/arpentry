#ifndef ARPENTRY_GLOBE_H
#define ARPENTRY_GLOBE_H

#include "math3d.h"
#include <stdbool.h>

/* WGS84 constants */
#define ARPT_WGS84_A  6378137.0             /* semi-major axis (m) */
#define ARPT_WGS84_B  6356752.314245179     /* semi-minor axis (m) */
#define ARPT_WGS84_E2 0.00669437999014      /* first eccentricity squared */

/**
 * Convert geodetic coordinates to ECEF (Earth-Centered, Earth-Fixed).
 * lon, lat in radians; alt in meters above ellipsoid.
 */
arpt_dvec3 arpt_geodetic_to_ecef(double lon, double lat, double alt);

/**
 * Convert ECEF back to geodetic coordinates.
 * Outputs: lon, lat in radians; alt in meters above ellipsoid.
 */
void arpt_ecef_to_geodetic(arpt_dvec3 ecef,
                            double *lon, double *lat, double *alt);

/**
 * Outward surface normal at a geodetic position (unit vector in ECEF).
 * lon, lat in radians.
 */
arpt_dvec3 arpt_surface_normal(double lon, double lat);

/**
 * Ray-ellipsoid intersection.
 * Returns true if the ray hits the WGS84 ellipsoid. On hit, *t is the
 * nearest positive parameter (origin + t*dir). Returns false on miss or
 * if the ray starts inside the ellipsoid with no forward intersection.
 */
bool arpt_ray_ellipsoid(arpt_dvec3 origin, arpt_dvec3 dir, double *t);

/**
 * Globe rotation matrix that places the interest point (lon, lat) facing
 * the camera (closest to origin along -Z).
 *
 * R_globe rotates ECEF so that the surface point at (lon, lat) maps to
 * (0, 0, -N) where N is the prime vertical radius.
 *
 * lon, lat in radians.
 */
arpt_dmat4 arpt_globe_rotation(double lon, double lat);

#endif /* ARPENTRY_GLOBE_H */
