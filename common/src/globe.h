#ifndef ARPENTRY_GLOBE_H
#define ARPENTRY_GLOBE_H

#include "math3d.h"
#include <stdbool.h>

/* WGS84 constants */
#define ARPT_WGS84_A 6378137.0         /* semi-major axis (m) */
#define ARPT_WGS84_B 6356752.314245179 /* semi-minor axis (m) */
#define ARPT_WGS84_E2 0.00669437999014 /* first eccentricity squared */

/* Convert geodetic (lon, lat rad; alt m) to ECEF. */
arpt_dvec3 arpt_geodetic_to_ecef(double lon, double lat, double alt);

/* Convert ECEF back to geodetic (lon, lat rad; alt m). */
void arpt_ecef_to_geodetic(arpt_dvec3 ecef, double *lon, double *lat,
                           double *alt);

/* Outward surface normal at a geodetic position (unit vector in ECEF). */
arpt_dvec3 arpt_surface_normal(double lon, double lat);

/* Ray-WGS84 intersection. On hit, *t = nearest positive parameter. */
bool arpt_ray_ellipsoid(arpt_dvec3 origin, arpt_dvec3 dir, double *t);

/* Globe rotation: rotates ECEF so (lon, lat) faces -Z. lon, lat in radians. */
arpt_dmat4 arpt_globe_rotation(double lon, double lat);

#endif /* ARPENTRY_GLOBE_H */
