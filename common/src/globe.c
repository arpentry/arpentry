#include "globe.h"
#include <math.h>

arpt_dvec3 arpt_geodetic_to_ecef(double lon, double lat, double alt) {
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double sin_lon = sin(lon), cos_lon = cos(lon);
    double N = ARPT_WGS84_A / sqrt(1.0 - ARPT_WGS84_E2 * sin_lat * sin_lat);
    return (arpt_dvec3){
        (N + alt) * cos_lat * cos_lon,
        (N + alt) * cos_lat * sin_lon,
        (N * (1.0 - ARPT_WGS84_E2) + alt) * sin_lat,
    };
}

void arpt_ecef_to_geodetic(arpt_dvec3 ecef,
                            double *lon, double *lat, double *alt) {
    /* Bowring's iterative method (converges in 2-3 iterations) */
    double x = ecef.x, y = ecef.y, z = ecef.z;
    double p = sqrt(x * x + y * y);

    *lon = atan2(y, x);

    /* Initial estimate */
    double lat_i = atan2(z, p * (1.0 - ARPT_WGS84_E2));
    double N;
    for (int i = 0; i < 5; i++) {
        double sin_lat = sin(lat_i);
        N = ARPT_WGS84_A / sqrt(1.0 - ARPT_WGS84_E2 * sin_lat * sin_lat);
        lat_i = atan2(z + ARPT_WGS84_E2 * N * sin_lat, p);
    }
    *lat = lat_i;

    double sin_lat = sin(lat_i);
    N = ARPT_WGS84_A / sqrt(1.0 - ARPT_WGS84_E2 * sin_lat * sin_lat);
    double cos_lat = cos(lat_i);
    if (fabs(cos_lat) > 1e-10)
        *alt = p / cos_lat - N;
    else
        *alt = fabs(z) - ARPT_WGS84_A * sqrt(1.0 - ARPT_WGS84_E2);
}

arpt_dvec3 arpt_surface_normal(double lon, double lat) {
    /* The geodetic surface normal is the unit vector perpendicular to the
       ellipsoid at the given geodetic coordinates. By definition of geodetic
       latitude, this is simply: */
    return (arpt_dvec3){
        cos(lat) * cos(lon),
        cos(lat) * sin(lon),
        sin(lat),
    };
}

bool arpt_ray_ellipsoid(arpt_dvec3 origin, arpt_dvec3 dir, double *t) {
    /* Ellipsoid: (x/a)^2 + (y/a)^2 + (z/b)^2 = 1
       Substitute ray: P = O + t*D and solve quadratic At^2 + Bt + C = 0 */
    double a2 = ARPT_WGS84_A * ARPT_WGS84_A;
    double b2 = ARPT_WGS84_B * ARPT_WGS84_B;

    double A = (dir.x * dir.x + dir.y * dir.y) / a2 +
               (dir.z * dir.z) / b2;
    double B = 2.0 * ((origin.x * dir.x + origin.y * dir.y) / a2 +
                       (origin.z * dir.z) / b2);
    double C = (origin.x * origin.x + origin.y * origin.y) / a2 +
               (origin.z * origin.z) / b2 - 1.0;

    double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) return false;

    double sqrt_disc = sqrt(disc);
    double t0 = (-B - sqrt_disc) / (2.0 * A);
    double t1 = (-B + sqrt_disc) / (2.0 * A);

    if (t0 > 0.0) { *t = t0; return true; }
    if (t1 > 0.0) { *t = t1; return true; }
    return false;
}

arpt_dmat4 arpt_globe_rotation(double lon, double lat) {
    /* Build a rotation that maps the ECEF position at (lon, lat) to face -Z.
       At the interest point, the local East-North-Up frame in ECEF is:
         east  = (-sin(lon), cos(lon), 0)
         north = (-sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat))
         up    = (cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat))

       We want the camera frame to have:
         camera X = east direction
         camera Y = north direction (up on screen)
         camera Z = -up direction (into the globe, since camera looks along -Z)

       So R_globe maps:  east → X,  north → Y,  -up → Z
       As a rotation matrix (rows are the target basis vectors):
         row 0 = east
         row 1 = north
         row 2 = -up
       Since our matrices are column-major, we store these as columns of
       the transpose, i.e., columns of R_globe^T... but actually we want
       columns of R_globe, so:
         col 0 = (east.x, north.x, -up.x)
         col 1 = (east.y, north.y, -up.y)
         col 2 = (east.z, north.z, -up.z)
    */
    double sin_lon = sin(lon), cos_lon = cos(lon);
    double sin_lat = sin(lat), cos_lat = cos(lat);

    arpt_dvec3 east  = {-sin_lon, cos_lon, 0.0};
    arpt_dvec3 north = {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat};
    arpt_dvec3 neg_up = {-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat};

    /* R_globe: columns are where ECEF X, Y, Z axes map to in camera space.
       Column j = (east[j], north[j], neg_up[j]) */
    arpt_dvec3 c0 = {east.x, north.x, neg_up.x};
    arpt_dvec3 c1 = {east.y, north.y, neg_up.y};
    arpt_dvec3 c2 = {east.z, north.z, neg_up.z};

    return arpt_dmat4_from_cols(c0, c1, c2, (arpt_dvec3){0, 0, 0});
}
