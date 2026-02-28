#include "terrain.h"
#include "noise.h"

#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

/* Continental shape: low-frequency smooth fBm */
#define CONTINENT_OCTAVES    6
#define CONTINENT_FREQ       1.8
#define CONTINENT_LACUNARITY 2.0
#define CONTINENT_PERSIST    0.5
#define CONTINENT_BIAS       0.45   /* ~55-60% ocean */

/* Mountain ridges: ridged noise at mountain-chain scale.
 * Freq 40 -> ~160 km features at the coarsest octave.
 * 5 octaves with high persistence (0.75) so the fine-scale
 * octaves (~10-4 km) carry real weight -> steep slopes. */
#define RIDGE_OCTAVES    5
#define RIDGE_FREQ       40.0
#define RIDGE_LACUNARITY 2.5
#define RIDGE_PERSIST    0.75
/* Spatial offset to decorrelate ridge pattern from continental shape */
#define RIDGE_OFFSET_X   53.7
#define RIDGE_OFFSET_Y   91.2
#define RIDGE_OFFSET_Z   37.4

/* Elevation scaling */
#define TERRAIN_LAND_HEIGHT 9500.0  /* max land peak (m) */
#define TERRAIN_OCEAN_DEPTH 11000.0 /* max ocean depth (m) */

/* Moisture parameters */
#define MOISTURE_FREQ 3.0
#define MOISTURE_OCTAVES 6
#define MOISTURE_OFFSET_X 17.3
#define MOISTURE_OFFSET_Y 31.7
#define MOISTURE_OFFSET_Z 5.9

/* Geodetic (lon, lat) in degrees -> unit-sphere (x, y, z). */
static void lonlat_to_sphere(double lon_deg, double lat_deg, double *sx,
                              double *sy, double *sz) {
    double lon_r = lon_deg * (PI / 180.0);
    double lat_r = lat_deg * (PI / 180.0);
    double cos_lat = cos(lat_r);
    *sx = cos_lat * cos(lon_r);
    *sy = cos_lat * sin(lon_r);
    *sz = sin(lat_r);
}

/* Ridged fBm: each octave is individually ridged before summing.
 * (1-|n|)^2 per octave gives full [0,1] range at every scale.
 * Returns [0, 1]. */
static double ridged_fbm3(double x, double y, double z, int octaves,
                          double lacunarity, double persistence) {
    double signal = 0.0;
    double freq = 1.0;
    double amp = 1.0;
    double amp_sum = 0.0;

    for (int i = 0; i < octaves; i++) {
        double n = arpt_simplex3(x * freq, y * freq, z * freq);
        n = 1.0 - fabs(n);
        n = n * n;
        signal += n * amp;
        amp_sum += amp;
        freq *= lacunarity;
        amp *= persistence;
    }

    return signal / amp_sum;
}

double terrain_elevation(double lon_deg, double lat_deg) {
    double sx, sy, sz;
    lonlat_to_sphere(lon_deg, lat_deg, &sx, &sy, &sz);

    /* Layer 1 -- continental shape: low-frequency smooth fBm.
     * Determines land vs ocean and the broad elevation envelope. */
    double cn = arpt_fbm3(sx * CONTINENT_FREQ, sy * CONTINENT_FREQ,
                          sz * CONTINENT_FREQ, CONTINENT_OCTAVES,
                          CONTINENT_LACUNARITY, CONTINENT_PERSIST);
    double ce = (cn + 1.0) * 0.5; /* [0, 1] */

    /* Ocean */
    if (ce < CONTINENT_BIAS) {
        double t = 1.0 - ce / CONTINENT_BIAS;
        return -t * t * TERRAIN_OCEAN_DEPTH;
    }

    /* Land envelope: normalise to [0, 1] */
    double t = (ce - CONTINENT_BIAS) / (1.0 - CONTINENT_BIAS);

    /* Layer 2 -- ridged fBm */
    double ridge = ridged_fbm3(sx * RIDGE_FREQ + RIDGE_OFFSET_X,
                               sy * RIDGE_FREQ + RIDGE_OFFSET_Y,
                               sz * RIDGE_FREQ + RIDGE_OFFSET_Z,
                               RIDGE_OCTAVES, RIDGE_LACUNARITY,
                               RIDGE_PERSIST);

    /* Combine: sqrt envelope rises quickly from the coast, giving
     * most of the land area full mountain-height ridge contrast. */
    double mt = sqrt(t);
    return mt * ridge * TERRAIN_LAND_HEIGHT;
}

/* Moisture noise: separate fBm pass decorrelated from elevation via offset. */
double terrain_moisture(double lon_deg, double lat_deg) {
    double sx, sy, sz;
    lonlat_to_sphere(lon_deg, lat_deg, &sx, &sy, &sz);

    double m = arpt_fbm3(sx * MOISTURE_FREQ + MOISTURE_OFFSET_X,
                         sy * MOISTURE_FREQ + MOISTURE_OFFSET_Y,
                         sz * MOISTURE_FREQ + MOISTURE_OFFSET_Z,
                         MOISTURE_OCTAVES, 2.0, 0.5);

    /* Map [-1, 1] -> [0, 1] */
    return (m + 1.0) * 0.5;
}

/* Octahedral encoding: unit normal -> int8x2 */
static void encode_octahedral(double nx, double ny, double nz, int8_t *ox,
                              int8_t *oy) {
    double ax = fabs(nx), ay = fabs(ny), az = fabs(nz);
    double sum = ax + ay + az;
    if (sum < 1e-15) {
        *ox = 0;
        *oy = 127;
        return;
    }

    double u = nx / sum;
    double v = ny / sum;

    /* Reflect lower hemisphere */
    if (nz < 0.0) {
        double old_u = u;
        u = (1.0 - fabs(v)) * (old_u >= 0.0 ? 1.0 : -1.0);
        v = (1.0 - fabs(old_u)) * (v >= 0.0 ? 1.0 : -1.0);
    }

    /* Quantize to int8 [-127, 127] */
    double cu = u * 127.0;
    double cv = v * 127.0;
    *ox = (int8_t)(cu >= 0.0 ? cu + 0.5 : cu - 0.5);
    *oy = (int8_t)(cv >= 0.0 ? cv + 0.5 : cv - 0.5);
}

double *build_elevation_grid(arpt_bounds bounds, double cell_lon,
                             double cell_lat) {
    int pad_w = TERRAIN_VERTS + 2; /* 67 */
    int pad_n = pad_w * pad_w;     /* 67x67 = 4489 */
    double *elev = malloc((size_t)pad_n * sizeof(double));
    if (!elev) return NULL;

    for (int row = -1; row <= TERRAIN_VERTS; row++) {
        double lat = bounds.south + (double)row * cell_lat;
        for (int col = -1; col <= TERRAIN_VERTS; col++) {
            double lon = bounds.west + (double)col * cell_lon;
            elev[(row + 1) * pad_w + (col + 1)] = terrain_elevation(lon, lat);
        }
    }
    return elev;
}

void build_vertices(arpt_bounds bounds, double cell_lon, double cell_lat,
                    double cell_w_m, double cell_h_m, const double *elev,
                    int pad_w, uint16_t *vx, uint16_t *vy, int32_t *vz,
                    int8_t *normals) {
    for (int row = 0; row < TERRAIN_VERTS; row++) {
        double lat = bounds.south + (double)row * cell_lat;
        for (int col = 0; col < TERRAIN_VERTS; col++) {
            int idx = row * TERRAIN_VERTS + col;
            double lon = bounds.west + (double)col * cell_lon;
            int pi = (row + 1) * pad_w + (col + 1);

            vx[idx] = arpt_quantize_lon(lon, bounds);
            vy[idx] = arpt_quantize_lat(lat, bounds);
            vz[idx] = arpt_meters_to_mm(elev[pi]);

            /* Centered finite differences using padded neighbors */
            double dz_dx = (elev[pi + 1] - elev[pi - 1]) / (2.0 * cell_w_m);
            double dz_dy =
                (elev[pi + pad_w] - elev[pi - pad_w]) / (2.0 * cell_h_m);

            /* ECEF normal from ENU basis vectors */
            double lon_r = lon * (PI / 180.0);
            double lat_r = lat * (PI / 180.0);
            double sin_lon = sin(lon_r), cos_lon = cos(lon_r);
            double sin_lat = sin(lat_r), cos_lat = cos(lat_r);

            double ex = -sin_lon, ey = cos_lon, ez = 0.0;
            double nx_e = -sin_lat * cos_lon, ny_e = -sin_lat * sin_lon,
                   nz_e = cos_lat;
            double ux = cos_lat * cos_lon, uy = cos_lat * sin_lon, uz = sin_lat;

            double nx = ux - dz_dx * ex - dz_dy * nx_e;
            double ny = uy - dz_dx * ey - dz_dy * ny_e;
            double nz = uz - dz_dx * ez - dz_dy * nz_e;
            double len = sqrt(nx * nx + ny * ny + nz * nz);
            nx /= len;
            ny /= len;
            nz /= len;

            encode_octahedral(nx, ny, nz, &normals[idx * 2],
                              &normals[idx * 2 + 1]);
        }
    }
}

void build_indices(uint32_t *indices) {
    int ii = 0;
    for (int row = 0; row < TERRAIN_GRID; row++) {
        for (int col = 0; col < TERRAIN_GRID; col++) {
            uint32_t tl = (uint32_t)(row * TERRAIN_VERTS + col);
            uint32_t tr = tl + 1;
            uint32_t bl = tl + TERRAIN_VERTS;
            uint32_t br = bl + 1;

            indices[ii++] = tl;
            indices[ii++] = bl;
            indices[ii++] = tr;

            indices[ii++] = tr;
            indices[ii++] = bl;
            indices[ii++] = br;
        }
    }
}
