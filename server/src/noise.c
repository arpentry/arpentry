#include "noise.h"

#include <math.h>
#include <stdint.h>

/* Permutation table (fixed seed for reproducibility) */

static const uint8_t perm[512] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
    140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
    57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
    60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
    65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
    200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
    52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
    207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
    119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
    129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
    81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
    184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
    222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
    /* Repeat for wrap-around */
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
    140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
    57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
    60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
    65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
    200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
    52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
    207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
    119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
    129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
    81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
    184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
    222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
};

/* 2D gradient vectors */

static const double grad2[][2] = {
    { 1, 1}, {-1, 1}, { 1,-1}, {-1,-1},
    { 1, 0}, {-1, 0}, { 0, 1}, { 0,-1},
    { 1, 1}, {-1, 1}, { 1,-1}, {-1,-1},
};

static inline double dot2(const double g[2], double x, double y) {
    return g[0] * x + g[1] * y;
}

/* 2D simplex noise */

#define F2  0.3660254037844386   /* (sqrt(3) - 1) / 2 */
#define G2  0.21132486540518713  /* (3 - sqrt(3)) / 6 */

double arpt_simplex2(double x, double y) {
    /* Skew input space to determine simplex cell */
    double s = (x + y) * F2;
    int i = (int)floor(x + s);
    int j = (int)floor(y + s);

    /* Unskew back to (x,y) space */
    double t = (i + j) * G2;
    double X0 = i - t;
    double Y0 = j - t;
    double x0 = x - X0;
    double y0 = y - Y0;

    /* Determine which simplex we're in */
    int i1, j1;
    if (x0 > y0) { i1 = 1; j1 = 0; }
    else          { i1 = 0; j1 = 1; }

    double x1 = x0 - i1 + G2;
    double y1 = y0 - j1 + G2;
    double x2 = x0 - 1.0 + 2.0 * G2;
    double y2 = y0 - 1.0 + 2.0 * G2;

    /* Hash corner coordinates to gradient indices */
    int ii = i & 255;
    int jj = j & 255;
    int gi0 = perm[ii     + perm[jj    ]] % 12;
    int gi1 = perm[ii + i1 + perm[jj + j1]] % 12;
    int gi2 = perm[ii + 1  + perm[jj + 1 ]] % 12;

    /* Calculate contributions from the three corners */
    double n0, n1, n2;

    double t0 = 0.5 - x0*x0 - y0*y0;
    if (t0 < 0.0) n0 = 0.0;
    else { t0 *= t0; n0 = t0 * t0 * dot2(grad2[gi0], x0, y0); }

    double t1 = 0.5 - x1*x1 - y1*y1;
    if (t1 < 0.0) n1 = 0.0;
    else { t1 *= t1; n1 = t1 * t1 * dot2(grad2[gi1], x1, y1); }

    double t2 = 0.5 - x2*x2 - y2*y2;
    if (t2 < 0.0) n2 = 0.0;
    else { t2 *= t2; n2 = t2 * t2 * dot2(grad2[gi2], x2, y2); }

    /* Scale to [-1, 1] */
    return 70.0 * (n0 + n1 + n2);
}

/* Fractal Brownian motion (2D) */

double arpt_fbm2(double x, double y, int octaves,
                 double lacunarity, double persistence)
{
    double sum = 0.0;
    double amplitude = 1.0;
    double frequency = 1.0;
    double max_amp = 0.0;

    for (int i = 0; i < octaves; i++) {
        sum += amplitude * arpt_simplex2(x * frequency, y * frequency);
        max_amp += amplitude;
        frequency *= lacunarity;
        amplitude *= persistence;
    }

    return sum / max_amp;
}

/* 3D gradient vectors (12 cube-edge midpoints) */

static const double grad3[][3] = {
    { 1, 1, 0}, {-1, 1, 0}, { 1,-1, 0}, {-1,-1, 0},
    { 1, 0, 1}, {-1, 0, 1}, { 1, 0,-1}, {-1, 0,-1},
    { 0, 1, 1}, { 0,-1, 1}, { 0, 1,-1}, { 0,-1,-1},
};

static inline double dot3(const double g[3], double x, double y, double z) {
    return g[0] * x + g[1] * y + g[2] * z;
}

/* 3D simplex noise */

#define F3  (1.0 / 3.0)
#define G3  (1.0 / 6.0)

double arpt_simplex3(double x, double y, double z) {
    /* Skew input space to determine simplex cell */
    double s = (x + y + z) * F3;
    int i = (int)floor(x + s);
    int j = (int)floor(y + s);
    int k = (int)floor(z + s);

    /* Unskew back to (x,y,z) space */
    double t = (i + j + k) * G3;
    double X0 = i - t;
    double Y0 = j - t;
    double Z0 = k - t;
    double x0 = x - X0;
    double y0 = y - Y0;
    double z0 = z - Z0;

    /* Determine which simplex (tetrahedron) we are in */
    int i1, j1, k1;  /* offsets for second corner */
    int i2, j2, k2;  /* offsets for third corner */

    if (x0 >= y0) {
        if (y0 >= z0)      { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; }
        else if (x0 >= z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; }
        else               { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; }
    } else {
        if (y0 < z0)       { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; }
        else if (x0 < z0)  { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; }
        else               { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; }
    }

    /* Offsets for remaining corners in unskewed coords */
    double x1 = x0 - i1 + G3;
    double y1 = y0 - j1 + G3;
    double z1 = z0 - k1 + G3;
    double x2 = x0 - i2 + 2.0 * G3;
    double y2 = y0 - j2 + 2.0 * G3;
    double z2 = z0 - k2 + 2.0 * G3;
    double x3 = x0 - 1.0 + 3.0 * G3;
    double y3 = y0 - 1.0 + 3.0 * G3;
    double z3 = z0 - 1.0 + 3.0 * G3;

    /* Hash corner coordinates to gradient indices */
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;
    int gi0 = perm[ii      + perm[jj      + perm[kk     ]]] % 12;
    int gi1 = perm[ii + i1 + perm[jj + j1 + perm[kk + k1]]] % 12;
    int gi2 = perm[ii + i2 + perm[jj + j2 + perm[kk + k2]]] % 12;
    int gi3 = perm[ii + 1  + perm[jj + 1  + perm[kk + 1 ]]] % 12;

    /* Calculate contributions from the four corners */
    double n0, n1, n2, n3;

    double t0 = 0.6 - x0*x0 - y0*y0 - z0*z0;
    if (t0 < 0.0) n0 = 0.0;
    else { t0 *= t0; n0 = t0 * t0 * dot3(grad3[gi0], x0, y0, z0); }

    double t1 = 0.6 - x1*x1 - y1*y1 - z1*z1;
    if (t1 < 0.0) n1 = 0.0;
    else { t1 *= t1; n1 = t1 * t1 * dot3(grad3[gi1], x1, y1, z1); }

    double t2 = 0.6 - x2*x2 - y2*y2 - z2*z2;
    if (t2 < 0.0) n2 = 0.0;
    else { t2 *= t2; n2 = t2 * t2 * dot3(grad3[gi2], x2, y2, z2); }

    double t3 = 0.6 - x3*x3 - y3*y3 - z3*z3;
    if (t3 < 0.0) n3 = 0.0;
    else { t3 *= t3; n3 = t3 * t3 * dot3(grad3[gi3], x3, y3, z3); }

    /* Scale to [-1, 1] */
    return 32.0 * (n0 + n1 + n2 + n3);
}

/* Fractal Brownian motion (3D) */

double arpt_fbm3(double x, double y, double z, int octaves,
                 double lacunarity, double persistence)
{
    double sum = 0.0;
    double amplitude = 1.0;
    double frequency = 1.0;
    double max_amp = 0.0;

    for (int i = 0; i < octaves; i++) {
        sum += amplitude * arpt_simplex3(x * frequency, y * frequency,
                                         z * frequency);
        max_amp += amplitude;
        frequency *= lacunarity;
        amplitude *= persistence;
    }

    return sum / max_amp;
}
