#ifndef ARPENTRY_NOISE_H
#define ARPENTRY_NOISE_H

/**
 * 2D simplex noise.
 *
 * Deterministic, continuous, roughly in [-1, 1].
 */
double arpt_simplex2(double x, double y);

/**
 * Fractal Brownian motion (fBm) using 2D simplex noise.
 *
 * Sums `octaves` layers, each with frequency *= lacunarity and
 * amplitude *= persistence.  Returns values roughly in [-1, 1].
 */
double arpt_fbm2(double x, double y, int octaves,
                 double lacunarity, double persistence);

/**
 * 3D simplex noise.
 *
 * Deterministic, continuous, roughly in [-1, 1].
 */
double arpt_simplex3(double x, double y, double z);

/**
 * Fractal Brownian motion (fBm) using 3D simplex noise.
 *
 * Sums `octaves` layers, each with frequency *= lacunarity and
 * amplitude *= persistence.  Returns values roughly in [-1, 1].
 */
double arpt_fbm3(double x, double y, double z, int octaves,
                 double lacunarity, double persistence);

#endif /* ARPENTRY_NOISE_H */
