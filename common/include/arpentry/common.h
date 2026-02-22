#ifndef ARPENTRY_COMMON_H
#define ARPENTRY_COMMON_H

#include <stdint.h>
#include <math.h>

/* ── Constants ──────────────────────────────────────────────────────────── */

#define ARPT_EXTENT  32768
#define ARPT_BUFFER  16384

/* ── Low-level quantization (normalized tile space) ─────────────────────── */

/**
 * Dequantize a uint16 coordinate back to normalized tile space.
 *
 * Tile extent = 32768, buffer = 16384 per side.
 * Raw tile proper: [16384, 49151], buffer zones: [0, 16383] and [49152, 65535].
 * Formula: (qx - 16384) / 32768.0
 */
static inline double arpt_dequantize(uint16_t q) {
    return ((double)q - 16384.0) / 32768.0;
}

/**
 * Quantize a normalized coordinate to uint16.
 *
 * Inverse of arpt_dequantize: q = round(x * 32768 + 16384).
 * Clamps result to [0, 65535].
 */
static inline uint16_t arpt_quantize(double x) {
    double v = x * 32768.0 + 16384.0;
    if (v < 0.0) return 0;
    if (v > 65535.0) return 65535;
    return (uint16_t)(v + 0.5);
}

/* ── Tile bounds ────────────────────────────────────────────────────────── */

typedef struct {
    double west;
    double south;
    double east;
    double north;
} arpt_bounds_t;

/**
 * Compute the geodetic bounds (degrees) for a tile at the given level, x, y.
 *
 * Tiling scheme: 2 root tiles at level 0 (west and east hemispheres).
 *   x: 0 .. 2^(level+1) - 1
 *   y: 0 .. 2^level - 1  (south to north)
 */
static inline arpt_bounds_t arpt_tile_bounds(int level, int x, int y) {
    double lon_span = 360.0 / (1 << (level + 1));
    double lat_span = 180.0 / (1 << level);
    arpt_bounds_t b;
    b.west  = -180.0 + x * lon_span;
    b.south = -90.0  + y * lat_span;
    b.east  = b.west  + lon_span;
    b.north = b.south + lat_span;
    return b;
}

/* ── Geodetic quantization ──────────────────────────────────────────────── */

/**
 * Quantize a longitude to uint16 within the given tile bounds.
 */
static inline uint16_t arpt_quantize_lon(double lon, arpt_bounds_t bounds) {
    double normalized = (lon - bounds.west) / (bounds.east - bounds.west);
    return arpt_quantize(normalized);
}

/**
 * Quantize a latitude to uint16 within the given tile bounds.
 */
static inline uint16_t arpt_quantize_lat(double lat, arpt_bounds_t bounds) {
    double normalized = (lat - bounds.south) / (bounds.north - bounds.south);
    return arpt_quantize(normalized);
}

/**
 * Dequantize a uint16 x coordinate back to longitude (degrees).
 */
static inline double arpt_dequantize_lon(uint16_t qx, arpt_bounds_t bounds) {
    return bounds.west + arpt_dequantize(qx) * (bounds.east - bounds.west);
}

/**
 * Dequantize a uint16 y coordinate back to latitude (degrees).
 */
static inline double arpt_dequantize_lat(uint16_t qy, arpt_bounds_t bounds) {
    return bounds.south + arpt_dequantize(qy) * (bounds.north - bounds.south);
}

/* ── Elevation ──────────────────────────────────────────────────────────── */

/**
 * Convert meters to int32 millimeters (the z encoding used in .arpt).
 */
static inline int32_t arpt_meters_to_mm(double meters) {
    return (int32_t)(meters * 1000.0 + (meters >= 0.0 ? 0.5 : -0.5));
}

/**
 * Convert int32 millimeters back to meters.
 */
static inline double arpt_mm_to_meters(int32_t mm) {
    return (double)mm * 0.001;
}

/* ── Level of Detail ────────────────────────────────────────────────────── */

/**
 * Deterministic geometric error for a tile at the given level.
 * geometric_error(level) = root_error / 2^level
 */
static inline double arpt_geometric_error(double root_error, int level) {
    return root_error / (double)(1 << level);
}

/**
 * Screen-space error in pixels.
 * sse = (geometric_error * viewport_height) / (2 * distance * tan(fov/2))
 */
static inline double arpt_screen_space_error(double geometric_error,
                                              double viewport_height,
                                              double distance,
                                              double fov_radians) {
    return (geometric_error * viewport_height) /
           (2.0 * distance * tan(fov_radians * 0.5));
}

#endif /* ARPENTRY_COMMON_H */
