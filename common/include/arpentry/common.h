#ifndef ARPENTRY_COMMON_H
#define ARPENTRY_COMMON_H

#include <stdint.h>

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

#endif /* ARPENTRY_COMMON_H */
