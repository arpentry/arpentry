#ifndef ARPENTRY_FONT_H
#define ARPENTRY_FONT_H

#include <stdint.h>

/* SDF font atlas parameters */
#define FONT_ATLAS_SIZE   512  /* atlas texture dimensions (512x512) */
#define FONT_FIRST_CHAR    32  /* space */
#define FONT_LAST_CHAR    126  /* tilde */
#define FONT_CHAR_COUNT   (FONT_LAST_CHAR - FONT_FIRST_CHAR + 1)

/* Per-glyph metrics for the shader (UV coordinates in atlas) */
typedef struct {
    float u0, v0; /* top-left UV in atlas */
    float u1, v1; /* bottom-right UV in atlas */
    float advance; /* horizontal advance in pixels at rendered size */
    float bearing_x; /* left side bearing in pixels */
    float bearing_y; /* top bearing in pixels (from baseline) */
    float width;  /* glyph bitmap width in pixels */
    float height; /* glyph bitmap height in pixels */
} font_glyph;

/* Generate SDF font atlas into caller-provided RGBA buffer.
 * Buffer must be FONT_ATLAS_SIZE * FONT_ATLAS_SIZE * 4 bytes.
 * Also fills glyph metrics array (FONT_CHAR_COUNT entries).
 * Returns the font pixel height used for rendering. */
float font_generate_atlas(uint8_t *rgba_out, font_glyph *glyphs_out);

#endif /* ARPENTRY_FONT_H */
