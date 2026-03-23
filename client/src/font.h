#ifndef ARPENTRY_FONT_H
#define ARPENTRY_FONT_H

#include <stdint.h>

/* SDF font atlas parameters */
#define FONT_ATLAS_SIZE  1024  /* atlas texture dimensions (1024x1024) */
#define FONT_FIRST_CHAR    32  /* space (U+0020) */
#define FONT_LAST_CHAR   591  /* U+024F — end of Latin Extended-B */
#define FONT_CHAR_COUNT  (FONT_LAST_CHAR - FONT_FIRST_CHAR + 1)

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

/* Decode one UTF-8 codepoint from *p, advance *p past it.
 * Returns the codepoint, or 0xFFFD (replacement char) on invalid input. */
static inline uint32_t font_utf8_decode(const char **p) {
    const unsigned char *s = (const unsigned char *)*p;
    uint32_t cp;
    if (s[0] < 0x80) {
        cp = s[0];
        *p += 1;
    } else if ((s[0] & 0xE0) == 0xC0 && (s[1] & 0xC0) == 0x80) {
        cp = ((uint32_t)(s[0] & 0x1F) << 6) | (s[1] & 0x3F);
        *p += 2;
    } else if ((s[0] & 0xF0) == 0xE0 && (s[1] & 0xC0) == 0x80 &&
               (s[2] & 0xC0) == 0x80) {
        cp = ((uint32_t)(s[0] & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) |
             (s[2] & 0x3F);
        *p += 3;
    } else if ((s[0] & 0xF8) == 0xF0 && (s[1] & 0xC0) == 0x80 &&
               (s[2] & 0xC0) == 0x80 && (s[3] & 0xC0) == 0x80) {
        cp = ((uint32_t)(s[0] & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) |
             ((uint32_t)(s[2] & 0x3F) << 6) | (s[3] & 0x3F);
        *p += 4;
    } else {
        cp = 0xFFFD;
        *p += 1;
    }
    return cp;
}

/* Generate SDF font atlas into caller-provided RGBA buffer.
 * Buffer must be FONT_ATLAS_SIZE * FONT_ATLAS_SIZE * 4 bytes.
 * Also fills glyph metrics array (FONT_CHAR_COUNT entries).
 * Returns the font pixel height used for rendering. */
float font_generate_atlas(uint8_t *rgba_out, font_glyph *glyphs_out);

#endif /* ARPENTRY_FONT_H */
