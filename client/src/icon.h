#ifndef ARPENTRY_ICON_H
#define ARPENTRY_ICON_H

#include <stdint.h>

/* SDF icon atlas parameters */
#define ICON_ATLAS_SIZE   512  /* atlas texture dimensions (512x512) */

/* Per-icon metrics (UV coordinates in atlas) */
typedef struct {
    float u0, v0; /* top-left UV in atlas */
    float u1, v1; /* bottom-right UV in atlas */
    float width;  /* glyph bitmap width in pixels */
    float height; /* glyph bitmap height in pixels */
} icon_glyph;

/* Generate SDF icon atlas into caller-provided RGBA buffer.
 * Buffer must be ICON_ATLAS_SIZE * ICON_ATLAS_SIZE * 4 bytes.
 * Also fills glyph metrics array (icon_count entries).
 * Returns the icon pixel height used for rendering. */
float icon_generate_atlas(uint8_t *rgba_out, icon_glyph *glyphs_out,
                           int *icon_count_out);

/* Find icon index by name. Returns -1 if not found. */
int icon_find(const char *name);

/* Return the number of icons in the atlas. */
int icon_count(void);

#endif /* ARPENTRY_ICON_H */
