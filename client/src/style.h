#ifndef ARPENTRY_STYLE_H
#define ARPENTRY_STYLE_H

#include "tile_decode.h"

#define ARPT_STYLE_CLASS_COUNT (ARPT_SURFACE_BUILDING + 1)

typedef struct {
    float colors[ARPT_STYLE_CLASS_COUNT][4];     /* RGBA per class */
    float stroke_widths[ARPT_STYLE_CLASS_COUNT]; /* half-width per class */
    float building[4];                            /* RGBA building material */
} arpt_style;

/** Fill style with hardcoded defaults (fallback if server unavailable). */
void arpt_style_defaults(arpt_style *s);

#endif /* ARPENTRY_STYLE_H */
