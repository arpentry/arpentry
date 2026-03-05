#include "style.h"
#include <string.h>

void arpt_style_defaults(arpt_style *s) {
    memset(s, 0, sizeof(*s));

    /* Surface fills (uint8 → float, matching server defaults) */
    static const uint8_t defaults[][4] = {
        [ARPT_SURFACE_UNKNOWN]     = {107, 158,  71, 255},
        [ARPT_SURFACE_WATER]       = { 23,  56, 115, 255},
        [ARPT_SURFACE_DESERT]      = {199, 173, 120, 255},
        [ARPT_SURFACE_FOREST]      = { 26,  77,  20, 255},
        [ARPT_SURFACE_GRASSLAND]   = {107, 158,  71, 255},
        [ARPT_SURFACE_CROPLAND]    = {166, 184,  77, 255},
        [ARPT_SURFACE_SHRUB]       = {128, 133,  82, 255},
        [ARPT_SURFACE_ICE]         = {224, 237, 250, 255},
        [ARPT_SURFACE_PRIMARY]     = { 89,  84,  77, 255},
        [ARPT_SURFACE_RESIDENTIAL] = {115, 110, 102, 255},
        [ARPT_SURFACE_BUILDING]    = {209, 196, 179, 255},
    };

    for (int i = 0; i < ARPT_STYLE_CLASS_COUNT; i++) {
        s->colors[i][0] = defaults[i][0] / 255.0f;
        s->colors[i][1] = defaults[i][1] / 255.0f;
        s->colors[i][2] = defaults[i][2] / 255.0f;
        s->colors[i][3] = defaults[i][3] / 255.0f;
    }

    /* Highway half-widths in quantized units */
    s->stroke_widths[ARPT_SURFACE_PRIMARY]     = 140.0f;
    s->stroke_widths[ARPT_SURFACE_RESIDENTIAL] =  90.0f;

    /* Building extrusion material */
    s->building[0] = 189.0f / 255.0f;
    s->building[1] = 186.0f / 255.0f;
    s->building[2] = 182.0f / 255.0f;
    s->building[3] = 255.0f / 255.0f;

    /* tree_style_count = 0; populated dynamically from style fetch */
}

int arpt_style_tree_index(const arpt_style *s, const char *class_name) {
    if (!s || !class_name) return -1;
    for (int i = 0; i < s->tree_style_count; i++) {
        if (strcmp(s->trees[i].class_name, class_name) == 0) return i;
    }
    return -1;
}
