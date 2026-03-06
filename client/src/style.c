#include "style.h"
#include <string.h>

int arpt_style_class_index(arpt_style *s, const char *name) {
    if (!s || !name) return 0;
    for (int i = 0; i < s->class_count; i++) {
        if (strcmp(s->class_names[i], name) == 0) return i;
    }
    if (s->class_count >= ARPT_MAX_CLASSES) return 0;
    int idx = s->class_count;
    strncpy(s->class_names[idx], name, 31);
    s->class_names[idx][31] = '\0';
    s->class_count++;
    return idx;
}

void arpt_style_defaults(arpt_style *s) {
    memset(s, 0, sizeof(*s));

    /* Index 0 = unknown/background */
    s->class_count = 1;
    strncpy(s->class_names[0], "unknown", 31);

    /* Helper macro: register class with color */
    #define REG(name, r, g, b, a) do { \
        int _i = arpt_style_class_index(s, name); \
        s->colors[_i][0] = (r) / 255.0f; \
        s->colors[_i][1] = (g) / 255.0f; \
        s->colors[_i][2] = (b) / 255.0f; \
        s->colors[_i][3] = (a) / 255.0f; \
    } while (0)

    /* Background / unknown */
    s->colors[0][0] = 107 / 255.0f;
    s->colors[0][1] = 158 / 255.0f;
    s->colors[0][2] =  71 / 255.0f;
    s->colors[0][3] = 255 / 255.0f;

    /* Surface fills (matching server defaults) */
    REG("water",        23,  56, 115, 255);
    REG("desert",      199, 173, 120, 255);
    REG("forest",       26,  77,  20, 255);
    REG("grassland",   107, 158,  71, 255);
    REG("cropland",    166, 184,  77, 255);
    REG("shrub",       128, 133,  82, 255);
    REG("ice",         224, 237, 250, 255);
    REG("primary",      89,  84,  77, 255);
    REG("residential", 115, 110, 102, 255);
    REG("building",    209, 196, 179, 255);

    #undef REG

    /* Highway half-widths in quantized units */
    int pri = arpt_style_class_index(s, "primary");
    int res = arpt_style_class_index(s, "residential");
    s->stroke_widths[pri] = 140.0f;
    s->stroke_widths[res] =  90.0f;

    /* tree_style_count = 0; populated dynamically from style fetch */
}

int arpt_style_tree_index(const arpt_style *s, const char *class_name) {
    if (!s || !class_name) return -1;
    for (int i = 0; i < s->tree_style_count; i++) {
        if (strcmp(s->trees[i].class_name, class_name) == 0) return i;
    }
    return -1;
}
