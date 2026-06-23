#ifndef ARPENTRY_STYLE_H
#define ARPENTRY_STYLE_H

#include "tile/decode.h"

/* Layer rendering type, matching LayerType enum in style.fbs. */
typedef enum {
    ARPT_LAYER_TERRAIN    = 0,
    ARPT_LAYER_TEXTURE    = 1,
    ARPT_LAYER_BUILDING   = 2,
    ARPT_LAYER_INSTANCE   = 3,
    ARPT_LAYER_LABEL      = 4,
    ARPT_LAYER_LINE       = 5,
    ARPT_LAYER_LINE_LABEL = 6,
} arpt_layer_type;

/* Per-layer entry parsed from the style. */
#define ARPT_MAX_STYLE_LAYERS 16

typedef struct {
    char source_layer[32];
    arpt_layer_type type;
    uint8_t min_level;
} arpt_layer_entry;

/* Per-tree-model style parameters, populated from style paint entries. */
#define ARPT_MAX_TREE_STYLES 8

typedef struct {
    char class_name[32];  /* class property value (e.g. "oak", "pine") */
    char model_name[32];  /* Model.name from ModelLibrary (e.g. "oak") */
    float min_scale;
    float max_scale;
    bool random_yaw;
    bool random_scale;
} arpt_tree_style;

typedef struct arpt_style {
    char class_names[ARPT_MAX_CLASSES][32];       /* runtime class registry */
    int class_count;                              /* number of registered classes */
    float colors[ARPT_MAX_CLASSES][4];            /* RGBA per class */
    float stroke_widths[ARPT_MAX_CLASSES];        /* half-width per class */
    float casing_colors[ARPT_MAX_CLASSES][4];     /* line casing RGBA per class */
    float casing_widths[ARPT_MAX_CLASSES];        /* extra casing half-width (0 = none) */
    uint8_t class_min_levels[ARPT_MAX_CLASSES];   /* min zoom level per class */
    arpt_tree_style trees[ARPT_MAX_TREE_STYLES];  /* per-model tree params */
    int tree_style_count;                          /* populated from style */
    arpt_layer_entry layers[ARPT_MAX_STYLE_LAYERS];
    int layer_count;

    /* Label style (Mapbox-like defaults) */
    float text_size;            /* default 14 */
    float text_color[4];        /* default dark gray */
    float text_halo_color[4];   /* default white */
    float text_halo_width;      /* default 2.0 */
    float icon_size;            /* default 20 */
    float icon_color[4];        /* default dark gray */
    float icon_halo_color[4];   /* default white */
    float icon_halo_width;      /* default 0.5 */

    /* Line-following label style (street names) */
    float line_text_size;            /* default 15 */
    float line_text_color[4];        /* default dark gray */
    float line_text_halo_color[4];   /* default white */
    float line_text_halo_width;      /* default 1.2 */
} arpt_style;

/** Find tree style index by class name. Returns -1 if not found. */
int arpt_style_tree_index(const arpt_style *s, const char *class_name);

/** Get or register a class name in the style registry.
 *  Returns the index (0 = "unknown"). Appends if not found. */
int arpt_style_class_index(arpt_style *s, const char *name);

/** Fill style with hardcoded defaults (fallback if server unavailable). */
void arpt_style_defaults(arpt_style *s);

#endif /* ARPENTRY_STYLE_H */
