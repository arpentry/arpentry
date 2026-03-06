#ifndef ARPENTRY_STYLE_H
#define ARPENTRY_STYLE_H

#include "tile_decode.h"

#define ARPT_STYLE_CLASS_COUNT (ARPT_SURFACE_BUILDING + 1)

/* Layer rendering type, matching LayerType enum in style.fbs. */
typedef enum {
    ARPT_LAYER_TERRAIN   = 0,
    ARPT_LAYER_TEXTURE   = 1,
    ARPT_LAYER_EXTRUSION = 2,
    ARPT_LAYER_INSTANCE  = 3,
    ARPT_LAYER_LABEL     = 4,
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
    float colors[ARPT_STYLE_CLASS_COUNT][4];     /* RGBA per class */
    float stroke_widths[ARPT_STYLE_CLASS_COUNT]; /* half-width per class */
    arpt_tree_style trees[ARPT_MAX_TREE_STYLES]; /* per-model tree params */
    int tree_style_count;                         /* populated from style */
    arpt_layer_entry layers[ARPT_MAX_STYLE_LAYERS];
    int layer_count;
} arpt_style;

/** Find tree style index by class name. Returns -1 if not found. */
int arpt_style_tree_index(const arpt_style *s, const char *class_name);

/** Fill style with hardcoded defaults (fallback if server unavailable). */
void arpt_style_defaults(arpt_style *s);

#endif /* ARPENTRY_STYLE_H */
