/* Layer name definitions shared between tile builder and CLI. */

#ifndef ARPT_LAYERS_H
#define ARPT_LAYERS_H

#define ARPT_MAX_LAYERS 16

/* Layer names indexed by layer number, matching style.json. */
static const char *const arpt_layer_names[ARPT_MAX_LAYERS] = {
    "terrain",   /* 0 */
    "surface",   /* 1 */
    "highway",   /* 2 */
    "building",  /* 3 */
    "tree",      /* 4 */
    "poi",       /* 5 */
    "layer6",  "layer7",  "layer8",  "layer9",
    "layer10", "layer11", "layer12", "layer13",
    "layer14", "layer15"
};

#endif
