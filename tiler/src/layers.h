/* Layer name definitions shared between tile builder and CLI. */

#ifndef ARPT_LAYERS_H
#define ARPT_LAYERS_H

#define ARPT_MAX_LAYERS 16

/* Layer names indexed by layer number, matching style.json. */
static const char *const arpt_layer_names[ARPT_MAX_LAYERS] = {
    "terrain",     /* 0 - auto-generated terrain mesh */
    "land_cover",  /* 1 */
    "bathymetry",  /* 2 */
    "water",       /* 3 */
    "land",    "layer5",  "layer6",  "layer7",
    "layer8",  "layer9",  "layer10", "layer11", "layer12",
    "layer13", "layer14", "layer15"
};

#endif
