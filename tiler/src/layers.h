/* Layer name definitions shared between tile builder and CLI. */

#ifndef ARPT_LAYERS_H
#define ARPT_LAYERS_H

/* Sort-key encodes layer in 4 bits, so the absolute ceiling is 16. */
#define ARPT_MAX_LAYERS 16

/* Indices 0..ARPT_NAMED_LAYERS-1 have stable, style-visible names.
 * Indices [ARPT_NAMED_LAYERS, ARPT_MAX_LAYERS) are reserved and fall
 * back to "layerN" in the emitted tile. */
#define ARPT_NAMED_LAYERS 7

/* Layer 0 is terrain, which is synthesized by the tiler itself — it is
 * never a user-supplied input. The CLI rejects --input 0:... */
#define ARPT_LAYER_TERRAIN 0

/* Layer names indexed by layer number, matching style.json. */
static const char *const arpt_layer_names[ARPT_MAX_LAYERS] = {
    "terrain",        /* 0 - auto-generated terrain mesh */
    "land_cover",     /* 1 */
    "bathymetry",     /* 2 */
    "water",          /* 3 */
    "land",           /* 4 */
    "transportation", /* 5 */
    "land_use",       /* 6 */
    "layer7",
    "layer8",  "layer9",  "layer10", "layer11", "layer12",
    "layer13", "layer14", "layer15"
};

#endif
