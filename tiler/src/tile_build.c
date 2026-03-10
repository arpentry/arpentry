#include "tile_build.h"
#include "tile_builder.h"

#include <brotli/encode.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Coordinate quantization: map geo coords within tile bounds to uint16.
   Tile proper: [16384, 49151], extent = 32768, buffer = 16384 per side. */
#define TILE_EXTENT  32768
#define TILE_BUFFER  16384

/* String dictionary for key/value deduplication */
typedef struct {
    char  **entries;
    uint32_t count;
    uint32_t cap;
} str_dict;

static void str_dict_init(str_dict *d) {
    d->entries = NULL;
    d->count = 0;
    d->cap = 0;
}

static void str_dict_free(str_dict *d) {
    for (uint32_t i = 0; i < d->count; i++) free(d->entries[i]);
    free(d->entries);
}

/* Returns index; adds if not present */
static uint32_t str_dict_intern(str_dict *d, const char *s) {
    for (uint32_t i = 0; i < d->count; i++) {
        if (strcmp(d->entries[i], s) == 0) return i;
    }
    if (d->count == d->cap) {
        uint32_t nc = d->cap ? d->cap * 2 : 16;
        char **p = realloc(d->entries, nc * sizeof(char *));
        if (!p) return 0;
        d->entries = p;
        d->cap = nc;
    }
    d->entries[d->count] = strdup(s);
    return d->count++;
}

/* Stored feature (accumulated before building FlatBuffer) */
typedef struct {
    uint32_t layer;
    uint32_t geom_type;    /* WKB type 1-6 */
    uint16_t *qx, *qy;
    int32_t  *qz;
    uint32_t n_coords;
    uint32_t *offsets;
    uint32_t n_offsets;
    uint32_t *props;       /* [key_idx, val_idx] pairs */
    uint32_t n_props;
} stored_feat;

/* Layer name */
static const char *layer_names[] = {
    "default", "layer1", "layer2", "layer3",
    "layer4", "layer5", "layer6", "layer7",
    "layer8", "layer9", "layer10", "layer11",
    "layer12", "layer13", "layer14", "layer15"
};

struct arpt_tile_builder {
    arpt_bounds bounds;

    str_dict keys;
    str_dict vals;

    stored_feat *feats;
    uint32_t n_feats;
    uint32_t feat_cap;
};

static uint16_t quantize_x(const arpt_bounds *b, double x) {
    double t = (x - b->min_x) / (b->max_x - b->min_x);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

static uint16_t quantize_y(const arpt_bounds *b, double y) {
    double t = (y - b->min_y) / (b->max_y - b->min_y);
    double q = t * TILE_EXTENT + TILE_BUFFER;
    if (q < 0.0) q = 0.0;
    if (q > 65535.0) q = 65535.0;
    return (uint16_t)q;
}

arpt_tile_builder *arpt_tile_builder_create(arpt_bounds bounds) {
    arpt_tile_builder *b = calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->bounds = bounds;
    str_dict_init(&b->keys);
    str_dict_init(&b->vals);
    return b;
}

bool arpt_tile_builder_add_feature(arpt_tile_builder *b,
                                   const arpt_feature *feat) {
    if (!b || !feat || !feat->geom) return false;
    const arpt_geom *g = feat->geom;
    if (g->n_coords == 0) return false;

    /* Grow feature array */
    if (b->n_feats == b->feat_cap) {
        uint32_t nc = b->feat_cap ? b->feat_cap * 2 : 16;
        stored_feat *p = realloc(b->feats, nc * sizeof(stored_feat));
        if (!p) return false;
        b->feats = p;
        b->feat_cap = nc;
    }

    stored_feat *sf = &b->feats[b->n_feats];
    memset(sf, 0, sizeof(*sf));
    sf->layer = feat->layer;
    sf->geom_type = g->type;
    sf->n_coords = g->n_coords;

    /* Quantize coordinates */
    sf->qx = malloc(g->n_coords * sizeof(uint16_t));
    sf->qy = malloc(g->n_coords * sizeof(uint16_t));
    sf->qz = calloc(g->n_coords, sizeof(int32_t));
    if (!sf->qx || !sf->qy || !sf->qz) {
        free(sf->qx); free(sf->qy); free(sf->qz);
        return false;
    }

    for (uint32_t i = 0; i < g->n_coords; i++) {
        sf->qx[i] = quantize_x(&b->bounds, g->x[i]);
        sf->qy[i] = quantize_y(&b->bounds, g->y[i]);
        if (g->z) sf->qz[i] = (int32_t)(g->z[i] * 1000.0);
    }

    /* Copy offsets */
    if (g->offsets && g->n_offsets > 0) {
        sf->offsets = malloc(g->n_offsets * sizeof(uint32_t));
        if (!sf->offsets) {
            free(sf->qx); free(sf->qy); free(sf->qz);
            return false;
        }
        memcpy(sf->offsets, g->offsets, g->n_offsets * sizeof(uint32_t));
        sf->n_offsets = g->n_offsets;
    }

    /* Intern properties */
    if (feat->n_props > 0 && feat->prop_keys && feat->prop_vals) {
        sf->props = malloc(feat->n_props * 2 * sizeof(uint32_t));
        if (!sf->props) {
            free(sf->qx); free(sf->qy); free(sf->qz); free(sf->offsets);
            return false;
        }
        for (uint32_t i = 0; i < feat->n_props; i++) {
            sf->props[i * 2]     = str_dict_intern(&b->keys, feat->prop_keys[i]);
            sf->props[i * 2 + 1] = str_dict_intern(&b->vals, feat->prop_vals[i]);
        }
        sf->n_props = feat->n_props;
    }

    b->n_feats++;
    return true;
}

/* Build geometry for one feature into the flatcc builder */
static void build_geom(flatcc_builder_t *fb, const stored_feat *sf) {
    switch (sf->geom_type) {
    case 1: case 4: { /* Point / MultiPoint */
        arpentry_tiles_PointGeometry_start(fb);
        arpentry_tiles_PointGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PointGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_PointGeometry_z_create(fb, sf->qz, sf->n_coords);
        arpentry_tiles_PointGeometry_ref_t ref = arpentry_tiles_PointGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PointGeometry_add(fb, ref);
        break;
    }
    case 2: case 5: { /* LineString / MultiLineString */
        arpentry_tiles_LineGeometry_start(fb);
        arpentry_tiles_LineGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_LineGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_LineGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_LineGeometry_line_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_LineGeometry_ref_t ref = arpentry_tiles_LineGeometry_end(fb);
        arpentry_tiles_Feature_geometry_LineGeometry_add(fb, ref);
        break;
    }
    case 3: case 6: { /* Polygon / MultiPolygon */
        arpentry_tiles_PolygonGeometry_start(fb);
        arpentry_tiles_PolygonGeometry_x_create(fb, sf->qx, sf->n_coords);
        arpentry_tiles_PolygonGeometry_y_create(fb, sf->qy, sf->n_coords);
        arpentry_tiles_PolygonGeometry_z_create(fb, sf->qz, sf->n_coords);
        if (sf->offsets && sf->n_offsets > 0) {
            arpentry_tiles_PolygonGeometry_ring_offsets_create(fb, sf->offsets, sf->n_offsets);
        }
        arpentry_tiles_PolygonGeometry_ref_t ref = arpentry_tiles_PolygonGeometry_end(fb);
        arpentry_tiles_Feature_geometry_PolygonGeometry_add(fb, ref);
        break;
    }
    default:
        break;
    }
}

void *arpt_tile_builder_finish(arpt_tile_builder *b, size_t *out_size) {
    if (!b) { if (out_size) *out_size = 0; return NULL; }

    flatcc_builder_t fb;
    flatcc_builder_init(&fb);

    arpentry_tiles_Tile_start_as_root(&fb);
    arpentry_tiles_Tile_version_add(&fb, 1);

    /* Keys dictionary */
    if (b->keys.count > 0) {
        arpentry_tiles_Tile_keys_start(&fb);
        for (uint32_t i = 0; i < b->keys.count; i++) {
            arpentry_tiles_Tile_keys_push_create_str(&fb, b->keys.entries[i]);
        }
        arpentry_tiles_Tile_keys_end(&fb);
    }

    /* Values dictionary (all stored as strings for now) */
    if (b->vals.count > 0) {
        arpentry_tiles_Tile_values_start(&fb);
        for (uint32_t i = 0; i < b->vals.count; i++) {
            arpentry_tiles_Tile_values_push_start(&fb);
            arpentry_tiles_Value_type_add(&fb, arpentry_tiles_PropertyValueType_String);
            arpentry_tiles_Value_string_value_create_str(&fb, b->vals.entries[i]);
            arpentry_tiles_Tile_values_push_end(&fb);
        }
        arpentry_tiles_Tile_values_end(&fb);
    }

    /* Group features by layer */
    uint32_t max_layer = 0;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        if (b->feats[i].layer > max_layer) max_layer = b->feats[i].layer;
    }

    arpentry_tiles_Tile_layers_start(&fb);
    for (uint32_t layer = 0; layer <= max_layer; layer++) {
        /* Check if any features in this layer */
        bool has_feats = false;
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer == layer) { has_feats = true; break; }
        }
        if (!has_feats) continue;

        arpentry_tiles_Tile_layers_push_start(&fb);
        const char *name = layer < 16 ? layer_names[layer] : "default";
        arpentry_tiles_Layer_name_create_str(&fb, name);

        arpentry_tiles_Layer_features_start(&fb);
        for (uint32_t i = 0; i < b->n_feats; i++) {
            if (b->feats[i].layer != layer) continue;
            const stored_feat *sf = &b->feats[i];

            arpentry_tiles_Layer_features_push_start(&fb);
            arpentry_tiles_Feature_id_add(&fb, i);

            build_geom(&fb, sf);

            /* Properties */
            if (sf->n_props > 0) {
                arpentry_tiles_Property_t *props =
                    malloc(sf->n_props * sizeof(arpentry_tiles_Property_t));
                if (props) {
                    for (uint32_t j = 0; j < sf->n_props; j++) {
                        props[j].key = sf->props[j * 2];
                        props[j].value = sf->props[j * 2 + 1];
                    }
                    arpentry_tiles_Feature_properties_create(&fb, props, sf->n_props);
                    free(props);
                }
            }

            arpentry_tiles_Layer_features_push_end(&fb);
        }
        arpentry_tiles_Layer_features_end(&fb);
        arpentry_tiles_Tile_layers_push_end(&fb);
    }
    arpentry_tiles_Tile_layers_end(&fb);

    arpentry_tiles_Tile_end_as_root(&fb);

    /* Finalize FlatBuffer */
    size_t fb_size;
    void *fb_buf = flatcc_builder_finalize_buffer(&fb, &fb_size);
    flatcc_builder_clear(&fb);

    if (!fb_buf) {
        if (out_size) *out_size = 0;
        return NULL;
    }

    /* Brotli compress */
    size_t max_compressed = BrotliEncoderMaxCompressedSize(fb_size);
    if (max_compressed == 0) max_compressed = fb_size + 64;

    uint8_t *compressed = malloc(max_compressed);
    if (!compressed) {
        free(fb_buf);
        if (out_size) *out_size = 0;
        return NULL;
    }

    size_t compressed_size = max_compressed;
    if (!BrotliEncoderCompress(4, BROTLI_DEFAULT_WINDOW, BROTLI_DEFAULT_MODE,
                               fb_size, (const uint8_t *)fb_buf,
                               &compressed_size, compressed)) {
        free(fb_buf);
        free(compressed);
        if (out_size) *out_size = 0;
        return NULL;
    }

    free(fb_buf);
    if (out_size) *out_size = compressed_size;
    return compressed;
}

void arpt_tile_builder_free(arpt_tile_builder *b) {
    if (!b) return;
    for (uint32_t i = 0; i < b->n_feats; i++) {
        free(b->feats[i].qx);
        free(b->feats[i].qy);
        free(b->feats[i].qz);
        free(b->feats[i].offsets);
        free(b->feats[i].props);
    }
    free(b->feats);
    str_dict_free(&b->keys);
    str_dict_free(&b->vals);
    free(b);
}
