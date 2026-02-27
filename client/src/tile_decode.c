#include "tile_decode.h"
#include "tile_reader.h"
#include <stdlib.h>
#include <string.h>

bool arpt_decode_terrain(const void *flatbuf, size_t size,
                          arpt_terrain_mesh *out) {
    if (!flatbuf || !out || size < 8) return false;

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
    if (!tile) return false;

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    if (!layers) return false;

    size_t n_layers = arpentry_tiles_Layer_vec_len(layers);

    /* Find the "terrain" layer by name */
    arpentry_tiles_Layer_table_t terrain_layer = NULL;
    for (size_t i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, i);
        flatbuffers_string_t name = arpentry_tiles_Layer_name(layer);
        if (name && strcmp(name, "terrain") == 0) {
            terrain_layer = layer;
            break;
        }
    }
    if (!terrain_layer) return false;

    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(terrain_layer);
    if (!features || arpentry_tiles_Feature_vec_len(features) == 0)
        return false;

    /* First feature */
    arpentry_tiles_Feature_table_t feat = arpentry_tiles_Feature_vec_at(features, 0);
    if (!feat) return false;

    /* Check geometry type is MeshGeometry */
    if (arpentry_tiles_Feature_geometry_type(feat) != arpentry_tiles_Geometry_MeshGeometry)
        return false;

    arpentry_tiles_MeshGeometry_table_t mesh =
        (arpentry_tiles_MeshGeometry_table_t)arpentry_tiles_Feature_geometry(feat);
    if (!mesh) return false;

    /* Extract arrays (zero-copy) */
    flatbuffers_uint16_vec_t xv = arpentry_tiles_MeshGeometry_x(mesh);
    flatbuffers_uint16_vec_t yv = arpentry_tiles_MeshGeometry_y(mesh);
    flatbuffers_int32_vec_t zv = arpentry_tiles_MeshGeometry_z(mesh);
    flatbuffers_uint32_vec_t iv = arpentry_tiles_MeshGeometry_indices(mesh);

    if (!xv || !yv || !zv || !iv) return false;

    size_t vcount = flatbuffers_uint16_vec_len(xv);
    if (flatbuffers_uint16_vec_len(yv) != vcount) return false;
    if (flatbuffers_int32_vec_len(zv) != vcount) return false;
    if (vcount == 0) return false;

    out->x = xv;
    out->y = yv;
    out->z = zv;
    out->vertex_count = vcount;
    out->indices = iv;
    out->index_count = flatbuffers_uint32_vec_len(iv);

    /* Normals are optional */
    flatbuffers_int8_vec_t nv = arpentry_tiles_MeshGeometry_normals(mesh);
    if (nv && flatbuffers_int8_vec_len(nv) == 2 * vcount)
        out->normals = nv;
    else
        out->normals = NULL;

    return true;
}

/* Landuse decoding */

static arpt_landuse_class classify_string(flatbuffers_string_t s) {
    if (!s) return ARPT_LANDUSE_UNKNOWN;
    if (strcmp(s, "grass") == 0)  return ARPT_LANDUSE_GRASS;
    if (strcmp(s, "forest") == 0) return ARPT_LANDUSE_FOREST;
    if (strcmp(s, "sand") == 0)   return ARPT_LANDUSE_SAND;
    return ARPT_LANDUSE_UNKNOWN;
}

bool arpt_decode_landuse(const void *flatbuf, size_t size,
                          arpt_landuse_data *out) {
    out->polygons = NULL;
    out->count = 0;

    if (!flatbuf || size < 8) return true; /* no data is OK */

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
    if (!tile) return true;

    /* Find "landuse" layer */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    if (!layers) return true;

    arpentry_tiles_Layer_table_t landuse_layer = NULL;
    size_t n_layers = arpentry_tiles_Layer_vec_len(layers);
    for (size_t i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, i);
        flatbuffers_string_t name = arpentry_tiles_Layer_name(layer);
        if (name && strcmp(name, "landuse") == 0) {
            landuse_layer = layer;
            break;
        }
    }
    if (!landuse_layer) return true;

    /* Resolve tile-scope property dictionary */
    flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
    arpentry_tiles_Value_vec_t values = arpentry_tiles_Tile_values(tile);

    /* Find index of "class" key */
    uint32_t class_key_idx = UINT32_MAX;
    if (keys) {
        size_t nkeys = flatbuffers_string_vec_len(keys);
        for (size_t i = 0; i < nkeys; i++) {
            flatbuffers_string_t k = flatbuffers_string_vec_at(keys, i);
            if (k && strcmp(k, "class") == 0) {
                class_key_idx = (uint32_t)i;
                break;
            }
        }
    }

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(landuse_layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    out->polygons = malloc(n_feat * sizeof(arpt_landuse_polygon));
    if (!out->polygons) return false;

    size_t count = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;

        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PolygonGeometry) continue;

        arpentry_tiles_PolygonGeometry_table_t poly =
            (arpentry_tiles_PolygonGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!poly) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_PolygonGeometry_x(poly);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_PolygonGeometry_y(poly);
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc || vc == 0) continue;

        /* Resolve class from properties */
        arpt_landuse_class cls = ARPT_LANDUSE_UNKNOWN;
        if (class_key_idx != UINT32_MAX && values) {
            arpentry_tiles_Property_vec_t props =
                arpentry_tiles_Feature_properties(feat);
            if (props) {
                size_t np = arpentry_tiles_Property_vec_len(props);
                for (size_t p = 0; p < np; p++) {
                    arpentry_tiles_Property_struct_t pr =
                        arpentry_tiles_Property_vec_at(props, p);
                    if (pr && pr->key == class_key_idx) {
                        size_t vi = pr->value;
                        if (vi < arpentry_tiles_Value_vec_len(values)) {
                            arpentry_tiles_Value_table_t val =
                                arpentry_tiles_Value_vec_at(values, vi);
                            cls = classify_string(
                                arpentry_tiles_Value_string_value(val));
                        }
                        break;
                    }
                }
            }
        }

        out->polygons[count].x = xv;
        out->polygons[count].y = yv;
        out->polygons[count].vertex_count = vc;
        out->polygons[count].cls = cls;
        count++;
    }

    out->count = count;
    return true;
}

void arpt_landuse_data_free(arpt_landuse_data *data) {
    if (data) {
        free(data->polygons);
        data->polygons = NULL;
        data->count = 0;
    }
}
