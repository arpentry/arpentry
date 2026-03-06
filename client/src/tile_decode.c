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
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        flatbuffers_string_t name = arpentry_tiles_Layer_name(layer);
        if (name && strcmp(name, "terrain") == 0) {
            terrain_layer = layer;
            break;
        }
    }
    if (!terrain_layer) return false;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(terrain_layer);
    if (!features || arpentry_tiles_Feature_vec_len(features) == 0)
        return false;

    /* First feature */
    arpentry_tiles_Feature_table_t feat =
        arpentry_tiles_Feature_vec_at(features, 0);
    if (!feat) return false;

    /* Check geometry type is MeshGeometry */
    if (arpentry_tiles_Feature_geometry_type(feat) !=
        arpentry_tiles_Geometry_MeshGeometry)
        return false;

    arpentry_tiles_MeshGeometry_table_t mesh =
        (arpentry_tiles_MeshGeometry_table_t)arpentry_tiles_Feature_geometry(
            feat);
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

/* Surface decoding */

/* Resolve the "class" property of a feature via the tile-scope dictionary,
   returning the index into the caller-provided class name list (0 = unknown). */
static uint8_t resolve_class(arpentry_tiles_Feature_table_t feat,
                              uint32_t class_key_idx,
                              arpentry_tiles_Value_vec_t values,
                              const char (*class_names)[32],
                              int class_count) {
    if (class_key_idx == UINT32_MAX || !values) return 0;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return 0;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == class_key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                flatbuffers_string_t s =
                    arpentry_tiles_Value_string_value(val);
                if (s) {
                    for (int ci = 0; ci < class_count; ci++) {
                        if (strcmp(s, class_names[ci]) == 0)
                            return (uint8_t)ci;
                    }
                }
            }
            break;
        }
    }
    return 0;
}

/* Resolve an integer property of a feature via the tile-scope dictionary. */
static int32_t resolve_int_property(arpentry_tiles_Feature_table_t feat,
                                    uint32_t key_idx,
                                    arpentry_tiles_Value_vec_t values) {
    if (key_idx == UINT32_MAX || !values) return 0;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return 0;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                return (int32_t)arpentry_tiles_Value_int_value(val);
            }
            break;
        }
    }
    return 0;
}

/* Find a layer by name and resolve "class", "height", and optionally "name"
 * key indices. */
static arpentry_tiles_Layer_table_t
find_layer_ex(const void *flatbuf, size_t size, const char *name,
              uint32_t *class_key_idx, uint32_t *height_key_idx,
              uint32_t *name_key_idx,
              arpentry_tiles_Value_vec_t *values) {
    *class_key_idx = UINT32_MAX;
    *height_key_idx = UINT32_MAX;
    if (name_key_idx) *name_key_idx = UINT32_MAX;
    *values = NULL;

    if (!flatbuf || size < 8) return NULL;

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
    if (!tile) return NULL;

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    if (!layers) return NULL;

    arpentry_tiles_Layer_table_t found = NULL;
    size_t n_layers = arpentry_tiles_Layer_vec_len(layers);
    for (size_t i = 0; i < n_layers; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        flatbuffers_string_t lname = arpentry_tiles_Layer_name(layer);
        if (lname && strcmp(lname, name) == 0) {
            found = layer;
            break;
        }
    }
    if (!found) return NULL;

    /* Resolve property dictionary */
    *values = arpentry_tiles_Tile_values(tile);
    flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
    if (keys) {
        size_t nkeys = flatbuffers_string_vec_len(keys);
        for (size_t i = 0; i < nkeys; i++) {
            flatbuffers_string_t k = flatbuffers_string_vec_at(keys, i);
            if (k && strcmp(k, "class") == 0)
                *class_key_idx = (uint32_t)i;
            else if (k && strcmp(k, "height") == 0)
                *height_key_idx = (uint32_t)i;
            else if (k && name_key_idx && strcmp(k, "name") == 0)
                *name_key_idx = (uint32_t)i;
        }
    }

    return found;
}

/* Convenience wrapper that doesn't resolve "name" key. */
static arpentry_tiles_Layer_table_t
find_layer(const void *flatbuf, size_t size, const char *name,
           uint32_t *class_key_idx, uint32_t *height_key_idx,
           arpentry_tiles_Value_vec_t *values) {
    return find_layer_ex(flatbuf, size, name, class_key_idx, height_key_idx,
                         NULL, values);
}

/* Decode all PolygonGeometry features from a named layer. */
static bool decode_polygon_layer(const void *flatbuf, size_t size,
                                 const char *layer_name,
                                 uint32_t height_key_override,
                                 const char (*class_names)[32],
                                 int class_count,
                                 arpt_surface_data *out) {
    out->polygons = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t height_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer = find_layer(
        flatbuf, size, layer_name, &class_key_idx, &height_key_idx, &values);
    if (!layer) return true;

    /* Allow caller to suppress height resolution */
    if (height_key_override != UINT32_MAX)
        height_key_idx = height_key_override;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    out->polygons = malloc(n_feat * sizeof(arpt_surface_polygon));
    if (!out->polygons) return false;

    size_t count = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;

        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PolygonGeometry)
            continue;

        arpentry_tiles_PolygonGeometry_table_t poly =
            (arpentry_tiles_PolygonGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!poly) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_PolygonGeometry_x(poly);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_PolygonGeometry_y(poly);
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc || vc == 0) continue;

        out->polygons[count].x = xv;
        out->polygons[count].y = yv;
        out->polygons[count].z = arpentry_tiles_PolygonGeometry_z(poly);
        out->polygons[count].vertex_count = vc;
        out->polygons[count].cls = resolve_class(feat, class_key_idx, values,
                                                    class_names, class_count);
        out->polygons[count].height_m =
            resolve_int_property(feat, height_key_idx, values);
        count++;
    }

    out->count = count;
    return true;
}

bool arpt_decode_surface(const void *flatbuf, size_t size,
                         const char (*class_names)[32], int class_count,
                         arpt_surface_data *out) {
    return decode_polygon_layer(flatbuf, size, "surface", UINT32_MAX,
                                class_names, class_count, out);
}

bool arpt_decode_buildings(const void *flatbuf, size_t size,
                           const char (*class_names)[32], int class_count,
                           arpt_surface_data *out) {
    return decode_polygon_layer(flatbuf, size, "building", UINT32_MAX,
                                class_names, class_count, out);
}

void arpt_surface_data_free(arpt_surface_data *data) {
    if (data) {
        free(data->polygons);
        data->polygons = NULL;
        data->count = 0;
    }
}

/* Highway decoding */

bool arpt_decode_highways(const void *flatbuf, size_t size,
                          const char (*class_names)[32], int class_count,
                          arpt_highway_data *out) {
    out->lines = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t height_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer(flatbuf, size, "highway", &class_key_idx, &height_key_idx,
                   &values);
    if (!layer) return true;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    out->lines = malloc(n_feat * sizeof(arpt_highway_line));
    if (!out->lines) return false;

    size_t count = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;

        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_LineGeometry)
            continue;

        arpentry_tiles_LineGeometry_table_t line =
            (arpentry_tiles_LineGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!line) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_LineGeometry_x(line);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_LineGeometry_y(line);
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc || vc < 2) continue;

        out->lines[count].x = xv;
        out->lines[count].y = yv;
        out->lines[count].vertex_count = vc;
        out->lines[count].cls = resolve_class(feat, class_key_idx, values,
                                                  class_names, class_count);
        count++;
    }

    out->count = count;
    return true;
}

void arpt_highway_data_free(arpt_highway_data *data) {
    if (data) {
        free(data->lines);
        data->lines = NULL;
        data->count = 0;
    }
}

/* Tree decoding */

/* Map tree class name to model index using the caller-provided class list. */
static uint8_t tree_model_from_class(arpentry_tiles_Feature_table_t feat,
                                     uint32_t class_key_idx,
                                     arpentry_tiles_Value_vec_t values,
                                     const char *const *class_names,
                                     int class_count) {
    if (class_key_idx == UINT32_MAX || !values) return 0;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return 0;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == class_key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                flatbuffers_string_t s =
                    arpentry_tiles_Value_string_value(val);
                if (s) {
                    for (int ci = 0; ci < class_count; ci++) {
                        if (strcmp(s, class_names[ci]) == 0)
                            return (uint8_t)ci;
                    }
                }
            }
            break;
        }
    }
    return 0;
}

bool arpt_decode_trees(const void *flatbuf, size_t size,
                       const char *const *class_names, int class_count,
                       arpt_tree_data *out) {
    out->points = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t height_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer(flatbuf, size, "tree", &class_key_idx, &height_key_idx,
                   &values);
    if (!layer) return true;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    /* Count total points across all features */
    size_t total = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;
        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PointGeometry)
            continue;
        arpentry_tiles_PointGeometry_table_t pt =
            (arpentry_tiles_PointGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!pt) continue;
        flatbuffers_uint16_vec_t xv = arpentry_tiles_PointGeometry_x(pt);
        if (xv) total += flatbuffers_uint16_vec_len(xv);
    }
    if (total == 0) return true;

    out->points = malloc(total * sizeof(arpt_tree_point));
    if (!out->points) return false;

    size_t count = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;
        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PointGeometry)
            continue;
        arpentry_tiles_PointGeometry_table_t pt =
            (arpentry_tiles_PointGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!pt) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_PointGeometry_x(pt);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_PointGeometry_y(pt);
        flatbuffers_int32_vec_t zv = arpentry_tiles_PointGeometry_z(pt);
        if (!xv || !yv || !zv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc) continue;
        if (flatbuffers_int32_vec_len(zv) != vc) continue;

        uint8_t mi = tree_model_from_class(feat, class_key_idx, values,
                                                 class_names, class_count);
        uint64_t fid = arpentry_tiles_Feature_id(feat);
        uint32_t id32 = (uint32_t)(fid ^ (fid >> 32));
        for (size_t v = 0; v < vc; v++) {
            out->points[count].qx = xv[v];
            out->points[count].qy = yv[v];
            out->points[count].z = zv[v];
            out->points[count].model_index = mi;
            out->points[count].id = id32;
            count++;
        }
    }

    out->count = count;
    return true;
}

void arpt_tree_data_free(arpt_tree_data *data) {
    if (data) {
        free(data->points);
        data->points = NULL;
        data->count = 0;
    }
}

/* POI decoding */

/* Resolve a string property by key index and copy into dst (up to max_len-1). */
static void resolve_string_property(arpentry_tiles_Feature_table_t feat,
                                    uint32_t key_idx,
                                    arpentry_tiles_Value_vec_t values,
                                    char *dst, size_t max_len) {
    dst[0] = '\0';
    if (key_idx == UINT32_MAX || !values) return;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                flatbuffers_string_t s =
                    arpentry_tiles_Value_string_value(val);
                if (s) {
                    size_t slen = strlen(s);
                    if (slen >= max_len) slen = max_len - 1;
                    memcpy(dst, s, slen);
                    dst[slen] = '\0';
                }
            }
            break;
        }
    }
}

bool arpt_decode_pois(const void *flatbuf, size_t size,
                      arpt_poi_data *out) {
    out->points = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t height_key_idx;
    uint32_t name_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer_ex(flatbuf, size, "poi", &class_key_idx, &height_key_idx,
                      &name_key_idx, &values);
    if (!layer) return true;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    /* Count total points */
    size_t total = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;
        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PointGeometry)
            continue;
        arpentry_tiles_PointGeometry_table_t pt =
            (arpentry_tiles_PointGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!pt) continue;
        flatbuffers_uint16_vec_t xv = arpentry_tiles_PointGeometry_x(pt);
        if (xv) total += flatbuffers_uint16_vec_len(xv);
    }
    if (total == 0) return true;

    out->points = malloc(total * sizeof(arpt_poi_point));
    if (!out->points) return false;

    size_t count = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat) continue;
        if (arpentry_tiles_Feature_geometry_type(feat) !=
            arpentry_tiles_Geometry_PointGeometry)
            continue;
        arpentry_tiles_PointGeometry_table_t pt =
            (arpentry_tiles_PointGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!pt) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_PointGeometry_x(pt);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_PointGeometry_y(pt);
        flatbuffers_int32_vec_t zv = arpentry_tiles_PointGeometry_z(pt);
        if (!xv || !yv || !zv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc) continue;
        if (flatbuffers_int32_vec_len(zv) != vc) continue;

        /* Resolve name once per feature (all points share same name) */
        char name[64];
        resolve_string_property(feat, name_key_idx, values, name, sizeof(name));

        for (size_t v = 0; v < vc; v++) {
            out->points[count].qx = xv[v];
            out->points[count].qy = yv[v];
            out->points[count].z = zv[v];
            memcpy(out->points[count].name, name, sizeof(name));
            count++;
        }
    }

    out->count = count;
    return true;
}

void arpt_poi_data_free(arpt_poi_data *data) {
    if (data) {
        free(data->points);
        data->points = NULL;
        data->count = 0;
    }
}
