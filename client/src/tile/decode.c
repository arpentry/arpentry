#include "decode.h"
#include "prepare.h"
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
        if (name && strcmp(name, ARPT_LAYER_TERRAIN_NAME) == 0) {
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

/* Find a layer by name and resolve "class" and optionally "name" key indices. */
static arpentry_tiles_Layer_table_t
find_layer_ex(const void *flatbuf, size_t size, const char *name,
              uint32_t *class_key_idx, uint32_t *name_key_idx,
              arpentry_tiles_Value_vec_t *values) {
    *class_key_idx = UINT32_MAX;
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
            else if (k && name_key_idx && strcmp(k, "name") == 0)
                *name_key_idx = (uint32_t)i;
        }
    }

    return found;
}

/* Convenience wrapper that doesn't resolve "name" key. */
static arpentry_tiles_Layer_table_t
find_layer(const void *flatbuf, size_t size, const char *name,
           uint32_t *class_key_idx, arpentry_tiles_Value_vec_t *values) {
    return find_layer_ex(flatbuf, size, name, class_key_idx, NULL, values);
}

/* Count total rings across all PolygonGeometry features in a layer. */
static size_t count_polygon_rings(arpentry_tiles_Feature_vec_t features,
                                  size_t n_feat) {
    size_t total = 0;
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
        flatbuffers_uint32_vec_t ring_off =
            arpentry_tiles_PolygonGeometry_ring_offsets(poly);
        if (ring_off && flatbuffers_uint32_vec_len(ring_off) >= 2)
            total += flatbuffers_uint32_vec_len(ring_off) - 1;
        else
            total += 1; /* single ring (no offsets) */
    }
    return total;
}

/* Decode all PolygonGeometry features from a named layer.
 * Each ring in the geometry becomes a separate surface polygon.
 * Holes are handled at render time via stencil-based even-odd fill. */
static bool decode_polygon_layer(const void *flatbuf, size_t size,
                                 const char *layer_name,
                                 const char (*class_names)[32],
                                 int class_count,
                                 arpt_surface_data *out) {
    out->polygons = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer(flatbuf, size, layer_name, &class_key_idx, &values);
    if (!layer) return true;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    size_t max_polys = count_polygon_rings(features, n_feat);
    if (max_polys == 0) return true;

    out->polygons = malloc(max_polys * sizeof(arpt_surface_polygon));
    if (!out->polygons) return false;

    size_t count = 0;
    uint16_t poly_id = 0;
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

        flatbuffers_int32_vec_t zv = arpentry_tiles_PolygonGeometry_z(poly);
        uint8_t cls = resolve_class(feat, class_key_idx, values,
                                    class_names, class_count);

        flatbuffers_uint32_vec_t ring_off =
            arpentry_tiles_PolygonGeometry_ring_offsets(poly);
        size_t n_rings = 1;
        if (ring_off && flatbuffers_uint32_vec_len(ring_off) >= 2)
            n_rings = flatbuffers_uint32_vec_len(ring_off) - 1;

        uint16_t this_poly_id = poly_id++;

        for (size_t ri = 0; ri < n_rings; ri++) {
            size_t ring_start = 0;
            size_t ring_end = vc;
            if (ring_off && flatbuffers_uint32_vec_len(ring_off) >= 2) {
                ring_start = ring_off[ri];
                ring_end = ring_off[ri + 1];
                if (ring_end > vc) ring_end = vc;
            }
            size_t ring_vc = ring_end - ring_start;
            if (ring_vc < 3) continue;

            out->polygons[count].x = xv + ring_start;
            out->polygons[count].y = yv + ring_start;
            out->polygons[count].z = zv ? zv + ring_start : NULL;
            out->polygons[count].vertex_count = ring_vc;
            out->polygons[count].cls = cls;
            out->polygons[count].poly_id = this_poly_id;
            count++;
        }
    }

    out->count = count;
    return true;
}

bool arpt_decode_surface_layer(const void *flatbuf, size_t size,
                               const char *layer_name,
                               const char (*class_names)[32], int class_count,
                               arpt_surface_data *out) {
    return decode_polygon_layer(flatbuf, size, layer_name, class_names,
                                class_count, out);
}

void arpt_surface_data_free(arpt_surface_data *data) {
    if (data) {
        free(data->polygons);
        data->polygons = NULL;
        data->count = 0;
    }
}

/* Locate a layer table by name (no property-dictionary resolution). */
static arpentry_tiles_Layer_table_t
find_layer_by_name(const void *flatbuf, size_t size, const char *name) {
    if (!flatbuf || size < 8) return NULL;
    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
    if (!tile) return NULL;
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    if (!layers) return NULL;
    size_t n = arpentry_tiles_Layer_vec_len(layers);
    for (size_t i = 0; i < n; i++) {
        arpentry_tiles_Layer_table_t layer =
            arpentry_tiles_Layer_vec_at(layers, i);
        flatbuffers_string_t lname = arpentry_tiles_Layer_name(layer);
        if (lname && strcmp(lname, name) == 0) return layer;
    }
    return NULL;
}

/* Resolve the dictionary index of a property key by name, or UINT32_MAX. */
static uint32_t find_key_index(const void *flatbuf, const char *key) {
    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
    if (!tile) return UINT32_MAX;
    flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
    if (!keys) return UINT32_MAX;
    size_t nkeys = flatbuffers_string_vec_len(keys);
    for (size_t i = 0; i < nkeys; i++) {
        flatbuffers_string_t k = flatbuffers_string_vec_at(keys, i);
        if (k && strcmp(k, key) == 0) return (uint32_t)i;
    }
    return UINT32_MAX;
}

/* Read an integer-valued property of a feature, or `dflt` when absent. */
static int64_t resolve_int(arpentry_tiles_Feature_table_t feat, uint32_t key_idx,
                           arpentry_tiles_Value_vec_t values, int64_t dflt) {
    if (key_idx == UINT32_MAX || !values) return dflt;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return dflt;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                return arpentry_tiles_Value_int_value(val);
            }
            break;
        }
    }
    return dflt;
}

/* Read a double-valued property of a feature, or `dflt` when absent. */
static double resolve_double(arpentry_tiles_Feature_table_t feat,
                             uint32_t key_idx,
                             arpentry_tiles_Value_vec_t values, double dflt) {
    if (key_idx == UINT32_MAX || !values) return dflt;
    arpentry_tiles_Property_vec_t props =
        arpentry_tiles_Feature_properties(feat);
    if (!props) return dflt;
    size_t np = arpentry_tiles_Property_vec_len(props);
    for (size_t p = 0; p < np; p++) {
        arpentry_tiles_Property_struct_t pr =
            arpentry_tiles_Property_vec_at(props, p);
        if (pr && pr->key == key_idx) {
            size_t vi = pr->value;
            if (vi < arpentry_tiles_Value_vec_len(values)) {
                arpentry_tiles_Value_table_t val =
                    arpentry_tiles_Value_vec_at(values, vi);
                return arpentry_tiles_Value_double_value(val);
            }
            break;
        }
    }
    return dflt;
}

/* Keep a feature given a level filter: 0 keeps all (buildings); +1 keeps only
   bridges (level > 0); -1 keeps only tunnels (level < 0). */
static bool keep_by_level(arpentry_tiles_Feature_table_t feat, int sign,
                          uint32_t level_key, arpentry_tiles_Value_vec_t values) {
    if (sign == 0) return true;
    int64_t lv = resolve_int(feat, level_key, values, 0);
    return sign > 0 ? lv > 0 : lv < 0;
}

/* Concatenate the MeshGeometry features of a layer into one mesh primitive
   (xy/z/normals/indices), offsetting indices per feature. `level_sign` filters
   by the reserved `level` property — 0 takes all (buildings), +1 only bridges,
   -1 only tunnels. Returns false (with `out` zeroed) when no matching mesh. */
static bool collect_layer_meshes(const void *flatbuf,
                                 arpentry_tiles_Layer_table_t layer,
                                 int level_sign,
                                 const char (*class_names)[32], int class_count,
                                 const float (*colors)[4],
                                 arpt_building_prim *out) {
    memset(out, 0, sizeof(*out));

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return false;
    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return false;

    /* Resolve the `level` key + value table once for the filtered passes. */
    uint32_t level_key = UINT32_MAX;
    arpentry_tiles_Value_vec_t values = NULL;
    if (level_sign != 0) {
        level_key = find_key_index(flatbuf, "level");
        arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
        values = tile ? arpentry_tiles_Tile_values(tile) : NULL;
    }

    /* Per-vertex deck colour: resolve each structure's road class against the
       style so its top face is painted the same grey its ribbon uses (not always
       motorway grey). Skipped when the caller passes no style (buildings). */
    uint32_t class_key = UINT32_MAX;
    bool want_color = colors != NULL && class_names != NULL && class_count > 0;
    if (want_color) {
        class_key = find_key_index(flatbuf, "class");
        if (!values) {
            arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
            values = tile ? arpentry_tiles_Tile_values(tile) : NULL;
        }
    }

    /* First pass: total vertices and indices across all matching mesh features. */
    size_t total_v = 0, total_i = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat || arpentry_tiles_Feature_geometry_type(feat) !=
                         arpentry_tiles_Geometry_MeshGeometry)
            continue;
        if (!keep_by_level(feat, level_sign, level_key, values)) continue;
        arpentry_tiles_MeshGeometry_table_t mesh =
            (arpentry_tiles_MeshGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!mesh) continue;
        flatbuffers_uint16_vec_t xv = arpentry_tiles_MeshGeometry_x(mesh);
        flatbuffers_uint32_vec_t iv = arpentry_tiles_MeshGeometry_indices(mesh);
        if (!xv || !iv) continue;
        total_v += flatbuffers_uint16_vec_len(xv);
        total_i += flatbuffers_uint32_vec_len(iv);
    }
    if (total_v == 0 || total_i == 0) return false;

    out->xy = malloc(total_v * 2 * sizeof(uint16_t));
    out->z = malloc(total_v * sizeof(int32_t));
    out->normals = calloc(total_v, 2);
    out->indices = malloc(total_i * sizeof(uint32_t));
    out->color = want_color ? malloc(total_v * 4) : NULL;
    if (!out->xy || !out->z || !out->normals || !out->indices ||
        (want_color && !out->color)) {
        free(out->xy);
        free(out->z);
        free(out->normals);
        free(out->indices);
        free(out->color);
        memset(out, 0, sizeof(*out));
        return false;
    }

    /* Second pass: concatenate the meshes, offsetting indices per feature. */
    size_t vi = 0, ii = 0;
    for (size_t i = 0; i < n_feat; i++) {
        arpentry_tiles_Feature_table_t feat =
            arpentry_tiles_Feature_vec_at(features, i);
        if (!feat || arpentry_tiles_Feature_geometry_type(feat) !=
                         arpentry_tiles_Geometry_MeshGeometry)
            continue;
        if (!keep_by_level(feat, level_sign, level_key, values)) continue;
        arpentry_tiles_MeshGeometry_table_t mesh =
            (arpentry_tiles_MeshGeometry_table_t)
                arpentry_tiles_Feature_geometry(feat);
        if (!mesh) continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_MeshGeometry_x(mesh);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_MeshGeometry_y(mesh);
        flatbuffers_int32_vec_t zv = arpentry_tiles_MeshGeometry_z(mesh);
        flatbuffers_uint32_vec_t iv = arpentry_tiles_MeshGeometry_indices(mesh);
        if (!xv || !yv || !zv || !iv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc ||
            flatbuffers_int32_vec_len(zv) != vc)
            continue;
        size_t ic = flatbuffers_uint32_vec_len(iv);

        flatbuffers_int8_vec_t nv = arpentry_tiles_MeshGeometry_normals(mesh);
        bool have_n = nv && flatbuffers_int8_vec_len(nv) == 2 * vc;

        /* Resolve this feature's deck colour once; alpha 0 (an unresolved class)
           tells the shader to fall back to its motorway-grey default. */
        uint8_t cr = 0, cg = 0, cb = 0, ca = 0;
        if (out->color) {
            uint8_t cls =
                resolve_class(feat, class_key, values, class_names, class_count);
            if (cls != 0) {
                const float *c = colors[cls];
                cr = (uint8_t)(c[0] <= 0.0f ? 0 : c[0] >= 1.0f ? 255 : c[0] * 255.0f + 0.5f);
                cg = (uint8_t)(c[1] <= 0.0f ? 0 : c[1] >= 1.0f ? 255 : c[1] * 255.0f + 0.5f);
                cb = (uint8_t)(c[2] <= 0.0f ? 0 : c[2] >= 1.0f ? 255 : c[2] * 255.0f + 0.5f);
                ca = 255;
            }
        }

        uint32_t base = (uint32_t)vi;
        for (size_t v = 0; v < vc; v++) {
            out->xy[(vi + v) * 2] = xv[v];
            out->xy[(vi + v) * 2 + 1] = yv[v];
            out->z[vi + v] = zv[v];
            if (have_n) {
                out->normals[(vi + v) * 2] = nv[2 * v];
                out->normals[(vi + v) * 2 + 1] = nv[2 * v + 1];
            }
            if (out->color) {
                out->color[(vi + v) * 4] = cr;
                out->color[(vi + v) * 4 + 1] = cg;
                out->color[(vi + v) * 4 + 2] = cb;
                out->color[(vi + v) * 4 + 3] = ca;
            }
        }
        for (size_t k = 0; k < ic; k++)
            out->indices[ii + k] = base + iv[k];

        vi += vc;
        ii += ic;
    }

    out->vertex_count = vi;
    out->index_count = ii;
    return true;
}

bool arpt_decode_building_mesh(const void *flatbuf, size_t size,
                               const char *layer_name,
                               arpt_building_prim *out) {
    memset(out, 0, sizeof(*out));

    arpentry_tiles_Layer_table_t layer =
        find_layer_by_name(flatbuf, size, layer_name);
    if (!layer) return false;

    /* Buildings ship as server-baked MeshGeometry; bail out (no mesh) when the
       layer's first feature is any other geometry type, so non-mesh layers are
       skipped without scanning them. */
    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features || arpentry_tiles_Feature_vec_len(features) == 0) return false;
    arpentry_tiles_Feature_table_t f0 =
        arpentry_tiles_Feature_vec_at(features, 0);
    if (!f0 || arpentry_tiles_Feature_geometry_type(f0) !=
                   arpentry_tiles_Geometry_MeshGeometry)
        return false;

    /* Buildings carry their own baked colours and use a different draw path, so
       no per-vertex road-class colour is resolved here. */
    return collect_layer_meshes(flatbuf, layer, 0, NULL, 0, NULL, out);
}

/* Road-structure box prisms ride in the transportation layer alongside the (more
   numerous) road lines, so — unlike buildings — the first feature is not a mesh.
   Scan the whole layer, collecting only the matching MeshGeometry features:
   bridges (`level_sign` +1) and tunnels (-1) are split so each colours its own.
   The style (`class_names`/`colors`) paints each deck top its road's own grey. */
static bool decode_structure_mesh(const void *flatbuf, size_t size,
                                  const char *layer_name, int level_sign,
                                  const char (*class_names)[32], int class_count,
                                  const float (*colors)[4],
                                  arpt_building_prim *out) {
    memset(out, 0, sizeof(*out));
    arpentry_tiles_Layer_table_t layer =
        find_layer_by_name(flatbuf, size, layer_name);
    if (!layer) return false;
    return collect_layer_meshes(flatbuf, layer, level_sign, class_names,
                                class_count, colors, out);
}

bool arpt_decode_bridge_mesh(const void *flatbuf, size_t size,
                             const char *layer_name,
                             const char (*class_names)[32], int class_count,
                             const float (*colors)[4], arpt_building_prim *out) {
    return decode_structure_mesh(flatbuf, size, layer_name, +1, class_names,
                                 class_count, colors, out);
}

bool arpt_decode_tunnel_mesh(const void *flatbuf, size_t size,
                             const char *layer_name,
                             const char (*class_names)[32], int class_count,
                             const float (*colors)[4], arpt_building_prim *out) {
    return decode_structure_mesh(flatbuf, size, layer_name, -1, class_names,
                                 class_count, colors, out);
}

/* Line decoding */

bool arpt_decode_lines(const void *flatbuf, size_t size,
                       const char *layer_name,
                       const char (*class_names)[32], int class_count,
                       arpt_line_data *out) {
    out->lines = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer(flatbuf, size, layer_name, &class_key_idx, &values);
    if (!layer) return true;

    /* Physical carriageway width the tiler bakes on drivable roads. */
    uint32_t width_key = find_key_index(flatbuf, "width_m");

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    /* First pass: count total lines (each part of a MultiLineString is one) */
    size_t total_lines = 0;
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
        flatbuffers_uint32_vec_t offsets =
            arpentry_tiles_LineGeometry_line_offsets(line);
        if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2)
            total_lines += flatbuffers_uint32_vec_len(offsets) - 1;
        else
            total_lines++;
    }

    out->lines = malloc(total_lines * sizeof(arpt_line_feature));
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

        /* Per-vertex road elevation the tiler baked from the terrain surface;
           absent (flat) on the DEM-less path. Honoured only when it covers every
           vertex. */
        flatbuffers_int32_vec_t zv = arpentry_tiles_LineGeometry_z(line);
        if (zv && flatbuffers_int32_vec_len(zv) != vc) zv = NULL;

        uint8_t cls = resolve_class(feat, class_key_idx, values,
                                    class_names, class_count);
        float width_m = (float)resolve_double(feat, width_key, values, 0.0);

        flatbuffers_uint32_vec_t offsets =
            arpentry_tiles_LineGeometry_line_offsets(line);
        size_t n_parts = 1;
        if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2)
            n_parts = flatbuffers_uint32_vec_len(offsets) - 1;

        for (size_t p = 0; p < n_parts; p++) {
            size_t start = 0, end = vc;
            if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2) {
                start = offsets[p];
                end = offsets[p + 1];
                if (end > vc) end = vc;
            }
            size_t part_vc = end - start;
            if (part_vc < 2) continue;

            out->lines[count].x = xv + start;
            out->lines[count].y = yv + start;
            out->lines[count].z = zv ? zv + start : NULL;
            out->lines[count].vertex_count = part_vc;
            out->lines[count].cls = cls;
            out->lines[count].width_m = width_m;
            count++;
        }
    }

    out->count = count;
    return true;
}

void arpt_line_data_free(arpt_line_data *data) {
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
                       const char *layer_name,
                       const char *const *class_names, int class_count,
                       arpt_tree_data *out) {
    out->points = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer(flatbuf, size, layer_name, &class_key_idx, &values);
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
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc) continue;
        if (zv && flatbuffers_int32_vec_len(zv) != vc) continue;

        uint8_t mi = tree_model_from_class(feat, class_key_idx, values,
                                                 class_names, class_count);
        uint64_t fid = arpentry_tiles_Feature_id(feat);
        uint32_t id32 = (uint32_t)(fid ^ (fid >> 32));
        for (size_t v = 0; v < vc; v++) {
            out->points[count].qx = xv[v];
            out->points[count].qy = yv[v];
            out->points[count].z = zv ? zv[v] : 0;
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
                      const char *layer_name,
                      arpt_poi_data *out) {
    out->points = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t name_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer_ex(flatbuf, size, layer_name, &class_key_idx,
                      &name_key_idx, &values);
    if (!layer) return true;

    /* Resolve "icon" key index from the tile-scope key dictionary */
    uint32_t icon_key_idx = UINT32_MAX;
    {
        arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
        flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
        if (keys) {
            size_t nkeys = flatbuffers_string_vec_len(keys);
            for (size_t i = 0; i < nkeys; i++) {
                flatbuffers_string_t k = flatbuffers_string_vec_at(keys, i);
                if (k && strcmp(k, "icon") == 0) {
                    icon_key_idx = (uint32_t)i;
                    break;
                }
            }
        }
    }

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
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc) continue;
        if (zv && flatbuffers_int32_vec_len(zv) != vc) continue;

        /* Resolve name and icon once per feature */
        char name[64];
        resolve_string_property(feat, name_key_idx, values, name, sizeof(name));
        char icon[32];
        resolve_string_property(feat, icon_key_idx, values, icon, sizeof(icon));

        for (size_t v = 0; v < vc; v++) {
            out->points[count].qx = xv[v];
            out->points[count].qy = yv[v];
            out->points[count].z = zv ? zv[v] : 0;
            memcpy(out->points[count].name, name, sizeof(name));
            memcpy(out->points[count].icon, icon, sizeof(icon));
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

/* Line label decoding */

bool arpt_decode_line_labels(const void *flatbuf, size_t size,
                             const char *layer_name,
                             arpt_line_label_data *out) {
    out->features = NULL;
    out->count = 0;

    uint32_t class_key_idx;
    uint32_t name_key_idx;
    arpentry_tiles_Value_vec_t values;
    arpentry_tiles_Layer_table_t layer =
        find_layer_ex(flatbuf, size, layer_name, &class_key_idx,
                      &name_key_idx, &values);
    if (!layer || name_key_idx == UINT32_MAX) return true;

    arpentry_tiles_Feature_vec_t features =
        arpentry_tiles_Layer_features(layer);
    if (!features) return true;

    size_t n_feat = arpentry_tiles_Feature_vec_len(features);
    if (n_feat == 0) return true;

    /* First pass: count parts of named lines */
    size_t total = 0;
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
        flatbuffers_uint32_vec_t offsets =
            arpentry_tiles_LineGeometry_line_offsets(line);
        if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2)
            total += flatbuffers_uint32_vec_len(offsets) - 1;
        else
            total++;
    }
    if (total == 0) return true;

    out->features = malloc(total * sizeof(arpt_line_label_feature));
    if (!out->features) return false;

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

        char name[64];
        resolve_string_property(feat, name_key_idx, values, name,
                                sizeof(name));
        if (name[0] == '\0') continue;

        flatbuffers_uint16_vec_t xv = arpentry_tiles_LineGeometry_x(line);
        flatbuffers_uint16_vec_t yv = arpentry_tiles_LineGeometry_y(line);
        if (!xv || !yv) continue;

        size_t vc = flatbuffers_uint16_vec_len(xv);
        if (flatbuffers_uint16_vec_len(yv) != vc || vc < 2) continue;

        flatbuffers_uint32_vec_t offsets =
            arpentry_tiles_LineGeometry_line_offsets(line);
        size_t n_parts = 1;
        if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2)
            n_parts = flatbuffers_uint32_vec_len(offsets) - 1;

        for (size_t p = 0; p < n_parts && count < total; p++) {
            size_t start = 0, end = vc;
            if (offsets && flatbuffers_uint32_vec_len(offsets) >= 2) {
                start = offsets[p];
                end = offsets[p + 1];
                if (end > vc) end = vc;
                if (start > end) continue;
            }
            size_t part_vc = end - start;
            if (part_vc < 2) continue;

            arpt_line_label_feature *f = &out->features[count];
            f->x = xv + start;
            f->y = yv + start;
            f->vertex_count = part_vc;
            memcpy(f->name, name, sizeof(name));
            count++;
        }
    }

    out->count = count;
    return true;
}

void arpt_line_label_data_free(arpt_line_label_data *data) {
    if (data) {
        free(data->features);
        data->features = NULL;
        data->count = 0;
    }
}
