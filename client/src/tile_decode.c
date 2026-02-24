#include "tile_decode.h"
#include "tile_reader.h"
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
