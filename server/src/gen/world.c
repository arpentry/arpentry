#include "world.h"
#include "terrain.h"
#include "surface.h"
#include "coords.h"
#include "tile.h"
#include "tile_builder.h"

#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define BROTLI_QUALITY 4

/* Build FlatBuffer tile from terrain + surface data. */
static void *build_tile_flatbuffer(const uint16_t *vx, const uint16_t *vy,
                                   const int32_t *vz, const int8_t *normals,
                                   int nv, const uint32_t *indices,
                                   int num_indices, const ms_patch *patches,
                                   int patch_count, size_t *fb_size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    /* Tile-scope property dictionary for surface "class" key */
    arpentry_tiles_Tile_keys_start(&builder);
    arpentry_tiles_Tile_keys_push_create_str(&builder, "class");
    arpentry_tiles_Tile_keys_end(&builder);

    /* Value dictionary: must match SURFACE_VAL_* index order */
#define PUSH_STR(s)                                                       \
    do {                                                                  \
        arpentry_tiles_Tile_values_push_start(&builder);                  \
        arpentry_tiles_Value_type_add(                                    \
            &builder, arpentry_tiles_PropertyValueType_String);           \
        arpentry_tiles_Value_string_value_create_str(&builder, (s));      \
        arpentry_tiles_Tile_values_push_end(&builder);                    \
    } while (0)
    arpentry_tiles_Tile_values_start(&builder);
    PUSH_STR("water");     /* 0 */
    PUSH_STR("desert");    /* 1 */
    PUSH_STR("forest");    /* 2 */
    PUSH_STR("grassland"); /* 3 */
    PUSH_STR("cropland");  /* 4 */
    PUSH_STR("shrub");     /* 5 */
    PUSH_STR("ice");       /* 6 */
    arpentry_tiles_Tile_values_end(&builder);
#undef PUSH_STR

    arpentry_tiles_Tile_layers_start(&builder);
    {
        /* Layer 0: terrain */
        arpentry_tiles_Tile_layers_push_start(&builder);
        arpentry_tiles_Layer_name_create_str(&builder, "terrain");

        arpentry_tiles_Layer_features_start(&builder);
        {
            arpentry_tiles_Layer_features_push_start(&builder);
            arpentry_tiles_Feature_id_add(&builder, 1);

            arpentry_tiles_MeshGeometry_ref_t mesh_ref;
            arpentry_tiles_MeshGeometry_start(&builder);
            arpentry_tiles_MeshGeometry_x_create(&builder, vx, (size_t)nv);
            arpentry_tiles_MeshGeometry_y_create(&builder, vy, (size_t)nv);
            arpentry_tiles_MeshGeometry_z_create(&builder, vz, (size_t)nv);
            arpentry_tiles_MeshGeometry_indices_create(&builder, indices,
                                                       (size_t)num_indices);
            arpentry_tiles_MeshGeometry_normals_create(&builder, normals,
                                                       (size_t)(nv * 2));

            arpentry_tiles_MeshGeometry_parts_start(&builder);
            arpentry_tiles_Part_t part = {0};
            part.first_index = 0;
            part.index_count = (uint32_t)num_indices;
            arpentry_tiles_MeshGeometry_parts_push(&builder, &part);
            arpentry_tiles_MeshGeometry_parts_end(&builder);

            mesh_ref = arpentry_tiles_MeshGeometry_end(&builder);
            arpentry_tiles_Feature_geometry_MeshGeometry_add(&builder,
                                                             mesh_ref);

            arpentry_tiles_Layer_features_push_end(&builder);
        }
        arpentry_tiles_Layer_features_end(&builder);
        arpentry_tiles_Tile_layers_push_end(&builder);

        /* Layer 1: surface */
        arpentry_tiles_Tile_layers_push_start(&builder);
        arpentry_tiles_Layer_name_create_str(&builder, "surface");

        arpentry_tiles_Layer_features_start(&builder);
        for (int pi = 0; pi < patch_count; pi++) {
            const ms_patch *p = &patches[pi];
            int32_t pz[9] = {0};
            uint32_t ring_off[2] = {0, (uint32_t)p->count};

            arpentry_tiles_Layer_features_push_start(&builder);
            arpentry_tiles_Feature_id_add(&builder, (uint64_t)(pi + 2));

            arpentry_tiles_PolygonGeometry_ref_t poly_ref;
            arpentry_tiles_PolygonGeometry_start(&builder);
            arpentry_tiles_PolygonGeometry_x_create(&builder, p->x,
                                                    (size_t)p->count);
            arpentry_tiles_PolygonGeometry_y_create(&builder, p->y,
                                                    (size_t)p->count);
            arpentry_tiles_PolygonGeometry_z_create(&builder, pz,
                                                    (size_t)p->count);
            arpentry_tiles_PolygonGeometry_ring_offsets_create(&builder,
                                                               ring_off, 2);
            poly_ref = arpentry_tiles_PolygonGeometry_end(&builder);
            arpentry_tiles_Feature_geometry_PolygonGeometry_add(&builder,
                                                                poly_ref);

            arpentry_tiles_Feature_properties_start(&builder);
            arpentry_tiles_Property_t prop;
            prop.key = 0;
            prop.value = (uint32_t)p->cls;
            arpentry_tiles_Feature_properties_push(&builder, &prop);
            arpentry_tiles_Feature_properties_end(&builder);

            arpentry_tiles_Layer_features_push_end(&builder);
        }
        arpentry_tiles_Layer_features_end(&builder);
        arpentry_tiles_Tile_layers_push_end(&builder);
    }
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    void *fb = flatcc_builder_finalize_buffer(&builder, fb_size);
    flatcc_builder_clear(&builder);
    return fb;
}

/* Public API */

bool arpt_generate_terrain(int level, int x, int y, uint8_t **out,
                           size_t *out_size) {
    if (!out || !out_size) return false;

    arpt_bounds bounds = arpt_tile_bounds(level, x, y);
    double lon_span = bounds.east - bounds.west;
    double lat_span = bounds.north - bounds.south;
    double cell_lon = lon_span / TERRAIN_GRID;
    double cell_lat = lat_span / TERRAIN_GRID;

    /* Approximate cell size in meters (for normal computation) */
    double mid_lat = (bounds.south + bounds.north) * 0.5;
    double cos_lat = cos(mid_lat * PI / 180.0);
    double cell_w_m = cell_lon * 111319.5 * cos_lat;
    double cell_h_m = cell_lat * 111319.5;

    int nv = TERRAIN_VERTS * TERRAIN_VERTS;
    int num_indices = TERRAIN_GRID * TERRAIN_GRID * 2 * 3;
    int pad_w = TERRAIN_VERTS + 2;

    /* Phase 1: padded elevation grid */
    double *elev = build_elevation_grid(bounds, cell_lon, cell_lat);
    if (!elev) return false;

    /* Phase 2-3: vertex + index arrays */
    uint16_t *vx = malloc((size_t)nv * sizeof(uint16_t));
    uint16_t *vy = malloc((size_t)nv * sizeof(uint16_t));
    int32_t *vz = malloc((size_t)nv * sizeof(int32_t));
    int8_t *normals = malloc((size_t)nv * 2 * sizeof(int8_t));
    uint32_t *indices = malloc((size_t)num_indices * sizeof(uint32_t));
    ms_patch *patches = NULL;
    if (!vx || !vy || !vz || !normals || !indices) goto fail;

    build_vertices(bounds, cell_lon, cell_lat, cell_w_m, cell_h_m, elev, pad_w,
                   vx, vy, vz, normals);
    build_indices(indices);
    free(elev);
    elev = NULL;

    /* Phase 4-5: surface classification + marching squares */
    int *vert_class = malloc(SURFACE_VERTS * SURFACE_VERTS * sizeof(int));
    if (!vert_class) goto fail;
    classify_surface(bounds, lon_span, lat_span, vert_class);

    patches =
        malloc((size_t)(SURFACE_TOTAL * SURFACE_TOTAL * 4) * sizeof(ms_patch));
    if (!patches) {
        free(vert_class);
        goto fail;
    }
    int patch_count = generate_surface_patches(vert_class, patches);
    free(vert_class);

    /* Phase 6: FlatBuffer + Brotli compression */
    size_t fb_size;
    void *fb =
        build_tile_flatbuffer(vx, vy, vz, normals, nv, indices, num_indices,
                              patches, patch_count, &fb_size);

    free(vx);
    free(vy);
    free(vz);
    free(normals);
    free(indices);
    free(patches);

    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;

fail:
    free(elev);
    free(vx);
    free(vy);
    free(vz);
    free(normals);
    free(indices);
    free(patches);
    return false;
}
