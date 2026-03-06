#include "world.h"
#include "terrain.h"
#include "surface.h"
#include "town.h"
#include "tree.h"
#include "poi.h"
#include "coords.h"
#include "tile.h"
#include "tile_builder.h"

#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define BROTLI_QUALITY 4
#define M_TO_DEG (1.0 / 111319.5) /* meters to degrees at equator */

/* Build FlatBuffer tile from terrain + surface + town data. */
static void *build_tile_flatbuffer(const uint16_t *vx, const uint16_t *vy,
                                   const int32_t *vz, const int8_t *normals,
                                   int nv, const uint32_t *indices,
                                   int num_indices, const ms_patch *patches,
                                   int patch_count, arpt_bounds bounds,
                                   size_t *fb_size) {
    bool has_town = town_overlaps(bounds);
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    /* Tile-scope property dictionary: keys */
    arpentry_tiles_Tile_keys_start(&builder);
    arpentry_tiles_Tile_keys_push_create_str(&builder, "class");  /* 0 */
    arpentry_tiles_Tile_keys_push_create_str(&builder, "height"); /* 1 */
    arpentry_tiles_Tile_keys_push_create_str(&builder, "name");   /* 2 POI_KEY_NAME */
    arpentry_tiles_Tile_keys_end(&builder);

    /* Value dictionary: must match SURFACE_VAL_* / TOWN_VAL_* index order */
#define PUSH_STR(s)                                                       \
    do {                                                                  \
        arpentry_tiles_Tile_values_push_start(&builder);                  \
        arpentry_tiles_Value_type_add(                                    \
            &builder, arpentry_tiles_PropertyValueType_String);           \
        arpentry_tiles_Value_string_value_create_str(&builder, (s));      \
        arpentry_tiles_Tile_values_push_end(&builder);                    \
    } while (0)
#define PUSH_INT(v)                                                       \
    do {                                                                  \
        arpentry_tiles_Tile_values_push_start(&builder);                  \
        arpentry_tiles_Value_type_add(                                    \
            &builder, arpentry_tiles_PropertyValueType_Int);              \
        arpentry_tiles_Value_int_value_add(&builder, (int64_t)(v));       \
        arpentry_tiles_Tile_values_push_end(&builder);                    \
    } while (0)
    arpentry_tiles_Tile_values_start(&builder);
    PUSH_STR("water");       /* 0  */
    PUSH_STR("desert");      /* 1  */
    PUSH_STR("forest");      /* 2  */
    PUSH_STR("grassland");   /* 3  */
    PUSH_STR("cropland");    /* 4  */
    PUSH_STR("shrub");       /* 5  */
    PUSH_STR("ice");         /* 6  */
    PUSH_STR("primary");     /* 7  TOWN_VAL_PRIMARY */
    PUSH_STR("residential"); /* 8  TOWN_VAL_RESIDENTIAL */
    PUSH_STR("building");    /* 9  TOWN_VAL_BUILDING */
    PUSH_INT(5);             /* 10 TOWN_VAL_H5 */
    PUSH_INT(8);             /* 11 TOWN_VAL_H8 */
    PUSH_INT(10);            /* 12 TOWN_VAL_H10 */
    PUSH_INT(12);            /* 13 TOWN_VAL_H12 */
    PUSH_INT(15);            /* 14 TOWN_VAL_H15 */
    PUSH_STR("oak");          /* 15 TREE_VAL_OAK */
    PUSH_STR("pine");         /* 16 TREE_VAL_PINE */
    PUSH_STR("birch");        /* 17 TREE_VAL_BIRCH */
    PUSH_STR("poi");          /* 18 POI_VAL_POI */
    /* POI name strings (19+) — must match poi_get_points() order */
    {
        const poi_point *pp = poi_get_points();
        int np = poi_count();
        for (int pi = 0; pi < np; pi++)
            PUSH_STR(pp[pi].name);
    }
    arpentry_tiles_Tile_values_end(&builder);
#undef PUSH_INT
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

        /* Layer 2: highway (only when tile overlaps the town) */
        if (has_town) {
            const town_road *roads = town_get_roads();
            int road_count = town_road_count();

            arpentry_tiles_Tile_layers_push_start(&builder);
            arpentry_tiles_Layer_name_create_str(&builder, "highway");

            arpentry_tiles_Layer_features_start(&builder);
            for (int ri = 0; ri < road_count; ri++) {
                const town_road *r = &roads[ri];
                uint16_t rx[2] = {arpt_quantize_lon(r->lon1, bounds),
                                  arpt_quantize_lon(r->lon2, bounds)};
                uint16_t ry[2] = {arpt_quantize_lat(r->lat1, bounds),
                                  arpt_quantize_lat(r->lat2, bounds)};
                int32_t rz[2] = {0, 0};

                arpentry_tiles_Layer_features_push_start(&builder);
                arpentry_tiles_Feature_id_add(&builder,
                                              (uint64_t)(100000 + ri));

                arpentry_tiles_LineGeometry_ref_t line_ref;
                arpentry_tiles_LineGeometry_start(&builder);
                arpentry_tiles_LineGeometry_x_create(&builder, rx, 2);
                arpentry_tiles_LineGeometry_y_create(&builder, ry, 2);
                arpentry_tiles_LineGeometry_z_create(&builder, rz, 2);
                line_ref = arpentry_tiles_LineGeometry_end(&builder);
                arpentry_tiles_Feature_geometry_LineGeometry_add(&builder,
                                                                 line_ref);

                arpentry_tiles_Feature_properties_start(&builder);
                arpentry_tiles_Property_t rprop = {0};
                rprop.key = 0;
                rprop.value = (uint32_t)r->cls;
                arpentry_tiles_Feature_properties_push(&builder, &rprop);
                arpentry_tiles_Feature_properties_end(&builder);

                arpentry_tiles_Layer_features_push_end(&builder);
            }
            arpentry_tiles_Layer_features_end(&builder);
            arpentry_tiles_Tile_layers_push_end(&builder);
        }

        /* Layer 3: building (only when tile overlaps the town) */
        if (has_town) {
            const town_building *bldgs = town_get_buildings();
            int bldg_count = town_building_count();

            arpentry_tiles_Tile_layers_push_start(&builder);
            arpentry_tiles_Layer_name_create_str(&builder, "building");

            arpentry_tiles_Layer_features_start(&builder);
            for (int bi = 0; bi < bldg_count; bi++) {
                const town_building *b = &bldgs[bi];
                double hw = b->w_m * M_TO_DEG * 0.5;
                double hh = b->h_m * M_TO_DEG * 0.5;

                /* CCW ring (matching surface convention): SW SE NE NW close */
                uint16_t bx[5], by[5];
                double lons[5] = {b->lon - hw, b->lon + hw, b->lon + hw,
                                  b->lon - hw, b->lon - hw};
                double lats[5] = {b->lat - hh, b->lat - hh, b->lat + hh,
                                  b->lat + hh, b->lat - hh};
                for (int vi = 0; vi < 5; vi++) {
                    bx[vi] = arpt_quantize_lon(lons[vi], bounds);
                    by[vi] = arpt_quantize_lat(lats[vi], bounds);
                }
                int32_t base_z =
                    arpt_meters_to_mm(terrain_elevation(b->lon, b->lat));
                int32_t bz[5] = {base_z, base_z, base_z, base_z, base_z};
                uint32_t ring_off[2] = {0, 5};

                arpentry_tiles_Layer_features_push_start(&builder);
                arpentry_tiles_Feature_id_add(&builder,
                                              (uint64_t)(200000 + bi));

                arpentry_tiles_PolygonGeometry_ref_t bpoly_ref;
                arpentry_tiles_PolygonGeometry_start(&builder);
                arpentry_tiles_PolygonGeometry_x_create(&builder, bx, 5);
                arpentry_tiles_PolygonGeometry_y_create(&builder, by, 5);
                arpentry_tiles_PolygonGeometry_z_create(&builder, bz, 5);
                arpentry_tiles_PolygonGeometry_ring_offsets_create(
                    &builder, ring_off, 2);
                bpoly_ref = arpentry_tiles_PolygonGeometry_end(&builder);
                arpentry_tiles_Feature_geometry_PolygonGeometry_add(
                    &builder, bpoly_ref);

                arpentry_tiles_Feature_properties_start(&builder);
                arpentry_tiles_Property_t bprop = {0};
                bprop.key = 0;
                bprop.value = (uint32_t)b->cls;
                arpentry_tiles_Feature_properties_push(&builder, &bprop);
                bprop.key = TOWN_KEY_HEIGHT;
                bprop.value = (uint32_t)b->height_val;
                arpentry_tiles_Feature_properties_push(&builder, &bprop);
                arpentry_tiles_Feature_properties_end(&builder);

                arpentry_tiles_Layer_features_push_end(&builder);
            }
            arpentry_tiles_Layer_features_end(&builder);
            arpentry_tiles_Tile_layers_push_end(&builder);
        }

        /* Layer 4: tree (forest point features) */
        {
            tree_point trees[TREE_GRID_MAX];
            int tree_count = generate_trees(bounds, trees, TREE_GRID_MAX);

            if (tree_count > 0) {
                arpentry_tiles_Tile_layers_push_start(&builder);
                arpentry_tiles_Layer_name_create_str(&builder, "tree");

                arpentry_tiles_Layer_features_start(&builder);
                for (int ti = 0; ti < tree_count; ti++) {
                    /* Only emit tree if it falls within this tile's proper area
                     * (not buffer zone) so each tree appears in exactly one tile. */
                    if (trees[ti].lon < bounds.west || trees[ti].lon >= bounds.east ||
                        trees[ti].lat < bounds.south || trees[ti].lat >= bounds.north)
                        continue;

                    uint16_t tx = arpt_quantize_lon(trees[ti].lon, bounds);
                    uint16_t ty = arpt_quantize_lat(trees[ti].lat, bounds);
                    int32_t tz = arpt_meters_to_mm(
                        terrain_elevation(trees[ti].lon, trees[ti].lat));

                    arpentry_tiles_Layer_features_push_start(&builder);
                    arpentry_tiles_Feature_id_add(&builder, trees[ti].id);

                    arpentry_tiles_PointGeometry_ref_t pt_ref;
                    arpentry_tiles_PointGeometry_start(&builder);
                    arpentry_tiles_PointGeometry_x_create(&builder, &tx, 1);
                    arpentry_tiles_PointGeometry_y_create(&builder, &ty, 1);
                    arpentry_tiles_PointGeometry_z_create(&builder, &tz, 1);
                    pt_ref = arpentry_tiles_PointGeometry_end(&builder);
                    arpentry_tiles_Feature_geometry_PointGeometry_add(&builder,
                                                                      pt_ref);

                    uint32_t tree_val;
                    switch (trees[ti].type) {
                    case TREE_TYPE_PINE:  tree_val = TREE_VAL_PINE;  break;
                    case TREE_TYPE_BIRCH: tree_val = TREE_VAL_BIRCH; break;
                    default:              tree_val = TREE_VAL_OAK;   break;
                    }
                    arpentry_tiles_Feature_properties_start(&builder);
                    arpentry_tiles_Property_t tprop = {0};
                    tprop.key = 0;
                    tprop.value = tree_val;
                    arpentry_tiles_Feature_properties_push(&builder, &tprop);
                    arpentry_tiles_Feature_properties_end(&builder);

                    arpentry_tiles_Layer_features_push_end(&builder);
                }
                arpentry_tiles_Layer_features_end(&builder);
                arpentry_tiles_Tile_layers_push_end(&builder);
            }
        }

        /* Layer 5: poi (named point features) */
        if (poi_overlaps(bounds)) {
            const poi_point *pp = poi_get_points();
            int np = poi_count();

            arpentry_tiles_Tile_layers_push_start(&builder);
            arpentry_tiles_Layer_name_create_str(&builder, "poi");

            arpentry_tiles_Layer_features_start(&builder);
            for (int pi = 0; pi < np; pi++) {
                /* Only emit POI if it falls within this tile's proper area
                 * (not buffer zone) so each POI appears in exactly one tile. */
                if (pp[pi].lon < bounds.west || pp[pi].lon >= bounds.east ||
                    pp[pi].lat < bounds.south || pp[pi].lat >= bounds.north)
                    continue;

                uint16_t px = arpt_quantize_lon(pp[pi].lon, bounds);
                uint16_t py = arpt_quantize_lat(pp[pi].lat, bounds);
                int32_t pz = arpt_meters_to_mm(
                    terrain_elevation(pp[pi].lon, pp[pi].lat));

                arpentry_tiles_Layer_features_push_start(&builder);
                arpentry_tiles_Feature_id_add(&builder,
                                              (uint64_t)(400000 + pi));

                arpentry_tiles_PointGeometry_ref_t poi_ref;
                arpentry_tiles_PointGeometry_start(&builder);
                arpentry_tiles_PointGeometry_x_create(&builder, &px, 1);
                arpentry_tiles_PointGeometry_y_create(&builder, &py, 1);
                arpentry_tiles_PointGeometry_z_create(&builder, &pz, 1);
                poi_ref = arpentry_tiles_PointGeometry_end(&builder);
                arpentry_tiles_Feature_geometry_PointGeometry_add(&builder,
                                                                   poi_ref);

                arpentry_tiles_Feature_properties_start(&builder);
                arpentry_tiles_Property_t pprop = {0};
                /* class = "poi" */
                pprop.key = 0;
                pprop.value = POI_VAL_POI;
                arpentry_tiles_Feature_properties_push(&builder, &pprop);
                /* name = poi name string */
                pprop.key = POI_KEY_NAME;
                pprop.value = (uint32_t)(POI_VAL_NAME_BASE + pi);
                arpentry_tiles_Feature_properties_push(&builder, &pprop);
                arpentry_tiles_Feature_properties_end(&builder);

                arpentry_tiles_Layer_features_push_end(&builder);
            }
            arpentry_tiles_Layer_features_end(&builder);
            arpentry_tiles_Tile_layers_push_end(&builder);
        }
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
                              patches, patch_count, bounds, &fb_size);

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
