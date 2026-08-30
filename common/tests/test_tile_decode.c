#include "unity.h"
#include "tile/decode.h"
#include "tile/prepare.h"
#include "tile_builder.h"

#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* Helpers */

/* Build a tile with a terrain layer containing a MeshGeometry.
   If include_normals is false, omits the normals array. */
static void *build_terrain_tile(size_t *out_size, bool include_normals) {
    flatcc_builder_t b;
    flatcc_builder_init(&b);

    arpentry_tiles_Tile_start_as_root(&b);
    arpentry_tiles_Tile_version_add(&b, 1);

    arpentry_tiles_Tile_layers_start(&b);
    arpentry_tiles_Tile_layers_push_start(&b);
    arpentry_tiles_Layer_name_create_str(&b, "terrain");

    arpentry_tiles_Layer_features_start(&b);
    arpentry_tiles_Layer_features_push_start(&b);
    arpentry_tiles_Feature_id_add(&b, 1);

    uint16_t xs[] = {16384, 49151, 49151, 16384};
    uint16_t ys[] = {16384, 16384, 49151, 49151};
    int32_t zs[] = {0, 100000, 200000, 50000};
    uint32_t indices[] = {0, 1, 2, 0, 2, 3};
    int8_t normals[] = {0, 127, 0, 127, 0, 127, 0, 127};

    arpentry_tiles_MeshGeometry_start(&b);
    arpentry_tiles_MeshGeometry_x_create(&b, xs, 4);
    arpentry_tiles_MeshGeometry_y_create(&b, ys, 4);
    arpentry_tiles_MeshGeometry_z_create(&b, zs, 4);
    arpentry_tiles_MeshGeometry_indices_create(&b, indices, 6);
    if (include_normals)
        arpentry_tiles_MeshGeometry_normals_create(&b, normals, 8);

    arpentry_tiles_MeshGeometry_ref_t ref = arpentry_tiles_MeshGeometry_end(&b);
    arpentry_tiles_Feature_geometry_MeshGeometry_add(&b, ref);

    arpentry_tiles_Layer_features_push_end(&b);
    arpentry_tiles_Layer_features_end(&b);
    arpentry_tiles_Tile_layers_push_end(&b);
    arpentry_tiles_Tile_layers_end(&b);
    arpentry_tiles_Tile_end_as_root(&b);

    void *buf = flatcc_builder_finalize_buffer(&b, out_size);
    flatcc_builder_clear(&b);
    return buf;
}

/* Build a tile with only a "points" layer (no terrain). */
static void *build_points_only_tile(size_t *out_size) {
    flatcc_builder_t b;
    flatcc_builder_init(&b);

    arpentry_tiles_Tile_start_as_root(&b);
    arpentry_tiles_Tile_version_add(&b, 1);

    arpentry_tiles_Tile_layers_start(&b);
    arpentry_tiles_Tile_layers_push_start(&b);
    arpentry_tiles_Layer_name_create_str(&b, "pois");

    arpentry_tiles_Layer_features_start(&b);
    arpentry_tiles_Layer_features_push_start(&b);

    uint16_t xs[] = {32768};
    uint16_t ys[] = {32768};
    int32_t zs[] = {0};

    arpentry_tiles_PointGeometry_start(&b);
    arpentry_tiles_PointGeometry_x_create(&b, xs, 1);
    arpentry_tiles_PointGeometry_y_create(&b, ys, 1);
    arpentry_tiles_PointGeometry_z_create(&b, zs, 1);
    arpentry_tiles_PointGeometry_ref_t ref =
        arpentry_tiles_PointGeometry_end(&b);
    arpentry_tiles_Feature_geometry_PointGeometry_add(&b, ref);

    arpentry_tiles_Layer_features_push_end(&b);
    arpentry_tiles_Layer_features_end(&b);
    arpentry_tiles_Tile_layers_push_end(&b);
    arpentry_tiles_Tile_layers_end(&b);
    arpentry_tiles_Tile_end_as_root(&b);

    void *buf = flatcc_builder_finalize_buffer(&b, out_size);
    flatcc_builder_clear(&b);
    return buf;
}

/* Build a tile with a "transportation" layer holding two LineGeometry
   features: the first carries a "name" property, the second does not.
   Tile.keys = ["class", "name"]; Tile.values = ["primary", "Grand-Rue"]. */
static void *build_line_label_tile(size_t *out_size) {
    flatcc_builder_t b;
    flatcc_builder_init(&b);

    arpentry_tiles_Tile_start_as_root(&b);
    arpentry_tiles_Tile_version_add(&b, 1);

    arpentry_tiles_Tile_layers_start(&b);
    arpentry_tiles_Tile_layers_push_start(&b);
    arpentry_tiles_Layer_name_create_str(&b, "transportation");

    arpentry_tiles_Layer_features_start(&b);

    /* Feature 0: a named three-point polyline. */
    {
        arpentry_tiles_Layer_features_push_start(&b);
        arpentry_tiles_Feature_id_add(&b, 1);
        uint16_t xs[] = {20000, 30000, 40000};
        uint16_t ys[] = {25000, 25000, 25000};
        arpentry_tiles_LineGeometry_start(&b);
        arpentry_tiles_LineGeometry_x_create(&b, xs, 3);
        arpentry_tiles_LineGeometry_y_create(&b, ys, 3);
        arpentry_tiles_Feature_geometry_LineGeometry_add(
            &b, arpentry_tiles_LineGeometry_end(&b));
        /* properties: name(key 1) -> "Grand-Rue"(value 1) */
        arpentry_tiles_Feature_properties_start(&b);
        arpentry_tiles_Feature_properties_push_create(&b, 1, 1);
        arpentry_tiles_Feature_properties_end(&b);
        arpentry_tiles_Layer_features_push_end(&b);
    }

    /* Feature 1: an unnamed polyline (only a class property). */
    {
        arpentry_tiles_Layer_features_push_start(&b);
        arpentry_tiles_Feature_id_add(&b, 2);
        uint16_t xs[] = {20000, 40000};
        uint16_t ys[] = {30000, 30000};
        arpentry_tiles_LineGeometry_start(&b);
        arpentry_tiles_LineGeometry_x_create(&b, xs, 2);
        arpentry_tiles_LineGeometry_y_create(&b, ys, 2);
        arpentry_tiles_Feature_geometry_LineGeometry_add(
            &b, arpentry_tiles_LineGeometry_end(&b));
        arpentry_tiles_Feature_properties_start(&b);
        arpentry_tiles_Feature_properties_push_create(&b, 0, 0);
        arpentry_tiles_Feature_properties_end(&b);
        arpentry_tiles_Layer_features_push_end(&b);
    }

    arpentry_tiles_Layer_features_end(&b);
    arpentry_tiles_Tile_layers_push_end(&b);
    arpentry_tiles_Tile_layers_end(&b);

    /* Key dictionary: class(0), name(1). */
    arpentry_tiles_Tile_keys_start(&b);
    arpentry_tiles_Tile_keys_push_create_str(&b, "class");
    arpentry_tiles_Tile_keys_push_create_str(&b, "name");
    arpentry_tiles_Tile_keys_end(&b);

    /* Value dictionary: "primary"(0), "Grand-Rue"(1). */
    arpentry_tiles_Tile_values_start(&b);
    arpentry_tiles_Tile_values_push_start(&b);
    arpentry_tiles_Value_type_add(&b, arpentry_tiles_PropertyValueType_String);
    arpentry_tiles_Value_string_value_create_str(&b, "primary");
    arpentry_tiles_Tile_values_push_end(&b);
    arpentry_tiles_Tile_values_push_start(&b);
    arpentry_tiles_Value_type_add(&b, arpentry_tiles_PropertyValueType_String);
    arpentry_tiles_Value_string_value_create_str(&b, "Grand-Rue");
    arpentry_tiles_Tile_values_push_end(&b);
    arpentry_tiles_Tile_values_end(&b);

    arpentry_tiles_Tile_end_as_root(&b);

    void *buf = flatcc_builder_finalize_buffer(&b, out_size);
    flatcc_builder_clear(&b);
    return buf;
}

/* A transportation tile of three level-0 surface meshes, emitted in the
   order [walk sheet 0, road sheet 1, road sheet 0] — the order the decoder
   must NOT keep. Each is one triangle whose x[0] names it: 100, 200, 300.
   Tile.keys = ["class", "level", "sheet"]; values = ["walk_surface",
   "road_surface", 0, 1]. */
static void *build_surface_tile(size_t *out_size) {
    flatcc_builder_t b;
    flatcc_builder_init(&b);
    arpentry_tiles_Tile_start_as_root(&b);
    arpentry_tiles_Tile_version_add(&b, 1);
    arpentry_tiles_Tile_layers_start(&b);
    arpentry_tiles_Tile_layers_push_start(&b);
    arpentry_tiles_Layer_name_create_str(&b, "transportation");
    arpentry_tiles_Layer_features_start(&b);
    const uint16_t tag[3] = {100, 200, 300};
    const uint32_t cls[3] = {0, 1, 1};   /* walk, road, road */
    const uint32_t sheet[3] = {2, 3, 2}; /* 0, 1, 0 */
    for (int f = 0; f < 3; f++) {
        arpentry_tiles_Layer_features_push_start(&b);
        arpentry_tiles_Feature_id_add(&b, (uint64_t)f + 1);
        uint16_t xs[] = {tag[f], 30000, 30000};
        uint16_t ys[] = {20000, 20000, 30000};
        int32_t zs[] = {0, 0, 0};
        uint32_t idx[] = {0, 1, 2};
        arpentry_tiles_MeshGeometry_start(&b);
        arpentry_tiles_MeshGeometry_x_create(&b, xs, 3);
        arpentry_tiles_MeshGeometry_y_create(&b, ys, 3);
        arpentry_tiles_MeshGeometry_z_create(&b, zs, 3);
        arpentry_tiles_MeshGeometry_indices_create(&b, idx, 3);
        arpentry_tiles_Feature_geometry_MeshGeometry_add(
            &b, arpentry_tiles_MeshGeometry_end(&b));
        arpentry_tiles_Feature_properties_start(&b);
        arpentry_tiles_Feature_properties_push_create(&b, 0, cls[f]);
        arpentry_tiles_Feature_properties_push_create(&b, 1, 2); /* level 0 */
        arpentry_tiles_Feature_properties_push_create(&b, 2, sheet[f]);
        arpentry_tiles_Feature_properties_end(&b);
        arpentry_tiles_Layer_features_push_end(&b);
    }
    arpentry_tiles_Layer_features_end(&b);
    arpentry_tiles_Tile_layers_push_end(&b);
    arpentry_tiles_Tile_layers_end(&b);
    arpentry_tiles_Tile_keys_start(&b);
    arpentry_tiles_Tile_keys_push_create_str(&b, "class");
    arpentry_tiles_Tile_keys_push_create_str(&b, "level");
    arpentry_tiles_Tile_keys_push_create_str(&b, "sheet");
    arpentry_tiles_Tile_keys_end(&b);
    arpentry_tiles_Tile_values_start(&b);
    const char *strs[2] = {"walk_surface", "road_surface"};
    for (int i = 0; i < 2; i++) {
        arpentry_tiles_Tile_values_push_start(&b);
        arpentry_tiles_Value_type_add(&b, arpentry_tiles_PropertyValueType_String);
        arpentry_tiles_Value_string_value_create_str(&b, strs[i]);
        arpentry_tiles_Tile_values_push_end(&b);
    }
    for (int i = 0; i < 2; i++) {
        arpentry_tiles_Tile_values_push_start(&b);
        arpentry_tiles_Value_type_add(&b, arpentry_tiles_PropertyValueType_Int);
        arpentry_tiles_Value_int_value_add(&b, i);
        arpentry_tiles_Tile_values_push_end(&b);
    }
    arpentry_tiles_Tile_values_end(&b);
    arpentry_tiles_Tile_end_as_root(&b);
    void *buf = flatcc_builder_finalize_buffer(&b, out_size);
    flatcc_builder_clear(&b);
    return buf;
}

/* Tests */

void test_surfaces_concatenate_in_stacking_priority(void) {
    /* (level, sheet, material) -> clamp(level+1)<<4 | sheet<<2 | material:
       road sheet 0 = 16, walk sheet 0 = 17, road sheet 1 = 20. Ascending, so
       the concatenated mesh runs road s0, walk s0, road s1 — the emitted
       order was walk s0, road s1, road s0. */
    size_t size;
    void *buf = build_surface_tile(&size);
    arpt_building_prim prim;
    TEST_ASSERT_TRUE(arpt_decode_bridge_mesh(buf, size, "transportation",
                                             NULL, 0, NULL, &prim));
    TEST_ASSERT_EQUAL_size_t(9, prim.vertex_count);
    TEST_ASSERT_NOT_NULL(prim.priority);
    TEST_ASSERT_EQUAL_UINT16(300, prim.xy[0 * 2]);
    TEST_ASSERT_EQUAL_INT8(16, prim.priority[0]);
    TEST_ASSERT_EQUAL_UINT16(100, prim.xy[3 * 2]);
    TEST_ASSERT_EQUAL_INT8(17, prim.priority[3]);
    TEST_ASSERT_EQUAL_UINT16(200, prim.xy[6 * 2]);
    TEST_ASSERT_EQUAL_INT8(20, prim.priority[6]);
    /* Indices were re-based to the new order. */
    TEST_ASSERT_EQUAL_UINT32(3, prim.indices[3]);
    TEST_ASSERT_EQUAL_UINT32(6, prim.indices[6]);
    free(prim.xy); free(prim.z); free(prim.normals); free(prim.indices);
    free(prim.color); free(prim.edge_across); free(prim.priority);
    free(buf);
}

void test_basic_extraction(void) {
    size_t size;
    void *buf = build_terrain_tile(&size, true);
    TEST_ASSERT_NOT_NULL(buf);

    arpt_terrain_mesh mesh = {0};
    TEST_ASSERT_TRUE(arpt_decode_terrain(buf, size, &mesh));

    TEST_ASSERT_EQUAL(4, mesh.vertex_count);
    TEST_ASSERT_EQUAL(6, mesh.index_count);
    TEST_ASSERT_NOT_NULL(mesh.x);
    TEST_ASSERT_NOT_NULL(mesh.y);
    TEST_ASSERT_NOT_NULL(mesh.z);
    TEST_ASSERT_NOT_NULL(mesh.indices);

    /* Verify values */
    TEST_ASSERT_EQUAL_UINT16(16384, mesh.x[0]);
    TEST_ASSERT_EQUAL_UINT16(49151, mesh.x[1]);
    TEST_ASSERT_EQUAL_INT32(0, mesh.z[0]);
    TEST_ASSERT_EQUAL_INT32(200000, mesh.z[2]);
    TEST_ASSERT_EQUAL_UINT32(0, mesh.indices[0]);
    TEST_ASSERT_EQUAL_UINT32(2, mesh.indices[2]);

    free(buf);
}

void test_normals_present(void) {
    size_t size;
    void *buf = build_terrain_tile(&size, true);
    arpt_terrain_mesh mesh = {0};
    arpt_decode_terrain(buf, size, &mesh);
    TEST_ASSERT_NOT_NULL(mesh.normals);
    TEST_ASSERT_EQUAL_INT8(0, mesh.normals[0]);
    TEST_ASSERT_EQUAL_INT8(127, mesh.normals[1]);
    free(buf);
}

void test_normals_absent(void) {
    size_t size;
    void *buf = build_terrain_tile(&size, false);
    arpt_terrain_mesh mesh = {0};
    arpt_decode_terrain(buf, size, &mesh);
    TEST_ASSERT_NULL(mesh.normals);
    /* Other arrays should still be valid */
    TEST_ASSERT_EQUAL(4, mesh.vertex_count);
    free(buf);
}

void test_no_terrain_layer(void) {
    size_t size;
    void *buf = build_points_only_tile(&size);
    arpt_terrain_mesh mesh = {0};
    TEST_ASSERT_FALSE(arpt_decode_terrain(buf, size, &mesh));
    free(buf);
}

void test_null_input(void) {
    arpt_terrain_mesh mesh = {0};
    TEST_ASSERT_FALSE(arpt_decode_terrain(NULL, 0, &mesh));
    TEST_ASSERT_FALSE(arpt_decode_terrain(NULL, 100, &mesh));
}

void test_line_labels_named_only(void) {
    size_t size;
    void *buf = build_line_label_tile(&size);
    TEST_ASSERT_NOT_NULL(buf);

    arpt_line_label_data data = {0};
    TEST_ASSERT_TRUE(
        arpt_decode_line_labels(buf, size, "transportation", &data));

    /* Only the named feature is kept; the nameless one is skipped. */
    TEST_ASSERT_EQUAL(1, data.count);
    TEST_ASSERT_EQUAL_STRING("Grand-Rue", data.features[0].name);
    TEST_ASSERT_EQUAL(3, data.features[0].vertex_count);
    TEST_ASSERT_EQUAL_UINT16(20000, data.features[0].x[0]);
    TEST_ASSERT_EQUAL_UINT16(40000, data.features[0].x[2]);

    arpt_line_label_data_free(&data);
    TEST_ASSERT_NULL(data.features);
    TEST_ASSERT_EQUAL(0, data.count);
    free(buf);
}

void test_line_labels_missing_layer(void) {
    size_t size;
    void *buf = build_line_label_tile(&size);
    arpt_line_label_data data = {0};
    /* Absent layer is not an error: returns true with an empty result. */
    TEST_ASSERT_TRUE(arpt_decode_line_labels(buf, size, "nope", &data));
    TEST_ASSERT_EQUAL(0, data.count);
    arpt_line_label_data_free(&data);
    free(buf);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_surfaces_concatenate_in_stacking_priority);
    RUN_TEST(test_basic_extraction);
    RUN_TEST(test_normals_present);
    RUN_TEST(test_normals_absent);
    RUN_TEST(test_no_terrain_layer);
    RUN_TEST(test_null_input);
    RUN_TEST(test_line_labels_named_only);
    RUN_TEST(test_line_labels_missing_layer);
    return UNITY_END();
}
