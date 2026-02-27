#include "unity.h"
#include "tile_decode.h"
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

/* Tests */

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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_basic_extraction);
    RUN_TEST(test_normals_present);
    RUN_TEST(test_normals_absent);
    RUN_TEST(test_no_terrain_layer);
    RUN_TEST(test_null_input);
    return UNITY_END();
}
