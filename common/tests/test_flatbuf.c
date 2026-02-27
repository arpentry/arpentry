#include "tile_builder.h"
#include "tile_reader.h"
#include "tile_verifier.h"
#include "unity.h"
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* Helpers */

/**
 * Build a minimal valid Tile with one "pois" layer containing a single
 * PointGeometry feature with properties. Returns the FlatBuffer via buf and size.
 */
static void build_test_tile(void **buf, size_t *size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    /* Property key dictionary */
    arpentry_tiles_Tile_keys_start(&builder);
    arpentry_tiles_Tile_keys_push_create_str(&builder, "class");
    arpentry_tiles_Tile_keys_push_create_str(&builder, "name");
    arpentry_tiles_Tile_keys_end(&builder);

    /* Property value dictionary */
    arpentry_tiles_Tile_values_start(&builder);

    arpentry_tiles_Tile_values_push_start(&builder);
    arpentry_tiles_Value_type_add(&builder, arpentry_tiles_PropertyValueType_String);
    arpentry_tiles_Value_string_value_create_str(&builder, "cafe");
    arpentry_tiles_Tile_values_push_end(&builder);

    arpentry_tiles_Tile_values_push_start(&builder);
    arpentry_tiles_Value_type_add(&builder, arpentry_tiles_PropertyValueType_String);
    arpentry_tiles_Value_string_value_create_str(&builder, "Central Perk");
    arpentry_tiles_Tile_values_push_end(&builder);

    arpentry_tiles_Tile_values_end(&builder);

    /* Layers */
    arpentry_tiles_Tile_layers_start(&builder);

    arpentry_tiles_Tile_layers_push_start(&builder);
    arpentry_tiles_Layer_name_create_str(&builder, "pois");

    /* Features */
    arpentry_tiles_Layer_features_start(&builder);

    arpentry_tiles_Layer_features_push_start(&builder);
    arpentry_tiles_Feature_id_add(&builder, 42);

    /* PointGeometry */
    uint16_t xs[] = {32768};
    uint16_t ys[] = {32768};
    int32_t zs[] = {100000};  /* 100m in mm */

    arpentry_tiles_PointGeometry_ref_t point_ref;
    arpentry_tiles_PointGeometry_start(&builder);
    arpentry_tiles_PointGeometry_x_create(&builder, xs, 1);
    arpentry_tiles_PointGeometry_y_create(&builder, ys, 1);
    arpentry_tiles_PointGeometry_z_create(&builder, zs, 1);
    point_ref = arpentry_tiles_PointGeometry_end(&builder);

    arpentry_tiles_Feature_geometry_PointGeometry_add(&builder, point_ref);

    /* Properties: class=cafe, name=Central Perk */
    arpentry_tiles_Property_t props[2];
    props[0].key = 0; props[0].value = 0;
    props[1].key = 1; props[1].value = 1;
    arpentry_tiles_Feature_properties_create(&builder, props, 2);

    arpentry_tiles_Layer_features_push_end(&builder);
    arpentry_tiles_Layer_features_end(&builder);

    arpentry_tiles_Tile_layers_push_end(&builder);
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    *buf = flatcc_builder_finalize_buffer(&builder, size);
    flatcc_builder_clear(&builder);
}

/* Tests */

void test_file_identifier(void) {
    void *buf; size_t size;
    build_test_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    TEST_ASSERT_TRUE(flatbuffers_has_identifier(buf, "arpt"));

    free(buf);
}

void test_tile_roundtrip(void) {
    void *buf; size_t size;
    build_test_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root_with_identifier(buf, "arpt");
    TEST_ASSERT_NOT_NULL(tile);

    /* Version */
    TEST_ASSERT_EQUAL_UINT16(1, arpentry_tiles_Tile_version(tile));

    /* Keys */
    flatbuffers_string_vec_t keys = arpentry_tiles_Tile_keys(tile);
    TEST_ASSERT_EQUAL(2, flatbuffers_string_vec_len(keys));
    TEST_ASSERT_EQUAL_STRING("class", flatbuffers_string_vec_at(keys, 0));
    TEST_ASSERT_EQUAL_STRING("name", flatbuffers_string_vec_at(keys, 1));

    /* Values */
    arpentry_tiles_Value_vec_t values = arpentry_tiles_Tile_values(tile);
    TEST_ASSERT_EQUAL(2, arpentry_tiles_Value_vec_len(values));

    arpentry_tiles_Value_table_t v0 = arpentry_tiles_Value_vec_at(values, 0);
    TEST_ASSERT_EQUAL(arpentry_tiles_PropertyValueType_String, arpentry_tiles_Value_type(v0));
    TEST_ASSERT_EQUAL_STRING("cafe", arpentry_tiles_Value_string_value(v0));

    arpentry_tiles_Value_table_t v1 = arpentry_tiles_Value_vec_at(values, 1);
    TEST_ASSERT_EQUAL_STRING("Central Perk", arpentry_tiles_Value_string_value(v1));

    /* Layers */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    TEST_ASSERT_EQUAL(1, arpentry_tiles_Layer_vec_len(layers));

    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    TEST_ASSERT_EQUAL_STRING("pois", arpentry_tiles_Layer_name(layer));

    free(buf);
}

void test_point_geometry(void) {
    void *buf; size_t size;
    build_test_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root_with_identifier(buf, "arpt");
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    TEST_ASSERT_EQUAL(1, arpentry_tiles_Feature_vec_len(features));

    arpentry_tiles_Feature_table_t feat = arpentry_tiles_Feature_vec_at(features, 0);
    TEST_ASSERT_EQUAL_UINT64(42, arpentry_tiles_Feature_id(feat));

    /* Union discriminator */
    TEST_ASSERT_EQUAL(arpentry_tiles_Geometry_PointGeometry,
                      arpentry_tiles_Feature_geometry_type(feat));

    arpentry_tiles_PointGeometry_table_t geom =
        (arpentry_tiles_PointGeometry_table_t)arpentry_tiles_Feature_geometry(feat);
    TEST_ASSERT_NOT_NULL(geom);

    flatbuffers_uint16_vec_t x = arpentry_tiles_PointGeometry_x(geom);
    flatbuffers_uint16_vec_t y = arpentry_tiles_PointGeometry_y(geom);
    flatbuffers_int32_vec_t z = arpentry_tiles_PointGeometry_z(geom);
    TEST_ASSERT_EQUAL(1, flatbuffers_uint16_vec_len(x));
    TEST_ASSERT_EQUAL_UINT16(32768, flatbuffers_uint16_vec_at(x, 0));
    TEST_ASSERT_EQUAL_UINT16(32768, flatbuffers_uint16_vec_at(y, 0));
    TEST_ASSERT_EQUAL_INT32(100000, flatbuffers_int32_vec_at(z, 0));

    free(buf);
}

void test_property_dictionary(void) {
    void *buf; size_t size;
    build_test_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root_with_identifier(buf, "arpt");
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    arpentry_tiles_Feature_table_t feat = arpentry_tiles_Feature_vec_at(features, 0);

    arpentry_tiles_Property_vec_t props = arpentry_tiles_Feature_properties(feat);
    TEST_ASSERT_EQUAL(2, arpentry_tiles_Property_vec_len(props));

    /* Dereference property 0: key=0 ("class"), value=0 ("cafe") */
    TEST_ASSERT_EQUAL_UINT32(0, arpentry_tiles_Property_key(arpentry_tiles_Property_vec_at(props, 0)));
    TEST_ASSERT_EQUAL_UINT32(0, arpentry_tiles_Property_value(arpentry_tiles_Property_vec_at(props, 0)));

    /* Dereference property 1: key=1 ("name"), value=1 ("Central Perk") */
    TEST_ASSERT_EQUAL_UINT32(1, arpentry_tiles_Property_key(arpentry_tiles_Property_vec_at(props, 1)));
    TEST_ASSERT_EQUAL_UINT32(1, arpentry_tiles_Property_value(arpentry_tiles_Property_vec_at(props, 1)));

    free(buf);
}

void test_verifier(void) {
    void *buf; size_t size;
    build_test_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    int ret = arpentry_tiles_Tile_verify_as_root_with_identifier(buf, size, "arpt");
    TEST_ASSERT_EQUAL_INT(0, ret);

    free(buf);
}

void test_mesh_geometry(void) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);

    arpentry_tiles_Tile_layers_start(&builder);
    arpentry_tiles_Tile_layers_push_start(&builder);
    arpentry_tiles_Layer_name_create_str(&builder, "terrain");

    arpentry_tiles_Layer_features_start(&builder);
    arpentry_tiles_Layer_features_push_start(&builder);

    /* Build a simple 1-triangle mesh */
    uint16_t xs[] = {16384, 49151, 32768};
    uint16_t ys[] = {16384, 16384, 49151};
    int32_t zs[] = {0, 0, 1000000};
    uint32_t indices[] = {0, 1, 2};

    arpentry_tiles_MeshGeometry_ref_t mesh_ref;
    arpentry_tiles_MeshGeometry_start(&builder);
    arpentry_tiles_MeshGeometry_x_create(&builder, xs, 3);
    arpentry_tiles_MeshGeometry_y_create(&builder, ys, 3);
    arpentry_tiles_MeshGeometry_z_create(&builder, zs, 3);
    arpentry_tiles_MeshGeometry_indices_create(&builder, indices, 3);

    /* One Part with inline material */
    arpentry_tiles_MeshGeometry_parts_start(&builder);
    arpentry_tiles_Part_t part;
    part.first_index = 0;
    part.index_count = 3;
    part.color.r = 128;
    part.color.g = 64;
    part.color.b = 32;
    part.color.a = 255;
    part.roughness = 200;
    part.metalness = 50;
    arpentry_tiles_MeshGeometry_parts_push(&builder, &part);
    arpentry_tiles_MeshGeometry_parts_end(&builder);

    mesh_ref = arpentry_tiles_MeshGeometry_end(&builder);
    arpentry_tiles_Feature_geometry_MeshGeometry_add(&builder, mesh_ref);

    arpentry_tiles_Layer_features_push_end(&builder);
    arpentry_tiles_Layer_features_end(&builder);

    arpentry_tiles_Tile_layers_push_end(&builder);
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    size_t size;
    void *buf = flatcc_builder_finalize_buffer(&builder, &size);
    flatcc_builder_clear(&builder);
    TEST_ASSERT_NOT_NULL(buf);

    /* Read back */
    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root_with_identifier(buf, "arpt");
    TEST_ASSERT_NOT_NULL(tile);

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    arpentry_tiles_Feature_table_t feat = arpentry_tiles_Feature_vec_at(features, 0);

    TEST_ASSERT_EQUAL(arpentry_tiles_Geometry_MeshGeometry,
                      arpentry_tiles_Feature_geometry_type(feat));

    arpentry_tiles_MeshGeometry_table_t mesh =
        (arpentry_tiles_MeshGeometry_table_t)arpentry_tiles_Feature_geometry(feat);

    flatbuffers_uint32_vec_t idx = arpentry_tiles_MeshGeometry_indices(mesh);
    TEST_ASSERT_EQUAL(3, flatbuffers_uint32_vec_len(idx));
    TEST_ASSERT_EQUAL_UINT32(2, flatbuffers_uint32_vec_at(idx, 2));

    /* Verify Part struct */
    arpentry_tiles_Part_vec_t parts = arpentry_tiles_MeshGeometry_parts(mesh);
    TEST_ASSERT_EQUAL(1, arpentry_tiles_Part_vec_len(parts));
    const arpentry_tiles_Part_t *p = arpentry_tiles_Part_vec_at(parts, 0);
    TEST_ASSERT_EQUAL_UINT32(0, p->first_index);
    TEST_ASSERT_EQUAL_UINT32(3, p->index_count);
    TEST_ASSERT_EQUAL_UINT8(128, p->color.r);
    TEST_ASSERT_EQUAL_UINT8(64, p->color.g);
    TEST_ASSERT_EQUAL_UINT8(32, p->color.b);
    TEST_ASSERT_EQUAL_UINT8(255, p->color.a);
    TEST_ASSERT_EQUAL_UINT8(200, p->roughness);
    TEST_ASSERT_EQUAL_UINT8(50, p->metalness);

    free(buf);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_file_identifier);
    RUN_TEST(test_tile_roundtrip);
    RUN_TEST(test_point_geometry);
    RUN_TEST(test_property_dictionary);
    RUN_TEST(test_verifier);
    RUN_TEST(test_mesh_geometry);
    return UNITY_END();
}
