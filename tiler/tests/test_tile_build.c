#include "unity.h"
#include "tile_build.h"
#include "tile.h"          /* arpt_decode from common */
#include "tile_reader.h"   /* generated flatcc reader */

#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void test_create_free(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);
    arpt_tile_builder_free(tb);
}

static void test_single_point(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    /* A point at the center of the tile */
    arpt_geom g = {0};
    g.type = 1;
    double x = 6.5, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    arpt_feature feat = {0};
    feat.layer = 0;
    feat.geom = &g;

    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &feat));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_TRUE(out_size > 0);

    /* Decompress and verify FlatBuffer */
    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    /* Parse with generated reader */
    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);
    TEST_ASSERT_NOT_NULL(tile);
    TEST_ASSERT_EQUAL_UINT16(1, arpentry_tiles_Tile_version(tile));

    /* Should have one layer with one feature */
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    TEST_ASSERT_NOT_NULL(layers);
    TEST_ASSERT_EQUAL_INT(1, (int)arpentry_tiles_Layer_vec_len(layers));

    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    TEST_ASSERT_EQUAL_INT(1, (int)arpentry_tiles_Feature_vec_len(features));

    /* Check geometry is PointGeometry */
    arpentry_tiles_Feature_table_t f = arpentry_tiles_Feature_vec_at(features, 0);
    TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_PointGeometry,
                          arpentry_tiles_Feature_geometry_type(f));

    /* Verify coordinates are quantized near center */
    arpentry_tiles_PointGeometry_table_t pg =
        (arpentry_tiles_PointGeometry_table_t)arpentry_tiles_Feature_geometry(f);
    flatbuffers_uint16_vec_t xs = arpentry_tiles_PointGeometry_x(pg);
    flatbuffers_uint16_vec_t ys = arpentry_tiles_PointGeometry_y(pg);
    TEST_ASSERT_EQUAL_INT(1, (int)flatbuffers_uint16_vec_len(xs));

    /* Center should quantize to ~32768 (TILE_BUFFER + TILE_EXTENT/2) */
    uint16_t qx = flatbuffers_uint16_vec_at(xs, 0);
    uint16_t qy = flatbuffers_uint16_vec_at(ys, 0);
    TEST_ASSERT_INT_WITHIN(100, 32768, (int)qx);
    TEST_ASSERT_INT_WITHIN(100, 32768, (int)qy);

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

static void test_with_properties(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    arpt_geom g = {0};
    g.type = 1;
    double x = 6.5, y = 46.5;
    g.x = &x;
    g.y = &y;
    g.n_coords = 1;

    const char *keys[] = {"class", "name"};
    const char *vals[] = {"building", "Town Hall"};

    arpt_feature feat = {0};
    feat.layer = 0;
    feat.geom = &g;
    feat.prop_keys = keys;
    feat.prop_vals = vals;
    feat.n_props = 2;

    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &feat));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);

    /* Decode and check dictionaries */
    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);
    flatbuffers_string_vec_t tile_keys = arpentry_tiles_Tile_keys(tile);
    TEST_ASSERT_EQUAL_INT(2, (int)flatbuffers_string_vec_len(tile_keys));
    TEST_ASSERT_EQUAL_STRING("class", flatbuffers_string_vec_at(tile_keys, 0));
    TEST_ASSERT_EQUAL_STRING("name", flatbuffers_string_vec_at(tile_keys, 1));

    arpentry_tiles_Value_vec_t tile_vals = arpentry_tiles_Tile_values(tile);
    TEST_ASSERT_EQUAL_INT(2, (int)arpentry_tiles_Value_vec_len(tile_vals));

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

static void test_line_geometry(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    arpt_geom g = {0};
    g.type = 2;
    double x[] = {6.2, 6.5, 6.8};
    double y[] = {46.2, 46.5, 46.8};
    g.x = x;
    g.y = y;
    g.n_coords = 3;

    arpt_feature feat = {0};
    feat.geom = &g;
    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &feat));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);

    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    arpentry_tiles_Feature_table_t f = arpentry_tiles_Feature_vec_at(features, 0);

    TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_LineGeometry,
                          arpentry_tiles_Feature_geometry_type(f));

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

static void test_polygon_geometry(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    arpt_geom g = {0};
    g.type = 3;
    double x[] = {6.2, 6.8, 6.8, 6.2, 6.2};
    double y[] = {46.2, 46.2, 46.8, 46.8, 46.2};
    g.x = x;
    g.y = y;
    g.n_coords = 5;
    uint32_t offsets[] = {0, 5};
    g.offsets = offsets;
    g.n_offsets = 2;

    arpt_feature feat = {0};
    feat.geom = &g;
    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &feat));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);

    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    arpentry_tiles_Layer_table_t layer = arpentry_tiles_Layer_vec_at(layers, 0);
    arpentry_tiles_Feature_vec_t features = arpentry_tiles_Layer_features(layer);
    arpentry_tiles_Feature_table_t f = arpentry_tiles_Feature_vec_at(features, 0);

    TEST_ASSERT_EQUAL_INT(arpentry_tiles_Geometry_PolygonGeometry,
                          arpentry_tiles_Feature_geometry_type(f));

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

static void test_property_dedup(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    /* Two features sharing the same property key "class" */
    arpt_geom g1 = {0};
    g1.type = 1;
    double x1 = 6.3, y1 = 46.3;
    g1.x = &x1; g1.y = &y1; g1.n_coords = 1;

    arpt_geom g2 = {0};
    g2.type = 1;
    double x2 = 6.7, y2 = 46.7;
    g2.x = &x2; g2.y = &y2; g2.n_coords = 1;

    const char *keys1[] = {"class"};
    const char *vals1[] = {"building"};
    const char *keys2[] = {"class"};
    const char *vals2[] = {"road"};

    arpt_feature f1 = {.geom = &g1, .prop_keys = keys1, .prop_vals = vals1, .n_props = 1};
    arpt_feature f2 = {.geom = &g2, .prop_keys = keys2, .prop_vals = vals2, .n_props = 1};

    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &f1));
    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &f2));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);

    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);

    /* "class" should appear only once in keys */
    flatbuffers_string_vec_t tile_keys = arpentry_tiles_Tile_keys(tile);
    TEST_ASSERT_EQUAL_INT(1, (int)flatbuffers_string_vec_len(tile_keys));

    /* "building" and "road" are different values: 2 entries */
    arpentry_tiles_Value_vec_t tile_vals = arpentry_tiles_Tile_values(tile);
    TEST_ASSERT_EQUAL_INT(2, (int)arpentry_tiles_Value_vec_len(tile_vals));

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

static void test_multi_layer(void) {
    arpt_bounds b = {6.0, 46.0, 7.0, 47.0};
    arpt_tile_builder *tb = arpt_tile_builder_create(b);
    TEST_ASSERT_NOT_NULL(tb);

    arpt_geom g1 = {0};
    g1.type = 1;
    double x1 = 6.3, y1 = 46.3;
    g1.x = &x1; g1.y = &y1; g1.n_coords = 1;

    arpt_geom g2 = {0};
    g2.type = 1;
    double x2 = 6.7, y2 = 46.7;
    g2.x = &x2; g2.y = &y2; g2.n_coords = 1;

    arpt_feature f1 = {.layer = 0, .geom = &g1};
    arpt_feature f2 = {.layer = 2, .geom = &g2};

    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &f1));
    TEST_ASSERT_TRUE(arpt_tile_builder_add_feature(tb, &f2));

    size_t out_size;
    void *out = arpt_tile_builder_finish(tb, &out_size);
    TEST_ASSERT_NOT_NULL(out);

    uint8_t *decoded;
    size_t decoded_size;
    TEST_ASSERT_TRUE(arpt_decode(out, out_size, &decoded, &decoded_size));

    arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(decoded);
    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    /* Layers 0 and 2 (layer 1 is skipped because empty) */
    TEST_ASSERT_EQUAL_INT(2, (int)arpentry_tiles_Layer_vec_len(layers));

    free(decoded);
    free(out);
    arpt_tile_builder_free(tb);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_create_free);
    RUN_TEST(test_single_point);
    RUN_TEST(test_with_properties);
    RUN_TEST(test_line_geometry);
    RUN_TEST(test_polygon_geometry);
    RUN_TEST(test_property_dedup);
    RUN_TEST(test_multi_layer);
    return UNITY_END();
}
