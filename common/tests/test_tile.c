#include "tile_builder.h"
#include "tile_reader.h"
#include "tile.h"
#include "unity.h"
#include <stdlib.h>
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* Build a minimal valid Tile FlatBuffer for testing */
static void build_minimal_tile(void **buf, size_t *size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tile_start_as_root(&builder);
    arpentry_tiles_Tile_version_add(&builder, 1);

    arpentry_tiles_Tile_layers_start(&builder);
    arpentry_tiles_Tile_layers_push_start(&builder);
    arpentry_tiles_Layer_name_create_str(&builder, "terrain");
    arpentry_tiles_Tile_layers_push_end(&builder);
    arpentry_tiles_Tile_layers_end(&builder);

    arpentry_tiles_Tile_end_as_root(&builder);

    *buf = flatcc_builder_finalize_buffer(&builder, size);
    flatcc_builder_clear(&builder);
}

/* ── Encode / Decode roundtrip ─────────────────────────────────────────── */

void test_encode_decode_roundtrip(void) {
    void *tile_buf; size_t tile_size;
    build_minimal_tile(&tile_buf, &tile_size);
    TEST_ASSERT_NOT_NULL(tile_buf);

    /* Encode */
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    TEST_ASSERT_TRUE(arpt_encode(tile_buf, tile_size,
                                 &encoded, &encoded_size, 4));
    TEST_ASSERT_NOT_NULL(encoded);
    TEST_ASSERT_TRUE(encoded_size > 0);

    /* Decode — returns verified FlatBuffer */
    uint8_t *decoded = NULL;
    size_t decoded_size = 0;
    TEST_ASSERT_TRUE(arpt_decode(encoded, encoded_size,
                                 &decoded, &decoded_size));
    TEST_ASSERT_NOT_NULL(decoded);

    /* Byte-for-byte match with original FlatBuffer */
    TEST_ASSERT_EQUAL(tile_size, decoded_size);
    TEST_ASSERT_EQUAL_MEMORY(tile_buf, decoded, tile_size);

    free(tile_buf);
    free(encoded);
    free(decoded);
}

/* ── Decoded buffer is usable by FlatCC reader ─────────────────────────── */

void test_decode_produces_readable_tile(void) {
    void *tile_buf; size_t tile_size;
    build_minimal_tile(&tile_buf, &tile_size);
    TEST_ASSERT_NOT_NULL(tile_buf);

    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    TEST_ASSERT_TRUE(arpt_encode(tile_buf, tile_size,
                                 &encoded, &encoded_size, 4));

    uint8_t *decoded = NULL;
    size_t decoded_size = 0;
    TEST_ASSERT_TRUE(arpt_decode(encoded, encoded_size,
                                 &decoded, &decoded_size));

    /* Use FlatCC reader directly on decoded buffer */
    arpentry_tiles_Tile_table_t tile =
        arpentry_tiles_Tile_as_root_with_identifier(decoded, "arpt");
    TEST_ASSERT_NOT_NULL(tile);
    TEST_ASSERT_EQUAL_UINT16(1, arpentry_tiles_Tile_version(tile));

    arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
    TEST_ASSERT_EQUAL(1, arpentry_tiles_Layer_vec_len(layers));
    TEST_ASSERT_EQUAL_STRING("terrain",
        arpentry_tiles_Layer_name(arpentry_tiles_Layer_vec_at(layers, 0)));

    free(tile_buf);
    free(encoded);
    free(decoded);
}

/* ── Decode rejects garbage ────────────────────────────────────────────── */

void test_decode_rejects_garbage(void) {
    uint8_t garbage[] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t *out = NULL;
    size_t out_size = 0;
    TEST_ASSERT_FALSE(arpt_decode(garbage, sizeof(garbage),
                                  &out, &out_size));
}

void test_decode_rejects_null(void) {
    uint8_t *out = NULL;
    size_t out_size = 0;
    TEST_ASSERT_FALSE(arpt_decode(NULL, 0, &out, &out_size));
}

/* ── Decode rejects valid Brotli with bad FlatBuffer ───────────────────── */

void test_decode_rejects_corrupt_flatbuffer(void) {
    /* Encode a valid tile, then corrupt the compressed payload so that
       decompression succeeds but verification fails. Easiest: compress
       arbitrary non-FlatBuffer data. */
    const char *junk = "this is not a flatbuffer";
    uint8_t *encoded = NULL;
    size_t encoded_size = 0;
    TEST_ASSERT_TRUE(arpt_encode(junk, strlen(junk),
                                 &encoded, &encoded_size, 1));

    uint8_t *out = NULL;
    size_t out_size = 0;
    TEST_ASSERT_FALSE(arpt_decode(encoded, encoded_size, &out, &out_size));

    free(encoded);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_decode_produces_readable_tile);
    RUN_TEST(test_decode_rejects_garbage);
    RUN_TEST(test_decode_rejects_null);
    RUN_TEST(test_decode_rejects_corrupt_flatbuffer);
    return UNITY_END();
}
