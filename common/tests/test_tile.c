#include "arpentry_tiles_builder.h"
#include "arpentry_tiles_reader.h"
#include "arpentry/tile_io.h"
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

/* ── Compress / Decompress roundtrip ────────────────────────────────────── */

void test_compress_decompress_roundtrip(void) {
    void *tile_buf; size_t tile_size;
    build_minimal_tile(&tile_buf, &tile_size);
    TEST_ASSERT_NOT_NULL(tile_buf);

    /* Compress */
    uint8_t *compressed = NULL;
    size_t compressed_size = 0;
    TEST_ASSERT_TRUE(arpt_compress((uint8_t *)tile_buf, tile_size,
                                    &compressed, &compressed_size, 4));
    TEST_ASSERT_NOT_NULL(compressed);
    TEST_ASSERT_TRUE(compressed_size > 0);

    /* Decompress */
    uint8_t *decompressed = NULL;
    size_t decompressed_size = 0;
    TEST_ASSERT_TRUE(arpt_decompress(compressed, compressed_size,
                                      &decompressed, &decompressed_size));
    TEST_ASSERT_NOT_NULL(decompressed);

    /* Verify roundtrip */
    TEST_ASSERT_EQUAL(tile_size, decompressed_size);
    TEST_ASSERT_EQUAL_MEMORY(tile_buf, decompressed, tile_size);

    free(tile_buf);
    free(compressed);
    free(decompressed);
}

/* ── Verify valid tile ──────────────────────────────────────────────────── */

void test_verify_valid_tile(void) {
    void *buf; size_t size;
    build_minimal_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    TEST_ASSERT_TRUE(arpt_tile_verify(buf, size));

    free(buf);
}

/* ── Reject invalid data ────────────────────────────────────────────────── */

void test_verify_rejects_garbage(void) {
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_FALSE(arpt_tile_verify(garbage, sizeof(garbage)));
}

void test_verify_rejects_null(void) {
    TEST_ASSERT_FALSE(arpt_tile_verify(NULL, 0));
}

void test_verify_rejects_too_small(void) {
    uint8_t tiny[] = {0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT_FALSE(arpt_tile_verify(tiny, sizeof(tiny)));
}

/* ── Reject wrong file identifier ───────────────────────────────────────── */

void test_verify_rejects_wrong_identifier(void) {
    /* Build a valid tile then corrupt the file identifier bytes */
    void *buf; size_t size;
    build_minimal_tile(&buf, &size);
    TEST_ASSERT_NOT_NULL(buf);

    /* FlatBuffer file identifier is at bytes 4-7 */
    memcpy((uint8_t *)buf + 4, "XXXX", 4);

    /* Verifier should reject: wrong identifier */
    TEST_ASSERT_FALSE(arpt_tile_verify(buf, size));

    free(buf);
}

/* ── Decompress rejects invalid input ───────────────────────────────────── */

void test_decompress_rejects_garbage(void) {
    uint8_t garbage[] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t *output = NULL;
    size_t output_size = 0;
    TEST_ASSERT_FALSE(arpt_decompress(garbage, sizeof(garbage),
                                       &output, &output_size));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_compress_decompress_roundtrip);
    RUN_TEST(test_verify_valid_tile);
    RUN_TEST(test_verify_rejects_garbage);
    RUN_TEST(test_verify_rejects_null);
    RUN_TEST(test_verify_rejects_too_small);
    RUN_TEST(test_verify_rejects_wrong_identifier);
    RUN_TEST(test_decompress_rejects_garbage);
    return UNITY_END();
}
