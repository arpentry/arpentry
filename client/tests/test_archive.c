/* Holds the C archive reader to the format the Rust tiler writes.
 *
 * The client reads `.arpa` files the tiler produced, so two things must agree
 * across the language boundary: the Hilbert tile id that keys the directory,
 * and the header/entry offsets that locate a blob. Both are asserted here
 * against a table shared with `server/src/hilbert.rs`
 * (`cross_language_tests::tile_ids_match_the_c_client_table`) and against an
 * archive built byte by byte to the layout in `server/src/archive.rs`. */

#include "unity.h"
#include "tile/archive.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define HEADER_SIZE    128
#define DIR_ENTRY_SIZE 40

static char g_path[256];

void setUp(void) {}
void tearDown(void) {}

/* The same table as the Rust test. Change one side and the other goes red. */
static void test_tile_id_matches_the_tiler(void) {
    struct { int z, x, y; uint64_t id; } table[] = {
        {0, 0, 0, UINT64_C(0)},
        {1, 0, 0, UINT64_C(4398046511104)},
        {1, 1, 0, UINT64_C(4398046511107)},
        {1, 0, 1, UINT64_C(4398046511105)},
        {1, 1, 1, UINT64_C(4398046511106)},
        {12, 2125, 3102, UINT64_C(52776567574951)},
        {15, 17013, 24837, UINT64_C(65971302143368)},
        {16, 34026, 49674, UINT64_C(70371162084898)},
    };
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        TEST_ASSERT_EQUAL_UINT64(table[i].id,
            arpt_archive_tile_id(table[i].z, table[i].x, table[i].y));
    }
}

static void wr_u32(uint8_t *d, size_t o, uint32_t v) { memcpy(d + o, &v, 4); }
static void wr_u64(uint8_t *d, size_t o, uint64_t v) { memcpy(d + o, &v, 8); }

/* Write a two-tile archive to g_path, laid out exactly as ArchiveWriter does.
   Entries must be sorted by tile id, which is what the reader binary-searches. */
static bool write_archive(const char *blob_a, const char *blob_b) {
    struct { int z, x, y; const char *blob; } tiles[2] = {
        {1, 0, 0, blob_a},   /* id 4398046511104 */
        {1, 1, 0, blob_b},   /* id 4398046511107 */
    };
    const char *meta = "metadata-blob";

    size_t len_a = strlen(blob_a), len_b = strlen(blob_b);
    uint64_t off_a = HEADER_SIZE, off_b = off_a + len_a;
    uint64_t dir_off = off_b + len_b;
    uint64_t meta_off = dir_off + 2 * DIR_ENTRY_SIZE;

    uint8_t header[HEADER_SIZE] = {0};
    wr_u32(header, 0, 0x61727061u); /* magic "arpa" */
    wr_u32(header, 4, 1u);          /* version */
    header[8] = 1;                  /* min zoom */
    header[9] = 1;                  /* max zoom */
    wr_u64(header, 56, 2u);         /* tile count */
    wr_u64(header, 64, dir_off);
    wr_u64(header, 72, meta_off);
    wr_u64(header, 80, strlen(meta));

    FILE *f = fopen(g_path, "wb");
    if (!f) return false;
    fwrite(header, 1, HEADER_SIZE, f);
    fwrite(blob_a, 1, len_a, f);
    fwrite(blob_b, 1, len_b, f);

    uint64_t offs[2] = {off_a, off_b};
    size_t lens[2] = {len_a, len_b};
    for (int i = 0; i < 2; i++) {
        uint8_t e[DIR_ENTRY_SIZE] = {0};
        wr_u64(e, 0, arpt_archive_tile_id(tiles[i].z, tiles[i].x, tiles[i].y));
        wr_u64(e, 8, offs[i]);
        wr_u64(e, 16, lens[i]);
        e[24] = (uint8_t)tiles[i].z;
        wr_u32(e, 28, (uint32_t)tiles[i].x);
        wr_u32(e, 32, (uint32_t)tiles[i].y);
        fwrite(e, 1, DIR_ENTRY_SIZE, f);
    }
    fwrite(meta, 1, strlen(meta), f);
    fclose(f);
    return true;
}

static void test_reads_back_what_the_writer_laid_down(void) {
    TEST_ASSERT_TRUE(write_archive("tile-A", "tile-BB"));

    arpt_archive *a = arpt_archive_open(g_path);
    TEST_ASSERT_NOT_NULL(a);
    TEST_ASSERT_EQUAL_UINT64(2, arpt_archive_tile_count(a));

    const uint8_t *blob = NULL;
    size_t size = 0;
    TEST_ASSERT_TRUE(arpt_archive_tile(a, 1, 0, 0, &blob, &size));
    TEST_ASSERT_EQUAL_size_t(6, size);
    TEST_ASSERT_EQUAL_MEMORY("tile-A", blob, 6);

    TEST_ASSERT_TRUE(arpt_archive_tile(a, 1, 1, 0, &blob, &size));
    TEST_ASSERT_EQUAL_size_t(7, size);
    TEST_ASSERT_EQUAL_MEMORY("tile-BB", blob, 7);

    TEST_ASSERT_TRUE(arpt_archive_metadata(a, &blob, &size));
    TEST_ASSERT_EQUAL_size_t(13, size);
    TEST_ASSERT_EQUAL_MEMORY("metadata-blob", blob, 13);

    arpt_archive_free(a);
}

/* A sparse archive is the normal case, and a miss is an ordinary answer —
   the same one a server gives with a 404. */
static void test_a_missing_tile_is_not_an_error(void) {
    TEST_ASSERT_TRUE(write_archive("tile-A", "tile-BB"));

    arpt_archive *a = arpt_archive_open(g_path);
    TEST_ASSERT_NOT_NULL(a);

    const uint8_t *blob = NULL;
    size_t size = 0;
    TEST_ASSERT_FALSE(arpt_archive_tile(a, 1, 0, 1, &blob, &size));
    TEST_ASSERT_FALSE(arpt_archive_tile(a, 7, 5, 5, &blob, &size));

    arpt_archive_free(a);
}

static void test_rejects_what_is_not_an_archive(void) {
    FILE *f = fopen(g_path, "wb");
    TEST_ASSERT_NOT_NULL(f);
    /* Long enough to hold a header, but not one. */
    uint8_t junk[HEADER_SIZE * 2] = {0};
    memcpy(junk, "not an arpa file", 16);
    fwrite(junk, 1, sizeof(junk), f);
    fclose(f);
    TEST_ASSERT_NULL(arpt_archive_open(g_path));

    /* Too short to hold a header at all. */
    f = fopen(g_path, "wb");
    TEST_ASSERT_NOT_NULL(f);
    fwrite("short", 1, 5, f);
    fclose(f);
    TEST_ASSERT_NULL(arpt_archive_open(g_path));

    TEST_ASSERT_NULL(arpt_archive_open("/nonexistent/path/to.arpa"));
}

/* A directory pointing past the end of the file must be refused at open, so
   no lookup can walk off the mapping. */
static void test_rejects_a_directory_outside_the_file(void) {
    uint8_t header[HEADER_SIZE] = {0};
    wr_u32(header, 0, 0x61727061u);
    wr_u32(header, 4, 1u);
    wr_u64(header, 56, 1000u);        /* claims 1000 tiles */
    wr_u64(header, 64, HEADER_SIZE);  /* ...in a file this size */

    FILE *f = fopen(g_path, "wb");
    TEST_ASSERT_NOT_NULL(f);
    fwrite(header, 1, HEADER_SIZE, f);
    fclose(f);

    TEST_ASSERT_NULL(arpt_archive_open(g_path));
}

int main(void) {
    snprintf(g_path, sizeof(g_path), "test_archive_%d.arpa", (int)getpid());

    UNITY_BEGIN();
    RUN_TEST(test_tile_id_matches_the_tiler);
    RUN_TEST(test_reads_back_what_the_writer_laid_down);
    RUN_TEST(test_a_missing_tile_is_not_an_error);
    RUN_TEST(test_rejects_what_is_not_an_archive);
    RUN_TEST(test_rejects_a_directory_outside_the_file);
    int rc = UNITY_END();

    remove(g_path);
    return rc;
}
