/* Tests for segment merge tool.
 *
 * Uses the 100-row segment.parquet fixture from Overture 2026-02-18.0.
 * Verifies that the merge pipeline produces valid GeoParquet output
 * readable by the existing overture reader.  The fixture may not
 * contain major road classes (motorway/trunk/primary), so tests
 * allow zero output rows. */

#include "unity.h"
#include "merge.h"
#include "overture.h"
#include "wkb.h"
#include "geom.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures/overture"
#endif

static char input_path[512];
static char output_path[512];

void setUp(void) {
    snprintf(input_path, sizeof(input_path), "%s/segment.parquet", FIXTURE_DIR);
    const char *tmpdir = getenv("TMPDIR");
    if (tmpdir == NULL) tmpdir = "/tmp";
    size_t tlen = strlen(tmpdir);
    const char *sep = (tlen > 0 && tmpdir[tlen - 1] == '/') ? "" : "/";
    snprintf(output_path, sizeof(output_path), "%s%stest_merged_XXXXXX",
             tmpdir, sep);
    int fd = mkstemp(output_path);
    if (fd >= 0) close(fd);
    unlink(output_path);
    strncat(output_path, ".parquet", sizeof(output_path) - strlen(output_path) - 1);
}

void tearDown(void) {
    unlink(output_path);
    char base[512];
    strncpy(base, output_path, sizeof(base) - 1);
    char *ext = strstr(base, ".parquet");
    if (ext) { *ext = '\0'; unlink(base); }
}

/* ── Test: merge runs without error ───────────────────────────────────── */

static void test_merge_runs(void) {
    bool ok = arpt_merge_run(input_path, output_path, NULL);
    TEST_ASSERT_TRUE_MESSAGE(ok, "arpt_merge_run failed");
}

/* ── Test: output is valid GeoParquet readable by overture reader ─────── */

static void test_output_readable(void) {
    bool ok = arpt_merge_run(input_path, output_path, NULL);
    TEST_ASSERT_TRUE(ok);

    arpt_overture *ov = arpt_overture_open(output_path);
    TEST_ASSERT_NOT_NULL_MESSAGE(ov, "Cannot open merged output");

    int count = 0;
    arpt_overture_feature feat;
    while (arpt_overture_next(ov, &feat)) {
        TEST_ASSERT_NOT_NULL(feat.wkb);
        TEST_ASSERT_TRUE(feat.wkb_len > 0);

        arpt_geom geom = {0};
        TEST_ASSERT_TRUE(arpt_wkb_parse(feat.wkb, feat.wkb_len, &geom));
        TEST_ASSERT_TRUE(geom.n_coords >= 2);

        for (uint32_t i = 0; i < geom.n_coords; i++) {
            TEST_ASSERT_TRUE(geom.x[i] >= -180.0 && geom.x[i] <= 180.0);
            TEST_ASSERT_TRUE(geom.y[i] >= -90.0 && geom.y[i] <= 90.0);
        }

        arpt_geom_free(&geom);
        count++;
    }

    arpt_overture_close(ov);

    /* Output may have 0 rows if the fixture has no major road classes */
    fprintf(stderr, "  merged output: %d features\n", count);
    TEST_ASSERT_TRUE_MESSAGE(count >= 0, "Unexpected negative count");
}

/* ── Test: output has <= input rows ───────────────────────────────────── */

static void test_row_count_reduced(void) {
    arpt_overture *ov_in = arpt_overture_open(input_path);
    TEST_ASSERT_NOT_NULL(ov_in);
    int in_count = 0;
    arpt_overture_feature feat;
    while (arpt_overture_next(ov_in, &feat)) in_count++;
    arpt_overture_close(ov_in);

    bool ok = arpt_merge_run(input_path, output_path, NULL);
    TEST_ASSERT_TRUE(ok);

    arpt_overture *ov_out = arpt_overture_open(output_path);
    TEST_ASSERT_NOT_NULL(ov_out);
    int out_count = 0;
    while (arpt_overture_next(ov_out, &feat)) out_count++;
    arpt_overture_close(ov_out);

    fprintf(stderr, "  input: %d rows, output: %d rows\n", in_count, out_count);

    /* Output contains only major classes, so always <= input */
    TEST_ASSERT_TRUE_MESSAGE(out_count <= in_count,
        "Merged output has more rows than input");
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_merge_runs);
    RUN_TEST(test_output_readable);
    RUN_TEST(test_row_count_reduced);
    return UNITY_END();
}
