#include "unity.h"
#include "archive.h"

void setUp(void) {}
void tearDown(void) {}

static void test_archive_writer_create_free(void) {
    arpt_archive_writer *w = arpt_archive_writer_create("/tmp/test.arpa");
    /* Stub returns NULL — just verify no crash. */
    arpt_archive_writer_free(w);
}

static void test_archive_reader_open_close(void) {
    arpt_archive_reader *r = arpt_archive_reader_open("/nonexistent");
    /* Stub returns NULL — just verify no crash. */
    arpt_archive_reader_close(r);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_archive_writer_create_free);
    RUN_TEST(test_archive_reader_open_close);
    return UNITY_END();
}
