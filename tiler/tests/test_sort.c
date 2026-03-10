#include "unity.h"
#include "sort.h"

void setUp(void) {}
void tearDown(void) {}

static void test_sorter_create_free(void) {
    arpt_sorter *s = arpt_sorter_create("/tmp", 1024);
    /* Stub returns NULL — just verify no crash. */
    arpt_sorter_free(s);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_sorter_create_free);
    return UNITY_END();
}
