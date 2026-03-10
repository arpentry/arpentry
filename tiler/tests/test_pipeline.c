#include "unity.h"
#include "pipeline.h"

void setUp(void) {}
void tearDown(void) {}

static void test_pipeline_run_null(void) {
    bool ok = arpt_pipeline_run(NULL);
    TEST_ASSERT_FALSE(ok);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pipeline_run_null);
    return UNITY_END();
}
