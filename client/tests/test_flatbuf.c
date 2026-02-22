#include "example_builder.h"
#include "example_reader.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_point_roundtrip(void) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    /* Build a Point */
    Example_Point_start_as_root(&builder);
    Example_Point_x_add(&builder, 1.0f);
    Example_Point_y_add(&builder, 2.5f);
    Example_Point_z_add(&builder, -3.0f);
    Example_Point_label_create_str(&builder, "origin");
    Example_Point_end_as_root(&builder);

    /* Get the buffer */
    size_t size;
    void *buf = flatcc_builder_finalize_buffer(&builder, &size);
    TEST_ASSERT_NOT_NULL(buf);

    /* Read it back */
    Example_Point_table_t point = Example_Point_as_root(buf);
    TEST_ASSERT_NOT_NULL(point);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, Example_Point_x(point));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, Example_Point_y(point));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.0f, Example_Point_z(point));
    TEST_ASSERT_EQUAL_STRING("origin", Example_Point_label(point));

    free(buf);
    flatcc_builder_clear(&builder);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_point_roundtrip);
    return UNITY_END();
}
