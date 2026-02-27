#include "unity.h"
#include "http.h"

#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* http_parse_request tests */

void test_parse_valid_get(void) {
    const char *raw = "GET /0/0/0.arpt HTTP/1.1\r\nHost: localhost\r\n\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_GREATER_THAN(0, consumed);
    TEST_ASSERT_EQUAL_STRING("GET", method);
    TEST_ASSERT_EQUAL_STRING("/0/0/0.arpt", uri);
}

void test_parse_valid_get_tileset(void) {
    const char *raw = "GET /tileset.json HTTP/1.1\r\nHost: localhost\r\n\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_GREATER_THAN(0, consumed);
    TEST_ASSERT_EQUAL_STRING("GET", method);
    TEST_ASSERT_EQUAL_STRING("/tileset.json", uri);
}

void test_parse_post_method(void) {
    const char *raw = "POST /data HTTP/1.1\r\nHost: localhost\r\n\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_GREATER_THAN(0, consumed);
    TEST_ASSERT_EQUAL_STRING("POST", method);
    TEST_ASSERT_EQUAL_STRING("/data", uri);
}

void test_parse_incomplete_request(void) {
    /* No \r\n yet — should return 0 (need more data) */
    const char *raw = "GET /foo HTTP/1.1";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_EQUAL_INT(0, consumed);
}

void test_parse_empty_input(void) {
    char method[8], uri[2048];
    int consumed =
        http_parse_request("", 0, method, sizeof(method), uri, sizeof(uri));
    TEST_ASSERT_EQUAL_INT(0, consumed);
}

void test_parse_malformed_no_space(void) {
    /* No spaces in request line */
    const char *raw = "GETFOO\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_EQUAL_INT(-1, consumed);
}

void test_parse_method_too_long(void) {
    /* Method won't fit in 8-byte buffer */
    const char *raw = "VERYLONGMETHOD /foo HTTP/1.1\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    TEST_ASSERT_EQUAL_INT(-1, consumed);
}

void test_parse_returns_correct_consumed(void) {
    /* Verify consumed count includes \r\n */
    const char *raw = "GET / HTTP/1.1\r\nHost: x\r\n\r\n";
    char method[8], uri[2048];
    int consumed = http_parse_request(raw, strlen(raw), method, sizeof(method),
                                      uri, sizeof(uri));
    /* "GET / HTTP/1.1\r\n" is 16 bytes */
    TEST_ASSERT_EQUAL_INT(16, consumed);
}

/* Main */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_parse_valid_get);
    RUN_TEST(test_parse_valid_get_tileset);
    RUN_TEST(test_parse_post_method);
    RUN_TEST(test_parse_incomplete_request);
    RUN_TEST(test_parse_empty_input);
    RUN_TEST(test_parse_malformed_no_space);
    RUN_TEST(test_parse_method_too_long);
    RUN_TEST(test_parse_returns_correct_consumed);
    return UNITY_END();
}
