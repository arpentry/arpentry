#include "unity.h"
#include "tile_fetch.h"
#include "demo_tile.h"
#include "tile_reader.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define TEST_PORT 19200
#define DRAIN_TIMEOUT_US 5000000 /* 5 seconds */
#define DRAIN_POLL_US 1000       /* 1 ms */

void setUp(void) {}
void tearDown(void) {}

/* In-process tile server (pthread) */

static uint8_t *g_demo_tile;
static size_t g_demo_tile_size;
static int g_listenfd;
static volatile int g_server_running;

static void server_write_all(int fd, const void *buf, size_t size) {
    const uint8_t *p = buf;
    size_t remaining = size;
    while (remaining > 0) {
        ssize_t n = write(fd, p, remaining);
        if (n <= 0) return;
        p += n;
        remaining -= (size_t)n;
    }
}

static void server_handle(int fd) {
    /* Read and discard request (just drain until \r\n\r\n) */
    char buf[2048];
    size_t filled = 0;
    while (filled < sizeof(buf) - 1) {
        ssize_t n = read(fd, buf + filled, sizeof(buf) - 1 - filled);
        if (n <= 0) break;
        filled += (size_t)n;
        buf[filled] = '\0';
        if (strstr(buf, "\r\n\r\n")) break;
    }

    /* Send tile response */
    char hdr[512];
    int hlen = snprintf(hdr, sizeof(hdr),
                        "HTTP/1.1 200 OK\r\n"
                        "Content-Type: application/x-arpt\r\n"
                        "Content-Length: %zu\r\n"
                        "Content-Encoding: br\r\n"
                        "Connection: close\r\n"
                        "Access-Control-Allow-Origin: *\r\n"
                        "\r\n",
                        g_demo_tile_size);

    server_write_all(fd, hdr, (size_t)hlen);
    server_write_all(fd, g_demo_tile, g_demo_tile_size);
    close(fd);
}

static void *server_thread(void *arg) {
    (void)arg;
    while (g_server_running) {
        int fd = accept(g_listenfd, NULL, NULL);
        if (fd < 0) break;
        server_handle(fd);
    }
    return NULL;
}

/* Fetch callback results */

static bool g_fetch_success;
static size_t g_fetch_size;
static volatile bool g_fetch_called;

static void on_tile_fetched(bool success, uint8_t *flatbuf, size_t size,
                            void *userdata) {
    (void)userdata;
    g_fetch_success = success;
    g_fetch_size = size;
    g_fetch_called = true;

    if (success && flatbuf) {
        /* Verify we can read the tile */
        arpentry_tiles_Tile_table_t tile = arpentry_tiles_Tile_as_root(flatbuf);
        TEST_ASSERT_NOT_NULL(tile);

        /* Check that layers exist */
        arpentry_tiles_Layer_vec_t layers = arpentry_tiles_Tile_layers(tile);
        TEST_ASSERT_NOT_NULL(layers);
        TEST_ASSERT_TRUE(arpentry_tiles_Layer_vec_len(layers) > 0);

        printf("  Fetched tile: %zu bytes, %zu layers\n", size,
               arpentry_tiles_Layer_vec_len(layers));
    }

    free(flatbuf);
}

/* Drain helper with timeout */

static bool drain_until_called(void) {
    int elapsed = 0;
    while (!g_fetch_called && elapsed < DRAIN_TIMEOUT_US) {
        arpt_fetch_drain();
        usleep(DRAIN_POLL_US);
        elapsed += DRAIN_POLL_US;
    }
    arpt_fetch_drain(); /* one final drain */
    return g_fetch_called;
}

/* Tests */

void test_fetch_tile_success(void) {
    g_fetch_called = false;
    g_fetch_success = false;
    g_fetch_size = 0;

    char base_url[64];
    snprintf(base_url, sizeof(base_url), "http://127.0.0.1:%d", TEST_PORT);

    bool initiated = arpt_fetch_tile(base_url, 0, 0, 0, on_tile_fetched, NULL);
    TEST_ASSERT_TRUE(initiated);
    TEST_ASSERT_TRUE(drain_until_called());
    TEST_ASSERT_TRUE(g_fetch_success);
    TEST_ASSERT_TRUE(g_fetch_size > 0);
}

void test_fetch_tile_different_coords(void) {
    g_fetch_called = false;

    char base_url[64];
    snprintf(base_url, sizeof(base_url), "http://127.0.0.1:%d", TEST_PORT);

    bool initiated =
        arpt_fetch_tile(base_url, 5, 32, 16, on_tile_fetched, NULL);
    TEST_ASSERT_TRUE(initiated);
    TEST_ASSERT_TRUE(drain_until_called());
    TEST_ASSERT_TRUE(g_fetch_success);
}

void test_fetch_tile_bad_url(void) {
    g_fetch_called = false;

    bool initiated =
        arpt_fetch_tile("http://127.0.0.1:1", 0, 0, 0, on_tile_fetched, NULL);
    /* Should initiate but callback reports failure (connection refused) */
    TEST_ASSERT_TRUE(initiated);
    TEST_ASSERT_TRUE(drain_until_called());
    TEST_ASSERT_FALSE(g_fetch_success);
}

void test_fetch_tile_null_url(void) {
    bool initiated = arpt_fetch_tile(NULL, 0, 0, 0, on_tile_fetched, NULL);
    TEST_ASSERT_FALSE(initiated);
}

/* Main */

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Build demo tile */
    if (!arpt_build_demo_tile(&g_demo_tile, &g_demo_tile_size)) {
        fprintf(stderr, "Failed to build demo tile\n");
        return 1;
    }

    /* Start in-process server */
    g_listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listenfd < 0) {
        fprintf(stderr, "Failed to create socket\n");
        free(g_demo_tile);
        return 1;
    }

    int opt = 1;
    setsockopt(g_listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = htons(TEST_PORT),
    };

    if (bind(g_listenfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "Failed to bind on port %d\n", TEST_PORT);
        close(g_listenfd);
        free(g_demo_tile);
        return 1;
    }

    if (listen(g_listenfd, 8) < 0) {
        fprintf(stderr, "Failed to listen\n");
        close(g_listenfd);
        free(g_demo_tile);
        return 1;
    }

    g_server_running = 1;
    pthread_t tid;
    pthread_create(&tid, NULL, server_thread, NULL);

    /* Initialize the fetch thread pool */
    if (!arpt_fetch_init(2)) {
        fprintf(stderr, "Failed to init fetch pool\n");
        g_server_running = 0;
        close(g_listenfd);
        pthread_join(tid, NULL);
        free(g_demo_tile);
        return 1;
    }

    int result;
    UNITY_BEGIN();
    RUN_TEST(test_fetch_tile_success);
    RUN_TEST(test_fetch_tile_different_coords);
    RUN_TEST(test_fetch_tile_bad_url);
    RUN_TEST(test_fetch_tile_null_url);
    result = UNITY_END();

    /* Shut down fetch pool, then server */
    arpt_fetch_shutdown();
    g_server_running = 0;
    close(g_listenfd);
    pthread_join(tid, NULL);
    free(g_demo_tile);

    return result;
}
