#include "civetweb.h"
#include "tile_path.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define READ_BUF_SIZE (64 * 1024)

static const char *g_tile_dir;
static volatile int g_stop;

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static void send_cors_headers(struct mg_connection *conn) {
    mg_printf(conn, "Access-Control-Allow-Origin: *\r\n");
}

static int send_file(struct mg_connection *conn, const char *path,
                     const char *content_type, const char *content_encoding) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        mg_printf(conn,
                  "HTTP/1.1 404 Not Found\r\n"
                  "Content-Length: 0\r\n\r\n");
        return 404;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    mg_printf(conn,
              "HTTP/1.1 200 OK\r\n"
              "Content-Type: %s\r\n"
              "Content-Length: %ld\r\n",
              content_type, size);
    if (content_encoding) {
        mg_printf(conn, "Content-Encoding: %s\r\n", content_encoding);
    }
    send_cors_headers(conn);
    mg_printf(conn, "\r\n");

    unsigned char buf[READ_BUF_SIZE];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        mg_write(conn, buf, n);
    }

    fclose(f);
    return 200;
}

static int handler(struct mg_connection *conn, void *cbdata) {
    (void)cbdata;
    const struct mg_request_info *ri = mg_get_request_info(conn);
    const char *uri = ri->local_uri;

    /* Tile request: /{level}/{x}/{y}.arpt */
    int level, x, y;
    if (arpt_parse_tile_path(uri, &level, &x, &y)) {
        char path[1024];
        snprintf(path, sizeof(path), "%s/%d/%d/%d.arpt",
                 g_tile_dir, level, x, y);
        return send_file(conn, path, "application/x-arpt", "br");
    }

    /* Tileset metadata */
    if (strcmp(uri, "/tileset.json") == 0) {
        char path[1024];
        snprintf(path, sizeof(path), "%s/tileset.json", g_tile_dir);
        return send_file(conn, path, "application/json", NULL);
    }

    /* Everything else: 404 */
    mg_printf(conn,
              "HTTP/1.1 404 Not Found\r\n"
              "Content-Length: 0\r\n\r\n");
    return 404;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: arpt_server <tile_dir> <port>\n");
        return 1;
    }

    g_tile_dir = argv[1];
    const char *port = argv[2];

    const char *options[] = {
        "listening_ports", port,
        "num_threads",     "4",
        NULL
    };

    struct mg_callbacks callbacks;
    memset(&callbacks, 0, sizeof(callbacks));

    struct mg_context *ctx = mg_start(&callbacks, NULL, options);
    if (!ctx) {
        fprintf(stderr, "Failed to start server on port %s\n", port);
        return 1;
    }

    mg_set_request_handler(ctx, "/", handler, NULL);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("Serving tiles from %s on port %s\n", g_tile_dir, port);

    while (!g_stop) {
        struct timespec ts = {0, 200000000}; /* 200ms */
        nanosleep(&ts, NULL);
    }

    printf("\nShutting down...\n");
    mg_stop(ctx);
    return 0;
}
