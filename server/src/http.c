#include "http.h"
#include "net.h"
#include "gen/world.h"
#include "tile_path.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Helpers */

static const char *status_text(int status) {
    switch (status) {
    case 200:
        return "OK";
    case 400:
        return "Bad Request";
    case 404:
        return "Not Found";
    case 405:
        return "Method Not Allowed";
    case 500:
        return "Internal Server Error";
    default:
        return "Unknown";
    }
}

/* Pure request parsing */

int http_parse_request(const char *data, size_t len, char *method,
                       size_t method_sz, char *uri, size_t uri_sz) {
    /* Find the end of the request line (\r\n) */
    const char *end = NULL;
    for (size_t i = 0; i + 1 < len; i++) {
        if (data[i] == '\r' && data[i + 1] == '\n') {
            end = data + i;
            break;
        }
    }
    if (!end) {
        /* Incomplete — need more data */
        return 0;
    }

    const char *line = data;
    size_t line_len = (size_t)(end - line);

    /* Parse "METHOD URI HTTP/1.x" */
    const char *sp1 = memchr(line, ' ', line_len);
    if (!sp1) return -1;

    size_t mlen = (size_t)(sp1 - line);
    if (mlen == 0 || mlen >= method_sz) return -1;
    memcpy(method, line, mlen);
    method[mlen] = '\0';

    const char *uri_start = sp1 + 1;
    size_t remaining = line_len - mlen - 1;
    const char *sp2 = memchr(uri_start, ' ', remaining);
    if (!sp2) return -1;

    size_t ulen = (size_t)(sp2 - uri_start);
    if (ulen == 0 || ulen >= uri_sz) return -1;
    memcpy(uri, uri_start, ulen);
    uri[ulen] = '\0';

    /* Return bytes consumed including \r\n */
    return (int)(end - data) + 2;
}

/* Response writing via net_conn */

static void write_response(struct net_conn *conn, int status,
                           const char *content_type,
                           const char *content_encoding, const void *body,
                           size_t body_len) {
    char hdr[1024];
    int n = snprintf(hdr, sizeof(hdr),
                     "HTTP/1.1 %d %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %zu\r\n"
                     "Connection: close\r\n"
                     "Access-Control-Allow-Origin: *\r\n",
                     status, status_text(status), content_type, body_len);
    if (n < 0 || (size_t)n >= sizeof(hdr)) return;

    if (content_encoding) {
        int m = snprintf(hdr + n, sizeof(hdr) - (size_t)n,
                         "Content-Encoding: %s\r\n", content_encoding);
        if (m < 0 || (size_t)(n + m) >= sizeof(hdr)) return;
        n += m;
    }

    /* End of headers */
    if ((size_t)n + 2 >= sizeof(hdr)) return;
    hdr[n++] = '\r';
    hdr[n++] = '\n';

    net_conn_out_write(conn, hdr, (size_t)n);
    if (body && body_len > 0) {
        net_conn_out_write(conn, body, body_len);
    }
}

static void write_error(struct net_conn *conn, int status) {
    write_response(conn, status, "text/plain", NULL, NULL, 0);
}

static void write_file_response(struct net_conn *conn, int status,
                                const char *content_type, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        write_error(conn, 404);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize <= 0) {
        fclose(f);
        write_error(conn, 404);
        return;
    }

    uint8_t *buf = malloc((size_t)fsize);
    if (!buf) {
        fclose(f);
        write_error(conn, 500);
        return;
    }

    size_t nread = fread(buf, 1, (size_t)fsize, f);
    fclose(f);

    write_response(conn, status, content_type, NULL, buf, nread);
    free(buf);
}

/* Request dispatch */

static void dispatch_request(struct net_conn *conn, struct server_ctx *ctx,
                             const char *method, const char *uri) {
    /* Only accept GET */
    if (strcmp(method, "GET") != 0) {
        write_error(conn, 405);
        return;
    }

    /* Tile request: /{level}/{x}/{y}.arpt */
    int level, x, y;
    if (arpt_parse_tile_path(uri, &level, &x, &y)) {
        uint8_t *tile_data = NULL;
        size_t tile_size = 0;
        if (arpt_generate_terrain(level, x, y, &tile_data, &tile_size)) {
            write_response(conn, 200, "application/x-arpt", "br", tile_data,
                           tile_size);
            free(tile_data);
        } else {
            write_error(conn, 500);
        }
        return;
    }

    /* Tileset metadata */
    if (strcmp(uri, "/tileset.json") == 0) {
        char path[1024];
        snprintf(path, sizeof(path), "%s/tileset.json", ctx->tile_dir);
        write_file_response(conn, 200, "application/json", path);
        return;
    }

    /* Everything else: 404 */
    write_error(conn, 404);
}

/* Per-connection state */

struct http_conn {
    char buf[HTTP_MAX_REQUEST];
    size_t filled;
};

http_conn *http_conn_new(void) {
    http_conn *hc = calloc(1, sizeof(http_conn));
    return hc;
}

void http_conn_free(http_conn *hc) {
    free(hc);
}

void http_conn_feed(http_conn *hc, struct net_conn *conn,
                    struct server_ctx *ctx, const void *data, size_t len) {
    /* Accumulate incoming data */
    if (len > 0) {
        size_t space = sizeof(hc->buf) - hc->filled;
        size_t copy = len < space ? len : space;
        memcpy(hc->buf + hc->filled, data, copy);
        hc->filled += copy;
    }

    /* Reject oversized requests */
    if (hc->filled >= sizeof(hc->buf)) {
        write_error(conn, 400);
        net_conn_close(conn);
        return;
    }

    /* Try to parse a complete request */
    char method[8];
    char uri[2048];
    int consumed = http_parse_request(hc->buf, hc->filled, method,
                                      sizeof(method), uri, sizeof(uri));
    if (consumed == 0) {
        /* Incomplete, wait for more data */
        return;
    }
    if (consumed < 0) {
        write_error(conn, 400);
        net_conn_close(conn);
        return;
    }

    /* Dispatch and close (HTTP/1.0 style: one request per connection) */
    dispatch_request(conn, ctx, method, uri);
    net_conn_close(conn);
}
