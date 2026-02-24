#ifndef ARPENTRY_HTTP_H
#define ARPENTRY_HTTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct net_conn;

/* Maximum size of an HTTP request we'll buffer before rejecting. */
#define HTTP_MAX_REQUEST 8192

/* ── Pure request parsing (no I/O, testable) ───────────────────────── */

/**
 * Parse an HTTP request line from a byte buffer.
 *
 * Looks for the first complete request line ending with \r\n.
 * On success, writes NUL-terminated method and URI into the provided buffers.
 *
 * @param data      Input buffer (need not be NUL-terminated).
 * @param len       Number of bytes available.
 * @param method    Output buffer for method (at least 8 bytes).
 * @param method_sz Size of method buffer.
 * @param uri       Output buffer for URI (at least 2048 bytes).
 * @param uri_sz    Size of URI buffer.
 * @return          Bytes consumed (up to and including \r\n) on success,
 *                  0 if the request is incomplete (need more data),
 *                  -1 on parse error.
 */
int http_parse_request(const char *data, size_t len,
                       char *method, size_t method_sz,
                       char *uri, size_t uri_sz);

/* ── Per-connection HTTP state ─────────────────────────────────────── */

struct server_ctx {
    const char *tile_dir;
    uint8_t *demo_tile;
    size_t demo_tile_size;
};

typedef struct {
    char buf[HTTP_MAX_REQUEST];
    size_t filled;
} http_conn;

/**
 * Create a new per-connection HTTP state.
 * Returns NULL on allocation failure.
 */
http_conn *http_conn_new(void);

/**
 * Free per-connection HTTP state.
 */
void http_conn_free(http_conn *hc);

/**
 * Feed incoming bytes to the HTTP connection.
 * When a complete request is detected, dispatches it and writes the
 * response to conn's output buffer via net_conn_out_write().
 */
void http_conn_feed(http_conn *hc, struct net_conn *conn,
                    struct server_ctx *ctx,
                    const void *data, size_t len);

#endif /* ARPENTRY_HTTP_H */
