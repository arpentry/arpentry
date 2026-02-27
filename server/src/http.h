#ifndef ARPENTRY_HTTP_H
#define ARPENTRY_HTTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct net_conn;

/* Maximum size of an HTTP request we'll buffer before rejecting. */
#define HTTP_MAX_REQUEST 8192

/* Pure request parsing (no I/O, testable) */

/* Parse an HTTP request line from a byte buffer.
 * Returns bytes consumed on success, 0 if incomplete, -1 on error. */
int http_parse_request(const char *data, size_t len,
                       char *method, size_t method_sz,
                       char *uri, size_t uri_sz);

/* Per-connection HTTP state */

struct server_ctx {
    const char *tile_dir;
};

typedef struct {
    char buf[HTTP_MAX_REQUEST];
    size_t filled;
} http_conn;

/* Create a new per-connection HTTP state. Returns NULL on allocation failure. */
http_conn *http_conn_new(void);

/* Free per-connection HTTP state. */
void http_conn_free(http_conn *hc);

/* Feed incoming bytes. Dispatches complete requests via net_conn_out_write(). */
void http_conn_feed(http_conn *hc, struct net_conn *conn,
                    struct server_ctx *ctx,
                    const void *data, size_t len);

#endif /* ARPENTRY_HTTP_H */
