#ifndef ARPENTRY_HTTP_H
#define ARPENTRY_HTTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct net_conn;
struct arpt_archive_reader;

/* Pure request parsing (no I/O, testable) */

/* Parse an HTTP request line from a byte buffer.
 * Returns bytes consumed on success, 0 if incomplete, -1 on error. */
int http_parse_request(const char *data, size_t len, char *method,
                       size_t method_sz, char *uri, size_t uri_sz);

/* Server context (opaque — constructed via arpt_server_ctx_create). */
typedef struct server_ctx server_ctx;

server_ctx *arpt_server_ctx_create(const char *tile_dir,
                                   const char *style_file,
                                   struct arpt_archive_reader *archive);
void arpt_server_ctx_free(server_ctx *ctx);

/* Per-connection HTTP state */

typedef struct http_conn http_conn;

/* Create a new per-connection HTTP state. Returns NULL on allocation failure.
 */
http_conn *http_conn_new(void);

/* Free per-connection HTTP state. */
void http_conn_free(http_conn *hc);

/* Feed incoming bytes. Dispatches complete requests via net_conn_out_write().
 */
void http_conn_feed(http_conn *hc, struct net_conn *conn,
                    server_ctx *ctx, const void *data, size_t len);

#endif /* ARPENTRY_HTTP_H */
