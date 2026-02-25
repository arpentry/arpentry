#ifndef ARPENTRY_HTTP_H
#define ARPENTRY_HTTP_H

#ifndef __EMSCRIPTEN__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *body;
    size_t body_size;
    int status;
    bool brotli;
} http_response_t;

/**
 * Parse "http://host:port/path" into components.
 * Only plain HTTP is supported (no TLS).
 */
bool http_parse_url(const char *url,
                    char *host, size_t host_cap,
                    int *port,
                    char *path, size_t path_cap);

/**
 * Perform an HTTP GET and return the response body.
 * Caller must free resp->body on success.
 */
bool http_get(const char *host, int port, const char *path,
              http_response_t *resp);

#endif /* !__EMSCRIPTEN__ */
#endif /* ARPENTRY_HTTP_H */
