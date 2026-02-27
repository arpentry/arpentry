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
} arpt_http_response;

/**
 * Perform an HTTP GET for the given URL and return the response body.
 * Only plain HTTP is supported (no TLS).
 * Brotli-compressed responses are decompressed automatically.
 * Caller must free resp->body on success.
 */
bool arpt_http_get(const char *url, arpt_http_response *resp);

#endif /* !__EMSCRIPTEN__ */
#endif /* ARPENTRY_HTTP_H */
