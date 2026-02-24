#include "tile_fetch.h"

#ifdef __EMSCRIPTEN__

#include <emscripten/fetch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tile_verifier.h"

typedef struct {
    arpt_tile_fetch_cb cb;
    void *userdata;
} FetchRequest;

static void on_fetch_success(emscripten_fetch_t *fetch) {
    FetchRequest *req = (FetchRequest *)fetch->userData;

    int rc = arpentry_tiles_Tile_verify_as_root_with_identifier(
        fetch->data, (size_t)fetch->numBytes, "arpt");

    if (rc == 0) {
        req->cb(true, fetch->data, (size_t)fetch->numBytes, req->userdata);
    } else {
        fprintf(stderr, "tile_fetch: FlatBuffer verification failed (rc=%d)\n", rc);
        req->cb(false, NULL, 0, req->userdata);
    }

    emscripten_fetch_close(fetch);
    free(req);
}

static void on_fetch_error(emscripten_fetch_t *fetch) {
    FetchRequest *req = (FetchRequest *)fetch->userData;
    fprintf(stderr, "tile_fetch: HTTP %d for %s\n", fetch->status, fetch->url);
    req->cb(false, NULL, 0, req->userdata);
    emscripten_fetch_close(fetch);
    free(req);
}

bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata) {
    if (!base_url || !cb) return false;

    /* Build URL: {base_url}/{level}/{x}/{y}.arpt */
    char url[512];
    int n = snprintf(url, sizeof(url), "%s/%d/%d/%d.arpt", base_url, level, x, y);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    FetchRequest *req = malloc(sizeof(*req));
    if (!req) return false;
    req->cb = cb;
    req->userdata = userdata;

    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY;
    attr.onsuccess = on_fetch_success;
    attr.onerror = on_fetch_error;
    attr.userData = req;

    emscripten_fetch(&attr, url);
    return true;
}

#else /* native: synchronous fetch via POSIX sockets */

#include "tile.h"
#include "tile_verifier.h"

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/**
 * Parse "http://host:port/path" into components.
 * Only plain HTTP is supported (no TLS).
 */
static bool parse_url(const char *url,
                      char *host, size_t host_cap,
                      int *port,
                      char *path, size_t path_cap) {
    const char *p = url;

    /* Skip scheme */
    if (strncmp(p, "http://", 7) == 0) p += 7;

    /* Host */
    const char *host_start = p;
    while (*p && *p != ':' && *p != '/') p++;
    size_t hlen = (size_t)(p - host_start);
    if (hlen == 0 || hlen >= host_cap) return false;
    memcpy(host, host_start, hlen);
    host[hlen] = '\0';

    /* Port (default 80) */
    *port = 80;
    if (*p == ':') {
        p++;
        char *end;
        long val = strtol(p, &end, 10);
        if (end == p || val <= 0 || val > 65535) return false;
        *port = (int)val;
        p = end;
    }

    /* Path (may be empty) */
    if (*p == '/') {
        size_t plen = strlen(p);
        while (plen > 1 && p[plen - 1] == '/') plen--;
        if (plen >= path_cap) return false;
        memcpy(path, p, plen);
        path[plen] = '\0';
    } else {
        path[0] = '\0';
    }

    return true;
}

/* ── Low-level helpers ─────────────────────────────────────────────────── */

static int tcp_connect(const char *host, int port) {
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", port);

    struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res;
    if (getaddrinfo(host, port_str, &hints, &res) != 0) return -1;

    int fd = -1;
    for (struct addrinfo *rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0) continue;
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    return fd;
}

static bool write_all(int fd, const void *buf, size_t size) {
    const uint8_t *p = buf;
    size_t remaining = size;
    while (remaining > 0) {
        ssize_t n = write(fd, p, remaining);
        if (n <= 0) return false;
        p += n;
        remaining -= (size_t)n;
    }
    return true;
}

/* Find needle in haystack (portable memmem replacement) */
static const uint8_t *find_bytes(const uint8_t *haystack, size_t hlen,
                                 const uint8_t *needle, size_t nlen) {
    if (nlen > hlen) return NULL;
    for (size_t i = 0; i <= hlen - nlen; i++) {
        if (memcmp(haystack + i, needle, nlen) == 0)
            return haystack + i;
    }
    return NULL;
}

/* Case-insensitive substring search within a bounded region */
static bool header_contains(const char *headers, size_t hdr_len,
                            const char *name, const char *value) {
    /* Search for "name: ...value..." line by line */
    size_t nlen = strlen(name);
    const char *p = headers;
    const char *end = headers + hdr_len;
    while (p < end) {
        const char *eol = memchr(p, '\r', (size_t)(end - p));
        if (!eol) eol = end;
        /* Check if line starts with "name:" (case-insensitive) */
        size_t line_len = (size_t)(eol - p);
        if (line_len > nlen + 1) {
            bool match = true;
            for (size_t i = 0; i < nlen; i++) {
                char a = p[i];
                char b = name[i];
                if (a >= 'A' && a <= 'Z') a += 32;
                if (b >= 'A' && b <= 'Z') b += 32;
                if (a != b) { match = false; break; }
            }
            if (match && p[nlen] == ':') {
                /* Check if value appears in the rest of the line */
                size_t vlen = strlen(value);
                const char *vstart = p + nlen + 1;
                size_t remaining = (size_t)(eol - vstart);
                for (size_t i = 0; i + vlen <= remaining; i++) {
                    bool vmatch = true;
                    for (size_t j = 0; j < vlen; j++) {
                        char a = vstart[i + j];
                        char b = value[j];
                        if (a >= 'A' && a <= 'Z') a += 32;
                        if (b >= 'A' && b <= 'Z') b += 32;
                        if (a != b) { vmatch = false; break; }
                    }
                    if (vmatch) return true;
                }
            }
        }
        p = (eol < end && eol[0] == '\r' && eol + 1 < end && eol[1] == '\n')
            ? eol + 2 : eol + 1;
    }
    return false;
}

/* ── HTTP GET ──────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t *body;
    size_t body_size;
    int status;
    bool brotli;
} http_response_t;

static bool http_get(const char *host, int port, const char *path,
                     http_response_t *resp) {
    memset(resp, 0, sizeof(*resp));

    /* Connect */
    int fd = tcp_connect(host, port);
    if (fd < 0) {
        fprintf(stderr, "tile_fetch: connect to %s:%d failed\n", host, port);
        return false;
    }

    /* Send request */
    char req[512];
    int n = snprintf(req, sizeof(req),
                     "GET %s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Connection: close\r\n"
                     "\r\n",
                     path, host);
    if (n < 0 || (size_t)n >= sizeof(req) || !write_all(fd, req, (size_t)n)) {
        close(fd);
        return false;
    }

    /* Read full response (server sends Connection: close) */
    size_t capacity = 8192;
    uint8_t *buf = malloc(capacity);
    if (!buf) { close(fd); return false; }

    size_t total = 0;
    for (;;) {
        if (total >= capacity) {
            capacity *= 2;
            uint8_t *tmp = realloc(buf, capacity);
            if (!tmp) { free(buf); close(fd); return false; }
            buf = tmp;
        }
        ssize_t nr = read(fd, buf + total, capacity - total);
        if (nr <= 0) break;
        total += (size_t)nr;
    }
    close(fd);

    /* Find header/body boundary */
    const uint8_t sep[] = "\r\n\r\n";
    const uint8_t *boundary = find_bytes(buf, total, sep, 4);
    if (!boundary) {
        free(buf);
        return false;
    }

    size_t hdr_len = (size_t)(boundary - buf);
    const uint8_t *body_start = boundary + 4;
    size_t body_len = total - hdr_len - 4;

    /* Parse status code from first line: "HTTP/1.x NNN ..." */
    if (hdr_len < 12 || buf[8] != ' ') {
        free(buf);
        return false;
    }
    resp->status = (buf[9] - '0') * 100 + (buf[10] - '0') * 10 + (buf[11] - '0');

    /* Check Content-Encoding: br */
    resp->brotli = header_contains((const char *)buf, hdr_len,
                                   "Content-Encoding", "br");

    /* Copy body */
    resp->body = malloc(body_len);
    if (!resp->body) { free(buf); return false; }
    memcpy(resp->body, body_start, body_len);
    resp->body_size = body_len;

    free(buf);
    return true;
}

/* ── Public API ────────────────────────────────────────────────────────── */

bool arpt_fetch_tile(const char *base_url, int level, int x, int y,
                     arpt_tile_fetch_cb cb, void *userdata) {
    if (!base_url || !cb) return false;

    /* Parse base URL */
    char host[256], path_prefix[256];
    int port;
    if (!parse_url(base_url, host, sizeof(host), &port,
                   path_prefix, sizeof(path_prefix)))
        return false;

    /* Build request path: {path_prefix}/{level}/{x}/{y}.arpt */
    char req_path[512];
    int n = snprintf(req_path, sizeof(req_path), "%s/%d/%d/%d.arpt",
                     path_prefix, level, x, y);
    if (n < 0 || (size_t)n >= sizeof(req_path)) return false;

    /* HTTP GET */
    http_response_t resp;
    if (!http_get(host, port, req_path, &resp)) {
        cb(false, NULL, 0, userdata);
        return true;
    }

    if (resp.status != 200) {
        fprintf(stderr, "tile_fetch: HTTP %d for %s\n", resp.status, req_path);
        free(resp.body);
        cb(false, NULL, 0, userdata);
        return true;
    }

    if (resp.brotli) {
        /* .arpt wire format: Brotli-compressed FlatBuffer */
        uint8_t *decoded = NULL;
        size_t decoded_size = 0;
        if (arpt_decode(resp.body, resp.body_size, &decoded, &decoded_size)) {
            cb(true, decoded, decoded_size, userdata);
            free(decoded);
        } else {
            fprintf(stderr, "tile_fetch: arpt_decode failed\n");
            cb(false, NULL, 0, userdata);
        }
    } else {
        /* Raw FlatBuffer (no transport compression) */
        int rc = arpentry_tiles_Tile_verify_as_root_with_identifier(
            resp.body, resp.body_size, "arpt");
        if (rc == 0) {
            cb(true, resp.body, resp.body_size, userdata);
        } else {
            fprintf(stderr, "tile_fetch: verification failed (rc=%d)\n", rc);
            cb(false, NULL, 0, userdata);
        }
    }

    free(resp.body);
    return true;
}

#endif
