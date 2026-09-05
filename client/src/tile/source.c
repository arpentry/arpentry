#include "source.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tile.h" /* arpt_brotli_decode */

#ifndef __EMSCRIPTEN__

#include <sys/stat.h>

#include "archive.h"
#include "http.h"

/* One archive per process, opened before any worker starts and closed after
   they all stop, so the workers may read it without a lock: nothing mutates
   it and mmap'd pages are shared. */
static arpt_archive *g_archive;
/* Directory holding the archive, where the style and model sidecars live. */
static char g_archive_dir[512];
/* The archive's own path, and the size and mtime it had when opened — what
   `arpt_source_archive_changed` compares against so a re-tile is noticed. */
static char g_archive_path[512];
static long long g_archive_size;
static long long g_archive_mtime;

/* Stat the archive, or leave the outputs at zero when it cannot be read. A
   file being rewritten is momentarily absent under some editors and tools;
   reporting zero makes that "changed", and the reload that follows fails
   safely rather than acting on half a file. */
static void archive_stamp(const char *path, long long *size, long long *mtime) {
    struct stat st;
    if (path && *path && stat(path, &st) == 0) {
        *size = (long long)st.st_size;
        *mtime = (long long)st.st_mtime;
    } else {
        *size = 0;
        *mtime = 0;
    }
}

bool arpt_source_open_archive(const char *path) {
    if (!path || !*path) return true;

    arpt_archive *a = arpt_archive_open(path);
    if (!a) return false;

    arpt_source_close();
    g_archive = a;

    /* Remember the containing directory; "" means the working directory. */
    const char *slash = strrchr(path, '/');
    size_t dir_len = slash ? (size_t)(slash - path) : 0;
    if (dir_len >= sizeof(g_archive_dir)) dir_len = sizeof(g_archive_dir) - 1;
    memcpy(g_archive_dir, path, dir_len);
    g_archive_dir[dir_len] = '\0';

    size_t path_len = strlen(path);
    if (path_len >= sizeof(g_archive_path)) path_len = sizeof(g_archive_path) - 1;
    memcpy(g_archive_path, path, path_len);
    g_archive_path[path_len] = '\0';
    archive_stamp(g_archive_path, &g_archive_size, &g_archive_mtime);

    printf("[SOURCE] archive %s (%llu tiles)\n", path,
           (unsigned long long)arpt_archive_tile_count(a));
    return true;
}

bool arpt_source_archive_changed(void) {
    if (!g_archive || !g_archive_path[0]) return false;
    long long size = 0, mtime = 0;
    archive_stamp(g_archive_path, &size, &mtime);
    return size != g_archive_size || mtime != g_archive_mtime;
}

bool arpt_source_reload_archive(void) {
    if (!g_archive || !g_archive_path[0]) return false;

    /* Open the new one *before* releasing the old: a failed reopen must leave
       the viewer drawing what it was drawing, not nothing. */
    arpt_archive *fresh = arpt_archive_open(g_archive_path);
    if (!fresh) return false;

    arpt_archive_free(g_archive);
    g_archive = fresh;
    archive_stamp(g_archive_path, &g_archive_size, &g_archive_mtime);

    printf("[SOURCE] archive reloaded: %s (%llu tiles)\n", g_archive_path,
           (unsigned long long)arpt_archive_tile_count(g_archive));
    return true;
}

void arpt_source_close(void) {
    if (!g_archive) return;
    arpt_archive_free(g_archive);
    g_archive = NULL;
    g_archive_path[0] = '\0';
    g_archive_size = 0;
    g_archive_mtime = 0;
}

bool arpt_source_is_archive(void) { return g_archive != NULL; }

/* Copy out and decompress a stored blob. Blobs in an archive are Brotli, the
   same as a server's `Content-Encoding: br` response, so callers see one
   shape either way. */
static bool take_blob(const uint8_t *blob, size_t blob_size, uint8_t **body,
                      size_t *size) {
    return arpt_brotli_decode(blob, blob_size, body, size);
}

/* Read a sidecar file beside the archive into a fresh buffer. */
static bool read_sidecar(const char *name, uint8_t **body, size_t *size) {
    char path[640];
    int n = snprintf(path, sizeof(path), "%s%s%s", g_archive_dir,
                     g_archive_dir[0] ? "" : ".", name);
    if (n < 0 || (size_t)n >= sizeof(path)) return false;

    FILE *f = fopen(path, "rb");
    if (!f) return false;

    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return false; }
    long end = ftell(f);
    if (end < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return false; }

    size_t raw_size = (size_t)end;
    uint8_t *raw = malloc(raw_size ? raw_size : 1);
    if (!raw) { fclose(f); return false; }
    size_t got = fread(raw, 1, raw_size, f);
    fclose(f);
    if (got != raw_size) { free(raw); return false; }

    /* Sidecars are written compressed, as the server sends them, but a hand-
       made one may not be. Fall back to the bytes as they are. */
    if (arpt_brotli_decode(raw, raw_size, body, size)) {
        free(raw);
        return true;
    }
    *body = raw;
    *size = raw_size;
    return true;
}

/* Parse "/{z}/{x}/{y}.arpt". Returns false for any other name. */
static bool parse_tile_name(const char *name, int *z, int *x, int *y) {
    int n = 0;
    if (sscanf(name, "/%d/%d/%d.arpt%n", z, x, y, &n) != 3) return false;
    return name[n] == '\0' && *z >= 0 && *x >= 0 && *y >= 0;
}

bool arpt_source_get(const char *base, const char *name, uint8_t **body,
                     size_t *size) {
    if (!name || !body || !size) return false;

    if (g_archive) {
        int z, x, y;
        if (parse_tile_name(name, &z, &x, &y)) {
            const uint8_t *blob;
            size_t blob_size;
            if (!arpt_archive_tile(g_archive, z, x, y, &blob, &blob_size))
                return false;
            return take_blob(blob, blob_size, body, size);
        }
        if (strcmp(name, "/index.arpi") == 0) {
            const uint8_t *blob;
            size_t blob_size;
            if (!arpt_archive_metadata(g_archive, &blob, &blob_size))
                return false;
            return take_blob(blob, blob_size, body, size);
        }
        /* The style and the model library are not the archive's to hold —
           they describe how to draw it, not what it contains — so they sit
           beside it. `arpentry_server --bundle` writes both. */
        return read_sidecar(name, body, size);
    }

    if (!base) return false;
    char url[768];
    int n = snprintf(url, sizeof(url), "%s%s", base, name);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    arpt_http_response resp = {0};
    if (!arpt_http_get(url, &resp)) return false;
    if (resp.status != 200) {
        free(resp.body);
        return false;
    }
    *body = resp.body;
    *size = resp.body_size;
    return true;
}

#else /* Emscripten: no local files, so every source is the server. */

bool arpt_source_open_archive(const char *path) { return path == NULL; }
void arpt_source_close(void) {}
bool arpt_source_is_archive(void) { return false; }
bool arpt_source_archive_changed(void) { return false; }
bool arpt_source_reload_archive(void) { return false; }

bool arpt_source_get(const char *base, const char *name, uint8_t **body,
                     size_t *size) {
    (void)base; (void)name; (void)body; (void)size;
    return false; /* the browser path fetches asynchronously; see fetch.c */
}

#endif /* !__EMSCRIPTEN__ */
