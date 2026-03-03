#include "http.h"
#include "net.h"
#include "gen/world.h"
#include "tile_path.h"
#include "tile.h"
#include "tileset_builder.h"
#include "style_builder.h"
#include "json.h"

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

/* Build a Brotli-compressed Tileset FlatBuffer. Caller frees *out. */

#define BROTLI_QUALITY 4

static bool build_tileset(uint8_t **out, size_t *out_size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_Tileset_start_as_root(&builder);
    arpentry_tiles_Tileset_version_add(&builder, 1);
    arpentry_tiles_Tileset_name_create_str(&builder, "Generated Terrain");

    arpentry_tiles_Bounds_t bounds = {
        .west = -180.0, .south = -90.0, .east = 180.0, .north = 90.0};
    arpentry_tiles_Tileset_bounds_add(&builder, &bounds);

    arpentry_tiles_ElevationRange_t elev = {.min = -500.0, .max = 4800.0};
    arpentry_tiles_Tileset_elevation_range_add(&builder, &elev);

    arpentry_tiles_Tileset_min_level_add(&builder, 0);
    arpentry_tiles_Tileset_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_root_error_add(&builder, 200000.0);

    /* Layers in decode-priority order (Section 9) */
    arpentry_tiles_Tileset_layers_start(&builder);

    /* terrain: Mesh, 0-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "terrain");
    arpentry_tiles_GeometryType_enum_t mesh_types[] = {
        arpentry_tiles_GeometryType_Mesh};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, mesh_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 0);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* surface: Polygon, 0-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "surface");
    arpentry_tiles_GeometryType_enum_t poly_types[] = {
        arpentry_tiles_GeometryType_Polygon};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, poly_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 0);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* highway: Line, 8-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "highway");
    arpentry_tiles_GeometryType_enum_t line_types[] = {
        arpentry_tiles_GeometryType_Line};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, line_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 8);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    /* building: Polygon, 13-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "building");
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, poly_types, 1);
    arpentry_tiles_LayerInfo_min_level_add(&builder, 13);
    arpentry_tiles_LayerInfo_max_level_add(&builder, 19);
    arpentry_tiles_Tileset_layers_push_end(&builder);

    arpentry_tiles_Tileset_layers_end(&builder);
    arpentry_tiles_Tileset_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);
    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;
}

/* Load style.json from tile_dir and build a Brotli-compressed Style
   FlatBuffer. Caller frees *out. Reloads the file on every request so
   edits take effect without restarting the server. */

static char *read_file(const char *path, size_t *out_size) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsize <= 0) { fclose(f); return NULL; }
    char *buf = malloc((size_t)fsize + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t nread = fread(buf, 1, (size_t)fsize, f);
    fclose(f);
    buf[nread] = '\0';
    *out_size = nread;
    return buf;
}

/* Parse a JSON RGBA array [r, g, b, a] into an arpentry_tiles_RGBA_t. */
static arpentry_tiles_RGBA_t parse_rgba(struct json arr) {
    arpentry_tiles_RGBA_t c = {0, 0, 0, 255};
    struct json el = json_first(arr);
    if (json_exists(el)) { c.r = (uint8_t)json_int(el); el = json_next(el); }
    if (json_exists(el)) { c.g = (uint8_t)json_int(el); el = json_next(el); }
    if (json_exists(el)) { c.b = (uint8_t)json_int(el); el = json_next(el); }
    if (json_exists(el)) { c.a = (uint8_t)json_int(el); }
    return c;
}

static bool build_style(const char *tile_dir, uint8_t **out, size_t *out_size) {
    /* Build path to style.json */
    char path[1024];
    int n = snprintf(path, sizeof(path), "%s/style.json", tile_dir);
    if (n < 0 || (size_t)n >= sizeof(path)) return false;

    size_t json_size;
    char *json_str = read_file(path, &json_size);
    if (!json_str) {
        fprintf(stderr, "style: cannot read %s\n", path);
        return false;
    }

    struct json root = json_parse(json_str);
    if (!json_exists(root)) {
        fprintf(stderr, "style: invalid JSON in %s\n", path);
        free(json_str);
        return false;
    }

    flatcc_builder_t builder;
    flatcc_builder_init(&builder);
    arpentry_tiles_Style_start_as_root(&builder);

    /* version */
    struct json jver = json_object_get(root, "version");
    arpentry_tiles_Style_version_add(
        &builder, json_exists(jver) ? (uint16_t)json_int(jver) : 1);

    /* name */
    struct json jname = json_object_get(root, "name");
    if (json_exists(jname)) {
        char name_buf[256];
        json_string_copy(jname, name_buf, sizeof(name_buf));
        arpentry_tiles_Style_name_create_str(&builder, name_buf);
    }

    /* background */
    struct json jbg = json_object_get(root, "background");
    if (json_exists(jbg)) {
        arpentry_tiles_RGBA_t bg = parse_rgba(jbg);
        arpentry_tiles_Style_background_add(&builder, &bg);
    }

    /* building */
    struct json jbldg = json_object_get(root, "building");
    if (json_exists(jbldg)) {
        arpentry_tiles_RGBA_t bldg = parse_rgba(jbldg);
        arpentry_tiles_Style_building_add(&builder, &bldg);
    }

    /* layers */
    struct json jlayers = json_object_get(root, "layers");
    if (json_exists(jlayers)) {
        arpentry_tiles_Style_layers_start(&builder);
        struct json jlayer = json_first(jlayers);
        while (json_exists(jlayer)) {
            arpentry_tiles_Style_layers_push_start(&builder);

            struct json jsl = json_object_get(jlayer, "source_layer");
            if (json_exists(jsl)) {
                char sl_buf[128];
                json_string_copy(jsl, sl_buf, sizeof(sl_buf));
                arpentry_tiles_LayerStyle_source_layer_create_str(&builder,
                                                                   sl_buf);
            }

            struct json jpaint = json_object_get(jlayer, "paint");
            if (json_exists(jpaint)) {
                arpentry_tiles_LayerStyle_paint_start(&builder);
                struct json jentry = json_first(jpaint);
                while (json_exists(jentry)) {
                    arpentry_tiles_LayerStyle_paint_push_start(&builder);

                    struct json jcls = json_object_get(jentry, "class");
                    if (json_exists(jcls)) {
                        char cls_buf[128];
                        json_string_copy(jcls, cls_buf, sizeof(cls_buf));
                        arpentry_tiles_PaintEntry_class_create_str(&builder,
                                                                    cls_buf);
                    }

                    struct json jcolor = json_object_get(jentry, "color");
                    if (json_exists(jcolor)) {
                        arpentry_tiles_RGBA_t c = parse_rgba(jcolor);
                        arpentry_tiles_PaintEntry_color_add(&builder, &c);
                    }

                    struct json jwidth = json_object_get(jentry, "width");
                    if (json_exists(jwidth))
                        arpentry_tiles_PaintEntry_width_add(
                            &builder, (float)json_double(jwidth));

                    arpentry_tiles_LayerStyle_paint_push_end(&builder);
                    jentry = json_next(jentry);
                }
                arpentry_tiles_LayerStyle_paint_end(&builder);
            }

            arpentry_tiles_Style_layers_push_end(&builder);
            jlayer = json_next(jlayer);
        }
        arpentry_tiles_Style_layers_end(&builder);
    }

    arpentry_tiles_Style_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);
    free(json_str);
    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;
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
    if (strcmp(uri, "/tileset.arts") == 0) {
        uint8_t *arts_data = NULL;
        size_t arts_size = 0;
        if (build_tileset(&arts_data, &arts_size)) {
            write_response(conn, 200, "application/x-arts", "br", arts_data,
                           arts_size);
            free(arts_data);
        } else {
            write_error(conn, 500);
        }
        return;
    }

    /* Style */
    if (strcmp(uri, "/style.arss") == 0) {
        uint8_t *arss_data = NULL;
        size_t arss_size = 0;
        if (build_style(ctx->tile_dir, &arss_data, &arss_size)) {
            write_response(conn, 200, "application/x-arss", "br", arss_data,
                           arss_size);
            free(arss_data);
        } else {
            write_error(conn, 500);
        }
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
