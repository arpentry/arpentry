#include "http.h"
#include "net.h"
#include "gen/world.h"
#include "tile_path.h"
#include "tile.h"
#include "tileset_builder.h"
#include "style_builder.h"
#include "model_builder.h"
#include "json.h"

#include <math.h>

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

    /* tree: Point, 13-19 */
    arpentry_tiles_Tileset_layers_push_start(&builder);
    arpentry_tiles_LayerInfo_name_create_str(&builder, "tree");
    arpentry_tiles_GeometryType_enum_t point_types[] = {
        arpentry_tiles_GeometryType_Point};
    arpentry_tiles_LayerInfo_geometry_types_create(&builder, point_types, 1);
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

/* Load style.json and build a Brotli-compressed Style FlatBuffer.
   Caller frees *out. Reloads the file on every request so edits take
   effect without restarting the server. */

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

static bool build_style(const char *style_file, uint8_t **out,
                        size_t *out_size) {
    size_t json_size;
    char *json_str = read_file(style_file, &json_size);
    if (!json_str) {
        fprintf(stderr, "style: cannot read %s\n", style_file);
        return false;
    }

    struct json root = json_parse(json_str);
    if (!json_exists(root)) {
        fprintf(stderr, "style: invalid JSON in %s\n", style_file);
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

                    struct json jmodel = json_object_get(jentry, "model");
                    if (json_exists(jmodel)) {
                        char model_buf[128];
                        json_string_copy(jmodel, model_buf, sizeof(model_buf));
                        arpentry_tiles_PaintEntry_model_create_str(&builder,
                                                                    model_buf);
                    }

                    struct json jmin_s = json_object_get(jentry, "min_scale");
                    if (json_exists(jmin_s))
                        arpentry_tiles_PaintEntry_min_scale_add(
                            &builder, (float)json_double(jmin_s));

                    struct json jmax_s = json_object_get(jentry, "max_scale");
                    if (json_exists(jmax_s))
                        arpentry_tiles_PaintEntry_max_scale_add(
                            &builder, (float)json_double(jmax_s));

                    struct json jryaw = json_object_get(jentry, "random_yaw");
                    if (json_exists(jryaw))
                        arpentry_tiles_PaintEntry_random_yaw_add(
                            &builder, json_bool(jryaw));

                    struct json jrscale = json_object_get(jentry, "random_scale");
                    if (json_exists(jrscale))
                        arpentry_tiles_PaintEntry_random_scale_add(
                            &builder, json_bool(jrscale));

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

/* Build a Brotli-compressed ModelLibrary FlatBuffer with 3 tree models
   (oak, pine, birch), each with a trunk cylinder + crown shape.
   The w component of uint16x4 vertex data stores part index (0=trunk, 1=crown).
   Caller frees *out. */

#define SIDES 8
#define CX 5000 /* model center x/y in mm */
#define CY 5000

/* Generate an 8-sided cylinder. Returns number of vertices written.
   part_idx is stored for later packing into the w component. */
static int gen_cylinder(uint16_t *vx, uint16_t *vy, uint16_t *vz,
                        uint16_t *vw, uint32_t *indices, int vi, int ii,
                        uint16_t radius, uint16_t z_bot, uint16_t z_top,
                        uint16_t part_idx) {
    int base = vi;
    /* Bottom ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_bot;
        vw[vi] = part_idx;
        vi++;
    }
    /* Top ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_top;
        vw[vi] = part_idx;
        vi++;
    }
    int bot0 = base;
    int top0 = base + SIDES;
    /* Side quads (2 tris each) */
    for (int i = 0; i < SIDES; i++) {
        int n = (i + 1) % SIDES;
        indices[ii++] = (uint32_t)(bot0 + i);
        indices[ii++] = (uint32_t)(bot0 + n);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(bot0 + n);
        indices[ii++] = (uint32_t)(top0 + n);
    }
    /* Bottom cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)(bot0);
        indices[ii++] = (uint32_t)(bot0 + i + 1);
        indices[ii++] = (uint32_t)(bot0 + i);
    }
    /* Top cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)(top0);
        indices[ii++] = (uint32_t)(top0 + i);
        indices[ii++] = (uint32_t)(top0 + i + 1);
    }
    (void)vi; /* suppress unused */
    return ii; /* return updated index position */
}

/* Generate an 8-sided cone. Returns updated index count. */
static int gen_cone(uint16_t *vx, uint16_t *vy, uint16_t *vz, uint16_t *vw,
                    uint32_t *indices, int vi, int ii, uint16_t radius,
                    uint16_t z_base, uint16_t z_apex, uint16_t part_idx,
                    int *out_vi) {
    int base = vi;
    /* Base ring */
    for (int i = 0; i < SIDES; i++) {
        double a = 2.0 * M_PI * i / SIDES;
        vx[vi] = (uint16_t)(CX + radius * cos(a));
        vy[vi] = (uint16_t)(CY + radius * sin(a));
        vz[vi] = z_base;
        vw[vi] = part_idx;
        vi++;
    }
    /* Apex */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = z_apex;
    vw[vi] = part_idx;
    int apex = vi;
    vi++;
    /* Side triangles */
    for (int i = 0; i < SIDES; i++) {
        int n = (i + 1) % SIDES;
        indices[ii++] = (uint32_t)apex;
        indices[ii++] = (uint32_t)(base + i);
        indices[ii++] = (uint32_t)(base + n);
    }
    /* Base cap */
    for (int i = 1; i < SIDES - 1; i++) {
        indices[ii++] = (uint32_t)base;
        indices[ii++] = (uint32_t)(base + i + 1);
        indices[ii++] = (uint32_t)(base + i);
    }
    *out_vi = vi;
    return ii;
}

/* Generate a UV sphere approximation. Returns updated index count. */
#define SPHERE_LAT 4
#define SPHERE_LON 8

static int gen_sphere(uint16_t *vx, uint16_t *vy, uint16_t *vz, uint16_t *vw,
                      uint32_t *indices, int vi, int ii, uint16_t radius,
                      uint16_t cz, uint16_t part_idx, int *out_vi) {
    int base = vi;
    /* Top pole */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = (uint16_t)(cz + radius);
    vw[vi] = part_idx;
    vi++;
    /* Latitude rings */
    for (int lat = 1; lat < SPHERE_LAT; lat++) {
        double phi = M_PI * lat / SPHERE_LAT;
        double sp = sin(phi);
        double cp = cos(phi);
        for (int lon = 0; lon < SPHERE_LON; lon++) {
            double theta = 2.0 * M_PI * lon / SPHERE_LON;
            vx[vi] = (uint16_t)(CX + radius * sp * cos(theta));
            vy[vi] = (uint16_t)(CY + radius * sp * sin(theta));
            vz[vi] = (uint16_t)(cz + (int16_t)(radius * cp));
            vw[vi] = part_idx;
            vi++;
        }
    }
    /* Bottom pole */
    vx[vi] = CX;
    vy[vi] = CY;
    vz[vi] = (uint16_t)(cz - radius);
    vw[vi] = part_idx;
    int bot_pole = vi;
    vi++;

    int top_pole = base;
    /* Top cap triangles */
    for (int i = 0; i < SPHERE_LON; i++) {
        int n = (i + 1) % SPHERE_LON;
        indices[ii++] = (uint32_t)top_pole;
        indices[ii++] = (uint32_t)(base + 1 + i);
        indices[ii++] = (uint32_t)(base + 1 + n);
    }
    /* Middle quads */
    for (int lat = 0; lat < SPHERE_LAT - 2; lat++) {
        int row0 = base + 1 + lat * SPHERE_LON;
        int row1 = row0 + SPHERE_LON;
        for (int i = 0; i < SPHERE_LON; i++) {
            int n = (i + 1) % SPHERE_LON;
            indices[ii++] = (uint32_t)(row0 + i);
            indices[ii++] = (uint32_t)(row1 + i);
            indices[ii++] = (uint32_t)(row0 + n);
            indices[ii++] = (uint32_t)(row0 + n);
            indices[ii++] = (uint32_t)(row1 + i);
            indices[ii++] = (uint32_t)(row1 + n);
        }
    }
    /* Bottom cap triangles */
    int last_row = base + 1 + (SPHERE_LAT - 2) * SPHERE_LON;
    for (int i = 0; i < SPHERE_LON; i++) {
        int n = (i + 1) % SPHERE_LON;
        indices[ii++] = (uint32_t)bot_pole;
        indices[ii++] = (uint32_t)(last_row + n);
        indices[ii++] = (uint32_t)(last_row + i);
    }
    *out_vi = vi;
    return ii;
}

/* Write one model into the builder: name, x/y/z/w arrays, indices, 2 Parts. */
static void emit_model(flatcc_builder_t *b, const char *name,
                        const uint16_t *vx, const uint16_t *vy,
                        const uint16_t *vz, const uint16_t *vw, int nv,
                        const uint32_t *indices, int ni, int trunk_last_idx,
                        arpentry_tiles_Color_t trunk_col,
                        arpentry_tiles_Color_t crown_col) {
    arpentry_tiles_ModelLibrary_models_push_start(b);
    arpentry_tiles_Model_name_create_str(b, name);

    arpentry_tiles_Model_x_create(b, vx, (size_t)nv);
    arpentry_tiles_Model_y_create(b, vy, (size_t)nv);
    arpentry_tiles_Model_z_create(b, vz, (size_t)nv);
    arpentry_tiles_Model_w_create(b, vw, (size_t)nv);
    arpentry_tiles_Model_indices_create(b, indices, (size_t)ni);

    arpentry_tiles_Model_parts_start(b);
    arpentry_tiles_Part_t trunk_part = {
        .first_index = 0,
        .index_count = (uint32_t)trunk_last_idx,
        .roughness = 200,
        .metalness = 0,
    };
    trunk_part.color = trunk_col;
    arpentry_tiles_Model_parts_push(b, &trunk_part);
    arpentry_tiles_Part_t crown_part = {
        .first_index = (uint32_t)trunk_last_idx,
        .index_count = (uint32_t)(ni - trunk_last_idx),
        .roughness = 200,
        .metalness = 0,
    };
    crown_part.color = crown_col;
    arpentry_tiles_Model_parts_push(b, &crown_part);
    arpentry_tiles_Model_parts_end(b);

    arpentry_tiles_ModelLibrary_models_push_end(b);
}

/* Max vertices/indices per model: trunk(16v) + sphere(26v) = 42v, ~600 idx */
#define MAX_MODEL_V 64
#define MAX_MODEL_I 1024

static bool build_models(uint8_t **out, size_t *out_size) {
    flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    arpentry_tiles_ModelLibrary_start_as_root(&builder);
    arpentry_tiles_ModelLibrary_version_add(&builder, 1);
    arpentry_tiles_ModelLibrary_models_start(&builder);

    uint16_t vx[MAX_MODEL_V], vy[MAX_MODEL_V], vz[MAX_MODEL_V],
        vw[MAX_MODEL_V];
    uint32_t indices[MAX_MODEL_I];

    /* --- Oak: short trunk (2m, r=400mm) + wide sphere crown (r=6m, cz=8m) */
    {
        int vi = 0, ii = 0;
        /* Trunk: bottom ring 0-7, top ring 8-15 */
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 400, 0, 2000, 0);
        vi = SIDES * 2;
        int trunk_ii = ii;
        /* Crown sphere at z=8000, radius=6000 */
        ii = gen_sphere(vx, vy, vz, vw, indices, vi, ii, 6000, 8000, 1, &vi);
        arpentry_tiles_Color_t trunk = {.r = 101, .g = 67, .b = 33, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 34, .g = 85, .b = 25, .a = 255};
        emit_model(&builder, "oak", vx, vy, vz, vw, vi, indices, ii, trunk_ii,
                   trunk, crown);
    }

    /* --- Pine: medium trunk (4m, r=300mm) + tall cone crown (base 4m, apex 18m, r=3m) */
    {
        int vi = 0, ii = 0;
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 300, 0, 4000, 0);
        vi = SIDES * 2;
        int trunk_ii = ii;
        ii = gen_cone(vx, vy, vz, vw, indices, vi, ii, 3000, 4000, 18000, 1,
                      &vi);
        arpentry_tiles_Color_t trunk = {.r = 101, .g = 67, .b = 33, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 20, .g = 70, .b = 20, .a = 255};
        emit_model(&builder, "pine", vx, vy, vz, vw, vi, indices, ii,
                   trunk_ii, trunk, crown);
    }

    /* --- Birch: slender trunk (5m, r=200mm) + small sphere crown (r=3m, cz=10m) */
    {
        int vi = 0, ii = 0;
        ii = gen_cylinder(vx, vy, vz, vw, indices, vi, ii, 200, 0, 5000, 0);
        vi = SIDES * 2;
        int trunk_ii = ii;
        ii = gen_sphere(vx, vy, vz, vw, indices, vi, ii, 3000, 10000, 1, &vi);
        arpentry_tiles_Color_t trunk = {.r = 200, .g = 200, .b = 195, .a = 255};
        arpentry_tiles_Color_t crown = {.r = 80, .g = 160, .b = 50, .a = 255};
        emit_model(&builder, "birch", vx, vy, vz, vw, vi, indices, ii,
                   trunk_ii, trunk, crown);
    }

    arpentry_tiles_ModelLibrary_models_end(&builder);
    arpentry_tiles_ModelLibrary_end_as_root(&builder);

    size_t fb_size;
    void *fb = flatcc_builder_finalize_buffer(&builder, &fb_size);
    flatcc_builder_clear(&builder);
    if (!fb) return false;

    bool ok = arpt_encode(fb, fb_size, out, out_size, BROTLI_QUALITY);
    free(fb);
    return ok;
}

#undef SIDES
#undef CX
#undef CY
#undef SPHERE_LAT
#undef SPHERE_LON
#undef MAX_MODEL_V
#undef MAX_MODEL_I

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
        if (build_style(ctx->style_file, &arss_data, &arss_size)) {
            write_response(conn, 200, "application/x-arss", "br", arss_data,
                           arss_size);
            free(arss_data);
        } else {
            write_error(conn, 500);
        }
        return;
    }

    /* Models */
    if (strcmp(uri, "/models.arpm") == 0) {
        uint8_t *arpm_data = NULL;
        size_t arpm_size = 0;
        if (build_models(&arpm_data, &arpm_size)) {
            write_response(conn, 200, "application/x-arpm", "br", arpm_data,
                           arpm_size);
            free(arpm_data);
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
