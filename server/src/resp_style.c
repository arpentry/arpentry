#include "resp_style.h"
#include "style_builder.h"
#include "json.h"
#include "tile.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BROTLI_QUALITY 4

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

bool resp_build_style(const char *style_file, uint8_t **out,
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

            struct json jtype = json_object_get(jlayer, "type");
            if (json_exists(jtype)) {
                char type_buf[32];
                json_string_copy(jtype, type_buf, sizeof(type_buf));
                arpentry_tiles_LayerType_enum_t lt =
                    arpentry_tiles_LayerType_Texture;
                if (strcmp(type_buf, "terrain") == 0)
                    lt = arpentry_tiles_LayerType_Terrain;
                else if (strcmp(type_buf, "texture") == 0)
                    lt = arpentry_tiles_LayerType_Texture;
                else if (strcmp(type_buf, "extrusion") == 0)
                    lt = arpentry_tiles_LayerType_Extrusion;
                else if (strcmp(type_buf, "instance") == 0)
                    lt = arpentry_tiles_LayerType_Instance;
                else if (strcmp(type_buf, "label") == 0)
                    lt = arpentry_tiles_LayerType_Label;
                else if (strcmp(type_buf, "line") == 0)
                    lt = arpentry_tiles_LayerType_Line;
                arpentry_tiles_LayerStyle_type_add(&builder, lt);
            }

            struct json jminlvl = json_object_get(jlayer, "min_level");
            if (json_exists(jminlvl))
                arpentry_tiles_LayerStyle_min_level_add(
                    &builder, (uint8_t)json_int(jminlvl));

            struct json jtext_size = json_object_get(jlayer, "text_size");
            if (json_exists(jtext_size))
                arpentry_tiles_LayerStyle_text_size_add(
                    &builder, (float)json_double(jtext_size));

            struct json jtext_color = json_object_get(jlayer, "text_color");
            if (json_exists(jtext_color)) {
                arpentry_tiles_RGBA_t c = parse_rgba(jtext_color);
                arpentry_tiles_LayerStyle_text_color_add(&builder, &c);
            }

            struct json jtext_halo_color = json_object_get(jlayer, "text_halo_color");
            if (json_exists(jtext_halo_color)) {
                arpentry_tiles_RGBA_t c = parse_rgba(jtext_halo_color);
                arpentry_tiles_LayerStyle_text_halo_color_add(&builder, &c);
            }

            struct json jtext_halo_width = json_object_get(jlayer, "text_halo_width");
            if (json_exists(jtext_halo_width))
                arpentry_tiles_LayerStyle_text_halo_width_add(
                    &builder, (float)json_double(jtext_halo_width));

            struct json jicon_size = json_object_get(jlayer, "icon_size");
            if (json_exists(jicon_size))
                arpentry_tiles_LayerStyle_icon_size_add(
                    &builder, (float)json_double(jicon_size));

            struct json jicon_color = json_object_get(jlayer, "icon_color");
            if (json_exists(jicon_color)) {
                arpentry_tiles_RGBA_t c = parse_rgba(jicon_color);
                arpentry_tiles_LayerStyle_icon_color_add(&builder, &c);
            }

            struct json jicon_halo_color = json_object_get(jlayer, "icon_halo_color");
            if (json_exists(jicon_halo_color)) {
                arpentry_tiles_RGBA_t c = parse_rgba(jicon_halo_color);
                arpentry_tiles_LayerStyle_icon_halo_color_add(&builder, &c);
            }

            struct json jicon_halo_width = json_object_get(jlayer, "icon_halo_width");
            if (json_exists(jicon_halo_width))
                arpentry_tiles_LayerStyle_icon_halo_width_add(
                    &builder, (float)json_double(jicon_halo_width));

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

                    struct json jminlvl_p = json_object_get(jentry, "min_level");
                    if (json_exists(jminlvl_p))
                        arpentry_tiles_PaintEntry_min_level_add(
                            &builder, (uint8_t)json_int(jminlvl_p));

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
