#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <webgpu/webgpu.h>
#include <GLFW/glfw3.h>
#include "glfw3webgpu.h"

#include "camera.h"
#include "control.h"
#include "renderer.h"
#include "tile/prepare.h"
#include "style.h"
#include "tile/manager.h"
#include "ui.h"
#include "screenshot.h"
#include "style_reader.h"
#include "style_verifier.h"
#include "tileset_reader.h"
#include "tileset_verifier.h"
#include "model_reader.h"
#include "model_verifier.h"

#ifndef __EMSCRIPTEN__
#include "http.h"
#include "tile.h"
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h> /* emscripten_get_element_css_size */
#include <emscripten/fetch.h>
#endif

/* Constants */

#define INITIAL_ALTITUDE 500000.0
#define WINDOW_W 800
#define WINDOW_H 600

/* CLI options (native only) */

#ifndef __EMSCRIPTEN__
typedef struct {
    char url[256];
    double lon;     /* degrees */
    double lat;     /* degrees */
    double alt;     /* meters */
    double bearing; /* degrees */
    double tilt;    /* degrees */
    int width;
    int height;
    char screenshot[512]; /* empty string = interactive mode */
} cli_opts;

static cli_opts opts = {
    .url = "http://localhost:8090",
    .lon = 0.0,
    .lat = 0.0,
    .alt = INITIAL_ALTITUDE,
    .bearing = 0.0,
    .tilt = 0.0,
    .width = WINDOW_W,
    .height = WINDOW_H,
    .screenshot = "",
};

static void parse_args(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--url") == 0 && i + 1 < argc) {
            snprintf(opts.url, sizeof(opts.url), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--lon") == 0 && i + 1 < argc) {
            opts.lon = atof(argv[++i]);
        } else if (strcmp(argv[i], "--lat") == 0 && i + 1 < argc) {
            opts.lat = atof(argv[++i]);
        } else if (strcmp(argv[i], "--alt") == 0 && i + 1 < argc) {
            opts.alt = atof(argv[++i]);
        } else if (strcmp(argv[i], "--bearing") == 0 && i + 1 < argc) {
            opts.bearing = atof(argv[++i]);
        } else if (strcmp(argv[i], "--tilt") == 0 && i + 1 < argc) {
            opts.tilt = atof(argv[++i]);
        } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
            opts.width = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
            opts.height = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            snprintf(opts.screenshot, sizeof(opts.screenshot), "%s",
                     argv[++i]);
        } else {
            fprintf(stderr,
                    "Usage: %s [--url <base>] [--lon <deg>] [--lat <deg>] "
                    "[--alt <m>] [--bearing <deg>] [--tilt <deg>] "
                    "[--width <px>] [--height <px>] [--screenshot <path>]\n",
                    argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}
#endif /* __EMSCRIPTEN__ */

/* App state */

typedef struct {
    GLFWwindow *window;
    WGPUInstance instance;
    WGPUSurface surface;
    WGPUAdapter adapter;
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat surface_format;

    arpt_camera *camera;
    arpt_control *control;
    arpt_renderer *renderer;
    arpt_tile_manager *tile_manager;
    arpt_ui *ui;
    double last_time;
    double smoothed_ground_elev;
    bool needs_redraw;
    char base_url[256];
    uint8_t *model_buf; /* kept alive for zero-copy model pointers */

#ifndef __EMSCRIPTEN__
    bool capture_next;
    bool screenshot_ok;
    int idle_frames; /* consecutive frames with 0 active fetches */
    int exit_code;
#endif
} App;

static App app = {0};

/* Forward declarations for refresh logic */
static bool fetch_tileset(const char *base_url,
                           arpt_tile_manager_config *config);
static bool fetch_style(const char *base_url, arpt_style *style);
static int fetch_models(const char *base_url, arpt_model *models,
                         int max_models, uint8_t **model_buf_out);
static void ui_overlay(WGPURenderPassEncoder pass, void *ud);

/* GLFW callbacks */

static void on_device_error(WGPUErrorType type, const char *msg, void *ud) {
    (void)ud;
    fprintf(stderr, "WebGPU device error (%d): %s\n", type, msg);
}

static void on_framebuffer_resize(GLFWwindow *w, int width, int height) {
    (void)w;
    if (width == 0 || height == 0) return;

    /* On web, sync_canvas_size() handles all resize logic each frame.
       This callback only needs to serve native targets. */
#ifdef __EMSCRIPTEN__
    (void)width;
    (void)height;
    return;
#else
    app.needs_redraw = true;
    /* On native: framebuffer pixels are physical; window pixels are logical
       and match glfwGetCursorPos. */
    int win_w, win_h;
    glfwGetWindowSize(app.window, &win_w, &win_h);
    arpt_camera_set_viewport(app.camera, win_w, win_h);

    arpt_renderer_resize(app.renderer, (uint32_t)width, (uint32_t)height);

    float ratio = (win_w > 0) ? (float)width / (float)win_w : 1.0f;
    if (app.ui)
        arpt_ui_resize(app.ui, (uint32_t)width, (uint32_t)height, ratio);

    WGPUSurfaceConfiguration cfg = {
        .device = app.device,
        .format = app.surface_format,
        .usage = WGPUTextureUsage_RenderAttachment,
        .width = (uint32_t)width,
        .height = (uint32_t)height,
        .presentMode = WGPUPresentMode_Fifo,
        .alphaMode = WGPUCompositeAlphaMode_Auto,
    };
    wgpuSurfaceConfigure(app.surface, &cfg);
#endif
}

/* Emscripten canvas sync */

#ifdef __EMSCRIPTEN__
static void sync_canvas_size(void) {
    /* Read CSS display size and DPR directly from JavaScript to avoid any
       discrepancy with the C-wrapper versions of these queries. */
    int css_w = EM_ASM_INT({
        return Math.round(
            document.getElementById('canvas').getBoundingClientRect().width);
    });
    int css_h = EM_ASM_INT({
        return Math.round(
            document.getElementById('canvas').getBoundingClientRect().height);
    });
    double dpr = EM_ASM_DOUBLE({ return window.devicePixelRatio || 1.0; });
    if (css_w == 0 || css_h == 0) return;

    int phys_w = (int)(css_w * dpr);
    int phys_h = (int)(css_h * dpr);

    /* Sync GLFW's internal window size to the actual CSS display size.
       glfwCreateWindow(800,600) leaves GLFW.active.width/height at 800×600
       even though the flex-layout canvas may be much larger.  Emscripten's
       GLFW3 port scales cursor coordinates by (GLFW.active.width / rect.width),
       so a stale value makes glfwGetCursorPos return wrong coordinates.
       Calling glfwSetWindowSize updates GLFW's internal bookkeeping (and
       briefly sets canvas.width = css_w); we then override the drawing
       buffer to physical pixels right after. */
    static int s_css_w, s_css_h;
    if (s_css_w != css_w || s_css_h != css_h) {
        s_css_w = css_w;
        s_css_h = css_h;
        glfwSetWindowSize(app.window, css_w, css_h);
    }

    /* Pin the canvas drawing buffer to physical pixels.  Browsers display the
       canvas at the CSS size regardless of canvas.width/height, so this gives
       crisp 1:1 physical-pixel rendering on HiDPI screens. */
    EM_ASM(
        {
            var c = document.getElementById('canvas');
            if (c.width !== $0 || c.height !== $1) {
                c.width = $0;
                c.height = $1;
            }
        },
        phys_w, phys_h);

    /* Camera viewport lives in CSS-pixel space so it matches glfwGetCursorPos
       (now correctly reporting CSS pixels after the glfwSetWindowSize above). */
    if (arpt_camera_vp_width(app.camera) != css_w ||
        arpt_camera_vp_height(app.camera) != css_h)
        arpt_camera_set_viewport(app.camera, css_w, css_h);

    /* Renderer and WebGPU surface use physical pixels. */
    static int s_phys_w, s_phys_h;
    if (s_phys_w != phys_w || s_phys_h != phys_h) {
        s_phys_w = phys_w;
        s_phys_h = phys_h;
        app.needs_redraw = true;
        arpt_renderer_resize(app.renderer, (uint32_t)phys_w, (uint32_t)phys_h);
        if (app.ui)
            arpt_ui_resize(app.ui, (uint32_t)phys_w, (uint32_t)phys_h,
                           (float)dpr);
        WGPUSurfaceConfiguration cfg = {
            .device = app.device,
            .format = app.surface_format,
            .usage = WGPUTextureUsage_RenderAttachment,
            .width = (uint32_t)phys_w,
            .height = (uint32_t)phys_h,
            .presentMode = WGPUPresentMode_Fifo,
            .alphaMode = WGPUCompositeAlphaMode_Auto,
        };
        wgpuSurfaceConfigure(app.surface, &cfg);
    }
}
#endif

/* Render frame */

static void render_frame(void) {
#ifdef __EMSCRIPTEN__
    glfwPollEvents();
    sync_canvas_size();
#else
    /* When idle, block instead of spinning to save CPU/battery.
       The 100ms timeout ensures tile fetch completions are polled. */
    if (app.needs_redraw)
        glfwPollEvents();
    else
        glfwWaitEventsTimeout(0.1);
#endif

    /* Compute dt and advance control */
    double now = glfwGetTime();
    double dt = now - app.last_time;
    app.last_time = now;
    if (app.control) arpt_control_update(app.control, dt);

    /* Drain tile fetch callbacks (lightweight even when idle) */
    if (app.tile_manager)
        arpt_tile_manager_update(app.tile_manager, app.camera);

    /* Handle full refresh (R key): re-fetch style/tileset, recreate renderer
       and tile manager so tiles are re-uploaded with new colors. */
    if (arpt_control_needs_refresh(app.control)) {
        arpt_style style;
        arpt_style_defaults(&style);
        fetch_style(app.base_url, &style);

        arpt_tile_manager_config tm_config = {
            .base_url = app.base_url,
            .root_error = 200000.0,
            .min_level = 0,
            .max_level = 19,
            .max_tiles = 200,
            .max_concurrent = 6,
        };
        fetch_tileset(app.base_url, &tm_config);

        /* Fetch model library */
        arpt_model refresh_models[ARPT_MAX_MODELS];
        if (app.model_buf) { free(app.model_buf); app.model_buf = NULL; }
        int nmodels = fetch_models(app.base_url, refresh_models,
                                   ARPT_MAX_MODELS, &app.model_buf);

        /* Tile manager must be freed before renderer (GPU resource deps) */
        arpt_tile_manager_free(app.tile_manager);
        arpt_renderer_free(app.renderer);

        int fb_w, fb_h;
        glfwGetFramebufferSize(app.window, &fb_w, &fb_h);
        int rbci = arpt_style_class_index(&style, "building");
        const float *rbldg = style.colors[rbci];
        app.renderer =
            arpt_renderer_create(app.device, app.queue, app.surface_format,
                                 (uint32_t)fb_w, (uint32_t)fb_h,
                                 style.colors[0], rbldg);

        /* Upload models, matching style params by model name */
        for (int si = 0; si < style.tree_style_count; si++) {
            for (int mi = 0; mi < nmodels; mi++) {
                if (strcmp(refresh_models[mi].name,
                           style.trees[si].model_name) == 0) {
                    refresh_models[mi].min_scale = style.trees[si].min_scale;
                    refresh_models[mi].max_scale = style.trees[si].max_scale;
                    refresh_models[mi].random_yaw = style.trees[si].random_yaw;
                    refresh_models[mi].random_scale =
                        style.trees[si].random_scale;
                    arpt_renderer_upload_model(app.renderer, si,
                                              &refresh_models[mi]);
                    break;
                }
            }
        }

        app.tile_manager =
            arpt_tile_manager_create(tm_config, app.renderer, &style);

        /* Restore UI overlay on the new renderer */
        if (app.ui)
            arpt_renderer_set_overlay(app.renderer, ui_overlay, app.ui);

        app.needs_redraw = true;
        printf("Refreshed style, tileset, and tiles\n");
    }

    /* Decide whether a redraw is needed */
    bool redraw = app.needs_redraw
               || arpt_control_needs_redraw(app.control)
               || (app.tile_manager &&
                   arpt_tile_manager_active_fetches(app.tile_manager) > 0);
    app.needs_redraw = false;

    if (!redraw) return;

    WGPUSurfaceTexture st;
    wgpuSurfaceGetCurrentTexture(app.surface, &st);
    if (st.status != WGPUSurfaceGetCurrentTextureStatus_Success) return;

    /* Update ground elevation from loaded tiles */
    if (app.tile_manager) {
        double target = arpt_tile_manager_ground_elevation(
            app.tile_manager, arpt_camera_lon(app.camera),
            arpt_camera_lat(app.camera));
        double alpha = 1.0 - exp(-dt * 8.0);
        app.smoothed_ground_elev +=
            (target - app.smoothed_ground_elev) * alpha;
        arpt_camera_set_ground_elevation(app.camera,
                                         app.smoothed_ground_elev);
    }

    /* Update global uniforms */
    arpt_mat4 projection = arpt_camera_projection(app.camera);
    arpt_vec3 sun_dir = {0.3f, 0.8f, 0.5f};
    arpt_renderer_set_globals(app.renderer, projection, sun_dir);

#ifndef __EMSCRIPTEN__
    /* Screenshot capture: render to an offscreen texture instead of the
       surface, because wgpu-native doesn't support copying from surface
       textures. */
    if (app.capture_next) {
        app.capture_next = false;
        uint32_t tw = wgpuTextureGetWidth(st.texture);
        uint32_t th = wgpuTextureGetHeight(st.texture);
        WGPUTextureDescriptor offscreen_desc = {
            .label = "screenshot_target",
            .usage = WGPUTextureUsage_RenderAttachment |
                     WGPUTextureUsage_CopySrc,
            .dimension = WGPUTextureDimension_2D,
            .size = {tw, th, 1},
            .format = app.surface_format,
            .mipLevelCount = 1,
            .sampleCount = 1,
        };
        WGPUTexture offscreen = wgpuDeviceCreateTexture(app.device,
                                                        &offscreen_desc);
        WGPUTextureView offscreen_view =
            wgpuTextureCreateView(offscreen, NULL);

        arpt_renderer_begin_frame(app.renderer, offscreen_view);
        if (app.tile_manager)
            arpt_tile_manager_draw(app.tile_manager, app.renderer, app.camera);
        arpt_renderer_end_frame(app.renderer);

        app.screenshot_ok = arpt_screenshot_save(
            app.instance, app.device, app.queue, offscreen,
            app.surface_format, tw, th, opts.screenshot);
        app.exit_code = app.screenshot_ok ? EXIT_SUCCESS : EXIT_FAILURE;

        wgpuTextureViewRelease(offscreen_view);
        wgpuTextureRelease(offscreen);
        wgpuTextureRelease(st.texture);
        glfwSetWindowShouldClose(app.window, GLFW_TRUE);
        return;
    }
#endif

    WGPUTextureView view = wgpuTextureCreateView(st.texture, NULL);

    arpt_renderer_begin_frame(app.renderer, view);

    /* Draw tiles from the tile manager (server-provided) */
    if (app.tile_manager)
        arpt_tile_manager_draw(app.tile_manager, app.renderer, app.camera);

    /* Update UI state (draw happens via overlay callback in end_frame) */
    if (app.ui) {
        double cx, cy;
        glfwGetCursorPos(app.window, &cx, &cy);
        arpt_ui_set_cursor(app.ui, (float)cx, (float)cy);
        arpt_ui_set_state(app.ui, (float)arpt_camera_bearing(app.camera),
                          (float)arpt_camera_tilt(app.camera));
    }

    arpt_renderer_end_frame(app.renderer);

    wgpuTextureViewRelease(view);
#ifndef __EMSCRIPTEN__
    wgpuSurfacePresent(app.surface);
#endif
    wgpuTextureRelease(st.texture);
}

/* UI overlay callback (invoked by renderer at end of frame) */

static void ui_overlay(WGPURenderPassEncoder pass, void *ud) {
    arpt_ui *ui = ud;
    arpt_ui_draw(ui, pass);
}

/* UI event filter */

#define UI_ZOOM_FACTOR 0.8 /* zoom in: altitude *= 0.8; zoom out: *= 1.25 */

static bool ui_event_filter(int button, int action, double sx, double sy,
                            void *ud) {
    (void)ud;
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return false;

    arpt_ui_action a = arpt_ui_hit_test(app.ui, (float)sx, (float)sy);
    if (a == ARPT_UI_NONE) return false;

    switch (a) {
    case ARPT_UI_ZOOM_IN:
        arpt_camera_zoom_at(app.camera, arpt_camera_vp_width(app.camera) / 2.0,
                            arpt_camera_vp_height(app.camera) / 2.0,
                            UI_ZOOM_FACTOR);
        break;
    case ARPT_UI_ZOOM_OUT:
        arpt_camera_zoom_at(app.camera, arpt_camera_vp_width(app.camera) / 2.0,
                            arpt_camera_vp_height(app.camera) / 2.0,
                            1.0 / UI_ZOOM_FACTOR);
        break;
    case ARPT_UI_RESET_NORTH:
        arpt_camera_set_bearing(app.camera, 0.0);
        break;
    case ARPT_UI_RESET_TILT:
        arpt_camera_set_tilt(app.camera, 0.0);
        break;
    default:
        break;
    }
    return true;
}

/* Fetch and parse tileset.arts, filling config fields from the FlatBuffer.
   base_url, max_tiles, and max_concurrent must be set by the caller. */

static bool fetch_tileset(const char *base_url,
                          arpt_tile_manager_config *config) {
    char url[512];
    int n = snprintf(url, sizeof(url), "%s/tileset.arts", base_url);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    uint8_t *buf = NULL;
    size_t buf_size = 0;

#ifdef __EMSCRIPTEN__
    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY | EMSCRIPTEN_FETCH_SYNCHRONOUS;
    emscripten_fetch_t *fetch = emscripten_fetch(&attr, url);
    if (!fetch || fetch->status != 200) {
        if (fetch) emscripten_fetch_close(fetch);
        return false;
    }
    buf_size = (size_t)fetch->numBytes;
    buf = malloc(buf_size);
    if (!buf) {
        emscripten_fetch_close(fetch);
        return false;
    }
    memcpy(buf, fetch->data, buf_size);
    emscripten_fetch_close(fetch);
#else
    arpt_http_response resp;
    if (!arpt_http_get(url, &resp) || resp.status != 200) {
        free(resp.body);
        return false;
    }
    buf = resp.body;
    buf_size = resp.body_size;
#endif

    int rc = arpentry_tiles_Tileset_verify_as_root_with_identifier(
        buf, buf_size, "arts");
    if (rc != 0) {
        fprintf(stderr, "tileset: verification failed (rc=%d)\n", rc);
        free(buf);
        return false;
    }

    arpentry_tiles_Tileset_table_t ts = arpentry_tiles_Tileset_as_root(buf);
    config->root_error = arpentry_tiles_Tileset_root_error(ts);
    config->min_level = arpentry_tiles_Tileset_min_level(ts);
    config->max_level = arpentry_tiles_Tileset_max_level(ts);

    flatbuffers_string_t name = arpentry_tiles_Tileset_name(ts);
    if (name)
        printf("Tileset: %s (levels %d-%d, root_error=%.0f)\n", name,
               config->min_level, config->max_level, config->root_error);

    free(buf);
    return true;
}

/* Fetch and parse style.arss, filling the arpt_style struct.
   Returns true on success. */

static bool fetch_style(const char *base_url, arpt_style *style) {
    char url[512];
    int n = snprintf(url, sizeof(url), "%s/style.arss", base_url);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    uint8_t *buf = NULL;
    size_t buf_size = 0;

#ifdef __EMSCRIPTEN__
    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY | EMSCRIPTEN_FETCH_SYNCHRONOUS;
    emscripten_fetch_t *fetch = emscripten_fetch(&attr, url);
    if (!fetch || fetch->status != 200) {
        if (fetch) emscripten_fetch_close(fetch);
        return false;
    }
    buf_size = (size_t)fetch->numBytes;
    buf = malloc(buf_size);
    if (!buf) {
        emscripten_fetch_close(fetch);
        return false;
    }
    memcpy(buf, fetch->data, buf_size);
    emscripten_fetch_close(fetch);
#else
    arpt_http_response resp;
    if (!arpt_http_get(url, &resp) || resp.status != 200) {
        free(resp.body);
        return false;
    }
    buf = resp.body;
    buf_size = resp.body_size;
#endif

    int rc = arpentry_tiles_Style_verify_as_root_with_identifier(
        buf, buf_size, "arss");
    if (rc != 0) {
        fprintf(stderr, "style: verification failed (rc=%d)\n", rc);
        free(buf);
        return false;
    }

    arpentry_tiles_Style_table_t st = arpentry_tiles_Style_as_root(buf);

    /* Parse background → colors[0] (unknown/background) */
    const arpentry_tiles_RGBA_t *bg = arpentry_tiles_Style_background(st);
    if (bg) {
        style->colors[0][0] = bg->r / 255.0f;
        style->colors[0][1] = bg->g / 255.0f;
        style->colors[0][2] = bg->b / 255.0f;
        style->colors[0][3] = bg->a / 255.0f;
    }

    /* Parse layers */
    arpentry_tiles_LayerStyle_vec_t layers = arpentry_tiles_Style_layers(st);
    size_t layer_count = layers ? arpentry_tiles_LayerStyle_vec_len(layers) : 0;
    for (size_t i = 0; i < layer_count; i++) {
        arpentry_tiles_LayerStyle_table_t layer =
            arpentry_tiles_LayerStyle_vec_at(layers, i);

        /* Extract layer metadata (source_layer, type, min_level) */
        if (style->layer_count < ARPT_MAX_STYLE_LAYERS) {
            arpt_layer_entry *le = &style->layers[style->layer_count];
            memset(le, 0, sizeof(*le));
            flatbuffers_string_t sl =
                arpentry_tiles_LayerStyle_source_layer(layer);
            if (sl)
                strncpy(le->source_layer, sl,
                        sizeof(le->source_layer) - 1);
            le->type = (arpt_layer_type)arpentry_tiles_LayerStyle_type(layer);
            le->min_level = arpentry_tiles_LayerStyle_min_level(layer);
            style->layer_count++;
        }

        arpentry_tiles_PaintEntry_vec_t paint =
            arpentry_tiles_LayerStyle_paint(layer);
        size_t paint_count = paint ? arpentry_tiles_PaintEntry_vec_len(paint) : 0;
        for (size_t j = 0; j < paint_count; j++) {
            arpentry_tiles_PaintEntry_table_t entry =
                arpentry_tiles_PaintEntry_vec_at(paint, j);
            flatbuffers_string_t cls_str =
                arpentry_tiles_PaintEntry_class(entry);
            /* Extract tree model style params from paint entries with "model" */
            flatbuffers_string_t model_str =
                arpentry_tiles_PaintEntry_model(entry);
            if (model_str && style->tree_style_count < ARPT_MAX_TREE_STYLES) {
                int ti = style->tree_style_count;
                if (cls_str) {
                    strncpy(style->trees[ti].class_name, cls_str,
                            sizeof(style->trees[ti].class_name) - 1);
                }
                strncpy(style->trees[ti].model_name, model_str,
                        sizeof(style->trees[ti].model_name) - 1);
                float ms = arpentry_tiles_PaintEntry_min_scale(entry);
                float xs = arpentry_tiles_PaintEntry_max_scale(entry);
                style->trees[ti].min_scale = ms > 0 ? ms : 1.0f;
                style->trees[ti].max_scale = xs > 0 ? xs : 1.0f;
                style->trees[ti].random_yaw =
                    arpentry_tiles_PaintEntry_random_yaw(entry);
                style->trees[ti].random_scale =
                    arpentry_tiles_PaintEntry_random_scale(entry);
                style->tree_style_count++;
            }

            /* Surface/highway/building color and stroke width */
            if (cls_str) {
                char name_buf[32] = {0};
                strncpy(name_buf, cls_str, sizeof(name_buf) - 1);
                int cls = arpt_style_class_index(style, name_buf);
                const arpentry_tiles_RGBA_t *c =
                    arpentry_tiles_PaintEntry_color(entry);
                if (c) {
                    style->colors[cls][0] = c->r / 255.0f;
                    style->colors[cls][1] = c->g / 255.0f;
                    style->colors[cls][2] = c->b / 255.0f;
                    style->colors[cls][3] = c->a / 255.0f;
                }
                float w = arpentry_tiles_PaintEntry_width(entry);
                if (w > 0) style->stroke_widths[cls] = w;
            }
        }
    }

    flatbuffers_string_t name = arpentry_tiles_Style_name(st);
    if (name)
        printf("Style: %s\n", name);

    free(buf);
    return true;
}

/* Fetch and decode model library (models.arpm).
   Populates an array of arpt_model structs from the library.
   The caller must keep model_buf alive while arpt_model pointers are used.
   Returns the number of models loaded. */

static int fetch_models(const char *base_url, arpt_model *models,
                        int max_models, uint8_t **model_buf_out) {
    char url[512];
    int n = snprintf(url, sizeof(url), "%s/models.arpm", base_url);
    if (n < 0 || (size_t)n >= sizeof(url)) return false;

    uint8_t *buf = NULL;
    size_t buf_size = 0;

#ifdef __EMSCRIPTEN__
    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    strcpy(attr.requestMethod, "GET");
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY | EMSCRIPTEN_FETCH_SYNCHRONOUS;
    emscripten_fetch_t *fetch = emscripten_fetch(&attr, url);
    if (!fetch || fetch->status != 200) {
        if (fetch) emscripten_fetch_close(fetch);
        return false;
    }
    buf_size = (size_t)fetch->numBytes;
    buf = malloc(buf_size);
    if (!buf) {
        emscripten_fetch_close(fetch);
        return false;
    }
    memcpy(buf, fetch->data, buf_size);
    emscripten_fetch_close(fetch);
#else
    arpt_http_response resp;
    if (!arpt_http_get(url, &resp) || resp.status != 200) {
        free(resp.body);
        return false;
    }
    buf = resp.body;
    buf_size = resp.body_size;
#endif

    int rc = arpentry_tiles_ModelLibrary_verify_as_root_with_identifier(
        buf, buf_size, "arpm");
    if (rc != 0) {
        fprintf(stderr, "models: verification failed (rc=%d)\n", rc);
        free(buf);
        return false;
    }

    arpentry_tiles_ModelLibrary_table_t lib =
        arpentry_tiles_ModelLibrary_as_root(buf);
    arpentry_tiles_Model_vec_t model_vec =
        arpentry_tiles_ModelLibrary_models(lib);
    if (!model_vec || arpentry_tiles_Model_vec_len(model_vec) == 0) {
        fprintf(stderr, "models: no models in library\n");
        free(buf);
        return 0;
    }

    size_t n_models = arpentry_tiles_Model_vec_len(model_vec);
    int loaded = 0;

    for (size_t mi = 0; mi < n_models && loaded < max_models; mi++) {
        arpentry_tiles_Model_table_t m =
            arpentry_tiles_Model_vec_at(model_vec, mi);
        flatbuffers_uint16_vec_t mx = arpentry_tiles_Model_x(m);
        flatbuffers_uint16_vec_t my = arpentry_tiles_Model_y(m);
        flatbuffers_uint16_vec_t mz = arpentry_tiles_Model_z(m);
        flatbuffers_uint32_vec_t mindices = arpentry_tiles_Model_indices(m);
        if (!mx || !my || !mz || !mindices) continue;

        arpt_model *mdl = &models[loaded];
        memset(mdl, 0, sizeof(*mdl));
        mdl->x = mx;
        mdl->y = my;
        mdl->z = mz;
        mdl->w = arpentry_tiles_Model_w(m); /* may be NULL */
        mdl->vertex_count = flatbuffers_uint16_vec_len(mx);
        mdl->indices = mindices;
        mdl->index_count = flatbuffers_uint32_vec_len(mindices);

        /* Extract name */
        flatbuffers_string_t name = arpentry_tiles_Model_name(m);
        if (name) strncpy(mdl->name, name, sizeof(mdl->name) - 1);

        /* Extract colors from Parts: Part[0]=trunk, Part[1]=crown */
        arpentry_tiles_Part_vec_t parts = arpentry_tiles_Model_parts(m);
        size_t nparts = parts ? arpentry_tiles_Part_vec_len(parts) : 0;
        if (nparts >= 1) {
            arpentry_tiles_Part_struct_t p0 =
                arpentry_tiles_Part_vec_at(parts, 0);
            if (p0) {
                mdl->trunk_color[0] = (float)p0->color.r / 255.0f;
                mdl->trunk_color[1] = (float)p0->color.g / 255.0f;
                mdl->trunk_color[2] = (float)p0->color.b / 255.0f;
                mdl->trunk_color[3] = (float)p0->color.a / 255.0f;
            }
        }
        if (nparts >= 2) {
            arpentry_tiles_Part_struct_t p1 =
                arpentry_tiles_Part_vec_at(parts, 1);
            if (p1) {
                mdl->crown_color[0] = (float)p1->color.r / 255.0f;
                mdl->crown_color[1] = (float)p1->color.g / 255.0f;
                mdl->crown_color[2] = (float)p1->color.b / 255.0f;
                mdl->crown_color[3] = (float)p1->color.a / 255.0f;
            }
        } else if (nparts == 1) {
            /* Single part: use trunk color for crown too */
            memcpy(mdl->crown_color, mdl->trunk_color,
                   sizeof(mdl->crown_color));
        }

        mdl->min_scale = 1.0f;
        mdl->max_scale = 1.0f;
        mdl->random_yaw = true;
        mdl->random_scale = true;

        printf("Model: %s (%zu verts, %zu indices)\n",
               mdl->name, mdl->vertex_count, mdl->index_count);
        loaded++;
    }

    *model_buf_out = buf; /* caller keeps alive for zero-copy pointers */
    return loaded;
}

/* Init after device */

static void init_viewer(void) {
    int fb_w, fb_h;
    glfwGetFramebufferSize(app.window, &fb_w, &fb_h);
    int win_w, win_h;
    glfwGetWindowSize(app.window, &win_w, &win_h);

    app.camera = arpt_camera_create();

#ifndef __EMSCRIPTEN__
    {
        double lon_rad = opts.lon * M_PI / 180.0;
        double lat_rad = opts.lat * M_PI / 180.0;
        arpt_camera_set_position(app.camera, lon_rad, lat_rad, opts.alt);
        arpt_camera_set_bearing(app.camera, opts.bearing * M_PI / 180.0);
        arpt_camera_set_tilt(app.camera, opts.tilt * M_PI / 180.0);
    }
#else
    arpt_camera_set_position(app.camera, 0.0, 0.0, INITIAL_ALTITUDE);
#endif

    /* Viewport in window (logical) pixels so cursor coords match on all
       targets. On Retina native fb_w > win_w; on web sync_canvas_size keeps
       them equal. */
    arpt_camera_set_viewport(app.camera, win_w, win_h);

    /* Fetch style (before renderer creation) */
#ifndef __EMSCRIPTEN__
    snprintf(app.base_url, sizeof(app.base_url), "%s", opts.url);
#else
    snprintf(app.base_url, sizeof(app.base_url), "http://localhost:8090");
#endif
    const char *base_url = app.base_url;
    arpt_style style;
    arpt_style_defaults(&style);
    if (!fetch_style(base_url, &style))
        fprintf(stderr, "Warning: style.arss fetch failed, using defaults\n");

    /* Fetch model library */
    arpt_model tree_models[ARPT_MAX_MODELS];
    if (app.model_buf) { free(app.model_buf); app.model_buf = NULL; }
    int nmodels = fetch_models(base_url, tree_models, ARPT_MAX_MODELS,
                               &app.model_buf);
    if (nmodels == 0)
        fprintf(stderr, "Warning: models.arpm fetch failed\n");

    /* Renderer */
    int bci = arpt_style_class_index(&style, "building");
    const float *bldg_color = style.colors[bci];
    app.renderer =
        arpt_renderer_create(app.device, app.queue, app.surface_format,
                             (uint32_t)fb_w, (uint32_t)fb_h, style.colors[0],
                             bldg_color);

    /* Upload tree models to GPU, matching style entries by model name.
       Each style tree entry (si) maps to a renderer model slot. */
    for (int si = 0; si < style.tree_style_count; si++) {
        for (int mi = 0; mi < nmodels; mi++) {
            if (strcmp(tree_models[mi].name,
                       style.trees[si].model_name) == 0) {
                tree_models[mi].min_scale = style.trees[si].min_scale;
                tree_models[mi].max_scale = style.trees[si].max_scale;
                tree_models[mi].random_yaw = style.trees[si].random_yaw;
                tree_models[mi].random_scale = style.trees[si].random_scale;
                arpt_renderer_upload_model(app.renderer, si, &tree_models[mi]);
                break;
            }
        }
    }

    /* Tile manager: fetch tileset metadata, then create manager */
    arpt_tile_manager_config tm_config = {
        .base_url = base_url,
        .root_error = 200000.0,
        .min_level = 0,
        .max_level = 19,
        .max_tiles = 200,
        .max_concurrent = 6,
    };
    if (!fetch_tileset(base_url, &tm_config))
        fprintf(stderr, "Warning: tileset.arts fetch failed, using defaults\n");
    app.tile_manager = arpt_tile_manager_create(tm_config, app.renderer, &style);

    /* Structured logging: camera position */
    printf("[CAMERA] lon=%.4f lat=%.4f alt=%.0f\n",
           arpt_camera_lon(app.camera) * 180.0 / M_PI,
           arpt_camera_lat(app.camera) * 180.0 / M_PI,
           arpt_camera_altitude(app.camera));

    /* Diagnostic: verify camera position */
    {
        printf("Window: %dx%d, Framebuffer: %dx%d, Scale: %.1fx\n", win_w,
               win_h, fb_w, fb_h, (double)fb_w / win_w);
        /* Cast ray from screen center — should hit near the interest point */
        double hit_lon, hit_lat;
        if (arpt_camera_screen_to_geodetic(app.camera, win_w / 2.0, win_h / 2.0,
                                           &hit_lon, &hit_lat)) {
            printf(
                "Center ray hits: lon=%.4f° lat=%.4f° (should match camera)\n",
                hit_lon * 180.0 / M_PI, hit_lat * 180.0 / M_PI);
        } else {
            printf("Center ray MISSES the globe! Camera may be inside.\n");
        }
    }

    /* UI overlay — skip in screenshot mode to avoid compass/zoom on capture */
#ifndef __EMSCRIPTEN__
    bool screenshot_mode = (opts.screenshot[0] != '\0');
#else
    bool screenshot_mode = false;
#endif
    if (!screenshot_mode) {
        float pixel_ratio = (win_w > 0) ? (float)fb_w / (float)win_w : 1.0f;
        app.ui = arpt_ui_create(app.device, app.queue, app.surface_format,
                                (uint32_t)fb_w, (uint32_t)fb_h, pixel_ratio);
        arpt_renderer_set_overlay(app.renderer, ui_overlay, app.ui);
    }

    /* Map control (mouse/keyboard/touch input) */
    app.control = arpt_control_create(app.camera, app.window);
    arpt_control_set_event_filter(app.control, ui_event_filter, NULL);
    app.last_time = glfwGetTime();

    /* Install GLFW callbacks (framebuffer resize is separate from input) */
    glfwSetFramebufferSizeCallback(app.window, on_framebuffer_resize);

    app.needs_redraw = true; /* force first frame */
}

/* WebGPU init chain */

static void on_device_done(WGPURequestDeviceStatus status, WGPUDevice device,
                           const char *msg, void *ud) {
    (void)ud;
    if (status != WGPURequestDeviceStatus_Success) {
        fprintf(stderr, "Device request failed: %s\n", msg ? msg : "unknown");
        return;
    }
    app.device = device;
    wgpuDeviceSetUncapturedErrorCallback(device, on_device_error, NULL);
    app.queue = wgpuDeviceGetQueue(device);

    /* Configure surface */
    int fb_w, fb_h;
    glfwGetFramebufferSize(app.window, &fb_w, &fb_h);

    WGPUSurfaceConfiguration cfg = {
        .device = device,
        .format = wgpuSurfaceGetPreferredFormat(app.surface, app.adapter),
        .usage = WGPUTextureUsage_RenderAttachment,
        .width = (uint32_t)fb_w,
        .height = (uint32_t)fb_h,
        .presentMode = WGPUPresentMode_Fifo,
        .alphaMode = WGPUCompositeAlphaMode_Auto,
    };
    app.surface_format = cfg.format;
    wgpuSurfaceConfigure(app.surface, &cfg);

    init_viewer();

    /* Start rendering */
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(render_frame, 0, 0);
#else
    if (opts.screenshot[0] != '\0') {
        /* Screenshot mode: render until tiles are loaded, then capture */
        while (!glfwWindowShouldClose(app.window)) {
            render_frame();

            int active = app.tile_manager
                             ? arpt_tile_manager_active_fetches(app.tile_manager)
                             : 0;
            if (active == 0)
                app.idle_frames++;
            else
                app.idle_frames = 0;

            if (app.idle_frames >= 3 && !app.capture_next) {
                printf("[READY] all visible tiles loaded\n");
                app.capture_next = true;
                /* Force one more redraw for capture */
                app.needs_redraw = true;
            }
        }
    } else {
        while (!glfwWindowShouldClose(app.window))
            render_frame();
    }
#endif
}

static void on_adapter_done(WGPURequestAdapterStatus status,
                            WGPUAdapter adapter, const char *msg, void *ud) {
    (void)ud;
    if (status != WGPURequestAdapterStatus_Success) {
        fprintf(stderr, "Adapter request failed: %s\n", msg ? msg : "unknown");
        return;
    }
    app.adapter = adapter;

#ifdef __EMSCRIPTEN__
    WGPUAdapterInfo info = {0};
    wgpuAdapterGetInfo(adapter, &info);
    printf("Adapter: %s (backend %d)\n",
           info.description ? info.description : "N/A", info.backendType);
#else
    WGPUAdapterProperties props = {0};
    wgpuAdapterGetProperties(adapter, &props);
    printf("Adapter: %s (backend %d)\n", props.name ? props.name : "N/A",
           props.backendType);
#endif

    WGPUDeviceDescriptor desc = {0};
    desc.label = "Arpentry Device";
    wgpuAdapterRequestDevice(adapter, &desc, on_device_done, NULL);
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOLBF, 0);

#ifndef __EMSCRIPTEN__
    parse_args(argc, argv);
#else
    (void)argc;
    (void)argv;
#endif

    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

#ifndef __EMSCRIPTEN__
    app.window = glfwCreateWindow(opts.width, opts.height,
                                  "Arpentry", NULL, NULL);
#else
    app.window = glfwCreateWindow(WINDOW_W, WINDOW_H, "Arpentry", NULL, NULL);
#endif
    if (!app.window) {
        fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    app.instance = wgpuCreateInstance(NULL);
    if (!app.instance) {
        fprintf(stderr, "Failed to create WebGPU instance\n");
        glfwDestroyWindow(app.window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    app.surface = glfwGetWGPUSurface(app.instance, app.window);

    WGPURequestAdapterOptions opts = {0};
    opts.compatibleSurface = app.surface;
    wgpuInstanceRequestAdapter(app.instance, &opts, on_adapter_done, NULL);

#ifndef __EMSCRIPTEN__
    /* Cleanup */
    if (app.tile_manager) arpt_tile_manager_free(app.tile_manager);
    if (app.ui) arpt_ui_free(app.ui);
    if (app.renderer) arpt_renderer_free(app.renderer);
    free(app.model_buf);
    if (app.control) arpt_control_free(app.control);
    if (app.camera) arpt_camera_free(app.camera);
    wgpuSurfaceUnconfigure(app.surface);
    if (app.queue) wgpuQueueRelease(app.queue);
    if (app.device) wgpuDeviceRelease(app.device);
    if (app.adapter) wgpuAdapterRelease(app.adapter);
    wgpuSurfaceRelease(app.surface);
    wgpuInstanceRelease(app.instance);
    glfwDestroyWindow(app.window);
    glfwTerminate();
#endif

#ifndef __EMSCRIPTEN__
    return app.exit_code;
#else
    return EXIT_SUCCESS;
#endif
}
