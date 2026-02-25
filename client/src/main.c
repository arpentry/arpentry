#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webgpu/webgpu.h>
#include <GLFW/glfw3.h>
#include "glfw3webgpu.h"

#include "camera.h"
#include "renderer.h"
#include "tile_decode.h"
#include "tile_manager.h"
#include "coords.h"
#include "tile_builder.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define DEMO_LEVEL  5
#define DEMO_X      34
#define DEMO_Y      22

#define INITIAL_ALTITUDE 500000.0
#define WINDOW_W         800
#define WINDOW_H         600

#define DEMO_GRID 5

/* ── App state ─────────────────────────────────────────────────────────── */

typedef struct {
    GLFWwindow *window;
    WGPUInstance instance;
    WGPUSurface surface;
    WGPUAdapter adapter;
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat surface_format;

    arpt_camera *camera;
    arpt_renderer *renderer;
    arpt_tile_manager *tile_manager;
    arpt_tile_gpu *demo_tile;

    /* Demo tile geodetic center (radians) */
    double tile_center_lon;
    double tile_center_lat;

} App;

static App app = {0};

/* ── Demo terrain tile builder ─────────────────────────────────────────── */

/* Build a DEMO_GRID x DEMO_GRID terrain grid tile for the given bounds. */
static void *build_demo_terrain(size_t *out_size,
                                 arpt_bounds_t bounds) {
    flatcc_builder_t b;
    flatcc_builder_init(&b);

    arpentry_tiles_Tile_start_as_root(&b);
    arpentry_tiles_Tile_version_add(&b, 1);

    arpentry_tiles_Tile_layers_start(&b);
    arpentry_tiles_Tile_layers_push_start(&b);
    arpentry_tiles_Layer_name_create_str(&b, "terrain");

    arpentry_tiles_Layer_features_start(&b);
    arpentry_tiles_Layer_features_push_start(&b);
    arpentry_tiles_Feature_id_add(&b, 1);

    int nv = DEMO_GRID * DEMO_GRID;
    int ni = (DEMO_GRID - 1) * (DEMO_GRID - 1) * 6;

    uint16_t xs[DEMO_GRID * DEMO_GRID], ys[DEMO_GRID * DEMO_GRID];
    int32_t zs[DEMO_GRID * DEMO_GRID];
    int8_t normals[DEMO_GRID * DEMO_GRID * 2];

    for (int row = 0; row < DEMO_GRID; row++) {
        for (int col = 0; col < DEMO_GRID; col++) {
            int idx = row * DEMO_GRID + col;
            double u = (double)col / (DEMO_GRID - 1);
            double v = (double)row / (DEMO_GRID - 1);
            double lon = bounds.west + u * (bounds.east - bounds.west);
            double lat = bounds.south + v * (bounds.north - bounds.south);
            xs[idx] = arpt_quantize_lon(lon, bounds);
            ys[idx] = arpt_quantize_lat(lat, bounds);

            /* Simple elevation: a smooth hill */
            double cx = u - 0.5, cy = v - 0.5;
            double elev = 2000.0 * exp(-(cx * cx + cy * cy) * 8.0);
            zs[idx] = arpt_meters_to_mm(elev);

            /* Up-facing normals (octahedral encoding of (0,0,1) = (0,0)) */
            normals[idx * 2] = 0;
            normals[idx * 2 + 1] = 0;
        }
    }

    uint32_t indices[(DEMO_GRID - 1) * (DEMO_GRID - 1) * 6];
    int ii = 0;
    for (int row = 0; row < DEMO_GRID - 1; row++) {
        for (int col = 0; col < DEMO_GRID - 1; col++) {
            uint32_t tl = (uint32_t)(row * DEMO_GRID + col);
            uint32_t tr = tl + 1;
            uint32_t bl = tl + DEMO_GRID;
            uint32_t br = bl + 1;
            indices[ii++] = tl; indices[ii++] = bl; indices[ii++] = tr;
            indices[ii++] = tr; indices[ii++] = bl; indices[ii++] = br;
        }
    }

    arpentry_tiles_MeshGeometry_start(&b);
    arpentry_tiles_MeshGeometry_x_create(&b, xs, nv);
    arpentry_tiles_MeshGeometry_y_create(&b, ys, nv);
    arpentry_tiles_MeshGeometry_z_create(&b, zs, nv);
    arpentry_tiles_MeshGeometry_indices_create(&b, indices, ni);
    arpentry_tiles_MeshGeometry_normals_create(&b, normals, nv * 2);

    arpentry_tiles_MeshGeometry_ref_t ref = arpentry_tiles_MeshGeometry_end(&b);
    arpentry_tiles_Feature_geometry_MeshGeometry_add(&b, ref);

    arpentry_tiles_Layer_features_push_end(&b);
    arpentry_tiles_Layer_features_end(&b);
    arpentry_tiles_Tile_layers_push_end(&b);
    arpentry_tiles_Tile_layers_end(&b);
    arpentry_tiles_Tile_end_as_root(&b);

    void *buf = flatcc_builder_finalize_buffer(&b, out_size);
    flatcc_builder_clear(&b);
    return buf;
}

/* ── GLFW callbacks ────────────────────────────────────────────────────── */

static void on_device_error(WGPUErrorType type, const char *msg, void *ud) {
    (void)ud;
    fprintf(stderr, "WebGPU device error (%d): %s\n", type, msg);
}

static void on_framebuffer_resize(GLFWwindow *w, int width, int height) {
    (void)w;
    if (width == 0 || height == 0) return;
    arpt_camera_set_viewport(app.camera, width, height);
    arpt_renderer_resize(app.renderer, (uint32_t)width, (uint32_t)height);

    /* Reconfigure surface */
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
}

/* ── Render frame ──────────────────────────────────────────────────────── */

static void render_frame(void) {
    glfwPollEvents();

    WGPUSurfaceTexture st;
    wgpuSurfaceGetCurrentTexture(app.surface, &st);
    if (st.status != WGPUSurfaceGetCurrentTextureStatus_Success)
        return;

    WGPUTextureView view = wgpuTextureCreateView(st.texture, NULL);

    /* Update tile manager (fetch new tiles, evict old ones) */
    if (app.tile_manager)
        arpt_tile_manager_update(app.tile_manager, app.camera);

    /* Update global uniforms */
    arpt_mat4 projection = arpt_camera_projection(app.camera);
    arpt_vec3 sun_dir = {0.3f, 0.8f, 0.5f};
    arpt_renderer_set_globals(app.renderer, projection, sun_dir);

    arpt_renderer_begin_frame(app.renderer, view);

    /* Draw tiles from the tile manager (server-provided) */
    if (app.tile_manager)
        arpt_tile_manager_draw(app.tile_manager, app.renderer, app.camera);

    /* Always draw the demo tile as fallback */
    if (app.demo_tile) {
        arpt_mat4 model = arpt_camera_tile_model(app.camera,
                                                  app.tile_center_lon,
                                                  app.tile_center_lat, 0.0);
        arpt_bounds_t bounds = arpt_tile_bounds(DEMO_LEVEL, DEMO_X, DEMO_Y);
        float bounds_rad[4] = {
            (float)(bounds.west * M_PI / 180.0),
            (float)(bounds.south * M_PI / 180.0),
            (float)(bounds.east * M_PI / 180.0),
            (float)(bounds.north * M_PI / 180.0),
        };
        arpt_tile_gpu_set_uniforms(app.demo_tile, model, bounds_rad,
                                    (float)app.tile_center_lon,
                                    (float)app.tile_center_lat);
        arpt_renderer_draw_tile(app.renderer, app.demo_tile);
    }

    arpt_renderer_end_frame(app.renderer);

    wgpuTextureViewRelease(view);
#ifndef __EMSCRIPTEN__
    wgpuSurfacePresent(app.surface);
#endif
    wgpuTextureRelease(st.texture);
}

/* ── Init after device ─────────────────────────────────────────────────── */

static void init_viewer(void) {
    int fb_w, fb_h;
    glfwGetFramebufferSize(app.window, &fb_w, &fb_h);

    /* Camera: start looking at a tile in Switzerland */
    app.camera = arpt_camera_create();
    arpt_bounds_t bounds = arpt_tile_bounds(DEMO_LEVEL, DEMO_X, DEMO_Y);
    double center_lon = (bounds.west + bounds.east) / 2.0 * M_PI / 180.0;
    double center_lat = (bounds.south + bounds.north) / 2.0 * M_PI / 180.0;
    app.tile_center_lon = center_lon;
    app.tile_center_lat = center_lat;
    arpt_camera_set_position(app.camera, center_lon, center_lat,
                              INITIAL_ALTITUDE);
    arpt_camera_set_viewport(app.camera, fb_w, fb_h);

    /* Renderer */
    app.renderer = arpt_renderer_create(app.device, app.queue,
                                         app.surface_format,
                                         (uint32_t)fb_w, (uint32_t)fb_h);

    /* Build and upload demo terrain tile */
    size_t tile_size;
    void *tile_buf = build_demo_terrain(&tile_size, bounds);
    if (tile_buf) {
        arpt_terrain_mesh mesh = {0};
        if (arpt_decode_terrain(tile_buf, tile_size, &mesh)) {
            app.demo_tile = arpt_renderer_upload_tile(app.renderer, &mesh);
        }
        free(tile_buf);
    }

    if (!app.demo_tile) {
        fprintf(stderr, "Failed to create demo tile\n");
    }

    /* Tile manager for server-provided tiles */
    arpt_tile_manager_config tm_config = {
        .base_url = "http://localhost:8090",
        .root_error = 50000.0,
        .min_level = 0,
        .max_level = 16,
        .max_tiles = 200,
        .max_concurrent = 6,
    };
    app.tile_manager = arpt_tile_manager_create(tm_config, app.renderer);

    /* Install GLFW callbacks */
    glfwSetFramebufferSizeCallback(app.window, on_framebuffer_resize);
}

/* ── WebGPU init chain ─────────────────────────────────────────────────── */

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
    while (!glfwWindowShouldClose(app.window))
        render_frame();
#endif
}

static void on_adapter_done(WGPURequestAdapterStatus status, WGPUAdapter adapter,
                            const char *msg, void *ud) {
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
    printf("Adapter: %s (backend %d)\n",
           props.name ? props.name : "N/A", props.backendType);
#endif

    WGPUDeviceDescriptor desc = {0};
    desc.label = "Arpentry Device";
    wgpuAdapterRequestDevice(adapter, &desc, on_device_done, NULL);
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);

    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    app.window = glfwCreateWindow(WINDOW_W, WINDOW_H, "Arpentry", NULL, NULL);
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
    if (app.demo_tile) arpt_tile_gpu_free(app.demo_tile);
    if (app.renderer) arpt_renderer_free(app.renderer);
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

    return EXIT_SUCCESS;
}
