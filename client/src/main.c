#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webgpu/webgpu.h>
#include <GLFW/glfw3.h>
#include "glfw3webgpu.h"

#include "camera.h"
#include "control.h"
#include "renderer.h"
#include "tile_manager.h"
#include "coords.h"

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
    arpt_control *control;
    arpt_renderer *renderer;
    arpt_tile_manager *tile_manager;
    double last_time;
} App;

static App app = {0};


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

    /* Compute dt and advance control */
    double now = glfwGetTime();
    double dt = now - app.last_time;
    app.last_time = now;
    if (app.control)
        arpt_control_update(app.control, dt);

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
    arpt_camera_set_position(app.camera, center_lon, center_lat,
                              INITIAL_ALTITUDE);
    arpt_camera_set_viewport(app.camera, fb_w, fb_h);

    /* Renderer */
    app.renderer = arpt_renderer_create(app.device, app.queue,
                                         app.surface_format,
                                         (uint32_t)fb_w, (uint32_t)fb_h);

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

    /* Diagnostic: verify camera position */
    {
        int win_w, win_h;
        glfwGetWindowSize(app.window, &win_w, &win_h);
        printf("Window: %dx%d, Framebuffer: %dx%d, Scale: %.1fx\n",
               win_w, win_h, fb_w, fb_h, (double)fb_w / win_w);
        printf("Camera: lon=%.4f° lat=%.4f° alt=%.0fm tilt=%.1f° bearing=%.1f°\n",
               arpt_camera_lon(app.camera) * 180.0 / M_PI,
               arpt_camera_lat(app.camera) * 180.0 / M_PI,
               arpt_camera_altitude(app.camera),
               arpt_camera_tilt(app.camera) * 180.0 / M_PI,
               arpt_camera_bearing(app.camera) * 180.0 / M_PI);
        /* Cast ray from screen center — should hit near the interest point */
        double hit_lon, hit_lat;
        if (arpt_camera_screen_to_geodetic(app.camera, fb_w / 2.0, fb_h / 2.0,
                                            &hit_lon, &hit_lat)) {
            printf("Center ray hits: lon=%.4f° lat=%.4f° (should match camera)\n",
                   hit_lon * 180.0 / M_PI, hit_lat * 180.0 / M_PI);
        } else {
            printf("Center ray MISSES the globe! Camera may be inside.\n");
        }
    }

    /* Map control (mouse/keyboard/touch input) */
    app.control = arpt_control_create(app.camera, app.window);
    app.last_time = glfwGetTime();

    /* Install GLFW callbacks (framebuffer resize is separate from input) */
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
    if (app.renderer) arpt_renderer_free(app.renderer);
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

    return EXIT_SUCCESS;
}
