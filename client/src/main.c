#include <stdio.h>
#include <stdlib.h>
#include <webgpu/webgpu.h>
#include <GLFW/glfw3.h>
#include "glfw3webgpu.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

typedef struct {
    GLFWwindow *window;
    WGPUInstance instance;
    WGPUSurface surface;
    WGPUAdapter adapter;
    WGPUDevice device;
    WGPUQueue queue;
    WGPURenderPipeline pipeline;
    WGPUTextureFormat surface_format;
} App;

static App app = {0};

static void on_device_error(WGPUErrorType type, const char *msg, void *ud) {
    (void)ud;
    fprintf(stderr, "WebGPU device error (%d): %s\n", type, msg);
}

static const char *triangle_wgsl =
    "struct Out { @builtin(position) pos: vec4f, @location(0) color: vec3f };\n"
    "\n"
    "@vertex fn vs(@builtin(vertex_index) i: u32) -> Out {\n"
    "    var p = array<vec2f, 3>(vec2f(0, 0.5), vec2f(-0.5, -0.5), vec2f(0.5, -0.5));\n"
    "    var c = array<vec3f, 3>(vec3f(1,0,0), vec3f(0,1,0), vec3f(0,0,1));\n"
    "    var out: Out;\n"
    "    out.pos = vec4f(p[i], 0, 1);\n"
    "    out.color = c[i];\n"
    "    return out;\n"
    "}\n"
    "\n"
    "@fragment fn fs(@location(0) color: vec3f) -> @location(0) vec4f {\n"
    "    return vec4f(color, 1);\n"
    "}\n";

static void create_pipeline(void) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = { .sType = WGPUSType_ShaderModuleWGSLDescriptor },
        .code = triangle_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {
        .nextInChain = &wgsl_desc.chain,
    };
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(app.device, &sm_desc);

    WGPUColorTargetState color_target = {
        .format = app.surface_format,
        .writeMask = WGPUColorWriteMask_All,
    };
    WGPUFragmentState frag = {
        .module = sm,
        .entryPoint = "fs",
        .targetCount = 1,
        .targets = &color_target,
    };
    WGPURenderPipelineDescriptor pip_desc = {
        .layout = NULL, /* auto layout */
        .vertex = {
            .module = sm,
            .entryPoint = "vs",
        },
        .primitive = {
            .topology = WGPUPrimitiveTopology_TriangleList,
        },
        .fragment = &frag,
        .multisample = {
            .count = 1,
            .mask = ~0u,
        },
    };
    app.pipeline = wgpuDeviceCreateRenderPipeline(app.device, &pip_desc);
    wgpuShaderModuleRelease(sm);
}

static void render_frame(void) {
    glfwPollEvents();

    WGPUSurfaceTexture st;
    wgpuSurfaceGetCurrentTexture(app.surface, &st);
    if (st.status != WGPUSurfaceGetCurrentTextureStatus_Success)
        return;

    WGPUTextureView view = wgpuTextureCreateView(st.texture, NULL);
    WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(app.device, NULL);

    WGPURenderPassColorAttachment color = {
        .view = view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Store,
        .clearValue = {0.15, 0.15, 0.20, 1.0},
#ifdef __EMSCRIPTEN__
        .depthSlice = WGPU_DEPTH_SLICE_UNDEFINED,
#endif
    };
    WGPURenderPassDescriptor rp = {
        .colorAttachmentCount = 1,
        .colorAttachments = &color,
    };

    WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &rp);
    wgpuRenderPassEncoderSetPipeline(pass, app.pipeline);
    wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(encoder, NULL);
    wgpuQueueSubmit(app.queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(encoder);
    wgpuTextureViewRelease(view);
#ifndef __EMSCRIPTEN__
    wgpuSurfacePresent(app.surface);
#endif
    wgpuTextureRelease(st.texture);
}

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
    create_pipeline();

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
    app.window = glfwCreateWindow(800, 600, "Arpentry", NULL, NULL);
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

    /* Native: callbacks fire synchronously, so the main loop has already run
       and exited by the time we reach here. Clean up.
       Emscripten: callbacks fire asynchronously. main() returns immediately
       and the runtime stays alive for the registered main loop. */
#ifndef __EMSCRIPTEN__
    if (app.pipeline) wgpuRenderPipelineRelease(app.pipeline);
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
