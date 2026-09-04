#include "present.h"

#include <GLFW/glfw3.h>
#include <glfw3webgpu.h>
#include <stdio.h>
#include <stdlib.h>

/* Headless advances at a fixed step so a capture is a function of the scene
   and not of the machine. 60 Hz matches the windowed swap interval. */
#define HEADLESS_STEP (1.0 / 60.0)

struct arpt_present {
    /* Windowed: both set. Headless: both NULL. */
    GLFWwindow *window;
    WGPUSurface surface;

    WGPUDevice device;
    WGPUTextureFormat format;
    uint32_t width;  /* headless framebuffer size; unused when windowed */
    uint32_t height;

    /* The frame in flight. `owned` is set when the texture came from us
       rather than from the surface, and so must be released by us. */
    WGPUTexture texture;
    WGPUTextureView view;
    bool owned;

    /* Offscreen target, kept across frames so a headless run allocates once
       and a windowed capture allocates only when it captures. */
    WGPUTexture offscreen;

    uint64_t frames;
    bool should_close;
};

/* Drop the offscreen target so the next acquire rebuilds it at the current
   size and format. */
static void drop_offscreen(arpt_present *p) {
    if (!p->offscreen) return;
    wgpuTextureRelease(p->offscreen);
    p->offscreen = NULL;
}

arpt_present *arpt_present_create_window(WGPUInstance instance, int width,
                                          int height, const char *title) {
    if (!instance) return NULL;

    arpt_present *p = calloc(1, sizeof(*p));
    if (!p) return NULL;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    p->window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!p->window) {
        fprintf(stderr, "Failed to create window\n");
        free(p);
        return NULL;
    }

    p->surface = glfwGetWGPUSurface(instance, p->window);
    if (!p->surface) {
        fprintf(stderr, "Failed to create WebGPU surface\n");
        glfwDestroyWindow(p->window);
        free(p);
        return NULL;
    }
    return p;
}

arpt_present *arpt_present_create_headless(uint32_t width, uint32_t height,
                                            WGPUTextureFormat format) {
    if (width == 0 || height == 0) return NULL;

    arpt_present *p = calloc(1, sizeof(*p));
    if (!p) return NULL;

    p->width = width;
    p->height = height;
    p->format = format;
    return p;
}

void arpt_present_free(arpt_present *p) {
    if (!p) return;
    if (p->view) wgpuTextureViewRelease(p->view);
    if (p->texture && p->owned) wgpuTextureRelease(p->texture);
    drop_offscreen(p);
    if (p->surface) {
        wgpuSurfaceUnconfigure(p->surface);
        wgpuSurfaceRelease(p->surface);
    }
    if (p->window) glfwDestroyWindow(p->window);
    free(p);
}

WGPUSurface arpt_present_surface(const arpt_present *p) {
    return p ? p->surface : NULL;
}

GLFWwindow *arpt_present_window(const arpt_present *p) {
    return p ? p->window : NULL;
}

void arpt_present_configure(arpt_present *p, WGPUDevice device,
                             WGPUTextureFormat format, uint32_t width,
                             uint32_t height) {
    if (!p || width == 0 || height == 0) return;

    p->device = device;
    if (p->format != format || p->width != width || p->height != height)
        drop_offscreen(p);
    p->format = format;
    p->width = width;
    p->height = height;

    if (!p->surface) return;

    WGPUSurfaceConfiguration cfg = {
        .device = device,
        .format = format,
        .usage = WGPUTextureUsage_RenderAttachment,
        .width = width,
        .height = height,
        .presentMode = WGPUPresentMode_Fifo,
        .alphaMode = WGPUCompositeAlphaMode_Auto,
    };
    wgpuSurfaceConfigure(p->surface, &cfg);
}

WGPUTextureFormat arpt_present_format(const arpt_present *p) {
    return p ? p->format : WGPUTextureFormat_Undefined;
}

/* Build (or reuse) the copyable offscreen target. */
static WGPUTexture offscreen_target(arpt_present *p) {
    if (p->offscreen) return p->offscreen;
    if (!p->device) return NULL;

    WGPUTextureDescriptor desc = {
        .label = "present_offscreen",
        .usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_CopySrc,
        .dimension = WGPUTextureDimension_2D,
        .size = {p->width, p->height, 1},
        .format = p->format,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    p->offscreen = wgpuDeviceCreateTexture(p->device, &desc);
    return p->offscreen;
}

WGPUTextureView arpt_present_acquire(arpt_present *p, bool capture) {
    if (!p || p->view) return NULL;

    if (p->surface && !capture) {
        WGPUSurfaceTexture st;
        wgpuSurfaceGetCurrentTexture(p->surface, &st);
        if (st.status != WGPUSurfaceGetCurrentTextureStatus_Success)
            return NULL;
        p->texture = st.texture;
        p->owned = true; /* the surface hands us a reference to release */
    } else {
        WGPUTexture target = offscreen_target(p);
        if (!target) return NULL;
        p->texture = target;
        p->owned = false; /* kept across frames; freed with the present */
    }

    p->view = wgpuTextureCreateView(p->texture, NULL);
    if (!p->view) {
        if (p->owned) wgpuTextureRelease(p->texture);
        p->texture = NULL;
        return NULL;
    }
    return p->view;
}

WGPUTexture arpt_present_texture(const arpt_present *p) {
    return p ? p->texture : NULL;
}

void arpt_present_end(arpt_present *p) {
    if (!p || !p->view) return;

    wgpuTextureViewRelease(p->view);
    p->view = NULL;

    /* A frame drawn offscreen was never on the swapchain, so there is
       nothing to present — including a windowed capture. */
#ifndef __EMSCRIPTEN__
    if (p->surface && p->texture != p->offscreen)
        wgpuSurfacePresent(p->surface);
#endif

    if (p->owned) wgpuTextureRelease(p->texture);
    p->texture = NULL;
    p->owned = false;
    p->frames++;
}

void arpt_present_framebuffer_size(const arpt_present *p, int *w, int *h) {
    if (!p) return;
    if (p->window) {
        glfwGetFramebufferSize(p->window, w, h);
        return;
    }
    if (w) *w = (int)p->width;
    if (h) *h = (int)p->height;
}

void arpt_present_window_size(const arpt_present *p, int *w, int *h) {
    if (!p) return;
    if (p->window) {
        glfwGetWindowSize(p->window, w, h);
        return;
    }
    /* No window, so no device-pixel ratio to divide out: logical pixels are
       physical pixels and the renderer scales by 1. */
    if (w) *w = (int)p->width;
    if (h) *h = (int)p->height;
}

double arpt_present_time(arpt_present *p) {
    if (!p) return 0.0;
    if (p->window) return glfwGetTime();
    return (double)p->frames * HEADLESS_STEP;
}

void arpt_present_poll(arpt_present *p, bool block) {
    if (!p || !p->window) return;
    if (block)
        glfwWaitEventsTimeout(0.1);
    else
        glfwPollEvents();
}

bool arpt_present_should_close(const arpt_present *p) {
    if (!p) return true;
    if (p->window) return glfwWindowShouldClose(p->window) != 0;
    return p->should_close;
}

void arpt_present_close(arpt_present *p) {
    if (!p) return;
    p->should_close = true;
    if (p->window) glfwSetWindowShouldClose(p->window, GLFW_TRUE);
}
