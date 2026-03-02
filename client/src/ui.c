#include "ui.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* WGSL shader (generated from client/shaders/ui.wgsl at build time) */

#include "ui.wgsl.h"

/* Layout constants (must match shader) */

#define UI_CY 48.0f
#define UI_COMP_R 32.0f
#define UI_COMP_CX 48.0f
#define UI_TILT_HW 19.0f
#define UI_TILT_HH 32.0f
#define UI_TILT_R 19.0f
#define UI_TILT_CX 109.0f
#define UI_ZOOM_HW 19.0f
#define UI_ZOOM_HH 32.0f
#define UI_ZOOM_R 19.0f
#define UI_ZOOM_CX 157.0f

/* Uniform buffer layout */

typedef struct {
    float screen[2];
    float scale;
    float bearing;
    float tilt;
    float cursor_x;
    float cursor_y;
    float _pad;
} ui_uniforms_t;

/* Internal struct */

struct arpt_ui {
    WGPUDevice device;
    WGPUQueue queue;
    WGPURenderPipeline pipeline;
    WGPUBindGroupLayout bgl;
    WGPUBindGroup bind_group;
    WGPUBuffer uniform_buf;

    uint32_t fb_width, fb_height;
    float pixel_ratio;
    float bearing, tilt;
    float cursor_x, cursor_y;
};

/* SDF helpers (C, for hit testing) */

static float sd_box_c(float px, float py, float bx, float by, float r) {
    float qx = fabsf(px) - bx + r;
    float qy = fabsf(py) - by + r;
    float mx = fmaxf(qx, 0.0f);
    float my = fmaxf(qy, 0.0f);
    return sqrtf(mx * mx + my * my) + fminf(fmaxf(qx, qy), 0.0f) - r;
}

static float sd_circle_c(float px, float py, float r) {
    return sqrtf(px * px + py * py) - r;
}

/* Public API */

arpt_ui *arpt_ui_create(WGPUDevice device, WGPUQueue queue,
                        WGPUTextureFormat surface_format, uint32_t fb_width,
                        uint32_t fb_height, float pixel_ratio) {
    arpt_ui *ui = calloc(1, sizeof(*ui));
    if (!ui) return NULL;

    ui->device = device;
    ui->queue = queue;
    ui->fb_width = fb_width;
    ui->fb_height = fb_height;
    ui->pixel_ratio = pixel_ratio;
    ui->cursor_x = -100.0f;
    ui->cursor_y = -100.0f;

    /* Shader module */
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = ui_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    /* Bind group layout: one uniform buffer */
    WGPUBindGroupLayoutEntry bgle = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(ui_uniforms_t)},
    };
    ui->bgl = wgpuDeviceCreateBindGroupLayout(
        device,
        &(WGPUBindGroupLayoutDescriptor){.entryCount = 1, .entries = &bgle});

    /* Pipeline layout */
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 1,
                                                .bindGroupLayouts = &ui->bgl});

    /* Color target with alpha blending for glass effect */
    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
        .alpha = {.srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
    };
    WGPUColorTargetState ct = {
        .format = surface_format,
        .blend = &blend,
        .writeMask = WGPUColorWriteMask_All,
    };
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    /* Depth: always pass, no write (overlay on top of 3D scene) */
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24Plus,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_Always,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask = 0,
        .stencilWriteMask = 0,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm,
                   .entryPoint = "vs",
                   .bufferCount = 0,
                   .buffers = NULL},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = 1, .mask = ~0u},
    };
    ui->pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);

    /* Uniform buffer */
    WGPUBufferDescriptor buf_desc = {
        .usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst,
        .size = sizeof(ui_uniforms_t),
    };
    ui->uniform_buf = wgpuDeviceCreateBuffer(device, &buf_desc);

    /* Bind group */
    WGPUBindGroupEntry bg_entry = {
        .binding = 0,
        .buffer = ui->uniform_buf,
        .offset = 0,
        .size = sizeof(ui_uniforms_t),
    };
    ui->bind_group = wgpuDeviceCreateBindGroup(
        device, &(WGPUBindGroupDescriptor){
                    .layout = ui->bgl, .entryCount = 1, .entries = &bg_entry});

    return ui;
}

void arpt_ui_free(arpt_ui *ui) {
    if (!ui) return;
    if (ui->bind_group) wgpuBindGroupRelease(ui->bind_group);
    if (ui->uniform_buf) wgpuBufferRelease(ui->uniform_buf);
    if (ui->bgl) wgpuBindGroupLayoutRelease(ui->bgl);
    if (ui->pipeline) wgpuRenderPipelineRelease(ui->pipeline);
    free(ui);
}

void arpt_ui_resize(arpt_ui *ui, uint32_t fb_width, uint32_t fb_height,
                    float pixel_ratio) {
    ui->fb_width = fb_width;
    ui->fb_height = fb_height;
    ui->pixel_ratio = pixel_ratio;
}

void arpt_ui_set_state(arpt_ui *ui, float bearing_rad, float tilt_rad) {
    ui->bearing = bearing_rad;
    ui->tilt = tilt_rad;
}

void arpt_ui_set_cursor(arpt_ui *ui, float screen_x, float screen_y) {
    ui->cursor_x = screen_x;
    ui->cursor_y = screen_y;
}

arpt_ui_action arpt_ui_hit_test(const arpt_ui *ui, float screen_x,
                                float screen_y) {
    float scr_w = (float)ui->fb_width / ui->pixel_ratio;
    float scr_h = (float)ui->fb_height / ui->pixel_ratio;
    float bx = scr_w - screen_x;
    float by = scr_h - screen_y;

    /* Compass (rightmost) */
    float dx = bx - UI_COMP_CX, dy = by - UI_CY;
    if (sd_circle_c(dx, dy, UI_COMP_R) < 0.0f) return ARPT_UI_RESET_NORTH;

    /* Tilt (middle) */
    dx = bx - UI_TILT_CX;
    dy = by - UI_CY;
    if (sd_box_c(dx, dy, UI_TILT_HW, UI_TILT_HH, UI_TILT_R) < 0.0f)
        return ARPT_UI_RESET_TILT;

    /* Zoom (leftmost, vertical — top half = plus, bottom half = minus) */
    dx = bx - UI_ZOOM_CX;
    dy = by - UI_CY;
    if (sd_box_c(dx, dy, UI_ZOOM_HW, UI_ZOOM_HH, UI_ZOOM_R) < 0.0f)
        return dy > 0.0f ? ARPT_UI_ZOOM_IN : ARPT_UI_ZOOM_OUT;

    return ARPT_UI_NONE;
}

void arpt_ui_draw(arpt_ui *ui, WGPURenderPassEncoder pass) {
    ui_uniforms_t u = {
        .screen = {(float)ui->fb_width, (float)ui->fb_height},
        .scale = ui->pixel_ratio,
        .bearing = ui->bearing,
        .tilt = ui->tilt,
        .cursor_x = ui->cursor_x,
        .cursor_y = ui->cursor_y,
    };
    wgpuQueueWriteBuffer(ui->queue, ui->uniform_buf, 0, &u, sizeof(u));

    wgpuRenderPassEncoderSetPipeline(pass, ui->pipeline);
    wgpuRenderPassEncoderSetBindGroup(pass, 0, ui->bind_group, 0, NULL);
    wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
}
