#include "info.h"
#include "font.h"
#include "renderer.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "info.wgsl.h"

/* Display parameters */

#define INFO_FONT_SIZE    16.0f    /* target font size in logical pixels */
#define INFO_MARGIN       12.0f    /* margin from bottom-left in logical px */
#define INFO_LINE_SPACING 1.4f     /* line height multiplier */
#define INFO_MAX_GLYPHS   256

/* Per-instance layout (32 bytes, matches shader) */

typedef struct {
    float sx, sy;              /* screen anchor in framebuffer pixels */
    float u0, v0, u1, v1;     /* atlas UV rect */
    float ox, oy;              /* offset normalized by glyph_scale */
} info_glyph_t;

/* Uniform layout (matches shader) */

typedef struct {
    float screen[2];
    float scale;
    float atlas_size;
    float glyph_scale;
    float display_scale;
    float px_range;     /* distance field range in atlas pixels */
    float _pad0;
} info_uniforms_t;

/* Internal struct */

struct arpt_info {
    WGPUDevice device;
    WGPUQueue queue;
    WGPURenderPipeline pipeline;
    WGPUBindGroupLayout bgl;
    WGPUBindGroup bind_group;
    WGPUBuffer uniform_buf;
    WGPUBuffer instance_buf;

    /* Font atlas */
    WGPUTexture font_texture;
    WGPUTextureView font_view;
    WGPUSampler font_sampler;
    font_glyph glyphs[FONT_CHAR_COUNT];
    float font_pixel_height;
    float font_px_range;
    float display_scale;

    uint32_t fb_width, fb_height;
    float pixel_ratio;

    /* Current text to render */
    info_glyph_t instances[INFO_MAX_GLYPHS];
    uint32_t glyph_count;
};

/* Text layout helpers */

static uint32_t layout_line(arpt_info *info, const char *text,
                            float anchor_x, float anchor_y) {
    uint32_t count = 0;
    float cursor = 0.0f;
    float font_size = info->font_pixel_height;

    const char *p = text;
    while (*p && info->glyph_count + count < INFO_MAX_GLYPHS) {
        uint32_t cp = font_utf8_decode(&p);
        if (cp < (uint32_t)FONT_FIRST_CHAR || cp > (uint32_t)FONT_LAST_CHAR)
            cp = (uint32_t)FONT_FIRST_CHAR;

        const font_glyph *g = &info->glyphs[cp - FONT_FIRST_CHAR];

        /* Skip space characters (no visible glyph) but advance cursor */
        if (g->width > 0.0f && g->height > 0.0f) {
            info_glyph_t *inst = &info->instances[info->glyph_count + count];
            inst->sx = anchor_x;
            inst->sy = anchor_y;
            inst->u0 = g->u0;
            inst->v0 = g->v0;
            inst->u1 = g->u1;
            inst->v1 = g->v1;
            inst->ox = (cursor + g->bearing_x) / font_size;
            inst->oy = g->bearing_y / font_size;
            count++;
        }
        cursor += g->advance;
    }
    return count;
}

/* Public API */

arpt_info *arpt_info_create(WGPUDevice device, WGPUQueue queue,
                            WGPUTextureFormat surface_format,
                            uint32_t fb_width, uint32_t fb_height,
                            float pixel_ratio) {
    arpt_info *info = calloc(1, sizeof(*info));
    if (!info) return NULL;

    info->device = device;
    info->queue = queue;
    info->fb_width = fb_width;
    info->fb_height = fb_height;
    info->pixel_ratio = pixel_ratio;

    /* Generate font atlas */
    size_t atlas_bytes = FONT_ATLAS_SIZE * FONT_ATLAS_SIZE * 4;
    uint8_t *atlas_data = malloc(atlas_bytes);
    if (!atlas_data) { free(info); return NULL; }

    info->font_pixel_height = font_load_atlas(atlas_data, info->glyphs,
                                              &info->font_px_range);
    info->display_scale = (info->font_pixel_height > 0.0f)
        ? INFO_FONT_SIZE / info->font_pixel_height : 1.0f;

    /* Font texture */
    WGPUTextureDescriptor td = {
        .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
        .size = {FONT_ATLAS_SIZE, FONT_ATLAS_SIZE, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    info->font_texture = wgpuDeviceCreateTexture(device, &td);
    info->font_view = wgpuTextureCreateView(info->font_texture, NULL);

    WGPUImageCopyTexture dst = {.texture = info->font_texture};
    WGPUTextureDataLayout layout = {
        .bytesPerRow = FONT_ATLAS_SIZE * 4,
        .rowsPerImage = FONT_ATLAS_SIZE,
    };
    WGPUExtent3D extent = {FONT_ATLAS_SIZE, FONT_ATLAS_SIZE, 1};
    wgpuQueueWriteTexture(queue, &dst, atlas_data, atlas_bytes,
                          &layout, &extent);
    free(atlas_data);

    /* Sampler */
    WGPUSamplerDescriptor sd = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
    };
    info->font_sampler = wgpuDeviceCreateSampler(device, &sd);

    /* Shader module */
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = info_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    /* Bind group layout: uniform + texture + sampler */
    WGPUBindGroupLayoutEntry bgl_entries[] = {
        {.binding = 0,
         .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
         .buffer = {.type = WGPUBufferBindingType_Uniform,
                    .minBindingSize = sizeof(info_uniforms_t)}},
        {.binding = 1,
         .visibility = WGPUShaderStage_Fragment,
         .texture = {.sampleType = WGPUTextureSampleType_Float,
                     .viewDimension = WGPUTextureViewDimension_2D}},
        {.binding = 2,
         .visibility = WGPUShaderStage_Fragment,
         .sampler = {.type = WGPUSamplerBindingType_Filtering}},
    };
    info->bgl = wgpuDeviceCreateBindGroupLayout(
        device,
        &(WGPUBindGroupLayoutDescriptor){.entryCount = 3,
                                          .entries = bgl_entries});

    /* Pipeline layout */
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 1,
                                                .bindGroupLayouts = &info->bgl});

    /* Vertex layout: per-instance attributes */
    WGPUVertexAttribute inst_attrs[] = {
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 0,
         .shaderLocation = 0},  /* screen_xy */
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 8,
         .shaderLocation = 1},  /* uv_rect */
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 24,
         .shaderLocation = 2},  /* offset */
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = sizeof(info_glyph_t),
        .stepMode = WGPUVertexStepMode_Instance,
        .attributeCount = 3,
        .attributes = inst_attrs,
    };

    /* Alpha blending (premultiplied) */
    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_One,
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
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct,
    };

    /* Depth: always pass, no write (overlay) */
    WGPUDepthStencilState ds = {
        .format = ARPT_DEPTH_FORMAT,
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
                   .bufferCount = 1,
                   .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleStrip,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = ARPT_MSAA_SAMPLES, .mask = ~0u},
    };
    info->pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);

    /* Uniform buffer */
    WGPUBufferDescriptor ubuf_desc = {
        .usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst,
        .size = sizeof(info_uniforms_t),
    };
    info->uniform_buf = wgpuDeviceCreateBuffer(device, &ubuf_desc);

    /* Instance buffer (dynamic, updated each frame) */
    WGPUBufferDescriptor ibuf_desc = {
        .usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst,
        .size = INFO_MAX_GLYPHS * sizeof(info_glyph_t),
    };
    info->instance_buf = wgpuDeviceCreateBuffer(device, &ibuf_desc);

    /* Bind group */
    WGPUBindGroupEntry bg_entries[] = {
        {.binding = 0, .buffer = info->uniform_buf, .offset = 0,
         .size = sizeof(info_uniforms_t)},
        {.binding = 1, .textureView = info->font_view},
        {.binding = 2, .sampler = info->font_sampler},
    };
    info->bind_group = wgpuDeviceCreateBindGroup(
        device,
        &(WGPUBindGroupDescriptor){.layout = info->bgl,
                                    .entryCount = 3,
                                    .entries = bg_entries});

    return info;
}

void arpt_info_free(arpt_info *info) {
    if (!info) return;
    if (info->bind_group) wgpuBindGroupRelease(info->bind_group);
    if (info->instance_buf) wgpuBufferRelease(info->instance_buf);
    if (info->uniform_buf) wgpuBufferRelease(info->uniform_buf);
    if (info->font_sampler) wgpuSamplerRelease(info->font_sampler);
    if (info->font_view) wgpuTextureViewRelease(info->font_view);
    if (info->font_texture) wgpuTextureRelease(info->font_texture);
    if (info->bgl) wgpuBindGroupLayoutRelease(info->bgl);
    if (info->pipeline) wgpuRenderPipelineRelease(info->pipeline);
    free(info);
}

void arpt_info_resize(arpt_info *info, uint32_t fb_width, uint32_t fb_height,
                      float pixel_ratio) {
    info->fb_width = fb_width;
    info->fb_height = fb_height;
    info->pixel_ratio = pixel_ratio;
}

void arpt_info_set_camera(arpt_info *info, double lon_deg, double lat_deg,
                          double altitude, double bearing_deg,
                          double tilt_deg, double zoom_level) {
    info->glyph_count = 0;

    (void)altitude;
    (void)bearing_deg;
    (void)tilt_deg;

    char line1[64];
    snprintf(line1, sizeof(line1), "Zoom: %.2f", zoom_level);

    char line2[64];
    snprintf(line2, sizeof(line2), "Longitude: %.4f", lon_deg);

    char line3[64];
    snprintf(line3, sizeof(line3), "Latitude: %.4f", lat_deg);

    /* Compute anchor positions in framebuffer pixels.
       Vertically centered with the UI controls (CY = 48 logical px from bottom). */
    float margin = INFO_MARGIN * info->pixel_ratio;
    float line_h = INFO_FONT_SIZE * INFO_LINE_SPACING * info->pixel_ratio;
    float fb_h = (float)info->fb_height;
    float center_y = fb_h - 48.0f * info->pixel_ratio;

    float anchor_x = margin;
    float anchor_y1 = center_y - line_h;  /* top line */
    float anchor_y2 = center_y;           /* middle line */
    float anchor_y3 = center_y + line_h;  /* bottom line */

    uint32_t n1 = layout_line(info, line1, anchor_x, anchor_y1);
    info->glyph_count += n1;

    uint32_t n2 = layout_line(info, line2, anchor_x, anchor_y2);
    info->glyph_count += n2;

    uint32_t n3 = layout_line(info, line3, anchor_x, anchor_y3);
    info->glyph_count += n3;
}

void arpt_info_draw(arpt_info *info, WGPURenderPassEncoder pass) {
    if (info->glyph_count == 0) return;

    /* Update uniforms */
    info_uniforms_t u = {
        .screen = {(float)info->fb_width, (float)info->fb_height},
        .scale = info->pixel_ratio,
        .atlas_size = (float)FONT_ATLAS_SIZE,
        .glyph_scale = info->font_pixel_height,
        .display_scale = info->display_scale * info->pixel_ratio,
        .px_range = info->font_px_range,
    };
    wgpuQueueWriteBuffer(info->queue, info->uniform_buf, 0, &u, sizeof(u));

    /* Upload glyph instances */
    size_t data_size = (size_t)info->glyph_count * sizeof(info_glyph_t);
    wgpuQueueWriteBuffer(info->queue, info->instance_buf, 0,
                         info->instances, data_size);

    /* Draw */
    wgpuRenderPassEncoderSetPipeline(pass, info->pipeline);
    wgpuRenderPassEncoderSetBindGroup(pass, 0, info->bind_group, 0, NULL);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, info->instance_buf, 0,
                                         data_size);
    wgpuRenderPassEncoderDraw(pass, 4, info->glyph_count, 0, 0);
}
