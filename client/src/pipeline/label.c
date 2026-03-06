#include "internal.h"
#include "globe.h"

#include "poi.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__label_create_pipeline(WGPUDevice device,
                                                WGPUTextureFormat format,
                                                WGPUBindGroupLayout global_bgl,
                                                WGPUBindGroupLayout tile_bgl,
                                                WGPUBindGroupLayout poi_bgl) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = poi_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    WGPUBindGroupLayout bgls[] = {global_bgl, tile_bgl, poi_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 3,
                                                .bindGroupLayouts = bgls});

    WGPUVertexAttribute inst_attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2,
         .offset = 0,
         .shaderLocation = 0},
        {.format = WGPUVertexFormat_Sint32,
         .offset = 4,
         .shaderLocation = 1},
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 8,
         .shaderLocation = 2},
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 24,
         .shaderLocation = 3},
    };

    WGPUVertexBufferLayout vbl = {
        .arrayStride = 32,
        .stepMode = WGPUVertexStepMode_Instance,
        .attributeCount = 4,
        .attributes = inst_attrs,
    };

    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
        .alpha = {.srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
    };
    WGPUColorTargetState ct = {.format = format,
                               .blend = &blend,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24Plus,
        .depthWriteEnabled = true,
        .depthCompare = WGPUCompareFunction_LessEqual,
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
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__label_init_font(arpt_renderer *r) {
    size_t atlas_bytes = FONT_ATLAS_SIZE * FONT_ATLAS_SIZE * 4;
    uint8_t *atlas_data = malloc(atlas_bytes);
    if (!atlas_data) return;

    r->font_pixel_height = font_generate_atlas(atlas_data, r->glyphs);

    WGPUTextureDescriptor ftd = {
        .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
        .size = {FONT_ATLAS_SIZE, FONT_ATLAS_SIZE, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    r->font_texture = wgpuDeviceCreateTexture(r->device, &ftd);
    r->font_view = wgpuTextureCreateView(r->font_texture, NULL);

    WGPUImageCopyTexture dst = {.texture = r->font_texture};
    WGPUTextureDataLayout layout = {
        .bytesPerRow = FONT_ATLAS_SIZE * 4,
        .rowsPerImage = FONT_ATLAS_SIZE};
    WGPUExtent3D extent = {FONT_ATLAS_SIZE, FONT_ATLAS_SIZE, 1};
    wgpuQueueWriteTexture(r->queue, &dst, atlas_data, atlas_bytes,
                          &layout, &extent);
    free(atlas_data);

    WGPUSamplerDescriptor fsd = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
    };
    r->font_sampler = wgpuDeviceCreateSampler(r->device, &fsd);

    poi_uniforms_t pu = {
        .glyph_scale = r->font_pixel_height,
        .atlas_size = (float)FONT_ATLAS_SIZE,
        .viewport_width = (float)r->width,
        .viewport_height = (float)r->height,
    };
    r->poi_uniform_buf =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform, &pu,
                      sizeof(poi_uniforms_t));

    WGPUBindGroupEntry poi_bg_entries[] = {
        {.binding = 0,
         .buffer = r->poi_uniform_buf,
         .offset = 0,
         .size = sizeof(poi_uniforms_t)},
        {.binding = 1, .textureView = r->font_view},
        {.binding = 2, .sampler = r->font_sampler},
    };
    r->poi_bind_group = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->poi_bgl,
                                               .entryCount = 3,
                                               .entries = poi_bg_entries});
}

/* POI GPU instance layout: matches arpt_glyph_inst but with GPU-friendly
 * field names (offset_x/offset_y instead of ox/oy). 32 bytes per instance. */

typedef struct {
    uint16_t qx, qy;
    int32_t qz;
    float u0, v0, u1, v1;
    float offset_x;
    float offset_y;
} poi_instance_t;

void arpt__label_upload(arpt_renderer *r, arpt_tile_gpu *t,
                        const arpt_label_prim *prim) {
    (void)r;
    if (!prim || prim->glyph_count == 0) return;

    /* Convert arpt_glyph_inst to GPU layout */
    poi_instance_t *instances = malloc(prim->glyph_count * sizeof(poi_instance_t));
    if (!instances) return;

    for (size_t i = 0; i < prim->glyph_count; i++) {
        const arpt_glyph_inst *g = &prim->glyphs[i];
        instances[i].qx = g->qx;
        instances[i].qy = g->qy;
        instances[i].qz = g->qz;
        instances[i].u0 = g->u0;
        instances[i].v0 = g->v0;
        instances[i].u1 = g->u1;
        instances[i].v1 = g->v1;
        instances[i].offset_x = g->ox;
        instances[i].offset_y = g->oy;
    }

    t->poi_instance_buf =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                      instances, prim->glyph_count * sizeof(poi_instance_t));
    t->poi_instance_count = (uint32_t)prim->glyph_count;
    free(instances);

    /* Copy per-label metadata for CPU-side collision detection */
    if (prim->label_count > 0) {
        t->poi_labels = malloc((size_t)prim->label_count * sizeof(*t->poi_labels));
        if (t->poi_labels) {
            t->poi_label_count = prim->label_count;
            for (int i = 0; i < prim->label_count; i++) {
                const arpt_label_meta *lm = &prim->labels[i];
                t->poi_labels[i].qx = lm->qx;
                t->poi_labels[i].qy = lm->qy;
                t->poi_labels[i].qz = lm->qz;
                t->poi_labels[i].label_w_px = lm->w_px;
                t->poi_labels[i].label_h_px = lm->h_px;
                t->poi_labels[i].first_instance = lm->first;
                t->poi_labels[i].instance_count = lm->count;
            }
        }
    }
}

void arpt__label_draw(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->poi_label_count == 0 || !r->poi_pipeline) return;

    bool drew_any = false;
    const float *proj = r->cached_projection.m;
    const float *mdl = tile->cached_model;
    float vw = (float)r->width;
    float vh = (float)r->height;

    for (int li = 0; li < tile->poi_label_count; li++) {
        float lon_w = tile->cached_bounds[0];
        float lat_s = tile->cached_bounds[1];
        float lon_e = tile->cached_bounds[2];
        float lat_n = tile->cached_bounds[3];
        float u = ((float)tile->poi_labels[li].qx - 16384.0f) / 32768.0f;
        float v = ((float)tile->poi_labels[li].qy - 16384.0f) / 32768.0f;
        double lon = lon_w + u * (lon_e - lon_w);
        double lat = lat_s + v * (lat_n - lat_s);
        double alt = (double)tile->poi_labels[li].qz * 0.001;

        arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon, lat, alt);
        arpt_dvec3 center_ecef = arpt_geodetic_to_ecef(
            (double)tile->cached_center_lon,
            (double)tile->cached_center_lat, 0.0);
        float lx = (float)(ecef.x - center_ecef.x);
        float ly = (float)(ecef.y - center_ecef.y);
        float lz = (float)(ecef.z - center_ecef.z);

        float mx = mdl[0]*lx + mdl[4]*ly + mdl[8]*lz + mdl[12];
        float my = mdl[1]*lx + mdl[5]*ly + mdl[9]*lz + mdl[13];
        float mz = mdl[2]*lx + mdl[6]*ly + mdl[10]*lz + mdl[14];
        float mw = mdl[3]*lx + mdl[7]*ly + mdl[11]*lz + mdl[15];

        float cx = proj[0]*mx + proj[4]*my + proj[8]*mz + proj[12]*mw;
        float cy = proj[1]*mx + proj[5]*my + proj[9]*mz + proj[13]*mw;
        float cw = proj[3]*mx + proj[7]*my + proj[11]*mz + proj[15]*mw;

        if (cw <= 0.0f) continue;

        float sx = (cx / cw * 0.5f + 0.5f) * vw;
        float sy = (1.0f - (cy / cw * 0.5f + 0.5f)) * vh;

        float hw = tile->poi_labels[li].label_w_px * 0.5f;
        float lh = tile->poi_labels[li].label_h_px;
        float pad = 4.0f;
        float x0 = sx - hw - pad;
        float y0 = sy - lh - pad;
        float x1 = sx + hw + pad;
        float y1 = sy + pad;

        bool collides = false;
        for (int pi = 0; pi < r->placed_label_count; pi++) {
            if (x0 < r->placed_labels[pi].x1 &&
                x1 > r->placed_labels[pi].x0 &&
                y0 < r->placed_labels[pi].y1 &&
                y1 > r->placed_labels[pi].y0) {
                collides = true;
                break;
            }
        }
        if (collides) continue;

        if (r->placed_label_count < 512) {
            r->placed_labels[r->placed_label_count].x0 = x0;
            r->placed_labels[r->placed_label_count].y0 = y0;
            r->placed_labels[r->placed_label_count].x1 = x1;
            r->placed_labels[r->placed_label_count].y1 = y1;
            r->placed_label_count++;
        }

        if (!drew_any) {
            wgpuRenderPassEncoderSetPipeline(r->pass, r->poi_pipeline);
            wgpuRenderPassEncoderSetBindGroup(r->pass, 0,
                                              r->global_bind_group, 0,
                                              NULL);
            wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group,
                                              0, NULL);
            wgpuRenderPassEncoderSetBindGroup(r->pass, 2, r->poi_bind_group,
                                              0, NULL);
            wgpuRenderPassEncoderSetVertexBuffer(
                r->pass, 0, tile->poi_instance_buf, 0,
                wgpuBufferGetSize(tile->poi_instance_buf));
            drew_any = true;
        }
        wgpuRenderPassEncoderDraw(
            r->pass, 4, tile->poi_labels[li].instance_count, 0,
            tile->poi_labels[li].first_instance);
    }

    if (drew_any) {
        wgpuRenderPassEncoderSetPipeline(r->pass, r->pipeline);
        wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group,
                                          0, NULL);
    }
}

void arpt__label_cleanup(arpt_renderer *r) {
    if (r->poi_pipeline) wgpuRenderPipelineRelease(r->poi_pipeline);
    if (r->poi_bind_group) wgpuBindGroupRelease(r->poi_bind_group);
    if (r->poi_uniform_buf) wgpuBufferRelease(r->poi_uniform_buf);
    if (r->font_view) wgpuTextureViewRelease(r->font_view);
    if (r->font_texture) wgpuTextureRelease(r->font_texture);
    if (r->font_sampler) wgpuSamplerRelease(r->font_sampler);
    if (r->poi_bgl) wgpuBindGroupLayoutRelease(r->poi_bgl);
}
