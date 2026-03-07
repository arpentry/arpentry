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
        .color = {.srcFactor = WGPUBlendFactor_One,
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
        .multisample = {.count = 4, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

/* Helper: create an SDF atlas texture + sampler + bind group */
static void init_sdf_atlas(arpt_renderer *r, uint8_t *atlas_data,
                            size_t atlas_size, float pixel_height,
                            float display_scale, float halo_width,
                            const float fill_color[4],
                            const float halo_color[4],
                            WGPUTexture *tex, WGPUTextureView *view,
                            WGPUSampler *samp, WGPUBuffer *ubuf,
                            WGPUBindGroup *bg) {
    size_t atlas_bytes = atlas_size * atlas_size * 4;

    WGPUTextureDescriptor td = {
        .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
        .size = {(uint32_t)atlas_size, (uint32_t)atlas_size, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    *tex = wgpuDeviceCreateTexture(r->device, &td);
    *view = wgpuTextureCreateView(*tex, NULL);

    WGPUImageCopyTexture dst = {.texture = *tex};
    WGPUTextureDataLayout layout = {
        .bytesPerRow = (uint32_t)(atlas_size * 4),
        .rowsPerImage = (uint32_t)atlas_size};
    WGPUExtent3D extent = {(uint32_t)atlas_size, (uint32_t)atlas_size, 1};
    wgpuQueueWriteTexture(r->queue, &dst, atlas_data, atlas_bytes,
                          &layout, &extent);

    WGPUSamplerDescriptor sd = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
    };
    *samp = wgpuDeviceCreateSampler(r->device, &sd);

    poi_uniforms_t pu = {
        .glyph_scale = pixel_height,
        .atlas_size = (float)atlas_size,
        .viewport_width = (float)r->width,
        .viewport_height = (float)r->height,
        .display_scale = display_scale,
        .halo_width = halo_width,
    };
    memcpy(pu.fill_color, fill_color, sizeof(pu.fill_color));
    memcpy(pu.halo_color, halo_color, sizeof(pu.halo_color));
    *ubuf = create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform,
                          &pu, sizeof(poi_uniforms_t));

    WGPUBindGroupEntry entries[] = {
        {.binding = 0, .buffer = *ubuf, .offset = 0,
         .size = sizeof(poi_uniforms_t)},
        {.binding = 1, .textureView = *view},
        {.binding = 2, .sampler = *samp},
    };
    *bg = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->poi_bgl,
                                               .entryCount = 3,
                                               .entries = entries});
}

void arpt__label_init_font(arpt_renderer *r) {
    /* Font atlas */
    size_t font_bytes = FONT_ATLAS_SIZE * FONT_ATLAS_SIZE * 4;
    uint8_t *font_data = malloc(font_bytes);
    if (!font_data) return;

    r->font_pixel_height = font_generate_atlas(font_data, r->glyphs);

    /* Compute display scales from style */
    r->text_display_scale = (r->font_pixel_height > 0)
        ? r->text_size / r->font_pixel_height : 1.0f;

    init_sdf_atlas(r, font_data, FONT_ATLAS_SIZE, r->font_pixel_height,
                   r->text_display_scale, r->text_halo_width,
                   r->text_color, r->text_halo_color,
                   &r->font_texture, &r->font_view, &r->font_sampler,
                   &r->poi_uniform_buf, &r->poi_bind_group);
    free(font_data);

    /* Icon atlas */
    size_t icon_bytes = ICON_ATLAS_SIZE * ICON_ATLAS_SIZE * 4;
    uint8_t *icon_data = malloc(icon_bytes);
    if (!icon_data) return;

    r->icon_pixel_height = icon_generate_atlas(icon_data, r->icon_glyphs,
                                                &r->icon_glyph_count);
    r->icon_display_scale = (r->icon_pixel_height > 0)
        ? r->icon_size / r->icon_pixel_height : 1.0f;

    init_sdf_atlas(r, icon_data, ICON_ATLAS_SIZE, r->icon_pixel_height,
                   r->icon_display_scale, r->icon_halo_width,
                   r->icon_color, r->icon_halo_color,
                   &r->icon_texture, &r->icon_view, &r->icon_sampler,
                   &r->icon_uniform_buf, &r->icon_bind_group);
    free(icon_data);
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

static void upload_instances(arpt_renderer *r, WGPUBuffer *buf,
                              uint32_t *count, const void *src,
                              size_t n, size_t elem_size) {
    poi_instance_t *instances = malloc(n * sizeof(poi_instance_t));
    if (!instances) return;

    /* Both arpt_glyph_inst and arpt_icon_inst have the same layout */
    const arpt_glyph_inst *glyphs = src;
    for (size_t i = 0; i < n; i++) {
        instances[i].qx = glyphs[i].qx;
        instances[i].qy = glyphs[i].qy;
        instances[i].qz = glyphs[i].qz;
        instances[i].u0 = glyphs[i].u0;
        instances[i].v0 = glyphs[i].v0;
        instances[i].u1 = glyphs[i].u1;
        instances[i].v1 = glyphs[i].v1;
        instances[i].offset_x = glyphs[i].ox;
        instances[i].offset_y = glyphs[i].oy;
    }

    (void)elem_size;
    *buf = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                         instances, n * sizeof(poi_instance_t));
    *count = (uint32_t)n;
    free(instances);
}

void arpt__label_upload(arpt_renderer *r, arpt_tile_gpu *t,
                        const arpt_label_prim *prim) {
    if (!prim) return;

    /* Upload text glyph instances */
    if (prim->glyph_count > 0) {
        upload_instances(r, &t->poi_instance_buf, &t->poi_instance_count,
                         prim->glyphs, prim->glyph_count,
                         sizeof(arpt_glyph_inst));
    }

    /* Upload icon instances */
    if (prim->icon_count > 0) {
        upload_instances(r, &t->icon_instance_buf, &t->icon_instance_count,
                         prim->icons, prim->icon_count,
                         sizeof(arpt_icon_inst));
    }

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

void arpt__label_collect(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->poi_label_count == 0 || !r->poi_pipeline) return;

    const float *proj = r->cached_projection.m;
    const float *mdl = tile->cached_model;
    float vw = (float)r->width;
    float vh = (float)r->height;

    for (int li = 0; li < tile->poi_label_count; li++) {
        if (r->pending_label_count >= 512) break;

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
        float cz = proj[2]*mx + proj[6]*my + proj[10]*mz + proj[14]*mw;
        float cw = proj[3]*mx + proj[7]*my + proj[11]*mz + proj[15]*mw;

        if (cw <= 0.0f || cz < 0.0f) continue;

        float sx = (cx / cw * 0.5f + 0.5f) * vw;
        float sy = (1.0f - (cy / cw * 0.5f + 0.5f)) * vh;

        float tds = r->text_display_scale;
        float hw = tile->poi_labels[li].label_w_px * 0.5f * tds;
        float lh = tile->poi_labels[li].label_h_px * tds;
        float icon_h = (tile->icon_instance_count > 0) ? r->icon_size : 0.0f;
        float pad = 4.0f;

        int idx = r->pending_label_count++;
        r->pending_labels[idx].tile = tile;
        r->pending_labels[idx].label_index = li;
        r->pending_labels[idx].depth = cz / cw;
        r->pending_labels[idx].x0 = sx - hw - pad;
        r->pending_labels[idx].y0 = sy - lh - icon_h - pad;
        r->pending_labels[idx].x1 = sx + hw + pad;
        r->pending_labels[idx].y1 = sy + pad;
    }
}

static int compare_pending_depth(const void *a, const void *b) {
    float da = ((const arpt_pending_label *)a)->depth;
    float db = ((const arpt_pending_label *)b)->depth;
    return (da > db) - (da < db);
}

void arpt__label_draw_all(arpt_renderer *r) {
    if (r->pending_label_count == 0 || !r->poi_pipeline) return;

    /* Sort candidates by depth (closest first) */
    qsort(r->pending_labels, (size_t)r->pending_label_count,
          sizeof(r->pending_labels[0]), compare_pending_depth);

    /* Resolve collisions: closest labels win */
    int visible_indices[512];
    int n_visible = 0;

    for (int i = 0; i < r->pending_label_count; i++) {
        float x0 = r->pending_labels[i].x0;
        float y0 = r->pending_labels[i].y0;
        float x1 = r->pending_labels[i].x1;
        float y1 = r->pending_labels[i].y1;

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

        if (n_visible < 512) visible_indices[n_visible++] = i;
    }

    if (n_visible == 0) return;

    bool drew_any = false;

    /* Draw icons first, then text — group draw calls by tile to minimize
     * bind group switches. */

    /* Icons pass */
    if (r->icon_bind_group) {
        arpt_tile_gpu *cur_tile = NULL;
        for (int vi = 0; vi < n_visible; vi++) {
            int idx = visible_indices[vi];
            arpt_tile_gpu *tile = r->pending_labels[idx].tile;
            int li = r->pending_labels[idx].label_index;

            if (!tile->icon_instance_buf || tile->icon_instance_count == 0)
                continue;
            if (li >= (int)tile->icon_instance_count) continue;

            if (tile != cur_tile) {
                wgpuRenderPassEncoderSetPipeline(r->pass, r->poi_pipeline);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 0,
                                                  r->global_bind_group, 0, NULL);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group,
                                                  0, NULL);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 2, r->icon_bind_group,
                                                  0, NULL);
                wgpuRenderPassEncoderSetVertexBuffer(
                    r->pass, 0, tile->icon_instance_buf, 0,
                    wgpuBufferGetSize(tile->icon_instance_buf));
                cur_tile = tile;
            }
            wgpuRenderPassEncoderDraw(r->pass, 4, 1, 0, (uint32_t)li);
            drew_any = true;
        }
    }

    /* Text pass */
    {
        arpt_tile_gpu *cur_tile = NULL;
        for (int vi = 0; vi < n_visible; vi++) {
            int idx = visible_indices[vi];
            arpt_tile_gpu *tile = r->pending_labels[idx].tile;
            int li = r->pending_labels[idx].label_index;

            if (!tile->poi_instance_buf || tile->poi_instance_count == 0)
                continue;

            if (tile != cur_tile) {
                wgpuRenderPassEncoderSetPipeline(r->pass, r->poi_pipeline);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 0,
                                                  r->global_bind_group, 0, NULL);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group,
                                                  0, NULL);
                wgpuRenderPassEncoderSetBindGroup(r->pass, 2, r->poi_bind_group,
                                                  0, NULL);
                wgpuRenderPassEncoderSetVertexBuffer(
                    r->pass, 0, tile->poi_instance_buf, 0,
                    wgpuBufferGetSize(tile->poi_instance_buf));
                cur_tile = tile;
            }
            wgpuRenderPassEncoderDraw(
                r->pass, 4, tile->poi_labels[li].instance_count, 0,
                tile->poi_labels[li].first_instance);
            drew_any = true;
        }
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
    if (r->icon_bind_group) wgpuBindGroupRelease(r->icon_bind_group);
    if (r->icon_uniform_buf) wgpuBufferRelease(r->icon_uniform_buf);
    if (r->icon_view) wgpuTextureViewRelease(r->icon_view);
    if (r->icon_texture) wgpuTextureRelease(r->icon_texture);
    if (r->icon_sampler) wgpuSamplerRelease(r->icon_sampler);
    if (r->poi_bgl) wgpuBindGroupLayoutRelease(r->poi_bgl);
}
