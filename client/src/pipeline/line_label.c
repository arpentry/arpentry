/* Line-following labels (street names).
 *
 * Tiles keep each named road polyline CPU-side (arpt_line_label). Every
 * frame, arpt__line_label_collect projects the polyline to screen space,
 * lays the glyphs out along the projected path (reading left-to-right,
 * skipping lines that are too short or bend too sharply), and queues the
 * label for the shared depth-sorted collision pass in label.c. Winning
 * labels' glyph instances are uploaded to a dynamic vertex buffer and drawn
 * by arpt__line_label_draw with a screen-space MTSDF pipeline that shares
 * the POI font atlas.
 */

#include "internal.h"
#include "globe.h"

#include "line_label.wgsl.h"

#include <stdlib.h>

static WGPURenderPipeline create_pipeline(WGPUDevice device,
                                          WGPUTextureFormat format,
                                          WGPUBindGroupLayout global_bgl,
                                          WGPUBindGroupLayout poi_bgl) {
    WGPUShaderModule sm = create_shader(device, line_label_wgsl);
    if (!sm) return NULL;

    WGPUBindGroupLayout bgls[] = {global_bgl, poi_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 2,
                                                .bindGroupLayouts = bgls});

    WGPUVertexAttribute inst_attrs[] = {
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 0,
         .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 8,
         .shaderLocation = 1},
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 16,
         .shaderLocation = 2},
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 24,
         .shaderLocation = 3},
    };

    WGPUVertexBufferLayout vbl = {
        .arrayStride = sizeof(arpt_line_glyph_inst),
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
    /* Screen-space overlay: drawn after everything else, no depth test. */
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
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__line_label_update_uniforms(arpt_renderer *r) {
    if (!r->line_label_ubuf) return;
    poi_uniforms_t pu = {
        .glyph_scale = r->font_pixel_height,
        .atlas_size = (float)FONT_ATLAS_SIZE,
        .viewport_width = (float)r->width,
        .viewport_height = (float)r->height,
        .display_scale = 1.0f, /* instances arrive in framebuffer pixels */
        .halo_width = r->line_text_halo_width * r->pixel_ratio,
        .px_range = r->font_px_range,
    };
    memcpy(pu.fill_color, r->line_text_color, sizeof(pu.fill_color));
    memcpy(pu.halo_color, r->line_text_halo_color, sizeof(pu.halo_color));
    wgpuQueueWriteBuffer(r->queue, r->line_label_ubuf, 0, &pu,
                         sizeof(poi_uniforms_t));
}

void arpt__line_label_init(arpt_renderer *r) {
    /* Shares the font atlas and bind group layout created by label.c. */
    if (!r->font_view || !r->font_sampler || !r->poi_bgl) return;

    r->line_label_pipeline = create_pipeline(r->device, r->surface_format,
                                             r->global_bgl, r->poi_bgl);

    r->line_label_ubuf = create_buffer(r->device, r->queue,
                                       WGPUBufferUsage_Uniform, NULL,
                                       sizeof(poi_uniforms_t));
    arpt__line_label_update_uniforms(r);

    WGPUBindGroupEntry entries[] = {
        {.binding = 0, .buffer = r->line_label_ubuf, .offset = 0,
         .size = sizeof(poi_uniforms_t)},
        {.binding = 1, .textureView = r->font_view},
        {.binding = 2, .sampler = r->font_sampler},
    };
    r->line_label_bind_group = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->poi_bgl,
                                              .entryCount = 3,
                                              .entries = entries});

    r->line_glyph_scratch =
        malloc(ARPT_MAX_LINE_GLYPHS * sizeof(arpt_line_glyph_inst));
    r->line_glyph_out =
        malloc(ARPT_MAX_LINE_GLYPHS * sizeof(arpt_line_glyph_inst));
}

void arpt__line_label_upload(arpt_renderer *r, arpt_tile_gpu *t,
                             const arpt_line_label_prim *prim) {
    (void)r;
    if (!prim || prim->count <= 0) return;

    t->line_labels = calloc((size_t)prim->count, sizeof(arpt_line_label));
    if (!t->line_labels) return;

    int n = 0;
    for (int i = 0; i < prim->count; i++) {
        const arpt_line_label *src = &prim->labels[i];
        if (!src->x || !src->y || src->vertex_count < 2) continue;
        arpt_line_label *dst = &t->line_labels[n];
        size_t bytes = src->vertex_count * sizeof(uint16_t);
        dst->x = malloc(bytes);
        dst->y = malloc(bytes);
        if (!dst->x || !dst->y) {
            free(dst->x);
            free(dst->y);
            dst->x = dst->y = NULL;
            continue;
        }
        memcpy(dst->x, src->x, bytes);
        memcpy(dst->y, src->y, bytes);
        dst->vertex_count = src->vertex_count;
        dst->qz = src->qz;
        memcpy(dst->name, src->name, sizeof(dst->name));
        dst->text_w_px = src->text_w_px;
        n++;
    }
    t->line_label_count = n;
}

/* Point and unit direction at arc distance `s` along the projected
 * polyline. `flip` walks from the far end with the direction negated, so a
 * label on a right-to-left line still reads left-to-right. */
static void sample_at(const float *sx, const float *sy, const float *cums,
                      int n, float s, bool flip,
                      float *px, float *py, float *dx, float *dy) {
    float total = cums[n - 1];
    if (flip) s = total - s;
    if (s < 0.0f) s = 0.0f;
    if (s > total) s = total;

    int i = 0;
    while (i < n - 2 && cums[i + 1] < s) i++;

    float seg = cums[i + 1] - cums[i];
    float t = (seg > 1e-6f) ? (s - cums[i]) / seg : 0.0f;
    float ex = sx[i + 1] - sx[i];
    float ey = sy[i + 1] - sy[i];
    *px = sx[i] + ex * t;
    *py = sy[i] + ey * t;

    float ddx = 1.0f, ddy = 0.0f;
    if (seg > 1e-6f) {
        ddx = ex / seg;
        ddy = ey / seg;
    }
    if (flip) {
        ddx = -ddx;
        ddy = -ddy;
    }
    *dx = ddx;
    *dy = ddy;
}

/* Adjacent glyphs may turn at most 45° relative to each other; sharper
 * bends produce overlapping, unreadable glyphs, so the label is dropped. */
#define LINE_LABEL_MAX_TURN_COS 0.70710678f

/* Extra screen-space length the line must have beyond the text width. */
#define LINE_LABEL_FIT_SLACK_PX 8.0f

void arpt__line_label_collect(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->line_label_count == 0 || !r->line_label_pipeline ||
        !r->line_glyph_scratch || r->font_pixel_height <= 0.0f)
        return;

    const float *proj = r->cached_projection.m;
    const float *mdl = tile->cached_model;
    float vw = (float)r->width;
    float vh = (float)r->height;

    float lon_w = tile->cached_bounds[0];
    float lat_s = tile->cached_bounds[1];
    float lon_e = tile->cached_bounds[2];
    float lat_n = tile->cached_bounds[3];

    arpt_dvec3 center_ecef = arpt_geodetic_to_ecef(
        (double)tile->cached_center_lon, (double)tile->cached_center_lat, 0.0);

    float text_px = r->line_text_size * r->pixel_ratio;
    float scale = text_px / r->font_pixel_height;
    /* Baseline sits below the line center so cap-height text is optically
       centered on the road. */
    float baseline = 0.36f * text_px;

    for (int li = 0; li < tile->line_label_count; li++) {
        if (r->pending_label_count >= ARPT_MAX_PENDING_LABELS) break;

        const arpt_line_label *ll = &tile->line_labels[li];
        uint32_t n = ll->vertex_count;
        if (n < 2 || n > ARPT_MAX_LINE_LABEL_POINTS) continue;

        float text_w = ll->text_w_px * scale;
        if (text_w <= 0.0f) continue;

        /* Project the polyline to screen space. */
        float sx[ARPT_MAX_LINE_LABEL_POINTS];
        float sy[ARPT_MAX_LINE_LABEL_POINTS];
        float cums[ARPT_MAX_LINE_LABEL_POINTS];
        double alt = (double)ll->qz * 0.001;
        float depth = 0.0f;
        bool visible = true;

        for (uint32_t i = 0; i < n; i++) {
            float u = ((float)ll->x[i] - (float)ARPT_BUFFER) /
                      (float)ARPT_EXTENT;
            float v = ((float)ll->y[i] - (float)ARPT_BUFFER) /
                      (float)ARPT_EXTENT;
            double lon = lon_w + u * (lon_e - lon_w);
            double lat = lat_s + v * (lat_n - lat_s);
            arpt_dvec3 ecef = arpt_geodetic_to_ecef(lon, lat, alt);

            if (i == n / 2) {
                /* Horizon culling at the midpoint (see label.c). */
                double cos_lat = cos(lat), sin_lat = sin(lat);
                double cos_lon = cos(lon), sin_lon = sin(lon);
                double nx = cos_lat * cos_lon;
                double ny = cos_lat * sin_lon;
                double nz = sin_lat;
                double ddx = r->camera_ecef[0] - ecef.x;
                double ddy = r->camera_ecef[1] - ecef.y;
                double ddz = r->camera_ecef[2] - ecef.z;
                if (nx * ddx + ny * ddy + nz * ddz < 0.0) {
                    visible = false;
                    break;
                }
            }

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
            if (cw <= 0.0f || cz > cw) {  /* behind the camera, see label.c */
                visible = false;
                break;
            }

            sx[i] = (cx / cw * 0.5f + 0.5f) * vw;
            sy[i] = (1.0f - (cy / cw * 0.5f + 0.5f)) * vh;
            cums[i] = (i == 0)
                ? 0.0f
                : cums[i - 1] + hypotf(sx[i] - sx[i - 1], sy[i] - sy[i - 1]);
            if (i == n / 2) depth = cz / cw;
        }
        if (!visible) continue;

        float total = cums[n - 1];
        if (total < text_w + LINE_LABEL_FIT_SLACK_PX) continue;

        /* Center the text on the line; flip the walk when it would read
           right-to-left on screen. */
        float s0 = (total - text_w) * 0.5f;
        float ax, ay, adx, ady, bx, by, bdx, bdy;
        sample_at(sx, sy, cums, (int)n, s0, false, &ax, &ay, &adx, &ady);
        sample_at(sx, sy, cums, (int)n, s0 + text_w, false,
                  &bx, &by, &bdx, &bdy);
        bool flip = bx < ax;

        /* Lay the glyphs out along the path. */
        uint32_t first = (uint32_t)r->line_glyph_scratch_count;
        float bx0 = 1e30f, by0 = 1e30f, bx1 = -1e30f, by1 = -1e30f;
        bool ok = true, has_prev = false;
        float prev_dx = 0.0f, prev_dy = 0.0f;
        float cursor = 0.0f;
        const char *sp = ll->name;

        while (*sp) {
            uint32_t cp = font_utf8_decode(&sp);
            if (cp < (uint32_t)FONT_FIRST_CHAR ||
                cp > (uint32_t)FONT_LAST_CHAR)
                cp = FONT_FIRST_CHAR;
            const font_glyph *g = &r->glyphs[cp - FONT_FIRST_CHAR];

            if (g->width > 0) {
                if (r->line_glyph_scratch_count >= ARPT_MAX_LINE_GLYPHS) {
                    ok = false;
                    break;
                }
                float s_mid = s0 +
                    (cursor + g->bearing_x + g->width * 0.5f) * scale;
                float px, py, dx, dy;
                sample_at(sx, sy, cums, (int)n, s_mid, flip,
                          &px, &py, &dx, &dy);

                if (has_prev &&
                    dx * prev_dx + dy * prev_dy < LINE_LABEL_MAX_TURN_COS) {
                    ok = false;
                    break;
                }
                prev_dx = dx;
                prev_dy = dy;
                has_prev = true;

                /* Glyph center, offset perpendicular to the line: bitmap
                   center relative to the baseline plus the baseline shift
                   (screen y grows downward; perpendicular = rotate (0,1)). */
                float cy_local =
                    (g->bearing_y + g->height * 0.5f) * scale + baseline;
                float cxp = px - dy * cy_local;
                float cyp = py + dx * cy_local;

                float w = g->width * scale;
                float h = g->height * scale;
                arpt_line_glyph_inst *gi =
                    &r->line_glyph_scratch[r->line_glyph_scratch_count++];
                gi->x = cxp;
                gi->y = cyp;
                gi->cos_a = dx;
                gi->sin_a = dy;
                gi->w = w;
                gi->h = h;
                gi->u0 = g->u0;
                gi->v0 = g->v0;
                gi->u1 = g->u1;
                gi->v1 = g->v1;

                /* Axis-aligned extent of the rotated quad. */
                float ex = 0.5f * (fabsf(w * dx) + fabsf(h * dy));
                float ey = 0.5f * (fabsf(w * dy) + fabsf(h * dx));
                if (cxp - ex < bx0) bx0 = cxp - ex;
                if (cyp - ey < by0) by0 = cyp - ey;
                if (cxp + ex > bx1) bx1 = cxp + ex;
                if (cyp + ey > by1) by1 = cyp + ey;
            }
            cursor += g->advance;
        }

        uint32_t count = (uint32_t)r->line_glyph_scratch_count - first;
        if (!ok || count == 0) {
            r->line_glyph_scratch_count = (int)first;
            continue;
        }

        float pad = 2.0f;
        bx0 -= pad;
        by0 -= pad;
        bx1 += pad;
        by1 += pad;
        if (bx1 < 0.0f || bx0 > vw || by1 < 0.0f || by0 > vh) {
            r->line_glyph_scratch_count = (int)first;
            continue;
        }

        int idx = r->pending_label_count++;
        r->pending_labels[idx].tile = tile;
        r->pending_labels[idx].label_index = li;
        r->pending_labels[idx].depth = depth;
        r->pending_labels[idx].x0 = bx0;
        r->pending_labels[idx].y0 = by0;
        r->pending_labels[idx].x1 = bx1;
        r->pending_labels[idx].y1 = by1;
        r->pending_labels[idx].kind = 1;
        r->pending_labels[idx].glyph_first = first;
        r->pending_labels[idx].glyph_count = count;
    }
}

void arpt__line_label_draw(arpt_renderer *r) {
    int count = r->line_glyph_out_count;
    if (count == 0 || !r->line_label_pipeline || !r->line_label_bind_group)
        return;

    /* Grow the instance buffer to the high-water mark. */
    if (!r->line_label_vbuf || r->line_label_vbuf_cap < (uint32_t)count) {
        if (r->line_label_vbuf) wgpuBufferRelease(r->line_label_vbuf);
        uint32_t cap = r->line_label_vbuf_cap > 0 ? r->line_label_vbuf_cap
                                                  : 1024;
        while (cap < (uint32_t)count) cap *= 2;
        r->line_label_vbuf = create_buffer(
            r->device, r->queue, WGPUBufferUsage_Vertex, NULL,
            (size_t)cap * sizeof(arpt_line_glyph_inst));
        r->line_label_vbuf_cap = r->line_label_vbuf ? cap : 0;
    }
    if (!r->line_label_vbuf) return;

    /* Queue writes execute before the frame's command buffer is submitted. */
    wgpuQueueWriteBuffer(r->queue, r->line_label_vbuf, 0, r->line_glyph_out,
                         (size_t)count * sizeof(arpt_line_glyph_inst));

    wgpuRenderPassEncoderSetPipeline(r->pass, r->line_label_pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0,
                                      NULL);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, r->line_label_bind_group,
                                      0, NULL);
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 0, r->line_label_vbuf, 0,
        (uint64_t)count * sizeof(arpt_line_glyph_inst));
    wgpuRenderPassEncoderDraw(r->pass, 4, (uint32_t)count, 0, 0);
}

void arpt__line_label_cleanup(arpt_renderer *r) {
    if (r->line_label_pipeline)
        wgpuRenderPipelineRelease(r->line_label_pipeline);
    if (r->line_label_bind_group)
        wgpuBindGroupRelease(r->line_label_bind_group);
    if (r->line_label_ubuf) wgpuBufferRelease(r->line_label_ubuf);
    if (r->line_label_vbuf) wgpuBufferRelease(r->line_label_vbuf);
    free(r->line_glyph_scratch);
    free(r->line_glyph_out);
}
