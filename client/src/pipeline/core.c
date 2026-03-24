#include "internal.h"

#include <stdlib.h>

/* 4x4 matrix inverse (column-major, general case) */
static void mat4_inverse(const float m[16], float out[16]) {
    float inv[16];
    inv[0]  =  m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15]
             + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4]  = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15]
             - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8]  =  m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15]
             + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14]
             - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1]  = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15]
             - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5]  =  m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15]
             + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9]  = -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15]
             - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] =  m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14]
             + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2]  =  m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15]
             + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6]  = -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15]
             - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] =  m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15]
             + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14]
             - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3]  = -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11]
             - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7]  =  m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11]
             + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11]
             - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] =  m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10]
             + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];

    float det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (det == 0.0f) { memset(out, 0, 64); return; }
    det = 1.0f / det;
    for (int i = 0; i < 16; i++) out[i] = inv[i] * det;
}

/* MSAA color texture (re)creation */

static void create_msaa_texture(arpt_renderer *r) {
    if (r->msaa_view) wgpuTextureViewRelease(r->msaa_view);
    if (r->msaa_texture) wgpuTextureRelease(r->msaa_texture);

    WGPUTextureDescriptor desc = {
        .usage = WGPUTextureUsage_RenderAttachment,
        .size = {r->width, r->height, 1},
        .format = r->surface_format,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 4,
    };
    r->msaa_texture = wgpuDeviceCreateTexture(r->device, &desc);
    r->msaa_view = wgpuTextureCreateView(r->msaa_texture, NULL);
}

/* Depth texture (re)creation */

static void create_depth_texture(arpt_renderer *r) {
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);

    WGPUTextureDescriptor desc = {
        .usage = WGPUTextureUsage_RenderAttachment,
        .size = {r->width, r->height, 1},
        .format = WGPUTextureFormat_Depth24Plus,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 4,
    };
    r->depth_texture = wgpuDeviceCreateTexture(r->device, &desc);
    r->depth_view = wgpuTextureCreateView(r->depth_texture, NULL);
}

/* 1x1 solid-color texture helper */

static void create_1x1_texture(WGPUDevice device, WGPUQueue queue,
                                const float color[4], WGPUTexture *tex,
                                WGPUTextureView *view) {
    WGPUTextureDescriptor dt = {
        .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
        .size = {1, 1, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    *tex = wgpuDeviceCreateTexture(device, &dt);
    *view = wgpuTextureCreateView(*tex, NULL);
    uint8_t pixel[4] = {(uint8_t)(color[0] * 255.0f + 0.5f),
                        (uint8_t)(color[1] * 255.0f + 0.5f),
                        (uint8_t)(color[2] * 255.0f + 0.5f),
                        (uint8_t)(color[3] * 255.0f + 0.5f)};
    WGPUImageCopyTexture dst = {.texture = *tex};
    WGPUTextureDataLayout layout = {.bytesPerRow = 4, .rowsPerImage = 1};
    WGPUExtent3D extent = {1, 1, 1};
    wgpuQueueWriteTexture(queue, &dst, pixel, 4, &layout, &extent);
}

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height,
                                    const float background[4],
                                    const float building_color[4]) {
    arpt_renderer *r = calloc(1, sizeof(*r));
    if (!r) return NULL;

    r->device = device;
    r->queue = queue;
    r->surface_format = format;
    r->width = width;
    r->height = height;
    r->pixel_ratio = 1.0f;
    memcpy(r->background, background, sizeof(r->background));
    memcpy(r->building_color, building_color, sizeof(r->building_color));

    /* Bind group layouts */
    WGPUBindGroupLayoutEntry global_entry = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(global_uniforms_t)},
    };
    r->global_bgl = wgpuDeviceCreateBindGroupLayout(
        device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                                 .entries = &global_entry});

    WGPUBindGroupLayoutEntry tile_entries[] = {
        {
            .binding = 0,
            .visibility = WGPUShaderStage_Vertex,
            .buffer = {.type = WGPUBufferBindingType_Uniform,
                       .minBindingSize = sizeof(tile_uniforms_t)},
        },
        {
            .binding = 1,
            .visibility = WGPUShaderStage_Fragment,
            .texture = {.sampleType = WGPUTextureSampleType_Float,
                        .viewDimension = WGPUTextureViewDimension_2D},
        },
        {
            .binding = 2,
            .visibility = WGPUShaderStage_Fragment,
            .sampler = {.type = WGPUSamplerBindingType_Filtering},
        },
    };
    r->tile_bgl = wgpuDeviceCreateBindGroupLayout(
        device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 3,
                                                 .entries = tile_entries});

    /* Terrain + building pipeline */
    r->pipeline =
        arpt__mesh_create_pipeline(device, format, r->global_bgl, r->tile_bgl);

    /* Wireframe placeholder pipeline */
    r->wireframe_pipeline = arpt__placeholder_create_wireframe_pipeline(
        device, format, r->global_bgl, r->tile_bgl);

    /* Model bind group layout */
    WGPUBindGroupLayoutEntry model_entry = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(model_uniforms_t)},
    };
    r->model_bgl = wgpuDeviceCreateBindGroupLayout(
        device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                                 .entries = &model_entry});

    r->tree_pipeline = arpt__instance_create_pipeline(
        device, format, r->global_bgl, r->tile_bgl, r->model_bgl);

    /* Set default label style before font init */
    r->text_size = 14.0f;
    r->text_color[0] = 0.2f; r->text_color[1] = 0.2f;
    r->text_color[2] = 0.2f; r->text_color[3] = 1.0f;
    r->text_halo_color[0] = 1.0f; r->text_halo_color[1] = 1.0f;
    r->text_halo_color[2] = 1.0f; r->text_halo_color[3] = 1.0f;
    r->text_halo_width = 2.0f;
    r->icon_size = 20.0f;
    r->icon_color[0] = 0.2f; r->icon_color[1] = 0.2f;
    r->icon_color[2] = 0.2f; r->icon_color[3] = 1.0f;
    r->icon_halo_color[0] = 1.0f; r->icon_halo_color[1] = 1.0f;
    r->icon_halo_color[2] = 1.0f; r->icon_halo_color[3] = 1.0f;
    r->icon_halo_width = 0.5f;

    /* POI text label pipeline + font atlas */
    {
        WGPUBindGroupLayoutEntry poi_entries[] = {
            {
                .binding = 0,
                .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
                .buffer = {.type = WGPUBufferBindingType_Uniform,
                           .minBindingSize = sizeof(poi_uniforms_t)},
            },
            {
                .binding = 1,
                .visibility = WGPUShaderStage_Fragment,
                .texture = {.sampleType = WGPUTextureSampleType_Float,
                            .viewDimension = WGPUTextureViewDimension_2D},
            },
            {
                .binding = 2,
                .visibility = WGPUShaderStage_Fragment,
                .sampler = {.type = WGPUSamplerBindingType_Filtering},
            },
        };
        r->poi_bgl = wgpuDeviceCreateBindGroupLayout(
            device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 3,
                                                     .entries = poi_entries});

        r->poi_pipeline = arpt__label_create_pipeline(
            device, format, r->global_bgl, r->tile_bgl, r->poi_bgl);

        arpt__label_init_font(r);
    }

    /* Sky / atmosphere pipeline */
    {
        WGPUBindGroupLayoutEntry sky_entry = {
            .binding = 0,
            .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
            .buffer = {.type = WGPUBufferBindingType_Uniform,
                       .minBindingSize = sizeof(sky_uniforms_t)},
        };
        r->sky_bgl = wgpuDeviceCreateBindGroupLayout(
            device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                                     .entries = &sky_entry});
        r->sky_pipeline =
            arpt__sky_create_pipeline(device, format, r->sky_bgl);

        r->sky_uniform_buf =
            create_buffer(device, queue, WGPUBufferUsage_Uniform, NULL,
                          sizeof(sky_uniforms_t));
        WGPUBindGroupEntry sky_bg_e = {
            .binding = 0,
            .buffer = r->sky_uniform_buf,
            .offset = 0,
            .size = sizeof(sky_uniforms_t),
        };
        r->sky_bind_group = wgpuDeviceCreateBindGroup(
            device, &(WGPUBindGroupDescriptor){.layout = r->sky_bgl,
                                               .entryCount = 1,
                                               .entries = &sky_bg_e});
    }

    /* Surface offscreen pipelines + sampler */
    r->surface_pipeline = arpt__texture_create_surface_pipeline(device);
    r->highway_pipeline = arpt__texture_create_highway_pipeline(device);
    WGPUSamplerDescriptor samp_desc = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
        .lodMaxClamp = 32.0f,
    };
    r->surface_sampler = wgpuDeviceCreateSampler(device, &samp_desc);

    /* Default 1x1 surface texture from background color */
    create_1x1_texture(device, queue, background, &r->default_surface_tex,
                       &r->default_surface_view);

    /* 1x1 building material texture */
    create_1x1_texture(device, queue, building_color, &r->building_tex,
                       &r->building_view);

    /* Global uniform buffer + bind group */
    r->global_uniform_buf =
        create_buffer(device, queue, WGPUBufferUsage_Uniform, NULL,
                      sizeof(global_uniforms_t));
    WGPUBindGroupEntry bg_e = {
        .binding = 0,
        .buffer = r->global_uniform_buf,
        .offset = 0,
        .size = sizeof(global_uniforms_t),
    };
    r->global_bind_group = wgpuDeviceCreateBindGroup(
        device, &(WGPUBindGroupDescriptor){.layout = r->global_bgl,
                                           .entryCount = 1,
                                           .entries = &bg_e});

    create_msaa_texture(r);
    create_depth_texture(r);
    arpt__placeholder_init(r);

    return r;
}

void arpt_renderer_free(arpt_renderer *r) {
    if (!r) return;
    arpt__placeholder_cleanup(r);
    arpt__label_cleanup(r);
    arpt__instance_cleanup(r);
    if (r->msaa_view) wgpuTextureViewRelease(r->msaa_view);
    if (r->msaa_texture) wgpuTextureRelease(r->msaa_texture);
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);
    if (r->global_bind_group) wgpuBindGroupRelease(r->global_bind_group);
    if (r->global_uniform_buf) wgpuBufferRelease(r->global_uniform_buf);
    if (r->sky_bind_group) wgpuBindGroupRelease(r->sky_bind_group);
    if (r->sky_uniform_buf) wgpuBufferRelease(r->sky_uniform_buf);
    if (r->sky_pipeline) wgpuRenderPipelineRelease(r->sky_pipeline);
    if (r->sky_bgl) wgpuBindGroupLayoutRelease(r->sky_bgl);
    if (r->pipeline) wgpuRenderPipelineRelease(r->pipeline);
    if (r->surface_pipeline) wgpuRenderPipelineRelease(r->surface_pipeline);
    if (r->highway_pipeline) wgpuRenderPipelineRelease(r->highway_pipeline);
    if (r->surface_sampler) wgpuSamplerRelease(r->surface_sampler);
    if (r->default_surface_view)
        wgpuTextureViewRelease(r->default_surface_view);
    if (r->default_surface_tex) wgpuTextureRelease(r->default_surface_tex);
    if (r->building_view) wgpuTextureViewRelease(r->building_view);
    if (r->building_tex) wgpuTextureRelease(r->building_tex);
    if (r->global_bgl) wgpuBindGroupLayoutRelease(r->global_bgl);
    if (r->tile_bgl) wgpuBindGroupLayoutRelease(r->tile_bgl);
    free(r);
}

void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height,
                           float pixel_ratio) {
    r->width = width;
    r->height = height;
    r->pixel_ratio = pixel_ratio;
    create_msaa_texture(r);
    create_depth_texture(r);

    /* Re-upload POI uniforms with new viewport dimensions.
     * display_scale is multiplied by pixel_ratio so that text_size (logical
     * pixels) maps correctly to the framebuffer-pixel viewport. */
    if (r->poi_uniform_buf) {
        poi_uniforms_t pu = {
            .glyph_scale = r->font_pixel_height,
            .atlas_size = (float)FONT_ATLAS_SIZE,
            .viewport_width = (float)width,
            .viewport_height = (float)height,
            .display_scale = r->text_display_scale * pixel_ratio,
            .halo_width = r->text_halo_width,
        };
        memcpy(pu.fill_color, r->text_color, sizeof(pu.fill_color));
        memcpy(pu.halo_color, r->text_halo_color, sizeof(pu.halo_color));
        wgpuQueueWriteBuffer(r->queue, r->poi_uniform_buf, 0, &pu,
                             sizeof(poi_uniforms_t));
    }
    if (r->icon_uniform_buf) {
        poi_uniforms_t pu = {
            .glyph_scale = r->icon_pixel_height,
            .atlas_size = (float)ICON_ATLAS_SIZE,
            .viewport_width = (float)width,
            .viewport_height = (float)height,
            .display_scale = r->icon_display_scale * pixel_ratio,
            .halo_width = r->icon_halo_width,
        };
        memcpy(pu.fill_color, r->icon_color, sizeof(pu.fill_color));
        memcpy(pu.halo_color, r->icon_halo_color, sizeof(pu.halo_color));
        wgpuQueueWriteBuffer(r->queue, r->icon_uniform_buf, 0, &pu,
                             sizeof(poi_uniforms_t));
    }
}

/* Label style setter */

void arpt_renderer_set_label_style(arpt_renderer *r,
                                    float text_size, const float text_color[4],
                                    const float text_halo_color[4],
                                    float text_halo_width,
                                    float icon_size, const float icon_color[4],
                                    const float icon_halo_color[4],
                                    float icon_halo_width) {
    if (!r) return;
    r->text_size = text_size;
    memcpy(r->text_color, text_color, sizeof(r->text_color));
    memcpy(r->text_halo_color, text_halo_color, sizeof(r->text_halo_color));
    r->text_halo_width = text_halo_width;
    r->icon_size = icon_size;
    memcpy(r->icon_color, icon_color, sizeof(r->icon_color));
    memcpy(r->icon_halo_color, icon_halo_color, sizeof(r->icon_halo_color));
    r->icon_halo_width = icon_halo_width;

    r->text_display_scale = (r->font_pixel_height > 0)
        ? text_size / r->font_pixel_height : 1.0f;
    r->icon_display_scale = (r->icon_pixel_height > 0)
        ? icon_size / r->icon_pixel_height : 1.0f;

    /* Update GPU uniform buffers */
    if (r->poi_uniform_buf) {
        poi_uniforms_t pu = {
            .glyph_scale = r->font_pixel_height,
            .atlas_size = (float)FONT_ATLAS_SIZE,
            .viewport_width = (float)r->width,
            .viewport_height = (float)r->height,
            .display_scale = r->text_display_scale * r->pixel_ratio,
            .halo_width = r->text_halo_width,
        };
        memcpy(pu.fill_color, r->text_color, sizeof(pu.fill_color));
        memcpy(pu.halo_color, r->text_halo_color, sizeof(pu.halo_color));
        wgpuQueueWriteBuffer(r->queue, r->poi_uniform_buf, 0, &pu,
                             sizeof(poi_uniforms_t));
    }
    if (r->icon_uniform_buf) {
        poi_uniforms_t pu = {
            .glyph_scale = r->icon_pixel_height,
            .atlas_size = (float)ICON_ATLAS_SIZE,
            .viewport_width = (float)r->width,
            .viewport_height = (float)r->height,
            .display_scale = r->icon_display_scale * r->pixel_ratio,
            .halo_width = r->icon_halo_width,
        };
        memcpy(pu.fill_color, r->icon_color, sizeof(pu.fill_color));
        memcpy(pu.halo_color, r->icon_halo_color, sizeof(pu.halo_color));
        wgpuQueueWriteBuffer(r->queue, r->icon_uniform_buf, 0, &pu,
                             sizeof(poi_uniforms_t));
    }
}

/* Model upload (delegates to render_instance.c) */

void arpt_renderer_upload_model(arpt_renderer *r, int model_index,
                                const arpt_model *model) {
    arpt__instance_upload_model(r, model_index, model);
}

int arpt_renderer_model_count(const arpt_renderer *r) {
    return r ? r->model_count : 0;
}

const font_glyph *arpt_renderer_font_glyphs(const arpt_renderer *r) {
    return r ? r->glyphs : NULL;
}

float arpt_renderer_font_height(const arpt_renderer *r) {
    return r ? r->font_pixel_height : 0.0f;
}

const icon_glyph *arpt_renderer_icon_glyphs(const arpt_renderer *r) {
    return r ? r->icon_glyphs : NULL;
}

int arpt_renderer_icon_count(const arpt_renderer *r) {
    return r ? r->icon_glyph_count : 0;
}

float arpt_renderer_icon_height(const arpt_renderer *r) {
    return r ? r->icon_pixel_height : 0.0f;
}

/* Tile upload */

arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_tile_prims *prims) {
    arpt_tile_gpu *t = calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->renderer = r;

    /* Upload terrain mesh + edge skirts */
    arpt__mesh_upload_terrain(r, t, &prims->terrain);
    arpt__mesh_upload_skirts(r, t, &prims->terrain);

    /* Upload building extrusion */
    arpt__extrusion_upload(r, t, &prims->extrusion);

    /* Upload tree instances */
    arpt__instance_upload(r, t, &prims->instances);

    /* Upload POI label glyphs */
    arpt__label_upload(r, t, &prims->labels);

    /* Rasterize surface texture */
    bool has_polys = prims->texture.poly_vert_count > 0 &&
                     prims->texture.poly_index_count > 0;
    bool has_lines = prims->texture.line_vert_count > 0 &&
                     prims->texture.line_index_count > 0;
    if (has_polys || has_lines) {
        t->surface_texture = arpt__texture_rasterize(r, &prims->texture);
        t->surface_view = wgpuTextureCreateView(t->surface_texture, NULL);
    }

    WGPUTextureView lu_view =
        t->surface_view ? t->surface_view : r->default_surface_view;

    /* Per-tile uniform buffer + bind group */
    t->uniform_buf = create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform,
                                   NULL, sizeof(tile_uniforms_t));
    WGPUBindGroupEntry entries[] = {
        {.binding = 0,
         .buffer = t->uniform_buf,
         .offset = 0,
         .size = sizeof(tile_uniforms_t)},
        {.binding = 1, .textureView = lu_view},
        {.binding = 2, .sampler = r->surface_sampler},
    };
    t->bind_group = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                              .entryCount = 3,
                                              .entries = entries});

    /* Building bind group: same uniforms, building color texture */
    if (t->bldg_index_count > 0) {
        WGPUBindGroupEntry bldg_entries[] = {
            {.binding = 0,
             .buffer = t->uniform_buf,
             .offset = 0,
             .size = sizeof(tile_uniforms_t)},
            {.binding = 1, .textureView = r->building_view},
            {.binding = 2, .sampler = r->surface_sampler},
        };
        t->bldg_bind_group = wgpuDeviceCreateBindGroup(
            r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                                  .entryCount = 3,
                                                  .entries = bldg_entries});
    }

    return t;
}

void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile, arpt_mat4 model,
                                const float bounds[4], float center_lon,
                                float center_lat) {
    tile_uniforms_t u = {0};
    memcpy(u.model, model.m, sizeof(u.model));
    memcpy(u.bounds, bounds, sizeof(u.bounds));
    u.center_lon = center_lon;
    u.center_lat = center_lat;
    /* Cache for CPU-side POI projection */
    memcpy(tile->cached_model, model.m, sizeof(tile->cached_model));
    memcpy(tile->cached_bounds, bounds, sizeof(tile->cached_bounds));
    tile->cached_center_lon = center_lon;
    tile->cached_center_lat = center_lat;

    wgpuQueueWriteBuffer(tile->renderer->queue, tile->uniform_buf, 0, &u,
                         sizeof(u));
}

void arpt_tile_gpu_free(arpt_tile_gpu *tile) {
    if (!tile) return;
    if (tile->buf_xy) wgpuBufferRelease(tile->buf_xy);
    if (tile->buf_z) wgpuBufferRelease(tile->buf_z);
    if (tile->buf_normals) wgpuBufferRelease(tile->buf_normals);
    if (tile->buf_indices) wgpuBufferRelease(tile->buf_indices);
    if (tile->skirt_buf_xy) wgpuBufferRelease(tile->skirt_buf_xy);
    if (tile->skirt_buf_z) wgpuBufferRelease(tile->skirt_buf_z);
    if (tile->skirt_buf_normals) wgpuBufferRelease(tile->skirt_buf_normals);
    if (tile->skirt_buf_indices) wgpuBufferRelease(tile->skirt_buf_indices);
    if (tile->bldg_buf_xy) wgpuBufferRelease(tile->bldg_buf_xy);
    if (tile->bldg_buf_z) wgpuBufferRelease(tile->bldg_buf_z);
    if (tile->bldg_buf_normals) wgpuBufferRelease(tile->bldg_buf_normals);
    if (tile->bldg_buf_indices) wgpuBufferRelease(tile->bldg_buf_indices);
    for (int mi = 0; mi < ARPT_MAX_MODELS; mi++) {
        if (tile->tree_instance_bufs[mi])
            wgpuBufferRelease(tile->tree_instance_bufs[mi]);
    }
    if (tile->poi_instance_buf) wgpuBufferRelease(tile->poi_instance_buf);
    if (tile->icon_instance_buf) wgpuBufferRelease(tile->icon_instance_buf);
    free(tile->poi_labels);
    if (tile->uniform_buf) wgpuBufferRelease(tile->uniform_buf);
    if (tile->bind_group) wgpuBindGroupRelease(tile->bind_group);
    if (tile->bldg_bind_group) wgpuBindGroupRelease(tile->bldg_bind_group);
    if (tile->surface_view) wgpuTextureViewRelease(tile->surface_view);
    if (tile->surface_texture) wgpuTextureRelease(tile->surface_texture);
    free(tile);
}

/* Placeholder rendering */

void arpt_renderer_draw_placeholder(arpt_renderer *r, int slot, arpt_mat4 model,
                                    const float bounds[4], float center_lon,
                                    float center_lat) {
    arpt__placeholder_draw(r, slot, model, bounds, center_lon, center_lat);
}

/* Frame rendering */

void arpt_renderer_set_globals(arpt_renderer *r, arpt_mat4 projection,
                               arpt_vec3 sun_dir) {
    global_uniforms_t u = {0};
    memcpy(u.projection, projection.m, sizeof(u.projection));
    u.sun_dir[0] = sun_dir.x;
    u.sun_dir[1] = sun_dir.y;
    u.sun_dir[2] = sun_dir.z;
    /* Non-sRGB formats need manual gamma correction in the shader */
    u.apply_gamma = (r->surface_format == WGPUTextureFormat_BGRA8UnormSrgb ||
                     r->surface_format == WGPUTextureFormat_RGBA8UnormSrgb)
                        ? 0.0f
                        : 1.0f;

    if (memcmp(&u, &r->prev_globals, sizeof(u)) == 0) return;

    r->prev_globals = u;
    memcpy(r->cached_projection.m, projection.m, sizeof(r->cached_projection.m));
    wgpuQueueWriteBuffer(r->queue, r->global_uniform_buf, 0, &u, sizeof(u));
}

void arpt_renderer_set_sky(arpt_renderer *r, arpt_mat4 projection,
                            arpt_vec3 sun_dir, float altitude) {
    sky_uniforms_t u = {0};
    mat4_inverse(projection.m, u.inv_projection);
    u.sun_dir[0] = sun_dir.x;
    u.sun_dir[1] = sun_dir.y;
    u.sun_dir[2] = sun_dir.z;
    u.altitude = altitude;
    wgpuQueueWriteBuffer(r->queue, r->sky_uniform_buf, 0, &u, sizeof(u));
}

void arpt_renderer_set_camera_ecef(arpt_renderer *r, double x, double y,
                                    double z) {
    if (!r) return;
    r->camera_ecef[0] = x;
    r->camera_ecef[1] = y;
    r->camera_ecef[2] = z;
}

void arpt_renderer_begin_frame(arpt_renderer *r, WGPUTextureView target_view) {
    r->encoder = wgpuDeviceCreateCommandEncoder(r->device, NULL);

    WGPURenderPassColorAttachment color = {
        .view = r->msaa_view,
        .resolveTarget = target_view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Discard,
        .clearValue = {0.0, 0.0, 0.0, 1.0},
#ifdef __EMSCRIPTEN__
        .depthSlice = WGPU_DEPTH_SLICE_UNDEFINED,
#endif
    };
    WGPURenderPassDepthStencilAttachment depth = {
        .view = r->depth_view,
        .depthLoadOp = WGPULoadOp_Clear,
        .depthStoreOp = WGPUStoreOp_Store,
        .depthClearValue = 1.0f,
    };
    WGPURenderPassDescriptor rp = {
        .colorAttachmentCount = 1,
        .colorAttachments = &color,
        .depthStencilAttachment = &depth,
    };

    r->pass = wgpuCommandEncoderBeginRenderPass(r->encoder, &rp);
    r->placed_label_count = 0;
    r->pending_label_count = 0;

    /* Draw sky first (writes depth=1.0 everywhere, terrain overwrites) */
    arpt__sky_draw(r);

    /* Set terrain pipeline for subsequent tile draws */
}

void arpt_renderer_draw_tile(arpt_renderer *r, arpt_tile_gpu *tile) {
    arpt__mesh_draw_terrain(r, tile);
    arpt__mesh_draw_skirts(r, tile);
    arpt__mesh_draw_extrusion(r, tile);
    arpt__instance_draw(r, tile);
    arpt__label_collect(r, tile);
}

void arpt_renderer_set_overlay(arpt_renderer *r, arpt_overlay_fn fn,
                               void *userdata) {
    if (!r) return;
    r->overlay_fn = fn;
    r->overlay_ud = userdata;
}

void arpt_renderer_end_frame(arpt_renderer *r) {
    /* Draw all labels sorted by depth (closest first) */
    arpt__label_draw_all(r);

    /* Invoke overlay (e.g. UI) before closing the pass */
    if (r->overlay_fn) r->overlay_fn(r->pass, r->overlay_ud);
    wgpuRenderPassEncoderEnd(r->pass);
    wgpuRenderPassEncoderRelease(r->pass);
    r->pass = NULL;

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(r->encoder, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(r->encoder);
    r->encoder = NULL;
}
