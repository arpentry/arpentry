#ifndef ARPENTRY_PIPELINE_INTERNAL_H
#define ARPENTRY_PIPELINE_INTERNAL_H

#include "renderer.h"
#include "tile/prepare.h"
#include "icon.h"
#include "math3d.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webgpu/webgpu.h>

/* Constants */

/* Surface rasterization target size. Power of two so the mip chain reaches
   1×1 exactly. Keep SURFACE_MIP_COUNT = log2(SURFACE_TEX_SIZE) + 1. This is
   the native (non-overzoomed) size; overzoomed tiles re-rasterize larger,
   up to SURFACE_TEX_MAX, to keep fills crisp past the tileset's max level. */
#define SURFACE_TEX_SIZE  1024
#define SURFACE_MIP_COUNT 11
#define SURFACE_TEX_MAX   4096
#define SURFACE_MAX_MIP_COUNT 13 /* log2(4096) + 1 */

/* Uniform layouts */

typedef struct {
    float projection[16];
    float sun_dir[3];
    float apply_gamma;
    float altitude;
    float _pad[3];
} global_uniforms_t;

typedef struct {
    float inv_projection[16];
    float sun_dir[3];
    float altitude;
    float earth_center[3];
    float earth_radius;
    float earth_color[3];
    float _pad0;
} sky_uniforms_t;

typedef struct {
    float model[16];
    float bounds[4];
    float center_lon;
    float center_lat;
    float _pad0;
    float _pad1;
} tile_uniforms_t;

typedef struct {
    float center[3];
    float _pad;
    float crown_color[3];
    uint32_t random_yaw;
    float min_scale;
    float max_scale;
    uint32_t random_scale;
    float _pad2;
    float trunk_color[3];
    float _pad3;
} model_uniforms_t;

typedef struct {
    float glyph_scale;
    float atlas_size;
    float viewport_width;
    float viewport_height;
    float display_scale;
    float halo_width;   /* halo width in framebuffer pixels */
    float px_range;     /* distance field range in atlas pixels */
    float _poi_pad0;
    float fill_color[4];
    float halo_color[4];
} poi_uniforms_t;

/* Tile GPU state */

struct arpt_tile_gpu {
    WGPUBuffer buf_xy;
    WGPUBuffer buf_z;
    WGPUBuffer buf_normals;
    WGPUBuffer buf_indices;
    WGPUBuffer uniform_buf;
    WGPUBindGroup bind_group;
    WGPUTexture surface_texture;
    WGPUTextureView surface_view;
    uint32_t index_count;
    arpt_renderer *renderer;

    /* Retained fill primitives (polygons + lines), owned by this tile, so the
       surface texture can be re-rasterized at a higher resolution when the
       tile is overzoomed.  Moved out of the prepared tile at upload time.
       surface_size is the current rasterized edge length (0 = no fill). */
    arpt_polygon_prim surf_polys;
    arpt_line_prim surf_lines;
    uint32_t surface_size;

    /* Terrain skirts (edge stitching, same pipeline) */
    WGPUBuffer skirt_buf_xy;
    WGPUBuffer skirt_buf_z;
    WGPUBuffer skirt_buf_normals;
    WGPUBuffer skirt_buf_indices;
    uint32_t skirt_index_count;

    /* Building mesh (separate draw call, same pipeline) */
    WGPUBuffer bldg_buf_xy;
    WGPUBuffer bldg_buf_z;
    WGPUBuffer bldg_buf_normals;
    WGPUBuffer bldg_buf_indices;
    WGPUBindGroup bldg_bind_group;
    uint32_t bldg_index_count;

    /* Tree instances split by model index */
    WGPUBuffer tree_instance_bufs[ARPT_MAX_MODELS];
    uint32_t tree_instance_counts[ARPT_MAX_MODELS];

    /* POI text label instances */
    WGPUBuffer poi_instance_buf;
    uint32_t poi_instance_count;

    /* POI icon instances */
    WGPUBuffer icon_instance_buf;
    uint32_t icon_instance_count;

    /* Per-POI metadata for CPU-side collision detection */
    struct {
        uint16_t qx, qy;
        int32_t qz;
        float label_w_px;
        float label_h_px;
        uint32_t first_instance;
        uint32_t instance_count;
    } *poi_labels;
    int poi_label_count;

    /* Line-following street labels: polylines kept CPU-side, glyphs are
       placed along the screen projection every frame (line_label.c) */
    arpt_line_label *line_labels;
    int line_label_count;

    /* Cached tile uniforms for CPU-side POI projection */
    float cached_model[16];
    float cached_bounds[4];
    float cached_center_lon;
    float cached_center_lat;
};

/* Maximum number of label candidates and placed labels per frame */
#define ARPT_MAX_PENDING_LABELS 2048
#define ARPT_MAX_PLACED_LABELS  2048

/* Pending label candidate for depth-sorted collision resolution */

typedef struct {
    arpt_tile_gpu *tile;
    int label_index;
    float depth;
    float x0, y0, x1, y1;
    uint8_t kind;         /* 0 = point label (POI), 1 = line label */
    uint32_t glyph_first; /* line labels: range into line_glyph_scratch */
    uint32_t glyph_count;
} arpt_pending_label;

/* Per-frame line-label glyph instance, in framebuffer pixels (40 bytes,
   matches the line_label.wgsl vertex buffer layout) */

typedef struct {
    float x, y;         /* glyph quad center */
    float cos_a, sin_a; /* rotation along the projected line */
    float w, h;         /* quad size */
    float u0, v0, u1, v1;
} arpt_line_glyph_inst;

/* Maximum line-label glyph instances per frame */
#define ARPT_MAX_LINE_GLYPHS 8192

/* Renderer state */

struct arpt_renderer {
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat surface_format;
    uint32_t width, height;
    float pixel_ratio;
    float background[4];
    float building_color[4];

    WGPURenderPipeline pipeline;
    WGPUBindGroupLayout global_bgl;
    WGPUBindGroupLayout tile_bgl;

    WGPUBuffer global_uniform_buf;
    WGPUBindGroup global_bind_group;
    global_uniforms_t prev_globals;

    WGPUTexture depth_texture;
    WGPUTextureView depth_view;

    WGPUTexture msaa_texture;
    WGPUTextureView msaa_view;

    /* Sky / atmosphere */
    WGPURenderPipeline sky_pipeline;
    WGPUBindGroupLayout sky_bgl;
    WGPUBuffer sky_uniform_buf;
    WGPUBindGroup sky_bind_group;

    /* Surface offscreen rasterization */
    WGPURenderPipeline surface_pipeline;
    WGPURenderPipeline line_pipeline;
    WGPURenderPipeline stencil_fill_pipeline;  /* stencil INVERT, color OFF */
    WGPURenderPipeline stencil_color_pipeline; /* stencil NotEqual(0), color ON */
    WGPURenderPipeline mipmap_pipeline;        /* downsample prev mip -> next */
    WGPUBindGroupLayout mipmap_bgl;
    WGPUTexture stencil_texture;
    WGPUTextureView stencil_view;
    WGPUSampler surface_sampler;
    WGPUTexture default_surface_tex;
    WGPUTextureView default_surface_view;
    WGPUTexture building_tex;
    WGPUTextureView building_view;


    /* Tree instancing — per-model GPU resources */
    WGPURenderPipeline tree_pipeline;
    WGPUBindGroupLayout model_bgl;
    int model_count;
    struct {
        WGPUBuffer buf_pos;
        WGPUBuffer buf_indices;
        uint32_t index_count;
        WGPUBuffer uniform_buf;
        WGPUBindGroup bind_group;
        float min_scale;
        float max_scale;
    } models[ARPT_MAX_MODELS];

    /* POI text label rendering */
    WGPURenderPipeline poi_pipeline;
    WGPUBindGroupLayout poi_bgl;
    WGPUTexture font_texture;
    WGPUTextureView font_view;
    WGPUSampler font_sampler;
    WGPUBuffer poi_uniform_buf;
    WGPUBindGroup poi_bind_group;
    font_glyph glyphs[FONT_CHAR_COUNT];
    float font_pixel_height;
    float font_px_range;        /* distance field range in atlas pixels */

    /* Icon atlas rendering (reuses poi_pipeline, separate bind group) */
    WGPUTexture icon_texture;
    WGPUTextureView icon_view;
    WGPUSampler icon_sampler;
    WGPUBuffer icon_uniform_buf;
    WGPUBindGroup icon_bind_group;
    icon_glyph icon_glyphs[64]; /* max icons (actual count in icon_glyph_count) */
    int icon_glyph_count;
    float icon_pixel_height;
    float icon_px_range;        /* distance field range in atlas pixels */

    /* Label style parameters (from style.json) */
    float text_size;
    float text_color[4];
    float text_halo_color[4];
    float text_halo_width;
    float icon_size;
    float icon_color[4];
    float icon_halo_color[4];
    float icon_halo_width;
    float text_display_scale;   /* text_size / font_pixel_height */
    float icon_display_scale;   /* icon_size / icon_pixel_height */

    /* Line-following label rendering (street names; shares the font atlas) */
    WGPURenderPipeline line_label_pipeline;
    WGPUBuffer line_label_ubuf;
    WGPUBindGroup line_label_bind_group;
    WGPUBuffer line_label_vbuf;     /* per-frame instances, grown on demand */
    uint32_t line_label_vbuf_cap;   /* capacity in instances */
    arpt_line_glyph_inst *line_glyph_scratch; /* per-frame candidates */
    int line_glyph_scratch_count;
    arpt_line_glyph_inst *line_glyph_out;     /* per-frame placed glyphs */
    int line_glyph_out_count;
    float line_text_size;
    float line_text_color[4];
    float line_text_halo_color[4];
    float line_text_halo_width;

    WGPUCommandEncoder encoder;
    WGPURenderPassEncoder pass;

    /* Camera ECEF position for horizon culling (set each frame) */
    double camera_ecef[3];

    /* POI label collision detection (reset each frame) */
    arpt_mat4 cached_projection;
    struct { float x0, y0, x1, y1; } placed_labels[ARPT_MAX_PLACED_LABELS];
    int placed_label_count;

    /* Deferred label candidates (collected per tile, sorted & drawn at end) */
    arpt_pending_label pending_labels[ARPT_MAX_PENDING_LABELS];
    int pending_label_count;

    /* Overlay callback (e.g. UI) invoked before pass ends */
    arpt_overlay_fn overlay_fn;
    void *overlay_ud;
};

/* Shared helpers */

static inline WGPUShaderModule create_shader(WGPUDevice device,
                                              const char *wgsl_code) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = wgsl_code,
    };
    WGPUShaderModuleDescriptor desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &desc);
    if (!sm) fprintf(stderr, "create_shader: shader module creation failed\n");
    return sm;
}

static inline void restore_terrain_pipeline(arpt_renderer *r) {
    wgpuRenderPassEncoderSetPipeline(r->pass, r->pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0,
                                      NULL);
}

static inline int8_t *pad_normals_2to4(const int8_t *normals, size_t count) {
    int8_t *padded = calloc(count, 4);
    if (!padded) return NULL;
    if (normals) {
        for (size_t i = 0; i < count; i++) {
            padded[i * 4]     = normals[i * 2];
            padded[i * 4 + 1] = normals[i * 2 + 1];
        }
    }
    return padded;
}

static inline WGPUBuffer create_buffer(WGPUDevice device, WGPUQueue queue,
                                        WGPUBufferUsageFlags usage,
                                        const void *data, size_t size) {
    size_t aligned = (size + 3) & ~(size_t)3;
    if (aligned > 200u * 1024u * 1024u) {
        fprintf(stderr, "WARNING: create_buffer: %zu bytes exceeds safety limit\n",
                aligned);
    }
    WGPUBufferDescriptor desc = {
        .usage = usage | WGPUBufferUsage_CopyDst,
        .size = aligned,
    };
    WGPUBuffer buf = wgpuDeviceCreateBuffer(device, &desc);
    if (!buf) return NULL;
    if (data) wgpuQueueWriteBuffer(queue, buf, 0, data, size);
    return buf;
}

/* Internal subsystem functions */

/* render_mesh.c */
WGPURenderPipeline arpt__mesh_create_pipeline(WGPUDevice device,
                                               WGPUTextureFormat format,
                                               WGPUBindGroupLayout global_bgl,
                                               WGPUBindGroupLayout tile_bgl);
void arpt__mesh_upload_terrain(arpt_renderer *r, arpt_tile_gpu *t,
                               const arpt_terrain_mesh *prim);
void arpt__mesh_upload_skirts(arpt_renderer *r, arpt_tile_gpu *t,
                               const arpt_terrain_mesh *prim);
void arpt__mesh_draw_terrain(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__mesh_draw_skirts(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__mesh_draw_buildings(arpt_renderer *r, arpt_tile_gpu *tile);

/* render_texture.c */
WGPURenderPipeline arpt__texture_create_surface_pipeline(WGPUDevice device);
WGPURenderPipeline arpt__texture_create_line_pipeline(WGPUDevice device);
WGPURenderPipeline arpt__texture_create_stencil_fill_pipeline(WGPUDevice device);
WGPURenderPipeline arpt__texture_create_stencil_color_pipeline(WGPUDevice device);
WGPURenderPipeline arpt__texture_create_mipmap_pipeline(WGPUDevice device,
                                                         WGPUBindGroupLayout bgl);
WGPUTexture arpt__texture_rasterize(arpt_renderer *r,
                                     const arpt_polygon_prim *polys,
                                     const arpt_line_prim *lines,
                                     uint32_t tex_size);

/* building.c */
void arpt__building_upload(arpt_renderer *r, arpt_tile_gpu *t,
                           const arpt_building_prim *prim);

/* render_instance.c */
WGPURenderPipeline arpt__instance_create_pipeline(WGPUDevice device,
                                                   WGPUTextureFormat format,
                                                   WGPUBindGroupLayout global_bgl,
                                                   WGPUBindGroupLayout tile_bgl,
                                                   WGPUBindGroupLayout model_bgl);
void arpt__instance_upload_model(arpt_renderer *r, int model_index,
                                 const arpt_model *model);
void arpt__instance_upload(arpt_renderer *r, arpt_tile_gpu *t,
                           const arpt_instance_prim *prim);
void arpt__instance_draw(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__instance_cleanup(arpt_renderer *r);

/* render_label.c */
WGPURenderPipeline arpt__label_create_pipeline(WGPUDevice device,
                                                WGPUTextureFormat format,
                                                WGPUBindGroupLayout global_bgl,
                                                WGPUBindGroupLayout tile_bgl,
                                                WGPUBindGroupLayout poi_bgl);
void arpt__label_init_font(arpt_renderer *r);
void arpt__label_upload(arpt_renderer *r, arpt_tile_gpu *t,
                        const arpt_label_prim *prim);
void arpt__label_collect(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__label_draw_all(arpt_renderer *r);
void arpt__label_cleanup(arpt_renderer *r);

/* line_label.c */
void arpt__line_label_init(arpt_renderer *r);
void arpt__line_label_update_uniforms(arpt_renderer *r);
void arpt__line_label_upload(arpt_renderer *r, arpt_tile_gpu *t,
                             const arpt_line_label_prim *prim);
void arpt__line_label_collect(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__line_label_draw(arpt_renderer *r);
void arpt__line_label_cleanup(arpt_renderer *r);

/* render_sky.c */
WGPURenderPipeline arpt__sky_create_pipeline(WGPUDevice device,
                                              WGPUTextureFormat format,
                                              WGPUBindGroupLayout sky_bgl);
void arpt__sky_draw(arpt_renderer *r);

#endif /* ARPENTRY_PIPELINE_INTERNAL_H */
