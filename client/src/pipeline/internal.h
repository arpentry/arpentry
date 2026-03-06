#ifndef ARPENTRY_PIPELINE_INTERNAL_H
#define ARPENTRY_PIPELINE_INTERNAL_H

#include "renderer.h"
#include "tile/prepare.h"
#include "math3d.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <webgpu/webgpu.h>

/* Constants */

#define SURFACE_TEX_SIZE 2048

/* Uniform layouts */

typedef struct {
    float projection[16];
    float sun_dir[3];
    float apply_gamma;
} global_uniforms_t;

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

    /* Building extrusion (separate draw call, same pipeline) */
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

    /* Cached tile uniforms for CPU-side POI projection */
    float cached_model[16];
    float cached_bounds[4];
    float cached_center_lon;
    float cached_center_lat;
};

/* Renderer state */

struct arpt_renderer {
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat surface_format;
    uint32_t width, height;
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

    /* Surface offscreen rasterization */
    WGPURenderPipeline surface_pipeline;
    WGPURenderPipeline highway_pipeline;
    WGPUSampler surface_sampler;
    WGPUTexture default_surface_tex;
    WGPUTextureView default_surface_view;
    WGPUTexture building_tex;
    WGPUTextureView building_view;

    /* Placeholder flat quads for tiles still loading */
    WGPUBuffer ph_buf_xy;
    WGPUBuffer ph_buf_z;
    WGPUBuffer ph_buf_normals;
    WGPUBuffer ph_buf_indices;
    uint32_t ph_index_count;
    WGPUTexture ph_texture;
    WGPUTextureView ph_texture_view;
    WGPUBuffer ph_uniform_bufs[ARPT_MAX_PLACEHOLDERS];
    WGPUBindGroup ph_bind_groups[ARPT_MAX_PLACEHOLDERS];

    /* Wireframe SDF overlay for placeholders */
    WGPURenderPipeline wireframe_pipeline;
    WGPUBuffer ph_wire_buf_xy;
    WGPUBuffer ph_wire_buf_z;
    WGPUBuffer ph_wire_buf_dist;
    WGPUBuffer ph_wire_indices;
    uint32_t ph_wire_index_count;

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

    WGPUCommandEncoder encoder;
    WGPURenderPassEncoder pass;

    /* POI label collision detection (reset each frame) */
    arpt_mat4 cached_projection;
    struct { float x0, y0, x1, y1; } placed_labels[512];
    int placed_label_count;

    /* Overlay callback (e.g. UI) invoked before pass ends */
    arpt_overlay_fn overlay_fn;
    void *overlay_ud;
};

/* Shared helper */

static inline WGPUBuffer create_buffer(WGPUDevice device, WGPUQueue queue,
                                        WGPUBufferUsageFlags usage,
                                        const void *data, size_t size) {
    size_t aligned = (size + 3) & ~(size_t)3;
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
                               const arpt_mesh_prim *prim);
void arpt__mesh_draw_terrain(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__mesh_draw_extrusion(arpt_renderer *r, arpt_tile_gpu *tile);

/* render_texture.c */
WGPURenderPipeline arpt__texture_create_surface_pipeline(WGPUDevice device);
WGPURenderPipeline arpt__texture_create_highway_pipeline(WGPUDevice device);
WGPUTexture arpt__texture_rasterize(arpt_renderer *r,
                                     const arpt_texture_prim *prim);

/* render_extrusion.c */
void arpt__extrusion_upload(arpt_renderer *r, arpt_tile_gpu *t,
                            const arpt_extrusion_prim *prim);

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
void arpt__label_draw(arpt_renderer *r, arpt_tile_gpu *tile);
void arpt__label_cleanup(arpt_renderer *r);

/* render_placeholder.c */
WGPURenderPipeline arpt__placeholder_create_wireframe_pipeline(
    WGPUDevice device, WGPUTextureFormat format,
    WGPUBindGroupLayout global_bgl, WGPUBindGroupLayout tile_bgl);
void arpt__placeholder_init(arpt_renderer *r);
void arpt__placeholder_draw(arpt_renderer *r, int slot, arpt_mat4 model,
                            const float bounds[4], float center_lon,
                            float center_lat);
void arpt__placeholder_cleanup(arpt_renderer *r);

#endif /* ARPENTRY_PIPELINE_INTERNAL_H */
