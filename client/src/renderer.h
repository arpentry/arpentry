#ifndef ARPENTRY_RENDERER_H
#define ARPENTRY_RENDERER_H

#include "coords.h"
#include "math3d.h"
#include "style.h"
#include "tile_decode.h"
#include <webgpu/webgpu.h>

typedef struct arpt_renderer arpt_renderer;
typedef struct arpt_tile_gpu arpt_tile_gpu;

/* Tree model (decoded from ModelLibrary) */

#define ARPT_MAX_MODELS 8

typedef struct {
    const uint16_t *x, *y, *z; /* model-local mm (zero-copy) */
    const uint16_t *w;          /* per-vertex part index (zero-copy, may be NULL) */
    size_t vertex_count;
    const uint32_t *indices;
    size_t index_count;
    float crown_color[4];       /* from Part[1] or Part[0] */
    float trunk_color[4];       /* from Part[0] */
    float min_scale;
    float max_scale;
    bool random_yaw;            /* apply random yaw rotation per instance */
    bool random_scale;          /* apply random scale variation per instance */
    char name[32];              /* model name for style matching */
} arpt_model;

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height, const arpt_style *style);
void arpt_renderer_free(arpt_renderer *r);

/** Recreate depth texture after window resize. */
void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height);

/** Upload a tree model's geometry to GPU at the given index (call per model). */
void arpt_renderer_upload_model(arpt_renderer *r, int model_index,
                                const arpt_model *model);

/** Return the number of uploaded models. */
int arpt_renderer_model_count(const arpt_renderer *r);

/* Tile GPU resources */

/** Upload a decoded terrain mesh to GPU buffers.
 *  Rasterizes surface, highway, and building features to a texture.
 *  Extrudes buildings with height_m > 0 into 3D wall + roof geometry.
 *  Optionally uploads tree instance data for instanced rendering. */
arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_terrain_mesh *mesh,
                                         const arpt_surface_data *surface,
                                         const arpt_highway_data *highways,
                                         const arpt_surface_data *buildings,
                                         const arpt_tree_data *trees,
                                         arpt_bounds bounds);

/** Update per-tile uniforms (model matrix, bounds, center). */
void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile, arpt_mat4 model,
                                const float bounds[4], float center_lon,
                                float center_lat);

void arpt_tile_gpu_free(arpt_tile_gpu *tile);

/* Placeholder rendering */

#define ARPT_MAX_PLACEHOLDERS 256

/** Draw a flat placeholder quad for a tile that is still loading.
 *  slot indexes into a pre-allocated pool [0, ARPT_MAX_PLACEHOLDERS). */
void arpt_renderer_draw_placeholder(arpt_renderer *r, int slot, arpt_mat4 model,
                                    const float bounds[4], float center_lon,
                                    float center_lat);

/* Frame rendering */

/** Overlay draw callback, invoked during end_frame before the pass closes. */
typedef void (*arpt_overlay_fn)(WGPURenderPassEncoder pass, void *userdata);

/** Register an overlay callback (e.g. for UI drawing). */
void arpt_renderer_set_overlay(arpt_renderer *r, arpt_overlay_fn fn,
                               void *userdata);

/** Set global uniforms for this frame (projection, sun direction). */
void arpt_renderer_set_globals(arpt_renderer *r, arpt_mat4 projection,
                               arpt_vec3 sun_dir);

/** Begin a frame: create encoder, begin render pass. */
void arpt_renderer_begin_frame(arpt_renderer *r, WGPUTextureView target_view);

/** Draw one tile. */
void arpt_renderer_draw_tile(arpt_renderer *r, arpt_tile_gpu *tile);

/** End the frame: finish render pass, submit command buffer. */
void arpt_renderer_end_frame(arpt_renderer *r);

#endif /* ARPENTRY_RENDERER_H */
