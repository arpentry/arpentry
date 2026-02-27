#ifndef ARPENTRY_RENDERER_H
#define ARPENTRY_RENDERER_H

#include "math3d.h"
#include "tile_decode.h"
#include <webgpu/webgpu.h>

typedef struct arpt_renderer arpt_renderer;
typedef struct arpt_tile_gpu arpt_tile_gpu;

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height);
void arpt_renderer_free(arpt_renderer *r);

/** Recreate depth texture after window resize. */
void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height);

/* Tile GPU resources */

/** Upload a decoded terrain mesh to GPU buffers.
 *  If landuse is non-NULL and has polygons, rasterizes them to a texture. */
arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_terrain_mesh *mesh,
                                         const arpt_landuse_data *landuse);

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
