#ifndef ARPENTRY_RENDERER_H
#define ARPENTRY_RENDERER_H

#include "coords.h"
#include "font.h"
#include "math3d.h"
#include <webgpu/webgpu.h>

typedef struct arpt_renderer arpt_renderer;
typedef struct arpt_tile_gpu arpt_tile_gpu;
typedef struct arpt_tile_prims arpt_tile_prims;
typedef struct arpt_model arpt_model;

#define ARPT_MAX_PLACEHOLDERS 256

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height,
                                    const float background[4],
                                    const float building_color[4]);
void arpt_renderer_free(arpt_renderer *r);

/** Recreate depth texture after window resize. */
void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height);

/** Upload a tree model's geometry to GPU at the given index (call per model). */
void arpt_renderer_upload_model(arpt_renderer *r, int model_index,
                                const arpt_model *model);

/** Return the number of uploaded models. */
int arpt_renderer_model_count(const arpt_renderer *r);

/** Font glyph metrics for tile_prepare label layout. */
const font_glyph *arpt_renderer_font_glyphs(const arpt_renderer *r);

/** Font pixel height for tile_prepare label layout. */
float arpt_renderer_font_height(const arpt_renderer *r);

/* Tile GPU resources */

/** Upload pre-prepared tile primitives to GPU buffers. */
arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_tile_prims *prims);

/** Update per-tile uniforms (model matrix, bounds, center). */
void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile, arpt_mat4 model,
                                const float bounds[4], float center_lon,
                                float center_lat);

void arpt_tile_gpu_free(arpt_tile_gpu *tile);

/* Placeholder rendering */

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
