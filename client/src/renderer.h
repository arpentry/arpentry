#ifndef ARPENTRY_RENDERER_H
#define ARPENTRY_RENDERER_H

#include "coords.h"
#include "font.h"
#include "icon.h"
#include "math3d.h"
#include <stdbool.h>
#include <webgpu/webgpu.h>

typedef struct arpt_renderer arpt_renderer;
typedef struct arpt_tile_gpu arpt_tile_gpu;
typedef struct arpt_tile_prims arpt_tile_prims;
typedef struct arpt_model arpt_model;

#define ARPT_MSAA_SAMPLES     4

/* Reversed-Z: near maps to depth 1, infinity to 0, on a float depth buffer
   (see arpt_mat4_perspective). The three settle one convention together and
   every 3D pipeline reads them from here, so no pipeline can disagree
   silently: a "nearer" fragment has the GREATER depth, the clear is the far
   value 0, and a bias toward the camera is a positive depthBias. Floats
   under reversed-Z hold near-constant relative precision along the whole
   ray (a ~0.2 mm quantum at 2 km eye distance, against ~0.2 m for a
   forward 24-bit buffer with a 1:10^7 near:far ratio), which is what lets
   a 12 cm kerb separate a walkway from its road at range without a
   metre-scale bias. */
#define ARPT_DEPTH_FORMAT     WGPUTextureFormat_Depth32Float
#define ARPT_DEPTH_COMPARE    WGPUCompareFunction_GreaterEqual
#define ARPT_DEPTH_CLEAR      0.0f

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height,
                                    const float background[4],
                                    const float building_color[4]);
void arpt_renderer_free(arpt_renderer *r);

/** Recreate depth texture after window resize.
 *  pixel_ratio = framebuffer pixels / window pixels (e.g. 2.0 on Retina). */
void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height,
                           float pixel_ratio);

/** Upload a tree model's geometry to GPU at the given index (call per model). */
void arpt_renderer_upload_model(arpt_renderer *r, int model_index,
                                const arpt_model *model);

/** Return the number of uploaded models. */
int arpt_renderer_model_count(const arpt_renderer *r);

/** Set label style parameters (text/icon size, colors, halo). */
void arpt_renderer_set_label_style(arpt_renderer *r,
                                    float text_size, const float text_color[4],
                                    const float text_halo_color[4],
                                    float text_halo_width,
                                    float icon_size, const float icon_color[4],
                                    const float icon_halo_color[4],
                                    float icon_halo_width);

/** Set line-following label style (street names along roads). */
void arpt_renderer_set_line_label_style(arpt_renderer *r, float text_size,
                                        const float text_color[4],
                                        const float halo_color[4],
                                        float halo_width);

/** Font glyph metrics for tile_prepare label layout. */
const font_glyph *arpt_renderer_font_glyphs(const arpt_renderer *r);

/** Font pixel height for tile_prepare label layout. */
float arpt_renderer_font_height(const arpt_renderer *r);

/** Icon glyph metrics for tile_prepare icon layout. */
const icon_glyph *arpt_renderer_icon_glyphs(const arpt_renderer *r);

/** Number of icons in the icon atlas. */
int arpt_renderer_icon_count(const arpt_renderer *r);

/** Icon pixel height for tile_prepare icon layout. */
float arpt_renderer_icon_height(const arpt_renderer *r);

/* Tile GPU resources */

/** Upload pre-prepared tile primitives to GPU buffers. Takes ownership of the
 *  fill (polygon + line) primitives, clearing them in `prims`; the rest stay
 *  owned by the caller. */
arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         arpt_tile_prims *prims);

/** Adapt a tile's surface (fill) texture resolution to the given overzoom
 *  amount (0 = native, 1 = one level past max, ...). Re-rasterizes from the
 *  retained fill primitives only when the resulting resolution changes;
 *  no-op (returns false) for tiles without a fill texture. Returns true if
 *  the texture was re-rasterized. */
bool arpt_renderer_tile_set_overzoom(arpt_renderer *r, arpt_tile_gpu *t,
                                     int overzoom);

/** Update per-tile uniforms (model matrix, bounds, center). The center is
 *  taken in double radians: the renderer derives the small relative bounds
 *  and the center's sin/cos in full precision, so the vertex shader can
 *  position vertices from well-conditioned deltas instead of absolute f32
 *  ECEF (whose ~0.5 m rounding scallops every straight edge). */
void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile, arpt_mat4 model,
                                const double bounds_rad[4], double center_lon,
                                double center_lat, float stroke_margin_m);

/* The road strokes' depth-only camera bias per rung (metres). On the coarse
   rungs the grade-limited roadbed cuts below a terrain mesh too coarse to
   follow it, and 12 m is what surfaces those cuttings while a real hill still
   occludes a road behind it. On the detail rung (the tileset's max level) the
   tiler cuts the ground away under the pavement (docs/GROUND.md), so there is
   nothing to surface and the paint only has to beat the deck it lies on: the
   deck margin plus the whole sheet ladder (terrain.wgsl), with room to spare. */
#define ARPT_STROKE_MARGIN_COARSE_M 12.0f
#define ARPT_STROKE_MARGIN_DETAIL_M 0.5f

void arpt_tile_gpu_free(arpt_tile_gpu *tile);

/* Frame rendering */

/** Overlay draw callback, invoked during end_frame before the pass closes. */
typedef void (*arpt_overlay_fn)(WGPURenderPassEncoder pass, void *userdata);

/** Register an overlay callback (e.g. for UI drawing). */
void arpt_renderer_set_overlay(arpt_renderer *r, arpt_overlay_fn fn,
                               void *userdata);

/** Set global uniforms for this frame (projection, sun direction, altitude). */
void arpt_renderer_set_globals(arpt_renderer *r, arpt_mat4 projection,
                               arpt_vec3 sun_dir, float altitude);

/** Set camera ECEF position for horizon culling of labels. */
void arpt_renderer_set_camera_ecef(arpt_renderer *r, double x, double y,
                                    double z);

/** Set sky uniforms for this frame (atmosphere rendering from space). */
void arpt_renderer_set_sky(arpt_renderer *r, arpt_mat4 projection,
                            arpt_vec3 sun_dir, float altitude,
                            arpt_vec3 earth_center_view);

/** Begin a frame: create encoder, begin render pass. */
void arpt_renderer_begin_frame(arpt_renderer *r, WGPUTextureView target_view);

/** Draw one tile. */
void arpt_renderer_draw_tile(arpt_renderer *r, arpt_tile_gpu *tile);

/** End the frame: finish render pass, submit command buffer. */
void arpt_renderer_end_frame(arpt_renderer *r);

#endif /* ARPENTRY_RENDERER_H */
