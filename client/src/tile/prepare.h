#ifndef ARPENTRY_TILE_PREPARE_H
#define ARPENTRY_TILE_PREPARE_H

#include "coords.h"
#include "font.h"
#include "style.h"
#include "decode.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Tree model (decoded from ModelLibrary) */

#define ARPT_MAX_MODELS 8

typedef struct arpt_model {
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

/* Mesh primitive (terrain — zero-copy pointers into FlatBuffer) */

typedef struct {
    const uint16_t *x, *y;
    const int32_t *z;
    const int8_t *normals;
    size_t vertex_count;
    const uint32_t *indices;
    size_t index_count;
} arpt_mesh_prim;

/* Pre-tessellated polygon+line vertices for offscreen texture rasterization */

typedef struct {
    uint16_t x, y;
    float r, g, b, a;
} arpt_poly_vertex;

typedef struct {
    uint16_t x, y;
    float r, g, b, a;
    float local_u, local_v;
    float hw, seg_len;
} arpt_line_vertex;

typedef struct {
    arpt_poly_vertex *poly_verts;
    uint32_t *poly_indices;
    size_t poly_vert_count, poly_index_count;
    arpt_line_vertex *line_verts;
    uint32_t *line_indices;
    size_t line_vert_count, line_index_count;
} arpt_texture_prim;

/* Extruded mesh (owns its buffers) */

typedef struct {
    uint16_t *xy;
    int32_t *z;
    int8_t *normals;
    uint32_t *indices;
    size_t vertex_count, index_count;
} arpt_extrusion_prim;

/* Instanced model batch */

typedef struct {
    uint16_t qx, qy;
    int32_t qz;
    float yaw_scale;
} arpt_instance_pt;

typedef struct {
    arpt_instance_pt *instances;
    size_t count;
    int model_index;
} arpt_instance_batch;

typedef struct {
    arpt_instance_batch *batches;
    int batch_count;
} arpt_instance_prim;

/* Text label glyphs + collision metadata */

typedef struct {
    uint16_t qx, qy;
    int32_t qz;
    float u0, v0, u1, v1;
    float ox, oy;
} arpt_glyph_inst;

typedef struct {
    uint16_t qx, qy;
    int32_t qz;
    float w_px, h_px;
    uint32_t first, count;
} arpt_label_meta;

typedef struct {
    arpt_glyph_inst *glyphs;
    size_t glyph_count;
    arpt_label_meta *labels;
    int label_count;
} arpt_label_prim;

/* Everything the renderer needs to upload one tile */

typedef struct arpt_tile_prims {
    arpt_mesh_prim terrain;
    arpt_texture_prim texture;
    arpt_extrusion_prim extrusion;
    arpt_instance_prim instances;
    arpt_label_prim labels;
    arpt_bounds bounds;
} arpt_tile_prims;

/* Prepare functions — convert decoded domain data to renderer primitives */

void arpt_prepare_terrain(const arpt_terrain_mesh *mesh, arpt_mesh_prim *out);

void arpt_prepare_texture(const arpt_surface_data *surface,
                          const arpt_highway_data *highways,
                          const arpt_style *style, arpt_texture_prim *out);

void arpt_prepare_extrusion(const arpt_surface_data *buildings,
                            arpt_bounds bounds, arpt_extrusion_prim *out);

void arpt_prepare_instances(const arpt_tree_data *trees, int model_count,
                            arpt_instance_prim *out);

void arpt_prepare_labels(const arpt_poi_data *pois, const font_glyph *glyphs,
                         float font_height, arpt_label_prim *out);

void arpt_tile_prims_free(arpt_tile_prims *p);

#endif /* ARPENTRY_TILE_PREPARE_H */
