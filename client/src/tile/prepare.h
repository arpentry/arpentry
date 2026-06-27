#ifndef ARPENTRY_TILE_PREPARE_H
#define ARPENTRY_TILE_PREPARE_H

#include "coords.h"
#include "font.h"
#include "icon.h"
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

/* Pre-tessellated polygon vertices for offscreen texture rasterization */

typedef struct {
    uint16_t x, y;
    float r, g, b, a;
} arpt_poly_vertex;

/* A group of polygon triangles sharing the same class, rendered together
   in a single stencil pass for correct even-odd fill (handles holes). */
typedef struct {
    uint32_t first_index;  /* offset into indices */
    uint32_t index_count;  /* number of indices in this group */
} arpt_poly_group;

typedef struct {
    arpt_poly_vertex *verts;
    uint32_t *indices;
    size_t vert_count, index_count;
    arpt_poly_group *groups;
    size_t group_count;
} arpt_polygon_prim;

/* Pre-tessellated line SDF quads, draped on the terrain surface and drawn as
   3D geometry (not rasterized to the surface texture).  qz is the terrain
   elevation (mm) sampled at the vertex so the road follows the ground. */

typedef struct {
    uint16_t x, y;
    int32_t qz;
    float r, g, b, a;
    float local_u, local_v;
    float hw, seg_len;
} arpt_line_vertex;

typedef struct {
    arpt_line_vertex *verts;
    uint32_t *indices;
    size_t vert_count, index_count;
} arpt_line_prim;

/* Building mesh — server-baked walls + roof (owns its buffers) */

typedef struct {
    uint16_t *xy;
    int32_t *z;
    int8_t *normals;
    uint32_t *indices;
    size_t vertex_count, index_count;
} arpt_building_prim;

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

/* Icon instance (one per POI, same vertex format as glyph for reuse) */
typedef struct {
    uint16_t qx, qy;
    int32_t qz;
    float u0, v0, u1, v1;
    float ox, oy;
} arpt_icon_inst;

typedef struct {
    arpt_glyph_inst *glyphs;
    size_t glyph_count;
    arpt_label_meta *labels;
    int label_count;
    arpt_icon_inst *icons;
    size_t icon_count;
} arpt_label_prim;

/* Line-following labels (street names): the polyline is kept CPU-side and
   glyphs are placed along its screen projection every frame. */

/* Per-frame placement projects every vertex, so bound the work (and the
   renderer's stack scratch) per label. */
#define ARPT_MAX_LINE_LABEL_POINTS 256

typedef struct {
    uint16_t *x, *y;       /* owned copy of the polyline, tile coords */
    uint32_t vertex_count;
    int32_t qz;            /* terrain elevation near the line midpoint, mm */
    char name[64];
    float text_w_px;       /* total advance at the atlas font size */
} arpt_line_label;

typedef struct {
    arpt_line_label *labels; /* malloc'd array */
    int count;
} arpt_line_label_prim;

/* Everything the renderer needs to upload one tile */

typedef struct arpt_tile_prims {
    arpt_terrain_mesh terrain;
    arpt_polygon_prim polygons;
    arpt_line_prim lines;
    arpt_building_prim buildings;
    arpt_building_prim bridges; /* server-baked bridge deck prisms (same form) */
    arpt_building_prim tunnels; /* server-baked tunnel bore prisms (same form) */
    arpt_instance_prim instances;
    arpt_label_prim labels;
    arpt_line_label_prim line_labels;
    arpt_bounds bounds;
} arpt_tile_prims;

/* Prepare functions — convert decoded domain data to renderer primitives */

void arpt_prepare_polygons(const arpt_surface_data *surface,
                           const arpt_style *style, arpt_polygon_prim *out);

void arpt_prepare_lines(const arpt_line_data *line_data,
                        const arpt_style *style, int level,
                        arpt_line_prim *out);

/* Buildings arrive as server-baked 3D meshes (MeshGeometry): walls + roof,
   anchored to the terrain, with roof shapes derived from source attributes.
   Decode every MeshGeometry feature of the named layer straight into the
   building primitive (xy/z/normals/indices). Returns true and fills `out` when
   the layer holds meshes; returns false (with `out` zeroed) when the layer is
   absent or empty. */
bool arpt_decode_building_mesh(const void *flatbuf, size_t size,
                               const char *layer_name,
                               arpt_building_prim *out);

/* Road structures arrive as server-baked box prisms (MeshGeometry) carried inside
   the transportation layer next to the road lines. These split them by the
   reserved `level` property so bridges and tunnels colour differently: each scans
   the named layer and concatenates only its bridge (level > 0) or tunnel
   (level < 0) meshes into the primitive (same form as buildings). Returns true
   and fills `out` when the layer holds matching meshes; false (with `out` zeroed)
   otherwise. */
bool arpt_decode_bridge_mesh(const void *flatbuf, size_t size,
                             const char *layer_name, arpt_building_prim *out);
bool arpt_decode_tunnel_mesh(const void *flatbuf, size_t size,
                             const char *layer_name, arpt_building_prim *out);

void arpt_prepare_instances(const arpt_tree_data *trees, int model_count,
                            arpt_instance_prim *out);

void arpt_prepare_labels(const arpt_poi_data *pois, const font_glyph *glyphs,
                         float font_height, const icon_glyph *icons,
                         int num_icons, float icon_height,
                         arpt_label_prim *out);

void arpt_prepare_line_labels(const arpt_line_label_data *data,
                              const arpt_terrain_mesh *terrain,
                              const font_glyph *glyphs,
                              arpt_line_label_prim *out);

void arpt_tile_prims_free(arpt_tile_prims *p);

#endif /* ARPENTRY_TILE_PREPARE_H */
