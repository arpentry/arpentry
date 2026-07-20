#include "internal.h"
#include "coords.h"

#include "terrain.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__mesh_create_pipeline(WGPUDevice device,
                                               WGPUTextureFormat format,
                                               WGPUBindGroupLayout global_bgl,
                                               WGPUBindGroupLayout tile_bgl,
                                               bool blend) {
    WGPUShaderModule sm = create_shader(device, terrain_wgsl);
    if (!sm) return NULL;

    WGPUBindGroupLayout bgls[] = {global_bgl, tile_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 2,
                                                .bindGroupLayouts = bgls});

    WGPUVertexAttribute attr_xy = {
        .format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0};
    WGPUVertexAttribute attr_z = {
        .format = WGPUVertexFormat_Sint32, .offset = 0, .shaderLocation = 1};
    WGPUVertexAttribute attr_n = {
        .format = WGPUVertexFormat_Sint8x2, .offset = 0, .shaderLocation = 2};
    WGPUVertexBufferLayout vbls[] = {
        {.arrayStride = 4,
         .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1,
         .attributes = &attr_xy},
        {.arrayStride = 4,
         .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1,
         .attributes = &attr_z},
        {.arrayStride = 4,
         .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1,
         .attributes = &attr_n},
    };

    /* The x-ray pipeline blends so the terrain's sub-1.0 alpha shows the buried
       tunnel (drawn before terrain) through the ground; depth write stays on so
       the surface still sorts buildings and roads. */
    WGPUBlendState bs = {
        .color = {.operation = WGPUBlendOperation_Add,
                  .srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha},
        .alpha = {.operation = WGPUBlendOperation_Add,
                  .srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha},
    };
    WGPUColorTargetState ct = {.format = format,
                               .blend = blend ? &bs : NULL,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};
    WGPUDepthStencilState ds = {
        .format = ARPT_DEPTH_FORMAT,
        .depthWriteEnabled = true,
        .depthCompare = WGPUCompareFunction_LessEqual,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask = 0,
        .stencilWriteMask = 0,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm,
                   .entryPoint = "vs",
                   .bufferCount = 3,
                   .buffers = vbls},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_Back,
                      .frontFace = WGPUFrontFace_CCW},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = ARPT_MSAA_SAMPLES, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__mesh_upload_terrain(arpt_renderer *r, arpt_tile_gpu *t,
                               const arpt_terrain_mesh *prim) {
    /* Interleave x,y into uint16 pairs */
    size_t vc = prim->vertex_count;
    uint16_t *xy = malloc(vc * 4);
    if (!xy) return;
    for (size_t i = 0; i < vc; i++) {
        xy[i * 2] = prim->x[i];
        xy[i * 2 + 1] = prim->y[i];
    }
    t->buf_xy =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex, xy, vc * 4);
    free(xy);

    t->buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                             prim->z, vc * sizeof(int32_t));

    /* Pad normals to 4-byte stride (terrain carries no across-coords → NULL) */
    {
        int8_t *padded = pad_normals_2to4(prim->normals, NULL, vc);
        if (!padded) return;
        t->buf_normals = create_buffer(r->device, r->queue,
                                       WGPUBufferUsage_Vertex, padded, vc * 4);
        free(padded);
    }

    t->buf_indices =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Index, prim->indices,
                      prim->index_count * sizeof(uint32_t));

    /* Mark drawable only once every buffer exists, so a partial upload
       (allocation failure above) is skipped by the draw path. */
    if (t->buf_xy && t->buf_z && t->buf_normals && t->buf_indices)
        t->index_count = (uint32_t)prim->index_count;
}

void arpt__mesh_draw_terrain(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->index_count == 0) return;
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group, 0, NULL);
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 0, tile->buf_xy, 0,
                                         wgpuBufferGetSize(tile->buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 1, tile->buf_z, 0,
                                         wgpuBufferGetSize(tile->buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 2, tile->buf_normals, 0,
                                         wgpuBufferGetSize(tile->buf_normals));
    wgpuRenderPassEncoderSetIndexBuffer(r->pass, tile->buf_indices,
                                        WGPUIndexFormat_Uint32, 0,
                                        wgpuBufferGetSize(tile->buf_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, tile->index_count, 1, 0, 0, 0);
}

/* Skirt generation: vertical walls along tile edges to hide cracks */

#define SKIRT_DROP 500000 /* 500 meters in millimeters */

/* Edge detection helper */
typedef struct {
    uint32_t idx;   /* original vertex index */
    uint16_t sort;  /* coordinate to sort by */
} edge_vert;

static int edge_vert_cmp(const void *a, const void *b) {
    const edge_vert *va = a;
    const edge_vert *vb = b;
    return (int)va->sort - (int)vb->sort;
}

void arpt__mesh_upload_skirts(arpt_renderer *r, arpt_tile_gpu *t,
                               const arpt_terrain_mesh *prim) {
    if (prim->vertex_count == 0) return;

    /* The tile proper spans quantized [BUFFER, BUFFER + EXTENT] inclusive:
       the server puts east/north edge vertices at exactly 49152 (see
       server/src/terrain.rs), not 49151. */
    uint16_t edge_min = (uint16_t)ARPT_BUFFER;
    uint16_t edge_max = (uint16_t)(ARPT_BUFFER + ARPT_EXTENT);

    /* Collect edge vertices for all 4 edges */
    size_t vc = prim->vertex_count;
    size_t max_edge = vc; /* worst case: all verts on an edge */
    edge_vert *west = malloc(max_edge * sizeof(*west));
    edge_vert *east = malloc(max_edge * sizeof(*east));
    edge_vert *south = malloc(max_edge * sizeof(*south));
    edge_vert *north = malloc(max_edge * sizeof(*north));
    if (!west || !east || !south || !north) {
        free(west); free(east); free(south); free(north);
        return;
    }

    size_t nw = 0, ne = 0, ns = 0, nn = 0;
    for (size_t i = 0; i < vc; i++) {
        uint16_t x = prim->x[i];
        uint16_t y = prim->y[i];
        if (x == edge_min) { west[nw].idx = (uint32_t)i; west[nw].sort = y; nw++; }
        if (x == edge_max) { east[ne].idx = (uint32_t)i; east[ne].sort = y; ne++; }
        if (y == edge_min) { south[ns].idx = (uint32_t)i; south[ns].sort = x; ns++; }
        if (y == edge_max) { north[nn].idx = (uint32_t)i; north[nn].sort = x; nn++; }
    }

    /* Sort each edge by varying coordinate */
    if (nw > 1) qsort(west, nw, sizeof(*west), edge_vert_cmp);
    if (ne > 1) qsort(east, ne, sizeof(*east), edge_vert_cmp);
    if (ns > 1) qsort(south, ns, sizeof(*south), edge_vert_cmp);
    if (nn > 1) qsort(north, nn, sizeof(*north), edge_vert_cmp);

    /* Count skirt vertices and indices:
       Each edge with N vertices produces 2*N vertices and 6*(N-1) indices */
    size_t total_edge_verts = nw + ne + ns + nn;
    if (total_edge_verts < 2) {
        free(west); free(east); free(south); free(north);
        return;
    }
    size_t total_skirt_verts = total_edge_verts * 2;
    size_t total_skirt_quads = 0;
    if (nw > 1) total_skirt_quads += nw - 1;
    if (ne > 1) total_skirt_quads += ne - 1;
    if (ns > 1) total_skirt_quads += ns - 1;
    if (nn > 1) total_skirt_quads += nn - 1;
    size_t total_skirt_indices = total_skirt_quads * 6;

    if (total_skirt_indices == 0) {
        free(west); free(east); free(south); free(north);
        return;
    }

    /* Allocate skirt buffers */
    uint16_t *s_xy = malloc(total_skirt_verts * 4);
    int32_t *s_z = malloc(total_skirt_verts * sizeof(int32_t));
    int8_t *s_normals = calloc(total_skirt_verts, 4);
    uint32_t *s_indices = malloc(total_skirt_indices * sizeof(uint32_t));
    if (!s_xy || !s_z || !s_normals || !s_indices) {
        free(s_xy); free(s_z); free(s_normals); free(s_indices);
        free(west); free(east); free(south); free(north);
        return;
    }

    uint32_t vi = 0; /* current vertex index in skirt buffer */
    uint32_t ii = 0; /* current index in skirt index buffer */

    /* Helper macro to emit one edge's skirt geometry */
#define EMIT_EDGE(edge_arr, edge_len)                                          \
    do {                                                                        \
        uint32_t base = vi;                                                    \
        for (size_t e = 0; e < (edge_len); e++) {                              \
            uint32_t oi = (edge_arr)[e].idx;                                   \
            /* Original vertex */                                              \
            s_xy[vi * 2] = prim->x[oi];                                       \
            s_xy[vi * 2 + 1] = prim->y[oi];                                   \
            s_z[vi] = prim->z[oi];                                            \
            if (prim->normals) {                                               \
                s_normals[vi * 4] = prim->normals[oi * 2];                    \
                s_normals[vi * 4 + 1] = prim->normals[oi * 2 + 1];           \
            }                                                                  \
            vi++;                                                              \
            /* Lowered vertex */                                               \
            s_xy[vi * 2] = prim->x[oi];                                       \
            s_xy[vi * 2 + 1] = prim->y[oi];                                   \
            s_z[vi] = prim->z[oi] - SKIRT_DROP;                               \
            if (prim->normals) {                                               \
                s_normals[vi * 4] = prim->normals[oi * 2];                    \
                s_normals[vi * 4 + 1] = prim->normals[oi * 2 + 1];           \
            }                                                                  \
            vi++;                                                              \
        }                                                                      \
        for (size_t e = 0; e + 1 < (edge_len); e++) {                         \
            uint32_t a = base + (uint32_t)(e * 2);                            \
            uint32_t b = a + 1;                                                \
            uint32_t c = a + 2;                                                \
            uint32_t d = a + 3;                                                \
            /* Two triangles forming a quad: (a, b, c), (c, b, d) */           \
            s_indices[ii++] = a; s_indices[ii++] = b; s_indices[ii++] = c;    \
            s_indices[ii++] = c; s_indices[ii++] = b; s_indices[ii++] = d;    \
        }                                                                      \
    } while (0)

    EMIT_EDGE(west, nw);
    EMIT_EDGE(east, ne);
    EMIT_EDGE(south, ns);
    EMIT_EDGE(north, nn);

#undef EMIT_EDGE

    free(west); free(east); free(south); free(north);

    /* Upload to GPU */
    t->skirt_buf_xy = create_buffer(r->device, r->queue,
                                     WGPUBufferUsage_Vertex, s_xy,
                                     total_skirt_verts * 4);
    t->skirt_buf_z = create_buffer(r->device, r->queue,
                                    WGPUBufferUsage_Vertex, s_z,
                                    total_skirt_verts * sizeof(int32_t));
    t->skirt_buf_normals = create_buffer(r->device, r->queue,
                                          WGPUBufferUsage_Vertex, s_normals,
                                          total_skirt_verts * 4);
    t->skirt_buf_indices = create_buffer(r->device, r->queue,
                                          WGPUBufferUsage_Index, s_indices,
                                          total_skirt_indices * sizeof(uint32_t));
    if (t->skirt_buf_xy && t->skirt_buf_z && t->skirt_buf_normals &&
        t->skirt_buf_indices)
        t->skirt_index_count = (uint32_t)total_skirt_indices;

    free(s_xy); free(s_z); free(s_normals); free(s_indices);
}

void arpt__mesh_draw_skirts(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->skirt_index_count == 0) return;
    /* Tile bind group is already set by draw_terrain */
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 0, tile->skirt_buf_xy, 0,
        wgpuBufferGetSize(tile->skirt_buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 1, tile->skirt_buf_z, 0,
        wgpuBufferGetSize(tile->skirt_buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 2, tile->skirt_buf_normals, 0,
        wgpuBufferGetSize(tile->skirt_buf_normals));
    wgpuRenderPassEncoderSetIndexBuffer(
        r->pass, tile->skirt_buf_indices, WGPUIndexFormat_Uint32, 0,
        wgpuBufferGetSize(tile->skirt_buf_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, tile->skirt_index_count, 1, 0, 0,
                                     0);
}

void arpt__mesh_draw_buildings(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->bldg_index_count == 0) return;
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bldg_bind_group, 0,
                                      NULL);
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 0, tile->bldg_buf_xy, 0,
        wgpuBufferGetSize(tile->bldg_buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 1, tile->bldg_buf_z, 0,
        wgpuBufferGetSize(tile->bldg_buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(
        r->pass, 2, tile->bldg_buf_normals, 0,
        wgpuBufferGetSize(tile->bldg_buf_normals));
    wgpuRenderPassEncoderSetIndexBuffer(
        r->pass, tile->bldg_buf_indices, WGPUIndexFormat_Uint32, 0,
        wgpuBufferGetSize(tile->bldg_buf_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, tile->bldg_index_count, 1, 0,
                                     0, 0);
}

/* Structure pipeline (bridge decks + tunnel bores): the terrain shader + vertex
   layout, drawn as ordinary opaque 3D geometry (depth-test LessEqual + depth
   write). A bridge deck stands above the terrain (a viaduct); a tunnel box is
   occluded by the terrain it passes under, so the bore reads as genuinely
   underground, surfacing only at the portals. Culling is off so the solid
   rectangular tube reads correctly from inside or out regardless of winding. */
WGPURenderPipeline arpt__mesh_create_structure_pipeline(WGPUDevice device,
                                                     const char *vs_entry,
                                                     const char *fs_entry,
                                                     WGPUTextureFormat format,
                                                     WGPUBindGroupLayout global_bgl,
                                                     WGPUBindGroupLayout tile_bgl,
                                                     bool depth_write) {
    WGPUShaderModule sm = create_shader(device, terrain_wgsl);
    if (!sm) return NULL;

    WGPUBindGroupLayout bgls[] = {global_bgl, tile_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 2,
                                                .bindGroupLayouts = bgls});

    WGPUVertexAttribute attr_xy = {
        .format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0};
    WGPUVertexAttribute attr_z = {
        .format = WGPUVertexFormat_Sint32, .offset = 0, .shaderLocation = 1};
    /* Two attributes share the 4-byte normals buffer: the int8×2 octahedral
       normal at byte 0, and the signed across-carriageway coord (snorm8) the
       server packs into byte 2 for analytic edge AA (Snorm8x2 → .x is the
       coord, .y the reserved byte 3). */
    WGPUVertexAttribute attr_norm[2] = {
        {.format = WGPUVertexFormat_Sint8x2, .offset = 0, .shaderLocation = 2},
        {.format = WGPUVertexFormat_Snorm8x2, .offset = 2, .shaderLocation = 5},
    };
    /* Per-vertex road-class deck colour (unorm8x4 → vec4<f32> 0..1), read by the
       structure vertex entries (vs_deck / vs_deck_bridge) for the asphalt top. */
    WGPUVertexAttribute attr_color = {
        .format = WGPUVertexFormat_Unorm8x4, .offset = 0, .shaderLocation = 3};
    WGPUVertexBufferLayout vbls[] = {
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_xy},
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_z},
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 2, .attributes = attr_norm},
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_color},
    };

    /* Alpha blending so the deck fragment's analytic edge coverage (fs_deck
       returns <1 only along a drivable surface's silhouette) fades that ~1px
       band into the ground; every other pixel returns alpha 1 and stays
       opaque. */
    WGPUBlendState blend = {
        .color = {.operation = WGPUBlendOperation_Add,
                  .srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha},
        .alpha = {.operation = WGPUBlendOperation_Add,
                  .srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha},
    };
    WGPUColorTargetState ct = {.format = format,
                               .blend = &blend,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = fs_entry, .targetCount = 1, .targets = &ct};
    WGPUDepthStencilState ds = {
        .format = ARPT_DEPTH_FORMAT,
        .depthWriteEnabled = depth_write,
        .depthCompare = WGPUCompareFunction_LessEqual,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask = 0,
        .stencilWriteMask = 0,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = vs_entry, .bufferCount = 4,
                   .buffers = vbls},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None,
                      .frontFace = WGPUFrontFace_CCW},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = ARPT_MSAA_SAMPLES, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__mesh_upload_structure(arpt_renderer *r, arpt_mesh_draw *d,
                                 const arpt_building_prim *prim) {
    if (!prim || prim->vertex_count == 0 || prim->index_count == 0 ||
        !prim->normals)
        return;

    size_t nv = prim->vertex_count;
    size_t ni = prim->index_count;

    d->buf_xy = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                              prim->xy, nv * 4);
    d->buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                             prim->z, nv * sizeof(int32_t));
    {
        int8_t *padded = pad_normals_2to4(prim->normals, prim->edge_across, nv);
        if (!padded) return;
        d->buf_normals = create_buffer(
            r->device, r->queue, WGPUBufferUsage_Vertex, padded, nv * 4);
        free(padded);
    }
    /* Per-vertex deck colour. The structure pipeline always binds vertex buffer
       3, so fall back to a neutral fill (alpha 0 → shader's motorway default)
       when the decoder shipped no colour, keeping the layout valid. */
    if (prim->color) {
        d->buf_color = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                     prim->color, nv * 4);
    } else {
        uint8_t *fill = calloc(nv, 4);
        if (fill) {
            d->buf_color = create_buffer(r->device, r->queue,
                                         WGPUBufferUsage_Vertex, fill, nv * 4);
            free(fill);
        }
    }
    d->buf_indices = create_buffer(r->device, r->queue,
                                   WGPUBufferUsage_Index, prim->indices,
                                   ni * sizeof(uint32_t));

    if (d->buf_xy && d->buf_z && d->buf_normals && d->buf_color && d->buf_indices)
        d->index_count = (uint32_t)ni;
}

void arpt__mesh_draw_structure(arpt_renderer *r, arpt_mesh_draw *d,
                               WGPURenderPipeline pipeline) {
    if (d->index_count == 0) return;
    wgpuRenderPassEncoderSetPipeline(r->pass, pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0, NULL);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, d->bind_group, 0, NULL);
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 0, d->buf_xy, 0,
                                         wgpuBufferGetSize(d->buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 1, d->buf_z, 0,
                                         wgpuBufferGetSize(d->buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 2, d->buf_normals, 0,
                                         wgpuBufferGetSize(d->buf_normals));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 3, d->buf_color, 0,
                                         wgpuBufferGetSize(d->buf_color));
    wgpuRenderPassEncoderSetIndexBuffer(
        r->pass, d->buf_indices, WGPUIndexFormat_Uint32, 0,
        wgpuBufferGetSize(d->buf_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, d->index_count, 1, 0, 0, 0);
    /* Restore terrain pipeline for subsequent tile draws. */
    restore_terrain_pipeline(r);
}
