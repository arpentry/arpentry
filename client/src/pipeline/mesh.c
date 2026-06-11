#include "internal.h"
#include "coords.h"

#include "terrain.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__mesh_create_pipeline(WGPUDevice device,
                                               WGPUTextureFormat format,
                                               WGPUBindGroupLayout global_bgl,
                                               WGPUBindGroupLayout tile_bgl) {
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

    WGPUColorTargetState ct = {.format = format,
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

    /* Pad normals to 4-byte stride */
    {
        int8_t *padded = pad_normals_2to4(prim->normals, vc);
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

    uint16_t edge_min = (uint16_t)ARPT_BUFFER;
    uint16_t edge_max = (uint16_t)(ARPT_BUFFER + ARPT_EXTENT - 1);

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

void arpt__mesh_draw_extrusion(arpt_renderer *r, arpt_tile_gpu *tile) {
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
