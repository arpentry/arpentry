#include "internal.h"

#include "road.wgsl.h"

#include <stdlib.h>

/* Draped roads: SDF stroke quads conforming to the terrain surface, drawn as
   3D geometry in the main pass so road edges stay crisp at any zoom (instead
   of being baked into the fixed-resolution surface texture).  Shares the
   global + tile bind group layouts with the terrain pipeline. */

WGPURenderPipeline arpt__road_create_pipeline(WGPUDevice device,
                                              WGPUTextureFormat format,
                                              WGPUBindGroupLayout global_bgl,
                                              WGPUBindGroupLayout tile_bgl) {
    WGPUShaderModule sm = create_shader(device, road_wgsl);
    if (!sm) return NULL;

    WGPUBindGroupLayout bgls[] = {global_bgl, tile_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 2,
                                                .bindGroupLayouts = bgls});

    /* Matches arpt_line_vertex (44-byte stride): qxy, qz, color, local, hw_len,
       centerline-xy. */
    WGPUVertexAttribute attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2,  .offset = 0,  .shaderLocation = 0},
        {.format = WGPUVertexFormat_Sint32,    .offset = 4,  .shaderLocation = 1},
        {.format = WGPUVertexFormat_Float32x4, .offset = 8,  .shaderLocation = 2},
        {.format = WGPUVertexFormat_Float32x2, .offset = 24, .shaderLocation = 3},
        {.format = WGPUVertexFormat_Float32x2, .offset = 32, .shaderLocation = 4},
        {.format = WGPUVertexFormat_Uint16x2,  .offset = 40, .shaderLocation = 5},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 44,
        .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 6,
        .attributes = attrs,
    };

    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
        .alpha = {.srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
    };
    WGPUColorTargetState ct = {.format = format,
                               .blend = &blend,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    /* Roads are decals: test against the terrain depth but do not write it.
       road.wgsl's camera-facing margin is DEPTH-ONLY (the projected position
       keeps the true vertex), so only a tiny constant depth bias is needed
       against residual z-fighting — NOT a slope-scaled one, which at
       grazing angles on steep terrain would punch roads through ridges.
       Under reversed-Z (renderer.h) "toward the camera" is +depth, so the
       one-unit bias is positive. */
    WGPUDepthStencilState ds = {
        .format = ARPT_DEPTH_FORMAT,
        .depthWriteEnabled = false,
        .depthCompare = ARPT_DEPTH_COMPARE,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask = 0,
        .stencilWriteMask = 0,
        .depthBias = 1,
        .depthBiasSlopeScale = 0.0f,
        .depthBiasClamp = 0.0f,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm,
                   .entryPoint = "vs",
                   .bufferCount = 1,
                   .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .depthStencil = &ds,
        .fragment = &frag,
        .multisample = {.count = ARPT_MSAA_SAMPLES, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__road_upload(arpt_renderer *r, arpt_tile_gpu *t,
                       const arpt_line_prim *prim) {
    if (!prim || prim->vert_count == 0 || prim->index_count == 0) return;

    t->road_buf_vert = create_buffer(
        r->device, r->queue, WGPUBufferUsage_Vertex, prim->verts,
        prim->vert_count * sizeof(arpt_line_vertex));
    t->road_buf_index = create_buffer(
        r->device, r->queue, WGPUBufferUsage_Index, prim->indices,
        prim->index_count * sizeof(uint32_t));

    if (t->road_buf_vert && t->road_buf_index)
        t->road_index_count = (uint32_t)prim->index_count;
}

void arpt__road_draw(arpt_renderer *r, arpt_tile_gpu *tile) {
    if (tile->road_index_count == 0) return;
    wgpuRenderPassEncoderSetPipeline(r->pass, r->road_pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0, NULL);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group, 0, NULL);
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 0, tile->road_buf_vert, 0,
                                         wgpuBufferGetSize(tile->road_buf_vert));
    wgpuRenderPassEncoderSetIndexBuffer(
        r->pass, tile->road_buf_index, WGPUIndexFormat_Uint32, 0,
        wgpuBufferGetSize(tile->road_buf_index));
    wgpuRenderPassEncoderDrawIndexed(r->pass, tile->road_index_count, 1, 0, 0,
                                     0);
    /* Restore terrain pipeline for subsequent tile draws (e.g. buildings). */
    restore_terrain_pipeline(r);
}
