#include "internal.h"

#include "terrain.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__mesh_create_pipeline(WGPUDevice device,
                                               WGPUTextureFormat format,
                                               WGPUBindGroupLayout global_bgl,
                                               WGPUBindGroupLayout tile_bgl) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = terrain_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

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
        .format = WGPUTextureFormat_Depth24Plus,
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
                      .frontFace = WGPUFrontFace_CW},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__mesh_upload_terrain(arpt_renderer *r, arpt_tile_gpu *t,
                               const arpt_mesh_prim *prim) {
    t->index_count = (uint32_t)prim->index_count;

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
        int8_t *padded = calloc(vc, 4);
        if (!padded) return;
        for (size_t i = 0; i < vc; i++) {
            if (prim->normals) {
                padded[i * 4] = prim->normals[i * 2];
                padded[i * 4 + 1] = prim->normals[i * 2 + 1];
            }
        }
        t->buf_normals = create_buffer(r->device, r->queue,
                                       WGPUBufferUsage_Vertex, padded, vc * 4);
        free(padded);
    }

    t->buf_indices =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Index, prim->indices,
                      prim->index_count * sizeof(uint32_t));
}

void arpt__mesh_draw_terrain(arpt_renderer *r, arpt_tile_gpu *tile) {
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
