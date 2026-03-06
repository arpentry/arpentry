#include "internal.h"

#include "surface.wgsl.h"
#include "highway.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__texture_create_surface_pipeline(WGPUDevice device) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = surface_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0,
                                                .bindGroupLayouts = NULL});

    WGPUVertexAttribute surface_attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 4,
         .shaderLocation = 1},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 20,
        .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 2,
        .attributes = surface_attrs,
    };

    WGPUColorTargetState ct = {.format = WGPUTextureFormat_RGBA8Unorm,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm,
                   .entryPoint = "vs",
                   .bufferCount = 1,
                   .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

WGPURenderPipeline arpt__texture_create_highway_pipeline(WGPUDevice device) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = highway_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0,
                                                .bindGroupLayouts = NULL});

    WGPUVertexAttribute highway_attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2,
         .offset = 0,
         .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 4,
         .shaderLocation = 1},
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 20,
         .shaderLocation = 2},
        {.format = WGPUVertexFormat_Float32x2,
         .offset = 28,
         .shaderLocation = 3},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 36,
        .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 4,
        .attributes = highway_attrs,
    };

    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
        .alpha = {.srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
    };
    WGPUColorTargetState ct = {.format = WGPUTextureFormat_RGBA8Unorm,
                               .blend = &blend,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm,
                   .entryPoint = "vs",
                   .bufferCount = 1,
                   .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

WGPUTexture arpt__texture_rasterize(arpt_renderer *r,
                                     const arpt_texture_prim *prim) {
    WGPUTextureDescriptor tex_desc = {
        .usage =
            WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding,
        .size = {SURFACE_TEX_SIZE, SURFACE_TEX_SIZE, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    WGPUTexture tex = wgpuDeviceCreateTexture(r->device, &tex_desc);

    bool has_polys = prim->poly_vert_count > 0 && prim->poly_index_count > 0;
    bool has_highways = prim->line_vert_count > 0 && prim->line_index_count > 0;

    if (!has_polys && !has_highways) {
        WGPUTextureView view = wgpuTextureCreateView(tex, NULL);
        WGPUCommandEncoder enc =
            wgpuDeviceCreateCommandEncoder(r->device, NULL);
        WGPURenderPassColorAttachment color = {
            .view = view,
            .loadOp = WGPULoadOp_Clear,
            .storeOp = WGPUStoreOp_Store,
            .clearValue = {r->background[0], r->background[1],
                          r->background[2], r->background[3]},
#ifdef __EMSCRIPTEN__
            .depthSlice = WGPU_DEPTH_SLICE_UNDEFINED,
#endif
        };
        WGPURenderPassDescriptor rp = {
            .colorAttachmentCount = 1,
            .colorAttachments = &color,
        };
        WGPURenderPassEncoder pass =
            wgpuCommandEncoderBeginRenderPass(enc, &rp);
        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);
        WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, NULL);
        wgpuQueueSubmit(r->queue, 1, &cmd);
        wgpuCommandBufferRelease(cmd);
        wgpuCommandEncoderRelease(enc);
        wgpuTextureViewRelease(view);
        return tex;
    }

    /* Build polygon GPU buffers */
    WGPUBuffer poly_vbuf = NULL, poly_ibuf = NULL;
    size_t poly_vb_size = 0, poly_draw_n = 0;
    if (has_polys) {
        poly_vb_size = prim->poly_vert_count * sizeof(arpt_poly_vertex);
        poly_vbuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Vertex, prim->poly_verts,
                                  poly_vb_size);
        poly_ibuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Index, prim->poly_indices,
                                  prim->poly_index_count * sizeof(uint32_t));
        poly_draw_n = prim->poly_index_count;
    }

    /* Build highway GPU buffers */
    WGPUBuffer hw_vbuf = NULL, hw_ibuf = NULL;
    size_t hw_vb_size = 0, hw_draw_n = 0;
    if (has_highways) {
        hw_vb_size = prim->line_vert_count * sizeof(arpt_line_vertex);
        hw_vbuf = create_buffer(r->device, r->queue,
                                WGPUBufferUsage_Vertex, prim->line_verts,
                                hw_vb_size);
        hw_ibuf = create_buffer(r->device, r->queue,
                                WGPUBufferUsage_Index, prim->line_indices,
                                prim->line_index_count * sizeof(uint32_t));
        hw_draw_n = prim->line_index_count;
    }

    /* Render pass */
    WGPUTextureView view = wgpuTextureCreateView(tex, NULL);
    WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(r->device, NULL);
    WGPURenderPassColorAttachment color = {
        .view = view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Store,
        .clearValue = {r->background[0], r->background[1],
                       r->background[2], r->background[3]},
#ifdef __EMSCRIPTEN__
        .depthSlice = WGPU_DEPTH_SLICE_UNDEFINED,
#endif
    };
    WGPURenderPassDescriptor rp_desc = {
        .colorAttachmentCount = 1,
        .colorAttachments = &color,
    };
    WGPURenderPassEncoder pass =
        wgpuCommandEncoderBeginRenderPass(enc, &rp_desc);

    if (poly_draw_n > 0) {
        wgpuRenderPassEncoderSetPipeline(pass, r->surface_pipeline);
        wgpuRenderPassEncoderSetVertexBuffer(pass, 0, poly_vbuf, 0,
                                             poly_vb_size);
        wgpuRenderPassEncoderSetIndexBuffer(pass, poly_ibuf,
                                            WGPUIndexFormat_Uint32, 0,
                                            poly_draw_n * sizeof(uint32_t));
        wgpuRenderPassEncoderDrawIndexed(pass, (uint32_t)poly_draw_n, 1, 0, 0,
                                         0);
    }

    if (hw_draw_n > 0) {
        wgpuRenderPassEncoderSetPipeline(pass, r->highway_pipeline);
        wgpuRenderPassEncoderSetVertexBuffer(pass, 0, hw_vbuf, 0,
                                             hw_vb_size);
        wgpuRenderPassEncoderSetIndexBuffer(pass, hw_ibuf,
                                            WGPUIndexFormat_Uint32, 0,
                                            hw_draw_n * sizeof(uint32_t));
        wgpuRenderPassEncoderDrawIndexed(pass, (uint32_t)hw_draw_n, 1, 0, 0,
                                         0);
    }

    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(enc);
    wgpuTextureViewRelease(view);

    if (poly_vbuf) wgpuBufferRelease(poly_vbuf);
    if (poly_ibuf) wgpuBufferRelease(poly_ibuf);
    if (hw_vbuf) wgpuBufferRelease(hw_vbuf);
    if (hw_ibuf) wgpuBufferRelease(hw_ibuf);

    return tex;
}
