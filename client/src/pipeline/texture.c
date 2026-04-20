#include "internal.h"

#include "surface.wgsl.h"
#include "line.wgsl.h"
#include "mipmap.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__texture_create_surface_pipeline(WGPUDevice device) {
    WGPUShaderModule sm = create_shader(device, surface_wgsl);

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

/* Stencil-fill pipeline: draws polygon triangles with color writes OFF,
   toggling the stencil bit via INVERT.  Used for even-odd fill rule. */
WGPURenderPipeline arpt__texture_create_stencil_fill_pipeline(WGPUDevice device) {
    WGPUShaderModule sm = create_shader(device, surface_wgsl);
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0});

    WGPUVertexAttribute attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x4, .offset = 4, .shaderLocation = 1},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 20, .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 2, .attributes = attrs,
    };
    WGPUColorTargetState ct = {
        .format = WGPUTextureFormat_RGBA8Unorm,
        .writeMask = WGPUColorWriteMask_None, /* color OFF */
    };
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};
    WGPUStencilFaceState stencil_face = {
        .compare = WGPUCompareFunction_Always,
        .passOp = WGPUStencilOperation_Invert,
        .failOp = WGPUStencilOperation_Keep,
        .depthFailOp = WGPUStencilOperation_Keep,
    };
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24PlusStencil8,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_Always,
        .stencilFront = stencil_face,
        .stencilBack = stencil_face,
        .stencilReadMask = 0xFF,
        .stencilWriteMask = 0xFF,
    };
    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = "vs",
                   .bufferCount = 1, .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .depthStencil = &ds,
        .fragment = &frag,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);
    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

/* Stencil-color pipeline: draws polygon triangles with color writes ON,
   but only where stencil != 0 (even-odd filled area).  Resets stencil
   to 0 on pass so the next group starts clean. */
WGPURenderPipeline arpt__texture_create_stencil_color_pipeline(WGPUDevice device) {
    WGPUShaderModule sm = create_shader(device, surface_wgsl);
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0});

    WGPUVertexAttribute attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x4, .offset = 4, .shaderLocation = 1},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 20, .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 2, .attributes = attrs,
    };
    WGPUColorTargetState ct = {
        .format = WGPUTextureFormat_RGBA8Unorm,
        .writeMask = WGPUColorWriteMask_All,
    };
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};
    WGPUStencilFaceState stencil_face = {
        .compare = WGPUCompareFunction_NotEqual,
        .passOp = WGPUStencilOperation_Zero,
        .failOp = WGPUStencilOperation_Keep,
        .depthFailOp = WGPUStencilOperation_Keep,
    };
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24PlusStencil8,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_Always,
        .stencilFront = stencil_face,
        .stencilBack = stencil_face,
        .stencilReadMask = 0xFF,
        .stencilWriteMask = 0xFF,
    };
    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = "vs",
                   .bufferCount = 1, .buffers = &vbl},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .depthStencil = &ds,
        .fragment = &frag,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);
    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

WGPURenderPipeline arpt__texture_create_line_pipeline(WGPUDevice device) {
    WGPUShaderModule sm = create_shader(device, line_wgsl);

    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0,
                                                .bindGroupLayouts = NULL});

    WGPUVertexAttribute line_attrs[] = {
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
        .attributes = line_attrs,
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

    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24PlusStencil8,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_Always,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask = 0,
        .stencilWriteMask = 0,
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
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

WGPURenderPipeline arpt__texture_create_mipmap_pipeline(WGPUDevice device,
                                                         WGPUBindGroupLayout bgl) {
    WGPUShaderModule sm = create_shader(device, mipmap_wgsl);

    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){
            .bindGroupLayoutCount = 1, .bindGroupLayouts = &bgl});

    WGPUColorTargetState ct = {.format = WGPUTextureFormat_RGBA8Unorm,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = "vs", .bufferCount = 0},
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

/* Generate mip chain by rendering each level from the previous one with a
   fullscreen triangle that bilinearly samples the source mip. */
static void generate_mipmaps(arpt_renderer *r, WGPUCommandEncoder enc,
                             WGPUTexture tex) {
    for (uint32_t level = 1; level < SURFACE_MIP_COUNT; level++) {
        WGPUTextureViewDescriptor src_desc = {
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureViewDimension_2D,
            .baseMipLevel = level - 1,
            .mipLevelCount = 1,
            .baseArrayLayer = 0,
            .arrayLayerCount = 1,
            .aspect = WGPUTextureAspect_All,
        };
        WGPUTextureView src_view = wgpuTextureCreateView(tex, &src_desc);

        WGPUTextureViewDescriptor dst_desc = {
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureViewDimension_2D,
            .baseMipLevel = level,
            .mipLevelCount = 1,
            .baseArrayLayer = 0,
            .arrayLayerCount = 1,
            .aspect = WGPUTextureAspect_All,
        };
        WGPUTextureView dst_view = wgpuTextureCreateView(tex, &dst_desc);

        WGPUBindGroupEntry entries[] = {
            {.binding = 0, .textureView = src_view},
            {.binding = 1, .sampler = r->surface_sampler},
        };
        WGPUBindGroup bg = wgpuDeviceCreateBindGroup(
            r->device, &(WGPUBindGroupDescriptor){.layout = r->mipmap_bgl,
                                                  .entryCount = 2,
                                                  .entries = entries});

        WGPURenderPassColorAttachment color = {
            .view = dst_view,
            .loadOp = WGPULoadOp_Clear,
            .storeOp = WGPUStoreOp_Store,
            .clearValue = {0.0, 0.0, 0.0, 0.0},
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
        wgpuRenderPassEncoderSetPipeline(pass, r->mipmap_pipeline);
        wgpuRenderPassEncoderSetBindGroup(pass, 0, bg, 0, NULL);
        wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);

        wgpuBindGroupRelease(bg);
        wgpuTextureViewRelease(src_view);
        wgpuTextureViewRelease(dst_view);
    }
}

WGPUTexture arpt__texture_rasterize(arpt_renderer *r,
                                     const arpt_polygon_prim *polys,
                                     const arpt_line_prim *lines) {
    WGPUTextureDescriptor tex_desc = {
        .usage =
            WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding,
        .size = {SURFACE_TEX_SIZE, SURFACE_TEX_SIZE, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = SURFACE_MIP_COUNT,
        .sampleCount = 1,
    };
    WGPUTexture tex = wgpuDeviceCreateTexture(r->device, &tex_desc);

    bool has_polys = polys && polys->vert_count > 0 && polys->index_count > 0;
    bool has_lines = lines && lines->vert_count > 0 && lines->index_count > 0;

    /* Render attachment must target a single mip level. */
    WGPUTextureViewDescriptor mip0_desc = {
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureViewDimension_2D,
        .baseMipLevel = 0,
        .mipLevelCount = 1,
        .baseArrayLayer = 0,
        .arrayLayerCount = 1,
        .aspect = WGPUTextureAspect_All,
    };

    if (!has_polys && !has_lines) {
        WGPUTextureView view = wgpuTextureCreateView(tex, &mip0_desc);
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
        generate_mipmaps(r, enc, tex);
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
        poly_vb_size = polys->vert_count * sizeof(arpt_poly_vertex);
        poly_vbuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Vertex, polys->verts,
                                  poly_vb_size);
        poly_ibuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Index, polys->indices,
                                  polys->index_count * sizeof(uint32_t));
        poly_draw_n = polys->index_count;
    }

    /* Build line GPU buffers */
    WGPUBuffer line_vbuf = NULL, line_ibuf = NULL;
    size_t line_vb_size = 0, line_draw_n = 0;
    if (has_lines) {
        line_vb_size = lines->vert_count * sizeof(arpt_line_vertex);
        line_vbuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Vertex, lines->verts,
                                  line_vb_size);
        line_ibuf = create_buffer(r->device, r->queue,
                                  WGPUBufferUsage_Index, lines->indices,
                                  lines->index_count * sizeof(uint32_t));
        line_draw_n = lines->index_count;
    }

    /* Render pass with stencil attachment for even-odd polygon fill */
    WGPUTextureView view = wgpuTextureCreateView(tex, &mip0_desc);
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
    WGPURenderPassDepthStencilAttachment ds_attach = {
        .view = r->stencil_view,
        .depthLoadOp = WGPULoadOp_Clear,
        .depthStoreOp = WGPUStoreOp_Discard,
        .depthClearValue = 1.0f,
        .stencilLoadOp = WGPULoadOp_Clear,
        .stencilStoreOp = WGPUStoreOp_Discard,
        .stencilClearValue = 0,
    };
    WGPURenderPassDescriptor rp_desc = {
        .colorAttachmentCount = 1,
        .colorAttachments = &color,
        .depthStencilAttachment = &ds_attach,
    };
    WGPURenderPassEncoder pass =
        wgpuCommandEncoderBeginRenderPass(enc, &rp_desc);

    /* Draw polygons with stencil-based even-odd fill: for each class group,
       first INVERT stencil for all triangles (exterior + hole rings),
       then color only where stencil != 0 (inside exterior, outside holes). */
    if (poly_draw_n > 0) {
        wgpuRenderPassEncoderSetVertexBuffer(pass, 0, poly_vbuf, 0,
                                             poly_vb_size);
        wgpuRenderPassEncoderSetIndexBuffer(pass, poly_ibuf,
                                            WGPUIndexFormat_Uint32, 0,
                                            poly_draw_n * sizeof(uint32_t));
        wgpuRenderPassEncoderSetStencilReference(pass, 0);

        for (size_t g = 0; g < polys->group_count; g++) {
            uint32_t fi = polys->groups[g].first_index;
            uint32_t ic = polys->groups[g].index_count;
            if (ic == 0) continue;

            /* Pass 1: build stencil mask (INVERT, no color) */
            wgpuRenderPassEncoderSetPipeline(pass, r->stencil_fill_pipeline);
            wgpuRenderPassEncoderDrawIndexed(pass, ic, 1, fi, 0, 0);

            /* Pass 2: color where stencil != 0, reset stencil to 0 */
            wgpuRenderPassEncoderSetPipeline(pass, r->stencil_color_pipeline);
            wgpuRenderPassEncoderDrawIndexed(pass, ic, 1, fi, 0, 0);
        }
    }

    if (line_draw_n > 0) {
        wgpuRenderPassEncoderSetPipeline(pass, r->line_pipeline);
        wgpuRenderPassEncoderSetVertexBuffer(pass, 0, line_vbuf, 0,
                                             line_vb_size);
        wgpuRenderPassEncoderSetIndexBuffer(pass, line_ibuf,
                                            WGPUIndexFormat_Uint32, 0,
                                            line_draw_n * sizeof(uint32_t));
        wgpuRenderPassEncoderDrawIndexed(pass, (uint32_t)line_draw_n, 1, 0, 0,
                                         0);
    }

    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);

    generate_mipmaps(r, enc, tex);

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(enc);
    wgpuTextureViewRelease(view);

    if (poly_vbuf) wgpuBufferRelease(poly_vbuf);
    if (poly_ibuf) wgpuBufferRelease(poly_ibuf);
    if (line_vbuf) wgpuBufferRelease(line_vbuf);
    if (line_ibuf) wgpuBufferRelease(line_ibuf);

    return tex;
}
