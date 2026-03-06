#include "internal.h"

#include "wireframe.wgsl.h"

WGPURenderPipeline arpt__placeholder_create_wireframe_pipeline(
    WGPUDevice device, WGPUTextureFormat format,
    WGPUBindGroupLayout global_bgl, WGPUBindGroupLayout tile_bgl) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = wireframe_wgsl,
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
    WGPUVertexAttribute attr_dist = {
        .format = WGPUVertexFormat_Float32, .offset = 0, .shaderLocation = 2};
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
         .attributes = &attr_dist},
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
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24Plus,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_LessEqual,
        .depthBias = -2,
        .depthBiasSlopeScale = -1.0f,
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
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__placeholder_init(arpt_renderer *r) {
#define PH_GRID 16
#define PH_VERTS (PH_GRID + 1)
#define PH_NV (PH_VERTS * PH_VERTS)
#define PH_NI (PH_GRID * PH_GRID * 6)

    uint16_t xy_data[PH_NV * 2];
    int32_t z_data[PH_NV];
    int8_t norm_data[PH_NV * 4];
    uint32_t idx_data[PH_NI];

    for (int row = 0; row < PH_VERTS; row++) {
        for (int col = 0; col < PH_VERTS; col++) {
            int vi = row * PH_VERTS + col;
            xy_data[vi * 2] = (uint16_t)(16384 + col * 32767 / PH_GRID);
            xy_data[vi * 2 + 1] = (uint16_t)(16384 + row * 32767 / PH_GRID);
            z_data[vi] = 0;
            norm_data[vi * 4] = 0;
            norm_data[vi * 4 + 1] = 0;
            norm_data[vi * 4 + 2] = 0;
            norm_data[vi * 4 + 3] = 0;
        }
    }

    int ii = 0;
    for (int row = 0; row < PH_GRID; row++) {
        for (int col = 0; col < PH_GRID; col++) {
            uint32_t tl = (uint32_t)(row * PH_VERTS + col);
            uint32_t tr = tl + 1;
            uint32_t bl = tl + PH_VERTS;
            uint32_t br = bl + 1;
            idx_data[ii++] = tl;
            idx_data[ii++] = bl;
            idx_data[ii++] = tr;
            idx_data[ii++] = tr;
            idx_data[ii++] = bl;
            idx_data[ii++] = br;
        }
    }

    r->ph_index_count = PH_NI;
    r->ph_buf_xy = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                 xy_data, sizeof(xy_data));
    r->ph_buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                z_data, sizeof(z_data));
    r->ph_buf_normals = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                      norm_data, sizeof(norm_data));
    r->ph_buf_indices = create_buffer(r->device, r->queue, WGPUBufferUsage_Index,
                                      idx_data, sizeof(idx_data));

    /* Wireframe SDF quads */
#define PH_WIRE_HW 200
#define PH_EDGE_COUNT (PH_VERTS * PH_GRID + PH_GRID * PH_VERTS)
#define PH_WIRE_NV (PH_EDGE_COUNT * 4)
#define PH_WIRE_NI2 (PH_EDGE_COUNT * 6)

    uint16_t w_xy[PH_WIRE_NV * 2];
    int32_t w_z[PH_WIRE_NV];
    float w_dist[PH_WIRE_NV];
    uint32_t w_idx[PH_WIRE_NI2];
    int wv = 0, wi = 0;

    /* Horizontal edges */
    for (int row = 0; row < PH_VERTS; row++) {
        for (int col = 0; col < PH_GRID; col++) {
            uint16_t x0 = (uint16_t)(16384 + col * 32767 / PH_GRID);
            uint16_t x1 = (uint16_t)(16384 + (col + 1) * 32767 / PH_GRID);
            int y = 16384 + row * 32767 / PH_GRID;
            uint16_t yl = (uint16_t)(y - PH_WIRE_HW);
            uint16_t yh = (uint16_t)(y + PH_WIRE_HW);
            uint32_t base = (uint32_t)wv;
            w_xy[wv*2]=x0; w_xy[wv*2+1]=yl; w_z[wv]=0; w_dist[wv]=(float)-PH_WIRE_HW; wv++;
            w_xy[wv*2]=x0; w_xy[wv*2+1]=yh; w_z[wv]=0; w_dist[wv]=(float)+PH_WIRE_HW; wv++;
            w_xy[wv*2]=x1; w_xy[wv*2+1]=yl; w_z[wv]=0; w_dist[wv]=(float)-PH_WIRE_HW; wv++;
            w_xy[wv*2]=x1; w_xy[wv*2+1]=yh; w_z[wv]=0; w_dist[wv]=(float)+PH_WIRE_HW; wv++;
            w_idx[wi++]=base; w_idx[wi++]=base+1; w_idx[wi++]=base+2;
            w_idx[wi++]=base+2; w_idx[wi++]=base+1; w_idx[wi++]=base+3;
        }
    }

    /* Vertical edges */
    for (int col = 0; col < PH_VERTS; col++) {
        for (int row = 0; row < PH_GRID; row++) {
            int x = 16384 + col * 32767 / PH_GRID;
            uint16_t y0 = (uint16_t)(16384 + row * 32767 / PH_GRID);
            uint16_t y1 = (uint16_t)(16384 + (row + 1) * 32767 / PH_GRID);
            uint16_t xl = (uint16_t)(x - PH_WIRE_HW);
            uint16_t xh = (uint16_t)(x + PH_WIRE_HW);
            uint32_t base = (uint32_t)wv;
            w_xy[wv*2]=xl; w_xy[wv*2+1]=y0; w_z[wv]=0; w_dist[wv]=(float)-PH_WIRE_HW; wv++;
            w_xy[wv*2]=xh; w_xy[wv*2+1]=y0; w_z[wv]=0; w_dist[wv]=(float)+PH_WIRE_HW; wv++;
            w_xy[wv*2]=xl; w_xy[wv*2+1]=y1; w_z[wv]=0; w_dist[wv]=(float)-PH_WIRE_HW; wv++;
            w_xy[wv*2]=xh; w_xy[wv*2+1]=y1; w_z[wv]=0; w_dist[wv]=(float)+PH_WIRE_HW; wv++;
            w_idx[wi++]=base; w_idx[wi++]=base+1; w_idx[wi++]=base+2;
            w_idx[wi++]=base+2; w_idx[wi++]=base+1; w_idx[wi++]=base+3;
        }
    }

    r->ph_wire_index_count = PH_WIRE_NI2;
    r->ph_wire_buf_xy = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                      w_xy, sizeof(w_xy));
    r->ph_wire_buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                     w_z, sizeof(w_z));
    r->ph_wire_buf_dist = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                        w_dist, sizeof(w_dist));
    r->ph_wire_indices = create_buffer(r->device, r->queue, WGPUBufferUsage_Index,
                                       w_idx, sizeof(w_idx));
#undef PH_WIRE_HW
#undef PH_EDGE_COUNT
#undef PH_WIRE_NV
#undef PH_WIRE_NI2

#undef PH_GRID
#undef PH_VERTS
#undef PH_NV
#undef PH_NI

    /* 1×1 placeholder texture */
    WGPUTextureDescriptor ph_td = {
        .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
        .size = {1, 1, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    r->ph_texture = wgpuDeviceCreateTexture(r->device, &ph_td);
    r->ph_texture_view = wgpuTextureCreateView(r->ph_texture, NULL);
    uint8_t ph_pixel[4] = {50, 60, 75, 255};
    WGPUImageCopyTexture ph_dst = {.texture = r->ph_texture};
    WGPUTextureDataLayout ph_layout = {.bytesPerRow = 4, .rowsPerImage = 1};
    WGPUExtent3D ph_extent = {1, 1, 1};
    wgpuQueueWriteTexture(r->queue, &ph_dst, ph_pixel, 4, &ph_layout,
                          &ph_extent);

    /* Pool of uniform buffers + bind groups */
    for (int i = 0; i < ARPT_MAX_PLACEHOLDERS; i++) {
        r->ph_uniform_bufs[i] =
            create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform, NULL,
                          sizeof(tile_uniforms_t));
        WGPUBindGroupEntry ph_entries[] = {
            {.binding = 0,
             .buffer = r->ph_uniform_bufs[i],
             .offset = 0,
             .size = sizeof(tile_uniforms_t)},
            {.binding = 1, .textureView = r->ph_texture_view},
            {.binding = 2, .sampler = r->surface_sampler},
        };
        r->ph_bind_groups[i] = wgpuDeviceCreateBindGroup(
            r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                                   .entryCount = 3,
                                                   .entries = ph_entries});
    }
}

void arpt__placeholder_draw(arpt_renderer *r, int slot, arpt_mat4 model,
                            const float bounds[4], float center_lon,
                            float center_lat) {
    if (slot < 0 || slot >= ARPT_MAX_PLACEHOLDERS) return;

    tile_uniforms_t u;
    memcpy(u.model, model.m, sizeof(u.model));
    memcpy(u.bounds, bounds, sizeof(u.bounds));
    u.center_lon = center_lon;
    u.center_lat = center_lat;
    u._pad0 = 0.0f;
    u._pad1 = 0.0f;
    wgpuQueueWriteBuffer(r->queue, r->ph_uniform_bufs[slot], 0, &u, sizeof(u));

    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, r->ph_bind_groups[slot], 0,
                                      NULL);
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 0, r->ph_buf_xy, 0,
                                         wgpuBufferGetSize(r->ph_buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 1, r->ph_buf_z, 0,
                                         wgpuBufferGetSize(r->ph_buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 2, r->ph_buf_normals, 0,
                                         wgpuBufferGetSize(r->ph_buf_normals));
    wgpuRenderPassEncoderSetIndexBuffer(r->pass, r->ph_buf_indices,
                                        WGPUIndexFormat_Uint32, 0,
                                        wgpuBufferGetSize(r->ph_buf_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, r->ph_index_count, 1, 0, 0, 0);

    /* Wireframe SDF overlay */
    wgpuRenderPassEncoderSetPipeline(r->pass, r->wireframe_pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0, NULL);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 1, r->ph_bind_groups[slot], 0,
                                      NULL);
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 0, r->ph_wire_buf_xy, 0,
                                         wgpuBufferGetSize(r->ph_wire_buf_xy));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 1, r->ph_wire_buf_z, 0,
                                         wgpuBufferGetSize(r->ph_wire_buf_z));
    wgpuRenderPassEncoderSetVertexBuffer(r->pass, 2, r->ph_wire_buf_dist, 0,
                                         wgpuBufferGetSize(r->ph_wire_buf_dist));
    wgpuRenderPassEncoderSetIndexBuffer(r->pass, r->ph_wire_indices,
                                        WGPUIndexFormat_Uint32, 0,
                                        wgpuBufferGetSize(r->ph_wire_indices));
    wgpuRenderPassEncoderDrawIndexed(r->pass, r->ph_wire_index_count, 1, 0, 0,
                                     0);
    wgpuRenderPassEncoderSetPipeline(r->pass, r->pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0, NULL);
}

void arpt__placeholder_cleanup(arpt_renderer *r) {
    for (int i = 0; i < ARPT_MAX_PLACEHOLDERS; i++) {
        if (r->ph_bind_groups[i]) wgpuBindGroupRelease(r->ph_bind_groups[i]);
        if (r->ph_uniform_bufs[i]) wgpuBufferRelease(r->ph_uniform_bufs[i]);
    }
    if (r->ph_buf_xy) wgpuBufferRelease(r->ph_buf_xy);
    if (r->ph_buf_z) wgpuBufferRelease(r->ph_buf_z);
    if (r->ph_buf_normals) wgpuBufferRelease(r->ph_buf_normals);
    if (r->ph_buf_indices) wgpuBufferRelease(r->ph_buf_indices);
    if (r->ph_wire_buf_xy) wgpuBufferRelease(r->ph_wire_buf_xy);
    if (r->ph_wire_buf_z) wgpuBufferRelease(r->ph_wire_buf_z);
    if (r->ph_wire_buf_dist) wgpuBufferRelease(r->ph_wire_buf_dist);
    if (r->ph_wire_indices) wgpuBufferRelease(r->ph_wire_indices);
    if (r->wireframe_pipeline) wgpuRenderPipelineRelease(r->wireframe_pipeline);
    if (r->ph_texture_view) wgpuTextureViewRelease(r->ph_texture_view);
    if (r->ph_texture) wgpuTextureRelease(r->ph_texture);
}
