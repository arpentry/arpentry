#include "internal.h"

#include "tree.wgsl.h"

#include <stdlib.h>

WGPURenderPipeline arpt__instance_create_pipeline(WGPUDevice device,
                                                   WGPUTextureFormat format,
                                                   WGPUBindGroupLayout global_bgl,
                                                   WGPUBindGroupLayout tile_bgl,
                                                   WGPUBindGroupLayout model_bgl) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = tree_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    WGPUBindGroupLayout bgls[] = {global_bgl, tile_bgl, model_bgl};
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 3,
                                                .bindGroupLayouts = bgls});

    WGPUVertexAttribute attr_model_pos = {
        .format = WGPUVertexFormat_Uint16x4,
        .offset = 0,
        .shaderLocation = 0,
    };

    WGPUVertexAttribute inst_attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2,
         .offset = 0,
         .shaderLocation = 1},
        {.format = WGPUVertexFormat_Sint32,
         .offset = 4,
         .shaderLocation = 2},
        {.format = WGPUVertexFormat_Float32,
         .offset = 8,
         .shaderLocation = 3},
    };

    WGPUVertexBufferLayout vbls[] = {
        {.arrayStride = 8,
         .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1,
         .attributes = &attr_model_pos},
        {.arrayStride = 12,
         .stepMode = WGPUVertexStepMode_Instance,
         .attributeCount = 3,
         .attributes = inst_attrs},
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
                   .bufferCount = 2,
                   .buffers = vbls},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_Back,
                      .frontFace = WGPUFrontFace_CCW},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = 1, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__instance_upload_model(arpt_renderer *r, int model_index,
                                 const arpt_model *model) {
    if (!r || !model || model->vertex_count == 0) return;
    if (model_index < 0 || model_index >= ARPT_MAX_MODELS) return;

    /* Pack uint16 x/y/z/w into uint16x4 (8 bytes per vertex) */
    size_t nv = model->vertex_count;
    uint16_t *padded = calloc(nv, 8);
    if (!padded) return;

    uint16_t min_x = model->x[0], max_x = model->x[0];
    uint16_t min_y = model->y[0], max_y = model->y[0];
    for (size_t i = 0; i < nv; i++) {
        padded[i * 4 + 0] = model->x[i];
        padded[i * 4 + 1] = model->y[i];
        padded[i * 4 + 2] = model->z[i];
        padded[i * 4 + 3] = model->w ? model->w[i] : 0;
        if (model->x[i] < min_x) min_x = model->x[i];
        if (model->x[i] > max_x) max_x = model->x[i];
        if (model->y[i] < min_y) min_y = model->y[i];
        if (model->y[i] > max_y) max_y = model->y[i];
    }

    r->models[model_index].buf_pos =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex, padded,
                      nv * 8);
    free(padded);

    r->models[model_index].buf_indices = create_buffer(
        r->device, r->queue, WGPUBufferUsage_Index, model->indices,
        model->index_count * sizeof(uint32_t));
    r->models[model_index].index_count = (uint32_t)model->index_count;
    r->models[model_index].min_scale = model->min_scale;
    r->models[model_index].max_scale = model->max_scale;

    model_uniforms_t mu = {
        .center = {(float)(min_x + max_x) * 0.5f * 0.001f,
                   (float)(min_y + max_y) * 0.5f * 0.001f,
                   0.0f},
        .crown_color = {model->crown_color[0], model->crown_color[1],
                        model->crown_color[2]},
        .random_yaw = model->random_yaw ? 1 : 0,
        .min_scale = model->min_scale,
        .max_scale = model->max_scale,
        .random_scale = model->random_scale ? 1 : 0,
        .trunk_color = {model->trunk_color[0], model->trunk_color[1],
                        model->trunk_color[2]},
    };
    r->models[model_index].uniform_buf =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform, &mu,
                      sizeof(mu));

    WGPUBindGroupEntry bg_entry = {
        .binding = 0,
        .buffer = r->models[model_index].uniform_buf,
        .size = sizeof(mu),
    };
    r->models[model_index].bind_group = wgpuDeviceCreateBindGroup(
        r->device,
        &(WGPUBindGroupDescriptor){.layout = r->model_bgl,
                                   .entryCount = 1,
                                   .entries = &bg_entry});

    if (model_index >= r->model_count)
        r->model_count = model_index + 1;
}

void arpt__instance_upload(arpt_renderer *r, arpt_tile_gpu *t,
                           const arpt_instance_prim *prim) {
    if (!prim || prim->batch_count == 0) return;
    if (r->model_count == 0) return;

    for (int bi = 0; bi < prim->batch_count; bi++) {
        const arpt_instance_batch *batch = &prim->batches[bi];
        int mi = batch->model_index;
        if (mi < 0 || mi >= r->model_count || !r->models[mi].buf_pos)
            continue;
        if (batch->count == 0) continue;

        t->tree_instance_bufs[mi] =
            create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                          batch->instances,
                          batch->count * sizeof(arpt_instance_pt));
        t->tree_instance_counts[mi] = (uint32_t)batch->count;
    }
}

void arpt__instance_draw(arpt_renderer *r, arpt_tile_gpu *tile) {
    bool drew_trees = false;
    for (int mi = 0; mi < r->model_count; mi++) {
        if (tile->tree_instance_counts[mi] == 0 || !r->models[mi].buf_pos)
            continue;
        if (!drew_trees) {
            wgpuRenderPassEncoderSetPipeline(r->pass, r->tree_pipeline);
            wgpuRenderPassEncoderSetBindGroup(r->pass, 0,
                                              r->global_bind_group, 0, NULL);
            wgpuRenderPassEncoderSetBindGroup(r->pass, 1, tile->bind_group, 0,
                                              NULL);
            drew_trees = true;
        }
        wgpuRenderPassEncoderSetBindGroup(r->pass, 2,
                                          r->models[mi].bind_group, 0, NULL);
        wgpuRenderPassEncoderSetVertexBuffer(
            r->pass, 0, r->models[mi].buf_pos, 0,
            wgpuBufferGetSize(r->models[mi].buf_pos));
        wgpuRenderPassEncoderSetVertexBuffer(
            r->pass, 1, tile->tree_instance_bufs[mi], 0,
            wgpuBufferGetSize(tile->tree_instance_bufs[mi]));
        wgpuRenderPassEncoderSetIndexBuffer(
            r->pass, r->models[mi].buf_indices, WGPUIndexFormat_Uint32, 0,
            wgpuBufferGetSize(r->models[mi].buf_indices));
        wgpuRenderPassEncoderDrawIndexed(r->pass, r->models[mi].index_count,
                                         tile->tree_instance_counts[mi], 0, 0,
                                         0);
    }
    if (drew_trees) {
        wgpuRenderPassEncoderSetPipeline(r->pass, r->pipeline);
        wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0,
                                          NULL);
    }
}

void arpt__instance_cleanup(arpt_renderer *r) {
    for (int mi = 0; mi < ARPT_MAX_MODELS; mi++) {
        if (r->models[mi].buf_pos) wgpuBufferRelease(r->models[mi].buf_pos);
        if (r->models[mi].buf_indices)
            wgpuBufferRelease(r->models[mi].buf_indices);
        if (r->models[mi].bind_group)
            wgpuBindGroupRelease(r->models[mi].bind_group);
        if (r->models[mi].uniform_buf)
            wgpuBufferRelease(r->models[mi].uniform_buf);
    }
    if (r->model_bgl) wgpuBindGroupLayoutRelease(r->model_bgl);
}
