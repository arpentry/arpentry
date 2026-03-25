#include "internal.h"

#include "sky.wgsl.h"

WGPURenderPipeline arpt__sky_create_pipeline(WGPUDevice device,
                                              WGPUTextureFormat format,
                                              WGPUBindGroupLayout sky_bgl) {
    WGPUShaderModule sm = create_shader(device, sky_wgsl);

    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 1,
                                                .bindGroupLayouts = &sky_bgl});

    WGPUColorTargetState ct = {.format = format,
                               .writeMask = WGPUColorWriteMask_All};
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    /* Depth: always pass, write 1.0 so terrain overwrites */
    WGPUDepthStencilState ds = {
        .format = ARPT_DEPTH_FORMAT,
        .depthWriteEnabled = true,
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
                   .bufferCount = 0,
                   .buffers = NULL},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = ARPT_MSAA_SAMPLES, .mask = ~0u},
    };
    WGPURenderPipeline pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);
    return pipeline;
}

void arpt__sky_draw(arpt_renderer *r) {
    wgpuRenderPassEncoderSetPipeline(r->pass, r->sky_pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->sky_bind_group, 0, NULL);
    wgpuRenderPassEncoderDraw(r->pass, 3, 1, 0, 0);

    restore_terrain_pipeline(r);
}
