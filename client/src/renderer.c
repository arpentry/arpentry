#include "renderer.h"
#include <stdlib.h>
#include <string.h>

/* ── WGSL Shader ───────────────────────────────────────────────────────── */

static const char *terrain_wgsl =
    "const WGS84_A: f32 = 6378137.0;\n"
    "const WGS84_E2: f32 = 0.00669437999014;\n"
    "\n"
    "struct GlobalUniforms {\n"
    "    projection: mat4x4<f32>,\n"
    "    sun_dir: vec3<f32>,\n"
    "};\n"
    "\n"
    "struct TileUniforms {\n"
    "    model: mat4x4<f32>,\n"
    "    bounds: vec4<f32>,\n"
    "    center_lon: f32,\n"
    "    center_lat: f32,\n"
    "    _pad0: f32,\n"
    "    _pad1: f32,\n"
    "};\n"
    "\n"
    "@group(0) @binding(0) var<uniform> globals: GlobalUniforms;\n"
    "@group(1) @binding(0) var<uniform> tile: TileUniforms;\n"
    "\n"
    "struct VsOut {\n"
    "    @builtin(position) pos: vec4<f32>,\n"
    "    @location(0) altitude: f32,\n"
    "    @location(1) normal_cam: vec3<f32>,\n"
    "};\n"
    "\n"
    "fn geodetic_to_ecef(lon: f32, lat: f32, alt: f32) -> vec3<f32> {\n"
    "    let sin_lat = sin(lat);\n"
    "    let cos_lat = cos(lat);\n"
    "    let sin_lon = sin(lon);\n"
    "    let cos_lon = cos(lon);\n"
    "    let N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);\n"
    "    return vec3<f32>(\n"
    "        (N + alt) * cos_lat * cos_lon,\n"
    "        (N + alt) * cos_lat * sin_lon,\n"
    "        (N * (1.0 - WGS84_E2) + alt) * sin_lat,\n"
    "    );\n"
    "}\n"
    "\n"
    "fn decode_octahedral(enc: vec2<f32>) -> vec3<f32> {\n"
    "    var n = vec3<f32>(enc.x, enc.y, 1.0 - abs(enc.x) - abs(enc.y));\n"
    "    if (n.z < 0.0) {\n"
    "        let old = n.xy;\n"
    "        n.x = (1.0 - abs(old.y)) * sign(old.x);\n"
    "        n.y = (1.0 - abs(old.x)) * sign(old.y);\n"
    "    }\n"
    "    return normalize(n);\n"
    "}\n"
    "\n"
    "@vertex fn vs(\n"
    "    @location(0) qxy: vec2<u32>,\n"     /* Uint16x2: x, y packed */
    "    @location(1) qz: i32,\n"            /* Sint32 */
    "    @location(2) oct_norm: vec2<i32>,\n" /* Sint8x2 */
    ") -> VsOut {\n"
    "    let lon_west = tile.bounds.x;\n"
    "    let lat_south = tile.bounds.y;\n"
    "    let lon_east = tile.bounds.z;\n"
    "    let lat_north = tile.bounds.w;\n"
    "\n"
    "    let lon = lon_west + ((f32(qxy.x) - 16384.0) / 32768.0) * (lon_east - lon_west);\n"
    "    let lat = lat_south + ((f32(qxy.y) - 16384.0) / 32768.0) * (lat_north - lat_south);\n"
    "    let alt = f32(qz) * 0.001;\n"
    "\n"
    "    let ecef = geodetic_to_ecef(lon, lat, alt);\n"
    "    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);\n"
    "    let local_ecef = ecef - center_ecef;\n"
    "\n"
    "    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);\n"
    "\n"
    "    var out: VsOut;\n"
    "    out.pos = globals.projection * world_pos;\n"
    "    out.altitude = alt;\n"
    "\n"
    "    let enc = vec2<f32>(f32(oct_norm.x) / 127.0, f32(oct_norm.y) / 127.0);\n"
    "    let obj_normal = decode_octahedral(enc);\n"
    "    let model3 = mat3x3<f32>(tile.model[0].xyz, tile.model[1].xyz, tile.model[2].xyz);\n"
    "    out.normal_cam = normalize(model3 * obj_normal);\n"
    "\n"
    "    return out;\n"
    "}\n"
    "\n"
    "@fragment fn fs(\n"
    "    @location(0) altitude: f32,\n"
    "    @location(1) normal_cam: vec3<f32>,\n"
    ") -> @location(0) vec4<f32> {\n"
    "    let t = clamp((altitude + 500.0) / 5000.0, 0.0, 1.0);\n"
    "    let low  = vec3<f32>(0.18, 0.32, 0.15);\n"
    "    let mid  = vec3<f32>(0.55, 0.45, 0.30);\n"
    "    let high = vec3<f32>(0.90, 0.90, 0.92);\n"
    "    var color: vec3<f32>;\n"
    "    if (t < 0.5) {\n"
    "        color = mix(low, mid, t * 2.0);\n"
    "    } else {\n"
    "        color = mix(mid, high, (t - 0.5) * 2.0);\n"
    "    }\n"
    "\n"
    "    let n = normalize(normal_cam);\n"
    "    let diffuse = max(dot(n, normalize(globals.sun_dir)), 0.0);\n"
    "    let lit = color * (0.15 + 0.85 * diffuse);\n"
    "\n"
    "    return vec4<f32>(lit, 1.0);\n"
    "}\n";

/* ── Uniform layouts ───────────────────────────────────────────────────── */

typedef struct {
    float projection[16];
    float sun_dir[3];
    float _pad;
} global_uniforms_t;

typedef struct {
    float model[16];
    float bounds[4];
    float center_lon;
    float center_lat;
    float _pad0;
    float _pad1;
} tile_uniforms_t;

/* ── Tile GPU state ────────────────────────────────────────────────────── */

struct arpt_tile_gpu {
    WGPUBuffer buf_xy;      /* interleaved uint16 x,y pairs */
    WGPUBuffer buf_z;       /* int32 elevation */
    WGPUBuffer buf_normals; /* int8x2 octahedral normals */
    WGPUBuffer buf_indices; /* uint32 triangle indices */
    WGPUBuffer uniform_buf;
    WGPUBindGroup bind_group;
    uint32_t index_count;
    arpt_renderer *renderer;
};

/* ── Renderer state ────────────────────────────────────────────────────── */

struct arpt_renderer {
    WGPUDevice device;
    WGPUQueue queue;
    WGPUTextureFormat surface_format;
    uint32_t width, height;

    WGPURenderPipeline pipeline;
    WGPUBindGroupLayout global_bgl;
    WGPUBindGroupLayout tile_bgl;

    WGPUBuffer global_uniform_buf;
    WGPUBindGroup global_bind_group;

    WGPUTexture depth_texture;
    WGPUTextureView depth_view;

    WGPUCommandEncoder encoder;
    WGPURenderPassEncoder pass;
};

/* ── Helpers ───────────────────────────────────────────────────────────── */

static WGPUBuffer create_buffer(WGPUDevice device, WGPUQueue queue,
                                 WGPUBufferUsageFlags usage,
                                 const void *data, size_t size) {
    /* Align buffer size up to 4 bytes (COPY_BUFFER_ALIGNMENT) */
    size_t aligned = (size + 3) & ~(size_t)3;
    WGPUBufferDescriptor desc = {
        .usage = usage | WGPUBufferUsage_CopyDst,
        .size = aligned,
    };
    WGPUBuffer buf = wgpuDeviceCreateBuffer(device, &desc);
    if (data)
        wgpuQueueWriteBuffer(queue, buf, 0, data, size);
    return buf;
}

static void create_depth_texture(arpt_renderer *r) {
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);

    WGPUTextureDescriptor desc = {
        .usage = WGPUTextureUsage_RenderAttachment,
        .size = {r->width, r->height, 1},
        .format = WGPUTextureFormat_Depth24Plus,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    r->depth_texture = wgpuDeviceCreateTexture(r->device, &desc);
    r->depth_view = wgpuTextureCreateView(r->depth_texture, NULL);
}

/* ── Pipeline creation ─────────────────────────────────────────────────── */

static WGPURenderPipeline create_pipeline(WGPUDevice device,
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
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(device,
        &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 2,
                                         .bindGroupLayouts = bgls});

    WGPUVertexAttribute attr_xy = {
        .format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0};
    WGPUVertexAttribute attr_z = {
        .format = WGPUVertexFormat_Sint32, .offset = 0, .shaderLocation = 1};
    WGPUVertexAttribute attr_n = {
        .format = WGPUVertexFormat_Sint8x2, .offset = 0, .shaderLocation = 2};
    WGPUVertexBufferLayout vbls[] = {
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_xy},
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_z},
        {.arrayStride = 4, .stepMode = WGPUVertexStepMode_Vertex,
         .attributeCount = 1, .attributes = &attr_n},
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
        .stencilBack  = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask  = 0,
        .stencilWriteMask = 0,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = "vs",
                   .bufferCount = 3, .buffers = vbls},
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

/* ── Renderer lifecycle ────────────────────────────────────────────────── */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                     WGPUTextureFormat format,
                                     uint32_t width, uint32_t height) {
    arpt_renderer *r = calloc(1, sizeof(*r));
    if (!r) return NULL;

    r->device = device;
    r->queue = queue;
    r->surface_format = format;
    r->width = width;
    r->height = height;

    /* Bind group layouts */
    WGPUBindGroupLayoutEntry global_entry = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(global_uniforms_t)},
    };
    r->global_bgl = wgpuDeviceCreateBindGroupLayout(device,
        &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                          .entries = &global_entry});

    WGPUBindGroupLayoutEntry tile_entry = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(tile_uniforms_t)},
    };
    r->tile_bgl = wgpuDeviceCreateBindGroupLayout(device,
        &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                          .entries = &tile_entry});

    r->pipeline = create_pipeline(device, format, r->global_bgl, r->tile_bgl);

    /* Global uniform buffer + bind group */
    r->global_uniform_buf = create_buffer(device, queue, WGPUBufferUsage_Uniform,
                                           NULL, sizeof(global_uniforms_t));
    WGPUBindGroupEntry bg_e = {
        .binding = 0, .buffer = r->global_uniform_buf,
        .offset = 0, .size = sizeof(global_uniforms_t),
    };
    r->global_bind_group = wgpuDeviceCreateBindGroup(device,
        &(WGPUBindGroupDescriptor){.layout = r->global_bgl,
                                    .entryCount = 1, .entries = &bg_e});

    create_depth_texture(r);
    return r;
}

void arpt_renderer_free(arpt_renderer *r) {
    if (!r) return;
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);
    if (r->global_bind_group) wgpuBindGroupRelease(r->global_bind_group);
    if (r->global_uniform_buf) wgpuBufferRelease(r->global_uniform_buf);
    if (r->pipeline) wgpuRenderPipelineRelease(r->pipeline);
    if (r->global_bgl) wgpuBindGroupLayoutRelease(r->global_bgl);
    if (r->tile_bgl) wgpuBindGroupLayoutRelease(r->tile_bgl);
    free(r);
}

void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height) {
    r->width = width;
    r->height = height;
    create_depth_texture(r);
}

/* ── Tile GPU ──────────────────────────────────────────────────────────── */

arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                          const arpt_terrain_mesh *mesh) {
    arpt_tile_gpu *t = calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->renderer = r;
    t->index_count = (uint32_t)mesh->index_count;

    /* Interleave x,y into uint16 pairs for Uint16x2 vertex format */
    size_t vc = mesh->vertex_count;
    uint16_t *xy = malloc(vc * 4);  /* 2 × uint16 per vertex */
    if (!xy) { free(t); return NULL; }
    for (size_t i = 0; i < vc; i++) {
        xy[i * 2]     = mesh->x[i];
        xy[i * 2 + 1] = mesh->y[i];
    }
    t->buf_xy = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                               xy, vc * 4);
    free(xy);

    t->buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                              mesh->z, vc * sizeof(int32_t));

    /* Pad normals to 4-byte stride (Sint8x2 + 2 padding bytes) */
    {
        int8_t *padded = calloc(vc, 4);
        if (!padded) { free(t); return NULL; }
        for (size_t i = 0; i < vc; i++) {
            if (mesh->normals) {
                padded[i * 4]     = mesh->normals[i * 2];
                padded[i * 4 + 1] = mesh->normals[i * 2 + 1];
            }
            /* bytes [2] and [3] stay zero (padding) */
        }
        t->buf_normals = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                        padded, vc * 4);
        free(padded);
    }

    t->buf_indices = create_buffer(r->device, r->queue, WGPUBufferUsage_Index,
                                    mesh->indices, mesh->index_count * sizeof(uint32_t));

    /* Per-tile uniform buffer + bind group */
    t->uniform_buf = create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform,
                                    NULL, sizeof(tile_uniforms_t));
    WGPUBindGroupEntry entry = {
        .binding = 0, .buffer = t->uniform_buf,
        .offset = 0, .size = sizeof(tile_uniforms_t),
    };
    t->bind_group = wgpuDeviceCreateBindGroup(r->device,
        &(WGPUBindGroupDescriptor){.layout = r->tile_bgl, .entryCount = 1, .entries = &entry});

    return t;
}

void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile,
                                 arpt_mat4 model,
                                 const float bounds[4],
                                 float center_lon, float center_lat) {
    tile_uniforms_t u;
    memcpy(u.model, model.m, sizeof(u.model));
    memcpy(u.bounds, bounds, sizeof(u.bounds));
    u.center_lon = center_lon;
    u.center_lat = center_lat;
    u._pad0 = 0.0f;
    u._pad1 = 0.0f;
    wgpuQueueWriteBuffer(tile->renderer->queue, tile->uniform_buf, 0, &u, sizeof(u));
}

void arpt_tile_gpu_free(arpt_tile_gpu *tile) {
    if (!tile) return;
    if (tile->buf_xy) wgpuBufferRelease(tile->buf_xy);
    if (tile->buf_z) wgpuBufferRelease(tile->buf_z);
    if (tile->buf_normals) wgpuBufferRelease(tile->buf_normals);
    if (tile->buf_indices) wgpuBufferRelease(tile->buf_indices);
    if (tile->uniform_buf) wgpuBufferRelease(tile->uniform_buf);
    if (tile->bind_group) wgpuBindGroupRelease(tile->bind_group);
    free(tile);
}

/* ── Frame rendering ───────────────────────────────────────────────────── */

void arpt_renderer_set_globals(arpt_renderer *r,
                                arpt_mat4 projection, arpt_vec3 sun_dir) {
    global_uniforms_t u;
    memcpy(u.projection, projection.m, sizeof(u.projection));
    u.sun_dir[0] = sun_dir.x;
    u.sun_dir[1] = sun_dir.y;
    u.sun_dir[2] = sun_dir.z;
    u._pad = 0.0f;
    wgpuQueueWriteBuffer(r->queue, r->global_uniform_buf, 0, &u, sizeof(u));
}

void arpt_renderer_begin_frame(arpt_renderer *r, WGPUTextureView target_view) {
    r->encoder = wgpuDeviceCreateCommandEncoder(r->device, NULL);

    WGPURenderPassColorAttachment color = {
        .view = target_view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Store,
        .clearValue = {0.05, 0.05, 0.08, 1.0},
#ifdef __EMSCRIPTEN__
        .depthSlice = WGPU_DEPTH_SLICE_UNDEFINED,
#endif
    };
    WGPURenderPassDepthStencilAttachment depth = {
        .view = r->depth_view,
        .depthLoadOp = WGPULoadOp_Clear,
        .depthStoreOp = WGPUStoreOp_Store,
        .depthClearValue = 1.0f,
    };
    WGPURenderPassDescriptor rp = {
        .colorAttachmentCount = 1,
        .colorAttachments = &color,
        .depthStencilAttachment = &depth,
    };

    r->pass = wgpuCommandEncoderBeginRenderPass(r->encoder, &rp);
    wgpuRenderPassEncoderSetPipeline(r->pass, r->pipeline);
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0, NULL);
}

void arpt_renderer_draw_tile(arpt_renderer *r, arpt_tile_gpu *tile) {
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

void arpt_renderer_end_frame(arpt_renderer *r) {
    wgpuRenderPassEncoderEnd(r->pass);
    wgpuRenderPassEncoderRelease(r->pass);
    r->pass = NULL;

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(r->encoder, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(r->encoder);
    r->encoder = NULL;
}
