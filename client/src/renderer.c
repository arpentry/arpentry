#include "renderer.h"
#include <stdlib.h>
#include <string.h>

/* WGSL Shader */

static const char *terrain_wgsl =
    "const WGS84_A: f32 = 6378137.0;\n"
    "const WGS84_E2: f32 = 0.00669437999014;\n"
    "\n"
    "struct GlobalUniforms {\n"
    "    projection: mat4x4<f32>,\n"
    "    sun_dir: vec3<f32>,\n"
    "    apply_gamma: f32,\n"
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
    "@group(1) @binding(1) var landuse_tex: texture_2d<f32>;\n"
    "@group(1) @binding(2) var landuse_samp: sampler;\n"
    "\n"
    "struct VsOut {\n"
    "    @builtin(position) pos: vec4<f32>,\n"
    "    @location(0) uv: vec2<f32>,\n"
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
    "    @location(0) qxy: vec2<u32>,\n"      /* Uint16x2: x, y packed */
    "    @location(1) qz: i32,\n"             /* Sint32 */
    "    @location(2) oct_norm: vec2<i32>,\n" /* Sint8x2 */
    ") -> VsOut {\n"
    "    let lon_west = tile.bounds.x;\n"
    "    let lat_south = tile.bounds.y;\n"
    "    let lon_east = tile.bounds.z;\n"
    "    let lat_north = tile.bounds.w;\n"
    "\n"
    "    let u = (f32(qxy.x) - 16384.0) / 32768.0;\n"
    "    let v = (f32(qxy.y) - 16384.0) / 32768.0;\n"
    "    let lon = lon_west + u * (lon_east - lon_west);\n"
    "    let lat = lat_south + v * (lat_north - lat_south);\n"
    "    let alt = f32(qz) * 0.001;\n"
    "\n"
    "    let ecef = geodetic_to_ecef(lon, lat, alt);\n"
    "    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, "
    "0.0);\n"
    "    let local_ecef = ecef - center_ecef;\n"
    "\n"
    "    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);\n"
    "\n"
    "    var out: VsOut;\n"
    "    out.pos = globals.projection * world_pos;\n"
    "    out.uv = vec2<f32>(u, v);\n"
    "\n"
    "    let enc = vec2<f32>(f32(oct_norm.x) / 127.0, f32(oct_norm.y) / "
    "127.0);\n"
    "    let obj_normal = decode_octahedral(enc);\n"
    "    let model3 = mat3x3<f32>(tile.model[0].xyz, tile.model[1].xyz, "
    "tile.model[2].xyz);\n"
    "    out.normal_cam = normalize(model3 * obj_normal);\n"
    "\n"
    "    return out;\n"
    "}\n"
    "\n"
    "@fragment fn fs(\n"
    "    @location(0) uv: vec2<f32>,\n"
    "    @location(1) normal_cam: vec3<f32>,\n"
    ") -> @location(0) vec4<f32> {\n"
    "    let margin = 0.125;\n"
    "    let tex_uv = (uv + vec2<f32>(margin, margin)) / (1.0 + 2.0 * "
    "margin);\n"
    "    let albedo = textureSample(landuse_tex, landuse_samp, tex_uv).rgb;\n"
    "\n"
    "    let n = normalize(normal_cam);\n"
    "    let sun = normalize(globals.sun_dir);\n"
    "    let NdotL = dot(n, sun);\n"
    "\n"
    "    // Hemisphere ambient: cool blue in shadow, warm fill on lit side\n"
    "    let shadow_color = vec3<f32>(0.20, 0.22, 0.28);\n"
    "    let fill_color   = vec3<f32>(0.28, 0.26, 0.22);\n"
    "    let hemi_t = NdotL * 0.5 + 0.5;\n"
    "    let ambient = mix(shadow_color, fill_color, hemi_t);\n"
    "\n"
    "    // Direct sunlight: clamped Lambertian at moderate intensity\n"
    "    let sun_color = vec3<f32>(0.65, 0.63, 0.58);\n"
    "    let direct = sun_color * max(NdotL, 0.0);\n"
    "\n"
    "    let lit = albedo * (ambient + direct);\n"
    "\n"
    "    // Apply sRGB gamma when surface format is non-sRGB (e.g. WebGPU in "
    "browser)\n"
    "    let out = select(lit, pow(lit, vec3<f32>(1.0 / 2.2)), "
    "globals.apply_gamma > 0.5);\n"
    "    return vec4<f32>(out, 1.0);\n"
    "}\n";

/* Landuse rasterization shader */

static const char *landuse_wgsl =
    "struct VsOut {\n"
    "    @builtin(position) pos: vec4<f32>,\n"
    "    @location(0) color: vec4<f32>,\n"
    "};\n"
    "\n"
    "@vertex fn vs(\n"
    "    @location(0) qxy: vec2<u32>,\n"
    "    @location(1) color: vec4<f32>,\n"
    ") -> VsOut {\n"
    "    let u = (f32(qxy.x) - 16384.0) / 32768.0;\n"
    "    let v = (f32(qxy.y) - 16384.0) / 32768.0;\n"
    "    var out: VsOut;\n"
    "    let margin = 0.125;\n"
    "    let scale = 1.0 / (1.0 + 2.0 * margin);\n"
    "    out.pos = vec4<f32>((u + margin) * scale * 2.0 - 1.0,\n"
    "                        1.0 - (v + margin) * scale * 2.0, 0.0, 1.0);\n"
    "    out.color = color;\n"
    "    return out;\n"
    "}\n"
    "\n"
    "@fragment fn fs(\n"
    "    @location(0) color: vec4<f32>,\n"
    ") -> @location(0) vec4<f32> {\n"
    "    return color;\n"
    "}\n";

/* Landuse color table */

#define LANDUSE_TEX_SIZE 1024
#define LANDUSE_MARGIN 0.125 /* = LANDUSE_BUFFER / LANDUSE_GRID = 8/64 */

typedef struct {
    float r, g, b, a;
} landuse_color_t;

static const landuse_color_t landuse_colors[] = {
    [ARPT_LANDUSE_UNKNOWN] = {0.35f, 0.52f, 0.22f, 1.0f}, /* default: grass */
    [ARPT_LANDUSE_GRASS] = {0.35f, 0.52f, 0.22f, 1.0f},
    [ARPT_LANDUSE_FOREST] = {0.15f, 0.35f, 0.12f, 1.0f},
    [ARPT_LANDUSE_SAND] = {0.72f, 0.65f, 0.52f, 1.0f},
};

/* Uniform layouts */

typedef struct {
    float projection[16];
    float sun_dir[3];
    float apply_gamma; /* 1.0 when surface is non-sRGB (needs manual correction)
                        */
} global_uniforms_t;

typedef struct {
    float model[16];
    float bounds[4];
    float center_lon;
    float center_lat;
    float _pad0;
    float _pad1;
} tile_uniforms_t;

/* Tile GPU state */

struct arpt_tile_gpu {
    WGPUBuffer buf_xy;      /* interleaved uint16 x,y pairs */
    WGPUBuffer buf_z;       /* int32 elevation */
    WGPUBuffer buf_normals; /* int8x2 octahedral normals */
    WGPUBuffer buf_indices; /* uint32 triangle indices */
    WGPUBuffer uniform_buf;
    WGPUBindGroup bind_group;
    WGPUTexture landuse_texture;
    WGPUTextureView landuse_view;
    uint32_t index_count;
    arpt_renderer *renderer;
};

/* Renderer state */

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
    global_uniforms_t prev_globals; /* skip upload when unchanged */

    WGPUTexture depth_texture;
    WGPUTextureView depth_view;

    /* Landuse offscreen rasterization */
    WGPURenderPipeline landuse_pipeline;
    WGPUSampler landuse_sampler;
    WGPUTexture default_landuse_tex;
    WGPUTextureView default_landuse_view;

    /* Placeholder flat quads for tiles still loading */
    WGPUBuffer ph_buf_xy;
    WGPUBuffer ph_buf_z;
    WGPUBuffer ph_buf_normals;
    WGPUBuffer ph_buf_indices;
    uint32_t ph_index_count;
    WGPUTexture ph_texture;
    WGPUTextureView ph_texture_view;
    WGPUBuffer ph_uniform_bufs[ARPT_MAX_PLACEHOLDERS];
    WGPUBindGroup ph_bind_groups[ARPT_MAX_PLACEHOLDERS];

    WGPUCommandEncoder encoder;
    WGPURenderPassEncoder pass;

    /* Overlay callback (e.g. UI) invoked before pass ends */
    arpt_overlay_fn overlay_fn;
    void *overlay_ud;
};

/* Helpers */

static WGPUBuffer create_buffer(WGPUDevice device, WGPUQueue queue,
                                WGPUBufferUsageFlags usage, const void *data,
                                size_t size) {
    /* Align buffer size up to 4 bytes (COPY_BUFFER_ALIGNMENT) */
    size_t aligned = (size + 3) & ~(size_t)3;
    WGPUBufferDescriptor desc = {
        .usage = usage | WGPUBufferUsage_CopyDst,
        .size = aligned,
    };
    WGPUBuffer buf = wgpuDeviceCreateBuffer(device, &desc);
    if (data) wgpuQueueWriteBuffer(queue, buf, 0, data, size);
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

/* Pipeline creation */

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

/* Landuse pipeline creation */

static WGPURenderPipeline create_landuse_pipeline(WGPUDevice device) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = landuse_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    /* No bind groups needed for landuse rasterization */
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(
        device, &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 0,
                                                .bindGroupLayouts = NULL});

    WGPUVertexAttribute landuse_attrs[] = {
        {.format = WGPUVertexFormat_Uint16x2, .offset = 0, .shaderLocation = 0},
        {.format = WGPUVertexFormat_Float32x4,
         .offset = 4,
         .shaderLocation = 1},
    };
    WGPUVertexBufferLayout vbl = {
        .arrayStride = 20, /* 4 bytes xy + 16 bytes color */
        .stepMode = WGPUVertexStepMode_Vertex,
        .attributeCount = 2,
        .attributes = landuse_attrs,
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

/* Landuse rasterization (offscreen, once per tile) */

typedef struct {
    uint16_t x, y; /* quantized position */
    float r, g, b, a;
} landuse_vertex_t;

static WGPUTexture rasterize_landuse(arpt_renderer *r,
                                     const arpt_landuse_data *landuse) {
    /* Create 256x256 RGBA8 offscreen texture */
    WGPUTextureDescriptor tex_desc = {
        .usage =
            WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding,
        .size = {LANDUSE_TEX_SIZE, LANDUSE_TEX_SIZE, 1},
        .format = WGPUTextureFormat_RGBA8Unorm,
        .dimension = WGPUTextureDimension_2D,
        .mipLevelCount = 1,
        .sampleCount = 1,
    };
    WGPUTexture tex = wgpuDeviceCreateTexture(r->device, &tex_desc);

    /* Build vertex + index buffers for all polygons as triangle fans */
    size_t total_verts = 0;
    size_t total_indices = 0;
    for (size_t i = 0; i < landuse->count; i++) {
        size_t vc = landuse->polygons[i].vertex_count;
        if (vc < 3) continue;
        total_verts += vc;
        total_indices += (vc - 2) * 3; /* triangle fan */
    }

    if (total_verts == 0 || total_indices == 0) {
        /* No polygons — render a clear pass with default color */
        WGPUTextureView view = wgpuTextureCreateView(tex, NULL);
        WGPUCommandEncoder enc =
            wgpuDeviceCreateCommandEncoder(r->device, NULL);
        WGPURenderPassColorAttachment color = {
            .view = view,
            .loadOp = WGPULoadOp_Clear,
            .storeOp = WGPUStoreOp_Store,
            .clearValue = {0.35, 0.52, 0.22, 1.0},
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

    landuse_vertex_t *verts = malloc(total_verts * sizeof(landuse_vertex_t));
    uint32_t *idxs = malloc(total_indices * sizeof(uint32_t));
    if (!verts || !idxs) {
        free(verts);
        free(idxs);
        return tex;
    }

    size_t vi = 0, ii = 0;
    for (size_t i = 0; i < landuse->count; i++) {
        const arpt_landuse_polygon *p = &landuse->polygons[i];
        if (p->vertex_count < 3) continue;

        landuse_color_t c = landuse_colors[p->cls];
        uint32_t base = (uint32_t)vi;

        for (size_t v = 0; v < p->vertex_count; v++) {
            verts[vi].x = p->x[v];
            verts[vi].y = p->y[v];
            verts[vi].r = c.r;
            verts[vi].g = c.g;
            verts[vi].b = c.b;
            verts[vi].a = c.a;
            vi++;
        }

        /* Triangle fan: (0, 1, 2), (0, 2, 3), ... */
        for (size_t v = 1; v + 1 < p->vertex_count; v++) {
            idxs[ii++] = base;
            idxs[ii++] = base + (uint32_t)v;
            idxs[ii++] = base + (uint32_t)v + 1;
        }
    }

    WGPUBuffer vbuf = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                    verts, vi * sizeof(landuse_vertex_t));
    WGPUBuffer ibuf = create_buffer(r->device, r->queue, WGPUBufferUsage_Index,
                                    idxs, ii * sizeof(uint32_t));
    free(verts);
    free(idxs);

    /* Render pass */
    WGPUTextureView view = wgpuTextureCreateView(tex, NULL);
    WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(r->device, NULL);
    WGPURenderPassColorAttachment color = {
        .view = view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Store,
        .clearValue = {0.35, 0.52, 0.22, 1.0}, /* default: grass */
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
    wgpuRenderPassEncoderSetPipeline(pass, r->landuse_pipeline);
    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, vbuf, 0,
                                         vi * sizeof(landuse_vertex_t));
    wgpuRenderPassEncoderSetIndexBuffer(pass, ibuf, WGPUIndexFormat_Uint32, 0,
                                        ii * sizeof(uint32_t));
    wgpuRenderPassEncoderDrawIndexed(pass, (uint32_t)ii, 1, 0, 0, 0);
    wgpuRenderPassEncoderEnd(pass);
    wgpuRenderPassEncoderRelease(pass);

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(enc, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(enc);
    wgpuTextureViewRelease(view);
    wgpuBufferRelease(vbuf);
    wgpuBufferRelease(ibuf);

    return tex;
}

/* Renderer lifecycle */

arpt_renderer *arpt_renderer_create(WGPUDevice device, WGPUQueue queue,
                                    WGPUTextureFormat format, uint32_t width,
                                    uint32_t height) {
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
    r->global_bgl = wgpuDeviceCreateBindGroupLayout(
        device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 1,
                                                 .entries = &global_entry});

    WGPUBindGroupLayoutEntry tile_entries[] = {
        {
            .binding = 0,
            .visibility = WGPUShaderStage_Vertex,
            .buffer = {.type = WGPUBufferBindingType_Uniform,
                       .minBindingSize = sizeof(tile_uniforms_t)},
        },
        {
            .binding = 1,
            .visibility = WGPUShaderStage_Fragment,
            .texture = {.sampleType = WGPUTextureSampleType_Float,
                        .viewDimension = WGPUTextureViewDimension_2D},
        },
        {
            .binding = 2,
            .visibility = WGPUShaderStage_Fragment,
            .sampler = {.type = WGPUSamplerBindingType_Filtering},
        },
    };
    r->tile_bgl = wgpuDeviceCreateBindGroupLayout(
        device, &(WGPUBindGroupLayoutDescriptor){.entryCount = 3,
                                                 .entries = tile_entries});

    r->pipeline = create_pipeline(device, format, r->global_bgl, r->tile_bgl);

    /* Landuse offscreen pipeline + sampler */
    r->landuse_pipeline = create_landuse_pipeline(device);
    WGPUSamplerDescriptor samp_desc = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
        .lodMaxClamp = 32.0f,
    };
    r->landuse_sampler = wgpuDeviceCreateSampler(device, &samp_desc);

    /* Default 1x1 grass-colored landuse texture for tiles without landuse */
    {
        WGPUTextureDescriptor dt = {
            .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
            .size = {1, 1, 1},
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureDimension_2D,
            .mipLevelCount = 1,
            .sampleCount = 1,
        };
        r->default_landuse_tex = wgpuDeviceCreateTexture(device, &dt);
        r->default_landuse_view =
            wgpuTextureCreateView(r->default_landuse_tex, NULL);
        uint8_t pixel[4] = {89, 133, 56, 255}; /* 0.35, 0.52, 0.22 */
        WGPUImageCopyTexture dst = {.texture = r->default_landuse_tex};
        WGPUTextureDataLayout layout = {.bytesPerRow = 4, .rowsPerImage = 1};
        WGPUExtent3D extent = {1, 1, 1};
        wgpuQueueWriteTexture(queue, &dst, pixel, 4, &layout, &extent);
    }

    /* Global uniform buffer + bind group */
    r->global_uniform_buf =
        create_buffer(device, queue, WGPUBufferUsage_Uniform, NULL,
                      sizeof(global_uniforms_t));
    WGPUBindGroupEntry bg_e = {
        .binding = 0,
        .buffer = r->global_uniform_buf,
        .offset = 0,
        .size = sizeof(global_uniforms_t),
    };
    r->global_bind_group = wgpuDeviceCreateBindGroup(
        device, &(WGPUBindGroupDescriptor){.layout = r->global_bgl,
                                           .entryCount = 1,
                                           .entries = &bg_e});

    create_depth_texture(r);

    /* Placeholder grid mesh */
    {
/* Subdivided grid so the placeholder follows globe curvature.
   The vertex shader does geodetic→ECEF per vertex, so more vertices
   means a smoother curved surface. */
#define PH_GRID 16
#define PH_VERTS (PH_GRID + 1) /* 17×17 = 289 vertices */
#define PH_NV (PH_VERTS * PH_VERTS)
#define PH_NI (PH_GRID * PH_GRID * 6) /* 1536 indices */

        uint16_t xy_data[PH_NV * 2];
        int32_t z_data[PH_NV];
        int8_t norm_data[PH_NV * 4]; /* padded to 4 bytes per vertex */
        uint32_t idx_data[PH_NI];

        /* Vertices: regular grid over tile proper [16384, 49151] */
        for (int row = 0; row < PH_VERTS; row++) {
            for (int col = 0; col < PH_VERTS; col++) {
                int vi = row * PH_VERTS + col;
                xy_data[vi * 2] = (uint16_t)(16384 + col * 32767 / PH_GRID);
                xy_data[vi * 2 + 1] = (uint16_t)(16384 + row * 32767 / PH_GRID);
                z_data[vi] = 0;
                /* Octahedral (0,0) → normal (0,0,1), padded */
                norm_data[vi * 4] = 0;
                norm_data[vi * 4 + 1] = 0;
                norm_data[vi * 4 + 2] = 0;
                norm_data[vi * 4 + 3] = 0;
            }
        }

        /* Indices: same winding as terrain_gen (tl, bl, tr, tr, bl, br) */
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
        r->ph_buf_xy = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                     xy_data, sizeof(xy_data));
        r->ph_buf_z = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                    z_data, sizeof(z_data));
        r->ph_buf_normals = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                          norm_data, sizeof(norm_data));
        r->ph_buf_indices = create_buffer(device, queue, WGPUBufferUsage_Index,
                                          idx_data, sizeof(idx_data));

#undef PH_GRID
#undef PH_VERTS
#undef PH_NV
#undef PH_NI

        /* 1×1 placeholder texture: neutral dark blue-gray */
        WGPUTextureDescriptor ph_td = {
            .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
            .size = {1, 1, 1},
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureDimension_2D,
            .mipLevelCount = 1,
            .sampleCount = 1,
        };
        r->ph_texture = wgpuDeviceCreateTexture(device, &ph_td);
        r->ph_texture_view = wgpuTextureCreateView(r->ph_texture, NULL);
        uint8_t ph_pixel[4] = {50, 60, 75, 255};
        WGPUImageCopyTexture ph_dst = {.texture = r->ph_texture};
        WGPUTextureDataLayout ph_layout = {.bytesPerRow = 4, .rowsPerImage = 1};
        WGPUExtent3D ph_extent = {1, 1, 1};
        wgpuQueueWriteTexture(queue, &ph_dst, ph_pixel, 4, &ph_layout,
                              &ph_extent);

        /* Pool of uniform buffers + bind groups for placeholder draws */
        for (int i = 0; i < ARPT_MAX_PLACEHOLDERS; i++) {
            r->ph_uniform_bufs[i] =
                create_buffer(device, queue, WGPUBufferUsage_Uniform, NULL,
                              sizeof(tile_uniforms_t));
            WGPUBindGroupEntry ph_entries[] = {
                {.binding = 0,
                 .buffer = r->ph_uniform_bufs[i],
                 .offset = 0,
                 .size = sizeof(tile_uniforms_t)},
                {.binding = 1, .textureView = r->ph_texture_view},
                {.binding = 2, .sampler = r->landuse_sampler},
            };
            r->ph_bind_groups[i] = wgpuDeviceCreateBindGroup(
                device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                                   .entryCount = 3,
                                                   .entries = ph_entries});
        }
    }

    return r;
}

void arpt_renderer_free(arpt_renderer *r) {
    if (!r) return;
    for (int i = 0; i < ARPT_MAX_PLACEHOLDERS; i++) {
        if (r->ph_bind_groups[i]) wgpuBindGroupRelease(r->ph_bind_groups[i]);
        if (r->ph_uniform_bufs[i]) wgpuBufferRelease(r->ph_uniform_bufs[i]);
    }
    if (r->ph_buf_xy) wgpuBufferRelease(r->ph_buf_xy);
    if (r->ph_buf_z) wgpuBufferRelease(r->ph_buf_z);
    if (r->ph_buf_normals) wgpuBufferRelease(r->ph_buf_normals);
    if (r->ph_buf_indices) wgpuBufferRelease(r->ph_buf_indices);
    if (r->ph_texture_view) wgpuTextureViewRelease(r->ph_texture_view);
    if (r->ph_texture) wgpuTextureRelease(r->ph_texture);
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);
    if (r->global_bind_group) wgpuBindGroupRelease(r->global_bind_group);
    if (r->global_uniform_buf) wgpuBufferRelease(r->global_uniform_buf);
    if (r->pipeline) wgpuRenderPipelineRelease(r->pipeline);
    if (r->landuse_pipeline) wgpuRenderPipelineRelease(r->landuse_pipeline);
    if (r->landuse_sampler) wgpuSamplerRelease(r->landuse_sampler);
    if (r->default_landuse_view)
        wgpuTextureViewRelease(r->default_landuse_view);
    if (r->default_landuse_tex) wgpuTextureRelease(r->default_landuse_tex);
    if (r->global_bgl) wgpuBindGroupLayoutRelease(r->global_bgl);
    if (r->tile_bgl) wgpuBindGroupLayoutRelease(r->tile_bgl);
    free(r);
}

void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height) {
    r->width = width;
    r->height = height;
    create_depth_texture(r);
}

/* Tile GPU */

arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_terrain_mesh *mesh,
                                         const arpt_landuse_data *landuse) {
    arpt_tile_gpu *t = calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->renderer = r;
    t->index_count = (uint32_t)mesh->index_count;

    /* Interleave x,y into uint16 pairs for Uint16x2 vertex format */
    size_t vc = mesh->vertex_count;
    uint16_t *xy = malloc(vc * 4); /* 2 × uint16 per vertex */
    if (!xy) {
        free(t);
        return NULL;
    }
    for (size_t i = 0; i < vc; i++) {
        xy[i * 2] = mesh->x[i];
        xy[i * 2 + 1] = mesh->y[i];
    }
    t->buf_xy =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex, xy, vc * 4);
    free(xy);

    t->buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                             mesh->z, vc * sizeof(int32_t));

    /* Pad normals to 4-byte stride (Sint8x2 + 2 padding bytes) */
    {
        int8_t *padded = calloc(vc, 4);
        if (!padded) {
            arpt_tile_gpu_free(t);
            return NULL;
        }
        for (size_t i = 0; i < vc; i++) {
            if (mesh->normals) {
                padded[i * 4] = mesh->normals[i * 2];
                padded[i * 4 + 1] = mesh->normals[i * 2 + 1];
            }
            /* bytes [2] and [3] stay zero (padding) */
        }
        t->buf_normals = create_buffer(r->device, r->queue,
                                       WGPUBufferUsage_Vertex, padded, vc * 4);
        free(padded);
    }

    t->buf_indices =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Index, mesh->indices,
                      mesh->index_count * sizeof(uint32_t));

    /* Rasterize landuse polygons to offscreen texture */
    if (landuse && landuse->count > 0) {
        t->landuse_texture = rasterize_landuse(r, landuse);
        t->landuse_view = wgpuTextureCreateView(t->landuse_texture, NULL);
    }

    WGPUTextureView lu_view =
        t->landuse_view ? t->landuse_view : r->default_landuse_view;

    /* Per-tile uniform buffer + bind group (uniform + texture + sampler) */
    t->uniform_buf = create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform,
                                   NULL, sizeof(tile_uniforms_t));
    WGPUBindGroupEntry entries[] = {
        {.binding = 0,
         .buffer = t->uniform_buf,
         .offset = 0,
         .size = sizeof(tile_uniforms_t)},
        {.binding = 1, .textureView = lu_view},
        {.binding = 2, .sampler = r->landuse_sampler},
    };
    t->bind_group = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                              .entryCount = 3,
                                              .entries = entries});

    return t;
}

void arpt_tile_gpu_set_uniforms(arpt_tile_gpu *tile, arpt_mat4 model,
                                const float bounds[4], float center_lon,
                                float center_lat) {
    tile_uniforms_t u;
    memcpy(u.model, model.m, sizeof(u.model));
    memcpy(u.bounds, bounds, sizeof(u.bounds));
    u.center_lon = center_lon;
    u.center_lat = center_lat;
    u._pad0 = 0.0f;
    u._pad1 = 0.0f;
    wgpuQueueWriteBuffer(tile->renderer->queue, tile->uniform_buf, 0, &u,
                         sizeof(u));
}

void arpt_tile_gpu_free(arpt_tile_gpu *tile) {
    if (!tile) return;
    if (tile->buf_xy) wgpuBufferRelease(tile->buf_xy);
    if (tile->buf_z) wgpuBufferRelease(tile->buf_z);
    if (tile->buf_normals) wgpuBufferRelease(tile->buf_normals);
    if (tile->buf_indices) wgpuBufferRelease(tile->buf_indices);
    if (tile->uniform_buf) wgpuBufferRelease(tile->uniform_buf);
    if (tile->bind_group) wgpuBindGroupRelease(tile->bind_group);
    if (tile->landuse_view) wgpuTextureViewRelease(tile->landuse_view);
    if (tile->landuse_texture) wgpuTextureRelease(tile->landuse_texture);
    free(tile);
}

/* Placeholder rendering */

void arpt_renderer_draw_placeholder(arpt_renderer *r, int slot, arpt_mat4 model,
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
}

/* Frame rendering */

void arpt_renderer_set_globals(arpt_renderer *r, arpt_mat4 projection,
                               arpt_vec3 sun_dir) {
    global_uniforms_t u;
    memcpy(u.projection, projection.m, sizeof(u.projection));
    u.sun_dir[0] = sun_dir.x;
    u.sun_dir[1] = sun_dir.y;
    u.sun_dir[2] = sun_dir.z;
    /* Non-sRGB formats need manual gamma correction in the shader */
    u.apply_gamma = (r->surface_format == WGPUTextureFormat_BGRA8UnormSrgb ||
                     r->surface_format == WGPUTextureFormat_RGBA8UnormSrgb)
                        ? 0.0f
                        : 1.0f;

    if (memcmp(&u, &r->prev_globals, sizeof(u)) == 0) return;

    r->prev_globals = u;
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
    wgpuRenderPassEncoderSetBindGroup(r->pass, 0, r->global_bind_group, 0,
                                      NULL);
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

void arpt_renderer_set_overlay(arpt_renderer *r, arpt_overlay_fn fn,
                               void *userdata) {
    if (!r) return;
    r->overlay_fn = fn;
    r->overlay_ud = userdata;
}

void arpt_renderer_end_frame(arpt_renderer *r) {
    /* Invoke overlay (e.g. UI) before closing the pass */
    if (r->overlay_fn) r->overlay_fn(r->pass, r->overlay_ud);
    wgpuRenderPassEncoderEnd(r->pass);
    wgpuRenderPassEncoderRelease(r->pass);
    r->pass = NULL;

    WGPUCommandBuffer cmd = wgpuCommandEncoderFinish(r->encoder, NULL);
    wgpuQueueSubmit(r->queue, 1, &cmd);

    wgpuCommandBufferRelease(cmd);
    wgpuCommandEncoderRelease(r->encoder);
    r->encoder = NULL;
}
