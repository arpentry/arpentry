#include "renderer.h"
#include "coords.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* WGSL shaders (generated from client/shaders/ at build time) */

#include "terrain.wgsl.h"
#include "surface.wgsl.h"
#include "highway.wgsl.h"
#include "wireframe.wgsl.h"

/* Surface color table */

#define SURFACE_TEX_SIZE 2048
#define SURFACE_MARGIN 0.0625 /* = SURFACE_BUFFER / SURFACE_GRID = 8/128 */

typedef struct {
    float r, g, b, a;
} surface_color_t;

static const surface_color_t surface_colors[] = {
    [ARPT_SURFACE_UNKNOWN]     = {0.42f, 0.62f, 0.28f, 1.0f},
    [ARPT_SURFACE_WATER]       = {0.09f, 0.22f, 0.45f, 1.0f},
    [ARPT_SURFACE_DESERT]      = {0.78f, 0.68f, 0.47f, 1.0f},
    [ARPT_SURFACE_FOREST]      = {0.10f, 0.30f, 0.08f, 1.0f},
    [ARPT_SURFACE_GRASSLAND]   = {0.42f, 0.62f, 0.28f, 1.0f},
    [ARPT_SURFACE_CROPLAND]    = {0.65f, 0.72f, 0.30f, 1.0f},
    [ARPT_SURFACE_SHRUB]       = {0.50f, 0.52f, 0.32f, 1.0f},
    [ARPT_SURFACE_ICE]         = {0.88f, 0.93f, 0.98f, 1.0f},
    [ARPT_SURFACE_PRIMARY]     = {0.35f, 0.33f, 0.30f, 1.0f}, /* dark asphalt */
    [ARPT_SURFACE_RESIDENTIAL] = {0.45f, 0.43f, 0.40f, 1.0f}, /* lighter road */
    [ARPT_SURFACE_BUILDING]    = {0.82f, 0.77f, 0.70f, 1.0f}, /* warm sandstone */
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
    WGPUTexture surface_texture;
    WGPUTextureView surface_view;
    uint32_t index_count;
    arpt_renderer *renderer;

    /* Building extrusion (separate draw call, same pipeline) */
    WGPUBuffer bldg_buf_xy;
    WGPUBuffer bldg_buf_z;
    WGPUBuffer bldg_buf_normals;
    WGPUBuffer bldg_buf_indices;
    WGPUBindGroup bldg_bind_group;
    uint32_t bldg_index_count;
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

    /* Surface offscreen rasterization */
    WGPURenderPipeline surface_pipeline;
    WGPURenderPipeline highway_pipeline;
    WGPUSampler surface_sampler;
    WGPUTexture default_surface_tex;
    WGPUTextureView default_surface_view;
    WGPUTexture building_tex;
    WGPUTextureView building_view;

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

    /* Wireframe SDF overlay for placeholders */
    WGPURenderPipeline wireframe_pipeline;
    WGPUBuffer ph_wire_buf_xy;
    WGPUBuffer ph_wire_buf_z;
    WGPUBuffer ph_wire_buf_dist;
    WGPUBuffer ph_wire_indices;
    uint32_t ph_wire_index_count;

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

/* Surface pipeline creation */

static WGPURenderPipeline create_surface_pipeline(WGPUDevice device) {
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = surface_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    /* No bind groups needed for surface rasterization */
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
        .arrayStride = 20, /* 4 bytes xy + 16 bytes color */
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

/* Highway pipeline — alpha-blended SDF lines with round caps */

static WGPURenderPipeline create_highway_pipeline(WGPUDevice device) {
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
        .arrayStride = 36, /* 4 + 16 + 8 + 8 */
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

/* Wireframe pipeline — SDF quads with alpha blending and depth bias */

static WGPURenderPipeline create_wireframe_pipeline(WGPUDevice device,
                                                     WGPUTextureFormat format,
                                                     WGPUBindGroupLayout global_bgl,
                                                     WGPUBindGroupLayout tile_bgl) {
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
        .depthWriteEnabled = false, /* overlay: read depth but don't write */
        .depthCompare = WGPUCompareFunction_LessEqual,
        .depthBias = -2, /* pull quads toward camera to overlay on fill */
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

/* Surface rasterization (offscreen, once per tile) */

typedef struct {
    uint16_t x, y; /* quantized position */
    float r, g, b, a;
} surface_vertex_t;

typedef struct {
    uint16_t x, y;         /* quantized position */
    float r, g, b, a;      /* color */
    float local_u, local_v; /* local SDF coordinates */
    float hw, seg_len;     /* half-width and segment length */
} highway_vertex_t;

/* Road half-widths in quantized units.
 * 1 quantized unit ≈ 0.028 pixels on the 1024-pixel surface texture.
 * Primary: ~4 px, residential: ~2.5 px. */
#define ROAD_HW_PRIMARY     140
#define ROAD_HW_RESIDENTIAL  90

/* Count vertices and triangle-fan indices for a set of surface polygons. */
static void count_polygon_geom(const arpt_surface_data *data,
                                size_t *out_verts, size_t *out_indices) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        size_t vc = data->polygons[i].vertex_count;
        if (vc < 3) continue;
        *out_verts += vc;
        *out_indices += (vc - 2) * 3;
    }
}

/* Append polygon vertices/indices into the combined buffers. */
static void emit_polygons(const arpt_surface_data *data,
                           surface_vertex_t *verts, uint32_t *idxs,
                           size_t *vi, size_t *ii) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        const arpt_surface_polygon *p = &data->polygons[i];
        if (p->vertex_count < 3) continue;

        surface_color_t c = surface_colors[p->cls];
        uint32_t base = (uint32_t)*vi;

        for (size_t v = 0; v < p->vertex_count; v++) {
            verts[*vi] = (surface_vertex_t){p->x[v], p->y[v],
                                             c.r, c.g, c.b, c.a};
            (*vi)++;
        }
        for (size_t v = 1; v + 1 < p->vertex_count; v++) {
            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + (uint32_t)v;
            idxs[(*ii)++] = base + (uint32_t)v + 1;
        }
    }
}

/* Expand highway segments to SDF quads (extended for round caps). */
static void emit_highway_sdf_quads(const arpt_highway_data *data,
                                    highway_vertex_t *verts, uint32_t *idxs,
                                    size_t *vi, size_t *ii) {
    if (!data) return;
    for (size_t i = 0; i < data->count; i++) {
        const arpt_highway_line *line = &data->lines[i];
        double hw = (line->cls == ARPT_SURFACE_PRIMARY) ? ROAD_HW_PRIMARY
                                                        : ROAD_HW_RESIDENTIAL;
        surface_color_t c = surface_colors[line->cls];

        for (size_t s = 0; s + 1 < line->vertex_count; s++) {
            double x1 = line->x[s], y1 = line->y[s];
            double x2 = line->x[s + 1], y2 = line->y[s + 1];
            double dx = x2 - x1, dy = y2 - y1;
            double len = sqrt(dx * dx + dy * dy);
            if (len < 1.0) continue;

            /* Unit direction and perpendicular */
            double ux = dx / len, uy = dy / len;
            double nx = -uy, ny = ux;

            /* Extend quad by hw along segment direction for round caps */
            double ex1 = x1 - ux * hw, ey1 = y1 - uy * hw;
            double ex2 = x2 + ux * hw, ey2 = y2 + uy * hw;

#define CLAMP16(v) ((uint16_t)((v) < 0 ? 0 : (v) > 65535 ? 65535 : (v)))
            uint32_t base = (uint32_t)*vi;
            verts[*vi] = (highway_vertex_t){
                CLAMP16(ex1 - nx * hw), CLAMP16(ey1 - ny * hw),
                c.r, c.g, c.b, c.a,
                (float)(-hw), (float)(-hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (highway_vertex_t){
                CLAMP16(ex1 + nx * hw), CLAMP16(ey1 + ny * hw),
                c.r, c.g, c.b, c.a,
                (float)(-hw), (float)(hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (highway_vertex_t){
                CLAMP16(ex2 + nx * hw), CLAMP16(ey2 + ny * hw),
                c.r, c.g, c.b, c.a,
                (float)(len + hw), (float)(hw), (float)hw, (float)len};
            (*vi)++;
            verts[*vi] = (highway_vertex_t){
                CLAMP16(ex2 - nx * hw), CLAMP16(ey2 - ny * hw),
                c.r, c.g, c.b, c.a,
                (float)(len + hw), (float)(-hw), (float)hw, (float)len};
            (*vi)++;
#undef CLAMP16

            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + 1;
            idxs[(*ii)++] = base + 2;
            idxs[(*ii)++] = base;
            idxs[(*ii)++] = base + 2;
            idxs[(*ii)++] = base + 3;
        }
    }
}

static WGPUTexture rasterize_surface(arpt_renderer *r,
                                     const arpt_surface_data *surface,
                                     const arpt_highway_data *highways) {
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

    /* Count polygon and highway geometry separately */
    size_t poly_verts = 0, poly_indices = 0;
    count_polygon_geom(surface, &poly_verts, &poly_indices);

    size_t hw_verts = 0, hw_indices = 0;
    if (highways) {
        for (size_t i = 0; i < highways->count; i++) {
            size_t segs = highways->lines[i].vertex_count - 1;
            hw_verts += segs * 4;
            hw_indices += segs * 6;
        }
    }

    bool has_polys = poly_verts > 0 && poly_indices > 0;
    bool has_highways = hw_verts > 0 && hw_indices > 0;

    if (!has_polys && !has_highways) {
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

    /* Build polygon GPU buffers */
    WGPUBuffer poly_vbuf = NULL, poly_ibuf = NULL;
    size_t poly_vb_size = 0, poly_draw_n = 0;
    if (has_polys) {
        surface_vertex_t *pv = malloc(poly_verts * sizeof(surface_vertex_t));
        uint32_t *pi = malloc(poly_indices * sizeof(uint32_t));
        if (pv && pi) {
            size_t vi = 0, ii = 0;
            emit_polygons(surface, pv, pi, &vi, &ii);
            poly_vb_size = vi * sizeof(surface_vertex_t);
            poly_vbuf = create_buffer(r->device, r->queue,
                                      WGPUBufferUsage_Vertex, pv,
                                      poly_vb_size);
            poly_ibuf = create_buffer(r->device, r->queue,
                                      WGPUBufferUsage_Index, pi,
                                      ii * sizeof(uint32_t));
            poly_draw_n = ii;
        }
        free(pv);
        free(pi);
    }

    /* Build highway GPU buffers */
    WGPUBuffer hw_vbuf = NULL, hw_ibuf = NULL;
    size_t hw_vb_size = 0, hw_draw_n = 0;
    if (has_highways) {
        highway_vertex_t *hv = malloc(hw_verts * sizeof(highway_vertex_t));
        uint32_t *hi = malloc(hw_indices * sizeof(uint32_t));
        if (hv && hi) {
            size_t vi = 0, ii = 0;
            emit_highway_sdf_quads(highways, hv, hi, &vi, &ii);
            hw_vb_size = vi * sizeof(highway_vertex_t);
            hw_vbuf = create_buffer(r->device, r->queue,
                                    WGPUBufferUsage_Vertex, hv,
                                    hw_vb_size);
            hw_ibuf = create_buffer(r->device, r->queue,
                                    WGPUBufferUsage_Index, hi,
                                    ii * sizeof(uint32_t));
            hw_draw_n = ii;
        }
        free(hv);
        free(hi);
    }

    /* Render pass: polygons (opaque), then highways (alpha-blended SDF) */
    WGPUTextureView view = wgpuTextureCreateView(tex, NULL);
    WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(r->device, NULL);
    WGPURenderPassColorAttachment color = {
        .view = view,
        .loadOp = WGPULoadOp_Clear,
        .storeOp = WGPUStoreOp_Store,
        .clearValue = {0.35, 0.52, 0.22, 1.0},
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

    /* Draw 1: surface polygons (opaque) */
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

    /* Draw 2: highways (alpha-blended SDF with round caps) */
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
    r->wireframe_pipeline =
        create_wireframe_pipeline(device, format, r->global_bgl, r->tile_bgl);

    /* Surface offscreen pipeline + sampler */
    r->surface_pipeline = create_surface_pipeline(device);
    r->highway_pipeline = create_highway_pipeline(device);
    WGPUSamplerDescriptor samp_desc = {
        .addressModeU = WGPUAddressMode_ClampToEdge,
        .addressModeV = WGPUAddressMode_ClampToEdge,
        .magFilter = WGPUFilterMode_Linear,
        .minFilter = WGPUFilterMode_Linear,
        .maxAnisotropy = 1,
        .lodMaxClamp = 32.0f,
    };
    r->surface_sampler = wgpuDeviceCreateSampler(device, &samp_desc);

    /* Default 1x1 grass-colored surface texture for tiles without surface data */
    {
        WGPUTextureDescriptor dt = {
            .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
            .size = {1, 1, 1},
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureDimension_2D,
            .mipLevelCount = 1,
            .sampleCount = 1,
        };
        r->default_surface_tex = wgpuDeviceCreateTexture(device, &dt);
        r->default_surface_view =
            wgpuTextureCreateView(r->default_surface_tex, NULL);
        uint8_t pixel[4] = {89, 133, 56, 255}; /* 0.35, 0.52, 0.22 */
        WGPUImageCopyTexture dst = {.texture = r->default_surface_tex};
        WGPUTextureDataLayout layout = {.bytesPerRow = 4, .rowsPerImage = 1};
        WGPUExtent3D extent = {1, 1, 1};
        wgpuQueueWriteTexture(queue, &dst, pixel, 4, &layout, &extent);
    }

    /* 1x1 gray building texture */
    {
        WGPUTextureDescriptor dt = {
            .usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst,
            .size = {1, 1, 1},
            .format = WGPUTextureFormat_RGBA8Unorm,
            .dimension = WGPUTextureDimension_2D,
            .mipLevelCount = 1,
            .sampleCount = 1,
        };
        r->building_tex = wgpuDeviceCreateTexture(device, &dt);
        r->building_view = wgpuTextureCreateView(r->building_tex, NULL);
        uint8_t pixel[4] = {189, 186, 182, 255}; /* light neutral gray */
        WGPUImageCopyTexture dst = {.texture = r->building_tex};
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

        /* Wireframe SDF quads: expand each grid edge into a quad with
         * signed distance for anti-aliased rendering.  Half-width in
         * quantized units — the shader adapts to screen-space via fwidth. */
#define PH_WIRE_HW 200
#define PH_EDGE_COUNT (PH_VERTS * PH_GRID + PH_GRID * PH_VERTS) /* 544 */
#define PH_WIRE_NV (PH_EDGE_COUNT * 4)  /* 4 vertices per quad  */
#define PH_WIRE_NI2 (PH_EDGE_COUNT * 6) /* 6 indices per quad   */

        uint16_t w_xy[PH_WIRE_NV * 2];
        int32_t w_z[PH_WIRE_NV];
        float w_dist[PH_WIRE_NV];
        uint32_t w_idx[PH_WIRE_NI2];
        int wv = 0, wi = 0;

        /* Horizontal edges: expand perpendicular (y ± hw) */
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

        /* Vertical edges: expand perpendicular (x ± hw) */
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
        r->ph_wire_buf_xy = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                          w_xy, sizeof(w_xy));
        r->ph_wire_buf_z = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                         w_z, sizeof(w_z));
        r->ph_wire_buf_dist = create_buffer(device, queue, WGPUBufferUsage_Vertex,
                                            w_dist, sizeof(w_dist));
        r->ph_wire_indices = create_buffer(device, queue, WGPUBufferUsage_Index,
                                           w_idx, sizeof(w_idx));
#undef PH_WIRE_HW
#undef PH_EDGE_COUNT
#undef PH_WIRE_NV
#undef PH_WIRE_NI2

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
                {.binding = 2, .sampler = r->surface_sampler},
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
    if (r->ph_wire_buf_xy) wgpuBufferRelease(r->ph_wire_buf_xy);
    if (r->ph_wire_buf_z) wgpuBufferRelease(r->ph_wire_buf_z);
    if (r->ph_wire_buf_dist) wgpuBufferRelease(r->ph_wire_buf_dist);
    if (r->ph_wire_indices) wgpuBufferRelease(r->ph_wire_indices);
    if (r->wireframe_pipeline) wgpuRenderPipelineRelease(r->wireframe_pipeline);
    if (r->ph_texture_view) wgpuTextureViewRelease(r->ph_texture_view);
    if (r->ph_texture) wgpuTextureRelease(r->ph_texture);
    if (r->depth_view) wgpuTextureViewRelease(r->depth_view);
    if (r->depth_texture) wgpuTextureRelease(r->depth_texture);
    if (r->global_bind_group) wgpuBindGroupRelease(r->global_bind_group);
    if (r->global_uniform_buf) wgpuBufferRelease(r->global_uniform_buf);
    if (r->pipeline) wgpuRenderPipelineRelease(r->pipeline);
    if (r->surface_pipeline) wgpuRenderPipelineRelease(r->surface_pipeline);
    if (r->highway_pipeline) wgpuRenderPipelineRelease(r->highway_pipeline);
    if (r->surface_sampler) wgpuSamplerRelease(r->surface_sampler);
    if (r->default_surface_view)
        wgpuTextureViewRelease(r->default_surface_view);
    if (r->default_surface_tex) wgpuTextureRelease(r->default_surface_tex);
    if (r->building_view) wgpuTextureViewRelease(r->building_view);
    if (r->building_tex) wgpuTextureRelease(r->building_tex);
    if (r->global_bgl) wgpuBindGroupLayoutRelease(r->global_bgl);
    if (r->tile_bgl) wgpuBindGroupLayoutRelease(r->tile_bgl);
    free(r);
}

void arpt_renderer_resize(arpt_renderer *r, uint32_t width, uint32_t height) {
    r->width = width;
    r->height = height;
    create_depth_texture(r);
}

/* Building extrusion */

#define DEG_TO_RAD (M_PI / 180.0)

/* Encode a unit ECEF normal vector to octahedral int8x2. */
static void encode_octahedral(double nx, double ny, double nz, int8_t *ox,
                               int8_t *oy) {
    double ax = fabs(nx), ay = fabs(ny), az = fabs(nz);
    double sum = ax + ay + az;
    if (sum < 1e-15) {
        *ox = 0;
        *oy = 127;
        return;
    }
    double u = nx / sum;
    double v = ny / sum;
    if (nz < 0.0) {
        double old_u = u, old_v = v;
        u = (1.0 - fabs(old_v)) * (old_u >= 0.0 ? 1.0 : -1.0);
        v = (1.0 - fabs(old_u)) * (old_v >= 0.0 ? 1.0 : -1.0);
    }
    double cu = u * 127.0;
    double cv = v * 127.0;
    *ox = (int8_t)(cu >= 0.0 ? cu + 0.5 : cu - 0.5);
    *oy = (int8_t)(cv >= 0.0 ? cv + 0.5 : cv - 0.5);
}

/* A building is "owned" by this tile if its centroid falls within the tile
 * proper [ARPT_BUFFER, ARPT_BUFFER + ARPT_EXTENT).  Buildings whose centroid
 * lies in the buffer zone belong to the adjacent tile, avoiding double render. */
static bool building_in_tile_proper(const arpt_surface_polygon *b) {
    size_t n = b->vertex_count - 1; /* closed ring: last == first */
    uint32_t sx = 0, sy = 0;
    for (size_t v = 0; v < n; v++) {
        sx += b->x[v];
        sy += b->y[v];
    }
    uint16_t cx = (uint16_t)(sx / n);
    uint16_t cy = (uint16_t)(sy / n);
    return cx >= ARPT_BUFFER && cx < (ARPT_BUFFER + ARPT_EXTENT) &&
           cy >= ARPT_BUFFER && cy < (ARPT_BUFFER + ARPT_EXTENT);
}

/* Count building extrusion vertices and indices.
 * For a building with N vertices (closed ring, N-1 unique):
 *   Wall: (N-1)*4 verts, (N-1)*6 indices
 *   Roof: (N-1) verts, (N-3)*3 indices */
static void count_building_extrusion(const arpt_surface_data *buildings,
                                     size_t *extra_verts,
                                     size_t *extra_indices) {
    if (!buildings) return;
    for (size_t i = 0; i < buildings->count; i++) {
        const arpt_surface_polygon *b = &buildings->polygons[i];
        if (b->height_m <= 0 || b->vertex_count < 4) continue;
        if (!building_in_tile_proper(b)) continue;
        size_t n = b->vertex_count - 1; /* unique vertices (closed ring) */
        *extra_verts += n * 4 + n;       /* wall + roof */
        *extra_indices += n * 6 + (n - 2) * 3;
    }
}

/* Emit building wall + roof geometry into pre-allocated arrays.
 * east/north/up are ENU basis vectors in ECEF at tile center.
 * base_idx is added to all generated index values. */
static void emit_building_extrusion(const arpt_surface_data *buildings,
                                    double east[3], double north[3],
                                    double up[3], arpt_bounds bounds,
                                    uint16_t *xy, int32_t *z, int8_t *norms,
                                    uint32_t *indices, size_t *vi,
                                    size_t *ii) {
    if (!buildings) return;

    /* Precompute roof normal in octahedral encoding */
    int8_t roof_ox, roof_oy;
    encode_octahedral(up[0], up[1], up[2], &roof_ox, &roof_oy);

    for (size_t bi = 0; bi < buildings->count; bi++) {
        const arpt_surface_polygon *b = &buildings->polygons[bi];
        if (b->height_m <= 0 || b->vertex_count < 4) continue;
        if (!building_in_tile_proper(b)) continue;

        size_t n = b->vertex_count - 1; /* unique vertices */
        int32_t base_z = (b->z && b->vertex_count > 0) ? b->z[0] : 0;
        int32_t height_mm = base_z + b->height_m * 1000;

        /* Wall quads: one quad per edge A→B */
        for (size_t e = 0; e < n; e++) {
            size_t next = (e + 1) % n;
            uint16_t ax = b->x[e], ay = b->y[e];
            uint16_t bx = b->x[next], by = b->y[next];

            /* Outward perpendicular in dequantized tile space.
             * Edge direction: (dx, dy) in normalized tile coords.
             * Outward normal (for CCW ring): (dy, -dx). */
            double dx = arpt_dequantize(bx) - arpt_dequantize(ax);
            double dy = arpt_dequantize(by) - arpt_dequantize(ay);
            double len = sqrt(dx * dx + dy * dy);
            if (len < 1e-12) len = 1e-12;

            /* Perpendicular scaled to approximate lon/lat */
            double lon_span = bounds.east - bounds.west;
            double lat_span = bounds.north - bounds.south;
            double perp_e = (dy / len) * lon_span;
            double perp_n = (-dx / len) * lat_span;
            double plen = sqrt(perp_e * perp_e + perp_n * perp_n);
            if (plen < 1e-12) plen = 1e-12;
            perp_e /= plen;
            perp_n /= plen;

            /* Wall normal in ECEF = perp_e * East + perp_n * North */
            double wnx = perp_e * east[0] + perp_n * north[0];
            double wny = perp_e * east[1] + perp_n * north[1];
            double wnz = perp_e * east[2] + perp_n * north[2];
            double wnlen = sqrt(wnx * wnx + wny * wny + wnz * wnz);
            if (wnlen > 1e-12) {
                wnx /= wnlen;
                wny /= wnlen;
                wnz /= wnlen;
            }
            int8_t wall_ox, wall_oy;
            encode_octahedral(wnx, wny, wnz, &wall_ox, &wall_oy);

            /* 4 vertices: A_bottom, A_top, B_top, B_bottom */
            uint32_t base = (uint32_t)*vi;

            xy[*vi * 2] = ax;
            xy[*vi * 2 + 1] = ay;
            z[*vi] = base_z;
            norms[*vi * 2] = wall_ox;
            norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = ax;
            xy[*vi * 2 + 1] = ay;
            z[*vi] = height_mm;
            norms[*vi * 2] = wall_ox;
            norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = bx;
            xy[*vi * 2 + 1] = by;
            z[*vi] = height_mm;
            norms[*vi * 2] = wall_ox;
            norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            xy[*vi * 2] = bx;
            xy[*vi * 2 + 1] = by;
            z[*vi] = base_z;
            norms[*vi * 2] = wall_ox;
            norms[*vi * 2 + 1] = wall_oy;
            (*vi)++;

            /* 2 CW triangles: (bottom_a, top_a, top_b), (bottom_a, top_b,
             * bottom_b) */
            indices[(*ii)++] = base;
            indices[(*ii)++] = base + 1;
            indices[(*ii)++] = base + 2;
            indices[(*ii)++] = base;
            indices[(*ii)++] = base + 2;
            indices[(*ii)++] = base + 3;
        }

        /* Roof: triangle fan from vertex 0 (CW winding) */
        uint32_t roof_base = (uint32_t)*vi;
        for (size_t v = 0; v < n; v++) {
            xy[*vi * 2] = b->x[v];
            xy[*vi * 2 + 1] = b->y[v];
            z[*vi] = height_mm;
            norms[*vi * 2] = roof_ox;
            norms[*vi * 2 + 1] = roof_oy;
            (*vi)++;
        }
        for (size_t v = 1; v + 1 < n; v++) {
            indices[(*ii)++] = roof_base;
            indices[(*ii)++] = roof_base + (uint32_t)(v + 1);
            indices[(*ii)++] = roof_base + (uint32_t)v;
        }
    }
}

/* Upload building extrusion geometry into separate GPU buffers on tile. */
static void upload_building_extrusion(arpt_tile_gpu *t, arpt_renderer *r,
                                      const arpt_surface_data *buildings,
                                      arpt_bounds bounds) {
    size_t nv = 0, ni = 0;
    count_building_extrusion(buildings, &nv, &ni);
    if (nv == 0 || ni == 0) return;

    /* Allocate building-only arrays */
    uint16_t *xy = malloc(nv * 4);
    int32_t *z = malloc(nv * sizeof(int32_t));
    int8_t *norms = calloc(nv, 2);
    uint32_t *idx = malloc(ni * sizeof(uint32_t));
    if (!xy || !z || !norms || !idx) {
        free(xy);
        free(z);
        free(norms);
        free(idx);
        return;
    }

    /* Compute ENU basis at tile center */
    double clon = (bounds.west + bounds.east) * 0.5 * DEG_TO_RAD;
    double clat = (bounds.south + bounds.north) * 0.5 * DEG_TO_RAD;
    double slon = sin(clon), clon_c = cos(clon);
    double slat = sin(clat), clat_c = cos(clat);

    double east_v[3] = {-slon, clon_c, 0.0};
    double north_v[3] = {-slat * clon_c, -slat * slon, clat_c};
    double up_v[3] = {clat_c * clon_c, clat_c * slon, slat};

    /* Building indices start at 0 (separate index buffer) */
    size_t vi = 0, ii = 0;
    emit_building_extrusion(buildings, east_v, north_v, up_v, bounds, xy, z,
                            norms, idx, &vi, &ii);

    /* Upload to GPU */
    t->bldg_buf_xy =
        create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex, xy, nv * 4);
    free(xy);

    t->bldg_buf_z = create_buffer(r->device, r->queue, WGPUBufferUsage_Vertex,
                                  z, nv * sizeof(int32_t));
    free(z);

    /* Pad normals to 4-byte stride */
    {
        int8_t *padded = calloc(nv, 4);
        if (!padded) {
            free(norms);
            return;
        }
        for (size_t i = 0; i < nv; i++) {
            padded[i * 4] = norms[i * 2];
            padded[i * 4 + 1] = norms[i * 2 + 1];
        }
        t->bldg_buf_normals = create_buffer(
            r->device, r->queue, WGPUBufferUsage_Vertex, padded, nv * 4);
        free(padded);
    }
    free(norms);

    t->bldg_buf_indices = create_buffer(r->device, r->queue,
                                        WGPUBufferUsage_Index, idx,
                                        ni * sizeof(uint32_t));
    free(idx);

    t->bldg_index_count = (uint32_t)ni;
}

/* Tile GPU */

arpt_tile_gpu *arpt_renderer_upload_tile(arpt_renderer *r,
                                         const arpt_terrain_mesh *mesh,
                                         const arpt_surface_data *surface,
                                         const arpt_highway_data *highways,
                                         const arpt_surface_data *buildings,
                                         arpt_bounds bounds) {
    arpt_tile_gpu *t = calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->renderer = r;
    t->index_count = (uint32_t)mesh->index_count;

    /* Upload terrain buffers (original zero-copy path) */

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

    /* Upload building extrusion as separate buffers */
    upload_building_extrusion(t, r, buildings, bounds);

    /* Rasterize surface + highway features to offscreen texture */
    bool has_surface = surface && surface->count > 0;
    bool has_highways = highways && highways->count > 0;
    bool has_buildings = buildings && buildings->count > 0;
    if (has_surface || has_highways) {
        t->surface_texture =
            rasterize_surface(r, has_surface ? surface : NULL,
                              has_highways ? highways : NULL);
        t->surface_view = wgpuTextureCreateView(t->surface_texture, NULL);
    }

    WGPUTextureView lu_view =
        t->surface_view ? t->surface_view : r->default_surface_view;

    /* Per-tile uniform buffer + bind group (uniform + texture + sampler) */
    t->uniform_buf = create_buffer(r->device, r->queue, WGPUBufferUsage_Uniform,
                                   NULL, sizeof(tile_uniforms_t));
    WGPUBindGroupEntry entries[] = {
        {.binding = 0,
         .buffer = t->uniform_buf,
         .offset = 0,
         .size = sizeof(tile_uniforms_t)},
        {.binding = 1, .textureView = lu_view},
        {.binding = 2, .sampler = r->surface_sampler},
    };
    t->bind_group = wgpuDeviceCreateBindGroup(
        r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                              .entryCount = 3,
                                              .entries = entries});

    /* Building bind group: same uniforms, but solid gray texture */
    if (t->bldg_index_count > 0) {
        WGPUBindGroupEntry bldg_entries[] = {
            {.binding = 0,
             .buffer = t->uniform_buf,
             .offset = 0,
             .size = sizeof(tile_uniforms_t)},
            {.binding = 1, .textureView = r->building_view},
            {.binding = 2, .sampler = r->surface_sampler},
        };
        t->bldg_bind_group = wgpuDeviceCreateBindGroup(
            r->device, &(WGPUBindGroupDescriptor){.layout = r->tile_bgl,
                                                  .entryCount = 3,
                                                  .entries = bldg_entries});
    }

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
    if (tile->bldg_buf_xy) wgpuBufferRelease(tile->bldg_buf_xy);
    if (tile->bldg_buf_z) wgpuBufferRelease(tile->bldg_buf_z);
    if (tile->bldg_buf_normals) wgpuBufferRelease(tile->bldg_buf_normals);
    if (tile->bldg_buf_indices) wgpuBufferRelease(tile->bldg_buf_indices);
    if (tile->uniform_buf) wgpuBufferRelease(tile->uniform_buf);
    if (tile->bind_group) wgpuBindGroupRelease(tile->bind_group);
    if (tile->bldg_bind_group) wgpuBindGroupRelease(tile->bldg_bind_group);
    if (tile->surface_view) wgpuTextureViewRelease(tile->surface_view);
    if (tile->surface_texture) wgpuTextureRelease(tile->surface_texture);
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

    /* Draw terrain mesh */
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

    /* Draw extruded buildings (same pipeline, building bind group) */
    if (tile->bldg_index_count > 0) {
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
