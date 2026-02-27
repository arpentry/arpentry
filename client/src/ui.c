#include "ui.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* WGSL Shader */

static const char *ui_wgsl =
    "struct Uniforms {\n"
    "    screen: vec2<f32>,\n"
    "    scale: f32,\n"
    "    bearing: f32,\n"
    "    tilt: f32,\n"
    "    cursor_x: f32,\n"
    "    cursor_y: f32,\n"
    "    _pad: f32,\n"
    "};\n"
    "\n"
    "@group(0) @binding(0) var<uniform> u: Uniforms;\n"
    "\n"
    "struct VsOut {\n"
    "    @builtin(position) pos: vec4<f32>,\n"
    "    @location(0) pixel: vec2<f32>,\n"
    "};\n"
    "\n"
    "// Full-screen triangle\n"
    "@vertex fn vs(@builtin(vertex_index) vi: u32) -> VsOut {\n"
    "    var p = array<vec2<f32>, 3>(\n"
    "        vec2<f32>(-1.0, 3.0),\n"
    "        vec2<f32>(-1.0, -1.0),\n"
    "        vec2<f32>(3.0, -1.0),\n"
    "    );\n"
    "    var out: VsOut;\n"
    "    out.pos = vec4<f32>(p[vi], 0.0, 1.0);\n"
    "    out.pixel = vec2<f32>(\n"
    "        (p[vi].x * 0.5 + 0.5) * u.screen.x,\n"
    "        (0.5 - p[vi].y * 0.5) * u.screen.y,\n"
    "    );\n"
    "    return out;\n"
    "}\n"
    "\n"
    "// SDF helpers\n"
    "fn sd_box(p: vec2<f32>, b: vec2<f32>, r: f32) -> f32 {\n"
    "    let q = abs(p) - b + r;\n"
    "    return length(max(q, vec2<f32>(0.0))) + min(max(q.x, q.y), 0.0) - r;\n"
    "}\n"
    "\n"
    "fn sd_circle(p: vec2<f32>, r: f32) -> f32 {\n"
    "    return length(p) - r;\n"
    "}\n"
    "\n"
    "fn fill(d: f32) -> f32 {\n"
    "    return clamp(0.5 - d, 0.0, 1.0);\n"
    "}\n"
    "\n"
    "// Layout constants (logical pixels, from bottom-right corner)\n"
    "// Order from right edge: compass, tilt, zoom. All 64px tall.\n"
    "const CY: f32      = 48.0;\n"
    "const COMP_R: f32  = 32.0;\n"
    "const COMP_CX: f32 = 48.0;\n"
    "const TILT_HW: f32 = 19.0;\n"
    "const TILT_HH: f32 = 32.0;\n"
    "const TILT_R: f32  = 19.0;\n"
    "const TILT_CX: f32 = 109.0;\n"
    "const ZOOM_HW: f32 = 19.0;\n"
    "const ZOOM_HH: f32 = 32.0;\n"
    "const ZOOM_R: f32  = 19.0;\n"
    "const ZOOM_CX: f32 = 157.0;\n"
    "\n"
    "@fragment fn fs(@location(0) pixel: vec2<f32>) -> @location(0) vec4<f32> {\n"
    "    let lp = pixel / u.scale;\n"
    "    let scr = u.screen / u.scale;\n"
    "    let br = scr - lp;\n"
    "\n"
    "    // Relative positions to each element center\n"
    "    let tilt_p = br - vec2<f32>(TILT_CX, CY);\n"
    "    let comp_p = br - vec2<f32>(COMP_CX, CY);\n"
    "    let zoom_p = br - vec2<f32>(ZOOM_CX, CY);\n"
    "\n"
    "    // Shape SDFs\n"
    "    let d_tilt = sd_box(tilt_p, vec2<f32>(TILT_HW, TILT_HH), TILT_R);\n"
    "    let d_comp = sd_circle(comp_p, COMP_R);\n"
    "    let d_zoom = sd_box(zoom_p, vec2<f32>(ZOOM_HW, ZOOM_HH), ZOOM_R);\n"
    "    let d_any = min(d_tilt, min(d_comp, d_zoom));\n"
    "\n"
    "    if (d_any > 1.5) { discard; }\n"
    "\n"
    "    // Glass background\n"
    "    var col = vec3<f32>(0.85, 0.88, 0.92);\n"
    "    var a = fill(d_any) * 0.5;\n"
    "\n"
    "    // Border highlight\n"
    "    let brd = fill(abs(d_any) - 0.75) * 0.4;\n"
    "    col = mix(col, vec3<f32>(1.0), brd);\n"
    "    a = max(a, brd);\n"
    "\n"
    "    let icon = vec3<f32>(0.22, 0.26, 0.32);\n"
    "\n"
    "    // ── Zoom controls (vertical pill) ─────────────\n"
    "    let iz = fill(d_zoom);\n"
    "\n"
    "    // Horizontal divider\n"
    "    let dv = fill(max(abs(zoom_p.y) - 0.5, abs(zoom_p.x) - 10.0));\n"
    "    let dv_a = dv * iz * 0.35;\n"
    "    col = mix(col, icon, dv_a);\n"
    "    a = max(a, dv_a);\n"
    "\n"
    "    // Plus sign (top half: positive zoom_p.y in br-coords)\n"
    "    let pp = zoom_p - vec2<f32>(0.0, 16.0);\n"
    "    let dp = fill(min(max(abs(pp.x) - 6.0, abs(pp.y) - 1.2),\n"
    "                      max(abs(pp.x) - 1.2, abs(pp.y) - 6.0)));\n"
    "    let dp_a = dp * iz;\n"
    "    col = mix(col, icon, dp_a);\n"
    "    a = max(a, dp_a);\n"
    "\n"
    "    // Minus sign (bottom half: negative zoom_p.y in br-coords)\n"
    "    let mp = zoom_p + vec2<f32>(0.0, 16.0);\n"
    "    let dm = fill(max(abs(mp.x) - 6.0, abs(mp.y) - 1.2));\n"
    "    let dm_a = dm * iz;\n"
    "    col = mix(col, icon, dm_a);\n"
    "    a = max(a, dm_a);\n"
    "\n"
    "    // ── Compass ────────────────────────────────────\n"
    "    let ic = fill(d_comp);\n"
    "\n"
    "    // Inner ring\n"
    "    let ring = fill(abs(sd_circle(comp_p, 27.0)) - 0.5) * 0.15;\n"
    "    col = mix(col, icon, ring * ic);\n"
    "    a = max(a, ring * ic);\n"
    "\n"
    "    // Rotate for bearing (convert br-coords to math coords by flipping x)\n"
    "    let mcp = vec2<f32>(-comp_p.x, comp_p.y);\n"
    "    let ca = -u.bearing;\n"
    "    let cc = cos(ca);\n"
    "    let cs = sin(ca);\n"
    "    let rp = vec2<f32>(cc * mcp.x - cs * mcp.y,\n"
    "                        cs * mcp.x + cc * mcp.y);\n"
    "\n"
    "    // North needle (red, tapered triangle)\n"
    "    let nw = 4.0 * clamp(1.0 - rp.y / 20.0, 0.0, 1.0);\n"
    "    let d_north = max(abs(rp.x) - nw, max(-rp.y, rp.y - 20.0));\n"
    "    let na = fill(d_north) * ic;\n"
    "    col = mix(col, vec3<f32>(0.85, 0.20, 0.15), na);\n"
    "    a = max(a, na);\n"
    "\n"
    "    // South needle (white, thinner)\n"
    "    let sw_val = 2.5 * clamp(1.0 + rp.y / 20.0, 0.0, 1.0);\n"
    "    let d_south = max(abs(rp.x) - sw_val, max(rp.y, -rp.y - 20.0));\n"
    "    let sa = fill(d_south) * ic;\n"
    "    col = mix(col, vec3<f32>(0.92, 0.92, 0.94), sa);\n"
    "    a = max(a, sa);\n"
    "\n"
    "    // Center dot\n"
    "    let cd = fill(sd_circle(comp_p, 2.5));\n"
    "    col = mix(col, vec3<f32>(0.95), cd * ic);\n"
    "    a = max(a, cd * ic);\n"
    "\n"
    "    // ── Tilt indicator ─────────────────────────────\n"
    "    let it = fill(d_tilt);\n"
    "\n"
    "    // Vertical track\n"
    "    let track = fill(max(abs(tilt_p.x) - 0.75, abs(tilt_p.y) - 20.0));\n"
    "    let tr_a = track * it * 0.25;\n"
    "    col = mix(col, icon, tr_a);\n"
    "    a = max(a, tr_a);\n"
    "\n"
    "    // Tilt dot (0 degrees at top, 60 degrees at bottom)\n"
    "    let tilt_t = clamp(u.tilt / 1.0472, 0.0, 1.0);\n"
    "    let dot_y = mix(16.0, -16.0, tilt_t);\n"
    "    let td = fill(sd_circle(tilt_p - vec2<f32>(0.0, dot_y), 4.5));\n"
    "    let td_a = td * it;\n"
    "    col = mix(col, icon, td_a);\n"
    "    a = max(a, td_a);\n"
    "\n"
    "    // ── Hover highlight ────────────────────────────\n"
    "    let cur = scr - vec2<f32>(u.cursor_x, u.cursor_y);\n"
    "    let hd_zoom = sd_box(cur - vec2<f32>(ZOOM_CX, CY),\n"
    "                          vec2<f32>(ZOOM_HW, ZOOM_HH), ZOOM_R);\n"
    "    let hd_comp = sd_circle(cur - vec2<f32>(COMP_CX, CY), COMP_R);\n"
    "    let hd_tilt = sd_box(cur - vec2<f32>(TILT_CX, CY),\n"
    "                          vec2<f32>(TILT_HW, TILT_HH), TILT_R);\n"
    "\n"
    "    var hover = 0.0;\n"
    "    if (hd_zoom < 0.0 && d_zoom < 0.0) {\n"
    "        let cz = cur.y - CY;\n"
    "        if ((cz > 0.0 && zoom_p.y > 0.0) ||\n"
    "            (cz <= 0.0 && zoom_p.y <= 0.0)) {\n"
    "            hover = 0.06;\n"
    "        }\n"
    "    }\n"
    "    if (hd_comp < 0.0 && d_comp < 0.0) { hover = 0.06; }\n"
    "    if (hd_tilt < 0.0 && d_tilt < 0.0) { hover = 0.06; }\n"
    "    col = col + vec3<f32>(hover);\n"
    "\n"
    "    return vec4<f32>(col, a);\n"
    "}\n";

/* Layout constants (must match shader) */

#define UI_CY      48.0f
#define UI_COMP_R  32.0f
#define UI_COMP_CX 48.0f
#define UI_TILT_HW 19.0f
#define UI_TILT_HH 32.0f
#define UI_TILT_R  19.0f
#define UI_TILT_CX 109.0f
#define UI_ZOOM_HW 19.0f
#define UI_ZOOM_HH 32.0f
#define UI_ZOOM_R  19.0f
#define UI_ZOOM_CX 157.0f

/* Uniform buffer layout */

typedef struct {
    float screen[2];
    float scale;
    float bearing;
    float tilt;
    float cursor_x;
    float cursor_y;
    float _pad;
} ui_uniforms_t;

/* Internal struct */

struct arpt_ui {
    WGPUDevice device;
    WGPUQueue queue;
    WGPURenderPipeline pipeline;
    WGPUBindGroupLayout bgl;
    WGPUBindGroup bind_group;
    WGPUBuffer uniform_buf;

    uint32_t fb_width, fb_height;
    float pixel_ratio;
    float bearing, tilt;
    float cursor_x, cursor_y;
};

/* SDF helpers (C, for hit testing) */

static float sd_box_c(float px, float py, float bx, float by, float r) {
    float qx = fabsf(px) - bx + r;
    float qy = fabsf(py) - by + r;
    float mx = fmaxf(qx, 0.0f);
    float my = fmaxf(qy, 0.0f);
    return sqrtf(mx * mx + my * my) + fminf(fmaxf(qx, qy), 0.0f) - r;
}

static float sd_circle_c(float px, float py, float r) {
    return sqrtf(px * px + py * py) - r;
}

/* Public API */

arpt_ui *arpt_ui_create(WGPUDevice device, WGPUQueue queue,
                         WGPUTextureFormat surface_format,
                         uint32_t fb_width, uint32_t fb_height,
                         float pixel_ratio) {
    arpt_ui *ui = calloc(1, sizeof(*ui));
    if (!ui) return NULL;

    ui->device = device;
    ui->queue = queue;
    ui->fb_width = fb_width;
    ui->fb_height = fb_height;
    ui->pixel_ratio = pixel_ratio;
    ui->cursor_x = -100.0f;
    ui->cursor_y = -100.0f;

    /* Shader module */
    WGPUShaderModuleWGSLDescriptor wgsl_desc = {
        .chain = {.sType = WGPUSType_ShaderModuleWGSLDescriptor},
        .code = ui_wgsl,
    };
    WGPUShaderModuleDescriptor sm_desc = {.nextInChain = &wgsl_desc.chain};
    WGPUShaderModule sm = wgpuDeviceCreateShaderModule(device, &sm_desc);

    /* Bind group layout: one uniform buffer */
    WGPUBindGroupLayoutEntry bgle = {
        .binding = 0,
        .visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment,
        .buffer = {.type = WGPUBufferBindingType_Uniform,
                   .minBindingSize = sizeof(ui_uniforms_t)},
    };
    ui->bgl = wgpuDeviceCreateBindGroupLayout(device,
        &(WGPUBindGroupLayoutDescriptor){.entryCount = 1, .entries = &bgle});

    /* Pipeline layout */
    WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(device,
        &(WGPUPipelineLayoutDescriptor){.bindGroupLayoutCount = 1,
                                         .bindGroupLayouts = &ui->bgl});

    /* Color target with alpha blending for glass effect */
    WGPUBlendState blend = {
        .color = {.srcFactor = WGPUBlendFactor_SrcAlpha,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
        .alpha = {.srcFactor = WGPUBlendFactor_One,
                  .dstFactor = WGPUBlendFactor_OneMinusSrcAlpha,
                  .operation = WGPUBlendOperation_Add},
    };
    WGPUColorTargetState ct = {
        .format = surface_format,
        .blend = &blend,
        .writeMask = WGPUColorWriteMask_All,
    };
    WGPUFragmentState frag = {
        .module = sm, .entryPoint = "fs", .targetCount = 1, .targets = &ct};

    /* Depth: always pass, no write (overlay on top of 3D scene) */
    WGPUDepthStencilState ds = {
        .format = WGPUTextureFormat_Depth24Plus,
        .depthWriteEnabled = false,
        .depthCompare = WGPUCompareFunction_Always,
        .stencilFront = {.compare = WGPUCompareFunction_Always},
        .stencilBack  = {.compare = WGPUCompareFunction_Always},
        .stencilReadMask  = 0,
        .stencilWriteMask = 0,
    };

    WGPURenderPipelineDescriptor pip = {
        .layout = pl,
        .vertex = {.module = sm, .entryPoint = "vs",
                   .bufferCount = 0, .buffers = NULL},
        .primitive = {.topology = WGPUPrimitiveTopology_TriangleList,
                      .cullMode = WGPUCullMode_None},
        .fragment = &frag,
        .depthStencil = &ds,
        .multisample = {.count = 1, .mask = ~0u},
    };
    ui->pipeline = wgpuDeviceCreateRenderPipeline(device, &pip);

    wgpuPipelineLayoutRelease(pl);
    wgpuShaderModuleRelease(sm);

    /* Uniform buffer */
    WGPUBufferDescriptor buf_desc = {
        .usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst,
        .size = sizeof(ui_uniforms_t),
    };
    ui->uniform_buf = wgpuDeviceCreateBuffer(device, &buf_desc);

    /* Bind group */
    WGPUBindGroupEntry bg_entry = {
        .binding = 0, .buffer = ui->uniform_buf,
        .offset = 0, .size = sizeof(ui_uniforms_t),
    };
    ui->bind_group = wgpuDeviceCreateBindGroup(device,
        &(WGPUBindGroupDescriptor){.layout = ui->bgl,
                                    .entryCount = 1, .entries = &bg_entry});

    return ui;
}

void arpt_ui_free(arpt_ui *ui) {
    if (!ui) return;
    if (ui->bind_group) wgpuBindGroupRelease(ui->bind_group);
    if (ui->uniform_buf) wgpuBufferRelease(ui->uniform_buf);
    if (ui->bgl) wgpuBindGroupLayoutRelease(ui->bgl);
    if (ui->pipeline) wgpuRenderPipelineRelease(ui->pipeline);
    free(ui);
}

void arpt_ui_resize(arpt_ui *ui, uint32_t fb_width, uint32_t fb_height,
                     float pixel_ratio) {
    ui->fb_width = fb_width;
    ui->fb_height = fb_height;
    ui->pixel_ratio = pixel_ratio;
}

void arpt_ui_set_state(arpt_ui *ui, float bearing_rad, float tilt_rad) {
    ui->bearing = bearing_rad;
    ui->tilt = tilt_rad;
}

void arpt_ui_set_cursor(arpt_ui *ui, float screen_x, float screen_y) {
    ui->cursor_x = screen_x;
    ui->cursor_y = screen_y;
}

arpt_ui_action arpt_ui_hit_test(const arpt_ui *ui,
                                  float screen_x, float screen_y) {
    float scr_w = (float)ui->fb_width / ui->pixel_ratio;
    float scr_h = (float)ui->fb_height / ui->pixel_ratio;
    float bx = scr_w - screen_x;
    float by = scr_h - screen_y;

    /* Compass (rightmost) */
    float dx = bx - UI_COMP_CX, dy = by - UI_CY;
    if (sd_circle_c(dx, dy, UI_COMP_R) < 0.0f)
        return ARPT_UI_RESET_NORTH;

    /* Tilt (middle) */
    dx = bx - UI_TILT_CX;
    dy = by - UI_CY;
    if (sd_box_c(dx, dy, UI_TILT_HW, UI_TILT_HH, UI_TILT_R) < 0.0f)
        return ARPT_UI_RESET_TILT;

    /* Zoom (leftmost, vertical — top half = plus, bottom half = minus) */
    dx = bx - UI_ZOOM_CX;
    dy = by - UI_CY;
    if (sd_box_c(dx, dy, UI_ZOOM_HW, UI_ZOOM_HH, UI_ZOOM_R) < 0.0f)
        return dy > 0.0f ? ARPT_UI_ZOOM_IN : ARPT_UI_ZOOM_OUT;

    return ARPT_UI_NONE;
}

void arpt_ui_draw(arpt_ui *ui, WGPURenderPassEncoder pass) {
    ui_uniforms_t u = {
        .screen = {(float)ui->fb_width, (float)ui->fb_height},
        .scale = ui->pixel_ratio,
        .bearing = ui->bearing,
        .tilt = ui->tilt,
        .cursor_x = ui->cursor_x,
        .cursor_y = ui->cursor_y,
    };
    wgpuQueueWriteBuffer(ui->queue, ui->uniform_buf, 0, &u, sizeof(u));

    wgpuRenderPassEncoderSetPipeline(pass, ui->pipeline);
    wgpuRenderPassEncoderSetBindGroup(pass, 0, ui->bind_group, 0, NULL);
    wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);
}
