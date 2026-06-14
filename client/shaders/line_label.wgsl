// Line-following labels (street names).
//
// Glyphs are placed along the screen projection of the road polyline on the
// CPU every frame (pipeline/line_label.c), so instances arrive in framebuffer
// pixels with a per-glyph rotation. The shader only expands the rotated quad
// and runs the same MTSDF coverage math as poi.wgsl.

struct GlobalUniforms {
    projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    apply_gamma: f32,
};

struct LabelUniforms {
    glyph_scale: f32,
    atlas_size: f32,
    viewport_width: f32,
    viewport_height: f32,
    display_scale: f32,
    halo_width: f32,    // halo width in framebuffer pixels
    px_range: f32,      // distance field range in atlas pixels
    _pad0: f32,
    fill_color: vec4<f32>,
    halo_color: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: GlobalUniforms;
@group(1) @binding(0) var<uniform> label: LabelUniforms;
@group(1) @binding(1) var font_tex: texture_2d<f32>;
@group(1) @binding(2) var font_samp: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

/* Per-instance data (framebuffer pixels):
 *   inst_center: glyph quad center
 *   inst_rot:    (cos a, sin a) rotation along the line
 *   inst_size:   quad width/height
 *   inst_uv:     glyph atlas UVs (u0, v0, u1, v1)
 */

@vertex fn vs(
    @builtin(vertex_index) vid: u32,
    @location(0) inst_center: vec2<f32>,
    @location(1) inst_rot: vec2<f32>,
    @location(2) inst_size: vec2<f32>,
    @location(3) inst_uv: vec4<f32>,
) -> VsOut {
    // Quad corners: 0=TL, 1=TR, 2=BL, 3=BR (triangle strip), y down.
    let corner_x = f32(vid & 1u);
    let corner_y = f32((vid >> 1u) & 1u);

    // Rotate the quad around its center (screen space, y down).
    let lx = (corner_x - 0.5) * inst_size.x;
    let ly = (corner_y - 0.5) * inst_size.y;
    let px = inst_center.x + lx * inst_rot.x - ly * inst_rot.y;
    let py = inst_center.y + lx * inst_rot.y + ly * inst_rot.x;

    var out: VsOut;
    out.pos = vec4<f32>(
        px / label.viewport_width * 2.0 - 1.0,
        1.0 - py / label.viewport_height * 2.0,
        0.0,
        1.0,
    );
    out.uv = vec2<f32>(
        mix(inst_uv.x, inst_uv.z, corner_x),
        mix(inst_uv.y, inst_uv.w, corner_y),
    );
    return out;
}

// Median of the three MSDF channels: the multi-channel field that keeps
// corners sharp where a single-channel SDF would round them off.
fn median3(v: vec3<f32>) -> f32 {
    return max(min(v.x, v.y), min(max(v.x, v.y), v.z));
}

@fragment fn fs(
    @location(0) uv: vec2<f32>,
) -> @location(0) vec4<f32> {
    // MTSDF atlas: rgb = multi-channel field (fill), a = true SDF (halo).
    let s = textureSample(font_tex, font_samp, uv);

    let unit_range = vec2<f32>(label.px_range) / vec2<f32>(label.atlas_size);
    let screen_size = vec2<f32>(1.0) / fwidth(uv);
    let spr = max(0.5 * dot(unit_range, screen_size), 1.0);

    let d_fill = spr * (median3(s.rgb) - 0.5);

    let halo_px = clamp(label.halo_width, 0.0, max(spr * 0.5 - 1.0, 0.0));
    let d_halo = spr * (s.a - 0.5) + halo_px;

    let fill_alpha = clamp(d_fill + 0.5, 0.0, 1.0);
    let halo_alpha = clamp(d_halo + 0.5, 0.0, 1.0);
    let alpha = max(fill_alpha, halo_alpha);

    if (alpha < 0.01) {
        discard;
    }

    let color = mix(label.halo_color.rgb, label.fill_color.rgb, fill_alpha);
    let out = select(color, pow(color, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    // Premultiplied alpha output — prevents seams between adjacent quads.
    return vec4<f32>(out * alpha, alpha);
}
