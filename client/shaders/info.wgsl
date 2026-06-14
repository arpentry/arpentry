struct InfoUniforms {
    screen: vec2<f32>,
    scale: f32,
    atlas_size: f32,
    glyph_scale: f32,
    display_scale: f32,
    px_range: f32,      // distance field range in atlas pixels
    _pad0: f32,
};

@group(0) @binding(0) var<uniform> info: InfoUniforms;
@group(0) @binding(1) var atlas_tex: texture_2d<f32>;
@group(0) @binding(2) var atlas_samp: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex fn vs(
    @builtin(vertex_index) vid: u32,
    @location(0) screen_xy: vec2<f32>,
    @location(1) uv_rect: vec4<f32>,
    @location(2) offset: vec2<f32>,
) -> VsOut {
    let corner_x = f32(vid & 1u);
    let corner_y = f32((vid >> 1u) & 1u);

    // Glyph size in atlas pixels
    let glyph_w = (uv_rect.z - uv_rect.x) * info.atlas_size;
    let glyph_h = (uv_rect.w - uv_rect.y) * info.atlas_size;

    // Recover pixel offset (offset is normalized by glyph_scale)
    let gs = info.glyph_scale;
    let ds = info.display_scale;
    let px_x = screen_xy.x + (offset.x * gs + corner_x * glyph_w) * ds;
    let px_y = screen_xy.y + (offset.y * gs + corner_y * glyph_h) * ds;

    // Convert framebuffer pixel to NDC
    let ndc_x = px_x / info.screen.x * 2.0 - 1.0;
    let ndc_y = 1.0 - px_y / info.screen.y * 2.0;

    let uv = vec2<f32>(
        mix(uv_rect.x, uv_rect.z, corner_x),
        mix(uv_rect.y, uv_rect.w, corner_y),
    );

    var out: VsOut;
    out.pos = vec4<f32>(ndc_x, ndc_y, 0.0, 1.0);
    out.uv = uv;
    return out;
}

// Median of the three MSDF channels: the multi-channel field that keeps
// corners sharp where a single-channel SDF would round them off.
fn median3(v: vec3<f32>) -> f32 {
    return max(min(v.x, v.y), min(max(v.x, v.y), v.z));
}

@fragment fn fs(@location(0) uv: vec2<f32>) -> @location(0) vec4<f32> {
    // MTSDF atlas: rgb = multi-channel field (fill), a = true SDF (halo)
    let s = textureSample(atlas_tex, atlas_samp, uv);

    // Scale from field units [0,1] to framebuffer pixels at the current
    // glyph magnification.
    let unit_range = vec2<f32>(info.px_range) / vec2<f32>(info.atlas_size);
    let screen_size = vec2<f32>(1.0) / fwidth(uv);
    let spr = max(0.5 * dot(unit_range, screen_size), 1.0);

    let d_fill = spr * (median3(s.rgb) - 0.5);

    // Halo for readability over the map: 1.5 logical px, limited so it
    // cannot reach the quad border where the distance field saturates.
    let halo_px = clamp(1.5 * info.scale, 0.0, max(spr * 0.5 - 1.0, 0.0));
    let d_halo = spr * (s.a - 0.5) + halo_px;

    let fill_alpha = clamp(d_fill + 0.5, 0.0, 1.0);
    let halo_alpha = clamp(d_halo + 0.5, 0.0, 1.0);
    let alpha = max(fill_alpha, halo_alpha);

    if (alpha < 0.01) { discard; }

    // White text with dark halo
    let color = mix(vec3<f32>(0.08, 0.10, 0.14), vec3<f32>(0.95, 0.96, 0.98), fill_alpha);

    return vec4<f32>(color * alpha, alpha);
}
