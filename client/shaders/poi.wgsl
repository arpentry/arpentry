const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;
const PI: f32 = 3.14159265;

struct GlobalUniforms {
    projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    apply_gamma: f32,
};

struct TileUniforms {
    model: mat4x4<f32>,
    bounds: vec4<f32>,
    center_lon: f32,
    center_lat: f32,
    _pad0: f32,
    _pad1: f32,
};

struct PoiUniforms {
    glyph_scale: f32,
    atlas_size: f32,
    viewport_width: f32,
    viewport_height: f32,
    display_scale: f32,
    halo_width: f32,
    _pad0: f32,
    _pad1: f32,
    fill_color: vec4<f32>,
    halo_color: vec4<f32>,
};

@group(0) @binding(0) var<uniform> globals: GlobalUniforms;
@group(1) @binding(0) var<uniform> tile: TileUniforms;
@group(2) @binding(0) var<uniform> poi: PoiUniforms;
@group(2) @binding(1) var font_tex: texture_2d<f32>;
@group(2) @binding(2) var font_samp: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

fn geodetic_to_ecef(lon: f32, lat: f32, alt: f32) -> vec3<f32> {
    let sin_lat = sin(lat);
    let cos_lat = cos(lat);
    let sin_lon = sin(lon);
    let cos_lon = cos(lon);
    let N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    return vec3<f32>(
        (N + alt) * cos_lat * cos_lon,
        (N + alt) * cos_lat * sin_lon,
        (N * (1.0 - WGS84_E2) + alt) * sin_lat,
    );
}

/* Per-instance data:
 *   inst_qxy:    uint16x2 — quantized position in tile
 *   inst_qz:     int32    — elevation in mm
 *   inst_uv:     float32x4 — glyph atlas UVs (u0, v0, u1, v1)
 *   inst_offset:  float32x2 — (x_offset, y_offset) in normalized font units
 */

@vertex fn vs(
    @builtin(vertex_index) vid: u32,
    @location(0) inst_qxy: vec2<u32>,
    @location(1) inst_qz: i32,
    @location(2) inst_uv: vec4<f32>,
    @location(3) inst_offset: vec2<f32>,
) -> VsOut {
    // Quad corners: 0=BL, 1=BR, 2=TL, 3=TR (triangle strip)
    let corner_x = f32(vid & 1u);
    let corner_y = f32((vid >> 1u) & 1u);

    // Dequantize instance tile position
    let lon_west = tile.bounds.x;
    let lat_south = tile.bounds.y;
    let lon_east = tile.bounds.z;
    let lat_north = tile.bounds.w;

    let u = (f32(inst_qxy.x) - 16384.0) / 32768.0;
    let v = (f32(inst_qxy.y) - 16384.0) / 32768.0;
    let inst_lon = lon_west + u * (lon_east - lon_west);
    let inst_lat = lat_south + v * (lat_north - lat_south);
    let inst_alt = f32(inst_qz) * 0.001;

    // Project anchor to clip space
    let inst_ecef = geodetic_to_ecef(inst_lon, inst_lat, inst_alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let local_ecef = inst_ecef - center_ecef;
    let anchor_clip = globals.projection * tile.model * vec4<f32>(local_ecef, 1.0);

    // Glyph size in atlas pixels (derived from UV rect)
    let glyph_w_px = (inst_uv.z - inst_uv.x) * poi.atlas_size;
    let glyph_h_px = (inst_uv.w - inst_uv.y) * poi.atlas_size;

    // Recover pixel offsets (inst_offset is normalized by font_pixel_height)
    let gs = poi.glyph_scale; /* = font_pixel_height */
    let px_x = inst_offset.x * gs;
    let px_y = inst_offset.y * gs;

    // Apply display scale to shrink/grow the glyph on screen
    let scale = poi.display_scale;
    let local_px_x = (px_x + corner_x * glyph_w_px) * scale;
    let local_px_y = (px_y - corner_y * glyph_h_px) * scale;

    // Convert pixel offset to clip-space offset (screen-aligned billboard)
    let clip_dx = local_px_x * 2.0 / poi.viewport_width * anchor_clip.w;
    let clip_dy = local_px_y * 2.0 / poi.viewport_height * anchor_clip.w;

    // Interpolate UV
    let uv = vec2<f32>(
        mix(inst_uv.x, inst_uv.z, corner_x),
        mix(inst_uv.y, inst_uv.w, corner_y),
    );

    // Place labels at a fixed near depth so they are never occluded by
    // terrain or buildings.  Under perspective, w = -z_view (large) and
    // the depth buffer is highly non-linear, so near_z = 0.01*w maps to
    // z_ndc = 0.01 — safely in front of all geometry.  Under ortho, w = 1
    // and depth is LINEAR, so terrain sits very close to z_ndc ≈ 0 and
    // we need an even smaller value to stay in front.
    let near_z = select(0.01, 0.0001, anchor_clip.w < 1.5) * anchor_clip.w;

    // Cull labels behind the camera: under perspective w<0 pushes them
    // offscreen, but under ortho w=1 always so we must check z explicitly.
    // Set w=0 to collapse the vertex to a degenerate triangle.
    let behind = anchor_clip.z < 0.0;
    let cull_w = select(anchor_clip.w, 0.0, behind);
    var out: VsOut;
    out.pos = vec4<f32>(anchor_clip.x + clip_dx, anchor_clip.y + clip_dy, near_z, cull_w);
    out.uv = uv;
    return out;
}

@fragment fn fs(
    @location(0) uv: vec2<f32>,
) -> @location(0) vec4<f32> {
    let sdf = textureSample(font_tex, font_samp, uv).r;

    // SDF rendering: 0.5 (=128/255) is the edge
    let edge = 0.5;
    // Clamp width: fwidth() can return very large values at quad edges
    // where screen-space derivatives are undefined, causing the smoothstep
    // to widen enough that SDF=0 (padding region) produces visible alpha.
    let width = min(fwidth(sdf) * 0.7, 0.1);

    // Halo edge: smaller edge = wider halo around the glyph
    let halo_edge = edge - poi.halo_width * 0.1;
    let alpha = smoothstep(halo_edge - width, halo_edge + width, sdf);

    if (alpha < 0.01) {
        discard;
    }

    // Blend from halo color to fill color at the glyph edge
    let text_alpha = smoothstep(edge - width, edge + width, sdf);
    let color = mix(poi.halo_color.rgb, poi.fill_color.rgb, text_alpha);
    let out = select(color, pow(color, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    // Premultiplied alpha output — prevents white seams where adjacent
    // glyph quads overlap (halo blending on top of fill).
    return vec4<f32>(out * alpha, alpha);
}
