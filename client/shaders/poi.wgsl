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
    glyph_scale: f32,   /* world-space meters per font unit */
    atlas_size: f32,    /* atlas texture size in pixels */
    _pad1: f32,
    _pad2: f32,
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

    // ENU basis at instance position
    let slat = sin(inst_lat);
    let clat = cos(inst_lat);
    let slon = sin(inst_lon);
    let clon = cos(inst_lon);
    let east  = vec3<f32>(-slon, clon, 0.0);
    let north = vec3<f32>(-slat * clon, -slat * slon, clat);
    let up    = vec3<f32>(clat * clon, clat * slon, slat);

    // Glyph size in atlas pixels (derived from UV rect)
    let glyph_w_px = (inst_uv.z - inst_uv.x) * poi.atlas_size;
    let glyph_h_px = (inst_uv.w - inst_uv.y) * poi.atlas_size;

    // Position: inst_offset is in normalized font units
    let gs = poi.glyph_scale;
    let base_x = inst_offset.x * gs;
    let base_y = inst_offset.y * gs;

    // Quad size in world meters (proportional to actual glyph pixel size)
    let quad_w = glyph_w_px / poi.atlas_size * gs * 2.5;
    let quad_h = glyph_h_px / poi.atlas_size * gs * 2.5;

    let local_x = base_x + corner_x * quad_w;
    let local_y = base_y - corner_y * quad_h;  // Y down in glyph space

    // Apply as ENU offset (east=right, up=above)
    let ecef_offset = east * local_x + up * local_y;

    let inst_ecef = geodetic_to_ecef(inst_lon, inst_lat, inst_alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let local_ecef = (inst_ecef + ecef_offset) - center_ecef;

    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);

    // Interpolate UV
    let uv = vec2<f32>(
        mix(inst_uv.x, inst_uv.z, corner_x),
        mix(inst_uv.y, inst_uv.w, corner_y),
    );

    var out: VsOut;
    out.pos = globals.projection * world_pos;
    out.uv = uv;
    return out;
}

@fragment fn fs(
    @location(0) uv: vec2<f32>,
) -> @location(0) vec4<f32> {
    let sdf = textureSample(font_tex, font_samp, uv).r;

    // SDF rendering: 0.5 (=128/255) is the edge
    let edge = 0.5;
    let width = fwidth(sdf) * 0.7;
    let alpha = smoothstep(edge - width, edge + width, sdf);

    if (alpha < 0.01) {
        discard;
    }

    // White text with dark outline for readability
    let text_color = vec3<f32>(1.0, 1.0, 1.0);
    let outline_color = vec3<f32>(0.1, 0.1, 0.15);
    let outline_edge = 0.35;
    let outline_alpha = smoothstep(outline_edge - width, outline_edge + width, sdf);

    let color = mix(outline_color, text_color, outline_alpha);
    let out = select(color, pow(color, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    return vec4<f32>(out, alpha);
}
