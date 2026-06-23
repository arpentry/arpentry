// Draped road geometry: terrain-conforming SDF stroke quads drawn as 3D
// geometry in the main scene pass (not rasterized into the surface texture),
// so road edges stay vector-crisp at any zoom.  The vertex transform mirrors
// terrain.wgsl (tile quantized coords -> geodetic -> ECEF -> world -> clip);
// the fragment reuses the signed-distance antialiasing of line.wgsl.

const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;

struct GlobalUniforms {
    projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    apply_gamma: f32,
    altitude: f32,
    _pad0: f32,
    _pad1: f32,
    _pad2: f32,
};

struct TileUniforms {
    model: mat4x4<f32>,
    bounds: vec4<f32>,
    center_lon: f32,
    center_lat: f32,
    _pad0: f32,
    _pad1: f32,
};

@group(0) @binding(0) var<uniform> globals: GlobalUniforms;
@group(1) @binding(0) var<uniform> tile: TileUniforms;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) local: vec2<f32>,
    @location(2) hw_len: vec2<f32>,
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

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) color: vec4<f32>,
    @location(3) local: vec2<f32>,
    @location(4) hw_len: vec2<f32>,
) -> VsOut {
    let lon_west = tile.bounds.x;
    let lat_south = tile.bounds.y;
    let lon_east = tile.bounds.z;
    let lat_north = tile.bounds.w;

    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    let lon = lon_west + u * (lon_east - lon_west);
    let lat = lat_south + v * (lat_north - lat_south);
    // Lift roads a couple of metres above the terrain so they read as a decal
    // sitting on the ground: enough to avoid z-fighting with the surface, small
    // enough that a hill still occludes roads behind it (no see-through).
    let alt = f32(qz) * 0.001 + 2.0;

    let ecef = geodetic_to_ecef(lon, lat, alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let world_pos = tile.model * vec4<f32>(ecef - center_ecef, 1.0);

    var out: VsOut;
    out.pos = globals.projection * world_pos;
    out.color = color;
    out.local = local;
    out.hw_len = hw_len;
    return out;
}

@fragment fn fs(
    @location(0) color: vec4<f32>,
    @location(1) local: vec2<f32>,
    @location(2) hw_len: vec2<f32>,
) -> @location(0) vec4<f32> {
    let hw = hw_len.x;
    let seg_len = hw_len.y;
    let cx = clamp(local.x, 0.0, seg_len);
    let dist = length(vec2<f32>(local.x - cx, local.y));
    let px = length(vec2<f32>(dpdx(local.y), dpdy(local.y)));
    let alpha = color.a * (1.0 - smoothstep(hw - px, hw, dist));
    return vec4<f32>(color.rgb, alpha);
}
