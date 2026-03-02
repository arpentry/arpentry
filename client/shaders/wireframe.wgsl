const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;

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

@group(0) @binding(0) var<uniform> globals: GlobalUniforms;
@group(1) @binding(0) var<uniform> tile: TileUniforms;

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

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) dist: f32,
};

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) dist: f32,
) -> VsOut {
    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    let lon = tile.bounds.x + u * (tile.bounds.z - tile.bounds.x);
    let lat = tile.bounds.y + v * (tile.bounds.w - tile.bounds.y);
    let alt = f32(qz) * 0.001;
    let ecef = geodetic_to_ecef(lon, lat, alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let local_ecef = ecef - center_ecef;
    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);
    var out: VsOut;
    out.pos = globals.projection * world_pos;
    out.dist = dist;
    return out;
}

@fragment fn fs(
    @location(0) dist: f32,
) -> @location(0) vec4<f32> {
    let hw_px = 0.75; // half-width in screen pixels
    let aa = fwidth(dist);
    let hw = hw_px * aa;
    let alpha = 1.0 - smoothstep(hw, hw + aa, abs(dist));
    let color = vec3<f32>(0.25, 0.28, 0.33);
    let out = select(color, pow(color, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    return vec4<f32>(out, alpha);
}
