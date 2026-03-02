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
@group(1) @binding(1) var surface_tex: texture_2d<f32>;
@group(1) @binding(2) var surface_samp: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) normal_cam: vec3<f32>,
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

fn decode_octahedral(enc: vec2<f32>) -> vec3<f32> {
    var n = vec3<f32>(enc.x, enc.y, 1.0 - abs(enc.x) - abs(enc.y));
    if (n.z < 0.0) {
        let old = n.xy;
        n.x = (1.0 - abs(old.y)) * sign(old.x);
        n.y = (1.0 - abs(old.x)) * sign(old.y);
    }
    return normalize(n);
}

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) oct_norm: vec2<i32>,
) -> VsOut {
    let lon_west = tile.bounds.x;
    let lat_south = tile.bounds.y;
    let lon_east = tile.bounds.z;
    let lat_north = tile.bounds.w;

    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    let lon = lon_west + u * (lon_east - lon_west);
    let lat = lat_south + v * (lat_north - lat_south);
    let alt = f32(qz) * 0.001;

    let ecef = geodetic_to_ecef(lon, lat, alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let local_ecef = ecef - center_ecef;

    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);

    var out: VsOut;
    out.pos = globals.projection * world_pos;
    out.uv = vec2<f32>(u, v);

    let enc = vec2<f32>(f32(oct_norm.x) / 127.0, f32(oct_norm.y) / 127.0);
    let obj_normal = decode_octahedral(enc);
    let model3 = mat3x3<f32>(tile.model[0].xyz, tile.model[1].xyz, tile.model[2].xyz);
    out.normal_cam = normalize(model3 * obj_normal);

    return out;
}

@fragment fn fs(
    @location(0) uv: vec2<f32>,
    @location(1) normal_cam: vec3<f32>,
) -> @location(0) vec4<f32> {
    let margin = 0.0625;
    let tex_uv = (uv + vec2<f32>(margin, margin)) / (1.0 + 2.0 * margin);
    let albedo = textureSample(surface_tex, surface_samp, tex_uv).rgb;

    let n = normalize(normal_cam);
    let sun = normalize(globals.sun_dir);
    let NdotL = dot(n, sun);

    // Hemisphere ambient: cool blue in shadow, warm fill on lit side
    let shadow_color = vec3<f32>(0.20, 0.22, 0.28);
    let fill_color   = vec3<f32>(0.28, 0.26, 0.22);
    let hemi_t = NdotL * 0.5 + 0.5;
    let ambient = mix(shadow_color, fill_color, hemi_t);

    // Direct sunlight: clamped Lambertian at moderate intensity
    let sun_color = vec3<f32>(0.65, 0.63, 0.58);
    let direct = sun_color * max(NdotL, 0.0);

    let lit = albedo * (ambient + direct);

    // Apply sRGB gamma when surface format is non-sRGB (e.g. WebGPU in browser)
    let out = select(lit, pow(lit, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    return vec4<f32>(out, 1.0);
}
