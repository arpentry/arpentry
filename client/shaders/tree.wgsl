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

@group(0) @binding(0) var<uniform> globals: GlobalUniforms;
@group(1) @binding(0) var<uniform> tile: TileUniforms;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) normal_cam: vec3<f32>,
    @location(1) color: vec3<f32>,
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
    // Per-vertex: model-local position (int16x4 unpacked as sint16x4)
    @location(0) model_pos: vec4<i32>,
    // Per-instance: tile-quantized position + yaw/scale packed
    @location(1) inst_qxy: vec2<u32>,
    @location(2) inst_qz: i32,
    @location(3) inst_yaw_scale: f32,
) -> VsOut {
    // Decode yaw and scale from packed float
    // Integer part = yaw index (0-255), fractional part = scale_01 (0-1)
    let packed = inst_yaw_scale;
    let yaw_part = floor(packed);
    let scale_01 = packed - yaw_part;
    let yaw = yaw_part / 256.0 * 2.0 * PI;
    // Reconstruct actual scale: hardcoded range matching style defaults
    // (min_scale=0.7, max_scale=1.3 from style.json)
    let scale = 0.7 + scale_01 * 0.6;

    // Model position in mm -> meters
    let mx = f32(model_pos.x) * 0.001 * scale;
    let my = f32(model_pos.y) * 0.001 * scale;
    let mz = f32(model_pos.z) * 0.001 * scale;

    // Rotate model around Z axis by yaw
    let cy = cos(yaw);
    let sy = sin(yaw);
    let rx = mx * cy - my * sy;
    let ry = mx * sy + my * cy;
    let rz = mz;

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

    // Compute ENU basis at instance position
    let slat = sin(inst_lat);
    let clat = cos(inst_lat);
    let slon = sin(inst_lon);
    let clon = cos(inst_lon);
    let east  = vec3<f32>(-slon, clon, 0.0);
    let north = vec3<f32>(-slat * clon, -slat * slon, clat);
    let up    = vec3<f32>(clat * clon, clat * slon, slat);

    // Transform rotated model offset from ENU to ECEF
    let ecef_offset = east * rx + north * ry + up * rz;

    // Instance ECEF position
    let inst_ecef = geodetic_to_ecef(inst_lon, inst_lat, inst_alt);
    let center_ecef = geodetic_to_ecef(tile.center_lon, tile.center_lat, 0.0);
    let local_ecef = (inst_ecef + ecef_offset) - center_ecef;

    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);

    // Approximate normal: for a cone, use the model vertex direction
    // Simplified: compute upward-biased normal from model position
    let model_r = sqrt(mx * mx + my * my);
    var obj_normal: vec3<f32>;
    if (model_pos.z > 0 && model_r < 0.001) {
        // Apex: point up
        obj_normal = up;
    } else if (model_r > 0.001) {
        // Cone surface: outward + upward
        let out_dir = east * (mx / model_r) + north * (my / model_r);
        obj_normal = normalize(out_dir * 0.8 + up * 0.6);
    } else {
        // Base: point down
        obj_normal = -up;
    }
    let model3 = mat3x3<f32>(tile.model[0].xyz, tile.model[1].xyz, tile.model[2].xyz);
    let normal_cam = normalize(model3 * obj_normal);

    var out: VsOut;
    out.pos = globals.projection * world_pos;
    out.normal_cam = normal_cam;
    out.color = vec3<f32>(40.0 / 255.0, 90.0 / 255.0, 30.0 / 255.0);
    return out;
}

@fragment fn fs(
    @location(0) normal_cam: vec3<f32>,
    @location(1) color: vec3<f32>,
) -> @location(0) vec4<f32> {
    let n = normalize(normal_cam);
    let sun = normalize(globals.sun_dir);
    let NdotL = dot(n, sun);

    // Hemisphere ambient
    let shadow_color = vec3<f32>(0.20, 0.22, 0.28);
    let fill_color   = vec3<f32>(0.28, 0.26, 0.22);
    let hemi_t = NdotL * 0.5 + 0.5;
    let ambient = mix(shadow_color, fill_color, hemi_t);

    // Direct sunlight
    let sun_color = vec3<f32>(0.65, 0.63, 0.58);
    let direct = sun_color * max(NdotL, 0.0);

    let lit = color * (ambient + direct);
    let out = select(lit, pow(lit, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    return vec4<f32>(out, 1.0);
}
