// Draped road geometry: terrain-conforming SDF stroke quads drawn as 3D
// geometry in the main scene pass (not rasterized into the surface texture),
// so road edges stay vector-crisp at any zoom.  The vertex transform mirrors
// terrain.wgsl (tile quantized coords -> geodetic -> ECEF -> world -> clip);
// the fragment reuses the signed-distance antialiasing of line.wgsl.

const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;

// How far, in metres, a road is biased toward the camera along the view ray so it
// wins the depth test against terrain that rises above it.  The grade-limited road
// (server `structures::limit_road_grade`) holds an engineered grade and so cuts a
// few metres below the coarse terrain mesh where it crosses a steep flank; without
// this it would be occluded (buried) by ground drawn in front of it.  Sized to the
// shallow cuttings the limiter carves — large enough to surface them, small enough
// that a genuine hill (deeper than this in front) still occludes a road behind it.
const ROAD_DEPTH_MARGIN_M: f32 = 12.0;

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
    // Bounds relative to the center (radians): west−λc, south−φc, east−λc,
    // north−φc. Small, so f32 carries them to sub-mm.
    rel_bounds: vec4<f32>,
    // sin λc, cos λc, sin φc, cos φc — computed in double on the CPU.
    sincos: vec4<f32>,
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

// ECEF offset from the tile center without forming absolute ECEF values —
// see terrain.wgsl's local_ecef_delta for the full derivation. Absolute f32
// ECEF rounds at ~0.5 m and scallops straight road edges.
fn local_ecef_delta(dlam: f32, dphi: f32, alt: f32) -> vec3<f32> {
    let slc = tile.sincos.x;
    let clc = tile.sincos.y;
    let spc = tile.sincos.z;
    let cpc = tile.sincos.w;
    let sdl = sin(dlam);
    let hdl = sin(dlam * 0.5);
    let cdl_m1 = -2.0 * hdl * hdl;
    let sdp = sin(dphi);
    let hdp = sin(dphi * 0.5);
    let cdp_m1 = -2.0 * hdp * hdp;
    let dsp = cpc * sdp + spc * cdp_m1;
    let sp = spc + dsp;
    let cp = cpc + (cpc * cdp_m1 - spc * sdp);
    let wc = sqrt(1.0 - WGS84_E2 * spc * spc);
    let w = sqrt(1.0 - WGS84_E2 * sp * sp);
    let n = WGS84_A / w;
    let dn = WGS84_A * WGS84_E2 * dsp * (sp + spc) / (w * wc * (w + wc));
    let a_full = (n + alt) * cp;
    let ncd = dn + n * cdp_m1;
    let da = cpc * ncd - n * spc * sdp + alt * cp;
    let t1 = da + a_full * cdl_m1;
    return vec3<f32>(
        t1 * clc - a_full * sdl * slc,
        t1 * slc + a_full * sdl * clc,
        (1.0 - WGS84_E2) * (spc * ncd + n * cpc * sdp) + alt * sp,
    );
}

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) color: vec4<f32>,
    @location(3) local: vec2<f32>,
    @location(4) hw_len: vec2<f32>,
) -> VsOut {
    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    let dlam = tile.rel_bounds.x + u * (tile.rel_bounds.z - tile.rel_bounds.x);
    let dphi = tile.rel_bounds.y + v * (tile.rel_bounds.w - tile.rel_bounds.y);
    let alt = f32(qz) * 0.001;

    var world_pos = tile.model * vec4<f32>(local_ecef_delta(dlam, dphi, alt), 1.0);

    // Bias the stroke toward the camera by a fixed world margin so it wins the
    // LessEqual depth test against terrain up to ROAD_DEPTH_MARGIN_M in front of it
    // — surfacing the shallow cuttings the grade-limited road carves — while ground
    // deeper than that (a real hill) still occludes a road behind it.  The tile
    // model is view-aligned (the eye at the origin, looking down -z), so moving the
    // vertex +margin along z reduces its depth by exactly the margin everywhere on
    // screen.  (Scaling toward the origin instead would shorten the margin for
    // off-centre roads, where the view ray is far from the depth axis, so a cutting
    // near the screen edge would stay buried.)  The shift is tiny next to the view
    // distance, so the road neither visibly floats nor z-fights.
    world_pos.z += ROAD_DEPTH_MARGIN_M;

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
