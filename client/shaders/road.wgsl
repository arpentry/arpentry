// Draped road geometry: terrain-conforming SDF stroke quads drawn as 3D
// geometry in the main scene pass (not rasterized into the surface texture),
// so road edges stay vector-crisp at any zoom.  The vertex transform mirrors
// terrain.wgsl (tile quantized coords -> geodetic -> ECEF -> world -> clip);
// the fragment reuses the signed-distance antialiasing of line.wgsl.

const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;

// How far, in metres, a road is biased toward the camera along the view ray so it
// wins the depth test against terrain that rises above it, per tile
// (`tile.stroke_margin_m`, set by the tile manager). On the coarse rungs the
// grade-limited road (server `structures::limit_road_grade`) holds an engineered
// grade and so cuts a few metres below the coarse terrain mesh where it crosses a
// steep flank; without the 12 m it would be occluded (buried) by ground drawn in
// front of it — sized to the shallow cuttings the limiter carves, large enough
// to surface them, small enough that a genuine hill still occludes a road behind
// it. On the detail rung the ground is cut away under the pavement, so the
// margin drops to the stack epsilon and paint sits tight on its deck.

// The margin as a fraction of the viewing distance, capping the absolute
// margin up close: a fixed 12 m is nothing against a 5 km map view but a
// fifth of a 65 m street-level view — strokes would X-ray through cutting
// walls and rises that genuinely stand in front of them. 3% of the distance
// keeps the full margin beyond 400 m and shrinks it smoothly below.
const ROAD_DEPTH_MARGIN_FRAC: f32 = 0.03;

struct GlobalUniforms {
    projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    apply_gamma: f32,
    altitude: f32,
    viewport_w: f32,
    viewport_h: f32,
    _pad2: f32,
};

// Minimum on-screen half-width of a road stroke, in framebuffer pixels. A flat
// ribbon lying on a surface foreshortens to nothing at a grazing view; below
// this floor the edge vertices are pushed apart in screen space so the stroke
// keeps a visible width on decks and terrain alike. Only shrinks strokes never
// widens ones already broader than the floor, so head-on roads are unchanged.
const MIN_HALF_WIDTH_PX: f32 = 2.0;

// Strokes physically narrower than this half-width (metres) are painted
// detail — centre/edge/lane markings, 6–8 cm — not roads. They get their own
// (thinner) screen-space floor below rather than the road floor.
const DETAIL_HALF_WIDTH_M: f32 = 0.2;

// Minimum on-screen half-width of a painted marking, in framebuffer pixels.
// A 12 cm marking is sub-pixel at the zooms markings first appear, so left at
// its true width it renders as a ragged, flickering sliver through MSAA. Held
// to this floor it stays a crisp thin line (thinner than the MIN_HALF_WIDTH_PX
// road floor so a dash doesn't read as a road-wide bar). Beyond the floor a
// marking that is genuinely sub-pixel fades by its true coverage so distant
// dashes thin away like ink rather than holding a hard bar.
const MARK_MIN_HALF_WIDTH_PX: f32 = 1.0;

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
    // Depth-only camera bias for this tile's strokes, metres (see above).
    stroke_margin_m: f32,
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

// sin for tile-relative angle deltas — see terrain.wgsl's sin_delta: the
// fast-math native sin() quantizes small arguments coarsely enough to put a
// metre-scale staircase on every edge once scaled by the earth radius.
fn sin_delta(x: f32) -> f32 {
    let x2 = x * x;
    let taylor = x * (1.0 - x2 * (1.0 / 6.0) * (1.0 - x2 * 0.05));
    return select(sin(x), taylor, abs(x) < 0.25);
}

// ECEF offset from the tile center without forming absolute ECEF values —
// see terrain.wgsl's local_ecef_delta for the full derivation. Absolute f32
// ECEF rounds at ~0.5 m and scallops straight road edges.
fn local_ecef_delta(dlam: f32, dphi: f32, alt: f32) -> vec3<f32> {
    let slc = tile.sincos.x;
    let clc = tile.sincos.y;
    let spc = tile.sincos.z;
    let cpc = tile.sincos.w;
    let sdl = sin_delta(dlam);
    let hdl = sin_delta(dlam * 0.5);
    let cdl_m1 = -2.0 * hdl * hdl;
    let sdp = sin_delta(dphi);
    let hdp = sin_delta(dphi * 0.5);
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

fn tile_to_world(qx: f32, qy: f32, alt: f32) -> vec4<f32> {
    let u = (qx - 16384.0) / 32768.0;
    let v = (qy - 16384.0) / 32768.0;
    let dlam = tile.rel_bounds.x + u * (tile.rel_bounds.z - tile.rel_bounds.x);
    let dphi = tile.rel_bounds.y + v * (tile.rel_bounds.w - tile.rel_bounds.y);
    return tile.model * vec4<f32>(local_ecef_delta(dlam, dphi, alt), 1.0);
}

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) color: vec4<f32>,
    @location(3) local: vec2<f32>,
    @location(4) hw_len: vec2<f32>,
    @location(5) cxy: vec2<u32>,
) -> VsOut {
    let alt = f32(qz) * 0.001;
    let world_pos = tile_to_world(f32(qxy.x), f32(qxy.y), alt);

    // Bias the stroke toward the camera by a fixed world margin so it wins the
    // depth test against terrain up to `tile.stroke_margin_m` in front of it
    // — surfacing the shallow cuttings the grade-limited road carves — while ground
    // deeper than that (a real hill) still occludes a road behind it.  The tile
    // model is view-aligned (the eye at the origin, looking down -z), so moving the
    // vertex +margin along z reduces its depth by exactly the margin everywhere on
    // screen.  (Scaling toward the origin instead would shorten the margin for
    // off-centre roads, where the view ray is far from the depth axis, so a cutting
    // near the screen edge would stay buried.)  The bias is DEPTH-ONLY: the depth
    // test uses the shifted point, but the projected screen position keeps the
    // true vertex — moving the position itself changes the projection, and at
    // street-level altitudes a 12 m shift visibly slides the paint off its deck
    // and parallaxes as the camera moves.
    var shifted = world_pos;
    shifted.z += min(tile.stroke_margin_m, length(world_pos.xyz) * ROAD_DEPTH_MARGIN_FRAC);

    var out: VsOut;
    let p = globals.projection * world_pos;
    let ps = globals.projection * shifted;

    // Screen-space width floor: project the centerline this vertex is offset
    // from, measure the on-screen offset in pixels, and if it has foreshortened
    // below MIN_HALF_WIDTH_PX push the vertex back out to the floor. The metric
    // `local` SDF coords are unchanged, so the fragment's antialiased edge just
    // rides the widened quad — a grazing ribbon stays a crisp, visible stroke
    // instead of collapsing to a sub-pixel sliver. Keep the true position when
    // p.w <= 0 (behind the eye) so the clip stays well-formed.
    var screen_xy = p.xy;
    var fade = 1.0;
    if (p.w > 1e-4) {
        let pc = globals.projection * tile_to_world(f32(cxy.x), f32(cxy.y), alt);
        if (pc.w > 1e-4) {
            let vp = vec2<f32>(globals.viewport_w, globals.viewport_h);
            let c_ndc = pc.xy / pc.w;
            let v_ndc = p.xy / p.w;
            let off_px = (v_ndc - c_ndc) * 0.5 * vp;
            let len_px = length(off_px);
            if (hw_len.x < DETAIL_HALF_WIDTH_M) {
                // Painted detail: hold a crisp thin on-screen width so a
                // sub-pixel dash renders as a clean line instead of a ragged
                // sliver. Widen the quad to the marking floor (the SDF fills it
                // because the edge verts keep their metric hw, so `dist` still
                // reaches hw at the widened edge). Fade only for marks far below
                // the floor so very distant dashes thin away like ink.
                if (len_px > 1e-4 && len_px < MARK_MIN_HALF_WIDTH_PX) {
                    let widened = c_ndc + (v_ndc - c_ndc) * (MARK_MIN_HALF_WIDTH_PX / len_px);
                    screen_xy = widened * p.w;
                    fade = clamp(len_px / MARK_MIN_HALF_WIDTH_PX + 0.5, 0.5, 1.0);
                }
            } else if (len_px > 1e-4 && len_px < MIN_HALF_WIDTH_PX) {
                let widened = c_ndc + (v_ndc - c_ndc) * (MIN_HALF_WIDTH_PX / len_px);
                screen_xy = widened * p.w;
            }
        }
    }

    out.pos = vec4<f32>(screen_xy, ps.z / ps.w * p.w, p.w);
    out.color = vec4<f32>(color.rgb, color.a * fade);
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
    // Antialias from the distance field itself: fwidth(dist) is the per-pixel
    // change of the SDF along BOTH screen axes, so it stays a true one-pixel
    // band however the stroke foreshortens. The old `dpdx/dpdy(local.y)` term
    // saw only the across-stroke coordinate; where a stroke turns to run down
    // the view ray (a curving deck seen at a grazing tilt) that gradient spikes
    // past `hw`, and `smoothstep(hw - px, hw, dist)` then dims the whole stroke
    // — centerline included — so it faded out on exactly the far, angled spans.
    let aa = max(fwidth(dist), 1e-4);
    // Linear (analytic) coverage of the stroke edge rather than a cubic
    // smoothstep: a stroke thinner than a pixel holds its real coverage instead
    // of collapsing to zero, so paint and markings stay visible down the deck.
    let alpha = color.a * clamp((hw - dist) / aa + 0.5, 0.0, 1.0);
    return vec4<f32>(color.rgb, alpha);
}
