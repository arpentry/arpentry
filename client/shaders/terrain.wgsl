const WGS84_A: f32 = 6378137.0;
const WGS84_E2: f32 = 0.00669437999014;

// Terrain opacity for the x-ray debug pipeline (1.0 = opaque). Lower it to see
// buried tunnel boxes through the ground more clearly.
const TERRAIN_ALPHA: f32 = 0.6;

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
@group(1) @binding(1) var surface_tex: texture_2d<f32>;
@group(1) @binding(2) var surface_samp: sampler;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) normal_cam: vec3<f32>,
    @location(2) view_pos: vec3<f32>,
    // Alignment of the vertex normal with the tile-centre vertical (ECEF up):
    // ~1 on an up-facing face (a bridge deck top), ~0 on a side wall. Lets the
    // deck fragment paint the road surface onto the top face itself, so the
    // road stays visible at grazing angles where the flat paint ribbon drawn
    // over the deck foreshortens to sub-pixel and drops out.
    @location(3) topness: f32,
    // Per-vertex road-class asphalt colour for the structure top (fs_deck). The
    // client resolves it from the same style entry the ribbon uses, so a
    // secondary/residential deck reads its own grey, not a hardcoded motorway
    // one. Unused by the terrain/building fragment (fs).
    @location(4) deck_color: vec4<f32>,
    // Signed across-carriageway coordinate: ±1 at the paved edge, 0 at the
    // centre (the interpolated midpoint of a band's edge pair). fs_deck fades
    // the outer ~1px of a drivable surface from `1 - |across|` for a crisp
    // analytic edge. 0 on every non-drivable face → no AA (MSAA).
    @location(5) across: f32,
};

// sin for tile-relative angle deltas. The native sin() is compiled with fast
// math and carries an *absolute* error orders of magnitude above the f32 ulp
// of a small argument; at the ~1e-4 rad deltas a street-level tile feeds in,
// that error scales by the ~6.4e6 m earth radius into a metre-scale, world-
// anchored staircase on every edge. A 5th-order Taylor is exact to ~2 ulp for
// |x| < 0.25 (every tile from about z5 in); the native path takes over on
// globe-scale tiles, where its absolute error is far below a pixel.
fn sin_delta(x: f32) -> f32 {
    let x2 = x * x;
    let taylor = x * (1.0 - x2 * (1.0 / 6.0) * (1.0 - x2 * 0.05));
    return select(sin(x), taylor, abs(x) < 0.25);
}

// ECEF offset of the vertex at tile-relative (dλ, dφ) radians and altitude
// `alt` from the tile center's ECEF at altitude 0 — computed *without ever
// forming an absolute ECEF coordinate*. Absolute ECEF is ~6.4e6 m, where an
// f32 ulp is ~0.5 m; subtracting two such values scallops every straight
// edge onto a half-metre lattice. Here every term is either a small
// quantity or a large quantity scaled by a small one, so the result is
// accurate to well under a millimetre across a tile. The trigonometric
// identities are exact, so the formulation stays valid (merely reverting to
// f32-level accuracy) even for globe-scale tiles.
fn local_ecef_delta(dlam: f32, dphi: f32, alt: f32) -> vec3<f32> {
    let slc = tile.sincos.x; // sin λc
    let clc = tile.sincos.y; // cos λc
    let spc = tile.sincos.z; // sin φc
    let cpc = tile.sincos.w; // cos φc
    let sdl = sin_delta(dlam);
    let hdl = sin_delta(dlam * 0.5);
    let cdl_m1 = -2.0 * hdl * hdl; // cos(dλ) − 1, computed without cancellation
    let sdp = sin_delta(dphi);
    let hdp = sin_delta(dphi * 0.5);
    let cdp_m1 = -2.0 * hdp * hdp; // cos(dφ) − 1
    let dsp = cpc * sdp + spc * cdp_m1; // sin φ − sin φc
    let sp = spc + dsp;                 // sin φ
    let cp = cpc + (cpc * cdp_m1 - spc * sdp); // cos φ
    let wc = sqrt(1.0 - WGS84_E2 * spc * spc);
    let w = sqrt(1.0 - WGS84_E2 * sp * sp);
    let n = WGS84_A / w; // prime-vertical radius N(φ)
    // N(φ) − N(φc), expanded so no ~6.4e6 m values are subtracted.
    let dn = WGS84_A * WGS84_E2 * dsp * (sp + spc) / (w * wc * (w + wc));
    let a_full = (n + alt) * cp;                            // (N+h)·cos φ
    let ncd = dn + n * cdp_m1;                              // N·cos dφ − Nc
    let da = cpc * ncd - n * spc * sdp + alt * cp;          // (N+h)cosφ − Nc·cosφc
    let t1 = da + a_full * cdl_m1;                          // A·cos dλ − Ac
    return vec3<f32>(
        t1 * clc - a_full * sdl * slc,
        t1 * slc + a_full * sdl * clc,
        (1.0 - WGS84_E2) * (spc * ncd + n * cpc * sdp) + alt * sp,
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

// Depth margin for bridge decks. An approach deck rides the engineered
// roadbed the ground stage carves at the same height, so deck and terrain
// are near-coplanar for long stretches; the depth test then draws their
// jagged, lattice-scale intersection contour as the apparent deck edge.
// Biasing the deck a few metres toward the camera (exactly the road paint's
// trick, see road.wgsl) makes the deck win those ties, so the visible edge
// is its own smooth geometry. Small enough that ground genuinely above the
// deck (a real hill, a buried run) still occludes it.
const BRIDGE_DEPTH_MARGIN_M: f32 = 3.0;

// The deck margin as a fraction of the viewing distance — a quarter of the
// road paint's fraction (road.wgsl), so the paint re-emitted over a deck
// always carries the larger margin and wins their coplanar tie at any range,
// while neither X-rays through terrain up close.
const BRIDGE_DEPTH_MARGIN_FRAC: f32 = 0.0075;

fn vs_common(qxy: vec2<u32>, qz: i32, oct_norm: vec2<i32>, depth_margin: f32,
             deck_color: vec4<f32>, across: f32) -> VsOut {
    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    let dlam = tile.rel_bounds.x + u * (tile.rel_bounds.z - tile.rel_bounds.x);
    let dphi = tile.rel_bounds.y + v * (tile.rel_bounds.w - tile.rel_bounds.y);
    let alt = f32(qz) * 0.001;

    let local_ecef = local_ecef_delta(dlam, dphi, alt);

    let world_pos = tile.model * vec4<f32>(local_ecef, 1.0);
    // Depth-only camera bias: the depth test compares against the vertex
    // moved `depth_margin` toward the camera (the tile model is view-aligned,
    // so +z is exactly that), but the projected screen position keeps the
    // TRUE vertex — shifting the position itself changes the projection and
    // makes biased geometry visibly parallax off its neighbours up close.
    // Splice the shifted point's depth (ps.z/ps.w, rescaled onto the true
    // clip w) into the true clip position.
    var out: VsOut;
    var p = globals.projection * world_pos;
    if (depth_margin != 0.0) {
        var shifted = world_pos;
        shifted.z += min(depth_margin, length(world_pos.xyz) * BRIDGE_DEPTH_MARGIN_FRAC);
        let ps = globals.projection * shifted;
        p = vec4<f32>(p.xy, ps.z / ps.w * p.w, p.w);
    }
    out.pos = p;
    out.uv = vec2<f32>(u, v);
    let enc = vec2<f32>(f32(oct_norm.x) / 127.0, f32(oct_norm.y) / 127.0);
    let obj_normal = decode_octahedral(enc);
    let model3 = mat3x3<f32>(tile.model[0].xyz, tile.model[1].xyz, tile.model[2].xyz);
    out.normal_cam = normalize(model3 * obj_normal);
    out.view_pos = world_pos.xyz;

    // ECEF up at the tile centre (normals are encoded in ECEF); its dot with
    // the object normal is ~1 on an up-facing face. sincos = (sinλc, cosλc,
    // sinφc, cosφc).
    let up_ecef = vec3<f32>(
        tile.sincos.w * tile.sincos.y,
        tile.sincos.w * tile.sincos.x,
        tile.sincos.z);
    out.topness = dot(obj_normal, up_ecef);
    out.deck_color = deck_color;
    out.across = across;

    return out;
}

// Terrain + buildings: no depth margin, no deck colour (fs ignores it).
@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) oct_norm: vec2<i32>,
) -> VsOut {
    return vs_common(qxy, qz, oct_norm, 0.0, vec4<f32>(0.0), 0.0);
}

// Tunnel bores: carry the road-class colour, but NO depth margin — a buried
// bore must stay occluded by the ground, not biased toward the camera.
@vertex fn vs_deck(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) oct_norm: vec2<i32>,
    @location(3) deck_color: vec4<f32>,
    @location(5) across_in: vec2<f32>,
) -> VsOut {
    return vs_common(qxy, qz, oct_norm, 0.0, deck_color, across_in.x);
}

// Bridge decks: road-class colour plus the small camera-facing margin that wins
// the deck's coplanar depth tie with its engineered roadbed.
@vertex fn vs_deck_bridge(
    @location(0) qxy: vec2<u32>,
    @location(1) qz: i32,
    @location(2) oct_norm: vec2<i32>,
    @location(3) deck_color: vec4<f32>,
    @location(5) across_in: vec2<f32>,
) -> VsOut {
    return vs_common(qxy, qz, oct_norm, BRIDGE_DEPTH_MARGIN_M, deck_color, across_in.x);
}

@fragment fn fs(
    @location(0) uv: vec2<f32>,
    @location(1) normal_cam: vec3<f32>,
    @location(2) view_pos: vec3<f32>,
    @location(3) topness: f32,
    // Unused here (terrain/buildings), but declared so this fragment's input
    // interface matches VsOut — wgpu requires the vertex-output and
    // fragment-input location sets to agree.
    @location(4) deck_color: vec4<f32>,
    @location(5) across: f32,
) -> @location(0) vec4<f32> {
    let margin = 0.0625;
    let tex_uv = (uv + vec2<f32>(margin, margin)) / (1.0 + 2.0 * margin);
    let albedo_srgb = textureSample(surface_tex, surface_samp, tex_uv).rgb;

    // Ocean sun glint: detect water by color heuristic (on sRGB values
    // before linearisation so the thresholds stay intuitive)
    let lum = dot(albedo_srgb, vec3<f32>(0.299, 0.587, 0.114));
    let is_water = step(albedo_srgb.r * 2.0, albedo_srgb.b)
                 * step(albedo_srgb.g * 1.5, albedo_srgb.b)
                 * step(lum, 0.3);

    // Linearise sRGB texture so lighting math is done in linear space
    let albedo = pow(albedo_srgb, vec3<f32>(2.2));

    let n = normalize(normal_cam);
    let sun = normalize(globals.sun_dir);
    let NdotL = dot(n, sun);

    // Lighting tuned so fill_color + sun_color = 1.0:
    // fully sun-lit surfaces reproduce the exact style color.
    let shadow_color = vec3<f32>(0.45, 0.46, 0.52);
    let fill_color   = vec3<f32>(0.55, 0.54, 0.50);
    let hemi_t = NdotL * 0.5 + 0.5;
    let ambient = mix(shadow_color, fill_color, hemi_t);

    let sun_color = vec3<f32>(0.45, 0.46, 0.50);
    let direct = sun_color * max(NdotL, 0.0);

    var lit = albedo * (ambient + direct);

    if (is_water > 0.5) {
        // Blinn-Phong specular
        let view_dir = normalize(-view_pos);
        let half_vec = normalize(view_dir + sun);
        let NdotH = max(dot(n, half_vec), 0.0);

        // Broad glint (low shininess) + tight highlight (high shininess)
        let spec_broad = pow(NdotH, 120.0) * 0.08;
        let spec_tight = pow(NdotH, 1200.0) * 0.25;
        let spec = (spec_broad + spec_tight) * max(NdotL, 0.0);

        let glint_color = vec3<f32>(1.0, 0.97, 0.90);
        lit += glint_color * spec;
    }

    let out = select(lit, pow(lit, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);
    // Alpha only takes effect on a pipeline with blending enabled — only the
    // terrain x-ray pipeline does, so the terrain mesh renders slightly
    // transparent (to debug tunnels buried under it) while structures and
    // buildings, sharing this shader on opaque pipelines, stay solid.
    return vec4<f32>(out, TERRAIN_ALPHA);
}

// Fallback road-surface grey (the motorway stroke colour, style RGB
// 148,151,157 → /255) for a structure top whose class the client could not
// resolve. The common path uses the per-vertex `deck_color` instead, so a deck
// matches whatever grey its own class's ribbon emits.
const DECK_ASPHALT: vec3<f32> = vec3<f32>(0.5804, 0.5922, 0.6157);

// Structure-top fragment (bridge decks and tunnel bores): the up-facing top
// face *is* the road, so paint it the flat asphalt the ribbon uses — the
// per-vertex `deck_color` the client resolved from the road's own style entry,
// so the top matches its ribbon's class colour (not always motorway grey). The
// side walls, underside and portal faces keep the lit concrete texture. Mixed at
// the *output* level (not albedo) so the top face matches the unlit ribbon
// exactly. No water path — a structure is never water.
@fragment fn fs_deck(
    @location(0) uv: vec2<f32>,
    @location(1) normal_cam: vec3<f32>,
    @location(2) view_pos: vec3<f32>,
    @location(3) topness: f32,
    @location(4) deck_color: vec4<f32>,
    @location(5) across: f32,
) -> @location(0) vec4<f32> {
    let margin = 0.0625;
    let tex_uv = (uv + vec2<f32>(margin, margin)) / (1.0 + 2.0 * margin);
    let concrete_srgb = textureSample(surface_tex, surface_samp, tex_uv).rgb;
    let albedo = pow(concrete_srgb, vec3<f32>(2.2));

    let n = normalize(normal_cam);
    let sun = normalize(globals.sun_dir);
    let NdotL = dot(n, sun);
    let shadow_color = vec3<f32>(0.45, 0.46, 0.52);
    let fill_color   = vec3<f32>(0.55, 0.54, 0.50);
    let ambient = mix(shadow_color, fill_color, NdotL * 0.5 + 0.5);
    let direct = vec3<f32>(0.45, 0.46, 0.50) * max(NdotL, 0.0);
    let lit = albedo * (ambient + direct);
    let concrete_out = select(lit, pow(lit, vec3<f32>(1.0 / 2.2)), globals.apply_gamma > 0.5);

    // Top face → flat asphalt in the road's own class colour (or the motorway
    // fallback when the client shipped none); sides → lit concrete.
    let asphalt = select(DECK_ASPHALT, deck_color.rgb, deck_color.a > 0.0);
    let top = smoothstep(0.55, 0.80, topness);
    let out = mix(concrete_out, asphalt, top);
    // Analytic edge antialiasing on the drivable top face: `across` is ±1 at the
    // paved silhouette and 0 at the centre, so `1 - |across|` is the normalized
    // distance in from the edge. `fwidth(across)` is that coordinate's per-pixel
    // change, so `edge / fwidth` fades the outer ~1px into the ground — a crisp
    // 1px edge at any tilt, where a flat MSAA silhouette frays at grazing angle.
    // `across` is 0 on every mesh (and face) without across-coords → edge = 1 →
    // cov = 1, so decks/tunnels/plates and all side faces stay fully opaque.
    let edge = 1.0 - abs(across);
    let edge_cov = clamp(edge / max(fwidth(across), 1e-5) + 0.5, 0.0, 1.0);
    let cov = mix(1.0, edge_cov, top);
    return vec4<f32>(out, cov);
}
