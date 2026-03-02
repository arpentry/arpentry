struct Uniforms {
    screen: vec2<f32>,
    scale: f32,
    bearing: f32,
    tilt: f32,
    cursor_x: f32,
    cursor_y: f32,
    _pad: f32,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) pixel: vec2<f32>,
};

// Full-screen triangle
@vertex fn vs(@builtin(vertex_index) vi: u32) -> VsOut {
    var p = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, 3.0),
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
    );
    var out: VsOut;
    out.pos = vec4<f32>(p[vi], 0.0, 1.0);
    out.pixel = vec2<f32>(
        (p[vi].x * 0.5 + 0.5) * u.screen.x,
        (0.5 - p[vi].y * 0.5) * u.screen.y,
    );
    return out;
}

// SDF helpers
fn sd_box(p: vec2<f32>, b: vec2<f32>, r: f32) -> f32 {
    let q = abs(p) - b + r;
    return length(max(q, vec2<f32>(0.0))) + min(max(q.x, q.y), 0.0) - r;
}

fn sd_circle(p: vec2<f32>, r: f32) -> f32 {
    return length(p) - r;
}

fn fill(d: f32) -> f32 {
    return clamp(0.5 - d, 0.0, 1.0);
}

// Layout constants (logical pixels, from bottom-right corner)
// Order from right edge: compass, tilt, zoom. All 64px tall.
const CY: f32      = 48.0;
const COMP_R: f32  = 32.0;
const COMP_CX: f32 = 48.0;
const TILT_HW: f32 = 19.0;
const TILT_HH: f32 = 32.0;
const TILT_R: f32  = 19.0;
const TILT_CX: f32 = 109.0;
const ZOOM_HW: f32 = 19.0;
const ZOOM_HH: f32 = 32.0;
const ZOOM_R: f32  = 19.0;
const ZOOM_CX: f32 = 157.0;

@fragment fn fs(@location(0) pixel: vec2<f32>) -> @location(0) vec4<f32> {
    let lp = pixel / u.scale;
    let scr = u.screen / u.scale;
    let br = scr - lp;

    // Relative positions to each element center
    let tilt_p = br - vec2<f32>(TILT_CX, CY);
    let comp_p = br - vec2<f32>(COMP_CX, CY);
    let zoom_p = br - vec2<f32>(ZOOM_CX, CY);

    // Shape SDFs
    let d_tilt = sd_box(tilt_p, vec2<f32>(TILT_HW, TILT_HH), TILT_R);
    let d_comp = sd_circle(comp_p, COMP_R);
    let d_zoom = sd_box(zoom_p, vec2<f32>(ZOOM_HW, ZOOM_HH), ZOOM_R);
    let d_any = min(d_tilt, min(d_comp, d_zoom));

    if (d_any > 1.5) { discard; }

    // Glass background
    var col = vec3<f32>(0.85, 0.88, 0.92);
    var a = fill(d_any) * 0.5;

    // Border highlight
    let brd = fill(abs(d_any) - 0.75) * 0.4;
    col = mix(col, vec3<f32>(1.0), brd);
    a = max(a, brd);

    let icon = vec3<f32>(0.22, 0.26, 0.32);

    // ── Zoom controls (vertical pill) ─────────────
    let iz = fill(d_zoom);

    // Horizontal divider
    let dv = fill(max(abs(zoom_p.y) - 0.5, abs(zoom_p.x) - 10.0));
    let dv_a = dv * iz * 0.35;
    col = mix(col, icon, dv_a);
    a = max(a, dv_a);

    // Plus sign (top half: positive zoom_p.y in br-coords)
    let pp = zoom_p - vec2<f32>(0.0, 16.0);
    let dp = fill(min(max(abs(pp.x) - 6.0, abs(pp.y) - 1.2),
                      max(abs(pp.x) - 1.2, abs(pp.y) - 6.0)));
    let dp_a = dp * iz;
    col = mix(col, icon, dp_a);
    a = max(a, dp_a);

    // Minus sign (bottom half: negative zoom_p.y in br-coords)
    let mp = zoom_p + vec2<f32>(0.0, 16.0);
    let dm = fill(max(abs(mp.x) - 6.0, abs(mp.y) - 1.2));
    let dm_a = dm * iz;
    col = mix(col, icon, dm_a);
    a = max(a, dm_a);

    // ── Compass ────────────────────────────────────
    let ic = fill(d_comp);

    // Inner ring
    let ring = fill(abs(sd_circle(comp_p, 27.0)) - 0.5) * 0.15;
    col = mix(col, icon, ring * ic);
    a = max(a, ring * ic);

    // Rotate for bearing (convert br-coords to math coords by flipping x)
    let mcp = vec2<f32>(-comp_p.x, comp_p.y);
    let ca = -u.bearing;
    let cc = cos(ca);
    let cs = sin(ca);
    let rp = vec2<f32>(cc * mcp.x - cs * mcp.y,
                        cs * mcp.x + cc * mcp.y);

    // North needle (red, tapered triangle)
    let nw = 4.0 * clamp(1.0 - rp.y / 20.0, 0.0, 1.0);
    let d_north = max(abs(rp.x) - nw, max(-rp.y, rp.y - 20.0));
    let na = fill(d_north) * ic;
    col = mix(col, vec3<f32>(0.85, 0.20, 0.15), na);
    a = max(a, na);

    // South needle (white, thinner)
    let sw_val = 2.5 * clamp(1.0 + rp.y / 20.0, 0.0, 1.0);
    let d_south = max(abs(rp.x) - sw_val, max(rp.y, -rp.y - 20.0));
    let sa = fill(d_south) * ic;
    col = mix(col, vec3<f32>(0.92, 0.92, 0.94), sa);
    a = max(a, sa);

    // Center dot
    let cd = fill(sd_circle(comp_p, 2.5));
    col = mix(col, vec3<f32>(0.95), cd * ic);
    a = max(a, cd * ic);

    // ── Tilt indicator ─────────────────────────────
    let it = fill(d_tilt);

    // Vertical track
    let track = fill(max(abs(tilt_p.x) - 0.75, abs(tilt_p.y) - 20.0));
    let tr_a = track * it * 0.25;
    col = mix(col, icon, tr_a);
    a = max(a, tr_a);

    // Tilt dot (0 degrees at top, 60 degrees at bottom)
    let tilt_t = clamp(u.tilt / 1.0472, 0.0, 1.0);
    let dot_y = mix(16.0, -16.0, tilt_t);
    let td = fill(sd_circle(tilt_p - vec2<f32>(0.0, dot_y), 4.5));
    let td_a = td * it;
    col = mix(col, icon, td_a);
    a = max(a, td_a);

    // ── Hover highlight ────────────────────────────
    let cur = scr - vec2<f32>(u.cursor_x, u.cursor_y);
    let hd_zoom = sd_box(cur - vec2<f32>(ZOOM_CX, CY),
                          vec2<f32>(ZOOM_HW, ZOOM_HH), ZOOM_R);
    let hd_comp = sd_circle(cur - vec2<f32>(COMP_CX, CY), COMP_R);
    let hd_tilt = sd_box(cur - vec2<f32>(TILT_CX, CY),
                          vec2<f32>(TILT_HW, TILT_HH), TILT_R);

    var hover = 0.0;
    if (hd_zoom < 0.0 && d_zoom < 0.0) {
        let cz = cur.y - CY;
        if ((cz > 0.0 && zoom_p.y > 0.0) ||
            (cz <= 0.0 && zoom_p.y <= 0.0)) {
            hover = 0.06;
        }
    }
    if (hd_comp < 0.0 && d_comp < 0.0) { hover = 0.06; }
    if (hd_tilt < 0.0 && d_tilt < 0.0) { hover = 0.06; }
    col = col + vec3<f32>(hover);

    return vec4<f32>(col, a);
}
