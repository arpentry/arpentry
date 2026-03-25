struct SkyUniforms {
    inv_projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    altitude: f32,
    earth_center: vec3<f32>,
    earth_radius: f32,
};

@group(0) @binding(0) var<uniform> sky: SkyUniforms;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

// Full-screen triangle (same pattern as ui.wgsl)
@vertex fn vs(@builtin(vertex_index) vi: u32) -> VsOut {
    var p = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, 3.0),
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
    );
    var out: VsOut;
    out.pos = vec4<f32>(p[vi], 1.0, 1.0);  // z=1.0 for far plane
    out.ndc = p[vi];
    return out;
}

const PI: f32 = 3.14159265;

// ---------------------------------------------------------------------------
// Ray-sphere intersection
// Returns (t_near, t_far). If no intersection, t_near > t_far.
// ---------------------------------------------------------------------------
fn ray_sphere(ro: vec3<f32>, rd: vec3<f32>, center: vec3<f32>, radius: f32) -> vec2<f32> {
    let oc = ro - center;
    let b = dot(oc, rd);
    let c = dot(oc, oc) - radius * radius;
    let disc = b * b - c;
    if (disc < 0.0) {
        return vec2<f32>(1.0, -1.0); // no hit sentinel
    }
    let sq = sqrt(disc);
    return vec2<f32>(-b - sq, -b + sq);
}

// ---------------------------------------------------------------------------
// Atmospheric scattering constants
// ---------------------------------------------------------------------------
const ATMO_SCALE: f32 = 0.035;            // atmosphere thickness as fraction of R
const RAYLEIGH_H: f32 = 0.125;            // scale height as fraction of atmo thickness
const MIE_H: f32 = 0.02;                  // Mie scale height as fraction
const NUM_VIEW_SAMPLES: i32 = 32;
const NUM_LIGHT_SAMPLES: i32 = 8;

// Rayleigh scattering coefficients (normalized, will be scaled by atmosphere)
const BETA_R: vec3<f32> = vec3<f32>(3.8e-6, 13.5e-6, 33.1e-6);
// Mie scattering coefficient
const BETA_M: f32 = 21.0e-6;
const MIE_G: f32 = 0.76;

// ---------------------------------------------------------------------------
// Phase functions
// ---------------------------------------------------------------------------
fn phase_rayleigh(cos_theta: f32) -> f32 {
    return 3.0 / (16.0 * PI) * (1.0 + cos_theta * cos_theta);
}

fn phase_mie(cos_theta: f32) -> f32 {
    let g2 = MIE_G * MIE_G;
    let num = 3.0 * (1.0 - g2) * (1.0 + cos_theta * cos_theta);
    let denom = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * MIE_G * cos_theta, 1.5);
    return num / denom;
}

// ---------------------------------------------------------------------------
// Approximate optical depth along a ray from a point toward the sun
// using the Chapman function approximation for a spherical atmosphere.
// This avoids expensive inner loop ray marching.
// h = altitude above surface (meters)
// cos_chi = cosine of zenith angle of the ray from the point
// H = scale height (meters)
// R = planet radius (meters)
// ---------------------------------------------------------------------------
fn chapman_approx(h: f32, cos_chi: f32, H: f32, R: f32) -> f32 {
    let x = (R + h) / H;
    // Simplified Chapman: for overhead sun, depth ~ H * exp(-h/H)
    // For grazing angles, multiply by sqrt(pi*x/2) approximately
    let base = exp(-h / H);
    if (cos_chi >= 0.0) {
        // Above horizon: simple secant approximation
        return H * base / max(cos_chi, 0.01);
    } else {
        // Below horizon: use grazing angle approximation
        let sin_chi = sqrt(1.0 - cos_chi * cos_chi);
        let r_h = R + h;
        let tangent_h = r_h * sin_chi - R;
        let grazing = 2.0 * H * exp(-tangent_h / H) * sqrt(PI * r_h / (2.0 * H));
        let overhead = H * base / 0.01;
        return min(grazing, overhead);
    }
}

// ---------------------------------------------------------------------------
// Single-scattering atmosphere: physically-based ray march with Chapman
// approximation for the light path optical depth.
// ---------------------------------------------------------------------------
fn atmosphere(rd: vec3<f32>, sun: vec3<f32>) -> vec3<f32> {
    let R = sky.earth_radius;
    let atmo_height = R * ATMO_SCALE;
    let R_atmo = R + atmo_height;
    let H_r = atmo_height * RAYLEIGH_H;    // ~20 km
    let H_m = atmo_height * MIE_H;         // ~3.2 km

    // Ray-atmosphere intersection (camera at origin in view space)
    let atmo_hit = ray_sphere(vec3<f32>(0.0), rd, sky.earth_center, R_atmo);
    if (atmo_hit.x > atmo_hit.y) {
        return vec3<f32>(0.0); // miss atmosphere -> black space
    }

    // Ray-earth intersection
    let earth_hit = ray_sphere(vec3<f32>(0.0), rd, sky.earth_center, R);
    let hits_earth = earth_hit.x < earth_hit.y && earth_hit.y > 0.0;

    // Clamp to visible atmosphere segment
    let t_start = max(atmo_hit.x, 0.0);
    var t_end = atmo_hit.y;
    if (hits_earth && earth_hit.x > 0.0) {
        t_end = earth_hit.x;
    }
    if (t_start >= t_end) {
        return vec3<f32>(0.0);
    }

    let step_len = (t_end - t_start) / f32(NUM_VIEW_SAMPLES);
    let cos_theta = dot(rd, sun);
    let pr = phase_rayleigh(cos_theta);
    let pm = phase_mie(cos_theta);

    var total_r = vec3<f32>(0.0);
    var total_m = vec3<f32>(0.0);
    var opt_r = 0.0;
    var opt_m = 0.0;

    for (var i = 0; i < NUM_VIEW_SAMPLES; i++) {
        let t = t_start + (f32(i) + 0.5) * step_len;
        let pos = rd * t;
        let to_center = pos - sky.earth_center;
        let dist = length(to_center);
        let h = dist - R;

        if (h < 0.0) { continue; } // inside earth, skip

        let rho_r = exp(-h / H_r) * step_len;
        let rho_m = exp(-h / H_m) * step_len;
        opt_r += rho_r;
        opt_m += rho_m;

        // Light optical depth using Chapman approximation
        let up = to_center / dist;
        let cos_chi = dot(up, sun);

        // Check if sun is below horizon from this point
        let sin_horizon = R / dist;
        let cos_horizon = -sqrt(1.0 - sin_horizon * sin_horizon);
        if (cos_chi < cos_horizon) {
            continue; // in shadow
        }

        let light_r = chapman_approx(h, cos_chi, H_r, R);
        let light_m = chapman_approx(h, cos_chi, H_m, R);

        let tau = BETA_R * (opt_r + light_r)
                + vec3<f32>(BETA_M * 1.1) * (opt_m + light_m);
        let attenuation = exp(-tau);

        total_r += rho_r * attenuation;
        total_m += rho_m * attenuation;
    }

    let scatter = total_r * BETA_R * pr + total_m * vec3<f32>(BETA_M) * pm;
    let sun_intensity = 20.0;
    return scatter * sun_intensity;
}

// ---------------------------------------------------------------------------
// Procedural star field
// ---------------------------------------------------------------------------
fn hash3(p: vec3<f32>) -> f32 {
    var q = fract(p * 0.1031);
    q += dot(q, q.zyx + 31.32);
    return fract((q.x + q.y) * q.z);
}

fn stars(rd: vec3<f32>) -> vec3<f32> {
    // Quantize direction into a grid on the unit sphere
    let grid = floor(rd * 400.0);
    let cell_id = hash3(grid);

    // ~1% of cells have a star
    if (cell_id > 0.01) {
        return vec3<f32>(0.0);
    }

    // Star position jittered within cell
    let cell_center = (grid + 0.5) / 400.0;
    let star_offset = vec3<f32>(
        hash3(grid + vec3<f32>(1.0, 0.0, 0.0)) - 0.5,
        hash3(grid + vec3<f32>(0.0, 1.0, 0.0)) - 0.5,
        hash3(grid + vec3<f32>(0.0, 0.0, 1.0)) - 0.5,
    ) * 0.002;
    let star_dir = normalize(cell_center + star_offset);

    // Angular distance — tight point
    let cos_dist = dot(rd, star_dir);
    if (cos_dist < 0.99998) {
        return vec3<f32>(0.0);
    }

    // Brightness: most stars dim, few bright (power curve)
    let raw = hash3(grid + vec3<f32>(7.0, 13.0, 17.0));
    let brightness = pow(raw, 3.0) * 0.7 + 0.08;

    // Slight color variation
    let color_t = hash3(grid + vec3<f32>(23.0, 37.0, 41.0));
    let star_color = mix(
        vec3<f32>(1.0, 0.95, 0.85),
        vec3<f32>(0.85, 0.92, 1.0),
        color_t,
    );

    return star_color * brightness;
}

// ---------------------------------------------------------------------------
// Ground-level sky (gradient for low altitudes)
// ---------------------------------------------------------------------------
fn sky_gradient(ray: vec3<f32>, sun: vec3<f32>) -> vec3<f32> {
    let elev = ray.y;

    let zenith_color = vec3<f32>(0.15, 0.25, 0.55);
    let horizon_color = vec3<f32>(0.55, 0.65, 0.80);
    let haze_color = vec3<f32>(0.70, 0.65, 0.58);

    var col: vec3<f32>;
    if (elev > 0.0) {
        let t = pow(elev, 0.6);
        col = mix(horizon_color, zenith_color, t);
    } else {
        let t = pow(clamp(-elev, 0.0, 1.0), 0.4);
        col = mix(horizon_color, haze_color, t);
    }

    // Sun disc with glow
    let cos_angle = dot(ray, sun);
    let disc = smoothstep(0.9995, 0.9999, cos_angle);
    let glow = pow(max(cos_angle, 0.0), 256.0) * 0.6;
    col += vec3<f32>(1.0, 0.95, 0.85) * (disc + glow);

    return col;
}

@fragment fn fs(@location(0) ndc: vec2<f32>) -> @location(0) vec4<f32> {
    let clip = vec4<f32>(ndc.x, ndc.y, 1.0, 1.0);
    let view_pos = sky.inv_projection * clip;
    let ray = normalize(view_pos.xyz / view_pos.w);
    let sun = normalize(sky.sun_dir);

    // Blend between ground-level sky and space atmosphere
    // Transition zone: 50 km to 200 km
    let space_start = 50000.0;
    let space_end = 200000.0;
    let space_t = clamp((sky.altitude - space_start) / (space_end - space_start), 0.0, 1.0);
    let blend = space_t * space_t * (3.0 - 2.0 * space_t);

    let ground_sky = sky_gradient(ray, sun);
    let space_sky = atmosphere(ray, sun);

    // Stars: visible in dark areas of the space view
    let star_light = stars(ray);
    // Fade stars where atmosphere is bright (prevents stars through the glow)
    let atmo_brightness = dot(space_sky, vec3<f32>(0.299, 0.587, 0.114));
    let star_mask = 1.0 - clamp(atmo_brightness * 10.0, 0.0, 1.0);
    let space_with_stars = space_sky + star_light * star_mask;

    let col = mix(ground_sky, space_with_stars, blend);

    return vec4<f32>(col, 1.0);
}
