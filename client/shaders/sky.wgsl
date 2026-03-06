struct SkyUniforms {
    inv_projection: mat4x4<f32>,
    sun_dir: vec3<f32>,
    altitude: f32,
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

@fragment fn fs(@location(0) ndc: vec2<f32>) -> @location(0) vec4<f32> {
    // Reconstruct view ray from NDC using inverse projection
    let clip = vec4<f32>(ndc.x, ndc.y, 1.0, 1.0);
    let view_pos = sky.inv_projection * clip;
    let ray = normalize(view_pos.xyz / view_pos.w);

    // ray.y approximates elevation angle in camera space:
    //   +y = up (zenith), -y = down (below horizon)
    let elev = ray.y;

    // Sky gradient: deep blue at zenith -> pale blue at horizon -> warm haze below
    let zenith_color = vec3<f32>(0.15, 0.25, 0.55);
    let horizon_color = vec3<f32>(0.55, 0.65, 0.80);
    let haze_color = vec3<f32>(0.70, 0.65, 0.58);

    var sky_color: vec3<f32>;
    if (elev > 0.0) {
        // Above horizon: blend from horizon to zenith
        let t = pow(elev, 0.6);
        sky_color = mix(horizon_color, zenith_color, t);
    } else {
        // Below horizon: blend from horizon to warm haze
        let t = pow(clamp(-elev, 0.0, 1.0), 0.4);
        sky_color = mix(horizon_color, haze_color, t);
    }

    // Sun disc with glow
    let sun = normalize(sky.sun_dir);
    let cos_angle = dot(ray, sun);
    // Tight bright disc
    let disc = smoothstep(0.9995, 0.9999, cos_angle);
    // Broader glow
    let glow = pow(max(cos_angle, 0.0), 256.0) * 0.6;
    let sun_contribution = vec3<f32>(1.0, 0.95, 0.85) * (disc + glow);
    sky_color = sky_color + sun_contribution;

    // Altitude fade: blend sky to black as camera goes into space
    // Start fading at 200km, fully faded by 2000km
    let fade_start = 200000.0;
    let fade_end = 2000000.0;
    let fade = 1.0 - clamp((sky.altitude - fade_start) / (fade_end - fade_start), 0.0, 1.0);
    sky_color = sky_color * fade;

    return vec4<f32>(sky_color, 1.0);
}
