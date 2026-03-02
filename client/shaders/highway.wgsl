struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) local: vec2<f32>,
    @location(2) hw_len: vec2<f32>,
};

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) color: vec4<f32>,
    @location(2) local: vec2<f32>,
    @location(3) hw_len: vec2<f32>,
) -> VsOut {
    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    var out: VsOut;
    let margin = 0.0625;
    let scale = 1.0 / (1.0 + 2.0 * margin);
    out.pos = vec4<f32>((u + margin) * scale * 2.0 - 1.0,
                        1.0 - (v + margin) * scale * 2.0, 0.0, 1.0);
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
    let px = fwidth(local.y);
    let alpha = color.a * (1.0 - smoothstep(hw - px, hw, dist));
    return vec4<f32>(color.rgb, alpha);
}
