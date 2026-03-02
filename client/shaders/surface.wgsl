struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex fn vs(
    @location(0) qxy: vec2<u32>,
    @location(1) color: vec4<f32>,
) -> VsOut {
    let u = (f32(qxy.x) - 16384.0) / 32768.0;
    let v = (f32(qxy.y) - 16384.0) / 32768.0;
    var out: VsOut;
    let margin = 0.0625;
    let scale = 1.0 / (1.0 + 2.0 * margin);
    out.pos = vec4<f32>((u + margin) * scale * 2.0 - 1.0,
                        1.0 - (v + margin) * scale * 2.0, 0.0, 1.0);
    out.color = color;
    return out;
}

@fragment fn fs(
    @location(0) color: vec4<f32>,
) -> @location(0) vec4<f32> {
    return color;
}
