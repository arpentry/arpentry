/* Downsample one mip level to the next by sampling the previous level with
   bilinear filtering. A fullscreen triangle covers the destination mip; each
   fragment bilinearly samples the source so a 2×2 block averages into a 1×1
   destination texel. */

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex fn vs(@builtin(vertex_index) i: u32) -> VsOut {
    /* Fullscreen triangle: vertices at (-1,1), (3,1), (-1,-3) in clip space;
       UVs at (0,0), (2,0), (0,2). The visible triangle in [-1,1]² maps to
       UV [0,1]². */
    let x = f32((i << 1u) & 2u);
    let y = f32(i & 2u);
    var out: VsOut;
    out.pos = vec4<f32>(x * 2.0 - 1.0, 1.0 - y * 2.0, 0.0, 1.0);
    out.uv = vec2<f32>(x, y);
    return out;
}

@group(0) @binding(0) var src_tex: texture_2d<f32>;
@group(0) @binding(1) var src_samp: sampler;

@fragment fn fs(in: VsOut) -> @location(0) vec4<f32> {
    return textureSample(src_tex, src_samp, in.uv);
}
