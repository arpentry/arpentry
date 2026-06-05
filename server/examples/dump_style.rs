//! Decode the `.arps` that the server would build from a JSON style file.
//! Usage: cargo run --example dump_style -- <style.json>

use arpentry_server::fb::style::arpentry::tiles as fbs;
use arpentry_server::style;

fn main() {
    let path = std::env::args().nth(1).expect("style.json path");
    let blob = style::build_from_file(std::path::Path::new(&path)).expect("build style");
    let mut raw = Vec::new();
    let mut input = blob.as_slice();
    brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
    println!("id={:?} version", &raw[4..8]);

    let s = fbs::root_as_style(&raw).expect("root_as_style");
    println!("version={} name={:?}", s.version(), s.name());
    if let Some(bg) = s.background() {
        println!("background=[{},{},{},{}]", bg.r(), bg.g(), bg.b(), bg.a());
    }
    let layers = s.layers().expect("layers");
    for i in 0..layers.len() {
        let l = layers.get(i);
        println!("layer[{i}] source_layer={:?} type={:?} min_level={}", l.source_layer(), l.type_(), l.min_level());
        if let Some(paint) = l.paint() {
            for j in 0..paint.len() {
                let p = paint.get(j);
                let c = p.color().map(|c| format!("[{},{},{},{}]", c.r(), c.g(), c.b(), c.a()));
                println!("    paint[{j}] class={:?} color={:?} width={} min_level={}",
                    p.class(), c, p.width(), p.min_level());
            }
        }
    }
}
