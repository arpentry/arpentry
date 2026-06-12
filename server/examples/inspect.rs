//! Inspect one tile of an .arpa archive: per-layer feature counts and, for the
//! transportation layer, the class distribution.
//!
//! Usage: cargo run --release --example inspect -- <archive> <z> <x> <y>

use arpentry_server::archive::Archive;
use arpentry_server::fb::tile::arpentry::tiles as tile;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args[2] == "scan" {
        scan(&args[1], args[3].parse().unwrap());
        return;
    }
    let (path, z, x, y) = (&args[1], args[2].parse::<u8>().unwrap(), args[3].parse::<u32>().unwrap(), args[4].parse::<u32>().unwrap());
    let data = std::fs::read(path).unwrap();
    let reader = Archive::open(&data).unwrap();
    let Some(blob) = reader.get(z, x, y) else {
        eprintln!("tile {}/{}/{} not found", z, x, y);
        let mut per_zoom: std::collections::BTreeMap<u8, usize> = Default::default();
        let mut ranges: std::collections::BTreeMap<u8, (u32, u32, u32, u32)> = Default::default();
        for e in reader.entries() {
            let (ez, ex, ey) = (e.z, e.x, e.y);
            *per_zoom.entry(ez).or_default() += 1;
            let r = ranges.entry(ez).or_insert((u32::MAX, 0, u32::MAX, 0));
            r.0 = r.0.min(ex); r.1 = r.1.max(ex);
            r.2 = r.2.min(ey); r.3 = r.3.max(ey);
        }
        for (ez, n) in per_zoom {
            let r = ranges[&ez];
            eprintln!("  z{:2}: {:6} tiles  x:[{}..{}] y:[{}..{}]", ez, n, r.0, r.1, r.2, r.3);
        }
        std::process::exit(1);
    };
    let mut input = std::io::Cursor::new(blob);
    let mut raw = Vec::new();
    brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
    println!("tile {}/{}/{}: {} bytes compressed, {} raw", z, x, y, blob.len(), raw.len());
    let t = tile::root_as_tile(&raw).unwrap();
    let keys: Vec<&str> = t.keys().map(|k| k.iter().collect()).unwrap_or_default();
    let values = t.values();
    for layer in t.layers().into_iter().flatten() {
        let feats = layer.features();
        let n = feats.map_or(0, |f| f.len());
        println!("layer {:16} {} features", layer.name(), n);
        if layer.name() == "transportation" {
            let mut by_class: std::collections::BTreeMap<String, (usize, usize)> = Default::default();
            for f in feats.into_iter().flatten() {
                let mut class = String::from("?");
                for p in f.properties().into_iter().flatten() {
                    if keys.get(p.key() as usize) == Some(&"class") {
                        if let Some(vals) = values {
                            let v = vals.get(p.value() as usize);
                            class = format!("{:?}", v);
                        }
                    }
                }
                let verts = match f.geometry_type() {
                    tile::Geometry::LineGeometry => {
                        f.geometry_as_line_geometry().map_or(0, |g| g.x().len())
                    }
                    _ => 0,
                };
                let e = by_class.entry(class).or_default();
                e.0 += 1;
                e.1 += verts;
            }
            for (c, (n, v)) in by_class {
                println!("    class {:40} {:6} features {:8} verts", c, n, v);
            }
        }
    }
}

/// Aggregates transportation class counts across every tile of one zoom.
fn scan(path: &str, z: u8) {
    let data = std::fs::read(path).unwrap();
    let reader = Archive::open(&data).unwrap();
    let mut by_class: std::collections::BTreeMap<String, usize> = Default::default();
    let mut tiles = 0usize;
    for e in reader.entries() {
        if e.z != z {
            continue;
        }
        tiles += 1;
        let blob = reader.get(e.z, e.x, e.y).unwrap();
        let mut input = std::io::Cursor::new(blob);
        let mut raw = Vec::new();
        brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
        let t = tile::root_as_tile(&raw).unwrap();
        let keys: Vec<&str> = t.keys().map(|k| k.iter().collect()).unwrap_or_default();
        let values = t.values();
        for layer in t.layers().into_iter().flatten() {
            if layer.name() != "transportation" {
                continue;
            }
            for f in layer.features().into_iter().flatten() {
                for p in f.properties().into_iter().flatten() {
                    if keys.get(p.key() as usize) == Some(&"class") {
                        if let Some(vals) = values {
                            let v = vals.get(p.value() as usize);
                            let class = v.string_value().unwrap_or("?").to_string();
                            *by_class.entry(class).or_default() += 1;
                        }
                    }
                }
            }
        }
    }
    println!("zoom {}: {} tiles", z, tiles);
    for (c, n) in by_class {
        println!("  {:20} {:8}", c, n);
    }
}
