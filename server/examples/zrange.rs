//! Print the terrain-mesh elevation range and per-layer z ranges of one tile.
//! Usage: cargo run --example zrange -- <archive.arpa> <z> <x> <y>

use arpentry_server::archive::Archive;
use arpentry_server::fb::tile::arpentry::tiles as fbt;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (path, z, x, y) = (
        &a[0],
        a[1].parse::<u8>().unwrap(),
        a[2].parse::<u32>().unwrap(),
        a[3].parse::<u32>().unwrap(),
    );
    let bytes = std::fs::read(path).unwrap();
    let archive = Archive::open(&bytes).unwrap();
    let blob = archive.get(z, x, y).expect("tile present");
    let mut raw = Vec::new();
    let mut input = blob;
    brotli::BrotliDecompress(&mut input, &mut raw).unwrap();
    let tile = fbt::root_as_tile(&raw).unwrap();
    for layer in tile.layers().unwrap() {
        let Some(feats) = layer.features() else { continue };
        let (mut lo, mut hi) = (i64::MAX, i64::MIN);
        let mut with_z = 0usize;
        for f in feats {
            let zs: Option<Vec<i32>> = if let Some(m) = f.geometry_as_mesh_geometry() {
                Some(m.z().iter().collect())
            } else if let Some(l) = f.geometry_as_line_geometry() {
                l.z().map(|v| v.iter().collect())
            } else if let Some(p) = f.geometry_as_polygon_geometry() {
                p.z().map(|v| v.iter().collect())
            } else {
                None
            };
            if let Some(zs) = zs {
                if !zs.is_empty() {
                    with_z += 1;
                    for v in zs {
                        lo = lo.min(v as i64);
                        hi = hi.max(v as i64);
                    }
                }
            }
        }
        if with_z > 0 {
            println!(
                "layer {:15} {:4} feats with z, range {:.1}..{:.1} m",
                layer.name(),
                with_z,
                lo as f64 / 1000.0,
                hi as f64 / 1000.0
            );
        } else {
            println!("layer {:15} no z", layer.name());
        }
    }
}
