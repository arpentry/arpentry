//! Every drawn mesh with geometry near one point, at one zoom: class, level,
//! band, and the height range its triangles span within a radius.
//!
//! Usage: cargo run --release --example at_probe -- <archive.arpa> <lon,lat> [radius_m] [zoom]

use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bytes = std::fs::read(&a[0]).unwrap();
    let at: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let (lon, lat) = (at[0], at[1]);
    let radius: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(3.0);
    let scan = ArchiveScan::open(&bytes).unwrap();
    let z = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(scan.max_zoom());

    for (tz, tx, ty, id) in scan.tiles_at(z) {
        let b = arpentry_server::project::Bounds::of_tile(tz, tx, ty);
        if !b.contains(lon, lat) {
            continue;
        }
        let Some(tile) = scan.decode(tz, tx, ty, id) else { continue };
        let (px, py) = (
            (lon - tile.bounds.west) / tile.bounds.width(),
            (lat - tile.bounds.south) / tile.bounds.height(),
        );
        println!("tile {tz}/{tx}/{ty}");
        for r in &tile.roads {
            if let Some(span) = r.mesh.span_near(px, py, &tile.scale, radius) {
                let at_h = r.mesh.height_at(px, py);
                println!(
                    "  {:<24} L{:<3} band {:<12} span {:.2}..{:.2}  at {}",
                    r.class,
                    r.level,
                    r.band,
                    span.0,
                    span.1,
                    at_h.map_or("-".into(), |h| format!("{h:.2}")),
                );
            }
        }
        if let Some(t) = &tile.terrain {
            if let Some(span) = t.span_near(px, py, &tile.scale, radius) {
                println!("  terrain                       span {:.2}..{:.2}", span.0, span.1);
            }
        }
    }
}
