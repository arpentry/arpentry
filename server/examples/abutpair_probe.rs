//! Every drawn stroke end near a site, with the facts the abutment pairing
//! reads: class, carried-ness (a deck/bore solid covering it in plan and
//! bracketing its height), heading, and the distance to every other end of the
//! same class within pairing reach.
//!
//! The abutment metric names a carried end and a distance; this says what else
//! was standing there, so a mis-pairing (the true partner skipped, a parallel
//! track picked instead) is distinguishable from a stroke that really does end
//! metres from its continuation.
//!
//! Usage: cargo run --release --example abutpair_probe -- <archive.arpa> <lon,lat> [radius_m]

use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let data = std::fs::read(&a[0]).expect("archive");
    let at: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let (lon0, lat0) = (at[0], at[1]);
    let radius = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(30.0);

    let scan = ArchiveScan::open(&data).expect("scan");
    let z = scan.max_zoom();
    for (z, x, y, id) in scan.tiles_at(z) {
        let Some(tile) = scan.decode(z, x, y, id) else { continue };
        if !(tile.bounds.west <= lon0
            && lon0 <= tile.bounds.east
            && tile.bounds.south <= lat0
            && lat0 <= tile.bounds.north)
        {
            continue;
        }
        let px0 = (lon0 - tile.bounds.west) / tile.bounds.width();
        let py0 = (lat0 - tile.bounds.south) / tile.bounds.height();
        println!("tile {z}/{x}/{y}");

        let solids: Vec<(&str, &arpentry_server::verify::mesh::SurfaceMesh)> = tile
            .roads
            .iter()
            .filter(|m| (m.is_deck() || m.is_bore()) && !m.is_fitted_deck())
            .map(|m| (m.class.as_str(), &m.mesh))
            .collect();
        println!("  {} structure solids", solids.len());

        struct End {
            class: String,
            px: f64,
            py: f64,
            h: f64,
            heading: f64,
            carried: bool,
            d_site: f64,
        }
        let mut ends: Vec<End> = Vec::new();
        for line in &tile.lines {
            if line.class == "marking" {
                continue;
            }
            for part in &line.parts {
                if part.len() < 2 {
                    continue;
                }
                let last = part.len() - 1;
                for (a, b) in [(part[1], part[0]), (part[last - 1], part[last])] {
                    let (dx, dy) = ((b.0 - a.0) * tile.scale.mx, (b.1 - a.1) * tile.scale.my);
                    if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
                        continue;
                    }
                    let d_site = tile.scale.dist(b.0, b.1, px0, py0);
                    if d_site > radius {
                        continue;
                    }
                    let carried = solids.iter().any(|(_, m)| {
                        m.height_range_at(b.0, b.1)
                            .is_some_and(|(lo, hi)| b.2 >= lo - 1.0 && b.2 <= hi + 1.0)
                    });
                    ends.push(End {
                        class: line.class.clone(),
                        px: b.0,
                        py: b.1,
                        h: b.2,
                        heading: dy.atan2(dx),
                        carried,
                        d_site,
                    });
                }
            }
        }
        ends.sort_by(|p, q| p.d_site.total_cmp(&q.d_site));
        println!("  {} stroke ends within {radius} m of the site:", ends.len());
        for (i, e) in ends.iter().enumerate() {
            let (lon, lat) = tile.lonlat(e.px, e.py);
            println!(
                "   [{i:2}] {:<16} {} h {:8.2}  heading {:6.1}°  owns {}  {:.6},{:.6}  d_site {:6.2}",
                e.class,
                if e.carried { "CARRIED" } else { "grade  " },
                e.h,
                e.heading.to_degrees(),
                if tile.owns(e.px, e.py) { "y" } else { "n" },
                lon,
                lat,
                e.d_site,
            );
        }
        // Pairwise distances between same-class ends, closest first.
        println!("  same-class end-to-end distances under 12.5 m:");
        for i in 0..ends.len() {
            for j in i + 1..ends.len() {
                if ends[i].class != ends[j].class {
                    continue;
                }
                let d = tile.scale.dist(ends[i].px, ends[i].py, ends[j].px, ends[j].py);
                if d <= 12.5 {
                    let dh = (ends[i].h - ends[j].h).abs();
                    println!("   [{i:2}]–[{j:2}]  {d:6.2} m  Δh {dh:5.2}");
                }
            }
        }
    }
}
