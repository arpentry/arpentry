//! What the archive *draws* a transportation feature as, at one zoom.
//!
//! Two questions, one scan:
//!
//! 1. **Which classes still draw as a cartographic stroke?** Per class, the
//!    owned centerline length in metres. A class with length here and no mesh
//!    is drawn as a line however close the camera gets.
//! 2. **How much of the drawn surface is the rim strip?** Per class family,
//!    the plan area of the `*_surface` interior against its `*_rim` strip.
//!
//! Usage: cargo run --release --example draw_census -- <archive.arpa> [zoom]

use std::collections::BTreeMap;

use arpentry_server::verify::scene::ArchiveScan;

#[derive(Default, Clone)]
struct Tally {
    length_m: f64,
    area_m2: f64,
    features: usize,
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let path = &a[0];
    let bytes = std::fs::read(path).unwrap();
    let scan = ArchiveScan::open(&bytes).unwrap();
    let z: u8 = a.get(1).map(|s| s.parse().unwrap()).unwrap_or(scan.max_zoom());

    let mut lines: BTreeMap<String, Tally> = BTreeMap::new();
    let mut meshes: BTreeMap<String, Tally> = BTreeMap::new();

    let tiles = scan.tiles_at(z);
    for (tz, x, y, id) in &tiles {
        let Some(tile) = scan.decode(*tz, *x, *y, *id) else { continue };
        let s = tile.scale;
        for l in &tile.lines {
            let e = lines.entry(l.class.clone()).or_default();
            e.features += 1;
            for part in &l.parts {
                for w in part.windows(2) {
                    let (ax, ay, _) = w[0];
                    let (bx, by, _) = w[1];
                    // Own the segment by its midpoint, the same rule the checks
                    // use, so a tile never scores its neighbour's geometry.
                    if tile.owns((ax + bx) * 0.5, (ay + by) * 0.5) {
                        e.length_m += s.dist(ax, ay, bx, by);
                    }
                }
            }
        }
        for r in &tile.roads {
            let e = meshes.entry(r.class.clone()).or_default();
            e.features += 1;
            let m = &r.mesh;
            for t in 0..m.triangle_count() {
                let [p, q, w] = m.triangle(t);
                let cx = (p.0 + q.0 + w.0) / 3.0;
                let cy = (p.1 + q.1 + w.1) / 3.0;
                if !tile.owns(cx, cy) {
                    continue;
                }
                let ax = (q.0 - p.0) * s.mx;
                let ay = (q.1 - p.1) * s.my;
                let bx = (w.0 - p.0) * s.mx;
                let by = (w.1 - p.1) * s.my;
                e.area_m2 += (ax * by - ay * bx).abs() * 0.5;
            }
        }
    }

    println!("draw census  {path}  z{z}  ({} tiles)\n", tiles.len());
    println!("STROKES (a class drawn as a cartographic line)");
    println!("{:<20} {:>14} {:>10}", "class", "length km", "features");
    let mut ls: Vec<_> = lines.into_iter().collect();
    ls.sort_by(|a, b| b.1.length_m.total_cmp(&a.1.length_m));
    let total_len: f64 = ls.iter().map(|(_, t)| t.length_m).sum();
    for (c, t) in &ls {
        println!("{:<20} {:>14.3} {:>10}", c, t.length_m / 1000.0, t.features);
    }
    println!("{:<20} {:>14.3}", "TOTAL", total_len / 1000.0);

    println!("\nSURFACES (a class drawn as a meshed area)");
    println!("{:<20} {:>14} {:>10}", "class", "area m2", "features");
    let mut ms: Vec<_> = meshes.into_iter().collect();
    ms.sort_by(|a, b| b.1.area_m2.total_cmp(&a.1.area_m2));
    for (c, t) in &ms {
        println!("{:<20} {:>14.0} {:>10}", c, t.area_m2, t.features);
    }

    println!("\nCOVER (of the still-stroked pedestrian length, how much stands on a drawn surface)");
    println!("{:<20} {:>12} {:>12} {:>10}", "class", "stroked km", "on band km", "bare %");
    let ped = ["path", "footway", "track", "steps", "pedestrian", "cycleway", "crossing"];
    let mut covered: BTreeMap<String, (f64, f64)> = BTreeMap::new();
    for (tz, x, y, id) in &tiles {
        let Some(tile) = scan.decode(*tz, *x, *y, *id) else { continue };
        let s = tile.scale;
        // Every drawn hard surface in this tile: a band, a carriageway, a
        // formation. A pedestrian way standing on any of them is drawn.
        let is_walk = |c: &str| {
            matches!(c, "walk_surface" | "walk_rim" | "path_surface" | "path_rim")
        };
        let surfaces: Vec<_> =
            tile.roads.iter().filter(|r| r.level == 0 && is_walk(&r.class)).map(|r| &r.mesh).collect();
        for l in tile.lines.iter().filter(|l| ped.contains(&l.class.as_str())) {
            let e = covered.entry(l.class.clone()).or_insert((0.0, 0.0));
            for part in &l.parts {
                for w in part.windows(2) {
                    let (ax, ay, _) = w[0];
                    let (bx, by, _) = w[1];
                    let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
                    if !tile.owns(mx, my) {
                        continue;
                    }
                    let d = s.dist(ax, ay, bx, by);
                    e.0 += d;
                    if surfaces.iter().any(|m| m.height_at(mx, my).is_some()) {
                        e.1 += d;
                    }
                }
            }
        }
    }
    let mut cs: Vec<_> = covered.into_iter().collect();
    cs.sort_by(|a, b| b.1 .0.total_cmp(&a.1 .0));
    let (mut ts, mut tc) = (0.0, 0.0);
    for (c, (st, cov)) in &cs {
        ts += st;
        tc += cov;
        println!(
            "{:<20} {:>12.3} {:>12.3} {:>9.1}%",
            c,
            st / 1000.0,
            cov / 1000.0,
            100.0 * (1.0 - cov / st.max(1e-9))
        );
    }
    println!(
        "{:<20} {:>12.3} {:>12.3} {:>9.1}%",
        "TOTAL",
        ts / 1000.0,
        tc / 1000.0,
        100.0 * (1.0 - tc / ts.max(1e-9))
    );

    joins(&scan, z);
    kerb(&scan, z);

    println!("\nRIM SHARE (the rim strip as a fraction of its family's drawn area)");
    for fam in ["road", "rail", "walk", "path"] {
        let get = |suffix: &str| {
            ms.iter()
                .find(|(c, _)| *c == format!("{fam}_{suffix}"))
                .map_or(0.0, |(_, t)| t.area_m2)
        };
        let (s, c) = (get("surface"), get("rim"));
        if s + c > 0.0 {
            println!("{:<8} surface {:>12.0}  rim {:>12.0}  {:>6.1}% rim", fam, s, c, 100.0 * c / (s + c));
        }
    }
}

/// Where the still-stroked pedestrian vertices that stand on a band sit within
/// their own feature: at an end (the join with the way they connect to) or in
/// the middle (a stretch drawn twice). Printed by `draw_census … --joins`.
fn joins(scan: &ArchiveScan, z: u8) {
    let ped = ["path", "footway", "track", "steps", "pedestrian", "cycleway"];
    let (mut ends, mut middles, mut feats, mut hit_feats) = (0usize, 0usize, 0usize, 0usize);
    let mut runs: Vec<usize> = Vec::new();
    for (tz, x, y, id) in &scan.tiles_at(z) {
        let Some(tile) = scan.decode(*tz, *x, *y, *id) else { continue };
        let bands: Vec<_> = tile
            .roads
            .iter()
            .filter(|r| {
                r.level == 0
                    && matches!(
                        r.class.as_str(),
                        "walk_surface" | "walk_rim" | "path_surface" | "path_rim"
                    )
            })
            .map(|r| &r.mesh)
            .collect();
        if bands.is_empty() {
            continue;
        }
        for l in tile.lines.iter().filter(|l| l.level == 0 && ped.contains(&l.class.as_str())) {
            feats += 1;
            let mut any = false;
            for part in &l.parts {
                let n = part.len();
                let mut run = 0usize;
                for (i, &(px, py, _)) in part.iter().enumerate() {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    if bands.iter().any(|m| m.height_at(px, py).is_some()) {
                        any = true;
                        // Within two vertices of either end of the part.
                        if i < 2 || i + 2 >= n {
                            ends += 1;
                        } else {
                            middles += 1;
                        }
                        run += 1;
                    } else if run > 0 {
                        runs.push(run);
                        run = 0;
                    }
                }
                if run > 0 {
                    runs.push(run);
                }
            }
            if any {
                hit_feats += 1;
            }
        }
    }
    runs.sort_unstable();
    println!(
        "\nJOINS  {hit_feats} of {feats} still-stroked features touch a band; \
         {ends} vertices at a part end, {middles} in the middle; \
         runs {:?} (median {})",
        runs,
        runs.get(runs.len() / 2).copied().unwrap_or(0)
    );
}

/// The drawn kerb: for every sidewalk-band vertex, the signed height against
/// the carriageway a short reach away. `KERB_RISE_M` says a band attached to a
/// street stands 0.12 m above it; this is what the archive actually draws.
fn kerb(scan: &ArchiveScan, z: u8) {
    let mut steps: Vec<f64> = Vec::new();
    for (tz, x, y, id) in &scan.tiles_at(z) {
        let Some(tile) = scan.decode(*tz, *x, *y, *id) else { continue };
        let s = tile.scale;
        let roads: Vec<_> = tile
            .roads
            .iter()
            .filter(|r| r.level == 0 && matches!(r.class.as_str(), "road_surface" | "road_rim"))
            .map(|r| &r.mesh)
            .collect();
        let walks: Vec<_> = tile
            .roads
            .iter()
            .filter(|r| r.level == 0 && matches!(r.class.as_str(), "walk_surface" | "walk_rim"))
            .collect();
        if roads.is_empty() || walks.is_empty() {
            continue;
        }
        for w in walks {
            let m = &w.mesh;
            for i in 0..m.vertex_count() {
                let (px, py, pz) = m.vertex(i);
                if !tile.owns(px, py) {
                    continue;
                }
                // The carriageway just across the kerb: march out in eight
                // directions and take the nearest asphalt within 2 m.
                let mut best: Option<(f64, f64)> = None;
                for k in 0..8 {
                    let a = k as f64 * std::f64::consts::FRAC_PI_4;
                    let (ux, uy) = (a.cos() / s.mx, a.sin() / s.my);
                    let mut d = 0.25;
                    while d <= 2.0 {
                        let (qx, qy) = (px + ux * d, py + uy * d);
                        if let Some(h) = roads.iter().find_map(|r| r.height_at(qx, qy)) {
                            if best.is_none_or(|(bd, _)| d < bd) {
                                best = Some((d, h));
                            }
                            break;
                        }
                        d += 0.25;
                    }
                }
                if let Some((_, h)) = best {
                    steps.push(pz - h);
                }
            }
        }
    }
    steps.sort_by(f64::total_cmp);
    if steps.is_empty() {
        println!("\nKERB  no sidewalk band sits within 2 m of a carriageway at this zoom");
        return;
    }
    let q = |f: f64| steps[((steps.len() - 1) as f64 * f) as usize];
    let above = steps.iter().filter(|&&v| v > 0.0).count();
    println!(
        "\nKERB  n={}  p10 {:+.3}  p25 {:+.3}  p50 {:+.3}  p75 {:+.3}  p90 {:+.3}   above the road: {:.1} %",
        steps.len(),
        q(0.10),
        q(0.25),
        q(0.50),
        q(0.75),
        q(0.90),
        100.0 * above as f64 / steps.len() as f64
    );
}
