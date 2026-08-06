//! How far does a drawn railway stand off the ground it is supposed to lie on?
//!
//! `docs/VERIFICATION.md` §10: histogram the population before believing a
//! number about it. A rail line lays no asphalt, so no carriageway mesh and no
//! kerb metric covers it — nothing in the scorecard can currently see a railway
//! floating. This walks every drawn rail centerline at level 0 in the emitted
//! archive and reports `stroke − drawn terrain` beneath it, split by class, so
//! the shape of the defect (and where its threshold can be cut) is visible.
//!
//! For contrast it walks the level-0 *road* centerlines the same way, since the
//! two share the drape path and only one of them is reported broken.
//!
//! Usage: cargo run --release --example rail_standoff -- <archive.arpa> [max_tiles]

use arpentry_server::priors::{Kind, RailClass};
use arpentry_server::verify::scene::ArchiveScan;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bytes = std::fs::read(&a[0]).expect("read archive");
    let scan = ArchiveScan::open(&bytes).expect("open archive");
    let max_tiles: usize = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(4096);
    let z = scan.max_zoom();

    // `(standoff, lon, lat)` per class, plus the pooled road population.
    let mut per_class: std::collections::BTreeMap<String, Vec<(f64, f64, f64)>> =
        std::collections::BTreeMap::new();
    let mut roads: Vec<(f64, f64, f64)> = Vec::new();
    let mut no_terrain = 0usize;
    // The population trap: a structure span emits its paint stroke *before* the
    // level ordinal is attached (pipeline.rs), so a rail line riding a bridge
    // deck arrives here at level 0 and metres above the ground, which is what a
    // deck is for. Counting those would measure the emit order, not the defect.
    let mut on_a_deck = 0usize;

    for (z, x, y, id) in scan.tiles_at(z).into_iter().take(max_tiles) {
        let Some(tile) = scan.decode(z, x, y, id) else { continue };
        let Some(terrain) = tile.terrain.as_ref() else {
            no_terrain += 1;
            continue;
        };
        for line in &tile.lines {
            if line.level != 0 {
                continue;
            }
            // The archive carries the class but not the subtype, so a rail line
            // is recognised by its class naming a gauge or a system.
            let rail = matches!(
                Kind::parse(Some("rail"), Some(&line.class), None),
                Kind::Rail(c) if c != RailClass::Unknown
            );
            for part in &line.parts {
                for &(px, py, h) in part {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    let Some(g) = terrain.height_at(px, py) else { continue };
                    // Paint on a deck: a structure surface sits right under the
                    // stroke at the stroke's own height.
                    if tile.roads.iter().filter(|m| m.is_deck()).any(|m| {
                        m.mesh.height_at(px, py).is_some_and(|d| (h - d).abs() < 1.0)
                    }) {
                        on_a_deck += 1;
                        continue;
                    }
                    let (lon, lat) = tile.lonlat(px, py);
                    if rail {
                        per_class.entry(line.class.clone()).or_default().push((h - g, lon, lat));
                    } else {
                        roads.push((h - g, lon, lat));
                    }
                }
            }
        }
    }

    if no_terrain > 0 {
        println!("{no_terrain} tiles carried no terrain mesh and were skipped");
    }
    println!("{on_a_deck} level-0 vertices sat on a structure surface (paint on a deck), excluded");
    let mut all: Vec<(f64, f64, f64)> = Vec::new();
    for (class, v) in &per_class {
        all.extend(v.iter().copied());
        report(class, &mut v.clone());
    }
    report("ALL RAIL", &mut all);
    report("level-0 road/path lines (contrast)", &mut roads);

    // Where the worst ones are, so a section can be cut there.
    all.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("\nhighest-standing rail samples");
    let mut shown = 0;
    let mut seen: Vec<(f64, f64)> = Vec::new();
    for &(d, lon, lat) in &all {
        if seen.iter().any(|&(a, b)| (a - lon).abs() < 3e-3 && (b - lat).abs() < 3e-3) {
            continue;
        }
        seen.push((lon, lat));
        println!("  {d:>8.2} m at {lon:.6},{lat:.6}");
        shown += 1;
        if shown >= 12 {
            break;
        }
    }
}

fn report(name: &str, v: &mut Vec<(f64, f64, f64)>) {
    if v.is_empty() {
        println!("\n{name}: no samples");
        return;
    }
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    let q = |f: f64| v[((v.len() as f64 - 1.0) * f) as usize].0;
    println!(
        "\n{name}: {} samples\n  p50 {:.2}  p75 {:.2}  p90 {:.2}  p95 {:.2}  p98 {:.2}  p99 {:.2}  p999 {:.2}  max {:.2}  min {:.2}",
        v.len(),
        q(0.50), q(0.75), q(0.90), q(0.95), q(0.98), q(0.99), q(0.999), q(1.0), q(0.0)
    );
    println!("  standoff (m)   share");
    for lo in [-8.0, -4.0, -2.0, -1.0, -0.5, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0] {
        let hi = match lo {
            -8.0 => -4.0,
            -4.0 => -2.0,
            -2.0 => -1.0,
            -1.0 => -0.5,
            -0.5 => 0.5,
            0.5 => 1.0,
            1.0 => 2.0,
            2.0 => 4.0,
            4.0 => 8.0,
            8.0 => 16.0,
            16.0 => 32.0,
            _ => f64::INFINITY,
        };
        let n = v.iter().filter(|s| s.0 >= lo && s.0 < hi).count();
        let share = 100.0 * n as f64 / v.len() as f64;
        let bar = "#".repeat((share * 0.8) as usize);
        println!("  {lo:>6.1}..{hi:<6.1} {share:>6.2}%  {bar}");
    }
}
