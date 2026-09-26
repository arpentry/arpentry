//! Scratch probe: what one slice of the rect by every material boundary
//! costs, and whether every face it returns can be tagged.
//!
//! The two things `data/plans/one-ground-2026-09-16.md` step 1 is not yet
//! measured on: `poly::slice` is validated on a two-stroke square, and the
//! loop box has thousands of rings. This runs it over a synthetic world so
//! the cost is a number before the step is built on it.
//!
//!   cargo run --release --example slice_probe -- TERRAIN SEGMENTS [SPACING]

use std::time::Instant;

use arpentry_server::dem::Dem;
use arpentry_server::project::Bounds;
use arpentry_world::pipeline::{self, Sources};
use arpentry_world::poly::{self, Shapes};
use arpentry_world::step::Step;
use arpentry_world::world::World;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a[0] == "stress" {
        let n: usize = a[1].parse().unwrap();
        let seed: u64 = a.get(2).map(|s| s.parse().unwrap()).unwrap_or(1);
        stress(n, seed);
        return;
    }
    let (terrain, segments) = (a[0].clone(), a[1].clone());
    let spacing: f64 = a.get(2).map(|s| s.parse().unwrap()).unwrap_or(10.0);

    let mut dem = Dem::open(std::path::Path::new(&terrain)).unwrap();
    let mut world = World::new(Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 });
    let mut src = Sources {
        dem: &mut dem,
        segments: std::path::Path::new(&segments),
        buildings: None,
        spacing,
        max_vertices: usize::MAX,
    };
    pipeline::upto(&mut world, Step::Sheet, &mut src, &mut |_, _| {}).unwrap();

    let rect = world.extent.rect;
    let outer = vec![poly::rect(rect.x0, rect.y0, rect.x1, rect.y1)];
    let surface = &world.room.as_ref().unwrap().surface;
    let sheets = world.sheets.as_ref().unwrap();

    // Every material boundary, as cut lines. The span mask goes in too: what
    // is over a span is paved but does not cut the ground, so it is a face of
    // its own rather than a term in an expression.
    let spanned = sheets.spanned();
    let materials: [(&str, &Shapes); 4] = [
        ("carriageway", &surface.carriageway),
        ("pavement", &surface.walk),
        ("ballast", &surface.ballast),
        ("spanned", &spanned),
    ];
    let mut cuts = Vec::new();
    for (name, s) in materials {
        let r = poly::rings(s);
        println!("{name:<12} regions={:<5} rings={:<6} m2={:.0}", s.len(), r.len(), poly::area(s));
        cuts.extend(r);
    }
    let vertices: usize = cuts.iter().map(Vec::len).sum();
    println!("\ncuts         rings={} vertices={vertices}", cuts.len());

    let t = Instant::now();
    let faces = poly::slice(&outer, &cuts);
    let sliced = t.elapsed().as_secs_f64();

    // A partition: the faces cover the rect exactly.
    let total: f64 = faces.iter().map(|f| poly::area(std::slice::from_ref(f))).sum();
    let want = rect.width() * rect.height();
    println!("slice        faces={} in {sliced:.2}s", faces.len());
    println!("             area={total:.1} of {want:.1}  ({:+.3e} m2)", total - want);

    // Tagging: one interior point per face, asked of each material in turn.
    // A face that cannot be probed is the open question — how thin do they
    // get, and how much area is in them.
    let t = Instant::now();
    let mut untagged = 0usize;
    let mut untagged_m2 = 0.0;
    let mut thinnest = f64::INFINITY;
    let mut per: [usize; 5] = [0; 5];
    let mut per_m2 = [0.0f64; 5];
    for f in &faces {
        let m2 = poly::area(std::slice::from_ref(f));
        let Some(probe) = poly::inside(f) else {
            untagged += 1;
            untagged_m2 += m2;
            thinnest = thinnest.min(m2);
            continue;
        };
        // First material that claims it; ground if none does.
        let k = materials.iter().position(|(_, s)| poly::contains(s, probe)).unwrap_or(4);
        per[k] += 1;
        per_m2[k] += m2;
    }
    let tagged = t.elapsed().as_secs_f64();
    println!("tag          {:.2}s", tagged);
    for (i, name) in ["carriageway", "pavement", "ballast", "spanned", "ground"].iter().enumerate() {
        println!("  {name:<12} faces={:<5} m2={:.0}", per[i], per_m2[i]);
    }
    println!("  {:<12} faces={untagged:<5} m2={untagged_m2:.4}  thinnest={thinnest:.2e}", "UNTAGGED");

    // The property the step is for: every face vertex away from the rect's
    // own edge is shared by the faces that meet there.
    let on_edge = |p: &[f64; 2]| {
        (p[0] - rect.x0).abs() < 1e-9 || (p[0] - rect.x1).abs() < 1e-9
            || (p[1] - rect.y0).abs() < 1e-9 || (p[1] - rect.y1).abs() < 1e-9
    };
    let mut seen: std::collections::HashMap<(u64, u64), usize> = std::collections::HashMap::new();
    for p in faces.iter().flatten().flatten() {
        *seen.entry((p[0].to_bits(), p[1].to_bits())).or_default() += 1;
    }
    let lone = faces
        .iter()
        .flatten()
        .flatten()
        .filter(|p| !on_edge(p))
        .filter(|p| seen[&(p[0].to_bits(), p[1].to_bits())] == 1)
        .count();
    println!("\ndistinct vertices={} interior-but-unshared={lone}", seen.len());
}

/// A stress case: `n` strokes scattered over the rect, buffered, unioned —
/// ring counts the synthetic networks cannot reach, so the cost of a slice
/// can be read as a curve against the loop box's thousands rather than
/// guessed at.
///
///   cargo run --release --example slice_probe -- stress N [SEED]
fn stress(n: usize, seed: u64) {
    let (w, h) = (1500.0f64, 1100.0f64);
    let outer = vec![poly::rect(0.0, 0.0, w, h)];
    // A cheap deterministic generator: no dependency, and the run reproduces.
    let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut strokes: Shapes = Vec::new();
    for _ in 0..n {
        let (x0, y0) = (next() * w, next() * h);
        let (a, len) = (next() * std::f64::consts::TAU, 40.0 + next() * 260.0);
        let line = [[x0, y0], [x0 + len * a.cos(), y0 + len * a.sin()]];
        strokes.extend(poly::buffer_line(&line, 3.0 + next() * 6.0));
    }
    let t = Instant::now();
    let paving = poly::union_all(&strokes);
    let unioned = t.elapsed().as_secs_f64();
    let cuts = poly::rings(&paving);
    let vertices: usize = cuts.iter().map(Vec::len).sum();

    let t = Instant::now();
    let faces = poly::slice(&outer, &cuts);
    let sliced = t.elapsed().as_secs_f64();

    let t = Instant::now();
    let (mut untagged, mut untagged_m2) = (0usize, 0.0f64);
    let mut paved_m2 = 0.0;
    for f in &faces {
        let m2 = poly::area(std::slice::from_ref(f));
        match poly::inside(f) {
            Some(p) if poly::contains(&paving, p) => paved_m2 += m2,
            Some(_) => {}
            None => {
                untagged += 1;
                untagged_m2 += m2;
            }
        }
    }
    let tagged = t.elapsed().as_secs_f64();
    let total: f64 = faces.iter().map(|f| poly::area(std::slice::from_ref(f))).sum();
    println!(
        "n={n:<5} rings={:<5} verts={vertices:<7} union={unioned:.2}s slice={sliced:.2}s \
         tag={tagged:.2}s faces={:<5} paved={paved_m2:.0}/{:.0} untagged={untagged} ({untagged_m2:.3} m2) \
         partition={:+.2e}",
        cuts.len(),
        faces.len(),
        poly::area(&poly::intersect(&outer, &paving)),
        total - w * h
    );
}
