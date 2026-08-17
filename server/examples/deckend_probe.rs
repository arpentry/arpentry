//! How far the fitted deck ramp misses the road at each structure-run
//! boundary — the residual `road_m − deck_m` at the first and last structure
//! node of every span, which is the step the band/deck handover inherits
//! (`verify/checks/handoff.rs`: the band reads `road_at_arc`, the deck
//! `deck_at_arc`, and nothing forces the two to agree at the boundary).
//!
//! Prints one line per structure-run end with the residual and the run's
//! length, then a small histogram — the population that sizes any fix.
//!
//! Usage: cargo run --release --example deckend_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    let mut residuals: Vec<(f64, f64, u32, f64)> = Vec::new(); // (|e|, run_m, corridor, arc)
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (arc, road, deck, at_grade) = (p.arc(), p.road_m(), p.deck_m(), p.at_grade());
        let n = arc.len();
        let mut i = 0;
        while i < n {
            if at_grade[i] {
                i += 1;
                continue;
            }
            let start = i;
            while i < n && !at_grade[i] {
                i += 1;
            }
            let end = i; // run is [start, end)
            let run_m = arc[end - 1] - arc[start];
            for k in [start, end - 1] {
                let e = (road[k] - deck[k]).abs();
                residuals.push((e, run_m, c.id, arc[k]));
            }
        }
    }
    residuals.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("{} structure-run ends across {} corridors", residuals.len(), scene.corridors.len());
    println!("\nworst 20 (|road−deck| at the boundary node):");
    for &(e, run_m, id, arc) in residuals.iter().take(20) {
        let c = &scene.corridors[id as usize];
        let p = solved.profile(id).unwrap();
        let pt = p.point_at_arc(arc);
        println!(
            "  {e:6.2} m  run {run_m:6.1} m  #{id} {:?} {} at {:.6},{:.6} arc {arc:.1}",
            c.kind, c.class_key, pt.x, pt.y
        );
    }
    let count = |lo: f64, hi: f64| residuals.iter().filter(|r| r.0 >= lo && r.0 < hi).count();
    println!("\nhistogram of |residual|:");
    for (lo, hi) in [
        (0.0, 0.01),
        (0.01, 0.05),
        (0.05, 0.1),
        (0.1, 0.25),
        (0.25, 0.5),
        (0.5, 1.0),
        (1.0, 2.0),
        (2.0, 5.0),
        (5.0, 1e9),
    ] {
        println!("  [{lo:5.2}, {hi:5.2})  {}", count(lo, hi));
    }
    let m = residuals.len();
    if m > 0 {
        let p = |q: f64| residuals[((1.0 - q) * (m - 1) as f64) as usize].0;
        println!("\np50 {:.3}  p90 {:.3}  p99 {:.3}  max {:.3}", p(0.5), p(0.9), p(0.99), residuals[0].0);
    }
}
