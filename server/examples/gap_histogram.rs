//! Where does a road stop standing on fill and start flying?
//!
//! `solve::structures` needs one number: how far the solved surface must stand
//! clear of the ground before a *deck* is the honest answer rather than an
//! embankment. Reasoning it out of the deck's own thickness gave 2.5 m and
//! 13,754 phantom bridges, because a street may leave its terrain by exactly
//! that much and still be on fill.
//!
//! `docs/VERIFICATION.md` §10: measure the anatomy of the population before
//! believing a number about it. This histograms `road − terrain` over every
//! at-grade node in the scene, split by whether the data annotated a structure
//! there, so the two modes can be seen rather than assumed.
//!
//! Usage: cargo run --release --example gap_histogram -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::solve;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // Two populations: nodes the data calls at-grade, and nodes it calls a
    // structure. If the threshold is real, they separate.
    let mut at_grade: Vec<f64> = Vec::new();
    let mut annotated: Vec<f64> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (arc, road, terr) = (p.arc(), p.road_m(), p.terrain_m());
        for i in 0..arc.len() {
            let inside = c
                .spans
                .iter()
                .any(|s| s.kind != SpanKind::Grade && arc[i] >= s.arc0 && arc[i] <= s.arc1);
            let gap = road[i] - terr[i];
            if inside {
                annotated.push(gap);
            } else {
                at_grade.push(gap);
            }
        }
    }

    report("at-grade in the data", &mut at_grade);
    report("annotated structure", &mut annotated);

    // What each candidate threshold would cost: how much at-grade road it calls
    // a deck, and how much annotated structure it misses.
    println!("\n{:>8}  {:>14}  {:>14}", "standoff", "at-grade→deck", "structure kept");
    for t in [2.5, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0, 20.0] {
        let false_deck = at_grade.iter().filter(|&&g| g > t).count() as f64 / at_grade.len() as f64;
        let kept = annotated.iter().filter(|&&g| g > t).count() as f64 / annotated.len() as f64;
        println!("{t:>8.1}  {:>13.2}%  {:>13.2}%", false_deck * 100.0, kept * 100.0);
    }
}

fn report(name: &str, v: &mut Vec<f64>) {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = |f: f64| v[((v.len() as f64 - 1.0) * f) as usize];
    println!(
        "\n{name}: {} nodes\n  p05 {:.1}  p25 {:.1}  p50 {:.1}  p75 {:.1}  p90 {:.1}  p95 {:.1}  p99 {:.1}  max {:.1}",
        v.len(),
        q(0.05), q(0.25), q(0.50), q(0.75), q(0.90), q(0.95), q(0.99), q(1.0)
    );
    // The histogram itself, in metre bins, so a second mode is visible rather
    // than inferred from quantiles.
    println!("  gap (m)   share");
    for lo in [-4.0, -2.0, 0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0] {
        let hi = match lo {
            -4.0 => -2.0,
            -2.0 => 0.0,
            24.0 => f64::INFINITY,
            x if x < 4.0 => x + 1.0,
            x if x < 8.0 => x + 2.0,
            x => x + 8.0,
        };
        let n = v.iter().filter(|&&g| g >= lo && g < hi).count();
        let share = 100.0 * n as f64 / v.len() as f64;
        let bar = "#".repeat((share * 0.8) as usize);
        println!("  {lo:>5.0}..{hi:<5.0} {share:>6.2}%  {bar}");
    }
}
