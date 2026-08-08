//! Which annotated structures does the short-span demotion delete, and by how
//! much did each miss the terrain test?
//!
//! `solve::reconcile_short_spans` keeps a sub-`MIN_STRUCTURE_M` span only where
//! the ground departs its end-to-end chord by more than `SHORT_STRUCTURE_DIP_M`
//! at a quarter, half or three-quarter point. This prints the spans assemble
//! produced, the departure each short one measured, and the verdict — so a
//! structure that vanished between the data and the drawing is attributable.
//!
//! Usage: cargo run --release --example shortspan_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor_id]

use arpentry_server::assemble;
use arpentry_server::priors::{Modality, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::solve;
use arpentry_server::dem::Dem;
use geo_types::Coord;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let only: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    let scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let over = solve::crossings::spans_over_a_corridor(&scene);
    let mut dem = Dem::open(&terrain).expect("dem");
    let z = 16u8;

    let (mut kept, mut lost, mut lost_m) = (0usize, 0usize, 0.0f64);
    for (ci, c) in scene.corridors.iter().enumerate() {
        if let Some(want) = only {
            if c.id != want {
                continue;
            }
            println!("corridor #{} {:?}  spans as assembled:", c.id, c.kind);
        }
        for (si, s) in c.spans.iter().enumerate() {
            if s.kind == SpanKind::Grade {
                continue;
            }
            let len = s.arc1 - s.arc0;
            let short = len < MIN_STRUCTURE_M;
            // Replicate the demotion test's sampling.
            let mut at = |t: f64| {
                let sa = s.arc0 + len * t;
                let i = c.arc.partition_point(|&x| x < sa).clamp(1, c.arc.len() - 1);
                let (a0, a1) = (c.arc[i - 1], c.arc[i]);
                let f = if a1 > a0 { ((sa - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
                let p = Coord {
                    x: c.nodes[i - 1].x + (c.nodes[i].x - c.nodes[i - 1].x) * f,
                    y: c.nodes[i - 1].y + (c.nodes[i].y - c.nodes[i - 1].y) * f,
                };
                solve::reference_surface(&mut dem, z, p.x, p.y)
            };
            let (h0, h1) = (at(0.0), at(1.0));
            let depart = (1..=3)
                .map(|k| {
                    let t = k as f64 / 4.0;
                    let chord = h0 + (h1 - h0) * t;
                    let ground = at(t);
                    match s.kind {
                        SpanKind::Bridge => chord - ground,
                        SpanKind::Tunnel => ground - chord,
                        SpanKind::Grade => 0.0,
                    }
                })
                .fold(f64::NEG_INFINITY, f64::max);
            let crosses = over[ci][si];
            let survives = !short || crosses || depart > SHORT_STRUCTURE_DIP_M;
            if only.is_some() {
                println!(
                    "  {:?} L{} arc {:.1}..{:.1} ({:.1} m)  {}  max departure {:+.2} m  -> {}",
                    s.kind, s.level, s.arc0, s.arc1, len,
                    if short { "SHORT" } else { "long " },
                    depart,
                    if !short { "kept (long)" } else if crosses { "KEPT: crosses a corridor" }
                    else if survives { "kept (dip)" } else { "DEMOTED to grade" },
                );
            } else if c.kind.modality() == Modality::Rail && short {
                if survives { kept += 1 } else { lost += 1; lost_m += len }
            }
        }
    }
    if only.is_none() {
        println!("rail short spans: {kept} kept, {lost} demoted ({lost_m:.0} m of annotated structure)");
    }
}
