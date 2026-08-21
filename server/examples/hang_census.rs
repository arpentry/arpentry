//! Which at-grade roads stand metres off their own ground, and what holds
//! them there?
//!
//! `contact.kerb_lip` and `order.deck_above_carriageway` both fire on the same
//! shape — a band paved at one height with the drawn ground far below its kerb
//! — and neither can say why the profile is up there. This walks every
//! corridor's solved profile, finds the maximal contiguous **at-grade** runs
//! standing more than `ABSORB_STANDOFF_M` off the reference, and attributes
//! each to what its ends touch:
//!
//! - **span-adjacent** — the run abuts a structure span of its own corridor,
//!   which is what `portals::absorb_hanging_approaches` already annexes;
//! - **on a structure junction** — the run reaches a junction whose *partner*
//!   is inside a structure span there, so the weld pinned this corridor to a
//!   deck height (the Chauderon stub cluster: a service road joining Route de
//!   Chernex on its bridge over the slot);
//! - **junction cluster** — the run reaches a junction whose partner is itself
//!   hanging, one link further from whatever started it;
//! - **free** — neither end explains it.
//!
//! Usage: cargo run --release --example hang_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor]

use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::{assemble, solve};

/// The standoff that makes a run "hanging" — `profile::ABSORB_STANDOFF_M`,
/// which is crate-private, so it is restated here.
const HANG_M: f64 = 5.0;

/// How close a run end must come to a junction to count as reaching it.
const JOIN_EPS_M: f64 = 1.0;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let dump: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // Pass 1: the hanging runs themselves, as (corridor, arc0, arc1, worst).
    let mut runs: Vec<(u32, f64, f64, f64)> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (arc, road, terr, ag) = (p.arc(), p.road_m(), p.terrain_m(), p.at_grade());
        let mut k = 0;
        while k < arc.len() {
            if !ag[k] || road[k] - terr[k] <= HANG_M {
                k += 1;
                continue;
            }
            let start = k;
            let mut worst = 0.0f64;
            while k < arc.len() && ag[k] && road[k] - terr[k] > HANG_M {
                worst = worst.max(road[k] - terr[k]);
                k += 1;
            }
            runs.push((c.id, arc[start], arc[k - 1], worst));
        }
    }
    // Which corridors hang anywhere — for the "cluster" test.
    let mut hanging: Vec<bool> = vec![false; scene.corridors.len()];
    for r in &runs {
        hanging[r.0 as usize] = true;
    }

    let mut by_cause: Vec<(&str, usize, f64, f64)> =
        vec![("span-adjacent", 0, 0.0, 0.0), ("on a structure junction", 0, 0.0, 0.0),
             ("junction cluster", 0, 0.0, 0.0), ("free", 0, 0.0, 0.0)];
    // (worst, lon, lat, corridor, len, cause, detail)
    let mut worst_sites: Vec<(f64, f64, f64, u32, f64, &'static str, String)> = Vec::new();

    for &(id, a0, a1, worst) in &runs {
        let c = &scene.corridors[id as usize];
        let Some(p) = solved.profile(id) else { continue };
        let pt = p.point_at_arc(0.5 * (a0 + a1));
        if pt.x < bbox.west || pt.x > bbox.east || pt.y < bbox.south || pt.y > bbox.north {
            continue;
        }
        let len = a1 - a0;
        // Does the run abut a structure span of its own corridor?
        let span_adjacent = c.spans.iter().any(|s| {
            s.kind != SpanKind::Grade
                && ((s.arc1 - a0).abs() < JOIN_EPS_M || (s.arc0 - a1).abs() < JOIN_EPS_M)
        });
        // Junctions this run reaches, and what the partner is doing there.
        let mut on_structure: Option<String> = None;
        let mut in_cluster: Option<String> = None;
        for j in &scene.junctions {
            let Some(me) = j.members.iter().find(|m| m.corridor == id) else { continue };
            if me.arc < a0 - JOIN_EPS_M || me.arc > a1 + JOIN_EPS_M {
                continue;
            }
            for o in j.members.iter().filter(|m| m.corridor != id) {
                let oc = &scene.corridors[o.corridor as usize];
                let carried = oc
                    .spans
                    .iter()
                    .any(|s| s.kind != SpanKind::Grade && o.arc >= s.arc0 && o.arc <= s.arc1);
                if carried {
                    on_structure = Some(format!(
                        "welded at arc {:.0} to #{} inside its {:?} span",
                        me.arc,
                        o.corridor,
                        oc.spans
                            .iter()
                            .find(|s| o.arc >= s.arc0 && o.arc <= s.arc1)
                            .map_or(SpanKind::Grade, |s| s.kind)
                    ));
                } else if hanging[o.corridor as usize] && in_cluster.is_none() {
                    in_cluster =
                        Some(format!("welded at arc {:.0} to #{}, also hanging", me.arc, o.corridor));
                }
            }
        }
        let (cause, detail) = if span_adjacent {
            (0, "abuts its own structure span".to_string())
        } else if let Some(d) = on_structure {
            (1, d)
        } else if let Some(d) = in_cluster {
            (2, d)
        } else {
            (3, "nothing structural at either end".to_string())
        };
        by_cause[cause].1 += 1;
        by_cause[cause].2 += len;
        by_cause[cause].3 = by_cause[cause].3.max(worst);
        let name = by_cause[cause].0;
        worst_sites.push((worst, pt.x, pt.y, id, len, name, detail));
        if dump == Some(id) {
            println!(
                "  #{id} [{a0:.0}, {a1:.0}] {len:.0} m, worst {worst:.2} m at {:.6},{:.6} — {name}",
                pt.x, pt.y
            );
        }
    }

    println!("\nHANGING AT-GRADE RUNS (standoff > {HANG_M:.0} m), {} of them", worst_sites.len());
    println!("  {:<26} {:>6} {:>10} {:>9}", "cause", "runs", "metres", "worst");
    for (name, n, m, w) in &by_cause {
        println!("  {name:<26} {n:>6} {m:>10.0} {w:>9.2}");
    }

    worst_sites.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("\nWORST 25 BY STANDOFF");
    for (worst, lon, lat, id, len, cause, detail) in worst_sites.iter().take(25) {
        println!("  {worst:6.2} m  {lon:.6},{lat:.6}  #{id} ({len:.0} m)  {cause} — {detail}");
    }
}
