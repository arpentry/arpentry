//! Where does a railway's *solved* height leave the ground, and which stage put
//! it there?
//!
//! `examples/rail_standoff` measures the emitted archive and reports railways
//! metres above the drawn terrain. That is the symptom. This asks the model the
//! same question at three places, so the stage responsible is identifiable
//! rather than guessed:
//!
//! - `road_m − terrain_m` at every at-grade node: what the *solve* asked for.
//! - the same after the fused relaxation, per stratum, since a senior alignment
//!   is not supposed to move for anything junior.
//! - whether the ground layer for stratum R actually benched under it — the
//!   `MAX_BENCH_FACE_M` cap declines a bench and leaves air (priors.rs).
//!
//! Usage: cargo run --release --example rail_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::ground;
use arpentry_server::priors::{Kind, Stratum};
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
    let stack = ground::derive(&scene, &solved, Some(&terrain), 0);

    // Per class: the solved standoff at at-grade nodes, and what the ground
    // stack did about it.
    let mut per_class: std::collections::BTreeMap<String, Vec<Sample>> = Default::default();
    let mut scratch: Vec<u32> = Vec::new();
    for c in &scene.corridors {
        if c.kind.modality() != arpentry_server::priors::Modality::Rail {
            continue;
        }
        let Some(p) = solved.profile(c.id) else {
            *per_class.entry(format!("{:?} NO PROFILE", c.kind)).or_default() = Vec::new();
            continue;
        };
        let (arc, road, terr, nodes) = (p.arc(), p.road_m(), p.terrain_m(), p.nodes());
        let derived = solved.structures.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        for i in 0..arc.len() {
            if !p.at_grade()[i] {
                continue; // the profile knows this one is on a structure
            }
            let annotated = c
                .spans
                .iter()
                .any(|s| s.kind != SpanKind::Grade && arc[i] >= s.arc0 && arc[i] <= s.arc1);
            if annotated {
                continue; // a deck or a bore is meant to leave the ground
            }
            let (lon, lat) = (nodes[i].x, nodes[i].y);
            // What the ground came out at, under the rail's own stratum and
            // under the whole stack: if R benched, the first already matches.
            let g_all = stack.height(lon, lat, terr[i], 0.0, &mut scratch);
            let benched = stack
                .layer(Stratum::R)
                .map(|l| l.covers(lon, lat, &mut scratch))
                .unwrap_or(false);
            per_class.entry(class_name(c.kind)).or_default().push(Sample {
                solved: road[i] - terr[i],
                drawn: road[i] - g_all,
                benched,
                // Whether `solve::structures` — which already runs, and which
                // nothing downstream consumes — says a deck belongs here.
                derived_structure: derived
                    .iter()
                    .any(|s| arc[i] >= s.arc0 && arc[i] <= s.arc1),
                lon,
                lat,
            });
        }
    }

    for (class, v) in &mut per_class {
        report(class, v);
    }
}

struct Sample {
    /// The solve's own answer: profile minus the raw terrain it sampled.
    solved: f64,
    /// After the ground answered: profile minus the engineered ground.
    drawn: f64,
    /// Whether stratum R's layer claims to cover this point at all.
    benched: bool,
    /// Whether `solve::structures` derived a structure run over this node.
    derived_structure: bool,
    lon: f64,
    lat: f64,
}

fn class_name(k: Kind) -> String {
    format!("{k:?}")
}

fn report(name: &str, v: &mut Vec<Sample>) {
    if v.is_empty() {
        println!("\n{name}: no at-grade nodes");
        return;
    }
    let n = v.len();
    let pct = |c: usize| 100.0 * c as f64 / n as f64;
    let floats: Vec<&Sample> = v.iter().filter(|s| s.drawn > 4.0).collect();
    println!(
        "\n{name}: {n} at-grade unannotated nodes, {:.1}% inside an R-layer footprint",
        pct(v.iter().filter(|s| s.benched).count())
    );
    println!(
        "  floating >4 m: {} nodes ({:.1}%), of which {:.1}% benched and {:.1}% covered by a *derived* structure run",
        floats.len(),
        pct(floats.len()),
        100.0 * floats.iter().filter(|s| s.benched).count() as f64 / floats.len().max(1) as f64,
        100.0 * floats.iter().filter(|s| s.derived_structure).count() as f64
            / floats.len().max(1) as f64,
    );
    for (label, f) in [
        ("solved − raw terrain", (|s: &Sample| s.solved) as fn(&Sample) -> f64),
        ("solved − engineered ground", |s: &Sample| s.drawn),
    ] {
        let mut d: Vec<f64> = v.iter().map(f).collect();
        d.sort_by(f64::total_cmp);
        let q = |x: f64| d[((d.len() as f64 - 1.0) * x) as usize];
        println!(
            "  {label:<28} p05 {:>7.2}  p50 {:>7.2}  p95 {:>7.2}  p99 {:>7.2}  max {:>7.2}  min {:>7.2}  |>4 m| {:>5.1}%",
            q(0.05), q(0.50), q(0.95), q(0.99), q(1.0), q(0.0),
            100.0 * d.iter().filter(|x| x.abs() > 4.0).count() as f64 / d.len() as f64
        );
    }
    v.sort_by(|a, b| b.drawn.total_cmp(&a.drawn));
    for s in v.iter().take(3) {
        println!(
            "    worst float {:>7.2} m (solve asked {:>7.2} m, benched {}) at {:.6},{:.6}",
            s.drawn, s.solved, s.benched, s.lon, s.lat
        );
    }
}
