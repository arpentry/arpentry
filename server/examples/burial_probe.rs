//! Where does the drawn ground end up *over* a railway, and which layer put it
//! there?
//!
//! `rail_standoff` is one-sided (higher_is_worse), so it cannot see a railway
//! buried under its own ground — and giving rail a bench moved a third of the
//! Montreux funicular's float into exactly that blind spot. This walks every
//! rail node, structure spans included (which `rail_probe` skips), and
//! attributes each burial to the stratum whose earthworks raised the ground
//! past the track: `ground::GroundStack::height_through` gives groundₙ, so the
//! layer that did it is a diff, not a guess.
//!
//! Usage: cargo run --release --example burial_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::ground;
use arpentry_server::priors::{Kind, Modality};
use arpentry_server::project::Bounds;
use arpentry_server::scene::SpanKind;
use arpentry_server::solve;

/// Ground this far over the track counts as buried.
const BURIED_M: f64 = 2.0;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");
    let stack = ground::derive(&scene, &solved, Some(&terrain), 0);
    let strata: Vec<String> = stack.layers().iter().map(|l| format!("{:?}", l.stratum)).collect();
    println!("ground layers, in imprint order: {}", strata.join(" -> "));

    // (span kind, attributed layer) -> count, plus the worst case seen.
    let mut tally: std::collections::BTreeMap<(String, String), (usize, f64, f64, f64, Vec<f64>, f64)> =
        Default::default();
    let mut scratch: Vec<u32> = Vec::new();
    let (mut nodes_seen, mut buried) = (0usize, 0usize);

    for c in &scene.corridors {
        if c.kind.modality() != Modality::Rail {
            continue;
        }
        let Some(p) = solved.profile(c.id) else { continue };
        let (arc, road, terr, pts) = (p.arc(), p.road_m(), p.terrain_m(), p.nodes());
        for i in 0..arc.len() {
            nodes_seen += 1;
            let (lon, lat) = (pts[i].x, pts[i].y);
            // groundₙ after each layer, so the jump that buries it is a diff.
            let mut hs = Vec::with_capacity(stack.layers().len() + 1);
            hs.push(terr[i]);
            for n in 1..=stack.layers().len() {
                hs.push(stack.height_through(n, lon, lat, terr[i], 0.0, &mut scratch));
            }
            let g_all = *hs.last().expect("raw at least");
            if road[i] - g_all >= -BURIED_M {
                continue;
            }
            buried += 1;
            // The layer with the largest upward step is the one that did it;
            // none means the raw DEM was already over the track.
            let mut worst_step = 0.0;
            let mut who = "raw DEM".to_string();
            for n in 1..hs.len() {
                let step = hs[n] - hs[n - 1];
                if step > worst_step {
                    worst_step = step;
                    who = strata[n - 1].clone();
                }
            }
            let kind = c
                .spans
                .iter()
                .find(|s| arc[i] >= s.arc0 && arc[i] <= s.arc1)
                .map_or(SpanKind::Grade, |s| s.kind);
            let key = (format!("{:?}/{kind:?}", c.kind), who);
            let e = tally.entry(key).or_insert((0, 0.0, 0.0, 0.0, Vec::new(), 0.0));
            e.0 += 1;
            let depth = g_all - road[i];
            if depth > e.1 {
                *e = (e.0, depth, lon, lat, hs.clone(), road[i]);
            }
        }
    }

    println!("\n{buried} of {nodes_seen} rail nodes are buried more than {BURIED_M} m\n");
    println!("{:<34} {:<10} {:>7}  {:>8}  worst site", "class/span", "buried by", "nodes", "worst m");
    for ((kind, who), (n, depth, lon, lat, hs, road)) in &tally {
        let steps: Vec<String> = hs.iter().map(|h| format!("{h:.2}")).collect();
        println!("{kind:<34} {who:<10} {n:>7}  {depth:>8.2}  {lon:.6},{lat:.6}  road {road:.2}  ground raw->{}", steps.join("->"));
    }
    let _ = Kind::Rail(arpentry_server::priors::RailClass::Funicular);
}
