//! What is under the crossings that lift the road network?
//!
//! `solve::graph::build_crossings` turns every derived crossing into a demand
//! that the *upper* side sit `clearance_over_m + DECK_THICKNESS_M` above the
//! lower side's **road surface**. Two populations in that set are suspect:
//!
//! - **The lower side is in a bore.** A tunnel is under the ground, and a road
//!   crossing the ground above it is at grade. The constraint that matters is
//!   the bore's cover, not a clearance over its carriageway — but the demand is
//!   written against the carriageway, so the surface road is asked to stand a
//!   whole tunnel-plus-slab above a road that is already underground.
//! - **Both sides are at grade in the data.** Then the ordering came from the
//!   solved surfaces being `SEPARATION_M` apart, and a railway whose profile
//!   floats manufactures the separation that manufactures the crossing. A level
//!   crossing is common and ordinary; a phantom grade separation over one is
//!   not.
//!
//! This counts both, and measures the lift each demand implies against where
//! the upper road's own profile sits, so the size of the disturbance is visible
//! rather than inferred.
//!
//! Usage: cargo run --release --example undercross_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble;
use arpentry_server::priors::{Kind, Modality, DECK_THICKNESS_M};
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

    let mut bore_lower: Vec<Row> = Vec::new();
    let mut both_grade: Vec<Row> = Vec::new();
    let mut rail_involved: Vec<Row> = Vec::new();
    let mut plain: usize = 0;

    for c in &solved.crossings {
        let Some(lower) = c.lower else { continue };
        let (uc, lc) = (&scene.corridors[c.upper as usize], &scene.corridors[lower as usize]);
        let (Some(up), Some(lp)) = (solved.profile(c.upper), solved.profile(lower)) else {
            continue;
        };
        let lower_kind_at = lc
            .spans
            .iter()
            .find(|s| c.lower_arc >= s.arc0 && c.lower_arc <= s.arc1)
            .map_or(SpanKind::Grade, |s| s.kind);
        // The demand the graph writes, against where the upper road actually is.
        let demand = lp.road_at_arc(c.lower_arc) + c.lower_kind.prior().clearance_over_m
            + DECK_THICKNESS_M;
        let row = Row {
            lift: demand - up.road_at_arc(c.upper_arc),
            // How far the upper road would then stand over its own terrain.
            over_terrain: demand - terrain_at(up, c.upper_arc),
            upper: format!("{:?}", uc.kind),
            lower: format!("{:?}", lc.kind),
            lon: c.point.x,
            lat: c.point.y,
        };
        if lower_kind_at == SpanKind::Tunnel {
            bore_lower.push(row);
        } else if c.upper_level == 0 && c.lower_level == 0 {
            let rail = uc.kind.modality() == Modality::Rail || lc.kind.modality() == Modality::Rail;
            if rail {
                rail_involved.push(row);
            } else {
                both_grade.push(row);
            }
        } else {
            plain += 1;
        }
    }

    println!("{} derived crossings", solved.crossings.len());
    println!("  {plain} ordered by a level hint over a non-tunnel lower side");
    report("lower side is IN A BORE (a tunnel under the crossing road)", &mut bore_lower);
    report("both sides AT GRADE in the data, one is RAIL (a level crossing?)", &mut rail_involved);
    report("both sides AT GRADE in the data, road over road", &mut both_grade);
}

/// The raw terrain the upper profile sampled at `arc` — nearest node, which is
/// as precise as the profile itself is.
fn terrain_at(p: &solve::Profile, arc_m: f64) -> f64 {
    let a = p.arc();
    let mut best = 0usize;
    for i in 0..a.len() {
        if (a[i] - arc_m).abs() < (a[best] - arc_m).abs() {
            best = i;
        }
    }
    p.terrain_m().get(best).copied().unwrap_or(0.0)
}

struct Row {
    /// How far the demand asks the upper road to rise from where it solved.
    lift: f64,
    /// Where the demand puts it relative to its own raw terrain.
    over_terrain: f64,
    upper: String,
    lower: String,
    lon: f64,
    lat: f64,
}

fn report(name: &str, v: &mut Vec<Row>) {
    if v.is_empty() {
        println!("\n{name}: none");
        return;
    }
    let n = v.len();
    println!("\n{name}: {n} crossings");
    for (label, f) in [
        ("demanded − solved profile", (|r: &Row| r.lift) as fn(&Row) -> f64),
        ("demanded − own raw terrain", |r: &Row| r.over_terrain),
    ] {
        let mut d: Vec<f64> = v.iter().map(f).collect();
        d.sort_by(f64::total_cmp);
        let q = |x: f64| d[((d.len() as f64 - 1.0) * x) as usize];
        println!(
            "  {label:<26} p50 {:>7.2}  p75 {:>7.2}  p95 {:>7.2}  max {:>7.2}   >1 m: {:>5.1}%",
            q(0.50), q(0.75), q(0.95), q(1.0),
            100.0 * d.iter().filter(|&&x| x > 1.0).count() as f64 / n as f64
        );
    }
    v.sort_by(|a, b| b.over_terrain.total_cmp(&a.over_terrain));
    for r in v.iter().take(5) {
        println!(
            "    demand stands {:>6.2} m over its own terrain (lift {:>6.2} m): {} over {} at {:.6},{:.6}",
            r.over_terrain, r.lift, r.upper, r.lower, r.lon, r.lat
        );
    }
}
