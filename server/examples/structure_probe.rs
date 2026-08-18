//! The anatomy of `structure.annotated_lost` and `structure.edge_drift`: per
//! annotated span, what the derive covers, what it trims at the ends, and what
//! it censors whole — with the gap profile that says *why*.
//!
//! The roadmap's item-2 question ("judge the §4.5 switch") turns on whether the
//! 93 % `annotated_lost` is one family or several. This splits it:
//!
//! - **end trim** — a derived run covers the span's middle but ends inside it.
//!   S5 says this is the correction (the road reaches the ground before the
//!   mapper's split point), unless the trim is most of the span.
//! - **whole loss** — no derived run of the span's kind overlaps it at all.
//!   For each, the probe prints the span's max departure from the terrain, so
//!   a threshold near-miss (an embankment-height "bridge") is distinguishable
//!   from a solve that draped the road (departure ≈ 0) and from a censored
//!   real structure.
//!
//! Usage:
//!   cargo run --release --example structure_probe -- \
//!       <segment.parquet> <w,s,e,n> <terrain.pmtiles> [--at lon,lat]
//!
//! Default is the census; `--at` dumps spans, runs and the per-node gap for
//! every corridor passing within ~150 m of the point.

use arpentry_server::assemble;
use arpentry_server::project::Bounds;
use arpentry_server::scene::{Span, SpanKind};
use arpentry_server::solve;
use arpentry_server::solve::structures::{StructureRun, BORE_COVER_M, DECK_STANDOFF_M};

/// Same as `verify::model::structures::EPS_M` — overlap shorter than this is
/// quantization at a shared edge, not agreement.
const EPS_M: f64 = 0.5;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let at: Option<(f64, f64)> = a.iter().position(|s| s == "--at").map(|i| {
        let (lon, lat) = a[i + 1].split_once(',').expect("--at lon,lat");
        (lon.trim().parse().unwrap(), lat.trim().parse().unwrap())
    });

    let segments = std::path::PathBuf::from(&a[0]);
    let water = segments.with_file_name("water.parquet");
    let mut scene = assemble::run(&segments, water.exists().then_some(water.as_path()), &bbox)
        .expect("assemble");
    let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
    let solved = solve::run(&mut scene, Some(&terrain), 16, threads).expect("solve");

    if let Some((lon, lat)) = at {
        site(&scene, &solved, lon, lat);
        return;
    }
    census(&scene, &solved);
}

/// The population split the verify metric cannot show: end trim vs whole loss,
/// with departure stats for the losses.
fn census(scene: &arpentry_server::scene::SceneGraph, solved: &solve::SolvedModel) {
    // metres by (kind, category)
    let mut covered_m = 0.0f64;
    let mut trim_m = 0.0f64;
    let mut trim_spans = 0usize;
    let mut loss_m = 0.0f64;
    let mut loss_spans = 0usize;
    let mut spans_total = 0usize;
    let mut annotated_m = 0.0f64;
    // Whole losses bucketed by how far the span's max departure missed the
    // derive threshold, in metres of centerline.
    let mut loss_by_depart: std::collections::BTreeMap<&'static str, (usize, f64)> =
        Default::default();
    let mut losses: Vec<(f64, f64, String)> = Vec::new(); // (len, max_depart, note)
    let mut trims: Vec<(f64, String)> = Vec::new(); // (missing, note)

    for c in &scene.corridors {
        let runs = solved.structures.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        let annotated: Vec<&Span> =
            scene.annotated(c.id).iter().filter(|s| s.kind != SpanKind::Grade).collect();
        let Some(p) = solved.profile(c.id) else { continue };

        for s in &annotated {
            let len = s.arc1 - s.arc0;
            spans_total += 1;
            annotated_m += len;
            let covered = overlap_with(runs, s.arc0, s.arc1, s.kind);
            let missing = (len - covered).max(0.0);
            covered_m += covered;
            if missing <= EPS_M {
                continue;
            }
            // Max departure from the terrain over the span, in the span's own
            // sense, against the derive's threshold for its kind.
            let depart = max_departure(p, s);
            let threshold = match s.kind {
                SpanKind::Bridge => DECK_STANDOFF_M,
                _ => BORE_COVER_M,
            };
            let mid = p.point_at_arc(0.5 * (s.arc0 + s.arc1));
            let note = format!(
                "#{:<6} {:<26} {:?} {:6.1} m  missing {:6.1}  depart {:+6.2} (thr {:.1})  ({:.5},{:.5})",
                c.id,
                format!("{:?}", c.kind),
                s.kind,
                len,
                missing,
                depart,
                threshold,
                mid.x,
                mid.y
            );
            if covered > EPS_M {
                trim_m += missing;
                trim_spans += 1;
                trims.push((missing, note));
            } else {
                loss_m += missing;
                loss_spans += 1;
                let bucket = if depart >= threshold {
                    "reaches threshold (run elsewhere/coalesce?)"
                } else if depart >= threshold - 1.5 {
                    "near miss (within 1.5 m of threshold)"
                } else if depart >= 1.0 {
                    "embankment/cutting-height (1 m .. thr-1.5)"
                } else {
                    "draped (max departure < 1 m)"
                };
                let e = loss_by_depart.entry(bucket).or_insert((0, 0.0));
                e.0 += 1;
                e.1 += len;
                losses.push((len, depart, note));
            }
        }
    }

    println!("annotated non-grade spans: {spans_total}, {annotated_m:.0} m");
    println!("  covered by derived runs: {covered_m:.0} m");
    println!("  end-trimmed:   {trim_spans:4} spans, {trim_m:.0} m missing");
    println!("  whole losses:  {loss_spans:4} spans, {loss_m:.0} m missing");
    println!("\nwhole losses by max departure from terrain:");
    for (bucket, (n, m)) in &loss_by_depart {
        println!("  {bucket:<44} {n:4} spans  {m:8.0} m");
    }

    losses.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
    println!("\nlargest whole losses:");
    for (_, _, note) in losses.iter().take(20) {
        println!("  {note}");
    }
    // The boundary population: how close do censored spans come to the
    // threshold, at full precision? Calibrates the ceiling-contact tolerance —
    // a licensed bore is *clamped to* exactly −BORE_COVER_M, and the question
    // is what the solver's convergence leaves there.
    println!("\nwhole losses within 0.25 m of their threshold (depart − threshold, mm):");
    let mut boundary: Vec<(f64, &str)> = losses
        .iter()
        .filter_map(|(len, depart, note)| {
            let thr = if note.contains("Tunnel") { BORE_COVER_M } else { DECK_STANDOFF_M };
            ((depart - thr).abs() <= 0.25).then(|| ((depart - thr) * 1000.0, note.as_str(), *len))
        })
        .map(|(mm, note, _)| (mm, note))
        .collect();
    boundary.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
    for (mm, note) in &boundary {
        println!("  {mm:+9.3} mm  {note}");
    }
    trims.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap());
    println!("\nlargest end trims:");
    for (_, note) in trims.iter().take(20) {
        println!("  {note}");
    }
}

/// Max departure of the solved road from the terrain over the span, in the
/// span's own sense (bridge: above; tunnel: below), sampled per node plus the
/// interpolated ends.
fn max_departure(p: &solve::profile::Profile, s: &Span) -> f64 {
    let (arc, road, terrain) = (p.arc(), p.road_m(), p.terrain_m());
    let sense = |r: f64, t: f64| match s.kind {
        SpanKind::Tunnel => t - r,
        _ => r - t,
    };
    let mut best = f64::NEG_INFINITY;
    for i in 0..arc.len() {
        if arc[i] >= s.arc0 && arc[i] <= s.arc1 {
            best = best.max(sense(road[i], terrain[i]));
        }
    }
    for a in [s.arc0, s.arc1] {
        best = best.max(sense(p.road_at_arc(a), p.surface_at_arc(a)));
    }
    best
}

/// Metres of `[arc0, arc1]` covered by runs of the same kind (the verify
/// metric's own overlap rule).
fn overlap_with(runs: &[StructureRun], arc0: f64, arc1: f64, kind: SpanKind) -> f64 {
    runs.iter()
        .filter(|r| r.kind == kind)
        .map(|r| (arc1.min(r.arc1) - arc0.max(r.arc0)).max(0.0))
        .sum()
}

/// Everything about the corridors passing near a point: annotated spans,
/// derived runs, and the gap profile across the spans' extent.
fn site(scene: &arpentry_server::scene::SceneGraph, solved: &solve::SolvedModel, lon: f64, lat: f64) {
    let cos_lat = lat.to_radians().cos();
    let near = |c: &arpentry_server::scene::Corridor| {
        c.nodes.iter().any(|n| {
            let de = (n.x - lon) * cos_lat * 111_320.0;
            let dn = (n.y - lat) * 111_320.0;
            de * de + dn * dn < 150.0 * 150.0
        })
    };
    for c in scene.corridors.iter().filter(|c| near(c)) {
        let runs = solved.structures.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        let annotated: Vec<&Span> =
            scene.annotated(c.id).iter().filter(|s| s.kind != SpanKind::Grade).collect();
        if annotated.is_empty() && runs.is_empty() {
            continue;
        }
        let Some(p) = solved.profile(c.id) else { continue };
        println!(
            "\ncorridor #{} {:?} ({} nodes, {:.0} m)",
            c.id,
            c.kind,
            c.nodes.len(),
            c.arc.last().copied().unwrap_or(0.0)
        );
        for s in &annotated {
            println!(
                "  annotated {:?} L{}  arc {:8.1} .. {:8.1}  ({:.1} m)  max depart {:+.2} m",
                s.kind,
                s.level,
                s.arc0,
                s.arc1,
                s.arc1 - s.arc0,
                max_departure(p, s)
            );
        }
        for r in runs {
            println!(
                "  derived   {:?}     arc {:8.1} .. {:8.1}  ({:.1} m)",
                r.kind,
                r.arc0,
                r.arc1,
                r.arc1 - r.arc0
            );
        }
        // The gap profile over the union of annotated extents, padded a bit.
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for s in &annotated {
            lo = lo.min(s.arc0);
            hi = hi.max(s.arc1);
        }
        if !annotated.is_empty() {
            let (arc, road, terrain) = (p.arc(), p.road_m(), p.terrain_m());
            println!("      arc      road   terrain      gap");
            for i in 0..arc.len() {
                if arc[i] < lo - 40.0 || arc[i] > hi + 40.0 {
                    continue;
                }
                // Every 4th node keeps a long span readable; ends always print.
                if i % 4 != 0 && arc[i] > lo && arc[i] < hi {
                    continue;
                }
                println!(
                    "  {:9.1} {:9.2} {:9.2} {:+8.2}",
                    arc[i],
                    road[i],
                    terrain[i],
                    road[i] - terrain[i]
                );
            }
        }
    }
}
