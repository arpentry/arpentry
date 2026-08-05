//! Why is an annotated structure not implied by the solve?
//!
//! `verify::model::structures` reports that 92.5 % of annotated spans lose
//! *some* metres, and `gap_histogram` says the losses are flat-ground bridges
//! (at or below 1 m clear) and flat-ground tunnels (at or above the ground).
//! Neither says which spans vanish and which merely end somewhere else, and the
//! difference decides everything: S5 says a derived structure is *supposed* to
//! end where the road reaches the ground rather than where a mapper split the
//! way, so a moved end is the correction and a missing middle is the defect.
//!
//! So this reports metres against **metres annotated**, and attributes each
//! loss to *what the solve derived there*:
//!
//! - **upper of a crossing** — the demand exists and this corridor is the side
//!   that must climb. If the span is still lost the lift did not survive: the
//!   demand was dropped, the ramp could not reach, or something undid it.
//! - **lower of a crossing** — the demand exists and this corridor is the side
//!   that must go under. Within one stratum the projection is raise-only, so
//!   nothing sinks it: this is the bucket M8 is about.
//! - **plan crossing, no demand** — two alignments cross here and the
//!   derivation declined to order them (shared connector, coincident surfaces,
//!   junior lower side).
//! - **nothing crosses** — no feature in the scene passes here at all. The
//!   annotation's evidence left with the D stratum (a footpath), or is water
//!   the H datum does not model (a river), or is not a crossing at all.
//!
//! What it found on Montreux: 74 % of annotated structure metres are already
//! implied, the losses in the first two buckets are span *ends* (median 14 m of
//! a span whose middle survives), and the only rows that vanish outright are
//! "plan crossing, no demand" — 69 % of bridge and 98 % of tunnel metres in
//! them. That bucket was one bug, not the mass question: see
//! `crossing_census`.
//!
//! Usage: cargo run --release --example structure_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use std::collections::HashMap;

use arpentry_server::assemble;
use arpentry_server::priors::DECK_THICKNESS_M;
use arpentry_server::project::Bounds;
use arpentry_server::scene::{Corridor, Crossing, SceneGraph, Span, SpanKind};
use arpentry_server::solve::{self, SolvedModel};
use geo_types::Coord;

/// Overlap shorter than this is quantization at a shared edge, not agreement —
/// `verify::model::structures`' own epsilon, so the two tools count the same
/// spans as lost.
const EPS_M: f64 = 0.5;

/// How far along a corridor a crossing may sit from a span and still be the
/// constraint that span exists for. An annotation edge lands where a mapper
/// split the way, which S10 says is routinely tens of metres off the physical
/// structure.
const NEAR_M: f64 = 40.0;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // Crossings by corridor, from each side, so a span can ask what constraint
    // the solve derived for it.
    let mut as_upper: HashMap<u32, Vec<&Crossing>> = HashMap::new();
    let mut as_lower: HashMap<u32, Vec<&Crossing>> = HashMap::new();
    for x in &solved.crossings {
        as_upper.entry(x.upper).or_default().push(x);
        if let Some(l) = x.lower {
            as_lower.entry(l).or_default().push(x);
        }
    }

    // Per bucket: spans, metres annotated, metres of that the solve does not
    // imply. The denominator matters — "lost" against nothing is a number that
    // cannot be read.
    let mut tally: HashMap<(&'static str, &'static str), (usize, f64, f64)> = HashMap::new();
    let mut worst: Vec<(f64, String)> = Vec::new();
    // Per bucket, the anatomy: the gap the derivation reads at mid-span, and —
    // where a crossing exists — what it demanded against what it got.
    let mut gaps: HashMap<(&'static str, &'static str), Vec<f64>> = HashMap::new();
    let mut sep: HashMap<&'static str, Vec<(f64, f64)>> = HashMap::new();
    let mut why: HashMap<&'static str, usize> = HashMap::new();
    for c in &scene.corridors {
        let runs = solved.structures.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        for s in c.spans.iter().filter(|s| s.kind != SpanKind::Grade) {
            let covered: f64 = runs
                .iter()
                .filter(|r| r.kind == s.kind)
                .map(|r| (s.arc1.min(r.arc1) - s.arc0.max(r.arc0)).max(0.0))
                .sum();
            let missing = (s.arc1 - s.arc0 - covered).max(0.0);
            let bucket = if missing <= EPS_M {
                "kept"
            } else {
                classify(&scene, &solved, c, s, &as_upper, &as_lower)
            };
            let kind = if s.kind == SpanKind::Bridge { "Bridge" } else { "Tunnel" };
            let e = tally.entry((kind, bucket)).or_insert((0, 0.0, 0.0));
            e.0 += 1;
            e.1 += s.arc1 - s.arc0;
            e.2 += missing;
            if missing > EPS_M {
                // What the derivation reads where the annotation claims a
                // structure: the whole of its decision is this one number.
                if let Some(p) = solved.profile(c.id) {
                    let a = 0.5 * (s.arc0 + s.arc1);
                    gaps.entry((kind, bucket))
                        .or_default()
                        .push(p.road_at_arc(a) - p.surface_at_arc(a));
                }
                // Where a demand exists: what it asked for against what the
                // solve delivered at that very crossing.
                for x in as_upper.get(&c.id).into_iter().flatten() {
                    if !near_span(s, x.upper_arc) {
                        continue;
                    }
                    if let Some((got, want)) = separation(&solved, &scene, x) {
                        sep.entry("bridge span, this corridor above").or_default().push((got, want));
                    }
                }
                for x in as_lower.get(&c.id).into_iter().flatten() {
                    if !near_span(s, x.lower_arc) {
                        continue;
                    }
                    if let Some((got, want)) = separation(&solved, &scene, x) {
                        sep.entry("tunnel span, this corridor below").or_default().push((got, want));
                    }
                }
                if bucket == "plan crossing, no demand" {
                    if let Some(other) = plan_crossing(&scene, &solved, c, s) {
                        *why.entry(rejection(&scene, &solved, c, s, other)).or_insert(0) += 1;
                    }
                }
                let p = solved.profile(c.id).map(|p| p.point_at_arc(0.5 * (s.arc0 + s.arc1)));
                let (lon, lat) = p.map_or((0.0, 0.0), |p| (p.x, p.y));
                worst.push((
                    missing,
                    format!(
                        "{missing:8.0} m  {:?}  {bucket:<22}  {lon:.5},{lat:.5}  {} ({:?})",
                        s.kind, c.class_key, c.kind.stratum()
                    ),
                ));
            }
        }
    }

    println!("\nannotated span metres, by kind and by what the solve derived there\n");
    println!(
        "{:<8} {:<26} {:>7} {:>12} {:>12} {:>7}",
        "kind", "bucket", "spans", "annotated", "lost", "share"
    );
    let mut keys: Vec<_> = tally.keys().copied().collect();
    keys.sort();
    for k in keys {
        let (n, ann, lost) = tally[&k];
        println!(
            "{:<8} {:<26} {n:>7} {ann:>12.0} {lost:>12.0} {:>6.0}%",
            k.0,
            k.1,
            100.0 * lost / ann.max(1e-9)
        );
    }

    println!("\nthe gap `road − terrain` the derivation reads at mid-span (m)\n");
    println!("{:<8} {:<24} {:>7}  {:>6} {:>6} {:>6} {:>6}", "kind", "bucket", "spans", "p10", "p50", "p90", "max");
    let mut gkeys: Vec<_> = gaps.keys().copied().collect();
    gkeys.sort();
    for k in gkeys {
        let v = gaps.get_mut(&k).expect("present");
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let q = |f: f64| v[((v.len() as f64 - 1.0) * f) as usize];
        println!(
            "{:<8} {:<24} {:>7}  {:>6.1} {:>6.1} {:>6.1} {:>6.1}",
            k.0, k.1, v.len(), q(0.1), q(0.5), q(0.9), q(1.0)
        );
    }

    println!("\nwhat the crossing demanded against what the solve delivered (m)\n");
    let mut skeys: Vec<_> = sep.keys().copied().collect();
    skeys.sort();
    for k in skeys {
        let v = &sep[k];
        let mut short: Vec<f64> = v.iter().map(|(got, want)| want - got).collect();
        short.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let q = |f: f64| short[((short.len() as f64 - 1.0) * f) as usize];
        let met = short.iter().filter(|&&d| d <= 0.01).count();
        println!(
            "{k:<34} {:>5} crossings, {met} met\n     shortfall (want − got)  p10 {:>6.1}  p50 {:>6.1}  p90 {:>6.1}  max {:>6.1}",
            v.len(), q(0.1), q(0.5), q(0.9), q(1.0)
        );
    }

    if !why.is_empty() {
        println!("\nwhy the derivation ordered no crossing where two alignments cross\n");
        let mut wkeys: Vec<_> = why.keys().copied().collect();
        wkeys.sort();
        for k in wkeys {
            println!("  {:<40} {:>5} spans", k, why[k]);
        }
    }

    worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("\nthe 20 largest losses\n");
    for (_, line) in worst.iter().take(20) {
        println!("{line}");
    }
}

/// Whether a crossing arc sits close enough to this span to be its reason.
fn near_span(s: &Span, arc: f64) -> bool {
    arc >= s.arc0 - NEAR_M && arc <= s.arc1 + NEAR_M
}

/// The vertical separation a crossing achieved, against the one it demanded.
/// `None` where either side has no profile to read.
fn separation(solved: &SolvedModel, scene: &SceneGraph, x: &Crossing) -> Option<(f64, f64)> {
    let up = solved.profile(x.upper)?.road_at_arc(x.upper_arc);
    let lower = x.lower?;
    let lo = solved.profile(lower)?.road_at_arc(x.lower_arc);
    let want = scene.corridors[lower as usize].kind.prior().clearance_over_m + DECK_THICKNESS_M;
    Some((up - lo, want))
}

/// Why `solve::crossings::derive` ordered nothing where two alignments plainly
/// cross. The three rejections it can make, re-posed against the same pair.
fn rejection(
    scene: &SceneGraph,
    solved: &SolvedModel,
    c: &Corridor,
    s: &Span,
    other_id: u32,
) -> &'static str {
    let other = &scene.corridors[other_id as usize];
    if c.connectors.iter().any(|k| other.connectors.binary_search(k).is_ok()) {
        // *Where* they share it is the question the test does not ask. Two
        // corridors meeting at one end and crossing at the other share a
        // connector and are not a junction here.
        let mid = 0.5 * (s.arc0 + s.arc1);
        let d = solved.profile(c.id).map_or(f64::INFINITY, |p| {
            let pt = p.point_at_arc(mid);
            scene
                .junctions
                .iter()
                .filter(|j| {
                    j.members.iter().any(|m| m.corridor == c.id)
                        && j.members.iter().any(|m| m.corridor == other_id)
                })
                .map(|j| {
                    let dx = (j.point.x - pt.x) * c.cos_lat * arpentry_server::scene::DEG_M;
                    let dy = (j.point.y - pt.y) * arpentry_server::scene::DEG_M;
                    (dx * dx + dy * dy).sqrt()
                })
                .fold(f64::INFINITY, f64::min)
        });
        return match d {
            d if d < 20.0 => "share a connector, and meet here (< 20 m)",
            d if d < 100.0 => "share a connector 20-100 m away",
            d if d.is_finite() => "share a connector, over 100 m away",
            _ => "share a connector at no common junction",
        };
    }
    // The level hints at the crossing. Equal ordinals send the decision to the
    // solved surfaces, and surfaces within SEPARATION_M are read as a braid.
    let mid = 0.5 * (s.arc0 + s.arc1);
    let level_c = s.level;
    let (Some(pc), Some(po)) = (solved.profile(c.id), solved.profile(other_id)) else {
        return "one side has no profile";
    };
    let pt = pc.point_at_arc(mid);
    let arc_o = po.arc_of(pt.x, pt.y);
    let level_o = other
        .spans
        .iter()
        .find(|t| arc_o >= t.arc0 && arc_o <= t.arc1)
        .map_or(0, |t| if t.kind == SpanKind::Grade { 0 } else { t.level });
    if level_c == level_o {
        let d = (pc.road_at_arc(mid) - po.road_at_arc(arc_o)).abs();
        if d < 3.0 {
            return "same level ordinal, surfaces coincident";
        }
        return "same level ordinal, surfaces apart";
    }
    // Ordered, so authority is what dropped it: the mover would have to be a
    // stratum that never owns this pair.
    let (upper, lower) = if level_c > level_o { (c, other) } else { (other, c) };
    if upper.kind.stratum() > lower.kind.stratum() {
        "the side that must move is junior to the side it crosses"
    } else {
        "ordered and same-stratum: derived elsewhere along the span"
    }
}

/// Which constraint the solve derived where this span is, if any.
fn classify(
    scene: &SceneGraph,
    solved: &SolvedModel,
    c: &Corridor,
    s: &Span,
    as_upper: &HashMap<u32, Vec<&Crossing>>,
    as_lower: &HashMap<u32, Vec<&Crossing>>,
) -> &'static str {
    let near = |arc: f64| arc >= s.arc0 - NEAR_M && arc <= s.arc1 + NEAR_M;
    let up = as_upper.get(&c.id).is_some_and(|xs| xs.iter().any(|x| near(x.upper_arc)));
    let lo = as_lower.get(&c.id).is_some_and(|xs| xs.iter().any(|x| near(x.lower_arc)));
    match (up, lo) {
        // A span can be both (a viaduct over one road and under another); the
        // side that matters is the one its own annotation claims.
        (true, _) if s.kind == SpanKind::Bridge => "upper of a crossing",
        (_, true) if s.kind == SpanKind::Tunnel => "lower of a crossing",
        (true, false) => "upper of a crossing",
        (false, true) => "lower of a crossing",
        (true, true) => "both sides",
        (false, false) => {
            // No demand. Does anything in the scene cross here at all?
            match plan_crossing(scene, solved, c, s) {
                Some(other) => {
                    if scene.corridors[other as usize].kind.stratum() == c.kind.stratum() {
                        "plan crossing, no demand"
                    } else {
                        "plan crossing, cross-stratum"
                    }
                }
                None => "nothing crosses",
            }
        }
    }
}

/// Any other corridor whose centerline crosses this span in plan, whatever the
/// derivation decided to do about it. Brute force over the scene's corridors
/// within the span's bounding box — this runs on the lost spans only.
fn plan_crossing(
    scene: &SceneGraph,
    solved: &SolvedModel,
    c: &Corridor,
    s: &Span,
) -> Option<u32> {
    let p = solved.profile(c.id)?;
    let (a0, a1) = (p.point_at_arc(s.arc0), p.point_at_arc(s.arc1));
    let bb = (
        a0.x.min(a1.x) - 0.001,
        a0.y.min(a1.y) - 0.001,
        a0.x.max(a1.x) + 0.001,
        a0.y.max(a1.y) + 0.001,
    );
    // The span's own polyline, node by node (the chord would miss a curve).
    let arc = p.arc();
    let lo = arc.partition_point(|&a| a < s.arc0).saturating_sub(1);
    let hi = arc.partition_point(|&a| a <= s.arc1).min(arc.len() - 1);
    let nodes = p.nodes();
    for other in &scene.corridors {
        if other.id == c.id {
            continue;
        }
        let ob = other.nodes.iter().fold(
            (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            |b, n| (b.0.min(n.x), b.1.min(n.y), b.2.max(n.x), b.3.max(n.y)),
        );
        if ob.0 > bb.2 || ob.2 < bb.0 || ob.1 > bb.3 || ob.3 < bb.1 {
            continue;
        }
        for i in lo..hi {
            for j in 0..other.nodes.len().saturating_sub(1) {
                if crosses(nodes[i], nodes[i + 1], other.nodes[j], other.nodes[j + 1], c.cos_lat) {
                    return Some(other.id);
                }
            }
        }
    }
    None
}

/// Whether two segments properly cross in the local metric frame.
fn crosses(a: Coord, b: Coord, c: Coord, d: Coord, cos_lat: f64) -> bool {
    let (ax, ay) = (a.x * cos_lat, a.y);
    let (bx, by) = (b.x * cos_lat, b.y);
    let (cx, cy) = (c.x * cos_lat, c.y);
    let (dx, dy) = (d.x * cos_lat, d.y);
    let (rx, ry) = (bx - ax, by - ay);
    let (sx, sy) = (dx - cx, dy - cy);
    let denom = rx * sy - ry * sx;
    if denom.abs() < 1e-18 {
        return false;
    }
    let t = ((cx - ax) * sy - (cy - ay) * sx) / denom;
    let u = ((cx - ax) * ry - (cy - ay) * rx) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}
