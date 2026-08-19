//! Every plan intersection in the scene, and what the derivation does with it.
//!
//! `solve::crossings::derive` rejects a pair of corridors that share a
//! connector, on the ground that features which *meet* reconcile through the
//! shared variable rather than through a clearance. The test is written over
//! the corridors' whole connector *sets*, so it answers "do these two ever
//! meet?" where the question is "do they meet **here**?" — and a corridor is a
//! spliced chain hundreds of metres long.
//!
//! `structure_probe` found the consequence on annotated structures: 60 spans,
//! 3,485 m, nearly all of it lost outright, and 45 of the 60 share their
//! connector at a junction that is not the crossing. This measures the whole
//! candidate population before anything is changed, because a rejection that
//! stops being made is a *new constraint* everywhere it fires — and §7's R1
//! says a change that multiplies clearance demands is the dangerous kind.
//!
//! Usage: cargo run --release --example crossing_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles>

use arpentry_server::assemble::{self, grid::GridIndex};
use arpentry_server::project::Bounds;
use arpentry_server::scene::{SceneGraph, SpanKind, DEG_M};
use arpentry_server::solve;
use geo_types::Coord;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");

    // The same walk `solve::crossings::derive` makes: every corridor edge in a
    // grid, every pair whose boxes overlap, every proper intersection.
    let mut edges: Vec<(u32, usize)> = Vec::new();
    let mut grid = GridIndex::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.insert(
                (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                edges.len() as u32,
            );
            edges.push((c.id, i));
        }
    }

    let mut total = 0u64;
    let mut shared = 0u64;
    // Distance from the intersection to the nearest junction the two corridors
    // are both members of — the number the identity test never looks at.
    let mut dist: Vec<f64> = Vec::new();
    let mut no_junction = 0u64;
    let mut ordered_by_hint = 0u64;
    let mut touch_here = 0u64;
    let mut apart = 0u64;
    let mut apart_ordered = 0u64;
    let mut listing: Vec<String> = Vec::new();
    let mut sites: std::collections::HashMap<(u32, u32, i64, i64), Vec<Coord>> =
        std::collections::HashMap::new();
    let mut candidates: Vec<u32> = Vec::new();
    for c in &scene.corridors {
        for i in 0..c.nodes.len().saturating_sub(1) {
            let (a, b) = (c.nodes[i], c.nodes[i + 1]);
            grid.query(
                (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                &mut candidates,
            );
            for &ei in candidates.iter() {
                let (oid, oi) = edges[ei as usize];
                if oid <= c.id {
                    continue;
                }
                let other = &scene.corridors[oid as usize];
                let (o_a, o_b) = (other.nodes[oi], other.nodes[oi + 1]);
                let Some((t, u)) = seg_intersect(a, b, o_a, o_b, c.cos_lat) else {
                    continue;
                };
                total += 1;
                // What `derive`'s dedup key keeps: one record per (upper,
                // lower, level pair). Two corridors that genuinely cross twice
                // — a ramp weaving over its mainline — collapse into one.
                let point0 = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                let near_node0 = |cc: &arpentry_server::scene::Corridor, e: (Coord, Coord)| {
                    metres(e.0, point0, cc.cos_lat).min(metres(e.1, point0, cc.cos_lat)) < 1.0
                };
                if !(near_node0(c, (a, b)) && near_node0(other, (o_a, o_b))) {
                    let la = level_at(&scene, c.id, c.arc[i] + t * (c.arc[i + 1] - c.arc[i]));
                    let lb = level_at(&scene, oid, other.arc[oi] + u * (other.arc[oi + 1] - other.arc[oi]));
                    if la != lb {
                        sites
                            .entry((c.id, oid, la, lb))
                            .or_insert_with(Vec::new)
                            .push(point0);
                    }
                }
                if !c.connectors.iter().any(|(k, _)| {
                    other.connectors.binary_search_by(|(o, _)| o.cmp(k)).is_ok()
                }) {
                    continue;
                }
                shared += 1;
                let point = Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
                // Do they *touch* here? Two alignments that meet share a
                // vertex — the connector is a point on both. Two that cross
                // pass between each other's vertices. This is the local
                // question the identity test answers globally.
                let near_node = |c: &arpentry_server::scene::Corridor| {
                    c.nodes.iter().any(|n| metres(*n, point, c.cos_lat) < 1.0)
                };
                if near_node(c) && near_node(other) {
                    touch_here += 1;
                } else {
                    apart += 1;
                    let jd = scene
                        .junctions
                        .iter()
                        .filter(|j| {
                            j.members.iter().any(|m| m.corridor == c.id)
                                && j.members.iter().any(|m| m.corridor == oid)
                        })
                        .map(|j| metres(j.point, point, c.cos_lat))
                        .fold(f64::INFINITY, f64::min);
                    let la = level_at(&scene, c.id, c.arc[i] + t * (c.arc[i + 1] - c.arc[i]));
                    let lb = level_at(&scene, oid, other.arc[oi] + u * (other.arc[oi + 1] - other.arc[oi]));
                    listing.push(format!(
                        "  {:.5},{:.5}  {:<12} L{la:<3} x {:<12} L{lb:<3}  shared junction {}",
                        point.x, point.y, c.class_key, other.class_key,
                        if jd.is_finite() { format!("{jd:.0} m away") } else { "none".into() }
                    ));
                    if level_at(&scene, c.id, c.arc[i] + t * (c.arc[i + 1] - c.arc[i]))
                        != level_at(
                            &scene,
                            oid,
                            other.arc[oi] + u * (other.arc[oi + 1] - other.arc[oi]),
                        )
                    {
                        apart_ordered += 1;
                    }
                }
                let d = scene
                    .junctions
                    .iter()
                    .filter(|j| {
                        j.members.iter().any(|m| m.corridor == c.id)
                            && j.members.iter().any(|m| m.corridor == oid)
                    })
                    .map(|j| metres(j.point, point, c.cos_lat))
                    .fold(f64::INFINITY, f64::min);
                if d.is_finite() {
                    dist.push(d);
                } else {
                    no_junction += 1;
                }
                // Would this rejection have been an *ordered* crossing? The
                // level hints at the two arcs say so, and that is the half
                // that costs a structure rather than a demand.
                let arc_c = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                let arc_o = other.arc[oi] + u * (other.arc[oi + 1] - other.arc[oi]);
                if level_at(&scene, c.id, arc_c) != level_at(&scene, oid, arc_o) {
                    ordered_by_hint += 1;
                }
            }
        }
    }

    println!("\nplan intersections between corridor pairs: {total}");
    println!("rejected because the two corridors share a connector: {shared}");
    println!("  of those, ordered by differing level hints: {ordered_by_hint}");
    println!("  of those, sharing it at no common junction:   {no_junction}");
    println!("\nthe local question — do the two alignments touch *here*?");
    println!("  share a vertex at the intersection (a meeting): {touch_here}");
    println!("  pass between each other's vertices (a crossing): {apart}");
    println!("    of those, ordered by differing level hints:    {apart_ordered}");
    listing.sort();
    println!("\nevery crossing the identity test rejects that is not a meeting\n");
    for l in &listing {
        println!("{l}");
    }
    let mut groups = 0usize;
    let mut collapsed = 0usize;
    for (key, pts) in &sites {
        // Distinct places, at 20 m: adjacent edges sharing a vertex report the
        // same crossing twice, and that is what the dedup is for.
        let mut distinct: Vec<Coord> = Vec::new();
        for p in pts {
            if !distinct.iter().any(|q| metres(*q, *p, scene.corridors[key.0 as usize].cos_lat) < 20.0) {
                distinct.push(*p);
            }
        }
        groups += 1;
        collapsed += distinct.len() - 1;
    }
    println!(
        "\nordered crossings by (upper, lower, level pair): {groups} groups, \
         {collapsed} distinct places the dedup key discards"
    );
    dist.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if !dist.is_empty() {
        let q = |f: f64| dist[((dist.len() as f64 - 1.0) * f) as usize];
        println!(
            "\ndistance from the intersection to the nearest junction the pair shares ({} of them)",
            dist.len()
        );
        println!(
            "  p10 {:.0} m  p25 {:.0} m  p50 {:.0} m  p75 {:.0} m  p90 {:.0} m  max {:.0} m",
            q(0.1), q(0.25), q(0.5), q(0.75), q(0.9), q(1.0)
        );
        println!("\n  within   share of the rejected pairs");
        for r in [1.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0] {
            let n = dist.iter().filter(|&&d| d <= r).count();
            println!("  {r:>6.0} m  {:>5.1}%", 100.0 * n as f64 / shared as f64);
        }
    }
    let _ = &solved;
}

fn level_at(scene: &SceneGraph, corridor: u32, arc: f64) -> i64 {
    scene.corridors[corridor as usize]
        .spans
        .iter()
        .find(|s| arc >= s.arc0 && arc <= s.arc1)
        .map_or(0, |s| if s.kind == SpanKind::Grade { 0 } else { s.level })
}

fn metres(a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let dx = (a.x - b.x) * cos_lat * DEG_M;
    let dy = (a.y - b.y) * DEG_M;
    (dx * dx + dy * dy).sqrt()
}

/// Proper intersection of two segments in the local metric frame, as the
/// fractions along each — `solve::crossings`' own predicate.
fn seg_intersect(a: Coord, b: Coord, c: Coord, d: Coord, cos_lat: f64) -> Option<(f64, f64)> {
    let (ax, ay) = (a.x * cos_lat, a.y);
    let (bx, by) = (b.x * cos_lat, b.y);
    let (cx, cy) = (c.x * cos_lat, c.y);
    let (dx, dy) = (d.x * cos_lat, d.y);
    let (rx, ry) = (bx - ax, by - ay);
    let (sx, sy) = (dx - cx, dy - cy);
    let denom = rx * sy - ry * sx;
    if denom.abs() < 1e-18 {
        return None;
    }
    let t = ((cx - ax) * sy - (cy - ay) * sx) / denom;
    let u = ((cx - ax) * ry - (cy - ay) * rx) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some((t, u))
}
