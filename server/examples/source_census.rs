//! Who moved the ground under a structure, and did it change the decision?
//!
//! `ground.single_source` (I1) reports that 8 % of structure nodes were decided
//! against a surface that is not the one finally drawn, and it cannot say more:
//! the metric is one subtraction. This walks the same population — every
//! profile node inside a structure span, scored as published ground minus the
//! reference the solve read, with the reference passed to the stack as its own
//! base — and decomposes each divergent node:
//!
//! - **who** — which layer of the accumulating stack (§4.3) moved it, which
//!   corridor's earthwork edge inside that layer, and whether that corridor is
//!   the node's own (its portal carve), a peer in the same stratum, or a
//!   *junior* stratum re-cutting the ground under a senior's structure.
//! - **what kind** of edge did it: a portal carve, a bench, a cutting face or
//!   an embankment fill.
//! - **where** — distance to the nearest end of the structure span, since a
//!   mouth is a place the ground is *meant* to be cut away.
//! - **whether it mattered** — the same decision restated against both grounds.
//!   A bore's ceiling was placed to leave [`TUNNEL_COVER_M`] over its roof; a
//!   deck's soffit was fitted to stand clear of the ground. A node is
//!   *consequential* when the decision holds against the reference the solve
//!   read and fails against the ground the tiler drew. Divergence that never
//!   crosses a decision boundary costs nothing and needs no second pass.
//!
//! Written to size the reconciliation fixpoint before building it.
//!
//! Usage: cargo run --release --example source_census -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor]

use arpentry_server::dem::Dem;
use arpentry_server::ground;
use arpentry_server::priors::{Stratum, DECK_THICKNESS_M, TUNNEL_COVER_M, TUNNEL_HEIGHT_M};
use arpentry_server::project::Bounds;
use arpentry_server::scene::{Span, SpanKind};
use arpentry_server::{assemble, solve};

/// Same line the metric draws: past this the two grounds are different surfaces
/// at the scale the structural decisions care about.
const SINGLE_SOURCE_M: f64 = 1.0;

/// What kind of earthwork edge produced a height.
#[derive(Clone, Copy, PartialEq, Debug)]
enum EdgeKind {
    Carve,
    Bench,
    Cut,
    Fill,
}

impl EdgeKind {
    fn name(self) -> &'static str {
        match self {
            EdgeKind::Carve => "portal carve",
            EdgeKind::Bench => "bench",
            EdgeKind::Cut => "cutting face",
            EdgeKind::Fill => "fill face",
        }
    }
}

/// The edge of one layer that best explains the height it resolved to, as
/// (chain, kind, value). Mirrors [`Earthworks::height`]'s own resolution —
/// bench wins outright, else the cut digs and the fill raises, and carves bound
/// the result from above — but keeps the *chain* of the winning edge, which is
/// the corridor that owns the earthwork.
fn explain(
    ew: &ground::modifiers::Earthworks,
    lon: f64,
    lat: f64,
    base: f64,
    resolved: f64,
) -> Option<(u32, EdgeKind)> {
    let cos_lat = lat.to_radians().cos();
    let mut best: Option<(f64, u32, EdgeKind)> = None;
    for e in ew.edges() {
        let px = (lon - e.a.x) * 111_320.0 * cos_lat;
        let py = (lat - e.a.y) * 110_540.0;
        let ex = (e.b.x - e.a.x) * 111_320.0 * cos_lat;
        let ey = (e.b.y - e.a.y) * 110_540.0;
        let len2 = ex * ex + ey * ey;
        let raw_t = if len2 > 0.0 { (px * ex + py * ey) / len2 } else { 0.0 };
        if e.headwall && raw_t < 0.0 {
            continue; // bounded by its own face: no reach behind `a`
        }
        let t = raw_t.clamp(0.0, 1.0);
        let (qx, qy) = (px - t * ex, py - t * ey);
        let d = (qx * qx + qy * qy).sqrt();
        let side = if ex * py - ey * px >= 0.0 { 0 } else { 1 };
        if d >= e.half_width_m[side] + e.batter_m[side] {
            continue;
        }
        let target = e.target_a + (e.target_b - e.target_a) * t;
        let rise = (d - e.half_width_m[side]).max(0.0) / e.batter_run[side];
        let (kind, value) = if e.carve {
            (EdgeKind::Carve, target + rise)
        } else if d <= e.half_width_m[side] {
            (EdgeKind::Bench, target)
        } else if target > base {
            (EdgeKind::Fill, target - rise)
        } else {
            (EdgeKind::Cut, target + rise)
        };
        // Whichever candidate lands closest to what the layer actually
        // resolved to is the one that produced it.
        let err = (value - resolved).abs();
        if best.is_none_or(|(b, _, _)| err < b) {
            best = Some((err, e.chain, kind));
        }
    }
    best.map(|(_, chain, kind)| (chain, kind))
}

fn span_at(spans: &[Span], arc: f64) -> Option<&Span> {
    spans.iter().find(|s| arc >= s.arc0 && arc <= s.arc1 && s.kind != SpanKind::Grade)
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let dump: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    let mut scene = assemble::run(std::path::Path::new(&a[0]), None, &bbox).expect("assemble");
    let solved = solve::run(&mut scene, Some(&terrain), 16, 0).expect("solve");
    let stack = ground::derive(&scene, &solved, &arpentry_server::assemble::facades::Facades::empty(), Some(&terrain), 0);
    let mut dem = Dem::open(&terrain).expect("dem");
    let mut scratch: Vec<u32> = Vec::new();
    let strata: Vec<Stratum> = stack.layers().iter().map(|l| l.stratum).collect();

    let mut nodes = 0u64;
    let mut over = 0u64;
    // [tunnel, bridge]
    let mut by_span = [0u64; 2];
    let mut down = 0u64;
    // Whose imprint: own corridor, peer in the same stratum, a junior stratum,
    // a senior stratum, unattributed.
    let mut whose = [0u64; 5];
    // Edge kinds, over all divergent nodes.
    let mut by_kind = [0u64; 4];
    // Distance to the nearest span end: 0-10, 10-25, 25-50, 50-100, 100+.
    let mut from_mouth = [0u64; 5];
    // The decision restated: held against both grounds / held against the
    // reference and fails against the drawn ground / already failed.
    let mut decision = [0u64; 3];
    // Consequential nodes cross-tabulated by whose imprint × distance to the
    // nearest span end, since a mouth is a place the ground is meant to go.
    let mut cross = [[0u64; 5]; 5];
    // (|v|, lon, lat, corridor, kind, span, read, published, culprit, consequence)
    let mut worst: Vec<(f64, f64, f64, u32, String, &'static str, f64, f64, String, String)> =
        Vec::new();
    // Consequential nodes only, same shape.
    let mut bad: Vec<(f64, f64, f64, u32, String, &'static str, f64, f64, String, String)> =
        Vec::new();
    let mut per_corridor: Vec<(u64, f64, u32, String)> = Vec::new();

    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        let (arc, at_grade, terrain_m) = (p.arc(), p.at_grade(), p.terrain_m());
        let mut corridor_over = 0u64;
        let mut corridor_worst = 0.0f64;
        for k in 0..arc.len() {
            if at_grade[k] {
                continue;
            }
            let pt = p.point_at_arc(arc[k]);
            if !bbox.contains(pt.x, pt.y) {
                continue;
            }
            let read = terrain_m[k];
            let published = stack.height(pt.x, pt.y, read, 0.0, &mut scratch);
            let v = (published - read).abs();
            nodes += 1;
            if dump == Some(c.id) {
                let s = span_at(&c.spans, arc[k]);
                println!(
                    "  arc {:7.1}  {:.6},{:.6}  read {read:8.2} published {published:8.2}  \
                     {:+7.2}  road {:8.2}  {} d_end {:.0}",
                    arc[k],
                    pt.x,
                    pt.y,
                    published - read,
                    p.road_at_arc(arc[k]),
                    s.map(|s| format!("{:?}", s.kind)).unwrap_or_else(|| "-".into()),
                    s.map(|s| (arc[k] - s.arc0).min(s.arc1 - arc[k])).unwrap_or(f64::NAN),
                );
            }
            if v <= SINGLE_SOURCE_M {
                continue;
            }
            over += 1;
            corridor_over += 1;
            corridor_worst = corridor_worst.max(v);
            if published < read {
                down += 1;
            }

            let span = span_at(&c.spans, arc[k]);
            let span_name = match span.map(|s| s.kind) {
                Some(SpanKind::Tunnel) => {
                    by_span[0] += 1;
                    "tunnel"
                }
                Some(SpanKind::Bridge) => {
                    by_span[1] += 1;
                    "bridge"
                }
                _ => "none",
            };
            let d_mouth = span
                .map(|s| (arc[k] - s.arc0).min(s.arc1 - arc[k]).max(0.0))
                .unwrap_or(f64::INFINITY);
            let mouth_bucket = match d_mouth {
                d if d < 10.0 => 0,
                d if d < 25.0 => 1,
                d if d < 50.0 => 2,
                d if d < 100.0 => 3,
                _ => 4,
            };
            from_mouth[mouth_bucket] += 1;

            // Which layer moved it most, and which of its edges did it.
            let mut prev = read;
            let mut moved: Option<(f64, usize, f64)> = None;
            for n in 0..strata.len() {
                let h = stack.height_through(n + 1, pt.x, pt.y, read, 0.0, &mut scratch);
                let delta = h - prev;
                if moved.is_none_or(|(d, _, _)| delta.abs() > d.abs()) {
                    moved = Some((delta, n, prev));
                }
                prev = h;
            }
            let mut whose_bucket = 4;
            let culprit = match moved {
                Some((delta, n, base)) => {
                    let layer = &stack.layers()[n];
                    let resolved = base + delta;
                    match explain(layer.earthworks(), pt.x, pt.y, base, resolved) {
                        Some((chain, kind)) => {
                            by_kind[kind as usize] += 1;
                            let owner = scene.corridors.iter().find(|o| o.id == chain);
                            let owner_stratum = owner.map(|o| o.kind.stratum());
                            let mine = c.kind.stratum();
                            let bucket = if chain == c.id {
                                0
                            } else {
                                match owner_stratum {
                                    Some(s) if s == mine => 1,
                                    Some(s) if s > mine => 2,
                                    Some(_) => 3,
                                    None => 4,
                                }
                            };
                            whose[bucket] += 1;
                            whose_bucket = bucket;
                            format!(
                                "{:?} layer, {} of {} {} ({:+.2} m)",
                                layer.stratum,
                                kind.name(),
                                owner.map(|o| format!("{:?}", o.kind)).unwrap_or_default(),
                                chain,
                                delta
                            )
                        }
                        None => {
                            whose[4] += 1;
                            format!("{:?} layer, unattributed ({delta:+.2} m)", layer.stratum)
                        }
                    }
                }
                None => {
                    whose[4] += 1;
                    "no layer moved it".into()
                }
            };

            // The same structural decision, restated against both grounds.
            let (margin_read, margin_pub, what) = match span.map(|s| s.kind) {
                Some(SpanKind::Tunnel) => {
                    let roof = p.road_at_arc(arc[k]) + TUNNEL_HEIGHT_M + TUNNEL_COVER_M;
                    (read - roof, published - roof, "cover over the roof")
                }
                Some(SpanKind::Bridge) => {
                    let soffit = p.deck_at_arc(arc[k]) - DECK_THICKNESS_M;
                    (soffit - read, soffit - published, "clearance under the soffit")
                }
                _ => (0.0, 0.0, "no span"),
            };
            let bucket = match (margin_read >= 0.0, margin_pub >= 0.0) {
                (_, true) => 0,
                (true, false) => 1,
                (false, false) => 2,
            };
            decision[bucket] += 1;
            let consequence = format!(
                "{d_mouth:.0} m from the span end; {what} {margin_read:+.2} m against the \
                 reference, {margin_pub:+.2} m against the drawn ground"
            );
            if bucket == 1 {
                cross[whose_bucket][mouth_bucket] += 1;
            }
            let row = (
                v,
                pt.x,
                pt.y,
                c.id,
                format!("{:?}", c.kind),
                span_name,
                read,
                published,
                culprit,
                consequence,
            );
            if bucket == 1 {
                bad.push(row.clone());
            }
            worst.push(row);
        }
        if corridor_over > 0 {
            per_corridor.push((corridor_over, corridor_worst, c.id, format!("{:?}", c.kind)));
        }
    }
    let _ = dem.elevation(bbox.west, bbox.south, 16);

    let pct = |n: u64| if over > 0 { 100.0 * n as f64 / over as f64 } else { 0.0 };
    println!("\n== structure nodes ==");
    println!("  scored              {nodes}");
    println!("  over {SINGLE_SOURCE_M:.1} m         {over}  ({:.1} %)", 100.0 * over as f64 / nodes.max(1) as f64);
    println!("  ground cut down     {down}  ({:.1} %)", pct(down));
    println!("  in a tunnel span    {}  ({:.1} %)", by_span[0], pct(by_span[0]));
    println!("  in a bridge span    {}  ({:.1} %)", by_span[1], pct(by_span[1]));

    println!("\n== whose imprint moved it ==");
    for (n, label) in [
        "own corridor (its own portal carve)",
        "a peer in the same stratum",
        "a JUNIOR stratum, under a senior structure",
        "a senior stratum",
        "unattributed",
    ]
    .iter()
    .enumerate()
    {
        println!("  {label:<40} {:>5}  ({:.1} %)", whose[n], pct(whose[n]));
    }

    println!("\n== what kind of edge ==");
    for (n, label) in ["portal carve", "bench", "cutting face", "fill face"].iter().enumerate() {
        println!("  {label:<40} {:>5}  ({:.1} %)", by_kind[n], pct(by_kind[n]));
    }

    println!("\n== distance to the nearest span end ==");
    for (n, label) in ["0-10 m", "10-25 m", "25-50 m", "50-100 m", "100+ m"].iter().enumerate() {
        println!("  {label:<40} {:>5}  ({:.1} %)", from_mouth[n], pct(from_mouth[n]));
    }

    println!("\n== the decision, restated against both grounds ==");
    println!("  holds against the drawn ground            {:>5}  ({:.1} %)", decision[0], pct(decision[0]));
    println!("  CONSEQUENTIAL: held, now fails           {:>5}  ({:.1} %)", decision[1], pct(decision[1]));
    println!("  already failed against the reference     {:>5}  ({:.1} %)", decision[2], pct(decision[2]));

    println!("\n== consequential nodes: whose imprint × distance to the span end ==");
    println!("  {:<40} {:>7} {:>8} {:>8} {:>9} {:>7}", "", "0-10 m", "10-25 m", "25-50 m", "50-100 m", "100+ m");
    for (n, label) in [
        "own corridor (its own portal carve)",
        "a peer in the same stratum",
        "a JUNIOR stratum",
        "a senior stratum",
        "unattributed",
    ]
    .iter()
    .enumerate()
    {
        println!(
            "  {label:<40} {:>7} {:>8} {:>8} {:>9} {:>7}",
            cross[n][0], cross[n][1], cross[n][2], cross[n][3], cross[n][4]
        );
    }

    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("\n== worst divergence ==");
    for w in worst.iter().take(12) {
        println!(
            "  {:6.2} m  {:.6},{:.6}  {} {} ({})  read {:.2} -> {:.2}\n            {}\n            {}",
            w.0, w.1, w.2, w.4, w.3, w.5, w.6, w.7, w.8, w.9
        );
    }

    bad.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!("\n== consequential sites ({}) ==", bad.len());
    for w in bad.iter().take(12) {
        println!(
            "  {:6.2} m  {:.6},{:.6}  {} {} ({})  read {:.2} -> {:.2}\n            {}\n            {}",
            w.0, w.1, w.2, w.4, w.3, w.5, w.6, w.7, w.8, w.9
        );
    }

    per_corridor.sort_by(|a, b| b.0.cmp(&a.0));
    println!("\n== worst corridors ==");
    for (n, v, id, kind) in per_corridor.iter().take(12) {
        println!("  {n:>4} nodes  worst {v:6.2} m  {kind} {id}");
    }
}
