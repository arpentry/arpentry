//! Which annotated structures does the short-span demotion delete, and by how
//! much did each miss the terrain test?
//!
//! `solve::reconcile_short_spans` keeps a sub-`MIN_STRUCTURE_M` span only where
//! the ground departs its end-to-end chord by more than `SHORT_STRUCTURE_DIP_M`
//! at a quarter, half or three-quarter point, or where it passes over or under
//! another corridor (`solve::crossings::spans_over_a_mapped_line`). This prints
//! the spans assemble produced, the departure each short one measured, and the
//! verdict — so a structure that vanished between the data and the drawing is
//! attributable.
//!
//! For every span the current rules demote, it also names the plan witnesses:
//! draped alignments and flowing-water lines that cross the span. Assemble
//! admits these into `SceneGraph::witnesses` and the exemption consults them,
//! so a *demoted* span reported here with a witness is a disagreement between
//! this probe's independent scan and the scene's own witness set — worth
//! attributing, not shrugging at.
//!
//! Usage: cargo run --release --example shortspan_probe -- <segment.parquet> <w,s,e,n> <terrain.pmtiles> [corridor_id]
//!
//! A `water.parquet` beside the segment input is picked up automatically for
//! the flowing-water witnesses.

use arpentry_server::assemble;
use arpentry_server::dem::Dem;
use arpentry_server::geoparquet::GeoParquet;
use arpentry_server::priors::{Kind, Modality, Stratum, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use arpentry_server::project::Bounds;
use arpentry_server::scene::{SpanKind, DEG_M};
use arpentry_server::solve;
use geo_types::{Coord, Geometry};

/// A crossing this close (metres) to a witness line's own terminus is a
/// meeting — a path ending at the railway — not a passage beneath it.
const MEET_EPS_M: f64 = 2.0;

struct Witness {
    line: Vec<Coord>,
    /// Cumulative arc (metres) at each vertex, for the terminus test.
    arc: Vec<f64>,
    water: bool,
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let bb: Vec<f64> = a[1].split(',').map(|s| s.parse().unwrap()).collect();
    let bbox = Bounds { west: bb[0], south: bb[1], east: bb[2], north: bb[3] };
    let terrain = std::path::PathBuf::from(&a[2]);
    let only: Option<u32> = a.get(3).and_then(|s| s.parse().ok());

    // The water input rides along when it sits beside the segment input, the
    // same way the tiler passes it — so the scene's witness lines (and with
    // them the crossing exemption) match what a real run computes.
    let segments = std::path::PathBuf::from(&a[0]);
    let water = segments.with_file_name("water.parquet");
    let scene = assemble::run(&segments, water.exists().then_some(water.as_path()), &bbox)
        .expect("assemble");
    let over = solve::crossings::spans_over_a_mapped_line(&scene);
    let mut dem = Dem::open(&terrain).expect("dem");
    let z = 16u8;

    let witnesses = read_witnesses(std::path::Path::new(&a[0]), &bbox);
    eprintln!(
        "witnesses: {} draped lines, {} flowing-water lines",
        witnesses.iter().filter(|w| !w.water).count(),
        witnesses.iter().filter(|w| w.water).count()
    );

    let (mut kept, mut lost, mut lost_m) = (0usize, 0usize, 0.0f64);
    // Demoted spans by (modality, witness category) -> (count, metres).
    let mut tally: std::collections::BTreeMap<(String, &str), (usize, f64)> = Default::default();
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
                    if !short { "kept (long)" } else if crosses { "KEPT: crosses a mapped line" }
                    else if survives { "kept (dip)" } else { "DEMOTED to grade" },
                );
                continue;
            }
            if c.kind.modality() == Modality::Rail && short {
                if survives { kept += 1 } else { lost += 1; lost_m += len }
            }
            if short && !survives {
                let (draped, water) = witnessed(c, s, &witnesses);
                let cat = match (draped, water) {
                    (true, _) => "crosses a draped line",
                    (false, true) => "crosses flowing water",
                    (false, false) => "no witness (dip miss)",
                };
                let mid = point_at_arc(&c.nodes, &c.arc, 0.5 * (s.arc0 + s.arc1));
                println!(
                    "corridor #{:<6} {:<28} {:?} L{} {:5.1} m  depart {:+.2} m  {}  ({:.5},{:.5})",
                    c.id,
                    format!("{:?}", c.kind),
                    s.kind,
                    s.level,
                    len,
                    depart,
                    cat,
                    mid.x,
                    mid.y,
                );
                let e = tally
                    .entry((format!("{:?}", c.kind.modality()), cat))
                    .or_insert((0, 0.0));
                e.0 += 1;
                e.1 += len;
            }
        }
    }
    if only.is_none() {
        println!();
        for ((modality, cat), (n, m)) in &tally {
            println!("{modality:<8} {cat:<24} {n:3} spans  {m:6.0} m");
        }
        println!("\nrail short spans: {kept} kept, {lost} demoted ({lost_m:.0} m of annotated structure)");
    }
}

/// Whether any witness line crosses the span's stretch of the corridor in
/// plan, strictly inside the witness (a terminus on the line is a meeting).
/// Returns (crosses a draped line, crosses a flowing-water line).
fn witnessed(
    c: &arpentry_server::scene::Corridor,
    s: &arpentry_server::scene::Span,
    witnesses: &[Witness],
) -> (bool, bool) {
    let (mut draped, mut water) = (false, false);
    for i in 0..c.nodes.len().saturating_sub(1) {
        if c.arc[i + 1] < s.arc0 || c.arc[i] > s.arc1 {
            continue;
        }
        let (a, b) = (c.nodes[i], c.nodes[i + 1]);
        let eb = (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y));
        for w in witnesses {
            if (w.water && water) || (!w.water && draped) {
                continue;
            }
            for j in 0..w.line.len() - 1 {
                let (wa, wb) = (w.line[j], w.line[j + 1]);
                if wa.x.max(wb.x) < eb.0
                    || wa.x.min(wb.x) > eb.2
                    || wa.y.max(wb.y) < eb.1
                    || wa.y.min(wb.y) > eb.3
                {
                    continue;
                }
                let Some((t, u)) = seg_intersect(a, b, wa, wb) else { continue };
                // Inside the span along the corridor…
                let arc_here = c.arc[i] + t * (c.arc[i + 1] - c.arc[i]);
                if arc_here < s.arc0 || arc_here > s.arc1 {
                    continue;
                }
                // …and clear of the witness's own termini.
                let w_arc = w.arc[j] + u * (w.arc[j + 1] - w.arc[j]);
                let w_len = *w.arc.last().unwrap();
                if w_arc < MEET_EPS_M || w_arc > w_len - MEET_EPS_M {
                    continue;
                }
                if w.water {
                    water = true;
                } else {
                    draped = true;
                }
                break;
            }
        }
        if draped && water {
            break;
        }
    }
    (draped, water)
}

/// Segment intersection in lon/lat; parameters on both segments. Existence is
/// affine-invariant, so no metric scaling is needed to ask *whether*.
fn seg_intersect(a: Coord, b: Coord, c: Coord, d: Coord) -> Option<(f64, f64)> {
    let (r, s) = (Coord { x: b.x - a.x, y: b.y - a.y }, Coord { x: d.x - c.x, y: d.y - c.y });
    let denom = r.x * s.y - r.y * s.x;
    if denom.abs() < f64::EPSILON {
        return None;
    }
    let ac = Coord { x: c.x - a.x, y: c.y - a.y };
    let t = (ac.x * s.y - ac.y * s.x) / denom;
    let u = (ac.x * r.y - ac.y * r.x) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some((t, u))
}

fn point_at_arc(nodes: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&x| x < s).clamp(1, arc.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
        y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
    }
}

/// The crossing witnesses assemble drops: draped alignments from the segment
/// input, and flowing-water lines from a `water.parquet` beside it.
fn read_witnesses(segments: &std::path::Path, bbox: &Bounds) -> Vec<Witness> {
    let bb = (bbox.west, bbox.south, bbox.east, bbox.north);
    let mut out = Vec::new();

    let gp = GeoParquet::open(segments).expect("segment input");
    let groups = gp.row_groups_intersecting(bb);
    for feature in gp.features(groups, &["subtype", "class", "subclass"]).expect("features") {
        let Ok(f) = feature else { continue };
        let kind = Kind::parse(
            arpentry_server::value::str_of(&f.properties, "subtype"),
            arpentry_server::value::str_of(&f.properties, "class"),
            arpentry_server::value::str_of(&f.properties, "subclass"),
        );
        if matches!(kind.stratum(), Stratum::H | Stratum::R | Stratum::S) {
            continue; // in the scene already; `spans_over_a_mapped_line` sees it
        }
        if let Geometry::LineString(line) = &f.geometry {
            push_line(&line.0, false, &mut out);
        }
    }

    let water = segments.with_file_name("water.parquet");
    if let Ok(gp) = GeoParquet::open(&water) {
        let groups = gp.row_groups_intersecting(bb);
        if let Ok(features) = gp.features(groups, &["subtype"]) {
            for f in features.flatten() {
                let flowing = matches!(
                    arpentry_server::value::str_of(&f.properties, "subtype"),
                    Some("river" | "stream" | "canal")
                );
                if !flowing {
                    continue;
                }
                match &f.geometry {
                    Geometry::LineString(line) => push_line(&line.0, true, &mut out),
                    Geometry::Polygon(p) => push_line(&p.exterior().0, true, &mut out),
                    Geometry::MultiPolygon(mp) => {
                        mp.0.iter().for_each(|p| push_line(&p.exterior().0, true, &mut out))
                    }
                    _ => {}
                }
            }
        }
    } else {
        eprintln!("no water.parquet beside the segment input; water witnesses skipped");
    }
    out
}

fn push_line(line: &[Coord], water: bool, out: &mut Vec<Witness>) {
    if line.len() < 2 {
        return;
    }
    let cos_lat = line[0].y.to_radians().cos();
    let mut arc = Vec::with_capacity(line.len());
    let mut acc = 0.0;
    arc.push(0.0);
    for i in 1..line.len() {
        let dx = (line[i].x - line[i - 1].x) * cos_lat;
        let dy = line[i].y - line[i - 1].y;
        acc += (dx * dx + dy * dy).sqrt() * DEG_M;
        arc.push(acc);
    }
    out.push(Witness { line: line.to_vec(), arc, water });
}
