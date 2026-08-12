//! Where does the paint actually land on the asphalt?
//!
//! `docs/ROADS.md` invariant 3 says no marking is placed by eye: every painted
//! line is a function of the cross-section, so it must lie *on* the carriageway
//! it belongs to and at the offset the cross-section puts it at. Nothing in the
//! scorecard measures that. This walks every `marking` stroke in the emitted
//! archive and reports, per vertex:
//!
//!   * whether it falls on drawn asphalt at all — the at-grade carriageway
//!     (the unioned `road_surface` interior *welded to its `road_casing` rim*,
//!     since the interior is triangulated to an inset of the true silhouette),
//!     or a deck / bore surface where the road is a structure;
//!   * how deep inside it lies, as the plan distance to the welded silhouette;
//!   * how far it sits from the nearest drawn road centerline of the same
//!     level, which is what says whether a *centre* line is centred;
//!   * how far its baked height stands off the surface under it.
//!
//! The population split is by painted width, the only thing the archive carries
//! that says which line a stroke is: 0.12 m is a centre or lane line, 0.15 m an
//! edge line (`priors::CENTRE_LINE_WIDTH_M` / `EDGE_LINE_WIDTH_M`).
//!
//! Usage: cargo run --release --example marking_probe -- <archive.arpa> [zoom] [max_tiles]

use std::collections::{BTreeMap, HashMap};

use arpentry_server::verify::mesh::{Scale, SurfaceMesh};
use arpentry_server::verify::scene::{ArchiveScan, RoadLine};

/// One measured marking vertex.
struct Sample {
    /// Plan distance to the asphalt silhouette in metres: positive inside the
    /// paved region, negative outside it.
    depth_m: f64,
    /// How far the asphalt reaches to the left and to the right of the painted
    /// line, measured perpendicular to it. For a centre line the two should
    /// match — that is what "centred" means — and their difference is the
    /// misalignment in metres. `None` when the vertex is not on asphalt.
    reach_m: Option<(f64, f64)>,
    /// Plan distance to the nearest drawn road centerline, in metres, with that
    /// road's stated carriageway width.
    to_axis_m: f64,
    axis_width_m: f64,
    /// Baked marking height less the surface height beneath, in metres. `None`
    /// when the vertex is off the asphalt (nothing to stand off).
    standoff_m: Option<f64>,
    /// What the archive *does* draw at this plan position, as
    /// `class:level@height-range` — the question every off-asphalt vertex
    /// raises: if not the carriageway, then what?
    over: String,
    lon: f64,
    lat: f64,
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.is_empty() {
        eprintln!("usage: marking_probe <archive.arpa> [zoom] [max_tiles]");
        std::process::exit(2);
    }
    let bytes = std::fs::read(&a[0]).expect("read archive");
    let scan = ArchiveScan::open(&bytes).expect("open archive");
    let z: u8 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or_else(|| scan.max_zoom());
    let max_tiles: usize = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(usize::MAX);

    let mut by_kind: BTreeMap<&'static str, Vec<Sample>> = BTreeMap::new();
    let (mut tiles, mut with_marks, mut no_asphalt_tile) = (0usize, 0usize, 0usize);
    let mut on_structure = 0usize;
    let mut marking_features = 0usize;
    let mut widths: BTreeMap<String, usize> = BTreeMap::new();

    for (z, x, y, id) in scan.tiles_at(z).into_iter().take(max_tiles) {
        let Some(tile) = scan.decode(z, x, y, id) else { continue };
        tiles += 1;
        let marks: Vec<&RoadLine> = tile.lines.iter().filter(|l| l.class == "marking").collect();
        if marks.is_empty() {
            continue;
        }
        with_marks += 1;
        marking_features += marks.len();

        // Every surface a marking may legitimately lie on. The at-grade
        // carriageway is two features — an inset interior and the rim that
        // covers the strip out to the true silhouette — so both are welded into
        // one population before anything is measured against it.
        // At-grade only: a deck top and a bore's tube answer `height_at` with
        // the *highest* face — a bridge parapet, a tunnel roof — so measuring a
        // cross-section against them measures the solid, not the carriageway.
        // Structure-borne paint is counted and set aside instead.
        let paved: Vec<&SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|m| m.is_pavement() || m.is_casing())
            .map(|m| &m.mesh)
            .collect();
        let structures: Vec<&SurfaceMesh> =
            tile.roads.iter().filter(|m| m.is_deck() || m.is_bore()).map(|m| &m.mesh).collect();
        if paved.is_empty() {
            no_asphalt_tile += 1;
            continue;
        }
        let rims = silhouette(&paved);

        // The drawn centerlines a marking could belong to: drivable strokes,
        // excluding the markings themselves.
        let axes: Vec<&RoadLine> =
            tile.lines.iter().filter(|l| l.class != "marking" && l.width_m >= 2.0).collect();

        for line in &marks {
            let kind = kind_of(line.width_m);
            *widths.entry(format!("{:.2}", line.width_m)).or_default() += 1;
            for part in &line.parts {
                for (i, &(px, py, h)) in part.iter().enumerate() {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    // Paint riding a structure: its cross-section is the deck's
                    // or the bore's, not the union's, so it is tallied and left
                    // out of the at-grade population.
                    if structures.iter().any(|m| {
                        m.height_range_at(px, py).is_some_and(|(lo, hi)| h >= lo - 1.0 && h <= hi + 1.0)
                    }) {
                        on_structure += 1;
                        continue;
                    }
                    let under = paved.iter().filter_map(|m| m.height_at(px, py)).fold(
                        None::<f64>,
                        |best, v| {
                            Some(best.map_or(v, |b: f64| {
                                if (v - h).abs() < (b - h).abs() { v } else { b }
                            }))
                        },
                    );
                    let rim = nearest_seg(&rims, px, py, &tile.scale);
                    let (to_axis, axis_w) = nearest_axis(&axes, px, py, h, &tile.scale);
                    let (lon, lat) = tile.lonlat(px, py);
                    // The painted line's own direction, from the segment it
                    // lies on — the cross-section is measured across *it*, not
                    // across whatever road happens to be nearest.
                    let j = if i + 1 < part.len() { i + 1 } else { i };
                    let k = if i + 1 < part.len() { i } else { i - 1 };
                    let reach = under.is_some().then(|| {
                        cross_reach(&paved, (px, py), (part[k], part[j]), &tile.scale)
                    });
                    let over = if under.is_some() {
                        String::new()
                    } else {
                        let mut seen: Vec<String> = tile
                            .roads
                            .iter()
                            .filter_map(|m| {
                                m.mesh.height_range_at(px, py).map(|(lo, hi)| {
                                    format!("{}:{}@{lo:.1}-{hi:.1}", m.class, m.level)
                                })
                            })
                            .collect();
                        if seen.is_empty() {
                            // Nothing at this position: how far is the nearest
                            // structure, and what is it? A deck box and a bore
                            // tube are closed solids, so they have no mesh
                            // silhouette to measure to — the distance has to be
                            // taken to their triangles, or rather (cheaply, and
                            // enough to tell a metre from fifty) to their
                            // vertices.
                            let mut best = (f64::INFINITY, String::from("none in tile"));
                            for m in tile.roads.iter().filter(|m| m.is_deck() || m.is_bore()) {
                                for v in 0..m.mesh.vertex_count() {
                                    let (vx, vy, _) = m.mesh.vertex(v);
                                    let d = tile.scale.dist(px, py, vx, vy);
                                    if d < best.0 {
                                        best = (d, format!("{}:{}", m.class, m.level));
                                    }
                                }
                            }
                            seen.push(format!(
                                "nothing; nearest structure {} at {:.1} m; terrain {}",
                                best.1,
                                best.0,
                                tile.terrain
                                    .as_ref()
                                    .and_then(|t| t.height_at(px, py))
                                    .map_or("absent".to_string(), |g| format!("{g:.1}"))
                            ));
                        }
                        format!("paint {h:.1} m over [{}]", seen.join(", "))
                    };
                    by_kind.entry(kind).or_default().push(Sample {
                        depth_m: if under.is_some() { rim } else { -rim },
                        over,
                        reach_m: reach,
                        to_axis_m: to_axis,
                        axis_width_m: axis_w,
                        standoff_m: under.map(|s| h - s),
                        lon,
                        lat,
                    });
                }
            }
        }
    }

    println!(
        "z{z}: {tiles} tiles scanned, {with_marks} carried markings ({marking_features} marking \
         features), {no_asphalt_tile} had markings but no paved surface at all"
    );
    println!(
        "{on_structure} marking vertices rode a deck or a bore and were set aside: their \
         cross-section is the structure's, not the union's"
    );
    println!("painted widths seen: {widths:?}\n");
    for (kind, s) in &by_kind {
        report(kind, s);
    }
}

/// Which painted line a stroke is, by its stated width.
fn kind_of(width_m: f64) -> &'static str {
    if (width_m - 0.15).abs() < 0.01 {
        "edge line (0.15 m)"
    } else if (width_m - 0.12).abs() < 0.01 {
        "centre/lane line (0.12 m)"
    } else {
        "other"
    }
}

/// The silhouette of a set of meshes: every edge no *other* triangle in the set
/// shares, keyed on the integer plan lattice so two meshes meeting along a
/// shared boundary — the carriageway interior and its rim — weld instead of
/// each reporting the join as an outer edge.
fn silhouette(meshes: &[&SurfaceMesh]) -> Vec<((f64, f64), (f64, f64))> {
    type Key = ((i64, i64), (i64, i64));
    let key = |v: (f64, f64, f64)| ((v.0 * 32768.0).round() as i64, (v.1 * 32768.0).round() as i64);
    let mut count: HashMap<Key, u32> = HashMap::new();
    let mut all: Vec<[(f64, f64, f64); 3]> = Vec::new();
    for m in meshes {
        for t in 0..m.triangle_count() {
            let tri = m.triangle(t);
            all.push(tri);
            for i in 0..3 {
                let (a, b) = (key(tri[i]), key(tri[(i + 1) % 3]));
                *count.entry(if a <= b { (a, b) } else { (b, a) }).or_insert(0) += 1;
            }
        }
    }
    let mut out = Vec::new();
    for tri in &all {
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            let (ka, kb) = (key(a), key(b));
            if count[&if ka <= kb { (ka, kb) } else { (kb, ka) }] == 1 {
                out.push(((a.0, a.1), (b.0, b.1)));
            }
        }
    }
    out
}

/// How far the asphalt reaches either side of a painted line, in metres:
/// marches perpendicular to the line's own direction in 5 cm steps until the
/// paved region ends, capped at [`REACH_CAP_M`].
///
/// This, not the distance to the nearest silhouette, is what says whether a
/// line is where the cross-section puts it: a centre line is centred exactly
/// when the two reaches match.
const REACH_CAP_M: f64 = 20.0;
const REACH_STEP_M: f64 = 0.05;

fn cross_reach(
    paved: &[&SurfaceMesh],
    at: (f64, f64),
    seg: ((f64, f64, f64), (f64, f64, f64)),
    scale: &Scale,
) -> (f64, f64) {
    let (de, dn) = ((seg.1 .0 - seg.0 .0) * scale.mx, (seg.1 .1 - seg.0 .1) * scale.my);
    let len = (de * de + dn * dn).sqrt();
    if len < 1e-9 {
        return (0.0, 0.0);
    }
    // Perpendicular in metres, converted back to unit space per axis.
    let (pe, pn) = (-dn / len, de / len);
    let mut out = [0.0f64; 2];
    for (i, sign) in [1.0f64, -1.0].into_iter().enumerate() {
        let mut d = 0.0;
        while d < REACH_CAP_M {
            let next = d + REACH_STEP_M;
            let qx = at.0 + sign * pe * next / scale.mx;
            let qy = at.1 + sign * pn * next / scale.my;
            if !paved.iter().any(|m| m.height_at(qx, qy).is_some()) {
                break;
            }
            d = next;
        }
        out[i] = d;
    }
    (out[0], out[1])
}

/// Plan distance in metres from a point to the nearest segment of a set.
fn nearest_seg(segs: &[((f64, f64), (f64, f64))], px: f64, py: f64, scale: &Scale) -> f64 {
    let (qx, qy) = (px * scale.mx, py * scale.my);
    let mut best = f64::INFINITY;
    for &((ax, ay), (bx, by)) in segs {
        best = best.min(point_seg(qx, qy, ax * scale.mx, ay * scale.my, bx * scale.mx, by * scale.my));
    }
    best
}

/// Plan distance in metres to the nearest drawn road centerline whose baked
/// height is within a couple of metres of the marking's (so a road passing
/// overhead is not mistaken for the one the paint is on), with that road's
/// stated carriageway width.
fn nearest_axis(axes: &[&RoadLine], px: f64, py: f64, h: f64, scale: &Scale) -> (f64, f64) {
    let (qx, qy) = (px * scale.mx, py * scale.my);
    let (mut best, mut width) = (f64::INFINITY, 0.0);
    for line in axes {
        for part in &line.parts {
            for w in part.windows(2) {
                if (w[0].2 - h).abs() > 2.0 && (w[1].2 - h).abs() > 2.0 {
                    continue;
                }
                let d = point_seg(
                    qx,
                    qy,
                    w[0].0 * scale.mx,
                    w[0].1 * scale.my,
                    w[1].0 * scale.mx,
                    w[1].1 * scale.my,
                );
                if d < best {
                    best = d;
                    width = line.width_m;
                }
            }
        }
    }
    (best, width)
}

fn point_seg(qx: f64, qy: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (ex, ey) = (bx - ax, by - ay);
    let len2 = ex * ex + ey * ey;
    let t = if len2 > 0.0 { (((qx - ax) * ex + (qy - ay) * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (dx, dy) = (qx - (ax + ex * t), qy - (ay + ey * t));
    (dx * dx + dy * dy).sqrt()
}

fn quantiles(v: &mut Vec<f64>) -> [f64; 7] {
    v.sort_by(f64::total_cmp);
    let q = |p: f64| v[((v.len() - 1) as f64 * p) as usize];
    [q(0.0), q(0.05), q(0.25), q(0.5), q(0.75), q(0.95), q(1.0)]
}

fn report(kind: &str, s: &[Sample]) {
    let n = s.len();
    let off = s.iter().filter(|v| v.depth_m < 0.0).count();
    println!("{kind}: {n} vertices, {off} off the asphalt ({:.2} %)", 100.0 * off as f64 / n as f64);

    let d = quantiles(&mut s.iter().map(|v| v.depth_m).collect());
    println!(
        "  depth inside the silhouette (m): min {:.2}  p05 {:.2}  p25 {:.2}  median {:.2}  \
         p75 {:.2}  p95 {:.2}  max {:.2}",
        d[0], d[1], d[2], d[3], d[4], d[5], d[6]
    );

    // The asphalt either side of the paint. A centre line is centred exactly
    // when the two reaches match; the signed difference is the misalignment,
    // and its *absolute* value is what a threshold would be cut on.
    let reach: Vec<(f64, f64)> = s.iter().filter_map(|v| v.reach_m).collect();
    if !reach.is_empty() {
        let l = quantiles(&mut reach.iter().map(|r| r.0).collect());
        let r = quantiles(&mut reach.iter().map(|r| r.1).collect());
        println!(
            "  asphalt to the left  (m): p05 {:.2}  p25 {:.2}  median {:.2}  p75 {:.2}  p95 {:.2}",
            l[1], l[2], l[3], l[4], l[5]
        );
        println!(
            "  asphalt to the right (m): p05 {:.2}  p25 {:.2}  median {:.2}  p75 {:.2}  p95 {:.2}",
            r[1], r[2], r[3], r[4], r[5]
        );
        // Capped reaches mean the paint is inside a wide plate (an interchange,
        // a junction) where "the other kerb" is not the carriageway's — those
        // say nothing about centring, so they are counted, not averaged in.
        let capped = reach.iter().filter(|r| r.0 >= REACH_CAP_M || r.1 >= REACH_CAP_M).count();
        let mut asym: Vec<f64> = reach
            .iter()
            .filter(|r| r.0 < REACH_CAP_M && r.1 < REACH_CAP_M)
            .map(|r| (r.0 - r.1).abs())
            .collect();
        if !asym.is_empty() {
            let a = quantiles(&mut asym);
            println!(
                "  |left − right| (m), the off-centre error: p05 {:.2}  p25 {:.2}  median {:.2}  \
                 p75 {:.2}  p95 {:.2}  max {:.2}   [{capped} vertices hit the {REACH_CAP_M:.0} m \
                 cap and are excluded]",
                a[1], a[2], a[3], a[4], a[5], a[6]
            );
        }
    }

    // How far off, for the ones that are off at all: a tail of centimetres at
    // the kerb is quantization, a tail of metres is a lost offset.
    let mut out: Vec<f64> = s.iter().filter(|v| v.depth_m < 0.0).map(|v| -v.depth_m).collect();
    if !out.is_empty() {
        let o = quantiles(&mut out);
        println!(
            "  of those off the asphalt, how far (m): p05 {:.2}  p25 {:.2}  median {:.2}  \
             p75 {:.2}  p95 {:.2}  max {:.2}",
            o[1], o[2], o[3], o[4], o[5], o[6]
        );
        let near = out.iter().filter(|&&d| d <= 0.25).count();
        println!(
            "    {near} of {} are within 0.25 m of the kerb (quantization), {} are beyond it",
            out.len(),
            out.len() - near
        );
    }

    // What the archive draws instead, for the ones genuinely off: the shape of
    // the residue, not six anecdotes from its extreme.
    let mut kinds: BTreeMap<String, (usize, f64)> = BTreeMap::new();
    for v in s.iter().filter(|v| v.depth_m < -0.25) {
        let key = if v.over.contains("nothing") {
            let d = v
                .over
                .split("at ")
                .nth(1)
                .and_then(|t| t.split(' ').next())
                .and_then(|t| t.parse::<f64>().ok())
                .unwrap_or(f64::INFINITY);
            let band = if d < 1.0 {
                "under 1 m"
            } else if d < 3.0 {
                "1-3 m"
            } else if d < 20.0 {
                "3-20 m"
            } else {
                "over 20 m"
            };
            format!("nothing under the paint; nearest structure {band} away")
        } else {
            v.over.split('[').nth(1).unwrap_or("").trim_end_matches(']').to_string()
        };
        let e = kinds.entry(key).or_insert((0, 0.0));
        e.0 += 1;
        e.1 = e.1.max(-v.depth_m);
    }
    let mut ranked: Vec<_> = kinds.into_iter().collect();
    ranked.sort_by_key(|(_, (n, _))| std::cmp::Reverse(*n));
    for (k, (n, worst)) in ranked.iter().take(6) {
        println!("    {n:6} vertices (worst {worst:.1} m off): {k}");
    }

    // Where the paint is furthest outside the asphalt — sites to cut a section
    // at, not a number to average away.
    let mut worst: Vec<&Sample> = s.iter().filter(|v| v.depth_m < 0.0).collect();
    worst.sort_by(|a, b| a.depth_m.total_cmp(&b.depth_m));
    for v in worst.iter().take(6) {
        println!(
            "    off the asphalt by {:.2} m at {:.6},{:.6} — {}",
            -v.depth_m, v.lon, v.lat, v.over
        );
    }

    let mut stand: Vec<f64> = s.iter().filter_map(|v| v.standoff_m).collect();
    if !stand.is_empty() {
        let h = quantiles(&mut stand);
        println!(
            "  height over the surface (m): min {:.3}  p05 {:.3}  median {:.3}  p95 {:.3}  \
             max {:.3}",
            h[0], h[1], h[3], h[5], h[6]
        );
    }
    println!();
}
