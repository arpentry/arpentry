//! Longitudinal road markings — the painted lines on the carriageway
//! (docs/ROADS.md P3).
//!
//! Markings are baked server-side as ordinary narrow line features of class
//! `marking`, so the client's existing stroke pipeline draws them —
//! SDF-antialiased, at physical width, over the road paint they follow in
//! draw order — with no client change; the style file gives the class its
//! colour. The lane model never reaches the client (ROADS.md §6.2): which
//! lines a road paints is decided here from the class ladder, the one-way
//! verdict, and the P1 width — a dashed centre line between opposing flows,
//! solid edge lines on the motorway network.
//!
//! Generation runs in **phase 1**, on the pre-clip geometry — the whole
//! unclaimed segment, or a corridor piece cut at its global span boundaries —
//! never on a tile window. That is what makes the dash phase global
//! (ROADS.md H4, invariant 4): the dashes are cut once from arclength zero of
//! a global object and then clipped like any feature, so any two tiles carry
//! identical copies of every dash. The emitted features ride a
//! [`Synth::Road`] tag and bake their heights in phase 2 exactly like the
//! road paint they lie on.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::building_mesh::Frame;
use crate::scene::DEG_M;
use crate::priors;
use crate::project::Bounds;
use crate::synth::area::Area;
use crate::value::Value;

/// Cap on the offset miter scale at a bend, matching the surface band's.
const MITER_MAX: f64 = 1.5;

/// Shortest painted solid line worth emitting, in metres: the stubs left
/// between close junctions read as noise, not as road markings.
const MIN_LINE_M: f64 = 8.0;

/// The centre guide line's dash pattern in metres — the Swiss Leitlinie runs
/// 3/3 m in town and 6/6 m outside; one compromise pattern keeps the phase
/// machinery simple.
const CENTRE_DASH_M: f64 = 4.0;
const CENTRE_GAP_M: f64 = 6.0;

/// The lane divider's dash pattern in metres — longer stride than the centre
/// guide line, as on a real carriageway.
const LANE_DASH_M: f64 = 5.0;
const LANE_GAP_M: f64 = 9.0;

/// One painted line to emit: its geometry and painted width. The caller
/// attaches the `marking` class and the synth tag of the road it lies on.
pub struct Marking {
    pub geometry: Geometry,
    pub width_m: f64,
    /// The style class the paint is emitted as: `"marking"` for road paint,
    /// `"rail_line"` for the rail heads riding the ballast band. Distinct
    /// classes because colour is keyed by class (paint is white, a rail is
    /// dark), and because the calibrated `paint.*` check populations filter
    /// on the literal `"marking"`.
    pub class: &'static str,
}

impl Marking {
    /// The tile properties of a marking feature.
    pub fn properties(&self) -> Vec<(String, Value)> {
        vec![
            ("class".to_string(), Value::String(self.class.to_string())),
            ("width_m".to_string(), Value::Double(self.width_m)),
        ]
    }
}

/// The painted lines for one road line: a dashed centre line between opposing
/// flows, dashed dividers between the same-direction lanes of a one-way
/// carriageway (their count inferred from the width — ROADS.md H2), and
/// solid edge lines on the motorway network, per the ladder
/// (`priors::has_centre_line` / `has_lane_lines` / `has_edge_lines`). `line`
/// must be pre-clip geometry — a whole segment or a corridor span piece — so
/// the dash phase anchors to a global arclength origin. `areas` are the paved
/// intersections near the line: solid lines stop at them, and any dash whose
/// midpoint falls inside one is dropped.
pub fn for_line(
    line: &LineString,
    class: &str,
    oneway: bool,
    width_m: f64,
    areas: &[&Area],
) -> Vec<Marking> {
    let centre = priors::has_centre_line(class, oneway);
    let lanes = if priors::has_lane_lines(class, oneway) {
        priors::lane_count(class, width_m)
    } else {
        1
    };
    let edges = priors::has_edge_lines(class);
    if (!centre && !edges && lanes < 2) || line.0.len() < 2 {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut push_dashes = |dashes: Vec<LineString>, width: f64| {
        let kept: Vec<LineString> =
            dashes.into_iter().filter(|d| !midpoint_paved(d, areas)).collect();
        if !kept.is_empty() {
            out.push(Marking {
                geometry: Geometry::MultiLineString(MultiLineString(kept)),
                width_m: width,
                class: "marking",
            });
        }
    };

    if centre {
        push_dashes(cut_dashes(line, CENTRE_DASH_M, CENTRE_GAP_M), priors::CENTRE_LINE_WIDTH_M);
    }
    // Dashed dividers at the n−1 interior lane boundaries of the carriageway.
    if lanes >= 2 {
        let frame = frame_at(line.0[0]);
        for k in 1..lanes {
            let off = -width_m * 0.5 + k as f64 * (width_m / lanes as f64);
            let boundary = if off.abs() < 1e-3 {
                Some(line.clone())
            } else {
                offset_line(line, off, &frame)
            };
            if let Some(b) = boundary {
                push_dashes(cut_dashes(&b, LANE_DASH_M, LANE_GAP_M), priors::CENTRE_LINE_WIDTH_M);
            }
        }
    }
    if edges {
        let inset = width_m * 0.5 - priors::EDGE_LINE_INSET_M;
        if inset > priors::EDGE_LINE_WIDTH_M {
            let frame = frame_at(line.0[0]);
            for side in [1.0, -1.0] {
                let Some(edge) = offset_line(line, inset * side, &frame) else {
                    continue;
                };
                let pieces: Vec<LineString> = trim_line(&edge, areas)
                    .into_iter()
                    .filter(|p| line_len_m(p, &frame) >= MIN_LINE_M)
                    .collect();
                if !pieces.is_empty() {
                    out.push(Marking {
                        geometry: Geometry::MultiLineString(MultiLineString(pieces)),
                        width_m: priors::EDGE_LINE_WIDTH_M,
                        class: "marking",
                    });
                }
            }
        }
    }
    out
}

/// A local ENU frame at a point (marking generation runs before any tile
/// exists, so there are no tile bounds to centre one on).
fn frame_at(c: Coord) -> Frame {
    frame_for(c.x, c.y)
}

fn frame_for(lon: f64, lat: f64) -> Frame {
    let b = Bounds { west: lon, south: lat, east: lon, north: lat };
    Frame::at_center(&b)
}

/// Cuts the line into dashes of `dash_m` every `dash_m + gap_m`, phase
/// anchored at the line's start (a global arclength origin — see the module
/// doc). Dashes clipped by the line's end shorter than half a dash are
/// dropped rather than painted as crumbs.
fn cut_dashes(line: &LineString, dash_m: f64, gap_m: f64) -> Vec<LineString> {
    let pts = &line.0;
    let arc = arc_lengths(pts);
    let total = *arc.last().expect("non-empty");
    let period = dash_m + gap_m;
    let mut out = Vec::new();
    let mut s = 0.0;
    while s < total {
        let e = (s + dash_m).min(total);
        if e - s >= dash_m * 0.5 {
            let piece = slice(pts, &arc, s, e);
            if piece.len() >= 2 {
                out.push(LineString(piece));
            }
        }
        s += period;
    }
    out
}

/// Whether a dash's midpoint falls on a paved intersection — no road marking
/// is painted through one.
fn midpoint_paved(dash: &LineString, areas: &[&Area]) -> bool {
    let pts = &dash.0;
    let arc = arc_lengths(pts);
    let mid = slice(pts, &arc, arc.last().copied().unwrap_or(0.0) * 0.5, f64::INFINITY);
    let Some(&m) = mid.first() else {
        return false;
    };
    areas.iter().any(|a| a.contains(m))
}

/// Cumulative arclength in metres at each vertex.
fn arc_lengths(pts: &[Coord]) -> Vec<f64> {
    let cosk = pts[0].y.to_radians().cos();
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0);
    for w in pts.windows(2) {
        let de = (w[1].x - w[0].x) * DEG_M * cosk;
        let dn = (w[1].y - w[0].y) * DEG_M;
        arc.push(arc.last().expect("non-empty") + (de * de + dn * dn).sqrt());
    }
    arc
}

/// The coordinates covering arclength `[a, b]`, interpolating the endpoints
/// and keeping the original vertices between them.
fn slice(pts: &[Coord], arc: &[f64], a: f64, b: f64) -> Vec<Coord> {
    let total = *arc.last().expect("non-empty");
    let (a, b) = (a.clamp(0.0, total), b.clamp(0.0, total));
    let at = |s: f64| -> Coord {
        let i = match arc.binary_search_by(|v| v.total_cmp(&s)) {
            Ok(i) => i,
            Err(i) => i - 1,
        };
        if i >= pts.len() - 1 {
            return pts[pts.len() - 1];
        }
        let len = arc[i + 1] - arc[i];
        let t = if len > 0.0 { ((s - arc[i]) / len).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
            y: pts[i].y + (pts[i + 1].y - pts[i].y) * t,
        }
    };
    let mut out = vec![at(a)];
    for (i, &s) in arc.iter().enumerate() {
        if s > a && s < b {
            out.push(pts[i]);
        }
    }
    out.push(at(b));
    out.dedup_by(|p, q| (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12);
    out
}

/// A line's length in metres (local equirectangular scale).
fn line_len_m(line: &LineString, frame: &Frame) -> f64 {
    line.0
        .windows(2)
        .map(|w| {
            let de = (w[1].x - w[0].x) * frame.m_per_deg_lon;
            let dn = (w[1].y - w[0].y) * DEG_M;
            (de * de + dn * dn).sqrt()
        })
        .sum()
}

/// The two rail heads of a track: the centerline offset ±half the class
/// gauge, each drawn as a thin dark line riding the ballast band the way
/// road paint rides the asphalt (docs/ROADS.md P3, extended to rail).
///
/// No dashes and no intersection trim, on purpose: a rail is continuous
/// steel, and at a level crossing it runs *through* the asphalt — trimming
/// it at the crossing's area would break exactly the place a rail is most
/// visible. The lateral offset survives the paint snap because
/// `Profile::smooth_at` re-applies the query point's offset about the raw
/// edge onto the smoothed curve (the two-curves fix; guarded by
/// `paint.edge_line_inset` for road paint).
pub fn rails_for_line(line: &LineString, gauge_m: f64) -> Vec<Marking> {
    if line.0.len() < 2 {
        return Vec::new();
    }
    let frame = frame_at(line.0[0]);
    [0.5, -0.5]
        .into_iter()
        .filter_map(|side| offset_line(line, gauge_m * side, &frame))
        .map(|rail| Marking {
            geometry: Geometry::LineString(rail),
            width_m: priors::RAIL_HEAD_WIDTH_M,
            class: "rail_line",
        })
        .collect()
}

/// The centerline offset sideways by `offset_m` (positive = left of travel),
/// with the same averaged-perpendicular miter the surface band uses, or
/// `None` for a degenerate line.
fn offset_line(line: &LineString, offset_m: f64, frame: &Frame) -> Option<LineString> {
    let pts = &line.0;
    if pts.len() < 2 {
        return None;
    }
    let m_lon = frame.m_per_deg_lon;
    let dir = |a: Coord, b: Coord| -> Option<(f64, f64)> {
        let (de, dn) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
        let len = (de * de + dn * dn).sqrt();
        (len > 1e-9).then(|| (de / len, dn / len))
    };
    let mut out = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let before = (i > 0).then(|| dir(pts[i - 1], pts[i])).flatten();
        let after = (i + 1 < pts.len()).then(|| dir(pts[i], pts[i + 1])).flatten();
        let (e, n, scale) = match (before, after) {
            (Some((e0, n0)), Some((e1, n1))) => {
                let (se, sn) = (e0 + e1, n0 + n1);
                let len = (se * se + sn * sn).sqrt();
                if len < 1e-9 {
                    continue;
                }
                (se / len, sn / len, (1.0 / (len * 0.5).min(1.0)).min(MITER_MAX))
            }
            (Some(d), None) | (None, Some(d)) => (d.0, d.1, 1.0),
            (None, None) => continue,
        };
        let (pe, pn) = (-n, e);
        let reach = offset_m * scale;
        out.push(Coord {
            x: pts[i].x + pe * reach / m_lon,
            y: pts[i].y + pn * reach / DEG_M,
        });
    }
    (out.len() >= 2).then(|| LineString(out))
}

/// Cuts the parts of a painted line that fall inside any intersection area,
/// returning the pieces that remain — the whole line when no area touches it.
/// Handles a line *ending* at an intersection and one passing through it alike,
/// and a line that leaves an area and re-enters it.
///
/// Longitudinal paint stops *at* the intersection, so the cut is exact: the
/// half-metre tuck this carried when it also trimmed surface bands existed only
/// to hide a band's end under a plate, and there are no plates now. Short
/// survivors are the caller's business — the marking ladder already drops stubs
/// under [`MIN_LINE_M`].
fn trim_line(line: &LineString, areas: &[&Area]) -> Vec<LineString> {
    let pts = &line.0;
    if pts.len() < 2 {
        return Vec::new();
    }
    // Cumulative arclength in metres (local equirectangular scale).
    let cosk = pts[0].y.to_radians().cos();
    let en = |from: Coord, to: Coord| {
        ((to.x - from.x) * DEG_M * cosk, (to.y - from.y) * DEG_M)
    };
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0);
    for w in pts.windows(2) {
        let (de, dn) = en(w[0], w[1]);
        arc.push(arc.last().expect("non-empty") + (de * de + dn * dn).sqrt());
    }
    let total = *arc.last().expect("non-empty");
    if total <= 0.0 {
        return Vec::new();
    }

    // The arc intervals the areas cover: per segment, clip the chord against
    // each area's leg rectangles.
    let mut cut: Vec<(f64, f64)> = Vec::new();
    let mut chord: Vec<(f64, f64)> = Vec::new();
    for area in areas {
        for (i, w) in pts.windows(2).enumerate() {
            chord.clear();
            area.clip_chord(w[0], w[1], &mut chord);
            let len = arc[i + 1] - arc[i];
            for &(t0, t1) in &chord {
                if t1 > t0 {
                    cut.push((arc[i] + t0 * len, arc[i] + t1 * len));
                }
            }
        }
    }
    if cut.is_empty() {
        return vec![line.clone()];
    }
    cut.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for iv in cut {
        match merged.last_mut() {
            Some(last) if iv.0 <= last.1 => last.1 = last.1.max(iv.1),
            _ => merged.push(iv),
        }
    }
    // Walk the complement, rebuilding each kept span's coordinates.
    let mut keep: Vec<(f64, f64)> = Vec::new();
    let mut s = 0.0;
    for &(a, b) in &merged {
        if a > s {
            keep.push((s, a));
        }
        s = s.max(b);
    }
    if total > s {
        keep.push((s, total));
    }
    keep.iter()
        .map(|&(a, b)| LineString(slice_arc(pts, &arc, a, b)))
        .filter(|l| l.0.len() >= 2)
        .collect()
}

/// The coordinates of the sub-line covering arclength `[a, b]`, interpolating
/// the cut endpoints and keeping the original vertices between them.
fn slice_arc(pts: &[Coord], arc: &[f64], a: f64, b: f64) -> Vec<Coord> {
    let at = |s: f64| -> Coord {
        let i = match arc.binary_search_by(|v| v.total_cmp(&s)) {
            Ok(i) => i,
            Err(i) => i - 1, // arc[0] = 0 ≤ s, so i ≥ 1 here
        };
        if i >= pts.len() - 1 {
            return pts[pts.len() - 1];
        }
        let len = arc[i + 1] - arc[i];
        let t = if len > 0.0 { ((s - arc[i]) / len).clamp(0.0, 1.0) } else { 0.0 };
        Coord {
            x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
            y: pts[i].y + (pts[i + 1].y - pts[i].y) * t,
        }
    };
    let mut out = vec![at(a)];
    for (i, &s) in arc.iter().enumerate() {
        if s > a && s < b {
            out.push(pts[i]);
        }
    }
    out.push(at(b));
    // A cut landing exactly on a vertex would duplicate it.
    out.dedup_by(|p, q| (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(len_m: f64) -> LineString {
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        LineString(vec![
            Coord { x: 7.0, y: cy },
            Coord { x: 7.0 + len_m / m_lon, y: cy },
        ])
    }

    #[test]
    fn dashes_cut_on_the_global_period() {
        // 50 m at 4/6: dashes at [0,4], [10,14], [20,24], [30,34], [40,44].
        let dashes = cut_dashes(&straight(50.0), 4.0, 6.0);
        assert_eq!(dashes.len(), 5);
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        for (k, d) in dashes.iter().enumerate() {
            let s = (d.0.first().expect("start").x - 7.0) * m_lon;
            let e = (d.0.last().expect("end").x - 7.0) * m_lon;
            assert!((s - 10.0 * k as f64).abs() < 0.01, "dash {k} starts at {s}");
            assert!((e - s - 4.0).abs() < 0.01, "dash {k} length {}", e - s);
        }
        // A tail crumb shorter than half a dash is dropped: 42 m ends mid-gap
        // for the 5th dash window at [40, 42] → kept (2 m = dash/2)…
        assert_eq!(cut_dashes(&straight(42.0), 4.0, 6.0).len(), 5);
        // …but 41 m leaves only 1 m of it → dropped.
        assert_eq!(cut_dashes(&straight(41.0), 4.0, 6.0).len(), 4);
    }

    #[test]
    fn centre_line_is_dashed_and_edge_lines_are_solid() {
        let line = straight(100.0);
        let centre = for_line(&line, "secondary", false, 6.0, &[]);
        assert_eq!(centre.len(), 1);
        assert!(matches!(&centre[0].geometry, Geometry::MultiLineString(m) if m.0.len() == 10));
        assert_eq!(centre[0].width_m, priors::CENTRE_LINE_WIDTH_M);
        // A one-way secondary of one lane's width paints nothing.
        assert!(for_line(&line, "secondary", true, 3.5, &[]).is_empty());
        // A two-lane motorway carriageway paints one dashed lane divider
        // (down its middle) plus two solid edge lines — no centre line.
        let mw = for_line(&line, "motorway", true, 9.0, &[]);
        assert_eq!(mw.len(), 3);
        let dashed = mw
            .iter()
            .filter(|m| matches!(&m.geometry, Geometry::MultiLineString(g) if g.0.len() > 1))
            .count();
        assert_eq!(dashed, 1, "one dashed divider");
        let solid = mw.iter().filter(|m| m.width_m == priors::EDGE_LINE_WIDTH_M).count();
        assert_eq!(solid, 2, "two solid edge lines");
        // A wide (three-lane) carriageway paints two dividers.
        let wide = for_line(&line, "motorway", true, 12.0, &[]);
        assert_eq!(wide.len(), 4, "two dividers + two edges");
    }

    #[test]
    fn lane_count_infers_back_from_the_width() {
        assert_eq!(priors::lane_count("motorway", 9.0), 2);
        assert_eq!(priors::lane_count("motorway", 12.5), 3);
        assert_eq!(priors::lane_count("primary", 7.0), 2);
        assert_eq!(priors::lane_count("primary", 3.5), 1);
        // Untagged motorways still divide: one-way by construction.
        assert!(priors::has_lane_lines("motorway", false));
        assert!(!priors::has_lane_lines("primary", false));
        assert!(priors::has_lane_lines("primary", true));
    }

    #[test]
    fn dashes_inside_an_intersection_are_dropped() {
        let line = straight(50.0);
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        // A crossroads over the middle: the [20,24] dash (mid 22) is on it.
        let centre = Coord { x: 7.0 + 22.0 / m_lon, y: cy };
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: 5.0 },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: 5.0 },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: 5.0 },
            crate::synth::area::Leg { e: 0.0, n: -1.0, half_w: 5.0 },
        ];
        let area = Area::new(centre, legs, 5.0).expect("an intersection");
        let out = for_line(&line, "secondary", false, 6.0, &[&area]);
        assert_eq!(out.len(), 1);
        let Geometry::MultiLineString(m) = &out[0].geometry else {
            panic!("a multiline of dashes");
        };
        assert_eq!(m.0.len(), 4, "the covered dash is dropped");
    }

    /// A square intersection of half-extent `half_m` centred on `c` — four
    /// equal legs at the compass points, the shape the trim tests cut against.
    fn square_area(c: Coord, half_m: f64) -> Area {
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: half_m },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: half_m },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: half_m },
            crate::synth::area::Leg { e: 0.0, n: -1.0, half_w: half_m },
        ];
        Area::new(c, legs, half_m).expect("a square area")
    }

    #[test]
    fn trim_splits_a_through_line_at_the_intersection() {
        // A 200 m west→east line through a 10 m intersection at its middle:
        // two pieces, each ending at the intersection edge (tucked under it).
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let c = Coord { x: 7.0, y: cy };
        let line = LineString(vec![
            Coord { x: c.x - 100.0 / m_lon, y: cy },
            Coord { x: c.x + 100.0 / m_lon, y: cy },
        ]);
        let area = square_area(c, 10.0);
        let pieces = trim_line(&line, &[&area]);
        assert_eq!(pieces.len(), 2, "the intersection splits the line");
        for p in &pieces {
            for v in &p.0 {
                let d = ((v.x - c.x) * m_lon).abs();
                // Up to the boundary less the tuck, never past it.
                assert!(d > 10.0 - 0.1, "piece vertex {d:.2} m inside the plate");
            }
        }
        // An end-of-line junction shortens rather than splits.
        let at_end = square_area(line.0[1], 10.0);
        let end = trim_line(&line, &[&at_end]);
        assert_eq!(end.len(), 1);
        let last = end[0].0.last().expect("non-empty");
        assert!(((last.x - line.0[1].x) * m_lon).abs() > 10.0 - 0.1);
        // No intersections → the whole line; one swallowing it → nothing.
        assert_eq!(trim_line(&line, &[]).len(), 1);
        let huge = square_area(c, 150.0);
        assert!(trim_line(&line, &[&huge]).is_empty());
    }

    #[test]
    fn trim_survives_a_line_that_leaves_and_re_enters() {
        // A line clipping the north arm of a cross, dipping out over the
        // corner notch and back in — two cuts, one kept piece between them.
        // The circular trim this replaced could only ever make one cut.
        let cy: f64 = 46.0;
        let m_lon = DEG_M * cy.to_radians().cos();
        let c = Coord { x: 7.0, y: cy };
        let legs = vec![
            crate::synth::area::Leg { e: 1.0, n: 0.0, half_w: 4.0 },
            crate::synth::area::Leg { e: -1.0, n: 0.0, half_w: 4.0 },
            crate::synth::area::Leg { e: 0.0, n: 1.0, half_w: 4.0 },
        ];
        // Reach 20 m: the three arms stick well out of the 8 m core, so a
        // line across them at 12 m north is inside, outside, inside.
        let area = Area::new(c, legs, 20.0).expect("a tee");
        let at = |de: f64, dn: f64| Coord { x: c.x + de / m_lon, y: c.y + dn / DEG_M };
        let line = LineString(vec![at(-30.0, 12.0), at(30.0, 12.0)]);
        let pieces = trim_line(&line, &[&area]);
        assert_eq!(pieces.len(), 2, "west and east of the north arm survive");
        for p in &pieces {
            for v in &p.0 {
                let de = (v.x - c.x) * m_lon;
                assert!(de.abs() > 4.0 - 0.1, "vertex {de:.2} m inside the arm");
            }
        }
    }
}
