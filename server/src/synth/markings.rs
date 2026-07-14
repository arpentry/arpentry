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

use crate::building_mesh::{Frame, M_PER_DEG_LAT, M_PER_DEG_LON_EQUATOR};
use crate::priors;
use crate::project::Bounds;
use crate::synth::surface::trim_line;
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

/// One painted line to emit: its geometry and painted width. The caller
/// attaches the `marking` class and the synth tag of the road it lies on.
pub struct Marking {
    pub geometry: Geometry,
    pub width_m: f64,
}

impl Marking {
    /// The tile properties of a marking feature.
    pub fn properties(&self) -> Vec<(String, Value)> {
        vec![
            ("class".to_string(), Value::String("marking".to_string())),
            ("width_m".to_string(), Value::Double(self.width_m)),
        ]
    }
}

/// The painted lines for one road line: a dashed centre line between opposing
/// flows and solid edge lines on the motorway network, per the ladder
/// (`priors::has_centre_line` / `has_edge_lines`). `line` must be pre-clip
/// geometry — a whole segment or a corridor span piece — so the dash phase
/// anchors to a global arclength origin. `disks` are the junction-plate trim
/// disks near the line: solid lines stop at them, and any dash whose midpoint
/// falls inside one is dropped.
pub fn for_line(
    line: &LineString,
    class: &str,
    oneway: bool,
    width_m: f64,
    disks: &[(Coord, f64)],
) -> Vec<Marking> {
    let centre = priors::has_centre_line(class, oneway);
    let edges = priors::has_edge_lines(class);
    if (!centre && !edges) || line.0.len() < 2 {
        return Vec::new();
    }
    let mut out = Vec::new();

    if centre {
        let dashes: Vec<LineString> = cut_dashes(line, CENTRE_DASH_M, CENTRE_GAP_M)
            .into_iter()
            .filter(|d| !midpoint_in_disk(d, disks))
            .collect();
        if !dashes.is_empty() {
            out.push(Marking {
                geometry: Geometry::MultiLineString(MultiLineString(dashes)),
                width_m: priors::CENTRE_LINE_WIDTH_M,
            });
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
                let pieces: Vec<LineString> = trim_line(&edge, disks)
                    .into_iter()
                    .filter(|p| line_len_m(p, &frame) >= MIN_LINE_M)
                    .collect();
                if !pieces.is_empty() {
                    out.push(Marking {
                        geometry: Geometry::MultiLineString(MultiLineString(pieces)),
                        width_m: priors::EDGE_LINE_WIDTH_M,
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

/// Whether a dash's midpoint lies inside any trim disk.
fn midpoint_in_disk(dash: &LineString, disks: &[(Coord, f64)]) -> bool {
    let pts = &dash.0;
    let arc = arc_lengths(pts);
    let mid = slice(pts, &arc, arc.last().copied().unwrap_or(0.0) * 0.5, f64::INFINITY);
    let Some(&m) = mid.first() else {
        return false;
    };
    let cosk = m.y.to_radians().cos();
    disks.iter().any(|&(c, r)| {
        let de = (m.x - c.x) * M_PER_DEG_LON_EQUATOR * cosk;
        let dn = (m.y - c.y) * M_PER_DEG_LAT;
        de * de + dn * dn < r * r
    })
}

/// Cumulative arclength in metres at each vertex.
fn arc_lengths(pts: &[Coord]) -> Vec<f64> {
    let cosk = pts[0].y.to_radians().cos();
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0);
    for w in pts.windows(2) {
        let de = (w[1].x - w[0].x) * M_PER_DEG_LON_EQUATOR * cosk;
        let dn = (w[1].y - w[0].y) * M_PER_DEG_LAT;
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
            let dn = (w[1].y - w[0].y) * M_PER_DEG_LAT;
            (de * de + dn * dn).sqrt()
        })
        .sum()
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
        let (de, dn) = ((b.x - a.x) * m_lon, (b.y - a.y) * M_PER_DEG_LAT);
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
            y: pts[i].y + pn * reach / M_PER_DEG_LAT,
        });
    }
    (out.len() >= 2).then(|| LineString(out))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight(len_m: f64) -> LineString {
        let cy: f64 = 46.0;
        let m_lon = M_PER_DEG_LON_EQUATOR * cy.to_radians().cos();
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
        let m_lon = M_PER_DEG_LON_EQUATOR * cy.to_radians().cos();
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
        // A one-way carriageway paints no centre line.
        assert!(for_line(&line, "secondary", true, 6.0, &[]).is_empty());
        // The motorway paints two solid edge lines and no centre.
        let mw = for_line(&line, "motorway", true, 9.0, &[]);
        assert_eq!(mw.len(), 2);
        for edge in &mw {
            assert_eq!(edge.width_m, priors::EDGE_LINE_WIDTH_M);
            assert!(matches!(&edge.geometry, Geometry::MultiLineString(m) if m.0.len() == 1));
        }
    }

    #[test]
    fn dashes_inside_a_plate_disk_are_dropped() {
        let line = straight(50.0);
        let cy: f64 = 46.0;
        let m_lon = M_PER_DEG_LON_EQUATOR * cy.to_radians().cos();
        // A disk over the middle: the [20,24] dash (mid 22) falls inside.
        let disk = (Coord { x: 7.0 + 22.0 / m_lon, y: cy }, 5.0);
        let out = for_line(&line, "secondary", false, 6.0, &[disk]);
        assert_eq!(out.len(), 1);
        let Geometry::MultiLineString(m) = &out[0].geometry else {
            panic!("a multiline of dashes");
        };
        assert_eq!(m.0.len(), 4, "the covered dash is dropped");
    }
}
