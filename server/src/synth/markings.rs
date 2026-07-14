//! Longitudinal road markings — the painted lines on the carriageway
//! (docs/ROADS.md P3, increment 1: the solid lines).
//!
//! Markings are baked server-side as ordinary narrow line features of class
//! `marking`, so the client's existing stroke pipeline draws them —
//! SDF-antialiased, at physical width, over the road paint they follow in
//! draw order — with no client change; the style file gives the class its
//! colour. The lane model never reaches the client (ROADS.md §6.2): which
//! lines a road paints is decided here from the class ladder, the one-way
//! verdict, and the P1 width — a centre line between opposing flows, edge
//! lines inset from the carriageway edge — and every line drapes on the same
//! road-surface height as the paint it rides ([`road::surface_height`] via
//! [`road::bake`]) and stops at the junction plates the surface bands stop
//! at. Dashed lane lines and symbols need the global-phase machinery
//! (ROADS.md H4) and wait for the next increments.

use geo_types::{Coord, Geometry, LineString};

use crate::building_mesh::{Frame, M_PER_DEG_LAT};
use crate::ground::sampler::GroundSampler;
use crate::priors;
use crate::project::Bounds;
use crate::solve::SolvedModel;
use crate::synth::junction::BakedJunction;
use crate::synth::surface::trim_line;
use crate::synth::{road, Synth};
use crate::tile_build::EncoderFeature;
use crate::value::Value;

/// Cap on the offset miter scale at a bend, matching the surface band's.
const MITER_MAX: f64 = 1.5;

/// Shortest painted line worth emitting, in metres: the stubs left between
/// close junctions read as noise, not as road markings.
const MIN_LINE_M: f64 = 8.0;

/// The marking lines painted on one road feature: empty below the marking
/// zoom, for non-drivable classes (no `width_m`), and for classes whose
/// ladder paints nothing. Call *before* [`crate::synth::emit`], on the raw
/// clipped centerline — the baked line's dense lattice-crossing vertices
/// make an offset wobble — and each marking then bakes itself onto the same
/// road surface (deck ramp included, so lines carry across bridges).
pub fn lines(
    f: &EncoderFeature,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    z: u8,
    bounds: &Bounds,
    plates: &[&BakedJunction],
) -> Vec<EncoderFeature> {
    if z < priors::MARKING_MIN_ZOOM || !sampler.has_elevation() {
        return Vec::new();
    }
    let Synth::Road { corridor, deck } = f.synth else {
        return Vec::new();
    };
    let mut class = None;
    let mut width_m = None;
    let mut oneway = false;
    for (k, v) in &f.properties {
        match (k.as_str(), v) {
            ("class", Value::String(s)) => class = Some(s.as_str()),
            ("width_m", Value::Double(w)) => width_m = Some(*w),
            ("oneway", Value::Bool(b)) => oneway = *b,
            _ => {}
        }
    }
    let (Some(class), Some(width_m)) = (class, width_m) else {
        return Vec::new();
    };
    let centre = priors::has_centre_line(class, oneway);
    let edges = priors::has_edge_lines(class);
    if !centre && !edges {
        return Vec::new();
    }

    let lines: Vec<&LineString> = match &f.geometry {
        Geometry::LineString(ls) => vec![ls],
        Geometry::MultiLineString(mls) => mls.0.iter().collect(),
        _ => return Vec::new(),
    };
    let half_m = width_m * 0.5 + priors::STRUCTURE_SHOULDER_M;
    let disks: Vec<(Coord, f64)> =
        plates.iter().map(|p| (p.point(), p.trim_radius_m(half_m))).collect();
    let frame = Frame::at_center(bounds);
    let profile = corridor.and_then(|c| solved.profile(c));

    let mut out = Vec::new();
    let mut emit = |geometry: LineString, paint_w: f64| {
        for piece in trim_line(&geometry, &disks) {
            if line_len_m(&piece, &frame) < MIN_LINE_M {
                continue;
            }
            let mut feat = EncoderFeature {
                id: f.id,
                geometry: Geometry::LineString(piece),
                properties: vec![
                    ("class".to_string(), Value::String("marking".to_string())),
                    ("width_m".to_string(), Value::Double(paint_w)),
                ],
                elevation: None,
                z: None,
                mesh: None,
                synth: Synth::None,
            };
            road::bake(&mut feat, profile, deck, sampler, z, solved.z_ref, bounds);
            out.push(feat);
        }
    };
    for line in lines {
        if centre {
            emit(line.clone(), priors::CENTRE_LINE_WIDTH_M);
        }
        if edges {
            let inset = width_m * 0.5 - priors::EDGE_LINE_INSET_M;
            if inset > priors::EDGE_LINE_WIDTH_M {
                for side in [1.0, -1.0] {
                    if let Some(edge) = offset_line(line, inset * side, &frame) {
                        emit(edge, priors::EDGE_LINE_WIDTH_M);
                    }
                }
            }
        }
    }
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

    #[test]
    fn offset_runs_parallel_at_the_offset_distance() {
        let b = Bounds::of_tile(15, 17000, 11600);
        let frame = Frame::at_center(&b);
        let cy = b.south + 0.5 * b.height();
        let line = LineString(vec![
            Coord { x: b.west + 0.3 * b.width(), y: cy },
            Coord { x: b.west + 0.7 * b.width(), y: cy },
        ]);
        let left = offset_line(&line, 2.0, &frame).expect("an offset line");
        assert_eq!(left.0.len(), 2);
        for c in &left.0 {
            let dn = (c.y - cy) * M_PER_DEG_LAT;
            assert!((dn - 2.0).abs() < 0.05, "offset {dn} != 2 m left (north)");
        }
        let right = offset_line(&line, -2.0, &frame).expect("an offset line");
        assert!(right.0.iter().all(|c| (c.y - cy) * M_PER_DEG_LAT < -1.9));
    }

    #[test]
    fn ladder_gates_centre_and_edge_lines() {
        // Engineered two-way roads paint a centre line; one-way carriageways
        // and quiet streets do not.
        assert!(priors::has_centre_line("primary", false));
        assert!(!priors::has_centre_line("primary", true));
        assert!(!priors::has_centre_line("residential", false));
        assert!(priors::has_edge_lines("motorway"));
        assert!(!priors::has_edge_lines("residential"));
    }
}
