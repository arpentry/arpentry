//! Road-surface bands — the asphalt under the painted carriageway
//! (docs/ROADS.md §6, P2 increment 1).
//!
//! At detail zooms every drivable at-grade road gains a draped surface mesh:
//! its baked centerline offset to the carriageway width plus the structure
//! shoulder, so the band frames the paint stroke exactly as a bridge deck
//! frames its ribbon and meets decks and junction plates with no width step.
//! Every band vertex drapes on the same road-surface height the paint bakes
//! ([`road::surface_height`] — ROADS.md invariant 5: one height answer for
//! every representation), sunk [`priors::SURFACE_SINK_M`] so plates and deck
//! tops win their overlaps cleanly; the client's deck depth margin lifts the
//! band over the terrain it drapes on. The band rides the same decode path as
//! the junction plates (a level-0 transportation mesh coloured by class), so
//! no client change is needed. Junction fillets and the true plate union are
//! later increments — where a band crosses a plate today they are coplanar
//! asphalt and the sink keeps the plate on top.

use geo_types::{Coord, Geometry, LineString, Point};

use crate::building_mesh::{Frame, M_PER_DEG_LAT, M_PER_DEG_LON_EQUATOR};
use crate::ground::sampler::GroundSampler;
use crate::priors;
use crate::project::{self, Bounds};
use crate::solve::SolvedModel;
use crate::synth::junction::BakedJunction;
use crate::synth::{road, Synth};
use crate::terrain::TerrainMesh;
use crate::tile_build::EncoderFeature;
use crate::value::Value;

/// Cap on the miter scale at a bend, so a hairpin vertex cannot spike the
/// band edge far past the carriageway.
const MITER_MAX: f64 = 1.5;

/// Shortest band fragment worth meshing after the junction trim, in metres —
/// anything shorter is a sliver the plates already cover.
const MIN_PIECE_M: f64 = 1.0;

/// Builds the surface band under one road feature, or `None` when the feature
/// carries none: below the surface zoom, no elevation, structure paint (its
/// deck is the surface), a non-drivable class (no `width_m`), or degenerate
/// geometry. Call after [`crate::synth::emit`] so the centerline is already
/// densified and snapped — the band reuses those vertices. `plates` are the
/// junctions near this tile: the band is trimmed back at each one's disk so
/// it lands under the plate mouth instead of running through the
/// intersection.
pub fn ribbon(
    f: &EncoderFeature,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    z: u8,
    bounds: &Bounds,
    plates: &[&BakedJunction],
) -> Option<EncoderFeature> {
    if z < priors::ROAD_SURFACE_MIN_ZOOM || !sampler.has_elevation() {
        return None;
    }
    // Only at-grade paint gets a band: a structure span already carries its
    // deck or bore solid, and the deck stroke rides its top.
    let Synth::Road { corridor, deck: false } = f.synth else {
        return None;
    };
    // Markings are also `Road { deck: false }` and carry their own thin
    // `width_m`, but they are painted lines, not carriageways — never give one
    // a band. (Also what lets `stamp_synth` drop a carriageway's fill stroke
    // "where a band exists" without mistaking a marking for a road.)
    if f.properties.iter().any(|(k, v)| {
        k.as_str() == "class" && matches!(v, Value::String(s) if s.as_str() == "marking")
    }) {
        return None;
    }
    let width_m = f.properties.iter().find_map(|(k, v)| match (k.as_str(), v) {
        ("width_m", Value::Double(w)) => Some(*w),
        _ => None,
    })?;
    let half_m = width_m * 0.5 + priors::STRUCTURE_SHOULDER_M;

    let lines: Vec<&LineString> = match &f.geometry {
        Geometry::LineString(ls) => vec![ls],
        Geometry::MultiLineString(mls) => mls.0.iter().collect(),
        _ => return None,
    };
    let disks: Vec<(Coord, f64)> =
        plates.iter().map(|p| (p.point(), p.trim_radius_m(half_m))).collect();
    let profile = corridor.and_then(|c| solved.profile(c));
    let frame = Frame::at_center(bounds);
    let up = frame.encode_enu(0.0, 0.0, 1.0);

    let mut mesh = TerrainMesh {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        indices: Vec::new(),
        normals: Vec::new(),
    };
    let mut anchor: Option<Coord> = None;
    for line in lines {
        anchor = anchor.or_else(|| line.0.first().copied());
        for piece in trim_line(line, &disks) {
            band(&piece, half_m, &frame, up, bounds, &mut mesh, &mut |lon, lat| {
                let h =
                    road::surface_height(profile, false, sampler, z, solved.z_ref, bounds, lon, lat);
                ((h - priors::SURFACE_SINK_M) * 1000.0).round() as i32
            });
        }
    }
    if mesh.indices.is_empty() {
        return None;
    }
    Some(EncoderFeature {
        id: f.id,
        geometry: Geometry::Point(Point(anchor?)),
        properties: f
            .properties
            .iter()
            .filter(|(k, _)| k == "class")
            .cloned()
            .collect(),
        elevation: None,
        z: None,
        mesh: Some(mesh),
        synth: Synth::None,
    })
}

/// Cuts the parts of a centerline inside any trim disk (a junction plate's
/// footprint), returning the pieces that remain — the whole line when no disk
/// touches it. Handles both a line *ending* at a junction and one passing
/// through it (a through leg). Pieces shorter than [`MIN_PIECE_M`] are
/// dropped; the plate covers them. Shared with the marking generator, whose
/// painted lines stop at the same plates.
pub(crate) fn trim_line(line: &LineString, disks: &[(Coord, f64)]) -> Vec<LineString> {
    let pts = &line.0;
    if pts.len() < 2 {
        return Vec::new();
    }
    // Cumulative arclength in metres (local equirectangular scale).
    let cosk = pts[0].y.to_radians().cos();
    let en = |from: Coord, to: Coord| {
        ((to.x - from.x) * M_PER_DEG_LON_EQUATOR * cosk, (to.y - from.y) * M_PER_DEG_LAT)
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

    // The arc intervals each disk covers: per segment, solve |f + t·d| = r.
    let mut cut: Vec<(f64, f64)> = Vec::new();
    for &(c, r) in disks {
        if r <= 0.0 {
            continue;
        }
        for (i, w) in pts.windows(2).enumerate() {
            let f = en(c, w[0]);
            let d = en(w[0], w[1]);
            let a2 = d.0 * d.0 + d.1 * d.1;
            if a2 < 1e-12 {
                continue;
            }
            let b = f.0 * d.0 + f.1 * d.1;
            let cc = f.0 * f.0 + f.1 * f.1 - r * r;
            let disc = b * b - a2 * cc;
            if disc <= 0.0 {
                continue;
            }
            let sq = disc.sqrt();
            let (t0, t1) = (((-b - sq) / a2).max(0.0), ((-b + sq) / a2).min(1.0));
            if t1 > t0 {
                let len = arc[i + 1] - arc[i];
                cut.push((arc[i] + t0 * len, arc[i] + t1 * len));
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
        if a - s >= MIN_PIECE_M {
            keep.push((s, a));
        }
        s = s.max(b);
    }
    if total - s >= MIN_PIECE_M {
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

/// Appends one line's band to the mesh: a vertex pair (left, right) per
/// usable centerline vertex, offset along the mitered perpendicular, and two
/// up-facing triangles per section. Heights come from `elev_mm` at the offset
/// position, so the band drapes across the road as well as along it.
fn band(
    line: &LineString,
    half_m: f64,
    frame: &Frame,
    up: (i8, i8),
    bounds: &Bounds,
    mesh: &mut TerrainMesh,
    elev_mm: &mut dyn FnMut(f64, f64) -> i32,
) {
    let pts = &line.0;
    if pts.len() < 2 {
        return;
    }
    let m_lon = frame.m_per_deg_lon;
    // Unit ENU direction of the chord a → b, or `None` for a degenerate one.
    let dir = |a: Coord, b: Coord| -> Option<(f64, f64)> {
        let (de, dn) = ((b.x - a.x) * m_lon, (b.y - a.y) * M_PER_DEG_LAT);
        let len = (de * de + dn * dn).sqrt();
        (len > 1e-9).then(|| (de / len, dn / len))
    };

    let base = mesh.x.len() as u32;
    let mut sections = 0u32;
    for i in 0..pts.len() {
        let before = (i > 0).then(|| dir(pts[i - 1], pts[i])).flatten();
        let after = (i + 1 < pts.len()).then(|| dir(pts[i], pts[i + 1])).flatten();
        // Averaged heading at the vertex (one-sided at the ends), with the
        // miter scale that keeps the band edges on the chords' offset lines:
        // |v0 + v1| / 2 = cos(θ/2), clamped so a hairpin cannot spike.
        let (e, n, scale) = match (before, after) {
            (Some((e0, n0)), Some((e1, n1))) => {
                let (se, sn) = (e0 + e1, n0 + n1);
                let len = (se * se + sn * sn).sqrt();
                if len < 1e-9 {
                    continue; // a fold-back vertex has no perpendicular
                }
                (se / len, sn / len, (1.0 / (len * 0.5).min(1.0)).min(MITER_MAX))
            }
            (Some(d), None) | (None, Some(d)) => (d.0, d.1, 1.0),
            (None, None) => continue,
        };
        let (pe, pn) = (-n, e); // left perpendicular
        let reach = half_m * scale;
        for side in [1.0f64, -1.0] {
            let c = Coord {
                x: pts[i].x + pe * reach * side / m_lon,
                y: pts[i].y + pn * reach * side / M_PER_DEG_LAT,
            };
            mesh.x.push(project::quantize_x(c.x, bounds));
            mesh.y.push(project::quantize_y(c.y, bounds));
            mesh.z.push(elev_mm(c.x, c.y));
            mesh.normals.push(up.0);
            mesh.normals.push(up.1);
        }
        sections += 1;
    }
    // Two up-facing (counter-clockwise from above) triangles per quad.
    for s in 1..sections {
        let (l0, r0) = (base + (s - 1) * 2, base + (s - 1) * 2 + 1);
        let (l1, r1) = (base + s * 2, base + s * 2 + 1);
        mesh.indices.extend_from_slice(&[l0, r0, r1, l0, r1, l1]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_mesh() -> TerrainMesh {
        TerrainMesh {
            x: Vec::new(),
            y: Vec::new(),
            z: Vec::new(),
            indices: Vec::new(),
            normals: Vec::new(),
        }
    }

    /// Dequantized metre distance between mesh vertices `a` and `b`.
    fn dist_m(mesh: &TerrainMesh, a: usize, b: usize, bounds: &Bounds, frame: &Frame) -> f64 {
        let p = |i: usize| {
            let lon = bounds.west + (mesh.x[i] as f64 - 16384.0) / 32768.0 * bounds.width();
            let lat = bounds.south + (mesh.y[i] as f64 - 16384.0) / 32768.0 * bounds.height();
            (lon, lat)
        };
        let ((x0, y0), (x1, y1)) = (p(a), p(b));
        let de = (x1 - x0) * frame.m_per_deg_lon;
        let dn = (y1 - y0) * M_PER_DEG_LAT;
        (de * de + dn * dn).sqrt()
    }

    #[test]
    fn straight_band_offsets_to_the_half_width() {
        let b = Bounds::of_tile(15, 17000, 11600);
        let frame = Frame::at_center(&b);
        let cy = b.south + 0.5 * b.height();
        let line = LineString(vec![
            Coord { x: b.west + 0.3 * b.width(), y: cy },
            Coord { x: b.west + 0.7 * b.width(), y: cy },
        ]);
        let mut mesh = empty_mesh();
        band(&line, 3.75, &frame, (0, 0), &b, &mut mesh, &mut |_, _| 1000);
        // Two sections of (left, right) → 4 vertices, 2 triangles.
        assert_eq!(mesh.x.len(), 4);
        assert_eq!(mesh.indices.len(), 6);
        assert!(mesh.z.iter().all(|&z| z == 1000));
        // The pair straddles the centerline at the full band width.
        let w = dist_m(&mesh, 0, 1, &b, &frame);
        assert!((w - 7.5).abs() < 0.1, "band width {w} != 7.5");
    }

    #[test]
    fn right_angle_miter_is_clamped() {
        let b = Bounds::of_tile(15, 17000, 11600);
        let frame = Frame::at_center(&b);
        let c = Coord { x: b.west + 0.5 * b.width(), y: b.south + 0.5 * b.height() };
        let line = LineString(vec![
            Coord { x: c.x - 0.2 * b.width(), y: c.y },
            c,
            Coord { x: c.x, y: c.y + 0.2 * b.height() },
        ]);
        let mut mesh = empty_mesh();
        band(&line, 3.0, &frame, (0, 0), &b, &mut mesh, &mut |_, _| 0);
        assert_eq!(mesh.x.len(), 6);
        assert_eq!(mesh.indices.len(), 12);
        // The corner pair is mitered (wider than the body) but clamped.
        let body = dist_m(&mesh, 0, 1, &b, &frame);
        let corner = dist_m(&mesh, 2, 3, &b, &frame);
        assert!(corner > body * 1.2, "corner {corner} not mitered vs body {body}");
        assert!(corner <= body * MITER_MAX + 0.1, "corner {corner} beyond the clamp");
    }

    #[test]
    fn trim_splits_a_through_line_at_the_disk() {
        // A 200 m west→east line through a 10 m trim disk at its middle:
        // two pieces, each ending one radius short of the centre.
        let cy: f64 = 46.0;
        let m_lon = M_PER_DEG_LON_EQUATOR * cy.to_radians().cos();
        let c = Coord { x: 7.0, y: cy };
        let line = LineString(vec![
            Coord { x: c.x - 100.0 / m_lon, y: cy },
            Coord { x: c.x + 100.0 / m_lon, y: cy },
        ]);
        let pieces = trim_line(&line, &[(c, 10.0)]);
        assert_eq!(pieces.len(), 2, "the disk splits the line");
        for p in &pieces {
            for v in &p.0 {
                let d = ((v.x - c.x) * m_lon).abs();
                assert!(d > 9.9, "piece vertex {d:.2} m inside the trim radius");
            }
        }
        // An end-of-line junction shortens rather than splits.
        let end = trim_line(&line, &[(line.0[1], 10.0)]);
        assert_eq!(end.len(), 1);
        let last = end[0].0.last().expect("non-empty");
        assert!(((last.x - line.0[1].x) * m_lon).abs() > 9.9);
        // No disks → the whole line; a disk swallowing it → nothing.
        assert_eq!(trim_line(&line, &[]).len(), 1);
        assert!(trim_line(&line, &[(c, 150.0)]).is_empty());
    }

    #[test]
    fn degenerate_lines_produce_nothing() {
        let b = Bounds::of_tile(15, 17000, 11600);
        let frame = Frame::at_center(&b);
        let mut mesh = empty_mesh();
        let p = Coord { x: b.west + 0.5 * b.width(), y: b.south + 0.5 * b.height() };
        band(&LineString(vec![p]), 3.0, &frame, (0, 0), &b, &mut mesh, &mut |_, _| 0);
        band(&LineString(vec![p, p]), 3.0, &frame, (0, 0), &b, &mut mesh, &mut |_, _| 0);
        assert!(mesh.indices.is_empty());
    }
}
