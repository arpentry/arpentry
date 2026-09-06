//! The plan-space network, drawn as lines — the network view.
//!
//! At the surface zooms a tile carries the *result* of the network synthesis
//! and nothing else: one unioned mesh per surface family, with the
//! cartographic stroke of every way that fed it deleted
//! (`pipeline::stamp_synth`), because a line painted over its own asphalt is a
//! second coat of the same paint. That is right for a map and useless for
//! anyone asking why the asphalt is *there*. The two things that question
//! needs — the centreline the synthesis consumed, and the edge the model says
//! it paved — are exactly the two the tile does not carry, so a disagreement
//! between plan and surface is invisible in the only place it can be seen.
//!
//! `--plan-lines` puts both back as their own classes, over the surface they
//! produced. It draws the [`SourceSeg`] population — the one input the drawn
//! surface actually has ([`synth::carriageway`]) — rather than the Overture
//! geometry, and that is the point of it: a sidewalk has no source line at
//! all (it is derived from the street it rides), and a corridor's line is
//! several joined segments. What a reader compares against the mesh is what
//! the mesher was given.
//!
//! Three classes per surface family:
//!
//! - `plan_axis_*` — the segment's centreline, at its solved surface height.
//!   Where the surface is, according to the model.
//! - `plan_edge_*` — the same segment offset to `sect_a`/`sect_b`, one line
//!   per side. What one segment alone claims, and **deliberately not a
//!   continuous curve**: consecutive segments' offsets step at every vertex
//!   and say nothing about the join, which is where a run's extent is
//!   actually decided. Read it as a cross-section sampled along the way, not
//!   as an outline.
//! - `plan_bound_*` — the boundary of the polygon the union actually
//!   consumes: the whole run stroked by `pavement::buffer_run`, joins,
//!   mitres, abutment trim and all, with holes as their own contours.
//!   **This is the model's real statement of where its surface ends**, and it
//!   is the same call the bake makes rather than a second derivation of it,
//!   so the gap between this line and the drawn rim beside it is the union's
//!   doing and nothing else's.
//!
//! The last two disagree wherever the join matters, and that disagreement is
//! the point: `plan_edge_*` is the cross-section the model wrote down,
//! `plan_bound_*` is the region that cross-section became.
//!
//! Nothing here infers: every number is read off the model. The module is a
//! projection of `CarriagewayModel` into line features, and it runs only when
//! the run asks for it — a `--plan-lines` archive is a debugging archive, and
//! the tiler says so in its run summary.
//!
//! [`synth::carriageway`]: crate::synth::carriageway

use geo_types::{Coord, Geometry, LineString};

use crate::layers;
use crate::priors::Surface;
use crate::project::Bounds;
use crate::scene::DEG_M;
use crate::synth::carriageway::{CarriagewayModel, SourceSeg};
use crate::synth::pavement;
use crate::tile_build::EncoderFeature;
use crate::value::Value;


/// The prefix every class this module emits carries.
///
/// The one place the name is written. `verify::scene` drops these classes on
/// the way in, so the scorecard never scores a plan line; both sides read this
/// constant so the drop cannot fall behind a rename.
pub const CLASS_PREFIX: &str = "plan_";

/// Whether `class` names a plan-space line rather than a drawn one.
pub fn is_plan_class(class: &str) -> bool {
    class.starts_with(CLASS_PREFIX)
}

/// Emits this tile's share of the plan-space network as line features.
///
/// A no-op below the surface zoom, where the source strokes are still drawn
/// and the plan *is* what the tile carries.
pub fn lines(
    buckets: &mut [Vec<EncoderFeature>],
    junctions: &CarriagewayModel,
    z: u8,
    bounds: &Bounds,
) {
    if z < crate::priors::ROAD_SURFACE_MIN_ZOOM {
        return;
    }
    // The tile proper, not the buffer the encoder could hold. `add_road_surface`
    // clips its regions to exactly this ("every tile emits exactly its own
    // piece and the pieces meet at a shared, snapped seam"), and a plan line
    // that reached further would be a line with no surface under it —
    // indistinguishable, to a reader, from the defect this view exists to
    // find. Clipping is not optional either way: `project::quantize` *clamps*,
    // so an endpoint outside the tile does not vanish, it lands on the corner
    // and draws a line that was never in the model.
    let (w, s, e, n) =
        (bounds.west, bounds.south, bounds.west + bounds.width(), bounds.south + bounds.height());
    let mut near = Vec::new();
    junctions.sources_near((w, s, e, n), &mut near);
    let out = &mut buckets[layers::TRANSPORTATION as usize];
    for i in near {
        // The ring's masked bands are in the source list and are not drawn
        // (`CarriagewayModel::drawn`); drawing their plan would put a second
        // pavement beside every ring, in a view whose whole job is to say
        // where the one pavement is.
        if !junctions.drawn(i) {
            continue;
        }
        let seg = junctions.source(i);
        let family = family(seg.surface);
        // The surface stands `rise_m` above the profile the segment carries —
        // a sidewalk's kerb (`synth::height`, `on_ground(..) + rise_m`). The
        // plan line is drawn on the surface it describes, not under it.
        let (ha, hb) = (seg.height_a + seg.rise_m, seg.height_b + seg.rise_m);
        push(out, (i, 0), seg.a, seg.b, (ha, hb), format!("{CLASS_PREFIX}axis_{family}"), seg.level, &(w, s, e, n));
        for side in 0..2usize {
            let n_hat = normal(seg, side);
            let (da, db) = (seg.sect_a.on(side), seg.sect_b.on(side));
            push(
                out,
                (i, 1 + side as u8),
                offset(seg.a, n_hat, da, seg.cos_lat),
                offset(seg.b, n_hat, db, seg.cos_lat),
                (ha, hb),
                format!("{CLASS_PREFIX}edge_{family}"),
                seg.level,
                &(w, s, e, n),
            );
        }
    }
    bounds_lines(out, junctions, bounds, &(w, s, e, n));
}

/// Draws each run's buffered boundary — the polygon the union consumes.
///
/// **Runs are grouped over a padded query, not over the tile.** A run chains
/// while consecutive sources meet end to end (`pavement::runs`), so grouping
/// over the tile alone would break every run at the tile edge and draw the cut
/// as if the model had put it there. The pad is the bake's own, so a run is
/// grouped here exactly as the chunk grouped it; the contours are then clipped
/// to the tile like every other line in this module.
///
/// The frame is the *chunk's*, not the tile's ([`pavement::chunk_frame_for`]),
/// because `i_overlay` snaps to a fixed grid anchored on the frame origin: any
/// other origin would put the boundary a fraction of a millimetre off the
/// vertices the bake actually produced, which is exactly the kind of
/// discrepancy this view exists to rule out rather than introduce.
fn bounds_lines(
    out: &mut Vec<EncoderFeature>,
    junctions: &CarriagewayModel,
    bounds: &Bounds,
    box_: &(f64, f64, f64, f64),
) {
    let frame = pavement::chunk_frame_for(bounds);
    let pad = crate::priors::PAVE_PAD_M / DEG_M;
    let mut near = Vec::new();
    junctions.sources_near(
        (box_.0 - pad, box_.1 - pad, box_.2 + pad, box_.3 + pad),
        &mut near,
    );
    near.retain(|&i| junctions.drawn(i));
    // `pavement::runs` chains on the shared endpoint but walks the list in
    // order, and the bake hands it a sorted one. A grid query does not.
    near.sort_unstable();
    for (r, run) in pavement::runs_of(junctions, &near).iter().enumerate() {
        let line: Vec<[f64; 2]> = run.line().iter().map(|&c| frame.to_m(c)).collect();
        let family = family(run.surface());
        let class = format!("{CLASS_PREFIX}bound_{family}");
        for (c, contour) in pavement::buffer_run_for_plan(run, &line, &frame).iter().flatten().enumerate() {
            let ring: Vec<Coord<f64>> = contour.iter().map(|&pt| frame.to_deg(pt)).collect();
            push_ring(out, (r as u32, c as u8), &ring, run, &class, box_);
        }
    }
}

/// Clips a closed contour to the tile and pushes what survives, as one feature
/// per surviving stretch. A ring is drawn, not filled, so a stretch is a line;
/// splitting at the tile edge is what keeps `project::quantize`'s clamp from
/// inventing a chord across the tile.
fn push_ring(
    out: &mut Vec<EncoderFeature>,
    id: (u32, u8),
    ring: &[Coord<f64>],
    run: &pavement::Run,
    class: &str,
    box_: &(f64, f64, f64, f64),
) {
    if ring.len() < 2 {
        return;
    }
    let mut piece: Vec<(Coord<f64>, f64)> = Vec::new();
    let mut part = 0u32;
    let mut flush = |piece: &mut Vec<(Coord<f64>, f64)>, part: &mut u32| {
        if piece.len() >= 2 {
            out.push(EncoderFeature {
                id: 0x0b_1b_00_00_00_00
                    | u64::from(id.0) << 12
                    | u64::from(id.1) << 8
                    | u64::from(*part & 0xff),
                geometry: Geometry::LineString(LineString(
                    piece.iter().map(|&(c, _)| c).collect(),
                )),
                properties: vec![
                    ("class".to_string(), Value::String(class.to_string())),
                    ("level".to_string(), Value::Int(run.level())),
                ],
                elevation: None,
                z: Some(piece.iter().map(|&(_, h)| (h * 1000.0).round() as i32).collect()),
                mesh: None,
                synth: crate::synth::Synth::None,
            });
            *part += 1;
        }
        piece.clear();
    };
    for k in 0..ring.len() {
        let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
        let Some((t0, t1)) = clip(a, b, box_) else {
            flush(&mut piece, &mut part);
            continue;
        };
        let at = |t: f64| Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
        let (p0, p1) = (at(t0), at(t1));
        // A stretch continues only while the clip kept both ends: a segment
        // entering the box starts a new one.
        if t0 > 0.0 {
            flush(&mut piece, &mut part);
        }
        if piece.is_empty() {
            piece.push((p0, run.height_at(p0)));
        }
        piece.push((p1, run.height_at(p1)));
        if t1 < 1.0 {
            flush(&mut piece, &mut part);
        }
    }
    flush(&mut piece, &mut part);
}

/// The class suffix for a surface family. `Surface::None` lays no band, so a
/// segment carrying it has no drawn surface to compare against and reads as a
/// plain road axis.
fn family(surface: Surface) -> &'static str {
    match surface {
        Surface::Ballast => "rail",
        Surface::Walkway => "walk",
        Surface::Path => "path",
        Surface::Asphalt | Surface::None => "road",
    }
}

/// The outward unit normal of `a`→`b` on `side` (0 left, 1 right), in local
/// east/north metres — the same convention `Section::on` and
/// `verify::model::street` index a side by.
fn normal(seg: &SourceSeg, side: usize) -> (f64, f64) {
    let m_lon = DEG_M * seg.cos_lat;
    let (dx, dy) = ((seg.b.x - seg.a.x) * m_lon, (seg.b.y - seg.a.y) * DEG_M);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return (0.0, 0.0);
    }
    let sgn = if side == 0 { 1.0 } else { -1.0 };
    (-sgn * dy / len, sgn * dx / len)
}

/// `p` moved `d` metres along a local east/north unit vector.
fn offset(p: Coord<f64>, n: (f64, f64), d: f64, cos_lat: f64) -> Coord<f64> {
    Coord { x: p.x + n.0 * d / (DEG_M * cos_lat), y: p.y + n.1 * d / DEG_M }
}

/// Clips one 2-point line to the tile's representable box and pushes it, with
/// its two ends' heights interpolated to the clipped parameters. Drops a line
/// that misses the box entirely.
fn push(
    out: &mut Vec<EncoderFeature>,
    // `(source index, 0 axis | 1 left edge | 2 right edge)`.
    id: (u32, u8),
    a: Coord<f64>,
    b: Coord<f64>,
    (ha, hb): (f64, f64),
    class: String,
    level: i64,
    box_: &(f64, f64, f64, f64),
) {
    let Some((t0, t1)) = clip(a, b, box_) else { return };
    let at = |t: f64| Coord { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
    let h = |t: f64| ((ha + (hb - ha) * t) * 1000.0).round() as i32;
    out.push(EncoderFeature {
        // Distinct from every other synthesized id, and distinct per line, so
        // one segment's three lines are three features.
        id: 0x0b_1a_00_00_00_00 | u64::from(id.0) << 2 | u64::from(id.1),
        geometry: Geometry::LineString(LineString(vec![at(t0), at(t1)])),
        properties: vec![
            ("class".to_string(), Value::String(class)),
            ("level".to_string(), Value::Int(level)),
        ],
        elevation: None,
        z: Some(vec![h(t0), h(t1)]),
        mesh: None,
        synth: crate::synth::Synth::None,
    })
}

/// Liang–Barsky: the parameter range of `a`→`b` inside the box, or `None`.
fn clip(a: Coord<f64>, b: Coord<f64>, box_: &(f64, f64, f64, f64)) -> Option<(f64, f64)> {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (p, q) in [
        (-dx, a.x - box_.0),
        (dx, box_.2 - a.x),
        (-dy, a.y - box_.1),
        (dy, box_.3 - a.y),
    ] {
        if p == 0.0 {
            if q < 0.0 {
                return None; // parallel to this edge and outside it
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                t0 = t0.max(r);
            } else {
                t1 = t1.min(r);
            }
            if t0 > t1 {
                return None;
            }
        }
    }
    Some((t0, t1))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(x: f64, y: f64) -> Coord<f64> {
        Coord { x, y }
    }

    #[test]
    fn every_class_this_module_emits_reads_as_plan_space() {
        // `verify::scene` drops these on the way in, so no check can score a
        // plan line. The drop is a prefix test, and this is what holds it to
        // the emitter: if a class here stopped matching, `paint.buried` would
        // start charging plan edges as buried paint again (0.082 % -> 1.092 %
        // over the Montreux cut, its whole worst tail plan geometry).
        for surface in
            [Surface::Asphalt, Surface::Ballast, Surface::Walkway, Surface::Path, Surface::None]
        {
            let f = family(surface);
            assert!(is_plan_class(&format!("{CLASS_PREFIX}axis_{f}")), "axis of {f}");
            assert!(is_plan_class(&format!("{CLASS_PREFIX}edge_{f}")), "edge of {f}");
            assert!(is_plan_class(&format!("{CLASS_PREFIX}bound_{f}")), "bound of {f}");
        }
        // And it swallows nothing the map actually draws.
        for class in ["marking", "residential", "rail_line", "footway", "sidewalk", "crossing"] {
            assert!(!is_plan_class(class), "{class} is a drawn class");
        }
    }

    #[test]
    fn clips_to_the_box_and_keeps_what_is_inside() {
        let b = (0.0, 0.0, 1.0, 1.0);
        assert_eq!(clip(c(0.2, 0.5), c(0.8, 0.5), &b), Some((0.0, 1.0)));
        // Enters partway: half the segment is inside.
        assert_eq!(clip(c(-1.0, 0.5), c(1.0, 0.5), &b), Some((0.5, 1.0)));
        assert_eq!(clip(c(2.0, 0.5), c(3.0, 0.5), &b), None);
        // Parallel to an edge and outside it.
        assert_eq!(clip(c(-1.0, 2.0), c(1.0, 2.0), &b), None);
    }

    #[test]
    fn the_left_normal_points_left_of_the_direction_of_travel() {
        // A segment heading east: left is north.
        let seg = SourceSeg {
            a: c(6.0, 46.0),
            b: c(6.001, 46.0),
            cos_lat: 46.0f64.to_radians().cos(),
            half_m: 3.0,
            sect_a: crate::assemble::facades::Section::uniform(3.0),
            sect_b: crate::assemble::facades::Section::uniform(3.0),
            level: 0,
            layer: 0,
            cut_a: None,
            cut_b: None,
            height_a: 400.0,
            height_b: 400.0,
            corridor: 0,
            surface: Surface::Asphalt,
            rise_m: 0.0,
            arc0: 0.0,
        };
        let (nx, ny) = normal(&seg, 0);
        assert!(ny > 0.9 && nx.abs() < 1e-9, "left of east is north, got ({nx}, {ny})");
        let (_, ry) = normal(&seg, 1);
        assert!(ry < -0.9, "right of east is south");
        // Offsetting 3 m north moves ~2.7e-5 degrees of latitude.
        let p = offset(seg.a, (nx, ny), 3.0, seg.cos_lat);
        assert!((p.y - seg.a.y - 3.0 / DEG_M).abs() < 1e-12);
    }
}
