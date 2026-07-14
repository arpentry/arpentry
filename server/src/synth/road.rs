//! Draped-road generator: bakes per-vertex elevation onto a road centerline.
//!
//! A road with no corridor profile sits on the rendered ground
//! ([`GroundSampler::surface`], the same lattice the terrain mesh
//! triangulates, consistent across tiles). A road *with* a solved profile
//! renders at
//!
//! ```text
//! surface(z) + max(road_m − surface(z_ref), 0)
//! ```
//!
//! — its solved height, expressed relative to the engineered rendered ground
//! at the reference zoom. Wherever the terrain lattice captured the
//! corridor's earthwork the correction is ~zero, so at `z_ref` the road is
//! exactly `road_m`: it meets its structures with no step (invariant 2) and
//! lies on the drawn embankment rather than floating twice its height above
//! it. At coarser zooms the per-zoom surface is the datum, so the road still
//! hugs that zoom's rendered terrain (invariant 4) with the same
//! zoom-independent engineered offset.
//!
//! The `max(…, 0)` clamps the correction to fills: the lattice is far coarser
//! than a cutting's footprint (a z14 cell spans ~150 m against a ~18 m
//! earthwork reach), so on bumpy relief a grade-limited cut often fails to
//! pull any lattice vertex down and the drawn ground keeps standing metres
//! above the solved grade. Paint baked *below* the drawn ground is beyond the
//! viewer's depth bias at close range — the road visibly breaks against every
//! such bump. Clamped, the paint rides the drawn ground through the missed
//! cutting (and exactly on it wherever the earthwork did capture the mesh),
//! staying visible at any viewing distance.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::ground::sampler::GroundSampler;
use crate::project::{self, Bounds};
use crate::solve::{self, Profile};
use crate::terrain::TERRAIN_GRID;
use crate::tile_build::EncoderFeature;

/// Target sub-segment length when densifying a ground road, in quantized tile
/// units, so the baked centerline tracks the terrain mesh's `~tile/16` cells.
const ROAD_SEGMENT_Q: f64 = 768.0;

/// Denser target for a corridor road, whose vertices snap onto the smoothed
/// sweep line: half the plain spacing keeps the snapped chords' sagitta
/// invisible on the curves an engineered road takes.
const CORRIDOR_SEGMENT_Q: f64 = 384.0;

/// How far a corridor road's vertex may be pulled sideways onto the smoothed
/// sweep line, in metres — the raw line's plausible digitising error. A
/// vertex farther out is left alone (a corridor-end overhang), so snapping
/// can never fold the line.
const PAINT_SNAP_MAX_M: f64 = 6.0;

/// Cap on densified vertices per linestring — a runaway guard.
const MAX_VERTS: usize = 4096;

/// Bakes a road's per-vertex elevation onto the feature, densifying the
/// (clipped) centerline so it follows the relief and writing the heights into
/// `f.z`. A no-op for non-line geometry. `deck` marks paint re-emitted over a
/// *structure* span — a bridge deck or a tunnel bore: it always rides the
/// solved deck ramp directly ([`Profile::deck_height_at`], the same heights
/// the deck and bore sweeps build their solids from), so the stroke lies on
/// the deck top of a bridge and on the bore's road surface of a tunnel at
/// every zoom, instead of following the per-zoom drape correction (which would
/// step off the structure wherever the coarse lattice disagrees with the
/// reference). Where a bore runs buried the ramp dips under the hill, so the
/// ribbon sinks with the mesh and the terrain occludes it rather than draping
/// the hillside above the buried span.
pub fn bake(
    f: &mut EncoderFeature,
    profile: Option<&Profile>,
    deck: bool,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
) {
    let mut height =
        |lon: f64, lat: f64| surface_height(profile, deck, sampler, z, z_ref, bounds, lon, lat);
    // A corridor road's paint follows the corridor's *smoothed* sweep line —
    // the same curve its bridges and tunnels are swept along — instead of
    // tracing the raw line's digitising wiggle beside them.
    let mut snap = |c: Coord| -> Coord {
        profile.and_then(|p| p.smooth_at(c.x, c.y, PAINT_SNAP_MAX_M)).unwrap_or(c)
    };
    let seg_q = if profile.is_some() { CORRIDOR_SEGMENT_Q } else { ROAD_SEGMENT_Q };
    if let Some((geom, zs)) =
        densify_with_surface(&f.geometry, bounds, seg_q, &mut snap, &mut height)
    {
        f.geometry = geom;
        f.z = Some(zs);
    }
}

/// The road-surface height at a point — the one function every road
/// representation reads (GENERATION.md invariant 2, ROADS.md invariant 5):
/// the paint stroke bakes its vertices on it and the surface band
/// (`synth::surface`) drapes its edges on the same answer, so paint and
/// asphalt can never drift apart.
pub(crate) fn surface_height(
    profile: Option<&Profile>,
    deck: bool,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
    lon: f64,
    lat: f64,
) -> f64 {
    match profile {
        // Structure paint rides the *deck ramp* at every zoom — the same
        // `deck_m` heights the deck/bore sweep builds its solid from (a
        // bridge's deck top, a tunnel bore's road surface) — not the road
        // profile, which the ramp fit (and the clearance clamps) diverge from
        // mid-span; paint baked at road height sinks inside the solid wherever
        // the fitted ramp rises above it.
        Some(p) if deck => p.deck_height_at(lon, lat),
        // At the reference zoom the datum and the reference surface are the
        // same sample, so this is max(road_m, surface): the solved height,
        // never below the drawn ground (see the module doc on the clamp).
        Some(p) if z == z_ref => {
            let surface = sampler.surface(bounds, lon, lat, z);
            p.height_at(lon, lat).max(surface)
        }
        Some(p) => {
            let surface = sampler.surface(bounds, lon, lat, z);
            let ref_bounds = solve::tile_containing(z_ref, lon, lat);
            let lift = p.height_at(lon, lat) - sampler.surface(&ref_bounds, lon, lat, z_ref);
            surface + lift.max(0.0)
        }
        // An unclaimed street rides its bed at the reference zoom: the ground
        // stage benches the terrain under every drivable road (flat across,
        // natural grade along — D3), but the rendered lattice is far coarser
        // than a street's footprint, so the drape takes the exact bed height
        // instead of the tilted in-cell interpolation. Coarser zooms keep the
        // per-zoom surface — the bench pulls their corners toward the bed
        // wherever the lattice can see it.
        None if z == z_ref => match sampler.bed_target(lon, lat) {
            Some(bed) => bed,
            None => sampler.surface(bounds, lon, lat, z),
        },
        None => sampler.surface(bounds, lon, lat, z),
    }
}

/// Densifies a (multi)linestring and samples the height at every vertex,
/// returning the new geometry and the matching `z` array (flattened in
/// `line_geometry` vertex order), or `None` for non-line / empty input.
fn densify_with_surface(
    g: &Geometry,
    bounds: &Bounds,
    seg_q: f64,
    snap: &mut dyn FnMut(Coord) -> Coord,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(Geometry, Vec<i32>)> {
    match g {
        Geometry::LineString(ls) => {
            let (xy, zs) = densify_road_line(ls, bounds, seg_q, snap, height);
            (xy.len() >= 2).then_some((Geometry::LineString(LineString(xy)), zs))
        }
        Geometry::MultiLineString(mls) => {
            let mut parts = Vec::new();
            let mut zs = Vec::new();
            for ls in &mls.0 {
                let (xy, z) = densify_road_line(ls, bounds, seg_q, snap, height);
                if xy.len() >= 2 {
                    parts.push(LineString(xy));
                    zs.extend(z);
                }
            }
            (!parts.is_empty()).then_some((Geometry::MultiLineString(MultiLineString(parts)), zs))
        }
        _ => None,
    }
}

/// Densifies one linestring to ~`seg_q` quantized spacing, snaps it onto the
/// smoothed sweep line, inserts a vertex at every rendered-terrain lattice
/// crossing of the *snapped* chords, and samples the height at each vertex.
///
/// The lattice crossings are what keep a draped road *on* the drawn ground:
/// the terrain mesh is planar inside each lattice triangle, so a chord whose
/// endpoints both lie on the surface stays on it only while it remains in one
/// triangle. A chord crossing a cell edge or diagonal under a convex break (a
/// ridge) sags below the drawn ground mid-segment, and at grazing view angles
/// even a small dip puts the stroke beyond the depth bias' reach — the road
/// visibly sinks into the hillside. With a vertex on every crossing, every
/// chord lies inside one triangle and the drape is chord-exact.
///
/// Two passes, and the order matters: snapping the paint onto the corridor's
/// smoothed sweep line moves a vertex up to `PAINT_SNAP_MAX_M` sideways, so a
/// crossing found on the *raw* chord no longer lands on its lattice line once
/// snapped — the emitted chord then spans a triangle break and the drape sags.
/// So pass 1 snaps the densified anchors first, and pass 2 finds the crossings
/// on the snapped chords that are actually emitted and leaves them unsnapped.
/// The anchors carry the sweep-line shape; the crossings hold the drape.
fn densify_road_line(
    line: &LineString,
    bounds: &Bounds,
    seg_q: f64,
    snap: &mut dyn FnMut(Coord) -> Coord,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> (Vec<Coord>, Vec<i32>) {
    let pts = &line.0;
    if pts.is_empty() {
        return (Vec::new(), Vec::new());
    }

    // Pass 1: densify the raw line to the comb spacing and snap every vertex
    // (original + inserted) onto the smoothed sweep line. These anchors are the
    // curve the paint should ride; they are not re-snapped afterward.
    let mut anchors = Vec::new();
    anchors.push(snap(pts[0]));
    'outer: for w in pts.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let qlen = quant_len(p0, p1, bounds);
        let n = ((qlen / seg_q).ceil() as usize).clamp(1, MAX_VERTS);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            anchors.push(snap(c));
            if anchors.len() >= MAX_VERTS {
                break 'outer;
            }
        }
    }

    // Pass 2: walk the snapped anchor chords, inserting a vertex at every
    // lattice crossing of the emitted chord (so it lands exactly on the lattice
    // line and stays there), and sample the height at each final vertex.
    let mut xy = Vec::new();
    let mut zs = Vec::new();
    let mut push = |c: Coord, xy: &mut Vec<Coord>, zs: &mut Vec<i32>| {
        zs.push(project::quantize_z(height(c.x, c.y)));
        xy.push(c);
    };
    push(anchors[0], &mut xy, &mut zs);
    let mut ts = Vec::new();
    for w in anchors.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        ts.clear();
        lattice_crossings(p0, p1, bounds, &mut ts);
        ts.sort_by(f64::total_cmp);
        ts.push(1.0);
        let mut prev = 0.0;
        for &t in &ts {
            if t - prev < 1e-9 {
                continue;
            }
            prev = t;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            push(c, &mut xy, &mut zs);
            if xy.len() >= MAX_VERTS {
                return (xy, zs);
            }
        }
    }
    (xy, zs)
}

/// Appends the parametric positions in (0, 1) where the segment `p0 → p1`
/// crosses a line of the tile's rendered-terrain lattice: a vertical cell
/// edge, a horizontal cell edge, or the cells' shared SW–NE diagonal (the
/// split [`crate::terrain::surface_height`] and the drawn mesh both use).
fn lattice_crossings(p0: Coord, p1: Coord, bounds: &Bounds, out: &mut Vec<f64>) {
    let grid = TERRAIN_GRID as f64;
    let (cw, ch) = (bounds.width() / grid, bounds.height() / grid);
    let g0x = (p0.x - bounds.west) / cw;
    let g1x = (p1.x - bounds.west) / cw;
    let g0y = (p0.y - bounds.south) / ch;
    let g1y = (p1.y - bounds.south) / ch;
    axis_crossings(g0x, g1x, out);
    axis_crossings(g0y, g1y, out);
    axis_crossings(g0x - g0y, g1x - g1y, out);
}

/// Appends the ts in (0, 1) where the linear value `g(t) = g0 + (g1 − g0) t`
/// crosses an integer. Endpoints already on an integer produce no crossing —
/// they are samples themselves.
fn axis_crossings(g0: f64, g1: f64, out: &mut Vec<f64>) {
    let d = g1 - g0;
    if d.abs() < 1e-12 {
        return;
    }
    let (lo, hi) = if d > 0.0 { (g0, g1) } else { (g1, g0) };
    let mut k = lo.floor() + 1.0;
    while k < hi {
        out.push((k - g0) / d);
        k += 1.0;
    }
}

/// Distance between two lon/lat points in quantized tile units.
fn quant_len(a: Coord, b: Coord, bounds: &Bounds) -> f64 {
    let dx = project::quantize_x(b.x, bounds) as f64 - project::quantize_x(a.x, bounds) as f64;
    let dy = project::quantize_y(b.y, bounds) as f64 - project::quantize_y(a.y, bounds) as f64;
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drape_samples_land_on_every_lattice_crossing() {
        // A west→east line off the lattice rows: densification must place a
        // vertex exactly on every cell edge and diagonal it crosses, so no
        // chord spans a triangle break and the drape stays on the drawn
        // ground (the mesh is planar only inside each triangle).
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = b.south + 0.53 * b.height();
        let (x0, x1) = (b.west + 0.30 * b.width(), b.west + 0.70 * b.width());
        let line = LineString(vec![Coord { x: x0, y: cy }, Coord { x: x1, y: cy }]);
        let (xy, zs) =
            densify_road_line(&line, &b, ROAD_SEGMENT_Q, &mut |c| c, &mut |_, _| 0.0);
        assert_eq!(xy.len(), zs.len());

        let cw = b.width() / TERRAIN_GRID as f64;
        for k in 0..=TERRAIN_GRID {
            let edge = b.west + k as f64 * cw;
            if edge > x0 + 1e-12 && edge < x1 - 1e-12 {
                assert!(
                    xy.iter().any(|c| (c.x - edge).abs() < 1e-9 * b.width()),
                    "expected a sample on vertical cell edge {k}"
                );
            }
        }
        // Diagonal crossings: gx − gy passes 6 integers over this span.
        let ch = b.height() / TERRAIN_GRID as f64;
        let on_diagonal = xy
            .iter()
            .filter(|c| {
                let g = (c.x - b.west) / cw - (c.y - b.south) / ch;
                (g - g.round()).abs() < 1e-7
            })
            .count();
        assert!(on_diagonal >= 6, "expected samples on cell diagonals, got {on_diagonal}");
    }

    #[test]
    fn snapping_does_not_break_chord_exactness() {
        // A corridor road's paint is snapped sideways onto the smoothed sweep
        // line before its height is sampled. Crossings must be found on the
        // *snapped* chords, so no emitted segment spans a lattice cell edge or
        // diagonal — otherwise the drape sags into a triangle break. Model the
        // snap as a lattice-scale sideways nudge and assert every emitted chord
        // stays inside one triangle (no lattice line strictly interior to it).
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = b.south + 0.53 * b.height();
        let (x0, x1) = (b.west + 0.30 * b.width(), b.west + 0.70 * b.width());
        let line = LineString(vec![Coord { x: x0, y: cy }, Coord { x: x1, y: cy }]);
        // A constant north nudge of ~0.3 cell: enough to shift a raw-chord
        // diagonal crossing well off its line if snapping ran after insertion.
        let dy = 0.02 * b.height();
        let mut snap = |c: Coord| Coord { x: c.x, y: c.y + dy };
        let (xy, _) =
            densify_road_line(&line, &b, ROAD_SEGMENT_Q, &mut snap, &mut |_, _| 0.0);

        let cw = b.width() / TERRAIN_GRID as f64;
        let ch = b.height() / TERRAIN_GRID as f64;
        for w in xy.windows(2) {
            let g = |c: Coord| ((c.x - b.west) / cw, (c.y - b.south) / ch);
            let (ax, ay) = g(w[0]);
            let (bx, by) = g(w[1]);
            for (u0, u1) in [(ax, bx), (ay, by), (ax - ay, bx - by)] {
                let (lo, hi) = (u0.min(u1), u0.max(u1));
                let mut k = lo.floor() + 1.0;
                while k < hi {
                    assert!(
                        k <= lo + 1e-6 || k >= hi - 1e-6,
                        "chord {:?}->{:?} spans lattice line {k} without a vertex",
                        w[0],
                        w[1]
                    );
                    k += 1.0;
                }
            }
        }
    }
}
