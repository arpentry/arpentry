//! Draped-road generator: bakes per-vertex elevation onto a road centerline.
//!
//! Two regimes, split at the reference zoom (docs/GROUND.md §4):
//!
//! - **At `z_ref`** the road reads the engineered ground exactly: inside a
//!   bench the exact roadbed target ([`GroundSampler::bed_target`] — the
//!   profile the earthworks were built from, which the breakline-constrained
//!   terrain mesh holds flat under the road), and on unbenched stretches the
//!   rendered surface itself ([`GroundSampler::surface`], within the
//!   earthwork threshold of the profile by construction). A cut renders as a
//!   cut: the terrain carves with the road, so no clamp over missed cuttings
//!   is needed.
//! - **At coarser zooms** the per-zoom surface is the datum and the road
//!   carries its zoom-independent engineered offset,
//!   `surface(z) + max(road_m − surface(z_ref), 0)`, clamped to fills: the
//!   coarse lattice cannot carry a bench, so the road hugs the terrain that
//!   *is* drawn (invariant 4) and never sinks below it.
//!
//! A road with no profile (a path, rail) sits on the rendered ground at
//! every zoom.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::ground::sampler::GroundSampler;
use crate::project::{self, Bounds};
use crate::solve::{self, Profile};
use crate::terrain;
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
    corridor: Option<crate::scene::CorridorId>,
    field: Option<&crate::synth::height::HeightField>,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
) {
    // At-grade paint reads the shared height field, so it lands on exactly the
    // asphalt beside it — including inside an intersection, where the field is
    // pinned to the height the solver made the legs share. Structure paint keeps
    // riding its own deck ramp: a deck is not part of the at-grade surface and
    // has no business being blended with it.
    let mut scratch: Vec<u32> = Vec::new();
    let mut height = |lon: f64, lat: f64| {
        match field {
            // Only geometry that is *part of* the paved surface reads the field,
            // and `synth::emit` decides which that is — a `None` here is a
            // footway, not an oversight. A deck is never blended: it is not part
            // of the at-grade surface and rides its own ramp.
            // The sheet is resolved per vertex, not per feature: paint must ride
            // the asphalt its own road belongs to, and a corridor that stacks
            // over itself belongs to two (`synth::sheets`).
            Some(f) if !deck && !f.is_empty() => {
                let layer =
                    corridor.map_or(0, |c| f.layer_at(c, lon, lat, &mut scratch));
                f.at(sampler, AT_GRADE_LEVEL, layer, z, z_ref, bounds, lon, lat, &mut scratch)
            }
            _ => surface_height(profile, deck, sampler, z, z_ref, bounds, lon, lat),
        }
    };
    // **Paint rides the line the surface under it was built from — and there is
    // now one of them.** A structure span's paint rides a deck or a bore, swept
    // along the corridor's *smoothed* sweep line (`Profile::deck_nodes` →
    // `smooth_point`); at-grade paint rides the unioned carriageway, which is
    // buffered around that same line (`synth::carriageway::carriageway_sources`).
    // Below the zooms that draw asphalt there is no surface to agree with — the
    // stroke *is* the road — and the smooth curve is what a stroke should trace
    // anyway, which is the cartographic reason this snap was written for in the
    // first place. So the carry is unconditional.
    //
    // It was not always. While the union was buffered around the corridor's raw
    // nodes, carrying at-grade paint onto the smooth line put it off the middle
    // of its own asphalt by the whole smoothing displacement — on a 6 m street,
    // a centre line sitting in a lane — so paint had to ride whichever of the
    // two curves lay under it, and step with them at every abutment. What fixed
    // that is not a paint change: the two surfaces are one curve now.
    let mut snap = |c: Coord| -> Coord {
        profile.and_then(|p| p.smooth_at(c.x, c.y, PAINT_SNAP_MAX_M)).unwrap_or(c)
    };
    let seg_q = if profile.is_some() { CORRIDOR_SEGMENT_Q } else { ROAD_SEGMENT_Q };
    let grid = terrain::grid_for(z, z_ref);
    if let Some((geom, zs)) =
        densify_with_surface(&f.geometry, bounds, grid, seg_q, &mut snap, &mut height)
    {
        f.geometry = geom;
        f.z = Some(zs);
    }
}

/// The `level` an at-grade road sits on. Grade spans carry level 0 by
/// construction (`solve::mod.rs:194`), and only at-grade roads read the field —
/// a structure rides its own deck ramp.
pub(crate) const AT_GRADE_LEVEL: i64 = 0;

/// The road-surface height at a point *for one corridor* — the per-corridor
/// answer, and the value [`crate::synth::height::HeightField`] blends.
///
/// Still the single definition of "where does this corridor's surface sit": the
/// field calls it rather than reproducing it, and structure paint calls it
/// directly for the deck ramp. What changed is that at-grade consumers now go
/// through the field, so several corridors meeting at one place get one answer
/// instead of each drawing its own (GENERATION.md I2, ROADS.md
/// invariant 5).
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
    // Structure paint rides the *deck ramp* at every zoom — the same `deck_m`
    // heights the deck/bore sweep builds its solid from (a bridge's deck top, a
    // tunnel bore's road surface) — not the road profile, which the ramp fit (and
    // the clearance clamps) diverge from mid-span; paint baked at road height
    // sinks inside the solid wherever the fitted ramp rises above it.
    if let (Some(p), true) = (profile, deck) {
        return p.deck_height_at(lon, lat);
    }
    let ground = ground_height(sampler, z, z_ref, bounds, lon, lat);
    on_ground(ground, profile, sampler, z, z_ref, lon, lat)
}

/// The engineered ground under a point — the half of the surface answer that does
/// **not** depend on which corridor is asking.
///
/// Split out because it is the expensive half: a `bed_target` query walks the
/// earthwork index (hundreds of thousands of edges on a real extract) and the
/// fallback evaluates the rendered terrain lattice. The height field blends
/// several corridors at one point, and calling the whole of [`surface_height`]
/// per corridor recomputed this identical value once per corridor. Now it is
/// computed once per point and each corridor only applies its own clamp.
pub(crate) fn ground_height(
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
    lon: f64,
    lat: f64,
) -> f64 {
    if z == z_ref {
        // The roadbed target inside a bench (the profile the earthworks were
        // built from, held flat by the breakline-constrained terrain), the
        // rendered surface on unbenched stretches.
        match sampler.bed_target(lon, lat) {
            Some(bed) => bed,
            None => sampler.surface(bounds, lon, lat, z),
        }
    } else {
        sampler.surface(bounds, lon, lat, z)
    }
}

/// One corridor's surface given the shared [`ground_height`] at the same point.
///
/// At the reference zoom a corridor road **is its own solved profile**, and at
/// coarser zooms it hugs the drawn ground plus the zoom-independent engineered
/// offset, clamped to fills.
///
/// **The clamp that used to stand here did two jobs, and the hole retired one.**
/// It was `ground.max(road_m)`, which reads as one rule and is two:
///
/// - *Never below the road's own profile.* Load-bearing, and kept. Where benches
///   overlap across stacked interchange corridors — a viaduct approach crossing
///   a lower road — the nearest bench may belong to the *other* road and fall
///   short of this one's fill; without this the road would drape onto its
///   neighbour's bed and step below its own bridge deck (ROADS.md invariant 5).
///   Taking the profile outright satisfies it exactly.
/// - *Never below the drawn ground.* This is what stopped the terrain poking up
///   through the asphalt, and since the detail terrain stops at the kerb
///   (docs/GROUND.md §3) there is no drawn ground under the carriageway left to
///   poke through. What the clamp still did was drag the surface up over every
///   DEM bump inside the paved region, so a road crossing a rough unbenched
///   flank came out folded — `slope.carriageway_face` at 40 % of samples over a
///   30 % grade, reaching 331 %, which is the "bumpy and unrealistic" a solved
///   profile exists to prevent. A road is exactly as smooth as its profile now.
///
/// So the clamp is dropped only where the hole is actually cut. Coarser rungs
/// draw ground under the asphalt and keep it.
pub(crate) fn on_ground(
    ground: f64,
    profile: Option<&Profile>,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    lon: f64,
    lat: f64,
) -> f64 {
    match profile {
        Some(p) if z == z_ref => {
            let road_m = p.height_at(lon, lat);
            if sampler.cuts_hole(z) {
                road_m
            } else {
                ground.max(road_m)
            }
        }
        Some(p) => {
            let ref_bounds = solve::tile_containing(z_ref, lon, lat);
            let lift = p.height_at(lon, lat) - sampler.surface(&ref_bounds, lon, lat, z_ref);
            ground + lift.max(0.0)
        }
        // An unclaimed road rides the engineered ground as before.
        None => ground,
    }
}

/// Densifies a (multi)linestring and samples the height at every vertex,
/// returning the new geometry and the matching `z` array (flattened in
/// `line_geometry` vertex order), or `None` for non-line / empty input.
fn densify_with_surface(
    g: &Geometry,
    bounds: &Bounds,
    grid: u32,
    seg_q: f64,
    snap: &mut dyn FnMut(Coord) -> Coord,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> Option<(Geometry, Vec<i32>)> {
    match g {
        Geometry::LineString(ls) => {
            let (xy, zs) = densify_road_line(ls, bounds, grid, seg_q, snap, height);
            (xy.len() >= 2).then_some((Geometry::LineString(LineString(xy)), zs))
        }
        Geometry::MultiLineString(mls) => {
            let mut parts = Vec::new();
            let mut zs = Vec::new();
            for ls in &mls.0 {
                let (xy, z) = densify_road_line(ls, bounds, grid, seg_q, snap, height);
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
    grid: u32,
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
        lattice_crossings(p0, p1, bounds, grid, &mut ts);
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
fn lattice_crossings(p0: Coord, p1: Coord, bounds: &Bounds, grid: u32, out: &mut Vec<f64>) {
    let grid = grid.max(1) as f64;
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
    use crate::terrain::{TERRAIN_GRID, TERRAIN_GRID_DETAIL};

    /// Densifies the test line at `grid` and asserts a vertex lands on every
    /// vertical cell edge and on the diagonals the span crosses.
    fn assert_crossings_at(grid: u32) {
        // A west→east line off the lattice rows: densification must place a
        // vertex exactly on every cell edge and diagonal it crosses, so no
        // chord spans a triangle break and the drape stays on the drawn
        // ground (the mesh is planar only inside each triangle).
        let b = Bounds::of_tile(14, 8500, 5800);
        let cy = b.south + 0.53 * b.height();
        let (x0, x1) = (b.west + 0.30 * b.width(), b.west + 0.70 * b.width());
        let line = LineString(vec![Coord { x: x0, y: cy }, Coord { x: x1, y: cy }]);
        let (xy, zs) =
            densify_road_line(&line, &b, grid, ROAD_SEGMENT_Q, &mut |c| c, &mut |_, _| 0.0);
        assert_eq!(xy.len(), zs.len());

        let cw = b.width() / grid as f64;
        for k in 0..=grid {
            let edge = b.west + k as f64 * cw;
            if edge > x0 + 1e-12 && edge < x1 - 1e-12 {
                assert!(
                    xy.iter().any(|c| (c.x - edge).abs() < 1e-9 * b.width()),
                    "expected a sample on vertical cell edge {k} of grid {grid}"
                );
            }
        }
        // Diagonal crossings: gx − gy passes ~0.4·grid integers over this span.
        let ch = b.height() / grid as f64;
        let on_diagonal = xy
            .iter()
            .filter(|c| {
                let g = (c.x - b.west) / cw - (c.y - b.south) / ch;
                (g - g.round()).abs() < 1e-7
            })
            .count();
        let expected = (grid as usize * 4) / 10;
        assert!(
            on_diagonal >= expected,
            "expected ≥{expected} samples on grid-{grid} cell diagonals, got {on_diagonal}"
        );
    }

    #[test]
    fn drape_samples_land_on_every_lattice_crossing() {
        assert_crossings_at(TERRAIN_GRID);
    }

    #[test]
    fn drape_samples_land_on_every_detail_lattice_crossing() {
        assert_crossings_at(TERRAIN_GRID_DETAIL);
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
        let (xy, _) = densify_road_line(
            &line,
            &b,
            TERRAIN_GRID,
            ROAD_SEGMENT_Q,
            &mut snap,
            &mut |_, _| 0.0,
        );

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
