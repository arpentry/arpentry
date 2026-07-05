//! Draped-road generator: bakes per-vertex elevation onto a road centerline.
//!
//! A road with no corridor profile sits on the rendered ground
//! ([`GroundSampler::surface`], the same lattice the terrain mesh
//! triangulates, consistent across tiles). A road *with* a solved profile
//! renders at
//!
//! ```text
//! surface(z) + road_m − surface(z_ref)
//! ```
//!
//! — its solved height, expressed relative to the engineered rendered ground
//! at the reference zoom. At `z_ref` (the zoom the solver anchored to, seen
//! close up) this is exactly `road_m`: the road meets its structures with no
//! step (invariant 2), and wherever the terrain lattice captured the
//! corridor's earthwork the correction is ~zero, so it lies on the drawn
//! embankment rather than floating twice its height above it. At coarser
//! zooms the per-zoom surface is the datum, so the road still hugs that
//! zoom's rendered terrain (invariant 4) with the same zoom-independent
//! engineered offset.

use geo_types::{Coord, Geometry, LineString, MultiLineString};

use crate::ground::sampler::GroundSampler;
use crate::project::{self, Bounds};
use crate::solve::{self, Profile};
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
/// `f.z`. A no-op for non-line geometry.
pub fn bake(
    f: &mut EncoderFeature,
    profile: Option<&Profile>,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
    bounds: &Bounds,
) {
    let mut height = |lon: f64, lat: f64| match profile {
        // At the reference zoom the correction cancels exactly (the emitting
        // lattice IS the reference lattice): the road renders at its solved
        // height, meeting decks and portals with no step.
        Some(p) if z == z_ref => p.height_at(lon, lat),
        Some(p) => {
            let surface = sampler.surface(bounds, lon, lat, z);
            let ref_bounds = solve::tile_containing(z_ref, lon, lat);
            surface + p.height_at(lon, lat) - sampler.surface(&ref_bounds, lon, lat, z_ref)
        }
        None => sampler.surface(bounds, lon, lat, z),
    };
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

/// Densifies one linestring to ~`seg_q` quantized spacing, snaps each
/// (original and inserted) vertex through `snap`, and samples the height at
/// the snapped position.
fn densify_road_line(
    line: &LineString,
    bounds: &Bounds,
    seg_q: f64,
    snap: &mut dyn FnMut(Coord) -> Coord,
    height: &mut dyn FnMut(f64, f64) -> f64,
) -> (Vec<Coord>, Vec<i32>) {
    let pts = &line.0;
    let mut xy = Vec::new();
    let mut zs = Vec::new();
    if pts.is_empty() {
        return (xy, zs);
    }
    let mut push = |c: Coord, xy: &mut Vec<Coord>, zs: &mut Vec<i32>| {
        let c = snap(c);
        zs.push(project::quantize_z(height(c.x, c.y)));
        xy.push(c);
    };
    push(pts[0], &mut xy, &mut zs);
    for w in pts.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        let qlen = quant_len(p0, p1, bounds);
        let n = ((qlen / seg_q).ceil() as usize).clamp(1, MAX_VERTS);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let c = Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t };
            push(c, &mut xy, &mut zs);
        }
    }
    (xy, zs)
}

/// Distance between two lon/lat points in quantized tile units.
fn quant_len(a: Coord, b: Coord, bounds: &Bounds) -> f64 {
    let dx = project::quantize_x(b.x, bounds) as f64 - project::quantize_x(a.x, bounds) as f64;
    let dy = project::quantize_y(b.y, bounds) as f64 - project::quantize_y(a.y, bounds) as f64;
    (dx * dx + dy * dy).sqrt()
}
