//! Step 16: the buildings — every footprint stood on the ground, walled,
//! and roofed.
//!
//! The facade step reads the footprints as a mask: the ground nothing paved
//! may enter. This step reads them as what they are, one building at a
//! time, and stands each on the terrain. The construction is the tiler's
//! (`server/src/building_mesh.rs`) in local metres instead of tile units,
//! and it keeps that module's three rules:
//!
//! - **A building stands on the highest ground along its outline and sinks
//!   [`FOUNDATION_M`] past the lowest**, so no flank swallows a wall and no
//!   valley shows daylight under one. The buried part is under the ground,
//!   which is opaque.
//! - **`height` is the ground to the top of the roof.** A pitched roof fits
//!   under it: the eave stands one rise below the top, never above it.
//! - **A roof the outline cannot carry is flat.** A gable needs a convex
//!   quad and a pyramid a convex outline, and neither can have a courtyard;
//!   flat and skillion roofs are planes, and a plane triangulates any
//!   outline, holes and all.
//!
//! Three things differ, each where the tiler's answer was wrong:
//!
//! - **The ground is read where the outline crosses the lattice**, not at
//!   its corners only. The terrain is linear inside each of its triangles,
//!   so those crossings hold every extreme the ground has along the
//!   outline, and a forty-metre facade on a flank has corners that say
//!   nothing about its middle.
//! - **The walls rise to the roof, not to the eave.** Every wall's top edge
//!   is the roof's own rim, so a building is closed. The tiler walled a
//!   skillion to its low eave and left three sides open above it.
//! - **The short side is measured across the longest edge**, not across the
//!   axes: the tiler's bounding box gave a house turned 45° a steeper roof
//!   than the same house square to north.

use crate::drape::drape;
use crate::mesh;
use crate::poly::{self, Pt, Ring, Shape, Shapes};
use crate::step::Summary;
use crate::world::{Buildings, Facade, Terrain, Tri};

/// A storey, in metres, where the source gives floors and no height.
pub const FLOOR_M: f64 = 3.0;

/// A building's height, in metres, where the source gives neither height
/// nor floors — which in Overture is most of them. Without it a town reads
/// as a few tall buildings on open ground.
pub const DEFAULT_HEIGHT_M: f64 = 5.0;

/// How far below the lowest ground along its outline a wall's foot stands,
/// in metres: past the ground's rounding and the DEM's jitter.
pub const FOUNDATION_M: f64 = 2.0;

/// A pitched roof's rise where the source gives none: this fraction of the
/// footprint's short side, held between the two bounds. Overture rarely
/// carries `roof_height`, so this is most pitched roofs.
const RISE_FRACTION: f64 = 0.5;
pub const MIN_RISE_M: f64 = 1.0;
pub const MAX_RISE_M: f64 = 6.0;

/// The roof shapes this step builds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoofShape {
    #[default]
    Flat,
    /// A ridge along the long axis of a quad, over the midpoints of its
    /// short edges.
    Gabled,
    /// An apex over the centroid of a convex outline.
    Pyramidal,
    /// One plane rising from the south edge to the north.
    Skillion,
}

impl RoofShape {
    /// Overture's `roof_shape`, as one of the four. A hip is built as a gable
    /// (a true hip needs a straight skeleton) and a dome as a pyramid; what
    /// is unknown is flat.
    pub fn parse(s: &str) -> RoofShape {
        match s {
            "gabled" | "hipped" | "half_hipped" | "round" | "gambrel" | "mansard" => RoofShape::Gabled,
            "pyramidal" | "dome" | "onion" | "cone" => RoofShape::Pyramidal,
            "skillion" | "lean_to" | "mono_pitch" | "shed" => RoofShape::Skillion,
            _ => RoofShape::Flat,
        }
    }
}

/// A roof as the source mapped it: a prior, which the outline may refuse
/// ([`form`]).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Roof {
    pub shape: RoofShape,
    /// Eave to ridge, in metres, where the source gives it.
    pub rise_m: Option<f64>,
}

impl Roof {
    /// The roof of a source's `roof_shape` and `roof_height`, either absent;
    /// a rise that is not positive is no rise.
    pub fn mapped(shape: Option<&str>, rise_m: Option<f64>) -> Roof {
        Roof { shape: shape.map_or(RoofShape::Flat, RoofShape::parse), rise_m: rise_m.filter(|h| *h > 0.0) }
    }
}

/// The height, in metres, a source gives a building: measured, else its
/// floors at [`FLOOR_M`]. `None` when it gives neither, and the reader
/// stands it at [`DEFAULT_HEIGHT_M`].
pub fn mapped_height(height: Option<f64>, floors: Option<f64>) -> Option<f64> {
    height.filter(|h| *h > 0.0).or_else(|| floors.map(|n| n * FLOOR_M).filter(|h| *h > 0.0))
}

/// What the step built, and what it had to refuse.
#[derive(Debug, Default)]
struct Tally {
    buildings: usize,
    /// Roofs by the shape built, one per footprint piece.
    built: [usize; 4],
    /// Pieces mapped with a pitched roof their outline could not carry.
    degraded: usize,
    /// Pieces the ear clipper refused: walled, and left without a roof.
    failed: usize,
    tallest: f64,
    relief: f64,
    /// Buildings whose ground falls away under them by more than their own
    /// height: where standing on the highest ground is a guess the
    /// downhill facade shows — more foundation than building.
    perched: usize,
    /// The roofs' plan area against the footprints', in square metres.
    lost_m2: f64,
}

/// Stands every building of `facade` on `terrain`.
pub fn run(terrain: &Terrain, facade: &Facade) -> (Buildings, Summary) {
    let mut out = Buildings::default();
    let mut n = Tally::default();
    for b in &facade.buildings {
        let Some((lo, hi)) = ground(terrain, &b.footprint) else {
            continue;
        };
        n.buildings += 1;
        n.tallest = n.tallest.max(b.height_m);
        n.relief = n.relief.max(hi - lo);
        n.perched += (hi - lo > b.height_m) as usize;
        let (foot, top) = (lo - FOUNDATION_M, hi + b.height_m);
        for shape in b.footprint.iter().filter_map(readable) {
            let form = form(b.roof.shape, &shape);
            n.built[form as usize] += 1;
            n.degraded += (b.roof.shape != form) as usize;
            let rise = match form {
                RoofShape::Flat => 0.0,
                _ => b
                    .roof
                    .rise_m
                    .unwrap_or_else(|| (short_side(&shape[0]) * RISE_FRACTION).max(MIN_RISE_M))
                    .min(MAX_RISE_M)
                    .min(b.height_m),
            };
            let from = out.roofs.indices.len();
            if !stand(&mut out, &shape, form, foot, top - rise, top) {
                n.failed += 1;
                continue;
            }
            n.lost_m2 += (plan_area(&out.roofs, from) - poly::area(&vec![shape])).abs();
        }
    }
    let [flat, gabled, pyramidal, skillion] = n.built;
    let mesh = |t: &Tri| format!("{}/{}", t.indices.len() / 3, t.positions.len());
    let summary = Summary::new()
        .with("buildings", n.buildings)
        .with("flat", flat)
        .with("gabled", gabled)
        .with("pyramidal", pyramidal)
        .with("skillion", skillion)
        .with("degraded", n.degraded)
        .with("failed", n.failed)
        .with("walls", mesh(&out.walls))
        .with("roofs", mesh(&out.roofs))
        .with("tallest", format!("{:.1}", n.tallest))
        .with("relief", format!("{:.1}", n.relief))
        .with_share("perched", n.perched, n.buildings)
        .with("lost_m2", format!("{:.1e}", n.lost_m2));
    (out, summary)
}

/// The lowest and highest ground along every ring of `footprint`, read
/// where the rings cross the lattice. `None` for a footprint of no rings.
fn ground(terrain: &Terrain, footprint: &Shapes) -> Option<(f64, f64)> {
    let mut span: Option<(f64, f64)> = None;
    for ring in footprint.iter().flatten() {
        let Some(&first) = ring.first() else {
            continue;
        };
        let mut closed = ring.clone();
        closed.push(first);
        for p in drape(terrain, &closed) {
            let (lo, hi) = span.unwrap_or((p[2], p[2]));
            span = Some((lo.min(p[2]), hi.max(p[2])));
        }
    }
    span
}

/// `shape` with every ring cleaned of the vertices that add no shape (so a
/// quad drawn with a fifth vertex along one side is a quad), the outer
/// counter-clockwise and the holes clockwise; `None` if the outer is gone.
fn readable(shape: &Shape) -> Option<Shape> {
    let mut rings = shape.iter().map(|r| mesh::cleaned(r)).filter(|r| r.len() >= 3);
    let outer = poly::oriented(rings.next()?, true);
    Some(std::iter::once(outer).chain(rings.map(|r| poly::oriented(r, false))).collect())
}

/// The roof `shape` can carry of the one mapped: a gable over a convex
/// quad, a pyramid over a convex outline, neither over a courtyard; flat
/// otherwise. A skillion is a plane and goes over anything.
pub fn form(mapped: RoofShape, shape: &Shape) -> RoofShape {
    let plain = shape.len() == 1 && convex(&shape[0]);
    match mapped {
        RoofShape::Skillion => RoofShape::Skillion,
        RoofShape::Gabled if plain && shape[0].len() == 4 => RoofShape::Gabled,
        RoofShape::Pyramidal if plain => RoofShape::Pyramidal,
        _ => RoofShape::Flat,
    }
}

/// Whether the counter-clockwise `ring` turns left at every vertex.
fn convex(ring: &Ring) -> bool {
    let n = ring.len();
    (0..n).all(|i| {
        let (a, b, c) = (ring[i], ring[(i + 1) % n], ring[(i + 2) % n]);
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.0
    })
}

/// The footprint's extent across its longest edge, or along it if that is
/// less, in metres: the width a roof spans, whichever way the house is
/// turned.
fn short_side(ring: &Ring) -> f64 {
    let edge = |i: usize| (ring[i], ring[(i + 1) % ring.len()]);
    let len = |(a, b): (Pt, Pt)| (b[0] - a[0]).hypot(b[1] - a[1]);
    let Some((a, b)) = (0..ring.len()).map(edge).max_by(|x, y| len(*x).total_cmp(&len(*y))) else {
        return 0.0;
    };
    let u = poly::unit([b[0] - a[0], b[1] - a[1]]);
    let extent = |d: Pt| {
        let s = ring.iter().map(|p| p[0] * d[0] + p[1] * d[1]);
        s.clone().fold(f64::NEG_INFINITY, f64::max) - s.fold(f64::INFINITY, f64::min)
    };
    extent(u).min(extent([-u[1], u[0]]))
}

/// Walls `shape` from `foot` up to a `form` roof whose eave stands at
/// `eave` and whose top at `top`, and roofs it. `false` if the roof could
/// not be triangulated; the walls stand regardless.
fn stand(out: &mut Buildings, shape: &Shape, form: RoofShape, foot: f64, eave: f64, top: f64) -> bool {
    let at = |p: Pt, z: f64| [p[0], p[1], z];
    let outer = &shape[0];
    match form {
        RoofShape::Flat | RoofShape::Skillion => {
            // One plane: level at the top, or rising south to north from
            // the eave to the top across the outer ring's span.
            let (y0, y1) = outer.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[1]), hi.max(p[1])));
            let z = |p: Pt| match form {
                RoofShape::Skillion if y1 > y0 => eave + (top - eave) * (p[1] - y0) / (y1 - y0),
                _ => top,
            };
            for ring in shape {
                walls(&mut out.walls, &ring.iter().map(|&p| at(p, z(p))).collect::<Vec<_>>(), foot);
            }
            let Some(ears) = mesh::ear_clip(shape) else {
                return false;
            };
            for ear in ears {
                out.roofs.triangle(ear.map(|p| at(p, z(p))));
            }
        }
        RoofShape::Pyramidal => {
            walls(&mut out.walls, &outer.iter().map(|&p| at(p, eave)).collect::<Vec<_>>(), foot);
            let n = outer.len() as f64;
            let apex = [outer.iter().map(|p| p[0]).sum::<f64>() / n, outer.iter().map(|p| p[1]).sum::<f64>() / n, top];
            for i in 0..outer.len() {
                out.roofs.triangle([at(outer[i], eave), at(outer[(i + 1) % outer.len()], eave), apex]);
            }
        }
        RoofShape::Gabled => {
            // The ridge runs along the longer pair of edges, over the
            // midpoints of the shorter pair; the gable ends are walls.
            let d = |a: Pt, b: Pt| (b[0] - a[0]).hypot(b[1] - a[1]);
            let [v0, v1, v2, v3] = [outer[0], outer[1], outer[2], outer[3]];
            let [a0, a1, b0, b1] = if d(v0, v1) >= d(v1, v2) { [v0, v1, v2, v3] } else { [v1, v2, v3, v0] };
            let mid = |a: Pt, b: Pt| [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
            let (r0, r1) = (at(mid(a1, b0), top), at(mid(b1, a0), top));
            let [a0, a1, b0, b1] = [at(a0, eave), at(a1, eave), at(b0, eave), at(b1, eave)];
            walls(&mut out.walls, &[a0, a1, r0, b0, b1, r1], foot);
            out.roofs.triangle([a0, a1, r0]);
            out.roofs.triangle([a0, r0, r1]);
            out.roofs.triangle([b0, b1, r1]);
            out.roofs.triangle([b0, r1, r0]);
        }
    }
    true
}

/// One wall per edge of the closed ring `rim`, from `foot` up to the rim,
/// facing out of a counter-clockwise ring and into a clockwise one.
fn walls(tri: &mut Tri, rim: &[[f64; 3]], foot: f64) {
    for (i, &b) in rim.iter().enumerate() {
        let a = rim[(i + rim.len() - 1) % rim.len()];
        tri.quad([[a[0], a[1], foot], [b[0], b[1], foot], b, a]);
    }
}

/// The plan area of the triangles of `tri` from index `from` on.
fn plan_area(tri: &Tri, from: usize) -> f64 {
    tri.indices[from..]
        .chunks_exact(3)
        .map(|t| {
            let [a, b, c] = [0, 1, 2].map(|k| tri.positions[t[k] as usize]);
            0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, plan};
    use crate::step::Step;
    use crate::terrain::{self, height_at, tests::{dem, extent}};
    use crate::world::{Building, World};

    use super::*;

    /// A straight road on `ground` with the houses of `house`, stood up.
    fn stood(ground: &str, house: &str) -> (World, Summary) {
        let mut steps = plan(Step::Facade);
        steps.push(Step::Building);
        let (w, ran) = built(ground, "net:straight?len=200", Some(house), 10.0, &steps);
        (w, ran.last())
    }

    fn buildings(w: &World) -> &Buildings {
        w.buildings.as_ref().expect("the building step ran")
    }

    fn zs(t: &Tri) -> (f64, f64) {
        t.positions.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[2]), hi.max(p[2])))
    }

    fn ground_at(w: &World, p: Pt) -> f64 {
        height_at(w.terrain.as_ref().unwrap(), p[0], p[1])
    }

    #[test]
    fn a_flat_house_stands_its_height_on_flat_ground() {
        let (w, s) = stood("flat", "house:beside?d=20&l=10&w=8&h=12");
        let b = buildings(&w);
        let g = ground_at(&w, [0.0, 24.0]);
        assert_eq!(s.num("buildings"), 1.0, "{s}");
        assert_eq!(s.num("flat"), 1.0, "{s}");
        // Four walls of two triangles, and a rectangle's two ears.
        assert_eq!(b.walls.indices.len(), 4 * 6);
        assert_eq!(b.roofs.indices.len(), 2 * 3);
        assert_eq!(zs(&b.roofs), (g + 12.0, g + 12.0));
        assert_eq!(zs(&b.walls), (g - FOUNDATION_M, g + 12.0));
        assert!(s.num("lost_m2") < 1e-9, "{s}");
    }

    #[test]
    fn a_house_on_a_slope_stands_on_its_high_side_and_sinks_past_its_low() {
        // 0.1 across a 10 m house: a metre of relief along the outline.
        let (w, s) = stood("ramp?grade=0.1&bearing=90&radius=100000", "house:beside?d=20&l=10&w=8&h=6");
        let (lo, hi) = (ground_at(&w, [-5.0, 20.0]), ground_at(&w, [5.0, 20.0]));
        assert!((hi - lo - 1.0).abs() < 1e-6);
        assert!((s.num("relief") - 1.0).abs() < 0.05, "{s}");
        let b = buildings(&w);
        assert!((zs(&b.roofs).1 - (hi + 6.0)).abs() < 1e-9);
        assert!((zs(&b.walls).0 - (lo - FOUNDATION_M)).abs() < 1e-9);
        assert_eq!(s.num("perched"), 0.0, "{s}");
        // On a 45° flank the same house stands on ten metres it is not.
        let (_, s) = stood("ramp?grade=1&bearing=90&radius=100000", "house:beside?d=20&l=10&w=8&h=6");
        assert_eq!(s.num("perched"), 1.0, "{s}");
    }

    #[test]
    fn a_gabled_roof_fits_under_the_height_with_its_ridge_along_the_house() {
        let (w, s) = stood("flat", "house:beside?d=20&l=12&w=8&h=10&roof=gabled&rise=3");
        assert_eq!(s.num("gabled"), 1.0, "{s}");
        let b = buildings(&w);
        let g = ground_at(&w, [0.0, 24.0]);
        assert_eq!(zs(&b.roofs), (g + 7.0, g + 10.0), "the eave a rise under the top");
        // The ridge runs along x, the 12 m side, down the middle of the 8.
        for p in b.roofs.positions.iter().filter(|p| p[2] == g + 10.0) {
            assert!((p[1] - 24.0).abs() < 1e-9 && (p[0].abs() - 6.0).abs() < 1e-9, "{p:?}");
        }
        // The gable ends are walls, up to the ridge.
        assert_eq!(zs(&b.walls).1, g + 10.0);
        assert!(s.num("lost_m2") < 1e-9, "{s}");
    }

    #[test]
    fn an_unmapped_rise_is_half_the_short_side_whichever_way_the_house_is_turned() {
        let (a, _) = stood("flat", "house:beside?d=20&l=12&w=8&h=10&roof=gabled");
        let (b, _) = stood("flat", "house:beside?d=20&l=12&w=8&h=10&roof=gabled&rot=45");
        let rise = |w: &World| {
            let (lo, hi) = zs(&buildings(w).roofs);
            hi - lo
        };
        assert!((rise(&a) - 4.0).abs() < 1e-9, "{}", rise(&a));
        assert!((rise(&b) - 4.0).abs() < 1e-6, "{}", rise(&b));
    }

    #[test]
    fn a_skillion_is_walled_to_its_roof_on_every_side() {
        let (w, s) = stood("flat", "house:beside?d=20&l=10&w=8&h=9&roof=skillion&rise=2");
        assert_eq!(s.num("skillion"), 1.0, "{s}");
        let b = buildings(&w);
        let g = ground_at(&w, [0.0, 24.0]);
        assert_eq!(zs(&b.roofs), (g + 7.0, g + 9.0), "low at the south, high at the north");
        // Every wall's top corner is a roof vertex at the same point: no
        // side is left open between the eave and the plane.
        for p in b.walls.positions.iter().filter(|p| p[2] > g) {
            assert!(b.roofs.positions.iter().any(|r| r == p), "a wall top off the roof: {p:?}");
        }
    }

    #[test]
    fn a_pyramid_over_a_notched_facade_is_flat_and_says_so() {
        let (w, s) = stood("flat", "house:beside?d=20&notch=2&h=8&roof=pyramidal");
        assert_eq!((s.num("pyramidal"), s.num("flat"), s.num("degraded")), (0.0, 1.0, 1.0), "{s}");
        let g = ground_at(&w, [0.0, 24.0]);
        assert_eq!(zs(&buildings(&w).roofs), (g + 8.0, g + 8.0));
        let (w, s) = stood("flat", "house:beside?d=20&h=8&roof=pyramidal");
        assert_eq!((s.num("pyramidal"), s.num("degraded")), (1.0, 0.0), "{s}");
        assert_eq!(zs(&buildings(&w).roofs).1, g + 8.0, "the apex at the height");
    }

    #[test]
    fn a_courtyard_is_walled_and_left_open_to_the_sky() {
        let (t, _) = terrain::run(&extent(), &mut dem("flat"), 10.0, usize::MAX);
        let outer = poly::rect(-10.0, -10.0, 10.0, 10.0).remove(0);
        let hole = poly::oriented(poly::rect(-4.0, -4.0, 4.0, 4.0).remove(0), false);
        let facade = Facade {
            buildings: vec![Building {
                footprint: vec![vec![outer, hole]],
                height_m: 10.0,
                roof: Roof { shape: RoofShape::Gabled, rise_m: None },
            }],
            ..Facade::default()
        };
        let (b, s) = run(&t, &facade);
        assert_eq!((s.num("flat"), s.num("degraded")), (1.0, 1.0), "{s}");
        assert_eq!(b.walls.indices.len(), 8 * 6, "the courtyard is walled too");
        assert!(s.num("lost_m2") < 1e-9, "the roof is the annulus: {s}");
        let inside = |t: &[u32]| {
            let [a, b, c] = [0, 1, 2].map(|k| b.roofs.positions[t[k] as usize]);
            let side = |p: [f64; 3], q: [f64; 3]| (q[0] - p[0]) * (0.0 - p[1]) - (q[1] - p[1]) * (0.0 - p[0]);
            side(a, b) > 0.0 && side(b, c) > 0.0 && side(c, a) > 0.0
        };
        assert!(!b.roofs.indices.chunks_exact(3).any(inside), "the roof covers the courtyard");
    }

    #[test]
    fn a_building_the_source_gives_no_height_is_guessed_and_counted() {
        let (w, _) = stood("flat", "house:beside?d=20");
        let g = ground_at(&w, [0.0, 25.0]);
        assert_eq!(zs(&buildings(&w).roofs).1, g + DEFAULT_HEIGHT_M);
        let (_, ran) = built("flat", "net:straight?len=200", Some("house:row?d=20"), 10.0, &plan(Step::Facade));
        assert_eq!(ran.last().num("guessed"), 2.0);
        let (_, ran) = built("flat", "net:straight?len=200", Some("house:row?d=20&h=9"), 10.0, &plan(Step::Facade));
        assert_eq!(ran.last().num("guessed"), 0.0);
        assert_eq!(mapped_height(None, Some(4.0)), Some(12.0));
        assert_eq!(mapped_height(Some(0.0), None), None);
    }

    #[test]
    fn a_roof_shape_is_read_as_one_of_four() {
        assert_eq!(RoofShape::parse("hipped"), RoofShape::Gabled);
        assert_eq!(RoofShape::parse("dome"), RoofShape::Pyramidal);
        assert_eq!(RoofShape::parse("lean_to"), RoofShape::Skillion);
        assert_eq!(RoofShape::parse("flat"), RoofShape::Flat);
        assert_eq!(RoofShape::parse("saltbox"), RoofShape::Flat);
        assert_eq!(Roof::mapped(None, Some(-1.0)), Roof::default());
    }
}
