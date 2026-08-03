//! The road height field — one continuous answer for where the road surface
//! sits (docs/ROADS.md invariant 5, docs/GROUND.md §4).
//!
//! Every road representation used to drape on its own function. The paint and
//! the surface band read [`road::surface_height`], which clamps the engineered
//! ground up to the corridor's own solved profile; a junction plate read the
//! bench target raw. Where a road sat in fill those two disagreed, so the
//! asphalt stepped at every plate mouth. And neither could answer the question a
//! *unioned* surface asks, because a vertex of a union is not owned by any one
//! corridor: it may be interior to an intersection, or born where two roads'
//! edges cross.
//!
//! So the height stops being a per-corridor lookup and becomes a field: given a
//! level and a plan position, one number, continuous everywhere. Two kinds of
//! source contribute, and the field is their normalized blend:
//!
//! - **Corridors.** A carriageway covers the band within its own half-width of
//!   its centerline, and its value there is exactly what
//!   [`road::surface_height`] already returned — the engineered ground, raised to
//!   the corridor's own profile where the road stands on fill. That clamp is
//!   load-bearing and is quoted unchanged (`road.rs:114-122`): a road must never
//!   drape below its own solved profile, or it steps under its own bridge deck.
//! - **Intersections.** A cluster covers its star-shaped [`Area`], and its value
//!   is the height the solve made its members *share*
//!   ([`SolvedModel::junction_height`], persisted for exactly this). This is the
//!   pin: the one place where several corridors' answers must agree, and where
//!   the solver has already computed the agreement.
//!
//! Four properties make the blend the right shape, and each is a test below:
//!
//! 1. **Identity.** Away from overlaps every covering source is a segment of the
//!    same corridor and returns that corridor's own height, so the normalized sum
//!    gives it back — to within float rounding, not bit-identically, because
//!    `(w·h)/w` is not exact in IEEE and a point beside a node is covered by both
//!    of its segments. The residual is nanometres against a millimetre
//!    quantization, so ordinary asphalt cannot regress.
//! 2. **Continuity.** The weight vanishes at a source's own edge, so a source
//!    entering or leaving the covering set contributes nothing at the moment it
//!    does — no step where a buffer begins. Note that continuous is not flat:
//!    blending between two roads at different heights has a real gradient across
//!    their overlap, which is the point.
//! 3. **Pinning.** A pin does not *vote* with the corridors — every leg of a
//!    junction converges on its centre, so as one voter among N it was
//!    outnumbered by its own legs precisely where it should be authoritative.
//!    Instead it *overrides*: a flat-topped dominance, exactly 1 at the centre and
//!    0 at the paved boundary, mixes the solved height over the carriageway
//!    blend. So the centre is exactly the solved height, the hand-back at the
//!    boundary is exact too, and the number of legs is irrelevant.
//! 4. **Determinism.** Sources come from [`GridIndex::query`], which returns a
//!    sorted, deduplicated id list, so the sum order is a function of the model
//!    and never of hashing.
//!
//! **Levels and grade-separation layers partition the field.** Only sources on
//! the queried level *and* layer are considered. That is what keeps a viaduct approach from blending with the
//! street beneath it — the case `road.rs`'s clamp comment warns about — and it is
//! why blending overlapping sources within a level is safe: two corridors at one
//! level whose carriageways overlap genuinely share that asphalt and must agree
//! on its height, which is precisely what a blend does.

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::ground::sampler::GroundSampler;
use crate::project::Bounds;
use crate::scene::DEG_M;
use crate::solve::{Profile, SolvedModel};
use crate::synth::area::Area;
use crate::synth::junction::JunctionModel;
use crate::synth::road;

/// One carriageway segment: the stretch of centerline between two nodes, and how
/// far either side of it that corridor's asphalt reaches.
struct Src<'a> {
    a: Coord,
    b: Coord,
    cos_lat: f64,
    half_m: f64,
    level: i64,
    layer: u32,
    profile: Option<&'a Profile>,
}

/// One intersection: its paved extent and the height its members share.
struct Pin<'a> {
    area: &'a Area,
    level: i64,
    height: f64,
}

/// The height field over one tile's neighbourhood. Built once per tile; `at` is
/// then called per mesh vertex, so the sources are indexed rather than scanned.
pub struct HeightField<'a> {
    srcs: Vec<Src<'a>>,
    src_grid: GridIndex,
    pins: Vec<Pin<'a>>,
    pin_grid: GridIndex,
}

impl<'a> HeightField<'a> {
    /// Collects the sources near `bounds` — padded by the widest reach that can
    /// influence a point inside it, so a road just outside the tile still pulls
    /// on the asphalt it shares.
    ///
    /// The corridor sources come straight from the scene graph and the pins from
    /// the baked intersections; nothing here infers anything the model does not
    /// already state.
    pub fn for_tile(
        junctions: &'a JunctionModel,
        solved: &'a SolvedModel,
        z: u8,
        bounds: &Bounds,
    ) -> HeightField<'a> {
        // Only the zooms that mesh asphalt need the field. Below them there is no
        // surface for the paint to stay continuous with, so the plain per-corridor
        // answer is both correct and what shipped before.
        //
        // This is not just a saving, it is the difference between linear and
        // absurd: a tile's source query is bounded by its own extent, and a z0
        // tile's extent is the world — so every coarse tile was collecting *every*
        // carriageway segment in the extract (~270k of them) and indexing them, to
        // draw no asphalt whatsoever. That dominated the whole tiling run.
        if z < crate::priors::ROAD_SURFACE_MIN_ZOOM {
            return HeightField {
                srcs: Vec::new(),
                src_grid: GridIndex::new(),
                pins: Vec::new(),
                pin_grid: GridIndex::new(),
            };
        }
        let pad = crate::priors::PAVE_PAD_M / DEG_M;
        let box_ = (
            bounds.west - pad,
            bounds.south - pad,
            bounds.east + pad,
            bounds.north + pad,
        );

        // The carriageway segments come from the baked model rather than from a
        // fresh walk of the scene: they are a pure function of it, and every zoom
        // of every tile would otherwise re-derive the same list.
        let mut srcs: Vec<Src<'a>> = Vec::new();
        let mut src_grid = GridIndex::new();
        let mut ids = Vec::new();
        junctions.sources_near(box_, &mut ids);
        for &i in &ids {
            let s = *junctions.source(i);
            let pad_s = s.half_m / DEG_M;
            let bb = (
                s.a.x.min(s.b.x) - pad_s,
                s.a.y.min(s.b.y) - pad_s,
                s.a.x.max(s.b.x) + pad_s,
                s.a.y.max(s.b.y) + pad_s,
            );
            src_grid.insert(bb, srcs.len() as u32);
            srcs.push(Src {
                a: s.a,
                b: s.b,
                cos_lat: s.cos_lat,
                half_m: s.half_m,
                level: s.level,
                layer: s.layer,
                profile: solved.profile(s.corridor),
            });
        }

        let mut pins: Vec<Pin<'a>> = Vec::new();
        let mut pin_grid = GridIndex::new();
        for j in junctions.near(box_) {
            let Some(height) = j.height() else {
                continue; // no solved height: this intersection pins nothing
            };
            let c = j.point();
            let reach = j.area().reach_deg();
            let bb = (c.x - reach.0, c.y - reach.1, c.x + reach.0, c.y + reach.1);
            pin_grid.insert(bb, pins.len() as u32);
            // Every plated intersection is at grade today; the level rides along
            // so the partition is already in place when levels differ.
            pins.push(Pin { area: j.area(), level: 0, height });
        }

        HeightField { srcs, src_grid, pins, pin_grid }
    }

    /// Whether the field carries no sources at all — below the surface zoom, or
    /// on a tile the network does not reach. Callers then use the per-corridor
    /// [`road::surface_height`] directly.
    pub fn is_empty(&self) -> bool {
        self.srcs.is_empty() && self.pins.is_empty()
    }

    /// The road-surface height in metres at a plan position on `level`.
    ///
    /// Below [`crate::priors::ROAD_SURFACE_MIN_ZOOM`] no asphalt is meshed, so
    /// this is the plain per-corridor answer; the blend only runs where a
    /// surface exists to be continuous across.
    pub fn at(
        &self,
        sampler: &mut GroundSampler,
        level: i64,
        layer: u32,
        z: u8,
        z_ref: u8,
        bounds: &Bounds,
        lon: f64,
        lat: f64,
        scratch: &mut Vec<u32>,
    ) -> f64 {
        // The engineered ground here, evaluated *once*. It is the same for every
        // corridor at this point and it is the expensive part of the answer (an
        // earthwork-index walk plus a lattice evaluation), so the blend below
        // applies each corridor's own clamp to it rather than re-deriving it.
        let ground = road::ground_height(sampler, z, z_ref, bounds, lon, lat);
        // Whether the drawn ground under this asphalt has been cut away. Where
        // it has, nothing can poke up through the carriageway and the raise-only
        // clamps below are not only unnecessary but harmful — see
        // [`road::on_ground`]. Asked once, and asked the same way by the pin and
        // by the corridor blend: letting the two disagree put a step wherever a
        // plate met a road, and a 0.36 m one across a tile border.
        let hole = sampler.cuts_hole(z);

        // The carriageway blend, over every corridor covering this point.
        let mut num = 0.0f64;
        let mut den = 0.0f64;
        // Nearest source, for the empty-covering-set fallback: a point in a
        // curb-return fillet is paved but inside nobody's buffer. Bounded — see
        // `NEAR_M` — so it answers for the fillet and not for the whole tile.
        let mut best: Option<(f64, f64, f64)> = None; // (distance, reach, height)

        self.src_grid.query((lon, lat, lon, lat), scratch);
        for &i in scratch.iter() {
            let s = &self.srcs[i as usize];
            if s.level != level || s.layer != layer {
                continue;
            }
            let d = point_to_segment_m(lon, lat, s.a, s.b, s.cos_lat);
            // *The* per-corridor answer, called rather than reproduced — the
            // shared ground raised to this corridor's own solved profile.
            // `surface_height` is exactly `on_ground(ground_height(..))`, so the
            // single-source case is still identical to it by construction rather
            // than by two copies of the arithmetic agreeing.
            let h = road::on_ground(ground, s.profile, sampler, z, z_ref, lon, lat);
            if best.is_none_or(|(bd, _, _)| d < bd) {
                best = Some((d, s.half_m, h));
            }
            if d <= s.half_m {
                let w = kernel(d, s.half_m);
                num += w * h;
                den += w;
            }
        }

        // The intersection pins, which *override* rather than vote. Every leg of a
        // junction converges on its centre, so as one voter among N the pin was
        // outnumbered by its own legs exactly where it is supposed to be
        // authoritative. Instead its dominance `lambda` — 1 at the centre, 0 at
        // the paved boundary — mixes the pinned height over the carriageway blend,
        // so the count of legs is irrelevant and both ends of the range are exact.
        let mut pin_num = 0.0f64;
        let mut pin_den = 0.0f64;
        let mut lambda = 0.0f64;

        self.pin_grid.query((lon, lat, lon, lat), scratch);
        for &i in scratch.iter() {
            let p = &self.pins[i as usize];
            // An intersection is a place on the ground network, so it pins only
            // the unranked layer — a flyover passing overhead must not be dragged
            // to the height of the junction beneath it.
            if p.level != level || layer != 0 {
                continue;
            }
            // The pin carries the same clamp every carriageway source carries
            // ([`road::on_ground`]), and drops it under the same condition. With
            // ground drawn under the asphalt, a junction solved into a cutting
            // the ground stage declined to dig — a bench too steep to be
            // plausible, or a neighbour's bench holding the ground above it —
            // pinned its plate metres inside the hillside while every road
            // meeting it rode the surface, which is the one disagreement the
            // field exists to prevent (ROADS.md invariant 5). With the ground
            // cut away there is no hillside to be inside of, and clamping here
            // while the corridors do not is itself a disagreement.
            let height = if hole { p.height } else { p.height.max(ground) };
            let (de, dn) = p.area.offset_m(Coord { x: lon, y: lat });
            let d = (de * de + dn * dn).sqrt();
            if best.is_none_or(|(bd, _, _)| d < bd) {
                best = Some((d, r_of(p, de, dn, d), height));
            }
            if d < 1e-9 {
                return height; // dead centre: the solved height, exactly
            }
            // The star radius along this bearing, so a pin's influence ends
            // exactly where its paved area does.
            let r = p.area.radius(de / d, dn / d);
            if d <= r {
                let w = pin_kernel(d, r);
                pin_num += w * height;
                pin_den += w;
                lambda = lambda.max(w);
            }
        }

        let blended = if den > 0.0 {
            num / den
        } else {
            // Outside every source — a curb-return fillet, or the residue of the
            // closing. The nearest source's answer, which agrees with the
            // single-source limit, so the field stays continuous here too; with
            // no source at all, the bare engineered ground.
            // Only *just* outside a source does the nearest answer apply. The
            // gap this covers is a curb-return fillet — paved, but inside nobody's
            // buffer — which is at most the closing radius wide. Unbounded, as
            // this first was, every point in the tile inherited the nearest road's
            // height, including its raise-only clamp to its own profile: ground
            // hundreds of metres from an embanked road came back at the
            // embankment's height.
            //
            // Handed back *continuously*. Switching from the nearest source's
            // height to the bare ground at a threshold is a step in the field
            // wherever the two differ, and property 2 above is that no source
            // may enter or leave the covering set with a step. It reached the
            // archive as `seam.pavement_step`: two border vertices a hair apart
            // straddling the threshold, one taking the road and one the ground,
            // 0.36 m apart in a metric that had been exactly zero. So the
            // hand-back ramps over the fillet's own width instead.
            match best {
                Some((d, reach, h)) => {
                    let over = (d - reach).max(0.0);
                    let w = (1.0 - over / crate::priors::CURB_RETURN_M).clamp(0.0, 1.0);
                    w * h + (1.0 - w) * ground
                }
                None => ground,
            }
        };
        // The pin mixes over *that*, whether or not a carriageway covered the
        // point. Returning the bare pinned height when none did — which is what
        // this used to do — steps by `(1 - lambda) * (blend - pin)` at the
        // moment the first carriageway enters the covering set, for the same
        // reason the hand-back above had to be ramped. `best` already tracks
        // pins as well as corridors, so the fallback under a plate is the
        // plate's own height and not the bare ground.
        if pin_den > 0.0 {
            let pinned = pin_num / pin_den;
            lambda.clamp(0.0, 1.0) * pinned + (1.0 - lambda.clamp(0.0, 1.0)) * blended
        } else {
            blended
        }
    }
}

/// A pin's reach along the bearing of a query, for the nearest-source bound.
fn r_of(p: &Pin, de: f64, dn: f64, d: f64) -> f64 {
    if d < 1e-9 {
        0.0
    } else {
        p.area.radius(de / d, dn / d)
    }
}

/// The blend weight at distance `d` from a source whose reach is `r`: `1` at the
/// source's own centre, falling smoothly to `0` at its edge.
///
/// Deliberately *not* the Shepard `1/(d² + ε)` weight that interpolation uses.
/// These sources are areas, not point samples: a carriageway's height is defined
/// across its whole band and already varies along it, so there is nothing to
/// reproduce exactly at the centerline. An inverse-distance factor would instead
/// put a singularity on every road axis — crossing a carriageway, the surface
/// spiked several metres toward that road's height within 20 cm of its centre and
/// back out again. Continuous, but a visible ridge down the middle of every road.
///
/// The plain compact bump blends across an overlap over its full width, is
/// smooth in `d`, and still vanishes at the edge so a source joining or leaving
/// the covering set changes nothing.
fn kernel(d: f64, r: f64) -> f64 {
    if !(r > 0.0) {
        return 0.0;
    }
    let t = ((r - d) / r).max(0.0);
    t * t
}

/// How strongly a pin overrides the carriageway blend at distance `d` from its
/// centre, given its reach `r` there: `1` at the centre, `0` at the boundary.
///
/// Flat-topped, unlike [`kernel`], and for the opposite reason. A corridor's
/// weight has to *vanish* smoothly at its edge so joining the covering set is
/// free, which makes it steepest in the middle. A pin has to *hold* across its
/// area — it is the one height the solver guarantees several corridors share — so
/// it needs to be flat where it is authoritative and fall off only as it hands
/// back at the boundary. With the corridor shape a pin was already 2 % diluted
/// 5 cm from the centre and a quarter-strength at half its radius, which is no
/// pin at all.
fn pin_kernel(d: f64, r: f64) -> f64 {
    if !(r > 0.0) {
        return 0.0;
    }
    let t = (d / r).min(1.0);
    1.0 - t * t
}

/// Distance in metres from a lon/lat point to the segment `a → b`.
fn point_to_segment_m(lon: f64, lat: f64, a: Coord, b: Coord, cos_lat: f64) -> f64 {
    let m_lon = DEG_M * cos_lat;
    let (px, py) = ((lon - a.x) * m_lon, (lat - a.y) * DEG_M);
    let (ex, ey) = ((b.x - a.x) * m_lon, (b.y - a.y) * DEG_M);
    let len2 = ex * ex + ey * ey;
    if len2 < 1e-18 {
        return (px * px + py * py).sqrt();
    }
    let t = ((px * ex + py * ey) / len2).clamp(0.0, 1.0);
    let (dx, dy) = (px - ex * t, py - ey * t);
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_kernel_vanishes_at_the_edge_and_peaks_at_the_centre() {
        // Property 2: a source joining the covering set contributes nothing.
        assert_eq!(kernel(4.0, 4.0), 0.0, "weight at the edge must be zero");
        assert_eq!(kernel(5.0, 4.0), 0.0, "weight outside must be zero");
        // Peaks at exactly 1 in the middle, so a pin's dominance is a fraction.
        assert_eq!(kernel(0.0, 4.0), 1.0, "the centre weight must be one");
        // No singularity: crossing a centerline must not spike the surface.
        assert!(kernel(0.01, 4.0) < 1.01, "the kernel spikes near the centre");
        // Monotone decreasing in between, so no interior bump.
        let mut prev = f64::INFINITY;
        for i in 0..=40 {
            let d = i as f64 * 0.1;
            let w = kernel(d, 4.0);
            assert!(w <= prev + 1e-12, "kernel rose at d={d}");
            prev = w;
        }
        assert_eq!(kernel(1.0, 0.0), 0.0, "a zero reach weighs nothing");
    }

    #[test]
    fn distance_to_a_segment_is_perpendicular_inside_and_radial_past_the_ends() {
        let cos_lat = 46f64.to_radians().cos();
        let m_lon = DEG_M * cos_lat;
        let a = Coord { x: 6.0, y: 46.0 };
        let b = Coord { x: 6.0 + 100.0 / m_lon, y: 46.0 };
        // Abeam the middle: the perpendicular offset.
        let mid_lon = 6.0 + 50.0 / m_lon;
        let d = point_to_segment_m(mid_lon, 46.0 + 7.0 / DEG_M, a, b, cos_lat);
        assert!((d - 7.0).abs() < 0.05, "perpendicular distance {d} != 7");
        // On the line: zero.
        assert!(point_to_segment_m(mid_lon, 46.0, a, b, cos_lat) < 0.05);
        // Past the end: the radial distance to the endpoint, not to the line.
        let past = point_to_segment_m(6.0 + 130.0 / m_lon, 46.0, a, b, cos_lat);
        assert!((past - 30.0).abs() < 0.05, "beyond-the-end distance {past} != 30");
        // A degenerate segment is a point.
        let dot = point_to_segment_m(6.0, 46.0 + 5.0 / DEG_M, a, a, cos_lat);
        assert!((dot - 5.0).abs() < 0.05, "point distance {dot} != 5");
    }

    // ---- End-to-end fixtures -------------------------------------------------
    //
    // A DEM-less sampler: `surface` reads 0 and `bed_target` is `None`, so a
    // corridor's height reduces to its own solved profile through
    // `road::surface_height`'s raise-only clamp. That is exactly the arithmetic
    // under test — the field's job is which sources contribute and how, not what
    // the ground is doing.

    use crate::ground::sampler::GroundSampler;
    use crate::ground::GroundModel;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, Junction, JunctionMember, SceneGraph};
    use std::sync::Arc;

    const Z: u8 = 15;
    const LAT: f64 = 46.0;

    fn sampler() -> GroundSampler {
        GroundSampler::new(None, Arc::new(GroundModel::empty()), Z)
    }

    fn m_lon() -> f64 {
        DEG_M * LAT.to_radians().cos()
    }

    /// An east–west corridor of `len_m` starting at `(lon0, LAT)`, `width_m`
    /// wide, with `n` evenly spaced nodes.
    fn corridor(id: u32, lon0: f64, len_m: f64, n: usize, width_m: f64) -> Corridor {
        let step = len_m / (n - 1) as f64;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: lon0 + i as f64 * step / m_lon(), y: LAT }).collect();
        Corridor {
            id,
            nodes,
            arc: (0..n).map(|i| i as f64 * step).collect(),
            cos_lat: LAT.to_radians().cos(),
            class: RoadClass::Minor,
            class_key: "residential".to_string(),
            link: false,
            drivable: true,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    /// A north–south corridor crossing `LAT` at `lon`.
    fn cross_corridor(id: u32, lon: f64, len_m: f64, n: usize, width_m: f64) -> Corridor {
        let step = len_m / (n - 1) as f64;
        let nodes: Vec<Coord> = (0..n)
            .map(|i| Coord { x: lon, y: LAT - 0.5 * len_m / DEG_M + i as f64 * step / DEG_M })
            .collect();
        Corridor {
            id,
            nodes,
            arc: (0..n).map(|i| i as f64 * step).collect(),
            cos_lat: LAT.to_radians().cos(),
            class: RoadClass::Minor,
            class_key: "residential".to_string(),
            link: false,
            drivable: true,
            width_m: Some(width_m),
            spans: Vec::new(),
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    #[test]
    fn away_from_an_overlap_the_field_is_the_corridors_own_surface() {
        // Property 1. One isolated corridor, no intersections: every sample must
        // read what `road::surface_height` reads for that corridor, because the
        // field is calling it and the normalization gives it straight back.
        let c = corridor(0, 6.0, 200.0, 11, 6.0);
        let nodes = c.nodes.clone();
        let scene = SceneGraph::new(vec![c]);
        let solved = SolvedModel::from_profiles(vec![Some(Profile::flat(&nodes, 400.0))], Z);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        let profile = solved.profile(0);
        let mut checked = 0;
        for i in 0..20 {
            // Along the road, and across it inside the carriageway.
            let lon = 6.0 + (10.0 + i as f64 * 9.0) / m_lon();
            for off_m in [-2.0, 0.0, 2.0] {
                let lat = LAT + off_m / DEG_M;
                let want =
                    road::surface_height(profile, false, &mut s, Z, Z, &bounds, lon, lat);
                let got = field.at(&mut s, 0, 0, Z, Z, &bounds, lon, lat, &mut scratch);
                assert!(
                    (got - want).abs() < 1e-9,
                    "field {got} != corridor surface {want} at {lon},{lat}"
                );
                checked += 1;
            }
        }
        assert_eq!(checked, 60, "the samples must actually have run");
    }

    #[test]
    fn the_field_is_pinned_at_the_solved_junction_height() {
        // Property 3. Three legs whose profiles deliberately disagree; the pin
        // says 500 m, and the field must read exactly that at the centre however
        // far the corridors' own answers are from it.
        let centre = Coord { x: 6.0, y: LAT };
        let a = corridor(0, 6.0 - 100.0 / m_lon(), 100.0, 6, 6.0);
        let b = corridor(1, 6.0, 100.0, 6, 6.0);
        let c = cross_corridor(2, 6.0, 200.0, 11, 6.0);
        let heights = [400.0, 410.0, 420.0];
        let profiles: Vec<Option<Profile>> = [&a, &b, &c]
            .iter()
            .zip(heights)
            .map(|(c, h)| Some(Profile::flat(&c.nodes, h)))
            .collect();
        let mut scene = SceneGraph::new(vec![a, b, c]);
        scene.junctions = vec![Junction {
            point: centre,
            connector: 0,
            members: vec![
                JunctionMember { corridor: 0, arc: 100.0 },
                JunctionMember { corridor: 1, arc: 0.0 },
                JunctionMember { corridor: 2, arc: 100.0 },
            ],
        }];
        let solved =
            SolvedModel::from_profiles(profiles, Z).with_junction_heights(vec![Some(500.0)]);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        assert_eq!(junctions.len(), 1, "the three legs plate as one intersection");

        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        let at = |s: &mut GroundSampler, east_m: f64, scratch: &mut Vec<u32>| {
            field.at(s, 0, 0, Z, Z, &bounds, centre.x + east_m / m_lon(), centre.y, scratch)
        };

        // Exact at the centre: this is the number the solver guarantees.
        let at_centre = at(&mut s, 0.0, &mut scratch);
        assert!((at_centre - 500.0).abs() < 1e-9, "centre reads {at_centre}, pin says 500");

        // And it *holds* near the centre rather than being diluted the moment you
        // step off it. The promise is a flat top: the pin's dominance is
        // `1 - (d/r)²`, so the deviation from the pinned height grows
        // *quadratically* with distance, not linearly. Testing that ratio tests
        // the design rather than a tolerance — the legs here disagree with the pin
        // by up to 100 m, so a linear falloff would show up loudly.
        let dev = |s: &mut GroundSampler, d: f64, scratch: &mut Vec<u32>| {
            (at(s, d, scratch) - 500.0).abs()
        };
        let near = dev(&mut s, 0.5, &mut scratch);
        let far = dev(&mut s, 1.0, &mut scratch);
        assert!(near > 0.0, "the legs never pulled at all — the test proves nothing");
        let ratio = far / near;
        assert!(
            (3.5..=4.5).contains(&ratio),
            "doubling the distance changed the deviation {ratio:.2}x, not ~4x: \
             the pin is not flat-topped ({near:.4} m at 0.5 m, {far:.4} m at 1 m)"
        );
        // A hair off centre the pin is still essentially untouched.
        assert!(dev(&mut s, 0.05, &mut scratch) < 0.05, "5 cm off-centre already diluted");

        // Handing back is monotone: the further out, the less the pin says, with
        // no reversal on the way.
        let mut prev = at_centre;
        for i in 1..=40 {
            let h = at(&mut s, i as f64 * 0.25, &mut scratch);
            assert!(h <= prev + 1e-9, "the hand-back reversed at {} m", i as f64 * 0.25);
            prev = h;
        }
    }

    /// A junction the solver put below the ground drawn under it — its members
    /// were profiled into a cutting the ground stage declined to dig.
    ///
    /// With ground drawn under the asphalt it must not pin its plate inside the
    /// hillside, so it carries the same raise-only clamp every carriageway
    /// source carries. With the ground cut back to the kerb there is no hillside
    /// to be inside of, the clamp is dropped (docs/GROUND.md §3), and the plate
    /// reads exactly what the solver decided — which is also what every road
    /// meeting it now reads, and *that agreement* is the property the field
    /// exists to hold.
    #[test]
    fn a_pin_never_sinks_below_the_ground_drawn_under_it() {
        let centre = Coord { x: 6.0, y: LAT };
        let a = corridor(0, 6.0 - 100.0 / m_lon(), 100.0, 6, 6.0);
        let b = corridor(1, 6.0, 100.0, 6, 6.0);
        let c = cross_corridor(2, 6.0, 200.0, 11, 6.0);
        let nodes: Vec<Vec<Coord>> = [&a, &b, &c].iter().map(|c| c.nodes.clone()).collect();
        // Every leg is solved at 500 m, so the ground benches to 500 there.
        let profiles: Vec<Option<Profile>> =
            nodes.iter().map(|n| Some(Profile::flat(n, 500.0))).collect();
        let mut scene = SceneGraph::new(vec![a, b, c]);
        scene.junctions = vec![Junction {
            point: centre,
            connector: 0,
            members: vec![
                JunctionMember { corridor: 0, arc: 100.0 },
                JunctionMember { corridor: 1, arc: 0.0 },
                JunctionMember { corridor: 2, arc: 100.0 },
            ],
        }];
        // …but the junction is pinned 20 m below it.
        let solved =
            SolvedModel::from_profiles(profiles, Z).with_junction_heights(vec![Some(480.0)]);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let ground = Arc::new(crate::ground::derive(&scene, &solved, None, 1));
        assert!(ground.earthwork_count() > 0, "the legs must bench the ground to 500");
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut scratch = Vec::new();
        let probe = |s: &mut GroundSampler, east_m: f64, scratch: &mut Vec<u32>| {
            field.at(s, 0, 0, Z, Z, &bounds, centre.x + east_m / m_lon(), centre.y, scratch)
        };

        // Ground drawn underneath: the clamp holds the plate up to it.
        let mut s = GroundSampler::new(None, ground.clone(), Z);
        s.set_hole(false);
        for east_m in [0.0, 0.05, 1.0, 3.0] {
            let h = probe(&mut s, east_m, &mut scratch);
            assert!(
                h >= 500.0 - 1e-9,
                "{east_m} m off the centre the plate reads {h}, under the 500 m ground"
            );
        }

        // Ground cut away: the plate is the solved height, and nothing is
        // buried by it because nothing is drawn under it.
        let mut s = GroundSampler::new(None, ground, Z);
        assert!(s.cuts_hole(Z), "the detail rung must cut a hole by default");
        assert!(
            (probe(&mut s, 0.0, &mut scratch) - 480.0).abs() < 1e-9,
            "dead centre must read the solved height"
        );
    }

    /// Walks a transect east along corridor `a`'s axis through a crossing road,
    /// sampling every `step_m`, and returns the largest change between
    /// consecutive samples.
    fn worst_step_over_transect(step_m: f64) -> f64 {
        let a = corridor(0, 6.0 - 150.0 / m_lon(), 300.0, 16, 8.0);
        let b = cross_corridor(1, 6.0, 200.0, 11, 6.0);
        let profiles =
            vec![Some(Profile::flat(&a.nodes, 400.0)), Some(Profile::flat(&b.nodes, 415.0))];
        let scene = SceneGraph::new(vec![a, b]);
        let solved = SolvedModel::from_profiles(profiles, Z);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        let n = (40.0 / step_m).round() as i32;
        let mut prev: Option<f64> = None;
        let mut worst = 0.0f64;
        for i in 0..=n {
            let lon = 6.0 + (-20.0 + i as f64 * step_m) / m_lon();
            let h = field.at(&mut s, 0, 0, Z, Z, &bounds, lon, LAT, &mut scratch);
            if let Some(p) = prev {
                worst = worst.max((h - p).abs());
            }
            prev = Some(h);
        }
        worst
    }

    #[test]
    fn the_field_is_continuous_across_a_buffer_boundary() {
        // Property 2, tested by refinement rather than by flatness. Crossing a
        // road whose surface is 15 m above this one, the field *must* climb — a
        // blend over a 4 m overlap has a real gradient of metres per metre, so
        // asserting near-zero steps would only be testing that the two roads
        // agree, which is not the property.
        //
        // What distinguishes continuous from discontinuous is how the largest
        // step behaves as the sampling shrinks: for a continuous field it shrinks
        // proportionally, and across a jump it does not shrink at all.
        let coarse = worst_step_over_transect(0.01);
        let fine = worst_step_over_transect(0.001);
        assert!(coarse > 0.0, "the transect never crossed anything");
        assert!(
            fine < coarse * 0.2,
            "10x finer sampling only shrank the worst step from {coarse:.5} to {fine:.5}: \
             the field has a jump, not a gradient"
        );
        // And the gradient itself is bounded: crossing the whole 15 m difference
        // takes the width of the overlap, never one step.
        assert!(coarse < 0.1, "the field moves {coarse:.4} m in a single centimetre");
    }

    #[test]
    fn the_field_never_drapes_below_a_corridors_own_profile() {
        // The rationale quoted at `road.rs:114-122`: a road on fill must not sink
        // to the ground under it, or it steps below its own bridge deck. The
        // DEM-less sampler puts the ground at 0, so a corridor solved at 400 m is
        // entirely on fill — the strongest form of the case.
        let c = corridor(0, 6.0, 200.0, 11, 6.0);
        let nodes = c.nodes.clone();
        let scene = SceneGraph::new(vec![c]);
        let solved = SolvedModel::from_profiles(vec![Some(Profile::flat(&nodes, 400.0))], Z);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        for i in 0..20 {
            let lon = 6.0 + (10.0 + i as f64 * 9.0) / m_lon();
            let h = field.at(&mut s, 0, 0, Z, Z, &bounds, lon, LAT, &mut scratch);
            assert!(h >= 400.0 - 1e-9, "the road sank to {h}, below its 400 m profile");
        }
    }

    #[test]
    fn a_road_on_fill_does_not_lift_the_ground_beside_it() {
        // The field answers for the *paved surface*. A point inside a road's
        // buffer reads that road's height — correct for the asphalt and for the
        // paint on it, and exactly wrong for a footway crossing underneath, which
        // is why `synth::emit` only hands the field to features carrying a
        // `width_m`. This test pins the field's half of that contract: the lift is
        // real and it is bounded by the buffer, so a consumer outside the buffer
        // is untouched.
        let c = corridor(0, 6.0 - 100.0 / m_lon(), 200.0, 11, 6.0);
        let nodes = c.nodes.clone();
        let scene = SceneGraph::new(vec![c]);
        // A road solved 30 m up: an embankment or a bridge approach.
        let solved = SolvedModel::from_profiles(vec![Some(Profile::flat(&nodes, 30.0))], Z);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        let at = |s: &mut GroundSampler, off_m: f64, scratch: &mut Vec<u32>| {
            field.at(s, 0, 0, Z, Z, &bounds, 6.0, LAT + off_m / DEG_M, scratch)
        };
        // On the carriageway: the road's own height, 30 m up.
        assert!((at(&mut s, 0.0, &mut scratch) - 30.0).abs() < 1e-9);
        // The half-width is 3 + STRUCTURE_SHOULDER_M = 4 m. Beyond it the field
        // has nothing covering, and with the DEM-less sampler the ground is 0 —
        // so a path out here is on the ground, not on the embankment.
        let outside = at(&mut s, 12.0, &mut scratch);
        assert!(
            outside.abs() < 1e-9,
            "a point 12 m from the centreline reads {outside}, not the ground"
        );
    }

    #[test]
    fn a_level_the_field_knows_nothing_about_falls_back_to_the_ground() {
        // The partition is real: querying a level with no sources on it must not
        // pick up the at-grade road's height.
        let c = corridor(0, 6.0, 200.0, 11, 6.0);
        let nodes = c.nodes.clone();
        let scene = SceneGraph::new(vec![c]);
        let solved = SolvedModel::from_profiles(vec![Some(Profile::flat(&nodes, 400.0))], Z);
        let junctions = crate::synth::junction::bake(&scene, &solved);
        let bounds = crate::solve::tile_containing(Z, 6.0, LAT);
        let field = HeightField::for_tile(&junctions, &solved, Z, &bounds);
        let mut s = sampler();
        let mut scratch = Vec::new();

        let lon = 6.0 + 100.0 / m_lon();
        let on_grade = field.at(&mut s, 0, 0, Z, Z, &bounds, lon, LAT, &mut scratch);
        let upstairs = field.at(&mut s, 3, 0, Z, Z, &bounds, lon, LAT, &mut scratch);
        assert!((on_grade - 400.0).abs() < 1e-9, "at grade reads {on_grade}");
        assert!(upstairs.abs() < 1e-9, "level 3 read {upstairs}, not the bare ground");
    }

}

