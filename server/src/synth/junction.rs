//! Junction plates — a filled road surface meshed across an intersection so
//! the legs meet on one paved area instead of overlapping strokes
//! (docs/GENERATION.md, scenario S4).
//!
//! Increment 1 plates the *corridor* junctions the solver already built
//! ([`SceneGraph::junctions`], Plan B): interchanges and ramp merges among the
//! graded/structure network, flat at the welded height every member shares
//! there. Each junction's legs are trimmed back a little and their mouth
//! corners fanned into a plate. The at-grade majority — every ordinary street
//! intersection, which is not in the scene graph — is a later increment that
//! drapes plates on the engineered ground.
//!
//! Plates are baked once from the solved model (heights are a pure function of
//! it) and emitted by the single tile that owns the junction centre, so tiles
//! agree at their seams (invariant 5). Coordinates are tile-local quantized
//! uint16 / int32-mm with an up ENU normal, matching `MeshGeometry`.

use geo_types::{Coord, Geometry, Point};

use crate::building_mesh::{Frame, M_PER_DEG_LAT};
use crate::ground::sampler::GroundSampler;
use crate::priors::RoadClass;
use crate::project::{self, Bounds};
use crate::scene::SceneGraph;
use crate::solve::SolvedModel;
use crate::terrain::TerrainMesh;
use crate::tile_build::EncoderFeature;
use crate::value::Value;

/// How far past a leg's mouth, in half-widths, the plate reaches — the trim
/// radius that sets the intersection's size. Larger than 1 so the plate laps
/// over the incoming carriageways and reads as one paved area.
const PLATE_REACH: f64 = 1.6;

/// How far a trimmed surface band runs on under the plate past its trim
/// radius, in metres — the overlap that guarantees no sliver of ground shows
/// between a band end and the plate mouth, whatever densification and
/// quantization do to either edge.
const BAND_TUCK_M: f64 = 0.75;

/// Interior points inserted per corner fillet (the quadratic Bézier between
/// two legs' mouth corners).
const FILLET_STEPS: usize = 6;

/// Farthest from the plate centre, in metres, a fillet's control point (the
/// carriageway-edge intersection) may sit. Beyond it the two edges barely
/// converge (a near-straight through pair) and a straight chord reads better
/// than a kilometre-flat arc.
const FILLET_MAX_M: f64 = 40.0;

/// A leg at a junction: unit heading away from the centre (ENU east, north) and
/// the road half-width there.
struct Leg {
    e: f64,
    n: f64,
    half_w: f64,
}

/// A baked junction plate: its centre, the styling class, its legs, and its
/// surface level — a fixed int32-mm height (a corridor junction, at its welded
/// level) or `None` for an at-grade road junction, which drapes on the ground.
pub struct BakedJunction {
    point: Coord,
    level_mm: Option<i32>,
    class: String,
    legs: Vec<Leg>,
}

impl BakedJunction {
    /// The junction centre.
    pub fn point(&self) -> Coord {
        self.point
    }

    /// How far from the centre, in metres, an approaching surface band of
    /// half-width `band_half_m` is trimmed: its own mouth distance, capped by
    /// the widest leg's mouth (a mapped-wide carriageway must not trim past
    /// the plate and leave a gap), less [`BAND_TUCK_M`] so the band always
    /// ends *under* the plate.
    pub fn trim_radius_m(&self, band_half_m: f64) -> f64 {
        let max_mouth =
            self.legs.iter().map(|l| l.half_w * PLATE_REACH).fold(0.0, f64::max);
        ((band_half_m * PLATE_REACH).min(max_mouth) - BAND_TUCK_M).max(0.0)
    }
}

/// Every junction plate, baked from the solved model — shared by the emit
/// workers through an `Arc`. A coarse geographic grid answers "which plates
/// are near this box" without a linear scan, which both the per-tile plate
/// emission and the per-segment marking trims (phase 1, millions of
/// segments) depend on.
pub struct JunctionModel {
    junctions: Vec<BakedJunction>,
    grid: std::collections::HashMap<(i32, i32), Vec<u32>>,
}

/// Grid cell size in degrees (~1 km): plates per cell stay in the tens even
/// in towns, and a tile or segment query touches a handful of cells.
const GRID_DEG: f64 = 0.01;

fn grid_cell(x: f64, y: f64) -> (i32, i32) {
    ((x / GRID_DEG).floor() as i32, (y / GRID_DEG).floor() as i32)
}

impl JunctionModel {
    fn build(junctions: Vec<BakedJunction>) -> JunctionModel {
        let mut grid: std::collections::HashMap<(i32, i32), Vec<u32>> =
            std::collections::HashMap::new();
        for (i, j) in junctions.iter().enumerate() {
            grid.entry(grid_cell(j.point.x, j.point.y)).or_default().push(i as u32);
        }
        JunctionModel { junctions, grid }
    }

    pub fn len(&self) -> usize {
        self.junctions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &BakedJunction> {
        self.junctions.iter()
    }

    /// The plates whose centres fall in the `(west, south, east, north)` box.
    /// The caller pads the box by whatever reach (trim radius, plate size)
    /// matters to it.
    pub fn near(&self, b: (f64, f64, f64, f64)) -> Vec<&BakedJunction> {
        let (x0, y0) = grid_cell(b.0, b.1);
        let (x1, y1) = grid_cell(b.2, b.3);
        let mut out = Vec::new();
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                if let Some(cell) = self.grid.get(&(cx, cy)) {
                    for &i in cell {
                        let p = self.junctions[i as usize].point;
                        if p.x >= b.0 && p.x <= b.2 && p.y >= b.1 && p.y <= b.3 {
                            out.push(&self.junctions[i as usize]);
                        }
                    }
                }
            }
        }
        out
    }
}

/// Bakes a plate for every corridor junction with three or more legs, at the
/// height its profiled members share (the Plan-B weld made them agree). A
/// junction with no profiled member has no known height and is skipped (its
/// at-grade plate belongs to the later ground-draped increment).
pub fn bake(scene: &SceneGraph, solved: &SolvedModel) -> JunctionModel {
    let mut junctions = Vec::new();
    for j in &scene.junctions {
        let mut legs = Vec::new();
        let mut level: Option<f64> = None;
        let mut class: Option<RoadClass> = None;
        for m in &j.members {
            let c = &scene.corridors[m.corridor as usize];
            // Legs span the surface band's edge (paint half-width plus the
            // structure shoulder), so a trimmed band meets the mouth flush.
            let half_w = c.class.half_width_m(c.link) + crate::priors::STRUCTURE_SHOULDER_M;
            for (e, n) in leg_headings(&c.nodes, &c.arc, c.cos_lat, m.arc, c.total()) {
                legs.push(Leg { e, n, half_w });
            }
            if let Some(p) = solved.profile(m.corridor) {
                let h = p.road_at_arc(m.arc);
                level = Some(level.map_or(h, |l| l.max(h)));
                // The highest-standing member sets the styling class.
                if class.is_none() || h >= level.unwrap() - 1e-9 {
                    class = Some(c.class);
                }
            }
        }
        let (Some(level_m), true) = (level, legs.len() >= 3) else {
            continue;
        };
        junctions.push(BakedJunction {
            point: j.point,
            level_mm: Some((level_m * 1000.0).round() as i32),
            class: class_name(class.unwrap_or(RoadClass::Minor)).to_string(),
            legs,
        });
    }
    // At-grade road junctions: legs already carry heading and half-width; the
    // plate drapes on the ground (no fixed level).
    for rj in &scene.road_junctions {
        if rj.legs.len() < 3 {
            continue;
        }
        junctions.push(BakedJunction {
            point: rj.point,
            level_mm: None,
            class: if rj.class.is_empty() { "residential".to_string() } else { rj.class.clone() },
            legs: rj.legs.iter().map(|&(e, n, half_w)| Leg { e, n, half_w }).collect(),
        });
    }
    JunctionModel::build(junctions)
}

/// The plate feature for `baked`, or `None` when this tile does not own the
/// junction centre (so exactly one tile emits it) or the plate is degenerate.
/// An at-grade junction drapes on the engineered ground through `sampler`; a
/// corridor junction sits at its fixed welded level.
pub fn plate(
    baked: &BakedJunction,
    bounds: &Bounds,
    sampler: &mut GroundSampler,
    z: u8,
) -> Option<EncoderFeature> {
    if !owns(bounds, baked.point) {
        return None;
    }
    let mesh = match baked.level_mm {
        Some(mm) => plate_mesh(baked, bounds, |_| mm),
        None => plate_mesh(baked, bounds, |c| {
            (sampler.surface(bounds, c.x, c.y, z) * 1000.0).round() as i32
        }),
    }?;
    Some(EncoderFeature {
        id: baked.point.x.to_bits() ^ baked.point.y.to_bits().rotate_left(32),
        geometry: Geometry::Point(Point(baked.point)),
        properties: vec![("class".to_string(), Value::String(baked.class.clone()))],
        elevation: None,
        z: None,
        mesh: Some(mesh),
        synth: crate::synth::Synth::None,
    })
}

/// A leg's mouth in metres relative to the plate centre: its unit heading,
/// bearing, and the two mouth corners (right and left of the heading, looking
/// out from the centre).
struct Mouth {
    ang: f64,
    e: f64,
    n: f64,
    right: (f64, f64),
    left: (f64, f64),
}

/// Fans the plate boundary into a mesh, taking each vertex's height in int32
/// mm from `elev` (a fixed level, or a drape onto the ground). The boundary
/// walks the legs counter-clockwise: each leg's straight mouth cross-edge
/// (where its trimmed surface band lands flush), then the corner fillet
/// curving to the next leg — so the intersection reads with rounded curb
/// returns instead of a straight-chorded fan.
fn plate_mesh(
    j: &BakedJunction,
    bounds: &Bounds,
    mut elev: impl FnMut(Coord) -> i32,
) -> Option<TerrainMesh> {
    let frame = Frame::at_center(bounds);
    let up = frame.encode_enu(0.0, 0.0, 1.0);
    let m_lon = frame.m_per_deg_lon;

    let mut mouths: Vec<Mouth> = j
        .legs
        .iter()
        .filter_map(|leg| {
            let len = (leg.e * leg.e + leg.n * leg.n).sqrt();
            if len < 1e-9 || leg.half_w <= 0.0 {
                return None;
            }
            let (e, n) = (leg.e / len, leg.n / len);
            let reach = leg.half_w * PLATE_REACH;
            let (pe, pn) = (-n, e); // left perpendicular
            Some(Mouth {
                ang: n.atan2(e),
                e,
                n,
                right: (e * reach - pe * leg.half_w, n * reach - pn * leg.half_w),
                left: (e * reach + pe * leg.half_w, n * reach + pn * leg.half_w),
            })
        })
        .collect();
    if mouths.len() < 3 {
        return None;
    }
    mouths.sort_by(|a, b| a.ang.total_cmp(&b.ang));

    // The boundary ring in metres: per leg its mouth corners, then the fillet
    // toward the next leg counter-clockwise.
    let mut ring_m: Vec<(f64, f64)> = Vec::with_capacity(mouths.len() * (2 + FILLET_STEPS));
    for i in 0..mouths.len() {
        let a = &mouths[i];
        let b = &mouths[(i + 1) % mouths.len()];
        ring_m.push(a.right);
        ring_m.push(a.left);
        fillet(a, b, &mut ring_m);
    }

    // Centre vertex 0, then the ring; each boundary edge fans a triangle.
    let mut x = Vec::with_capacity(ring_m.len() + 1);
    let mut y = Vec::with_capacity(ring_m.len() + 1);
    let mut z = Vec::with_capacity(ring_m.len() + 1);
    let mut normals = Vec::with_capacity((ring_m.len() + 1) * 2);
    let mut push = |c: Coord| {
        x.push(project::quantize_x(c.x, bounds));
        y.push(project::quantize_y(c.y, bounds));
        z.push(elev(c));
        normals.push(up.0);
        normals.push(up.1);
    };
    push(j.point);
    for &(me, mn) in &ring_m {
        push(Coord { x: j.point.x + me / m_lon, y: j.point.y + mn / M_PER_DEG_LAT });
    }
    let m = ring_m.len() as u32;
    let mut indices = Vec::with_capacity(ring_m.len() * 3);
    for i in 0..m {
        let a = 1 + i;
        let b = 1 + (i + 1) % m;
        indices.extend_from_slice(&[0, a, b]);
    }
    Some(TerrainMesh { x, y, z, indices, normals })
}

/// Appends the interior points of the corner fillet from `a`'s left corner to
/// `b`'s right corner (`b` counter-clockwise of `a`): a quadratic Bézier whose
/// control point is the intersection of the two carriageway edges — the
/// standard curb-return approximation. Appends nothing (a straight chord)
/// when no plausible corner exists: a gap of a half-turn or more (the plate's
/// flat side, or a reflex gap whose arc would cross the centre), near-parallel
/// edges, or an intersection behind the corners or absurdly far out.
fn fillet(a: &Mouth, b: &Mouth, out: &mut Vec<(f64, f64)>) {
    let gap = (b.ang - a.ang).rem_euclid(std::f64::consts::TAU);
    if gap >= std::f64::consts::PI - 1e-6 {
        return;
    }
    // Edge lines run from each corner back toward the centre along the leg:
    // a.left + t·(−a.heading) = b.right + s·(−b.heading).
    let p = a.left;
    let q = b.right;
    let (d1e, d1n) = (-a.e, -a.n);
    let (d2e, d2n) = (-b.e, -b.n);
    let denom = d1e * d2n - d1n * d2e;
    if denom.abs() < 1e-3 {
        return;
    }
    let (re, rn) = (q.0 - p.0, q.1 - p.1);
    let t = (re * d2n - rn * d2e) / denom;
    let s = (re * d1n - rn * d1e) / denom;
    if t <= 0.0 || s <= 0.0 {
        return;
    }
    let c = (p.0 + t * d1e, p.1 + t * d1n);
    if (c.0 * c.0 + c.1 * c.1).sqrt() > FILLET_MAX_M {
        return;
    }
    for k in 1..FILLET_STEPS {
        let u = k as f64 / FILLET_STEPS as f64;
        let w0 = (1.0 - u) * (1.0 - u);
        let w1 = 2.0 * u * (1.0 - u);
        let w2 = u * u;
        out.push((w0 * p.0 + w1 * c.0 + w2 * q.0, w0 * p.1 + w1 * c.1 + w2 * q.1));
    }
}

/// The unit ENU heading(s) of a corridor leg at arc `at`: one pointing into the
/// corridor from an end, both directions from an interior through-node.
fn leg_headings(nodes: &[Coord], arc: &[f64], cos_lat: f64, at: f64, total: f64) -> Vec<(f64, f64)> {
    if nodes.len() < 2 {
        return Vec::new();
    }
    let i = edge_at(arc, at);
    let (a, b) = (nodes[i], nodes[i + 1]);
    let (de, dn) = ((b.x - a.x) * cos_lat, b.y - a.y);
    let len = (de * de + dn * dn).sqrt();
    if len < 1e-12 {
        return Vec::new();
    }
    let (e, n) = (de / len, dn / len);
    const END_EPS_M: f64 = 1.5;
    if at <= END_EPS_M {
        vec![(e, n)] // starts here: heading forward into the corridor
    } else if at >= total - END_EPS_M {
        vec![(-e, -n)] // ends here: heading back into the corridor
    } else {
        vec![(e, n), (-e, -n)] // a through node: the road leaves both ways
    }
}

/// The edge index whose arc span contains `at` (clamped to a valid edge).
fn edge_at(arc: &[f64], at: f64) -> usize {
    match arc.binary_search_by(|v| v.partial_cmp(&at).expect("finite arc")) {
        Ok(i) => i.min(arc.len() - 2),
        Err(i) => i.saturating_sub(1).min(arc.len() - 2),
    }
}

/// Whether the tile owns a world point: half-open bounds, so exactly one tile
/// of a shared junction emits its plate.
fn owns(b: &Bounds, c: Coord) -> bool {
    c.x >= b.west && c.x < b.east && c.y >= b.south && c.y < b.north
}

/// The Overture class string for a [`RoadClass`], for plate styling.
fn class_name(c: RoadClass) -> &'static str {
    match c {
        RoadClass::Motorway => "motorway",
        RoadClass::Trunk => "trunk",
        RoadClass::Primary => "primary",
        RoadClass::Secondary => "secondary",
        RoadClass::Minor => "residential",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baked_cross() -> BakedJunction {
        // A four-way crossing at (6, 46): legs east/west/north/south, 4 m wide.
        let legs = vec![
            Leg { e: 1.0, n: 0.0, half_w: 4.0 },
            Leg { e: -1.0, n: 0.0, half_w: 4.0 },
            Leg { e: 0.0, n: 1.0, half_w: 4.0 },
            Leg { e: 0.0, n: -1.0, half_w: 4.0 },
        ];
        BakedJunction {
            point: Coord { x: 6.0, y: 46.0 },
            level_mm: Some(372_000),
            class: "secondary".into(),
            legs,
        }
    }

    /// The flat plate mesh of a baked junction, bypassing the ground sampler
    /// (a corridor junction sits at its fixed level).
    fn flat_mesh(j: &BakedJunction, bounds: &Bounds) -> Option<TerrainMesh> {
        plate_mesh(j, bounds, |_| j.level_mm.expect("a fixed level in this test"))
    }

    #[test]
    fn plate_meshes_a_filleted_fan_over_the_owning_tile() {
        let bounds = Bounds { west: 5.9, south: 45.9, east: 6.1, north: 46.1 };
        let baked = baked_cross();
        assert!(owns(&bounds, baked.point), "the tile owns the junction centre");
        let mesh = flat_mesh(&baked, &bounds).expect("a plate mesh");
        // Eight mouth corners plus the four corners' fillet points, fanned
        // from the centre: one triangle per boundary edge.
        let ring = 4 * 2 + 4 * (FILLET_STEPS - 1);
        assert_eq!(mesh.x.len(), ring + 1, "centre + mouth corners + fillets");
        assert_eq!(mesh.indices.len(), ring * 3, "one triangle per boundary edge");
        // Flat: every vertex at the level.
        assert!(mesh.z.iter().all(|&z| z == 372_000));
    }

    #[test]
    fn fillet_curves_inward_between_perpendicular_legs() {
        // East and north legs of a 4 m road: the curb return must bow toward
        // the centre relative to the straight corner chord.
        let m = |e: f64, n: f64, hw: f64| {
            let reach = hw * PLATE_REACH;
            let (pe, pn) = (-n, e);
            Mouth {
                ang: n.atan2(e),
                e,
                n,
                right: (e * reach - pe * hw, n * reach - pn * hw),
                left: (e * reach + pe * hw, n * reach + pn * hw),
            }
        };
        let (a, b) = (m(1.0, 0.0, 4.0), m(0.0, 1.0, 4.0));
        let mut pts = Vec::new();
        fillet(&a, &b, &mut pts);
        assert_eq!(pts.len(), FILLET_STEPS - 1);
        let dist = |p: (f64, f64)| (p.0 * p.0 + p.1 * p.1).sqrt();
        let chord_mid = ((a.left.0 + b.right.0) * 0.5, (a.left.1 + b.right.1) * 0.5);
        let arc_mid = pts[pts.len() / 2];
        assert!(
            dist(arc_mid) < dist(chord_mid) - 0.3,
            "fillet mid {arc_mid:?} does not bow inward of the chord {chord_mid:?}"
        );
        // Every fillet point stays outside the centre (the fan stays valid).
        assert!(pts.iter().all(|&p| dist(p) > 2.0));
    }

    #[test]
    fn opposite_legs_get_a_straight_side() {
        // A through pair (gap of a half-turn) must not fillet — the plate's
        // flat side stays a straight chord.
        let m = |e: f64, n: f64| Mouth {
            ang: (n as f64).atan2(e),
            e,
            n,
            right: (e * 6.4 - -n * 4.0, n * 6.4 - e * 4.0),
            left: (e * 6.4 + -n * 4.0, n * 6.4 + e * 4.0),
        };
        let mut pts = Vec::new();
        fillet(&m(1.0, 0.0), &m(-1.0, 0.0), &mut pts);
        assert!(pts.is_empty(), "a half-turn gap must stay straight");
    }

    #[test]
    fn trim_radius_tucks_under_the_mouth_and_is_capped() {
        let j = baked_cross(); // legs 4 m: mouths at 6.4 m
        // A matching band trims tucked just inside its own mouth.
        let r = j.trim_radius_m(4.0);
        assert!((r - (6.4 - BAND_TUCK_M)).abs() < 1e-9);
        // A mapped-wide band cannot trim past the widest mouth (no gap).
        let wide = j.trim_radius_m(12.0);
        assert!((wide - (6.4 - BAND_TUCK_M)).abs() < 1e-9);
    }

    #[test]
    fn a_tile_that_does_not_own_the_centre_emits_nothing() {
        // Bounds to the east of the junction: not owned, so its owner emits it
        // and this tile does not double it.
        let bounds = Bounds { west: 6.5, south: 45.9, east: 6.7, north: 46.1 };
        assert!(!owns(&bounds, baked_cross().point));
    }
}
