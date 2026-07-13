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

/// Every junction plate, baked from the solved model — shared by the emit
/// workers through an `Arc`.
pub struct JunctionModel {
    junctions: Vec<BakedJunction>,
}

impl JunctionModel {
    pub fn len(&self) -> usize {
        self.junctions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &BakedJunction> {
        self.junctions.iter()
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
            let half_w = c.class.half_width_m(c.link);
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
            class: "residential".to_string(),
            legs: rj.legs.iter().map(|&(e, n, half_w)| Leg { e, n, half_w }).collect(),
        });
    }
    JunctionModel { junctions }
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

/// Fans the leg mouth corners into a plate mesh, taking each vertex's height in
/// int32 mm from `elev` (a fixed level, or a drape onto the ground).
fn plate_mesh(
    j: &BakedJunction,
    bounds: &Bounds,
    mut elev: impl FnMut(Coord) -> i32,
) -> Option<TerrainMesh> {
    let frame = Frame::at_center(bounds);
    let up = frame.encode_enu(0.0, 0.0, 1.0);
    let m_lon = frame.m_per_deg_lon;
    let cos_lat = j.point.y.to_radians().cos();

    // Two mouth corners per leg, sorted by bearing around the centre so the fan
    // walks the plate boundary in order.
    let mut corners: Vec<(f64, Coord)> = Vec::with_capacity(j.legs.len() * 2);
    for leg in &j.legs {
        let reach = leg.half_w * PLATE_REACH;
        let mouth =
            Coord { x: j.point.x + leg.e * reach / m_lon, y: j.point.y + leg.n * reach / M_PER_DEG_LAT };
        // Left-perpendicular of the heading.
        let (le, ln) = (-leg.n, leg.e);
        for side in [1.0, -1.0] {
            let c = Coord {
                x: mouth.x + le * leg.half_w * side / m_lon,
                y: mouth.y + ln * leg.half_w * side / M_PER_DEG_LAT,
            };
            let ang = (c.y - j.point.y).atan2((c.x - j.point.x) * cos_lat);
            corners.push((ang, c));
        }
    }
    if corners.len() < 3 {
        return None;
    }
    corners.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("finite bearing"));

    // Centre vertex 0, then the boundary ring; each edge fans a triangle.
    let mut x = Vec::with_capacity(corners.len() + 1);
    let mut y = Vec::with_capacity(corners.len() + 1);
    let mut z = Vec::with_capacity(corners.len() + 1);
    let mut normals = Vec::with_capacity((corners.len() + 1) * 2);
    let mut push = |c: Coord| {
        x.push(project::quantize_x(c.x, bounds));
        y.push(project::quantize_y(c.y, bounds));
        z.push(elev(c));
        normals.push(up.0);
        normals.push(up.1);
    };
    push(j.point);
    for (_, c) in &corners {
        push(*c);
    }
    let m = corners.len() as u32;
    let mut indices = Vec::with_capacity(corners.len() * 3);
    for i in 0..m {
        let a = 1 + i;
        let b = 1 + (i + 1) % m;
        indices.extend_from_slice(&[0, a, b]);
    }
    Some(TerrainMesh { x, y, z, indices, normals })
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
    fn plate_meshes_a_fan_over_the_owning_tile() {
        let bounds = Bounds { west: 5.9, south: 45.9, east: 6.1, north: 46.1 };
        let baked = baked_cross();
        assert!(owns(&bounds, baked.point), "the tile owns the junction centre");
        let mesh = flat_mesh(&baked, &bounds).expect("a plate mesh");
        // Eight mouth corners plus the centre, fanned into eight triangles.
        assert_eq!(mesh.x.len(), 9, "centre + 8 corners");
        assert_eq!(mesh.indices.len(), 8 * 3, "one triangle per boundary edge");
        // Flat: every vertex at the level.
        assert!(mesh.z.iter().all(|&z| z == 372_000));
    }

    #[test]
    fn a_tile_that_does_not_own_the_centre_emits_nothing() {
        // Bounds to the east of the junction: not owned, so its owner emits it
        // and this tile does not double it.
        let bounds = Bounds { west: 6.5, south: 45.9, east: 6.7, north: 46.1 };
        assert!(!owns(&bounds, baked_cross().point));
    }
}
