//! Stage 1 — the facades a street is bounded by.
//!
//! A street is a room between buildings. The carriageway prior says how wide a
//! `residential` road is *in general*; the facades say how much room this
//! particular stretch of it actually has. Without them nothing owns a street's
//! cross-section: the carriageway takes its band, the bench takes a wider one,
//! and both are drawn straight through whatever wall happens to stand there —
//! 28,719 m² of drawn asphalt inside 1,662 of Montreux's 8,615 footprints
//! (`order.building_overlap`, VERIFICATION.md).
//!
//! This module is the evidence, not the policy. It reads the building input
//! once, keeps every footprint edge in a [`GridIndex`], and answers one
//! question: [`Facades::room`] — standing here on a centerline, how far is the
//! nearest wall to my left, and to my right? Who spends that room, and in what
//! order, belongs to the consumers (`synth::carriageway` takes the carriageway's
//! share first).
//!
//! **The index is never queried per lattice vertex.** The room is resolved once
//! per corridor station, at bake time, into half-widths baked onto the
//! carriageway sources; after that the ground field stays a pure function of
//! bench geometry (GENERATION.md invariant 5). A spatial index inside
//! `GroundStack::height` was measured at 38 s of tiling time for the rejected
//! lateral-trench rule, and that is the shape of mistake this note exists to
//! prevent.

use std::path::Path;

use geo_types::{Coord, Geometry, Polygon};

use crate::geoparquet::{GeoParquet, ReadError};
use crate::scene::DEG_M;

use super::grid::GridIndex;

/// Grid cell size for the facade index, in metres. Sized to the query, which
/// reaches a carriageway half-width plus a station's window rather than an
/// earthwork's feather: a Montreux z16 town centre holds ~9 footprint edges per
/// cell at this size against ~140 at [`super::grid::CELL_M`].
const CELL_M: f64 = 32.0;

/// One footprint edge, in lon/lat. Walls are what a room is bounded by, so the
/// index holds edges rather than polygons — a road that passes *through* a
/// large footprint sees no wall within its window, which is exactly right: that
/// is a level relation to be resolved elsewhere and not a width to be narrowed
/// (the worst site in the extract is a 7,533 m² casino with an `unknown`-class
/// way through it).
type Edge = [Coord; 2];

/// The clear distance from a point on a centerline out to the nearest facade on
/// each side, in metres, measured across the street. Both fields are capped at
/// the reach the query was asked for, so "no wall near me" and "a wall exactly
/// at my reach" are the same answer — which is what makes the caller's cap a
/// no-op away from buildings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Room {
    pub left: f64,
    pub right: f64,
}

impl Room {
    /// The room an open-ground centerline has: `reach` on both sides.
    pub fn open(reach_m: f64) -> Room {
        Room { left: reach_m, right: reach_m }
    }
}

/// Every building footprint edge in the run's window, indexed by plan position.
pub struct Facades {
    edges: Vec<Edge>,
    grid: GridIndex,
    footprints: usize,
}

impl Default for Facades {
    fn default() -> Self {
        Facades::empty()
    }
}

impl Facades {
    /// The index for a run with no building input: every query answers "open
    /// ground". Nothing downstream needs to branch on whether buildings were
    /// supplied.
    pub fn empty() -> Facades {
        Facades { edges: Vec::new(), grid: GridIndex::with_cell_m(CELL_M), footprints: 0 }
    }

    /// Reads the footprints intersecting `bbox` from the building input.
    ///
    /// Interior rings count: a courtyard's wall bounds the street through it
    /// exactly as the outer wall does. No class filter — a shed is a wall and
    /// the drawn asphalt has no business inside one either.
    pub fn read(path: &Path, bbox: (f64, f64, f64, f64)) -> Result<Facades, ReadError> {
        let gp = GeoParquet::open(path)?;
        let row_groups = gp.row_groups_intersecting(bbox);
        let mut out = Facades::empty();
        // The row-group prune is coarse; a footprint a kilometre outside the
        // window can never bound a street inside it, and carrying it costs a
        // cell in every query that lands near it.
        let pad = PAD_M / DEG_M;
        let keep = |c: Coord| {
            c.x >= bbox.0 - pad && c.x <= bbox.2 + pad && c.y >= bbox.1 - pad && c.y <= bbox.3 + pad
        };
        for feature in gp.features(row_groups, &["subtype"])? {
            let f = feature?;
            let before = out.edges.len();
            match &f.geometry {
                Geometry::Polygon(p) => out.push_polygon(p, &keep),
                Geometry::MultiPolygon(mp) => {
                    mp.0.iter().for_each(|p| out.push_polygon(p, &keep))
                }
                _ => continue,
            }
            if out.edges.len() > before {
                out.footprints += 1;
            }
        }
        Ok(out)
    }

    fn push_polygon(&mut self, p: &Polygon, keep: &impl Fn(Coord) -> bool) {
        let mut ring = |r: &[Coord]| {
            for e in r.windows(2) {
                if keep(e[0]) || keep(e[1]) {
                    self.push_edge([e[0], e[1]]);
                }
            }
        };
        ring(&p.exterior().0);
        for hole in p.interiors() {
            ring(&hole.0);
        }
    }

    /// An index over a given set of wall edges — the fixture entry point, and
    /// the one a probe uses to ask what a specific footprint does to a street.
    pub fn from_edges(edges: impl IntoIterator<Item = Edge>) -> Facades {
        let mut out = Facades::empty();
        for e in edges {
            out.push_edge(e);
        }
        out.footprints = usize::from(!out.edges.is_empty());
        out
    }

    fn push_edge(&mut self, e: Edge) {
        let bbox = (
            e[0].x.min(e[1].x),
            e[0].y.min(e[1].y),
            e[0].x.max(e[1].x),
            e[0].y.max(e[1].y),
        );
        self.grid.insert(bbox, self.edges.len() as u32);
        self.edges.push(e);
    }

    pub fn footprint_count(&self) -> usize {
        self.footprints
    }

    /// Every wall edge, in no particular geographic order but in a stable one —
    /// the order the input yielded them. What a check walks to ask what the
    /// ground under a wall is made of.
    pub fn edges(&self) -> &[Edge] {
        &self.edges
    }

    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    /// The room at `p` on a centerline running along `tangent` (a unit vector
    /// in local east/north metres), looking out to `reach_m` on each side.
    ///
    /// **A cross-section, not a proximity.** Only the stretch of wall within
    /// `window_m` of `p` *along* the centerline is measured, and it is measured
    /// by its lateral offset — so a building at the head of a cul-de-sac does
    /// not narrow the street leading to it, and a wall running parallel does
    /// narrow it for its whole length. The window is what makes consecutive
    /// stations see the same corner: it wants to be a couple of station
    /// spacings, so a wall that begins abruptly is seen by several stations and
    /// the width they interpolate between never crosses it.
    ///
    /// An edge that straddles the centerline within the window returns zero
    /// room on both sides. That is honest — the centerline is in the wall —
    /// and it is the caller's floor, not this function, that decides a street
    /// may not be narrowed away.
    ///
    /// `scratch` is the caller's query buffer, reused across stations.
    pub fn room(
        &self,
        p: Coord,
        cos_lat: f64,
        tangent: (f64, f64),
        reach_m: f64,
        window_m: f64,
        scratch: &mut Vec<u32>,
    ) -> Room {
        let mut room = Room::open(reach_m);
        if self.edges.is_empty() || !(reach_m > 0.0) {
            return room;
        }
        let m_lon = DEG_M * cos_lat;
        if !(m_lon > 0.0) {
            return room;
        }
        // The query box covers the rotated window rectangle whatever the
        // tangent's bearing, so the grid never has to know which way the road
        // points.
        let r = reach_m.hypot(window_m);
        self.grid.query((p.x - r / m_lon, p.y - r / DEG_M, p.x + r / m_lon, p.y + r / DEG_M), scratch);
        let (tx, ty) = tangent;
        for &i in scratch.iter() {
            let e = &self.edges[i as usize];
            // Local metres about `p`, then split into longitudinal (along the
            // tangent) and lateral (left of it) components.
            let along = |c: Coord| {
                let (dx, dy) = ((c.x - p.x) * m_lon, (c.y - p.y) * DEG_M);
                (dx * tx + dy * ty, -dx * ty + dy * tx)
            };
            let (la, sa) = along(e[0]);
            let (lb, sb) = along(e[1]);
            // Clip the edge to the window slab. `u` runs 0→1 along the edge.
            let (mut u0, mut u1) = (0.0f64, 1.0f64);
            let dl = lb - la;
            if dl.abs() < SLAB_EPS {
                if la.abs() > window_m {
                    continue;
                }
            } else {
                let (s, t) = ((-window_m - la) / dl, (window_m - la) / dl);
                u0 = u0.max(s.min(t));
                u1 = u1.min(s.max(t));
                if u0 > u1 {
                    continue;
                }
            }
            let s0 = sa + (sb - sa) * u0;
            let s1 = sa + (sb - sa) * u1;
            if (s0 <= 0.0) != (s1 <= 0.0) || s0 == 0.0 || s1 == 0.0 {
                return Room { left: 0.0, right: 0.0 }; // the centerline is in the wall
            }
            if s0 > 0.0 {
                room.left = room.left.min(s0.min(s1));
            } else {
                room.right = room.right.min((-s0).min(-s1));
            }
        }
        room
    }
}

/// How far outside the run's window a footprint is still kept, in metres. A
/// street on the boundary is bounded by the buildings just past it.
const PAD_M: f64 = 100.0;

/// Below this the edge is parallel to the centerline in the window sense and
/// the slab clip degenerates; a metre of longitudinal extent over a footprint
/// edge is not something float noise produces.
const SLAB_EPS: f64 = 1e-9;

#[cfg(test)]
mod tests {
    use super::*;

    /// A facade parallel to an east-west centerline, `north_m` to its north.
    fn wall(lat0: f64, north_m: f64, x0: f64, x1: f64) -> Facades {
        let mut f = Facades::empty();
        let y = lat0 + north_m / DEG_M;
        f.push_edge([Coord { x: x0, y }, Coord { x: x1, y }]);
        f.footprints = 1;
        f
    }

    const P: Coord = Coord { x: 6.9, y: 46.44 };
    const EAST: (f64, f64) = (1.0, 0.0);

    fn cos_lat() -> f64 {
        P.y.to_radians().cos()
    }

    #[test]
    fn open_ground_returns_the_reach_on_both_sides() {
        let f = Facades::empty();
        let r = f.room(P, cos_lat(), EAST, 4.0, 8.0, &mut Vec::new());
        assert_eq!(r, Room::open(4.0));
    }

    #[test]
    fn a_wall_alongside_is_measured_on_its_own_side() {
        let f = wall(P.y, 3.0, 6.89, 6.91);
        let r = f.room(P, cos_lat(), EAST, 6.0, 8.0, &mut Vec::new());
        assert!((r.left - 3.0).abs() < 1e-6, "left {}", r.left);
        assert_eq!(r.right, 6.0, "the far side keeps its full room");
    }

    #[test]
    fn a_wall_past_the_reach_does_not_narrow_anything() {
        let f = wall(P.y, 9.0, 6.89, 6.91);
        let r = f.room(P, cos_lat(), EAST, 6.0, 8.0, &mut Vec::new());
        assert_eq!(r, Room::open(6.0));
    }

    #[test]
    fn a_wall_ahead_is_not_a_wall_beside() {
        // Same 3 m offset, but the edge only exists 40 m down the road.
        let mut f = Facades::empty();
        let east = 40.0 / (DEG_M * cos_lat());
        let y = P.y + 3.0 / DEG_M;
        f.push_edge([Coord { x: P.x + east, y }, Coord { x: P.x + east * 2.0, y }]);
        let r = f.room(P, cos_lat(), EAST, 6.0, 8.0, &mut Vec::new());
        assert_eq!(r, Room::open(6.0), "outside the window, so not this station's wall");
    }

    #[test]
    fn a_wall_across_the_centerline_leaves_no_room_at_all() {
        let mut f = Facades::empty();
        let east = 2.0 / (DEG_M * cos_lat());
        f.push_edge([
            Coord { x: P.x + east, y: P.y - 5.0 / DEG_M },
            Coord { x: P.x + east, y: P.y + 5.0 / DEG_M },
        ]);
        let r = f.room(P, cos_lat(), EAST, 6.0, 8.0, &mut Vec::new());
        assert_eq!(r, Room { left: 0.0, right: 0.0 });
    }

    #[test]
    fn walls_on_both_sides_are_measured_independently() {
        let mut f = wall(P.y, 3.5, 6.89, 6.91);
        f.push_edge([
            Coord { x: 6.89, y: P.y - 2.5 / DEG_M },
            Coord { x: 6.91, y: P.y - 2.5 / DEG_M },
        ]);
        let r = f.room(P, cos_lat(), EAST, 6.0, 8.0, &mut Vec::new());
        assert!((r.left - 3.5).abs() < 1e-6, "left {}", r.left);
        assert!((r.right - 2.5).abs() < 1e-6, "right {}", r.right);
    }

    #[test]
    fn the_measurement_turns_with_the_road() {
        // The same wall, read by a centerline running north instead of east:
        // it is now ahead, not beside.
        let f = wall(P.y, 3.0, 6.89, 6.91);
        let r = f.room(P, cos_lat(), (0.0, 1.0), 6.0, 2.0, &mut Vec::new());
        assert_eq!(r, Room::open(6.0));
    }

    #[test]
    fn a_wall_at_an_angle_is_measured_at_its_nearest_point_in_the_window() {
        // Runs from 5 m north at the station to 1 m north 10 m east; only the
        // stretch inside the ±4 m window counts, whose nearest point is at
        // 4 m east and (5 - 0.4·4) = 3.4 m north.
        let mut f = Facades::empty();
        let east = 10.0 / (DEG_M * cos_lat());
        f.push_edge([
            Coord { x: P.x, y: P.y + 5.0 / DEG_M },
            Coord { x: P.x + east, y: P.y + 1.0 / DEG_M },
        ]);
        let r = f.room(P, cos_lat(), EAST, 6.0, 4.0, &mut Vec::new());
        assert!((r.left - 3.4).abs() < 1e-3, "left {}", r.left);
    }
}
