//! Invariant 4 at the kerb: "nothing floats and nothing is buried by accident",
//! measured where the asphalt and the drawn ground part company.
//!
//! **Why the kerb and not under the road.** This check began by sampling the
//! carriageway's interior against the terrain's triangles, which is where the
//! defect lived while ground was drawn under the asphalt: not a wrong *height*
//! — the road height field and the ground function agree to the centimetre
//! wherever a bench exists — but a wrong *surface*, two triangulations of the
//! same intent crossing each other between their shared vertices. Cutting the
//! terrain back to the kerb (docs/GROUND.md §3) removed the drawn ground under
//! the asphalt, and with it that instrument's whole population: `road − ground`
//! went from ~400 k samples to eighteen, not because the heights came right but
//! because there was nothing left to compare against.
//!
//! The gap did not vanish, it moved to the boundary, so that is where it is
//! measured. Two metrics, from one walk each:
//!
//! - [`contact.kerb_lip`] — how tall a wall the model implies, taken at every
//!   carriageway silhouette edge against the ground a metre outside it. Not a
//!   defect to drive to zero: a road on a real embankment has a real drop at its
//!   edge. It is the honest size of the earthwork the profile asked for, and the
//!   only thing left that can see a road standing on an embankment nobody built.
//! - [`contact.kerb_unwalled`] — how much of that wall is missing, walked along
//!   the terrain's own hole rim. This is the gate.

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// A kerb standing this far above the ground beside it is not a kerb. Real
/// ones run to about a quarter-metre and the boundary carries quantization and
/// a metre of probe offset on a cross-slope, so half a metre is comfortably
/// past what kerb-ness explains and far short of the metres a missing
/// retaining wall costs.
const LIP_M: f64 = 0.5;

/// How far, in plan, an apron may stand from the kerb edge it closes and still
/// count as standing on it. The apron sits on the silhouette the boundary edge
/// was derived from, so this only absorbs quantization and the midpoint offset
/// along a curving kerb.
const APRON_NEAR_M: f64 = 1.5;

/// How far an apron's span may fall short of the drop it is meant to close and
/// still count as closing it: the kerb probe stands a metre out on ground that
/// may be sloping, and both surfaces carry millimetre quantization.
const APRON_SLOP_M: f64 = 0.5;

/// How far outside the kerb, in metres, the ground is asked for. Far enough to
/// clear the rounding on the shared boundary vertices, near enough that it is
/// still the ground *at* the kerb and not the next thing along.
const LIP_PROBE_M: f64 = 1.0;

/// How far past an abutment the ground is followed, and in what steps, looking
/// for the wall the deck should have started on top of. Matched to
/// `synth::draped`'s own reach and step so the two see the same wall.
const SEAT_PROBE_M: f64 = 20.0;
const SEAT_STEP_M: f64 = 2.0;

/// The climb that makes ground beside an abutment a wall rather than a slope
/// the path walks up. `synth::draped`'s `WALL_GRADE`, restated here because
/// this check must be able to disagree with the generator: sharing the constant
/// by import would make a change to the rule silently move the measurement too.
const SEAT_WALL_GRADE: f64 = 0.6;

/// How far a fitted deck's abutment may stand below the wall beside it before
/// it reads as seated in the notch rather than on its bank. The seating rule
/// declines corrections under a metre, and one probe step of a 60 % wall is
/// another 1.2 m, so anything under about that is the rule working rather than
/// failing.
const SEAT_M: f64 = 2.0;

pub struct Contact {
    lip: Dist,
    lip_worst: Worst,
    unwalled: Dist,
    unwalled_worst: Worst,
    seat: Dist,
    seat_worst: Worst,
}

impl Contact {
    pub fn new(opt: &Options) -> Contact {
        Contact {
            lip: Dist::metres(),
            lip_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            unwalled: Dist::metres(),
            unwalled_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            seat: Dist::metres(),
            seat_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }

    /// Invariant 4 at a footbridge's abutment: how far the deck's lower end
    /// stands below the wall that runs up beside it.
    ///
    /// A fitted deck is chorded between the ground at its annotated ends
    /// (`synth::draped`). Where that end landed part way down a gorge wall the
    /// bridge starts in the riverbed it is supposed to cross, and the path
    /// dives in after it. The wall is what separates that from a bridge that
    /// correctly ends at the foot of a slope the path climbs: ground going up
    /// at a walkable grade is ground the path can be on, and only ground
    /// steeper than [`SEAT_WALL_GRADE`] is followed.
    ///
    /// Asked at the *lower* abutment, and bounded by the height of the higher
    /// one. Both halves are needed, and each excludes a whole population the
    /// other admits: without the wall test a footbridge landing at the foot of
    /// a slope reads as buried, and without the bound a level footbridge on a
    /// steep hillside — a hairpin path crossing its own gully — reads as
    /// twenty metres wrong because the mountain is above it, which it is
    /// supposed to be. What is left is the thing that is actually broken: a
    /// deck tilted because one of its ends fell down a wall, measured against
    /// the end that did not.
    fn visit_seats(&mut self, tile: &TileScene, terrain: &SurfaceMesh) {
        for deck in tile.roads.iter().filter(|r| r.is_fitted_deck()) {
            let Some([a, b]) = deck_ends(&deck.mesh) else { continue };
            // A deck clipped by the tile border has an "end" that is the
            // border; the tile that owns it answers for it.
            if !tile.owns(a.x, a.y) || !tile.owns(b.x, b.y) {
                continue;
            }
            let (low, high) = if a.top <= b.top { (a, b) } else { (b, a) };
            let (dx, dy) = (
                (low.x - high.x) * tile.scale.mx,
                (low.y - high.y) * tile.scale.my,
            );
            let len = dx.hypot(dy);
            if len < 1.0 {
                continue;
            }
            let (ux, uy) = (dx / len / tile.scale.mx, dy / len / tile.scale.my);
            // The ground *at* the abutment counts before any marching: a deck
            // whose end is already under the surface it is supposed to start on
            // is buried, and that needs no wall to be a defect.
            let mut prev = drawn_ground(tile, terrain, low.x, low.y).unwrap_or(low.top);
            let mut climbed = (prev - low.top).max(0.0);
            let mut d = SEAT_STEP_M;
            while d <= SEAT_PROBE_M {
                let Some(h) = drawn_ground(tile, terrain, low.x + ux * d, low.y + uy * d) else {
                    break;
                };
                if (h - prev) / SEAT_STEP_M < SEAT_WALL_GRADE {
                    break; // the climb relaxed: the wall, if any, ends here
                }
                climbed = climbed.max(h - low.top);
                prev = h;
                d += SEAT_STEP_M;
            }
            let v = climbed.clamp(0.0, high.top - low.top);
            self.seat.push(v);
            if v > SEAT_M {
                let (lon, lat) = tile.lonlat(low.x, low.y);
                self.seat_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: v,
                    note: format!(
                        "{} deck starts at {:.2} m against {:.2} m at its far end, with wall \
                         above it the whole way",
                        deck.class, low.top, high.top
                    ),
                });
            }
        }
    }
}

/// The drawn ground at a plan point, through the hole cut under the asphalt.
///
/// The terrain mesh stops at the kerb (docs/GROUND.md §3), so a probe that
/// walked onto a road would find nothing and stop — which is exactly where a
/// footbridge's bank tends to be, since the path arrives at a street. Where
/// there is no terrain the at-grade carriageway *is* the drawn ground.
fn drawn_ground(tile: &TileScene, terrain: &SurfaceMesh, px: f64, py: f64) -> Option<f64> {
    terrain.height_at(px, py).or_else(|| {
        tile.roads
            .iter()
            .filter(|r| r.is_pavement() || r.is_casing())
            .find_map(|r| r.mesh.height_at(px, py))
    })
}

/// One end of a deck: a point on its centerline and the deck top there.
struct DeckEnd {
    x: f64,
    y: f64,
    top: f64,
}

/// A deck's two ends, in unit plan space.
///
/// The long axis is taken as the vertex furthest from the centroid and the
/// vertex furthest from *that*, which for a swept slab is a pair of opposite
/// end corners. Each end is then the centroid of the vertices in the outermost
/// tenth of the projection onto that axis, so the point sits on the deck's
/// centerline rather than on a corner — a corner query lands on the boundary,
/// where the terrain lookup is a coin toss.
fn deck_ends(m: &SurfaceMesh) -> Option<[DeckEnd; 2]> {
    let n = m.vertex_count();
    if n < 3 {
        return None;
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let (x, y, _) = m.vertex(i);
        cx += x / n as f64;
        cy += y / n as f64;
    }
    let far = |from: (f64, f64)| {
        (0..n).fold(((0.0, 0.0), -1.0), |best, i| {
            let (x, y, _) = m.vertex(i);
            let d = (x - from.0).hypot(y - from.1);
            if d > best.1 {
                ((x, y), d)
            } else {
                best
            }
        })
        .0
    };
    let a = far((cx, cy));
    let b = far(a);
    let (ax, ay) = (b.0 - a.0, b.1 - a.1);
    let len2 = ax * ax + ay * ay;
    if len2 <= 0.0 {
        return None;
    }
    let t_of = |i: usize| {
        let (x, y, _) = m.vertex(i);
        ((x - a.0) * ax + (y - a.1) * ay) / len2
    };
    let end = |near_zero: bool| {
        let (mut sx, mut sy, mut top, mut k) = (0.0, 0.0, f64::NEG_INFINITY, 0usize);
        for i in 0..n {
            let t = t_of(i);
            if (near_zero && t <= 0.1) || (!near_zero && t >= 0.9) {
                let (x, y, z) = m.vertex(i);
                sx += x;
                sy += y;
                top = top.max(z);
                k += 1;
            }
        }
        (k > 0).then(|| DeckEnd { x: sx / k as f64, y: sy / k as f64, top })
    };
    Some([end(true)?, end(false)?])
}

impl Check for Contact {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        let Some(terrain) = &tile.terrain else { return };
        self.visit_seats(tile, terrain);
        // The kerb lip: at every silhouette edge of the carriageway, the road's
        // own height against the ground a metre outside it.
        //
        // A few centimetres is a kerb. Fifteen metres is a retaining wall the
        // model implies and does not draw — a hole you can see the hillside
        // through.
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            for (a, b, opp) in road.mesh.boundary_edges() {
                let (ax, ay, az) = road.mesh.vertex(a);
                let (bx, by, bz) = road.mesh.vertex(b);
                let (ox, oy, _) = road.mesh.vertex(opp);
                let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
                if !tile.owns(mx, my) {
                    continue;
                }
                // Outward is away from the one triangle holding this edge.
                let (dx, dy) = (mx - ox, my - oy);
                let len = tile.scale.dist(0.0, 0.0, dx, dy);
                if len <= 0.0 {
                    continue;
                }
                let (px, py) = (
                    mx + dx / len * LIP_PROBE_M,
                    my + dy / len * LIP_PROBE_M,
                );
                let Some(gz) = terrain.height_at(px, py) else { continue };
                let kerb_z = (az + bz) * 0.5;
                let v = kerb_z - gz;
                self.lip.push(v);
                if v > LIP_M {
                    let (lon, lat) = tile.lonlat(mx, my);
                    self.lip_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: v,
                        note: format!("kerb {kerb_z:.2} m, ground {gz:.2} m a metre outside it"),
                    });
                }
            }
        }
        // Watertightness, asked of the hole's own rim.
        //
        // The asphalt's interior mesh stops an inset short of its silhouette
        // and the terrain's hole is cut *at* the silhouette, so the two
        // boundaries are 35 cm apart and no query anchored on one finds the
        // other. Anchoring on the terrain's rim instead makes it structural:
        // every terrain boundary edge that is not the tile's own edge is a hole
        // rim, the asphalt (interior or casing) answers for the road's height
        // over it, and anything between the two heights that no apron spans is
        // a gap you can see through.
        for (a, b, _) in terrain.boundary_edges() {
            let (ax, ay, az) = terrain.vertex(a);
            let (bx, by, bz) = terrain.vertex(b);
            let (mx, my) = ((ax + bx) * 0.5, (ay + by) * 0.5);
            if !tile.owns(mx, my) {
                continue;
            }
            // The tile's own edge is not a hole: the neighbour's terrain
            // continues across it.
            let on_edge = |v: f64| v.abs() < 1e-6 || (v - 1.0).abs() < 1e-6;
            if on_edge(mx) || on_edge(my) {
                continue;
            }
            let rim_z = (az + bz) * 0.5;
            let Some(road_z) = tile
                .roads
                .iter()
                .filter(|r| r.is_pavement() || r.is_casing())
                .filter_map(|r| r.mesh.height_at(mx, my))
                .next()
            else {
                continue; // no asphalt over it: not the hole's rim
            };
            let gap = (road_z - rim_z).abs();
            let (lo_z, hi_z) = (road_z.min(rim_z), road_z.max(rim_z));
            let walled = gap <= LIP_M
                || tile
                    .roads
                    .iter()
                    .filter(|r| r.is_apron())
                    .filter_map(|r| r.mesh.span_near(mx, my, &tile.scale, APRON_NEAR_M))
                    .any(|(lo, hi)| hi >= hi_z - APRON_SLOP_M && lo <= lo_z + APRON_SLOP_M);
            self.unwalled.push(if walled { 0.0 } else { gap });
            if !walled {
                let (lon, lat) = tile.lonlat(mx, my);
                self.unwalled_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: gap,
                    note: format!(
                        "asphalt {road_z:.2} m, terrain rim {rim_z:.2} m, nothing between them"
                    ),
                });
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let skipped = || {
            "no at-grade road surface at this zoom (below ROAD_SURFACE_MIN_ZOOM, or the archive \
             carries no DEM)"
                .to_string()
        };
        vec![
            Metric {
                id: "contact.kerb_unwalled".into(),
                invariant: Invariant::I4,
                title: "Gap at the hole's rim with nothing spanning it".into(),
                population: format!(
                    "Every terrain-mesh boundary edge midpoint that is not on the tile's own \
                     edge and has at-grade asphalt or casing over it. A cut edge carries no \
                     apron by design and is excluded with the tile edge. The apron is vertical, \
                     so its span is asked within {APRON_NEAR_M:.1} m of the rim rather than at \
                     it, with {APRON_SLOP_M:.1} m of slack at each end."
                ),
                detail: "Watertightness, walked along the terrain's own hole rim: at every \
                         terrain boundary edge that is not the tile's edge, the asphalt's height \
                         over it against the terrain's, where no apron spans the difference. \
                         Anchored on the rim rather than on the asphalt because the two \
                         boundaries are an inset apart, so a query anchored on one never finds \
                         the other — and asked at the same point rather than a metre out, \
                         because a cutting's terrain rises steeply but perfectly continuously \
                         and there is nothing to see through."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: LIP_M,
                skipped: self.unwalled.is_empty().then(skipped),
                dist: self.unwalled,
                worst: self.unwalled_worst.into_vec(),
            },
            Metric {
                id: "contact.kerb_lip".into(),
                invariant: Invariant::I4,
                title: "Drop from the kerb to the ground beside it".into(),
                population: format!(
                    "Every silhouette (boundary) edge midpoint of every level-0 road_surface \
                     mesh inside the tile proper, probed {LIP_PROBE_M:.0} m along the outward \
                     normal. Structures are excluded: a deck edge is not a kerb."
                ),
                detail: format!(
                    "Carriageway edge height minus the drawn ground {LIP_PROBE_M:.0} m outside \
                     it. With the ground cut back to the kerb this is where the road and the \
                     terrain part company, and it is the only place left that can see a road \
                     standing on an embankment nobody built. Not a defect to drive to zero: it \
                     is the honest height of the earthwork the profile asked for, and past \
                     {LIP_M:.2} m the model implies a retaining wall it must also draw \
                     (`contact.kerb_unwalled` is how much of it is missing)."
                ),
                sense: Sense::HigherIsWorse,
                threshold: LIP_M,
                skipped: self.lip.is_empty().then(skipped),
                dist: self.lip,
                worst: self.lip_worst.into_vec(),
            },
            Metric {
                id: "contact.deck_seat".into(),
                invariant: Invariant::I4,
                title: "Wall standing over a footbridge's lower abutment".into(),
                population: format!(
                    "The lower abutment of every *fitted* deck — a draped class carrying an \
                     elevated span, whose deck is chorded to the ground rather than solved \
                     (`synth::draped`) — lying wholly inside the tile proper. Followed \
                     {SEAT_PROBE_M:.0} m outward along the deck's own axis in \
                     {SEAT_STEP_M:.0} m steps, and only while the ground climbs at \
                     {:.0} % or more. Three coverage limits: a deck clipped by the tile \
                     border is left to the tile that owns it; a span whose abutment stands on \
                     a wall the drawn terrain mesh is too coarse to resolve reads lower than \
                     it is; and a span with *both* abutments down in the notch reads zero, \
                     because the bound is its own far end and there is nothing in the archive \
                     that says how high the path beyond it goes.",
                    SEAT_WALL_GRADE * 100.0
                ),
                detail: format!(
                    "How far a footbridge's lower abutment stands below the wall beside it, \
                     bounded by its own far end. A fitted deck's ends come from the ground at \
                     the annotated span's edges, and against a near-vertical wall two metres \
                     of plan disagreement between the annotation and the DEM is a dozen metres \
                     of height: the bridge begins part way down the gorge it crosses and the \
                     path dives in after it. Ground under {:.0} % is a slope the path walks \
                     rather than a wall it cannot, and the bound keeps a level footbridge on a \
                     hillside from scoring the hillside. Past {SEAT_M:.1} m the deck is tilted \
                     into the notch instead of sitting on its bank.",
                    SEAT_WALL_GRADE * 100.0
                ),
                sense: Sense::HigherIsWorse,
                threshold: SEAT_M,
                skipped: self.seat.is_empty().then(|| {
                    "no fitted decks at this zoom — the extract carries no draped feature with \
                     an elevated span, or the archive carries no DEM"
                        .to_string()
                }),
                dist: self.seat,
                worst: self.seat_worst.into_vec(),
            },
        ]
    }
}

/// Signed clearance of one surface over another at a plan point, for callers
/// that already hold both. Kept here so the sign convention has one home.
pub fn clearance_over(upper: &SurfaceMesh, lower: &SurfaceMesh, px: f64, py: f64) -> Option<f64> {
    let u = upper.height_range_at(px, py)?.0;
    let l = lower.height_range_at(px, py)?.1;
    Some(u - l)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::Scale;
    use crate::verify::scene::RoadMesh;
    use std::collections::HashMap;

    const LIP: &str = "contact.kerb_lip";
    const UNWALLED: &str = "contact.kerb_unwalled";

    fn quad(x0: f32, x1: f32, z: f32) -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![x0, x1, x1, x0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![z; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    /// A vertical wall along `x`, spanning `lo..hi` — the shape `build_apron`
    /// emits: plan-degenerate, so only a `span_near` query can see it.
    fn wall(x: f32, lo: f32, hi: f32) -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![x, x, x, x],
            vec![0.0, 1.0, 1.0, 0.0],
            vec![hi, hi, lo, lo],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    /// The shape the hole leaves: asphalt over `x < 0.5` at `road_m`, ground
    /// starting exactly at the kerb and lying at `ground_m` outside it, plus
    /// whatever `extra` features stand between them.
    fn kerbed(road_m: f32, ground_m: f32, extra: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        let mut roads = vec![RoadMesh {
            class: "road_surface".into(),
            level: 0,
            mesh: quad(0.0, 0.5, road_m),
        }];
        roads.extend(extra);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(quad(0.5, 1.0, ground_m)),
            roads,
            lines: Vec::new(),
        }
    }

    /// Metrics by id, so a test names what it asserts on and adding a metric
    /// does not silently repoint every index in the module.
    fn run(tile: &TileScene) -> HashMap<String, Metric> {
        let opt = Options { spacing_m: 1.0, ..Default::default() };
        let mut c = Box::new(Contact::new(&opt));
        c.visit(tile, &opt);
        c.finish().into_iter().map(|m| (m.id.clone(), m)).collect()
    }

    #[test]
    fn a_kerb_standing_on_nothing_is_caught_as_a_lip() {
        // Asphalt ten metres above the ground its kerb meets. The lip reports
        // the wall's height in metres, not a ratio — that is what makes it
        // readable as "the model implies a ten-metre retaining wall here".
        let m = run(&kerbed(110.0, 100.0, vec![]));
        let lip = &m[LIP];
        assert!(lip.violations() > 0, "a 10 m drop at the kerb must be caught");
        assert!(
            (lip.worst_value().unwrap() - 10.0).abs() < 0.5,
            "the wall's height, not a ratio: {:?}",
            lip.worst_value()
        );
    }

    #[test]
    fn a_road_level_with_the_ground_beside_it_has_no_lip() {
        // The common case, and the one that must not be reported: a bench holds
        // and the two surfaces meet at the kerb.
        let m = run(&kerbed(100.0, 100.0, vec![]));
        assert!(!m[LIP].dist.is_empty(), "the kerb must actually be walked");
        assert_eq!(m[LIP].violations(), 0, "a flush kerb is not a lip");
        assert_eq!(m[UNWALLED].violations(), 0, "and there is nothing to wall");
    }

    #[test]
    fn a_lip_with_no_apron_over_it_is_unwalled() {
        // The gate. The same ten-metre drop, with nothing drawn between the
        // asphalt and the terrain's rim: a hole you can see the hillside
        // through.
        let m = run(&kerbed(110.0, 100.0, vec![]));
        let unwalled = &m[UNWALLED];
        assert!(unwalled.violations() > 0, "an unspanned 10 m gap must be caught");
        assert!(
            (unwalled.worst_value().unwrap() - 10.0).abs() < 0.5,
            "the gap's own height: {:?}",
            unwalled.worst_value()
        );
        assert!(!unwalled.worst.is_empty(), "a violation must name a place");
    }

    #[test]
    fn an_apron_spanning_the_drop_closes_it() {
        // The same scene with the wall drawn. The lip is unchanged — the
        // earthwork is still ten metres tall, and that is honest — but nothing
        // is unwalled, which is the property the apron exists to give.
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, mesh: wall(0.5, 100.0, 110.0) };
        let m = run(&kerbed(110.0, 100.0, vec![apron]));
        assert!(m[LIP].violations() > 0, "the lip is a fact about the model, not a defect");
        assert_eq!(m[UNWALLED].violations(), 0, "the apron spans it, so nothing is unwalled");
    }

    #[test]
    fn an_apron_too_short_for_the_drop_does_not_close_it() {
        // A wall covering the top three metres of a ten-metre drop leaves seven
        // metres of sky. Spanning *part* of the gap must not count as spanning
        // it, or a truncated apron reads as a closed one.
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, mesh: wall(0.5, 107.0, 110.0) };
        let m = run(&kerbed(110.0, 100.0, vec![apron]));
        assert!(m[UNWALLED].violations() > 0, "a partial wall does not close the gap");
    }

    #[test]
    fn a_road_in_a_cutting_is_walled_the_other_way_round() {
        // Both directions. A road below the ground it is cut into opens the same
        // gap with the sign reversed, and the apron closes it the same way —
        // which is the case that only appeared once the carriageway stopped
        // being clamped up to the terrain (`road::on_ground`).
        let m = run(&kerbed(100.0, 108.0, vec![]));
        assert!(m[UNWALLED].violations() > 0, "a cutting leaves the same open gap");
        let apron =
            RoadMesh { class: "road_apron".into(), level: 0, mesh: wall(0.5, 100.0, 108.0) };
        let m = run(&kerbed(100.0, 108.0, vec![apron]));
        assert_eq!(m[UNWALLED].violations(), 0, "and the same apron closes it");
    }

    #[test]
    fn a_tile_without_terrain_is_skipped_rather_than_scored_clean() {
        let mut tile = kerbed(110.0, 100.0, vec![]);
        tile.terrain = None;
        let m = run(&tile);
        assert!(m[LIP].skipped.is_some(), "no ground must read as skipped, not as passing");
        assert_eq!(m[LIP].violations(), 0);
    }

    #[test]
    fn buffer_geometry_is_the_neighbours_business() {
        // A road running well past the tile edge must contribute only the part
        // inside the tile proper, or every border defect is counted twice.
        let b = Bounds::of_tile(16, 34000, 23000);
        let tile = TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(quad(0.5, 1.5, 100.0)),
            roads: vec![RoadMesh {
                class: "road_surface".into(),
                level: 0,
                mesh: quad(-0.4, 0.5, 110.0),
            }],
            lines: Vec::new(),
        };
        let m = run(&tile);
        // The kerb at x = 0.5 is inside the tile and reported; the outer edge at
        // x = -0.4 is the neighbour's.
        assert!(m[LIP].violations() > 0, "the interior kerb must be walked");
        for o in &m[LIP].worst {
            let (lon, _) = (o.lon, o.lat);
            assert!(lon >= b.west && lon <= b.east, "offender at {lon} is outside the tile");
        }
    }

    /// A terrain profile as a strip of quads along `x`, from `(x, height)`
    /// breakpoints — the ground a deck's abutment is judged against.
    fn ground_strip(pts: &[(f32, f32)]) -> SurfaceMesh {
        let (mut x, mut y, mut z, mut idx) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (i, &(px, pz)) in pts.iter().enumerate() {
            x.extend([px, px]);
            y.extend([-1.0, 2.0]);
            z.extend([pz, pz]);
            if i > 0 {
                let b = (i as u32 - 1) * 2;
                idx.extend([b, b + 1, b + 3, b, b + 3, b + 2]);
            }
        }
        SurfaceMesh::from_parts(x, y, z, idx).unwrap()
    }

    /// A deck: a slab from `x0` to `x1` with its top running `z0` to `z1`.
    fn deck(class: &str, x0: f32, x1: f32, z0: f32, z1: f32) -> RoadMesh {
        RoadMesh {
            class: class.into(),
            level: 1,
            mesh: SurfaceMesh::from_parts(
                vec![x0, x1, x1, x0],
                vec![0.4, 0.4, 0.6, 0.6],
                vec![z0, z1, z1, z0],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap(),
        }
    }

    fn with_ground(terrain: SurfaceMesh, roads: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads,
            lines: Vec::new(),
        }
    }

    const SEAT: &str = "contact.deck_seat";

    /// The defect: a footbridge whose lower end fell down a gorge wall, so it
    /// starts part way into what it is supposed to cross. The tile is ~600 m
    /// wide, so 0.02 of unit space is about 12 m.
    #[test]
    fn a_deck_that_starts_down_a_wall_is_caught() {
        // Bank at 435 to x = 0.30, a wall to the riverbed at 415, far bank at
        // 435. The deck runs from 421 — part way down the near wall — to 435.
        let terrain = ground_strip(&[
            (0.0, 435.0),
            (0.30, 435.0),
            (0.33, 415.0),
            (0.37, 415.0),
            (0.40, 435.0),
            (1.0, 435.0),
        ]);
        let m = run(&with_ground(terrain, vec![deck("footway", 0.32, 0.40, 421.0, 435.0)]));
        assert!(m[SEAT].violations() > 0, "an abutment 14 m down a wall must be caught");
        assert!(
            m[SEAT].worst_value().unwrap() > 5.0,
            "the wall above it is metres, not centimetres: {:?}",
            m[SEAT].worst_value()
        );
    }

    /// The same bridge, seated: both ends on their banks, the deck level over
    /// the gorge. Nothing to report, though the ground under mid-span is 20 m
    /// below the deck — which is what a bridge is.
    #[test]
    fn a_deck_seated_on_its_banks_is_clean() {
        let terrain = ground_strip(&[
            (0.0, 435.0),
            (0.30, 435.0),
            (0.33, 415.0),
            (0.37, 415.0),
            (0.40, 435.0),
            (1.0, 435.0),
        ]);
        let m = run(&with_ground(terrain, vec![deck("footway", 0.29, 0.41, 435.0, 435.0)]));
        assert_eq!(m[SEAT].violations(), 0, "a level deck bank to bank is not a defect");
    }

    /// A level footbridge on a mountainside keeps its clean bill: the ground
    /// climbing away above it is the mountain, and the deck's own far end —
    /// level with the near one — is the bound that says so.
    #[test]
    fn a_level_deck_on_a_hillside_does_not_score_the_hillside() {
        let terrain = ground_strip(&[(0.0, 560.0), (0.5, 500.0), (1.0, 440.0)]);
        let m = run(&with_ground(terrain, vec![deck("path", 0.48, 0.52, 502.4, 502.4)]));
        assert_eq!(
            m[SEAT].violations(),
            0,
            "a 100 % hillside above a level deck is terrain, not a defect: {:?}",
            m[SEAT].worst_value()
        );
    }

    /// A solved deck is not this check's business: its abutments come from a
    /// profile with anchors and a grade ceiling, not from the ground at an
    /// annotation's edge.
    #[test]
    fn a_solved_deck_is_not_in_the_population() {
        let terrain = ground_strip(&[(0.0, 435.0), (0.30, 435.0), (0.33, 415.0), (1.0, 415.0)]);
        let m = run(&with_ground(terrain, vec![deck("motorway", 0.32, 0.40, 421.0, 435.0)]));
        assert!(m[SEAT].skipped.is_some(), "no fitted decks here: {:?}", m[SEAT].dist.count());
    }
}
