//! Invariant 3 (vertical order) and the flying half of invariant 4 (nothing
//! floats and nothing is buried by accident), measured between the emitted
//! solids.
//!
//! The solver already proves it satisfied the clearance inequalities it was
//! given; `solve::consistency` reports the shortfall as zero. What it cannot
//! see is whether the *swept solids* honour it — a deck is a slab with a
//! soffit, a bore is a tube with a roof, and the gap the eye reads is between
//! those faces, not between the two centrelines the constraint graph shared.
//!
//! ## Why the class gap is not measured here
//!
//! The obvious check — "is the soffit 5 m above the road it crosses" — cannot
//! be posed from the archive, and the first version of this module got it
//! wrong. The at-grade carriageway is one unioned region, so a deck sample that
//! finds asphalt beneath it has no way to know whether that asphalt is a road
//! being *crossed* or the deck's own approach it is about to *join*. Measured
//! anyway, the population came out plainly bimodal: 23 % of samples sat within
//! half a metre of the carriageway — abutments, where the deck meets the road
//! at grade and owes it nothing — and every one was counted as a 5 m shortfall.
//! The metric read 36 % violations and was measuring touchdowns.
//!
//! The clearance inequality belongs at stage 2, where the crossing set is
//! *known* rather than inferred from plan overlap, and it is already measured
//! there as `consistency.max_clearance_violation_m`. What the archive can
//! answer without ambiguity is the weaker, prior-free half of invariant 3: the
//! **level ordering**. A level-1 structure below a level-0 surface is wrong
//! whether they cross or merge, so that is what is measured.
//!
//! ## Where the thresholds come from
//!
//! Each of these has a legitimate contact band that a naive zero threshold
//! would report as a defect, so the thresholds are read off the measured
//! distributions rather than assumed:
//!
//! - a deck touching down at its abutment has its soffit a deck-thickness
//!   below the ground, so burial only becomes a finding past that;
//! - a bore's roof crosses the ground surface at the portal mouth by design,
//!   so cover only becomes a finding once the tube is clear of it;
//! - a deck meeting the road at grade sits level with it, so ordering only
//!   becomes a finding once the structure is properly underneath.

use crate::priors::DECK_THICKNESS_M;
use crate::verify::dist::Dist;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// A deck within this of the carriageway is touching down on it, not crossing
/// under it. Half a metre: the measured touchdown spike spans about ±0.5 m.
const TOUCHDOWN_M: f64 = -0.5;

/// A soffit this far below the drawn ground is buried deeper than an abutment
/// can explain — the deck's own thickness, plus half a metre of slack for the
/// approach ramp's last cell.
const BURIED_DECK_M: f64 = -(DECK_THICKNESS_M + 0.5);

/// A bore roof this far above the drawn ground is out in the open air rather
/// than emerging at a portal mouth.
const EXPOSED_BORE_M: f64 = -1.0;

/// Two at-grade surface bands further apart than this are a drawn
/// impossibility: at grade means *on the ground*, and there is one ground.
/// Below it sit the pairs the sheets machinery legitimately layers — split
/// carriageways on a cross-slope, a kerb-height braid — and
/// `solve::crossings::SEPARATION_M` already encodes the same boundary from the
/// other side: within 3 m two alignments are a braid, never a grade
/// separation. Past it, one band is standing in the air over the other with no
/// structure under it, which is how a formation whose mapped bore stopped
/// short of a crossing looks — the buried tail is paved as open cut and slides
/// beneath the crossing feature's band (the Collonge funicular over the
/// rack railway's portal was drawn exactly so).
const GRADE_STACK_M: f64 = 3.0;

pub struct Clearance {
    /// Plan sample spacing, kept so each metric can state its own population.
    spacing_m: f64,
    order: Dist,
    order_worst: Worst,
    over_ground: Dist,
    over_ground_worst: Worst,
    bore_cover: Dist,
    bore_cover_worst: Worst,
    stack: Dist,
    stack_worst: Worst,
}

impl Clearance {
    pub fn new(opt: &Options) -> Clearance {
        Clearance {
            spacing_m: opt.spacing_m,
            order: Dist::metres(),
            order_worst: Worst::new(Sense::LowerIsWorse, opt.worst_k),
            over_ground: Dist::metres(),
            over_ground_worst: Worst::new(Sense::LowerIsWorse, opt.worst_k),
            bore_cover: Dist::metres(),
            bore_cover_worst: Worst::new(Sense::LowerIsWorse, opt.worst_k),
            stack: Dist::metres(),
            stack_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }
}

impl Check for Clearance {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        for deck in tile.roads.iter().filter(|r| r.is_deck()) {
            deck.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((soffit, top)) = deck.mesh.height_range_at(px, py) else { return };

                // Level ordering: an elevated structure must not be under the
                // at-grade surface. Its own running surface, not its soffit —
                // the soffit is legitimately below the road it merges onto.
                let under = tile
                    .roads
                    .iter()
                    .filter(|r| r.is_pavement())
                    .filter_map(|r| r.mesh.height_range_at(px, py))
                    .map(|(_, hi)| hi)
                    .fold(f64::NEG_INFINITY, f64::max);
                if under.is_finite() {
                    let v = top - under;
                    self.order.push(v);
                    if v < TOUCHDOWN_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.order_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: v,
                            note: format!(
                                "{} L{} surface {top:.2} m under at-grade asphalt {under:.2} m",
                                deck.class, deck.level
                            ),
                        });
                    }
                }

                // Against the drawn ground: a deck below it is buried, which no
                // amount of solved clearance would show.
                if let Some(terrain) = &tile.terrain {
                    if let Some(gz) = terrain.height_at(px, py) {
                        let v = soffit - gz;
                        self.over_ground.push(v);
                        if v < BURIED_DECK_M {
                            let (lon, lat) = tile.lonlat(px, py);
                            self.over_ground_worst.offer(Offender {
                                lon,
                                lat,
                                zoom: tile.z,
                                value: v,
                                note: format!(
                                    "{} L{} soffit {soffit:.2} m, drawn ground {gz:.2} m",
                                    deck.class, deck.level
                                ),
                            });
                        }
                    }
                }
            });
        }

        // Two at-grade bands at one plan point: each stacked pair is measured
        // once, from its upper side. Border-vertex coincidences are already
        // `order.at_grade_overlap`; this is the whole-mesh interior that check's
        // own doc note says it cannot see.
        for (i, a) in tile.roads.iter().enumerate() {
            if !a.is_pavement() {
                continue;
            }
            a.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((_, own)) = a.mesh.height_range_at(px, py) else { return };
                let mut below = f64::NEG_INFINITY;
                let mut under_class = "";
                for (j, b) in tile.roads.iter().enumerate() {
                    if j == i || !b.is_pavement() {
                        continue;
                    }
                    let Some((_, top)) = b.mesh.height_range_at(px, py) else { continue };
                    if top <= own && top > below {
                        below = top;
                        under_class = &b.class;
                    }
                }
                if below.is_finite() {
                    let v = own - below;
                    self.stack.push(v);
                    if v > GRADE_STACK_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.stack_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: v,
                            note: format!(
                                "at-grade {} at {own:.2} m over at-grade {under_class} at \
                                 {below:.2} m, nothing between them",
                                a.class
                            ),
                        });
                    }
                }
            });
        }

        let Some(terrain) = &tile.terrain else { return };
        for bore in tile.roads.iter().filter(|r| r.is_bore()) {
            bore.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((_, roof)) = bore.mesh.height_range_at(px, py) else { return };
                let Some(gz) = terrain.height_at(px, py) else { return };
                let v = gz - roof;
                self.bore_cover.push(v);
                if v < EXPOSED_BORE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    self.bore_cover_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: v,
                        note: format!(
                            "{} L{} roof {roof:.2} m, drawn ground {gz:.2} m",
                            bore.class, bore.level
                        ),
                    });
                }
            });
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let none = |d: &Dist, what: &str| {
            d.is_empty().then(|| format!("no {what} in the tiles measured"))
        };
        vec![
            Metric {
                id: "order.deck_above_carriageway".into(),
                invariant: Invariant::I3,
                title: "Elevated structure above the at-grade asphalt".into(),
                population: format!(
                    "Structure surface samples at {:.1} m plan spacing where at-grade asphalt \
                     shares the plan position, in the tile proper. Excludes decks over bare \
                     ground, which have nothing to be ordered against.",
                    self.spacing_m
                ),
                detail: "Deck running surface minus the at-grade carriageway sharing its plan \
                         position. Negative past the touchdown band means a level-1 structure is \
                         drawn underneath a level-0 one — the ordinal ordering inverted, which \
                         is wrong whether the two cross or merge. The class clearance gap is \
                         measured at stage 2 (consistency.max_clearance_violation_m), where the \
                         crossing set is known instead of inferred."
                    .into(),
                sense: Sense::LowerIsWorse,
                threshold: TOUCHDOWN_M,
                skipped: none(&self.order, "deck sharing plan with at-grade asphalt"),
                dist: self.order,
                worst: self.order_worst.into_vec(),
            },
            Metric {
                id: "clearance.deck_over_ground".into(),
                invariant: Invariant::I4,
                title: "Bridge soffit above the drawn ground".into(),
                population: format!(
                    "Every deck (level > 0) surface sample at {:.1} m plan spacing in the tile \
                     proper, against the terrain mesh of the same tile.",
                    self.spacing_m
                ),
                detail: format!(
                    "Deck underside minus the terrain mesh beneath it. A deck touching down at \
                     its abutment sits a deck-thickness low by construction, so the threshold is \
                     {BURIED_DECK_M:.1} m; past it the deck ploughs into a hillside it is \
                     supposed to fly over."
                ),
                sense: Sense::LowerIsWorse,
                threshold: BURIED_DECK_M,
                skipped: none(&self.over_ground, "deck over drawn ground"),
                dist: self.over_ground,
                worst: self.over_ground_worst.into_vec(),
            },
            Metric {
                id: "clearance.bore_cover".into(),
                invariant: Invariant::I4,
                title: "Ground cover over the tunnel roof".into(),
                population: format!(
                    "Every bore (level < 0) surface sample at {:.1} m plan spacing in the tile \
                     proper, against the terrain mesh of the same tile.",
                    self.spacing_m
                ),
                detail: format!(
                    "Terrain mesh minus the bore roof. The roof crosses the surface at a portal \
                     mouth by design, so the threshold is {EXPOSED_BORE_M:.1} m; past it the tube \
                     is in open air. Expected to fire: the roof-cover clamp was deliberately \
                     dropped for the constant-section tube, so this metric exists to keep an \
                     accepted cost from drifting."
                ),
                sense: Sense::LowerIsWorse,
                threshold: EXPOSED_BORE_M,
                skipped: none(&self.bore_cover, "tunnel bore under drawn ground"),
                dist: self.bore_cover,
                worst: self.bore_cover_worst.into_vec(),
            },
            Metric {
                id: "order.grade_stack".into(),
                invariant: Invariant::I3,
                title: "At-grade band standing over another at-grade band".into(),
                population: format!(
                    "Surface samples of every level-0 band (carriageway or rail formation) at \
                     {:.1} m plan spacing in the tile proper, where another level-0 band lies \
                     at or below the same plan position. Each stacked pair is measured once, \
                     from its upper side. Abutting regions meet at zero and sit in the \
                     population's floor.",
                    self.spacing_m
                ),
                detail: format!(
                    "Vertical separation between two at-grade surface bands at one plan point. \
                     At grade means on the ground, and there is one ground: past \
                     {GRADE_STACK_M:.1} m — the same boundary crossings::SEPARATION_M draws \
                     from the model side — the upper band is in the air over the lower with no \
                     structure between them. The class this exists to keep dead: a mapped \
                     bore's still-buried tail paved as open cut, sliding beneath the band of \
                     the feature that crosses just past its portal."
                ),
                sense: Sense::HigherIsWorse,
                threshold: GRADE_STACK_M,
                skipped: none(&self.stack, "two at-grade bands sharing a plan position"),
                dist: self.stack,
                worst: self.stack_worst.into_vec(),
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::{Scale, SurfaceMesh};
    use crate::verify::scene::RoadMesh;

    /// A flat slab spanning the middle of the tile: `top` at its running
    /// surface, `top - thickness` at its soffit, both faces present.
    fn slab(top: f32, thickness: f32) -> SurfaceMesh {
        let (x, y) = (vec![0.3, 0.7, 0.7, 0.3], vec![0.4, 0.4, 0.6, 0.6]);
        let mut xs = x.clone();
        xs.extend(x);
        let mut ys = y.clone();
        ys.extend(y);
        let mut zs = vec![top; 4];
        zs.extend(vec![top - thickness; 4]);
        SurfaceMesh::from_parts(xs, ys, zs, vec![0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7]).unwrap()
    }

    fn flat(z: f32) -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![z; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap()
    }

    fn tile(roads: Vec<RoadMesh>, terrain: Option<SurfaceMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain,
            roads,
            lines: Vec::new(),
        }
    }

    fn run(t: &TileScene) -> Vec<Metric> {
        let opt = Options { spacing_m: 3.0, ..Default::default() };
        let mut c = Box::new(Clearance::new(&opt));
        c.visit(t, &opt);
        c.finish()
    }

    #[test]
    fn a_deck_meeting_the_road_at_grade_is_not_a_violation() {
        // The regression the first version of this module shipped: an abutment,
        // where the deck surface sits level with the road it joins. It owes
        // that road no clearance and must not be scored as if it did.
        let t = tile(
            vec![
                RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) },
                RoadMesh { class: "motorway".into(), level: 1, band: String::new(), mesh: slab(100.0, 1.5) },
            ],
            Some(flat(100.0)),
        );
        let m = run(&t);
        assert_eq!(m[0].violations(), 0, "touchdown is not an ordering inversion");
        assert_eq!(m[1].violations(), 0, "a soffit one thickness low is a touchdown");
    }

    #[test]
    fn a_deck_flying_over_the_road_is_not_a_violation_either() {
        let t = tile(
            vec![
                RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) },
                RoadMesh { class: "motorway".into(), level: 1, band: String::new(), mesh: slab(107.0, 1.5) },
            ],
            Some(flat(100.0)),
        );
        let m = run(&t);
        assert_eq!(m[0].violations(), 0);
        assert!((m[0].worst_value().unwrap() - 7.0).abs() < 1e-3);
    }

    #[test]
    fn an_elevated_structure_under_the_at_grade_asphalt_is_caught() {
        // A footbridge drawn below the road: the level ordinal says it is above,
        // the geometry says otherwise.
        let t = tile(
            vec![
                RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) },
                RoadMesh { class: "path".into(), level: 1, band: String::new(), mesh: slab(97.5, 1.5) },
            ],
            Some(flat(90.0)),
        );
        let m = run(&t);
        assert!(m[0].violations() > 0);
        assert!((m[0].worst_value().unwrap() + 2.5).abs() < 1e-3);
        assert!(!m[0].worst.is_empty());
    }

    #[test]
    fn a_deck_ploughing_into_the_hillside_is_caught_past_its_own_thickness() {
        // Ground at 110 m, soffit at 105 m: 5 m under, far past a touchdown.
        let t = tile(
            vec![RoadMesh { class: "motorway".into(), level: 1, band: String::new(), mesh: slab(106.5, 1.5) }],
            Some(flat(110.0)),
        );
        let m = run(&t);
        assert!(m[1].violations() > 0);
        assert!((m[1].worst_value().unwrap() + 5.0).abs() < 1e-3);
    }

    #[test]
    fn a_bore_in_the_open_air_is_caught_but_a_portal_mouth_is_not() {
        // Roof 3 m above ground: a tube in daylight.
        let t = tile(
            vec![RoadMesh { class: "motorway".into(), level: -1, band: String::new(), mesh: slab(103.0, 5.0) }],
            Some(flat(100.0)),
        );
        assert!(run(&t)[2].violations() > 0);
        // Roof level with the ground: a portal mouth, which is the design.
        let t = tile(
            vec![RoadMesh { class: "motorway".into(), level: -1, band: String::new(), mesh: slab(100.0, 5.0) }],
            Some(flat(100.0)),
        );
        assert_eq!(run(&t)[2].violations(), 0);
    }

    #[test]
    fn a_buried_bore_reports_its_cover_as_a_positive_depth() {
        let t = tile(
            vec![RoadMesh { class: "motorway".into(), level: -1, band: String::new(), mesh: slab(90.0, 5.0) }],
            Some(flat(100.0)),
        );
        let m = run(&t);
        assert_eq!(m[2].violations(), 0);
        assert!((m[2].worst_value().unwrap() - 10.0).abs() < 1e-3);
    }

    #[test]
    fn two_at_grade_bands_stacked_in_the_air_are_caught() {
        // The Collonge drawing: a rail formation's open cut sliding 8.5 m under
        // the band of the funicular crossing just past its mapped portal. Both
        // are level 0; only a whole-mesh sample can see the interior overlap.
        let t = tile(
            vec![
                RoadMesh { class: "rail_surface".into(), level: 0, band: String::new(), mesh: flat(524.0) },
                RoadMesh { class: "rail_surface".into(), level: 0, band: String::new(), mesh: flat(532.5) },
            ],
            None,
        );
        let m = run(&t);
        let stack = m.iter().find(|x| x.id == "order.grade_stack").unwrap();
        assert!(stack.violations() > 0);
        assert!((stack.worst_value().unwrap() - 8.5).abs() < 1e-3);
        assert_eq!(stack.invariant, Invariant::I3);
    }

    #[test]
    fn abutting_bands_at_one_height_are_the_populations_floor() {
        let t = tile(
            vec![
                RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) },
                RoadMesh { class: "rail_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) },
            ],
            None,
        );
        let m = run(&t);
        let stack = m.iter().find(|x| x.id == "order.grade_stack").unwrap();
        assert_eq!(stack.violations(), 0, "coincident bands owe each other nothing");
        assert!(!stack.dist.is_empty(), "but the pair is in the population");
    }

    #[test]
    fn an_extract_without_structures_skips_rather_than_scoring_clean() {
        let t = tile(
            vec![RoadMesh { class: "road_surface".into(), level: 0, band: String::new(), mesh: flat(100.0) }],
            Some(flat(100.0)),
        );
        let m = run(&t);
        assert!(m.iter().all(|x| x.skipped.is_some()), "no structures means nothing measured");
    }
}
