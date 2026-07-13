//! Stage 3 — the engineered ground (docs/GENERATION.md §6, invariant 1).
//!
//! One authoritative ground function that every later consumer reads: terrain
//! meshing, road draping, building founding, structure contact. The function
//! is the natural DEM plus the earthworks the solved model implies, applied
//! as local modifiers ([`modifiers::Earthworks`]).
//!
//! [`derive`] translates the solved model into earthworks: wherever a
//! profiled corridor's at-grade road departs the natural ground — a
//! grade-limited cut through a bump, the embankment ramp the clearance solver
//! demanded for an overpass approach — the ground is reshaped to carry it
//! (D3). Consumers are untouched: they already read through
//! [`sampler::GroundSampler`].

pub mod modifiers;
pub mod sampler;

use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::dem::Dem;
use crate::priors::{
    DECK_THICKNESS_M, EARTHWORK_BATTER, EARTHWORK_MIN_FEATHER_M, EARTHWORK_SHOULDER_M,
    MAX_CLEARANCE_LIFT_M, MIN_EARTHWORK_M, PORTAL_CLEARANCE_M, PORTAL_CUT_LEN_M, WATER_LEVEL_PCTL,
};
use crate::scene::{SceneGraph, SpanKind};
use crate::solve::{portals, reference_surface, SolvedModel};

use modifiers::{EarthworkEdge, Earthworks, WaterFill, Waters};

/// Most shoreline vertices sampled when reading a water body's level — enough
/// to be robust on a big lake, bounded so a many-thousand-vertex ring is cheap.
const SHORELINE_SAMPLES: usize = 128;

/// The engineered ground: the single ground function of invariant 1. Queries
/// apply the covering water surface and earthworks to the raw DEM sample in a
/// fixed global order, so any two tiles (and any two zooms) derive identical
/// ground for shared world points.
pub struct GroundModel {
    earthworks: Earthworks,
    waters: Waters,
}

impl GroundModel {
    /// A ground model with no modifiers: the raw DEM passes through.
    pub fn empty() -> GroundModel {
        GroundModel { earthworks: Earthworks::new(Vec::new()), waters: Waters::new(Vec::new()) }
    }

    /// Number of earthwork edges, for run stats.
    pub fn earthwork_count(&self) -> usize {
        self.earthworks.len()
    }

    /// Number of flattened water bodies, for run stats.
    pub fn water_count(&self) -> usize {
        self.waters.len()
    }

    pub fn earthworks(&self) -> &Earthworks {
        &self.earthworks
    }

    /// THE ground function: the engineered height at `(lon, lat)`, given the
    /// raw DEM sample `raw` for that point. `scratch` is the caller's reusable
    /// query buffer (see [`sampler::GroundSampler`]).
    ///
    /// Water flattens the ground first; a road earthwork (a bridge abutment's
    /// approach berm at the shore) then overrides it where the two overlap, so
    /// the roadbed wins over the water it climbs away from.
    pub fn height(&self, lon: f64, lat: f64, raw: f64, scratch: &mut Vec<u32>) -> f64 {
        let base = if self.waters.is_empty() {
            raw
        } else {
            self.waters.level_at(lon, lat, scratch).unwrap_or(raw)
        };
        if self.earthworks.is_empty() {
            return base;
        }
        self.earthworks.height(lon, lat, base, scratch)
    }
}

/// Derives the engineered ground from the solved model: one earthwork run per
/// at-grade stretch where the solved road departs the natural terrain by more
/// than [`MIN_EARTHWORK_M`], and a daylighting cut in front of every solved
/// tunnel portal (S5 — the mouth face must not hide below grade).
pub fn derive(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> GroundModel {
    let mut edges: Vec<EarthworkEdge> = Vec::new();
    for c in &scene.corridors {
        let Some(p) = solved.profile(c.id) else { continue };
        // Earthworks run along the *smoothed* sweep line — the same curve the
        // decks are swept along and the paint snaps to — so the roadbed crest
        // stays parallel to a deck edge instead of wiggling ±1–2 m beside it
        // (at a grazing view the crest occludes the deck's lower edge, and a
        // wiggling crest reads as a jagged deck).
        let nodes = p.smooth();
        let road = p.road_m();
        let terrain = p.terrain_m();
        let at_grade = p.at_grade();
        let half_width = c.class.half_width_m(c.link) + EARTHWORK_SHOULDER_M;

        let needs = |i: usize| at_grade[i] && (road[i] - terrain[i]).abs() > MIN_EARTHWORK_M;
        let mut i = 0;
        while i < nodes.len() {
            if !needs(i) {
                i += 1;
                continue;
            }
            // Maximal run of earthwork nodes, padded by one at-grade node on
            // each side so the reshaping eases in at natural ground.
            let start = i;
            while i < nodes.len() && needs(i) {
                i += 1;
            }
            let lo = if start > 0 && at_grade[start - 1] { start - 1 } else { start };
            let hi = if i < nodes.len() && at_grade[i] { i } else { i - 1 };
            for k in lo..hi {
                let lift = (road[k] - terrain[k]).abs().max((road[k + 1] - terrain[k + 1]).abs());
                edges.push(EarthworkEdge {
                    a: nodes[k],
                    b: nodes[k + 1],
                    target_a: road[k],
                    target_b: road[k + 1],
                    half_width_m: half_width,
                    feather_m: (EARTHWORK_BATTER * lift).max(EARTHWORK_MIN_FEATHER_M),
                    cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                    carve: false,
                });
            }
        }

        // Deck daylighting (the S10 mirror of the portal cut): inside a
        // mapped bridge span the deck is trusted to fly, but a DEM bump can
        // poke above it — a wooded ridge a surface DEM reads as ground — and
        // the terrain, drawn first, swallows the deck there. The bump is
        // carved to just below the deck underside. Interior bumps only: a
        // run reaching a span end is the deck legitimately meeting the
        // ground (an abutment, a portal in the hillside, S7) and stays for
        // the occlusion to work. A bump deeper than [`MAX_CLEARANCE_LIFT_M`]
        // is a data contradiction (a "bridge" through a real hill): the
        // terrain is trusted and the deck stays buried.
        let arcs = p.arc();
        for span in c.spans.iter().filter(|s| s.kind == SpanKind::Bridge) {
            let s0 = arcs.partition_point(|&a| a < span.arc0);
            let s1 = arcs.partition_point(|&a| a <= span.arc1);
            let intrudes = |i: usize| terrain[i] > road[i] - DECK_THICKNESS_M;
            let mut i = s0;
            while i < s1 {
                if !intrudes(i) {
                    i += 1;
                    continue;
                }
                let first = i;
                while i < s1 && intrudes(i) {
                    i += 1;
                }
                let last = i - 1;
                if first == s0 || last + 1 == s1 {
                    continue; // touches a span end: the deck meets the ground
                }
                let depth = (first..=last)
                    .map(|k| terrain[k] - (road[k] - DECK_THICKNESS_M))
                    .fold(0.0, f64::max);
                if depth > MAX_CLEARANCE_LIFT_M {
                    continue;
                }
                // Pad one node each side so the notch eases in below grade.
                let (lo, hi) = (first - 1, (last + 1).min(nodes.len() - 1));
                for k in lo..hi {
                    edges.push(EarthworkEdge {
                        a: nodes[k],
                        b: nodes[k + 1],
                        target_a: road[k] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        target_b: road[k + 1] - DECK_THICKNESS_M - PORTAL_CLEARANCE_M,
                        half_width_m: half_width,
                        feather_m: (EARTHWORK_BATTER * depth).max(EARTHWORK_MIN_FEATHER_M),
                        cos_lat: crate::scene::run_cos_lat(&[nodes[k], nodes[k + 1]]),
                        carve: true,
                    });
                }
            }
        }

        // Portal daylighting: carve the ground down to the bore floor in a
        // short cut outward from each solved portal, so the mouth's lower
        // metres stand clear instead of hiding below grade. Cut-only — where
        // the ground has already fallen away there is nothing to remove.
        for portal in portals::portals(p, &c.spans) {
            let a = p.point_at_arc(portal.arc);
            let b = p.point_at_arc(portal.arc + portal.outward * PORTAL_CUT_LEN_M);
            if a == b {
                continue; // portal at the corridor end: no outward run
            }
            edges.push(EarthworkEdge {
                a,
                b,
                target_a: portal.floor_m,
                target_b: portal.floor_m,
                half_width_m: c.class.half_width_m(c.link) + EARTHWORK_SHOULDER_M,
                feather_m: EARTHWORK_MIN_FEATHER_M,
                cos_lat: crate::scene::run_cos_lat(&[a, b]),
                carve: true,
            });
        }
    }
    let waters = derive_waters(scene, solved, terrain_path, threads);
    GroundModel { earthworks: Earthworks::new(edges), waters }
}

/// Reads a flat surface level for every still water body from the DEM along its
/// shoreline, so the ground stage can burn it flat (invariant 4). Parallelized
/// over `threads` workers (each forks the DEM to share the decoded-tile cache),
/// since sampling scattered shorelines is DEM-decode bound. Without a DEM there
/// is nothing to read a level from; the water stays draped on the terrain.
fn derive_waters(
    scene: &SceneGraph,
    solved: &SolvedModel,
    terrain_path: Option<&Path>,
    threads: usize,
) -> Waters {
    let bodies = &scene.water;
    if bodies.is_empty() {
        return Waters::new(Vec::new());
    }
    let Some(path) = terrain_path else {
        return Waters::new(Vec::new());
    };
    let Ok(primary) = Dem::open(path) else {
        return Waters::new(Vec::new()); // no DEM: leave the water draped
    };
    let z_ref = solved.z_ref;
    let n = bodies.len();
    let threads = threads.max(1).min(n);
    let next = Mutex::new(0usize);
    let fills: Mutex<Vec<Option<WaterFill>>> = Mutex::new(vec![None; n]);
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let Ok(mut dem) = primary.fork() else { return };
                loop {
                    let i = {
                        let mut cur = next.lock().expect("water queue poisoned");
                        if *cur >= n {
                            break;
                        }
                        let i = *cur;
                        *cur += 1;
                        i
                    };
                    let w = &bodies[i];
                    if let Some(level) =
                        water_level(&w.exterior, |c| reference_surface(&mut dem, z_ref, c.x, c.y))
                    {
                        fills.lock().expect("water fills poisoned")[i] = Some(WaterFill {
                            exterior: w.exterior.clone(),
                            holes: w.holes.clone(),
                            bbox: w.bbox,
                            level,
                        });
                    }
                }
            });
        }
    });
    Waters::new(fills.into_inner().expect("water fills poisoned").into_iter().flatten().collect())
}

/// A still water body's surface level: a low percentile of the DEM sampled
/// along its shoreline. The exterior ring traces the waterline, so the ground
/// there images the water level; a low percentile ([`WATER_LEVEL_PCTL`]) leans
/// toward the water rather than a vertex that climbed the bank, so the flat
/// surface sits in its basin instead of spilling over the shore. The ring is
/// subsampled to at most [`SHORELINE_SAMPLES`] points so a huge lake stays
/// cheap. The sampler is injected so the level is testable without a DEM.
fn water_level(ring: &[Coord], mut sample: impl FnMut(Coord) -> f64) -> Option<f64> {
    if ring.len() < 3 {
        return None;
    }
    let step = (ring.len() / SHORELINE_SAMPLES).max(1);
    let mut hs: Vec<f64> = ring.iter().step_by(step).map(|&c| sample(c)).collect();
    if hs.is_empty() {
        return None;
    }
    hs.sort_by(|a, b| a.partial_cmp(b).expect("finite elevations"));
    let idx = ((hs.len() as f64 * WATER_LEVEL_PCTL) as usize).min(hs.len() - 1);
    Some(hs[idx])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::RoadClass;
    use crate::scene::{Corridor, CrossedKind, Crossing, SegmentRef, Span, SpanKind, DEG_M};
    use geo_types::Coord;

    /// A viaduct over a valley with one sharp DEM bump poking above the deck
    /// mid-span (a wooded ridge a surface DEM reads as ground): the bump is
    /// carved to below the deck underside so the deck stays visible, while
    /// the valley floor and the span ends are untouched.
    #[test]
    fn a_bump_over_a_deck_is_daylighted() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 200.0, arc1: 800.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 800.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        // Plateaus at 100 m, a valley 40 m deep under the span, and a bump at
        // mid-span rising to 105 m — 5 m above the ~100 m deck line.
        let terrain = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m; // corridor metres
            let bump = (1.0 - ((x - 500.0) / 40.0).abs()).max(0.0) * 45.0;
            if !(200.0..=800.0).contains(&x) { 100.0 } else { (60.0 + bump).min(105.0) }
        };
        let scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Motorway,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let profiles = vec![crate::solve::profile::solve(&nodes, &spans, Some(0.06), &mut |c| {
            terrain(c)
        })];
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, None, 1);

        let mut scratch = Vec::new();
        let at = |x_m: f64| Coord { x: 6.0 + deg * x_m / len_m, y: 46.0 };
        // On the bump crest the ground is cut below the ~100 m deck.
        let crest = at(500.0);
        let cut = ground.height(crest.x, crest.y, terrain(crest), &mut scratch);
        assert!(cut < 99.0, "the bump must be carved below the deck, got {cut}");
        assert!(cut > 90.0, "the notch is a daylight cut, not a canyon, got {cut}");
        // The valley floor under the deck is untouched.
        let floor = at(350.0);
        assert_eq!(ground.height(floor.x, floor.y, terrain(floor), &mut scratch), terrain(floor));
        // The at-grade approach is untouched by the carve.
        let approach = at(100.0);
        let h = ground.height(approach.x, approach.y, terrain(approach), &mut scratch);
        assert!((h - 100.0).abs() < 0.5, "approach ground stays natural, got {h}");
    }

    /// The S4 scene end-to-end through solve + derive: a flat-ground overpass
    /// leaves embankment approaches in the engineered ground.
    #[test]
    fn overpass_approaches_become_embankments() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 450.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 450.0, arc1: 550.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 550.0, arc1: 1000.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc,
            cos_lat,
            class: RoadClass::Secondary,
            link: false,
            spans: spans.clone(),
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }]);
        let mid = Coord { x: 6.0 + deg * 0.5, y: 46.0 };
        scene.crossings = vec![Crossing {
            upper: 0,
            upper_arc: 500.0,
            point: mid,
            lower: None,
            lower_kind: CrossedKind::Road,
            upper_level: 1,
            lower_level: 0,
        }];

        let mut profiles =
            vec![crate::solve::profile::solve(&nodes, &spans, None, &mut |_| 372.0)];
        crate::solve::crossings::apply(&scene, &mut profiles);
        let solved = crate::solve::SolvedModel::from_profiles(profiles, 14);
        let ground = derive(&scene, &solved, None, 1);
        assert!(ground.earthwork_count() > 0, "the lifted approaches must become earthworks");

        // On the approach centerline (~80 m before the crossing, 30 m before
        // the span edge) the engineered ground rises to the solved road; far
        // away it is natural.
        let mut scratch = Vec::new();
        let approach = Coord { x: mid.x - 80.0 / (DEG_M * cos_lat), y: 46.0 };
        let road_there = solved.profile(0).unwrap().height_at(approach.x, approach.y);
        let h = ground.height(approach.x, approach.y, 372.0, &mut scratch);
        assert!(
            (h - road_there).abs() < 1e-6,
            "engineered ground {h} must meet the road {road_there}"
        );
        assert!(h > 372.5, "the approach is a real embankment, got {h}");
        let far = Coord { x: 6.0 + deg * 0.02, y: 46.0 };
        assert_eq!(ground.height(far.x, far.y, 372.0, &mut scratch), 372.0);
        // Under the bridge span itself the natural ground is untouched — the
        // deck stands on air, not on a berm.
        assert_eq!(ground.height(mid.x, mid.y, 372.0, &mut scratch), 372.0);
    }

    #[test]
    fn water_level_takes_a_low_shoreline_percentile() {
        // A shoreline that mostly images 372 m with a few bank vertices at
        // 378 m: the level must land on the waterline, not the bank.
        let ring: Vec<Coord> =
            (0..20).map(|i| Coord { x: 6.0 + i as f64 * 0.001, y: 46.0 }).collect();
        let level = water_level(&ring, |c| {
            if ((c.x * 1000.0).round() as i64) % 5 == 0 { 378.0 } else { 372.0 }
        })
        .expect("a level from a 20-vertex ring");
        assert!((level - 372.0).abs() < 1e-9, "level {level} must be the waterline, not the bank");
    }

    #[test]
    fn ground_flattens_water_and_a_shore_earthwork_overrides_it() {
        let mut scratch = Vec::new();
        let cos_lat = 46.0_f64.to_radians().cos();
        // A square lake [6.00, 6.01] × [46.00, 46.01], flattened to 372 m.
        let exterior = vec![
            Coord { x: 6.00, y: 46.00 },
            Coord { x: 6.01, y: 46.00 },
            Coord { x: 6.01, y: 46.01 },
            Coord { x: 6.00, y: 46.01 },
            Coord { x: 6.00, y: 46.00 },
        ];
        let waters = Waters::new(vec![WaterFill {
            exterior,
            holes: vec![],
            bbox: (6.00, 46.00, 6.01, 46.01),
            level: 372.0,
        }]);
        // A shore berm (a bridge approach earthwork) crossing the lake middle,
        // target 375 m — it must win over the water where they overlap.
        let earthworks = Earthworks::new(vec![EarthworkEdge {
            a: Coord { x: 6.00, y: 46.005 },
            b: Coord { x: 6.01, y: 46.005 },
            target_a: 375.0,
            target_b: 375.0,
            half_width_m: 8.0,
            feather_m: 4.0,
            cos_lat,
            carve: false,
        }]);
        let g = GroundModel { earthworks, waters };
        // Open water away from the berm: flattened to the level (over raw 360).
        assert_eq!(g.height(6.002, 46.002, 360.0, &mut scratch), 372.0);
        // On the berm centerline inside the lake: the road overrides the water.
        assert!((g.height(6.005, 46.005, 360.0, &mut scratch) - 375.0).abs() < 1e-9);
        // Outside the lake: the raw DEM passes through.
        assert_eq!(g.height(6.02, 46.02, 360.0, &mut scratch), 360.0);
    }
}
