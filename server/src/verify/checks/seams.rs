//! Invariant 2 at the tile border, and invariant 5 with it.
//!
//! docs/GROUND.md states the contract exactly: "Interior triangulations of
//! adjacent tiles need not match — only border vertex positions and heights
//! must." That is a promise with no threshold and no prior attached, which
//! makes it the cleanest thing in the whole scorecard to measure: two tiles
//! either derived the same height for the same point or they did not, and any
//! disagreement at all is a staircase at a seam (invariant 6's "never produce
//! spectacle") and a tile-window dependence (invariant 5).
//!
//! The join is done in exact integers. A border vertex sits at quantized 49152
//! on one side and 16384 on the other, and `tile · 32768 + (q − 16384)` maps
//! both onto the same global lattice coordinate with no arithmetic slack — so a
//! reported step is a real disagreement, never a rounding artifact of the
//! check itself.
//!
//! ## Two defects, not one
//!
//! The first version of this check kept a single min and max per lattice point
//! and called the spread a seam step. That conflates two unrelated things, and
//! on real data the conflation mattered: of 42 stepping points, only 16 were
//! genuine cross-tile disagreements, and the worst — 3.8 m — was a *single
//! tile* holding two coincident vertices 3.8 m apart. A surface that carries
//! two different heights at one plan position has split open; that is a defect,
//! but it is not a seam, and attributing it to one would send anyone who
//! chased it to the wrong module.
//!
//! So the two are separated: `step` compares one tile's answer against its
//! neighbour's, and the second metric reports a surface disagreeing with
//! *itself* at one plan position.
//!
//! For the terrain that second case is a crack, and it reads zero. For the
//! carriageway it is not a crack at all, which took a third pass to establish:
//! the paved region is keyed by `(level, layer)` in `synth::pavement`, where
//! `layer` is the grade-separation layer, and its own doc note says regions on
//! different layers "overlap in plan but are metres apart vertically". So a
//! tile legitimately carries several level-0 asphalt meshes. What is *not*
//! legitimate is two of them at one ordinal: until 2026-08-30
//! `add_road_surface` encoded only `level`, the layer that separated them was
//! dropped, and the client received several opaque surfaces with nothing to
//! order them by. The layer now travels as the `sheet` property, and this
//! check reads it: two meshes on different sheets are stacked by design and
//! are not charged; two on the same sheet, or two with no sheet at all (an
//! archive cut before the property existed), still are. Hence the metric name
//! — this is an ordering gap, not a seam.
//!
//! The sheet also decides what a *seam* compares. Where each tile holds one
//! surface at a border point the two are compared as before, whatever their
//! numbers — a corridor may change sheet number at a z13 chunk border, and
//! its seam is still one seam. Only where a tile holds several surfaces at
//! one point are they paired across the border by sheet, which is what lets
//! stacked asphalt be seam-checked at all instead of excluded as ambiguous.

use std::collections::HashMap;

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

const EXTENT: i64 = 32768;

/// Anything above this is a visible step. Well under a centimetre, which is the
/// format's own resolution: heights are int32 millimetres, so two tiles that
/// agree in the model agree here to the millimetre or not at all.
const STEP_M: f64 = 0.005;

/// What one tile says about one lattice point on its border.
#[derive(Clone, Copy)]
struct Claim {
    tile: u64,
    /// The mesh's `sheet` ordinal; `None` for the terrain and for a surface
    /// emitted without one. One claim per `(tile, sheet)`: two meshes on
    /// different sheets are two answers by design, not one surface split open.
    sheet: Option<i64>,
    lo: f32,
    hi: f32,
}

/// Every border lattice point, and what each tile touching it claimed.
type Shared = HashMap<(i64, i64), Vec<Claim>>;

pub struct Seams {
    terrain: Shared,
    pavement: Shared,
    /// Pavement meshes seen, and how many of them carried no `sheet` — an
    /// archive cut before the property was emitted, or an emitter that
    /// dropped it. The ordering metric reports the count, since without the
    /// ordinal the client is back to ordering by chance.
    meshes: u64,
    sheetless: u64,
    worst_k: usize,
    zoom: u8,
}

impl Seams {
    pub fn new(opt: &Options) -> Seams {
        Seams {
            terrain: Shared::new(),
            pavement: Shared::new(),
            meshes: 0,
            sheetless: 0,
            worst_k: opt.worst_k,
            zoom: 0,
        }
    }
}

/// Folds every border vertex of `mesh` into `into`.
fn collect(into: &mut Shared, mesh: &SurfaceMesh, sheet: Option<i64>, tile: &TileScene) {
    let id = (tile.x as u64) << 32 | tile.y as u64;
    for i in 0..mesh.vertex_count() {
        let (px, py, pz) = mesh.vertex(i);
        // Unit coordinates come from an exact integer quantum, so this recovers
        // the original uint16 without slack.
        let qx = (px * EXTENT as f64).round() as i64;
        let qy = (py * EXTENT as f64).round() as i64;
        if !(qx == 0 || qx == EXTENT || qy == 0 || qy == EXTENT) {
            continue;
        }
        // Off-border axes outside the tile proper belong to a corner of some
        // other pair of tiles; skip rather than key them into a phantom group.
        if !(0..=EXTENT).contains(&qx) || !(0..=EXTENT).contains(&qy) {
            continue;
        }
        let key = (tile.x as i64 * EXTENT + qx, tile.y as i64 * EXTENT + qy);
        let z = pz as f32;
        let claims = into.entry(key).or_default();
        match claims.iter_mut().find(|c| c.tile == id && c.sheet == sheet) {
            Some(c) => {
                c.lo = c.lo.min(z);
                c.hi = c.hi.max(z);
            }
            None => claims.push(Claim { tile: id, sheet, lo: z, hi: z }),
        }
    }
}

/// Geodetic position of a global border lattice point.
fn lonlat(gx: i64, gy: i64, zoom: u8) -> (f64, f64) {
    let n = (1u64 << zoom) as f64;
    (
        -180.0 + (gx as f64 / EXTENT as f64) * (360.0 / n),
        -90.0 + (gy as f64 / EXTENT as f64) * (180.0 / n),
    )
}

/// What it means for one tile to hold two heights at a single plan point.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SelfDisagreement {
    /// One surface, so two heights is a crack in it.
    Crack,
    /// Several surfaces by design (grade-separation layers), so two heights is
    /// an ordering the format failed to carry.
    UnorderedOverlap,
}

/// Turns one collected map into its two metrics.
fn measure(
    map: Shared,
    zoom: u8,
    worst_k: usize,
    what: &str,
    subject: &str,
    kind: SelfDisagreement,
) -> Vec<Metric> {
    let mut step = Dist::new(0.0, 32.0);
    let mut step_worst = Worst::new(Sense::HigherIsWorse, worst_k);
    let mut split = Dist::new(0.0, 32.0);
    let mut split_worst = Worst::new(Sense::HigherIsWorse, worst_k);
    let mut shared = 0u64;
    let mut unusable = 0u64;

    for ((gx, gy), claims) in &map {
        // A surface disagreeing with itself: measurable from one tile alone,
        // so this does not require a neighbour.
        for c in claims {
            let s = (c.hi - c.lo) as f64;
            split.push(s);
            if s > STEP_M {
                let (lon, lat) = lonlat(*gx, *gy, zoom);
                split_worst.offer(Offender {
                    lon,
                    lat,
                    zoom,
                    value: s,
                    note: match c.sheet {
                        Some(sheet) => format!(
                            "one tile holds coincident vertices at {:.3} m and {:.3} m on sheet {sheet}",
                            c.lo, c.hi
                        ),
                        None => format!(
                            "one tile holds coincident vertices at {:.3} m and {:.3} m",
                            c.lo, c.hi
                        ),
                    },
                });
            }
        }

        // Which claims to compare across the border. Where every tile holds
        // one surface at this point, compare them all whatever their sheet
        // numbers say — the numbers are per chunk, and one surface meeting
        // one surface is one seam. Where a tile holds several, pair them by
        // sheet: asphalt stacked over asphalt has a partner on the other side
        // only on its own sheet.
        let mut tiles: Vec<u64> = claims.iter().map(|c| c.tile).collect();
        tiles.sort_unstable();
        tiles.dedup();
        let one_each = tiles.len() == claims.len();
        let mut groups: Vec<Vec<&Claim>> = if one_each {
            vec![claims.iter().collect()]
        } else {
            let mut sheets: Vec<Option<i64>> = claims.iter().map(|c| c.sheet).collect();
            sheets.sort_unstable();
            sheets.dedup();
            sheets.into_iter().map(|s| claims.iter().filter(|c| c.sheet == s).collect()).collect()
        };
        for group in groups.iter_mut() {
            // A point only one tile ever saw proves nothing about agreement.
            if group.len() < 2 {
                continue;
            }
            // A tile that has split open has no single answer at this point,
            // so there is nothing to compare its neighbour against. Whatever
            // extra height it carries is already reported as a crack; charging
            // the difference to the seam as well would count one defect twice
            // and send the reader to the wrong module.
            if group.iter().any(|c| (c.hi - c.lo) as f64 > STEP_M) {
                unusable += 1;
                continue;
            }
            shared += 1;
            // Compare like with like: each tile's lowest against the others',
            // and each tile's highest against the others'.
            let spread = |f: fn(&Claim) -> f32| {
                let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
                for c in group.iter() {
                    lo = lo.min(f(c));
                    hi = hi.max(f(c));
                }
                (hi - lo) as f64
            };
            let s = spread(|c| c.lo).max(spread(|c| c.hi));
            step.push(s);
            if s > STEP_M {
                let (lon, lat) = lonlat(*gx, *gy, zoom);
                    let heights: Vec<String> = group
                        .iter()
                        .map(|c| {
                            format!(
                                "{}/{} s{:?} {:.3}",
                                c.tile >> 32,
                                c.tile & 0xffff_ffff,
                                c.sheet,
                                c.lo
                            )
                        })
                        .collect();
                step_worst.offer(Offender {
                    lon,
                    lat,
                    zoom,
                    value: s,
                    note: format!(
                        "{} neighbouring tiles read {} (lattice {gx},{gy})",
                        group.len(),
                        heights.join(" / ")
                    ),
                });
            }
        }
    }

    let no_neighbour = (shared == 0)
        .then(|| "no border vertex was seen from both sides (single tile, or none at this zoom)".to_string());
    let nothing = split.is_empty().then(|| format!("no {subject} border vertices at this zoom"));
    let mut step_detail = "Spread of the heights adjacent tiles derive for the same border \
                           lattice point, compared low-against-low and high-against-high. \
                           Anything non-zero is a staircase at the seam and proof that a height \
                           depended on the tile window."
        .to_string();
    if unusable > 0 {
        step_detail.push_str(&format!(
            " {unusable} shared points were excluded because a tile had split open there and \
             had no single answer to compare; they are counted under seam.{what}_split."
        ));
    }
    vec![
        Metric {
            id: format!("seam.{what}_step"),
            invariant: Invariant::I2,
            population: format!(
                "Every global border lattice point of the {subject} mesh claimed by two or more \
                 tiles at this zoom. Points where one side had split open have no single answer \
                 to compare and are excluded — they are counted under seam.{what}_split."
            ),
            title: format!("{subject} height disagreement across a tile border"),
            detail: step_detail,
            sense: Sense::HigherIsWorse,
            threshold: STEP_M,
            skipped: no_neighbour,
            dist: step,
            worst: step_worst.into_vec(),
        },
        match kind {
            SelfDisagreement::Crack => Metric {
                id: format!("seam.{what}_split"),
                invariant: Invariant::I2,
                population: format!(
                    "Every pair of coincident {subject} border vertices inside one tile. Needs \
                     no neighbour, so it covers border tiles the step metric cannot score."
                ),
                title: format!("{subject} disagreeing with itself at one point"),
                detail: "Spread between coincident vertices inside a single tile. One surface \
                         carrying two heights at one plan position has split open — a crack, not \
                         a seam, and it needs no neighbour to detect."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: STEP_M,
                skipped: nothing,
                dist: split,
                worst: split_worst.into_vec(),
            },
            SelfDisagreement::UnorderedOverlap => Metric {
                id: "order.at_grade_overlap".into(),
                invariant: Invariant::I3,
                population: "Coincident border vertices of two different level-0 road_surface \
                             meshes in one tile that carry the same `sheet` ordinal, or none. \
                             Border vertices only — that is what this pass collects; a \
                             whole-mesh version would find more overlap."
                    .into(),
                title: "Overlapping at-grade asphalt with nothing to order it".into(),
                detail: "Vertical separation where two level-0 paved regions share a plan \
                         position and an ordinal. Several regions per level are by design — \
                         `synth::pavement` keys them by (level, layer), and different \
                         grade-separation layers overlap in plan while sitting metres apart — \
                         and since 2026-08-30 the layer reaches the client as the `sheet` \
                         property, so two surfaces on different sheets are ordered and not \
                         charged. What remains is two surfaces the client cannot order: the \
                         same sheet at two heights, or an archive with no `sheet` at all. \
                         Scoped to border vertices, which is what this check collects; a \
                         whole-mesh version would find more."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: STEP_M,
                skipped: nothing,
                dist: split,
                worst: split_worst.into_vec(),
            },
        },
    ]
}

impl Check for Seams {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        self.zoom = tile.z;
        if let Some(t) = &tile.terrain {
            collect(&mut self.terrain, t, None, tile);
        }
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            self.meshes += 1;
            if road.sheet.is_none() {
                self.sheetless += 1;
            }
            collect(&mut self.pavement, &road.mesh, road.sheet, tile);
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let mut out = measure(
            self.terrain,
            self.zoom,
            self.worst_k,
            "terrain",
            "Terrain",
            SelfDisagreement::Crack,
        );
        let mut pavement = measure(
            self.pavement,
            self.zoom,
            self.worst_k,
            "pavement",
            "Carriageway",
            SelfDisagreement::UnorderedOverlap,
        );
        if let Some(m) = pavement.iter_mut().find(|m| m.id == "order.at_grade_overlap") {
            m.detail.push_str(&format!(
                " {} of {} carriageway meshes carried a `sheet`{}.",
                self.meshes - self.sheetless,
                self.meshes,
                if self.sheetless > 0 {
                    " — the rest reach the client with no ordinal at all"
                } else {
                    ""
                }
            ));
        }
        out.extend(pavement);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::Scale;
    use crate::verify::scene::RoadMesh;

    /// A tile whose terrain is a strip touching its east and west borders, with
    /// the two border heights given.
    fn strip(x: u32, west_m: f32, east_m: f32) -> TileScene {
        let b = Bounds::of_tile(16, x, 23000);
        let terrain = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![west_m, east_m, east_m, west_m],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        TileScene {
            z: 16,
            x,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    fn run(tiles: &[TileScene]) -> Vec<Metric> {
        let opt = Options::default();
        let mut s = Box::new(Seams::new(&opt));
        for t in tiles {
            s.visit(t, &opt);
        }
        s.finish()
    }

    fn metric<'a>(m: &'a [Metric], id: &str) -> &'a Metric {
        m.iter().find(|x| x.id == id).expect(id)
    }

    #[test]
    fn neighbours_agreeing_on_the_border_report_a_zero_step() {
        // Tile 100's east edge is 250 m; tile 101's west edge is 250 m.
        let m = run(&[strip(100, 200.0, 250.0), strip(101, 250.0, 300.0)]);
        let step = metric(&m, "seam.terrain_step");
        assert!(!step.dist.is_empty(), "the shared border must have been joined");
        assert_eq!(step.violations(), 0);
        assert_eq!(step.worst_value(), Some(0.0));
    }

    #[test]
    fn a_disagreement_at_the_border_is_caught_and_placed() {
        let m = run(&[strip(100, 200.0, 250.0), strip(101, 253.5, 300.0)]);
        let step = metric(&m, "seam.terrain_step");
        assert!(step.violations() > 0);
        assert!((step.worst_value().unwrap() - 3.5).abs() < 1e-3);
        let o = &step.worst[0];
        assert_eq!(o.zoom, 16);
        let expect = Bounds::of_tile(16, 100, 23000).east;
        assert!((o.lon - expect).abs() < 1e-9, "offender at {} not {expect}", o.lon);
    }

    #[test]
    fn one_tile_alone_proves_nothing_about_a_seam() {
        let m = run(&[strip(100, 200.0, 250.0)]);
        assert!(metric(&m, "seam.terrain_step").skipped.is_some());
    }

    #[test]
    fn a_surface_splitting_inside_one_tile_is_not_reported_as_a_seam() {
        // The regression the first version shipped: two coincident border
        // vertices 9 m apart inside a single tile. That is a crack in the
        // surface, and calling it a seam step sends the reader to the wrong
        // module. It must be found — and found under the right name.
        let b = Bounds::of_tile(16, 100, 23000);
        let terrain = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 1.0, 0.0],
            vec![200.0, 250.0, 250.0, 200.0, 209.0],
            vec![0, 1, 2, 0, 2, 3, 0, 4, 1],
        )
        .unwrap();
        let t = TileScene {
            z: 16,
            x: 100,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };
        let m = run(&[t]);
        assert!(metric(&m, "seam.terrain_step").skipped.is_some(), "no neighbour, so no seam");
        let split = metric(&m, "seam.terrain_split");
        assert!(split.violations() > 0, "but the crack must be found");
        assert!((split.worst_value().unwrap() - 9.0).abs() < 1e-3);
    }

    #[test]
    fn a_tiles_own_spread_is_not_charged_to_its_neighbour() {
        // Tile 100 splits open at the shared north-east corner: vertex 4 is
        // coincident with vertex 2 in plan but 3.5 m higher. Tile 101 agrees
        // with the lower of the two. Comparing 100's high against 101's only
        // value would invent a seam disagreement that is really 100's own
        // crack, so the seam metric must see nothing here at all.
        let b100 = Bounds::of_tile(16, 100, 23000);
        let split_tile = TileScene {
            z: 16,
            x: 100,
            y: 23000,
            scale: Scale::of(&b100),
            bounds: b100,
            terrain: Some(
                SurfaceMesh::from_parts(
                    vec![0.0, 1.0, 1.0, 0.0, 1.0, 0.5],
                    vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.5],
                    vec![200.0, 250.0, 250.0, 200.0, 253.5, 220.0],
                    vec![0, 1, 2, 0, 2, 3, 4, 5, 2],
                )
                .unwrap(),
            ),
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };
        let m = run(&[split_tile, strip(101, 250.0, 300.0)]);
        let step = metric(&m, "seam.terrain_step");
        let split = metric(&m, "seam.terrain_split");
        assert_eq!(step.violations(), 0, "worst step was {:?}", step.worst_value());
        assert!(split.violations() > 0, "the crack is still reported, under its own name");
        assert!((split.worst_value().unwrap() - 3.5).abs() < 1e-3);
    }

    /// A full-tile level-0 `road_surface` slab at height `h` on `sheet`.
    fn region(h: f32, sheet: Option<i64>) -> RoadMesh {
        RoadMesh {
            class: "road_surface".into(),
            level: 0,
            band: String::new(), fades: false, sheet,
            mesh: SurfaceMesh::from_parts(
                vec![0.0, 1.0, 1.0, 0.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![h; 4],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap(),
        }
    }

    /// One tile holding `roads`, and no terrain.
    fn stacked(x: u32, roads: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, x, 23000);
        TileScene {
            z: 16,
            x,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: None,
            roads,
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    #[test]
    fn two_at_grade_regions_at_one_point_are_reported_as_an_ordering_gap() {
        // Two level-0 `road_surface` meshes overlapping in plan, 8.8 m apart,
        // in an archive that carries no `sheet`. Legitimate as geometry — they
        // are different grade-separation layers — but nothing orders them. It
        // must land under the ordering metric, not under a seam or a crack.
        let m = run(&[stacked(100, vec![region(480.0, None), region(488.8, None)])]);
        let overlap = metric(&m, "order.at_grade_overlap");
        assert!(overlap.violations() > 0);
        assert!((overlap.worst_value().unwrap() - 8.8).abs() < 1e-3);
        assert_eq!(overlap.invariant, Invariant::I3, "this is a vertical-ordering finding");
        assert!(m.iter().all(|x| x.id != "seam.pavement_split"), "not a crack");
        // And the seam metric must not double-count it: the tile has no single
        // answer at those points, so there is nothing to compare a neighbour to.
        assert_eq!(metric(&m, "seam.pavement_step").violations(), 0);
    }

    #[test]
    fn two_regions_on_different_sheets_are_ordered_and_not_a_gap() {
        // The same stack, with the sheet ordinal emitted: the client can order
        // the two, so there is nothing to charge.
        let m = run(&[stacked(100, vec![region(480.0, Some(0)), region(488.8, Some(1))])]);
        assert_eq!(metric(&m, "order.at_grade_overlap").violations(), 0);
        // Two regions on ONE sheet at two heights are still a gap.
        let m = run(&[stacked(100, vec![region(480.0, Some(1)), region(488.8, Some(1))])]);
        assert!(metric(&m, "order.at_grade_overlap").violations() > 0);
    }

    #[test]
    fn stacked_asphalt_is_seam_checked_sheet_by_sheet() {
        // Two tiles, each holding two stacked sheets. The lower sheet agrees
        // across the border and the upper does not; the seam must pair by
        // sheet and find the upper one's 0.7 m, rather than exclude the point
        // as ambiguous or compare the lower sheet against the upper.
        let a = stacked(100, vec![region(480.0, Some(0)), region(488.8, Some(1))]);
        let b = stacked(101, vec![region(480.0, Some(0)), region(489.5, Some(1))]);
        let m = run(&[a, b]);
        let step = metric(&m, "seam.pavement_step");
        assert!(step.violations() > 0);
        assert!((step.worst_value().unwrap() - 0.7).abs() < 1e-3);
        assert_eq!(metric(&m, "order.at_grade_overlap").violations(), 0);
    }

    #[test]
    fn one_surface_each_side_is_one_seam_whatever_its_sheet_number() {
        // Sheet numbers are per chunk: across a chunk border the same corridor
        // may be sheet 0 on one side and sheet 2 on the other. With one
        // surface per tile at the point, the two are still compared.
        let a = stacked(100, vec![region(480.0, Some(0))]);
        let b = stacked(101, vec![region(480.9, Some(2))]);
        let m = run(&[a, b]);
        let step = metric(&m, "seam.pavement_step");
        assert!(step.violations() > 0);
        assert!((step.worst_value().unwrap() - 0.9).abs() < 1e-3);
    }

    #[test]
    fn the_carriageway_seam_is_measured_separately_from_the_ground() {
        let mut a = strip(100, 200.0, 250.0);
        let mut b = strip(101, 250.0, 300.0);
        let pave = |west: f32, east: f32| RoadMesh {
            class: "road_surface".into(),
            level: 0,
            band: String::new(), fades: false, sheet: None,
            mesh: SurfaceMesh::from_parts(
                vec![0.0, 1.0, 1.0, 0.0],
                vec![0.4, 0.4, 0.6, 0.6],
                vec![west, east, east, west],
                vec![0, 1, 2, 0, 2, 3],
            )
            .unwrap(),
        };
        a.roads.push(pave(201.0, 251.0));
        b.roads.push(pave(251.9, 301.0));
        let m = run(&[a, b]);
        assert_eq!(metric(&m, "seam.terrain_step").violations(), 0, "the ground still agrees");
        let pav = metric(&m, "seam.pavement_step");
        assert!(pav.violations() > 0, "the asphalt does not");
        assert!((pav.worst_value().unwrap() - 0.9).abs() < 1e-3);
    }
}
