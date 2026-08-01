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

use std::collections::HashMap;

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Metric, Offender, Sense, Worst};

use super::{Check, Options};

const EXTENT: i64 = 32768;

/// Anything above this is a visible step. Well under a centimetre, which is the
/// format's own resolution: heights are int32 millimetres, so two tiles that
/// agree in the model agree here to the millimetre or not at all.
const STEP_M: f64 = 0.005;

/// What one lattice point on a tile border has been seen to be.
#[derive(Clone, Copy)]
struct Shared {
    lo: f32,
    hi: f32,
    /// Which tile contributed first, so a vertex repeated inside one tile is
    /// not mistaken for two tiles disagreeing.
    tile: u64,
    tiles: u32,
}

pub struct Seams {
    terrain: HashMap<(i64, i64), Shared>,
    pavement: HashMap<(i64, i64), Shared>,
    worst_k: usize,
    zoom: u8,
}

impl Seams {
    pub fn new(opt: &Options) -> Seams {
        Seams { terrain: HashMap::new(), pavement: HashMap::new(), worst_k: opt.worst_k, zoom: 0 }
    }
}

/// Folds every border vertex of `mesh` into `into`.
fn collect(into: &mut HashMap<(i64, i64), Shared>, mesh: &SurfaceMesh, tile: &TileScene) {
    let id = (tile.x as u64) << 32 | tile.y as u64;
    for i in 0..mesh.vertex_count() {
        let (px, py, pz) = mesh.vertex(i);
        // Unit coordinates come from an exact integer quantum, so this recovers
        // the original uint16 without slack.
        let qx = (px * EXTENT as f64).round() as i64;
        let qy = (py * EXTENT as f64).round() as i64;
        let on_border = qx == 0 || qx == EXTENT || qy == 0 || qy == EXTENT;
        if !on_border {
            continue;
        }
        // Off-border axes outside the tile proper belong to a corner of some
        // other pair of tiles; skip rather than key them into a phantom group.
        if !(0..=EXTENT).contains(&qx) || !(0..=EXTENT).contains(&qy) {
            continue;
        }
        let key = (tile.x as i64 * EXTENT + qx, tile.y as i64 * EXTENT + qy);
        let z = pz as f32;
        into.entry(key)
            .and_modify(|s| {
                s.lo = s.lo.min(z);
                s.hi = s.hi.max(z);
                if s.tile != id {
                    s.tile = id;
                    s.tiles += 1;
                }
            })
            .or_insert(Shared { lo: z, hi: z, tile: id, tiles: 1 });
    }
}

/// Turns one collected map into its metric.
fn measure(
    map: HashMap<(i64, i64), Shared>,
    zoom: u8,
    worst_k: usize,
    id: &str,
    title: &str,
    detail: &str,
) -> Metric {
    let mut dist = Dist::new(0.0, 32.0);
    let mut worst = Worst::new(Sense::HigherIsWorse, worst_k);
    let mut shared = 0u64;
    for ((gx, gy), s) in &map {
        // A point only one tile ever saw proves nothing about agreement.
        if s.tiles < 2 {
            continue;
        }
        shared += 1;
        let step = (s.hi - s.lo) as f64;
        dist.push(step);
        if step > STEP_M {
            let n = (1u64 << zoom) as f64;
            let lon = -180.0 + (*gx as f64 / EXTENT as f64) * (360.0 / n);
            let lat = -90.0 + (*gy as f64 / EXTENT as f64) * (180.0 / n);
            worst.offer(Offender {
                lon,
                lat,
                zoom,
                value: step,
                note: format!("neighbouring tiles read {:.3} m and {:.3} m", s.lo, s.hi),
            });
        }
    }
    Metric {
        id: id.into(),
        invariant: 2,
        title: title.into(),
        detail: detail.into(),
        sense: Sense::HigherIsWorse,
        threshold: STEP_M,
        skipped: (shared == 0)
            .then(|| "no border vertex was seen from both sides (single tile, or none at this zoom)".to_string()),
        dist,
        worst: worst.into_vec(),
    }
}

impl Check for Seams {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        self.zoom = tile.z;
        if let Some(t) = &tile.terrain {
            collect(&mut self.terrain, t, tile);
        }
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            collect(&mut self.pavement, &road.mesh, tile);
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![
            measure(
                self.terrain,
                self.zoom,
                self.worst_k,
                "seam.terrain_step",
                "Terrain height disagreement across a tile border",
                "Spread of the heights two adjacent tiles derive for the same border lattice \
                 point. Anything non-zero is a staircase at the seam and proof that a height \
                 depended on the tile window.",
            ),
            measure(
                self.pavement,
                self.zoom,
                self.worst_k,
                "seam.pavement_step",
                "Carriageway height disagreement across a tile border",
                "The same, for the at-grade road surface. The paved region is clipped from one \
                 global union, so the two sides share a snapped seam by construction; a step \
                 here means that construction leaked.",
            ),
        ]
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

    #[test]
    fn neighbours_agreeing_on_the_border_report_a_zero_step() {
        // Tile 100's east edge is 250 m; tile 101's west edge is 250 m.
        let m = run(&[strip(100, 200.0, 250.0), strip(101, 250.0, 300.0)]);
        assert!(!m[0].dist.is_empty(), "the shared border must have been joined");
        assert_eq!(m[0].violations(), 0);
        assert_eq!(m[0].worst_value(), Some(0.0));
    }

    #[test]
    fn a_disagreement_at_the_border_is_caught_and_placed() {
        let m = run(&[strip(100, 200.0, 250.0), strip(101, 253.5, 300.0)]);
        assert!(m[0].violations() > 0);
        assert!((m[0].worst_value().unwrap() - 3.5).abs() < 1e-3);
        let o = &m[0].worst[0];
        assert_eq!(o.zoom, 16);
        // The joined point is the shared border: tile 100's east edge.
        let expect = Bounds::of_tile(16, 100, 23000).east;
        assert!((o.lon - expect).abs() < 1e-9, "offender at {} not {expect}", o.lon);
    }

    #[test]
    fn one_tile_alone_proves_nothing_and_is_not_scored() {
        let m = run(&[strip(100, 200.0, 250.0)]);
        assert!(m[0].skipped.is_some(), "a lone tile has no agreement to measure");
    }

    #[test]
    fn a_vertex_repeated_inside_one_tile_is_not_two_tiles_disagreeing() {
        // Same tile, two coincident border vertices at different heights: a
        // defect, but not this one, and it must not be attributed to a seam.
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
        };
        let m = run(&[t]);
        assert!(m[0].skipped.is_some());
        assert_eq!(m[0].violations(), 0);
    }

    #[test]
    fn the_carriageway_seam_is_measured_separately_from_the_ground() {
        let mut a = strip(100, 200.0, 250.0);
        let mut b = strip(101, 250.0, 300.0);
        let pave = |west: f32, east: f32| {
            RoadMesh {
                class: "road_surface".into(),
                level: 0,
                mesh: SurfaceMesh::from_parts(
                    vec![0.0, 1.0, 1.0, 0.0],
                    vec![0.4, 0.4, 0.6, 0.6],
                    vec![west, east, east, west],
                    vec![0, 1, 2, 0, 2, 3],
                )
                .unwrap(),
            }
        };
        a.roads.push(pave(201.0, 251.0));
        b.roads.push(pave(251.9, 301.0));
        let m = run(&[a, b]);
        assert_eq!(m[0].violations(), 0, "the ground still agrees");
        assert!(m[1].violations() > 0, "the asphalt does not");
        assert!((m[1].worst_value().unwrap() - 0.9).abs() < 1e-3);
    }
}
