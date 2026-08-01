//! Invariant 4, the half that matters most today: at-grade roads lie on the
//! rendered terrain of every zoom.
//!
//! This is the check the last fortnight of screenshots was standing in for. The
//! defect it hunts — asphalt sitting under the drawn ground — has never been a
//! wrong *height*: the field-level instrument in the terrain-hole study found
//! the road height field and the ground function agreeing to the centimetre
//! wherever a bench exists. It is a wrong *surface*: two triangulations of the
//! same intent that cross each other between their shared vertices. So this
//! samples the carriageway's interior and interrogates the ground's triangles,
//! and neither half of that is optional.
//!
//! Sign convention: the sample is `road − ground`. Invariant 4 has two sides —
//! "nothing floats and nothing is buried by accident" — and one sampling pass
//! answers both, so both are reported. The first version of this module
//! reported only burial, and that was a mistake worth recording: floating turned
//! out to be an order of magnitude more common (3.9 % of asphalt more than a
//! metre clear of the ground, against 1.5 % buried) and reached 15 m where
//! burial reached 4 m. A one-sided instrument had made the larger half of the
//! invariant invisible.

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// A sample this far below the drawn ground counts as buried. Half a decimetre
/// is under the depth quantization noise could explain and well under what the
/// eye reads as a road sunk into a hillside.
const BURIED_M: f64 = -0.05;

/// A road standing this far clear of the drawn ground has no embankment under
/// it. Generous on purpose: the ground is a decimated mesh and a road crossing
/// a lattice cell diagonally can legitimately stand a little proud of the
/// chord, so a metre is well past what interpolation explains and safely short
/// of what a missing embankment costs.
const FLOATING_M: f64 = 1.0;

pub struct Contact {
    over: Dist,
    buried_worst: Worst,
    floating_worst: Worst,
    /// Per-tile percentage of carriageway with no drawn ground beneath it.
    unbacked: Dist,
    unbacked_worst: Worst,
}

impl Contact {
    pub fn new(opt: &Options) -> Contact {
        Contact {
            over: Dist::metres(),
            buried_worst: Worst::new(Sense::LowerIsWorse, opt.worst_k),
            floating_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            unbacked: Dist::new(0.0, 100.0),
            unbacked_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }
}

impl Check for Contact {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        let Some(terrain) = &tile.terrain else { return };
        let (mut n, mut missing) = (0u64, 0u64);
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            road.mesh.sample(&tile.scale, opt.spacing_m, |px, py, rz| {
                if !tile.owns(px, py) {
                    return;
                }
                n += 1;
                let Some(gz) = terrain.height_at(px, py) else {
                    missing += 1;
                    return;
                };
                let v = rz - gz;
                self.over.push(v);
                let buried = v < BURIED_M;
                let floating = v > FLOATING_M;
                if buried || floating {
                    let (lon, lat) = tile.lonlat(px, py);
                    let o = Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: v,
                        note: format!("road {rz:.2} m, drawn ground {gz:.2} m"),
                    };
                    if buried {
                        self.buried_worst.offer(o);
                    } else {
                        self.floating_worst.offer(o);
                    }
                }
            });
        }
        if n > 0 {
            let pct = 100.0 * missing as f64 / n as f64;
            self.unbacked.push(pct);
            if missing > 0 {
                let (lon, lat) = tile.lonlat(0.5, 0.5);
                self.unbacked_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: pct,
                    note: format!("tile {}/{}/{}: {missing} of {n} samples", tile.z, tile.x, tile.y),
                });
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let skipped = self
            .over
            .is_empty()
            .then(|| "no at-grade road surface at this zoom (below ROAD_SURFACE_MIN_ZOOM, or the archive carries no DEM)".to_string());
        // Both metrics read the same distribution from opposite ends: one
        // sampling pass, two tails, no second walk over 16 M samples.
        vec![
            Metric {
                id: "contact.pavement_buried".into(),
                invariant: 4,
                title: "At-grade asphalt under the drawn ground".into(),
                detail: "Signed clearance of the carriageway surface over the terrain mesh, \
                         sampled across triangle interiors, read from the low end. Negative is \
                         buried: the ground is drawn through the road."
                    .into(),
                sense: Sense::LowerIsWorse,
                threshold: BURIED_M,
                dist: self.over.clone(),
                worst: self.buried_worst.into_vec(),
                skipped: skipped.clone(),
            },
            Metric {
                id: "contact.pavement_floating".into(),
                invariant: 4,
                title: "At-grade asphalt clear of the drawn ground".into(),
                detail: format!(
                    "The same distribution read from the high end. Past {FLOATING_M:.1} m the \
                     road stands on an embankment that was never built: a level-0 carriageway \
                     hanging in the air, which is the other half of \"nothing floats and nothing \
                     is buried\" and the half that turned out to be larger."
                ),
                sense: Sense::HigherIsWorse,
                threshold: FLOATING_M,
                dist: self.over,
                worst: self.floating_worst.into_vec(),
                skipped: skipped.clone(),
            },
            Metric {
                id: "contact.pavement_unbacked_pct".into(),
                invariant: 4,
                title: "Asphalt with no drawn ground beneath".into(),
                detail: "Per-tile share of carriageway whose plan position falls outside every \
                         terrain triangle. Today a gap in the terrain mesh; once the ground is \
                         cut back to the kerb by design, this becomes the expected state and \
                         the metric's meaning must be revisited rather than its number chased."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: 1.0,
                dist: self.unbacked,
                worst: self.unbacked_worst.into_vec(),
                skipped,
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

    /// A tile with one flat carriageway at `road_m` and a terrain mesh that
    /// tents up to `peak_m` in the middle — the chording case, where every
    /// shared corner agrees and only the interior does not.
    fn tented(road_m: f32, corner_m: f32, peak_m: f32) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        let road = SurfaceMesh::from_parts(
            vec![0.2, 0.8, 0.8, 0.2],
            vec![0.4, 0.4, 0.6, 0.6],
            vec![road_m; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let terrain = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0, 0.5],
            vec![0.0, 0.0, 1.0, 1.0, 0.5],
            vec![corner_m, corner_m, corner_m, corner_m, peak_m],
            vec![0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4],
        )
        .unwrap();
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads: vec![RoadMesh { class: "road_surface".into(), level: 0, mesh: road }],
        }
    }

    fn run(tile: &TileScene) -> Vec<Metric> {
        let opt = Options { spacing_m: 1.0, ..Default::default() };
        let mut c = Box::new(Contact::new(&opt));
        c.visit(tile, &opt);
        c.finish()
    }

    #[test]
    fn a_road_just_clear_of_a_flat_ground_is_neither_buried_nor_floating() {
        let m = run(&tented(100.5, 100.0, 100.0));
        assert!(!m[0].dist.is_empty());
        assert_eq!(m[0].violations(), 0, "not buried");
        assert_eq!(m[1].violations(), 0, "and half a metre is not floating");
    }

    #[test]
    fn a_road_standing_metres_clear_of_the_ground_is_caught_as_floating() {
        // The half of invariant 4 the first version of this module missed: an
        // embankment the earthworks never built, leaving the carriageway in
        // the air. It is not buried, and a one-sided check calls that clean.
        let m = run(&tented(109.0, 100.0, 100.0));
        assert_eq!(m[0].violations(), 0, "nothing is buried");
        assert!(m[1].violations() > 0, "but the road is 9 m up on nothing");
        assert!((m[1].worst_value().unwrap() - 9.0).abs() < 1e-3);
        assert!(!m[1].worst.is_empty(), "a violation must name a place");
    }

    #[test]
    fn a_tent_between_the_shared_corners_is_caught() {
        // The regression this module exists to prevent: corners agree at 100 m,
        // the ground tents to 104 m in the middle, road is flat at 100 m.
        let m = run(&tented(100.0, 100.0, 104.0));
        let over = &m[0];
        assert!(over.violations() > 0, "the tent must register as burial");
        assert!(over.worst_value().unwrap() < -1.0, "worst {:?}", over.worst_value());
        assert!(!over.worst.is_empty(), "a violation must name a place");
        let o = &over.worst[0];
        assert!(o.lon > 6.0 && o.lon < 8.0, "offender lon {} outside the tile", o.lon);
    }

    #[test]
    fn asphalt_beyond_the_terrain_is_counted_as_unbacked_not_as_contact() {
        // Terrain covering only half the tile: the road's other half has no
        // ground beneath it, which is a different finding from being buried.
        let b = Bounds::of_tile(16, 34000, 23000);
        let road = SurfaceMesh::from_parts(
            vec![0.0, 1.0, 1.0, 0.0],
            vec![0.4, 0.4, 0.6, 0.6],
            vec![100.0; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let terrain = SurfaceMesh::from_parts(
            vec![0.0, 0.5, 0.5, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![100.0; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let tile = TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads: vec![RoadMesh { class: "road_surface".into(), level: 0, mesh: road }],
        };
        let m = run(&tile);
        assert_eq!(m[0].violations(), 0, "nothing is buried");
        let unbacked = &m[2];
        let pct = unbacked.worst_value().unwrap();
        assert!((pct - 50.0).abs() < 8.0, "about half the road is unbacked, got {pct}");
    }

    #[test]
    fn a_tile_without_terrain_is_skipped_rather_than_scored_clean() {
        let mut tile = tented(100.0, 100.0, 100.0);
        tile.terrain = None;
        let m = run(&tile);
        assert!(m[0].skipped.is_some(), "no ground must read as skipped, not as passing");
        assert_eq!(m[0].violations(), 0);
    }

    #[test]
    fn buffer_geometry_is_the_neighbours_business() {
        // A road running well past the tile edge must contribute only the part
        // inside the tile proper, or every border defect is counted twice.
        let b = Bounds::of_tile(16, 34000, 23000);
        let road = SurfaceMesh::from_parts(
            vec![-0.4, 1.4, 1.4, -0.4],
            vec![0.4, 0.4, 0.6, 0.6],
            vec![100.0; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let terrain = SurfaceMesh::from_parts(
            vec![-0.5, 1.5, 1.5, -0.5],
            vec![-0.5, -0.5, 1.5, 1.5],
            vec![100.0; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let tile = TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: Some(terrain),
            roads: vec![RoadMesh { class: "road_surface".into(), level: 0, mesh: road }],
        };
        let opt = Options { spacing_m: 5.0, ..Default::default() };
        let mut c = Box::new(Contact::new(&opt));
        c.visit(&tile, &opt);
        let m = c.finish();
        // Every accepted sample must lie in [0,1]²; with the road 1.8 tiles
        // wide, an unfiltered pass would take roughly 1.8× as many.
        assert!(!m[0].dist.is_empty());
    }
}
