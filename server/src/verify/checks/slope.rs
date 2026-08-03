//! Invariant 6: annotation noise and DEM outliers may cost detail, but must
//! never produce spectacle.
//!
//! The cheapest useful check in the set, and the only one that needs no second
//! surface: the steepest edge of each triangle, as rise over plan run. It earns
//! its place because it names a pathology the cross-mesh checks structurally
//! cannot see.
//!
//! When the earthwork bench limit was raised to close the field-level gaps, the
//! *drawn* result got worse, and the reason was that each new bench on a steep
//! flank manufactured a retaining wall: at one spot three terrain vertices
//! within 0.4 m of each other spanning 10 m of height. That triangle holds
//! almost no plan area, so it covers almost no road samples and barely moves a
//! contact metric — but it is a vertical cliff in the middle of a hillside, and
//! it is what the tail of the burial distribution was made of. As a slope it is
//! a 25:1 face and impossible to miss.
//!
//! Two filters keep the metric meaning what its title says, and both were added
//! after measuring what an unfiltered version actually counted:
//!
//! - **A rise floor.** Ratio alone cannot tell a wall from a sliver: a face
//!   spanning two millimetres at 4:1 is quantization, one spanning eight metres
//!   at 4:1 is a cliff. Only faces spanning at least [`VISIBLE_M`] count.
//! - **Silhouette exclusion, for the carriageway only.** The asphalt's boundary
//!   *is* the kerb, and a kerb is vertical by design; unfiltered, 44 % of the
//!   steep faces counted were that rim, which no change could ever remove. The
//!   terrain has no such designed edge, so its boundary still counts — and once
//!   the ground is cut back to the kerb, the edge of that hole is exactly where
//!   a new wall would appear.
//!
//! ## Steepness is not the whole of spectacle
//!
//! A steepness metric answers "how steep", and cannot answer "does the ground
//! agree with itself" — the two come apart precisely where this invariant is
//! hardest. A retaining wall beside a road is steep and *correct*: it is what
//! is there, and drawn as a face it reads as one. The same wall drawn as a row
//! of triangular teeth is the same steepness and a defect, and it was the
//! defect a camera 25 m over a Territet switchback found while every metric in
//! this scorecard sat still.
//!
//! So the second question is asked separately, per *vertex*: how far does the
//! surface stand off the plane its own neighbours define
//! ([`SurfaceMesh::vertex_residuals`])? Along a wall the answer is near zero —
//! the wall's vertices lie on the wall. Along a sawtooth every second vertex
//! stands off it by the tooth's height, in alternating directions. That is
//! `slope.terrain_tearing`, and it is the only metric here that moves when a
//! field steps somewhere no contact line runs.

use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// Steeper than any drivable road: 30 % is beyond the steepest public highway
/// grade, so a carriageway face past it is not a road, it is a fold.
const ROAD_GRADE: f64 = 0.30;

/// A 2:1 face. Natural ground reaches this on a cliff; engineered ground
/// reaches it only where an earthwork manufactured a wall, which is what the
/// batter limits exist to prevent.
const GROUND_SLOPE: f64 = 2.0;

/// A face spanning less height than this cannot be seen however steep its
/// ratio, so counting it only dilutes the metric.
const VISIBLE_M: f64 = 0.10;

/// How far a terrain vertex may stand off the plane of its neighbours before
/// the surface counts as torn rather than shaped.
///
/// Set from the measured population, not from taste. Over the Montreux extract
/// at z16 the residual is a spike at zero with a long tail: the median is
/// centimetres — a lattice on a DEM is very nearly planar cell to cell — and
/// real landform (a ridge crest, a stream notch, the top of a drawn wall)
/// occupies the range up to a few tens of centimetres. Past
/// [`TEARING_M`] the neighbours no longer describe a surface the vertex is on,
/// which at the ~3 m detail cell means the mesh is alternating.
const TEARING_M: f64 = 0.50;

pub struct Slope {
    road: Dist,
    road_worst: Worst,
    ground: Dist,
    ground_worst: Worst,
    tearing: Dist,
    tearing_worst: Worst,
}

impl Slope {
    pub fn new(opt: &Options) -> Slope {
        Slope {
            // Slopes are unbounded ratios; 0–64 covers everything short of the
            // degenerate vertical face, which the exact maximum still reports.
            road: Dist::new(0.0, 64.0),
            road_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            ground: Dist::new(0.0, 64.0),
            ground_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            // Magnitude only: a tooth's pit is the same defect as its peak, and
            // keeping the sign would let the two cancel in every summary.
            tearing: Dist::new(0.0, 32.0),
            tearing_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }

    fn visit_terrain_tearing(&mut self, tile: &TileScene, terrain: &SurfaceMesh) {
        terrain.vertex_residuals(&tile.scale, |s| {
            let amp = s.tearing();
            self.tearing.push(amp);
            if amp > TEARING_M {
                let (lon, lat) = tile.lonlat(s.x, s.y);
                self.tearing_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: amp,
                    note: format!(
                        "terrain alternates {:.2} m up / {:.2} m down between neighbouring \
                         vertices, tile {}/{}/{}",
                        s.residual.abs(),
                        s.opposed.abs(),
                        tile.z,
                        tile.x,
                        tile.y
                    ),
                });
            }
        });
    }
}

impl Check for Slope {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        // Every face, not each tile's worst: the question is how much of the
        // drawn ground is wall, and a per-tile maximum answers only how many
        // tiles contain at least one.
        if let Some(terrain) = &tile.terrain {
            terrain.face_slopes(&tile.scale, |f| {
                if f.rise < VISIBLE_M {
                    return;
                }
                self.ground.push(f.slope);
                if f.slope > GROUND_SLOPE {
                    let (lon, lat) = tile.lonlat(f.x, f.y);
                    self.ground_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: f.slope,
                        note: format!(
                            "terrain face at {:.0}:1 spanning {:.2} m, tile {}/{}/{}",
                            f.slope, f.rise, tile.z, tile.x, tile.y
                        ),
                    });
                }
            });
            self.visit_terrain_tearing(tile, terrain);
        }
        for road in tile.roads.iter().filter(|r| r.is_pavement()) {
            let rim = road.mesh.boundary_faces();
            road.mesh.face_slopes(&tile.scale, |f| {
                if f.rise < VISIBLE_M || rim.get(f.index).copied().unwrap_or(false) {
                    return;
                }
                self.road.push(f.slope);
                if f.slope > ROAD_GRADE {
                    let (lon, lat) = tile.lonlat(f.x, f.y);
                    self.road_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: f.slope,
                        note: format!(
                            "interior asphalt at {:.0} % spanning {:.2} m",
                            f.slope * 100.0,
                            f.rise
                        ),
                    });
                }
            });
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![
            Metric {
                id: "slope.terrain_face".into(),
                invariant: 6,
                title: "Drawn terrain face steepness".into(),
                detail: format!(
                    "Rise over plan run of every terrain triangle spanning at least \
                     {VISIBLE_M:.2} m of height. A wall sliver — vertices centimetres apart \
                     spanning metres of height — is a manufactured cliff holding almost no plan \
                     area, so it hides from every check that samples by area while being exactly \
                     what the burial tail is made of."
                ),
                sense: Sense::HigherIsWorse,
                threshold: GROUND_SLOPE,
                skipped: self.ground.is_empty().then(|| "no terrain mesh at this zoom".to_string()),
                dist: self.ground,
                worst: self.ground_worst.into_vec(),
            },
            Metric {
                id: "slope.carriageway_face".into(),
                invariant: 6,
                title: "Interior carriageway face steepness".into(),
                detail: format!(
                    "The same for the at-grade road surface, excluding the mesh silhouette — the \
                     kerb rim is vertical by design. A drivable surface has a class grade \
                     ceiling, so an interior face past {:.0} % spanning {VISIBLE_M:.2} m or more \
                     is a fold in the meshing, not a hill.",
                    ROAD_GRADE * 100.0
                ),
                sense: Sense::HigherIsWorse,
                threshold: ROAD_GRADE,
                skipped: self
                    .road
                    .is_empty()
                    .then(|| "no at-grade road surface at this zoom".to_string()),
                dist: self.road,
                worst: self.road_worst.into_vec(),
            },
            Metric {
                id: "slope.terrain_tearing".into(),
                invariant: 6,
                title: "Drawn terrain standing off its own neighbours".into(),
                detail: format!(
                    "How far each interior terrain vertex sits from the plane its neighbours \
                     define. Steepness cannot separate a wall from a wall drawn as teeth — both \
                     are steep — but a wall's vertices lie along it and a sawtooth's alternate \
                     across it. Past {TEARING_M:.2} m at a detail cell the ground is stepping \
                     where nothing holds the step, which at a grazing view is a torn edge."
                ),
                sense: Sense::HigherIsWorse,
                threshold: TEARING_M,
                skipped: self
                    .tearing
                    .is_empty()
                    .then(|| "no terrain mesh at this zoom".to_string()),
                dist: self.tearing,
                worst: self.tearing_worst.into_vec(),
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

    fn tile(terrain: Option<SurfaceMesh>, roads: Vec<RoadMesh>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene { z: 16, x: 34000, y: 23000, scale: Scale::of(&b), bounds: b, terrain, roads }
    }

    fn run(t: &TileScene) -> Vec<Metric> {
        let opt = Options::default();
        let mut s = Box::new(Slope::new(&opt));
        s.visit(t, &opt);
        s.finish()
    }

    #[test]
    fn ordinary_ground_is_not_flagged() {
        // A hillside climbing 20 m across a third of the tile: well under 2:1.
        let ground = SurfaceMesh::from_parts(
            vec![0.0, 0.3, 0.3, 0.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![100.0, 120.0, 120.0, 100.0],
            vec![0, 1, 2, 0, 2, 3],
        )
        .unwrap();
        let m = run(&tile(Some(ground), Vec::new()));
        assert_eq!(m[0].violations(), 0, "slope was {:?}", m[0].worst_value());
    }

    #[test]
    fn a_manufactured_wall_sliver_is_caught() {
        // The pathology from the terrain-hole study: three vertices within
        // half a metre of each other spanning 10 m of height.
        let b = Bounds::of_tile(16, 34000, 23000);
        let scale = Scale::of(&b);
        let d = (0.5 / scale.mx) as f32;
        let ground = SurfaceMesh::from_parts(
            vec![0.5, 0.5 + d, 0.5],
            vec![0.5, 0.5, 0.5 + d],
            vec![100.0, 110.0, 100.0],
            vec![0, 1, 2],
        )
        .unwrap();
        let m = run(&tile(Some(ground), Vec::new()));
        assert!(m[0].violations() > 0, "a 20:1 face must be flagged");
        assert!(m[0].worst_value().unwrap() > 10.0, "{:?}", m[0].worst_value());
        assert!(m[0].worst[0].note.contains("spanning"), "the note must give the height spanned");
    }

    #[test]
    fn a_steep_face_spanning_nothing_is_not_a_wall() {
        // Vertices half a metre apart spanning 2 cm: a 1:25 ratio in the other
        // direction — steep by ratio, invisible in fact.
        let b = Bounds::of_tile(16, 34000, 23000);
        let scale = Scale::of(&b);
        let d = (0.005 / scale.mx) as f32; // 5 mm apart in plan
        let ground = SurfaceMesh::from_parts(
            vec![0.5, 0.5 + d, 0.5],
            vec![0.5, 0.5, 0.5 + d],
            vec![100.0, 100.02, 100.0],
            vec![0, 1, 2],
        )
        .unwrap();
        let m = run(&tile(Some(ground), Vec::new()));
        assert!(m[0].dist.is_empty(), "a 2 cm rise must not be counted as a cliff");
    }

    #[test]
    fn the_kerb_rim_is_not_counted_but_interior_asphalt_is() {
        // A carriageway whose silhouette drops vertically (the kerb) and whose
        // interior is flat. Every face here is silhouette, so nothing counts.
        let b = Bounds::of_tile(16, 34000, 23000);
        let scale = Scale::of(&b);
        let w = (4.0 / scale.mx) as f32;
        let rim = RoadMesh {
            class: "road_surface".into(),
            level: 0,
            mesh: SurfaceMesh::from_parts(
                vec![0.5, 0.5 + w, 0.5 + w],
                vec![0.5, 0.5, 0.5 + w],
                vec![100.0, 100.0, 96.0],
                vec![0, 1, 2],
            )
            .unwrap(),
        };
        let m = run(&tile(None, vec![rim]));
        assert!(m[1].dist.is_empty(), "an all-silhouette mesh has no interior to score");
    }

    /// A fully triangulated grid of carriageway, `xs`/`ys` in metres from the
    /// tile centre, with a height per column. Only the outermost ring of
    /// triangles is silhouette, so the columns in the middle are genuinely
    /// interior — which is what the rim exclusion makes it necessary to build.
    fn grid(xs: &[f64], ys: &[f64], z_per_col: &[f32]) -> RoadMesh {
        let scale = Scale::of(&Bounds::of_tile(16, 34000, 23000));
        let (nx, ny) = (xs.len(), ys.len());
        let (mut px, mut py, mut pz) = (Vec::new(), Vec::new(), Vec::new());
        for &y in ys {
            for (i, &x) in xs.iter().enumerate() {
                px.push((0.5 + x / scale.mx) as f32);
                py.push((0.5 + y / scale.my) as f32);
                pz.push(z_per_col[i]);
            }
        }
        let mut idx = Vec::new();
        for r in 0..ny - 1 {
            for c in 0..nx - 1 {
                let (a, b) = ((r * nx + c) as u32, (r * nx + c + 1) as u32);
                let (d, e) = (((r + 1) * nx + c) as u32, ((r + 1) * nx + c + 1) as u32);
                idx.extend_from_slice(&[a, b, e, a, e, d]);
            }
        }
        RoadMesh {
            class: "road_surface".into(),
            level: 0,
            mesh: SurfaceMesh::from_parts(px, py, pz, idx).unwrap(),
        }
    }

    #[test]
    fn an_interior_fold_in_the_asphalt_is_caught() {
        // Columns a metre apart across the middle of the sheet drop 5 m: a
        // vertical fold with triangles on both sides, so no silhouette edge.
        let road = grid(
            &[0.0, 10.0, 11.0, 21.0, 31.0],
            &[0.0, 10.0, 20.0, 30.0],
            &[100.0, 100.0, 95.0, 95.0, 95.0],
        );
        let m = run(&tile(None, vec![road]));
        assert!(m[1].violations() > 0, "a 5 m drop over 1 m of asphalt must be flagged");
        assert!(m[1].worst_value().unwrap() > 4.0, "{:?}", m[1].worst_value());
        assert!(m[1].worst[0].note.contains("interior"));
    }

    /// A fully triangulated terrain lattice, `xs`/`ys` in metres from the tile
    /// centre, with a height per column — the same shape as [`grid`] but as
    /// ground rather than asphalt.
    fn ground(xs: &[f64], ys: &[f64], z_per_col: &[f32]) -> SurfaceMesh {
        grid(xs, ys, z_per_col).mesh
    }

    /// The tearing metric, which is the third one [`Slope`] reports.
    fn tearing(t: &TileScene) -> Metric {
        run(t).swap_remove(2)
    }

    #[test]
    fn a_wall_is_a_face_and_not_a_tear() {
        // A clean 6 m step across the middle of a 3 m lattice. Its crest reads
        // below the plane of its neighbours and the vertex down the face reads
        // above — one opposite-signed pair, which every drawn wall has and
        // which must not be counted.
        let xs = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0];
        let ys = [0.0, 3.0, 6.0, 9.0, 12.0];
        let m = tearing(&tile(
            Some(ground(&xs, &ys, &[100.0, 100.0, 100.0, 106.0, 106.0, 106.0, 106.0])),
            Vec::new(),
        ));
        assert!(!m.dist.is_empty(), "the lattice interior must have been measured");
        assert_eq!(m.violations(), 0, "a wall must not read as tearing: {:?}", m.worst_value());
    }

    #[test]
    fn a_sawtooth_is_caught() {
        // The Territet defect in the small: the same flank, but with the
        // ground alternating 1.5 m about it cell by cell.
        let xs = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0];
        let ys = [0.0, 3.0, 6.0, 9.0, 12.0];
        let m = tearing(&tile(
            Some(ground(&xs, &ys, &[100.0, 101.5, 100.0, 101.5, 100.0, 101.5, 100.0])),
            Vec::new(),
        ));
        assert!(m.violations() > 0, "an alternating lattice must be flagged");
        // Reported in metres, and less than the peak-to-trough 1.5 m: the
        // teeth here run in ridges, so two of a vertex's six neighbours sit on
        // its own tooth and the plane it is measured against tilts towards it.
        // The metric is a departure from the surface, not a tooth height.
        let w = m.worst_value().unwrap();
        assert!((0.9..1.5).contains(&w), "amplitude in metres expected, got {w}");
        assert!(m.worst[0].note.contains("alternates"));
    }

    #[test]
    fn a_smooth_hillside_does_not_tear() {
        let xs = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0];
        let ys = [0.0, 3.0, 6.0, 9.0, 12.0];
        let m = tearing(&tile(
            Some(ground(&xs, &ys, &[100.0, 102.5, 105.0, 107.5, 110.0, 112.5, 115.0])),
            Vec::new(),
        ));
        assert!(!m.dist.is_empty());
        assert_eq!(m.violations(), 0, "a 1:1.2 flank is steep, not torn");
    }

    #[test]
    fn a_genuinely_steep_street_is_left_alone() {
        // S9 says knowing when to do nothing is part of the job: a 15 % climb
        // is a real road and must not be reported as a fold.
        let road = grid(
            &[0.0, 20.0, 40.0, 60.0, 80.0],
            &[0.0, 6.0, 12.0, 18.0],
            &[100.0, 103.0, 106.0, 109.0, 112.0],
        );
        let m = run(&tile(None, vec![road]));
        assert!(!m[1].dist.is_empty(), "the interior must actually have been measured");
        assert_eq!(m[1].violations(), 0, "worst was {:?}", m[1].worst_value());
    }
}
