//! Per-worker ground sampler: the one handle everything in the emit path reads
//! ground heights through.
//!
//! Wraps the worker's own DEM reader (its decoded-tile cache is not shareable)
//! around the shared, immutable [`GroundModel`]. Consumers ask either for the
//! raw engineered ground ([`GroundSampler::ground`]) or for the *rendered*
//! ground ([`GroundSampler::surface`]) — the triangulated terrain-lattice
//! height that matches what the client draws at that zoom, which is what
//! draped geometry must sit on (invariant 4).

use std::collections::HashMap;
use std::sync::Arc;

use crate::dem::Dem;
use crate::ground::GroundModel;
use crate::project::Bounds;
use crate::terrain::{self, TerrainMesh};
use crate::terrain_cdt;

/// Corner-memo capacity. Entries are ~50 B, so this is a few MiB per worker —
/// hundreds of tiles' worth of lattice corners before an (unlikely) reset.
const CORNER_CAP: usize = 262_144;

pub struct GroundSampler {
    dem: Option<Dem>,
    ground: Arc<GroundModel>,
    /// The run's reference zoom, keying the per-zoom lattice resolution
    /// ([`terrain::grid_for`]) `surface` reads through.
    z_ref: u8,
    /// Whether detail meshes are breakline-constrained (docs/GROUND.md §3);
    /// `--no-breaklines` turns it off.
    breaklines: bool,
    /// Reusable earthwork-query buffer (grid hits per sample).
    scratch: Vec<u32>,
    /// Memoized engineered heights at rendered-lattice corners, keyed by the
    /// corner's exact coordinate bits and the sampled zoom. The lattice is
    /// global per zoom and a tile's width is a dyadic rational, so every
    /// frame computes bit-identical corner coordinates — one memo entry
    /// serves the terrain mesh and every draped road vertex that touches the
    /// corner, collapsing the 4-corner fan-out of `surface` into ~one DEM
    /// sample per distinct corner.
    corners: HashMap<(u8, u64, u64), f64>,
}

impl GroundSampler {
    pub fn new(dem: Option<Dem>, ground: Arc<GroundModel>, z_ref: u8) -> GroundSampler {
        GroundSampler {
            dem,
            ground,
            z_ref,
            breaklines: true,
            scratch: Vec::new(),
            corners: HashMap::new(),
        }
    }

    /// Turns the breakline-constrained detail meshes off (`--no-breaklines`).
    pub fn set_breaklines(&mut self, on: bool) {
        self.breaklines = on;
    }

    /// Whether the run has real elevation at all (a DEM was configured).
    pub fn has_elevation(&self) -> bool {
        self.dem.is_some()
    }

    /// The engineered ground height at `(lon, lat)`, sampling the DEM at zoom
    /// `z`. Zero without a DEM (the flat parity world).
    pub fn ground(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        let raw = match &mut self.dem {
            Some(d) => d.elevation(lon, lat, z),
            None => 0.0,
        };
        self.ground.height(lon, lat, raw, &mut self.scratch)
    }

    /// The engineered ground at a rendered-lattice corner, memoized (see
    /// [`GroundSampler::corners`]). The terrain mesh and `surface` both read
    /// corners through this, so each distinct corner costs one DEM sample per
    /// worker however many queries land on it.
    pub fn corner(&mut self, lon: f64, lat: f64, z: u8) -> f64 {
        corner_memo(&mut self.dem, &self.ground, &mut self.corners, &mut self.scratch, lon, lat, z)
    }

    /// The *rendered* ground at `(lon, lat)`: the engineered ground evaluated
    /// through the global zoom-`z` terrain lattice anchored at `bounds`, so it
    /// matches the triangulated mesh the client draws exactly.
    pub fn surface(&mut self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> f64 {
        let grid = terrain::grid_for(z, self.z_ref);
        terrain::surface_height(bounds, grid, lon, lat, &mut |a, o| self.corner(a, o, z))
    }

    /// The exact engineered roadbed height under `(lon, lat)` when the point
    /// lies fully inside a road earthwork's held width — a corridor's roadbed
    /// or a street bench — else `None`. The drape rides this at the reference
    /// zoom, where the rendered lattice is too coarse to hold a street-wide
    /// bench (see `synth::road::surface_height`).
    pub fn bed_target(&mut self, lon: f64, lat: f64) -> Option<f64> {
        self.ground.earthworks().target_at(lon, lat, &mut self.scratch)
    }

    /// The tile's terrain mesh: the engineered ground on the regular lattice,
    /// except at the detail resolution where bench contact lines cross the
    /// tile — there the mesh is the breakline-constrained triangulation that
    /// holds the benches exactly (docs/GROUND.md §3), falling back to the
    /// lattice when the triangulation abstains (invariant 6).
    pub fn terrain_mesh(&mut self, bounds: &Bounds, z: u8) -> (TerrainMesh, f64, f64) {
        let grid = terrain::grid_for(z, self.z_ref);
        if self.breaklines && grid == terrain::TERRAIN_GRID_DETAIL {
            // Pad the query by one cell so a line grazing the border still
            // constrains the edge cells it touches.
            let pad = bounds.width().max(bounds.height()) / grid as f64;
            let bbox =
                (bounds.west - pad, bounds.south - pad, bounds.east + pad, bounds.north + pad);
            let mut ids = Vec::new();
            let mut segments = Vec::new();
            self.ground.breaklines().query(bbox, &mut ids, &mut segments);
            if !segments.is_empty() {
                let (dem, ground, corners, scratch) =
                    (&mut self.dem, &self.ground, &mut self.corners, &mut self.scratch);
                if let Some(mesh) =
                    terrain_cdt::constrained_mesh(grid, bounds, &segments, &mut |lon, lat| {
                        corner_memo(dem, ground, corners, scratch, lon, lat, z)
                    })
                {
                    return mesh;
                }
            }
        }
        terrain::elevated_mesh(grid, bounds, |lon, lat| self.corner(lon, lat, z))
    }
}

/// [`GroundSampler::corner`] with the sampler's fields split apart, so a
/// closure holding the field borrows can call it (the borrow checker cannot
/// split `self` through a closure).
fn corner_memo(
    dem: &mut Option<Dem>,
    ground: &GroundModel,
    corners: &mut HashMap<(u8, u64, u64), f64>,
    scratch: &mut Vec<u32>,
    lon: f64,
    lat: f64,
    z: u8,
) -> f64 {
    let key = (z, lon.to_bits(), lat.to_bits());
    if let Some(&h) = corners.get(&key) {
        return h;
    }
    let raw = match dem {
        Some(d) => d.elevation(lon, lat, z),
        None => 0.0,
    };
    let h = ground.height(lon, lat, raw, scratch);
    if corners.len() >= CORNER_CAP {
        corners.clear();
    }
    corners.insert(key, h);
    h
}
