//! Per-worker ground sampler: the one handle everything in the emit path reads
//! ground heights through.
//!
//! Wraps the worker's own DEM reader (its decoded-tile cache is not shareable)
//! around the shared, immutable [`GroundModel`]. Consumers ask either for the
//! raw engineered ground ([`GroundSampler::ground`]) or for the *rendered*
//! ground ([`GroundSampler::surface`]) — the triangulated terrain-lattice
//! height that matches what the client draws at that zoom, which is what
//! draped geometry must sit on (invariant 4).

use std::sync::Arc;

use crate::dem::Dem;
use crate::ground::GroundModel;
use crate::project::Bounds;
use crate::terrain;

pub struct GroundSampler {
    dem: Option<Dem>,
    ground: Arc<GroundModel>,
    /// Reusable earthwork-query buffer (grid hits per sample).
    scratch: Vec<u32>,
}

impl GroundSampler {
    pub fn new(dem: Option<Dem>, ground: Arc<GroundModel>) -> GroundSampler {
        GroundSampler { dem, ground, scratch: Vec::new() }
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

    /// The *rendered* ground at `(lon, lat)`: the engineered ground evaluated
    /// through the global zoom-`z` terrain lattice anchored at `bounds`, so it
    /// matches the triangulated mesh the client draws exactly.
    pub fn surface(&mut self, bounds: &Bounds, lon: f64, lat: f64, z: u8) -> f64 {
        let (dem, model, scratch) = (&mut self.dem, &self.ground, &mut self.scratch);
        terrain::surface_height(bounds, lon, lat, &mut |a, o| {
            let raw = match dem {
                Some(d) => d.elevation(a, o, z),
                None => 0.0,
            };
            model.height(a, o, raw, scratch)
        })
    }
}
