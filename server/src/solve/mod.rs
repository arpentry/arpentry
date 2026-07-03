//! Stage 2 — solve the vertical model (docs/GENERATION.md §6).
//!
//! One pass over the assembled scene graph turns topology into geometry: every
//! corridor that needs a vertical model (a structure span, or an engineered
//! grade) gets a [`Profile`] — road-surface heights everywhere along it,
//! anchored to the reference terrain at its at-grade spans and interpolated at
//! a gentle grade across its structures.
//!
//! The reference terrain is the *rendered* ground at the reference zoom (the
//! run's maximum): the same global [`terrain::surface_height`] lattice the
//! emit workers mesh, so a solved at-grade anchor sits exactly on the drawn
//! ground at that zoom. The solved heights are a function of the scene graph
//! and the DEM only — never of a tile window — so every tile fragment reads
//! identical heights (invariant 5), and heights do not change between zoom
//! levels (no popping).

pub mod crossings;
pub mod portals;
pub mod profile;

use std::path::Path;
use std::sync::Mutex;

use crate::dem::Dem;
use crate::project::Bounds;
use crate::scene::{CorridorId, SceneGraph};
use crate::terrain;

pub use profile::Profile;

type Error = Box<dyn std::error::Error + Send + Sync>;

/// The solved vertical model: one profile per corridor that needs one, indexed
/// by [`CorridorId`]. Immutable after the solve; shared by every emit worker.
pub struct SolvedModel {
    profiles: Vec<Option<Profile>>,
    /// The zoom whose rendered terrain lattice anchored the solve.
    pub z_ref: u8,
}

impl SolvedModel {
    /// A model with no profiles — the DEM-less run, where nothing is elevated.
    pub fn empty(z_ref: u8) -> SolvedModel {
        SolvedModel { profiles: Vec::new(), z_ref }
    }

    /// Wraps already-solved profiles — for tests and stage-isolated tooling.
    pub fn from_profiles(profiles: Vec<Option<Profile>>, z_ref: u8) -> SolvedModel {
        SolvedModel { profiles, z_ref }
    }

    pub fn profile(&self, corridor: CorridorId) -> Option<&Profile> {
        self.profiles.get(corridor as usize)?.as_ref()
    }

    /// Number of corridors carrying a solved profile.
    pub fn solved_count(&self) -> usize {
        self.profiles.iter().filter(|p| p.is_some()).count()
    }
}

/// Solves the scene graph against the DEM at reference zoom `z_ref`,
/// parallelized over `threads` workers (each owning its own DEM reader).
/// Without a DEM there is nothing to anchor to: the model is empty and roads
/// stay flat, exactly like the terrain they would drape on.
pub fn run(
    scene: &SceneGraph,
    terrain_path: Option<&Path>,
    z_ref: u8,
    threads: usize,
) -> Result<SolvedModel, Error> {
    let Some(path) = terrain_path else {
        return Ok(SolvedModel::empty(z_ref));
    };

    let todo: Vec<usize> = scene
        .corridors
        .iter()
        .enumerate()
        .filter(|(_, c)| c.needs_profile())
        .map(|(i, _)| i)
        .collect();
    let mut profiles: Vec<Option<Profile>> = Vec::new();
    profiles.resize_with(scene.corridors.len(), || None);

    let threads = threads.max(1).min(todo.len().max(1));
    let next = Mutex::new(0usize);
    let results: Mutex<&mut Vec<Option<Profile>>> = Mutex::new(&mut profiles);
    std::thread::scope(|scope| -> Result<(), Error> {
        let mut handles = Vec::with_capacity(threads);
        for _ in 0..threads {
            handles.push(scope.spawn(|| -> Result<(), Error> {
                let mut dem = Dem::open(path)?;
                loop {
                    let i = {
                        let mut n = next.lock().expect("solve queue poisoned");
                        if *n >= todo.len() {
                            break;
                        }
                        let i = *n;
                        *n += 1;
                        i
                    };
                    let c = &scene.corridors[todo[i]];
                    let solved = profile::solve(&c.nodes, &c.spans, c.class.grade_limit(), &mut |p| {
                        reference_surface(&mut dem, z_ref, p.x, p.y)
                    });
                    results.lock().expect("solve results poisoned")[todo[i]] = solved;
                }
                Ok(())
            }));
        }
        for handle in handles {
            handle.join().map_err(|_| "solve worker panicked")??;
        }
        Ok(())
    })?;

    // Clearance at crossings: turn the level ordering into geometry
    // (invariant 3). Runs after every base profile exists, so a lower
    // corridor's height is its solved one.
    crossings::apply(scene, &mut profiles);

    Ok(SolvedModel { profiles, z_ref })
}

/// The rendered-ground height at `(lon, lat)` on the global zoom-`z` lattice —
/// the same surface [`terrain::surface_height`] gives an emit worker meshing
/// the containing tile, so solved anchors sit exactly on the drawn ground.
pub fn reference_surface(dem: &mut Dem, z: u8, lon: f64, lat: f64) -> f64 {
    let b = tile_containing(z, lon, lat);
    terrain::surface_height(&b, lon, lat, &mut |a, o| dem.elevation(a, o, z))
}

/// Bounds of the zoom-`z` tile containing `(lon, lat)` (the lattice anchor;
/// any covering tile yields the same surface since the lattice is global).
pub fn tile_containing(z: u8, lon: f64, lat: f64) -> Bounds {
    let n = (1u64 << z as u32) as f64;
    let x = (((lon + 180.0) / 360.0) * n).floor().clamp(0.0, n - 1.0) as u32;
    let y = (((lat + 90.0) / 180.0) * n).floor().clamp(0.0, n - 1.0) as u32;
    Bounds::of_tile(z, x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_containing_agrees_with_of_tile() {
        let b = tile_containing(14, 6.9185, 46.4355);
        assert!(b.contains(6.9185, 46.4355));
        // Consistent with the tiling scheme: the tile's own bounds contain it.
        let n = (1u64 << 14) as f64;
        let x = (((6.9185 + 180.0) / 360.0) * n).floor() as u32;
        let y = (((46.4355 + 90.0) / 180.0) * n).floor() as u32;
        let direct = Bounds::of_tile(14, x, y);
        assert_eq!(b.west, direct.west);
        assert_eq!(b.south, direct.south);
    }

    #[test]
    fn empty_model_has_no_profiles() {
        let m = SolvedModel::empty(14);
        assert!(m.profile(0).is_none());
        assert_eq!(m.solved_count(), 0);
    }
}
