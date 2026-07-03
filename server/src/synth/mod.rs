//! Stage 4 — synthesize geometry from the solved model (docs/GENERATION.md §6).
//!
//! Parameterized generators per feature kind, all reading solved heights
//! ([`SolvedModel`]) and the engineered ground ([`GroundSampler`]), adding no
//! new inference. Which generator runs is decided when the feature is emitted
//! in phase 1 and carried on its sort record as a [`Synth`] tag; the emit
//! worker dispatches on it here ([`emit`]).
//!
//! Every generator degrades rather than fails (invariant 6): a structure whose
//! corridor has no solved profile, or whose solid comes out empty (a tunnel
//! annotation over flat ground), falls back to a plain draped road — something
//! plain, never something wrong.

pub mod road;
pub mod structure;

use crate::ground::sampler::GroundSampler;
use crate::project::Bounds;
use crate::scene::{CorridorId, SpanKind};
use crate::solve::SolvedModel;
use crate::tile_build::EncoderFeature;

/// How a feature's 3D geometry is generated at emit time. Decided once, in
/// phase 1, from the scene graph; carried on the sort record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Synth {
    /// No vertical synthesis (land, water, boundaries; buildings and POIs keep
    /// their own elevation stamping for now).
    #[default]
    None,
    /// A road draped on the rendered ground; with a corridor, lifted by the
    /// corridor's solved cut/fill so engineered classes hold their grade.
    Road { corridor: Option<CorridorId> },
    /// A bridge deck or tunnel bore swept along the corridor's solved profile.
    Structure { corridor: CorridorId, kind: SpanKind },
}

/// Runs the feature's generator. A no-op for [`Synth::None`] and for DEM-less
/// runs (nothing is elevated in the flat parity world).
pub fn emit(
    f: &mut EncoderFeature,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    z: u8,
    bounds: &Bounds,
) {
    if !sampler.has_elevation() {
        return;
    }
    match f.synth {
        Synth::None => {}
        Synth::Road { corridor } => {
            let profile = corridor.and_then(|c| solved.profile(c));
            road::bake(f, profile, sampler, z, solved.z_ref, bounds);
        }
        Synth::Structure { corridor, kind } => {
            match solved.profile(corridor) {
                Some(p) if structure::stamp(f, p, kind, bounds) => {}
                // Degradation ladder: no solved profile, or no solid to draw
                // (a tunnel tagged over flat ground) → a plain draped road.
                other => road::bake(f, other, sampler, z, solved.z_ref, bounds),
            }
        }
    }
}
