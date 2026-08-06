//! Stage 4 — synthesize geometry from the solved model (docs/GENERATION.md §5).
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

pub mod area;
pub mod carried;
pub mod draped;
pub mod height;
pub mod junction;
pub mod markings;
pub mod pave_mesh;
pub mod pavement;
pub mod poly;
pub mod region;
pub mod sheets;
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
    /// `deck` marks the paint stroke re-emitted over a *structure* span — a
    /// bridge deck or a tunnel bore. Both carry the road surface on the same
    /// solved ramp (`Profile::deck_m`, which the bore sweep rides too), so the
    /// paint rides that ramp directly at every zoom rather than draping: it
    /// lies on the deck top of a bridge and on the bore's road surface of a
    /// tunnel, so the road surface continues across the structure instead of
    /// stopping at the abutment or portal. Where a bore runs buried the ramp
    /// dips under the hill, so the ribbon sinks with the mesh and the terrain
    /// occludes it — never floating over the ground the tunnel passes beneath.
    Road { corridor: Option<CorridorId>, deck: bool },
    /// A bridge deck or tunnel bore swept along the corridor's solved profile.
    Structure { corridor: CorridorId, kind: SpanKind },
    /// A structure carried by a **draped** feature — a footbridge, a path over
    /// a stream. It has no corridor and no solved profile, because carrying a
    /// span is not a promotion (§4.2): the deck is fitted to the finished
    /// ground at its two ends and constrains nothing. See [`draped`].
    DrapedDeck,
}

/// Runs the feature's generator. A no-op for [`Synth::None`] and for DEM-less
/// runs (nothing is elevated in the flat parity world).
pub fn emit(
    f: &mut EncoderFeature,
    field: &height::HeightField,
    sampler: &mut GroundSampler,
    solved: &SolvedModel,
    z: u8,
    bounds: &Bounds,
) {
    if !sampler.has_elevation() {
        return;
    }
    // `width_m` is what marks a feature as belonging to the paved surface: a
    // carriageway has one, and so does a marking painted on it (which must ride
    // the same answer). A footway, cycleway or track has none — it is draped
    // geometry beside the network, not part of it, and blending it into the
    // field would hand it whatever road happens to cover the point, including
    // that road's raise-only clamp to its own profile. A path crossing under a
    // bridge approach was lifted metres into the air by exactly that.
    let paved = f.properties.iter().any(|(k, v)| {
        k.as_str() == "width_m" && matches!(v, crate::value::Value::Double(w) if *w > 0.0)
    });
    let paved_field = paved.then_some(field);
    match f.synth {
        Synth::None => {}
        Synth::Road { corridor, deck } => {
            let profile = corridor.and_then(|c| solved.profile(c));
            // The corridor rides along so the paint can ask, per vertex, which
            // grade-separation sheet its own road is on there — it must not
            // blend with whatever passes beneath.
            road::bake(f, profile, deck, corridor, paved_field, sampler, z, solved.z_ref, bounds);
        }
        Synth::DrapedDeck => {
            // Fitted, not solved. Falls back to a plain draped line when there
            // is no solid to draw, exactly as a solved structure does.
            if !draped::stamp(f, sampler, solved.z_ref, bounds) {
                road::bake(f, None, false, None, paved_field, sampler, z, solved.z_ref, bounds);
            }
        }
        Synth::Structure { corridor, kind } => {
            match solved.profile(corridor) {
                Some(p) if structure::stamp(f, p, kind, bounds) => {}
                // Degradation ladder: no solved profile, or no solid to draw
                // (a tunnel tagged over flat ground) → a plain draped road, on
                // the same terms any other road gets. Its own layer, and the
                // field only if it is part of the paved surface: a demoted
                // structure is still the corridor it was, and reading the field
                // on layer 0 would drape a flyover onto the street beneath it.
                other => {
                    road::bake(
                        f,
                        other,
                        false,
                        Some(corridor),
                        paved_field,
                        sampler,
                        z,
                        solved.z_ref,
                        bounds,
                    )
                }
            }
        }
    }
}
