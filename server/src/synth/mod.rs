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
pub mod carriageway;
pub mod carried;
pub mod cross;
pub mod datum;
pub mod draped;
pub mod height;
pub mod markings;
pub mod pave_mesh;
pub mod pavement;
pub mod poly;
pub mod region;
pub mod sheets;
pub mod road;
pub mod structure;
pub mod walkway;

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
    /// `deck` marks the paint stroke re-emitted over a *bridge* span: it rides
    /// the solved ramp (`Profile::deck_m`) directly at every zoom rather than
    /// draping, so it lies on the deck top and the painted carriageway
    /// continues across the span instead of stopping at the abutment. A tunnel
    /// span re-emits no paint at all — the stroke, its markings and its rail
    /// heads stop at the portal (`pipeline::process_feature`), because a
    /// ribbon riding the bore's road surface is invisible where the terrain
    /// buries it and drawn across the mountain wherever a coarse rung's
    /// chords disagree with the buried run (`paint.buried`).
    Road { corridor: Option<CorridorId>, deck: bool },
    /// A draped pedestrian way the walkway model **drew as a band** — a
    /// sidewalk seated on its street's cross-section, a path standing on the
    /// ground, a registered crossing's zebra (`synth::walkway`). Geometrically
    /// it is a plain draped road and it bakes as one; the tag exists so the
    /// tile stage can drop its cartographic stroke at the walk zooms, where the
    /// band *is* the way and the line would be a second coat over its own
    /// surface (`pipeline::stamp_synth`).
    ///
    /// Phase 1 is the only place that knows: the answer is a lookup in the
    /// walkway model, which stage 4 cannot see and the tile properties do not
    /// carry — `profile::profile` keeps a fixed whitelist of source attributes
    /// and an extra one invented here is dropped on the way to the sorter.
    /// The synth tag is the channel that does survive, and it is already the
    /// answer to "what did phase 1 decide this feature is drawn as".
    DrapedBand,
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
    //
    // A rail stroke has a width too — its formation is in the union — and it
    // still must not read the field. Below the surface zoom the stroke is the
    // railway (from `ROAD_SURFACE_MIN_ZOOM` `paves_via_union` deletes it, the
    // band and decks carrying the surface), and the field's per-vertex sheet
    // resolution (`layer_at`) picks the nearest of the *corridor's own*
    // sources: two rail alignments running metres apart in plan and tens of
    // metres apart in height flip that answer between consecutive vertices —
    // measured as a 446 % grade over one 0.8 m chord. The ballast band cannot
    // flip — a region's layer is fixed — and the stroke lands on it by reading
    // the same per-corridor profile the band's bench holds, so the field buys
    // the rail stroke nothing and costs it a cliff.
    let has_width = crate::value::f64_of(&f.properties, "width_m").is_some_and(|w| w > 0.0);
    let class = crate::value::str_of(&f.properties, "class");
    let paved = has_width
        && (class == Some("marking")
            || crate::priors::Kind::parse(None, class, None).prior().surface
                == crate::priors::Surface::Asphalt);
    let paved_field = paved.then_some(field);
    match f.synth {
        Synth::None => {}
        // A band's line is a draped road in every respect the generator cares
        // about — no corridor, no deck, no field (it has no `width_m`).
        Synth::DrapedBand => {
            road::bake(f, None, false, None, None, sampler, z, solved.z_ref, bounds)
        }
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
            if !draped::stamp(f, sampler, z, solved.z_ref, bounds) {
                road::bake(f, None, false, None, paved_field, sampler, z, solved.z_ref, bounds);
            }
        }
        Synth::Structure { corridor, kind } => {
            let profile = solved.profile(corridor);
            let stamped =
                profile.map(|p| structure::stamp(f, p, kind, sampler, z, solved.z_ref, bounds));
            match stamped {
                // A solid, or a tube the finished ground hides: either way this
                // feature has had everything it is owed. Draping the hidden one
                // would paint a surface line over a bore running under the
                // terrace that line would be drawn on.
                Some(structure::Stamped::Solid | structure::Stamped::Hidden) => {}
                // Degradation ladder: no solved profile, or no solid to draw
                // (a tunnel tagged over flat ground) → a plain draped road, on
                // the same terms any other road gets. Its own layer, and the
                // field only if it is part of the paved surface: a demoted
                // structure is still the corridor it was, and reading the field
                // on layer 0 would drape a flyover onto the street beneath it.
                _ => road::bake(
                    f,
                    profile,
                    false,
                    Some(corridor),
                    paved_field,
                    sampler,
                    z,
                    solved.z_ref,
                    bounds,
                ),
            }
        }
    }
}
