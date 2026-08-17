//! Per-zoom structure datum: at coarse zooms, structures ride the drawn
//! ground by the same local displacement the at-grade world does.
//!
//! At coarse zooms the at-grade world is expressed relative to that zoom's
//! drawn ground (docs/GROUND.md §4: `surface(z) + max(road − surface(z_ref),
//! 0)`), while structures are swept from absolute solved ramps. At any
//! structure interface the two datums differ by the coarse lattice's error —
//! metres on a real flank — which is what `seam.band_deck_step` and
//! `seam.abutment_step` read at 90+ % over at z14/z15.
//!
//! The correction is the local **datum-shift field**: what the drawn ground
//! at the asking zoom reads over what the reference rung reads at the same
//! point. Every structure vertex adds it, so a span rides the coarse canvas
//! by the same local displacement the at-grade world already does, and every
//! *relative* vertical relation — the step at an abutment, the clearance over
//! a crossed road, a bore roof's burial — is inherited from the reference
//! world by construction. Zero at the reference zoom by definition, so the
//! detail rung is untouched.
//!
//! A first cut blended one correction per span between its two end arrivals;
//! it closed the abutments but daylit bore roofs mid-span and missed the
//! lattice error at mid-span crossings (`clearance.bore_cover` 6.8 → 20 % at
//! z14). The field form has neither failure mode: burial and clearance are
//! preserved pointwise, not interpolated.
//!
//! Everything here is a function of the two global per-zoom lattices, so
//! every tile that asks derives the identical shift (invariant 5).

use crate::ground::sampler::GroundSampler;
use crate::solve;

/// Whether the per-zoom datum is on for this run. `ARPT_NO_ZOOM_DATUM=1`
/// restores absolute structures at coarse zooms, so an A/B re-tile of this is
/// a flag rather than a patch — the same reason `--no-hole` and
/// `ARPT_NO_ABUTMENT_CUT` exist. Read once: the shift runs per vertex.
pub fn enabled() -> bool {
    static DISABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    !*DISABLED.get_or_init(|| std::env::var_os("ARPT_NO_ZOOM_DATUM").is_some())
}

/// The datum shift at `(lon, lat)` for zoom `z`: drawn ground at `z` minus
/// drawn ground at the reference rung. Zero at the reference zoom or with the
/// prototype off.
///
/// This is the *pointwise* field, for consumers whose neighbours are also
/// pointwise — the intersection pin, whose legs each read `surface(z)` at the
/// same vertex. Anything on a structure span reads [`shift_at_arc`] instead.
pub fn shift(sampler: &mut GroundSampler, z: u8, z_ref: u8, lon: f64, lat: f64) -> f64 {
    if z >= z_ref || !enabled() {
        return 0.0;
    }
    let zb = solve::tile_containing(z, lon, lat);
    let rb = solve::tile_containing(z_ref, lon, lat);
    sampler.surface(&zb, lon, lat, z) - sampler.surface(&rb, lon, lat, z_ref)
}

/// Station spacing along the corridor arc, metres — the breakpoints of the
/// piecewise-linear shift every span consumer reads. Matches the structure
/// sweep's own `SEGMENT_M`, so the solid's sections are at least as dense as
/// the curve they sample.
const STATION_M: f64 = 4.0;

/// The datum shift at arc position `arc` of `profile`, interpolated in a
/// piecewise-linear curve whose stations are global multiples of
/// [`STATION_M`] along the corridor arc.
///
/// The solid's sweep sections and the paint's densified vertices sit at
/// *different* stations, and two piecewise-linear reconstructions of the raw
/// pointwise field diverge mid-segment wherever the coarse lattice creases —
/// at z15 that put a marking a metre off the solid it rides (past the verify
/// bracket) while both were individually "correct". Reading one shared curve
/// makes every consumer of a span reconstruct the same function; stations
/// are anchored to the global arc, so every tile derives the identical curve
/// (invariant 5).
pub fn shift_at_arc(
    profile: &crate::solve::Profile,
    arc: f64,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
) -> f64 {
    if z >= z_ref || !enabled() {
        return 0.0;
    }
    let (a0, a1) = bracket(profile, arc);
    if a1 - a0 < 1e-9 {
        return station(profile, arc, sampler, z, z_ref);
    }
    let t = ((arc - a0) / (a1 - a0)).clamp(0.0, 1.0);
    let d0 = station(profile, a0, sampler, z, z_ref);
    let d1 = station(profile, a1, sampler, z, z_ref);
    d0 + (d1 - d0) * t
}

/// The station bracket around `arc`: the enclosing [`STATION_M`] multiples,
/// clamped to any at-grade-run boundary falling inside them.
///
/// The clamp is what keeps a span end exact. The reference field legitimately
/// *steps* at a run boundary — the ground leaves the approach's bench and
/// falls away under the span, which is the abutment face — and a bracket
/// straddling it smears that wall into the last metres of the deck. Measured
/// as decimetre steps at every coarse-rung handover pairing point
/// (`seam.abutment_step` z14 80 % over at a 5 cm gate) while the same build's
/// tails were fine. Clamped, the shift at the boundary is the boundary's own
/// pointwise value — exactly what the at-grade side computes at the same
/// plan point — and the transition happens across the first in-span interval,
/// where the reference ground really does leave the bench.
fn bracket(profile: &crate::solve::Profile, arc: f64) -> (f64, f64) {
    let mut a0 = (arc / STATION_M).floor() * STATION_M;
    let mut a1 = a0 + STATION_M;
    let arcs = profile.arc();
    let at = profile.at_grade();
    if arcs.len() != at.len() || at.is_empty() {
        return (a0, a1);
    }
    let lo = arcs.partition_point(|&v| v < a0).max(1);
    let hi = arcs.partition_point(|&v| v <= a1).min(arcs.len());
    for i in lo..hi {
        if at[i] != at[i - 1] {
            let b = arcs[i];
            if b <= arc {
                a0 = a0.max(b);
            } else {
                a1 = a1.min(b);
            }
        }
    }
    (a0, a1)
}

/// The pointwise shift at one arc station. `point_at_arc` clamps beyond the
/// corridor's ends, so stations past a terminal hold the end value and the
/// curve stays defined (and flat) there.
fn station(
    profile: &crate::solve::Profile,
    a: f64,
    sampler: &mut GroundSampler,
    z: u8,
    z_ref: u8,
) -> f64 {
    let pt = profile.point_at_arc(a);
    let zb = solve::tile_containing(z, pt.x, pt.y);
    let rb = solve::tile_containing(z_ref, pt.x, pt.y);
    sampler.surface(&zb, pt.x, pt.y, z) - sampler.surface(&rb, pt.x, pt.y, z_ref)
}
