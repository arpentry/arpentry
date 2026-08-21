//! Portal placement — where a tunnel actually emerges from the ground
//! (docs/GENERATION.md S5).
//!
//! An annotation edge is where a mapper split the segment, not where the road
//! pierces the hillside; the geometric facts are two signed gaps. The run's
//! *interior* is where the road line runs below the terrain; its *ends* are
//! where the drawn tube — a constant-section solid a [`TUNNEL_HEIGHT_M`]
//! tall — last fits under it ([`roof_gap`]). Judged by the line alone, an
//! end tail grazing half a metre under the DEM kept its tunnel span and drew
//! a roof standing four metres proud of the hillside (the Territet
//! funicular's gallery, `clearance.bore_cover` at 66 % of a tile's samples);
//! judged by the roof alone, an interior thin-cover dip — a gully crossing
//! the alignment — split one tunnel into two and benched an open trench
//! between them (see [`span_bounds`]). Each tunnel span's buried run is
//! searched outward past the annotation edges up to [`PORTAL_MAX_M`], the
//! same trust model the bore mesh uses. The solved portals feed the ground
//! stage (the carve that daylights the mouth); the mesh caps its tube on the
//! same roof crossings from the same profile, so the two agree by
//! construction.

use crate::priors::{DECK_THICKNESS_M, PORTAL_MAX_M, TUNNEL_HEIGHT_M};
use crate::scene::{Span, SpanKind};

use super::profile::{Profile, ABSORB_STANDOFF_M};

/// One solved portal: its arc position, which way "out of the hill" faces
/// along the corridor (`-1.0` toward decreasing arc, `+1.0` increasing), and
/// the bore floor height at the mouth.
#[derive(Debug, Clone, Copy)]
pub struct Portal {
    pub arc: f64,
    pub outward: f64,
    pub floor_m: f64,
}

/// The signed daylight of the drawn tube at one node: how far the bore's
/// roof stands above the natural ground. Negative is buried — the whole
/// constant-section tube fits under the hill — and the zero crossing is the
/// portal. This is *the* burial criterion: the solve-side reconciliation
/// ([`span_bounds`], [`grow_spans`]) and the mesh sweep
/// (`synth::structure::bore_section`) all read it, so where a bore is kept,
/// where its mouth lands and where the tube is drawn cannot disagree. The
/// [`crate::priors::TUNNEL_COVER_M`] margin is deliberately *not* demanded
/// here: the covered-bore ceiling (`relax::seed_bore_ceilings`) buries a
/// licensed crossing to exactly roof + cover, and a criterion that demanded
/// the cover back would sit bit-for-bit on that boundary and drop the one
/// stretch the license just paid for. Cover is a quality margin, measured by
/// `clearance.bore_cover`; the roof is the draw/no-draw fact.
pub fn roof_gap(road_m: f64, terrain_m: f64) -> f64 {
    road_m + TUNNEL_HEIGHT_M - terrain_m
}

/// The first node (in arc order) of the *dominant* buried run overlapping the
/// annotated span: among the maximal contiguous stretches where `road` runs
/// below `terrain` and that touch `[arc0, arc1]`, the one with the greatest
/// integrated burial (Σ −gap over the whole run). Returns that run's first
/// node that falls inside the annotation, so [`span_bounds`]' outward
/// expansion walks the winning run in both directions.
///
/// This is the guard against a shallow terrain graze (real relief noise, or a
/// brief emergence on the approach) sitting between the portal and the true
/// bore: scored by depth×length it cannot outweigh the run it belongs to, so
/// the tunnel is solved under the hill instead of collapsing onto the graze.
fn dominant_buried_seed(arc: &[f64], road: &[f64], terrain: &[f64], span: &Span) -> Option<usize> {
    let n = arc.len();
    let gap = |i: usize| road[i] - terrain[i];
    let in_span = |i: usize| arc[i] >= span.arc0 && arc[i] <= span.arc1;
    let mut best: Option<usize> = None;
    let mut best_score = 0.0_f64;
    let mut i = 0;
    while i < n {
        if gap(i) >= 0.0 {
            i += 1;
            continue;
        }
        // A maximal buried run [start, i); score its full extent but seed on
        // its first in-annotation node so the caller's expansion stays anchored
        // inside the mapper's span.
        let mut score = 0.0;
        let mut seed = None;
        while i < n && gap(i) < 0.0 {
            score += -gap(i);
            if seed.is_none() && in_span(i) {
                seed = Some(i);
            }
            i += 1;
        }
        if let Some(seed) = seed {
            if best.is_none() || score > best_score {
                best = Some(seed);
                best_score = score;
            }
        }
    }
    best
}

/// Whether the tube fits under the ground over the **majority** of the
/// line-buried run containing `at_arc` — the per-run answer to "is this a
/// bore with shallow mouths, or a surface gallery with one licensed dip?".
///
/// A real tunnel's line-buried run holds the tube almost everywhere and
/// grazes only at its mouths: the A9 Glion bore fits over 99 % of its run,
/// the Caux and Veytaux galleries over 90 %. The Territet funicular's
/// "tunnel" is the opposite — 45 m of line grazing a metre under its slope,
/// fitting the tube only in the 10 m the covered-crossing ceiling bought —
/// and drawing it as a tube snaked a roof metres proud of the hillside on
/// both sides of the road it crossed under. The majority is the same logic
/// [`dominant_buried_seed`] already applies between runs, applied within
/// one: whichever reading of the annotation covers most of the geometry is
/// the annotation's meaning.
///
/// Judged blanket-by-the-roof instead, every real portal moved into the
/// hill: the freed line-buried mouths were benched as open rail under the
/// Veytaux shore road (`order.grade_stack` 12.9 → 14.2) and the deepened
/// portal cuts tore the A9 and Caux flanks (`slope.terrain_tearing`
/// 6.5 → 11.1). Shared with the bore mesh (`synth::structure`), which picks
/// its cap criterion per piece from the same walk, so the span truth and the
/// drawn tube cannot disagree.
///
/// The walk is deliberately *unclamped* (no [`PORTAL_MAX_M`] reach) so both
/// callers see the same run whatever window they hold. `false` when no
/// line-buried node exists at or beside `at_arc`.
pub fn tube_fit_majority(profile: &Profile, at_arc: f64) -> bool {
    let Some((f, l)) = line_buried_run(profile, at_arc) else {
        return false;
    };
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let fit = (f..=l).filter(|&i| roof_gap(road[i], terrain[i]) < 0.0).count();
    2 * fit >= l - f + 1
}

/// The **line-buried run** containing `at_arc`: the maximal stretch of nodes
/// where the road runs below the reference surface, as inclusive node indices.
///
/// The run, not the annotation: a mapper's span edge is where a way was split,
/// and every consumer that needs "this tunnel, whole" needs the geometry's own
/// answer instead. Unclamped by [`PORTAL_MAX_M`] on purpose, so a caller
/// holding a tile-clipped piece of the tunnel still sees the same run as one
/// holding the corridor — which is what lets a per-tile drawing decision come
/// out the same in every tile (invariant 5).
pub fn line_buried_run(profile: &Profile, at_arc: f64) -> Option<(usize, usize)> {
    let arc = profile.arc();
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let n = arc.len();
    if n == 0 {
        return None;
    }
    let line = |i: usize| road[i] - terrain[i];
    let mut s = arc.partition_point(|&a| a < at_arc).min(n - 1);
    if line(s) >= 0.0 {
        if s > 0 && line(s - 1) < 0.0 {
            s -= 1;
        } else if s + 1 < n && line(s + 1) < 0.0 {
            s += 1;
        } else {
            return None;
        }
    }
    let mut f = s;
    while f > 0 && line(f - 1) < 0.0 {
        f -= 1;
    }
    let mut l = s;
    while l + 1 < n && line(l + 1) < 0.0 {
        l += 1;
    }
    Some((f, l))
}

/// The buried run of one tunnel span: its **interior by the line, its ends
/// by whichever criterion the run's own geometry elects**
/// ([`tube_fit_majority`]).
///
/// The run's extent is where the road runs below the terrain — searched
/// outward past the annotation edges (mapper cuts, not geometry) up to
/// [`PORTAL_MAX_M`]. When the tube fits under the majority of that run, the
/// bounds are the line's zero crossings, exactly as a portal has always been
/// placed: the shallow mouths are the transition band of a real bore, and
/// pulling them back only deepened the daylighting cut and benched the freed
/// metres as open track under whatever runs above. When the tube fits only a
/// minority, the annotation names a surface gallery: each end is pulled back
/// to the last node where the tube fits, the bound interpolated onto the
/// roof's zero crossing, and the freed tails degrade to the open cutting
/// they are.
///
/// An *interior dip* — a gully crossing the alignment, a shore gallery whose
/// cover thins mid-run — never splits the run under either criterion: the
/// line stays under the ground through it, so it is a covered stretch of one
/// tunnel, not two tunnels with a benched trench between them.
///
/// `None` when the span has no buried node, or when nothing in a
/// minority-fit run holds the tube — a tunnel tagged over flat or shallow
/// ground end to end has no bore, and the open cutting is what is there. A
/// side whose run never surfaces within reach reports `None` for that
/// crossing (the bore runs out of data, not out of the hill).
pub fn span_bounds(profile: &Profile, span: &Span) -> Option<(Option<f64>, Option<f64>)> {
    let arc = profile.arc();
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let n = arc.len();
    let line = |i: usize| road[i] - terrain[i];
    let roof = |i: usize| roof_gap(road[i], terrain[i]);

    // Seed on the *dominant* buried run overlapping the annotation — not the
    // first buried node. Seeding on the first node let a shallow DEM-noise
    // graze on the approach capture the whole solve: the graze became the
    // "tunnel" and the real, deep run past it was re-covered as at-grade road
    // painted over the massif (docs/GENERATION.md S5, S10). The deepest run
    // outscores a brief graze by orders of magnitude, so the bore lands under
    // the hill it belongs to.
    let lo_arc = span.arc0 - PORTAL_MAX_M;
    let hi_arc = span.arc1 + PORTAL_MAX_M;
    let seed = dominant_buried_seed(arc, road, terrain, span)?;
    let mut f = seed;
    while f > 0 && line(f - 1) < 0.0 && arc[f - 1] >= lo_arc {
        f -= 1;
    }
    let mut l = seed;
    while l + 1 < n && line(l + 1) < 0.0 && arc[l + 1] <= hi_arc {
        l += 1;
    }
    if tube_fit_majority(profile, arc[seed]) {
        // A bore with shallow mouths: portals on the line's own crossings.
        let low = (f > 0 && line(f - 1) >= 0.0).then(|| {
            let t = line(f - 1) / (line(f - 1) - line(f));
            arc[f - 1] + t * (arc[f] - arc[f - 1])
        });
        let high = (l + 1 < n && line(l + 1) >= 0.0).then(|| {
            let t = line(l) / (line(l) - line(l + 1));
            arc[l] + t * (arc[l + 1] - arc[l])
        });
        return Some((low, high));
    }
    // A surface gallery: pull each end back to the tube's fit. A run the tube
    // fits nowhere in is no bore at all.
    while f <= l && roof(f) >= 0.0 {
        f += 1;
    }
    while l > f && roof(l) >= 0.0 {
        l -= 1;
    }
    if f > l || roof(f) >= 0.0 {
        return None;
    }
    // Interpolate each bounding crossing onto the roof's zero crossing, when
    // the neighbour outside the run has emerged.
    let low = (f > 0 && roof(f - 1) >= 0.0).then(|| {
        let t = roof(f - 1) / (roof(f - 1) - roof(f));
        arc[f - 1] + t * (arc[f] - arc[f - 1])
    });
    let high = (l + 1 < n && roof(l + 1) >= 0.0).then(|| {
        let t = roof(l) / (roof(l) - roof(l + 1));
        arc[l] + t * (arc[l + 1] - arc[l])
    });
    Some((low, high))
}

/// The portals of every tunnel span of a corridor: the gap zero-crossings
/// bounding each span's buried run ([`span_bounds`]).
pub fn portals(profile: &Profile, spans: &[Span]) -> Vec<Portal> {
    let mut out = Vec::new();
    for span in spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
        let Some((low, high)) = span_bounds(profile, span) else {
            continue;
        };
        if let Some(a) = low {
            out.push(Portal { arc: a, outward: -1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
        }
        if let Some(a) = high {
            out.push(Portal { arc: a, outward: 1.0, floor_m: profile.road_at_arc(a) - DECK_THICKNESS_M });
        }
    }
    out
}

/// Structure spans grown over the profile's absorbed stretches: where the
/// solve flipped at-grade nodes into a structure run (an infeasible anchor —
/// the annotation ended before the road reached the ground, see
/// `profile::solve`), the adjacent structure span is extended to cover them
/// and the grade span shrunk to match, so the deck/bore sweep and the paint
/// follow the solved geometry instead of the annotation. The span list stays
/// a partition of the corridor: each boundary moves, none overlap.
pub fn grow_spans(profile: &Profile, spans: &[Span]) -> Vec<Span> {
    let arc = profile.arc();
    let at_grade = profile.at_grade();
    if spans.len() < 2 || at_grade.is_empty() {
        return spans.to_vec();
    }
    let (road, terrain) = (profile.road_m(), profile.terrain_m());
    // A deck grows only over nodes it actually clears, a bore only over nodes
    // it actually runs beneath — the line criterion, because growth covers
    // *interior* absorbed material and the ends are pulled back to the tube's
    // fit by [`span_bounds`] afterwards. Absorption marks a node as structure
    // from the *profile's* side; it does not promise the ground stayed out of
    // the way, and a deck swept past the point where the hillside comes back
    // up is a deck buried in it. The Territet funicular's bridge grew 20 m
    // past its annotated end into the embankment of the road above and sat
    // 9.8 m under the drawn ground.
    let holds = |kind: SpanKind, k: usize| match kind {
        SpanKind::Bridge => road[k] >= terrain[k],
        SpanKind::Tunnel => road[k] <= terrain[k],
        SpanKind::Grade => false,
    };
    let mut out = spans.to_vec();
    for i in 0..out.len() {
        if out[i].kind == SpanKind::Grade {
            continue;
        }
        // Backward over the preceding grade span's absorbed tail.
        if i > 0 && out[i - 1].kind == SpanKind::Grade {
            let mut a0 = out[i].arc0;
            for k in (0..arc.len()).rev() {
                if arc[k] >= out[i].arc0 {
                    continue;
                }
                if arc[k] <= out[i - 1].arc0 || at_grade[k] || !holds(out[i].kind, k) {
                    break;
                }
                a0 = arc[k];
            }
            if a0 < out[i].arc0 {
                out[i].arc0 = a0;
                out[i - 1].arc1 = a0;
            }
        }
        // Forward over the following grade span's absorbed head.
        if i + 1 < out.len() && out[i + 1].kind == SpanKind::Grade {
            let mut a1 = out[i].arc1;
            for k in 0..arc.len() {
                if arc[k] <= out[i].arc1 {
                    continue;
                }
                if arc[k] >= out[i + 1].arc1 || at_grade[k] || !holds(out[i].kind, k) {
                    break;
                }
                a1 = arc[k];
            }
            if a1 > out[i].arc1 {
                out[i].arc1 = a1;
                out[i + 1].arc0 = a1;
            }
        }
    }
    // A grade span fully absorbed from one side collapses to nothing: drop it.
    out.retain(|s| s.arc1 - s.arc0 > f64::EPSILON);
    out
}

/// The post-relax twin of `profile::solve`'s infeasible-anchor absorption
/// (S10), for the modes that never absorb and the hangs only the fused
/// relaxation creates. A crossing clearance lifts a deck; the approaches are
/// re-pinned to the lifted ends and, held to their class grade against a
/// reference that falls away faster, arrive nowhere: at-grade nodes standing
/// many metres over their own terrain. At grade they pave and bench —
/// measured at the Chauderon slot (S20), the two approaches of a correctly
/// mapped 20 m bridge hung +8..+17 m for ~100 m, and their benches dammed
/// the gorge with 15.8 m kerb walls (`contact.kerb_lip` 25.7 % over in that
/// tile, `order.deck_above_carriageway` 25.6 % — asphalt drawn over the
/// gorge-floor footbridge).
///
/// Beside each bridge span, every contiguous at-grade run still standing
/// more than [`ABSORB_STANDOFF_M`] off the reference after the relaxation
/// belongs to the structure: flagged into the profile here, extended into
/// the span partition by [`grow_spans`] on the next step of the write-back.
/// The ordinary approach embankment survives untouched — p99 approach
/// standoff across the network is ~2.5 m, and a ramp descending to flat
/// ground drops under the threshold within a few nodes of the deck. Reads
/// only this stratum's own solved heights, so it is the stratum deciding
/// its own structures (§4.5), not a cross-stratum coupling.
pub fn absorb_hanging_approaches(
    profile: &mut Profile,
    spans: &[Span],
    deck_follows_road: bool,
) {
    let mut runs: Vec<(f64, f64)> = Vec::new();
    {
        let arc = profile.arc();
        let at_grade = profile.at_grade();
        let (road, terrain) = (profile.road_m(), profile.terrain_m());
        let hanging = |k: usize| at_grade[k] && road[k] - terrain[k] > ABSORB_STANDOFF_M;
        // A node landing on the span edge to within float noise is the
        // span's own boundary node, not the approach's first.
        const EDGE_EPS_M: f64 = 1e-6;
        for s in spans.iter().filter(|s| s.kind == SpanKind::Bridge) {
            // Backward over the approach before the span.
            let mut lo = None;
            for k in (0..arc.len()).rev() {
                if arc[k] >= s.arc0 - EDGE_EPS_M {
                    continue;
                }
                if !hanging(k) {
                    break;
                }
                lo = Some(arc[k]);
            }
            if let Some(lo) = lo {
                runs.push((lo, s.arc0));
            }
            // Forward over the approach after it.
            let mut hi = None;
            for k in 0..arc.len() {
                if arc[k] <= s.arc1 + EDGE_EPS_M {
                    continue;
                }
                if !hanging(k) {
                    break;
                }
                hi = Some(arc[k]);
            }
            if let Some(hi) = hi {
                runs.push((s.arc1, hi));
            }
        }
    }
    for (a0, a1) in runs {
        profile.annex_structure(a0, a1, deck_follows_road);
    }
}

/// The same absorption asked across a junction: a road welded into another
/// corridor **inside that corridor's deck** is standing on the deck, and what
/// carries it from there is a structure of its own.
///
/// [`absorb_hanging_approaches`] walks along one corridor from its own span,
/// and a corridor with no span of its own is invisible to it. That is the
/// whole Chauderon residue: after the slot crossing was promoted and its
/// approaches absorbed (S20), two service stubs — 22 m and 18 m — remain
/// welded to Route de Chernex at junctions *inside* its bridge span, held by
/// the weld at 527.7 m over a gorge floor their own reference reads at 512.5.
/// At grade they pave, and their bands are drawn over the footbridge crossing
/// the slot beneath: `order.deck_above_carriageway` −15.36 m, the worst
/// inversion in the extract, with `contact.kerb_lip`'s worst on the same kerb.
///
/// Censused over the Montreux zone, every at-grade run standing past
/// [`ABSORB_STANDOFF_M`] falls into four kinds, and this is the only one with
/// a structural explanation at its end: 3 runs, 40 m. The 275 m of *junction
/// cluster* behind them — stubs welded to those stubs, and a 91 m residential
/// ramp descending to its ground — is deliberately **not** taken. Propagating
/// the absorption through the cluster would draw a soffit under an embankment,
/// and the instrument that gates a walled terrace, `contact.kerb_unwalled`,
/// reads zero over the whole cluster: the walls the model implies are all
/// drawn, which is what S13 says a road cut into a flank has.
///
/// Whether this profile has **no ground of its own**: every node the partition
/// still calls at grade stands more than [`ABSORB_STANDOFF_M`] off its own
/// reference. `false` for a profile with no at-grade node at all — that one is
/// already a structure end to end.
///
/// This is the discriminator between a stub held in the air and an approach
/// embankment, and it is what stops the absorption from walking a whole
/// terrace. Measured at Chauderon: the two service stubs welded onto Route de
/// Chernex's deck hang over all of their 22 m and 18 m, while Chemin des
/// Vuarennes hangs over 91 m of its 156 m and comes down to +1.7 m at its far
/// end. The first pair is a span between two points in the air; the second is
/// a road on a long fill, and a fill drawn with a soffit under it is worse
/// than a fill drawn with the wall the ground stage already gives it
/// (`contact.kerb_unwalled` reads zero across the whole cluster).
pub fn hangs_end_to_end(profile: &Profile) -> bool {
    let at_grade = profile.at_grade();
    let (road, terrain) = (profile.road_m(), profile.terrain_m());
    let mut any = false;
    for k in 0..at_grade.len() {
        if !at_grade[k] {
            continue;
        }
        any = true;
        if road[k] - terrain[k] <= ABSORB_STANDOFF_M {
            return false;
        }
    }
    any
}

/// Every grade span of a corridor turned into a level-1 deck, with the profile
/// flagged to match — for a stub [`hangs_end_to_end`] says has no ground of
/// its own and whose weld lands inside someone else's deck.
///
/// [`absorb_hanging_approaches`] walks along one corridor from its own span,
/// and a corridor with no span of its own is invisible to it. That is the
/// whole Chauderon residue: after the slot crossing was promoted and its
/// approaches absorbed (S20), two service stubs remain welded to Route de
/// Chernex at junctions *inside* its bridge, held by the weld at 526.7 and
/// 527.7 m over a gorge floor their own reference reads at 512. At grade they
/// pave, and their bands are drawn over the footbridge crossing the slot
/// beneath — `order.deck_above_carriageway` −15.36 m, the worst inversion in
/// the extract, with `contact.kerb_lip`'s worst on the same kerb. A road
/// standing on a deck at one end and in the air everywhere else is a
/// structure, which is what §4.5 says a surface departed from the ground is.
pub fn carry_whole_corridor(
    profile: &mut Profile,
    spans: &[Span],
    deck_follows_road: bool,
) -> Vec<Span> {
    let total = profile.arc().last().copied().unwrap_or(0.0);
    profile.annex_structure(0.0, total, deck_follows_road);
    spans
        .iter()
        .map(|s| match s.kind {
            SpanKind::Grade => Span { level: 1, kind: SpanKind::Bridge, ..*s },
            _ => *s,
        })
        .collect()
}

/// Extends span `i` forward to `target`, eating the grade spans in the way.
/// A mapped bridge or bore is a boundary the annex never moves. Returns
/// whether the span's end actually shifted.
fn grow_high(out: &mut [Span], i: usize, target: f64) -> bool {
    if target <= out[i].arc1 {
        return false;
    }
    let mut a1 = out[i].arc1;
    for s in out.iter_mut().skip(i + 1) {
        if s.arc1 - s.arc0 <= f64::EPSILON {
            continue; // already eaten by an earlier annex
        }
        if s.kind != SpanKind::Grade || a1 >= target {
            break;
        }
        if s.arc1 - target < MIN_ANNEX_STUB_M {
            a1 = s.arc1; // the whole span, rather than leaving a sliver
            s.arc0 = s.arc1;
        } else {
            a1 = target;
            s.arc0 = target;
        }
    }
    let moved = a1 > out[i].arc1;
    out[i].arc1 = a1;
    moved
}

/// [`grow_high`] mirrored onto the low side.
fn grow_low(out: &mut [Span], i: usize, target: f64) -> bool {
    if target >= out[i].arc0 {
        return false;
    }
    let mut a0 = out[i].arc0;
    for s in out[..i].iter_mut().rev() {
        if s.arc1 - s.arc0 <= f64::EPSILON {
            continue;
        }
        if s.kind != SpanKind::Grade || a0 <= target {
            break;
        }
        if target - s.arc0 < MIN_ANNEX_STUB_M {
            a0 = s.arc0;
            s.arc1 = s.arc0;
        } else {
            a0 = target;
            s.arc1 = target;
        }
    }
    let moved = a0 < out[i].arc0;
    out[i].arc0 = a0;
    moved
}

/// Structure spans extended through the crossings they pass beneath or carry
/// over — the *growing* half of span reconciliation. Both halves run in the
/// per-stratum write-back (`solve::reconcile_stratum`): the annex first, then
/// the shrinking half ([`reconcile_spans`]), and the result is written into
/// the scene as the one span truth every consumer cuts (§4.5).
///
/// An annotation edge is where a mapper split the segment (S5), not where the
/// structure stops doing its job, and both kinds of span are mapped shorter
/// than the crossing they serve.
///
/// **A bore whose tail is still below its own terrain when another mapped
/// alignment crosses it has not emerged**: the ground it must pierce includes
/// that feature's band and bench. Left as annotated, the buried tail is paved
/// as open formation — benched, holed, sheeted — sliding beneath the crossing
/// feature's band metres up, which is exactly the two-superposed-lines drawing
/// the Collonge funicular made over the rack railway's short portal
/// (`order.grade_stack` keeps the class dead). The gate is deliberately
/// double: the tail must be buried (the tube still fits under the ground —
/// [`roof_gap`], searched by [`span_bounds`] out to [`PORTAL_MAX_M`]) *and*
/// crossed (`crossings::plan_crossings`). Burial alone would swallow the open
/// trench approach of a flat-ground underpass (S6); a crossing alone is any
/// street the line meets at grade.
///
/// **A deck is mapped to its own length, and the thing under it is as wide as
/// it is** — divided by the sine of the crossing angle, which no mapper
/// adjusts for. Where the lower feature's band straddles the deck's end, the
/// deck's own *formation band* is drawn at grade over the lower band with
/// nothing between them: three 10 m rail decks over one road cut at Burier
/// (S17) leave 3.5 m of formation each hanging 6.6 m over the road. Censused
/// over the Montreux zone, 23 decks are shorter than the crossing they carry,
/// by 111 m in total. The gate here is the level ordinals — `carried` holds
/// only the crossings a mapped alignment annotated *below* this span makes,
/// the mirror of the burial license's "from above" — and the reach is one
/// band, so a deck grows at most a `clear_m` at each end. No height is read
/// on either side (§4.1); the DEM has no cut for the road anyway, because the
/// road digs it.
///
/// Each qualifying side extends to the last such crossing plus its `clear_m`,
/// eating only neighbouring *grade* spans. Returns `None` when nothing
/// changed.
pub fn annex_spans(
    profile: &Profile,
    spans: &[Span],
    crossings: &[(f64, f64)],
    carried: &[(f64, f64)],
) -> Option<Vec<Span>> {
    if crossings.is_empty() && carried.is_empty() {
        return None;
    }
    let total = *profile.arc().last()?;
    let mut out = spans.to_vec();
    let mut changed = false;
    for i in 0..out.len() {
        let (lo, hi) = match out[i].kind {
            SpanKind::Grade => continue,
            SpanKind::Bridge => {
                // The lower feature's band must actually straddle the end:
                // a crossing wholly inside the deck is already carried, and
                // one wholly outside is another span's business.
                let (a0, a1) = (out[i].arc0, out[i].arc1);
                let lo = carried
                    .iter()
                    .filter(|&&(x, clear)| x - clear < a0 && x + clear > a0)
                    .fold(a0, |t, &(x, clear)| t.min((x - clear).max(0.0)));
                let hi = carried
                    .iter()
                    .filter(|&&(x, clear)| x - clear < a1 && x + clear > a1)
                    .fold(a1, |t, &(x, clear)| t.max((x + clear).min(total)));
                (lo, hi)
            }
            SpanKind::Tunnel => {
                let Some((low, high)) = span_bounds(profile, &out[i]) else {
                    continue;
                };
                // The buried run past the annotation edge, bounded by the true
                // emergence when one exists and by the search reach when the
                // run never surfaces (out of data, not out of the hill). A
                // crossing qualifies when its *band* pokes past the span end —
                // its centre may sit metres inside the annotation and the band
                // still straddle the edge, which was the Collonge measurement:
                // crossings 1.4 m inside a snapped span end, band reaching
                // 4.4 m beyond it.
                let tail_hi = high.unwrap_or(out[i].arc1 + PORTAL_MAX_M).min(total);
                let tail_lo = low.unwrap_or(out[i].arc0 - PORTAL_MAX_M).max(0.0);
                let (a0, a1) = (out[i].arc0, out[i].arc1);
                let hi = crossings
                    .iter()
                    .filter(|&&(x, clear)| x > a0 && x <= tail_hi && x + clear > a1)
                    .fold(a1, |t, &(x, clear)| t.max((x + clear).min(total)));
                let lo = crossings
                    .iter()
                    .filter(|&&(x, clear)| x < a1 && x >= tail_lo && x - clear < a0)
                    .fold(a0, |t, &(x, clear)| t.min((x - clear).max(0.0)));
                (lo, hi)
            }
        };
        changed |= grow_high(&mut out, i, hi);
        changed |= grow_low(&mut out, i, lo);
    }
    if !changed {
        return None;
    }
    out.retain(|s| s.arc1 - s.arc0 > f64::EPSILON);
    Some(out)
}

/// A grade span left shorter than this by an annex is absorbed whole: a
/// half-metre of "open" formation wedged between a bore and the band it dives
/// under is the sliver the annex exists to remove.
const MIN_ANNEX_STUB_M: f64 = 2.0;

/// Corridor spans reconciled with the solved geometry: structure spans grown
/// over the profile's absorbed stretches ([`grow_spans`]), then each tunnel
/// span clamped to its buried run (the solved portal crossings), and the freed
/// annotation slack — the stretch a mapper tagged "tunnel" where the road in
/// fact still runs above ground — is re-covered by grade spans, so the
/// approach up to a portal mouth is painted road instead of naked ground. A
/// tunnel with no buried run at all becomes grade end to end. Called once per
/// corridor from the per-stratum write-back (`solve::reconcile_stratum`) and
/// written into the scene, so paint, bands, benches and solids all cut one
/// partition (§4.5). Only shrinking is reconciled: a buried run reaching
/// *past* the annotation is left to the bore sweep's own outward march, where
/// the neighbouring span's paint simply passes under the ground it is buried
/// by.
///
/// **The shrink stops at the burial license.** `covered` carries this
/// corridor's covered-crossing windows — the §4.5 license
/// (`crossings::covered_bores`) that seeded the bore's ceilings and that
/// [`annex_spans`] grew the span through — and no arc inside one is ever
/// re-covered as grade. The buried run is measured against the *reference*
/// surface, and inside a licensed window that surface is the open cutting the
/// crossing feature bridges or embanks over: the ground the tube must fit
/// under there is that feature's roadbed, which the DEM does not carry. Judged
/// by the reference alone, the shrink undid the annex the same sweep it ran —
/// a service road's 12 m underpass at Clarens (6.9165,46.4305) came back out
/// as 2 m of tube between two open trenches, and the Veytaux shore line's
/// portal tail was paved as open formation 13.5 m under the primary road
/// crossing above it. That is the superposition `order.grade_stack` counts,
/// and it was 61 % of the metric's violations over the Montreux extract.
/// The license is annotation-only on both sides, so this reads no junior's
/// solved height (§4.1); it decides only whether a stretch is *drawn* open,
/// never how deep the profile runs.
pub fn reconcile_spans(profile: &Profile, spans: &[Span], covered: &[(f64, f64)]) -> Vec<Span> {
    /// Shortest grade stub worth emitting, in metres — below this the piece
    /// quantizes away.
    const MIN_STUB_M: f64 = 0.25;
    let spans = grow_spans(profile, spans);
    let mut out = Vec::with_capacity(spans.len() + 4);
    for s in &spans {
        if s.kind != SpanKind::Tunnel {
            out.push(*s);
            continue;
        }
        // The licensed part of this span, if any: one interval, so a window
        // straddling a thin-cover dip cannot split one tunnel into two (the
        // reason [`span_bounds`] spans its dips as well).
        let licensed = covered
            .iter()
            .map(|&(w0, w1)| (w0.max(s.arc0), w1.min(s.arc1)))
            .filter(|(l0, l1)| l1 > l0)
            .fold(None, |acc: Option<(f64, f64)>, (l0, l1)| {
                Some(acc.map_or((l0, l1), |(a, b)| (a.min(l0), b.max(l1))))
            });
        let fitted = span_bounds(profile, s)
            .map(|(low, high)| {
                (low.map_or(s.arc0, |a| a.max(s.arc0)), high.map_or(s.arc1, |a| a.min(s.arc1)))
            })
            .filter(|(a0, a1)| a1 - a0 >= MIN_STUB_M);
        let kept = match (fitted, licensed) {
            (Some((a0, a1)), Some((l0, l1))) => Some((a0.min(l0), a1.max(l1))),
            (Some(f), None) => Some(f),
            (None, Some(l)) => Some(l),
            (None, None) => None,
        };
        let Some((a0, a1)) = kept.filter(|(a0, a1)| a1 - a0 >= MIN_STUB_M) else {
            out.push(Span { level: 0, kind: SpanKind::Grade, ..*s });
            continue;
        };
        if a0 - s.arc0 > MIN_STUB_M {
            out.push(Span { arc0: s.arc0, arc1: a0, level: 0, kind: SpanKind::Grade });
        }
        out.push(Span { arc0: a0, arc1: a1, ..*s });
        if s.arc1 - a1 > MIN_STUB_M {
            out.push(Span { arc0: a1, arc1: s.arc1, level: 0, kind: SpanKind::Grade });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::DEG_M;
    use geo_types::Coord;

    fn span(arc0: f64, arc1: f64) -> Span {
        Span { arc0, arc1, level: -1, kind: SpanKind::Tunnel }
    }

    /// A 1 km corridor with a hill in the middle: road flat at 100, terrain
    /// rising to 130 over the central third — buried between the flanks.
    fn hill() -> (Profile, f64) {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                let d = (u - 0.5).abs();
                if d < 0.15 { 130.0 - d / 0.15 * 40.0 } else { 90.0 } // crosses 100 at d=0.1125
            })
            .collect();
        (Profile::from_heights(&nodes, road, terrain), len)
    }

    #[test]
    fn portals_sit_on_the_gap_zero_crossings() {
        let (p, len) = hill();
        // Annotation roughly over the buried middle (mapper slop included).
        let ps = portals(&p, &[span(0.42 * len, 0.58 * len)]);
        assert_eq!(ps.len(), 2, "a through-tunnel has two portals");
        // A deep hill fits the tube over most of its run (majority-fit), so
        // this is a real bore and its mouths sit on the *line's* crossings at
        // u = 0.5 ± 0.1125 — the shallow approach is the portal transition,
        // not open cutting.
        assert!((ps[0].arc - 387.5).abs() < 10.0, "west portal at {}", ps[0].arc);
        assert!((ps[1].arc - 612.5).abs() < 10.0, "east portal at {}", ps[1].arc);
        assert_eq!(ps[0].outward, -1.0);
        assert_eq!(ps[1].outward, 1.0);
        assert!((ps[0].floor_m - 98.5).abs() < 0.1, "floor = road − slab");
    }

    #[test]
    fn an_interior_thin_cover_dip_does_not_split_the_tunnel() {
        // Two deep hills with a saddle between them where the cover thins to
        // 2 m: the line stays buried through the saddle (gap −2) but the
        // 5 m tube does not fit under it. That saddle is a covered stretch of
        // one tunnel — a gully crossing the alignment, the Veytaux shore
        // gallery — not two tunnels: judged by the roof alone, the saddle
        // degraded to a few metres of open rail whose bench dug a 25 m slot
        // through the gully wall (the MGN at 6.9211,46.4336). The bounds must
        // span hill to hill, ends on the outer roof crossings.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let ramp = |u: f64, u0: f64, u1: f64, h0: f64, h1: f64| h0 + (h1 - h0) * (u - u0) / (u1 - u0);
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                match u {
                    _ if u < 0.30 => 90.0,
                    _ if u < 0.34 => ramp(u, 0.30, 0.34, 90.0, 130.0),
                    _ if u < 0.42 => 130.0,
                    _ if u < 0.46 => ramp(u, 0.42, 0.46, 130.0, 102.0),
                    _ if u < 0.54 => 102.0,
                    _ if u < 0.58 => ramp(u, 0.54, 0.58, 102.0, 130.0),
                    _ if u < 0.66 => 130.0,
                    _ if u < 0.70 => ramp(u, 0.66, 0.70, 130.0, 90.0),
                    _ => 90.0,
                }
            })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let (low, high) = span_bounds(&p, &span(0.28 * len, 0.72 * len)).expect("buried");
        // Majority-fit (the two hills dominate the run), so the ends are the
        // line's crossings on the *outer* flanks: u = 0.31 and 0.69. The
        // saddle never splits the run.
        let low = low.expect("west portal");
        let high = high.expect("east portal");
        assert!((low - 310.0).abs() < 10.0, "west end on the outer flank, at {low}");
        assert!((high - 690.0).abs() < 10.0, "east end on the outer flank, at {high}");
    }

    #[test]
    fn a_minority_fit_gallery_pulls_its_ends_to_the_tubes_fit() {
        // The Territet funicular's shape: a "tunnel" whose line grazes 2 m
        // under its slope for most of the run and holds real depth only in a
        // short licensed window. Drawn to the line's crossings, the tube
        // snaked its roof metres proud of the hillside on both sides of the
        // road above the window; a minority-fit run is a surface gallery, so
        // its ends pull back to the roof's crossings and the shallow tails
        // degrade to the open cutting they are.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let ramp = |u: f64, u0: f64, u1: f64, h0: f64, h1: f64| h0 + (h1 - h0) * (u - u0) / (u1 - u0);
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                match u {
                    _ if u < 0.30 => 90.0,
                    _ if u < 0.32 => ramp(u, 0.30, 0.32, 90.0, 102.0),
                    _ if u < 0.45 => 102.0,
                    _ if u < 0.47 => ramp(u, 0.45, 0.47, 102.0, 112.0),
                    _ if u < 0.53 => 112.0,
                    _ if u < 0.55 => ramp(u, 0.53, 0.55, 112.0, 102.0),
                    _ if u < 0.68 => 102.0,
                    _ if u < 0.70 => ramp(u, 0.68, 0.70, 102.0, 90.0),
                    _ => 90.0,
                }
            })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let (low, high) = span_bounds(&p, &span(0.28 * len, 0.72 * len)).expect("buried");
        // Roof (105) crossings on the notch flanks: u = 0.456 and 0.544.
        let low = low.expect("west portal");
        let high = high.expect("east portal");
        assert!((low - 456.0).abs() < 10.0, "west end at the tube's fit, at {low}");
        assert!((high - 544.0).abs() < 10.0, "east end at the tube's fit, at {high}");
    }

    #[test]
    fn a_shallow_graze_does_not_capture_the_solve_from_the_real_run() {
        // Annotation covers a shallow DEM-noise graze (road 0.5 m under terrain
        // over ~20 m) on the approach, then the real 60 m-deep bore. span_bounds
        // must lock onto the deep run — not the graze that appears first in arc
        // order — so the portals land at the hill, the bore is built, and the
        // deep stretch is not painted as at-grade road over the massif.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 1000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 201;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                if (0.30..0.32).contains(&u) {
                    100.5 // shallow graze: 0.5 m of burial
                } else if (0.40..0.60).contains(&u) {
                    160.0 // the real bore: 60 m of burial
                } else {
                    90.0
                }
            })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let (low, high) =
            span_bounds(&p, &span(0.25 * len, 0.70 * len)).expect("a buried run exists");
        let lo = low.expect("west portal on the deep run");
        let hi = high.expect("east portal on the deep run");
        assert!((380.0..405.0).contains(&lo), "west portal at the hill, got {lo}");
        assert!((595.0..620.0).contains(&hi), "east portal at the hill, got {hi}");
    }

    #[test]
    fn a_crossing_in_the_buried_tail_annexes_the_tunnel_through_it() {
        // The Collonge case: the mapped portal sits at 550 but the run is
        // buried out to ≈612, and another alignment crosses at 560. The
        // annotation edge is not an emergence; the bore extends past the
        // crossing plus its clearance, and the grade span gives up the tail
        // that would otherwise be paved as open cut beneath the crossing band.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.42 * len, level: 0, kind: SpanKind::Grade },
            span(0.42 * len, 0.55 * len),
            Span { arc0: 0.55 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = annex_spans(&p, &spans, &[(560.0, 6.0)], &[]).expect("the span must grow");
        assert_eq!(out.len(), 3, "the partition keeps its three spans: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Tunnel);
        assert!((out[1].arc1 - 566.0).abs() < 1e-9, "portal past the crossing, got {}", out[1].arc1);
        assert!((out[2].arc0 - 566.0).abs() < 1e-9, "the grade span shrinks to match");
        assert!((out[1].arc0 - 0.42 * len).abs() < 1e-9, "the uncrossed side is untouched");
    }

    #[test]
    fn a_band_straddling_the_span_end_annexes_too() {
        // The measured Collonge geometry: the crossing centre sits inside the
        // snapped annotation (545 < 550) but its band reaches 551 — the open
        // formation would still start mid-band. The reach, not the centre, is
        // what must clear the span end.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.42 * len, level: 0, kind: SpanKind::Grade },
            span(0.42 * len, 0.55 * len),
            Span { arc0: 0.55 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = annex_spans(&p, &spans, &[(545.0, 6.0)], &[]).expect("the span must grow");
        assert!((out[1].arc1 - 551.0).abs() < 1e-9, "portal past the band, got {}", out[1].arc1);
    }

    #[test]
    fn a_crossing_past_the_emergence_annexes_nothing() {
        // At 700 the road has been above ground for ~90 m: whatever crosses
        // there is met in the open, and a level crossing is information the
        // annex must not delete (§4.5).
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.42 * len, level: 0, kind: SpanKind::Grade },
            span(0.42 * len, 0.55 * len),
            Span { arc0: 0.55 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        assert!(annex_spans(&p, &spans, &[(700.0, 6.0)], &[]).is_none());
    }

    #[test]
    fn an_annex_never_eats_a_mapped_bridge() {
        // S7: bridge directly at the portal. The crossing sits under the
        // bridge's own span; the tunnel must not grow across it.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.42 * len, level: 0, kind: SpanKind::Grade },
            span(0.42 * len, 0.55 * len),
            Span { arc0: 0.55 * len, arc1: 0.60 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.60 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        assert!(annex_spans(&p, &spans, &[(570.0, 6.0)], &[]).is_none());
    }

    #[test]
    fn a_stub_with_no_ground_of_its_own_is_carried_whole() {
        // The Chauderon stubs: 100 m of road held in the air end to end. The
        // discriminator is having no at-grade node back on its own reference —
        // an approach that comes down to the ground is an embankment, and the
        // ground stage already walls those.
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 100.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..11).map(|i| Coord { x: 6.0 + deg * i as f64 / 10.0, y: 46.0 }).collect();
        let spans = vec![Span { arc0: 0.0, arc1: 100.0, level: 0, kind: SpanKind::Grade }];

        let mut hung = Profile::from_heights(&nodes, vec![520.0; 11], vec![508.0; 11]);
        assert!(hangs_end_to_end(&hung), "12 m off its reference the whole way");
        let out = carry_whole_corridor(&mut hung, &spans, false);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].kind, SpanKind::Bridge);
        assert_eq!(out[0].level, 1);
        assert!(hung.at_grade().iter().all(|&g| !g), "and nothing is at grade any more");

        // A ramp that reaches its ground keeps every metre of itself at grade.
        let terrain: Vec<f64> = (0..11).map(|i| 508.0 + 1.2 * i as f64).collect();
        let ramp = Profile::from_heights(&nodes, vec![520.0; 11], terrain);
        assert!(!hangs_end_to_end(&ramp), "its far end is on the ground");
    }

    #[test]
    fn a_deck_grows_over_the_band_it_carries() {
        // S17 at Burier: a 10 m rail deck over a road whose band reaches 7 m
        // either side of the crossing. Mapped, the deck ends 2 m short at each
        // end and the rail's own formation is drawn at grade over the road
        // band. The reach is one band and no more — a crossing wholly inside
        // the deck is already carried and moves nothing.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.50 * len, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.50 * len, arc1: 0.51 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.51 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = annex_spans(&p, &spans, &[], &[(505.0, 7.0)]).expect("the deck must grow");
        assert_eq!(out.len(), 3, "the partition keeps its three spans: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Bridge);
        assert!((out[1].arc0 - 498.0).abs() < 1e-9, "low end over the band, got {}", out[1].arc0);
        assert!((out[1].arc1 - 512.0).abs() < 1e-9, "high end over the band, got {}", out[1].arc1);
        assert!(annex_spans(&p, &out, &[], &[(505.0, 4.0)]).is_none(), "a carried band moves nothing");
    }

    #[test]
    fn a_deck_never_grows_over_a_crossing_it_does_not_reach() {
        // The mirror of the tunnel's emergence test: a crossing a band-width
        // clear of the deck is met on the embankment, and growing the deck to
        // it would fly a structure over ground that carries it.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.50 * len, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.50 * len, arc1: 0.51 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.51 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        assert!(annex_spans(&p, &spans, &[], &[(560.0, 7.0)]).is_none());
    }

    #[test]
    fn a_tunnel_over_flat_ground_has_no_portals() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        assert!(portals(&p, &[span(300.0, 700.0)]).is_empty());
    }

    #[test]
    fn reconciled_tunnel_shrinks_to_its_buried_run_with_grade_stubs() {
        // Annotation [0.40, 0.62] but the road is buried only over
        // [≈0.3875, ≈0.6125] of a 1 km corridor: the low side grows nothing
        // (crossing outside the annotation is left to the bore's own march),
        // the high side frees [crossing, 0.62] as a painted grade stub.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.40 * len, level: 0, kind: SpanKind::Grade },
            span(0.40 * len, 0.62 * len),
            Span { arc0: 0.62 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = reconcile_spans(&p, &spans, &[]);
        assert_eq!(out.len(), 4, "tunnel + freed high-side stub: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Tunnel);
        assert!((out[1].arc0 - 0.40 * len).abs() < 1e-9, "low side stays annotated");
        assert!((out[1].arc1 - 612.5).abs() < 10.0, "high side clamps to the crossing");
        assert_eq!(out[2].kind, SpanKind::Grade);
        assert!((out[2].arc1 - 0.62 * len).abs() < 1e-9, "stub re-covers the slack");
    }

    #[test]
    fn the_shrink_does_not_take_back_what_the_burial_license_holds() {
        // The same annotation and the same geometry as above, but another
        // mapped alignment's band crosses the freed stub at 617 m. The
        // reference surface there is the cutting that alignment bridges or
        // embanks over, so the roof-fit run ends at 612.5 and the shrink used
        // to pave the last 7.5 m as open formation directly under the
        // crossing band — the Clarens underpass and the Veytaux shore line,
        // and 61 % of `order.grade_stack`'s violations over Montreux.
        let (p, len) = hill();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.40 * len, level: 0, kind: SpanKind::Grade },
            span(0.40 * len, 0.62 * len),
            Span { arc0: 0.62 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = reconcile_spans(&p, &spans, &[(611.0, 623.0)]);
        assert_eq!(out.len(), 3, "no stub is freed under the crossing: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Tunnel);
        assert!(
            (out[1].arc1 - 0.62 * len).abs() < 1e-9,
            "the tunnel keeps the licensed arc, clamped to the annotation"
        );
        // A window past the annotation grows nothing: growing is the annex's
        // half of the reconciliation, and it runs before this one.
        let far = reconcile_spans(&p, &spans, &[(630.0, 660.0)]);
        assert_eq!(far.len(), 4, "an unlicensed stub is still freed: {far:?}");
        assert!((far[1].arc1 - 612.5).abs() < 10.0);
    }

    #[test]
    fn reconciled_flat_ground_tunnel_becomes_grade() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        let out = reconcile_spans(&p, &[span(300.0, 700.0)], &[]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].kind, SpanKind::Grade, "nothing buried: paint it as road");
    }

    #[test]
    fn a_bridge_stops_growing_where_the_ground_comes_back_up() {
        // The Territet funicular's case: absorption marks nodes past the
        // annotated deck end as structure, but the ground rises over the road
        // there (an embankment). Grown blindly, the deck is swept into the
        // hillside and drawn metres under the terrain.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 400.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 101;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        // Road climbs steadily; ground dips under the deck, then overtakes it.
        let road: Vec<f64> = (0..n).map(|i| 100.0 + i as f64).collect();
        let terrain: Vec<f64> = (0..n)
            .map(|i| if i < 60 { 100.0 + i as f64 - 10.0 } else { 100.0 + i as f64 + 20.0 })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        // `from_heights` flags every node at grade, so drive the growth off a
        // profile whose absorbed run reaches past the rise.
        let spans = vec![
            Span { arc0: 0.0, arc1: 100.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 100.0, arc1: 150.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 150.0, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let out = grow_spans(&p, &spans);
        let deck = out.iter().find(|s| s.kind == SpanKind::Bridge).expect("bridge kept");
        let arc = p.arc();
        let over = arc.partition_point(|&a| a < deck.arc1).min(arc.len() - 1);
        assert!(
            p.road_m()[over] >= p.terrain_m()[over] - 1e-6,
            "deck ends at arc {:.1} where the road is {:.1} under the ground",
            deck.arc1,
            p.terrain_m()[over] - p.road_m()[over],
        );
    }

    #[test]
    fn grown_spans_cover_the_absorbed_stretch() {
        // A grade-limited solve that absorbs a cliff into a bridge span (see
        // profile::tests::a_structure_ending_at_a_cliff_is_extended_not_pitched):
        // grow_spans must extend the bridge over the absorbed nodes and shrink
        // the following grade span to keep the partition.
        let cos_lat = 46.0_f64.to_radians().cos();
        let len = 4000.0;
        let deg = len / (DEG_M * cos_lat);
        let n = 512;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let mut elev = |c: Coord| {
            let x = (c.x - 6.0) / deg; // 0..1
            if x < 0.5 {
                100.0
            } else if x < 0.51 {
                100.0 + 3000.0 * (x - 0.5) // the wall: +30 m over ~40 m
            } else {
                130.0
            }
        };
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.3 * len, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.3 * len, arc1: 0.5 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.5 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        let p = crate::solve::profile::solve(&nodes, &spans, crate::solve::profile::Mode::Engineered { grade: 0.06 }, &mut elev)
            .expect("non-degenerate corridor");
        let out = grow_spans(&p, &spans);
        assert_eq!(out.len(), 3, "the partition keeps its three spans: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Bridge);
        assert!(
            out[1].arc1 > 0.5 * len + 10.0,
            "bridge must grow over the absorbed wall, got arc1 {}",
            out[1].arc1
        );
        assert!((out[2].arc0 - out[1].arc1).abs() < 1e-9, "grade span shrinks to match");
        assert!((out[2].arc1 - len).abs() < 1e-9, "the far boundary is untouched");
        assert!((out[1].arc0 - 0.3 * len).abs() < 1e-9, "the feasible low side is untouched");
    }

    #[test]
    fn a_run_to_the_corridor_end_keeps_that_side_open() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let n = 101;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        // Buried from the very start, emerging at u≈0.6.
        let road = vec![100.0; n];
        let terrain: Vec<f64> = (0..n)
            .map(|i| if (i as f64 / (n - 1) as f64) < 0.6 { 120.0 } else { 90.0 })
            .collect();
        let p = Profile::from_heights(&nodes, road, terrain);
        let ps = portals(&p, &[span(0.0, 550.0)]);
        assert_eq!(ps.len(), 1, "only the emerging side gets a portal");
        assert_eq!(ps[0].outward, 1.0);
    }

    /// A 500 m corridor of 101 nodes with a bridge span in the middle and
    /// per-node road/terrain as given — the hanging-approach fixture.
    fn hung(road: Vec<f64>, terrain: Vec<f64>) -> (Profile, Vec<Span>) {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 500.0 / (DEG_M * cos_lat);
        let n = road.len();
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 225.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 225.0, arc1: 275.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 275.0, arc1: 500.0, level: 0, kind: SpanKind::Grade },
        ];
        (Profile::from_heights(&nodes, road, terrain), spans)
    }

    /// The Chauderon shape (S20): the relaxation left both approaches of a
    /// mapped bridge 8–17 m over a reference that plunges faster than the
    /// road may descend. The hanging runs are the structure's; the write-back
    /// (absorb, then grow) extends the deck over them, and the far at-grade
    /// road survives.
    #[test]
    fn a_hanging_approach_beside_a_bridge_is_absorbed_into_the_deck() {
        let n = 101; // node every 5 m
        // Road: level 100 everywhere (the lifted deck height carried outward).
        let road = vec![100.0; n];
        // Terrain: at road level on the outer thirds, diving to 84 under the
        // span and its approaches — hanging (>5 m) for u in (0.30, 0.70).
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                let d = (u - 0.5).abs();
                if d < 0.2 { 84.0 + (d / 0.2) * 16.0 } else { 100.0 }
            })
            .collect();
        let (mut p, spans) = hung(road, terrain);
        absorb_hanging_approaches(&mut p, &spans, false);
        let out = reconcile_spans(&p, &spans, &[]);
        assert_eq!(out.len(), 3, "the partition keeps three spans: {out:?}");
        let deck = &out[1];
        assert_eq!(deck.kind, SpanKind::Bridge);
        // The deck now covers the hanging runs on both sides — standoff
        // crosses 5 m at u = 0.5 ± 0.1375 (arc 181/319), so with 5 m nodes
        // the grown ends land at 185 and 315 — and no further.
        assert!(
            (deck.arc0 - 185.0).abs() < 6.0,
            "deck start must reach the 5 m-standoff point, got {:.1}",
            deck.arc0
        );
        assert!(
            (deck.arc1 - 315.0).abs() < 6.0,
            "deck end must reach the 5 m-standoff point, got {:.1}",
            deck.arc1
        );
        // Beyond the absorbed runs the road is on the ground and stays grade.
        assert_eq!(out[0].kind, SpanKind::Grade);
        assert_eq!(out[2].kind, SpanKind::Grade);
    }

    /// An ordinary embankment approach — under the absorb threshold beside
    /// the deck — is ground the earthworks own, and stays at grade (the p99
    /// approach berm is ~2.5 m; absorbing it would turn every overpass ramp
    /// into deck).
    #[test]
    fn an_ordinary_embankment_approach_stays_at_grade() {
        let n = 101;
        let road = vec![100.0; n];
        // Terrain 4 m under the road beside the span: an embankment, not a hang.
        let terrain: Vec<f64> = (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                if (0.45..=0.55).contains(&u) { 90.0 } else { 96.0 }
            })
            .collect();
        let (mut p, spans) = hung(road, terrain);
        let before = spans.clone();
        absorb_hanging_approaches(&mut p, &spans, false);
        assert!(p.at_grade().iter().enumerate().all(|(k, &g)| {
            let inside = p.arc()[k] >= 225.0 && p.arc()[k] <= 275.0;
            g || inside
        }), "no node outside the mapped span may be absorbed");
        let out = reconcile_spans(&p, &spans, &[]);
        assert_eq!(out, before, "an embankment approach leaves the spans alone");
    }
}
