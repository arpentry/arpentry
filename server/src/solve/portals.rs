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

use super::profile::Profile;

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
    let arc = profile.arc();
    let road = profile.road_m();
    let terrain = profile.terrain_m();
    let n = arc.len();
    if n == 0 {
        return false;
    }
    let line = |i: usize| road[i] - terrain[i];
    let mut s = arc.partition_point(|&a| a < at_arc).min(n - 1);
    if line(s) >= 0.0 {
        if s > 0 && line(s - 1) < 0.0 {
            s -= 1;
        } else if s + 1 < n && line(s + 1) < 0.0 {
            s += 1;
        } else {
            return false;
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
    let fit = (f..=l).filter(|&i| roof_gap(road[i], terrain[i]) < 0.0).count();
    2 * fit >= l - f + 1
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

/// Tunnel spans extended through the crossings their still-buried tails pass
/// beneath — the *growing* half of portal placement. Both halves run in the
/// per-stratum write-back (`solve::reconcile_stratum`): the annex first, then
/// the shrinking half ([`reconcile_spans`]), and the result is written into
/// the scene as the one span truth every consumer cuts (§4.5).
///
/// An annotation edge is where a mapper split the segment (S5), and a bore
/// whose tail is still below its own terrain when another mapped alignment
/// crosses it has not emerged: the ground it must pierce includes that
/// feature's band and bench. Left as annotated, the buried tail is paved as
/// open formation — benched, holed, sheeted — sliding beneath the crossing
/// feature's band metres up, which is exactly the two-superposed-lines drawing
/// the Collonge funicular made over the rack railway's short portal
/// (`order.grade_stack` keeps the class dead).
///
/// The gate is deliberately double: the tail must be buried (the tube still
/// fits under the ground — [`roof_gap`], searched by [`span_bounds`] out to
/// [`PORTAL_MAX_M`]) *and*
/// crossed (`crossings::plan_crossings`). Burial alone would swallow the open
/// trench approach of a flat-ground underpass (S6); a crossing alone is any
/// street the line meets at grade. Each qualifying side extends to the last
/// such crossing plus its `clear_m`, eating only neighbouring *grade* spans —
/// a mapped bridge or a second bore is a boundary the annex never moves.
///
/// Returns `None` when nothing changed.
pub fn annex_spans(
    profile: &Profile,
    spans: &[Span],
    crossings: &[(f64, f64)],
) -> Option<Vec<Span>> {
    if crossings.is_empty() {
        return None;
    }
    let total = *profile.arc().last()?;
    let mut out = spans.to_vec();
    let mut changed = false;
    for i in 0..out.len() {
        if out[i].kind != SpanKind::Tunnel {
            continue;
        }
        let Some((low, high)) = span_bounds(profile, &out[i]) else {
            continue;
        };
        // High side: the buried run past the annotation edge, bounded by the
        // true emergence when one exists and by the search reach when the run
        // never surfaces (out of data, not out of the hill). A crossing
        // qualifies when its *band* pokes past the span end — its centre may
        // sit metres inside the annotation and the band still straddle the
        // edge, which was the Collonge measurement: crossings 1.4 m inside a
        // snapped span end, band reaching 4.4 m beyond it.
        let tail_hi = high.unwrap_or(out[i].arc1 + PORTAL_MAX_M).min(total);
        let mut target = out[i].arc1;
        for &(x, clear) in crossings {
            if x > out[i].arc0 && x <= tail_hi && x + clear > out[i].arc1 {
                target = target.max((x + clear).min(total));
            }
        }
        if target > out[i].arc1 {
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
            if a1 > out[i].arc1 {
                out[i].arc1 = a1;
                changed = true;
            }
        }
        // Low side, mirrored.
        let tail_lo = low.unwrap_or(out[i].arc0 - PORTAL_MAX_M).max(0.0);
        let mut target = out[i].arc0;
        for &(x, clear) in crossings {
            if x < out[i].arc1 && x >= tail_lo && x - clear < out[i].arc0 {
                target = target.min((x - clear).max(0.0));
            }
        }
        if target < out[i].arc0 {
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
            if a0 < out[i].arc0 {
                out[i].arc0 = a0;
                changed = true;
            }
        }
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
pub fn reconcile_spans(profile: &Profile, spans: &[Span]) -> Vec<Span> {
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
        let Some((low, high)) = span_bounds(profile, s) else {
            out.push(Span { level: 0, kind: SpanKind::Grade, ..*s });
            continue;
        };
        let a0 = low.map_or(s.arc0, |a| a.max(s.arc0));
        let a1 = high.map_or(s.arc1, |a| a.min(s.arc1));
        if a1 - a0 < MIN_STUB_M {
            out.push(Span { level: 0, kind: SpanKind::Grade, ..*s });
            continue;
        }
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
        let out = annex_spans(&p, &spans, &[(560.0, 6.0)]).expect("the span must grow");
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
        let out = annex_spans(&p, &spans, &[(545.0, 6.0)]).expect("the span must grow");
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
        assert!(annex_spans(&p, &spans, &[(700.0, 6.0)]).is_none());
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
        assert!(annex_spans(&p, &spans, &[(570.0, 6.0)]).is_none());
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
        let out = reconcile_spans(&p, &spans);
        assert_eq!(out.len(), 4, "tunnel + freed high-side stub: {out:?}");
        assert_eq!(out[1].kind, SpanKind::Tunnel);
        assert!((out[1].arc0 - 0.40 * len).abs() < 1e-9, "low side stays annotated");
        assert!((out[1].arc1 - 612.5).abs() < 10.0, "high side clamps to the crossing");
        assert_eq!(out[2].kind, SpanKind::Grade);
        assert!((out[2].arc1 - 0.62 * len).abs() < 1e-9, "stub re-covers the slack");
    }

    #[test]
    fn reconciled_flat_ground_tunnel_becomes_grade() {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 1000.0 / (DEG_M * cos_lat);
        let nodes: Vec<Coord> =
            (0..101).map(|i| Coord { x: 6.0 + deg * i as f64 / 100.0, y: 46.0 }).collect();
        let p = Profile::from_heights(&nodes, vec![100.0; 101], vec![95.0; 101]);
        let out = reconcile_spans(&p, &[span(300.0, 700.0)]);
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
}
