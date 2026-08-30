//! Stage 3 — solve the vertical model (docs/GENERATION.md §5).
//!
//! One pass over the assembled scene graph turns topology into geometry:
//! every corridor that needs a vertical model — every drivable road, plus
//! anything carrying a structure span — gets a [`Profile`] (docs/GROUND.md
//! §1): road-surface heights everywhere along it, anchored to the reference
//! terrain at its at-grade spans and interpolated at a gentle grade across
//! its structures. The [`Mode`] the corridor's class implies parameterises
//! the solve; there is no second vertical model.
//!
//! The reference terrain is the *rendered* ground at the reference zoom (the
//! run's maximum): the same global [`terrain::surface_height`] lattice the
//! emit workers mesh, so a solved at-grade anchor sits exactly on the drawn
//! ground at that zoom. The solved heights are a function of the scene graph
//! and the DEM only — never of a tile window — so every tile fragment reads
//! identical heights (invariant 5), and heights do not change between zoom
//! levels (no popping).

pub mod consistency;
pub mod crossings;
pub mod graph;
pub mod partition;
pub mod portals;
pub mod profile;
pub mod relax;
pub mod structures;

use std::path::Path;
use std::sync::Mutex;

use geo_types::Coord;

use crate::dem::Dem;
use crate::priors::{Stratum, MIN_STRUCTURE_M, SHORT_STRUCTURE_DIP_M};
use crate::project::Bounds;
use crate::scene::{CorridorId, SceneGraph, Span, SpanKind};
use crate::terrain;

pub use profile::{Mode, Profile};

type Error = Box<dyn std::error::Error + Send + Sync>;

/// One measured site of the crossing premise: a mapped bore, crossed by an
/// at-grade band, scored by how far its roof-plus-cover stands above this
/// corridor's own ground there. Positive means the "tunnel" the crossing
/// machinery declined to buy clearance from does not actually pass beneath
/// the ground the crossing feature rides on — the two drawn surfaces then
/// stack with neither a bore nor a deck between them.
#[derive(Debug, Clone, Copy)]
pub struct Daylight {
    pub corridor: CorridorId,
    pub arc: f64,
    pub lon: f64,
    pub lat: f64,
    /// `road + TUNNEL_HEIGHT_M + TUNNEL_COVER_M − terrain`, signed: negative
    /// is honest burial margin, positive is roof daylighting through the
    /// crossing feature's roadbed.
    pub deficit_m: f64,
}

/// Where the pure partition ([`partition::partition`]) disagrees with the
/// fold's spans for one corridor — the diagnostic of the two-pass refactor's
/// first step (`data/plans/pure-partition-2026-08-28.md` §5).
#[derive(Debug, Clone, Copy)]
pub struct PartitionDivergence {
    pub corridor: CorridorId,
    pub lon: f64,
    pub lat: f64,
    pub d: partition::Divergence,
}

/// One stratum's reconciliation write-back — where the annotation hands over
/// to the solved truth (§4.5).
///
/// For each of the stratum's corridors: grow its tunnel spans through the
/// crossings their buried tails pass beneath ([`portals::annex_spans`]),
/// measure the crossing premise at the covered-bore sites
/// (`structure.bore_daylight` — before reconciliation rewrites the spans it
/// is stated against), then clamp every tunnel span to its buried run and
/// re-cover the freed slack as grade ([`portals::reconcile_spans`]), flipping
/// the profile's at-grade flags over each degraded stretch so the benches and
/// the bands read the same partition as the paint. The result is written into
/// `scene.corridors[..].spans`: after this, there is exactly one span truth
/// and every consumer — junior solves included — reads it.
///
/// The two halves read the *same* burial license `covered` that seeded the
/// ceilings (`relax::seed_bore_ceilings`): the annex grows a span through the
/// crossings its buried tail passes beneath, and the shrink may not take back
/// what the license holds. Handed the reference surface alone, the shrink
/// undid the annex on the same corridor in the same sweep, which is the
/// superposition `order.grade_stack` counts.
fn reconcile_stratum(
    scene: &mut SceneGraph,
    profiles: &mut [Option<Profile>],
    stratum: Stratum,
    reaches: &[Vec<(f64, f64)>],
    carried: &[Vec<(f64, f64)>],
    covered: &[Vec<(f64, f64)>],
    sites: &[Vec<crossings::PlanCrossing>],
    daylight: &mut Vec<Daylight>,
    divergence: &mut Vec<PartitionDivergence>,
    pass: usize,
) {
    // ARPT_DEBUG_ANNEX: one line per tunnel-bearing corridor with crossings —
    // the tail bounds against the crossing arcs, and whether the annex took.
    let debug_annex = std::env::var_os("ARPT_DEBUG_ANNEX").is_some();
    // The twin-bore entity (S8): computed before the per-corridor loop so the
    // windows read every sibling's annotation, spliced where a twin is at
    // grade, and held below against the shrink where it is not.
    let twins = twin_bore_windows(scene, profiles, stratum);
    apply_twin_windows(scene, profiles, &twins, debug_annex);
    for c in scene.corridors.iter_mut() {
        if c.kind.stratum() != stratum {
            continue;
        }
        let Some(p) = profiles.get_mut(c.id as usize).and_then(|p| p.as_mut()) else {
            continue;
        };
        let reaches = reaches.get(c.id as usize).cloned().unwrap_or_default();
        if debug_annex && c.spans.iter().any(|s| s.kind == SpanKind::Tunnel) {
            for s in c.spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
                let bounds = portals::span_bounds(p, s);
                let near: Vec<&(f64, f64)> = reaches
                    .iter()
                    .filter(|(x, _)| *x > s.arc0 - 250.0 && *x < s.arc1 + 250.0)
                    .collect();
                eprintln!(
                    "[annex] corridor {} {:?} tunnel [{:.1}, {:.1}] bounds {:?} crossings {:?}",
                    c.id, c.kind, s.arc0, s.arc1, bounds, near
                );
            }
        }
        // The deck contract mirrors `relax::reconstruct`: a monotone class's
        // deck is its line; everyone else refits per-run ramps.
        let deck_follows_road = c.kind.prior().monotone
            && profile::monotone_direction(p.terrain_m()).is_some();
        let mut spans = std::mem::take(&mut c.spans);
        // The spans as they enter the fold: the pure partition's input.
        let entering = spans.clone();
        // **Pass 2 freezes the partition** (the quiet week's first lever,
        // chosen from the two bba7dbd names): the structure predicates read
        // pass 1's verdicts as settled — no annex, no absorb, no shrink —
        // because they are not idempotent under the re-relax (graph-build ∘
        // relax moves heights from its own output, approaches hang again,
        // and the fold grew 1,834 m of bridge on identical profiles). Pass 1
        // already wrote the reconciled truth; pass 2 re-solves heights
        // against it and hands it back unchanged.
        if pass > 0 {
            c.spans = spans;
            continue;
        }
        let carried = carried.get(c.id as usize).cloned().unwrap_or_default();
        if let Some(annexed) = portals::annex_spans(p, &spans, &reaches, &carried) {
            if debug_annex {
                eprintln!("[annex] corridor {} {:?} annexed: {:?}", c.id, c.kind, annexed);
            }
            for s in annexed.iter().filter(|s| s.kind != SpanKind::Grade) {
                p.annex_structure(s.arc0, s.arc1, deck_follows_road);
            }
            spans = annexed;
        }
        // The crossing premise, measured against the spans the solve used.
        for x in &sites[c.id as usize] {
            let roof = p.road_at_arc(x.arc)
                + crate::priors::TUNNEL_HEIGHT_M
                + crate::priors::TUNNEL_COVER_M;
            let pt = p.point_at_arc(x.arc);
            if let Some(dbg) = std::env::var_os("ARPT_DEBUG_BURY") {
                if dbg.to_string_lossy().parse::<u32>() == Ok(c.id) {
                    eprintln!(
                        "[bury] daylight corridor {} arc={:.1} road={:.2} surface={:.2} deficit={:+.2}",
                        c.id,
                        x.arc,
                        p.road_at_arc(x.arc),
                        p.surface_at_arc(x.arc),
                        roof - p.surface_at_arc(x.arc)
                    );
                }
            }
            daylight.push(Daylight {
                corridor: c.id,
                arc: x.arc,
                lon: pt.x,
                lat: pt.y,
                deficit_m: roof - p.surface_at_arc(x.arc),
            });
        }
        // The deck twin of the tunnel annex above: approaches the relaxation
        // left hanging beside a bridge span (standoff past the absorb
        // threshold, at grade) are the structure's — flagged here, grown into
        // the partition by `reconcile_spans`' grow step (S10/S20).
        portals::absorb_hanging_approaches(p, &spans, deck_follows_road);
        // Shrink to the geometry: each tunnel clamped to its buried run, the
        // freed annotation slack re-covered as painted grade, a tunnel with
        // no buried run at all degraded end to end — except where the burial
        // license holds, which the reference surface cannot see, and where a
        // twin's vetted bore runs beside it, which this line's own fit
        // cannot see either (the twin-bore entity, [`twin_bore_windows`]).
        let covered = covered.get(c.id as usize).map(Vec::as_slice).unwrap_or(&[]);
        let twin: Vec<(f64, f64)> = twins[c.id as usize].iter().map(|&(a, b, _)| (a, b)).collect();
        let mut reconciled = portals::reconcile_spans(p, &spans, covered, &twin);
        // The bridge half of the pure partition, in the fold where every
        // consumer reads it (`ARPT_BRIDGE_TRIM=1`). The 2026-08-28 slice was
        // withdrawn because the trimmed stretches were never degraded: the
        // spans said grade, the profile still held the annotated deck ramp,
        // and the handover cut and the sweep — built from the ramp — did not
        // move with the partition, so every trimmed bridge gained a joint
        // that could not meet. The degrade below is the tunnel loop's exact
        // treatment applied to bridges, which is what "the partition moves
        // with its consumers" means here: the ramp refits to the trimmed
        // extent, and the cut, the sweep, the sheets and the benches all read
        // the same trimmed truth.
        if std::env::var_os("ARPT_BRIDGE_TRIM").is_some() {
            reconciled = partition::bridge_trim(p, &reconciled, c.kind.prior());
        }
        for g in reconciled.iter().filter(|s| s.kind == SpanKind::Grade) {
            for t in spans.iter().filter(|s| s.kind != SpanKind::Grade) {
                let (lo, hi) = (g.arc0.max(t.arc0), g.arc1.min(t.arc1));
                if hi - lo > f64::EPSILON {
                    p.degrade_structure(lo, hi, deck_follows_road);
                }
            }
        }
        // Two-pass attribution (`ARPT_TWO_PASS_DIVERGENCE=1`, pass 2 only):
        // where the second pass's fold output differs from the first's — the
        // non-fixpoint 167a143 measured, per corridor and per kind family, so
        // step 3's blocker is a list of mechanisms instead of a total.
        // `entering` IS pass 1's reconciled spans on pass 2, so the divergence
        // between the passes is divergence(entering, reconciled) verbatim.
        if pass > 0 && std::env::var_os("ARPT_TWO_PASS_DIVERGENCE").is_some() {
            let d = partition::divergence(&entering, &reconciled);
            if d.metres > 0.5 {
                let pt = p.point_at_arc(d.worst_arc);
                eprintln!(
                    "[two-pass] corridor {} {:?} {:.1} m differ: b→g {:.1} g→b {:.1} t→g {:.1} g→t {:.1} other {:.1} at {:.6},{:.6} (longest {:.0} m)",
                    c.id, c.kind, d.metres, d.bridge_to_grade, d.grade_to_bridge,
                    d.tunnel_to_grade, d.grade_to_tunnel, d.other, pt.x, pt.y, d.worst_metres
                );
            }
        }
        // The pure partition, computed from the same inputs and compared —
        // never written. Its distance from the fold is what the two-pass
        // switch is judged against before anything moves.
        let pure = partition::partition(
            p,
            &entering,
            &partition::Licenses { covered, twin: &twin, reaches: &reaches, carried: &carried },
            c.kind.prior(),
        );
        let d = partition::divergence(&reconciled, &pure);
        if d.metres > 0.0 || reconciled.iter().any(|s| s.kind != SpanKind::Grade) {
            let pt = p.point_at_arc(if d.metres > 0.0 { d.worst_arc } else { 0.5 * p.arc().last().copied().unwrap_or(0.0) });
            divergence.push(PartitionDivergence { corridor: c.id, lon: pt.x, lat: pt.y, d });
        }
        c.spans = reconciled;
    }
    carry_stubs_welded_onto_decks(scene, profiles, stratum, debug_annex);
}

/// S8's entity rule, applied to rail twins: where a corridor's twin runs in a
/// bore beside it, the corridor is in that bore too — one formation, one
/// trench, one portal, per twin pair.
///
/// One physical double-track railway arrives as two corridors, each carrying
/// its own span annotation and each vetted against its *own* solved line, and
/// the two verdicts can disagree: at Chamby both lines are annotated in
/// tunnel end to end, but the reconciliation's shrink kept 118 m of bore on
/// one line and 9 m on its twin four metres away — so the twin's freed slack
/// benched a 5.6 m open cutting through its sibling's cover, and
/// `clearance.bore_cover` read the roof that far proud (134 m of it over the
/// Montreux extract, the family's whole real mass once the accepted
/// mouth-transition design is set aside; the standard-gauge pair at Territet
/// is the other site). The same reasoning that gives a structure entity one
/// grade line under parallel carriageways (§4.4), applied to the span
/// partition.
///
/// The windows computed here are the mechanism, used twice:
///
/// - where the twin is annotated at *grade*, the window is spliced in as a
///   tunnel span ([`apply_twin_windows`]) and the reconciliation that follows
///   vets it like any annotated tunnel;
/// - where the twin's own tunnel is about to *shrink*, the window rides into
///   [`portals::reconcile_spans`] beside the burial license: the kept
///   interval may not fall below what the sibling's bore holds beside it.
///
/// Both directions are needed because the vetting is per-line and the entity
/// is the pair; neither alone survived measurement (the splice was clamped
/// straight back out by the shrink it did not protect against).
///
/// The gates: rail only and same class (the census's population — the
/// lateral gate alone would admit a narrow pair of parallel streets whose
/// tunnels are genuinely separate); every windowed node within
/// [`graph::TWIN_TRACK_LATERAL_M`] of the sibling's line (the width that
/// already welds a passing loop's variables); and the two solved beds within
/// [`TWIN_LEVEL_M`] of each other — parallel tracks on one roadbed agree to
/// well under a metre, and without the height gate a station siding running
/// six metres *above* a bore under its throat would adopt that bore and draw
/// itself underground. Twins share a stratum, so reading both solved heights
/// inverts no authority (§4.1).
fn twin_bore_windows(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    stratum: Stratum,
) -> Vec<Vec<(f64, f64, i64)>> {
    use crate::priors::Kind;
    /// Shortest window worth acting on, in metres — below this it is
    /// quantization noise between two mouths, not a shared bore.
    const MIN_ADOPT_M: f64 = 2.0;
    /// How far apart the two solved beds may sit where they share a
    /// formation, in metres.
    const TWIN_LEVEL_M: f64 = 1.5;
    let mut windows: Vec<Vec<(f64, f64, i64)>> = vec![Vec::new(); scene.corridors.len()];
    let rail = |c: &crate::scene::Corridor| {
        c.kind.stratum() == stratum && matches!(c.kind, Kind::Rail(_))
    };
    let rails: Vec<u32> = scene
        .corridors
        .iter()
        .filter(|c| rail(c))
        .filter(|c| profiles.get(c.id as usize).and_then(|p| p.as_ref()).is_some())
        .map(|c| c.id)
        .collect();
    if rails.len() < 2 {
        return windows;
    }
    // Every vetted bore of the stratum: the fitted interval of each annotated
    // tunnel span, in its own corridor's arc.
    let mut bores: Vec<(u32, f64, f64, i64)> = Vec::new();
    for &a in &rails {
        let c = &scene.corridors[a as usize];
        let p = profiles[a as usize].as_ref().expect("profiled");
        for s in c.spans.iter().filter(|s| s.kind == SpanKind::Tunnel) {
            let Some((low, high)) = portals::span_bounds(p, s) else {
                continue; // no buried run: this tunnel is about to degrade
            };
            let (lo, hi) =
                (low.map_or(s.arc0, |x| x.max(s.arc0)), high.map_or(s.arc1, |x| x.min(s.arc1)));
            if hi - lo >= MIN_ADOPT_M {
                bores.push((a, lo, hi, s.level));
            }
        }
    }
    if bores.is_empty() {
        return windows;
    }
    for &b in &rails {
        let cb = &scene.corridors[b as usize];
        let pb = profiles[b as usize].as_ref().expect("profiled");
        for &(a, t0, t1, level) in &bores {
            if a == b || scene.corridors[a as usize].class_key != cb.class_key {
                continue;
            }
            let ca = &scene.corridors[a as usize];
            let pa = profiles[a as usize].as_ref().expect("profiled");
            // The stretch of `b` standing on the sibling's formation over its
            // bore: maximal node runs within the twin width of the bored
            // interval, at the formation's own level. Nodes, not span ends —
            // the twins are parameterised independently and only the plan can
            // align them.
            let (nodes, arc) = (pb.nodes(), pb.arc());
            let mut start: Option<usize> = None;
            for k in 0..=nodes.len() {
                let inside = k < nodes.len() && {
                    let q = nodes[k];
                    let aa = pa.arc_of(q.x, q.y);
                    aa >= t0 && aa <= t1 && {
                        let w = pa.point_at_arc(aa);
                        let dx = (w.x - q.x) * ca.cos_lat * crate::scene::DEG_M;
                        let dy = (w.y - q.y) * crate::scene::DEG_M;
                        (dx * dx + dy * dy).sqrt() <= graph::TWIN_TRACK_LATERAL_M
                            && (pb.road_at_arc(arc[k]) - pa.road_at_arc(aa)).abs()
                                <= TWIN_LEVEL_M
                    }
                };
                match (inside, start) {
                    (true, None) => start = Some(k),
                    (false, Some(s0)) => {
                        if k - s0 >= 2 && arc[k - 1] - arc[s0] >= MIN_ADOPT_M {
                            windows[b as usize].push((arc[s0], arc[k - 1], level));
                        }
                        start = None;
                    }
                    _ => {}
                }
            }
        }
    }
    for w in windows.iter_mut() {
        w.sort_by(|x, y| x.0.total_cmp(&y.0));
    }
    windows
}

/// The splice half of the twin-bore entity ([`twin_bore_windows`]): windows
/// falling on a twin's *grade* spans become tunnel spans, with the profile
/// reshaped over them ([`Profile::annex_structure`], exactly as an annexed
/// span is). The per-corridor reconciliation that follows vets the splice
/// like any annotated tunnel — clamps it to the twin's own buried run plus
/// the window itself — so an over-generous splice costs nothing, and no
/// fixpoint is needed (the one built here before was refuted, 99b66e1).
fn apply_twin_windows(
    scene: &mut SceneGraph,
    profiles: &mut [Option<Profile>],
    windows: &[Vec<(f64, f64, i64)>],
    debug: bool,
) {
    const MIN_ADOPT_M: f64 = 2.0;
    for (cid, wins) in windows.iter().enumerate() {
        if wins.is_empty() {
            continue;
        }
        let c = &mut scene.corridors[cid];
        let p = profiles[cid].as_mut().expect("windowed corridors are profiled");
        for &(lo, hi, level) in wins {
            for (g0, g1) in grade_clip(&c.spans, lo, hi) {
                if g1 - g0 < MIN_ADOPT_M {
                    continue;
                }
                if debug {
                    eprintln!(
                        "[annex] corridor {} {:?} adopts twin bore [{g0:.1}, {g1:.1}]",
                        c.id, c.kind
                    );
                }
                splice_tunnel(&mut c.spans, g0, g1, level);
                let deck_follows_road = c.kind.prior().monotone
                    && profile::monotone_direction(p.terrain_m()).is_some();
                p.annex_structure(g0, g1, deck_follows_road);
            }
        }
    }
}

/// The sub-intervals of `[lo, hi]` not claimed by any structure span — what a
/// span list holds at grade, explicit grade spans and gaps alike.
fn grade_clip(spans: &[Span], lo: f64, hi: f64) -> Vec<(f64, f64)> {
    let mut taken: Vec<(f64, f64)> = spans
        .iter()
        .filter(|s| s.kind != SpanKind::Grade && s.arc1 > lo && s.arc0 < hi)
        .map(|s| (s.arc0.max(lo), s.arc1.min(hi)))
        .collect();
    taken.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut out = Vec::new();
    let mut cursor = lo;
    for (a0, a1) in taken {
        if a0 > cursor {
            out.push((cursor, a0));
        }
        cursor = cursor.max(a1);
    }
    if hi > cursor {
        out.push((cursor, hi));
    }
    out
}

/// Splices a tunnel over `[lo, hi]` into a span list, trimming the grade spans
/// it displaces and merging with an abutting tunnel of the same level — two
/// spans meeting inside one hill would each clamp to the shared buried run
/// and mesh a joint where no daylight is.
fn splice_tunnel(spans: &mut Vec<Span>, lo: f64, hi: f64, level: i64) {
    /// A displaced grade remnant shorter than this quantizes away.
    const MIN_STUB_M: f64 = 0.25;
    let mut out: Vec<Span> = Vec::with_capacity(spans.len() + 2);
    for s in spans.iter() {
        if s.kind != SpanKind::Grade || s.arc1 <= lo || s.arc0 >= hi {
            out.push(*s);
            continue;
        }
        if lo - s.arc0 > MIN_STUB_M {
            out.push(Span { arc1: lo, ..*s });
        }
        if s.arc1 - hi > MIN_STUB_M {
            out.push(Span { arc0: hi, ..*s });
        }
    }
    out.push(Span { arc0: lo, arc1: hi, level, kind: SpanKind::Tunnel });
    out.sort_by(|x, y| x.arc0.total_cmp(&y.arc0));
    let mut merged: Vec<Span> = Vec::with_capacity(out.len());
    for s in out {
        match merged.last_mut() {
            Some(t)
                if t.kind == SpanKind::Tunnel
                    && s.kind == SpanKind::Tunnel
                    && t.level == s.level
                    && s.arc0 - t.arc1 <= MIN_STUB_M =>
            {
                t.arc1 = t.arc1.max(s.arc1)
            }
            _ => merged.push(s),
        }
    }
    *spans = merged;
}

/// The write-back's second sweep: stubs a junction weld leaves standing in the
/// air on somebody's deck (`portals::carry_whole_corridor`).
///
/// Run after the whole stratum's spans are final, because the deck a stub
/// stands on may itself have been grown by the first sweep — Route de
/// Chernex's bridge reaches one of the two Chauderon welds only after
/// `absorb_hanging_approaches` extends it. Taken to a fixpoint, so the answer
/// does not depend on the order corridors are visited in: absorbing a stub can
/// only ever make another corridor eligible, never ineligible, so the least
/// fixpoint is unique. It terminates on the geometry rather than on a hop
/// count — a corridor with any ground of its own is never a candidate
/// ([`portals::hangs_end_to_end`]), so the cluster of hanging streets behind
/// the two stubs is not walked: the first of them, Chemin des Vuarennes,
/// descends to its own ground and is an embankment, not a span.
///
/// The deck must belong to this stratum or one senior to it. A senior's spans
/// are a published fact; reading a junior's would invert authority (§4.1) and
/// break the perturbation experiment.
fn carry_stubs_welded_onto_decks(
    scene: &mut SceneGraph,
    profiles: &mut [Option<Profile>],
    stratum: Stratum,
    debug: bool,
) {
    let mut candidates: Vec<CorridorId> = scene
        .corridors
        .iter()
        .filter(|c| c.kind.stratum() == stratum)
        .filter(|c| {
            profiles
                .get(c.id as usize)
                .and_then(|p| p.as_ref())
                .is_some_and(portals::hangs_end_to_end)
        })
        .map(|c| c.id)
        .collect();
    while !candidates.is_empty() {
        let Some(pick) = candidates.iter().position(|&id| {
            scene.junctions.iter().any(|j| {
                if !j.members.iter().any(|m| m.corridor == id) {
                    return false;
                }
                j.members.iter().filter(|o| o.corridor != id).any(|o| {
                    let oc = &scene.corridors[o.corridor as usize];
                    oc.kind.stratum() <= stratum
                        && oc.spans.iter().any(|s| {
                            s.kind == SpanKind::Bridge && o.arc >= s.arc0 && o.arc <= s.arc1
                        })
                })
            })
        }) else {
            break; // nothing left that stands on a deck
        };
        let id = candidates.swap_remove(pick);
        let Some(p) = profiles.get_mut(id as usize).and_then(|p| p.as_mut()) else { continue };
        let c = &mut scene.corridors[id as usize];
        let deck_follows_road = c.kind.prior().monotone
            && profile::monotone_direction(p.terrain_m()).is_some();
        c.spans = portals::carry_whole_corridor(p, &c.spans, deck_follows_road);
        if debug {
            eprintln!("[annex] corridor {id} {:?} carried whole: {:?}", c.kind, c.spans);
        }
    }
}

/// The solved vertical model: one profile per corridor that needs one, indexed
/// by [`CorridorId`]. Immutable after the solve; shared by every emit worker.
pub struct SolvedModel {
    /// The structures the solved heights imply, by [`CorridorId`] (§4.5).
    ///
    /// An *output*. A deck exists where the solved surface departs the ground,
    /// not where a mapper wrote `bridge` — so "a crossing whose bridge was
    /// deleted" is unrepresentable rather than merely rare.
    pub structures: Vec<Vec<structures::StructureRun>>,
    /// The crossings this solve derived (§4.5). They live on the *output*
    /// because they are a consequence of the solved heights, not an input to
    /// them: stored on the scene they went stale the moment anything changed a
    /// span, and nothing re-derived them.
    pub crossings: Vec<crate::scene::Crossing>,
    /// What the relaxation could not honour — the clearance demands its
    /// plausibility cap rejected. Carried on the model so the run can report
    /// them: a silently dropped constraint is indistinguishable from one that
    /// was satisfied.
    pub relaxed: relax::Relaxed,
    /// Per-constraint-family residuals at the solved point
    /// ([`relax::residuals`]), merged across strata. Carried on the model so
    /// the scorecard can say which constraints actually hold at the output —
    /// the pass order in the relaxation is load-bearing, and before this the
    /// only evidence it worked on a given scene was the absence of drawn
    /// artifacts downstream.
    pub residuals: Vec<relax::PassResidual>,
    /// The crossing premise, measured (`structure.bore_daylight`): one entry
    /// per place a mapped tunnel span is crossed by another alignment's
    /// at-grade band. Carried on the model because the premise is about how
    /// the scene was *solved* — the crossing machinery waives clearance
    /// wherever the lower side's annotation says "bore"
    /// (`graph::in_immovable_bore`), and these entries are that waiver's
    /// collateral, measured against the solved heights before any
    /// reconciliation rewrites the spans.
    pub daylight: Vec<Daylight>,
    /// Where the pure partition disagrees with the written spans
    /// (`partition.divergence`), one entry per corridor with any structure.
    pub partition: Vec<PartitionDivergence>,
    profiles: Vec<Option<Profile>>,
    /// The height every junction's members share, by index into
    /// `SceneGraph::junctions`; `None` where no member carries a profile. Dense
    /// because that index is already the currency of every junction consumer,
    /// and because a hashed order is a determinism hazard.
    junction_h: Vec<Option<f64>>,
    /// The zoom whose rendered terrain lattice anchored the solve.
    pub z_ref: u8,
}

impl SolvedModel {
    /// A model with no profiles — the DEM-less run, where nothing is elevated.
    pub fn empty(z_ref: u8) -> SolvedModel {
        SolvedModel {
            structures: Vec::new(),
            crossings: Vec::new(),
            relaxed: relax::Relaxed::default(),
            residuals: Vec::new(),
            daylight: Vec::new(),
            partition: Vec::new(),
            profiles: Vec::new(),
            junction_h: Vec::new(),
            z_ref,
        }
    }

    /// Wraps already-solved profiles — for tests and stage-isolated tooling.
    /// Junction heights are unknown on this path; the surface then falls back to
    /// the corridors' own profiles, which is what it does at an unprofiled
    /// intersection anyway.
    pub fn from_profiles(profiles: Vec<Option<Profile>>, z_ref: u8) -> SolvedModel {
        SolvedModel {
            structures: Vec::new(),
            crossings: Vec::new(),
            relaxed: relax::Relaxed::default(),
            residuals: Vec::new(),
            daylight: Vec::new(),
            partition: Vec::new(),
            profiles,
            junction_h: Vec::new(),
            z_ref,
        }
    }

    /// Attaches solved junction heights to a model assembled in stages — the
    /// counterpart of [`SolvedModel::from_profiles`] for a caller that also ran
    /// the fuse.
    pub fn with_junction_heights(mut self, junction_h: Vec<Option<f64>>) -> SolvedModel {
        self.junction_h = junction_h;
        self
    }

    pub fn profile(&self, corridor: CorridorId) -> Option<&Profile> {
        self.profiles.get(corridor as usize)?.as_ref()
    }

    /// The solved height of a junction, by its index in `SceneGraph::junctions`.
    /// `None` when the intersection has no profiled member, so nothing is known
    /// about where its surface sits.
    pub fn junction_height(&self, junction: usize) -> Option<f64> {
        self.junction_h.get(junction).copied().flatten()
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
///
/// The scene is mutable for one reconciliation the assemble stage could not
/// make: the terrain fate of sub-[`MIN_STRUCTURE_M`] structure spans
/// ([`reconcile_short_spans`]), which rewrites the corridor spans every later
/// consumer (profiles, earthworks, emit) reads.
pub fn run(
    scene: &mut SceneGraph,
    terrain_path: Option<&Path>,
    z_ref: u8,
    threads: usize,
) -> Result<SolvedModel, Error> {
    run_licensed(scene, terrain_path, z_ref, threads, None)
}

/// The plan skeleton, as the solve consumes it: per corridor, the burial
/// licenses (`crossings::covered_bores`), the crossing reaches the annex
/// walks (`crossings::reaches`), and the spans-over-a-corridor exemptions the
/// short-span demotion honours (`crossings::spans_over_a_mapped_line` — indexed
/// per span, parallel to the corridor's annotated spans). Heights-free by
/// construction — arcs, band reaches, level ordinals and crossing existence
/// only.
pub struct PlanPin {
    pub covered: Vec<Vec<(f64, f64)>>,
    /// The parallel-overlap burial windows (`crossings::lateral_cover`),
    /// `(arc0, arc1, surface_m)` per corridor. Carries a raw-DEM surface —
    /// input data, not a junior's solved height — but derives from junior
    /// alignments' existence, so the perturbation experiment must pin it
    /// with the rest of the skeleton.
    pub lateral: Vec<Vec<(f64, f64, f64)>>,
    pub reaches: Vec<Vec<(f64, f64)>>,
    /// The carrying license (`crossings::carried_crossings`): where a mapped
    /// bridge span crosses an alignment annotated below it. Pinned with the
    /// rest of the skeleton for the same reason `covered` is — it is a junior
    /// alignment's *existence* classifying a senior's spans.
    pub carried: Vec<Vec<(f64, f64)>>,
    pub over: Vec<Vec<bool>>,
}

/// [`run`], with the plan skeleton supplied rather than derived from this
/// scene.
///
/// The skeleton is **input data** — where mapped alignments cross, how far
/// their bands reach, and their level ordinals — which I7 counts alongside
/// the annotation, never as a junior's solved output (no junior *height* is
/// in it). The perturbation experiment (`authority.inversion_*`) is what
/// needs the override: it deletes the junior corridors and re-solves, and
/// holding the skeleton at the full scene's values is exactly the statement
/// "senior heights are a function of the strata and the plan skeleton, and of
/// nothing the deleted juniors *solved*". All three limbs must be pinned —
/// the burial ceilings read `covered`, the reconciliation write-back walks
/// `reaches`, and the short-span demotion honours `over` — because each is a
/// place a junior's plan existence classifies a senior's spans, and a span
/// classification feeds the senior's own solve. Measured on the Lausanne m2
/// twins: unpinned `over` demoted their 26 m station tunnel once the junior
/// streets crossing it were deleted, the burial ceiling then had no tunnel
/// node to cap, and the pair read 11.46 m shallower — an "authority
/// violation" that was really the experiment deleting part of the input.
pub fn run_licensed(
    scene: &mut SceneGraph,
    terrain_path: Option<&Path>,
    z_ref: u8,
    threads: usize,
    licenses: Option<PlanPin>,
) -> Result<SolvedModel, Error> {
    let Some(path) = terrain_path else {
        // No DEM: no terrain test — every short span demotes, so a flat run
        // never bakes tiny decks floating over its flat ground.
        for c in &mut scene.corridors {
            demote_short_spans(&mut c.spans, &mut |_, _| true);
        }
        return Ok(SolvedModel::empty(z_ref));
    };
    let (pin_over, pin_covered, pin_reaches, pin_carried, pin_lateral) = match licenses {
        Some(p) => {
            (Some(p.over), Some(p.covered), Some(p.reaches), Some(p.carried), Some(p.lateral))
        }
        None => (None, None, None, None, None),
    };
    // One primary DEM handle; the reconcile pass and every solve worker fork
    // it to share the decoded-tile cache.
    let primary_dem = Dem::open(path)?;
    {
        let mut dem = primary_dem.fork()?;
        // S20 first, so the demotion test below re-validates every promoted
        // span the same way it judges an annotated one. Idempotent on a
        // re-entrant scene (the perturbation experiment, the determinism
        // check): a promoted interval no longer sits inside a Grade span.
        promote_notch_crossings(scene, &mut |c: Coord| {
            reference_surface(&mut dem, z_ref, c.x, c.y)
        });
        reconcile_short_spans(
            scene,
            &mut |c: Coord| reference_surface(&mut dem, z_ref, c.x, c.y),
            pin_over.as_deref(),
        );
    }

    // Spans are settled until each stratum's own write-back
    // (`reconcile_stratum`): the workers and every read inside the loop see
    // either the annotation (their own stratum, not yet solved) or a senior's
    // reconciled truth (already written back).
    // The `let scene = &*scene_mut` reborrow is deliberate and not a swap:
    // the per-stratum loop below needs the scene mutable for its write-back and
    // immutable for every read, so the mutable handle is renamed once here and
    // an immutable view taken from it. Written as two same-named `let`s it
    // tripped clippy's `almost_swapped`, which is deny-by-default and so failed
    // the whole lint run.
    let scene_mut = scene;
    let scene: &SceneGraph = &*scene_mut;
    // **The partition** (§4.4): one solver, run over the strata in authority
    // order. Each stratum fuses its own corridors into one graph — junction
    // connectors are shared height variables, so continuity (I2) holds by
    // construction — and reads every senior stratum as a *constant*: a
    // published height with no variable of its own, which is the mechanical
    // statement of authority and the whole of I7.
    //
    // Crossings are derived per stratum, from the solved profiles, and handed
    // straight to the graph (§4.5). Nothing can mutate the model between
    // deriving them and consuming them, because there is nowhere for them to
    // wait.
    let mut crossings: Vec<crate::scene::Crossing> = Vec::new();
    let mut junction_h: Vec<Option<f64>> = vec![None; scene.junctions.len()];
    let mut relaxed = relax::Relaxed::default();
    let mut residuals: Vec<relax::PassResidual> = Vec::new();
    // The plan facts, once: arcs and identities only, no heights (§4.1 — a
    // junior's warm start is not a fact, so nothing height-bearing may cross a
    // stratum boundary here). `covered` is the burial license the bore
    // ceilings need: where an at-grade band crosses a mapped tunnel span, the
    // bore must actually pass beneath the ground that band rides on.
    let plan = crossings::plan_index(scene);
    let covered = pin_covered.unwrap_or_else(|| crossings::covered_bores(scene, &plan));
    // The parallel-overlap half of the license (S21): windows where a
    // higher-ordinal alignment runs lengthwise above a bore, each carrying
    // the covering side's own raw surface — a street downhill of the bore
    // line is the case the own-terrain ceiling cannot see.
    let lateral = match pin_lateral {
        Some(l) => l,
        None => {
            let mut dem = primary_dem.fork()?;
            crossings::lateral_cover(scene, &mut |c: Coord| {
                reference_surface(&mut dem, z_ref, c.x, c.y)
            })
        }
    };
    let reaches =
        pin_reaches.unwrap_or_else(|| plan.iter().map(|l| crossings::reaches(l)).collect());
    // The carrying license, the mirror of `covered`: where a mapped deck
    // crosses an alignment annotated beneath it, and so must span its band.
    let carried = pin_carried.unwrap_or_else(|| crossings::carried_crossings(scene, &plan));
    // Every stratum with members, in authority order — including D. A draped
    // feature has no business in the scene at all (§4.2), and after M2 none
    // is, except the railway `paves_today` still admits as a street. Skipping
    // D would leave those corridors with a per-corridor profile and no graph:
    // no shared junction variable, no clearance, no relax. Measured, that put
    // a 2.40 m step where two railways meet, one classed `unknown` and one
    // `standard_gauge`. Solving D last — junior to everything, reading R and S
    // as constants — is both correct and what M6 will inherit.
    // The covered-bore sites, kept alongside the windows: the write-back
    // measures the crossing premise at exactly the sites the ceilings were
    // seeded from (`structure.bore_daylight`).
    let sites = crossings::covered_sites(scene, &plan);
    let mut daylight: Vec<Daylight> = Vec::new();
    let mut partition_div: Vec<PartitionDivergence> = Vec::new();
    // **The two-pass solve** (`data/plans/pure-partition-2026-08-28.md` §4,
    // step 2): under `ARPT_TWO_PASS=1` the whole solve — per-corridor
    // profiles, then the strata in authority order with their write-back —
    // runs a second time, with the spans pass 1 reconciled standing as the
    // priors the annotation stood as, and the licenses unchanged (plan facts,
    // computed once above). The fold still writes back after pass 2, so this
    // step changes nothing where pass 1's partition was already the truth:
    // the residual rows and the scorecard must read the same, which is the
    // fixpoint claim (99b66e1) verified per run before the write-back moves.
    let mut profiles: Vec<Option<Profile>> = Vec::new();
    let passes = if std::env::var_os("ARPT_TWO_PASS").is_some() { 2 } else { 1 };
    for pass in 0..passes {
        let scene: &SceneGraph = &*scene_mut;
        if pass > 0 {
            crossings.clear();
            junction_h.iter_mut().for_each(|h| *h = None);
            relaxed = relax::Relaxed::default();
            residuals.clear();
            daylight.clear();
            partition_div.clear();
        }
    // Every corridor in the scene is solved. The gate upstream admits only
    // strata that solve (`assemble::run`), so "does this need a profile" is no
    // longer a question asked here — a draped feature never reaches this point.
    let todo: Vec<usize> = (0..scene.corridors.len()).collect();
    // Pass 1 only: the per-corridor solve. Pass 2 keeps pass 1's profiles —
    // the ramps the fold's annex/absorb/degrade left behind — because spans
    // alone cannot seed a fixpoint: re-solving from them refits deck ramps,
    // their approaches hang, and absorb/grow extend the spans again
    // (grade→bridge 2,571 m of 3,250 m divergence, ARPT_TWO_PASS_DIVERGENCE).
    // This is §4's "seeded from pass 1's heights", taken literally.
    if pass == 0 {
        profiles = Vec::new();
        profiles.resize_with(scene.corridors.len(), || None);

        let threads = threads.max(1).min(todo.len().max(1));
        let next = Mutex::new(0usize);
        let results: Mutex<&mut Vec<Option<Profile>>> = Mutex::new(&mut profiles);
        std::thread::scope(|scope| -> Result<(), Error> {
            let mut handles = Vec::with_capacity(threads);
            for _ in 0..threads {
                handles.push(scope.spawn(|| -> Result<(), Error> {
                    let mut dem = primary_dem.fork()?;
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
                        let mode = Mode::for_kind(c.kind);
                        let solved = profile::solve(&c.nodes, &c.spans, mode, &mut |p| {
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
    }

    for stratum in [Stratum::H, Stratum::R, Stratum::S, Stratum::D, Stratum::B] {
        // Fresh immutable view per stratum: the write-back below needs the
        // scene mutable, and each iteration's reads must see the seniors'
        // reconciled truth, not the annotation they were assembled with.
        let scene: &SceneGraph = scene_mut;
        if !scene.corridors.iter().any(|c| c.kind.stratum() == stratum) {
            continue;
        }
        let derived = crossings::derive(scene, &profiles, stratum);
        let mut g = graph::build(scene, &profiles, &derived, stratum, &covered, &lateral);
        let r = relax::solve(&mut g);
        relax::reconstruct(&g, &mut profiles);
        // Each stratum publishes the junction heights it owns; a junction
        // belongs to exactly one, so the slots never contend.
        for (ji, h) in relax::junction_heights(&g).into_iter().enumerate() {
            if h.is_some() {
                junction_h[ji] = h;
            }
        }
        relaxed.sweeps = relaxed.sweeps.max(r.sweeps);
        relaxed.demands_dropped += r.demands_dropped;
        relaxed.worst_dropped_m = relaxed.worst_dropped_m.max(r.worst_dropped_m);
        // Which constraints actually hold at this stratum's output — measured
        // after the heights were read back, so a pass that perturbs ceilings
        // while measuring can no longer influence anything.
        for pr in relax::residuals(&mut g) {
            match residuals.iter_mut().find(|p: &&mut relax::PassResidual| p.name == pr.name) {
                Some(p) => p.dist.merge(&pr.dist),
                None => residuals.push(pr),
            }
        }
        crossings.extend(derived);
        // **One truth per stratum** (§4.5): the annotation served as the
        // solve's prior; what survives it is the *reconciled* partition —
        // tunnels grown through the crossings their buried tails pass beneath
        // (annex), then clamped to their buried runs, the freed slack
        // re-covered as grade — written back before any junior stratum or any
        // consumer reads the spans. A junior deciding "the senior is in a
        // bore here" (`graph::in_immovable_bore`) then reads a bore that
        // exists, and the bands, benches, sheets, paint and solids all cut
        // one partition. The split this closes: paint reconciled privately at
        // emit while the surfaces read the annotation, so a dismissed tunnel
        // was stroked as a road over ground that never benched it.
        reconcile_stratum(
            scene_mut,
            &mut profiles,
            stratum,
            &reaches,
            &carried,
            &covered,
            &sites,
            &mut daylight,
            &mut partition_div,
            pass,
        );
    }
    }

    // The structures the result implies, derived once the heights are final.
    let structures = scene_mut
        .corridors
        .iter()
        .map(|c| match profiles.get(c.id as usize).and_then(|p| p.as_ref()) {
            Some(p) => structures::derive(p, c.kind.prior()),
            None => Vec::new(),
        })
        .collect();

    Ok(SolvedModel {
        structures,
        relaxed,
        residuals,
        crossings,
        daylight,
        partition: partition_div,
        profiles,
        junction_h,
        z_ref,
    })
}

/// Sampling step for the notch-crossing detector, metres. Fine enough that a
/// slot narrower than the gap between two mapped nodes cannot hide between
/// samples; the forked DEM's tile cache makes the extra reads cheap.
const NOTCH_SAMPLE_STEP_M: f64 = 4.0;

/// The terrain fate of a *mapped-at-grade* run over a slot the conditioning
/// refuses (S20). The anchor surface keeps every notch deeper than the fill
/// cap (`profile::refused_notches`); where a corridor's Grade span crosses
/// one transversely — in and out within a single notch span, both rims
/// inside the same at-grade run — the level annotation and the DEM
/// contradict each other, and "level across a 16 m slot" is a structure,
/// not a bed. The crossing is spliced into the span partition as a bridge
/// span, entering the solve exactly as a mapped `is_bridge` would: a prior
/// on the constraint (§4.5), which [`reconcile_short_spans`]' own terrain
/// test re-validates — a false positive over ground the closing merely
/// mis-read demotes straight back.
///
/// Left alone deliberately: an interval not wholly inside one Grade span
/// (the mapped annotation already claims the slot, or the notch is a
/// structure's own approach); an interval reaching a corridor end (a
/// cul-de-sac at a gorge rim ends at a retaining face — there is no far rim
/// to bridge to); and every Stratum::H corridor (water *descends through*
/// the notch — that is the one class for which the dive is the truth).
///
/// The measured need is the Chauderon slot at Route de Chernex (S20): the
/// rim streets' at-grade bands dammed a 16 m gorge with 15.8 m kerb cliffs
/// (`contact.kerb_lip` 25.7 % over in that tile) and stood drawn asphalt
/// over the gorge-floor footbridge (`order.deck_above_carriageway` 25.6 %).
fn promote_notch_crossings(scene: &mut SceneGraph, sample: &mut impl FnMut(Coord) -> f64) {
    for c in &mut scene.corridors {
        if c.nodes.len() < 2 || c.kind.stratum() == Stratum::H {
            continue;
        }
        let total = *c.arc.last().expect("a corridor has arc entries");
        if !(total > 0.0) {
            continue;
        }
        // A densified terrain profile: mapped nodes are sparse enough to
        // straddle a whole slot, so the detector may not run on `c.arc` alone.
        let steps = ((total / NOTCH_SAMPLE_STEP_M).ceil() as usize).max(1);
        let mut sarc = Vec::with_capacity(steps + 1);
        let mut sh = Vec::with_capacity(steps + 1);
        let mut seg = 0usize;
        for k in 0..=steps {
            let a = total * k as f64 / steps as f64;
            while seg + 2 < c.arc.len() && c.arc[seg + 1] < a {
                seg += 1;
            }
            let (a0, a1) = (c.arc[seg], c.arc[seg + 1]);
            let t = if a1 > a0 { ((a - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
            let (p0, p1) = (c.nodes[seg], c.nodes[seg + 1]);
            sarc.push(a);
            sh.push(sample(Coord { x: p0.x + (p1.x - p0.x) * t, y: p0.y + (p1.y - p0.y) * t }));
        }
        for (n0, n1) in profile::refused_notches(&sarc, &sh) {
            // One sampling step of margin each side lands the span edge on
            // the rim rather than on the last lifted sample.
            let (a0, a1) = (n0 - NOTCH_SAMPLE_STEP_M, n1 + NOTCH_SAMPLE_STEP_M);
            // Strictly interior to one Grade span (see the doc for why).
            let Some(i) = c
                .spans
                .iter()
                .position(|s| s.kind == SpanKind::Grade && s.arc0 < a0 && a1 < s.arc1)
            else {
                continue;
            };
            let host = c.spans[i];
            c.spans[i] = Span { arc0: host.arc0, arc1: a0, level: host.level, kind: SpanKind::Grade };
            c.spans.insert(i + 1, Span { arc0: a0, arc1: a1, level: 1, kind: SpanKind::Bridge });
            c.spans
                .insert(i + 2, Span { arc0: a1, arc1: host.arc1, level: host.level, kind: SpanKind::Grade });
        }
    }
}

/// The terrain fate of the short structure spans assemble keeps
/// (sub-[`MIN_STRUCTURE_M`], `assemble::corridors::resolve_spans`): a short
/// bridge stays a deck only where the ground genuinely falls away beneath it
/// — its mid-span terrain more than [`SHORT_STRUCTURE_DIP_M`] below the
/// span's end-to-end chord — and a short tunnel only where the ground rises
/// over it. Everything else demotes to grade: on near-flat ground the drape
/// (with the notch closing) carries the road, and a tiny baked deck would
/// float over the hill. The deep-gully case is why the test exists at all: a
/// 25 m annotated bridge over a 30 m stream cut, demoted blindly, dived
/// through the gorge and dragged its earthworks with it.
///
/// A span that passes over or under another mapped alignment is exempt, whether
/// or not the ground moves under it ([`crossings::spans_over_a_mapped_line`]): what
/// makes it a structure is the carriageway, path or watercourse beneath, and
/// the annotation is the only thing in the data that says which of the two is
/// on top. Demoting it hands that ordering to the derivation, which reads it
/// off metre-scale differences between solved surfaces — so one alignment ends
/// up crossing over some roads and under others. The alignments include the
/// scene's witness lines (`SceneGraph::witnesses`): most short annotated
/// bridges span a mapped stream or footpath whose metre-wide cut no DEM
/// resolves, and those lines are the only evidence left.
fn reconcile_short_spans(
    scene: &mut SceneGraph,
    sample: &mut impl FnMut(Coord) -> f64,
    // The crossing exemptions, pinned by the perturbation experiment: they
    // are a limb of the plan skeleton (I7), and recomputing them with the
    // juniors deleted demotes exactly the short senior structures the juniors
    // justify. `None` derives them from this scene.
    over_pin: Option<&[Vec<bool>]>,
) {
    // Against the whole scene, before any corridor is mutated.
    let over_own;
    let over: &[Vec<bool>] = match over_pin {
        Some(o) => o,
        None => {
            over_own = crossings::spans_over_a_mapped_line(scene);
            &over_own
        }
    };
    for (ci, c) in scene.corridors.iter_mut().enumerate() {
        let (nodes, arc) = (std::mem::take(&mut c.nodes), std::mem::take(&mut c.arc));
        let over = &over[ci];
        demote_short_spans(&mut c.spans, &mut |i: usize, span: &Span| {
            !over.get(i).copied().unwrap_or(false) && !spans_a_gap(&nodes, &arc, span, sample)
        });
        (c.nodes, c.arc) = (nodes, arc);
    }
}

/// Demotes every sub-[`MIN_STRUCTURE_M`] structure span for which `demote`
/// says so, then coalesces the adjacent same-kind spans the demotions leave.
fn demote_short_spans(spans: &mut Vec<Span>, demote: &mut impl FnMut(usize, &Span) -> bool) {
    let mut changed = false;
    for (i, s) in spans.iter_mut().enumerate() {
        if s.kind != SpanKind::Grade && s.arc1 - s.arc0 < MIN_STRUCTURE_M && demote(i, s) {
            s.kind = SpanKind::Grade;
            s.level = 0;
            changed = true;
        }
    }
    if changed {
        spans.dedup_by(|cur, prev| {
            if prev.kind == cur.kind && prev.level == cur.level {
                prev.arc1 = cur.arc1;
                true
            } else {
                false
            }
        });
    }
}

/// Whether the terrain departs from the span's end-to-end chord — dips below
/// it for a bridge, rises above it for a tunnel — by more than
/// [`SHORT_STRUCTURE_DIP_M`] anywhere across the span's interior quarters.
fn spans_a_gap(
    nodes: &[Coord],
    arc: &[f64],
    span: &Span,
    sample: &mut impl FnMut(Coord) -> f64,
) -> bool {
    let h0 = sample(point_at_arc(nodes, arc, span.arc0));
    let h1 = sample(point_at_arc(nodes, arc, span.arc1));
    (1..=3).any(|k| {
        let t = k as f64 / 4.0;
        let chord = h0 + (h1 - h0) * t;
        let ground = sample(point_at_arc(nodes, arc, span.arc0 + (span.arc1 - span.arc0) * t));
        let depart = match span.kind {
            SpanKind::Bridge => chord - ground,
            SpanKind::Tunnel => ground - chord,
            SpanKind::Grade => 0.0,
        };
        depart > SHORT_STRUCTURE_DIP_M
    })
}

/// The corridor centerline point at arc `s`, linearly interpolated between
/// the bracketing nodes (clamped to the ends).
fn point_at_arc(nodes: &[Coord], arc: &[f64], s: f64) -> Coord {
    let i = arc.partition_point(|&a| a < s).clamp(1, arc.len() - 1);
    let (a0, a1) = (arc[i - 1], arc[i]);
    let t = if a1 > a0 { ((s - a0) / (a1 - a0)).clamp(0.0, 1.0) } else { 0.0 };
    Coord {
        x: nodes[i - 1].x + (nodes[i].x - nodes[i - 1].x) * t,
        y: nodes[i - 1].y + (nodes[i].y - nodes[i - 1].y) * t,
    }
}

/// The rendered-ground height at `(lon, lat)` on the global zoom-`z` lattice —
/// the same surface [`terrain::surface_height`] gives an emit worker meshing
/// the containing tile, so solved anchors sit exactly on the drawn ground.
/// Only ever called at the reference zoom, so the lattice is the detail grid
/// (`grid_for(z, z)`) — the resolution the z_ref mesh actually renders at;
/// anchors, bed targets, and water levels must all read that same surface.
pub fn reference_surface(dem: &mut Dem, z: u8, lon: f64, lat: f64) -> f64 {
    let b = tile_containing(z, lon, lat);
    let grid = terrain::grid_for(z, z);
    terrain::surface_height(&b, grid, lon, lat, &mut |a, o| dem.elevation(a, o, z))
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

    /// A short annotated bridge keeps its deck over a real gully and demotes
    /// on near-flat ground — the terrain test the assemble stage defers here.
    #[test]
    fn short_spans_resolve_against_the_terrain() {
        use crate::priors::{Kind, RoadClass};
        use crate::scene::{Corridor, SegmentRef, DEG_M};
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 200.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        let nodes: Vec<Coord> =
            (0..n).map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 }).collect();
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let corridor = |spans: Vec<Span>| Corridor {
            id: 0,
            nodes: nodes.clone(),
            arc: arc.clone(),
            cos_lat,
            kind: Kind::Road(RoadClass::Residential),
            class_key: String::new(),
            link: false,
            width_m: Some(5.5),
            spans,
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        };
        let short_bridge = || {
            vec![
                Span { arc0: 0.0, arc1: 90.0, level: 0, kind: SpanKind::Grade },
                Span { arc0: 90.0, arc1: 115.0, level: 1, kind: SpanKind::Bridge },
                Span { arc0: 115.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
            ]
        };
        // A 30 m-deep gully under the 25 m span: the deck survives.
        let gully = |c: Coord| {
            let x = (c.x - 6.0) / deg * len_m;
            500.0 - (1.0 - ((x - 102.5) / 15.0).abs()).max(0.0) * 30.0
        };
        let mut scene = SceneGraph::new(vec![corridor(short_bridge())]);
        reconcile_short_spans(&mut scene, &mut |c| gully(c), None);
        let kinds: Vec<SpanKind> = scene.corridors[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(kinds, vec![SpanKind::Grade, SpanKind::Bridge, SpanKind::Grade]);

        // Flat ground: the footbridge annotation demotes, spans coalesce to one.
        let mut scene = SceneGraph::new(vec![corridor(short_bridge())]);
        reconcile_short_spans(&mut scene, &mut |_| 500.0, None);
        let spans = &scene.corridors[0].spans;
        assert_eq!(spans.len(), 1, "demoted spans must coalesce, got {spans:?}");
        assert_eq!(spans[0].kind, SpanKind::Grade);
        assert_eq!((spans[0].arc0, spans[0].arc1), (0.0, 200.0));

        // A long bridge faces no test: it stays whatever the ground does.
        let long = vec![
            Span { arc0: 0.0, arc1: 80.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 80.0, arc1: 160.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 160.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene = SceneGraph::new(vec![corridor(long)]);
        reconcile_short_spans(&mut scene, &mut |_| 500.0, None);
        assert!(scene.corridors[0].spans.iter().any(|s| s.kind == SpanKind::Bridge));
    }

    use crate::priors::{Kind, RailClass, RoadClass, WaterClass};
    use crate::scene::{Corridor, SegmentRef, DEG_M};

    /// A 200 m corridor along a parallel of latitude, `spans` as given.
    fn test_corridor(kind: Kind, spans: Vec<Span>) -> Corridor {
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 200.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 41;
        Corridor {
            id: 0,
            nodes: (0..n)
                .map(|i| Coord { x: 6.0 + deg * i as f64 / (n - 1) as f64, y: 46.0 })
                .collect(),
            arc: (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect(),
            cos_lat,
            kind,
            class_key: String::new(),
            link: false,
            width_m: Some(5.5),
            spans,
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        }
    }

    /// A V-slot `depth_m` deep and 30 m wide at `at_m` along the test
    /// corridor — narrower than `NOTCH_SPAN_M`, so past the fill cap it is a
    /// *refused* notch (S20).
    fn slot(at_m: f64, depth_m: f64) -> impl Fn(Coord) -> f64 {
        let cos_lat = 46.0_f64.to_radians().cos();
        let deg = 200.0 / (DEG_M * cos_lat);
        move |c: Coord| {
            let x = (c.x - 6.0) / deg * 200.0;
            500.0 - depth_m * (1.0 - ((x - at_m) / 15.0).abs()).max(0.0)
        }
    }

    /// The S20 case: a street mapped level across a slot the fill cap
    /// refuses earns a bridge span over it, and the demotion test that runs
    /// next keeps the deck the same way it would keep an annotated one.
    #[test]
    fn a_grade_run_across_a_refused_slot_earns_a_bridge_span() {
        let all_grade = vec![Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade }];
        let mut scene =
            SceneGraph::new(vec![test_corridor(Kind::Road(RoadClass::Residential), all_grade)]);
        let gorge = slot(100.0, 20.0);
        promote_notch_crossings(&mut scene, &mut |c| gorge(c));
        let kinds: Vec<SpanKind> = scene.corridors[0].spans.iter().map(|s| s.kind).collect();
        assert_eq!(
            kinds,
            vec![SpanKind::Grade, SpanKind::Bridge, SpanKind::Grade],
            "a refused slot under a mapped-level run is a crossing, got {:?}",
            scene.corridors[0].spans
        );
        let deck = scene.corridors[0].spans[1];
        assert_eq!(deck.level, 1);
        assert!(
            deck.arc0 > 70.0 && deck.arc1 < 130.0 && deck.arc1 - deck.arc0 > 15.0,
            "the span brackets the slot, not the corridor: {deck:?}"
        );
        // The demotion pass re-validates rather than undoes the promotion.
        let gorge = slot(100.0, 20.0);
        reconcile_short_spans(&mut scene, &mut |c| gorge(c), None);
        assert!(scene.corridors[0].spans.iter().any(|s| s.kind == SpanKind::Bridge));
        // Idempotent: a promoted interval no longer sits inside a Grade span.
        let gorge = slot(100.0, 20.0);
        promote_notch_crossings(&mut scene, &mut |c| gorge(c));
        assert_eq!(scene.corridors[0].spans.len(), 3, "re-promotion must not re-splice");
    }

    /// The boundary cases that must NOT promote: a slot the closing fills
    /// (shallower than the cap), a slot at the corridor end (no far rim),
    /// a slot an annotated span already claims, and a watercourse (which
    /// genuinely descends through its own notch).
    #[test]
    fn a_slot_promotes_only_interior_grade_crossings() {
        let all_grade = || vec![Span { arc0: 0.0, arc1: 200.0, level: 0, kind: SpanKind::Grade }];
        // Shallow: the conditioning fills it; the road rides the fill (S9).
        let mut scene =
            SceneGraph::new(vec![test_corridor(Kind::Road(RoadClass::Residential), all_grade())]);
        let dip = slot(100.0, 8.0);
        promote_notch_crossings(&mut scene, &mut |c| dip(c));
        assert_eq!(scene.corridors[0].spans.len(), 1, "a filled notch is not a crossing");
        // At the corridor start: no far rim to bridge to.
        let mut scene =
            SceneGraph::new(vec![test_corridor(Kind::Road(RoadClass::Residential), all_grade())]);
        let edge = slot(10.0, 20.0);
        promote_notch_crossings(&mut scene, &mut |c| edge(c));
        assert_eq!(scene.corridors[0].spans.len(), 1, "an end slot keeps its face");
        // Already annotated: the mapped span claims the slot, nothing to add.
        let annotated = vec![
            Span { arc0: 0.0, arc1: 85.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 85.0, arc1: 118.0, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 118.0, arc1: 200.0, level: 0, kind: SpanKind::Grade },
        ];
        let mut scene =
            SceneGraph::new(vec![test_corridor(Kind::Rail(RailClass::NarrowGauge), annotated.clone())]);
        let gorge = slot(100.0, 20.0);
        promote_notch_crossings(&mut scene, &mut |c| gorge(c));
        assert_eq!(scene.corridors[0].spans, annotated, "annotation wins where present");
        // A watercourse descends through the notch: that dive is the truth.
        let mut scene =
            SceneGraph::new(vec![test_corridor(Kind::Water(WaterClass::Watercourse), all_grade())]);
        let gorge = slot(100.0, 20.0);
        promote_notch_crossings(&mut scene, &mut |c| gorge(c));
        assert_eq!(scene.corridors[0].spans.len(), 1, "water is never bridged over its own bed");
    }

    /// S8 for rail twins: a line at grade over the hill its twin is bored
    /// under adopts the bore; a parallel line a street away does not. The
    /// Chamby double track is the type specimen — one formation, two
    /// corridors, and only one of them annotated through the hill.
    #[test]
    fn a_twin_adopts_its_siblings_bore_and_a_neighbour_does_not() {
        use crate::priors::{Kind, RailClass};
        use crate::scene::{Corridor, SegmentRef, DEG_M};
        let cos_lat = 46.0_f64.to_radians().cos();
        let len_m = 1000.0;
        let deg = len_m / (DEG_M * cos_lat);
        let n = 201;
        let line = |off_m: f64| -> Vec<Coord> {
            (0..n)
                .map(|i| Coord {
                    x: 6.0 + deg * i as f64 / (n - 1) as f64,
                    y: 46.0 + off_m / DEG_M,
                })
                .collect()
        };
        let arc: Vec<f64> = (0..n).map(|i| len_m * i as f64 / (n - 1) as f64).collect();
        let corridor = |id: u32, nodes: &Vec<Coord>, spans: Vec<Span>| Corridor {
            id,
            nodes: nodes.clone(),
            arc: arc.clone(),
            cos_lat,
            kind: Kind::Rail(RailClass::NarrowGauge),
            class_key: "narrow_gauge".to_string(),
            link: false,
            width_m: Some(3.0),
            spans,
            segments: vec![SegmentRef { source: 1, node0: 0, node1: n - 1, properties: vec![] }],
            connectors: vec![],
        };
        // Road flat at 100 under a hill to 130 over the middle third — the
        // portals fixture's shape, so the tunnel's fitted interval is real.
        let profile = |nodes: &[Coord]| {
            let road = vec![100.0; n];
            let terrain: Vec<f64> = (0..n)
                .map(|i| {
                    let u = i as f64 / (n - 1) as f64;
                    let d = (u - 0.5_f64).abs();
                    if d < 0.15 { 130.0 - d / 0.15 * 40.0 } else { 90.0 }
                })
                .collect();
            Profile::from_heights(nodes, road, terrain)
        };
        let grade = vec![Span { arc0: 0.0, arc1: len_m, level: 0, kind: SpanKind::Grade }];
        let tunnel = vec![
            Span { arc0: 0.0, arc1: 400.0, level: 0, kind: SpanKind::Grade },
            Span { arc0: 400.0, arc1: 600.0, level: -1, kind: SpanKind::Tunnel },
            Span { arc0: 600.0, arc1: len_m, level: 0, kind: SpanKind::Grade },
        ];
        let (a, b, far) = (line(0.0), line(4.0), line(30.0));
        let mut scene = SceneGraph::new(vec![
            corridor(0, &a, tunnel),
            corridor(1, &b, grade.clone()),
            corridor(2, &far, grade),
        ]);
        let stratum = scene.corridors[0].kind.stratum();
        let mut profiles = vec![Some(profile(&a)), Some(profile(&b)), Some(profile(&far))];
        let windows = twin_bore_windows(&scene, &profiles, stratum);
        assert!(!windows[1].is_empty(), "the twin must see its sibling's bore");
        assert!(windows[2].is_empty(), "30 m away is not a twin");
        apply_twin_windows(&mut scene, &mut profiles, &windows, false);
        let bored = |c: &Corridor| -> Vec<(f64, f64)> {
            c.spans
                .iter()
                .filter(|s| s.kind == SpanKind::Tunnel)
                .map(|s| (s.arc0, s.arc1))
                .collect()
        };
        let twin = bored(&scene.corridors[1]);
        assert_eq!(twin.len(), 1, "the twin must adopt one bore, got {twin:?}");
        // It covers the sibling's fitted interval, to a node step.
        assert!(twin[0].0 < 420.0 && twin[0].1 > 580.0, "adopted {twin:?}");
        // The spans still tile the corridor end to end.
        let s = &scene.corridors[1].spans;
        assert!((s[0].arc0 - 0.0).abs() < 1e-9);
        assert!((s[s.len() - 1].arc1 - len_m).abs() < 1e-9);
        for w in s.windows(2) {
            assert!((w[1].arc0 - w[0].arc1).abs() < 1e-9, "gap between {:?} and {:?}", w[0], w[1]);
        }
        assert!(bored(&scene.corridors[2]).is_empty(), "30 m away is not a twin");
    }
}
