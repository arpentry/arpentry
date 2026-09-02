//! The junction-joint span weld: at a junction two corridors share, the span
//! truth is joint (docs/GENERATION.md §4.5's sentence, one junction at a time).
//!
//! Reconciliation is corridor-local, and `partition.junction_joint` measures
//! what that cannot reach: a Bridge span ending metres short of a junction
//! whose one aligned continuation carries the structure onward — the mapped
//! edge of a physical viaduct that happens to be split across corridors the
//! splice refused to join. The band stops at its corridor's end, the deck
//! starts past the junction, and `seam.band_deck_bare` draws the ground
//! between them.
//!
//! The weld is deliberately `carry_stubs_welded_onto_decks`' shape, because
//! that is the one cross-corridor mutation that survived measurement:
//!
//! - **Grow-only, toward a fixed target.** A span end within [`JOINT_REACH_M`]
//!   of the junction arc grows *to* the junction arc — a plan fact — eating
//!   only Grade ([`super::portals::grow_high`]); nothing ever shrinks and no
//!   `degrade_structure` exists here, so the operator is monotone and any
//!   visit order reaches the same least fixpoint. This is what the two-pass
//!   experiment lacked: the weld reads no height it writes.
//! - **The profile moves with the partition.** Every grow is paired with
//!   [`crate::solve::Profile::annex_structure`] over the grown range, so the
//!   deck ramp, the handover cut and the sweep move together — the exact
//!   coupling whose absence sank the withdrawn bridge trim
//!   (`solve::partition`'s module doc).
//! - **Authority holds.** Only the solving stratum's spans and profiles are
//!   written. A continuation is judged by tangents — the plan skeleton, input
//!   data (I7) — but its *height* is read only from a profile the solve has
//!   already produced (its own stratum, or a senior's published one), and a
//!   junior member can license nothing.
//!
//! What declines, and why, in gate order: two or more aligned continuations
//! (a fork — welding the wrong leg draws a deck across a side street, so
//! conservatism wins and the census counts the site); a continuation more
//! than [`JOINT_LEVEL_M`] away in height (the relax's junction weld already
//! pulled genuine joints together, so a residual step is a data error, not a
//! seam); and a gap whose ground genuinely comes back up over the deck —
//! unless the flank probe says that "ground" is the DEM's own rasterised
//! image of the structure ([`super::partition::dem_blind`]), which is exactly
//! the abutment-fill case the probe was built for.
//!
//! Opt-in under `ARPT_JOINT_WELD` until the scorecard blesses it.

use geo_types::Coord;

use crate::priors::Stratum;
use crate::scene::{CorridorId, SceneGraph, SpanKind};
use crate::solve::{profile, Profile};

use super::{partition, portals};

/// Mirror of `verify::model::joints::JOINT_REACH_M` — the census's population
/// is the weld's, or the gate is judged against sites the weld never saw.
const JOINT_REACH_M: f64 = 12.0;

/// A span already at the junction within this has nothing to weld.
const AT_M: f64 = 0.5;

/// Two tangents whose |cosine| clears this continue one another — the
/// boundary `assemble::corridors::continues_through` draws for the splice.
const CONTINUES_DOT: f64 = 0.5;

/// The two sides of a joint may disagree by this much in solved height and
/// still be one structure: the relax's junction variable has already welded
/// genuine joints, so this absorbs profile-node quantization, not a step.
const JOINT_LEVEL_M: f64 = 1.0;

/// One eligible grow: `corridor`'s Bridge span ending at `span_end` grows to
/// `target` (the junction arc), `high` when the growth is arc-forward.
struct JointGrow {
    corridor: CorridorId,
    span_end: f64,
    target: f64,
    high: bool,
}

/// Grows every eligible span to its junction, with the profile mutation each
/// grow owes. Call after the per-corridor reconciliation has written every
/// span back, and before the carry-stub weld, which can only become *more*
/// eligible from a grow, never less.
pub fn weld_junction_joints(
    scene: &mut SceneGraph,
    profiles: &mut [Option<Profile>],
    stratum: Stratum,
    flank: &mut dyn FnMut(Coord) -> f64,
    debug: bool,
) {
    let grows = joint_grows(scene, profiles, stratum);
    let mut welded = 0usize;
    let mut ground_declined = 0usize;
    for g in &grows {
        let Some(p) = profiles.get_mut(g.corridor as usize).and_then(|s| s.as_mut()) else {
            continue;
        };
        let (lo, hi) =
            if g.high { (g.span_end, g.target) } else { (g.target, g.span_end) };
        // The gap must be ground the deck clears — the line criterion the
        // shrink uses — or ground the flank probe convicts as the DEM's own
        // image of the structure. Growing a deck into a hillside that
        // genuinely comes back up is the Territet failure, and the census
        // keeps the declined site visible.
        let (road, terrain, arc) = (p.road_m(), p.terrain_m(), p.arc());
        let clears = arc
            .iter()
            .enumerate()
            .filter(|(_, &a)| a >= lo && a <= hi)
            .all(|(k, _)| road[k] >= terrain[k]);
        if !clears && !partition::dem_blind(p, lo, hi, flank) {
            ground_declined += 1;
            if debug {
                let pt = p.point_at_arc(0.5 * (lo + hi));
                eprintln!(
                    "[joint-weld] declined on ground: corridor {} gap [{lo:.1}, {hi:.1}] \
                     at {:.6},{:.6}",
                    g.corridor, pt.x, pt.y
                );
            }
            continue;
        }
        let c = &mut scene.corridors[g.corridor as usize];
        let Some(i) = c.spans.iter().position(|s| {
            s.kind == SpanKind::Bridge
                && (if g.high { (s.arc1 - g.span_end).abs() } else { (s.arc0 - g.span_end).abs() })
                    < 1e-6
        }) else {
            continue; // an earlier grow already moved this end
        };
        let grew = if g.high {
            portals::grow_high(&mut c.spans, i, g.target)
        } else {
            portals::grow_low(&mut c.spans, i, g.target)
        };
        if grew {
            let deck_follows_road = c.kind.prior().monotone
                && profile::monotone_direction(p.terrain_m()).is_some();
            p.annex_structure(lo, hi, deck_follows_road);
            welded += 1;
            if debug {
                let pt = p.point_at_arc(g.target);
                eprintln!(
                    "[joint-weld] corridor {} grew its deck {:.1} m to the junction at \
                     {:.6},{:.6}",
                    g.corridor,
                    hi - lo,
                    pt.x,
                    pt.y
                );
            }
        }
    }
    if debug || std::env::var_os("ARPT_JOINT_WELD_CENSUS").is_some() {
        eprintln!(
            "[joint-weld] {stratum:?}: {} eligible, {welded} welded, {ground_declined} \
             declined on ground",
            grows.len()
        );
    }
}

/// The eligible grows, read-only. Every eligibility fact is a plan fact or an
/// already-solved height, so the list is independent of apply order.
fn joint_grows(
    scene: &SceneGraph,
    profiles: &[Option<Profile>],
    stratum: Stratum,
) -> Vec<JointGrow> {
    let mut out = Vec::new();
    for j in scene.junctions.iter() {
        for member in &j.members {
            let Some(c) = scene.corridors.get(member.corridor as usize) else { continue };
            if c.kind.stratum() != stratum {
                continue;
            }
            let Some(p) = profiles.get(member.corridor as usize).and_then(|s| s.as_ref())
            else {
                continue;
            };
            // The nearest Bridge span end on either side of the member's arc
            // with only Grade between — the same population the census reads.
            let mut best: Option<(f64, f64, bool)> = None; // (gap, span_end, high)
            for s in &c.spans {
                let (gap, end, high) = if s.kind != SpanKind::Bridge {
                    continue;
                } else if s.arc1 <= member.arc {
                    (member.arc - s.arc1, s.arc1, true)
                } else if s.arc0 >= member.arc {
                    (s.arc0 - member.arc, s.arc0, false)
                } else {
                    continue;
                };
                let blocked = c.spans.iter().any(|o| {
                    o.kind != SpanKind::Grade
                        && o.arc1 - o.arc0 > f64::EPSILON
                        && if high {
                            o.arc0 >= end && o.arc1 <= member.arc + AT_M && (o.arc0 - end).abs() > 1e-9
                        } else {
                            o.arc1 <= end && o.arc0 >= member.arc - AT_M && (o.arc1 - end).abs() > 1e-9
                        }
                });
                if blocked {
                    continue;
                }
                if best.is_none_or(|(b, _, _)| gap < b) {
                    best = Some((gap, end, high));
                }
            }
            let Some((gap, span_end, high)) = best else { continue };
            if gap <= AT_M || gap > JOINT_REACH_M {
                continue;
            }
            let Some(ta) = tangent(c, member.arc) else { continue };
            // Exactly one aligned continuation, profiled, of this stratum or
            // a senior one — its solved height a fact the weld may read.
            let mut continuation: Option<f64> = None;
            let mut candidates = 0usize;
            for o in &j.members {
                if o.corridor == member.corridor {
                    continue;
                }
                let Some(oc) = scene.corridors.get(o.corridor as usize) else { continue };
                let Some(tb) = tangent(oc, o.arc) else { continue };
                if (ta.0 * tb.0 + ta.1 * tb.1).abs() <= CONTINUES_DOT {
                    continue;
                }
                candidates += 1;
                if oc.kind.stratum() <= stratum {
                    continuation = profiles
                        .get(o.corridor as usize)
                        .and_then(|s| s.as_ref())
                        .map(|op| op.road_at_arc(o.arc));
                }
            }
            if candidates != 1 {
                continue;
            }
            let Some(other_h) = continuation else { continue };
            if (p.road_at_arc(member.arc) - other_h).abs() > JOINT_LEVEL_M {
                continue;
            }
            out.push(JointGrow {
                corridor: member.corridor,
                span_end,
                target: member.arc,
                high,
            });
        }
    }
    out
}

/// The corridor's unit tangent (metric space) at an arc.
fn tangent(c: &crate::scene::Corridor, at: f64) -> Option<(f64, f64)> {
    if c.nodes.len() < 2 {
        return None;
    }
    let i = c.arc.partition_point(|&a| a < at).clamp(1, c.nodes.len() - 1);
    let (a, b) = (c.nodes[i - 1], c.nodes[i]);
    let (dx, dy) = ((b.x - a.x) * c.cos_lat, b.y - a.y);
    let len = dx.hypot(dy);
    (len > 0.0).then(|| (dx / len, dy / len))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::Kind;
    use crate::scene::{cumulative_arc, Junction, JunctionMember, Span, DEG_M};

    fn line_m(x0: f64, x1: f64, n: usize) -> Vec<Coord> {
        (0..n)
            .map(|i| Coord {
                x: (x0 + (x1 - x0) * i as f64 / (n - 1) as f64) / DEG_M,
                y: 0.0,
            })
            .collect()
    }

    fn corridor(id: CorridorId, x0: f64, x1: f64, spans: Vec<Span>) -> crate::scene::Corridor {
        let nodes = line_m(x0, x1, 11);
        let arc = cumulative_arc(&nodes);
        crate::scene::Corridor {
            id,
            nodes,
            arc,
            cos_lat: 1.0,
            kind: Kind::parse(None, Some("residential"), None),
            class_key: "residential".into(),
            link: false,
            width_m: Some(6.0),
            spans,
            segments: Vec::new(),
            connectors: Vec::new(),
        }
    }

    fn flat_profile(c: &crate::scene::Corridor, road: f64, terrain: f64) -> Profile {
        let n = c.nodes.len();
        Profile::from_heights(&c.nodes, vec![road; n], vec![terrain; n])
    }

    fn span(a0: f64, a1: f64, kind: SpanKind) -> Span {
        Span { arc0: a0, arc1: a1, level: i64::from(kind == SpanKind::Bridge), kind }
    }

    /// A 50 m corridor whose bridge ends 6 m short of the junction at its
    /// end, and a continuation carrying on from there.
    fn scene_with_gap() -> (SceneGraph, Vec<Option<Profile>>) {
        let a = corridor(
            0,
            0.0,
            50.0,
            vec![
                span(0.0, 20.0, SpanKind::Grade),
                span(20.0, 44.0, SpanKind::Bridge),
                span(44.0, 50.0, SpanKind::Grade),
            ],
        );
        let b = corridor(1, 50.0, 100.0, vec![span(0.0, 50.0, SpanKind::Grade)]);
        let pa = flat_profile(&a, 101.0, 100.0); // the deck clears its ground
        let pb = flat_profile(&b, 101.0, 100.0);
        let mut scene = SceneGraph::default();
        scene.corridors = vec![a, b];
        scene.junctions = vec![Junction {
            point: Coord { x: 50.0 / DEG_M, y: 0.0 },
            connector: 7,
            members: vec![
                JunctionMember { corridor: 0, arc: 50.0 },
                JunctionMember { corridor: 1, arc: 0.0 },
            ],
        }];
        (scene, vec![Some(pa), Some(pb)])
    }

    fn no_flank(_: Coord) -> f64 {
        // The flank stands as high as anything: `dem_blind` convicts when the
        // ground *beside* the axis falls a deck-standoff below the road, so a
        // flank at +inf never convicts and the on-axis ground is believed.
        f64::INFINITY
    }

    #[test]
    fn a_deck_short_of_its_junction_grows_to_it() {
        let (mut scene, mut profiles) = scene_with_gap();
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let spans = &scene.corridors[0].spans;
        let bridge = spans.iter().find(|s| s.kind == SpanKind::Bridge).unwrap();
        assert!(
            (bridge.arc1 - 50.0).abs() < 1e-9,
            "the deck grew to the junction, got {}",
            bridge.arc1
        );
    }

    #[test]
    fn the_weld_is_idempotent() {
        let (mut scene, mut profiles) = scene_with_gap();
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let once = scene.corridors[0].spans.clone();
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        assert_eq!(once.len(), scene.corridors[0].spans.len());
        for (a, b) in once.iter().zip(scene.corridors[0].spans.iter()) {
            assert_eq!(a.arc0.to_bits(), b.arc0.to_bits());
            assert_eq!(a.arc1.to_bits(), b.arc1.to_bits());
            assert_eq!(a.kind, b.kind);
        }
    }

    #[test]
    fn a_terminal_deck_end_does_not_grow() {
        let (mut scene, mut profiles) = scene_with_gap();
        scene.junctions[0].members.truncate(1); // no continuation at all
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let bridge =
            scene.corridors[0].spans.iter().find(|s| s.kind == SpanKind::Bridge).unwrap();
        assert!((bridge.arc1 - 44.0).abs() < 1e-9, "a data edge is never welded");
    }

    #[test]
    fn a_fork_with_two_aligned_candidates_welds_neither() {
        let (mut scene, mut profiles) = scene_with_gap();
        let b2 = corridor(2, 50.0, 100.0, vec![span(0.0, 50.0, SpanKind::Grade)]);
        let pb2 = flat_profile(&b2, 101.0, 100.0);
        scene.corridors.push(b2);
        profiles.push(Some(pb2));
        scene.junctions[0].members.push(JunctionMember { corridor: 2, arc: 0.0 });
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let bridge =
            scene.corridors[0].spans.iter().find(|s| s.kind == SpanKind::Bridge).unwrap();
        assert!((bridge.arc1 - 44.0).abs() < 1e-9, "ambiguity is conservatism's case");
    }

    #[test]
    fn a_continuation_a_storey_away_does_not_weld() {
        let (mut scene, mut profiles) = scene_with_gap();
        let b = &scene.corridors[1];
        profiles[1] = Some(flat_profile(b, 95.0, 94.0)); // 6 m below the deck side
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let bridge =
            scene.corridors[0].spans.iter().find(|s| s.kind == SpanKind::Bridge).unwrap();
        assert!((bridge.arc1 - 44.0).abs() < 1e-9, "a height step is a data error, not a seam");
    }

    #[test]
    fn ground_standing_over_the_gap_declines_the_grow() {
        let (mut scene, mut profiles) = scene_with_gap();
        // The terrain climbs over the road across the gap: the deck would be
        // buried, and no flank probe excuses it.
        let c = &scene.corridors[0];
        let n = c.nodes.len();
        let terrain: Vec<f64> = c
            .arc
            .iter()
            .map(|&a| if a > 44.0 { 103.0 } else { 100.0 })
            .collect();
        profiles[0] = Some(Profile::from_heights(&c.nodes, vec![101.0; n], terrain));
        weld_junction_joints(&mut scene, &mut profiles, Stratum::S, &mut no_flank, false);
        let bridge =
            scene.corridors[0].spans.iter().find(|s| s.kind == SpanKind::Bridge).unwrap();
        assert!((bridge.arc1 - 44.0).abs() < 1e-9, "a deck is not grown into a hillside");
    }
}
