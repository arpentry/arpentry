//! The pure partition — `partition(profile, reference, licenses) -> spans` —
//! scaffolded beside the fold it is meant to replace and called only
//! diagnostically (`data/plans/pure-partition-2026-08-28.md` §5 step 1).
//!
//! The fold in [`super::reconcile_stratum`] arrives at a corridor's spans by
//! thirteen mutations spread over the profile and the scene: annex, absorb,
//! grow, shrink, degrade, carry. This module states the same answer as one
//! function of its inputs, composed from the very predicates the fold uses —
//! `annex_spans` for the license arithmetic, `reconcile_spans` for the tunnel
//! half — plus the bridge half the fold never had: a `Bridge` span clamped to
//! the deck run the solved heights actually imply
//! ([`super::structures::derive`], [`super::structures::DECK_STANDOFF_M`]),
//! which is the 2026-08-28 slice that fired correctly and was withdrawn
//! because it moved one consumer ahead of the others. Here it moves none:
//! the result is compared with the fold's and the metres of disagreement are
//! reported as `partition.divergence`, so the two-pass switch starts from a
//! measured distance rather than a sketch.
//!
//! The whole-span guard of that slice is kept (§7): a bridge no derived run
//! overlaps at all keeps its annotation, since a DEM-blind span has no
//! departure to trim to.

use geo_types::Coord;

use crate::priors::Prior;
use crate::scene::{Span, SpanKind};

use super::portals;
use super::profile::Profile;
use super::structures;

/// Shortest grade stub worth keeping, in metres — the fold's own
/// `reconcile_spans` floor, so the two do not differ by quantization.
const MIN_STUB_M: f64 = 0.25;

/// Everything the partition may read besides the profile: plan facts and
/// annotations, computed once, never edited.
pub struct Licenses<'a> {
    /// Burial windows: stretches another feature crosses over, under which
    /// the reference surface is not the ground the tube fits (`covered`).
    pub covered: &'a [(f64, f64)],
    /// A same-formation twin's vetted bore beside this corridor.
    pub twin: &'a [(f64, f64)],
    /// Crossing reaches the annex grows a tunnel through.
    pub reaches: &'a [(f64, f64)],
    /// Windows a senior structure carries this corridor across.
    pub carried: &'a [(f64, f64)],
}

/// The partition of a corridor as a pure function.
pub fn partition(
    profile: &Profile,
    annotated: &[Span],
    lic: &Licenses<'_>,
    prior: &Prior,
    flank: &mut dyn FnMut(Coord) -> f64,
) -> Vec<Span> {
    // License arithmetic: crossing reaches and carried windows as inputs.
    let spans = portals::annex_spans(profile, annotated, lic.reaches, lic.carried)
        .unwrap_or_else(|| annotated.to_vec());
    // The tunnel half, verbatim: grow over absorbed stretches, clamp each
    // bore to its buried run, hold the licensed windows.
    let spans = portals::reconcile_spans(profile, &spans, lic.covered, lic.twin);
    // The bridge half.
    bridge_trim(profile, &spans, prior, flank)
}

/// How far beside the centerline the DEM is asked whether a "grounded"
/// stretch of an annotated bridge is really ground, in metres. Wide enough
/// to step off a viaduct's deck as the DEM rasterised it, narrow enough to
/// stay inside the gorge it spans.
const FLANK_M: f64 = 25.0;

/// Station spacing of the flank probe along a candidate stretch, metres.
const FLANK_STEP_M: f64 = 16.0;

/// Whether a stretch of an annotated bridge that hugs its own terrain is
/// DEM-blind — the DEM carries the deck, so the on-axis gap says "ground"
/// while the ground beside the corridor still falls away. Probed on both
/// flanks at [`FLANK_M`]; a majority of stations with either flank more than
/// the deck standoff below the road says the "ground" is the deck itself
/// (§7's F4 family, measured 2026-08-30 as grade_stack 9.5 → 60.7 % when
/// trimmed without this guard).
pub(crate) fn dem_blind(
    profile: &Profile,
    a0: f64,
    a1: f64,
    flank: &mut dyn FnMut(Coord) -> f64,
) -> bool {
    let ratio = dem_blind_ratio(profile, a0, a1, flank);
    if std::env::var_os("ARPT_BRIDGE_TRIM_CENSUS").is_some() {
        let pt = profile.point_at_arc(0.5 * (a0 + a1));
        eprintln!(
            "[trim-census] ratio {ratio:.2} len {:.0} m at {:.6},{:.6}",
            a1 - a0,
            pt.x,
            pt.y
        );
    }
    2.0 * ratio > 1.0
}

/// The fraction of a stretch's probe stations whose flanks say "deck": the
/// number the calibration reads (`ARPT_BRIDGE_TRIM_CENSUS=1` histograms it
/// per candidate stretch).
fn dem_blind_ratio(
    profile: &Profile,
    a0: f64,
    a1: f64,
    flank: &mut dyn FnMut(Coord) -> f64,
) -> f64 {
    let n = (((a1 - a0) / FLANK_STEP_M).ceil() as usize).clamp(1, 64);
    let mut blind = 0usize;
    for k in 0..n {
        let arc = a0 + (a1 - a0) * (k as f64 + 0.5) / n as f64;
        let p0 = profile.point_at_arc((arc - 2.0).max(a0));
        let p1 = profile.point_at_arc((arc + 2.0).min(a1));
        let pt = profile.point_at_arc(arc);
        let m_lon = crate::scene::DEG_M * pt.y.to_radians().cos();
        let (dx, dy) = ((p1.x - p0.x) * m_lon, (p1.y - p0.y) * crate::scene::DEG_M);
        let len = (dx * dx + dy * dy).sqrt();
        if !(len > 0.0) {
            continue;
        }
        let (nx, ny) = (-dy / len, dx / len);
        let road = profile.road_at_arc(arc);
        let mut side = |sign: f64| {
            flank(Coord {
                x: pt.x + sign * nx * FLANK_M / m_lon,
                y: pt.y + sign * ny * FLANK_M / crate::scene::DEG_M,
            })
        };
        let left = side(1.0);
        let right = side(-1.0);
        let low = left.min(right);
        if road - low > structures::DECK_STANDOFF_M {
            blind += 1;
        }
    }
    blind as f64 / n as f64
}

/// Each `Bridge` span clamped to the extent of the deck runs the solved
/// heights imply inside it, the slack re-covered as grade — except where the
/// slack is DEM-blind ([`dem_blind`]): there the annotation stands, because
/// the "ground" the derive saw is the deck the DEM rasterised. A span no
/// deck run overlaps at all is kept whole (the original whole-span guard).
pub(crate) fn bridge_trim(
    profile: &Profile,
    spans: &[Span],
    prior: &Prior,
    flank: &mut dyn FnMut(Coord) -> f64,
) -> Vec<Span> {
    let runs = structures::derive(profile, prior);
    let mut out = Vec::with_capacity(spans.len() + 4);
    for s in spans {
        if s.kind != SpanKind::Bridge {
            out.push(*s);
            continue;
        }
        let fitted = runs
            .iter()
            .filter(|r| r.kind == SpanKind::Bridge && r.arc1 > s.arc0 && r.arc0 < s.arc1)
            .fold(None, |acc: Option<(f64, f64)>, r| {
                let (a, b) = (r.arc0.max(s.arc0), r.arc1.min(s.arc1));
                Some(acc.map_or((a, b), |(x, y)| (x.min(a), y.max(b))))
            });
        let Some((mut a0, mut a1)) = fitted.filter(|(a0, a1)| a1 - a0 >= MIN_STUB_M) else {
            out.push(*s);
            continue;
        };
        // The slack on either side keeps its annotation where it is DEM-blind:
        // grow the kept extent back over it rather than re-cover deck as grade.
        if a0 - s.arc0 > MIN_STUB_M && dem_blind(profile, s.arc0, a0, flank) {
            a0 = s.arc0;
        }
        if s.arc1 - a1 > MIN_STUB_M && dem_blind(profile, a1, s.arc1, flank) {
            a1 = s.arc1;
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

/// Where two partitions of one corridor disagree: metres of centerline on
/// which they name different kinds, split by what each says, and the arc at
/// the middle of the longest disagreeing stretch.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Divergence {
    pub metres: f64,
    /// Metres the fold calls Bridge and the partition calls Grade — the
    /// bridge-end family the withdrawn slice trimmed.
    pub bridge_to_grade: f64,
    pub grade_to_bridge: f64,
    pub tunnel_to_grade: f64,
    pub grade_to_tunnel: f64,
    pub other: f64,
    pub worst_arc: f64,
    pub worst_metres: f64,
}

fn kind_at(spans: &[Span], arc: f64) -> Option<SpanKind> {
    spans.iter().find(|s| s.arc0 <= arc && arc < s.arc1).map(|s| s.kind)
}

/// The divergence between the fold's spans `fold` and the partition's `pure`.
pub fn divergence(fold: &[Span], pure: &[Span]) -> Divergence {
    let mut cuts: Vec<f64> =
        fold.iter().chain(pure.iter()).flat_map(|s| [s.arc0, s.arc1]).collect();
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let mut d = Divergence::default();
    let mut run: Option<(f64, f64)> = None; // current disagreeing stretch
    for w in cuts.windows(2) {
        let (x0, x1) = (w[0], w[1]);
        let mid = 0.5 * (x0 + x1);
        let (a, b) = (kind_at(fold, mid), kind_at(pure, mid));
        let differs = a != b && a.is_some() && b.is_some();
        if differs {
            let len = x1 - x0;
            d.metres += len;
            match (a, b) {
                (Some(SpanKind::Bridge), Some(SpanKind::Grade)) => d.bridge_to_grade += len,
                (Some(SpanKind::Grade), Some(SpanKind::Bridge)) => d.grade_to_bridge += len,
                (Some(SpanKind::Tunnel), Some(SpanKind::Grade)) => d.tunnel_to_grade += len,
                (Some(SpanKind::Grade), Some(SpanKind::Tunnel)) => d.grade_to_tunnel += len,
                _ => d.other += len,
            }
            run = Some(run.map_or((x0, x1), |(r0, _)| (r0, x1)));
        }
        if !differs || x1 >= *cuts.last().unwrap_or(&x1) {
            if let Some((r0, r1)) = run.take() {
                if r1 - r0 > d.worst_metres {
                    d.worst_metres = r1 - r0;
                    d.worst_arc = 0.5 * (r0 + r1);
                }
            }
        }
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::DEG_M;
    use geo_types::Coord;

    /// A 1 km flat road at 100 m over ground that drops into a gorge 30 m
    /// deep between 40 % and 60 % of the way, with the bridge annotated
    /// generously from 30 % to 70 %.
    fn gorge() -> (Profile, Vec<Span>, f64) {
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
                if (0.4..=0.6).contains(&u) { 70.0 } else { 100.0 }
            })
            .collect();
        let spans = vec![
            Span { arc0: 0.0, arc1: 0.3 * len, level: 0, kind: SpanKind::Grade },
            Span { arc0: 0.3 * len, arc1: 0.7 * len, level: 1, kind: SpanKind::Bridge },
            Span { arc0: 0.7 * len, arc1: len, level: 0, kind: SpanKind::Grade },
        ];
        (Profile::from_heights(&nodes, road, terrain), spans, len)
    }

    #[test]
    fn a_bridge_is_clamped_to_the_deck_the_heights_imply() {
        let (p, spans, len) = gorge();
        let prior = crate::priors::Kind::Road(crate::priors::RoadClass::Residential).prior();
        // The gorge is a gorge beside the line too: the flank probe reads the
        // same field, so the trim's verdict is untouched by the guard.
        let mut flank = |c: Coord| if (0.4..=0.6).contains(&(((c.x - 6.0) * DEG_M * 46.0_f64.to_radians().cos()) / len)) { 70.0 } else { 100.0 };
        let out = bridge_trim(&p, &spans, prior, &mut flank);
        let bridge = out.iter().find(|s| s.kind == SpanKind::Bridge).expect("a bridge");
        // The deck run starts where the gap crosses the standoff — within a
        // node spacing of the gorge lip, not at the mapper's 30 %.
        assert!((bridge.arc0 - 0.4 * len).abs() < 6.0, "arc0 {}", bridge.arc0);
        assert!((bridge.arc1 - 0.6 * len).abs() < 6.0, "arc1 {}", bridge.arc1);
        // Re-covered as grade on both sides, still a partition of the corridor.
        assert_eq!(out.first().unwrap().arc0, 0.0);
        assert!((out.last().unwrap().arc1 - len).abs() < 1e-9);
        for w in out.windows(2) {
            assert!((w[0].arc1 - w[1].arc0).abs() < 1e-9, "gap between spans");
        }
        let d = divergence(&spans, &out);
        assert!((d.bridge_to_grade - 200.0).abs() < 12.0, "{d:?}");
        assert_eq!(d.grade_to_bridge, 0.0);
    }

    #[test]
    fn a_dem_blind_bridge_keeps_its_annotation() {
        let (p, mut spans, len) = gorge();
        // Same annotation over flat ground: no derived run, span kept whole.
        let n = p.arc().len();
        let flat = Profile::from_heights(
            &(0..n)
                .map(|i| p.point_at_arc(len * i as f64 / (n - 1) as f64))
                .collect::<Vec<_>>(),
            vec![100.0; n],
            vec![100.0; n],
        );
        spans[1].kind = SpanKind::Bridge;
        let prior = crate::priors::Kind::Road(crate::priors::RoadClass::Residential).prior();
        let out = bridge_trim(&flat, &spans, prior, &mut |_| 100.0);
        assert_eq!(out, spans);
        assert_eq!(divergence(&spans, &out).metres, 0.0);
    }
}
