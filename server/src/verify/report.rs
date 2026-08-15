//! Turning the scorecard into something to act on: a table to read, JSON to
//! commit, and a diff against the last committed run.
//!
//! The diff is the part that changes how the work feels. A single run answers
//! "is anything wrong", which is rarely in doubt; a diff answers "did what I
//! just did make it better", which is the question every iteration is actually
//! asking and the one a screenshot answers worst. It is also the regression net:
//! a change that fixes the mountain tunnel and quietly breaks the river bridge
//! shows up as one column improving and another regressing, in the same table,
//! without anyone having to remember to go and look at the river.

use serde_json::{json, Value as Json};

use super::{Metric, Offender, Scope, Scorecard, Sense};

/// How a metric moved against a baseline.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Move {
    Improved,
    Regressed,
    /// The distribution held and only the single most extreme sample moved.
    ///
    /// Reported, never gating. `worst` is a maximum over as many as thirteen
    /// million samples, so it is the least stable number on the scorecard: one
    /// new sliver triangle moved `slope.terrain_face` from 201.8 to 349.9 while
    /// the median, the tail and the violation rate were all unchanged to three
    /// decimals. A gate keyed on that fires constantly and gets ignored, which
    /// is worse than no gate. A real per-site catastrophe is what the corpus
    /// scenarios and the offender lists below are for.
    Outlier,
    Same,
    /// In one run but not the other — a check was added, or could not run.
    Absent,
}

/// The statistics describing one side of a comparison, in the order the verdict
/// consults them: how often the defect happens, how bad it is in the bulk of
/// the tail, and the single extreme.
#[derive(Clone, Copy, Default)]
pub struct Stats {
    /// Share of samples on the wrong side of the threshold, in percent.
    pub pct: Option<f64>,
    /// The p99.9 (or p0.1) figure a single outlier cannot dominate.
    pub tail: Option<f64>,
    /// The most extreme sample, in the direction that counts as bad.
    pub worst: Option<f64>,
    /// How many samples the metric saw. Never a verdict on its own — it is the
    /// context that says whether a move is a change or a different extent.
    pub samples: Option<u64>,
}

/// One metric's before/after.
pub struct Change {
    pub id: String,
    pub title: String,
    pub was: Stats,
    pub now: Stats,
    pub verdict: Move,
}

/// Below this, a difference in the extreme is noise rather than a change. One
/// centimetre: the heights themselves are millimetre-quantized, and no defect
/// worth a commit message moves the worst case by less.
const NOISE_M: f64 = 0.01;

/// Below this, a move in the tail is quantization rather than a change. The
/// distribution bins at 1.3 cm ([`super::dist::Dist::metres`]), so a quantile
/// that slips a single bin has not measured anything; five centimetres is four
/// bins, and is also the order of the contact band docs/VERIFICATION.md gives
/// every structure-versus-surface check.
const NOISE_TAIL_M: f64 = 0.05;

/// Absolute floor, in percentage points, under which a change in how often the
/// defect happens is noise.
const NOISE_PCT: f64 = 0.01;

/// Relative floor on the same quantity, so a metric already firing on an eighth
/// of its samples is not called changed by a two-hundredth of a point. Both
/// floors must be cleared: the absolute one protects a rare defect from being
/// swamped, the relative one protects a common one from hair-trigger verdicts.
const NOISE_PCT_REL: f64 = 0.02;

/// Binomial standard errors of the rate that a move must clear to count.
///
/// A rate measured over twenty samples is not the same evidence as one measured
/// over four hundred thousand, and without this the small-population metrics
/// gate on a single site: `seam.abutment_bare` has twenty samples, so one new
/// offender is a five-point move, and `seam.abutment_plan` has a hundred and
/// forty, so one is nearly a point. Both reported as regressions on a run where
/// nothing about them had meaningfully changed.
const RATE_SIGMA: f64 = 3.0;

/// Samples a metric needs before its tail is allowed to gate.
///
/// [`super::Metric::tail`] is the p99.9, which only means "the bulk of the
/// tail" once the top thousandth holds several samples. Below ten thousand it
/// *is* the extreme wearing a different name — at `seam.abutment_bare`'s twenty
/// samples, p99.9 and the maximum are the same number — and gating on it would
/// smuggle back exactly the single-outlier gate this ladder exists to remove.
/// A metric with fewer samples than this gates on its rate alone, which is an
/// honest statement of what a few hundred samples can support.
const TAIL_MIN_SAMPLES: u64 = 10_000;

/// How far the sample count may move before the two runs are measuring
/// different ground. Populations wobble a little between archives even at a
/// fixed bbox (a feature crosses a tile edge differently); past this the
/// comparison is between two extents, not two trees.
const POPULATION_DRIFT: f64 = 0.05;

impl Scorecard {
    /// The scorecard as JSON, suitable for committing as a baseline.
    pub fn to_json(&self) -> Json {
        json!({
            "archive": self.archive,
            "zooms": self.zooms,
            "scope": scope_json(&self.scope),
            "metrics": self.metrics.iter().map(metric_json).collect::<Vec<_>>(),
        })
    }

    /// Compares against a baseline produced by [`Scorecard::to_json`].
    pub fn diff(&self, baseline: &Json) -> Vec<Change> {
        let empty = Vec::new();
        let base = baseline.get("metrics").and_then(Json::as_array).unwrap_or(&empty);
        let find = |id: &str| base.iter().find(|m| m.get("id").and_then(Json::as_str) == Some(id));

        let mut out: Vec<Change> = self
            .metrics
            .iter()
            .map(|m| {
                let was = find(&m.id).map(baseline_stats).unwrap_or_default();
                let now = Stats {
                    pct: (!m.dist.is_empty()).then(|| m.violation_pct()),
                    tail: m.tail(),
                    worst: m.worst_value(),
                    samples: Some(m.dist.count()),
                };
                Change {
                    id: m.id.clone(),
                    title: m.title.clone(),
                    verdict: verdict(m.sense, was, now),
                    was,
                    now,
                }
            })
            .collect();

        // A metric the baseline had and this run does not is a finding too: a
        // check that stopped running looks exactly like a check that passed.
        for b in base {
            let Some(id) = b.get("id").and_then(Json::as_str) else { continue };
            if self.get(id).is_none() {
                out.push(Change {
                    id: id.to_string(),
                    title: b.get("title").and_then(Json::as_str).unwrap_or("").to_string(),
                    was: baseline_stats(b),
                    now: Stats::default(),
                    verdict: Move::Absent,
                });
            }
        }
        out
    }

    /// How this run's ground differs from the baseline's, in the terms that can
    /// move a metric without anything being wrong. Empty when they agree, or
    /// when the baseline predates the scope record and cannot say.
    ///
    /// This is the other half of the gate. The verdicts below compare
    /// distributions, which is only meaningful if both runs sampled the same
    /// place at the same density: a baseline cut over a different bbox produces
    /// a full column of confident, meaningless verdicts.
    pub fn scope_drift(&self, baseline: &Json) -> Vec<String> {
        let Some(b) = baseline.get("scope") else {
            return vec![
                "the baseline records no scope, so nothing confirms it measured this extent \
                 — re-cut it to make the comparison meaningful"
                    .into(),
            ];
        };
        let mut out = Vec::new();
        let f = |k: &str| b.get(k).and_then(Json::as_f64);
        let s = &self.scope;

        if let Some(t) = b.get("tiles").and_then(Json::as_u64) {
            let (was, now) = (t as f64, s.tiles as f64);
            if was > 0.0 && (now - was).abs() / was > POPULATION_DRIFT {
                out.push(format!("tiles visited {t} → {}", s.tiles));
            }
        }
        if let (Some(bb), Some(nb)) = (b.get("bbox").and_then(Json::as_array), s.bbox) {
            let got: Vec<f64> = bb.iter().filter_map(Json::as_f64).collect();
            let want = [nb.0, nb.1, nb.2, nb.3];
            if got.len() == 4 && got.iter().zip(want).any(|(a, b)| (a - b).abs() > 1e-9) {
                out.push(format!(
                    "extent {:.4},{:.4},{:.4},{:.4} → {:.4},{:.4},{:.4},{:.4}",
                    got[0], got[1], got[2], got[3], want[0], want[1], want[2], want[3]
                ));
            }
        }
        if let Some(v) = f("spacing_m") {
            if (v - s.spacing_m).abs() > 1e-9 {
                out.push(format!("sample spacing {v} m → {} m", s.spacing_m));
            }
        }
        if b.get("truncated").and_then(Json::as_bool) != Some(s.truncated) {
            out.push(format!("tile cap bit: {:?} → {}", b.get("truncated"), s.truncated));
        }
        out
    }
}

/// How far the rate could move on sampling alone, in percentage points:
/// [`RATE_SIGMA`] binomial standard errors at the pooled rate over the smaller
/// of the two populations.
///
/// Zero when either side has no sample count — a baseline old enough not to
/// record one gets the other two floors and nothing else, rather than a floor
/// invented from a number that is not there.
fn rate_noise_pp(was: Stats, now: Stats, a_pct: f64, b_pct: f64) -> f64 {
    let (Some(na), Some(nb)) = (was.samples, now.samples) else { return 0.0 };
    let n = na.min(nb) as f64;
    if n <= 0.0 {
        return 0.0;
    }
    let p = ((a_pct + b_pct) * 0.5 / 100.0).clamp(0.0, 1.0);
    // A rate of exactly zero has no spread of its own, and would let the first
    // violation ever seen gate. Floor the variance at the one-sample scale, so
    // "none of twenty, then one of twenty" is what it is: one sample.
    let var = (p * (1.0 - p)).max(1.0 / n);
    RATE_SIGMA * (var / n).sqrt() * 100.0
}

/// Reads one baseline metric's statistics back. A baseline written before a
/// field existed simply has `None` there, and the verdict falls through to
/// whatever it can still compare.
fn baseline_stats(b: &Json) -> Stats {
    Stats {
        pct: b.get("violation_pct").and_then(Json::as_f64),
        tail: b.get("tail").and_then(Json::as_f64),
        worst: b.get("worst").and_then(Json::as_f64),
        samples: b.get("samples").and_then(Json::as_u64),
    }
}

/// Which way a metric moved, ranked by how much of the distribution has to
/// change for the answer to change.
///
/// The order is the whole point. **How often** the defect happens comes first:
/// it is a statistic over every sample, so a change in it means the geometry
/// moved. **How bad the tail is** comes second, for the case where the same
/// number of sites fail but each fails worse. The **single worst sample** comes
/// last and cannot gate at all ([`Move::Outlier`]).
///
/// The previous order was exactly inverted — `worst` first, the rate consulted
/// only if the worst moved less than a centimetre. Since `worst` is a maximum
/// over millions of samples it essentially always moves, so the rate branch was
/// all but dead and the gate was reading the noisiest number on the card. On
/// the Montreux extract that reported seven regressions of which none had moved
/// its median, and called `contact.kerb_lip` a regression on the run where its
/// violation rate fell from 12.8 % to 8.7 %.
fn verdict(sense: Sense, was: Stats, now: Stats) -> Move {
    // Nothing measured on one side: a check was added, or stopped running.
    if was.worst.is_none() || now.worst.is_none() {
        return Move::Absent;
    }
    // 1. How often.
    if let (Some(a), Some(b)) = (was.pct, now.pct) {
        let d = b - a;
        let floor = NOISE_PCT.max(NOISE_PCT_REL * a).max(rate_noise_pp(was, now, a, b));
        if d.abs() >= floor {
            return if d < 0.0 { Move::Improved } else { Move::Regressed };
        }
    }
    // 2. How bad, in the bulk of the tail — where there is enough population
    //    for a tail to be distinct from the extreme.
    let tail_supported = was.samples.unwrap_or(0) >= TAIL_MIN_SAMPLES
        && now.samples.unwrap_or(0) >= TAIL_MIN_SAMPLES;
    if tail_supported {
        if let (Some(a), Some(b)) = (was.tail, now.tail) {
            let d = b - a;
            if d.abs() >= NOISE_TAIL_M {
                return match (sense, d > 0.0) {
                    (Sense::LowerIsWorse, true) | (Sense::HigherIsWorse, false) => Move::Improved,
                    _ => Move::Regressed,
                };
            }
        }
    }
    // 3. The extreme alone. Reported, not gated.
    if let (Some(a), Some(b)) = (was.worst, now.worst) {
        if (b - a).abs() >= NOISE_M {
            return Move::Outlier;
        }
    }
    Move::Same
}

pub fn metric_json(m: &Metric) -> Json {
    json!({
        "id": m.id,
        "invariant": m.invariant.as_str(),
        "population": m.population,
        "title": m.title,
        "sense": match m.sense { Sense::LowerIsWorse => "lower_is_worse", _ => "higher_is_worse" },
        "threshold": m.threshold,
        "skipped": m.skipped,
        "samples": m.dist.count(),
        "violations": m.violations(),
        "violation_pct": m.violation_pct(),
        "worst": m.worst_value(),
        "tail": m.tail(),
        "median": m.dist.quantile(0.5),
        "offenders": m.worst.iter().map(offender_json).collect::<Vec<_>>(),
    })
}

fn offender_json(o: &Offender) -> Json {
    json!({ "lon": o.lon, "lat": o.lat, "zoom": o.zoom, "value": o.value, "note": o.note })
}

fn scope_json(s: &Scope) -> Json {
    json!({
        "commit": s.commit,
        "tiles": s.tiles,
        "bbox": s.bbox.map(|(w, so, e, n)| vec![w, so, e, n]),
        "spacing_m": s.spacing_m,
        "max_tiles": s.max_tiles,
        "at": s.at.map(|(lon, lat)| vec![lon, lat]),
        "truncated": s.truncated,
    })
}

/// The human-readable scorecard.
pub fn table(card: &Scorecard) -> String {
    let mut s = String::new();
    let zooms: Vec<String> = card.zooms.iter().map(|z| format!("z{z}")).collect();
    s.push_str(&format!("scorecard  {}  {}\n\n", card.archive, zooms.join(" ")));
    s.push_str(&format!(
        "{:<34} {:>3} {:>10} {:>10} {:>9} {:>9}\n",
        "metric", "inv", "samples", "worst", "tail", "over"
    ));
    s.push_str(&"-".repeat(80));
    s.push('\n');
    for m in &card.metrics {
        if let Some(why) = &m.skipped {
            s.push_str(&format!("{:<34} {:>3} {:>10}   {why}\n", m.id, m.invariant, "-"));
            continue;
        }
        s.push_str(&format!(
            "{:<34} {:>3} {:>10} {:>10} {:>9} {:>8.3}%\n",
            m.id,
            m.invariant,
            m.dist.count(),
            fmt(m.worst_value()),
            fmt(m.tail()),
            m.violation_pct(),
        ));
    }
    s.push('\n');
    for m in &card.metrics {
        if m.worst.is_empty() {
            continue;
        }
        s.push_str(&format!("{} — {}\n", m.id, m.title));
        for o in m.worst.iter().take(5) {
            s.push_str(&format!(
                "    {:>9.3}  {:.6},{:.6}  z{}  {}\n",
                o.value, o.lon, o.lat, o.zoom, o.note
            ));
        }
        s.push('\n');
    }
    s
}

/// The diff table: what this run changed against the baseline.
///
/// Every statistic the verdict consulted is shown, in the order it consulted
/// them, so the verdict can always be checked against the numbers behind it —
/// and so the two ways a number moves without the geometry changing (a
/// different population, a lone outlier) are visible rather than inferred.
pub fn diff_table(changes: &[Change]) -> String {
    let mut s = String::new();
    s.push_str(&format!(
        "{:<32} {:>17} {:>8} {:>15} {:>17} {:>8}  {}\n",
        "metric", "over % was→now", "Δpp", "tail was→now", "worst was→now", "samples", "verdict"
    ));
    s.push_str(&"-".repeat(110));
    s.push('\n');
    for c in changes {
        let dpp = match (c.was.pct, c.now.pct) {
            (Some(a), Some(b)) => format!("{:+.3}", b - a),
            _ => "-".into(),
        };
        s.push_str(&format!(
            "{:<32} {:>17} {:>8} {:>15} {:>17} {:>8}  {}\n",
            c.id,
            pair(c.was.pct, c.now.pct, 3),
            dpp,
            pair(c.was.tail, c.now.tail, 2),
            pair(c.was.worst, c.now.worst, 2),
            samples(c.was.samples, c.now.samples),
            match c.verdict {
                Move::Improved => "improved",
                Move::Regressed => "REGRESSED",
                Move::Outlier => "outlier only",
                Move::Same => "same",
                Move::Absent => "absent",
            }
        ));
    }
    s
}

/// `was→now` in one column, or a single value when only one side exists.
fn pair(was: Option<f64>, now: Option<f64>, dp: usize) -> String {
    match (was, now) {
        (Some(a), Some(b)) => format!("{}→{}", num(a, dp), num(b, dp)),
        (Some(a), None) => format!("{}→-", num(a, dp)),
        (None, Some(b)) => format!("-→{}", num(b, dp)),
        (None, None) => "-".into(),
    }
}

/// The population change as a percentage, since the counts themselves are too
/// wide to read side by side. Flagged when it is large enough that the two runs
/// are measuring different ground.
fn samples(was: Option<u64>, now: Option<u64>) -> String {
    match (was, now) {
        (Some(a), Some(b)) if a > 0 => {
            let d = (b as f64 - a as f64) / a as f64;
            let mark = if d.abs() > POPULATION_DRIFT { "!" } else { "" };
            format!("{:+.1}%{mark}", d * 100.0)
        }
        _ => "-".into(),
    }
}

fn num(v: f64, dp: usize) -> String {
    if v.is_infinite() {
        "inf".into()
    } else {
        format!("{v:.dp$}")
    }
}

fn fmt(v: Option<f64>) -> String {
    match v {
        Some(v) if v.is_infinite() => "inf".into(),
        Some(v) => format!("{v:.3}"),
        None => "-".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::verify::dist::Dist;

    fn card(id: &str, sense: Sense, samples: &[f64]) -> Scorecard {
        let mut d = Dist::metres();
        for &s in samples {
            d.push(s);
        }
        Scorecard {
            archive: "test.arpa".into(),
            zooms: vec![16],
            scope: crate::verify::Scope {
                tiles: 1,
                bbox: Some((0.0, 0.0, 1.0, 1.0)),
                spacing_m: 1.0,
                max_tiles: 4096,
                at: None,
                truncated: false,
                commit: None,
            },
            metrics: vec![Metric {
                id: id.into(),
                invariant: crate::verify::Invariant::I4,
                population: String::new(),
                title: "t".into(),
                detail: "d".into(),
                sense,
                threshold: if sense == Sense::LowerIsWorse { -0.05 } else { 0.05 },
                dist: d,
                worst: Vec::new(),
                skipped: None,
            }],
        }
    }

    /// `n` samples of which `bad` sit at `value` and the rest are clean — the
    /// shape the rate-first verdict is actually about.
    fn mix(sense: Sense, n: usize, bad: usize, value: f64) -> Scorecard {
        let mut s = vec![0.0; n - bad];
        s.extend(std::iter::repeat(value).take(bad));
        card("m", sense, &s)
    }

    #[test]
    fn a_defect_that_happens_more_often_is_a_regression() {
        let before = mix(Sense::LowerIsWorse, 1000, 10, -1.0);
        let after = mix(Sense::LowerIsWorse, 1000, 80, -1.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    #[test]
    fn a_defect_that_happens_less_often_is_an_improvement() {
        let before = mix(Sense::LowerIsWorse, 1000, 80, -1.0);
        let after = mix(Sense::LowerIsWorse, 1000, 10, -1.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Improved);
    }

    /// The case the old order got backwards, and the reason for this rewrite.
    /// `contact.kerb_lip` on Montreux: the violation rate fell from 12.8 % to
    /// 8.7 % on a distribution whose median did not move, while one new outlier
    /// took the worst from 15 m to 76 m. That is an improvement with an
    /// outlier, and reporting it as a regression sends someone to revert a good
    /// change.
    #[test]
    fn a_falling_rate_beats_a_lone_new_outlier() {
        let before = mix(Sense::HigherIsWorse, 1000, 128, 15.0);
        let mut after = mix(Sense::HigherIsWorse, 1000, 87, 15.0);
        after.metrics[0].dist.push(76.0);
        let d = after.diff(&before.to_json());
        assert_eq!(d[0].verdict, Move::Improved, "rate 12.8 % → 8.7 % is the finding");
        assert_eq!(d[0].now.worst, Some(76.0), "and the outlier is still reported");
    }

    /// `slope.terrain_face`: one sliver triangle in 12.9 million moved the
    /// worst from 201.8 to 349.9 with the rate and the tail unchanged. It must
    /// be visible and it must not gate.
    #[test]
    fn a_lone_outlier_on_a_held_distribution_does_not_gate() {
        let before = {
            let mut c = mix(Sense::HigherIsWorse, 10_000, 60, 3.0);
            c.metrics[0].dist.push(201.8);
            c
        };
        let after = {
            let mut c = mix(Sense::HigherIsWorse, 10_000, 60, 3.0);
            c.metrics[0].dist.push(349.9);
            c
        };
        let d = after.diff(&before.to_json());
        assert_eq!(d[0].verdict, Move::Outlier);
        assert_ne!(d[0].verdict, Move::Regressed, "an outlier must never trip the gate");
        assert_eq!(d[0].now.worst, Some(349.9));
    }

    /// With the rate held, a tail that deepens is still a regression: the same
    /// number of sites failing, each one worse.
    #[test]
    fn a_deepening_tail_at_a_held_rate_is_a_regression() {
        let before = mix(Sense::HigherIsWorse, 20_000, 2_000, 1.0);
        let after = mix(Sense::HigherIsWorse, 20_000, 2_000, 4.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    #[test]
    fn the_sense_decides_which_direction_the_tail_is_better() {
        let before = mix(Sense::LowerIsWorse, 20_000, 2_000, -1.0);
        let after = mix(Sense::LowerIsWorse, 20_000, 2_000, -4.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    #[test]
    fn millimetre_wobble_is_not_a_change() {
        let before = mix(Sense::LowerIsWorse, 1000, 100, -1.0);
        let after = mix(Sense::LowerIsWorse, 1000, 100, -1.003);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Same);
    }

    /// A single-bin quantile slip is quantization, not a move: the histogram
    /// bins at 1.3 cm, and `slope.terrain_face`'s tail "changed" by exactly one
    /// bin between two runs of the same tree. Held at a fixed extreme, so this
    /// tests the tail floor alone.
    #[test]
    fn a_single_bin_slip_in_the_tail_is_not_a_change() {
        let held = |at: f64| {
            let mut c = mix(Sense::HigherIsWorse, 20_000, 1_999, at);
            c.metrics[0].dist.push(50.0);
            c
        };
        let d = held(3.5133).diff(&held(3.5000).to_json());
        assert_eq!(d[0].was.tail.zip(d[0].now.tail).map(|(a, b)| b - a < 0.05), Some(true));
        assert_eq!(d[0].verdict, Move::Same);
    }

    /// A common defect must not be called changed by a rounding-scale wobble in
    /// its rate, or every run reads as a change.
    #[test]
    fn a_hair_trigger_move_on_a_common_defect_is_not_a_change() {
        let before = mix(Sense::HigherIsWorse, 100_000, 12_000, 1.0);
        let after = mix(Sense::HigherIsWorse, 100_000, 12_010, 1.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Same);
    }

    /// `seam.abutment_bare` has twenty samples. One new offender is a five-point
    /// move in the rate and means nothing, and gating on it sends someone
    /// hunting a regression that is one site.
    #[test]
    fn one_sample_in_a_small_population_is_not_a_regression() {
        let before = mix(Sense::HigherIsWorse, 20, 0, 0.0);
        let after = mix(Sense::HigherIsWorse, 20, 1, 0.25);
        let d = after.diff(&before.to_json());
        // Reported — one site did appear — but not gated, because one site in
        // twenty is not evidence that anything changed.
        assert_eq!(d[0].verdict, Move::Outlier);
        assert_eq!(d[0].now.worst, Some(0.25), "and the site is still named");
    }

    /// …but the same five-point move over a real population is evidence.
    #[test]
    fn the_same_move_over_a_large_population_is_a_regression() {
        let before = mix(Sense::HigherIsWorse, 20_000, 0, 0.0);
        let after = mix(Sense::HigherIsWorse, 20_000, 1_000, 0.25);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    /// A rare defect in a big population still gates on a small move:
    /// `contact.kerb_unwalled` went 0.003 % → 0.017 % over 440 000 samples, and
    /// that is thirteen thousandths of a point worth believing.
    #[test]
    fn a_rare_defect_in_a_large_population_still_registers() {
        let before = mix(Sense::HigherIsWorse, 440_000, 13, 1.0);
        let after = mix(Sense::HigherIsWorse, 440_000, 75, 1.0);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    #[test]
    fn a_check_that_stopped_running_is_reported_not_silently_passed() {
        let before = card("m", Sense::LowerIsWorse, &[-4.0]);
        let after = Scorecard {
            archive: "t".into(),
            zooms: vec![16],
            scope: crate::verify::Scope::default(),
            metrics: Vec::new(),
        };
        let d = after.diff(&before.to_json());
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].verdict, Move::Absent);
        assert_eq!(d[0].was.worst, Some(-4.0));
    }

    #[test]
    fn json_round_trips_through_a_baseline_file() {
        let c = card("m", Sense::LowerIsWorse, &[-4.2, -0.1, 0.0, 1.0]);
        let j = c.to_json();
        let m = &j["metrics"][0];
        assert_eq!(m["id"], "m");
        assert_eq!(m["samples"], 4);
        assert_eq!(m["violations"], 2);
        assert!((m["worst"].as_f64().unwrap() + 4.2).abs() < 1e-9);
        // And a run diffed against its own baseline must be flat.
        assert!(c.diff(&j).iter().all(|d| d.verdict == Move::Same));
    }

    #[test]
    fn a_baseline_over_different_ground_is_called_out_rather_than_compared() {
        let before = card("m", Sense::LowerIsWorse, &[-1.0]);
        let mut after = card("m", Sense::LowerIsWorse, &[-1.0]);
        after.scope.bbox = Some((10.0, 10.0, 11.0, 11.0));
        after.scope.tiles = 900;
        let drift = after.scope_drift(&before.to_json());
        assert!(drift.iter().any(|d| d.contains("extent")), "{drift:?}");
        assert!(drift.iter().any(|d| d.contains("tiles visited")), "{drift:?}");
    }

    /// A baseline written before the scope record must say so, not pass
    /// silently — that is exactly the state the committed Montreux baseline was
    /// in, and it is indistinguishable from a scope that matches.
    #[test]
    fn a_baseline_without_a_scope_says_so() {
        let after = card("m", Sense::LowerIsWorse, &[-1.0]);
        let legacy = json!({ "archive": "old.arpa", "zooms": [16], "metrics": [] });
        assert!(!after.scope_drift(&legacy).is_empty());
    }

    #[test]
    fn the_scope_survives_a_round_trip() {
        let c = card("m", Sense::LowerIsWorse, &[-1.0]);
        assert!(c.scope_drift(&c.to_json()).is_empty(), "a card must not drift from itself");
    }

    #[test]
    fn the_table_names_skipped_checks_rather_than_printing_zero() {
        let mut c = card("m", Sense::LowerIsWorse, &[]);
        c.metrics[0].skipped = Some("no terrain mesh".into());
        let t = table(&c);
        assert!(t.contains("no terrain mesh"), "{t}");
        assert!(!t.contains("0.000%"), "a skipped check must not look clean:\n{t}");
    }
}
