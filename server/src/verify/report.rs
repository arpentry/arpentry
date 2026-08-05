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

use super::{Metric, Offender, Scorecard, Sense};

/// How a metric moved against a baseline.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Move {
    Improved,
    Regressed,
    Same,
    /// In one run but not the other — a check was added, or could not run.
    Absent,
}

/// One metric's before/after.
pub struct Change {
    pub id: String,
    pub title: String,
    pub was: Option<f64>,
    pub now: Option<f64>,
    pub was_pct: Option<f64>,
    pub now_pct: Option<f64>,
    pub verdict: Move,
}

/// Below this, a difference is noise rather than a change. One centimetre: the
/// heights themselves are millimetre-quantized, and no defect worth a commit
/// message moves the worst case by less.
const NOISE_M: f64 = 0.01;

impl Scorecard {
    /// The scorecard as JSON, suitable for committing as a baseline.
    pub fn to_json(&self) -> Json {
        json!({
            "archive": self.archive,
            "zooms": self.zooms,
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
                let b = find(&m.id);
                let was = b.and_then(|b| b.get("worst")).and_then(Json::as_f64);
                let was_pct = b.and_then(|b| b.get("violation_pct")).and_then(Json::as_f64);
                let now = m.worst_value();
                let now_pct = (!m.dist.is_empty()).then(|| m.violation_pct());
                Change {
                    id: m.id.clone(),
                    title: m.title.clone(),
                    verdict: verdict(m.sense, was, now, was_pct, now_pct),
                    was,
                    now,
                    was_pct,
                    now_pct,
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
                    was: b.get("worst").and_then(Json::as_f64),
                    now: None,
                    was_pct: b.get("violation_pct").and_then(Json::as_f64),
                    now_pct: None,
                    verdict: Move::Absent,
                });
            }
        }
        out
    }
}

fn verdict(
    sense: Sense,
    was: Option<f64>,
    now: Option<f64>,
    was_pct: Option<f64>,
    now_pct: Option<f64>,
) -> Move {
    let (Some(was), Some(now)) = (was, now) else { return Move::Absent };
    let delta = now - was;
    if delta.abs() >= NOISE_M {
        return match (sense, delta > 0.0) {
            (Sense::LowerIsWorse, true) | (Sense::HigherIsWorse, false) => Move::Improved,
            _ => Move::Regressed,
        };
    }
    // The extreme held; the tally can still have moved, and a change that
    // halves how often a defect happens is worth seeing even when the single
    // worst case is unchanged.
    match (was_pct, now_pct) {
        (Some(a), Some(b)) if (b - a).abs() > 0.005 => {
            if b < a {
                Move::Improved
            } else {
                Move::Regressed
            }
        }
        _ => Move::Same,
    }
}

fn metric_json(m: &Metric) -> Json {
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
pub fn diff_table(changes: &[Change]) -> String {
    let mut s = String::new();
    s.push_str(&format!(
        "{:<34} {:>10} {:>10} {:>10} {:>11}\n",
        "metric", "was", "now", "delta", "verdict"
    ));
    s.push_str(&"-".repeat(80));
    s.push('\n');
    for c in changes {
        let delta = match (c.was, c.now) {
            (Some(a), Some(b)) => format!("{:+.3}", b - a),
            _ => "-".into(),
        };
        s.push_str(&format!(
            "{:<34} {:>10} {:>10} {:>10} {:>11}\n",
            c.id,
            fmt(c.was),
            fmt(c.now),
            delta,
            match c.verdict {
                Move::Improved => "improved",
                Move::Regressed => "REGRESSED",
                Move::Same => "same",
                Move::Absent => "absent",
            }
        ));
    }
    s
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

    #[test]
    fn a_deeper_burial_reads_as_a_regression() {
        let before = card("m", Sense::LowerIsWorse, &[-1.0, 0.0]);
        let after = card("m", Sense::LowerIsWorse, &[-4.2, 0.0]);
        let d = after.diff(&before.to_json());
        assert_eq!(d[0].verdict, Move::Regressed);
        assert_eq!(d[0].was, Some(-1.0));
        assert_eq!(d[0].now, Some(-4.2));
    }

    #[test]
    fn a_shallower_burial_reads_as_an_improvement() {
        let before = card("m", Sense::LowerIsWorse, &[-4.2, 0.0]);
        let after = card("m", Sense::LowerIsWorse, &[-1.0, 0.0]);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Improved);
    }

    #[test]
    fn the_sense_decides_which_direction_is_better() {
        let before = card("m", Sense::HigherIsWorse, &[0.0, 1.0]);
        let after = card("m", Sense::HigherIsWorse, &[0.0, 4.0]);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Regressed);
    }

    #[test]
    fn millimetre_wobble_is_not_a_change() {
        let before = card("m", Sense::LowerIsWorse, &[-1.0]);
        let after = card("m", Sense::LowerIsWorse, &[-1.003]);
        assert_eq!(after.diff(&before.to_json())[0].verdict, Move::Same);
    }

    #[test]
    fn a_held_extreme_with_a_moved_tally_still_registers() {
        // The single worst case is unchanged, but the defect now happens half
        // as often. That is progress and must not print as "same".
        let before = card("m", Sense::LowerIsWorse, &[-4.0, -1.0, -1.0, 0.0]);
        let after = card("m", Sense::LowerIsWorse, &[-4.0, 0.0, 0.0, 0.0]);
        let d = after.diff(&before.to_json());
        assert_eq!(d[0].verdict, Move::Improved);
        assert_eq!(d[0].now, Some(-4.0));
    }

    #[test]
    fn a_check_that_stopped_running_is_reported_not_silently_passed() {
        let before = card("m", Sense::LowerIsWorse, &[-4.0]);
        let after = Scorecard { archive: "t".into(), zooms: vec![16], metrics: Vec::new() };
        let d = after.diff(&before.to_json());
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].verdict, Move::Absent);
        assert_eq!(d[0].was, Some(-4.0));
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
    fn the_table_names_skipped_checks_rather_than_printing_zero() {
        let mut c = card("m", Sense::LowerIsWorse, &[]);
        c.metrics[0].skipped = Some("no terrain mesh".into());
        let t = table(&c);
        assert!(t.contains("no terrain mesh"), "{t}");
        assert!(!t.contains("0.000%"), "a skipped check must not look clean:\n{t}");
    }
}
