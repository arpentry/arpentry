//! Scene verification — the numbers every change has to beat.
//!
//! `solve::consistency` measures the solved model, and reports it consistent.
//! That is stage 3 of five (docs/GENERATION.md §5), and it is not where the
//! remaining defects are: asphalt chording over the ground, a crest sampling a
//! neighbour's bench, a plate z-fighting its neighbour, a deck stepping at its
//! abutment. Each is a *relation between two emitted surfaces*, invisible to
//! anything that reads only `SolvedModel`, and until now visible only to
//! someone looking at a rendered picture.
//!
//! Looking at pictures is a fine way to *discover* a defect and a poor way to
//! keep one dead. This module is the other half: every invariant in
//! GENERATION.md §7 turned into a measurement over the shipped archive, so a
//! change is judged by a table diff rather than an impression, and so a defect
//! found once by eye becomes a number that can never quietly come back.
//!
//! Design notes worth keeping in mind when adding a check:
//!
//! - **Measure the surface, not the vertices.** See `mesh`.
//! - **Report a distribution, not a verdict.** The thresholds are priors; the
//!   comparison against last week's run is not. See `dist`.
//! - **Name the place.** A metric that moved without a coordinate to look at is
//!   a metric nobody can act on, so every check keeps its worst offenders.
//! - **Scope to what the design actually promises.** At-grade road height is
//!   deliberately zoom-dependent below the reference rung (docs/GROUND.md §4,
//!   the datum lift), so the cross-zoom check applies to structures only.
//!   Checking a property the design never claimed produces noise, and noise is
//!   what makes a scorecard get ignored.

pub mod checks;
pub mod corpus;
pub mod dist;
pub mod mesh;
pub mod model;
pub mod report;
pub mod scene;
pub mod section;

use dist::Dist;

/// The invariants of docs/GENERATION.md §7, as a closed set.
///
/// A hand-typed integer let `slope.road_grade` claim invariant 6 while §8's
/// binding table gives the grade ceiling to I2, and nothing in the code could
/// notice the disagreement. Naming them makes the §8 table a type: a check
/// declares which predicate it measures, and adding an invariant to the doc
/// without a check — or a check with no invariant — stops compiling.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Invariant {
    /// One ground function: every consumer reads the same engineered ground.
    I1,
    /// Surface continuity: zero step at shared geometry, grade within the
    /// (modality, class) ceiling along every drawn centerline.
    I2,
    /// Vertical order with plausible clearance — equality where at grade.
    I3,
    /// Support and contact: nothing floats, nothing is buried by accident.
    I4,
    /// Determinism across cuts: tiles and zooms derive identical heights.
    I5,
    /// Graceful degradation: lost detail, never spectacle.
    I6,
    /// Datum monotonicity: a height depends only on its own stratum and its
    /// seniors.
    I7,
    /// Ground monotonicity: a layer changes the ground only inside its own
    /// declared footprints, exactly once.
    I8,
    /// Closure: wherever two drawn elements are plan-adjacent and differ in
    /// height past the contact band, a face spans the step. Air is legal only
    /// where a structure separates two levels, and then the structure's own
    /// solids are the closure.
    I9,
}

impl Invariant {
    pub fn as_str(self) -> &'static str {
        match self {
            Invariant::I1 => "I1",
            Invariant::I2 => "I2",
            Invariant::I3 => "I3",
            Invariant::I4 => "I4",
            Invariant::I5 => "I5",
            Invariant::I6 => "I6",
            Invariant::I7 => "I7",
            Invariant::I8 => "I8",
            Invariant::I9 => "I9",
        }
    }
}

impl std::fmt::Display for Invariant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Which direction is a defect. Determines what "worst" means and which side of
/// the threshold counts as a violation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sense {
    /// Signed clearances: the further below the threshold, the worse. A
    /// pavement 4 m under the drawn ground is worse than one 4 cm under.
    LowerIsWorse,
    /// Magnitudes: a step, a disagreement, an excursion. Zero is perfect.
    HigherIsWorse,
}

/// A place worth looking at, carried alongside the number that found it.
#[derive(Clone, Debug)]
pub struct Offender {
    pub lon: f64,
    pub lat: f64,
    pub zoom: u8,
    pub value: f64,
    pub note: String,
}

/// Keeps the `k` most severe offenders seen, without holding the rest.
pub struct Worst {
    sense: Sense,
    k: usize,
    items: Vec<Offender>,
}

impl Worst {
    pub fn new(sense: Sense, k: usize) -> Worst {
        Worst { sense, k, items: Vec::new() }
    }

    pub fn offer(&mut self, o: Offender) {
        self.items.push(o);
        // Re-sorting only when the buffer has grown well past `k` keeps this
        // O(1) amortized on the hot path, where it runs per sample.
        if self.items.len() >= self.k * 4 + 32 {
            self.trim();
        }
    }

    fn trim(&mut self) {
        let sense = self.sense;
        self.items.sort_by(|a, b| match sense {
            Sense::LowerIsWorse => a.value.partial_cmp(&b.value).unwrap_or(std::cmp::Ordering::Equal),
            Sense::HigherIsWorse => {
                b.value.partial_cmp(&a.value).unwrap_or(std::cmp::Ordering::Equal)
            }
        });
        self.items.truncate(self.k);
    }

    pub fn into_vec(mut self) -> Vec<Offender> {
        self.trim();
        self.items
    }

    /// Folds another collector in — the checks accumulate per tile.
    pub fn merge(&mut self, other: Worst) {
        self.items.extend(other.items);
        self.trim();
    }
}

/// One measured property of the emitted scene.
pub struct Metric {
    /// Stable across runs; the key a baseline diff joins on. This is the check
    /// name docs/GENERATION.md §8 binds to an invariant.
    pub id: String,
    /// Which §7 invariant this measures — §8's binding, made a type.
    pub invariant: Invariant,
    pub title: String,
    /// Exactly what is sampled, and what is *not*. §8: "Every check states its
    /// population and its coverage limits explicitly. A metric that silently
    /// samples a subset reads as 'covered everything' when it did not."
    pub population: String,
    /// One line: what a violation means and what it would look like on screen.
    pub detail: String,
    pub sense: Sense,
    /// The side of this counts as a violation. A prior, and only ever used to
    /// produce the tally column — the distribution stands without it.
    pub threshold: f64,
    pub dist: Dist,
    pub worst: Vec<Offender>,
    /// Set when a check could not run (no terrain mesh at this zoom, no
    /// structures in the extract). An empty metric and a clean metric are very
    /// different things and must never print the same.
    pub skipped: Option<String>,
}

impl Metric {
    /// The single number that names the defect: the most extreme sample, in the
    /// direction that counts as bad.
    pub fn worst_value(&self) -> Option<f64> {
        match self.sense {
            Sense::LowerIsWorse => self.dist.min(),
            Sense::HigherIsWorse => self.dist.max(),
        }
    }

    /// How many samples fall on the wrong side of the threshold.
    pub fn violations(&self) -> u64 {
        match self.sense {
            Sense::LowerIsWorse => self.dist.count_below(self.threshold),
            Sense::HigherIsWorse => self.dist.count_above(self.threshold),
        }
    }

    pub fn violation_pct(&self) -> f64 {
        if self.dist.is_empty() {
            return 0.0;
        }
        100.0 * self.violations() as f64 / self.dist.count() as f64
    }

    /// The bulk-of-the-tail figure a single outlier cannot dominate — the same
    /// role `p99_junction_step_m` plays in `solve::consistency`.
    pub fn tail(&self) -> Option<f64> {
        match self.sense {
            Sense::LowerIsWorse => self.dist.quantile(0.001),
            Sense::HigherIsWorse => self.dist.quantile(0.999),
        }
    }
}

/// What a run actually measured — the provenance a committed baseline needs.
///
/// A scorecard is only comparable to another one taken over the same ground.
/// Without this, a baseline is a column of numbers with no way to tell whether
/// it describes the same extent, the same zoom or the same tree: the committed
/// Montreux baseline recorded only its archive *filename*, which pointed at a
/// throwaway A/B archive in a session scratchpad, so nothing about it could be
/// reproduced or even located. Every field here is something that changes the
/// population a metric is measured over, which is to say something that can
/// move a number without anything being wrong.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scope {
    /// Tiles actually decoded and visited, across all measured zooms.
    pub tiles: usize,
    /// Plan extent of those tiles: `(west, south, east, north)`. `None` when
    /// nothing was visited.
    pub bbox: Option<(f64, f64, f64, f64)>,
    /// The sampling options that decide how densely the surface is probed.
    pub spacing_m: f64,
    pub max_tiles: usize,
    /// Set when `--at` scoped the run to one tile.
    pub at: Option<(f64, f64)>,
    /// Whether `max_tiles` bit, so partial coverage never reads as full.
    pub truncated: bool,
    /// The tree that produced the run, best-effort `git rev-parse --short
    /// HEAD`. `None` outside a repository — the tool still works, it just
    /// cannot say what it was measuring.
    pub commit: Option<String>,
}

/// A run's full set of measurements.
pub struct Scorecard {
    pub archive: String,
    pub zooms: Vec<u8>,
    pub scope: Scope,
    pub metrics: Vec<Metric>,
}

impl Scorecard {
    pub fn get(&self, id: &str) -> Option<&Metric> {
        self.metrics.iter().find(|m| m.id == id)
    }

    /// Whether any check found a violation. Not a pass/fail gate on its own —
    /// the archive has known deviations (invariant 4's missing pier support is
    /// documented as deliberate) — but the summary line the CLI prints.
    pub fn total_violations(&self) -> u64 {
        self.metrics.iter().map(|m| m.violations()).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metric(sense: Sense, threshold: f64, samples: &[f64]) -> Metric {
        let mut d = Dist::metres();
        for &s in samples {
            d.push(s);
        }
        Metric {
            id: "t".into(),
            invariant: Invariant::I4,
            title: "t".into(),
            population: "t".into(),
            detail: "t".into(),
            sense,
            threshold,
            dist: d,
            worst: Vec::new(),
            skipped: None,
        }
    }

    #[test]
    fn worst_follows_the_sense() {
        let lower = metric(Sense::LowerIsWorse, -0.05, &[-4.2, 0.0, 3.0]);
        assert_eq!(lower.worst_value(), Some(-4.2));
        let higher = metric(Sense::HigherIsWorse, 0.05, &[0.0, 0.1, 2.5]);
        assert_eq!(higher.worst_value(), Some(2.5));
    }

    #[test]
    fn violations_count_only_the_bad_side() {
        let m = metric(Sense::LowerIsWorse, -0.05, &[-4.2, -1.0, 0.0, 3.0]);
        assert_eq!(m.violations(), 2);
        assert_eq!(m.violation_pct(), 50.0);
    }

    #[test]
    fn an_empty_metric_reports_no_violations_rather_than_dividing_by_zero() {
        let m = metric(Sense::LowerIsWorse, -0.05, &[]);
        assert_eq!(m.violations(), 0);
        assert_eq!(m.violation_pct(), 0.0);
        assert_eq!(m.worst_value(), None);
    }

    #[test]
    fn worst_keeps_the_k_most_severe_in_order() {
        let mut w = Worst::new(Sense::LowerIsWorse, 3);
        for v in [-1.0, -9.0, -0.5, -4.0, -7.0, 0.0] {
            w.offer(Offender { lon: v, lat: 0.0, zoom: 16, value: v, note: String::new() });
        }
        let got: Vec<f64> = w.into_vec().iter().map(|o| o.value).collect();
        assert_eq!(got, vec![-9.0, -7.0, -4.0]);
    }

    #[test]
    fn worst_survives_more_offers_than_its_trim_interval() {
        // The amortized trim must not lose a severe offender seen early.
        let mut w = Worst::new(Sense::HigherIsWorse, 2);
        w.offer(Offender { lon: 0.0, lat: 0.0, zoom: 16, value: 99.0, note: String::new() });
        for i in 0..500 {
            let v = i as f64 / 1000.0;
            w.offer(Offender { lon: 0.0, lat: 0.0, zoom: 16, value: v, note: String::new() });
        }
        assert_eq!(w.into_vec()[0].value, 99.0);
    }

    #[test]
    fn merging_collectors_keeps_the_global_worst() {
        let mut a = Worst::new(Sense::LowerIsWorse, 2);
        let mut b = Worst::new(Sense::LowerIsWorse, 2);
        a.offer(Offender { lon: 0.0, lat: 0.0, zoom: 16, value: -1.0, note: String::new() });
        b.offer(Offender { lon: 0.0, lat: 0.0, zoom: 16, value: -8.0, note: String::new() });
        a.merge(b);
        assert_eq!(a.into_vec()[0].value, -8.0);
    }
}
