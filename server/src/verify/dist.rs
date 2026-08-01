//! The distribution behind every measurement.
//!
//! A check does not answer "is this right?" — it answers "by how much, and how
//! often". A boolean would need a threshold, and the thresholds here are priors
//! nobody knows in advance (how far below the drawn ground may asphalt sit
//! before it reads as buried?). A distribution needs none: it is comparable
//! against the same measurement taken before the change, which is the question
//! actually being asked.
//!
//! Backed by a fixed-width histogram so a hundred million samples cost a
//! bounded 100 kB, with the count, extremes and violation tallies kept exactly
//! outside it — those are the numbers quoted, and quoting a binned extreme
//! would understate a defect.

use std::fmt;

/// Number of histogram bins. With the ±32 m span the checks use, one bin is
/// 1.3 cm — finer than any defect worth naming, and the extremes are exact
/// regardless.
const BINS: usize = 4800;

/// A measured quantity: every sample folded into a histogram, with the count,
/// extremes and sum kept exactly.
#[derive(Clone)]
pub struct Dist {
    lo: f64,
    hi: f64,
    bins: Box<[u64; BINS]>,
    /// Samples below `lo` / at or above `hi`. Percentiles walk these as the
    /// outermost buckets, so a long tail outside the range still lands the
    /// percentile on the correct side rather than being silently clamped.
    under: u64,
    over: u64,
    n: u64,
    min: f64,
    max: f64,
    sum: f64,
}

impl Dist {
    /// A distribution binning `[lo, hi)`; samples outside still count, and
    /// still move `min`/`max`, they just share the two outermost buckets.
    pub fn new(lo: f64, hi: f64) -> Dist {
        Dist {
            lo,
            hi,
            bins: Box::new([0; BINS]),
            under: 0,
            over: 0,
            n: 0,
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            sum: 0.0,
        }
    }

    /// The span the checks in this module measure over: ±32 m covers a viaduct
    /// deck over a ravine and a tunnel under a hill, and anything beyond it is
    /// a spectacle the extremes will name anyway.
    pub fn metres() -> Dist {
        Dist::new(-32.0, 32.0)
    }

    /// Folds one sample in. NaN is dropped rather than poisoning the extremes —
    /// a degenerate triangle should cost one sample, not the whole measurement
    /// (DESIGN.md, define errors out of existence).
    pub fn push(&mut self, v: f64) {
        if v.is_nan() {
            return;
        }
        self.n += 1;
        self.sum += v;
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        if v < self.lo {
            self.under += 1;
        } else if v >= self.hi {
            self.over += 1;
        } else {
            let t = (v - self.lo) / (self.hi - self.lo);
            self.bins[(t * BINS as f64) as usize] += 1;
        }
    }

    /// Merges another distribution over the same range — the checks accumulate
    /// per tile and reduce at the end.
    pub fn merge(&mut self, other: &Dist) {
        debug_assert!((self.lo - other.lo).abs() < 1e-9 && (self.hi - other.hi).abs() < 1e-9);
        for (a, b) in self.bins.iter_mut().zip(other.bins.iter()) {
            *a += *b;
        }
        self.under += other.under;
        self.over += other.over;
        self.n += other.n;
        self.sum += other.sum;
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }

    pub fn count(&self) -> u64 {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Smallest sample, or `None` when nothing was measured. Exact.
    pub fn min(&self) -> Option<f64> {
        (self.n > 0).then_some(self.min)
    }

    /// Largest sample, or `None` when nothing was measured. Exact.
    pub fn max(&self) -> Option<f64> {
        (self.n > 0).then_some(self.max)
    }

    pub fn mean(&self) -> Option<f64> {
        (self.n > 0).then(|| self.sum / self.n as f64)
    }

    /// The `p`-quantile (`p` in `[0, 1]`), to bin resolution. Returns the exact
    /// extreme when the quantile falls in an out-of-range bucket, so a
    /// percentile never reads *less* extreme than the truth.
    pub fn quantile(&self, p: f64) -> Option<f64> {
        if self.n == 0 {
            return None;
        }
        let target = (p * self.n as f64).ceil().max(1.0) as u64;
        if target <= self.under {
            return Some(self.min);
        }
        let mut seen = self.under;
        for (i, &c) in self.bins.iter().enumerate() {
            seen += c;
            if seen >= target {
                let w = (self.hi - self.lo) / BINS as f64;
                return Some(self.lo + (i as f64 + 0.5) * w);
            }
        }
        Some(self.max)
    }

    /// How many samples fall below `t`. Exact for `t` outside the binned range;
    /// to bin resolution inside it.
    pub fn count_below(&self, t: f64) -> u64 {
        if t <= self.lo {
            return if t <= self.min { 0 } else { self.under };
        }
        let mut seen = self.under;
        let w = (self.hi - self.lo) / BINS as f64;
        for (i, &c) in self.bins.iter().enumerate() {
            if self.lo + (i as f64 + 1.0) * w > t {
                break;
            }
            seen += c;
        }
        seen
    }

    /// How many samples fall above `t`.
    pub fn count_above(&self, t: f64) -> u64 {
        self.n - self.count_below(t) - self.count_at_or_between(t)
    }

    /// Samples in the single bin straddling `t`; they are neither clearly below
    /// nor clearly above at bin resolution.
    fn count_at_or_between(&self, t: f64) -> u64 {
        if t < self.lo || t >= self.hi {
            return 0;
        }
        let w = (self.hi - self.lo) / BINS as f64;
        let i = ((t - self.lo) / w) as usize;
        self.bins.get(i).copied().unwrap_or(0)
    }

    /// The fraction of samples below `t`, as a percentage.
    pub fn pct_below(&self, t: f64) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        100.0 * self.count_below(t) as f64 / self.n as f64
    }
}

impl fmt::Debug for Dist {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.n == 0 {
            return write!(f, "Dist(empty)");
        }
        write!(
            f,
            "Dist(n={} min={:.3} p50={:.3} max={:.3})",
            self.n,
            self.min,
            self.quantile(0.5).unwrap_or(f64::NAN),
            self.max
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_reports_nothing_rather_than_zero() {
        let d = Dist::metres();
        assert!(d.is_empty());
        assert_eq!(d.min(), None);
        assert_eq!(d.max(), None);
        assert_eq!(d.quantile(0.5), None);
    }

    #[test]
    fn extremes_are_exact_not_binned() {
        let mut d = Dist::metres();
        d.push(-4.2001);
        d.push(0.0);
        d.push(13.37);
        assert_eq!(d.min(), Some(-4.2001));
        assert_eq!(d.max(), Some(13.37));
        assert_eq!(d.count(), 3);
    }

    #[test]
    fn extremes_survive_samples_outside_the_binned_range() {
        // A phantom viaduct at +243 m must be reported at +243 m, not clamped
        // to the histogram edge — the number is the whole point.
        let mut d = Dist::metres();
        d.push(243.0);
        d.push(-100.0);
        assert_eq!(d.max(), Some(243.0));
        assert_eq!(d.min(), Some(-100.0));
        assert_eq!(d.quantile(1.0), Some(243.0));
        assert_eq!(d.quantile(0.0), Some(-100.0));
    }

    #[test]
    fn quantiles_track_a_known_ramp() {
        let mut d = Dist::new(0.0, 100.0);
        for i in 0..1000 {
            d.push(i as f64 / 10.0);
        }
        let p50 = d.quantile(0.5).unwrap();
        assert!((p50 - 50.0).abs() < 0.2, "p50 {p50}");
        let p99 = d.quantile(0.99).unwrap();
        assert!((p99 - 99.0).abs() < 0.2, "p99 {p99}");
    }

    #[test]
    fn counts_below_a_threshold_are_the_violation_tally() {
        let mut d = Dist::metres();
        for v in [-3.0, -1.0, -0.2, 0.0, 0.5, 2.0] {
            d.push(v);
        }
        assert_eq!(d.count_below(-0.05), 3);
        assert_eq!(d.pct_below(-0.05), 50.0);
    }

    #[test]
    fn merge_is_addition() {
        let mut a = Dist::metres();
        let mut b = Dist::metres();
        a.push(1.0);
        a.push(2.0);
        b.push(-5.0);
        a.merge(&b);
        assert_eq!(a.count(), 3);
        assert_eq!(a.min(), Some(-5.0));
        assert_eq!(a.max(), Some(2.0));
    }

    #[test]
    fn nan_costs_one_sample_not_the_measurement() {
        let mut d = Dist::metres();
        d.push(1.0);
        d.push(f64::NAN);
        assert_eq!(d.count(), 1);
        assert_eq!(d.max(), Some(1.0));
    }
}
