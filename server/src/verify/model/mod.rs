//! The half of the scorecard the archive cannot answer.
//!
//! `verify::checks` measures the emitted `.arpa`, which is the right place for
//! every relation between two drawn surfaces. But three of the invariants are
//! not about the output at all — they are about *how it was computed*, and no
//! amount of geometry can distinguish a scene where authority held from one
//! where it was violated and the numbers happened to come out plausible.
//!
//! docs/GENERATION.md §8 is explicit about the difference:
//!
//! > I7 and I8 are *structural* claims: they are established by construction
//! > and falsifiable by a single perturbation experiment, not sampled by a
//! > metric.
//!
//! So these run against the model, in process, and emit [`Metric`]s in the same
//! shape as the archive checks so one scorecard and one baseline diff covers
//! both. A perturbation experiment is not a distribution — its honest reading is
//! a count of features that moved when nothing should have — so the "worst"
//! column carries the largest movement, which is the number to go and look at.

pub mod authority;
pub mod datum;
pub mod footprint;
pub mod structures;

use crate::ground::GroundStack;
use crate::scene::SceneGraph;
use crate::solve::SolvedModel;
use crate::verify::Metric;

/// What a model check is given: the assembled scene, what the solve made of it,
/// the ground that fell out, and the DEM to re-run against.
pub struct Model<'a> {
    pub scene: &'a SceneGraph,
    pub solved: &'a SolvedModel,
    pub ground: &'a GroundStack,
    pub terrain: Option<&'a std::path::Path>,
    pub threads: usize,
}

/// Runs every model check.
pub fn run(m: &Model<'_>) -> Vec<Metric> {
    let mut out = Vec::new();
    out.extend(authority::determinism(m));
    out.extend(authority::inversion(m));
    out.extend(datum::check(m));
    out.extend(footprint::check(m));
    out.extend(structures::check(m));
    out
}

/// Serializes model metrics into the same JSON the archive scorecard writes, so
/// `arpentry_verify --model` can merge them and the baseline diff joins on id
/// without knowing which half a metric came from.
pub fn to_json(metrics: &[Metric]) -> serde_json::Value {
    serde_json::json!({
        "metrics": metrics.iter().map(crate::verify::report::metric_json).collect::<Vec<_>>(),
    })
}
