//! The pipeline's steps, in order, and what each one reports.

use std::fmt;
use std::str::FromStr;

use crate::world::{Profile, Solved};

/// One step of the pipeline.
///
/// Each is a function of the layers it reads, returning what it makes
/// (`fn run(inputs…) -> (Layer, Summary)`). Most make one layer. Three make
/// something the pipeline installs into a layer built before them — the
/// reference's terrain priors, the crossing's re-solved profiles, the
/// partition's cut — and they *return* it rather than writing it, so a
/// step's signature is still the whole of its interface.
///
/// The order is [`Step::ALL`]'s alone. It is deliberately not written
/// anywhere else: the modules used to open with "Step 7" in their headers
/// and eleven of the seventeen then had drifted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// The terrain mesh over the bbox, from the DEM.
    Terrain,
    /// Way centrelines read from the transportation source, draped exactly
    /// onto the terrain mesh.
    Drape,
    /// The surface a way is solved against: the terrain with its blind runs
    /// bridged, its narrow notches filled and its narrow bumps shaved, and
    /// the runs the two passes refused — the terrain's own structure priors.
    Reference,
    /// One height along every carriageway axis: the ground, grade-limited
    /// and boxed per class, chorded across the mapped spans.
    Profile,
    /// The clearances a grade separation demands, spread along the network
    /// as a floor the profile re-solves on.
    Crossing,
    /// Where each way is a deck, a bore, or on the ground — and the one place
    /// the geometry is cut. Runs after the heights are solved, because that
    /// is what it reads.
    Partition,
    /// Building footprints read from the buildings source: what nothing
    /// paved may enter, less the passages ways run through.
    Facade,
    /// Every way buffered to its width: one polygon each, unmerged.
    Ribbon,
    /// The ribbons unioned per family, the asphalt subtracted from the walk.
    Surface,
    /// Sidewalks attached to their streets, the strip to the kerb filled.
    Kerb,
    /// The junctions built from the legs meeting there: the carriageway as
    /// one polygon per junction and the trimmed edges between them, with the
    /// kerb returns (`docs/plans/plan-chain-from-legs.md`, first slice).
    Legs,
    /// The room between the facades filled: pavement from the kerb to
    /// every wall within reach.
    Room,
    /// The paved surface partitioned into the sheets that may merge: one
    /// per connected group of pieces, so a junction and the structure it
    /// stands on are one surface and a viaduct is not the street below it.
    Sheet,
    /// The rect as one planar subdivision: every material boundary cut at
    /// once, every face tagged. One set of split points, so the ground and
    /// the paving beside it share their boundary rather than each computing
    /// it (`data/plans/one-ground-2026-09-16.md` step 1).
    Arrangement,
    /// The paved surface as triangles, each inside one terrain triangle,
    /// on the ground.
    Mesh,
    /// The room lifted off the ground onto the height its profile solved.
    Lift,
    /// The ground benched to the lifted room, and the footpaths regraded
    /// onto it.
    Earthwork,
    /// The faces that close the room onto the ground and onto itself: the
    /// walls and the kerbs.
    Bench,
    /// The decks and bores the solved profile implies, and the roadway
    /// over every span.
    Structure,
    /// Every building stood on the ground, walled to its roof, and roofed.
    Building,
}

impl Step {
    /// Every step, in the order the pipeline runs them.
    pub const ALL: [Step; 20] = [
        Step::Terrain,
        Step::Drape,
        Step::Reference,
        Step::Profile,
        Step::Crossing,
        Step::Partition,
        Step::Facade,
        Step::Ribbon,
        Step::Surface,
        Step::Kerb,
        Step::Legs,
        Step::Room,
        Step::Sheet,
        Step::Arrangement,
        Step::Mesh,
        Step::Lift,
        Step::Earthwork,
        Step::Bench,
        Step::Structure,
        Step::Building,
    ];

    /// The name the CLI prints and `--until` accepts.
    pub fn name(self) -> &'static str {
        match self {
            Step::Terrain => "terrain",
            Step::Drape => "drape",
            Step::Reference => "reference",
            Step::Profile => "profile",
            Step::Crossing => "crossing",
            Step::Partition => "partition",
            Step::Facade => "facade",
            Step::Ribbon => "ribbon",
            Step::Surface => "surface",
            Step::Kerb => "kerb",
            Step::Legs => "legs",
            Step::Room => "room",
            Step::Sheet => "sheet",
            Step::Arrangement => "arrangement",
            Step::Mesh => "mesh",
            Step::Lift => "lift",
            Step::Earthwork => "earthwork",
            Step::Bench => "bench",
            Step::Structure => "structure",
            Step::Building => "building",
        }
    }
}

impl FromStr for Step {
    type Err = String;

    fn from_str(s: &str) -> Result<Step, String> {
        Step::ALL
            .into_iter()
            .find(|step| step.name() == s)
            .ok_or_else(|| {
                let names: Vec<&str> = Step::ALL.iter().map(|s| s.name()).collect();
                format!("unknown step `{s}` (expected one of: {})", names.join(", "))
            })
    }
}

/// What a step reports when it is done: a few labelled counts, printed on
/// one line beside the step's name and elapsed time.
#[derive(Debug, Default, Clone)]
pub struct Summary {
    pub counts: Vec<(&'static str, String)>,
}

impl Summary {
    pub fn new() -> Summary {
        Summary::default()
    }

    /// This line followed by `other`'s: a step's build tallies and then
    /// what its check measured of the layer.
    pub fn and(mut self, other: Summary) -> Summary {
        self.counts.extend(other.counts);
        self
    }

    /// Appends one labelled value.
    pub fn with(mut self, label: &'static str, value: impl fmt::Display) -> Summary {
        self.counts.push((label, value.to_string()));
        self
    }

    /// An area, to the square metre.
    pub fn with_m2(self, label: &'static str, m2: f64) -> Summary {
        self.with(label, format!("{m2:.0}"))
    }

    /// A set of regions, as `regions/holes`.
    pub fn with_regions(self, label: &'static str, shapes: &crate::poly::Shapes) -> Summary {
        self.with(label, format!("{}/{}", shapes.len(), crate::poly::holes(shapes)))
    }

    /// A count that is part of a whole, as `n (p%)`.
    pub fn with_part(self, label: &'static str, n: usize, of: usize) -> Summary {
        self.with(label, format!("{n} ({:.1}%)", pct(n, of)))
    }

    /// A count of failures out of the checks made, as `n/of (p%)`.
    pub fn with_share(self, label: &'static str, n: usize, of: usize) -> Summary {
        self.with(label, format!("{n}/{of} ({:.2}%)", pct(n, of)))
    }

    /// How far this step's own output stands off the raw DEM, as
    /// `p50/p90/max`. See [`Residual`]: the label is always `dem_residual`,
    /// because one quantity reported under one name down the whole run is
    /// what makes a height attributable to the step that made it.
    pub fn with_residual(self, residual: Residual) -> Summary {
        self.with_quantiles("dem_residual", residual)
    }

    /// A population of distances as `p50/p90/max`, or `-` when it is empty.
    pub fn with_quantiles(self, label: &'static str, mut residual: Residual) -> Summary {
        match residual.quantiles() {
            Some((p50, p90, max)) => self.with(label, format!("{p50:.2}/{p90:.2}/{max:.2}")),
            None => self.with(label, "-"),
        }
    }

    /// The value reported under `label`, if any.
    pub fn get(&self, label: &str) -> Option<&str> {
        self.counts.iter().find(|(l, _)| *l == label).map(|(_, v)| v.as_str())
    }

    /// The leading number of the value under `label`: the count of an
    /// `n/of`, the area of an `m2`.
    #[cfg(test)]
    pub fn num(&self, label: &str) -> f64 {
        let v = self.get(label).unwrap_or_else(|| panic!("no `{label}` in {self}"));
        let end = v.find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e'))).unwrap_or(v.len());
        v[..end].parse().unwrap_or_else(|_| panic!("`{label}={v}` is not a number"))
    }
}

/// How far a step has moved the world off the **raw DEM**, in metres.
///
/// One quantity with one name. It used to be two: `off` meant "distance from
/// the raw DEM" in `reference` and `profile` and "how far a lattice triangle
/// stands off the engineered ground" in `bench`, so a reader had to know which
/// step's line they were on before they knew what they were reading.
///
/// **The baseline is the same for every step** — [`crate::lattice::height_at`],
/// never the step's own input — which is the whole point: it makes the lines
/// comparable down a run, so a height can be attributed to the step that made
/// it rather than to the step that paid for it. The numbers are absolute
/// rather than per-step increments, because a step that *undoes* its
/// predecessor's move shows as a smaller number and an increment would hide
/// that. `reference` moving the surface 5.81 m and `profile` landing 0.48 m
/// from the DEM at p90 is the fact that wanted reporting, and no pair of
/// increments states it.
///
/// Reported as `p50/p90/max`, or `-` when the step moved nothing.
#[derive(Debug, Default, Clone)]
pub struct Residual(Vec<f64>);

impl Residual {
    pub fn new() -> Residual {
        Residual::default()
    }

    /// One sample: a height the world draws, and the raw DEM beneath it.
    pub fn push(&mut self, drawn: f64, dem: f64) {
        self.0.push((drawn - dem).abs());
    }

    /// The quantiles, sorted in place. `f` is a fraction of the population:
    /// 1.0 is the maximum.
    fn quantiles(&mut self) -> Option<(f64, f64, f64)> {
        if self.0.is_empty() {
            return None;
        }
        self.0.sort_by(|a, b| a.partial_cmp(b).expect("a residual is finite"));
        let q = |f: f64| self.0[((self.0.len() as f64 - 1.0) * f).round() as usize];
        Some((q(0.5), q(0.9), q(1.0)))
    }
}

fn pct(n: usize, of: usize) -> f64 {
    if of == 0 {
        0.0
    } else {
        100.0 * n as f64 / of as f64
    }
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, (label, value)) in self.counts.iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{label}={value}")?;
        }
        Ok(())
    }
}

/// The solved surface against the raw DEM, over every **at-grade** station of
/// every profile: [`crate::step::Residual`], read off the profiles a step
/// hands on.
///
/// The population is `Solved::Grade` alone, and that is the whole reason this
/// is one function rather than a loop in each step. A chord standing thirty
/// metres over a gorge is not a departure from the ground — it is a bridge,
/// and the structure step answers for it — so counting it would swamp the
/// number that matters with the number that does not. It is also what the
/// bench actually benches, which is what makes the lines comparable: every
/// step reports the same quantity over the same population against the same
/// baseline, so the differences down a run attribute a height to the step
/// that made it.
pub fn residual_of(profiles: &[Profile]) -> Residual {
    let mut r = Residual::new();
    for p in profiles {
        for st in p.stations.iter().filter(|st| st.solved == Solved::Grade) {
            r.push(st.h, st.ground);
        }
    }
    r
}
