//! The pipeline's steps, in order, and what each one reports.

use std::fmt;
use std::str::FromStr;

/// **The pipeline, once**: every step in the order it runs, the field of
/// [`crate::world::World`] its layer lands in, and the layer's type.
///
/// Everything that has to agree about the steps is expanded from this one
/// list — the [`Step`] enum and its order, the world's fields, the pipeline's
/// accessors and the dump's save and load — so none of them can disagree
/// with another about a step. A layer's field is named after its step.
///
/// Called with the name of a macro, which it invokes over the whole table.
macro_rules! steps {
    ($then:ident) => {
        $then! {
            /// The terrain mesh over the bbox, from the DEM.
            Terrain => terrain: crate::world::Terrain,
            /// Way centrelines read from the transportation source, draped
            /// exactly onto the terrain mesh.
            Drape => drape: crate::world::Roads,
            /// The surface a way is solved against: the terrain with its
            /// blind runs bridged, its narrow notches filled and its narrow
            /// bumps shaved, and the runs the two passes refused — the
            /// terrain's own structure priors.
            Reference => reference: crate::world::Reference,
            /// One height along every solving way (the carriageways and the
            /// railways): the reference, grade-limited and boxed per class,
            /// chorded across the mapped spans.
            Profile => profile: crate::world::Profiles,
            /// The clearances a grade separation demands, spread along the
            /// network as a floor the profile re-solves on.
            Crossing => crossing: crate::world::Crossings,
            /// Where each way is a deck, a bore, or on the ground — and the
            /// one place the geometry is cut.
            Partition => partition: crate::world::Partition,
            /// Building footprints read from the buildings source: what
            /// nothing paved may enter, less the passages ways run through.
            Facade => facade: crate::world::Facade,
            /// Every way buffered to its width: one polygon each, unmerged.
            Ribbon => ribbon: crate::world::Ribbons,
            /// The ribbons unioned per family, the asphalt subtracted from
            /// the walk.
            Surface => surface: crate::world::Surface,
            /// Sidewalks attached to their streets, the strip to the kerb
            /// filled.
            Kerb => kerb: crate::world::Kerb,
            /// The junctions built from the legs meeting there: the
            /// carriageway as one polygon per junction and the trimmed edges
            /// between them, with the kerb returns.
            Legs => legs: crate::world::Legs,
            /// The room between the facades filled: pavement from the kerb
            /// to every wall within reach.
            Room => room: crate::world::Room,
            /// The paved surface partitioned into the sheets that may merge:
            /// one per connected group of pieces, so a junction and the
            /// structure it stands on are one surface and a viaduct is not
            /// the street below it.
            Sheet => sheet: crate::world::Sheets,
            /// The rect as one planar subdivision: every material boundary
            /// cut at once, every face tagged, so the ground and the paving
            /// beside it share their boundary.
            Arrangement => arrangement: crate::world::Arrangement,
            /// The whole rect as one triangulation of the arrangement's
            /// faces, every triangle inside one terrain triangle, on the
            /// ground.
            Mesh => mesh: crate::world::Mesh,
            /// The room lifted off the ground onto the height its profile
            /// solved.
            Lift => lift: crate::world::Lifted,
            /// The ground benched to the lifted room, and the footpaths
            /// regraded onto it.
            Earthwork => earthwork: crate::world::Earthwork,
            /// The faces that close the room onto the ground and onto
            /// itself: the walls, the kerbs and the splits.
            Bench => bench: crate::world::Bench,
            /// The decks and bores the solved profile implies, and the
            /// paving no sheet lays: a bore's floor and a walk span.
            Structure => structure: crate::world::Structure,
            /// Every building stood on the ground, walled to its roof, and
            /// roofed.
            Building => building: crate::world::Buildings,
        }
    };
}
pub(crate) use steps;

/// Expands the [`Step`] enum from the table.
macro_rules! step_enum {
    ($($(#[$doc:meta])* $step:ident => $field:ident: $layer:ty,)*) => {
        /// One step of the pipeline: a function of the layers it reads,
        /// returning the one it makes (`fn run(inputs…) -> (Layer,
        /// Summary)`). Which layer feeds which step is
        /// [`crate::pipeline`]'s; the order is [`Step::ALL`]'s.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
        pub enum Step {
            $($(#[$doc])* $step,)*
        }

        impl Step {
            /// Every step, in the order the pipeline runs them.
            pub const ALL: &'static [Step] = &[$(Step::$step,)*];

            /// The name the CLI prints and `--until` accepts: the name of the
            /// step's module and of its layer's field.
            pub fn name(self) -> &'static str {
                match self {
                    $(Step::$step => stringify!($field),)*
                }
            }
        }
    };
}
steps!(step_enum);

impl FromStr for Step {
    type Err = String;

    fn from_str(s: &str) -> Result<Step, String> {
        Step::ALL
            .iter()
            .copied()
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
/// One quantity with one name, whichever step reports it.
///
/// **The baseline is the same for every step** — [`crate::lattice::height_at`],
/// never the step's own input — which is the whole point: it makes the lines
/// comparable down a run, so a height can be attributed to the step that made
/// it rather than to the step that paid for it. The numbers are absolute
/// rather than per-step increments, because a step that *undoes* its
/// predecessor's move shows as a smaller number and an increment would hide
/// that.
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

    /// The median, the 90th percentile and the maximum, sorting the samples
    /// in place.
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
