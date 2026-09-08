//! The pipeline's steps, in order, and what each one reports.

use std::fmt;
use std::str::FromStr;

/// One step of the pipeline. Each adds exactly one layer to the world.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// The terrain mesh over the bbox, from the DEM.
    Terrain,
    /// Way centrelines read from the transportation source, draped exactly
    /// onto the terrain mesh.
    Drape,
    /// One height along every carriageway axis: the ground, grade-limited
    /// and boxed per class, chorded across the mapped spans.
    Profile,
    /// Building footprints read from the buildings source: what nothing
    /// paved may enter, less the passages ways run through.
    Facade,
    /// Every way buffered to its width: one polygon each, unmerged.
    Ribbon,
    /// The ribbons unioned per family, the asphalt subtracted from the walk.
    Surface,
    /// Sidewalks attached to their streets, the strip to the kerb filled.
    Kerb,
    /// The kerb returns: junction notches rounded by a masked closing.
    Fillet,
    /// The room between the facades filled: pavement from the kerb to
    /// every wall within reach.
    Room,
    /// The paved surface as triangles, each inside one terrain triangle,
    /// on the ground.
    Mesh,
    /// The room lifted off the ground onto the height its profile solved.
    Bench,
}

impl Step {
    /// Every step, in the order the pipeline runs them.
    pub const ALL: [Step; 11] = [
        Step::Terrain,
        Step::Drape,
        Step::Profile,
        Step::Facade,
        Step::Ribbon,
        Step::Surface,
        Step::Kerb,
        Step::Fillet,
        Step::Room,
        Step::Mesh,
        Step::Bench,
    ];

    /// The name the CLI prints and `--until` accepts.
    pub fn name(self) -> &'static str {
        match self {
            Step::Terrain => "terrain",
            Step::Drape => "drape",
            Step::Profile => "profile",
            Step::Facade => "facade",
            Step::Ribbon => "ribbon",
            Step::Surface => "surface",
            Step::Kerb => "kerb",
            Step::Fillet => "fillet",
            Step::Room => "room",
            Step::Mesh => "mesh",
            Step::Bench => "bench",
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
