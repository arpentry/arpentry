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
    /// Every way buffered to its width: one polygon each, unmerged.
    Ribbon,
    /// The ribbons unioned per family, the asphalt subtracted from the walk.
    Surface,
    /// Sidewalks attached to their streets, the strip to the kerb filled.
    Kerb,
    /// The kerb returns: junction notches rounded by a masked closing.
    Fillet,
}

impl Step {
    /// Every step, in the order the pipeline runs them.
    pub const ALL: [Step; 6] =
        [Step::Terrain, Step::Drape, Step::Ribbon, Step::Surface, Step::Kerb, Step::Fillet];

    /// The name the CLI prints and `--until` accepts.
    pub fn name(self) -> &'static str {
        match self {
            Step::Terrain => "terrain",
            Step::Drape => "drape",
            Step::Ribbon => "ribbon",
            Step::Surface => "surface",
            Step::Kerb => "kerb",
            Step::Fillet => "fillet",
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
