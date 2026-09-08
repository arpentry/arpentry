//! The vertical priors: what a class holds along its length, and how far
//! it may leave the ground to hold it.
//!
//! One table, keyed on class, beside [`crate::width`]'s. Every height the
//! world solves reads it here and nowhere else (docs/GENERATION.md §9). The
//! values are the server's, which were set against Swiss norms and a summer
//! of measuring the Montreux extract:
//!
//! - **Engineered** classes hold a true engineering grade — a motorway is
//!   built to 6 %, a primary to 8 % — and may leave the ground by metres to
//!   hold it: a cutting, an embankment, and beyond that a structure.
//! - **Street** classes are not limited at all: the DEM under a street *is*
//!   the street, and a lane mapped at 20 % up the old town really climbs
//!   20 % (S9: knowing when to do nothing). Held to a 15 % ceiling inside
//!   a 2.5 m box, a street on a steeper hill spent the whole box before it
//!   broke grade and drew every block in two metres of fill. Their ceiling
//!   is the ramp grade a *lift* may use — an overpass approach, a crossing's
//!   clearance — and their box is what such a lift may spend; neither
//!   touches a street the ground already carries.
//! - **Draped** classes hold nothing: a footpath, steps, a track sample the
//!   finished ground exactly and never solve (stratum D). Their entry is
//!   here so that the question "does this class solve?" has one answer.

/// How a class's alignment behaves along its length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// A surveyed grade line: a low ceiling and a wide deviation budget.
    Engineered,
    /// A street on whatever hill it sits on: the ground, exactly. Its
    /// ceiling and box bound the lifts later steps may ask of it.
    Street,
    /// No profile: the way samples the ground.
    Draped,
}

/// The vertical prior of one class.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grade {
    pub mode: Mode,
    /// The steepest rise over run the alignment holds where it can — a
    /// limit on an engineered class, a ramp grade for a street's lifts;
    /// `None` for a draped class.
    pub ceiling: Option<f64>,
    /// How far, in metres, the solved height may stand from the ground at
    /// an at-grade station; 0 for a draped class.
    pub deviation_m: f64,
}

impl Grade {
    /// Whether the class solves a profile at all.
    pub fn solves(&self) -> bool {
        self.mode != Mode::Draped
    }

    /// Whether the class's grade is limited against the ground.
    pub fn limited(&self) -> bool {
        self.mode == Mode::Engineered
    }
}

/// Station spacing along an axis, in metres: the profile's resolution.
/// Fine enough that a 4 m node step at the class ceiling is a quarter of
/// a metre; coarse enough that the box's 1 200 road ways are 50 k stations.
pub const NODE_M: f64 = 4.0;

/// The height off the ground, in metres, at which a station of a mapped
/// structure span is a deck (above) or a bore (below) rather than grade.
/// Below it the annotation degrades to ground: a structure that never
/// leaves the ground by half a metre is a culvert, and a culvert is ground.
pub const STRUCTURE_MIN_M: f64 = 0.5;

/// The vertical prior of a way of `class`.
pub fn of(class: &str) -> Grade {
    match class {
        "motorway" | "trunk" => Grade { mode: Mode::Engineered, ceiling: Some(0.06), deviation_m: 8.0 },
        "primary" => Grade { mode: Mode::Engineered, ceiling: Some(0.08), deviation_m: 4.0 },
        "secondary" => Grade { mode: Mode::Engineered, ceiling: Some(0.10), deviation_m: 4.0 },
        "tertiary" | "unclassified" | "residential" | "living_street" | "service" | "pedestrian"
        | "unknown" => Grade { mode: Mode::Street, ceiling: Some(0.15), deviation_m: 2.5 },
        _ => Grade { mode: Mode::Draped, ceiling: None, deviation_m: 0.0 },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_admitted_class_has_a_mode() {
        for class in [
            "motorway",
            "trunk",
            "primary",
            "secondary",
            "tertiary",
            "unclassified",
            "residential",
            "living_street",
            "service",
            "pedestrian",
            "unknown",
        ] {
            let g = of(class);
            assert!(g.solves(), "{class}");
            assert!(g.ceiling.is_some() && g.deviation_m > 0.0, "{class}");
        }
        for class in ["footway", "steps", "path", "track", "cycleway", "bridleway"] {
            let g = of(class);
            assert!(!g.solves(), "{class}");
            assert_eq!((g.ceiling, g.deviation_m), (None, 0.0), "{class}");
        }
    }

    #[test]
    fn the_ladder_is_ordered() {
        // The stiffer the class, the lower its ceiling and the wider its box.
        let (m, p, r) = (of("motorway"), of("primary"), of("residential"));
        assert!(m.ceiling < p.ceiling && p.ceiling < r.ceiling);
        assert!(m.deviation_m > p.deviation_m && p.deviation_m > r.deviation_m);
        assert_eq!(m.mode, Mode::Engineered);
        assert_eq!(r.mode, Mode::Street);
        assert!(m.limited() && p.limited() && !r.limited() && !of("footway").limited());
    }
}
