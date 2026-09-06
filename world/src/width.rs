//! The cross-section function: what a way is, and how wide it is.
//!
//! One function, read once per way by the reader and carried on the line —
//! the plan preview's stroke, the ribbon, the kerb's index and the fillet
//! all read that one number — so no two consumers can disagree about where
//! a kerb is (docs/ROADS.md invariant 1).
//!
//! The width is the measured `width_rules` where a segment has one, else a
//! class prior narrowed for a one-way carriageway. The priors were set
//! against the mapped widths of the Swiss extract (2026-09-06, 2.94 M road
//! segments, `width_rules` on a few percent of them): per class, the median
//! of the dominant rule, leaning to the upper quartile where the sample is
//! large, because people tag a width when it is notable and the sample runs
//! narrow. Measured medians, one-way in brackets:
//!
//! ```text
//! motorway 7.5 (p75 11)   trunk 8.0        primary 7.0 (5.0)   secondary 6.0 (5.0)
//! tertiary 6.0 (4.4)      residential 4.9, p75 5.5 (5.0)       unclassified 4.0 (4.0)
//! living_street 4.5 (4.1) service 3.0      parking_aisle 4.0   link 5.0
//! pedestrian 3.5, p75 5   track 2.5        footway 1.5, p75 2  sidewalk 1.5, p75 2
//! path 1.0                steps 1.5        cycleway 2.0        crosswalk 3.0
//! ```
//!
//! The motorway prior stays above its median: a tagged `width` is the
//! lanes, and a Swiss carriageway of two 3.75 m lanes and a shoulder is
//! wider than the 7.5 m the tags say.
//!
//! A way also belongs to a [`Family`]: the surfaces that union into one
//! region. Asphalt joins asphalt at a junction; a pavement joins a pavement
//! round a corner; the two never merge, and where they overlap the asphalt
//! wins.

/// The surface a way is part of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Family {
    /// The drivable surface: every road class, however minor.
    Carriageway,
    /// Where people walk: sidewalks, paths, steps, crossings, cycleways.
    Walk,
}

impl Family {
    pub fn name(self) -> &'static str {
        match self {
            Family::Carriageway => "carriageway",
            Family::Walk => "walk",
        }
    }
}

/// Width in metres of a sidewalk or a cycle track: the Swiss norm's 2 m,
/// the mapped upper quartile.
pub const WALK_M: f64 = 2.0;

/// Width in metres of a service way — a driveway, an alley: one car's track
/// plus margins.
pub const SERVICE_M: f64 = 3.0;

/// Width in metres of a ramp (`subclass = link`), whatever class it carries:
/// a single lane plus shoulders.
pub const LINK_M: f64 = 5.0;

/// A measured width outside this range, in metres, is a typo (a whole
/// right-of-way on a footpath, a lane count in the width field) and the
/// prior stands.
pub const MEASURED_M: std::ops::RangeInclusive<f64> = 1.0..=30.0;

/// The full width in metres of a way of `class` and `subclass`: the prior
/// for a two-way carriageway. Every class [`crate::roads`] admits has an
/// answer; an unknown one gets a service way's, which is the narrowest
/// thing a car uses.
pub fn of(class: &str, subclass: &str) -> f64 {
    if subclass == "link" && family(class) == Family::Carriageway {
        return LINK_M;
    }
    match (class, subclass) {
        ("motorway", _) => 9.0,
        ("trunk", _) => 8.0,
        ("primary", _) => 7.0,
        ("secondary", _) | ("tertiary", _) => 6.0,
        ("residential", _) => 5.5,
        ("unclassified", _) | ("living_street", _) => 4.5,
        ("service", "parking_aisle") => 4.0,
        // A pedestrianised street: a lane's width with a pavement's use.
        ("pedestrian", _) => 4.0,
        ("footway", "crosswalk") => 3.0,
        ("footway", _) | ("cycleway", _) => WALK_M,
        ("track", _) => 2.5,
        ("steps", _) | ("bridleway", _) => 1.5,
        ("path", _) => 1.2,
        _ => SERVICE_M,
    }
}

/// The width of one way: `measured` when the segment carries a sane
/// `width_rules`, else the prior, narrowed when `oneway` on the classes
/// where the mapped one-way widths sit well below the two-way ones.
pub fn of_way(class: &str, subclass: &str, oneway: bool, measured: Option<f64>) -> f64 {
    if let Some(w) = measured.filter(|w| MEASURED_M.contains(w)) {
        return w;
    }
    let prior = of(class, subclass);
    if !oneway || subclass == "link" {
        return prior;
    }
    match class {
        "primary" | "secondary" => 5.0,
        "tertiary" => 4.5,
        "unclassified" | "living_street" => 4.0,
        _ => prior,
    }
}

/// The kerb-return radius in metres at a junction whose widest leg is of
/// `class`: what a vehicle turning out of it needs. A prior; nothing in the
/// data says it.
pub fn fillet_m(class: &str) -> f64 {
    match class {
        "motorway" | "trunk" | "primary" | "secondary" => 8.0,
        "tertiary" | "residential" | "unclassified" | "living_street" => 4.0,
        _ => 3.0,
    }
}

/// The family of a way of `class`.
pub fn family(class: &str) -> Family {
    match class {
        "pedestrian" | "footway" | "path" | "steps" | "cycleway" | "bridleway" | "track" => {
            Family::Walk
        }
        _ => Family::Carriageway,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_link_is_one_lane_whatever_its_class() {
        assert_eq!(of("motorway", "link"), LINK_M);
        assert_eq!(of("primary", "link"), LINK_M);
        assert_eq!(of("primary", ""), 7.0);
        // A footway tagged link is still a footway.
        assert_eq!(of("footway", "link"), WALK_M);
    }

    #[test]
    fn measured_beats_the_prior_and_oneway_narrows_it() {
        assert_eq!(of_way("primary", "", false, None), 7.0);
        assert_eq!(of_way("primary", "", true, None), 5.0);
        assert_eq!(of_way("primary", "", true, Some(8.2)), 8.2);
        assert_eq!(of_way("primary", "link", true, None), LINK_M);
        // A residential one-way is no narrower than a two-way one in the data.
        assert_eq!(of_way("residential", "", true, None), 5.5);
        // A typo'd width is ignored.
        assert_eq!(of_way("footway", "", false, Some(0.3)), WALK_M);
        assert_eq!(of_way("footway", "", false, Some(45.0)), WALK_M);
        assert_eq!(of_way("footway", "sidewalk", false, Some(1.4)), 1.4);
    }

    #[test]
    fn every_admitted_class_has_a_width_and_a_family() {
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
            "footway",
            "steps",
            "path",
            "track",
            "cycleway",
            "bridleway",
            "unknown",
        ] {
            assert!(of(class, "") > 0.0, "{class}");
        }
        assert_eq!(family("residential"), Family::Carriageway);
        assert_eq!(family("footway"), Family::Walk);
        assert_eq!(family("service"), Family::Carriageway);
    }
}
