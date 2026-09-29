//! The cross-section function: what a way is, and how wide it is.
//!
//! One function, read once per way by the reader and carried on the line —
//! the plan preview's stroke, the ribbon, the kerb's index and the fillet
//! all read that one number — so no two consumers can disagree about where
//! a kerb is (docs/ROADS.md invariant 1).
//!
//! A carriageway's width is the measured `width_rules` where a segment has
//! one, else a class prior narrowed for a one-way. A walk's width is one
//! constant, [`WALK_M`], whatever the class and whatever is mapped: the
//! measured value covers 7 of the 1344 walk ways in the Montreux loop box,
//! and every difference in width between two walks that meet is a shoulder
//! in their union (a 3 m crossing ending on a 2 m sidewalk pokes its cap
//! out the far side; a 1.2 m path joining a 2 m footway steps the outline)
//! that no reader can see the source of. One width makes the kerb offset a
//! constant too. The one exception is a pedestrianised street: an area
//! drawn as a line, kept at a lane's width. The priors were set
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
//!
//! **A railway is a third family, not a class of carriageway.** It solves a
//! profile, lays a surface and gets structures the way a road does — the
//! server learned that every mechanism that makes a road robust to a wrong
//! height was missing for rail, "and rail paid the whole price in daylight"
//! (`data/plans/rail-formation-surface.md`) — but it takes no kerb, no
//! sidewalk, no kerb return and no room, and its ballast unions with ballast
//! only. Only the independent classes are admitted (stratum R: the gauges,
//! the subway, the funicular). Street-running rail — tram, light rail,
//! monorail — lies *on* a carriageway and has no surface of its own, and
//! an `unknown` railway is not granted a formation on the strength of a
//! default (docs/GENERATION.md §4.6).

use serde::{Deserialize, Serialize};

/// The surface a way is part of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Family {
    /// The drivable surface: every road class, however minor.
    Carriageway,
    /// Where people walk: sidewalks, paths, steps, crossings, cycleways.
    Walk,
    /// The track bed of an independent railway: ballast.
    Rail,
}

impl Family {
    /// Every family, in the order their regions are indexed.
    pub const ALL: [Family; 3] = [Family::Carriageway, Family::Walk, Family::Rail];

    pub fn name(self) -> &'static str {
        match self {
            Family::Carriageway => "carriageway",
            Family::Walk => "walk",
            Family::Rail => "rail",
        }
    }

    /// Whether ways of this family solve a profile and lay a surface of
    /// their own: the carriageway and the railway, not the walk.
    pub fn solves(self) -> bool {
        self != Family::Walk
    }
}

/// The rail classes admitted: the independent ones, which hold a surveyed
/// alignment on a formation of their own.
pub const RAIL_CLASSES: &[&str] = &["standard_gauge", "broad_gauge", "subway", "narrow_gauge", "funicular"];

/// Width in metres of a standard-gauge track's drawn bed: the *track zone*,
/// a 2.6 m sleeper plus the tamped ballast shoulder, not the formation. The
/// server drew the 5 m formation first and "the railway read as wide as a
/// residential street" (`priors::MAINLINE`); the earthworks beyond the
/// track zone are the bench's.
pub const RAIL_M: f64 = 3.5;

/// The same for metre gauge and a funicular: a 1.8 m sleeper plus the same
/// shoulder.
pub const NARROW_RAIL_M: f64 = 2.6;

/// Width in metres of every walk that is not a pedestrianised street — a
/// sidewalk, a path, steps, a track, a cycle track, a crossing: the Swiss
/// norm's 2 m for a sidewalk, the mapped upper quartile.
pub const WALK_M: f64 = 2.0;

/// Width in metres of a pedestrianised street (`class = pedestrian`): a
/// lane's width with a pavement's use. An area mapped as a line, so a
/// walk's width would be visibly wrong through it.
pub const PEDESTRIAN_M: f64 = 4.0;

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
        ("standard_gauge" | "broad_gauge" | "subway", _) => RAIL_M,
        ("narrow_gauge" | "funicular", _) => NARROW_RAIL_M,
        ("motorway", _) => 9.0,
        ("trunk", _) => 8.0,
        ("primary", _) => 7.0,
        ("secondary", _) | ("tertiary", _) => 6.0,
        ("residential", _) => 5.5,
        ("unclassified", _) | ("living_street", _) => 4.5,
        ("service", "parking_aisle") => 4.0,
        ("pedestrian", _) => PEDESTRIAN_M,
        _ if family(class) == Family::Walk => WALK_M,
        _ => SERVICE_M,
    }
}

/// The measured width of a way of `class` whose segment carries
/// `width_rules`: the mapped value when it is sane and the way is a
/// carriageway. A walk's width is never measured, whatever is mapped.
pub fn measured(class: &str, width_rules: Option<f64>) -> Option<f64> {
    width_rules.filter(|w| MEASURED_M.contains(w) && family(class) == Family::Carriageway)
}

/// The width of one way: [`measured`] where there is one, else the prior,
/// narrowed when `oneway` on the classes where the mapped one-way widths
/// sit well below the two-way ones.
pub fn of_way(class: &str, subclass: &str, oneway: bool, width_rules: Option<f64>) -> f64 {
    let prior = of(class, subclass);
    if let Some(w) = measured(class, width_rules) {
        return w;
    }
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
        _ if RAIL_CLASSES.contains(&class) => Family::Rail,
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
        assert_eq!(of_way("residential", "", false, Some(0.3)), 5.5);
        assert_eq!(of_way("residential", "", false, Some(45.0)), 5.5);
        assert_eq!(of_way("residential", "", false, Some(4.2)), 4.2);
    }

    #[test]
    fn every_walk_is_one_width_but_a_pedestrian_street() {
        for (class, subclass) in [
            ("footway", ""),
            ("footway", "sidewalk"),
            ("footway", "crosswalk"),
            ("path", ""),
            ("steps", ""),
            ("track", ""),
            ("cycleway", ""),
            ("bridleway", ""),
        ] {
            assert_eq!(of(class, subclass), WALK_M, "{class} {subclass}");
            // Neither a mapped width nor oneway moves it.
            assert_eq!(of_way(class, subclass, true, Some(1.4)), WALK_M, "{class} {subclass}");
        }
        assert_eq!(of("pedestrian", ""), PEDESTRIAN_M);
        assert_eq!(of_way("pedestrian", "", false, Some(6.0)), PEDESTRIAN_M);
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

    #[test]
    fn a_railway_is_its_own_family_at_its_track_zone() {
        for class in RAIL_CLASSES {
            assert_eq!(family(class), Family::Rail, "{class}");
            assert!(family(class).solves(), "{class}");
            // A mapped width is a carriageway's alone, and a railway is not
            // one-way narrowed: the track zone is the track zone.
            assert_eq!(of_way(class, "", true, Some(8.0)), of(class, ""), "{class}");
        }
        assert_eq!(of("standard_gauge", ""), RAIL_M);
        assert_eq!(of("narrow_gauge", ""), NARROW_RAIL_M);
        assert_eq!(of("funicular", ""), NARROW_RAIL_M);
        // Street rail is not admitted, so it has no family of its own: the
        // question is never asked of it, and the fallback is the road's.
        assert!(!RAIL_CLASSES.contains(&"tram") && !RAIL_CLASSES.contains(&"unknown"));
        assert!(!Family::Walk.solves());
    }
}
