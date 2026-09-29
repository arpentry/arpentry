//! The vertical priors: what a class holds along its length, and how far
//! it may leave the ground to hold it.
//!
//! One table, keyed on class, beside [`crate::width`]'s. Every height the
//! world solves reads it here and nowhere else (docs/GENERATION.md §9). The
//! values are the server's, set against Swiss norms and measured on the
//! Montreux extract:
//!
//! - **Engineered** classes hold a true engineering grade — a motorway is
//!   built to 6 %, a primary to 8 % — and may leave the ground by metres to
//!   hold it: a cutting, an embankment, and beyond that a structure.
//! - **Street** classes are not limited at all: the DEM under a street *is*
//!   the street, and a lane mapped at 20 % up the old town really climbs
//!   20 % (knowing when to do nothing). Held to a 15 % ceiling inside a
//!   2.5 m box, a street on a steeper hill would spend the whole box before
//!   it broke grade and draw every block in two metres of fill. Their ceiling
//!   is the ramp grade a *lift* may use — an overpass approach, a crossing's
//!   clearance — and their box is what such a lift may spend; neither
//!   touches a street the ground already carries.
//! - **Draped** classes hold nothing: a footpath, steps, a track sample the
//!   finished ground exactly and never solve (stratum D). Their entry is
//!   here so that the question "does this class solve?" has one answer.
//! - **Railways** are engineered, and stiffer than any road: a mainline
//!   holds 3 % on a 2 km vertical curve, metre gauge 7 % on 500 m, inside
//!   the motorway's 8 m box. The terrain is a response to the railway, not
//!   the other way round (§4.2). But a railway's class does not say how it
//!   climbs — a rack railway is tagged `narrow_gauge` and runs at 20 %, a
//!   funicular at 57 % — so a rail ceiling is [`Grade::measured`]: raised to
//!   the bed the line actually rides where the ground earns it
//!   ([`limit`]).

use crate::world::{station_runs, Span, Station};

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
    /// The tightest **vertical curve** the alignment holds, as a radius in
    /// metres; `None` for a class that holds none.
    ///
    /// The ceiling bounds how steep a road is; this bounds how fast it may
    /// *change* how steep it is. Switzerland has roads at 20 %, and they are
    /// fine — what no car can drive is a 20 % road meeting a flat one in a
    /// metre, because the transition is a ramp the underbody grounds on. A
    /// radius `R` lets the grade change by `1 / R` per metre, so a street at
    /// [`RADIUS_STREET_M`] takes 20 m to go from level to 20 %, and a
    /// motorway at [`RADIUS_MOTORWAY_M`] takes 240 m.
    ///
    /// Only the *engineered* radii are norm-shaped; the street's is a
    /// drivability floor rather than a comfort figure, chosen to leave a real
    /// hillside street alone while ironing the node-scale kinks a DEM leaves
    /// behind.
    pub radius_m: Option<f64>,
    /// Whether the ceiling is a floor under the grade the line is measured
    /// to ride rather than a limit on it: raised to the bed along the way's
    /// own at-grade stretches, within `measured_cap`. Only a railway: a
    /// road's class says how it climbs, and a railway's does not.
    pub measured: bool,
}

/// The most a measured ceiling may rise to: `max(MEASURED_FLOOR, ceiling ×
/// MEASURED_HEADROOM)`. The server's numbers
/// (`solve::profile::MEASURED_GRADE_*`): the headroom is what lets a
/// funicular classed at 70 % reach the 57 % it runs at when the DEM reads it
/// steeper still, and the floor is what lets a rack railway classed at 7 %
/// reach its 20 %.
pub const MEASURED_FLOOR: f64 = 0.30;
const MEASURED_HEADROOM: f64 = 1.5;

/// The cap on a measured ceiling for a class of prior `ceiling`.
fn measured_cap(ceiling: f64) -> f64 {
    MEASURED_FLOOR.max(ceiling * MEASURED_HEADROOM)
}

/// Shortest length, in metres, of a way's at-grade stretches before its bed
/// is read as its grade: under this the read means nothing, and the class
/// ceiling stands. The server's `MEASURED_GRADE_MIN_M`.
const MEASURED_MIN_M: f64 = 100.0;

/// The percentile of the at-grade bed's station-to-station grades that is
/// read as the grade the line rides: a *sustained* climb raises the ceiling,
/// a local plunge at a structure end does not. The server's
/// `MEASURED_GRADE_PCTL`, and per edge rather than windowed: a window
/// reaching over a notch reads the notch's plunge as the line's grade.
const MEASURED_PCTL: f64 = 0.90;

/// The grade the limiter holds a way of `class` to along `stations`, whose mapped spans
/// are `spans`: the class's own for an engineered road, the grade its bed
/// is measured to ride for a railway ([`Grade::measured`]) where
/// that is steeper, within `measured_cap`. Unbounded for a class
/// that is not grade-limited.
///
/// Read off the **reference** of the at-grade stretches: the conditioned
/// ground is the formation, cuttings and embankments included, and the
/// railway is the reason that shape is there (§4.2). An edge into or out of
/// a structure is a chord, and its pitch is the solve's, so it is not read.
pub fn limit(class: &str, stations: &[Station], spans: &[Span]) -> f64 {
    let g = of(class);
    let Some(c) = g.ceiling.filter(|_| g.limited()) else {
        return f64::INFINITY;
    };
    if !g.measured {
        return c;
    }
    let mut grades: Vec<f64> = Vec::new();
    let mut spanned = 0.0;
    for (k0, k1, kind) in station_runs(stations, spans) {
        if kind.is_structure() {
            continue;
        }
        for w in stations[k0..=k1].windows(2) {
            let run = w[1].s - w[0].s;
            if run > 0.0 {
                spanned += run;
                grades.push((w[1].reference - w[0].reference).abs() / run);
            }
        }
    }
    if spanned < MEASURED_MIN_M || grades.is_empty() {
        return c;
    }
    grades.sort_by(f64::total_cmp);
    let k = ((grades.len() - 1) as f64 * MEASURED_PCTL).round() as usize;
    c.max(grades[k].min(measured_cap(c)))
}

/// The grade a way of `class` is held to along `stations`: its [`limit`]
/// where the class is grade-limited, else the ramp grade a lift on it may use
/// (a street's ceiling), which is what a clearance spreads along it at.
pub fn held(class: &str, stations: &[Station], spans: &[Span]) -> f64 {
    let g = of(class);
    if g.limited() {
        limit(class, stations, spans)
    } else {
        g.ceiling.unwrap_or(f64::INFINITY)
    }
}

/// The tightest vertical curve a mainline railway holds, in metres, and a
/// metre-gauge one (the server's `priors::MAINLINE`, `priors::NARROW`),
/// held by the same `bend` every road holds.
const RADIUS_MAINLINE_M: f64 = 2000.0;
const RADIUS_NARROW_M: f64 = 500.0;

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

/// The tightest vertical curve a motorway or trunk holds, in metres. A
/// surveyed alignment at speed: 6 % to level takes 240 m.
///
/// **These three are design facts, not knobs, and the box is what cannot pay
/// for them.** On a mountainside many engineered runs fail to hold them, and
/// lowering them would only report fewer failures without making anything
/// truer. A real motorway gets the earthworks and structures its curve
/// needs; this model gives it [`Grade::deviation_m`], and eight metres of
/// box does not buy a 4 km curve on a mountainside. The profile step's
/// `boxed` counts that deferral.
pub const RADIUS_MOTORWAY_M: f64 = 4000.0;

/// The same for a primary, and for a secondary at two thirds of it.
const RADIUS_PRIMARY_M: f64 = 2000.0;
pub const RADIUS_SECONDARY_M: f64 = 1500.0;

/// The tightest vertical curve a street holds, in metres — a **drivability**
/// floor, not a comfort figure. A car grounds out where a grade changes
/// faster than its wheelbase can bridge; 100 m spends 20 m of road going from
/// level to 20 %, which a hillside lane really does and a DEM kink does not.
///
/// **It is the knee, and it was chosen by sweeping it.** The cost of a larger
/// radius — how far the road stands off the DEM — is nearly flat, so the
/// choice is not made on cost. What makes it is whether the constraint can
/// be *met*: past 100 m the share of street runs still bent tighter than
/// their class allows (`kink`) climbs steeply, because smoothing that hard
/// would take the road further from its reference than
/// [`Grade::deviation_m`] permits. **100 m is the largest curve a street can
/// actually hold inside its own budget.**
///
/// It agrees with the physics from the other side: 0.3 g of vertical
/// acceleration at 60 km/h — the fastest thing in this bucket, a tertiary —
/// wants `v²/a` ≈ 96 m.
pub const RADIUS_STREET_M: f64 = 100.0;

/// The vertical prior of a way of `class`.
pub fn of(class: &str) -> Grade {
    match class {
        "motorway" | "trunk" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.06),
            deviation_m: 8.0,
            radius_m: Some(RADIUS_MOTORWAY_M),
            measured: false,
        },
        "primary" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.08),
            deviation_m: 4.0,
            radius_m: Some(RADIUS_PRIMARY_M),
            measured: false,
        },
        "secondary" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.10),
            deviation_m: 4.0,
            radius_m: Some(RADIUS_SECONDARY_M),
            measured: false,
        },
        "tertiary" | "unclassified" | "residential" | "living_street" | "service" | "pedestrian"
        | "unknown" => Grade {
            mode: Mode::Street,
            ceiling: Some(0.15),
            deviation_m: 2.5,
            radius_m: Some(RADIUS_STREET_M),
            measured: false,
        },
        // A surveyed alignment on its own formation, senior to every road
        // and engineered like a motorway but tighter (server
        // `priors::MAINLINE`).
        "standard_gauge" | "broad_gauge" | "subway" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.03),
            deviation_m: 8.0,
            radius_m: Some(RADIUS_MAINLINE_M),
            measured: true,
        },
        // Built to reach places standard gauge could not: steeper, and
        // tighter in its curves (`priors::NARROW`).
        "narrow_gauge" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.07),
            deviation_m: 8.0,
            radius_m: Some(RADIUS_NARROW_M),
            measured: true,
        },
        // A funicular is laid *on* its hillside — no cuttings, no
        // embankments, a pair of rails pinned to the slope — so its bed is
        // the answer and its box is tight (`priors::FUNICULAR`). The ceiling
        // is a class convention covering the steepest the class runs, not a
        // constant gradient: the line arrives in fragments, and a chord
        // between fragment ends is neither the funicular's gradient nor the
        // ground's. It holds no vertical curve: the bed is its curve.
        "funicular" => Grade {
            mode: Mode::Engineered,
            ceiling: Some(0.70),
            deviation_m: 2.5,
            radius_m: None,
            measured: true,
        },
        // A draped class holds nothing, and steps least of all: a stair is
        // a sequence of vertical breaks and bounding them would be a lie.
        _ => Grade { mode: Mode::Draped, ceiling: None, deviation_m: 0.0, radius_m: None, measured: false },
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
            assert_eq!((g.ceiling, g.deviation_m, g.radius_m), (None, 0.0, None), "{class}");
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

    #[test]
    fn a_railway_is_stiffer_than_any_road_and_measured() {
        let (main, narrow, funi) = (of("standard_gauge"), of("narrow_gauge"), of("funicular"));
        for g in [main, narrow, funi] {
            assert_eq!(g.mode, Mode::Engineered);
            assert!(g.measured);
        }
        assert!(main.ceiling < of("motorway").ceiling && main.ceiling < narrow.ceiling);
        assert!(main.radius_m < of("motorway").radius_m && narrow.radius_m < main.radius_m);
        // The funicular's box is tight and it holds no curve: its bed is.
        assert!(funi.deviation_m < main.deviation_m && funi.radius_m.is_none());
        // Only a railway's ceiling is measured.
        assert!(!of("motorway").measured && !of("residential").measured);
        // The cap: a rack railway classed at 7 % may reach 30 %, a
        // funicular classed at 70 % may reach 105 %.
        assert!((measured_cap(0.07) - 0.30).abs() < 1e-12);
        assert!((measured_cap(0.70) - 1.05).abs() < 1e-12);
    }
}
