//! Engineering priors — everything the map data does not say, as named,
//! class-keyed parameters in one place (docs/GENERATION.md §6).
//!
//! The vector data gives topology (what is above what, roughly where things
//! start and end); the render needs geometry (heights everywhere). The missing
//! numbers — grade ceilings, clearances, deck thickness, structure widths —
//! are engineering conventions, not measurements. Keeping them here makes them
//! tunable, testable, and honest about being priors rather than data.

/// A road's functional class, parsed once from the Overture `class` string.
/// Keys every class-dependent prior below; unknown or missing classes take the
/// `Minor` defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoadClass {
    Motorway,
    Trunk,
    Primary,
    Secondary,
    Minor,
}

impl RoadClass {
    pub fn parse(class: Option<&str>) -> RoadClass {
        match class {
            Some("motorway") => RoadClass::Motorway,
            Some("trunk") => RoadClass::Trunk,
            Some("primary") => RoadClass::Primary,
            Some("secondary") => RoadClass::Secondary,
            _ => RoadClass::Minor,
        }
    }

    /// Maximum grade (rise/run) the class is engineered to hold, or `None` to
    /// leave the road on the terrain. Only the engineered high classes are
    /// capped; a residential street or track genuinely follows whatever slope
    /// it is built on (scenario S9: grade limits must not "fix" a road that
    /// genuinely climbs).
    pub fn grade_limit(self) -> Option<f64> {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => Some(0.06),
            _ => None,
        }
    }

    /// Half-width in metres of a swept structure box — bigger roads, bigger
    /// structures.
    pub fn half_width_m(self) -> f64 {
        match self {
            RoadClass::Motorway | RoadClass::Trunk => 7.5,
            RoadClass::Primary | RoadClass::Secondary => 6.0,
            RoadClass::Minor => 4.0,
        }
    }
}

/// Vertical clearance a bridge deck's *underside* must keep over a crossed
/// feature (scenario S4, invariant 3). About 5 m over a road, more over rail
/// (catenary). The data never states built clearances; these are the
/// engineering minimums.
pub fn clearance_m(lower: crate::scene::CrossedKind) -> f64 {
    match lower {
        crate::scene::CrossedKind::Road => 5.0,
        crate::scene::CrossedKind::Rail => 7.0,
    }
}

/// Approach-ramp grade for a road class with no engineered ceiling: how fast
/// an overpass approach may climb onto its embankment. Engineered classes use
/// their own [`RoadClass::grade_limit`] instead.
pub const RAMP_GRADE: f64 = 0.08;

/// Smallest cut/fill (|solved road − natural terrain|, metres) that earns an
/// earthwork: below this the road drapes on the natural ground and no ground
/// modifier is emitted.
pub const MIN_EARTHWORK_M: f64 = 0.3;

/// How far an earthwork's slope reaches beyond the road shoulder per metre of
/// cut/fill depth (a 1:2.5 embankment batter), and the floor on that reach so
/// even shallow earthworks blend smoothly into the ground.
pub const EARTHWORK_BATTER: f64 = 2.5;
pub const EARTHWORK_MIN_FEATHER_M: f64 = 2.0;

/// Extra width beyond the structure half-width that an earthwork holds at
/// road height (the shoulder) before the slope starts.
pub const EARTHWORK_SHOULDER_M: f64 = 1.0;

/// Thickness of a bridge deck slab in metres — deck surface to its underside.
pub const DECK_THICKNESS_M: f64 = 1.5;

/// Vertical clearance of a tunnel bore in metres — road floor to its flat roof.
pub const TUNNEL_HEIGHT_M: f64 = 6.0;

/// How far, in metres, an engineered road may sit above (fill) or below (cut)
/// the draped terrain. The grade ceiling alone, held across a long mountain
/// climb, drifts the road tens of metres from the ground — a phantom viaduct or
/// a road buried deep under a hill that should be a tunnel. Bounding the
/// deviation keeps the road hugging the ground. Sized so the residual cut is
/// shallow enough for the client's road depth-bias to surface
/// (`client/shaders/road.wgsl` `ROAD_DEPTH_MARGIN_M`).
pub const MAX_ROAD_DEVIATION_M: f64 = 8.0;

/// Shortest bridge/tunnel span that is lifted/sunk as a structure. Below this a
/// span (a footbridge, a few-metre covered stretch) stays at grade — baking a
/// deck on it only leaves a tiny box floating over the hill.
pub const MIN_STRUCTURE_M: f64 = 40.0;

/// A grade sliver shorter than this (metres), sandwiched between two structure
/// spans, is treated as an annotation-edge mismatch rather than real at-grade
/// road, and dropped so the structures abut (scenario S10). Genuine at-grade
/// stretches between structures are far longer.
pub const SNAP_RUN_M: f64 = 10.0;

/// Step length when marching a buried portal end outward to find the
/// road/terrain crossing where a bore emerges.
pub const PORTAL_MARCH_M: f64 = 3.0;

/// Furthest a portal is marched outward before giving up and capping there. A
/// runaway guard for a road whose approach stays buried (e.g. a coarse DEM
/// that never dips below the road grade); real portals emerge well within this.
pub const PORTAL_MAX_M: f64 = 200.0;

/// Emergence clearance: a portal cap is placed this far *past* the crossing so
/// the mouth sits just clear of the terrain rather than flush with it.
pub const PORTAL_CLEARANCE_M: f64 = 1.0;

/// Longest chain of segments joined into one corridor, in metres. Corridors
/// longer than this are split; junction-continuity constraints (solve stage)
/// carry coherence across the cut. Bounds the profile arrays and keeps a
/// mis-joined ring road from swallowing a region.
pub const MAX_CORRIDOR_M: f64 = 30_000.0;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_engineered_classes_are_grade_limited() {
        assert!(RoadClass::parse(Some("motorway")).grade_limit().is_some());
        assert!(RoadClass::parse(Some("trunk")).grade_limit().is_some());
        assert!(RoadClass::parse(Some("residential")).grade_limit().is_none());
        assert!(RoadClass::parse(None).grade_limit().is_none());
    }

    #[test]
    fn half_width_scales_with_class() {
        assert!(RoadClass::Motorway.half_width_m() > RoadClass::Minor.half_width_m());
        assert_eq!(RoadClass::parse(None).half_width_m(), RoadClass::Minor.half_width_m());
    }
}
