//! One ground: the specification of how the paved surface and the ground
//! meet, written as checks on synthetic specimens.
//!
//! > **One arrangement. One height per vertex. A step is an edge you declare.**
//! >
//! > **And the DEM is a measurement: a standard is a floor on what it cannot
//! > see, never a correction to what it can.**
//!
//! **Built**, and checked live: every drawn edge is welded or walled; the
//! ground and the surface share their boundary, vertex for vertex; and every
//! step that moves a height says how far from the DEM it has moved it.
//!
//! **Open**, `#[ignore]`d with the rule it needs, so `cargo test --
//! --ignored` lists it: a crossing the DEM already separates spends no
//! clearance standard, and the crossing step says which of the two answered.
//!
//! **The specimens stand on a 150 % flank ([`FLANK`]) on purpose.** A
//! specimen has only its slope, while a real hillside reaches the same
//! defects at 30–60 % by *combining* cross-slope, the profile's departure
//! from the DEM and the clearance lift. A flank steeper than any hillside is
//! the corpus's cheapest route to a defect the real world arrives at by
//! another road; on gentler ground every `net:` specimen reads zero whether
//! the rule holds or not.
//!
//! **The earthwork is not the road's.** The drawn ground stands further off
//! the DEM than the road it carries, because the paved surface is held level
//! crosswise over its whole plan extent and the batter and the wall pay for
//! that at its edge. No constant of the bench's own reaches that number;
//! reading the `dem_residual` of each step against the earthwork's is what
//! attributes it.

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::{Step, Summary};

    /// A flank steep enough for the corpus to state the defect on: on
    /// `net:sidewalk` below a grade of 1.0 no bench is ever walled and below
    /// 1.5 no drawn edge is ever a step, because a specimen has only its
    /// slope where the real world also has a profile departure and a
    /// clearance lift.
    const FLANK: &str = "ramp?grade=1.5&bearing=45&radius=100000";

    /// A junction with pavement all round it, on a ring whose axis closes on
    /// itself: the specimen that asks most of the step rule on [`FLANK`].
    const JUNCTION: &str = "net:roundabout";

    /// The world through the bench step, and the lines of the lift, the
    /// earthwork and the bench as one.
    fn bench(ground: &str, net: &str, houses: Option<&str>) -> Summary {
        built(ground, net, houses, 5.0, &upto(Step::Bench)).1.merged(&crate::pipeline::tests::STAND)
    }

    /// **Every drawn edge is welded or walled.**
    ///
    /// `step` counts drawn edges the heights **jump** across: over
    /// `KERB_RISE_M`, steeper than `STEP_GRADE`, and with the edge's own rule
    /// sampled along it moving by a jump rather than continuously. A steep
    /// edge a continuous rule answers is `steep`, and the raw DEM's own
    /// cliffs are `dem_steep`; neither is a step.
    ///
    /// What makes it hold: a paved triangle takes one rule at its centroid
    /// ([`crate::copies::Rule`] — the field, the axis, the stretch of it, the
    /// face or the drape) and all three corners are answered by it; a vertex
    /// two rules answer differently is two copies, welded within a kerb's
    /// rise and split with a face between them otherwise. A foot on a
    /// polyline axis is blended with its neighbour's on the inside of a bend,
    /// including across the seam where a ring's axis closes on itself, and
    /// two legs of a junction weigh against the nearer leg.
    #[test]
    fn every_drawn_edge_is_welded_or_walled() {
        let s = bench(FLANK, JUNCTION, None);
        assert_eq!(s.num("step"), 0.0, "the field is discontinuous across a drawn edge: {s}");
    }

    /// **The ground and the surface share their boundary.**
    ///
    /// `seam` is how often an outline vertex has no copy on the paving's
    /// side; `unmet` how many paved rim edges have nothing beyond them.
    /// There is one mesh (`mesh::run`), and each of its vertices is copied
    /// once per surface that reaches it, so an outline vertex's paved copy
    /// and its ground copy are the same index. Both read **exactly** zero by
    /// construction rather than by a repair — which is why this asserts
    /// `== 0.0` and not a tolerance. The houses are what make this specimen
    /// state the boundary at all: without them `seam` and `unmet` read zero
    /// on every `net:` specimen at every grade, whether the rule holds or
    /// not.
    #[test]
    fn the_ground_and_the_surface_share_their_boundary() {
        let s = bench(FLANK, JUNCTION, Some("house:row?gap=2"));
        assert_eq!(s.num("seam"), 0.0, "the outline is not the mesh's: {s}");
        assert_eq!(s.num("unmet"), 0.0, "the rim does not meet the ground: {s}");
    }

    /// **The earthwork is attributable to a step.**
    ///
    /// Each step that moves a height says how far it has moved it, against
    /// one baseline ([`crate::step::Residual`]), so a height made in one step
    /// and paid for in another is attributed by one run rather than by
    /// sweeping constants and watching what fails to move — which says only
    /// where a number is *not* made.
    ///
    /// The check is that the instrument keeps existing. It reads no value,
    /// because a value here would be a second copy of what the run prints; it
    /// asserts that no step which moves heights can stop saying by how much.
    #[test]
    fn the_earthwork_is_attributable() {
        let (_, ran) = built(FLANK, JUNCTION, None, 5.0, &upto(Step::Bench));
        for step in [Step::Reference, Step::Profile, Step::Partition, Step::Earthwork] {
            let s = ran.of(step);
            assert!(
                s.get("dem_residual").is_some(),
                "the {} step moves heights and does not say by how much: {s}",
                step.name()
            );
        }
    }

    /// **A measured crossing spends no standard.**
    ///
    /// The rule: where the DEM resolves both sides of a crossing the measured
    /// separation stands and the demand is met by definition; the standard
    /// supplies the number only where the ground has swallowed the structure.
    ///
    /// **The specimen for the full rule does not exist.** It needs a ground
    /// that *already carries* the separation, so that spending the standard
    /// is visibly redundant. `flat` is the opposite case — two ways crossing
    /// on flat ground genuinely do need separating, so the embankment there
    /// is correct, and a check written against it would assert something
    /// false. `specs::spans`'s `a_shelf_is_a_deck_the_dem_has_swallowed` is
    /// blocked on the neighbouring half of the same missing rung.
    ///
    /// So this states the rule where the corpus can reach it: the crossing
    /// step says *which* of the two answered.
    #[test]
    #[ignore = "the crossing step does not yet say whether the measured or the standard clearance answered"]
    fn a_measured_crossing_says_which_answered() {
        let (_, ran) = built("gorge?depth=30&width=40", "net:overpass", None, 5.0, &upto(Step::Crossing));
        let s = ran.of(Step::Crossing);
        assert!(
            s.get("measured").is_some(),
            "the crossing step does not say whether the ground answered: {s}"
        );
    }
}
