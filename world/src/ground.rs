//! One ground — the checks, ahead of the steps.
//!
//! **Steps 0 and 1 are built and the rest is not.** This module is the specification of
//! `data/plans/one-ground-2026-09-16.md`, written as the checks that plan has
//! to pass, ahead of the steps themselves. Every check below is `#[ignore]`d
//! with the rule it names and the number the world reads today, so
//! `cargo test -- --ignored` is the plan's own to-do list and the suite stays
//! green meanwhile. A check comes off `#[ignore]` when the rule it names
//! lands, and never before.
//!
//! The plan, in one rule:
//!
//! > **One arrangement. One height per vertex. A step is an edge you declare.**
//! >
//! > **And the DEM is a measurement: a standard is a floor on what it cannot
//! > see, never a correction to what it can.**
//!
//! ## What the corpus can and cannot say
//!
//! Writing these checks turned up a gap worth more than the checks, and it is
//! the same disease `junction.rs` diagnosed once already — "the same corpus
//! gap that let 232 green tests sit over the pavement defect".
//!
//! **Every specimen in `net:` reads `seam=0 unmet=0 step=0 wall_m2=0` on every
//! ground in `--terrain`,** while the Montreux junction box reads `seam` 4.95
//! %, `unmet` 3.46 %, `step` 404 and `wall_m2` 1654. Two hundred and
//! forty-four green tests sit over the whole defect family.
//!
//! Half of it is reachable, and the way in is raw slope. On
//! `ramp?bearing=45` with `net:sidewalk?d=6`:
//!
//! | grade | `cut` | `step` | `worst` | `walled` | `wall_m2` |
//! |---|---|---|---|---|---|
//! | 0.3 | 1.365 | 0/1952 | 0.000 | 0/182 | 0 |
//! | 0.6 | 2.850 | 0/1952 | 0.000 | 0/182 | 0 |
//! | 1.0 | 4.830 | 0/1952 | 0.000 | **87**/182 | 995 |
//! | 1.5 | 7.305 | **998**/1952 | 5.301 | 89/182 | 1519 |
//! | 2.0 | 9.779 | 1496/1952 | 7.068 | 173/182 | 2822 |
//!
//! The knees are `MAX_BENCH_FACE_M` for `walled` and, half a grade later,
//! `STEP_GRADE` for `step`. It takes a 150 % flank because a specimen has
//! only its slope, while Montreux reaches the same place at 30–60 % by
//! *combining* cross-slope, the profile's departure from the DEM and the
//! clearance lift. So [`FLANK`] is steeper than any hillside in the zone on
//! purpose: it is the corpus's cheapest route to a defect the real world
//! arrives at by another road.
//!
//! **The other half took three tries to reach, and one combination reaches
//! it.** `seam` and `unmet` — the two numbers that measured the two-mesh
//! reconciliation — read exactly 0 on `cross`, `sidewalk`, `crossing` and
//! `corner[&split=1]` at every grade, with and without houses. [`JUNCTION`]
//! with `house:row?gap=2` on [`FLANK`] was the first thing in the corpus that
//! stated them (`seam` 30/2150, `unmet` 2/984). Step 1 has landed since, and
//! both read zero there by construction; see
//! `tests::the_ground_and_the_surface_share_their_boundary`.
//!
//! ## And the earthwork is not the road's
//!
//! On the Montreux junction box `cut=7.925 fill=11.076 worst=8.061` are
//! *bit-identical* under the bench reach swept 6 → 0 and under both clearance
//! standards swept to 0.1 m: `bench` cannot reach the number it is
//! compensating for with any constant of its own. Which step makes it was the
//! open question until `dem_residual` landed (plan §7 step 0,
//! [`tests::the_earthwork_is_attributable`]), and the answer was none of the
//! three suspected.
//!
//! On the loop box the road, after everything upstream has had its say, stands
//! 0.03/0.40/**9.17** m off the raw DEM. The **drawn ground** stands
//! 0.00/0.91/**24.63**. The earthwork is 2.7× the departure of the thing it
//! exists to support, and what makes it is neither a solved height nor a
//! standard: it is the paved surface being held **level crosswise** over its
//! whole plan extent, with the batter and the wall running out of whatever
//! that costs at the edge. The specimen says the same at a tenth of a second —
//! [`JUNCTION`] with houses on [`FLANK`] reads `bench` 0.00/0.00/11.03 against
//! a road at 0.00/2.50/6.50.

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::{Step, Summary};

    /// A flank steep enough for the corpus to state the defect on. See the
    /// module's table: below 1.0 no bench is ever walled and below 1.5 no
    /// drawn edge is ever a step, because a specimen has only its slope where
    /// the real world also has a profile departure and a clearance lift.
    const FLANK: &str = "ramp?grade=1.5&bearing=45&radius=100000";

    /// A junction with pavement all round it and the largest `worst` of any
    /// specimen: `step` 2304/6368, `worst` 15.432 m on [`FLANK`].
    const JUNCTION: &str = "net:roundabout";

    /// The world through the bench step — the last one that moves the ground.
    fn bench(ground: &str, net: &str, houses: Option<&str>) -> Summary {
        built(ground, net, houses, 5.0, &upto(Step::Bench)).1.of(Step::Bench)
    }

    /// **Step 2 — every drawn edge is welded or walled. Built 2026-09-27;
    /// live since 2026-09-28.**
    ///
    /// `step` counts drawn edges the heights **jump** across: over
    /// `KERB_RISE_M`, steeper than `STEP_GRADE`, and with a midpoint the
    /// edge's own rule does not put near the mean of its ends. It used to
    /// count every edge over the grade, which on this 150 % flank is most of
    /// a street that follows the ground; those are `steep` now, and the
    /// raw DEM's own cliffs are `dem_steep`.
    ///
    /// Built: a paved triangle takes one rule at its centroid
    /// (`bench::Rule` — the field, the axis, the stretch of it, the face or
    /// the drape) and all three corners are answered by it; a vertex two
    /// rules answer differently is two copies, welded within a kerb's rise
    /// and split with a face between them otherwise. A foot on a polyline
    /// axis is blended with its neighbour's on the inside of a bend, and two
    /// legs of a junction weigh against the nearer leg. Loop box `step`
    /// **18 408 → 144**, worst 11.85 → 3.74 m; the Montreux junction box and
    /// every other specimen read 0.
    ///
    /// It read 26 of 6 644 on the ring's seam (its axis closes on itself, so
    /// the bend's blend did not hand over there), then 2 once the blend
    /// walked the seam. With the carriageway built from its legs rather than
    /// by the fillet's closing (`legs`, wired 2026-09-28) it reads 0; which
    /// of the fillet's returns the two were was not read.
    #[test]
    fn every_drawn_edge_is_welded_or_walled() {
        let s = bench(FLANK, JUNCTION, None);
        assert_eq!(s.num("step"), 0.0, "the field is discontinuous across a drawn edge: {s}");
    }

    /// **Step 1 — the ground and the surface share their boundary. Landed
    /// 2026-09-27, and live.**
    ///
    /// `seam` is how often an outline vertex has no copy on the paving's
    /// side; `unmet` how many paved rim edges have nothing beyond them. Both
    /// used to exist because `mesh` triangulated the room and `bench::Ground`
    /// re-triangulated `rect − room` apart, and the two were reconciled at
    /// `poly::GRID_M` with an eight-neighbour search: this specimen read
    /// `seam` **30/2150** and `unmet` **2/984**, Montreux 4.95 % and 3.46 %.
    ///
    /// There is one mesh now (`mesh::run`), and the bench copies each of its
    /// vertices once per surface that reaches it, so an outline vertex's
    /// paved copy and its ground copy are the same index. Both read
    /// **exactly** zero, which is the difference between a construction and a
    /// repair — and why this asserts `== 0.0` and not a tolerance. (Its guard,
    /// `the_corpus_states_the_seam`, kept the specimen stating the defect
    /// until this landed, and went with it.)
    #[test]
    fn the_ground_and_the_surface_share_their_boundary() {
        let s = bench(FLANK, JUNCTION, Some("house:row?gap=2"));
        assert_eq!(s.num("seam"), 0.0, "the outline is not the mesh's: {s}");
        assert_eq!(s.num("unmet"), 0.0, "the rim does not meet the ground: {s}");
    }

    /// **Step 0 — the earthwork is attributable to a step. Landed
    /// 2026-09-16, and live.**
    ///
    /// Before it, a height made in `reference` and paid for in `bench` could
    /// be attributed only by sweeping constants and watching what failed to
    /// move. That is how the plan's §1.1 was found, and it is an expensive way
    /// to learn one fact: on the Montreux junction box `cut`, `fill` and
    /// `worst` came back bit-identical from three sweeps — the bench reach
    /// 6 → 0, `RAIL_CLEARANCE_M` 7 → 0.1, `ROAD_CLEARANCE_M` 5 → 0.1 — which
    /// says only where the number is *not* made.
    ///
    /// Each step that moves a height now says how far it has moved it, against
    /// one baseline ([`crate::step::Residual`]). One run attributes what three
    /// sweeps could not: on flat ground the whole departure is `crossing`'s —
    /// `reference` and `profile` read 0.00/0.00/0.00, `crossing` reads
    /// 0.00/1.73/8.50 — and on the real ground `crossing` doubles p90
    /// (0.48 → 0.98) where `reference` had moved p90 to 0.32.
    ///
    /// The check is that the instrument keeps existing. It reads no value,
    /// because a value here would be a second copy of what the run prints; it
    /// asserts that no step which moves heights can stop saying by how much.
    #[test]
    fn the_earthwork_is_attributable() {
        let (_, ran) = built(FLANK, JUNCTION, None, 5.0, &upto(Step::Bench));
        for step in [Step::Reference, Step::Profile, Step::Partition, Step::Bench] {
            let s = ran.of(step);
            assert!(
                s.get("dem_residual").is_some(),
                "the {} step moves heights and does not say by how much: {s}",
                step.name()
            );
        }
    }

    /// **Step 4 — a measured crossing spends no standard.**
    ///
    /// The rule: where the DEM resolves both sides of a crossing the measured
    /// separation stands and the demand is met by definition; the standard
    /// supplies the number only where the ground has swallowed the structure.
    ///
    /// **The specimen for the full rule does not exist, and the gap is a
    /// deferral rather than an omission.** It needs a ground that *already
    /// carries* the separation, so that spending the standard is visibly
    /// redundant. `flat` is the opposite case — two ways crossing on flat
    /// ground genuinely do need separating, so the embankment there is
    /// correct, and a check written against it would assert something false.
    /// (The plan's §1 uses flat ground for the one thing it can say: that the
    /// earthwork is *generated* by the clearance solve rather than by the
    /// terrain — `step` 399 → 22, `wall_m2` 1025 → 330 when the two standards
    /// go 7/5 → 1/1.) `partition`'s own
    /// `a_shelf_is_a_deck_the_dem_has_swallowed` is blocked on the
    /// neighbouring half of the same missing rung.
    ///
    /// So this states the rule where the corpus can reach it — that the
    /// crossing step says *which* of the two answered — and that much is
    /// landable now.
    #[test]
    #[ignore = "evidence-before-standards (plan §3.4, step 4) is not built"]
    fn a_measured_crossing_says_which_answered() {
        let (_, ran) = built("gorge?depth=30&width=40", "net:overpass", None, 5.0, &upto(Step::Crossing));
        let s = ran.of(Step::Crossing);
        assert!(
            s.get("measured").is_some(),
            "the crossing step does not say whether the ground answered: {s}"
        );
    }
}
