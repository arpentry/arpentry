//! One surface at a junction — the checks, ahead of the steps.
//!
//! **Nothing here is built yet.** This module is the specification of
//! `data/plans/one-surface-at-a-junction-2026-09-14.md`, written as the checks
//! that plan has to pass, ahead of the steps themselves. Every check below is
//! `#[ignore]`d with the rule it names and the number the world reads today,
//! so `cargo test -- --ignored` is the plan's own to-do list and the suite
//! stays green meanwhile. A check comes off `#[ignore]` when the rule it names
//! lands, and never before.
//!
//! The defect, in one sentence: the paved surface is built twice — the ground
//! pieces through `ribbon → surface → … → mesh → bench`, the span pieces
//! through `structure`'s own sweep — and the two meet neither in plan nor in
//! height. Measured on the loop box, the unioned `carriageway` covers 0.00 %
//! of its 637 000 m² twice while the swept `roadway` covers 1.43 % of its
//! 72 000 m² twice, and at a handover seam the step between them is 0.43 m at
//! p90 and 2.79 m at worst.
//!
//! The rule the plan arrives at:
//!
//! > **Regions merge where they meet. They stay apart where they cross.**
//!
//! A level ordinal orders what *crosses* — `crossing` reads it for two
//! interiors that cross with no connector between them. Two pieces that share
//! a connector meet at one point in the ground truth and no tag makes them
//! otherwise, so the partition into groups that may merge is the connected
//! components of the piece graph, not the level.
//!
//! **One specimen is missing and the gap is a deferral, not an omission.**
//! Nothing in `net:` makes two *spans* meet at a shared connector — `tee` and
//! `cross` are junctions on the ground, `overpass` and `underpass` are
//! interiors crossing with no connector between them. The check that a
//! junction standing on a structure comes out as one surface needs a
//! `net:tee?span=…` (or the like) before it can be written at all, and it is
//! the check the whole plan exists for. Until then the two below state the
//! same rule where the corpus can reach it: a span meeting its own ground
//! piece, which is the same seam with two legs instead of three.

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    use crate::world::World;

    /// A world built through the structure step.
    fn world(ground: &str, net: &str) -> World {
        built(ground, net, None, 5.0, &upto(Step::Structure)).0
    }

    /// **Step 0 — the specimen.** Both checks below are blocked on the same
    /// missing thing, and it is worth being exact about why.
    ///
    /// The defect needs a *junction* standing on a structure: three legs at
    /// one connector, at least one of them a span. Then the ground legs are
    /// lifted by `bench` with its junction blend over `JOINT_M` while the
    /// span leg is lifted by `structure`'s `decks` field afterwards, the two
    /// disagree across the boundary they share, and the seam steps.
    ///
    /// `net:` cannot say that. `tee` and `cross` are junctions with no span
    /// on them; `straight?span=` is a span with no junction; `overpass` and
    /// `underpass` are interiors crossing with **no connector between them**,
    /// which is the case the rule deliberately keeps apart. Written against
    /// `straight?span=` anyway, both checks pass today and guard nothing —
    /// one road on a uniform ramp has no second leg to blend with, and `lap`
    /// is 0 because its ground pieces never reach the span.
    ///
    /// That is the same corpus gap that let 232 green tests sit over the
    /// pavement defect for as long as they did. So the first thing the plan
    /// builds is a `net:tee?span=…`, and these two come off `#[ignore]`
    /// against it.
    const JUNCTION_ON_A_DECK: &str = "net:tee?span=0.3&kind=bridge";

    /// The ground it needs. A *mapped* span is only a prior — "a mapped
    /// bridge whose chord never left the ground gets no deck" — so on a
    /// uniform ramp the specimen builds `decks=0`, the chord lies at grade,
    /// and the seam check passes without stating anything. Over a gorge the
    /// three legs stand off the ground (`decks=3`, `clear=2.50`) and the two
    /// constructions have something to disagree about.
    const GORGE: &str = "gorge?depth=30&width=40";

    /// **Step 3 — the junction and its spans are one region.** Not one
    /// *surface* yet: `ribbon::grouped` says what the regions are, and
    /// steps 4 and 5 are what mesh and lift them. But two regions where
    /// there should be one is what the seam is a symptom of, and that much
    /// can be checked now — so this one is live rather than `#[ignore]`d,
    /// and it is the first thing that would break if the grouping regressed.
    #[test]
    fn a_junction_on_a_structure_is_one_region() {
        let w = world(GORGE, JUNCTION_ON_A_DECK);
        let net = w.network().expect("the partition ran");
        let groups = &w.partition.as_ref().expect("the partition ran").groups;
        let car: Vec<(crate::width::Family, usize, crate::poly::Shapes)> = crate::ribbon::grouped(net, &groups.of)
            .into_iter()
            .filter(|(f, ..)| *f == crate::width::Family::Carriageway)
            .collect();
        assert_eq!(car.len(), 1, "the tee's ground legs and its spans must be one group, got {}", car.len());
        assert_eq!(car[0].2.len(), 1, "and one region, not {}", car[0].2.len());
    }

    /// **I2 — the handover is not a boundary at all.** The surface must
    /// cross it with no step, and the way it does is that there is nothing
    /// to cross: the deck and its approaches are one polygon of one sheet,
    /// meshed once and lifted by one field.
    ///
    /// The check `seam` was written for compared two meshes and asked how
    /// far apart they stood — p90 0.432 m, max 2.79 m on the loop box, 28.6
    /// % of seams over 10 cm. There is no second mesh to compare against
    /// now, so what is asserted is that there is only one, that it is one
    /// region, and that its rim closes.
    #[test]
    fn a_junction_on_a_structure_is_one_surface() {
        let (w, ran) = built(GORGE, JUNCTION_ON_A_DECK, None, 5.0, &upto(Step::Structure));
        let s = ran.last();
        let bench = w.bench.as_ref().expect("the bench step ran");
        let st = w.structure.as_ref().expect("the structure step ran");
        assert!(
            st.roadway.positions.is_empty(),
            "the structure step still lays a roadway: {s}"
        );
        assert!(!bench.carriageway.positions.is_empty(), "and the bench lays one: {s}");
        // One sheet, one region: the tee's three ground legs and its three
        // spans, with no boundary anywhere in it.
        let sheets = w.sheets.as_ref().expect("the sheet step ran");
        let car: Vec<&crate::world::Sheet> =
            sheets.of(crate::width::Family::Carriageway).collect();
        assert_eq!(car.len(), 1, "one sheet: {s}");
        assert_eq!(car[0].shapes.len(), 1, "and one region: {s}");
        assert!(car[0].spanning && !car[0].spans.is_empty(), "holding the spans: {s}");
        // And the rim closes. `unmet` counts rim vertices of the paved
        // mesh that neither meet the ground nor are excluded as a deck's
        // own edge, a portal or a seam inside the room — which is what a
        // handover between two constructions showed up as. There is one
        // construction, and it reads zero.
        let b = ran.merged(&crate::pipeline::tests::BENCH);
        assert_eq!(b.num("unmet"), 0.0, "the surface does not close: {b}");
    }

    /// **The lap goes to zero by construction.** `structure` reports `lap` —
    /// roadway drawn over ground the carriageway also paves, 137 m on the
    /// loop box. It is not a defect to trim away: the abutment margin is
    /// what carries a deck out to its rims, and dropping it took the gorge
    /// specimen's deck from 40 m to 32 m, ending the roadway over the void
    /// while the soffit spanned on. It measures a boundary two constructions
    /// do not share, so after step 5 there is nothing left to measure.
    #[test]
    fn the_lap_is_gone() {
        let (w, ran) =
            built(GORGE, JUNCTION_ON_A_DECK, None, 5.0, &upto(Step::Structure));
        let s = ran.last();
        assert!(s.num("decks") > 0.0, "the specimen must build a deck on the junction: plan step 0 — {s}");
        // `lap` measured roadway drawn over ground the carriageway also
        // paved: a boundary two constructions did not share. There is one
        // construction now, so what it measured cannot be measured — the
        // number is gone from the summary rather than reading zero, which
        // is the honest way for a measurement to end.
        assert_eq!(s.get("lap"), None, "`lap` still has a subject: {s}");
        let st = w.structure.as_ref().expect("the structure step ran");
        assert!(st.roadway.positions.is_empty(), "there is a roadway to lap with: {s}");
    }
}
