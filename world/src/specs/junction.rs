//! One surface at a junction: the specification of how pieces of the paving
//! merge, written as a check on a synthetic specimen.
//!
//! > **Regions merge where they meet. They stay apart where they cross.**
//!
//! A level ordinal orders what *crosses* — `crossing` reads it for two
//! interiors that cross with no connector between them. Two pieces that share
//! a connector meet at one point in the ground truth and no tag makes them
//! otherwise, so the partition into groups that may merge is the connected
//! components of the piece graph, not the level.
//!
//! The case that matters is a junction standing on a structure: built twice —
//! the ground pieces through the surface steps, the span pieces through a
//! sweep of their own — the paving would meet neither in plan nor in height
//! at the handover. **Built**, and checked live: the `sheet` step paves a
//! span in one polygon with the ground pieces it meets, so the deck and its
//! approaches are one region, meshed once and lifted by one field. Nothing
//! here is open.

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;

    /// **The specimen: a junction standing on a structure** — three legs at
    /// one connector, with spans on them. Nothing less states the rule:
    /// `tee` and `cross` are junctions with no span on them,
    /// `straight?span=` is a span with no junction and no second leg to
    /// blend with, and `overpass` and `underpass` are interiors crossing with
    /// **no connector between them**, which is the case the rule
    /// deliberately keeps apart.
    const JUNCTION_ON_A_DECK: &str = "net:tee?span=0.3&kind=bridge";

    /// The ground it needs. A *mapped* span is only a prior — a mapped
    /// bridge whose chord never leaves the ground gets no deck — so on a
    /// uniform ramp the chord lies at grade and the check passes without
    /// stating anything. Over a gorge the legs stand off the ground and a
    /// handover would have something to disagree about.
    const GORGE: &str = "gorge?depth=30&width=40";

    /// **The handover is not a boundary at all** (invariant I2, surface
    /// continuity). The surface must cross it with no step, and the way it
    /// does is that there is nothing to cross: the deck and its approaches
    /// are one polygon of one sheet, meshed once and lifted by one field. So
    /// what is asserted is that there is only one construction, that it is
    /// one region, and that its rim closes.
    #[test]
    fn a_junction_on_a_structure_is_one_surface() {
        let (w, ran) = built(GORGE, JUNCTION_ON_A_DECK, None, 5.0, &upto(Step::Structure));
        let s = ran.last();
        let bench = w.bench.as_ref().expect("the bench step ran");
        let st = w.structure.as_ref().expect("the structure step ran");
        assert!(
            st.roadway.positions.is_empty(),
            "the structure step lays a roadway: {s}"
        );
        assert!(!bench.carriageway.positions.is_empty(), "and the bench lays one: {s}");
        // One sheet, one region: the tee's three ground legs and its three
        // spans, with no boundary anywhere in it.
        let sheets = w.sheet.as_ref().expect("the sheet step ran");
        let car: Vec<&crate::world::Sheet> =
            sheets.of(crate::width::Family::Carriageway).collect();
        assert_eq!(car.len(), 1, "one sheet: {s}");
        assert_eq!(car[0].shapes.len(), 1, "and one region: {s}");
        assert!(car[0].spanning && !car[0].spans.is_empty(), "holding the spans: {s}");
        // And the rim closes. `unmet` counts paved rim edges with nothing
        // beyond them away from the rect's border — which is what a
        // handover between two constructions would show up as. There is one
        // construction, and it reads zero.
        let b = ran.merged(&crate::pipeline::tests::STAND);
        assert_eq!(b.num("unmet"), 0.0, "the surface does not close: {b}");
    }
}
