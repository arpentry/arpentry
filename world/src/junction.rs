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
    use crate::world::{Tri, World};

    /// A world built through the structure step.
    fn world(ground: &str, net: &str) -> World {
        built(ground, net, None, 5.0, &upto(Step::Structure)).0
    }

    /// The worst height disagreement, in metres, where two meshes put a
    /// vertex at the same plan position.
    ///
    /// This is the question the seam asks, and `abutment` does not: that
    /// compares one point on the axis — a span's end *station* against the
    /// ground piece's — before any blending, and says nothing about the
    /// surface across its width.
    fn seam(a: &Tri, b: &Tri) -> f64 {
        let key = |p: &[f64; 3]| [(p[0] / 1e-3).round() as i64, (p[1] / 1e-3).round() as i64];
        let mut at: std::collections::HashMap<[i64; 2], f64> = std::collections::HashMap::new();
        for p in &b.positions {
            at.entry(key(p)).and_modify(|h| *h = h.min(p[2])).or_insert(p[2]);
        }
        let mut worst = 0.0f64;
        for p in &a.positions {
            if let Some(h) = at.get(&key(p)) {
                worst = worst.max((h - p[2]).abs());
            }
        }
        worst
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
    /// *surface* yet: `surface::grouped` says what the regions are, and
    /// steps 4 and 5 are what mesh and lift them. But two regions where
    /// there should be one is what the seam is a symptom of, and that much
    /// can be checked now — so this one is live rather than `#[ignore]`d,
    /// and it is the first thing that would break if the grouping regressed.
    #[test]
    fn a_junction_on_a_structure_is_one_region() {
        let w = world(GORGE, JUNCTION_ON_A_DECK);
        let roads = w.roads.as_ref().expect("the reader ran");
        let car: Vec<(crate::width::Family, usize, crate::poly::Shapes)> = crate::surface::grouped(roads)
            .into_iter()
            .filter(|(f, ..)| *f == crate::width::Family::Carriageway)
            .collect();
        assert_eq!(car.len(), 1, "the tee's ground legs and its spans must be one group, got {}", car.len());
        assert_eq!(car[0].2.len(), 1, "and one region, not {}", car[0].2.len());
    }

    /// **I2 — two groups that share a connector agree at it.** The surface
    /// must cross a handover with no step: one function, evaluated once, for
    /// both sides.
    ///
    /// Today they are two functions, and on the loop box the seam reads p90
    /// 0.432 m, max 2.79 m, with 28.6 % of seams over 10 cm.
    #[test]
    #[ignore = "step 0: `net:` cannot put a span on a junction yet"]
    fn a_junction_on_a_structure_is_one_surface() {
        let w = world(GORGE, JUNCTION_ON_A_DECK);
        let b = w.bench.as_ref().expect("the bench step ran");
        let s = w.structure.as_ref().expect("the structure step ran");
        assert!(
            !s.roadway.positions.is_empty(),
            "the specimen must put a span on the junction, and `{JUNCTION_ON_A_DECK}` does not build one: \
             that is plan step 0, and until it exists this check states nothing"
        );
        let step = seam(&s.roadway, &b.carriageway);
        assert!(step < 1e-6, "the seam steps by {step:.3} m: the two sides are two functions");
    }

    /// **The lap goes to zero by construction.** `structure` reports `lap` —
    /// roadway drawn over ground the carriageway also paves, 137 m on the
    /// loop box. It is not a defect to trim away: the abutment margin is
    /// what carries a deck out to its rims, and dropping it took the gorge
    /// specimen's deck from 40 m to 32 m, ending the roadway over the void
    /// while the soffit spanned on. It measures a boundary two constructions
    /// do not share, so after step 5 there is nothing left to measure.
    #[test]
    #[ignore = "step 0: `net:` cannot put a span on a junction yet"]
    fn the_lap_is_gone() {
        let (_, ran) =
            built(GORGE, JUNCTION_ON_A_DECK, None, 5.0, &upto(Step::Structure));
        let s = ran.last();
        assert!(s.num("decks") > 0.0, "the specimen must build a deck on the junction: plan step 0 — {s}");
        assert_eq!(s.num("lap"), 0.0, "{s}");
    }
}
