//! The partition — where a way is a deck, a bore, or on the ground.
//!
//! **Nothing here is built yet.** This module is the specification of the step
//! `data/plans/spans-are-derived-2026-09-09.md` describes, written as the
//! checks that step has to pass, ahead of the step itself. Every check below
//! is `#[ignore]`d with the rule it names and the number the world reads
//! today, so `cargo test -- --ignored` is the plan's own to-do list and the
//! suite stays green meanwhile. A check comes off `#[ignore]` when the rule it
//! names lands, and never before: that is the whole of "write the check before
//! the fix".
//!
//! What the step will be, when it is:
//!
//! ```text
//! spans(profile, annotated, licenses, prior) -> Vec<Span>
//! ```
//!
//! one function of its inputs, computed once, never mutated — and the place
//! the geometry is cut into ground pieces and structure pieces, which the
//! reader ([`crate::roads`]) does at the source's annotation edges today.
//!
//! The four rules the checks name, in the plan's numbering:
//!
//! - **R1** the annotation is a prior and may not be cut into geometry;
//! - **R2** one departure criterion, symmetric, from the ground stage —
//!   a deck past `MAX_BENCH_FACE_M` over the reference, a bore past
//!   `TUNNEL_HEIGHT_M + TUNNEL_COVER_M` under it;
//! - **R3** the end of a run is elected by the run, not by the mapper;
//! - **R4** licenses are inputs, and read level ordinals rather than heights.
//!
//! Two more of the plan's specimens are not here, because the terrain dial
//! cannot yet state them: a shallow graze on the approach to a deep bore
//! (`dominant_buried_seed`), and a run whose cover thins mid-way without
//! daylighting (the gully rule). Both need a ground that is the sum of two
//! features, and the field takes one. They are named so the gap is a
//! deferral rather than an omission.

#[cfg(test)]
mod tests {
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;
    

    use crate::step::Summary;
    
    
    

    /// The width of a `residential` way, in metres: what a roadway area is
    /// divided by to read back a length.
    const W: f64 = 5.5;

    /// What one specimen came to: the three summaries any of these checks
    /// reads. The profile says what the heights are, the bench says what the
    /// ground had to do about them, and the structure says what was built —
    /// a rule about structures that leaves the other two alone has not been
    /// stated properly.
    struct Ran {
        profile: Summary,
        /// Where the cut fell: how much of a bore went to its portal
        /// cuttings, among the rest.
        partition: Summary,
        bench: Summary,
        structure: Summary,
    }

    /// A world on `ground` with the network of `net`, built to the end.
    fn run(ground: &str, net: &str) -> Ran {
        let (_, ran) = built(ground, net, None, 5.0, &upto(Step::Structure));
        Ran {
            profile: ran.of(Step::Profile),
            partition: ran.of(Step::Partition),
            bench: ran.merged(&crate::pipeline::tests::BENCH),
            structure: ran.of(Step::Structure),
        }
    }

    /// The length of roadway the structure step swept, in metres.
    fn roadway_m(r: &Ran) -> f64 {
        r.structure.num("span_m2") / W
    }

    /// A 30 m slot 40 m across, at the middle of a 200 m way: the way is
    /// carried over it on 40 m of deck landing on its two rims.
    const GORGE: &str = "gorge?depth=30&width=40";

    /// A 40 m mass 120 m across, at the middle of a 400 m way.
    const RIDGE: &str = "ridge?height=40&width=120";

    // ---------------------------------------------------------------- R2

    /// **A bridge from the terrain.** The source says nothing; the ground
    /// says a 30 m slot crosses the way. A deck is what is there.
    ///
    /// **Landed 2026-09-10** (migration step 5b, the terrain prior applied).
    /// Before it: `decks=0`, and the street dived in and out at 20 % of its
    /// pairs, leaving 80 of 826 mesh edges the bench field was discontinuous
    /// across, the worst by 10.9 m.
    ///
    /// The chain is three steps long and no step knows about bridges. The
    /// closing **refuses** the slot as too deep to fill; the refusal is
    /// written into the way's span table as a prior; the profile chords
    /// across it like any other span; and §4.5's consequence rule reads a
    /// deck off the result. Nothing anywhere says "gorges get bridges".
    #[test]
    fn a_gorge_is_a_bridge_without_being_told() {
        let r = run(GORGE, "net:straight");
        assert_eq!(r.structure.num("decks"), 1.0, "one deck: {}", r.structure);
        // The deck is the gorge's 40 m plus one station of margin at each end
        // — the margin is what lands the edge on the rim rather than on the
        // last sample the closing refused — and not the way's 200.
        let deck = roadway_m(&r);
        assert!((40.0..=56.0).contains(&deck), "deck {deck:.1} m: {}", r.structure);
        // And nothing dives: the road is level across the slot, so the bench
        // has no discontinuity to build.
        assert_eq!(r.profile.num("steep"), 0.0, "the road dives: {}", r.profile);
        assert_eq!(r.bench.num("step"), 0.0, "the field breaks: {}", r.bench);
    }

    /// **A culvert is crossed, not dived into.** A notch inside the
    /// closing's budget — 4 m deep, 20 m across, well under `NOTCH_SPAN_M`
    /// and `NOTCH_FILL_MAX_M` — is ground continuity that was engineered,
    /// not relief. The road holds level over it and no structure is built:
    /// the conditioned reference is what makes both true at once.
    ///
    /// **Landed 2026-09-10** (migration step 3): the profile solves against
    /// the reference, the closing fills the notch, and the road holds level
    /// across it. Before that it read `steep 4/50` — a dive through a ditch
    /// the road was engineered over.
    #[test]
    fn a_culvert_is_crossed_not_dived_into() {
        let r = run("gorge?depth=4&width=20", "net:straight");
        assert_eq!(r.structure.num("decks"), 0.0, "no deck over a culvert: {}", r.structure);
        assert_eq!(r.profile.num("steep"), 0.0, "the road dives: {}", r.profile);
    }

    /// **A deck the DEM has swallowed.** `shelf` is a 30 m trench with a
    /// causeway across it: level along the way from rim to rim, falling away
    /// 8 m to either side. A surface DEM that has imaged a viaduct as ground.
    /// Read on the axis alone the road is perfectly at grade; the flanks are
    /// the only evidence, and the blindness mask is what turns them into an
    /// answer.
    ///
    /// Today every instrument in the run reads zero — `steep 0/50`,
    /// `cut 0.000`, `fill 0.000`, `off 1.7e-13`, `decks=0`. That is the
    /// specimen's point: the defect produces no signal at all, which is why
    /// the guard cannot live downstream of the reference.
    /// **Blocked on the terrain, not on the partition.** The mask marks the
    /// run and the prior can be painted — but the consequence rule reads
    /// `h − ground`, and over a causeway the ground *is* the deck: the chord
    /// and the DEM agree exactly, so no departure exists to derive. Painting
    /// it anyway put eight spans on the loop box that built no geometry.
    /// What this needs is the **terrain** to lose the causeway the DEM
    /// imaged, which is the ground stage's work and not a span table's.
    #[test]
    #[ignore = "needs the ground stage to carve an imaged deck out of the DEM"]
    fn a_shelf_is_a_deck_the_dem_has_swallowed() {
        let r = run("shelf?drop=30&width=40&flank=8", "net:straight");
        assert_eq!(r.structure.num("decks"), 1.0, "one deck: {}", r.structure);
        // The 40 m of causeway, on the trench's own rims.
        assert!((roadway_m(&r) - 40.0).abs() < 6.0, "deck length: {}", r.structure);
    }

    // ---------------------------------------------------------------- R3

    /// **A generous annotation is trimmed to the gorge.** The mapper's span
    /// is 80 m over a 40 m slot. The 20 m at each end never leaves the
    /// ground, and a deck built there is a slab seated on a block over
    /// ground it never left.
    ///
    /// **Landed 2026-09-10** (migration step 5c). Before it: `decks=1` but
    /// 80 m of it (`roadway_m2 440`), only 11 of 21 of the span's stations
    /// reading as a deck at all, and 2 of 11 seated on an abutment block over
    /// ground the deck never left.
    #[test]
    fn a_generous_annotation_is_trimmed_to_the_gorge() {
        let r = run(GORGE, "net:straight?span=0.30,0.70&kind=bridge");
        assert_eq!(r.structure.num("decks"), 1.0, "still one deck: {}", r.structure);
        assert!((roadway_m(&r) - 40.0).abs() < 6.0, "trimmed to the rims: {}", r.structure);
    }

    /// **A short annotation grows to the rims.** The mapper's span is 20 m
    /// over the same 40 m slot, so 10 m at each end of the gorge is left as
    /// open road — and the road there dives to a floor 30 m down.
    ///
    /// **Landed 2026-09-10.** Before it: `decks=1` of 20 m
    /// (`roadway_m2 110`), `steep 6/46`, and the bench building 68 of 772
    /// discontinuous edges, worst 8.3 m, standing 7.3 m off the ground.
    ///
    /// The annotation and the terrain each supply half of it. The source says
    /// *there is a bridge here* and the closing's refusal says *the slot is
    /// this wide*; painted together they say how long the bridge is, and the
    /// derivation then reads a deck off the chord that spans it.
    #[test]
    fn a_short_annotation_grows_to_the_rims() {
        let r = run(GORGE, "net:straight?span=0.45,0.55&kind=bridge");
        assert_eq!(r.structure.num("decks"), 1.0, "one deck: {}", r.structure);
        assert!((roadway_m(&r) - 40.0).abs() < 6.0, "grown to the rims: {}", r.structure);
        assert_eq!(r.profile.num("steep"), 0.0, "the approach dives: {}", r.profile);
        assert_eq!(r.bench.num("step"), 0.0, "the field breaks: {}", r.bench);
    }

    /// The same, downward: a bore mapped short of the mass it passes under
    /// leaves its two approaches climbing the ridge at 20 % and being
    /// benched, when they are inside the tunnel.
    ///
    /// Today: `bores=1` of 40 m over a 120 m ridge, `steep 18/90`,
    /// `bench step 32/1492` worst 5.2 m, `off 2.0`.
    /// **Half landed, and the other half needs a licence.** The terrain grows
    /// it: a refused crest overlapping the mapped tunnel extends it from the
    /// mapper's 40 m to the 64 m of mass the closing would not shave. Growing
    /// further — to where the road truly emerges from the ridge — means
    /// growing past the annotation, and given a free reach to do it a
    /// flat-ground underpass swallowed both its approach cuttings whole. That
    /// needs the server's licence (a crossing the buried tail passes beneath),
    /// which this model has not got. The approaches then still climb the
    /// remaining flanks, which is the same deferral the crest prior has:
    /// **the terrain says a mass is passed through, not where the road gets
    /// in.**
    #[test]
    #[ignore = "needs a licence to grow a span past its annotation, and a portal the class can reach"]
    fn a_bore_grows_to_its_portals() {
        let r = run(RIDGE, "net:straight?len=400&span=0.45,0.55&kind=tunnel");
        assert_eq!(r.structure.num("bores"), 1.0, "one bore: {}", r.structure);
        assert!((roadway_m(&r) - 120.0).abs() < 12.0, "grown to the portals: {}", r.structure);
        assert_eq!(r.profile.num("steep"), 0.0, "the approach climbs: {}", r.profile);
    }

    /// And a bore mapped longer than the mass is trimmed to it, or the tube
    /// is drawn out in the open at both ends.
    ///
    /// **Landed 2026-09-10** (migration step 5d, the elected end criterion).
    /// Before it: `bores=1` of 200 m over a 120 m ridge (`roadway_m2 1100`),
    /// with 22 of 51 of the span's stations not reading as a bore at all.
    ///
    /// The bore's ends are the *line's* crossings here, not the roof's,
    /// because the tube fits over the majority of the buried run: a real bore
    /// holds its tube almost everywhere and grazes only at its mouths.
    ///
    /// **Since 2026-09-11 those shallow mouths are open cutting.** Between
    /// the line's crossing and the roof's fit the road runs under the ground
    /// by less than its tube: the span carried it on with nothing over it but
    /// the terrain, and the portal a camera should see was hill
    /// (`structure covered`). The partition now gives that stretch back to
    /// the ground ([`crate::partition`]'s `open_portals`), so the elected
    /// bore is still trimmed to the mass, and it is the tube and its two
    /// cuttings together that span it.
    #[test]
    fn a_generous_bore_is_trimmed_to_the_mass() {
        let r = run(RIDGE, "net:straight?len=400&span=0.25,0.75&kind=tunnel");
        assert_eq!(r.structure.num("bores"), 1.0, "one bore: {}", r.structure);
        let (tube, cutting) = (roadway_m(&r), r.partition.num("portal_m"));
        assert!((tube + cutting - 120.0).abs() < 12.0, "trimmed to the mass: {tube} + {cutting}, {}", r.structure);
        assert!(tube < 120.0 && cutting > 0.0, "the mouths are cut open: {tube} + {cutting}");
        assert_eq!(r.structure.num("open"), 0.0, "the tube breaks surface: {}", r.structure);
        assert_eq!(r.structure.num("covered"), 0.0, "the terrain lies on the road: {}", r.structure);
    }

    // ------------------------------------------- the terrain prior's gate

    /// **A motorway that cannot climb a ridge passes under it.** The mirror
    /// of the refused notch, and the reason it is safe: the gate is the
    /// class's own ladder. Held to 6 % inside an 8 m box, a motorway cannot
    /// cross 40 m of mass in 120 m, and where it cannot the crest is
    /// something it goes through.
    ///
    /// **Landed 2026-09-10** (migration step 5e). Before it: no bore,
    /// `grade 20/100` breaking the ceiling at a fifth of its pairs, the
    /// deviation box spent to its 8 m limit end to end, and the ground
    /// closing an 8.3 m wall along a third of the section (`walled 111/330`,
    /// `wall_m2 1792`).
    #[test]
    fn a_ridge_a_motorway_cannot_climb_is_a_bore() {
        let r = run(RIDGE, "net:straight?len=400&class=motorway");
        assert_eq!(r.structure.num("bores"), 1.0, "one bore: {}", r.structure);
        // The mass is passed through rather than cut open: the ground owes
        // almost no wall where it used to close a third of the section.
        assert!(r.bench.num("wall_m2") < 300.0, "the ground still walls it: {}", r.bench);
        // And the approach, standing eight metres off the ground on its own
        // deviation box, reads as the deck it is — R2, on a fill no ground
        // stage could batter.
        assert_eq!(r.structure.num("decks"), 1.0, "{}", r.structure);
        // **The approaches still break grade, and that is a deferral, not a
        // bug in this rule.** The prior covers the run the opening refused —
        // the crest's own core — so the portals land on its shoulders, thirty
        // metres over the flat ground either side, and a motorway held to 6 %
        // cannot climb to them in the forty metres of flank that remain.
        // Placing a portal where the class can actually reach it is a design
        // this model has not made; what the terrain can say is *that* a mass
        // is passed through, and it says it.
        assert!(r.profile.num("grade") <= 10.0, "grade got no better: {}", r.profile);
    }

    /// **And a street climbs the same ridge.** The gate, from the other
    /// side: the DEM under a street *is* the street (S9), a lane mapped at
    /// 26 % really climbs 26 %, and no prior may turn that into a tunnel.
    ///
    /// This one passes today and must keep passing — it is the regression
    /// guard on the check above, not a to-do.
    #[test]
    fn a_street_climbs_the_ridge_it_is_mapped_over() {
        let r = run(RIDGE, "net:straight?len=400");
        assert_eq!(r.structure.num("bores"), 0.0, "a street got a bore: {}", r.structure);
        assert_eq!(r.structure.num("decks"), 0.0, "a street got a deck: {}", r.structure);
        // It climbs, and that is the right answer: 26 % of its station pairs
        // are over the 15 % a street's *lift* may use, and none is limited.
        assert!(r.profile.num("steep") > 0.0, "the street was limited: {}", r.profile);
        assert_eq!(r.profile.num("grade"), 0.0, "a street has no ceiling: {}", r.profile);
    }
}
