//! What a viewer sees: the census of the drawn world, asked of every corpus
//! specimen ([`crate::census`]).
//!
//! > **Nothing drawn is open, stands up, folds, fights or is buried.**
//!
//! Every other spec states one rule of one construction. This one states
//! what all of them owe together, on the triangles a viewer draws, so a fix
//! that moves a defect from one step into the next is caught here even when
//! both steps' own lines read better. `crack` is counted and not asserted:
//! a T-junction is nothing to see through.
//!
//! The specimens are `scripts/world-corpus.sh`'s, on its five-metre lattice.
//! A specimen that does not read clean yet is `#[ignore]`d with what it
//! reads, so `cargo test -- --ignored` is the list of what is still visible.

#[cfg(test)]
mod tests {
    use crate::census::{Census, Species};
    use crate::pipeline::tests::{built, upto};
    use crate::step::Step;

    /// The species a viewer can see: all but `crack`.
    const CHARGED: [Species; 6] =
        [Species::Gap, Species::Fin, Species::Flip, Species::Buried, Species::Fight, Species::Overlap];

    fn clean(ground: &str, net: &str, houses: Option<&str>) {
        let (w, _) = built(ground, net, houses, 5.0, &upto(Step::Building));
        let c = Census::take(&w);
        let open: Vec<String> = CHARGED
            .iter()
            .filter(|&&s| c.total(s).0 > 0)
            .flat_map(|&s| c.defects.iter().filter(move |d| d.species == s).take(3))
            .map(|d| {
                format!(
                    "{} {:.3} (rise {:.2}, width {:.3}) at {:.2},{:.2},{:.2} in {}",
                    d.species.name(),
                    d.size,
                    d.rise,
                    d.width,
                    d.at[0],
                    d.at[1],
                    d.at[2],
                    d.layers.join("+")
                )
            })
            .collect();
        assert!(open.is_empty(), "census {}\n  {}", c.summary(), open.join("\n  "));
    }

    #[test]
    fn a_sidewalk_on_a_slope() {
        clean("ramp?grade=0.3&bearing=45", "net:sidewalk?d=6", None);
    }

    #[test]
    fn a_sidewalk_over_a_cliff() {
        clean("step?rise=10", "net:sidewalk?d=6", None);
    }

    #[test]
    fn a_house_the_way_runs_through() {
        clean("hill?amp=20&radius=300", "net:tee", Some("house:across?rot=30"));
    }

    /// Two fins of 0.23 m² in the junction's corners.
    #[test]
    #[ignore = "fin 2"]
    fn a_crossroads_on_a_slope() {
        clean("ramp?grade=0.15&bearing=45", "net:cross", None);
    }

    /// Paved triangles 45° steeper than a 150 % flank, up to 13 m tall:
    /// the ring and its legs pulled apart where the rules meet.
    #[test]
    #[ignore = "fin 86"]
    fn a_roundabout_on_a_flank() {
        clean("ramp?grade=1.5&bearing=45&radius=100000", "net:roundabout", Some("house:row?gap=2"));
    }

    #[test]
    #[ignore = "gap 4, fight 2"]
    fn an_overpass() {
        clean("flat", "net:overpass?len=201", None);
    }

    /// The bore's floor and the cutting's carriageway coplanar over the
    /// tube's reach past the portal.
    #[test]
    #[ignore = "fight 2"]
    fn an_underpass() {
        clean("flat", "net:underpass", None);
    }

    /// The road's rim open at each abutment, where the edge rule draws no
    /// face beside a span.
    #[test]
    #[ignore = "gap 4"]
    fn a_bridge_over_a_gorge() {
        clean("gorge?depth=30&width=40", "net:straight", None);
    }

    #[test]
    #[ignore = "gap 5, fin 1, fight 2"]
    fn a_motorway_through_a_ridge() {
        clean("ridge?height=40&width=120", "net:straight?class=motorway", None);
    }

    /// The kerb faces across the level crossing float off both surfaces.
    #[test]
    #[ignore = "gap 4"]
    fn a_level_crossing() {
        clean("ramp?grade=0.05", "net:level", None);
    }
}
