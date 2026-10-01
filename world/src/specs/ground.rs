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

    /// **A footpath between two pins that disagree is a ramp, not a fold.**
    ///
    /// Passive pavement takes the ground's residual, and where two outline
    /// pins that disagree face each other across it the batter's blend
    /// hands one over to the other in a metre: the path folds at two to four
    /// metres per metre. The rule: no footpath triangle stands on a residual
    /// steeper than the steepest batter (1 in 1) *or* than the pins it lies
    /// between make necessary — their difference over their distance, for
    /// every two pins within a batter's reach whose way through the triangle
    /// is hardly longer than the straight one. A corner at a split, where the
    /// ground meets two surfaces at one point at two heights, is exempt: no
    /// surface through it is gentler than the split, and that is the edge
    /// rule's face, not a ramp's.
    ///
    /// The specimen: a sidewalk and a driveway beside a row of houses on a
    /// 60 % flank. Past the room's reach the sidewalk drapes, and it runs
    /// between the natural ground uphill (pinned at none) and the corner of
    /// a cutting pinned three metres down. Before the ramp a footway triangle
    /// there stood on 2.21 m/m where its pins needed 1.34, and the census
    /// read two pavement fins of 17.6 m², up to 4.2 m tall; after it, one of
    /// 1.4 m², the sliver fanned round the split.
    #[test]
    fn a_footpath_between_two_pins_is_a_ramp_not_a_fold() {
        let (w, _) = built(
            "ramp?grade=0.6&bearing=0&radius=100000",
            "net:driveway?d=9",
            Some("house:row?gap=2"),
            5.0,
            &upto(Step::Earthwork),
        );
        let (terrain, mesh, e) = (w.terrain.as_ref().unwrap(), w.mesh.as_ref().unwrap(), w.earthwork.as_ref().unwrap());
        let natural = |q: [f64; 2]| crate::lattice::height_at(terrain, q[0], q[1]);
        let clamp = crate::standard::MAX_BATTER_FACE_M;
        // Every pin: where the outline meets the ground, and at what residual.
        let mut pins: Vec<([f64; 2], f64)> = Vec::new();
        for (&(u, v), top) in e.outline.iter().zip(&e.top) {
            for (w, h) in [(u, top[0]), (v, top[1])] {
                let q = mesh.tri.positions[w as usize];
                let q = [q[0], q[1]];
                pins.push((q, (h - natural(q)).clamp(-clamp, clamp)));
            }
        }
        let reach = crate::standard::EARTHWORK_BATTER * clamp + 1.0;
        let part = &e.copies.pavement;
        let (mut paths, mut worst) = (0, (0.0f64, 0.0f64, [0.0; 2]));
        for (t, &key) in part.tri.indices.chunks_exact(3).zip(&part.face_key) {
            if !crate::copies::passive(key) {
                continue;
            }
            let c = [0, 1, 2].map(|k| part.tri.positions[t[k] as usize]);
            let p = c.map(|q| [q[0], q[1]]);
            let twice = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
            if twice.abs() < 0.02 {
                continue;
            }
            paths += 1;
            let r = [0, 1, 2].map(|k| c[k][2] - part.natural[t[k] as usize]);
            let (d1, d2) = (r[1] - r[0], r[2] - r[0]);
            let (ux, uy, vx, vy) = (p[1][0] - p[0][0], p[1][1] - p[0][1], p[2][0] - p[0][0], p[2][1] - p[0][1]);
            let grade = ((d1 * vy - d2 * uy) / twice).hypot((ux * d2 - vx * d1) / twice);
            let at = [(p[0][0] + p[1][0] + p[2][0]) / 3.0, (p[0][1] + p[1][1] + p[2][1]) / 3.0];
            let near: Vec<&([f64; 2], f64)> =
                pins.iter().filter(|(q, _)| (q[0] - at[0]).hypot(q[1] - at[1]) <= reach).collect();
            // Two pins face each other across the triangle when it lies
            // between them: the way from one to the other through it is
            // hardly longer than the straight one.
            let dist = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
            let mut need = 1.0f64;
            for (i, a) in near.iter().enumerate() {
                for b in &near[i + 1..] {
                    let d = dist(a.0, b.0);
                    if d >= 0.5 && dist(a.0, at) + dist(at, b.0) <= 1.2 * d {
                        need = need.max((a.1 - b.1).abs() / d);
                    }
                }
            }
            // A corner at a split: pins the ground meets at two heights a
            // metre apart, within a hand's breadth of each other.
            let split = p.iter().any(|q| {
                let at_q: Vec<f64> =
                    pins.iter().filter(|(x, _)| (x[0] - q[0]).hypot(x[1] - q[1]) < 0.25).map(|x| x.1).collect();
                at_q.iter().any(|a| at_q.iter().any(|b| (a - b).abs() > 1.0))
            });
            if !split && grade - need > worst.0 - worst.1 {
                worst = (grade, need, at);
            }
        }
        assert!(paths > 0, "the specimen has no footpath to fold");
        let (grade, need, at) = worst;
        assert!(
            grade <= need + 0.1,
            "a footpath at {:.1},{:.1} stands on a residual {grade:.2} m/m steep where its pins need {need:.2}",
            at[0],
            at[1]
        );
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
