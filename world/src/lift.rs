//! The lift: the room holds its height.
//!
//! The mesh step put every paved triangle on the raw ground, coplanar with
//! it. This step gives the room the height the profile solved.
//!
//! **The room takes the profile.** The height of a point of the room — the
//! carriageway, its kerb returns and the pavement together — is the profile
//! height at the perpendicular foot on the nearest carriageway axis, and
//! the pavement stands [`crate::standard::KERB_RISE_M`] above it. The road is therefore
//! level crosswise: a 5.5 m residential along the contour of a 30 % slope
//! is cut 0.825 m at its uphill kerb and filled 0.825 m at its downhill
//! one, exactly, because half its width times the slope is what the ground
//! does across it. There is no crossfall, and the earth those two numbers
//! name is the earth the `earthwork` step answers with.
//!
//! Only the *ground* pieces' axes are in the field. A deck's height is the
//! chord the profile solved; were it in the field, the road under a viaduct
//! would take the viaduct's height wherever the deck's axis happened to be
//! the nearer one.
//!
//! **A cross-section reaches as far as the room does.** The pavement rides
//! the road while it is within [`crate::standard::ROOM_REACH_M`] of the asphalt's edge —
//! the reach the room step itself paves to — and a walk band farther out
//! than that comes down a face at [`crate::standard::EARTHWORK_BATTER`] and stops exactly
//! where it meets the ground. Neither half of that rule will do alone.
//! Asked per region, a footway that merely touches a kerb at one end would
//! be dragged up the slope with the road at its far end. Asked per point
//! with nothing in between, it would stand on a cliff at the line where the
//! answer changes. A fixed-width band down to the ground would, on a steep
//! flank, come out steeper than the wall it is there to avoid.
//!
//! **Where two carriageways' domains meet at different heights** — two
//! terraces on a flank, two one-way carriageways across a slope — the
//! heights step on the line between them, and the `bench` step draws a face
//! there: a retaining wall, which is what the hillside physically has. A
//! blend would ramp a pavement at 60 % between two terraces, which is
//! spectacle (invariant 6). The step is declared by the triangle's rule
//! ([`crate::copies::Rule`]), not stumbled on.
//!
//! **Except where they meet.** Every leg of a junction is level crosswise,
//! so on a flank the legs' cross-sections disagree everywhere off the
//! connector they share, and the nearest axis alone would step on the line
//! where two legs are equidistant: up to 0.875 m on a 15 % flank. So near a
//! connector two or more axes share, the legs meeting there are blended
//! ([`crate::field::Field::at`]): a warp in the junction, not a ramp between
//! terraces. A road that meets another nowhere near is never blended with
//! it, so the terraces keep their wall; and two legs whose blend would stand
//! the paving up steeper than one in one over their own grade are two rules
//! with a face between them, not a warp (`field::Joint::apart`).

use crate::copies::{boundaries, Copies, Fields, Rule, Stats, Surface};
use crate::field::Foot;
use crate::lattice::height_at;
use crate::poly;
use crate::step::Summary;
use crate::world::{Arrangement, Lifted, Mesh, Profiles, Sheets, Terrain};

/// Lifts every paved copy of the one mesh onto the profile.
pub fn run(terrain: &Terrain, profiles: &Profiles, mesh: &Mesh, sheets: &Sheets, arrangement: &Arrangement) -> (Lifted, Summary) {
    // Where the paving is over a span rather than on the ground, as the
    // arrangement tagged it ([`crate::arrangement::over_spans`]): the sheets'
    // span paving, the walk a deck carries, and a centimetre of rim, so the
    // hole and the lift agree on it.
    let over = poly::Indexed::new(&arrangement.over);
    let fields = Fields::new(profiles, sheets);
    let natural = |q: [f64; 2]| height_at(terrain, q[0], q[1]);

    // **Every paved triangle's rule, at its centroid.**
    let rules: Vec<Rule> = mesh
        .tri
        .indices
        .chunks_exact(3)
        .zip(&mesh.of_face)
        .map(|(t, &f)| {
            let Some(surface) = Surface::paved(arrangement.face(f)) else { return Rule::FREE };
            let c = [0usize, 1].map(|k| t.iter().map(|&v| mesh.tri.positions[v as usize][k]).sum::<f64>() / 3.0);
            fields.lift(surface, &over).rule(c, natural(c))
        })
        .collect();
    let mut copies = Copies::new(mesh, arrangement, &rules);
    let mut feet: [Vec<Option<Foot>>; 3] = Default::default();
    for (part, feet) in [&mut copies.carriageway, &mut copies.ballast, &mut copies.pavement].into_iter().zip(&mut feet) {
        for i in 0..part.tri.positions.len() {
            let (surface, rule) = part.key[i];
            let q = part.tri.positions[i];
            let (h, foot) = fields.lift(surface, &over).height([q[0], q[1]], part.natural[i], rule);
            part.tri.positions[i][2] = h;
            feet.push(foot);
        }
    }
    let welded = copies.weld();
    let bounds = boundaries(mesh, arrangement, &rules);
    let summary = Summary::new().with("axes", fields.axes()).with("welded", welded);
    (Lifted { fields, copies, feet, bounds }, summary)
}

/// Where the lifted room's vertices stand: taking the road's height, on a
/// batter face, or at the natural ground, and the cut and fill that leaves
/// the ground to answer. Counted once welded, over the copies a triangle
/// still names: a copy merged into its twin is not a vertex of the world.
pub fn check(arrangement: &Arrangement, lifted: &Lifted) -> Summary {
    let over = poly::Indexed::new(&arrangement.over);
    let copies = &lifted.copies;
    let mut stats = Stats::default();
    for (part, feet) in [&copies.carriageway, &copies.ballast, &copies.pavement].into_iter().zip(&lifted.feet) {
        let mut used = vec![false; part.tri.positions.len()];
        for &i in &part.tri.indices {
            used[i as usize] = true;
        }
        for i in (0..used.len()).filter(|&i| used[i]) {
            let (surface, rule) = part.key[i];
            lifted.fields.lift(surface, &over).account(&mut stats, rule, part.tri.positions[i][2], part.natural[i], feet[i]);
        }
    }
    Summary::new()
        .with_part("lifted", stats.lifted, stats.vertices)
        .with("battered", stats.battered)
        .with("draped", stats.draped)
        .with("cut", format!("{:.3}", stats.cut))
        .with("fill", format!("{:.3}", stats.fill))
        .with("flown", stats.flown)
        .with_part("free", stats.free, stats.draped)
}

#[cfg(test)]
mod tests {
    use crate::field::Field;
    use crate::pipeline::tests::stood as world;
    use crate::poly::Pt;
    use crate::standard::{KERB_RISE_M, MAX_BATTER_FACE_M, ROOM_REACH_M};
    use crate::world::{Bench, Material, Tri, World};

    fn bench(w: &World) -> &Bench {
        w.bench.as_ref().unwrap()
    }

    #[test]
    fn flat_ground_moves_nothing() {
        let (w, s) = world("flat", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "{s}");
        assert_eq!(s.num("cut"), 0.0);
        assert_eq!(s.num("fill"), 0.0);
        assert_eq!(s.num("step"), 0.0);
        assert_eq!(s.num("lifted"), b.carriageway.positions.len() as f64);
    }

    #[test]
    fn a_road_along_the_contour_is_level_crosswise() {
        // A 5.5 m residential across a 30 %
        // slope. Its axis is level, so the road is level, and the ground
        // it stands in is half the width times the slope — 0.825 m of cut
        // at the uphill kerb and 0.825 m of fill at the downhill one,
        // exactly. That is the earth the ground has still to move.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "the road is not level");
        assert!((s.num("cut") - 0.825).abs() < 1e-6, "{s}");
        assert!((s.num("fill") - 0.825).abs() < 1e-6, "{s}");
        assert_eq!(s.num("step"), 0.0, "{s}");
        // The road reaches both kerbs: the two numbers above are the
        // ground at the edges of the asphalt, not at some point inside it.
        assert!(b.carriageway.positions.iter().any(|p| (p[1] - 2.75).abs() < 1e-9));
        assert!(b.carriageway.positions.iter().any(|p| (p[1] + 2.75).abs() < 1e-9));
    }

    #[test]
    fn a_road_up_the_slope_is_the_slope() {
        // The same 30 % ramp with the road running straight up it: a
        // street is not grade-limited, so its profile is the ground and
        // the lift moves nothing at all (S9).
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200", None);
        assert!(s.num("cut") < 1e-9 && s.num("fill") < 1e-9, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().expect("the terrain step ran");
        for p in &b.carriageway.positions {
            assert!((p[2] - crate::lattice::height_at(t, p[0], p[1])).abs() < 1e-9, "{p:?}");
        }
    }

    #[test]
    fn the_pavement_stands_a_kerb_above_the_road() {
        let (w, s) = world("flat", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(!b.pavement.positions.is_empty());
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9), "{s}");
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9));
        assert_eq!(s.num("draped"), 0.0, "a sidewalk is pavement, not a free band: {s}");
    }

    /// **Under an overpass there are two roads, and neither is hung on the
    /// other.** The street keeps its asphalt at the ground under the deck,
    /// the deck keeps its own at its height over it, and no triangle of the
    /// asphalt or of the kerb's faces reaches from one to the other. The
    /// three rules that hold it are `sheet`'s apart axes, the arrangement's
    /// second layer of deck faces (`Arrangement::decks`) and the slab cap in
    /// `bench::edge_faces`; without them the street under a viaduct is bare
    /// terrain and the deck's asphalt hangs down onto it in curtains.
    #[test]
    fn a_street_under_a_deck_is_paved_and_the_deck_is_not_hung_on_it() {
        let (w, s) = world("flat?h=400", "net:overpass?len=201", None);
        let b = bench(&w);
        // Heights of the asphalt over a point of the crossing square, off
        // every edge of it.
        let q = [0.3, 0.2];
        let mut over: Vec<f64> = Vec::new();
        let c = &b.carriageway;
        for t in c.indices.chunks_exact(3) {
            let v: Vec<[f64; 3]> = t.iter().map(|&i| c.positions[i as usize]).collect();
            let side = |a: [f64; 3], b: [f64; 3]| (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]);
            let d = [side(v[0], v[1]), side(v[1], v[2]), side(v[2], v[0])];
            if d.iter().all(|x| *x > 0.0) || d.iter().all(|x| *x < 0.0) {
                over.extend(v.iter().map(|p| p[2]));
            }
        }
        assert!(over.iter().any(|z| (z - 400.0).abs() < 0.5), "no street under the deck: {over:?} {s}");
        assert!(over.iter().any(|z| *z > 405.0), "no deck over the street: {over:?} {s}");
        // The tallest triangle of each: a ramp's rise over a cell, not a
        // curtain. The deck climbs 6.5 m over some 45 m, so nothing honest
        // is near five.
        for (name, t) in [("carriageway", &b.carriageway), ("kerb", &b.kerb)] {
            let worst = t
                .indices
                .chunks_exact(3)
                .map(|tr| {
                    let z: Vec<f64> = tr.iter().map(|&i| t.positions[i as usize][2]).collect();
                    z.iter().cloned().fold(f64::MIN, f64::max) - z.iter().cloned().fold(f64::MAX, f64::min)
                })
                .fold(0.0, f64::max);
            assert!(worst < 5.0, "{name} has a triangle {worst:.2} m tall: {s}");
        }
    }

    #[test]
    fn the_pavement_rides_the_road_it_is_beside() {
        // On the contour, the sidewalk 6 m off the axis is level with the
        // road plus the kerb — not on the hillside it lies on, which at
        // 6 m out stands 1.8 m higher.
        let (w, _) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9));
        let m = w.mesh.as_ref().unwrap();
        let pavement = m.view(w.arrangement.as_ref().unwrap(), |f| f.material == Material::Pavement);
        assert!(pavement.positions.iter().any(|p| p[2] > 401.5), "the ground under it climbs");
    }

    #[test]
    fn a_footway_leaving_the_room_leaves_it_over_one_face() {
        // The stub footway runs 20 m north from the kerb, up a 30 %
        // slope. Its first metres are pavement and ride the road; past
        // the room's reach it comes down a face at 1 in 2.5; and where
        // the hill has climbed further than one face can follow, the band
        // is no longer this road's pavement and takes the ground.
        //
        // That last transition is a wall, and the numbers of it are the
        // measurement. The room's plateau ends 8.75 m out, where the
        // ground stands 2.51 m over the road. The face closes 0.4 m of
        // the difference per metre and the hill opens 0.3 m of it, so
        // they never meet: at 10.4 m out the difference reaches the
        // 3 m a face may be and the band steps to the ground, 2.34 m of
        // wall in the continuum and 3.53 m across the mesh edge that
        // straddles it. It is counted, it is marked in the plan view, and
        // the ground's answer — a batter cut into the hill, in the
        // `earthwork` step — is what removes it.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:stub?d=0.5", None);
        let b = bench(&w);
        assert!(s.num("lifted") > 0.0 && s.num("draped") > 0.0, "{s}");
        // Bounded: the plateau is 2.51 m of cut and nothing else is, and
        // exactly that. The arrangement cuts the walk at the room's reach,
        // so a vertex lands on the plateau's own edge 8.75 m out: 8.75 m of
        // 30 % slope less the kerb's rise, 2.505.
        assert!((s.num("cut") - 2.505).abs() < 0.01, "{s}");
        // Asked per region, the far end of this footway would be dragged
        // metres into the air.
        assert!(s.num("cut") < 3.0, "{s}");
        // And it is **drawn**: the band's triangles on the face and those
        // that drape take two rules, and the edge between them is split with
        // a face on it rather than stretched across — so it is a wall and
        // not a step.
        // One edge is left, and it is a centimetre wide: where the band's
        // face meets its drape *on the outline*, two edges pin one vertex
        // differently and the ground has one copy there, so the earthwork is
        // two-valued at that point. Continuing the split into the ground
        // would remove it; it is counted, and bounded.
        assert!(s.num("step") <= 1.0 && s.num("worst") < 0.6, "{s}");
        assert!(s.num("split_m2") > 0.0 && s.num("kerb_max") < 4.0, "{s}");
        // The wall stands where the hill outruns the face, not at the kerb:
        // every face taller than a kerb is out beyond the plateau.
        for q in b.kerb.positions.chunks_exact(4) {
            if (q[0][2] - q[1][2]).abs().max((q[3][2] - q[2][2]).abs()) > 2.0 * KERB_RISE_M {
                assert!(q[0][1] > 2.75 + ROOM_REACH_M, "a wall at the kerb: {q:?}");
            }
        }
    }

    /// A band whose lift/drape boundary falls in its own **interior**,
    /// which is the case the rest of this corpus cannot express.
    ///
    /// A mapped sidewalk is centred on the cut: 2.75 m of half-width plus
    /// the room's 6 m reach puts it 8.75 m off the axis, and the band is
    /// [`crate::width::WALK_M`] wide, so half lies within the reach and half
    /// beyond. On a 50 % flank the ground there stands 4.255 m over the road
    /// — 8.75 × 0.5 less the kerb's rise, past one face — so the far half
    /// drapes to the ground while the near half rides the road.
    ///
    /// **Every other specimen puts that boundary on a band's own edge**,
    /// where it is already a rim, and every face the bench draws is built
    /// off a rim: a narrow band cannot reach across the reach, and only a
    /// mapped walk can. Both ways this can regress are caught here — merge
    /// the sheets and an edge spans the step; give the far sheet `batter`
    /// instead of `face` and the two agree on the cut, putting the step back
    /// one vertex out, inside the far sheet, where mesh edges span the drop
    /// instead of a face.
    #[test]
    fn a_band_across_the_reach_steps_on_a_rim_not_inside_itself() {
        let (w, s) = world("ramp?grade=0.5&bearing=0&radius=100000", "net:sidewalk?d=8.75", None);
        let b = bench(&w);
        assert!(!b.pavement.positions.is_empty(), "{s}");
        // Not vacuous: the band straddles, so both rules fire on it, and
        // the drop it straddles is the one a face cannot follow.
        assert!(s.num("lifted") > 0.0 && s.num("draped") > 0.0, "the band must straddle: {s}");
        assert!((s.num("cut") - 4.255).abs() < 0.01, "{s}");
        assert!(s.num("cut") > MAX_BATTER_FACE_M, "the drop must be past one face: {s}");
        // The property: no mesh edge spans the step. The sheets share no
        // vertex along the cut, so the drop is a face and not a stretch.
        assert_eq!(s.num("step"), 0.0, "an edge spans the walk's own step: {s}");
        assert_eq!(s.num("worst"), 0.0, "{s}");
        // And nothing in the pavement stands up: the steepest triangle is the
        // far half following the ground's batter — the 50 % flank, 1 in 2.5
        // of residual on it, and a little more where two outline segments
        // hand over and the blend's weight turns (0.993 here) — not the
        // 4.255 m the field steps by, which is a face.
        let steepest = b
            .pavement
            .indices
            .chunks_exact(3)
            .map(|t| {
                let [p, q, r] = [t[0], t[1], t[2]].map(|i| b.pavement.positions[i as usize]);
                let (u, v) = ([q[0] - p[0], q[1] - p[1], q[2] - p[2]], [r[0] - p[0], r[1] - p[1], r[2] - p[2]]);
                let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
                if n[2].abs() < 1e-12 { 0.0 } else { n[0].hypot(n[1]) / n[2].abs() }
            })
            .fold(0.0f64, f64::max);
        assert!(steepest < 1.0, "a pavement triangle climbs {steepest:.3}: {s}");
    }

    #[test]
    fn a_junction_on_a_hill_has_one_height() {
        // Four legs meeting at the origin: the profile pins the connector,
        // so the four surfaces meet there without a step, and the field is
        // continuous across the whole junction.
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        assert_eq!(s.num("step"), 0.0, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().expect("the terrain step ran");
        let centre: Vec<f64> =
            b.carriageway.positions.iter().filter(|p| p[0].hypot(p[1]) < 1.0).map(|p| p[2]).collect();
        assert!(!centre.is_empty());
        let top = crate::lattice::height_at(t, 0.0, 0.0);
        assert!(centre.iter().all(|h| (h - top).abs() < 1e-9), "the junction is not at the ground's height");
    }

    /// The three steepest edges of `tri` — rise over run, and where the
    /// edge lies — over the edges long enough for a grade to mean
    /// something. Where the worst edge is says which rule made it: a
    /// junction's own corner, or the ring where the blend hands back.
    fn steepest(tri: &Tri) -> Vec<(f64, Pt)> {
        let mut edges: Vec<(f64, Pt)> = tri
            .indices
            .chunks_exact(3)
            .flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
            .map(|(a, b)| {
                let (p, q) = (tri.positions[a as usize], tri.positions[b as usize]);
                let run = (q[0] - p[0]).hypot(q[1] - p[1]);
                let grade = if run < 0.1 { 0.0 } else { (q[2] - p[2]).abs() / run };
                (grade, [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])
            })
            .collect();
        edges.sort_by(|a, b| b.0.total_cmp(&a.0));
        edges.truncate(3);
        edges
    }

    #[test]
    fn a_junction_on_a_slope_is_one_surface() {
        // Every leg of a junction is level crosswise, so on a flank the
        // legs' cross-sections disagree everywhere off the connector: by
        // g·s where a leg climbing the flank meets one along its contour,
        // by twice that where two legs climb it at 45°. Asked of the
        // nearest axis alone, the room would step on the line where two
        // legs are equidistant — 0.875 m on a 15 % flank — and every
        // junction on a hillside would read as a bump.
        let mut got = Vec::new();
        for ground in ["ramp?grade=0.15&bearing=0&radius=100000", "ramp?grade=0.15&bearing=45&radius=100000"] {
            for net in ["net:tee", "net:cross"] {
                let (w, s) = world(ground, net, None);
                let worst = steepest(&bench(&w).carriageway);
                let from_joint = worst[0].1[0].hypot(worst[0].1[1]);
                eprintln!("{ground} {net}: step={} worst={worst:.3?} at {from_joint:.1} m from the joint", s.num("step"));
                got.push((ground, net, s.num("step"), worst[0].0));
            }
        }
        // What is left in the corners is a twist, not a step: a level road
        // meeting a 15 % side street has to warp through its kerb returns,
        // and every worst edge is one of those, ~6 m from the connector.
        // The bound sits just above those twists, which read well under it
        // for any blend width, and well below the 1 to 2 the nearest axis
        // alone gives.
        for (ground, net, step, steep) in got {
            assert_eq!(step, 0.0, "{ground} {net}");
            assert!(steep < 0.5, "{ground} {net}: an edge of the junction climbs {steep:.3}");
        }
    }

    /// The track bed takes its railway's height at its foot, and nothing
    /// else rides the railway: the asphalt crossing it is the road's.
    #[test]
    fn the_ballast_rides_its_railway_and_nothing_else_does() {
        let (w, s) = world("hill?amp=20&radius=150", "net:level?len=300", None);
        let b = bench(&w);
        assert!(!b.ballast.indices.is_empty(), "{s}");
        let profiles = &w.solved().unwrap().profiles;
        let rail = profiles.iter().find(|p| p.id == "rail").unwrap();
        let road = profiles.iter().find(|p| p.id == "road").unwrap();
        let (rails, roads) = (Field::new(std::slice::from_ref(rail)), Field::new(std::slice::from_ref(road)));
        for v in &b.ballast.positions {
            let foot = rails.at([v[0], v[1]]).expect("every bed vertex is beside its railway");
            assert!((v[2] - foot.h).abs() < 1e-9, "{v:?} vs {}", foot.h);
        }
        for v in &b.carriageway.positions {
            let foot = roads.at([v[0], v[1]]).expect("every asphalt vertex is beside its road");
            assert!((v[2] - foot.h).abs() < 1e-9, "{v:?} vs {}", foot.h);
        }
    }
}
