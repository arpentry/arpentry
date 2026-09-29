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
//! does across it. A crossfall is a later refinement, and the earth those
//! two numbers name is the earth the ground answers with in the next step.
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
//! Asked per region, a 20 m footway that merely touches a kerb at one end
//! was dragged 5.88 m up the ramp with the road at its far end. Asked per
//! point with nothing in between, it would stand on a 3.8 m cliff at the
//! line where the answer changed. A fixed band was tried first and
//! rejected: 6 m of it on the box's 30 % flank came out at 103 %, steeper
//! than the wall it was there to avoid.
//!
//! **Where two carriageways' domains meet at different heights** — two
//! terraces on a flank, two one-way carriageways across a slope — the
//! heights step on the line between them, and the bench step draws a face
//! there: a retaining wall, which is what the hillside physically has. A
//! blend would ramp a pavement at 60 % between two terraces, which is
//! spectacle (invariant 6). The step is declared by the triangle's rule
//! ([`crate::copies::Rule`]), not stumbled on.
//!
//! **Except where they meet.** Every leg of a junction is level crosswise,
//! so on a flank the legs' cross-sections disagree everywhere off the
//! connector they share, and the nearest axis alone stepped on the line
//! where two legs are equidistant: up to 0.875 m on a 15 % flank. So near a
//! connector two or more axes share, the legs meeting there are blended
//! ([`crate::field::Field::at`]): a warp in the junction, not a ramp between
//! terraces. A road that meets another nowhere near is never blended with
//! it, so the terraces keep their wall.

use crate::copies::{boundaries, Copies, Fields, Rule, Surface};
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
            fields.lift(surface, Some(&over)).rule(c, natural(c))
        })
        .collect();
    let mut copies = Copies::new(mesh, arrangement, &rules);
    let mut feet: [Vec<Option<Foot>>; 3] = Default::default();
    for (part, feet) in [&mut copies.carriageway, &mut copies.ballast, &mut copies.pavement].into_iter().zip(&mut feet) {
        for i in 0..part.tri.positions.len() {
            let (surface, rule) = part.key[i];
            let q = part.tri.positions[i];
            let (h, foot) = fields.lift(surface, Some(&over)).height([q[0], q[1]], part.natural[i], rule);
            part.tri.positions[i][2] = h;
            feet.push(foot);
        }
    }
    let welded = copies.weld();
    let bounds = boundaries(mesh, arrangement, &rules);
    let summary = Summary::new().with("axes", fields.axes()).with("welded", welded);
    (Lifted { fields, rules, copies, feet, bounds, welded }, summary)
}
