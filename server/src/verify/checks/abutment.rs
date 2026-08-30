//! Does the road survive the handoff at a bridge's end?
//!
//! docs/ROADS.md invariant 5: "The stroke (coarse zooms), the surface (detail
//! zooms), and the structure decks read the same centerline, the same width
//! function, and the same road-surface heights; **nothing moves at the
//! handoff**." Invariant 2 says the same thing in the height dimension — zero
//! step at shared geometry.
//!
//! An abutment is the handoff. `pipeline::process_feature` cuts a claimed
//! segment into `Corridor::pieces` at the solved span boundaries and emits each
//! piece separately: the at-grade approach as a draped road, the span as a
//! structure. Adjacent pieces **share their cut vertex exactly** — it is one
//! coordinate in the model, handed to two generators. So the two strokes that
//! meet at a span end started life as one point, and any distance between them
//! in the archive is something a generator moved. There is no prior to argue
//! about and no legitimate band: the correct answer is zero, to quantization.
//!
//! That makes this the cheapest possible instrument for a defect that is
//! otherwise only visible as a shape — a bridge that does not line up with its
//! own road. Two things move that point, and they are separated here because
//! they have different fixes:
//!
//! - [`seam.abutment_plan`] — the two pieces are on **different curves**. Both
//!   strokes ride the corridor's smoothed sweep line now (`synth::road::bake`
//!   snaps unconditionally), so the correct value really is zero; what a break
//!   here reports is one side leaving that line — a snap the
//!   `PAINT_SNAP_MAX_M` cap refused, a piece with no profile to snap to.
//! - [`seam.abutment_step`] — the deck ramp does not arrive at the road. The
//!   deck's height is `Profile::deck_m`, the approach's is `road_m`; the ramp
//!   fit is pinned back to the road at every anchored span boundary
//!   (`solve::profile::deck_ramp`), so a step is a boundary the pin does not
//!   reach, or the sweep line having slid *along* the alignment so the deck
//!   carries the height solved for a different station. A plan break and a
//!   height step can share a cause, and this separates them anyway, because
//!   either can occur alone.
//!
//! ## What is paired, and why it is not guesswork
//!
//! The archive does not carry corridor identity, so the pairing is made on what
//! it does carry: a stroke's `class`, whether a drawn structure **carries** its
//! end, and the fact that both are *part endpoints*.
//!
//! The carried test is what a `level` test cannot do here. A solved structure
//! emits twice — the solid, which takes the `level` ordinal, and the road paint
//! re-emitted over it so the carriageway continues across the span
//! (`pipeline::process_feature`) — and the paint is emitted *before* the
//! ordinal is pushed, so a viaduct's own stroke arrives at level 0 exactly like
//! the approach it meets. Reading `level` off the stroke therefore sees only
//! the fitted footbridges (`synth::draped`), which take their level another
//! way, and misses every solved bridge in the archive. So an end is called a
//! structure end when a deck or bore mesh covers it in plan and brackets its
//! height: the solid is the thing that says a structure is there, and it is
//! drawn.
//!
//! An at-grade stroke ends only where its span partition ends or where the tile
//! clipped it, and a clip cuts one stroke without producing a partner at the
//! same place, so an unpaired end is silently dropped rather than counted as a
//! break. A road passing *under* a bridge is not an endpoint at all and cannot
//! be paired with its abutment.
//!
//! Only ends inside the tile proper are measured. Strokes are clipped to the
//! tile plus its buffer, so a neighbour draws its own copy of every abutment
//! near a border; counting those would report each one several times.
//!
//! ## Where the population lives
//!
//! From `priors::ROAD_SURFACE_MIN_ZOOM` the union paves carriageway and rail
//! formation alike and `pipeline::paves_via_union` deletes both strokes, so at
//! the detail rung this check's population is the classes that still stroke —
//! street-running rail and draped ways carried by a solid — which on most
//! extracts is empty. That is by design: at those zooms the surface handoff is
//! measured on the meshes themselves (`seam.band_deck_*`, `verify::checks::handoff`),
//! and this file earns its keep on the pre-surface rungs, where the stroke is
//! the road.

use crate::verify::dist::Dist;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// How far apart, in plan, two ends may be and still be read as the two halves
/// of one abutment.
///
/// This is a *pairing* radius, not a threshold: it decides which ends belong to
/// each other, and the measurement is then whatever the distance turns out to
/// be. It has to exceed the break itself — a rule that only paired ends already
/// close together would report every abutment as perfect and the broken ones as
/// absent, which is the failure mode of a search radius chosen from the answer.
/// Twelve metres clears the deviation the centerline smoother is allowed
/// (`solve::profile::SMOOTH_MAX_DEV_M`) in each of the two components at once,
/// several times over, and it is a long way past anything the fix left behind.
const PAIR_MAX_M: f64 = 12.0;

/// How far the two ends' headings may differ and still be read as one road cut
/// in two, in radians.
///
/// A span boundary is a cut across a continuous alignment, not a corner: the
/// approach and the span leave it pointing the same way. Two ends that disagree
/// by more than this are two different roads whose ends happen to be near each
/// other — a footway meeting a street beside a bridge — and pairing them would
/// report the angle between two roads as a break in one. Deliberately loose at
/// about 34°, because the whole point is to measure how far the two ends have
/// moved apart, and a tight cone would reject the very breaks being looked for.
const PAIR_MAX_TURN_RAD: f64 = 0.6;

/// What counts as moved, in metres. Two vertices that were one coordinate are
/// quantized independently onto the tile lattice — about 1.9 cm at z16, so up
/// to ~2.6 cm of plan disagreement is the format and not the generator. Five
/// centimetres is past that and far below anything visible, which is the point:
/// the contract says *nothing* moves, so the gate sits just above the noise
/// rather than at a negotiated tolerance.
const BREAK_M: f64 = 0.05;

/// How far the approach is followed, and in what steps, looking for the asphalt
/// that should meet the abutment. Capped well past the longest half-segment a
/// mapped road carries between vertices, so a saturated march means "no band
/// here" rather than "the band is just past where I stopped".
const BARE_REACH_M: f64 = 30.0;
const BARE_STEP_M: f64 = 0.25;

/// How far a stroke's height may sit from where its structure solid carries
/// the road — a deck's *top*, a bore's floor plus the
/// [`crate::priors::DECK_THICKNESS_M`] the floor hangs below the road — and
/// still count as carried by it. Anchored there rather than anywhere inside
/// the solid's vertical range, because the range test cannot tell a carried
/// stroke from one passing *underneath*: a rail line under a bridge whose
/// clearance demand went unmet runs at soffit level, inside the range, and
/// read as that bridge's own stroke. The slack covers the millimetre
/// quantization of two separately encoded features and the deck's thickness
/// at a sloping end cap.
const CARRIED_SLACK_M: f64 = 1.0;

/// One end of a drawn road stroke.
struct End {
    class: String,
    /// Whether a drawn deck or bore carries this end — what makes it the
    /// structure half of a handoff rather than the approach half.
    carried: bool,
    px: f64,
    py: f64,
    h: f64,
    /// Direction of the segment this end terminates, pointing out of the part.
    heading: f64,
}

/// Whether a class draws a surface band of its own — a carriageway or a rail
/// formation, not a footway.
///
/// The population rule, not a nicety. A class with no surface contributes
/// nothing to the union (`synth::carriageway::carriageway_sources` skips it), so
/// "the band that continues this abutment" does not exist for it and the march
/// finds whatever asphalt happens to be nearest — a footway ending near a road
/// bridge measured 19 m of "bare ground" that is simply a footway. It is also
/// junior geometry that no corridor partition cut, so the shared-coordinate
/// premise is not its premise either.
fn paves(class: &str) -> bool {
    crate::priors::class_paves(class)
}

/// Whether two ends leave their cut along the same alignment. The two pieces
/// point away from each other there, so the approach's outward heading is the
/// reverse of the span's.
fn aligned(a: &End, b: &End) -> bool {
    let mut d = (a.heading - b.heading).abs() % std::f64::consts::TAU;
    if d > std::f64::consts::PI {
        d = std::f64::consts::TAU - d;
    }
    (d - std::f64::consts::PI).abs() <= PAIR_MAX_TURN_RAD
}

pub struct Abutment {
    plan: Dist,
    plan_worst: Worst,
    step: Dist,
    step_worst: Worst,
    bare: Dist,
    bare_worst: Worst,
    /// Structure ends that found no at-grade partner — a tile-clipped end, or a
    /// span meeting another span. Reported as a coverage limit rather than
    /// left to be assumed away.
    unpaired: usize,
    paired: usize,
    /// Carried ends whose nearest partner is a lower-indexed carried end: the
    /// other half of a flush joint already measured once. Reported so the
    /// population change from retiring the spurious re-pairings is visible.
    second_half: usize,
}

impl Abutment {
    pub fn new(opt: &Options) -> Abutment {
        Abutment {
            plan: Dist::metres(),
            plan_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            step: Dist::metres(),
            step_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            bare: Dist::metres(),
            bare_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            unpaired: 0,
            paired: 0,
            second_half: 0,
        }
    }
}

impl Check for Abutment {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        // Solved structures only. A *fitted* deck (`synth::draped` — a
        // footbridge, a path over a stream) is not cut from the corridor
        // partition at all: it is fitted to the finished ground, and its
        // abutment is deliberately walked along the bank to seat it on the wall
        // beside it (`contact.deck_seat`). Its end is therefore *supposed* to
        // leave the annotation's edge, so the shared-coordinate premise this
        // whole check rests on does not hold for it, and measuring it here
        // reported the seating rule working as an 11.9 m defect.
        // `(is bore, mesh)`: where the solid carries its road differs — a
        // deck on its top, a bore on the floor a deck thickness under it.
        let solids: Vec<(bool, &crate::verify::mesh::SurfaceMesh)> = tile
            .roads
            .iter()
            .filter(|m| (m.is_deck() || m.is_bore()) && !m.is_fitted_deck())
            .map(|m| (m.is_bore(), &m.mesh))
            .collect();
        if solids.is_empty() {
            return;
        }
        // The drawn at-grade asphalt: the interior band welded to its rim
        // rim, because the interior alone is an inset of the true silhouette
        // and marching to it would report the rim's width as bare ground.
        let paved: Vec<&crate::verify::mesh::SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|m| m.is_pavement() || m.is_rim())
            .map(|m| &m.mesh)
            .collect();
        let carried = |px: f64, py: f64, h: f64| {
            solids.iter().any(|(bore, m)| {
                m.height_range_at(px, py).is_some_and(|(lo, hi)| {
                    let road = if *bore { lo + crate::priors::DECK_THICKNESS_M } else { hi };
                    (h - road).abs() <= CARRIED_SLACK_M
                })
            })
        };
        let mut ends: Vec<End> = Vec::new();
        for line in &tile.lines {
            // Paint is not the road: a marking stroke ends where its dash does,
            // and pairing two dashes across a span boundary would measure the
            // dash phase. The carriageway's own stroke is the one thing here
            // that runs the length of a piece.
            if line.class == "marking" || !paves(&line.class) {
                continue;
            }
            for part in &line.parts {
                if part.len() < 2 {
                    continue;
                }
                let last = part.len() - 1;
                // Each end with the heading of the segment it terminates,
                // pointing *out* of the part, so two pieces cut from one line
                // leave the cut back to back.
                for (a, b) in [(part[1], part[0]), (part[last - 1], part[last])] {
                    let (dx, dy) = ((b.0 - a.0) * tile.scale.mx, (b.1 - a.1) * tile.scale.my);
                    if dx.abs() < 1e-12 && dy.abs() < 1e-12 {
                        continue;
                    }
                    ends.push(End {
                        class: line.class.clone(),
                        carried: carried(b.0, b.1, b.2),
                        px: b.0,
                        py: b.1,
                        h: b.2,
                        heading: dy.atan2(dx),
                    });
                }
            }
        }
        for (i, s) in ends.iter().enumerate() {
            if !s.carried || !tile.owns(s.px, s.py) {
                continue;
            }

            // The nearest aligned end of the same class, over *all* candidates
            // — carried or not. Nearest rather than any-within-radius: at a
            // junction of two same-class roads several ends sit close
            // together, and the one that shares this abutment is the one the
            // cut produced, which is the nearest by construction. And nearest
            // FIRST, eligibility second: at a flush joint both halves read as
            // carried (the deck's end cap covers the shared vertex), the pair
            // is counted from its lower index, and the higher index must then
            // contribute *nothing* — not go looking for the next candidate in
            // reach. Removing the true partner from the search instead of
            // skipping the sample paired the leftover half with whatever else
            // stood within 12 m: the far end of the same short span (its
            // length reported as a plan break) or the parallel track (the
            // climb across the span reported as a height step). Every top
            // offender of `seam.abutment_plan` was one of these — a 0.00 m
            // joint scored as 11 m.
            let mut best: Option<(f64, usize)> = None;
            for (j, g) in ends.iter().enumerate() {
                if i == j || g.class != s.class || !aligned(s, g) {
                    continue;
                }
                let d = tile.scale.dist(s.px, s.py, g.px, g.py);
                if d <= PAIR_MAX_M && best.is_none_or(|(bd, _)| d < bd) {
                    best = Some((d, j));
                }
            }
            let Some((plan, j)) = best else {
                self.unpaired += 1;
                continue;
            };
            if ends[j].carried && j < i {
                // The other half of a both-carried pair: measured once, from
                // its lower index.
                self.second_half += 1;
                continue;
            }
            let g = &ends[j];
            self.paired += 1;
            let step = (s.h - g.h).abs();
            self.plan.push(plan);
            self.step.push(step);
            // How much bare ground lies between the abutment and the asphalt
            // that should meet it: from the approach end, march *into* the
            // approach — the direction its own last segment points — to the
            // first sample the drawn band covers.
            // Bare ground is only meaningful against the *approach*: if the
            // partner is itself carried this is a span meeting a span, and
            // there is no band that ought to be there.
            let bare = if paved.is_empty() || g.carried {
                None
            } else {
                let (ux, uy) = (g.heading.cos() / tile.scale.mx, g.heading.sin() / tile.scale.my);
                let mut found = None;
                let mut d = 0.0;
                while d <= BARE_REACH_M {
                    let (qx, qy) = (g.px + ux * d, g.py + uy * d);
                    // Surfaces are clipped to the tile *proper*, so a march that
                    // leaves it stops finding asphalt for a reason that has
                    // nothing to do with the abutment: the neighbour drew it.
                    // Abandon rather than report the tile border as a gap.
                    if !tile.owns(qx, qy) {
                        break;
                    }
                    if paved.iter().any(|m| m.height_at(qx, qy).is_some()) {
                        found = Some(d);
                        break;
                    }
                    d += BARE_STEP_M;
                }
                found
            };
            if let Some(bare) = bare {
                self.bare.push(bare);
                if bare > BREAK_M {
                    let (lon, lat) = tile.lonlat(g.px, g.py);
                    self.bare_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: bare,
                        note: format!(
                            "{bare:.2} m of bare ground between a {} abutment and the at-grade \
                             band that continues it",
                            s.class
                        ),
                    });
                }
            }
            let (lon, lat) = tile.lonlat(s.px, s.py);
            if plan > BREAK_M {
                self.plan_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: plan,
                    note: format!(
                        "a carried {} ends {plan:.2} m in plan from the at-grade road it \
                         continues into (and {step:.2} m in height)",
                        s.class
                    ),
                });
            }
            if step > BREAK_M {
                self.step_worst.offer(Offender {
                    lon,
                    lat,
                    zoom: tile.z,
                    value: step,
                    note: format!(
                        "a carried {} arrives {step:.2} m {} the at-grade road it continues \
                         into (and {plan:.2} m away in plan)",
                        s.class,
                        if s.h > g.h { "above" } else { "below" },
                    ),
                });
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let population = format!(
            "Every end of every road stroke inside the tile proper that a drawn deck or bore \
             carries — its height within {CARRIED_SLACK_M:.1} m of where the solid carries its \
             road: a deck's top, a bore's floor plus the deck thickness. Inside the solid's \
             range is not enough: a stroke at soffit level is passing underneath. Each is \
             paired with the *nearest* aligned stroke end of the same class within \
             {PAIR_MAX_M:.0} m: the two halves of one abutment or portal, which \
             `Corridor::pieces` cut from a single shared coordinate. {} paired, {} carried ends \
             left unpaired and not counted: a stroke the tile clipped (a clip cuts one piece \
             without producing its neighbour at the same place), or a span meeting another span \
             rather than the ground. {} more were the second half of a flush both-carried joint \
             (the deck's end cap covers the shared vertex, so both halves read as carried): the \
             pair is measured once from its lower index, and the leftover half contributes \
             nothing rather than pairing with the next end in reach — which used to score the \
             far end of the same short span, or the parallel track, as an 11 m break at a joint \
             that measures 0.00 m. Carried-ness is read from the drawn solid rather than from \
             the stroke's `level`, which a solved structure's paint does not carry. Only the tile \
             proper is measured, since strokes are clipped to the tile *plus buffer* and every \
             neighbour draws its own copy of a border abutment. A tile with no structure solid at \
             all contributes nothing.",
            self.paired, self.unpaired, self.second_half
        );
        vec![
            Metric {
                id: "seam.abutment_plan".into(),
                invariant: Invariant::I2,
                title: "Road centerline breaking in plan at a structure end".into(),
                population: population.clone(),
                detail: format!(
                    "Plan distance between the two ends. The correct value is zero — they are \
                     one coordinate in the model, and both strokes are snapped onto the same \
                     smoothed sweep line — so anything above the {BREAK_M:.2} m quantization \
                     floor is a generator moving one of them off that line: a sideways snap the \
                     `PAINT_SNAP_MAX_M` cap refused, or a piece with no profile to snap to."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BREAK_M,
                skipped: self.plan.is_empty().then(|| {
                    format!(
                        "no stroke meets a structure stroke of its own class at this zoom — from \
                         z{} the union paves carriageway and rail formation alike and both \
                         strokes are deleted; the surface handoff is seam.band_deck_*",
                        crate::priors::ROAD_SURFACE_MIN_ZOOM
                    )
                }),
                dist: self.plan,
                worst: self.plan_worst.into_vec(),
            },
            Metric {
                id: "seam.abutment_step".into(),
                invariant: Invariant::I2,
                title: "Road height stepping at a structure end".into(),
                population,
                detail: format!(
                    "Height difference between the two ends, which invariant 2 puts at zero: a \
                     deck ramp (`Profile::deck_m`) must arrive at the road it launches from \
                     (`road_m`). A step here is either the two fits disagreeing at the anchor or \
                     the sweep line having slid *along* the alignment, which makes the deck carry \
                     the height solved for a different station — the slide times the grade. Past \
                     {BREAK_M:.2} m it is neither quantization nor a fit residual."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BREAK_M,
                skipped: self.step.is_empty().then(|| {
                    format!(
                        "no stroke meets a structure stroke of its own class at this zoom — from \
                         z{} the union paves carriageway and rail formation alike and both \
                         strokes are deleted; the surface handoff is seam.band_deck_*",
                        crate::priors::ROAD_SURFACE_MIN_ZOOM
                    )
                }),
                dist: self.step,
                worst: self.step_worst.into_vec(),
            },
            Metric {
                id: "seam.abutment_bare".into(),
                invariant: Invariant::I2,
                title: "Bare ground between an abutment and its own carriageway".into(),
                population: format!(
                    "The approach half of every paired abutment above, on a tile that draws \
                     at-grade asphalt, marched up to {BARE_REACH_M:.0} m in {BARE_STEP_M:.2} m \
                     steps along its own last segment. Two marches yield no sample rather than a \
                     number, because neither measures this abutment: one that saturates (no band \
                     within {BARE_REACH_M:.0} m is a road that paves nothing there, not a gap of \
                     a known size), and one that leaves the tile proper, since surfaces are \
                     clipped to it and the asphalt the approach runs into is then the \
                     neighbour's to draw."
                ),
                detail: "How far the drawn carriageway stops short of the structure it \
                     continues into — the hole a bridge's approach leaves in the asphalt. This \
                     is a different quantity from the plan break above, and it has a different \
                     cause: the strokes are cut at the exact span arc, while the at-grade *band* \
                     is assembled from the corridor's own runs (`synth::carriageway::level_runs`). \
                     Assembled from whole mapped segments, the band ended at a vertex while the \
                     deck began at the boundary, and the difference — up to half a segment of a \
                     road digitised at whatever spacing a mapper chose — was drawn as ground. \
                     Zero is the only correct answer: the carriageway is one surface."
                    .to_string(),
                sense: Sense::HigherIsWorse,
                threshold: BREAK_M,
                skipped: self.bare.is_empty().then(|| {
                    "no paired abutment lies on a tile that draws at-grade asphalt".to_string()
                }),
                dist: self.bare,
                worst: self.bare_worst.into_vec(),
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::{Scale, SurfaceMesh};
    use crate::verify::scene::{RoadLine, RoadMesh, TileScene};

    fn tile(roads: Vec<RoadMesh>, lines: Vec<RoadLine>) -> TileScene {
        let bounds = Bounds::of_tile(16, 34028, 49670);
        TileScene {
            z: 16,
            x: 34028,
            y: 49670,
            scale: Scale::of(&bounds),
            bounds,
            terrain: None,
            roads,
            lines,
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    /// A flat deck solid from `x0` to `x1` across the middle of the tile.
    fn deck(x0: f64, x1: f64, h: f32) -> RoadMesh {
        let (y0, y1) = (0.49, 0.51);
        let mesh = SurfaceMesh::from_parts(
            vec![x0 as f32, x1 as f32, x1 as f32, x0 as f32],
            vec![y0, y0, y1, y1],
            vec![h; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .expect("a quad meshes");
        RoadMesh { class: "residential".into(), level: 1, band: String::new(), fades: false, sheet: None, mesh }
    }

    /// A west→east stroke from `x0` to `x1` at height `h`.
    fn stroke(x0: f64, x1: f64, h: f64) -> RoadLine {
        RoadLine {
            class: "residential".into(),
            level: 0,
            width_m: 0.0,
            parts: vec![vec![(x0, 0.5, h), (x1, 0.5, h)]],
        }
    }

    #[test]
    fn a_flush_joint_is_measured_once_and_as_zero() {
        // approach | 8 m deck | approach, every piece cut from a shared vertex
        // — so four carried ends stand in two co-located pairs. Each pair must
        // yield exactly one sample of ~0; the leftover halves must NOT pair
        // with the far end of the span 8 m away, which is what reported every
        // flush short-span joint as a metres-long break.
        let b = Bounds::of_tile(16, 34028, 49670);
        let mx = Scale::of(&b).mx;
        let (lo, hi) = (0.5, 0.5 + 8.0 / mx);
        // The solid overhangs the shared vertices by a hair, the way
        // quantization lands a flush end strictly inside the end cap.
        let eps = 0.05 / mx;
        let scene = tile(
            vec![deck(lo - eps, hi + eps, 400.0)],
            vec![stroke(0.44, lo, 400.0), stroke(lo, hi, 400.0), stroke(hi, 0.56, 400.0)],
        );
        let mut c = Box::new(Abutment::new(&Options::default()));
        c.visit(&scene, &Options::default());
        let plan = c.finish().swap_remove(0).dist;
        assert_eq!(plan.count(), 2, "one sample per joint, not per carried end");
        assert!(
            plan.max().expect("samples") <= BREAK_M,
            "a flush joint measured {:?} m of plan break",
            plan.max()
        );
    }
}
