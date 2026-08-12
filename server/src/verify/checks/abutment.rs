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
//! - [`seam.abutment_plan`] — the two pieces are on **different curves**. A
//!   structure is swept along the corridor's *smoothed* sweep line
//!   (`Profile::deck_nodes` → `smooth_point`) and its paint is carried onto it;
//!   the at-grade band is buffered around the **raw** `Corridor::nodes` and its
//!   paint stays there (`synth::road::bake`'s own note: "at an abutment the
//!   at-grade band and the deck are themselves that far apart in plan"). The
//!   break is the smoothing displacement, and it is both lateral (the deck sits
//!   beside the approach) and longitudinal (the deck starts short of or past
//!   the abutment, leaving bare ground or an overlap).
//! - [`seam.abutment_step`] — the deck ramp does not arrive at the road. The
//!   deck's height is `Profile::deck_m`, the approach's is `road_m`, and they
//!   are fitted separately; where the sweep line has also slid *along* the
//!   alignment the deck carries the height solved for a different station, so
//!   a plan break and a height step have a common cause and this separates
//!   them anyway, because either can occur alone.
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

/// How far a stroke's height may sit from a structure solid's own height range
/// at that point and still count as carried by it. A deck's top *is* the road
/// surface and a bore's floor is aligned to it, so the stroke lies inside the
/// range by construction; the slack covers the millimetre quantization of two
/// separately encoded features and the deck's own thickness at a sloping end
/// cap.
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
/// nothing to the union (`synth::junction::carriageway_sources` skips it), so
/// "the band that continues this abutment" does not exist for it and the march
/// finds whatever asphalt happens to be nearest — a footway ending near a road
/// bridge measured 19 m of "bare ground" that is simply a footway. It is also
/// junior geometry that no corridor partition cut, so the shared-coordinate
/// premise is not its premise either.
fn paves(class: &str) -> bool {
    use crate::priors::{Kind, Surface};
    Kind::parse(None, Some(class), None).prior().surface != Surface::None
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
        let solids: Vec<&crate::verify::mesh::SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|m| (m.is_deck() || m.is_bore()) && !m.is_fitted_deck())
            .map(|m| &m.mesh)
            .collect();
        if solids.is_empty() {
            return;
        }
        // The drawn at-grade asphalt: the interior band welded to its casing
        // rim, because the interior alone is an inset of the true silhouette
        // and marching to it would report the rim's width as bare ground.
        let paved: Vec<&crate::verify::mesh::SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|m| m.is_pavement() || m.is_casing())
            .map(|m| &m.mesh)
            .collect();
        let carried = |px: f64, py: f64, h: f64| {
            solids.iter().any(|m| {
                m.height_range_at(px, py).is_some_and(|(lo, hi)| {
                    h >= lo - CARRIED_SLACK_M && h <= hi + CARRIED_SLACK_M
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
            // The nearest at-grade end of the same class. Nearest rather than
            // any-within-radius: at a junction of two same-class roads several
            // ends sit close together, and the one that shares this abutment is
            // the one the cut produced, which is the nearest by construction.
            let mut best: Option<(f64, &End)> = None;
            for (j, g) in ends.iter().enumerate() {
                if i == j || g.carried || g.class != s.class || !aligned(s, g) {
                    continue;
                }
                let d = tile.scale.dist(s.px, s.py, g.px, g.py);
                if d <= PAIR_MAX_M && best.is_none_or(|(bd, _)| d < bd) {
                    best = Some((d, g));
                }
            }
            let Some((plan, g)) = best else {
                self.unpaired += 1;
                continue;
            };
            self.paired += 1;
            let step = (s.h - g.h).abs();
            self.plan.push(plan);
            self.step.push(step);
            // How much bare ground lies between the abutment and the asphalt
            // that should meet it: from the approach end, march *into* the
            // approach — the direction its own last segment points — to the
            // first sample the drawn band covers.
            let bare = if paved.is_empty() {
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
             carries — its height inside the solid's own range there, within {CARRIED_SLACK_M:.1} \
             m — and that has an *uncarried* stroke of the same class ending within \
             {PAIR_MAX_M:.0} m: the two halves of one abutment or portal, which \
             `Corridor::pieces` cut from a single shared coordinate. {} paired, {} carried ends \
             left unpaired and not counted: a stroke the tile clipped (a clip cuts one piece \
             without producing its neighbour at the same place), or a span meeting another span \
             rather than the ground. Carried-ness is read from the drawn solid rather than from \
             the stroke's `level`, which a solved structure's paint does not carry. Only the tile \
             proper is measured, since strokes are clipped to the tile *plus buffer* and every \
             neighbour draws its own copy of a border abutment. A tile with no structure solid at \
             all contributes nothing.",
            self.paired, self.unpaired
        );
        vec![
            Metric {
                id: "seam.abutment_plan".into(),
                invariant: Invariant::I2,
                title: "Road centerline breaking in plan at a structure end".into(),
                population: population.clone(),
                detail: format!(
                    "Plan distance between the two ends. The correct value is zero — they are \
                     one coordinate in the model — so anything above the {BREAK_M:.2} m \
                     quantization floor is a generator moving it. The known cause is that the \
                     two pieces ride different curves: a structure is swept along the smoothed \
                     sweep line and the at-grade band is buffered around the raw corridor nodes. \
                     Across the road it puts the deck beside its own approach; along it, the \
                     deck starts short of or past the abutment, which is the gap seen at a \
                     bridge's ends."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BREAK_M,
                skipped: self.plan.is_empty().then(|| {
                    "no structure stroke meets an at-grade stroke of its own class at this zoom"
                        .to_string()
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
                    "no structure stroke meets an at-grade stroke of its own class at this zoom"
                        .to_string()
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
                     is assembled from the corridor's own runs (`synth::junction::level_runs`). \
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
