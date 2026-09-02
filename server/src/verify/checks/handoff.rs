//! Does the drawn road *surface* survive the handoff at a bridge's end?
//!
//! [`super::abutment`] asks the same question of the **strokes**, and at the
//! zooms where the road surface actually exists it cannot answer it: from
//! [`crate::priors::ROAD_SURFACE_MIN_ZOOM`] the tiler deletes the stroke of
//! every class the union paves — carriageway and rail formation alike
//! (`pipeline::paves_via_union`) — so at z16 the stroke check has no
//! population at all and this file is the only abutment instrument.
//!
//! Neither modality hides the joint any more. A rail bridge used to draw its
//! track ribbon as one continuous object over both the ballast band and the
//! deck, so a break underneath was hidden by the very thing being measured;
//! now a ballast band must meet its deck as nakedly as asphalt does, and
//! every millimetre between the two meshes shows.
//!
//! ## What is paired, and against what
//!
//! The anchor is the **band's own silhouette** — a boundary edge of the drawn
//! at-grade surface ([`SurfaceMesh::boundary_edges`], the edges no second
//! triangle shares). Most of them are kerb, where the ground beside the road
//! is the correct neighbour. An **abutment edge** is one where a drawn
//! structure solid takes over instead: marching out along the edge's own
//! outward normal reaches a deck whose *top* is at the band's height there.
//! That last test is what separates a handoff from a viaduct passing
//! overhead — and from a band passing *under* a low deck: a deck's top *is*
//! the road surface, so an approach meeting it arrives at the top, while a
//! band at soffit level is in an underpass whose clearance is somebody
//! else's defect (`order.grade_stack`, `clearance.deck_over_ground`).
//!
//! Two quantities come out of that march, with different causes and different
//! fixes:
//!
//! - [`seam.band_deck_bare`] — drawn ground between where the at-grade surface
//!   stops and where the deck starts. The correct answer is zero: the band is
//!   cut at the exact span arc (`synth::carriageway::level_runs`) and the deck is
//!   swept from that same arc, so the two share a boundary rather than
//!   approach one.
//! - [`seam.band_deck_step`] — the height the surface jumps across that joint.
//!   The band's vertices are sampled from the height field over `road_at_arc`;
//!   the deck's come from `Profile::deck_at_arc`, a ramp fitted to the middle
//!   two thirds of the structure run — and pinned to the road at every
//!   anchored boundary (`solve::profile::deck_ramp`), which is what makes the
//!   two agree at the span arc. Invariant 2 says they must; what a step here
//!   reports now is a boundary the pin does not reach — a run at its
//!   corridor's own end, a handover fed by a different corridor at a junction
//!   — or the two sides disagreeing about the per-zoom datum shift.
//!
//! ## The band does not draw its own edge
//!
//! The measurement trap that decides whether this check means anything. The
//! `road_surface` mesh is an **inset** of the paved region: the outer
//! [`crate::priors::PAVE_RIM_M`] is a separate `road_rim` feature
//! (`synth::pave_mesh`). Marching to the interior alone reports the rim's width
//! as bare ground at every abutment in the extract — a floor of 0.35 m that no
//! change could ever remove. So the march *starts* at the interior's edge but
//! counts the rim as surface, and what it reports is the distance between
//! the last drawn surface and the first drawn deck.
//!
//! Bores are deliberately outside the population: a bore's drawn top is its
//! roof, [`crate::priors::TUNNEL_HEIGHT_M`] above the road, and the road surface inside it is
//! not drawn at all at these zooms. A portal is a real handoff and it needs its
//! own instrument, not this one with an assumed thickness subtracted.

use crate::priors::PAVE_RIM_M;
use crate::verify::dist::Dist;
use crate::verify::mesh::SurfaceMesh;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// How far past the band's edge a deck may start and still be read as the
/// structure that continues it, in metres.
///
/// A *pairing* radius, not a threshold: it decides which edges are abutments,
/// and the measurement is then whatever the distance turns out to be. It has to
/// exceed the break itself — a reach chosen from the answer would report every
/// abutment as flush and the broken ones as absent. Twelve metres is what
/// `abutment::PAIR_MAX_M` uses for the same reason, several times the deviation
/// the centerline smoother is allowed, and a long way past anything a joint
/// this check has found.
const REACH_M: f64 = 12.0;

/// March resolution, in metres. An order below the rim width the march has to
/// resolve through, and fine enough that a flush joint reports one step rather
/// than a fraction of a road width.
const STEP_M: f64 = 0.1;

/// What counts as bare, in metres. Two march steps: the first is the
/// instrument's own resolution and the second covers the tile lattice
/// (~1.9 cm at z16) under two independently encoded surfaces. Below anything
/// visible, which is the point — the contract says the band ends where the deck
/// begins, so the gate sits just above the noise rather than at a negotiated
/// tolerance.
const BARE_M: f64 = 0.15;

/// What counts as a step, in metres. Heights are stored in millimetres and the
/// two surfaces are sampled a march step apart in plan, which on the steepest
/// carriageway the priors allow is under a centimetre of legitimate difference.
const STEP_BREAK_M: f64 = 0.05;

/// How far the band's height may sit from the deck's *top* and still count as
/// handed over to it.
///
/// The joint's own geometry sets this: a deck's top *is* the road surface, so
/// an approach meeting it arrives at the top, and the slack only has to cover
/// quantization, the grade over the marched gap, and the deck's thickness at
/// a sloping end cap. Anchoring on the top rather than bracketing the solid's
/// whole vertical range is what separates a handoff from an *underpass*: a
/// band passing under a bridge whose clearance demand went unmet sits at
/// soffit level — inside the old bracket — and a whole cluster of such bands
/// beside the Villeneuve rail yard scored their bridge's unmet clearance as
/// 2.48 m handover steps (one deck thickness: the soffit resting on the road
/// below). That defect is real but it is `order.grade_stack` /
/// `clearance.deck_over_ground` material, not a seam. The slack is also this
/// check's coverage limit in the height dimension — a joint that steps by
/// more than it stops looking like a handoff at all and is not counted;
/// `order.deck_above_carriageway` owns that case.
const CARRIED_SLACK_M: f64 = 1.0;

/// How far back inside the band the overlap probe stands, in metres. Far
/// enough to be unambiguously *on* the band rather than on its boundary, and
/// short enough that the smallest overlap it lets through is smaller than the
/// gap the bare metric can resolve.
const OVERLAP_PROBE_M: f64 = 0.5;

/// Thickest vertical answer a structure may give at one plan point and still
/// have an unambiguous road surface, in metres.
///
/// A deck is a constant-section slab: top and soffit are a
/// [`crate::priors::DECK_THICKNESS_M`] apart wherever a point-in-triangle query
/// finds them, sloping or not, because the end caps are vertical and answer no
/// such query at all. A fatter answer means the feature's own mesh covers that
/// point *twice* — a ramp crossing over itself, two spans of one structure
/// stacked — and then "the top" is not the road surface, it is whichever of the
/// two decks is higher. Reported as a 13.06 m step at Montreux station before
/// this test existed, where the band it was compared against belonged to the
/// lower slab.
const SLAB_MAX_M: f64 = crate::priors::DECK_THICKNESS_M + CARRIED_SLACK_M;

/// Where in the rim strip the kerb-line probe stands, as a fraction of
/// [`PAVE_RIM_M`]. The strip runs from the interior mesh's inset edge out to the
/// true silhouette, so its middle is unambiguously inside it and clear of both
/// boundaries' quantization.
const KERB_PROBE_FRAC: f64 = 0.5;

/// One measured handoff.
struct Handoffs {
    bare: Dist,
    open: Dist,
    open_worst: Worst,
    bare_worst: Worst,
    step: Dist,
    step_worst: Worst,
    /// One per handoff: 1 where a rim is drawn across the joint, 0 where
    /// the surface runs through it.
    kerb: Dist,
    kerb_worst: Worst,
    /// Marched edges that found their deck, split by what the band is made of —
    /// the split that motivated the check, since the ballast half is also
    /// covered by `abutment` and the asphalt half by nothing else.
    asphalt: usize,
    ballast: usize,
    /// Band edges a deck already covers in plan: an overlap, not a handoff.
    overlapped: usize,
    /// Drawn decks, and how many of them name the band they continue
    /// (`band_class`). An archive tiled before that property states none, and
    /// the modality then falls back to re-deriving from the road class — so the
    /// ratio says which rule this run actually used.
    decks: usize,
    decks_named: usize,
}

pub struct Handoff(Handoffs);

impl Handoff {
    pub fn new(opt: &Options) -> Handoff {
        Handoff(Handoffs {
            bare: Dist::metres(),
            open: Dist::metres(),
            open_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            bare_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            step: Dist::metres(),
            step_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            kerb: Dist::new(0.0, 1.0),
            kerb_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            asphalt: 0,
            ballast: 0,
            overlapped: 0,
            decks: 0,
            decks_named: 0,
        })
    }
}

/// A mesh's plan extent, grown by `pad_m`, as `[west, south, east, north]` in
/// unit tile space. The cheap reject that keeps a kerb edge far from every
/// structure to one comparison instead of a 12 m march.
fn grown_box(m: &SurfaceMesh, tile: &TileScene, pad_m: f64) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for i in 0..m.vertex_count() {
        let (x, y, _) = m.vertex(i);
        b[0] = b[0].min(x);
        b[1] = b[1].min(y);
        b[2] = b[2].max(x);
        b[3] = b[3].max(y);
    }
    let (px, py) = (pad_m / tile.scale.mx, pad_m / tile.scale.my);
    [b[0] - px, b[1] - py, b[2] + px, b[3] + py]
}

fn in_box(b: &[f64; 4], x: f64, y: f64) -> bool {
    x >= b[0] && x <= b[2] && y >= b[1] && y <= b[3]
}

/// Whether a drawn structure's class names a railway.
///
/// The pairing rule, not a nicety. The union dissolves road identity — a
/// `road_surface` region is every carriageway that touched it — so the only
/// thing left to say whether a band and a deck are the same *feature* is what
/// they are made of. Without this test a street ending near a railway viaduct
/// pairs with it, and on the Montreux extract that was a third of everything
/// the metric called a gap: asphalt bands handing over to `narrow_gauge` and
/// `funicular` decks, which is not a handoff any road makes.
fn is_rail_class(class: &str) -> bool {
    crate::priors::class_is_rail(class)
}

impl Check for Handoff {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        // Solved bridge decks only. A *fitted* deck (`synth::draped`) is not
        // cut from the corridor partition, so its end is supposed to leave the
        // annotation's edge and `contact.deck_seat` measures it on its own
        // terms; a bore's drawn top is its roof, not a road surface.
        // `(class, is rail, mesh)`. The modality comes from the structure's own
        // `band_class` where it has one — the tiler stating which band this top
        // *is* — and from its road class otherwise, which is what an archive
        // tiled before that property carries.
        let decks: Vec<(&str, bool, &SurfaceMesh)> = tile
            .roads
            .iter()
            .filter(|m| m.is_deck() && !m.is_fitted_deck())
            .map(|m| {
                let rail = match m.band.as_str() {
                    "rail_surface" => true,
                    "road_surface" => false,
                    _ => is_rail_class(&m.class),
                };
                (m.class.as_str(), rail, &m.mesh)
            })
            .collect();
        if decks.is_empty() {
            return;
        }
        // The drawn at-grade surface: the interior band welded to its rim.
        // The interior alone is an inset of the true silhouette, and a
        // march that stopped at it would report `PAVE_RIM_M` of bare ground at
        // every abutment ever built. Split by modality, like the decks: the
        // surface that continues a ballast band is ballast, and only ballast.
        // The slab filter alone cannot say so — where a rail bridge crosses a
        // road, the road's asphalt approaches its underpass *through* the
        // band-height slab, the march walks `paved_h` down it, and the joint
        // is scored against the road below: a flush rack-line abutment at
        // 6.9474,46.4342 measured 2.76 m of step that way.
        let paved_of = |rail: bool| -> Vec<&SurfaceMesh> {
            tile.roads
                .iter()
                .filter(|m| m.is_pavement() || m.is_rim())
                .filter(|m| m.class.starts_with("rail") == rail)
                .map(|m| &m.mesh)
                .collect()
        };
        let paved_by: [Vec<&SurfaceMesh>; 2] = [paved_of(false), paved_of(true)];
        if paved_by.iter().all(|p| p.is_empty()) {
            return;
        }
        // **The rims that draw something of their own.** A rim takes its
        // surface's own colour (`pipeline::add_road_surface`), so the only
        // thing it can put across a joint is its fade — and it carries one only
        // where the ground under the asphalt is not cut away. A rim without a
        // fade *is* the surface there, and counting it would report a border
        // that nothing draws.
        let rims_of = |rail: bool| -> Vec<&SurfaceMesh> {
            tile.roads
                .iter()
                .filter(|m| m.is_rim() && m.fades && m.class.starts_with("rail") == rail)
                .map(|m| &m.mesh)
                .collect()
        };
        let rims_by: [Vec<&SurfaceMesh>; 2] = [rims_of(false), rims_of(true)];
        self.0.decks += decks.len();
        self.0.decks_named += tile
            .roads
            .iter()
            .filter(|m| m.is_deck() && !m.is_fitted_deck() && !m.band.is_empty())
            .count();
        let boxes: Vec<[f64; 4]> =
            decks.iter().map(|(_, _, m)| grown_box(m, tile, REACH_M)).collect();
        let (mx, my) = (tile.scale.mx, tile.scale.my);

        for band in tile.roads.iter().filter(|m| m.is_pavement()) {
            let ballast = band.class.starts_with("rail");
            let paved = &paved_by[ballast as usize];
            if paved.is_empty() {
                continue;
            }
            for (ia, ib, iopp) in band.mesh.boundary_edges() {
                let (ax, ay, az) = band.mesh.vertex(ia);
                let (bx, by, bz) = band.mesh.vertex(ib);
                let (ox, oy, _) = band.mesh.vertex(iopp);
                let (px, py) = (0.5 * (ax + bx), 0.5 * (ay + by));
                if !tile.owns(px, py) {
                    continue;
                }
                if !boxes.iter().any(|b| in_box(b, px, py)) {
                    continue; // no structure within reach: ordinary kerb
                }
                // Outward normal: perpendicular to the edge, pointing away from
                // the triangle's third corner — which is the only thing in a
                // silhouette edge that says which side the mesh is on.
                let (ex, ey) = ((bx - ax) * mx, (by - ay) * my);
                let len = (ex * ex + ey * ey).sqrt();
                if len < 1e-9 {
                    continue;
                }
                let (vx, vy) = ((px - ox) * mx, (py - oy) * my);
                let (mut nx, mut ny) = (ey / len, -ex / len);
                if nx * vx + ny * vy < 0.0 {
                    nx = -nx;
                    ny = -ny;
                }
                // Back to unit space, holding the metric step length.
                let (ux, uy) = (nx / mx, ny / my);
                let h = 0.5 * (az + bz);

                let carried = |q: (f64, f64)| -> Option<(f64, &str)> {
                    for ((class, rail, m), b) in decks.iter().zip(&boxes) {
                        if !in_box(b, q.0, q.1) || *rail != ballast {
                            continue; // a road does not hand over to a railway
                        }
                        if let Some((lo, hi)) = m.height_range_at(q.0, q.1) {
                            if hi - lo > SLAB_MAX_M {
                                continue; // stacked on itself: no single top
                            }
                            // At the *top*: a band at soffit level is passing
                            // underneath, not handing over.
                            if (h - hi).abs() <= CARRIED_SLACK_M {
                                return Some((hi, class));
                            }
                        }
                    }
                    None
                };

                // A deck standing over the band *itself* is an overlap, not a
                // handoff, and belongs to `order.deck_above_carriageway`. The
                // probe steps back inside the band to ask it: at a joint that
                // is exactly flush the deck's end cap sits on the band's edge
                // and answers a point-in-triangle query there, so testing the
                // edge itself would throw away the one case this check exists
                // to confirm.
                if carried((px - ux * OVERLAP_PROBE_M, py - uy * OVERLAP_PROBE_M)).is_some() {
                    self.0.overlapped += 1;
                    if std::env::var_os("ARPT_DEBUG_OVERLAP").is_some() {
                        let (lon, lat) = tile.lonlat(px, py);
                        eprintln!("[overlap] {lon:.6},{lat:.6}");
                    }
                    continue;
                }

                let (mut last_paved, mut paved_h) = (0.0f64, h);
                let mut found: Option<(f64, f64, &str)> = None;
                // Which march steps any drawn road surface covers in plan at
                // *any* height — the split between drawn ground the eye can
                // see the world through and a stretch a grade-separated road
                // closes from another rung (its own hole cut, its own aprons).
                let mut covered: Vec<bool> = Vec::new();
                let mut d = STEP_M;
                while d <= REACH_M {
                    let q = (px + ux * d, py + uy * d);
                    // Surfaces are clipped to the tile proper, so a march that
                    // leaves it stops finding asphalt for a reason that has
                    // nothing to do with the abutment: the neighbour drew it.
                    if !tile.owns(q.0, q.1) {
                        break;
                    }
                    if let Some((top, class)) = carried(q) {
                        found = Some((d, top, class));
                        break;
                    }
                    covered.push(
                        tile.roads.iter().any(|m| m.mesh.height_at(q.0, q.1).is_some()),
                    );
                    // Surface at this sample, but only surface that could be
                    // *this* band continuing. A drawn region metres below the
                    // edge is another road passing under the joint, and
                    // counting it would both shrink the gap it lies in and
                    // hand the step the wrong height to compare against: at
                    // Montreux station, where two at-grade bands stand 13 m
                    // apart (`order.grade_stack`), that reported the height
                    // between two unrelated roads as a 13.06 m step at a joint
                    // measured 0.10 m wide. What a real continuation can vary
                    // by over the reach is bounded by the grade — 12 m at the
                    // steepest a carriageway climbs is inside this — so the
                    // same slab bound serves.
                    if let Some(z) = paved
                        .iter()
                        .filter_map(|m| m.height_at(q.0, q.1))
                        .filter(|z| (z - h).abs() <= SLAB_MAX_M)
                        .fold(None::<f64>, |acc, z| Some(acc.map_or(z, |a: f64| a.max(z))))
                    {
                        last_paved = d;
                        paved_h = z;
                    }
                    d += STEP_M;
                }
                let Some((deck_d, top, class)) = found else { continue };
                if ballast {
                    self.0.ballast += 1;
                } else {
                    self.0.asphalt += 1;
                }

                let bare = (deck_d - last_paved).max(0.0);
                let step = (top - paved_h).abs();
                self.0.bare.push(bare);
                self.0.step.push(step);
                // The open metres of the bare stretch: steps between the last
                // near-height asphalt and the deck that no drawn road surface
                // covers at any height.
                let open = covered
                    .iter()
                    .enumerate()
                    .map(|(k, &c)| (STEP_M * (k + 1) as f64, c))
                    .filter(|&(sd, c)| sd > last_paved && sd < deck_d && !c)
                    .count() as f64
                    * STEP_M;
                let open = open.min(bare);
                self.0.open.push(open);
                if open > BARE_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    self.0.open_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: open,
                        note: format!(
                            "{open:.2} m of the {bare:.2} m between the {} band and the                              {class} deck is open ground no road covers at any height",
                            if ballast { "ballast" } else { "asphalt" }
                        ),
                    });
                }
                // Is a kerb line drawn across this joint? The probe stands in
                // the middle of the rim strip — between the interior mesh's
                // inset edge, which is where this march started, and the band's
                // true silhouette.
                let rim = PAVE_RIM_M * KERB_PROBE_FRAC;
                let kerb = rims_by[ballast as usize]
                    .iter()
                    .any(|m| m.height_at(px + ux * rim, py + uy * rim).is_some());
                self.0.kerb.push(if kerb { 1.0 } else { 0.0 });
                let (lon, lat) = tile.lonlat(px, py);
                if kerb {
                    self.0.kerb_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: 1.0,
                        note: format!(
                            "a kerb line is drawn across the {} band where it hands over to the \
                             {class} deck",
                            if ballast { "ballast" } else { "asphalt" }
                        ),
                    });
                }
                let surface = if ballast { "ballast" } else { "asphalt" };
                if bare > BARE_M {
                    self.0.bare_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: bare,
                        note: format!(
                            "{bare:.2} m of drawn ground between the {surface} band and the \
                             {class} deck that continues it"
                        ),
                    });
                }
                if step > STEP_BREAK_M {
                    self.0.step_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: step,
                        note: format!(
                            "the {class} deck starts {step:.2} m {} the {surface} band it takes \
                             over from (and {bare:.2} m past its edge)",
                            if top > paved_h { "above" } else { "below" },
                        ),
                    });
                }
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        let s = self.0;
        let population = format!(
            "Every boundary edge of a drawn at-grade surface band, inside the tile proper, that a \
             solved bridge deck continues: marching out along the edge's own outward normal \
             reaches a deck whose top is at the band's height there, within \
             {CARRIED_SLACK_M:.1} m — at the *top*, because a band at soffit level is passing \
             under a low bridge, not handing over to it. {} edges on asphalt and {} on ballast; \
             {} more were skipped \
             because a deck already covers the band's edge in plan, which is an overlap rather \
             than a handoff (`order.deck_above_carriageway`). One abutment contributes several \
             edges, so the population is weighted by how wide the joint is. {} of {} drawn decks \
             name the band they continue (`band_class`); the rest fall back to re-deriving the \
             modality from the road class, which is what an archive tiled before that property \
             gives. \
             \
             The band's *interior* mesh is an inset of the paved region and the outer \
             {PAVE_RIM_M:.2} m is the separate rim, so the march anchors on the interior \
             edge but counts interior and rim alike as surface: what is reported is the \
             distance between the last drawn surface and the first drawn deck. Only surface \
             within {SLAB_MAX_M:.1} m of the edge's own height, and of the band's own modality, \
             counts as that band continuing — a region metres below is a road passing under the \
             joint, and a road *approaching* its underpass passes through the height slab too, \
             so without the modality test the march walks the step's reference down the other \
             road's ramp. Counting either shrinks the gap it lies in and hands the step an \
             unrelated height to compare against. \
             \
             Coverage limits, all of them silent otherwise: bores are excluded, since a bore's \
             drawn top is its roof rather than a road surface — a portal needs its own \
             instrument; fitted decks are excluded, since their ends are deliberately walked to \
             seat them (`contact.deck_seat`); a joint that steps by more than a deck thickness \
             plus the slack no longer reads as a handoff and drops out of the population rather \
             than being counted as a huge one; and a deck whose approach band is missing \
             altogether has no edge to anchor on and contributes nothing.",
            s.asphalt, s.ballast, s.overlapped, s.decks_named, s.decks
        );
        let skipped = (s.asphalt + s.ballast == 0)
            .then(|| "no at-grade band meets a solved deck at this zoom".to_string());
        vec![
            Metric {
                id: "seam.band_deck_bare".into(),
                invariant: Invariant::I2,
                title: "Ground drawn between the road surface and the deck continuing it".into(),
                population: population.clone(),
                detail: format!(
                    "Plan distance from the last drawn at-grade surface to the first drawn deck, \
                     along the band edge's outward normal. Zero is the correct answer: the band \
                     is cut at the exact span arc (`synth::carriageway::level_runs`) and the deck is \
                     swept from that same arc, so the two share a boundary rather than approach \
                     one. On screen it is the defect this check was written for — the carriageway \
                     stopping short of its own bridge, with the hillside showing through the gap. \
                     Nothing hides it on either modality: the rail stroke is deleted with the \
                     carriageway's once the union paves the formation, so a ballast band meets \
                     its deck as nakedly as asphalt does ({BARE_M:.2} m gate)."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BARE_M,
                skipped: skipped.clone(),
                dist: s.bare,
                worst: s.bare_worst.into_vec(),
            },
            Metric {
                id: "seam.band_deck_open".into(),
                invariant: Invariant::I2,
                title: "The bare stretch no road covers at any height".into(),
                population: population.clone(),
                detail: format!(
                    "The same march as seam.band_deck_bare, keeping only the metres no drawn \
                     road surface covers in plan at *any* height. The bare metric's height \
                     window is deliberate — a region metres below the edge is another road \
                     passing under the joint — but at a grade-separated site that discipline \
                     charges the senior road's own footprint as bare ground, although the \
                     world there is closed by that road's surface, its hole and its aprons \
                     (I9). This is the open-ground half: the stretch the eye genuinely sees \
                     hillside through, and the population any mechanism should be judged on. \
                     Diagnosed 2026-09-02 after three refuted fixes — the junction weld (its \
                     sites are disjoint), the span partition (the union region covers the \
                     worst site end to end), and the one-mesh (ARPT_NO_ONE_MESH reads the \
                     same rate) — and the split is what says which family remains \
                     ({BARE_M:.2} m gate)."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BARE_M,
                skipped: skipped.clone(),
                dist: s.open,
                worst: s.open_worst.into_vec(),
            },
            Metric {
                id: "seam.band_deck_step".into(),
                invariant: Invariant::I2,
                title: "Road surface stepping where the band hands over to the deck".into(),
                population: population.clone(),
                detail: format!(
                    "Height difference across the same joint, which invariant 2 puts at zero. The \
                     band's vertices come from the height field over `Profile::road_at_arc`, the \
                     deck's from `Profile::deck_at_arc` — a ramp fitted to the middle two thirds \
                     of the structure run (`fit_ramp` trims a sixth at each end) whose ends are \
                     pinned back to the road at every anchored boundary \
                     (`solve::profile::deck_ramp`), so at the span arc the two are the same \
                     number by construction. What remains here is a boundary the pin does not \
                     reach: a structure run at its corridor's own end, a handover fed by a \
                     different corridor at a junction, or the per-zoom datum shift read \
                     differently by the two sides. Separated from the gap because either occurs \
                     alone, and because a deck that is short in plan carries the height solved \
                     for a different station and produces both ({STEP_BREAK_M:.2} m gate). The \
                     instrument's ceiling is about {:.1} m — a deck further than that from the \
                     band no longer reads as taking over from it, and surface that far from the \
                     edge no longer reads as the band — so a joint that has come apart \
                     completely leaves this population rather than topping it; \
                     `order.deck_above_carriageway` is where it lands.",
                    SLAB_MAX_M + CARRIED_SLACK_M
                ),
                sense: Sense::HigherIsWorse,
                threshold: STEP_BREAK_M,
                skipped: skipped.clone(),
                dist: s.step,
                worst: s.step_worst.into_vec(),
            },
            Metric {
                id: "seam.handover_kerb".into(),
                invariant: Invariant::I2,
                title: "Kerb line drawn across the carriageway at a bridge's end".into(),
                population,
                detail: format!(
                    "One per handoff: 1 where a *fading* rim covers the joint, 0 where the \
                     surface runs through it — so the `over` column is the share of bridge ends \
                     wearing a kerb line. The rim's job is to edge the paved surface against the \
                     ground it stops at, and at a handoff it stops at nothing: the road continues \
                     onto the deck, and a fading rim there draws a {PAVE_RIM_M:.2} m strip of \
                     half-alpha carriageway straight across the road a third of a metre before \
                     the bridge. The deck carries no matching rim, so the joint reads as a border \
                     rather than as a road. Only the fade counts: a rim takes its surface's own \
                     colour, so where the terrain hole makes the fade unnecessary the rim over a \
                     joint is simply the road. Probed at {KERB_PROBE_FRAC:.1} of the rim width \
                     outside the interior mesh's inset edge, which is where the strip is."
                ),
                sense: Sense::HigherIsWorse,
                threshold: 0.5,
                skipped,
                dist: s.kerb,
                worst: s.kerb_worst.into_vec(),
            },
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::mesh::Scale;
    use crate::verify::scene::RoadMesh;

    /// A tile 1000 m across, so unit space and metres convert by a round number.
    fn tile(roads: Vec<RoadMesh>) -> TileScene {
        let bounds = Bounds::of_tile(16, 34028, 49670);
        TileScene {
            z: 16,
            x: 34028,
            y: 49670,
            scale: Scale::of(&bounds),
            bounds,
            terrain: None,
            roads,
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    /// A flat quad from `x0` to `x1` across the middle of the tile, at height
    /// `h` — a stand-in for a stretch of carriageway running east.
    fn quad(class: &str, level: i64, x0: f64, x1: f64, h: f32) -> RoadMesh {
        let (y0, y1) = (0.49, 0.51);
        let mesh = SurfaceMesh::from_parts(
            vec![x0 as f32, x1 as f32, x1 as f32, x0 as f32],
            vec![y0, y0, y1, y1],
            vec![h; 4],
            vec![0, 1, 2, 0, 2, 3],
        )
        .expect("a quad meshes");
        RoadMesh { class: class.to_string(), level, sheet: None, band: String::new(), fades: false, mesh }
    }

    fn measure(roads: Vec<RoadMesh>) -> (Option<f64>, Option<f64>, u64) {
        let mut c = Box::new(Handoff::new(&Options::default()));
        c.visit(&tile(roads), &Options::default());
        let m = c.finish();
        assert_eq!(m[1].id, "seam.band_deck_open");
        assert_eq!(m[2].id, "seam.band_deck_step");
        (m[0].dist.max(), m[2].dist.max(), m[0].dist.count())
    }

    /// Metres per unit of longitude on the test tile.
    fn mx() -> f64 {
        Scale::of(&Bounds::of_tile(16, 34028, 49670)).mx
    }

    #[test]
    fn a_flush_joint_measures_nothing() {
        // Band to the west, deck taking over at exactly its edge.
        let edge = 0.5;
        let (bare, step, n) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            quad("residential", 1, edge, 0.6, 400.0),
        ]);
        assert!(n > 0, "the abutment edge was not paired at all");
        assert!(bare.expect("bare") <= BARE_M, "flush joint read {bare:?} of bare ground");
        assert!(step.expect("step") <= STEP_BREAK_M, "flush joint read {step:?} of step");
    }

    #[test]
    fn a_deck_starting_short_reports_the_gap() {
        // Two metres of nothing between the band's edge and the deck.
        let gap_m = 2.0;
        let edge = 0.5;
        let (bare, _, n) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            quad("residential", 1, edge + gap_m / mx(), 0.6, 400.0),
        ]);
        assert!(n > 0, "the abutment was not paired");
        let bare = bare.expect("bare");
        assert!((bare - gap_m).abs() < 0.25, "gap of {gap_m} m measured as {bare}");
    }

    #[test]
    fn a_deck_arriving_high_reports_the_step() {
        let edge = 0.5;
        let (_, step, _) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            quad("residential", 1, edge, 0.6, 400.6),
        ]);
        let step = step.expect("step");
        assert!((step - 0.6).abs() < 0.05, "0.60 m step measured as {step}");
    }

    /// A quad with distinct top and soffit sheets, so `height_range_at`
    /// answers the solid's real vertical range the way a swept deck's does.
    fn slab(class: &str, level: i64, x0: f64, x1: f64, soffit: f32, top: f32) -> RoadMesh {
        let (y0, y1) = (0.49, 0.51);
        let (x0, x1) = (x0 as f32, x1 as f32);
        let mesh = SurfaceMesh::from_parts(
            vec![x0, x1, x1, x0, x0, x1, x1, x0],
            vec![y0, y0, y1, y1, y0, y0, y1, y1],
            vec![top, top, top, top, soffit, soffit, soffit, soffit],
            vec![0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        )
        .expect("a slab meshes");
        RoadMesh { class: class.to_string(), level, sheet: None, band: String::new(), fades: false, mesh }
    }

    #[test]
    fn a_band_under_a_low_deck_is_not_a_handoff() {
        // A bridge whose clearance demand went unmet: the soffit sits on the
        // band passing beneath. The band's height is inside the solid's
        // vertical range — the old bracket paired it and scored the missing
        // clearance as a deck-thickness step — but it is nowhere near the
        // top, and a handoff hands the surface to the top.
        let edge = 0.5;
        let (_, _, n) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            slab("residential", 1, edge, 0.6, 400.0, 402.5),
        ]);
        assert_eq!(n, 0, "an underpass at soffit level was paired as an abutment");
    }

    #[test]
    fn a_viaduct_overhead_is_not_a_handoff() {
        // The deck is where an abutment would be, but eight metres up: the
        // road passes under it and there is no joint to measure. Without the
        // height gate this is the false positive that would swamp the metric.
        let edge = 0.5;
        let (_, _, n) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            quad("residential", 1, edge, 0.6, 408.0),
        ]);
        assert_eq!(n, 0, "a deck 8 m overhead was paired as an abutment");
    }

    #[test]
    fn a_footbridge_is_left_to_its_own_check() {
        let edge = 0.5;
        let (_, _, n) = measure(vec![
            quad("road_surface", 0, 0.4, edge, 400.0),
            quad("footway", 1, edge, 0.6, 400.0),
        ]);
        assert_eq!(n, 0, "a fitted deck belongs to contact.deck_seat");
    }

    #[test]
    fn the_rim_is_surface_and_not_a_gap() {
        // The interior band stops a rim short of the true edge, exactly as
        // `synth::pave_mesh` emits it. Counting the rim as bare ground would
        // put a floor of PAVE_RIM_M under every abutment in the archive.
        let edge = 0.5;
        let inset = edge - PAVE_RIM_M / mx();
        let (bare, _, n) = measure(vec![
            quad("road_surface", 0, 0.4, inset, 400.0),
            quad("road_rim", 0, inset, edge, 400.0),
            quad("residential", 1, edge, 0.6, 400.0),
        ]);
        assert!(n > 0, "the abutment was not paired");
        assert!(
            bare.expect("bare") <= BARE_M,
            "the rim was counted as bare ground: {bare:?}"
        );
    }
}
