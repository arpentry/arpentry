//! Is the paint on the asphalt, at the offset the cross-section put it at —
//! and above the ground a viewer can see it from?
//!
//! `docs/ROADS.md` invariant 3: "Markings are functions of the cross-section.
//! Every painted element is placed by the lane model… No marking is placed by
//! eye." Nothing measured that. Every other check reads a *height* — the paint
//! is the one thing in the model whose whole correctness is a **plan** offset,
//! signed across the road, and a marking that has lost it is still perfectly
//! draped on the surface it is no longer registered to.
//!
//! It is measured here, and not in `contact`, because the contact family is
//! anchored on the kerb and asks vertical questions of it. The first two ask
//! where a line lies *across* the carriageway:
//!
//! - [`paint.marking_offside`] — how far a painted line lies outside the drawn
//!   asphalt. The universal one: whatever a marking is, it is on the road.
//! - [`paint.edge_line_inset`] — how far an *edge* line stands from its own
//!   kerb. Edge lines are the one painted line the archive can name (they are
//!   the only 0.15 m stroke), and the cross-section fixes their offset exactly,
//!   so this is the one place a lost lateral offset is visible as a number
//!   rather than as a shape.
//!
//! The defect that motivated them: the emit stage snapped every road vertex
//! onto the corridor's smoothed sweep line to keep paint from tracing digitising
//! wiggle beside its own smooth-swept bridges — by *projecting* it, which throws
//! away exactly the signed offset that makes a marking a marking. Both edge
//! lines and every lane divider collapsed onto the axis: a motorway drew three
//! coincident lines down its middle and none at its edges.
//!
//! The third asks the vertical question no other check owns:
//!
//! - [`paint.buried`] — how far under the drawn terrain a stroke vertex sits.
//!   The client strokes lines as decals depth-tested against the ground, so a
//!   buried vertex is paint nobody can see — and at the coarse rungs, paint
//!   that surfaces wherever the buried run and the lattice's chords disagree.
//!   The population it exists to keep dead: a tunnel span's paint. The stroke,
//!   the markings and the rail heads all stop at the portal now
//!   (`pipeline::process_feature`), so nothing legitimate is left below the
//!   ground it is drawn under.

use crate::verify::dist::Dist;
use crate::verify::mesh::{Scale, SurfaceMesh};
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// The painted width, in metres, that names a stroke an edge line. The archive
/// carries a marking's painted width and nothing else about it, and the two
/// widths the baker emits are distinct: 0.12 m for a centre line or a lane
/// divider, 0.15 m for an edge line (`priors::EDGE_LINE_WIDTH_M`). Restated
/// here rather than imported, so a change to the palette cannot silently move
/// the measurement with it.
const EDGE_LINE_WIDTH_M: f64 = 0.15;
const MARKING_WIDTH_EPS: f64 = 0.01;

/// How far outside the drawn asphalt a marking vertex may lie and still count
/// as on it. The carriageway silhouette is quantized to the tile lattice (about
/// 1.3 cm at z16) and the paint is quantized separately, so a line painted
/// exactly on the kerb lands either side of it; a quarter-metre is past what
/// that explains and far short of the lane widths a lost offset costs.
const OFFSIDE_M: f64 = 0.25;

/// How far inside the tile proper a marking vertex must lie to be measured.
///
/// **The population trap this closes.** Surfaces are clipped to the tile
/// *proper* — a deck box by `synth::structure::proper_pieces` ("an opaque solid
/// must not extend into the format's buffer or neighbouring tiles would each
/// rebuild and overlap it"), the paved region by the same rule on its lattice
/// samples. Markings are ordinary line features and are clipped to the tile
/// *plus its buffer*. So a road running just outside a border paints a line
/// that lands inside the neighbouring tile while every surface it belongs to
/// stays out — and a per-tile check reads that as paint hundreds of metres from
/// any asphalt, when on screen the neighbour's own deck is drawn under it and
/// nothing is wrong at all.
///
/// A vertex this far inside has its centerline inside the tile too, since no
/// marking is offset further than half a wide carriageway, so the surface that
/// answers for it is one this tile drew. At z16 it costs about 7 % of the tile
/// area, and the sample count says so.
const BORDER_MARGIN_M: f64 = 8.0;

/// Where the cross-section puts an edge line's near kerb: the edge inset
/// (`priors::EDGE_LINE_INSET_M`, 0.30 m) plus the shoulder the surface is
/// buffered by beyond the carriageway (`priors::STRUCTURE_SHOULDER_M`, 1.0 m).
/// Restated for the same reason as the widths above.
const EDGE_INSET_EXPECTED_M: f64 = 1.30;

/// How far an edge line's near kerb may sit from [`EDGE_INSET_EXPECTED_M`]
/// before the line is not at the edge of anything.
///
/// Slack, deliberately. A real carriageway's asphalt widens where the union
/// takes in a ramp, a bus bay or a junction mouth, and the near kerb goes with
/// it — legitimately, and by metres. What this has to catch is the collapse,
/// which puts the near kerb at *half a carriageway* — 4.5 m on a 9 m motorway,
/// 5.5 m with the shoulder — so a threshold anywhere between the two separates
/// them. Cut at 2 m: past that the line is not doing an edge line's job.
const EDGE_INSET_SLOP_M: f64 = 2.0;

/// How far across the road the asphalt is followed when looking for a line's
/// near kerb, and in what steps. Wide enough to clear half of a fused dual
/// carriageway, so a line stranded in the middle of one is still measured
/// rather than dropped; a vertex with no kerb inside it on either side is not
/// beside a carriageway at all and yields no sample.
const REACH_CAP_M: f64 = 12.0;
const REACH_COARSE_M: f64 = 0.25;
const REACH_FINE_M: f64 = 0.05;

/// How far under the drawn terrain a stroke vertex may sit and still count as
/// on the ground. An at-grade stroke is draped chord-exactly on the rendered
/// surface of its own zoom (`synth::road::densify_road_line` inserts a vertex
/// at every lattice crossing), so the bulk of the population reads millimetres
/// (measured median 7 mm at z16) and the metre clears its quantization noise
/// outright. What stays above the gate is the portal-mouth approach: the last
/// metres of an at-grade piece pass under the cut face climbing over the bore,
/// which on a cliff portal stands up to ~9 m overhead within the approach's
/// final vertices — about 0.03 % of the extract's samples, recorded in the
/// baseline. The mode this metric exists to keep dead read 4.2 % of vertices
/// and 592 m under: a tunnel span's own paint riding the bore.
const BURIED_M: f64 = 1.0;

/// The classes drawn as a pedestrian band from [`WALK_SURFACE_MIN_ZOOM`]
/// ([`crate::priors::earns_walk_band`], read back from the archive's `class`
/// alone). Restated here rather than derived, so a change to what earns a band
/// cannot silently move the population this metric watches.
const BANDED_CLASSES: [&str; 6] =
    ["footway", "path", "steps", "cycleway", "pedestrian", "track"];

pub struct Paint {
    offside: Dist,
    offside_worst: Worst,
    inset: Dist,
    inset_worst: Worst,
    buried: Dist,
    buried_worst: Worst,
    doubled: Dist,
    doubled_worst: Worst,
    doubled_seen: bool,
}

impl Paint {
    pub fn new(opt: &Options) -> Paint {
        Paint {
            offside: Dist::metres(),
            offside_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            inset: Dist::metres(),
            inset_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            buried: Dist::metres(),
            buried_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            doubled: Dist::new(0.0, 1.0),
            doubled_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
            doubled_seen: false,
        }
    }

    /// Every owned vertex of every at-grade pedestrian stroke, scored 1 where a
    /// drawn pedestrian band already covers it.
    ///
    /// A way the walkway model banded has its cartographic stroke deleted at
    /// these zooms, because the band *is* the way there
    /// (`pipeline::paves_via_walkway`). A stroke standing on a band is that
    /// deletion having failed, and the failure is silent: the line renders in
    /// its class colour on top of its own surface, which looks like a way that
    /// is merely styled oddly rather than like a bug.
    ///
    /// It was silent for exactly that reason. The deletion test used to read
    /// two properties phase 1 stamped on the feature, and `profile::profile`
    /// builds a tile's properties from a fixed whitelist of *source*
    /// attributes — so the stamps never arrived and the test could never fire.
    /// Every banded way in the archive drew twice: 351 km of stroke at z16 on
    /// the Montreux extract, 82 % of it directly over its own band.
    fn visit_doubled(&mut self, tile: &TileScene) {
        let bands: Vec<&SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|r| r.level == 0 && is_walk_material(&r.class))
            .map(|r| &r.mesh)
            .collect();
        if bands.is_empty() {
            return;
        }
        self.doubled_seen = true;
        for line in tile
            .lines
            .iter()
            .filter(|l| l.level == 0 && BANDED_CLASSES.contains(&l.class.as_str()))
        {
            for part in &line.parts {
                for &(px, py, _) in part {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    let on = bands.iter().any(|m| m.height_at(px, py).is_some());
                    self.doubled.push(if on { 1.0 } else { 0.0 });
                    if on {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.doubled_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: 1.0,
                            note: format!(
                                "a {} stroke is drawn over the pedestrian band that replaced it",
                                line.class
                            ),
                        });
                    }
                }
            }
        }
    }

    /// Every owned vertex of every stroke at level ≤ 0, against the drawn
    /// terrain over it. Positive levels are excluded — a bridge stroke rides
    /// its deck above the ground by design, and its solid answers to the
    /// clearance checks. A vertex with no terrain over it (the pavement hole,
    /// a portal cut, a zoom with no terrain mesh) contributes nothing: there
    /// is no drawn ground there to be buried under.
    fn visit_buried(&mut self, tile: &TileScene) {
        let Some(terrain) = &tile.terrain else { return };
        for line in tile.lines.iter().filter(|l| l.level <= 0) {
            for part in &line.parts {
                for &(px, py, h) in part {
                    if !tile.owns(px, py) {
                        continue;
                    }
                    let Some(gz) = terrain.height_at(px, py) else { continue };
                    let under = (gz - h).max(0.0);
                    self.buried.push(under);
                    if under > BURIED_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.buried_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: under,
                            note: format!(
                                "a {} stroke (level {}) runs {under:.2} m under the drawn ground",
                                line.class, line.level
                            ),
                        });
                    }
                }
            }
        }
    }
}

impl Check for Paint {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        self.visit_buried(tile);
        self.visit_doubled(tile);
        let marks: Vec<_> = tile.lines.iter().filter(|l| l.class == "marking").collect();
        if marks.is_empty() {
            return;
        }
        // The at-grade carriageway is two features — an interior triangulated to
        // an inset of the silhouette, and the rim that covers the strip
        // out to it (`synth::pave_mesh`) — so both are needed before anything is
        // asked whether it is "on the asphalt", and the silhouette has to be
        // taken across the welded pair rather than from either alone.
        let paved: Vec<&SurfaceMesh> = tile
            .roads
            .iter()
            .filter(|m| m.is_pavement() || m.is_rim())
            .map(|m| &m.mesh)
            .collect();
        if paved.is_empty() {
            return;
        }
        // Paint riding a structure answers to the deck or the bore it is on, not
        // to the union, and those solids report their *roof* — a parapet, a
        // tunnel crown — so a cross-section measured against them would measure
        // the solid. Left to the structure checks.
        let structures: Vec<&SurfaceMesh> =
            tile.roads.iter().filter(|m| m.is_deck() || m.is_bore()).map(|m| &m.mesh).collect();
        let rim = silhouette(&paved);

        let (inx, iny) = (BORDER_MARGIN_M / tile.scale.mx, BORDER_MARGIN_M / tile.scale.my);
        for line in marks {
            let is_edge = (line.width_m - EDGE_LINE_WIDTH_M).abs() < MARKING_WIDTH_EPS;
            for part in &line.parts {
                for (i, &(px, py, h)) in part.iter().enumerate() {
                    if !tile.owns(px, py)
                        || px < inx
                        || px > 1.0 - inx
                        || py < iny
                        || py > 1.0 - iny
                    {
                        continue;
                    }
                    if structures.iter().any(|m| {
                        m.height_range_at(px, py)
                            .is_some_and(|(lo, hi)| h >= lo - 1.0 && h <= hi + 1.0)
                    }) {
                        continue;
                    }
                    let on = paved.iter().any(|m| m.height_at(px, py).is_some());
                    let out = if on { 0.0 } else { nearest(&rim, px, py, &tile.scale) };
                    self.offside.push(out);
                    if out > OFFSIDE_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.offside_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: out,
                            note: format!(
                                "a {:.2} m painted line runs {out:.2} m clear of any drawn \
                                 carriageway",
                                line.width_m
                            ),
                        });
                    }
                    if !on || !is_edge || part.len() < 2 {
                        continue;
                    }
                    // The line's own direction, from the segment it lies on: the
                    // cross-section is taken across *it*, not across whichever
                    // road happens to be nearest.
                    let (a, b) = if i + 1 < part.len() {
                        (part[i], part[i + 1])
                    } else {
                        (part[i - 1], part[i])
                    };
                    let Some(near) = near_kerb(&paved, (px, py), (a, b), &tile.scale) else {
                        continue;
                    };
                    let err = (near - EDGE_INSET_EXPECTED_M).abs();
                    self.inset.push(err);
                    if err > EDGE_INSET_SLOP_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.inset_worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: err,
                            note: format!(
                                "edge line stands {near:.2} m from its nearest kerb, not \
                                 {EDGE_INSET_EXPECTED_M:.2} m — it is not at an edge"
                            ),
                        });
                    }
                }
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![
            Metric {
                id: "paint.marking_offside".into(),
                invariant: Invariant::I4,
                title: "Painted marking lying off the drawn carriageway".into(),
                population: format!(
                    "Every vertex of every `marking` stroke at least {BORDER_MARGIN_M:.0} m \
                     inside the tile proper, at a zoom that draws the road surface. The drawn \
                     carriageway is the `road_surface` interior welded to its `road_rim` rim \
                     — the interior alone is an inset of the true silhouette, and measuring \
                     against it would report the rim's width as a defect. Vertices riding a deck \
                     or a bore are excluded: their cross-section is the structure's, and a swept \
                     solid answers with its roof. The border margin is not decoration: surfaces \
                     are clipped to the tile proper and markings to the tile *plus buffer*, so \
                     without it a road running along a border paints into its neighbour and \
                     reads as hundreds of metres from any asphalt while rendering perfectly. \
                     Coverage limits: that margin drops about 7 % of a z16 tile's area, and a \
                     tile carrying markings but no paved surface at all contributes nothing, so \
                     a road that paved *nothing* is invisible here."
                ),
                detail: format!(
                    "Plan distance from a painted line to the asphalt it is painted on, zero \
                     when it is on it. Contact in plan rather than in height (I4): a marking \
                     that has lost its lateral registration is still perfectly draped, and \
                     every height check reports it clean. Past {OFFSIDE_M:.2} m it is wider \
                     than the two quantizations can explain and the paint is on the verge."
                ),
                sense: Sense::HigherIsWorse,
                threshold: OFFSIDE_M,
                skipped: self
                    .offside
                    .is_empty()
                    .then(|| "no marking stroke lies on a tile that draws asphalt".to_string()),
                dist: self.offside,
                worst: self.offside_worst.into_vec(),
            },
            Metric {
                id: "paint.edge_line_inset".into(),
                invariant: Invariant::I4,
                title: "Edge line standing off its own kerb".into(),
                population: format!(
                    "Every on-asphalt vertex of every {EDGE_LINE_WIDTH_M:.2} m stroke — the \
                     painted width the baker gives edge lines and nothing else, and the only \
                     thing the archive carries that names which line a marking is. Measured on \
                     the near side only, marching across the line's own direction to the first \
                     gap in the asphalt, capped at {REACH_CAP_M:.0} m — a vertex with no kerb \
                     inside that on either side yields no sample, because a saturated march is \
                     not a measurement. Centre lines and lane \
                     dividers are *not* here: both are painted {:.2} m wide, so the archive \
                     cannot tell a divider that belongs a third of the way across from a centre \
                     line that belongs in the middle, and a metric that guessed would report \
                     correct paint as broken.",
                    0.12
                ),
                detail: format!(
                    "How far an edge line's near kerb sits from the {EDGE_INSET_EXPECTED_M:.2} m \
                     the cross-section puts it at (a {:.2} m inset inside a carriageway the \
                     surface buffers by a {:.1} m shoulder). This is what a lost lateral offset \
                     looks like as a number: paint projected onto the road's axis reads half a \
                     carriageway here — over 5 m on a motorway — instead of a metre and a bit. \
                     The {EDGE_INSET_SLOP_M:.1} m gate is loose on purpose, since real asphalt \
                     widens at ramps and junction mouths and takes the near kerb with it.",
                    0.30, 1.0
                ),
                sense: Sense::HigherIsWorse,
                threshold: EDGE_INSET_SLOP_M,
                skipped: self.inset.is_empty().then(|| {
                    "no edge line lies on drawn asphalt at this zoom — the extract has no \
                     motorway or trunk carriageway"
                        .to_string()
                }),
                dist: self.inset,
                worst: self.inset_worst.into_vec(),
            },
            Metric {
                id: "paint.buried".into(),
                invariant: Invariant::I4,
                title: "Painted stroke running under the drawn ground".into(),
                population: "Every vertex the tile owns of every transportation stroke at \
                     level ≤ 0 — the road and rail fill strokes of the pre-surface rungs, \
                     markings, rail heads, and whatever draped ways still stroke — against \
                     the drawn terrain over it. Positive levels are excluded: a bridge \
                     stroke rides its deck above the ground by design, and the deck itself \
                     answers to the clearance checks. A vertex with no terrain over it \
                     contributes nothing — the pavement hole and the portal cuts are exactly \
                     where a stroke is supposed to have no drawn ground overhead, and a zoom \
                     with no terrain mesh has nothing to measure against.\n\n\
                     **The population thins as the surface model grows, and the rate is not \
                     comparable across that.** Each class the drawing promotes from a stroke \
                     to a band leaves here: the carriageways at \
                     `ROAD_SURFACE_MIN_ZOOM`, then the pedestrian ways at \
                     `WALK_SURFACE_MIN_ZOOM` — which alone took 68 km of footway, path, track \
                     and steps off the Montreux zone's z16 tally, 39 % of the samples, \
                     without moving the worst by a millimetre. What is left is denser in \
                     genuine paint, so the rate rises on an unchanged defect. Those ways are \
                     not unmeasured: a band's contact with the ground is \
                     `contact.walk_rim` and `contact.sidewalk_grade`, which is where the \
                     samples went."
                    .into(),
                detail: format!(
                    "How far under the drawn terrain the stroke sits. The client strokes \
                     lines as decals depth-tested against the ground, so a buried vertex is \
                     paint nobody can see — and at the coarse rungs, paint that surfaces \
                     wherever the buried run and the lattice's chords disagree. The \
                     population this keeps dead: a tunnel span's paint. The stroke, its \
                     markings and its rail heads all stop at the portal \
                     (`pipeline::process_feature`), so nothing is emitted riding a bore's \
                     road surface under the hill any more. An at-grade stroke is draped \
                     chord-exactly on its own zoom's rendered surface, so the bulk reads \
                     millimetres and the {BURIED_M:.1} m gate clears its quantization noise \
                     outright. The residue above the gate is the portal-mouth approach — \
                     the last metres of an at-grade piece passing under the cut face that \
                     climbs over the bore mouth, up to ~9 m overhead on a cliff portal — \
                     which is the approach running where it should, under ground that is \
                     really there. It is a fraction of a percent and the baseline records \
                     it; the mode to watch for is the rate jumping back toward the 4 % the \
                     tunnel paint read."
                ),
                sense: Sense::HigherIsWorse,
                threshold: BURIED_M,
                skipped: self
                    .buried
                    .is_empty()
                    .then(|| "no stroke lies over drawn terrain at this zoom".to_string()),
                dist: self.buried,
                worst: self.buried_worst.into_vec(),
            },
            Metric {
                id: "paint.stroke_over_band".into(),
                invariant: Invariant::I1,
                title: "Cartographic stroke drawn over the band that replaced it".into(),
                population: format!(
                    "Every owned vertex of every at-grade stroke whose class is drawn as a \
                     pedestrian band ({}), on a tile that draws at least one such band. The \
                     sample is 1 where a band covers the vertex and 0 where it does not, so \
                     the `over` column is the share of pedestrian stroke standing on its own \
                     surface. Tiles with no band contribute nothing: below \
                     `WALK_SURFACE_MIN_ZOOM` the stroke is the only thing a pedestrian way \
                     has and drawing it is correct.",
                    BANDED_CLASSES.join(", ")
                ),
                detail: "A way the walkway model banded has its stroke deleted at the walk \
                     zooms, because the band is the way there — so a stroke over a band is a \
                     line painted on its own surface, in the class colour, at a constant \
                     screen width that ignores the surface underneath. It fails silently: two \
                     coats of the same object read as odd styling rather than as a bug, which \
                     is how the deletion stayed broken.\n\n\
                     **The floor is not zero, and what sets it is the join.** A way that \
                     survives here is one the model did *not* band — a way the seat had no \
                     room for, or the ground fit declined — and `banded_walks` is per source, \
                     so such a way has no band anywhere along it. Its own vertices therefore \
                     score 0 except where it *meets* a way that was banded: the last vertex or \
                     two of an unbanded path standing on the pavement it runs into. Measured \
                     on the Montreux zone: 46 of 494 vertices, over 26 of the 178 still-stroked \
                     features, **37 of the 46 within two vertices of a part end** and the hit \
                     runs a median of 1 vertex long (the longest, 8). That residue scales with \
                     the number of joins, not with length, so read the count rather than the \
                     rate — the population shrinks as the model bands more, which inflates the \
                     share a fixed residue represents. A reading that grows in the *middle* of \
                     features is the real defect coming back."
                    .into(),
                sense: Sense::HigherIsWorse,
                threshold: 0.5,
                skipped: (!self.doubled_seen).then(|| {
                    "no tile at this zoom draws a pedestrian band — below \
                     WALK_SURFACE_MIN_ZOOM the stroke is the pedestrian way"
                        .to_string()
                }),
                dist: self.doubled,
                worst: self.doubled_worst.into_vec(),
            },
        ]
    }
}

/// Whether a class names drawn pedestrian pavement — a band, or its rim.
fn is_walk_material(class: &str) -> bool {
    matches!(class, "walk_surface" | "walk_rim" | "path_surface" | "path_rim")
}

/// The silhouette of a set of meshes: every edge no other triangle in the set
/// shares, keyed on the integer plan lattice so two meshes meeting along a
/// shared boundary — a carriageway interior and its rim — weld instead of each
/// reporting the join as an outer edge.
fn silhouette(meshes: &[&SurfaceMesh]) -> Vec<((f64, f64), (f64, f64))> {
    use std::collections::HashMap;
    type Key = ((i64, i64), (i64, i64));
    let key = |v: (f64, f64, f64)| ((v.0 * 32768.0).round() as i64, (v.1 * 32768.0).round() as i64);
    let mut count: HashMap<Key, u32> = HashMap::new();
    let mut tris: Vec<[(f64, f64, f64); 3]> = Vec::new();
    for m in meshes {
        for t in 0..m.triangle_count() {
            let tri = m.triangle(t);
            for i in 0..3 {
                let (a, b) = (key(tri[i]), key(tri[(i + 1) % 3]));
                *count.entry(if a <= b { (a, b) } else { (b, a) }).or_insert(0) += 1;
            }
            tris.push(tri);
        }
    }
    let mut out = Vec::new();
    for tri in &tris {
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            let (ka, kb) = (key(a), key(b));
            if count[&if ka <= kb { (ka, kb) } else { (kb, ka) }] == 1 {
                out.push(((a.0, a.1), (b.0, b.1)));
            }
        }
    }
    out
}

/// Plan distance in metres to the nearest silhouette segment.
fn nearest(segs: &[((f64, f64), (f64, f64))], px: f64, py: f64, scale: &Scale) -> f64 {
    let (qx, qy) = (px * scale.mx, py * scale.my);
    let mut best = f64::INFINITY;
    for &((ax, ay), (bx, by)) in segs {
        best =
            best.min(point_seg(qx, qy, ax * scale.mx, ay * scale.my, bx * scale.mx, by * scale.my));
    }
    best
}

/// How far the asphalt reaches on the *nearer* side of a painted line, in
/// metres, marching perpendicular to the line's own direction. `None` for a
/// degenerate segment (no direction to be perpendicular to) or when neither
/// side ends inside [`REACH_CAP_M`] — a line in the middle of a plate, whose
/// near kerb is not its carriageway's.
fn near_kerb(
    paved: &[&SurfaceMesh],
    at: (f64, f64),
    seg: ((f64, f64, f64), (f64, f64, f64)),
    scale: &Scale,
) -> Option<f64> {
    let (de, dn) = ((seg.1 .0 - seg.0 .0) * scale.mx, (seg.1 .1 - seg.0 .1) * scale.my);
    let len = (de * de + dn * dn).sqrt();
    if len < 1e-9 {
        return None;
    }
    let (pe, pn) = (-dn / len, de / len);
    let mut best: Option<f64> = None;
    for sign in [1.0f64, -1.0] {
        let on = |d: f64| {
            let qx = at.0 + sign * pe * d / scale.mx;
            let qy = at.1 + sign * pn * d / scale.my;
            paved.iter().any(|m| m.height_at(qx, qy).is_some())
        };
        // Coarse march to the first gap, then refine inside the step that
        // straddles it: the answer is a kerb position, so it wants centimetres,
        // but paying for centimetres over the whole reach would cost the pass
        // its budget. A side whose asphalt runs past the cap yields nothing
        // rather than the cap itself — reporting a saturated march as a
        // measurement would put a fixed number at the top of the offender list
        // and read as a defect of that exact size.
        let mut lo = 0.0;
        let mut gap = None;
        while lo < REACH_CAP_M {
            let hi = (lo + REACH_COARSE_M).min(REACH_CAP_M);
            if !on(hi) {
                gap = Some((lo, hi));
                break;
            }
            lo = hi;
        }
        let Some((lo, hi)) = gap else { continue };
        let mut fine = lo;
        while fine + REACH_FINE_M < hi && on(fine + REACH_FINE_M) {
            fine += REACH_FINE_M;
        }
        best = Some(best.map_or(fine, |b: f64| b.min(fine)));
    }
    best
}

fn point_seg(qx: f64, qy: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (ex, ey) = (bx - ax, by - ay);
    let len2 = ex * ex + ey * ey;
    let t = if len2 > 0.0 { (((qx - ax) * ex + (qy - ay) * ey) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (dx, dy) = (qx - (ax + ex * t), qy - (ay + ey * t));
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::Bounds;
    use crate::verify::scene::{RoadLine, RoadMesh};

    /// A flat quad covering `[x0, x1] × [0, 1]` of the tile, as a road surface.
    fn slab(class: &str, x0: f64, x1: f64) -> RoadMesh {
        let x = vec![x0 as f32, x1 as f32, x1 as f32, x0 as f32];
        let y = vec![0.0f32, 0.0, 1.0, 1.0];
        let z = vec![100.0f32; 4];
        RoadMesh {
            class: class.into(),
            level: 0,
            band: String::new(), fades: false,
            mesh: SurfaceMesh::from_parts(x, y, z, vec![0, 1, 2, 0, 2, 3]).expect("a slab"),
        }
    }

    /// A north–south painted line at plan `px`, `width_m` wide.
    fn stripe(width_m: f64, px: f64) -> RoadLine {
        RoadLine {
            class: "marking".into(),
            level: 0,
            width_m,
            parts: vec![vec![(px, 0.2, 100.0), (px, 0.5, 100.0), (px, 0.8, 100.0)]],
        }
    }

    fn scene(roads: Vec<RoadMesh>, lines: Vec<RoadLine>) -> TileScene {
        let b = Bounds::of_tile(16, 34000, 23000);
        TileScene {
            z: 16,
            x: 34000,
            y: 23000,
            scale: Scale::of(&b),
            bounds: b,
            terrain: None,
            roads,
            lines,
            waters: Vec::new(),
            buildings: Vec::new(),
        }
    }

    fn run(tile: &TileScene) -> Vec<Metric> {
        let mut c = Box::new(Paint::new(&Options::default()));
        c.visit(tile, &Options::default());
        c.finish()
    }

    /// Half the tile's width in metres, the unit the fixtures are laid out in.
    fn mx() -> f64 {
        Scale::of(&Bounds::of_tile(16, 34000, 23000)).mx
    }

    #[test]
    fn paint_on_the_asphalt_reports_nothing() {
        let m = run(&scene(vec![slab("road_surface", 0.4, 0.6)], vec![stripe(0.12, 0.5)]));
        assert_eq!(m[0].violations(), 0, "a centred line is on the road");
        assert!(m[0].dist.count() > 0, "and the population is counted, not skipped");
    }

    #[test]
    fn paint_beside_the_asphalt_is_caught() {
        // The stripe runs 5 m clear of the slab's edge.
        let gap = 5.0 / mx();
        let m = run(&scene(vec![slab("road_surface", 0.4, 0.6)], vec![stripe(0.12, 0.6 + gap)]));
        assert!(m[0].violations() > 0, "paint off the asphalt must be caught");
        let worst = m[0].worst_value().expect("a worst offender");
        assert!((worst - 5.0).abs() < 0.3, "the gap in metres: {worst}");
    }

    #[test]
    fn an_edge_line_collapsed_onto_the_axis_is_caught() {
        // A 9 m carriageway with a 1 m shoulder either side: 11 m of asphalt.
        // Its edge line belongs 4.2 m out, 1.3 m inside the kerb.
        let half = 5.5 / mx();
        let road = slab("road_surface", 0.5 - half, 0.5 + half);
        let placed = run(&scene(vec![road], vec![stripe(0.15, 0.5 + 4.2 / mx())]));
        assert_eq!(placed[1].violations(), 0, "an edge line at its own inset is right");

        let road = slab("road_surface", 0.5 - half, 0.5 + half);
        let collapsed = run(&scene(vec![road], vec![stripe(0.15, 0.5)]));
        assert!(
            collapsed[1].violations() > 0,
            "an edge line projected onto the axis must be caught"
        );
        let worst = collapsed[1].worst_value().expect("a worst offender");
        assert!((worst - (5.5 - 1.3)).abs() < 0.3, "how far off its inset, in metres: {worst}");
    }

    /// A full-tile flat terrain slab at height `h`.
    fn terrain(h: f32) -> SurfaceMesh {
        let x = vec![0.0f32, 1.0, 1.0, 0.0];
        let y = vec![0.0f32, 0.0, 1.0, 1.0];
        let z = vec![h; 4];
        SurfaceMesh::from_parts(x, y, z, vec![0, 1, 2, 0, 2, 3]).expect("a terrain slab")
    }

    /// A stroke on the drawn ground reads zero; one metres under it — a tunnel
    /// span's paint — is caught, at its burial depth.
    #[test]
    fn paint_under_the_drawn_ground_is_caught() {
        let mut tile = scene(Vec::new(), vec![stripe(0.12, 0.5)]);
        tile.terrain = Some(terrain(100.0));
        let m = run(&tile);
        assert_eq!(m[2].violations(), 0, "a stroke on the ground is not buried");
        assert!(m[2].dist.count() > 0, "and it is counted, not skipped");

        let mut buried = stripe(0.12, 0.5);
        for part in &mut buried.parts {
            for v in part.iter_mut() {
                v.2 = 90.0;
            }
        }
        let mut tile = scene(Vec::new(), vec![buried]);
        tile.terrain = Some(terrain(100.0));
        let m = run(&tile);
        assert!(m[2].violations() > 0, "paint under the ground must be caught");
        let worst = m[2].worst_value().expect("a worst offender");
        assert!((worst - 10.0).abs() < 0.1, "the burial depth in metres: {worst}");
    }

    /// A bridge stroke rides its deck above the ground by design and is not in
    /// the population; a tunnel stroke (level < 0) is.
    #[test]
    fn a_bridge_stroke_is_not_in_the_buried_population() {
        let mut deck = stripe(0.12, 0.5);
        deck.level = 1;
        for part in &mut deck.parts {
            for v in part.iter_mut() {
                v.2 = 90.0; // under the slab, but a positive level is excluded
            }
        }
        let mut tile = scene(Vec::new(), vec![deck]);
        tile.terrain = Some(terrain(100.0));
        let m = run(&tile);
        assert!(m[2].skipped.is_some(), "a positive level contributes nothing");

        let mut bore = stripe(0.12, 0.5);
        bore.level = -1;
        for part in &mut bore.parts {
            for v in part.iter_mut() {
                v.2 = 90.0;
            }
        }
        let mut tile = scene(Vec::new(), vec![bore]);
        tile.terrain = Some(terrain(100.0));
        let m = run(&tile);
        assert!(m[2].violations() > 0, "a tunnel stroke under the ground is caught");
    }

    /// A centre line is 0.12 m wide, so it never enters the inset population —
    /// the archive cannot tell it from a lane divider, and guessing would
    /// report correct paint as broken.
    #[test]
    fn a_centre_line_is_not_judged_as_an_edge_line() {
        let half = 5.5 / mx();
        let m = run(&scene(vec![slab("road_surface", 0.5 - half, 0.5 + half)], vec![stripe(0.12, 0.5)]));
        assert!(m[1].skipped.is_some(), "no edge lines here, so nothing to say about insets");
    }
}
