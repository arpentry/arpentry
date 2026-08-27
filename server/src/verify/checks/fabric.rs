//! Invariant 9 (closure), measured between the emitted surfaces.
//!
//! The drawn world at the detail zooms is one fabric: wherever two drawn
//! elements are plan-adjacent and differ in height past the contact band, a
//! face spans the step. Air is legal only where a structure separates two
//! levels — and then the structure's own solids are the closure.
//!
//! `order.grade_stack` measures the ordering half of this ("two at-grade
//! bands should not stack") and cannot see the other half: whether anything
//! *closes* the step. A road on a terrace over a rail cutting is drawn
//! correctly when the cutting's wall rises between the two bands, and is a
//! hole in the world when it does not — the two cases read identically to a
//! band-over-band probe. This check asks the closure question directly: at
//! every stacked pair of at-grade surfaces, how much of the vertical interval
//! between them does no drawn geometry span?
//!
//! The population deliberately includes the pedestrian surfaces
//! (`walk_surface`, `path_surface`) that `order.grade_stack` excludes: a
//! footway band hanging over a street, or a street overhanging the path at
//! the foot of its retaining wall, is the same defect in a junior material.
//!
//! The other closure population — the terrain hole rim, where the ground
//! stops at a band's edge — stays `contact.kerb_unwalled`: it is anchored on
//! the terrain's own boundary edges, which need a different walk.

use crate::verify::dist::Dist;
use crate::verify::scene::{RoadMesh, TileScene};
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// Two at-grade surfaces further apart than this are a step something must
/// span. The same boundary `order.grade_stack` and
/// `solve::crossings::SEPARATION_M` draw: below it sit the pairs the sheets
/// machinery legitimately layers (split carriageways on a cross-slope, a
/// kerb-height braid); past it there is visible air.
const STACK_M: f64 = 3.0;

/// How far, in plan, spanning geometry may stand from the sample and still
/// count as closing the step. Wider than `contact.rs`'s `APRON_NEAR_M`
/// because the closure here is usually a terrain face or an apron on the
/// *lower* band's rim, which stands up to a band's edge inset away from the
/// upper band's interior sample; past a couple of metres the upper band is a
/// cantilever over open air whatever stands beyond it.
const SPAN_NEAR_M: f64 = 2.5;

/// Slack at each end of the interval a face must cover: both surfaces carry
/// quantization, and a face meets a band under its edge rather than at the
/// band's sampled interior point.
const SPAN_SLOP_M: f64 = 0.75;

/// At-grade surface bands of any modality: the fabric's walkable/drivable
/// top. Rims are outlines at surface height and aprons are vertical faces;
/// neither is a *surface* to stack.
fn is_surface(r: &RoadMesh) -> bool {
    r.level == 0
        && matches!(
            r.class.as_str(),
            "road_surface" | "rail_surface" | "walk_surface" | "path_surface"
        )
}

/// Every drawn mesh can close a vertical step — aprons, structure solids,
/// buildings, the drawn terrain, and the surface bands themselves: where the
/// paved union welds two corridors at different heights into one region, the
/// weld is an interior asphalt face, which is `slope.carriageway_face`'s
/// defect (wrong material) but not a hole in the world. This check asks only
/// whether the world can be seen through; what the closing face is made of is
/// other metrics' business. A flat band near the sample contributes only a
/// thin span, so admitting the surfaces closes nothing that is genuinely open.
fn is_closure(r: &RoadMesh) -> bool {
    !r.class.ends_with("_rim")
}

pub struct Fabric {
    spacing_m: f64,
    closure: Dist,
    closure_worst: Worst,
}

impl Fabric {
    pub fn new(opt: &Options) -> Fabric {
        Fabric {
            spacing_m: opt.spacing_m,
            closure: Dist::metres(),
            closure_worst: Worst::new(Sense::HigherIsWorse, opt.worst_k),
        }
    }
}

/// The largest sub-interval of `[lo, hi]` at `(px, py)` that no drawn mesh
/// spans — the closure question (I9), asked pointwise: aprons, solids,
/// buildings, the drawn terrain and the surface bands themselves all count
/// as spans, rims do not.
///
/// Shared with `street`'s `contact.sidewalk_grade`, whose population gate
/// asks the same question with the opposite sense: a walkway a storey from a
/// kerb *behind a closed face* is a terrace with a wall, not that street's
/// pavement, and the relation the check would score does not exist.
pub(super) fn open_step(tile: &TileScene, px: f64, py: f64, lo: f64, hi: f64) -> f64 {
    let mut spans: Vec<(f64, f64)> = Vec::new();
    for r in tile.roads.iter() {
        if !is_closure(r) {
            continue;
        }
        if let Some(s) = r.mesh.span_near(px, py, &tile.scale, SPAN_NEAR_M) {
            spans.push(s);
        }
    }
    if let Some(terrain) = &tile.terrain {
        if let Some(s) = terrain.span_near(px, py, &tile.scale, SPAN_NEAR_M) {
            spans.push(s);
        }
    }
    for (_, b) in &tile.buildings {
        if let Some(s) = b.span_near(px, py, &tile.scale, SPAN_NEAR_M) {
            spans.push(s);
        }
    }
    largest_uncovered(lo, hi, &mut spans)
}

/// The largest sub-interval of `[lo, hi]` no span covers, with
/// [`SPAN_SLOP_M`] forgiven at each end.
fn largest_uncovered(lo: f64, hi: f64, spans: &mut Vec<(f64, f64)>) -> f64 {
    let (lo, hi) = (lo + SPAN_SLOP_M, hi - SPAN_SLOP_M);
    if hi <= lo {
        return 0.0;
    }
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut cursor = lo;
    let mut worst: f64 = 0.0;
    for &(a, b) in spans.iter() {
        if a > cursor {
            worst = worst.max(a.min(hi) - cursor);
        }
        cursor = cursor.max(b);
        if cursor >= hi {
            return worst;
        }
    }
    worst.max(hi - cursor)
}

#[cfg(test)]
mod tests {
    use super::largest_uncovered;

    #[test]
    fn an_unspanned_step_is_open_end_to_end() {
        // 10 m of air, nothing spanning: the slop is forgiven at each end.
        let open = largest_uncovered(0.0, 10.0, &mut vec![]);
        assert!((open - (10.0 - 2.0 * super::SPAN_SLOP_M)).abs() < 1e-9);
    }

    #[test]
    fn a_face_covering_the_step_closes_it() {
        let open = largest_uncovered(0.0, 10.0, &mut vec![(-1.0, 11.0)]);
        assert_eq!(open, 0.0);
    }

    #[test]
    fn a_face_covering_half_leaves_the_other_half() {
        // A wall from below up to 5 m under a band at 10 m: the open air is
        // the top half, less the slop at the band's end.
        let open = largest_uncovered(0.0, 10.0, &mut vec![(-2.0, 5.0)]);
        assert!((open - (10.0 - super::SPAN_SLOP_M - 5.0)).abs() < 1e-9);
    }

    #[test]
    fn two_faces_leaving_a_slot_report_the_slot() {
        let open = largest_uncovered(0.0, 12.0, &mut vec![(0.0, 4.0), (8.0, 12.0)]);
        assert!((open - 4.0).abs() < 1e-9);
    }

    #[test]
    fn a_step_inside_the_slop_is_closed() {
        let open = largest_uncovered(0.0, 2.0 * super::SPAN_SLOP_M, &mut vec![]);
        assert_eq!(open, 0.0);
    }
}

impl Check for Fabric {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        // Each stacked pair is measured once, from its upper side — the same
        // walk `order.grade_stack` takes, extended to the pedestrian bands and
        // followed by the closure question.
        for (i, a) in tile.roads.iter().enumerate() {
            if !is_surface(a) {
                continue;
            }
            a.mesh.sample(&tile.scale, opt.spacing_m, |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((_, own)) = a.mesh.height_range_at(px, py) else { return };
                let mut below = f64::NEG_INFINITY;
                let mut under_class = "";
                for (j, b) in tile.roads.iter().enumerate() {
                    if j == i || !is_surface(b) {
                        continue;
                    }
                    let Some((_, top)) = b.mesh.height_range_at(px, py) else { continue };
                    if top <= own && top > below {
                        below = top;
                        under_class = &b.class;
                    }
                }
                if !below.is_finite() {
                    return;
                }
                let gap = own - below;
                if gap <= STACK_M {
                    // The contact band: braids, sheets, kerb steps. Closed by
                    // definition — the fabric's floor.
                    self.closure.push(0.0);
                    return;
                }
                // The step is real. What spans [below, own] here? The upper
                // band's own mesh is consulted too: where the union welds two
                // heights into one region, the weld face is that mesh's — and
                // a drawn cutting or embankment face between the two bands is
                // exactly the closure this invariant asks for.
                let open = open_step(tile, px, py, below, own);
                self.closure.push(open);
                if open > STACK_M {
                    let (lon, lat) = tile.lonlat(px, py);
                    self.closure_worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: open,
                        note: format!(
                            "{} at {own:.2} m over {under_class} at {below:.2} m; {open:.2} m \
                             of the step no face spans",
                            a.class
                        ),
                    });
                }
            });
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![Metric {
            id: "fabric.closure".into(),
            invariant: Invariant::I9,
            title: "Vertical step between drawn surfaces with no spanning face".into(),
            population: format!(
                "Surface samples of every level-0 band — carriageway, rail formation, walkway \
                 or path — at {:.1} m plan spacing in the tile proper, where another level-0 \
                 band lies at or below the same plan position; each stacked pair measured once, \
                 from its upper side. Pairs within {STACK_M:.1} m are the contact band (braids, \
                 sheets, kerb steps) and enter as closed. The terrain hole rim is \
                 contact.kerb_unwalled's population, not this one.",
                self.spacing_m
            ),
            detail: format!(
                "The largest sub-interval of the vertical step between two stacked at-grade \
                 surfaces that no drawn geometry spans — aprons, structure solids, buildings, \
                 the drawn terrain, or a surface mesh's own interior face — within \
                 {SPAN_NEAR_M:.1} m in plan, with {SPAN_SLOP_M:.2} m of slack at each end. \
                 Invariant 9: air between two surfaces is legal only under a structure. A road \
                 on a terrace over a rail cutting is correct when the cutting's drawn wall \
                 rises between the bands, and a hole in the world when it does not; \
                 order.grade_stack cannot tell those apart, this can. What a closing face is \
                 made of is deliberately not asked: an interior asphalt wall closes the world \
                 and fails slope.carriageway_face, each defect scored by its own instrument."
            ),
            sense: Sense::HigherIsWorse,
            threshold: STACK_M,
            skipped: self
                .closure
                .is_empty()
                .then(|| "no two at-grade bands share a plan position".to_string()),
            dist: self.closure,
            worst: self.closure_worst.into_vec(),
        }]
    }
}
