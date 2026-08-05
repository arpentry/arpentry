//! Polygon booleans and offsets — the one module that knows `i_overlay`
//! (docs/ROADS.md §6.1).
//!
//! The road surface is an *area*, and an area needs three operations a stroke
//! never did: buffer a centerline to its carriageway width, union the results so
//! legs meeting at a junction become one region, and offset a boundary inward or
//! outward (for the curb-return closing and the antialiasing rim). Everything
//! above this module speaks [`geo_types::Coord`] and metres; only this file names
//! the crate, so swapping the kernel is a one-file change.
//!
//! Three decisions are load-bearing and are the reason this wrapper exists at
//! all rather than callers using the crate directly:
//!
//! 1. **Everything happens in local ENU metres, never in degrees.** A buffer in
//!    degree space is anisotropic — a round join comes out an ellipse and a
//!    carriageway's width depends on its latitude. [`MFrame`] is the conversion,
//!    using the same equirectangular scaling as `synth::area`.
//!
//! 2. **One fixed float→integer grid ([`GRID_M`]), pinned, for every
//!    operation.** `i_overlay` works on an integer lattice; by default it derives
//!    that lattice from the *input bounding box*, which would make the same road
//!    snap differently depending on what else happened to be in the batch. The
//!    `_fixed_scale` entry points take the scale explicitly, so identical input
//!    geometry yields identical output vertices no matter what it is batched
//!    with — the determinism the tiler needs (`terrain_cdt.rs:10-22` makes the
//!    same promise for the CDT). At 0.1 mm over a ±2 km chunk the `i64` engine
//!    uses ±2·10⁷ of its ±9·10¹⁸ range, so overflow is not reachable.
//!
//! 3. **Joins are mitered at the same limit the old band used.** `MITER_MAX =
//!    1.5` in the band this replaces (`synth/surface.rs`) clamped the offset
//!    scale at a bend; the equivalent here is a minimum join angle, because
//!    scale `1/sin(α/2) = 1.5` at `α = 83.6° = 1.46 rad`. Turns sharper than
//!    that bevel instead of spiking, exactly as before, so the surface does not
//!    move at the stroke→mesh handoff (ROADS.md invariant 5).
//!
//! Shapes are `i_overlay`'s own nesting rather than a wrapper struct: a shape is
//! a list of contours whose first is the counter-clockwise outer boundary and
//! whose rest are clockwise holes. Results feed straight back in as input, and a
//! newtype would mean rebuilding the nesting on every call.

use geo_types::Coord;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::scale::FixedScaleFloatOverlay;
use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle, StrokeStyle};

use crate::building_mesh::{M_PER_DEG_LAT, M_PER_DEG_LON_EQUATOR};

/// A point in local ENU metres. `i_overlay`'s native point type, so no
/// conversion happens on the hot path.
pub type Pt = [f64; 2];

/// A closed contour. Counter-clockwise when it bounds paved area, clockwise
/// when it bounds a hole; not explicitly closed (the last point does not repeat
/// the first).
pub type Ring = Vec<Pt>;

/// One region: `[0]` is the outer boundary, the rest are holes.
pub type Shape = Vec<Ring>;

/// A set of disjoint regions.
pub type Shapes = Vec<Shape>;

/// The float→integer grid, in metres — 0.1 mm, four orders of magnitude below
/// the centimetre that survives tile quantization, so snapping is invisible.
const GRID_M: f64 = 1e-4;

/// The scale `i_overlay`'s `_fixed_scale` entry points want: reciprocal grid.
const SCALE: f64 = 1.0 / GRID_M;

/// Minimum join angle in radians before a miter bevels — the `MITER_MAX = 1.5`
/// clamp of the band this replaces, expressed as the angle where the miter scale
/// `1/sin(α/2)` reaches 1.5.
const MITER_MIN_RAD: f64 = 1.46;

/// Arc resolution for offset joins, as `L/R` (segment length over radius). At a
/// 3 m curb return this puts a vertex every ~0.6 m, fine enough that the arc
/// reads as curved at the reference zoom without flooding the mesh. `i_overlay`
/// clamps this to `[0.01π, 0.25π]`.
const ARC_STEP: f64 = 0.2;

/// The local metre frame about an origin: the conversion every boolean and
/// offset runs inside. Equirectangular, matching `synth::area`'s scaling, which
/// is exact enough over a chunk (~2 km) and keeps the grid uniform.
#[derive(Debug, Clone, Copy)]
pub struct MFrame {
    origin: Coord,
    m_per_deg_lon: f64,
}

impl MFrame {
    /// The frame about `origin`.
    pub fn of(origin: Coord) -> MFrame {
        MFrame { origin, m_per_deg_lon: M_PER_DEG_LON_EQUATOR * origin.y.to_radians().cos() }
    }

    /// The metre offset of a world point from the origin.
    pub fn to_m(&self, c: Coord) -> Pt {
        [(c.x - self.origin.x) * self.m_per_deg_lon, (c.y - self.origin.y) * M_PER_DEG_LAT]
    }

    /// The world point at a metre offset from the origin.
    pub fn to_deg(&self, p: Pt) -> Coord {
        Coord {
            x: self.origin.x + p[0] / self.m_per_deg_lon,
            y: self.origin.y + p[1] / M_PER_DEG_LAT,
        }
    }
}

/// The paved region of one centerline: the line buffered to `half_m` either
/// side, with butt ends. Butt rather than round because a carriageway ends
/// square — a round cap would bulge a half-disc of asphalt past every corridor
/// end, and an end that continues into a junction is covered by the legs that
/// meet there anyway.
///
/// Empty for a degenerate line (under two distinct points, or a non-positive
/// width): a caller with nothing to buffer gets nothing, not an error.
pub fn buffer_line(line: &[Pt], half_m: f64) -> Shapes {
    if line.len() < 2 || !(half_m > 0.0) {
        return Vec::new();
    }
    let style = StrokeStyle::new(half_m * 2.0).line_join(LineJoin::Miter(MITER_MIN_RAD));
    line.stroke_fixed_scale_as::<i64>(style, false, SCALE).unwrap_or_default()
}

/// The union of everything in `shapes`, as disjoint regions with their holes.
///
/// One boolean pass, not a fold: overlapping counter-clockwise contours under
/// the non-zero fill rule are already the filled region, so the answer is a
/// function of the *set* and cannot depend on the order they were collected in.
pub fn union_all(shapes: &Shapes) -> Shapes {
    if shapes.is_empty() {
        return Vec::new();
    }
    let empty: Shapes = Vec::new();
    shapes
        .overlay_with_fixed_scale_as::<i64>(&empty, OverlayRule::Subject, FillRule::NonZero, SCALE)
        .unwrap_or_default()
}

/// `shapes` grown by `r_m` on every side, corners arced.
pub fn dilate(shapes: &Shapes, r_m: f64) -> Shapes {
    offset(shapes, r_m)
}

/// `shapes` shrunk by `r_m` on every side. Regions narrower than `2·r_m`
/// vanish, which is what makes the closing below work.
pub fn erode(shapes: &Shapes, r_m: f64) -> Shapes {
    offset(shapes, -r_m)
}

fn offset(shapes: &Shapes, delta_m: f64) -> Shapes {
    if shapes.is_empty() || delta_m == 0.0 {
        return shapes.clone();
    }
    let style = OutlineStyle::new(delta_m).line_join(LineJoin::Round(ARC_STEP));
    shapes.outline_fixed_scale_as::<i64>(&style, SCALE).unwrap_or_default()
}

/// The intersection of two sets of regions.
pub fn intersect(a: &Shapes, b: &Shapes) -> Shapes {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    a.overlay_with_fixed_scale_as::<i64>(b, OverlayRule::Intersect, FillRule::NonZero, SCALE)
        .unwrap_or_default()
}

/// `a` minus `b`.
pub fn difference(a: &Shapes, b: &Shapes) -> Shapes {
    if a.is_empty() || b.is_empty() {
        return a.clone();
    }
    a.overlay_with_fixed_scale_as::<i64>(b, OverlayRule::Difference, FillRule::NonZero, SCALE)
        .unwrap_or_default()
}

/// The parts of `shapes` inside the axis-aligned metre rect
/// `(x0, y0, x1, y1)`.
pub fn intersect_rect(shapes: &Shapes, rect: (f64, f64, f64, f64)) -> Shapes {
    let (x0, y0, x1, y1) = rect;
    if x1 <= x0 || y1 <= y0 {
        return Vec::new();
    }
    // Counter-clockwise, so it reads as filled area under the non-zero rule.
    let clip: Shapes = vec![vec![vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]]];
    intersect(shapes, &clip)
}

/// A **morphological closing restricted to `masks`**: rounds the reflex corners
/// of `open` at radius `r_m`, but only where a mask says a corner is wanted.
///
/// Dilating then eroding by `r_m` rounds every concave corner in one step —
/// which is exactly the curb return where two carriageways meet, and exactly
/// wrong everywhere else, because it also bridges any gap narrower than `2·r_m`:
/// the two carriageways of a divided road would fuse into one slab and a narrow
/// median would disappear. Intersecting the closed region with the intersection
/// extents before unioning it back keeps the fillets local, so a long straight
/// road and the median beside it come out of this untouched.
///
/// Degrades to `open` unchanged when the offsets fail — hard corners, never a
/// broken region (docs/GENERATION.md I6).
pub fn close_within(open: &Shapes, r_m: f64, masks: &Shapes) -> Shapes {
    if open.is_empty() || masks.is_empty() || !(r_m > 0.0) {
        return open.clone();
    }
    let closed = erode(&dilate(open, r_m), r_m);
    if closed.is_empty() {
        return open.clone();
    }
    // `closed ⊇ open`, so unioning `open` with the masked closing is the same as
    // unioning it with the masked *fillet material* — one boolean fewer.
    let fillets = intersect(&closed, masks);
    if fillets.is_empty() {
        return open.clone();
    }
    let mut both = open.clone();
    both.extend(fillets);
    let out = union_all(&both);
    if out.is_empty() {
        open.clone()
    } else {
        out
    }
}

/// Total area in square metres — outer contours positive, holes negative.
/// The shoelace sum, used by the tests and by the archive-size check.
pub fn area(shapes: &Shapes) -> f64 {
    shapes.iter().flatten().map(|r| ring_area(r)).sum::<f64>()
}

/// Twice the signed area of one contour, halved: positive counter-clockwise.
fn ring_area(ring: &Ring) -> f64 {
    if ring.len() < 3 {
        return 0.0;
    }
    let mut acc = 0.0;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        acc += a[0] * b[1] - b[0] * a[1];
    }
    acc * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A straight run west→east through the origin.
    fn straight(len_m: f64) -> Vec<Pt> {
        vec![[-0.5 * len_m, 0.0], [0.5 * len_m, 0.0]]
    }

    #[test]
    fn a_buffered_line_is_a_rectangle_of_its_width() {
        // Butt ends, so the area is exactly length × width with no cap discs.
        let shapes = buffer_line(&straight(100.0), 4.0);
        assert_eq!(shapes.len(), 1, "one region");
        assert!(shapes[0].len() == 1, "no holes: {:?}", shapes[0].len());
        let a = area(&shapes);
        assert!((a - 800.0).abs() < 800.0 * 0.01, "area {a} is not 100 x 8");
    }

    #[test]
    fn a_degenerate_line_buffers_to_nothing() {
        assert!(buffer_line(&[], 4.0).is_empty(), "no points");
        assert!(buffer_line(&[[0.0, 0.0]], 4.0).is_empty(), "one point");
        assert!(buffer_line(&straight(100.0), 0.0).is_empty(), "no width");
        assert!(buffer_line(&straight(100.0), -1.0).is_empty(), "negative width");
    }

    #[test]
    fn two_crossing_roads_union_into_one_region() {
        let ew = buffer_line(&straight(100.0), 4.0);
        let ns = buffer_line(&[[0.0, -50.0], [0.0, 50.0]], 3.0);
        let mut all = ew.clone();
        all.extend(ns.clone());
        let u = union_all(&all);
        assert_eq!(u.len(), 1, "the crossing is one region");
        assert_eq!(u[0].len(), 1, "and it has no hole");
        // Inclusion–exclusion: 100x8 + 100x6 - the 8x6 shared square.
        let want = 800.0 + 600.0 - 48.0;
        let got = area(&u);
        assert!((got - want).abs() < want * 0.01, "area {got} != {want}");
    }

    #[test]
    fn the_union_does_not_depend_on_input_order() {
        let a = buffer_line(&straight(100.0), 4.0);
        let b = buffer_line(&[[0.0, -50.0], [0.0, 50.0]], 3.0);
        let c = buffer_line(&[[-30.0, -30.0], [30.0, 30.0]], 2.5);
        let mut abc = a.clone();
        abc.extend(b.clone());
        abc.extend(c.clone());
        let mut cba = c;
        cba.extend(b);
        cba.extend(a);
        assert_eq!(union_all(&abc), union_all(&cba), "union is order-dependent");
    }

    #[test]
    fn a_ring_of_roads_keeps_its_island() {
        // Four 3 m-wide sides of a 40 m square: a roundabout's ring of arcs.
        let corners = [[-20.0, -20.0], [20.0, -20.0], [20.0, 20.0], [-20.0, 20.0]];
        let mut all: Shapes = Vec::new();
        for i in 0..4 {
            all.extend(buffer_line(&[corners[i], corners[(i + 1) % 4]], 3.0));
        }
        let u = union_all(&all);
        assert_eq!(u.len(), 1, "one band");
        assert_eq!(u[0].len(), 2, "outer boundary plus one island hole");
        // The island is the square inset by the half-width: 34 x 34.
        let hole = ring_area(&u[0][1]).abs();
        assert!((hole - 34.0 * 34.0).abs() < 34.0 * 34.0 * 0.02, "island {hole} != 1156");
    }

    #[test]
    fn eroding_past_the_half_width_removes_a_region() {
        let shapes = buffer_line(&straight(100.0), 4.0);
        let thin = erode(&shapes, 3.0);
        assert!(!thin.is_empty(), "4 m half-width survives a 3 m erode");
        assert!(area(&thin) < area(&shapes), "erode did not shrink");
        assert!(erode(&shapes, 5.0).is_empty(), "a 4 m half-width cannot survive 5 m");
    }

    #[test]
    fn closing_rounds_a_reflex_corner_inside_a_mask() {
        // An L of two carriageways meeting at the origin has one reflex corner
        // on its inner side. A mask over the corner should round it.
        let mut all = buffer_line(&[[-50.0, 0.0], [0.0, 0.0]], 4.0);
        all.extend(buffer_line(&[[0.0, 0.0], [0.0, 50.0]], 4.0));
        let open = union_all(&all);
        let mask: Shapes = vec![vec![vec![
            [-12.0, -12.0],
            [12.0, -12.0],
            [12.0, 12.0],
            [-12.0, 12.0],
        ]]];
        let closed = close_within(&open, 3.0, &mask);
        assert!(area(&closed) > area(&open), "the fillet added no asphalt");
        // The fillet is small: a quarter-disc-ish of radius 3 is well under 10 m².
        assert!(area(&closed) - area(&open) < 10.0, "the closing added far too much");
    }

    #[test]
    fn closing_leaves_a_divided_carriageway_unfused() {
        // Two parallel 8 m carriageways with a 5 m median — closer than the
        // 2 x 3 m a global closing would bridge. With no mask over them they
        // must come out untouched, still two regions.
        let mut all = buffer_line(&[[-50.0, 6.5], [50.0, 6.5]], 4.0);
        all.extend(buffer_line(&[[-50.0, -6.5], [50.0, -6.5]], 4.0));
        let open = union_all(&all);
        assert_eq!(open.len(), 2, "the median keeps them apart to begin with");
        // A mask far away from the median: the closing must not reach it.
        let mask: Shapes =
            vec![vec![vec![[200.0, 200.0], [220.0, 200.0], [220.0, 220.0], [200.0, 220.0]]]];
        let closed = close_within(&open, 3.0, &mask);
        assert_eq!(closed.len(), 2, "the carriageways fused across the median");
        assert_eq!(closed, open, "an unmasked region was modified");
    }

    #[test]
    fn closing_degrades_to_the_open_region() {
        let open = buffer_line(&straight(100.0), 4.0);
        assert_eq!(close_within(&open, 3.0, &Vec::new()), open, "no masks: unchanged");
        assert_eq!(close_within(&open, 0.0, &open), open, "no radius: unchanged");
        assert!(close_within(&Vec::new(), 3.0, &open).is_empty(), "nothing to close");
    }

    #[test]
    fn a_rect_clip_lands_exactly_on_the_rect_bound() {
        let shapes = buffer_line(&straight(100.0), 4.0);
        let clipped = intersect_rect(&shapes, (-10.0, -10.0, 10.0, 10.0));
        assert_eq!(clipped.len(), 1);
        // Every vertex is inside, and the cut ones sit exactly on the bound —
        // the property two neighbouring chunks rely on to agree at their seam.
        let mut on_east = 0;
        for &p in clipped.iter().flatten().flatten() {
            assert!(p[0] >= -10.0 - GRID_M && p[0] <= 10.0 + GRID_M, "x {} escaped", p[0]);
            assert!(p[1] >= -10.0 - GRID_M && p[1] <= 10.0 + GRID_M, "y {} escaped", p[1]);
            if p[0] == 10.0 {
                on_east += 1;
            }
        }
        assert_eq!(on_east, 2, "the east cut should be two exact vertices");
        let a = area(&clipped);
        assert!((a - 20.0 * 8.0).abs() < 1.0, "clipped area {a} != 160");
    }

    #[test]
    fn an_empty_rect_clips_to_nothing() {
        let shapes = buffer_line(&straight(100.0), 4.0);
        assert!(intersect_rect(&shapes, (10.0, 0.0, -10.0, 5.0)).is_empty(), "inverted x");
        assert!(intersect_rect(&shapes, (0.0, 5.0, 5.0, 5.0)).is_empty(), "zero height");
        assert!(intersect_rect(&shapes, (500.0, 500.0, 510.0, 510.0)).is_empty(), "disjoint");
    }

    #[test]
    fn the_metre_frame_round_trips() {
        let f = MFrame::of(Coord { x: 6.6, y: 46.5 });
        for &(e, n) in &[(0.0, 0.0), (100.0, -250.0), (-1800.0, 1800.0)] {
            let c = f.to_deg([e, n]);
            let back = f.to_m(c);
            assert!((back[0] - e).abs() < 1e-6, "east {} != {e}", back[0]);
            assert!((back[1] - n).abs() < 1e-6, "north {} != {n}", back[1]);
        }
    }
}
