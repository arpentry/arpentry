//! Polygons: the one module that knows `i_overlay`.
//!
//! A surface is an area, and an area needs what a line never did: a
//! centreline buffered to its width, regions unioned so legs meeting at a
//! junction become one, a boundary offset for the kerb returns. Everything
//! above this module speaks metres and [`Shapes`]; only this file names the
//! crate, so swapping the kernel is a one-file change. The server's
//! `synth::poly` made the same choice and the same two decisions, which are
//! repeated here because they are the reason the wrapper exists:
//!
//! 1. **One fixed float→integer grid, [`GRID_M`], for every operation.**
//!    `i_overlay` works on an integer lattice and by default derives it from
//!    the input's bounding box, so the same road would snap differently
//!    depending on what it was batched with. The `_fixed_scale` entry points
//!    take the scale explicitly, and the booleans go further and share one
//!    adapter centred on the frame origin ([`PIN_M`]), so an integer in is
//!    an integer out and only newly created intersection vertices round —
//!    once. Identical input yields identical output, which is what makes
//!    the SVG and the GLB functions of the world alone. At 0.1 mm over a
//!    ±16 km world the `i64` engine uses ±1.6·10⁸ of its ±9·10¹⁸ range.
//!
//! 2. **Round joins, and round caps where legs join.** The tiler buffered
//!    with butt ends and miters because its band had to agree with a stroke
//!    the client drew at coarse zooms (docs/ROADS.md invariant 5). This
//!    crate has no stroke to agree with. A round join never spikes, however
//!    sharp the hairpin, and at a connector a round cap is a disc that
//!    meets every leg whatever the angle, where a butt cap leaves a notch at
//!    every leg that is not collinear. The cap is a join device and nothing
//!    more: where an end joins nothing ([`crate::ribbon`] decides) it is
//!    square, because nothing built ends in a semicircle of its own
//!    half-width.
//!
//! Shapes are `i_overlay`'s own nesting rather than a wrapper struct: a shape
//! is a list of contours whose first is the counter-clockwise outer boundary
//! and whose rest are clockwise holes. Results feed straight back in as
//! input.

use std::collections::HashMap;

use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::ShapeType;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::overlay::FloatOverlay;
use i_overlay::i_float::adapter::FloatPointAdapter;
use i_overlay::i_float::float::rect::FloatRect;
use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, OutlineStyle, StrokeStyle};

/// A point in local metres. `i_overlay`'s native point type, so nothing is
/// converted on the way in or out.
pub type Pt = [f64; 2];

/// A closed contour, not explicitly closed (the last point does not repeat
/// the first). Counter-clockwise when it bounds area, clockwise for a hole.
pub type Ring = Vec<Pt>;

/// One region: `[0]` is the outer boundary, the rest are holes.
pub type Shape = Vec<Ring>;

/// A set of disjoint regions.
pub type Shapes = Vec<Shape>;

/// The float→integer grid, in metres: 0.1 mm.
pub const GRID_M: f64 = 1e-4;

/// The scale `i_overlay`'s `_fixed_scale` entry points want: reciprocal grid.
const SCALE: f64 = 1.0 / GRID_M;

/// Half-extent in metres of the pinned adapter's rect, centred on the frame
/// origin so the lattice offset is exactly zero. Wider than any world this
/// crate builds (a bbox is a zone plus its margin, under 15 km).
const PIN_M: f64 = 16384.0;

/// One boolean on the pinned lattice.
fn overlay(a: &Shapes, b: &Shapes, rule: OverlayRule) -> Shapes {
    let rect = FloatRect::new(-PIN_M, PIN_M, -PIN_M, PIN_M);
    let adapter = FloatPointAdapter::<Pt, i64>::with_scale(rect, SCALE);
    let cap = a.iter().chain(b.iter()).flatten().map(Vec::len).sum();
    FloatOverlay::<Pt, i64>::with_adapter(adapter, cap)
        .unsafe_add_source(a, ShapeType::Subject)
        .unsafe_add_source(b, ShapeType::Clip)
        .overlay(rule, FillRule::NonZero)
}

/// The union of everything in `shapes`, as disjoint regions with their
/// holes. One boolean pass, not a fold: overlapping counter-clockwise
/// contours under the non-zero rule are already the filled region, so the
/// answer is a function of the set and not of the order it was collected in.
pub fn union_all(shapes: &Shapes) -> Shapes {
    if shapes.is_empty() {
        return Vec::new();
    }
    overlay(shapes, &Vec::new(), OverlayRule::Subject)
}

/// [`union_all`] over several sets at once: one pass over their
/// concatenation.
pub fn union_of(parts: &[&Shapes]) -> Shapes {
    let all: Shapes = parts.iter().flat_map(|s| s.iter().cloned()).collect();
    union_all(&all)
}

/// `a` minus `b`.
pub fn difference(a: &Shapes, b: &Shapes) -> Shapes {
    if a.is_empty() || b.is_empty() {
        return a.clone();
    }
    overlay(a, b, OverlayRule::Difference)
}

/// The intersection of two sets of regions.
pub fn intersect(a: &Shapes, b: &Shapes) -> Shapes {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    overlay(a, b, OverlayRule::Intersect)
}

/// Arc resolution for round caps and joins, as `L/R` (segment length over
/// radius). At a 2.75 m half-width this puts a vertex every 0.55 m round a
/// cap, fine enough to read as a curve from above at a metre per pixel.
/// `i_overlay` clamps it to `[0.01π, 0.25π]`.
pub const ARC_STEP: f64 = 0.2;

/// `line` buffered to `width_m`, round-capped and round-joined, as a set of
/// disjoint regions. Empty for a degenerate line (under two points, or a
/// non-positive width): a caller with nothing to buffer gets nothing.
pub fn buffer_line(line: &[Pt], width_m: f64) -> Shapes {
    buffer_line_capped(line, width_m, [true, true])
}

/// [`buffer_line`] with each end round (`true`) or squared off at the
/// endpoint (`false`).
pub fn buffer_line_capped(line: &[Pt], width_m: f64, round: [bool; 2]) -> Shapes {
    if line.len() < 2 || !(width_m > 0.0) {
        return Vec::new();
    }
    let cap = |round: bool| if round { LineCap::Round(ARC_STEP) } else { LineCap::Butt };
    let style = StrokeStyle::new(width_m)
        .start_cap(cap(round[0]))
        .end_cap(cap(round[1]))
        .line_join(LineJoin::Round(ARC_STEP));
    line.stroke_fixed_scale_as::<i64>(style, false, SCALE).unwrap_or_default()
}

/// `shapes` grown by `r_m` on every side, corners arced.
pub fn dilate(shapes: &Shapes, r_m: f64) -> Shapes {
    offset(shapes, r_m, LineJoin::Round(ARC_STEP))
}

/// `shapes` grown by `r_m` on every side, corners kept sharp: the inverse
/// of an erosion by `r_m`, which an arced dilation is not — an arc about a
/// convex corner stops `r·(1/sin(θ/2) − 1)` short of the point the erosion
/// took away.
pub fn dilate_sharp(shapes: &Shapes, r_m: f64) -> Shapes {
    offset(shapes, r_m, LineJoin::Miter(MITER_MIN_RAD))
}

/// `shapes` shrunk by `r_m` on every side. Regions narrower than `2·r_m`
/// vanish, which is what makes a closing round a notch and nothing else.
pub fn erode(shapes: &Shapes, r_m: f64) -> Shapes {
    offset(shapes, -r_m, LineJoin::Round(ARC_STEP))
}

/// The sharpest corner a mitered offset still miters, in radians; sharper
/// ones bevel. `i_overlay` clamps it to `[0.01π, 0.99π]`.
const MITER_MIN_RAD: f64 = 0.05;

fn offset(shapes: &Shapes, delta_m: f64, join: LineJoin<f64>) -> Shapes {
    if shapes.is_empty() || delta_m == 0.0 {
        return shapes.clone();
    }
    let style = OutlineStyle::new(delta_m).line_join(join);
    shapes.outline_fixed_scale_as::<i64>(&style, SCALE).unwrap_or_default()
}

/// `shapes` with every hole under `min_m2` filled: a hole that small is a
/// sliver a boolean left where two pieces met, not a feature the data
/// could describe.
pub fn fill_holes_under(shapes: Shapes, min_m2: f64) -> Shapes {
    shapes
        .into_iter()
        .map(|shape| {
            let mut it = shape.into_iter();
            let outer = it.next();
            outer.into_iter().chain(it.filter(|ring| -ring_area(ring) >= min_m2)).collect()
        })
        .collect()
}

/// Total area in square metres: outer contours positive, holes negative.
pub fn area(shapes: &Shapes) -> f64 {
    // `Sum` for f64 starts from −0.0, which `{:.0}` prints as "-0".
    0.0 + shapes.iter().flatten().map(|r| ring_area(r)).sum::<f64>()
}

/// How many holes the regions of `shapes` have between them.
pub fn holes(shapes: &Shapes) -> usize {
    shapes.iter().map(|s| s.len() - 1).sum()
}

/// The signed area of one contour: positive counter-clockwise.
pub fn ring_area(ring: &Ring) -> f64 {
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

/// `ring` wound counter-clockwise (`ccw`) or clockwise: reversed if it
/// runs the other way, left alone if it has no area.
pub fn oriented(mut ring: Ring, ccw: bool) -> Ring {
    let a = ring_area(&ring);
    if (ccw && a < 0.0) || (!ccw && a > 0.0) {
        ring.reverse();
    }
    ring
}

/// `ring` counter-clockwise, or `None` if it has no area.
pub fn ccw(ring: Ring) -> Option<Ring> {
    (ring_area(&ring).abs() >= 1e-6).then(|| oriented(ring, true))
}

/// The convex hull of `pts`, counter-clockwise, or `None` if it has no
/// area: fewer than three distinct points, or all on a line.
pub fn convex_hull(mut pts: Vec<Pt>) -> Option<Ring> {
    pts.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    pts.dedup();
    if pts.len() < 3 {
        return None;
    }
    let cross = |o: Pt, a: Pt, b: Pt| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut hull: Vec<Pt> = Vec::new();
    for &p in &pts {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower = hull.len() + 1;
    for &p in pts.iter().rev() {
        while hull.len() >= lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();
    ccw(hull)
}

/// The axis-aligned box `[x0, y0, x1, y1]` as a counter-clockwise region.
pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Shape {
    vec![vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]]
}

/// The bounding box `[x0, y0, x1, y1]` of `pts`; `None` of nothing.
pub fn bounds(pts: impl IntoIterator<Item = Pt>) -> Option<[f64; 4]> {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in pts {
        b[0] = b[0].min(p[0]);
        b[1] = b[1].min(p[1]);
        b[2] = b[2].max(p[0]);
        b[3] = b[3].max(p[1]);
    }
    (b[0] <= b[2]).then_some(b)
}

/// `v` scaled to unit length; the zero vector stays zero.
pub fn unit(v: Pt) -> Pt {
    let len = v[0].hypot(v[1]);
    if len < 1e-12 {
        [0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len]
    }
}

/// The length of the polyline `pts`, in metres.
pub fn length(pts: &[Pt]) -> f64 {
    pts.windows(2).map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1])).sum()
}

/// How far the path `a → b → c` turns at `b`, in degrees: positive to the
/// left, negative to the right.
pub fn turn_deg(a: Pt, b: Pt, c: Pt) -> f64 {
    let (u, v) = (unit([b[0] - a[0], b[1] - a[1]]), unit([c[0] - b[0], c[1] - b[1]]));
    (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).to_degrees()
}

/// The point of the segment `ab` nearest `p`.
pub fn nearest_on_segment(a: Pt, b: Pt, p: Pt) -> Pt {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    [a[0] + dx * t, a[1] + dy * t]
}

/// The distance from `p` to the segment `ab`.
pub fn segment_distance(a: Pt, b: Pt, p: Pt) -> f64 {
    let f = nearest_on_segment(a, b, p);
    (p[0] - f[0]).hypot(p[1] - f[1])
}

/// Whether `p` lies inside `shapes`: inside some region's outer boundary and
/// outside its holes. Even-odd over each region's rings, which is the same
/// answer as non-zero because a hole lies inside its outer boundary. A point
/// on a boundary may answer either way.
pub fn contains(shapes: &Shapes, p: Pt) -> bool {
    shapes.iter().any(|shape| {
        let crossings: usize = shape.iter().map(|ring| ring_crossings(ring, p)).sum();
        crossings % 2 == 1
    })
}

/// Cell size of the region index, in metres: a few houses per cell, and a
/// world-sized region in a few tens of thousands.
pub const CELL_M: f64 = 32.0;

/// The cell of `p` on a grid of `cell_m`.
pub fn cell_of(p: Pt, cell_m: f64) -> (i32, i32) {
    ((p[0] / cell_m).floor() as i32, (p[1] / cell_m).floor() as i32)
}

/// Every cell of a grid of `cell_m` the box `b` (`[x0, y0, x1, y1]`)
/// touches, column-major.
pub fn cells_over(b: [f64; 4], cell_m: f64) -> impl Iterator<Item = (i32, i32)> {
    let (c0, r0) = cell_of([b[0], b[1]], cell_m);
    let (c1, r1) = cell_of([b[2], b[3]], cell_m);
    (c0..=c1).flat_map(move |c| (r0..=r1).map(move |r| (c, r)))
}

/// Regions on a grid, for many `contains` queries from a point. Only the
/// regions whose box covers the point's cell are asked, and each is asked
/// over the edges that span the point's row alone: the ray toward +x is
/// crossed by no other, so the parity — the answer — is [`contains`]'s
/// exactly, at a few edges per query where a region the size of the world
/// has tens of thousands.
pub struct Indexed {
    cells: HashMap<(i32, i32), Vec<usize>>,
    /// The edges of region `i` spanning row `r`, keyed `(i, r)`.
    edges: HashMap<(usize, i32), Vec<(Pt, Pt)>>,
}

impl Indexed {
    pub fn new(shapes: &Shapes) -> Indexed {
        let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        let mut edges: HashMap<(usize, i32), Vec<(Pt, Pt)>> = HashMap::new();
        for (i, shape) in shapes.iter().enumerate() {
            let Some(b) = bounds(shape.iter().flatten().copied()) else {
                continue;
            };
            for cell in cells_over(b, CELL_M) {
                cells.entry(cell).or_default().push(i);
            }
            for ring in shape {
                for k in 0..ring.len() {
                    let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                    let (r0, r1) = ((a[1].min(b[1]) / CELL_M).floor() as i32, (a[1].max(b[1]) / CELL_M).floor() as i32);
                    for r in r0..=r1 {
                        edges.entry((i, r)).or_default().push((a, b));
                    }
                }
            }
        }
        Indexed { cells, edges }
    }

    /// [`contains`], over the regions whose box covers `p`'s cell.
    pub fn contains(&self, p: Pt) -> bool {
        let (c, r) = cell_of(p, CELL_M);
        self.cells.get(&(c, r)).is_some_and(|v| {
            v.iter().any(|&i| {
                self.edges.get(&(i, r)).is_some_and(|es| es.iter().filter(|&&(a, b)| crosses(a, b, p)).count() % 2 == 1)
            })
        })
    }

    /// Whether any region is indexed at all.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

/// The edges of a set of regions, bucketed by cell, for many "is this point
/// within `r` of the boundary" queries: a point is checked against the
/// edges of its own cell and the eight around it, so `r` may not exceed
/// the cell size.
pub struct Edges {
    cells: HashMap<(i32, i32), Vec<(Pt, Pt)>>,
    cell_m: f64,
}

impl Edges {
    pub fn new(shapes: &Shapes, cell_m: f64) -> Edges {
        let mut cells: HashMap<(i32, i32), Vec<(Pt, Pt)>> = HashMap::new();
        for ring in shapes.iter().flatten() {
            for i in 0..ring.len() {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                for cell in cells_over([a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])], cell_m) {
                    cells.entry(cell).or_default().push((a, b));
                }
            }
        }
        Edges { cells, cell_m }
    }

    /// Whether some edge passes within `r` of `p`.
    pub fn within(&self, p: Pt, r: f64) -> bool {
        debug_assert!(r <= self.cell_m, "a reach past the neighbouring cells is not searched");
        let (c, row) = cell_of(p, self.cell_m);
        (-1..=1).any(|dc| {
            (-1..=1).any(|dr| {
                self.cells
                    .get(&(c + dc, row + dr))
                    .is_some_and(|v| v.iter().any(|&(a, b)| segment_distance(a, b, p) <= r))
            })
        })
    }
}

/// Whether the edge `ab` crosses the ray from `p` toward +x.
fn crosses(a: Pt, b: Pt, p: Pt) -> bool {
    (a[1] > p[1]) != (b[1] > p[1]) && a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) > p[0]
}

/// How many edges of `ring` a ray from `p` toward +x crosses.
fn ring_crossings(ring: &Ring, p: Pt) -> usize {
    (0..ring.len()).filter(|&i| crosses(ring[i], ring[(i + 1) % ring.len()], p)).count()
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    /// A straight of length `len` and width `w` with two round caps.
    fn straight_area(len: f64, w: f64) -> f64 {
        len * w + PI * (w / 2.0) * (w / 2.0)
    }

    #[test]
    fn a_straight_is_a_rectangle_with_two_half_discs() {
        let shapes = buffer_line(&[[-100.0, 0.0], [100.0, 0.0]], 5.5);
        assert_eq!(shapes.len(), 1);
        assert_eq!(shapes[0].len(), 1, "no holes");
        let a = area(&shapes);
        let exact = straight_area(200.0, 5.5);
        // The caps are polygons inscribed in the disc: under the exact
        // area, and by no more than the arc step's sagitta allows.
        let disc = PI * 2.75 * 2.75;
        assert!(a <= exact + 1e-3, "{a} > {exact}");
        assert!(a >= exact - 0.01 * disc, "{a} < {exact}");
    }

    #[test]
    fn inside_and_outside_are_the_half_width() {
        let shapes = buffer_line(&[[-100.0, 0.0], [100.0, 0.0]], 5.5);
        for x in [-99.0, -50.0, 0.0, 42.0, 99.0] {
            assert!(contains(&shapes, [x, 2.7]), "{x}");
            assert!(contains(&shapes, [x, -2.7]), "{x}");
            assert!(!contains(&shapes, [x, 2.8]), "{x}");
            assert!(!contains(&shapes, [x, -2.8]), "{x}");
        }
        // The cap: inside a little past the end, outside at the radius.
        assert!(contains(&shapes, [102.0, 0.0]));
        assert!(!contains(&shapes, [102.8, 0.0]));
        assert!(!contains(&shapes, [102.0, 2.0]));
    }

    #[test]
    fn a_hairpin_is_one_simple_region() {
        // 20° between the legs: the inner offsets cross each other.
        let a = (10.0f64).to_radians();
        let line = [[100.0 * a.cos(), 100.0 * a.sin()], [0.0, 0.0], [100.0 * a.cos(), -100.0 * a.sin()]];
        let shapes = buffer_line(&line, 5.5);
        assert_eq!(shapes.len(), 1, "{shapes:?}");
        assert_eq!(shapes[0].len(), 1, "no holes");
        // Less than two straights: the legs overlap near the bend.
        assert!(area(&shapes) < 2.0 * straight_area(100.0, 5.5));
        // Wider than one: the legs are not on top of each other.
        assert!(area(&shapes) > 1.5 * straight_area(100.0, 5.5));
    }

    #[test]
    fn output_is_a_function_of_the_input() {
        let line = [[-100.0, 0.0], [0.0, 30.0], [100.0, 0.0]];
        assert_eq!(buffer_line(&line, 5.5), buffer_line(&line, 5.5));
        // Snapped to the grid: every coordinate is a multiple of GRID_M.
        for p in buffer_line(&line, 5.5).iter().flatten().flatten() {
            for v in p {
                let k = (v / GRID_M).round();
                assert!((v - k * GRID_M).abs() < 1e-9, "{v}");
            }
        }
    }

    #[test]
    fn degenerate_input_buffers_to_nothing() {
        assert!(buffer_line(&[[0.0, 0.0]], 5.5).is_empty());
        assert!(buffer_line(&[[0.0, 0.0], [1.0, 0.0]], 0.0).is_empty());
        assert!(buffer_line(&[], 5.5).is_empty());
    }

    fn square(x: f64, y: f64, s: f64) -> Shape {
        rect(x, y, x + s, y + s)
    }

    #[test]
    fn a_union_merges_what_overlaps_and_keeps_what_does_not() {
        let shapes: Shapes = vec![square(0.0, 0.0, 10.0), square(5.0, 0.0, 10.0), square(30.0, 0.0, 10.0)];
        let u = union_all(&shapes);
        assert_eq!(u.len(), 2, "{u:?}");
        assert!((area(&u) - 250.0).abs() < 1e-6);
        assert_eq!(union_all(&Vec::new()), Vec::<Shape>::new());
    }

    #[test]
    fn a_ring_of_ribbons_has_a_hole() {
        // Four squares round an empty middle: one region, one hole.
        let shapes: Shapes = vec![
            square(0.0, 0.0, 30.0),
            vec![vec![[10.0, 10.0], [10.0, 20.0], [20.0, 20.0], [20.0, 10.0]]],
        ];
        let u = union_all(&shapes);
        assert_eq!(u.len(), 1);
        assert_eq!(u[0].len(), 2, "outer plus hole: {u:?}");
        assert!((area(&u) - 800.0).abs() < 1e-6);
        assert!(!contains(&u, [15.0, 15.0]));
    }

    #[test]
    fn a_difference_bites_and_an_intersection_keeps() {
        let a: Shapes = vec![square(0.0, 0.0, 10.0)];
        let b: Shapes = vec![square(5.0, 0.0, 10.0)];
        let d = difference(&a, &b);
        assert!((area(&d) - 50.0).abs() < 1e-6);
        assert!(contains(&d, [2.0, 5.0]) && !contains(&d, [7.0, 5.0]));
        let i = intersect(&a, &b);
        assert!((area(&i) - 50.0).abs() < 1e-6);
        assert!(contains(&i, [7.0, 5.0]) && !contains(&i, [2.0, 5.0]));
        assert!(intersect(&a, &vec![square(20.0, 0.0, 5.0)]).is_empty());
        assert_eq!(difference(&a, &Vec::new()), a);
    }

    #[test]
    fn a_closing_rounds_a_notch_and_leaves_a_convex_shape_alone() {
        // An L: a 30×30 square less its top-right 20×20 corner.
        let l: Shapes = vec![vec![vec![
            [0.0, 0.0],
            [30.0, 0.0],
            [30.0, 10.0],
            [10.0, 10.0],
            [10.0, 30.0],
            [0.0, 30.0],
        ]]];
        let closed = erode(&dilate(&l, 4.0), 4.0);
        let gain = area(&closed) - area(&l);
        let exact = 16.0 * (1.0 - PI / 4.0);
        assert!((gain - exact).abs() < 0.05 * exact, "{gain} vs {exact}");
        // A convex shape comes back to within the arcs' sagitta: the
        // dilated corners are inscribed polygons, so the erosion clips a
        // few square centimetres off each corner.
        let sq: Shapes = vec![vec![vec![[0.0, 0.0], [30.0, 0.0], [30.0, 30.0], [0.0, 30.0]]]];
        let closed = erode(&dilate(&sq, 4.0), 4.0);
        assert!((area(&closed) - 900.0).abs() < 0.5, "{}", area(&closed));
        // A gap narrower than 2r bridges; wider does not.
        let two: Shapes = vec![square(0.0, 0.0, 10.0), square(16.0, 0.0, 10.0)];
        assert_eq!(erode(&dilate(&two, 4.0), 4.0).len(), 1);
        let two: Shapes = vec![square(0.0, 0.0, 10.0), square(19.0, 0.0, 10.0)];
        assert_eq!(erode(&dilate(&two, 4.0), 4.0).len(), 2);
    }

    #[test]
    fn a_sharp_dilation_undoes_an_erosion_at_a_corner() {
        let sq: Shapes = vec![vec![vec![[0.0, 0.0], [30.0, 0.0], [30.0, 30.0], [0.0, 30.0]]]];
        let back = dilate_sharp(&erode(&sq, 1.0), 1.0);
        assert!((area(&back) - 900.0).abs() < 1e-6, "{}", area(&back));
        assert!(contains(&back, [0.01, 0.01]));
        // Arced, the corner is lost.
        let back = dilate(&erode(&sq, 1.0), 1.0);
        assert!(!contains(&back, [0.05, 0.05]));
    }

    #[test]
    fn the_index_agrees_with_contains() {
        // A bent road and a square with a hole: regions spanning many rows.
        let mut shapes = buffer_line(&[[-100.0, 0.0], [0.0, 30.0], [100.0, 0.0]], 5.5);
        shapes.push(vec![rect(-80.0, 40.0, 0.0, 120.0)[0].clone(), oriented(rect(-60.0, 60.0, -20.0, 100.0)[0].clone(), false)]);
        let index = Indexed::new(&shapes);
        assert!(!index.is_empty() && Indexed::new(&Vec::new()).is_empty());
        for p in [[-50.0, 15.0], [-50.0, 20.0], [0.0, 30.0], [0.0, 40.0], [200.0, 0.0], [-40.0, 50.0], [-40.0, 80.0], [-70.0, 80.0]] {
            assert_eq!(index.contains(p), contains(&shapes, p), "{p:?}");
        }
        let mut y = -10.0;
        while y < 130.0 {
            let mut x = -110.0;
            while x < 110.0 {
                assert_eq!(index.contains([x, y]), contains(&shapes, [x, y]), "{x} {y}");
                x += 3.7;
            }
            y += 2.9;
        }
    }

    #[test]
    fn a_convex_hull_is_counter_clockwise_and_minimal() {
        let hull = convex_hull(vec![[0.0, 0.0], [2.0, 0.0], [1.0, 0.5], [2.0, 2.0], [0.0, 2.0], [0.0, 0.0]]).unwrap();
        assert_eq!(hull, vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
        assert!(convex_hull(vec![[1.0, 1.0], [1.0, 1.0]]).is_none());
        assert!(convex_hull(vec![[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]]).is_none(), "a line has no area");
        assert_eq!(oriented(vec![[0.0, 0.0], [0.0, 1.0], [1.0, 0.0]], true), vec![[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]]);
        assert!((turn_deg([0.0, 0.0], [1.0, 0.0], [1.0, 1.0]) - 90.0).abs() < 1e-9);
        assert!((turn_deg([0.0, 0.0], [1.0, 0.0], [1.0, -1.0]) + 90.0).abs() < 1e-9);
        assert_eq!(bounds([[1.0, 2.0], [-1.0, 5.0]]), Some([-1.0, 2.0, 1.0, 5.0]));
        assert_eq!(bounds([]), None);
        assert!((segment_distance([0.0, 0.0], [2.0, 0.0], [3.0, 1.0]) - 2.0f64.sqrt()).abs() < 1e-12);
        assert_eq!(nearest_on_segment([0.0, 0.0], [2.0, 0.0], [1.0, 1.0]), [1.0, 0.0]);
    }

    #[test]
    fn a_hole_is_outside() {
        // A square with a square hole, by hand.
        let shapes: Shapes = vec![vec![
            vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            vec![[4.0, 4.0], [4.0, 6.0], [6.0, 6.0], [6.0, 4.0]],
        ]];
        assert!((area(&shapes) - 96.0).abs() < 1e-12);
        assert!(contains(&shapes, [1.0, 1.0]));
        assert!(!contains(&shapes, [5.0, 5.0]));
        assert!(!contains(&shapes, [11.0, 5.0]));
    }
}
