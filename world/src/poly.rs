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
use i_overlay::float::string_overlay::FloatStringOverlay;
use i_overlay::string::rule::StringRule;
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

/// `outer` cut into faces by `cuts`: **one planar subdivision**, in one pass.
///
/// Every returned face is a region of `outer`, the faces are disjoint, and
/// together they are `outer` — but the point of the operation is what they
/// share rather than what they cover. A boolean expression computes each face
/// separately and snaps *each* result to the lattice, so the same conceptual
/// edge computed two ways lands on two sets of points up to half a grid
/// apart: that is the whole of `bench`'s `seam` and `unmet`, and it is why a
/// cross-mesh lookup has to ask the eight cells around a key
/// (`data/plans/one-ground-2026-09-16.md` §3.1, §4.2). Sliced instead, every
/// face's boundary is built from **one** set of split points, so a vertex on
/// a shared edge is the same `Pt` — bit for bit — in both faces that carry
/// it.
///
/// `cuts` are open or closed lines, not regions: a ring passed here is a
/// *cut*, and which side of it is which is not asked. Tagging the faces is
/// the caller's, and is a point-in-region test per face rather than another
/// boolean.
///
/// The scale is the pinned one [`overlay`] uses, so a slice and a boolean
/// over the same world land on the same lattice.
pub fn slice(outer: &Shapes, cuts: &[Ring]) -> Shapes {
    if outer.is_empty() {
        return Vec::new();
    }
    if cuts.is_empty() {
        return outer.clone();
    }
    // **On the pinned lattice, like every other boolean here.**
    // `slice_by_fixed_scale` takes a scale but builds its adapter from the
    // *input's own bounds*, so its origin moves with the data and its output
    // floats land between [`overlay`]'s. Measured on `house:across`: the
    // union of faces cut that way invented **all 27** of its vertices —
    // not one of them was a face vertex — because every one of them was off
    // this lattice. Built through the same adapter, a sliced vertex and an
    // overlaid one are the same `f64`.
    let rect = FloatRect::new(-PIN_M, PIN_M, -PIN_M, PIN_M);
    let adapter = FloatPointAdapter::<Pt, i64>::with_scale(rect, SCALE);
    let cap = outer.iter().flatten().map(Vec::len).sum::<usize>()
        + cuts.iter().map(Vec::len).sum::<usize>();
    FloatStringOverlay::<Pt, i64>::with_adapter(adapter, cap)
        .unsafe_add_shapes(outer)
        .unsafe_add_string_lines(&cuts.to_vec())
        .build_graph_view(FillRule::NonZero)
        .map(|graph| graph.extract_shapes(StringRule::Slice))
        .unwrap_or_default()
}

/// `shapes` with every ring subdivided at every vertex of `shapes` that lies
/// on it: one **edge-consistent** subdivision.
///
/// A slice gives faces that share their *split points*, which is enough for a
/// mesh to weld — but not enough for a rule about edges. Where a boundary is
/// cut on one side and not on the other the two disagree segment for segment:
/// on `net:level` the carriageway's edge along `y = 2.75` is one 200 m
/// segment, while the ground's side of the same line is three, because the
/// railway crosses there and cuts the ground but not the road. That is a real
/// T-junction, and `data/plans/one-ground-2026-09-16.md` §3.3's rule — an
/// edge is welded or split, and the mesher emits the quad — cannot be written
/// over one.
///
/// After this every segment away from the outer border is carried by exactly
/// two rings, which is what `dangling` measures.
///
/// A vertex counts as lying on a segment when it is within [`GRID_M`] of it
/// and is not one of its ends — the points are all on one lattice already, so
/// the test is about *which* lattice point, not about a tolerance.
pub fn conform(shapes: &Shapes) -> Shapes {
    let mut index: HashMap<(i32, i32), Vec<Pt>> = HashMap::new();
    for p in shapes.iter().flatten().flatten() {
        index.entry(cell_of(*p, CELL_M)).or_default().push(*p);
    }
    let same = |a: Pt, b: Pt| (a[0] - b[0]).abs() < GRID_M && (a[1] - b[1]).abs() < GRID_M;
    shapes
        .iter()
        .map(|shape| {
            shape
                .iter()
                .map(|ring| {
                    let mut out: Ring = Vec::with_capacity(ring.len());
                    for k in 0..ring.len() {
                        let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                        out.push(a);
                        let len = (b[0] - a[0]).hypot(b[1] - a[1]);
                        if !(len > 0.0) {
                            continue;
                        }
                        let bbox = [
                            a[0].min(b[0]) - GRID_M,
                            a[1].min(b[1]) - GRID_M,
                            a[0].max(b[0]) + GRID_M,
                            a[1].max(b[1]) + GRID_M,
                        ];
                        let mut on: Vec<(f64, Pt)> = Vec::new();
                        for cell in cells_over(bbox, CELL_M) {
                            for &p in index.get(&cell).into_iter().flatten() {
                                if same(p, a) || same(p, b) || segment_distance(a, b, p) > GRID_M {
                                    continue;
                                }
                                let s = ((p[0] - a[0]) * (b[0] - a[0])
                                    + (p[1] - a[1]) * (b[1] - a[1]))
                                    / (len * len);
                                on.push((s, p));
                            }
                        }
                        on.sort_by(|x, y| x.0.total_cmp(&y.0));
                        on.dedup_by(|x, y| same(x.1, y.1));
                        out.extend(on.into_iter().map(|(_, p)| p));
                    }
                    out
                })
                .collect()
        })
        .collect()
}

/// Every ring of `shapes`, outer boundaries and holes alike, as cut lines for
/// [`slice`].
///
/// **Closed explicitly.** A [`Ring`] does not repeat its first point, and a
/// cut is a *string* — an open polyline. Passed as it is stored, every ring
/// loses the edge from its last vertex back to its first, so the cut does not
/// close and the region it was meant to separate stays joined to its
/// neighbour. Measured on `net:roundabout`: 6 rings of 376 vertices cut the
/// rect into 3 faces with the carriageway missing entirely, against 8 faces
/// with the rings closed.
pub fn rings(shapes: &Shapes) -> Vec<Ring> {
    shapes
        .iter()
        .flatten()
        .filter(|r| r.len() >= 3)
        .map(|r| {
            let mut closed = r.clone();
            closed.push(r[0]);
            closed
        })
        .collect()
}

/// A point strictly inside `shape`, for asking a face what material it is.
///
/// Steps inward from the midpoint of each edge of the outer ring — the
/// interior of a counter-clockwise ring is to the left of every directed edge
/// — and takes the point that stands furthest from a boundary. **Tested
/// against the whole shape**, holes included, so a probe never lands in a
/// courtyard and reports the material of the thing around it.
///
/// A diagonal between a vertex and the one two along is the textbook answer
/// and is wrong here: it is inside any *simple* polygon, and a face with
/// holes is not one. The rect's own face is the case — four corners, every
/// ear midpoint the rect's centre, and the centre is in the roundabout. It
/// returned `None` for the 1.7 km² ground face of `net:roundabout`.
///
/// `None` for a degenerate ring, and for a face thinner than
/// [`PROBE_MIN_M`] — which has no inside this can name, and which a caller
/// must not tag by guessing.
pub fn inside(shape: &Shape) -> Option<Pt> {
    let ring = shape.first()?;
    let n = ring.len();
    if n < 3 {
        return None;
    }
    let one = vec![shape.clone()];
    // **Strictly inside, and provably so.** `contains` on a point that lies
    // *on* a boundary may answer either way, and the answer it gives is the
    // one that matters here: the tag. So a candidate is accepted only if it
    // and its four neighbours a [`PROBE_MIN_M`] away are all inside, which
    // puts it at least that far from any edge.
    //
    // Without it the preference for the largest step picks the ambiguous
    // point: on a band a metre wide, stepping 1.0 m in from one long edge
    // lands exactly on the other, `contains` said yes, and the probe for the
    // *far* half of a sidewalk sat on the line dividing it from the near
    // half. Every face came back tagged near, `walk_far_m2` read 0, and the
    // bench lifted a band it should have draped.
    let strict = |q: Pt| {
        contains(&one, q)
            && [[PROBE_MIN_M, 0.0], [-PROBE_MIN_M, 0.0], [0.0, PROBE_MIN_M], [0.0, -PROBE_MIN_M]]
                .iter()
                .all(|d| contains(&one, [q[0] + d[0], q[1] + d[1]]))
    };
    let mut best: Option<(f64, Pt)> = None;
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if !(len > 0.0) {
            continue;
        }
        let inward = [-dy / len, dx / len];
        let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        // Furthest first: a probe well clear of the boundary is one the
        // lattice cannot make ambiguous. Half the edge's own length is as
        // far as a triangle could ever allow.
        for step in [0.5 * len, 1.0, 0.1, 0.01, 2.0 * PROBE_MIN_M] {
            if step < 2.0 * PROBE_MIN_M {
                continue;
            }
            let q = [m[0] + inward[0] * step, m[1] + inward[1] * step];
            if strict(q) {
                if best.is_none_or(|(w, _)| step > w) {
                    best = Some((step, q));
                }
                break;
            }
        }
    }
    best.map(|(_, q)| q)
}

/// How far inside a face [`inside`] must reach before it will name a point,
/// in metres: ten lattice steps, a millimetre.
///
/// Under this the answer is not wrong so much as unasked — `contains` at the
/// lattice cannot separate just-inside from just-outside — and a face this
/// thin carries no area worth tagging.
pub const PROBE_MIN_M: f64 = 10.0 * GRID_M;

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
///
/// Taken by slice so the area of a single region is
/// `area(std::slice::from_ref(&shape))` rather than a `vec![shape.clone()]`
/// that deep-copies every ring to ask.
pub fn area(shapes: &[Shape]) -> f64 {
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
        self.which(p).is_some()
    }

    /// The same, answering *which* region holds `p` — the lowest index of
    /// them, though the regions of a `Shapes` are disjoint so there is at
    /// most one. A caller that has to attribute a point to the region it
    /// fell in wants this; one that only asks whether it fell in any wants
    /// [`Indexed::contains`].
    pub fn which(&self, p: Pt) -> Option<usize> {
        let (c, r) = cell_of(p, CELL_M);
        self.cells.get(&(c, r))?.iter().copied().find(|&i| {
            self.edges.get(&(i, r)).is_some_and(|es| es.iter().filter(|&&(a, b)| crosses(a, b, p)).count() % 2 == 1)
        })
    }

    /// Every region holding `p`, lowest index first: for regions that are
    /// *not* disjoint — the paving of two sheets, one flying over the other
    /// — where [`Indexed::which`] would name only the first.
    pub fn all(&self, p: Pt) -> Vec<usize> {
        let (c, r) = cell_of(p, CELL_M);
        let Some(cands) = self.cells.get(&(c, r)) else {
            return Vec::new();
        };
        cands
            .iter()
            .copied()
            .filter(|&i| {
                self.edges.get(&(i, r)).is_some_and(|es| es.iter().filter(|&&(a, b)| crosses(a, b, p)).count() % 2 == 1)
            })
            .collect()
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

    /// Every vertex of `shapes`, as the exact bits it is stored as.
    fn keys(shapes: &Shapes) -> std::collections::HashSet<(u64, u64)> {
        shapes.iter().flatten().flatten().map(|p| (p[0].to_bits(), p[1].to_bits())).collect()
    }

    /// The specimen: a square, and two crossing strokes at angles that put
    /// every intersection off the lattice.
    fn cut_square() -> (Shapes, Shapes, Shapes) {
        let square = vec![rect(-50.0, -50.0, 50.0, 50.0)];
        let a = buffer_line(&[[-60.0, -13.0], [60.0, 17.0]], 7.3);
        let b = buffer_line(&[[-11.0, -60.0], [19.0, 60.0]], 5.1);
        (square, a, b)
    }

    /// Vertices of `ground` that lie on no vertex of `paved` and are not on
    /// the square's own edge: the ones a cross-mesh lookup cannot weld, which
    /// is what `bench`'s `seam` and `unmet` count.
    fn orphans(ground: &Shapes, paved: &Shapes) -> usize {
        let pk = keys(paved);
        ground
            .iter()
            .flatten()
            .flatten()
            .filter(|p| p[0].abs() < 50.0 - 1e-9 && p[1].abs() < 50.0 - 1e-9)
            .filter(|p| !pk.contains(&(p[0].to_bits(), p[1].to_bits())))
            .count()
    }

    /// **An extra boolean over the same operands moves nothing**, and that is
    /// worth knowing, because it is not what CLAUDE.md and
    /// `data/plans/one-ground-2026-09-16.md` say the defect is.
    ///
    /// The stated mechanism is that "a point that has been through one more
    /// boolean than its neighbour lands up to half a grid away". It cannot:
    /// [`overlay`] pins the adapter, so every output point is an exact
    /// multiple of [`GRID_M`], and feeding one back in maps to the same
    /// integer. **Snapping is idempotent, and re-rounding is the identity.**
    #[test]
    fn a_boolean_over_a_snapped_operand_is_idempotent() {
        let (square, a, b) = cut_square();
        let once = difference(&square, &union_of(&[&a, &b]));
        let twice = difference(&square, &union_of(&[&union_all(&a), &union_all(&b)]));
        assert_eq!(keys(&once), keys(&twice), "a second snap moved a point");
        assert_eq!(orphans(&once, &twice), 0);
    }

    /// **What does make two boundaries is two different regions**, which is
    /// what the chain actually builds.
    ///
    /// The mesher is handed the paved pieces *separately* — `mesh::by_sheet`
    /// meshes one sheet at a time — while the hole is cut from their
    /// **union** (`bench`: `union_all(sheets.shapes()) − spanned`). Where two
    /// pieces touch, the union dissolves the edge between them and puts
    /// vertices where they crossed; the separately meshed pieces keep that
    /// edge and have no such vertices. The two are not one boundary rounded
    /// twice. They are two boundaries, and no amount of care with the lattice
    /// reconciles them.
    #[test]
    fn a_union_dissolves_the_edge_the_mesher_kept() {
        let (square, a, b) = cut_square();
        // As the mesher gets it: two overlapping pieces, each on its own.
        let pieces: Shapes = a.iter().chain(b.iter()).cloned().collect();
        // As the hole gets it: their union.
        let ground = difference(&square, &union_of(&[&a, &b]));

        let orphaned = orphans(&ground, &pieces);
        assert!(
            orphaned > 0,
            "the specimen no longer states the defect: the union and the pieces \
             already agree, so the slice below is not being asked anything"
        );
        println!("union vs pieces: {orphaned} ground vertices weld to nothing");
    }

    /// **One slice gives one set of split points.**
    ///
    /// The property §3.1 needs, and the answer to the test above: cut the
    /// square by the paving's own rings and the ground's boundary *is* those
    /// rings — every vertex of it, away from the square's edge, is a vertex
    /// of the paved faces, bit for bit. `seam` and `unmet` then have no
    /// subject rather than a smaller value.
    #[test]
    fn one_slice_gives_one_set_of_split_points() {
        let (square, a, b) = cut_square();
        let paving = union_of(&[&a, &b]);

        let faces = slice(&square, &rings(&paving));
        assert!(faces.len() >= 2, "the cuts divide the square: {}", faces.len());
        // A partition: the faces cover the square exactly and do not overlap.
        let total: f64 = faces.iter().map(|f| area(std::slice::from_ref(f))).sum();
        assert!((total - 100.0 * 100.0).abs() < 1e-6, "{total}");

        // Tagged by asking one interior point of each face — no boolean.
        let (mut paved, mut ground): (Shapes, Shapes) = (Vec::new(), Vec::new());
        for f in faces {
            let probe = inside(&f).expect("a face has an inside");
            if contains(&paving, probe) { paved.push(f) } else { ground.push(f) }
        }
        assert!(!paved.is_empty() && !ground.is_empty(), "both materials are there");
        // The areas are the two sides of the same boundary.
        let want = area(&intersect(&square, &paving));
        assert!((area(&paved) - want).abs() < 1e-6, "{} vs {want}", area(&paved));

        assert_eq!(orphans(&ground, &paved), 0, "every shared vertex is one vertex");
    }

    /// **A T-junction is a segment cut on one side and not the other**, and
    /// [`conform`] removes it.
    ///
    /// The case `net:level` states: one face's edge runs the length of a
    /// road, while the faces on the other side of the same line are three,
    /// because a railway crosses there and cuts one side only.
    #[test]
    fn conform_gives_both_sides_of_an_edge_the_same_vertices() {
        // A long face below the line y = 0, and three short ones above it,
        // meeting it end to end.
        let long: Shape = vec![vec![[0.0, -1.0], [9.0, -1.0], [9.0, 0.0], [0.0, 0.0]]];
        let short = |x0: f64, x1: f64| -> Shape {
            vec![vec![[x0, 0.0], [x1, 0.0], [x1, 1.0], [x0, 1.0]]]
        };
        let faces: Shapes = vec![long, short(0.0, 3.0), short(3.0, 6.0), short(6.0, 9.0)];

        let segs = |s: &Shapes| -> std::collections::HashMap<((u64, u64), (u64, u64)), usize> {
            let mut out = std::collections::HashMap::new();
            for ring in s.iter().flatten() {
                for k in 0..ring.len() {
                    let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                    let (ka, kb) = ((a[0].to_bits(), a[1].to_bits()), (b[0].to_bits(), b[1].to_bits()));
                    *out.entry(if ka <= kb { (ka, kb) } else { (kb, ka) }).or_default() += 1;
                }
            }
            out
        };
        // The property is about the shared line: how many segments lying on
        // y = 0 are carried by one face alone.
        let on_line = |s: &Shapes| -> usize {
            segs(s)
                .iter()
                .filter(|(_, n)| **n == 1)
                .filter(|((a, b), _)| {
                    f64::from_bits(a.1) == 0.0 && f64::from_bits(b.1) == 0.0
                })
                .count()
        };
        // Before: the long face's 9 m edge and the three short faces' 3 m
        // edges are four different segments, and no two of them match.
        assert_eq!(on_line(&faces), 4, "the shared line is four unmatched segments");

        let done = conform(&faces);
        // After: the long edge is split at x = 3 and x = 6, so each piece is
        // carried by both the face below it and the face above.
        assert_eq!(on_line(&done), 0, "the shared line is shared segment for segment");
        // Nothing moved and nothing was lost.
        assert!((area(&done) - area(&faces)).abs() < 1e-12);
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
