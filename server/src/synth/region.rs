//! The paved region of one tile and one level, as both meshers need to see it.
//!
//! The asphalt is meshed from a set of rings; the terrain is meshed *around*
//! the same rings (docs/GROUND.md §3, "the hole"). So two callers need the same
//! three answers about one ring set — is this point inside it, what are the
//! rings as constraint edges, and what height does the road carry on the
//! boundary — and they must agree exactly, because a disagreement between them
//! is a crack along every kerb in the map.
//!
//! **Why a row index.** The winding test itself is unchanged from the one
//! `pave_mesh` has always used, but its cost is not. The paved mesh pays it per
//! face and per candidate lattice point; the terrain pays it for ~33 k faces
//! plus ~16 k lattice vertices on a dense detail tile, against a ring set of
//! thousands of vertices. Linear scans there are the "minutes per tile" mistake
//! this module's neighbours already document. Every edge is bucketed by the
//! rows of quantized y it spans, and a query tests only the edges that could
//! straddle its own row.
//!
//! **Why winding and not even-odd.** Sutherland–Hodgman clipping against a tile
//! border joins a shape's pieces along the border rather than splitting them,
//! producing a single self-touching ring — and even-odd parity is ambiguous on a
//! ring that touches itself, so faces near such a border were accepted or
//! rejected essentially at random. Winding is well defined there, and the clip
//! preserves each ring's orientation (outer counter-clockwise, holes clockwise),
//! so a hole still subtracts.

use crate::project;

/// How many row buckets the quantized coordinate range is divided into. A
/// detail tile's lattice is 128 cells, so a bucket is a few cells tall: fine
/// enough that a query touches a handful of edges, coarse enough that a ring
/// edge spanning the tile lands in few buckets.
const ROWS: usize = 256;

/// The full quantized span of a tile including both buffer zones (FORMAT.md §5).
const SPAN: f64 = project::EXTENT + 2.0 * project::BUFFER;

/// One ring edge, as `(ring, first vertex)`. The second vertex is the next one
/// round, so an edge needs no more than this.
type EdgeRef = (u32, u32);

/// A tile's paved region at one level: rings in quantized tile-local
/// coordinates, optionally with the road-surface height the asphalt emitted at
/// each ring vertex.
pub struct Region {
    rings: Vec<Vec<(f64, f64)>>,
    /// Quantized z per ring vertex, parallel to `rings`. Empty when the region
    /// was built for containment alone.
    heights: Vec<Vec<i32>>,
    rows: Vec<Vec<EdgeRef>>,
}

impl Region {
    /// A region for containment queries only — no boundary heights.
    pub fn outline(rings: Vec<Vec<(f64, f64)>>) -> Region {
        Region::build(rings, Vec::new())
    }

    /// A region that also carries what the asphalt emitted at each ring vertex,
    /// so the terrain can land its boundary on exactly those heights.
    ///
    /// `heights` must be parallel to `rings`; a mismatched entry is treated as
    /// absent rather than panicking, so a caller that meshes one ring and not
    /// another still gets a usable region.
    pub fn with_heights(rings: Vec<Vec<(f64, f64)>>, heights: Vec<Vec<i32>>) -> Region {
        Region::build(rings, heights)
    }

    fn build(rings: Vec<Vec<(f64, f64)>>, heights: Vec<Vec<i32>>) -> Region {
        let mut rows: Vec<Vec<EdgeRef>> = vec![Vec::new(); ROWS];
        for (ri, ring) in rings.iter().enumerate() {
            let n = ring.len();
            if n < 3 {
                continue;
            }
            for k in 0..n {
                let (_, y0) = ring[k];
                let (_, y1) = ring[(k + 1) % n];
                let (lo, hi) = (row_of(y0.min(y1)), row_of(y0.max(y1)));
                for r in rows.iter_mut().take(hi + 1).skip(lo) {
                    r.push((ri as u32, k as u32));
                }
            }
        }
        Region { rings, heights, rows }
    }

    pub fn is_empty(&self) -> bool {
        self.rings.iter().all(|r| r.len() < 3)
    }

    /// The rings, for use as constraint edges.
    pub fn rings(&self) -> &[Vec<(f64, f64)>] {
        &self.rings
    }

    /// Whether `p` lies inside the region, by winding number.
    pub fn contains(&self, p: (f64, f64)) -> bool {
        let mut winding = 0i32;
        for &(ri, k) in &self.rows[row_of(p.1)] {
            let ring = &self.rings[ri as usize];
            let a = ring[k as usize];
            let b = ring[(k as usize + 1) % ring.len()];
            if a.1 <= p.1 {
                if b.1 > p.1 && cross(a, b, p) > 0.0 {
                    winding += 1; // upward crossing to the left of p
                }
            } else if b.1 <= p.1 && cross(a, b, p) < 0.0 {
                winding -= 1; // downward crossing to the right of p
            }
        }
        winding != 0
    }

    /// The road-surface height on the region's boundary at `p`, or `None` where
    /// `p` is not on the boundary (or the region carries no heights).
    ///
    /// Two cases, and the second is what makes the seam exact:
    ///
    /// - **On a ring vertex.** Quantized positions compare exactly, and the
    ///   terrain's triangulation inserts the very points the paved mesh emitted,
    ///   so this is the common case and it is a lookup.
    /// - **On a ring edge**, where a crest breakline crossing the kerb made the
    ///   triangulation split it. The height is the *linear interpolation* of the
    ///   edge's endpoints — not a re-evaluation of the height field. That is not
    ///   a convenience: along that edge the asphalt's own triangle is planar, so
    ///   interpolating puts the terrain vertex exactly on the asphalt's edge and
    ///   the crack is zero, whereas re-evaluating the (non-linear) field would
    ///   pull it off that straight edge and open one.
    pub fn height_on_boundary(&self, p: (f64, f64)) -> Option<i32> {
        if self.heights.is_empty() {
            return None;
        }
        // Deterministic: ring order then vertex order, first match wins. The
        // row bucket preserves insertion order, which is that order.
        for &(ri, k) in &self.rows[row_of(p.1)] {
            let ring = &self.rings[ri as usize];
            let Some(hs) = self.heights.get(ri as usize) else { continue };
            if hs.len() != ring.len() {
                continue;
            }
            let (i, j) = (k as usize, (k as usize + 1) % ring.len());
            let (a, b) = (ring[i], ring[j]);
            if p == a {
                return Some(hs[i]);
            }
            if p == b {
                return Some(hs[j]);
            }
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len2 = dx * dx + dy * dy;
            if len2 <= 0.0 {
                continue;
            }
            // Off the line by more than a quantization step is not on the edge.
            // A split vertex lands on the segment but is stored rounded, so the
            // tolerance is the rounding, not a fudge factor.
            let d = cross(a, b, p);
            if d * d > ON_EDGE_Q * ON_EDGE_Q * len2 {
                continue;
            }
            let t = ((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2;
            if !(0.0..=1.0).contains(&t) {
                continue;
            }
            return Some((hs[i] as f64 + (hs[j] - hs[i]) as f64 * t).round() as i32);
        }
        None
    }
}

/// How far off a ring edge, in quantized units, a vertex may sit and still count
/// as on it. One unit is about 1.8 cm on a z16 tile — the rounding the
/// triangulation applies to its own split vertices.
const ON_EDGE_Q: f64 = 1.5;

/// Which row bucket a quantized y falls in.
fn row_of(y: f64) -> usize {
    let f = (y / SPAN * ROWS as f64).floor();
    (f.max(0.0) as usize).min(ROWS - 1)
}

/// Cross product of `a → b` with `a → p`: positive when `p` is left of the edge.
pub fn cross(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> f64 {
    (b.0 - a.0) * (p.1 - a.1) - (p.0 - a.0) * (b.1 - a.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The unindexed winding test the row index must agree with.
    fn brute(p: (f64, f64), rings: &[Vec<(f64, f64)>]) -> bool {
        let mut w = 0i32;
        for q in rings {
            let n = q.len();
            if n < 3 {
                continue;
            }
            for k in 0..n {
                let (a, b) = (q[k], q[(k + 1) % n]);
                if a.1 <= p.1 {
                    if b.1 > p.1 && cross(a, b, p) > 0.0 {
                        w += 1;
                    }
                } else if b.1 <= p.1 && cross(a, b, p) < 0.0 {
                    w -= 1;
                }
            }
        }
        w != 0
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64, ccw: bool) -> Vec<(f64, f64)> {
        let r = vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
        if ccw {
            r
        } else {
            r.into_iter().rev().collect()
        }
    }

    #[test]
    fn the_row_index_agrees_with_a_brute_force_scan() {
        // An outer ring, a hole inside it, and a second disjoint outer ring —
        // spanning many row buckets so the index is actually exercised.
        let rings = vec![
            rect(20000.0, 20000.0, 46000.0, 46000.0, true),
            rect(28000.0, 28000.0, 34000.0, 34000.0, false),
            rect(50000.0, 8000.0, 60000.0, 60000.0, true),
        ];
        let region = Region::outline(rings.clone());
        // A deterministic lattice of probes, including points on the rings.
        let mut checked = 0;
        for i in 0..97 {
            for j in 0..89 {
                let p = (i as f64 * 677.0, j as f64 * 733.0);
                assert_eq!(
                    region.contains(p),
                    brute(p, &rings),
                    "disagreed at {p:?}"
                );
                checked += 1;
            }
        }
        assert!(checked > 8000);
    }

    #[test]
    fn a_self_touching_ring_is_read_by_winding() {
        // What a Sutherland–Hodgman clip leaves when a shape exits and re-enters
        // the tile: one ring that touches itself along the border. Even-odd is
        // ambiguous here; winding is not, and both lobes must read as inside.
        let ring = vec![
            (10000.0, 10000.0),
            (30000.0, 10000.0),
            (30000.0, 30000.0),
            (10000.0, 30000.0),
            (10000.0, 10000.0), // back to the start, then out again
            (30000.0, 5000.0),
            (40000.0, 5000.0),
            (40000.0, 8000.0),
            (10000.0, 8000.0),
        ];
        let region = Region::outline(vec![ring.clone()]);
        for p in [(20000.0, 20000.0), (35000.0, 6500.0)] {
            assert!(region.contains(p), "{p:?} must read inside");
            assert_eq!(region.contains(p), brute(p, &[ring.clone()]));
        }
        assert!(!region.contains((50000.0, 50000.0)));
    }

    #[test]
    fn a_hole_subtracts() {
        let rings = vec![
            rect(20000.0, 20000.0, 46000.0, 46000.0, true),
            rect(28000.0, 28000.0, 34000.0, 34000.0, false),
        ];
        let region = Region::outline(rings);
        assert!(region.contains((22000.0, 22000.0)), "inside the outer ring");
        assert!(!region.contains((31000.0, 31000.0)), "inside the hole");
    }

    #[test]
    fn a_ring_vertex_returns_its_own_height() {
        let ring = rect(20000.0, 20000.0, 40000.0, 40000.0, true);
        let heights = vec![1000, 2000, 3000, 4000];
        let region = Region::with_heights(vec![ring.clone()], vec![heights.clone()]);
        for (k, v) in ring.iter().enumerate() {
            assert_eq!(region.height_on_boundary(*v), Some(heights[k]), "at vertex {k}");
        }
    }

    #[test]
    fn a_split_vertex_takes_the_edges_own_line() {
        // The T-junction case: a vertex the triangulation created part-way along
        // a ring edge must land on the asphalt's straight edge, which is the
        // linear interpolation of the endpoints — not whatever the height field
        // says there.
        let ring = vec![(0.0, 10000.0), (10000.0, 10000.0), (10000.0, 20000.0), (0.0, 20000.0)];
        let heights = vec![1000, 3000, 3000, 1000];
        let region = Region::with_heights(vec![ring], vec![heights]);
        assert_eq!(region.height_on_boundary((5000.0, 10000.0)), Some(2000));
        assert_eq!(region.height_on_boundary((2500.0, 10000.0)), Some(1500));
        // A point off the boundary is not on it, however close the ring passes.
        assert_eq!(region.height_on_boundary((5000.0, 15000.0)), None);
    }

    #[test]
    fn a_vertex_a_rounding_off_the_edge_still_counts() {
        // Split vertices are stored rounded, so "on the edge" has to tolerate
        // the rounding — but only the rounding.
        let ring = vec![(0.0, 10000.0), (10000.0, 10000.0), (10000.0, 20000.0), (0.0, 20000.0)];
        let heights = vec![1000, 3000, 3000, 1000];
        let region = Region::with_heights(vec![ring], vec![heights]);
        assert_eq!(region.height_on_boundary((5000.0, 10001.0)), Some(2000));
        assert_eq!(region.height_on_boundary((5000.0, 10004.0)), None);
    }

    #[test]
    fn a_region_without_heights_answers_no_boundary_height() {
        let region = Region::outline(vec![rect(0.0, 0.0, 10000.0, 10000.0, true)]);
        assert_eq!(region.height_on_boundary((0.0, 0.0)), None);
    }
}
