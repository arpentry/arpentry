//! The paved region of one tile and one level, as both meshers need to see it.
//!
//! The asphalt is meshed from a set of rings; the terrain is meshed *around*
//! the same rings (docs/GROUND.md §3, "the hole"). So two callers need the same
//! two answers about one ring set — is this point inside it, and what are the
//! rings as constraint edges — and they must agree exactly, because a
//! disagreement between them is a crack along every kerb in the map.
//!
//! The region carries no heights. The terrain's rim takes the *ground's* height
//! there and the difference is drawn as an explicit apron wall
//! (docs/GROUND.md §3), so nothing needs to ask the region what the road was
//! doing at a ring vertex.
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
/// coordinates, indexed for point-in-region queries.
pub struct Region {
    rings: Vec<Vec<(f64, f64)>>,
    rows: Vec<Vec<EdgeRef>>,
}

impl Region {
    pub fn new(rings: Vec<Vec<(f64, f64)>>) -> Region {
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
        Region { rings, rows }
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
}

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
        let region = Region::new(rings.clone());
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
        let region = Region::new(vec![ring.clone()]);
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
        let region = Region::new(rings);
        assert!(region.contains((22000.0, 22000.0)), "inside the outer ring");
        assert!(!region.contains((31000.0, 31000.0)), "inside the hole");
    }




}
