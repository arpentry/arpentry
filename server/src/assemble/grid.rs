//! A uniform grid spatial index over lon/lat bounding boxes.
//!
//! The workhorse for the scene-graph's geometric queries: crossing detection
//! (bridge-span edges vs. every road edge) and the engineered ground's
//! modifier lookup (terrain vertex vs. earthwork corridor edges). Items are
//! `u32` handles into a caller-owned array; the grid stores which cells each
//! item's bbox touches. Hand-rolled (~no dependencies): the datasets are
//! bounded (a region's structure edges), cells are sized to the query radius,
//! and a `HashMap` of small vectors is plenty.

use std::collections::HashMap;

/// Grid cell size in metres. Sized to the queries' reach: an earthwork's
/// half-width + feather and a crossing test's edge lengths are all well under
/// this, so a query touches at most a couple of cells per axis.
pub const CELL_M: f64 = 128.0;

pub struct GridIndex {
    /// Cell size in degrees (latitude metres; longitude cells are narrower on
    /// the ground, which only makes queries slightly conservative).
    cell_deg: f64,
    cells: HashMap<(i32, i32), Vec<u32>>,
}

impl GridIndex {
    pub fn new() -> GridIndex {
        GridIndex::with_cell_m(CELL_M)
    }

    /// A grid whose cells are `cell_m` across, for a population denser than
    /// [`CELL_M`] was sized for. Cells are sized to the *query* radius, and a
    /// facade lookup reaches a carriageway's half-width rather than an
    /// earthwork's — with 128 m cells a town centre puts a hundred-odd
    /// footprint edges in every cell a 4 m query touches.
    pub fn with_cell_m(cell_m: f64) -> GridIndex {
        GridIndex { cell_deg: cell_m / crate::scene::DEG_M, cells: HashMap::new() }
    }

    fn cell_of(&self, x: f64, y: f64) -> (i32, i32) {
        ((x / self.cell_deg).floor() as i32, (y / self.cell_deg).floor() as i32)
    }

    /// Inserts an item covering the bbox `(west, south, east, north)`.
    pub fn insert(&mut self, bbox: (f64, f64, f64, f64), id: u32) {
        let (c0x, c0y) = self.cell_of(bbox.0, bbox.1);
        let (c1x, c1y) = self.cell_of(bbox.2, bbox.3);
        for cy in c0y..=c1y {
            for cx in c0x..=c1x {
                self.cells.entry((cx, cy)).or_default().push(id);
            }
        }
    }

    /// Collects the (deduplicated, sorted) item ids whose cells intersect the
    /// bbox into `out`.
    pub fn query(&self, bbox: (f64, f64, f64, f64), out: &mut Vec<u32>) {
        out.clear();
        let (c0x, c0y) = self.cell_of(bbox.0, bbox.1);
        let (c1x, c1y) = self.cell_of(bbox.2, bbox.3);
        for cy in c0y..=c1y {
            for cx in c0x..=c1x {
                if let Some(ids) = self.cells.get(&(cx, cy)) {
                    out.extend_from_slice(ids);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

impl Default for GridIndex {
    fn default() -> Self {
        GridIndex::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_items_in_and_near_their_cells() {
        let mut g = GridIndex::new();
        // Two items ~500 m apart at the equator (0.0045° ≈ 500 m).
        g.insert((6.0, 46.0, 6.0001, 46.0001), 1);
        g.insert((6.0045, 46.0, 6.0046, 46.0001), 2);

        let mut out = Vec::new();
        g.query((5.9999, 45.9999, 6.0002, 46.0002), &mut out);
        assert_eq!(out, vec![1]);
        g.query((6.0, 46.0, 6.005, 46.001), &mut out);
        assert_eq!(out, vec![1, 2]);
        g.query((6.1, 46.1, 6.1001, 46.1001), &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn spanning_items_land_in_every_cell()
    {
        let mut g = GridIndex::new();
        // An item spanning ~1 km of longitude touches several cells; a query
        // anywhere along it finds it exactly once.
        g.insert((6.0, 46.0, 6.013, 46.0001), 7);
        let mut out = Vec::new();
        g.query((6.006, 46.0, 6.0061, 46.0001), &mut out);
        assert_eq!(out, vec![7]);
    }
}
