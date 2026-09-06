//! The world: what the steps build, in local metres.

use arpentry_server::project::Bounds;

use crate::frame::{Frame, Rect};
use crate::grid::Grid;

/// The world for one bounding box. Every layer is `None` until its step has
/// run; a step reads the layers before it and fills its own.
#[derive(Debug)]
pub struct World {
    /// The bounding box the world was asked for, in degrees. Never inferred
    /// from the data: a cut zone holds the zone plus a margin.
    pub bbox: Bounds,
    /// The local metric frame, centred on the bbox.
    pub frame: Frame,
    /// The bbox in the local frame.
    pub rect: Rect,
    pub terrain: Option<Terrain>,
    pub roads: Option<Roads>,
}

impl World {
    pub fn new(bbox: Bounds) -> World {
        let frame = Frame::centred(&bbox);
        let rect = frame.rect(&bbox);
        World { bbox, frame, rect, terrain: None, roads: None }
    }
}

/// The terrain mesh: a regular lattice over the bbox with a height per vertex.
///
/// Positions are derived from the grid and `z`, never stored twice, so the
/// mesh a viewer draws and the surface [`crate::terrain::height_at`] evaluates
/// read one array.
#[derive(Debug, Clone)]
pub struct Terrain {
    pub grid: Grid,
    /// Height per lattice vertex, indexed by [`Grid::index`].
    pub z: Vec<f64>,
    /// Unit normal per vertex, z up.
    pub normals: Vec<[f32; 3]>,
    /// Triangle list, counter-clockwise seen from above.
    pub indices: Vec<u32>,
    pub zmin: f64,
    pub zmax: f64,
}

impl Terrain {
    /// The position of vertex `i`.
    pub fn position(&self, i: usize) -> [f64; 3] {
        let (c, r) = self.grid.vertex_of(i);
        let [x, y] = self.grid.vertex(c, r);
        [x, y, self.z[i]]
    }
}

/// A plan-space polyline, in local metres.
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline2 {
    pub id: String,
    pub class: String,
    pub pts: Vec<[f64; 2]>,
}

/// A polyline with heights, in local metres.
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline3 {
    pub id: String,
    pub class: String,
    pub pts: Vec<[f64; 3]>,
}

/// Road centrelines lying on the terrain.
#[derive(Debug, Clone, Default)]
pub struct Roads {
    pub lines: Vec<Polyline3>,
}
