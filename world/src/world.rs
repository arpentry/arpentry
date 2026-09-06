//! The world: what the steps build, in local metres.

use arpentry_server::project::Bounds;

use crate::frame::{Frame, Rect};
use crate::grid::Grid;
use crate::poly::Shapes;
use crate::width::Family;

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
    pub ribbons: Option<Ribbons>,
    pub surface: Option<Surface>,
    pub kerb: Option<Kerb>,
    pub fillet: Option<Fillet>,
}

impl World {
    pub fn new(bbox: Bounds) -> World {
        let frame = Frame::centred(&bbox);
        let rect = frame.rect(&bbox);
        World {
            bbox,
            frame,
            rect,
            terrain: None,
            roads: None,
            ribbons: None,
            surface: None,
            kerb: None,
            fillet: None,
        }
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
    /// Overture's scalar `subclass`, or empty. `sidewalk` and `crosswalk` on
    /// a footway are the two that matter to the surface.
    pub subclass: String,
    /// The way's full width in metres ([`crate::width::of_way`]), decided
    /// once by the reader and read by every step after it.
    pub width_m: f64,
    pub pts: Vec<[f64; 2]>,
}

/// A polyline with heights, in local metres.
#[derive(Debug, Clone, PartialEq)]
pub struct Polyline3 {
    pub id: String,
    pub class: String,
    pub subclass: String,
    pub width_m: f64,
    pub pts: Vec<[f64; 3]>,
}

/// Way centrelines — roads and pedestrian ways — lying on the terrain.
#[derive(Debug, Clone, Default)]
pub struct Roads {
    /// The mapped lines, clipped to the rect, as the source drew them.
    pub plan: Vec<Polyline2>,
    /// The same lines draped exactly onto the terrain mesh.
    pub lines: Vec<Polyline3>,
}

/// One way's polygon: its centreline buffered to its width.
#[derive(Debug, Clone, PartialEq)]
pub struct Ribbon {
    pub id: String,
    pub class: String,
    pub subclass: String,
    pub family: Family,
    pub shape: Shapes,
}

/// Every way as a polygon, unmerged.
#[derive(Debug, Clone, Default)]
pub struct Ribbons {
    pub ribbons: Vec<Ribbon>,
}

/// The paved surface: one set of disjoint regions per family, the two
/// never overlapping — the asphalt has been subtracted from the walk.
#[derive(Debug, Clone, Default)]
pub struct Surface {
    pub carriageway: Shapes,
    pub walk: Shapes,
}

impl Surface {
    /// The regions of `family`.
    pub fn of(&self, family: Family) -> &Shapes {
        match family {
            Family::Carriageway => &self.carriageway,
            Family::Walk => &self.walk,
        }
    }
}

/// The pavement: the walk surface with the strip between every attached
/// sidewalk and its kerb filled, so its inner edge is the kerb.
#[derive(Debug, Clone, Default)]
pub struct Kerb {
    /// The ladder that filled the strips, unioned; kept for the plan view.
    pub rungs: Shapes,
    /// `(walk ∪ rungs) − carriageway`.
    pub pavement: Shapes,
    /// Every attached station, for the kerb-gap check downstream.
    pub attached: Vec<crate::kerb::Attached>,
}

/// The junctions rounded: the carriageway with its kerb returns, and the
/// pavement re-cut by them.
#[derive(Debug, Clone, Default)]
pub struct Fillet {
    pub junctions: Vec<crate::fillet::Junction>,
    /// What the closing added, unioned.
    pub fillets: Shapes,
    pub carriageway: Shapes,
    pub pavement: Shapes,
}

impl World {
    /// The latest walk surface: the pavement once the kerb step has run,
    /// re-cut once the fillet has, the plain union before either.
    pub fn walk(&self) -> Option<&Shapes> {
        self.fillet
            .as_ref()
            .map(|f| &f.pavement)
            .or_else(|| self.kerb.as_ref().map(|k| &k.pavement))
            .or_else(|| self.surface.as_ref().map(|s| &s.walk))
    }

    /// The latest carriageway: filleted once the fillet step has run.
    pub fn carriageway(&self) -> Option<&Shapes> {
        self.fillet.as_ref().map(|f| &f.carriageway).or_else(|| self.surface.as_ref().map(|s| &s.carriageway))
    }
}
