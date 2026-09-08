//! The world: what the steps build, in local metres.

use arpentry_server::project::Bounds;

use crate::frame::{Frame, Rect};
use crate::grid::Grid;
use crate::poly::{self, Shapes};
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
    pub profile: Option<Profiles>,
    pub crossing: Option<Crossings>,
    pub facade: Option<Facade>,
    pub ribbons: Option<Ribbons>,
    pub surface: Option<Surface>,
    pub kerb: Option<Kerb>,
    pub fillet: Option<Fillet>,
    pub room: Option<Room>,
    pub mesh: Option<Mesh>,
    pub bench: Option<Bench>,
    pub structure: Option<Structure>,
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
            profile: None,
            crossing: None,
            facade: None,
            ribbons: None,
            surface: None,
            kerb: None,
            fillet: None,
            room: None,
            mesh: None,
            bench: None,
            structure: None,
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

/// Where a piece of a way stands against the ground, as the source mapped
/// it: Overture's `level_rules` (an ordinal: above, below), the
/// `is_bridge`/`is_tunnel` flags, and `is_indoor`. A prior on the profile,
/// never a command to build anything (docs/GENERATION.md §4.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    /// On the ground: what the surface is built from.
    Ground,
    /// Above it, at this level ordinal (positive).
    Bridge(i64),
    /// Below it, at this level ordinal (negative).
    Tunnel(i64),
    /// Inside a building.
    Indoor,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Ground => "ground",
            Kind::Bridge(_) => "bridge",
            Kind::Tunnel(_) => "tunnel",
            Kind::Indoor => "indoor",
        }
    }

    /// A bridge or a tunnel: a piece the profile chords across.
    pub fn is_structure(self) -> bool {
        matches!(self, Kind::Bridge(_) | Kind::Tunnel(_))
    }
}

/// A plan-space polyline, in local metres: one piece of a way, all of one
/// [`Kind`]. Overture references a way's bridge, tunnel and indoor spans as
/// fractions of its length, so the reader cuts every way at every span
/// boundary; the pieces of one way share its `id` and their end vertices.
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
    pub kind: Kind,
    pub pts: Vec<[f64; 2]>,
}

/// Way ends closer than this, in metres, meet at one connector. The frame
/// maps a shared source coordinate to one local point exactly; the slack
/// is for hand-made specimens.
const SNAP_M: f64 = 0.01;

/// The connector at `p`: the key two way vertices share when they meet.
pub fn connector(p: [f64; 2]) -> (i64, i64) {
    ((p[0] / SNAP_M).round() as i64, (p[1] / SNAP_M).round() as i64)
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

/// Way centrelines — roads and pedestrian ways — as the source drew them,
/// cut into pieces by kind.
#[derive(Debug, Clone, Default)]
pub struct Roads {
    /// The pieces on the ground, clipped to the rect: what every surface
    /// step builds from. Nothing in it crosses anything else at another
    /// level, so the steps after this one may union freely.
    pub plan: Vec<Polyline2>,
    /// The pieces above or below the ground, or indoors, clipped likewise.
    /// No surface step reads them; the profile chords across the bridges
    /// and tunnels, and the structures are built from what it solves.
    pub spans: Vec<Polyline2>,
    /// The ground pieces draped exactly onto the terrain mesh.
    pub lines: Vec<Polyline3>,
}

impl Roads {
    /// Every piece, on the ground or off it.
    pub fn pieces(&self) -> impl Iterator<Item = &Polyline2> {
        self.plan.iter().chain(self.spans.iter())
    }
}

/// One station of a solved profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Station {
    /// Arc length from the piece's start, in metres.
    pub s: f64,
    /// The plan position on the axis.
    pub p: [f64; 2],
    /// The ground there ([`crate::terrain::height_at`]).
    pub ground: f64,
    /// The solved height of the surface.
    pub h: f64,
    /// What the solved height makes of the station.
    pub solved: Solved,
}

/// What a station is once solved: a consequence of the profile against the
/// ground, never of the annotation alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Solved {
    /// On the ground, within the bench's reach.
    Grade,
    /// Standing off the ground: a deck.
    Deck,
    /// Running under it: a bore.
    Bore,
}

/// The solved profile of one carriageway piece.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    pub id: String,
    pub class: String,
    pub width_m: f64,
    /// The kind the source mapped the piece as.
    pub mapped: Kind,
    pub stations: Vec<Station>,
}

impl Profile {
    /// The profile as a 3D polyline, for the layers that draw it.
    pub fn line(&self) -> Vec<[f64; 3]> {
        self.stations.iter().map(|st| [st.p[0], st.p[1], st.h]).collect()
    }
}

/// Every carriageway piece's profile: one height along every axis.
#[derive(Debug, Clone, Default)]
pub struct Profiles {
    pub profiles: Vec<Profile>,
}

/// One place two carriageway axes cross in plan with no connector between
/// them: a grade separation, and the only thing in the model that couples
/// the height of one way to the height of another.
#[derive(Debug, Clone, PartialEq)]
pub struct Crossing {
    /// Where the two axes cross, in local metres.
    pub at: [f64; 2],
    /// The piece above, by the level ordinals, and the piece below: an
    /// index into [`Profiles::profiles`], and the level the source mapped.
    pub upper: (usize, i64),
    pub lower: (usize, i64),
    /// The separation the pair needs, in metres, between the two roadways.
    pub need: f64,
    /// What they had before the floor, and what they have after it.
    pub had: f64,
    pub have: f64,
}

impl Crossing {
    /// What the crossing is short of, in metres, after the solve; zero or
    /// less is met.
    pub fn shortfall(&self) -> f64 {
        self.need - self.have
    }
}

/// Every crossing the network has, and what the floor spent on them.
#[derive(Debug, Clone, Default)]
pub struct Crossings {
    pub crossings: Vec<Crossing>,
    /// Two axes crossing at the same level with no connector between them:
    /// a data error, counted and not solved.
    pub same: Vec<[f64; 2]>,
    /// The floor, in metres, at every station of every profile: what the
    /// crossings asked the ground to become. Indexed as the profiles.
    pub floor: Vec<Vec<f64>>,
}

/// The buildings: what nothing paved may enter.
#[derive(Debug, Clone, Default)]
pub struct Facade {
    /// Every footprint touching the rect, clipped to it and unioned: two
    /// houses sharing a wall are one region, a courtyard is a hole.
    pub footprints: Shapes,
    /// Where a way's axis runs inside a footprint, the corridor the
    /// building yields to it ([`crate::facade::PASSAGE_M`]).
    pub passages: Shapes,
    /// `footprints − passages`: what nothing paved enters.
    pub solid: Shapes,
    /// The solid with the pockets — the notches and the gaps between
    /// houses the closing filled, less the lanes ways run down — which the
    /// asphalt keeps out of and the pavement may fill.
    pub built: Shapes,
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

/// The corners rounded: the carriageway with its kerb returns, and the
/// pavement re-cut by them.
#[derive(Debug, Clone, Default)]
pub struct Fillet {
    pub corners: Vec<crate::fillet::Corner>,
    /// What the closing added, unioned.
    pub fillets: Shapes,
    pub carriageway: Shapes,
    pub pavement: Shapes,
}

/// The room filled: the pavement extended to every wall within reach.
#[derive(Debug, Clone, Default)]
pub struct Room {
    /// The bands and rungs, unioned; kept for the plan view.
    pub room: Shapes,
    pub pavement: Shapes,
    /// The kerb stations `kerb_gap` still counts as bare after this step,
    /// for the plan view and for finding them.
    pub gaps: Vec<[f64; 2]>,
}

/// A triangle mesh: positions in local metres, indices in triples,
/// counter-clockwise seen from above, vertices shared by position.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Tri {
    pub positions: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}

/// The paved surface as triangles, one mesh per family, every triangle
/// inside one triangle of the terrain.
#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub carriageway: Tri,
    pub pavement: Tri,
}

/// The room at the height the profile solved: the mesh step's triangles,
/// every vertex of a paved region that runs beside a carriageway moved
/// from the ground to the road's own surface.
#[derive(Debug, Clone, Default)]
pub struct Bench {
    pub carriageway: Tri,
    pub pavement: Tri,
    /// The engineered ground: the terrain with the room cut out of it and
    /// a batter run from the room's outline down to the natural ground.
    pub ground: Tri,
    /// The midpoint of every mesh edge the height field steps across —
    /// the line between two carriageways whose domains meet at different
    /// heights, which is a retaining wall — for the plan view and for
    /// finding them.
    pub steps: Vec<[f64; 2]>,
}

/// The structures: what the solved profile implies where it left the
/// ground. The roadway is every span piece's own paving, which no surface
/// step lays; the deck is the solid under a deck run and the bore the
/// tube over a bore run, so no two of the three are coplanar.
#[derive(Debug, Clone, Default)]
pub struct Structure {
    pub roadway: Tri,
    /// The solid under every deck run: the slab where its soffit clears
    /// the ground and the abutment block where it does not, which is one
    /// body and one surface.
    pub deck: Tri,
    pub bore: Tri,
    /// The columns under the decks that fly high enough to need them.
    pub pier: Tri,
    /// Every span's outline in plan, with the kind the source mapped it,
    /// for the plan view.
    pub plan: Vec<(Kind, Shapes)>,
    /// Every pier's footprint in plan, for the plan view: where a column
    /// stands is a 2D fact, and the one rule it has — that it may not
    /// stand in the road it crosses — is a 2D rule.
    pub piers: Vec<Shapes>,
}

static NONE: Shapes = Vec::new();

impl World {
    /// What the buildings refuse: empty before the facade step, and empty
    /// when no building input was given — open ground everywhere, so no
    /// step branches on whether buildings were read.
    pub fn solid(&self) -> &Shapes {
        self.facade.as_ref().map(|f| &f.solid).unwrap_or(&NONE)
    }

    /// The building footprints, on the same terms as [`World::solid`].
    pub fn walls(&self) -> &Shapes {
        self.facade.as_ref().map(|f| &f.footprints).unwrap_or(&NONE)
    }

    /// What the asphalt keeps out of: the solid with its pockets, on the
    /// same terms as [`World::solid`].
    pub fn built(&self) -> &Shapes {
        self.facade.as_ref().map(|f| &f.built).unwrap_or(&NONE)
    }

    /// The latest walk surface: the pavement once the kerb step has run,
    /// re-cut once the fillet has, the plain union before either.
    pub fn walk(&self) -> Option<&Shapes> {
        self.room
            .as_ref()
            .map(|r| &r.pavement)
            .or_else(|| self.fillet.as_ref().map(|f| &f.pavement))
            .or_else(|| self.kerb.as_ref().map(|k| &k.pavement))
            .or_else(|| self.surface.as_ref().map(|s| &s.walk))
    }

    /// The latest carriageway: filleted once the fillet step has run.
    pub fn carriageway(&self) -> Option<&Shapes> {
        self.fillet.as_ref().map(|f| &f.carriageway).or_else(|| self.surface.as_ref().map(|s| &s.carriageway))
    }

    /// `raw` as a carriageway: the buildings win, at the closed facade.
    pub fn asphalt(&self, raw: &Shapes) -> Shapes {
        poly::difference(raw, self.built())
    }

    /// `raw` as a pavement beside `carriageway`: the asphalt wins, and the
    /// walls win. Every step that draws a pavement finishes it here, so
    /// no two can disagree about where the kerb is or where a wall stands.
    pub fn pavement(&self, raw: &Shapes, carriageway: &Shapes) -> Shapes {
        poly::difference(&poly::difference(raw, carriageway), self.solid())
    }
}
