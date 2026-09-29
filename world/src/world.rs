//! The world: what the steps build, in local metres.

use serde::{Deserialize, Serialize};
use arpentry_server::project::Bounds;

use crate::frame::Extent;
use crate::grid::Grid;
use crate::poly::{self, Pt, Shape, Shapes};
use crate::width::Family;

/// Expands [`World`] from the steps table: one field per step, named after
/// it, holding its layer.
macro_rules! world_struct {
    ($($(#[$doc:meta])* $step:ident => $field:ident: $layer:ty,)*) => {
        /// The world for one bounding box: the layers the steps build, each
        /// `None` until its step has run.
        ///
        /// It is a record, not an object. No step reads it — every step takes
        /// the layers it needs as arguments and returns the one it makes — so
        /// the only code that knows what a layer depends on is
        /// [`crate::pipeline`], and the only code that reads a *partial* world
        /// is the two renderers, which draw whatever has been built.
        #[derive(Debug)]
        pub struct World {
            /// The patch of earth this world is.
            pub extent: Extent,
            $($(#[$doc])* pub $field: Option<$layer>,)*
        }

        impl World {
            pub fn new(bbox: Bounds) -> World {
                World { extent: Extent::of(bbox), $($field: None,)* }
            }
        }
    };
}
crate::step::steps!(world_struct);

impl World {
    /// The paved surface as far as it has been built: the carriageway, the
    /// pavement and the ballast at their benched heights once the bench has
    /// run, else on the raw ground as the mesh step laid them, else nothing.
    /// For the renderers, so a viewer opens the same nodes whichever step the
    /// run stopped after.
    pub fn paving(&self) -> Option<[std::borrow::Cow<'_, Tri>; 3]> {
        use std::borrow::Cow;
        if let Some(b) = &self.bench {
            return Some([Cow::Borrowed(&b.carriageway), Cow::Borrowed(&b.pavement), Cow::Borrowed(&b.ballast)]);
        }
        let (m, a) = (self.mesh.as_ref()?, self.arrangement.as_ref()?);
        let of = |x: Material| Cow::Owned(m.view(a, |f| f.material == x));
        Some([of(Material::Carriageway), of(Material::Pavement), of(Material::Ballast)])
    }

    /// The network the partition cut, once it has run.
    ///
    /// For the renderers and the probes, which read whatever has been built.
    /// No step calls it: a step is given its layers by [`crate::pipeline`].
    pub fn network(&self) -> Option<&Network> {
        self.partition.as_ref().map(|p| &p.network)
    }

    /// The latest solved profiles: the partition's, else the crossing's
    /// re-solve, else the profile step's first solve.
    ///
    /// For the renderers and the probes, like [`World::network`]. Each of the
    /// three is still its own step's layer, so a probe that wants to compare
    /// them reads the fields.
    pub fn solved(&self) -> Option<&Profiles> {
        self.partition
            .as_ref()
            .map(|p| &p.profiles)
            .or(self.crossing.as_ref().map(|c| &c.profiles))
            .or(self.profile.as_ref())
    }
}

/// A triangle mesh: positions in local metres, indices in triples,
/// counter-clockwise seen from above, vertices shared by position.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Tri {
    pub positions: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}

impl Tri {
    /// One triangle, in the order given, on vertices of its own.
    pub fn triangle(&mut self, t: [[f64; 3]; 3]) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&t);
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }

    /// One quad, as two triangles, in the order given, on vertices of its
    /// own.
    pub fn quad(&mut self, q: [[f64; 3]; 4]) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&q);
        self.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// A vertical face standing on the plan segment `p → q`, from height
    /// `at_p[0]` to `at_p[1]` at `p` and `at_q[0]` to `at_q[1]` at `q`, on
    /// vertices of its own; its area in square metres. Where the face tapers
    /// to nothing at one end, the half that would be a line is left out
    /// rather than drawn flat.
    pub fn face(&mut self, p: [f64; 3], q: [f64; 3], at_p: [f64; 2], at_q: [f64; 2]) -> f64 {
        let (dp, dq) = ((at_p[0] - at_p[1]).abs(), (at_q[0] - at_q[1]).abs());
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&[
            [p[0], p[1], at_p[0]],
            [p[0], p[1], at_p[1]],
            [q[0], q[1], at_q[1]],
            [q[0], q[1], at_q[0]],
        ]);
        if dp > f64::EPSILON {
            self.indices.extend_from_slice(&[base, base + 1, base + 2]);
        }
        if dq > f64::EPSILON {
            self.indices.extend_from_slice(&[base, base + 2, base + 3]);
        }
        (dp + dq) / 2.0 * (q[0] - p[0]).hypot(q[1] - p[1])
    }

    /// `other`'s triangles added on vertices of their own: its indices are
    /// shifted past this mesh's positions, never welded into them.
    pub fn append(&mut self, other: Tri) {
        let base = self.positions.len() as u32;
        self.positions.extend(other.positions);
        self.indices.extend(other.indices.iter().map(|i| i + base));
    }
}

/// The terrain mesh: a regular lattice over the bbox with a height per vertex.
///
/// Positions are derived from the grid and `z`, never stored twice, so the
/// mesh a viewer draws and the surface [`crate::lattice::height_at`] evaluates
/// read one array.
#[derive(Debug, Clone, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Kind {
    /// On the ground: what the surface is built from.
    Ground,
    /// Above it, at this level ordinal — positive where the source gave one,
    /// and **zero for a deck the terrain implied**: a refused notch says a
    /// structure is needed here, and says nothing at all about who is on top
    /// of whom. An ordinal is a claim about a *pair*, and the ground has no
    /// opinion about pairs.
    Bridge(i64),
    /// Below it, at this level ordinal — negative where the source gave one,
    /// and **zero for a bore the terrain implied**, for the same reason a
    /// promoted deck reads zero: the ground says a structure is needed and
    /// says nothing about who is under whom.
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

    /// A bridge: a structure whose paving is continuous with the ground it
    /// runs onto, as a bore's is not.
    pub fn is_deck(self) -> bool {
        matches!(self, Kind::Bridge(_))
    }
}

/// One span of a way: an arc interval and what the source says the way is
/// over it. A way's spans partition `[0, len]` — every arc named once,
/// nothing overlapping, nothing dropped.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Span {
    pub a0: f64,
    pub a1: f64,
    pub kind: Kind,
}

impl Span {
    pub fn len(&self) -> f64 {
        self.a1 - self.a0
    }

    pub fn is_empty(&self) -> bool {
        !(self.len() > 0.0)
    }
}

/// A whole way, clipped to the rect but **not cut at its annotation edges**.
///
/// The source encodes a bridge, a tunnel or an indoor stretch as a span of a
/// segment. Cut there, a mapper's split point would become a survey point: a
/// piece end is a connector, and a connector is where the profile pins a
/// height to the ground, so a bridge annotated short of the gorge lip would
/// have its deck pinned to the DEM inside the approach.
///
/// So a way stays whole, carries its spans as an *attribute* in arc, and is
/// cut only once the heights are solved — by [`crate::partition`], which is
/// where the annotation hands over to the geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Way {
    pub id: String,
    pub class: String,
    pub subclass: String,
    pub width_m: f64,
    pub pts: Vec<[f64; 2]>,
    /// The source's own spans, in arc along `pts`.
    pub spans: Vec<Span>,
    /// Stretches the source put at a level with no bridge or tunnel over
    /// them, as `(a0, a1, level)` in arc: an **ordinal**, not a structure. A
    /// road mapped at level −1 because it passes under a viaduct is on the
    /// ground; the level says only which of the two is on top, and only the
    /// crossing step asks.
    pub layers: Vec<(f64, f64, i64)>,
}

impl Way {
    /// The level of the way at arc `s`, as the crossing step reads it: the
    /// ordinal of the structure the source mapped there, else the layer of
    /// a stretch it put at a level without one, else the ground's zero.
    /// Indoor is not stacked against the ground and reads zero.
    pub fn level_at_arc(&self, s: f64) -> i64 {
        match self.kind_at_arc(s) {
            Kind::Bridge(n) | Kind::Tunnel(n) => n,
            Kind::Indoor => 0,
            Kind::Ground => self.layers.iter().find(|l| s >= l.0 && s <= l.1).map_or(0, |l| l.2),
        }
    }

    /// The way's length in metres.
    pub fn len(&self) -> f64 {
        crate::line::length(&self.pts)
    }

    /// Whether the way has no geometry.
    pub fn is_empty(&self) -> bool {
        self.pts.len() < 2
    }

    /// Whether any span of the way is off the ground or indoors.
    pub fn has_structure(&self) -> bool {
        self.spans.iter().any(|s| s.kind != Kind::Ground)
    }

    /// The kind the source mapped at arc `s`. Out of range reads as ground:
    /// a span table partitions the way, and float slop at an end is not a
    /// structure.
    pub fn kind_at_arc(&self, s: f64) -> Kind {
        self.spans.iter().find(|sp| s >= sp.a0 && s <= sp.a1).map_or(Kind::Ground, |sp| sp.kind)
    }

    /// The span containing arc `s`, as `(a0, a1)` — the window a consumer
    /// holding one crossing needs, so a way that runs near a point twice is
    /// read at the right place.
    pub fn span_at_arc(&self, s: f64) -> (f64, f64) {
        self.spans
            .iter()
            .find(|sp| s >= sp.a0 && s <= sp.a1)
            .map_or((0.0, f64::INFINITY), |sp| (sp.a0, sp.a1))
    }
}

/// A polyline with heights, in local metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Polyline3 {
    pub id: String,
    pub class: String,
    pub subclass: String,
    pub width_m: f64,
    pub pts: Vec<[f64; 3]>,
}

/// Way centrelines — roads and pedestrian ways — as the source drew them:
/// what the drape step reads and leaves.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Roads {
    /// The whole ways, clipped to the rect and uncut, with the spans the
    /// source mapped. A way is not split at its annotation edges, so no
    /// mapper's cut is an anchor.
    pub ways: Vec<Way>,
    /// Every way draped exactly onto the terrain mesh.
    pub lines: Vec<Polyline3>,
}

/// The conditioned surface along every solving axis, one [`Axis`] per
/// solving way in [`crate::reference::solving_of`]'s order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reference {
    pub axes: Vec<Axis>,
    /// The ways with the terrain's own bridge and tunnel priors written into
    /// their span tables: the network the profile solves.
    pub ways: Vec<Way>,
}

/// The reference along one axis, at the stations the profile will solve it
/// at ([`crate::standard::NODE_M`] apart, the way's own vertices kept).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Axis {
    /// The way this axis is of: an index into [`Roads::ways`]. Every step
    /// that holds a profile needs it, and it is known here and nowhere
    /// cheaper.
    pub way: usize,
    /// Arc length at each station, from the way's start.
    pub s: Vec<f64>,
    /// The plan position of each station.
    pub p: Vec<[f64; 2]>,
    /// The raw terrain there: what the earthwork is owed against, and what
    /// a departure is measured from.
    pub ground: Vec<f64>,
    /// The conditioned surface: blind runs bridged, notches filled, bumps
    /// shaved. What a profile is solved against.
    pub h: Vec<f64>,
    /// Where the ground under the axis is a structure's own top.
    pub blind: Vec<bool>,
    /// Where the ground under the axis is a mapped structure's slot or mass
    /// rather than the way's own ground (`reference::spanned_mask`).
    pub spanned: Vec<bool>,
    /// Notches the closing refused, as `(arc, arc)`: the terrain's bridge
    /// priors.
    pub refused_notch: Vec<(f64, f64)>,
    /// Crests the opening refused: the terrain's tunnel priors, for the
    /// classes whose ladder cannot climb them.
    pub refused_crest: Vec<(f64, f64)>,
    /// What each of the three passes moved, separately.
    pub moved: Moved,
}

/// How far each pass moved the surface, and at how many stations.
///
/// Kept per pass rather than as one composite, because the composite cannot
/// be read: the bridging lowers a causeway onto its rims, the closing lifts a
/// notch and the opening shaves a crest, and a station that has been through
/// two of them reports a number belonging to neither — a composite `shave`
/// can exceed the opening's own budget when it is the bridging at work.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Moved {
    pub bridged: usize,
    pub bridge_m: f64,
    pub filled: usize,
    pub fill_m: f64,
    pub shaved: usize,
    pub shave_m: f64,
}

/// Every solving way's profile: one height along every axis.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Profiles {
    pub profiles: Vec<Profile>,
}

impl Profiles {
    /// The solved surface against the raw DEM over every **at-grade**
    /// station: [`crate::step::Residual`], the same population in every step
    /// that reports it.
    ///
    /// At grade alone, because a chord standing thirty metres over a gorge is
    /// not a departure from the ground — it is a bridge, and the structure
    /// step answers for it — and counting it would swamp the number that
    /// matters. It is also what the earthwork actually benches. Every step
    /// reporting the same quantity over the same population against the same
    /// baseline is what lets the differences down a run attribute a height to
    /// the step that made it.
    pub fn residual(&self) -> crate::step::Residual {
        let mut r = crate::step::Residual::new();
        for p in &self.profiles {
            for st in p.stations.iter().filter(|st| st.solved == Solved::Grade) {
                r.push(st.h, st.ground);
            }
        }
        r
    }
}

/// The solved profile of one whole way.
///
/// One profile per way, not per piece: the way's annotation is carried in
/// [`Profile::spans`] as a *prior*, and the heights are solved along the
/// whole of it, so no mapper's split point is an anchor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Profile {
    /// The way this profile is of: an index into [`Roads::ways`].
    ///
    /// Not every way solves one ([`crate::reference::solving_of`]), so the
    /// profiles are a *subset* of the ways in their order, and every step
    /// holding a profile needs the way behind it — the crossing for the
    /// level ordinals, the partition to write the cut back, the sheet to
    /// group the pieces. Recorded here, the selection is made once rather
    /// than re-derived by every step that needs it.
    pub way: usize,
    pub id: String,
    pub class: String,
    pub width_m: f64,
    /// The spans the source mapped, in arc: a prior on the solve, never a
    /// command. What the heights make of them is [`Station::solved`].
    pub spans: Vec<Span>,
    pub stations: Vec<Station>,
}

impl Profile {
    /// The profile as a 3D polyline, for the layers that draw it.
    pub fn line(&self) -> Vec<[f64; 3]> {
        self.stations.iter().map(|st| [st.p[0], st.p[1], st.h]).collect()
    }

    /// Whether any span is a bridge or a bore — a piece the profile chords
    /// across, as against merely indoors.
    pub fn has_chord(&self) -> bool {
        self.spans.iter().any(|s| s.kind.is_structure())
    }

    /// The kind mapped at the way's low (`false`) or high (`true`) end.
    pub fn end_kind(&self, high: bool) -> Kind {
        let s = if high { self.spans.last() } else { self.spans.first() };
        s.map_or(Kind::Ground, |s| s.kind)
    }

    /// Every station's verdict, from its solved height and the span table:
    /// the consequence rule. A station of a structure run is a deck where it
    /// stands off the ground, a bore where it runs under, and at grade
    /// between ([`Solved::of`]); a station of a ground run is at grade.
    pub fn classify(&mut self) {
        for st in self.stations.iter_mut() {
            st.solved = Solved::Grade;
        }
        for (k0, k1, kind) in self.runs() {
            if kind.is_structure() {
                for st in &mut self.stations[k0..=k1] {
                    st.solved = Solved::of(st.h, st.ground);
                }
            }
        }
    }

    /// The stations `k0..=k1` with the abutment on either side: a chord's
    /// height is its two abutment heights, so whatever reads or moves a
    /// structure run takes the at-grade station at each end with it.
    pub fn with_abutments(&self, k0: usize, k1: usize) -> (usize, usize) {
        (k0.saturating_sub(1), (k1 + 1).min(self.stations.len().saturating_sub(1)))
    }

    /// The maximal runs of stations the source mapped the same, as inclusive
    /// index pairs with their kind. The partition of the way, in stations.
    pub fn runs(&self) -> Vec<(usize, usize, Kind)> {
        station_runs(&self.stations, &self.spans)
    }
}

/// The maximal runs of `stations` sharing one span's kind, as inclusive
/// index pairs. Every station lands in exactly one run, so the result
/// partitions the way however coarsely the stations sample it.
pub fn station_runs(stations: &[Station], spans: &[Span]) -> Vec<(usize, usize, Kind)> {
    // A station on a span boundary lies in two spans, and it is the
    // **abutment**: it belongs to the ground, so a chord starts from a height
    // the at-grade solve owns rather than from one of its own. Where both
    // sides are structures — a deck running straight into a bore — the first
    // takes it, and the two chords meet there.
    let kind_at = |s: f64| {
        let mut hit = spans.iter().filter(|sp| s >= sp.a0 && s <= sp.a1);
        let first = hit.next();
        match (first, hit.next()) {
            (Some(a), Some(b)) if a.kind.is_structure() && !b.kind.is_structure() => b.kind,
            (Some(a), _) => a.kind,
            (None, _) => Kind::Ground,
        }
    };
    let mut out: Vec<(usize, usize, Kind)> = Vec::new();
    for (k, st) in stations.iter().enumerate() {
        let kind = kind_at(st.s);
        match out.last_mut() {
            Some(run) if run.2 == kind => run.1 = k,
            _ => out.push((k, k, kind)),
        }
    }
    out
}

/// One station of a solved profile.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Station {
    /// Arc length from the way's start, in metres.
    pub s: f64,
    /// The plan position on the axis.
    pub p: [f64; 2],
    /// The raw ground there ([`crate::lattice::height_at`]): what the
    /// earthwork is owed against, and what a departure is measured
    /// from. **Not** what the profile is solved against.
    pub ground: f64,
    /// The conditioned surface there ([`Axis::h`]): the
    /// target the profile is solved toward, and the centre of the deviation
    /// box. Equal to `ground` wherever the DEM needed nothing done to it,
    /// which on flat ground is everywhere.
    pub reference: f64,
    /// The solved height of the surface.
    pub h: f64,
    /// What the solved height makes of the station.
    pub solved: Solved,
}

/// What a station is once solved: a consequence of the profile against the
/// ground, never of the annotation alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Solved {
    /// On the ground, within the earthwork's reach.
    Grade,
    /// Standing off the ground: a deck.
    Deck,
    /// Running under it: a bore.
    Bore,
}

impl Solved {
    /// What a structure station solved to `h` over `ground` is: a deck or a
    /// bore past [`crate::standard::STRUCTURE_MIN_M`] off the ground, at grade
    /// within it.
    pub fn of(h: f64, ground: f64) -> Solved {
        use crate::standard::STRUCTURE_MIN_M;
        if h - ground >= STRUCTURE_MIN_M {
            Solved::Deck
        } else if ground - h >= STRUCTURE_MIN_M {
            Solved::Bore
        } else {
            Solved::Grade
        }
    }
}

/// Every crossing the network has, and what the floor spent on them.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Crossings {
    pub crossings: Vec<Crossing>,
    /// Two axes crossing at the same level with no connector between them:
    /// a data error, counted and not solved.
    pub same: Vec<[f64; 2]>,
    /// The floor, in metres, at every station of every profile: what the
    /// crossings asked the ground to become. Indexed as the profiles.
    pub floor: Vec<Vec<f64>>,
    /// The profiles re-solved over that floor. The profile step's own layer
    /// is left as it solved, so the two can be compared.
    pub profiles: Profiles,
}

/// One place two carriageway axes cross in plan with no connector between
/// them: a grade separation, and the only thing in the model that couples
/// the height of one way to the height of another.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Crossing {
    /// Where the two axes cross, in local metres.
    pub at: [f64; 2],
    /// The way above, by the level ordinals, and the way below: an index
    /// into [`Profiles::profiles`], and the level the source mapped there.
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

/// What the cut came to: the network, the profiles written back with it, and
/// the groups its pieces fall into.
///
/// **A layer of its own, not a rewrite of earlier ones**: the drape's span
/// tables and the profile step's profiles stay as their own steps made
/// them, and the cut's versions of both are here.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Partition {
    /// The network as the cut leaves it: every way's span table as the cut
    /// derived it, and the pieces cut from it.
    pub network: Network,
    /// The profiles with that table written back and every station's verdict
    /// recomputed from it.
    pub profiles: Profiles,
    /// The pieces grouped into the surfaces that may merge
    /// ([`crate::partition::groups`]).
    pub groups: Groups,
}

/// What [`crate::partition::groups`] found: the grouping itself and what it
/// took to get it.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Groups {
    /// The group of every piece, in [`Network::pieces`]'s order.
    pub of: Vec<usize>,
    /// The group pairs that may never be one surface, as `(min, max)` — one
    /// per crossing pair, including those that needed no split.
    pub rivals: Vec<(usize, usize)>,
    /// Every group the split created, and the group it was carried out of —
    /// `(new, was)`. The split moves a structure run's pieces to a fresh id
    /// so it is never unioned with what it crosses, but `was` is exactly
    /// the id its own ground pieces kept, so this is the one thing that
    /// still says a peeled span belongs with its own approach rather than
    /// with nothing at all. [`crate::sheet`] is the reader: without it, a
    /// span the split ever touched has no ground piece anywhere sharing its
    /// group, so nothing can claim it — not "kept apart", which the model
    /// has a rule for, but unreachable, which it does not.
    pub parent: Vec<(usize, usize)>,
}

/// The network as the partition cut it: what every step after the heights
/// builds from.
///
/// A separate type from [`Roads`] because it is a separate layer. The span
/// tables here are the partition's, not the source's, and the pieces exist
/// only once the cut has been made; the drape's layer keeps what the source
/// said.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Network {
    /// The whole ways, with the span table the cut derived.
    pub ways: Vec<Way>,
    /// The pieces on the ground, clipped to the rect: what every surface
    /// step builds from. Nothing in it crosses anything else at another
    /// level, so the steps after this one may union freely.
    pub plan: Vec<Polyline2>,
    /// The pieces above or below the ground, or indoors, clipped likewise.
    /// No surface step reads them; the profile chords across the bridges
    /// and tunnels, and the structures are built from what it solves.
    pub spans: Vec<Polyline2>,
}

impl Network {
    /// Every piece, on the ground or off it.
    pub fn pieces(&self) -> impl Iterator<Item = &Polyline2> {
        self.plan.iter().chain(self.spans.iter())
    }
}

/// A plan-space polyline, in local metres: one piece of a way, all of one
/// [`Kind`]. The partition step cuts every way at every boundary of its span
/// table ([`crate::partition::cut_at`]); the pieces of one way share its `id`
/// and their end vertices.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
    /// Which way of [`Network::ways`] this piece was cut from, and the arc
    /// range of the span it is: what a consumer holding a piece needs to
    /// find the profile its way was solved into.
    pub way: usize,
    pub a0: f64,
    pub a1: f64,
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

/// The buildings: what nothing paved may enter.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Facade {
    /// Every building touching the rect, one by one: what the building step
    /// stands up. The masks below are their footprints unioned.
    pub buildings: Vec<Building>,
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

impl Facade {
    /// `raw` as a carriageway: the buildings win, at the **closed** facade,
    /// so the asphalt's edge does not follow every notch of an outline.
    pub fn asphalt(&self, raw: &Shapes) -> Shapes {
        poly::difference(raw, &self.built)
    }

    /// `raw` as a pavement beside `carriageway`: the asphalt wins, and the
    /// walls win. Every step that draws a pavement finishes it here, so no
    /// two can disagree about where the kerb is or where a wall stands.
    pub fn pavement(&self, raw: &Shapes, carriageway: &Shapes) -> Shapes {
        poly::difference(&poly::difference(raw, carriageway), &self.solid)
    }
}

/// One building as the source mapped it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Building {
    /// Its footprint, clipped to the rect and oriented: several shapes for
    /// a multipolygon, or for one the rect cut in two.
    pub footprint: Shapes,
    /// The ground to the top of its roof, in metres
    /// ([`crate::facade::mapped_height`]), decided once by the reader.
    pub height_m: f64,
    pub roof: crate::world::Roof,
}

/// A roof as the source mapped it: a prior, which the outline may refuse
/// ([`crate::building::form`]).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Roof {
    pub shape: RoofShape,
    /// Eave to ridge, in metres, where the source gives it.
    pub rise_m: Option<f64>,
}

impl Roof {
    /// The roof of a source's `roof_shape` and `roof_height`, either absent;
    /// a rise that is not positive is no rise.
    pub fn mapped(shape: Option<&str>, rise_m: Option<f64>) -> Roof {
        Roof { shape: shape.map_or(RoofShape::Flat, RoofShape::parse), rise_m: rise_m.filter(|h| *h > 0.0) }
    }
}

/// The roof shapes the building step builds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RoofShape {
    #[default]
    Flat,
    /// A ridge along the long axis of a quad, over the midpoints of its
    /// short edges.
    Gabled,
    /// An apex over the centroid of a convex outline.
    Pyramidal,
    /// One plane rising from the south edge to the north.
    Skillion,
}

impl RoofShape {
    /// Overture's `roof_shape`, as one of the four. A hip is built as a gable
    /// (a true hip needs a straight skeleton) and a dome as a pyramid; what
    /// is unknown is flat.
    pub fn parse(s: &str) -> RoofShape {
        match s {
            "gabled" | "hipped" | "half_hipped" | "round" | "gambrel" | "mansard" => RoofShape::Gabled,
            "pyramidal" | "dome" | "onion" | "cone" => RoofShape::Pyramidal,
            "skillion" | "lean_to" | "mono_pitch" | "shed" => RoofShape::Skillion,
            _ => RoofShape::Flat,
        }
    }
}

/// Every way as a polygon, unmerged, and the span pieces as the regions the
/// later steps need of them.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Ribbons {
    pub ribbons: Vec<Ribbon>,
    /// Every deck's ribbons unioned per `(family, group)`, capped round
    /// where a span meets another piece: the paving a sheet welds onto its
    /// approach ([`crate::ribbon::run`]).
    pub spans: Vec<(Family, usize, Shapes)>,
    /// The same, capped square where a span hands over to the ground: the
    /// mask of what is over a deck ([`crate::ribbon::run`]).
    pub masks: Vec<(Family, usize, Shapes)>,
    /// How many pieces of a family touch each connector, at a vertex and
    /// not only at an end ([`crate::ribbon::joints`]).
    pub joints: std::collections::BTreeMap<(Family, (i64, i64)), usize>,
}

/// One way's polygon: its centreline buffered to its width.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ribbon {
    pub id: String,
    pub class: String,
    pub subclass: String,
    pub family: Family,
    pub shape: Shapes,
}

/// The paved surface: one set of disjoint regions per family, none
/// overlapping another — the asphalt has been subtracted from the ballast
/// and both from the walk.
///
/// **Four steps produce one of these, and each is the surface as it stood
/// when that step finished**: the surface step lays it, the kerb fills the
/// strip to every attached sidewalk, the legs build the junctions, the room
/// paves the edges. A step passes through what it did not change, so the
/// last one to run holds the whole of the paving and "which layer is the
/// latest" is not a question anything has to answer — it is the last layer
/// that laid any.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Surface {
    pub carriageway: Shapes,
    pub walk: Shapes,
    /// The asphalt that is there but not on the ground: every solving
    /// family's **span** ribbons, grouped and unioned.
    ///
    /// Not paving — `carriageway` is the ground pieces' and stays so, which
    /// is what keeps a viaduct no part of the street beneath it until
    /// [`crate::sheet`] has grouped them. This is the *seniority* half of
    /// the same fact: a pavement may not be laid where a deck already is,
    /// and every step that decides where the walk may go has to be able to
    /// ask. Read through [`Surface::senior`].
    pub spanned: Shapes,
    /// The railways' track bed. It stops at the asphalt (a level crossing
    /// is the road's surface with the rails through it) and at nothing
    /// else: not at a building, because a station roof over its platforms
    /// is a level relation the model cannot state.
    pub ballast: Shapes,
}

impl Surface {
    /// The regions of `family`.
    pub fn of(&self, family: Family) -> &Shapes {
        match family {
            Family::Carriageway => &self.carriageway,
            Family::Walk => &self.walk,
            Family::Rail => &self.ballast,
        }
    }

    /// What a pavement stops at: the asphalt, the track bed and the spans
    /// together.
    pub fn senior(&self) -> Shapes {
        poly::union_of(&[&self.carriageway, &self.ballast, &self.spanned])
    }
}

/// The pavement: the walk surface with the strip between every attached
/// sidewalk and its kerb filled, so its inner edge is the kerb.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Kerb {
    /// The paved surface as this step leaves it: `walk` is the walk and the
    /// rungs unioned, its small holes filled, less the senior surface and the
    /// facades; the other three are the surface step's, untouched.
    pub surface: Surface,
    /// Every attached station, for the kerb-gap check downstream.
    pub attached: Vec<Attached>,
}

/// One attached station.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Attached {
    /// The station on the pedestrian way.
    pub station: Pt,
    /// Its foot on the road's axis.
    pub foot: Pt,
    /// The road's half-width there.
    pub half_m: f64,
    /// The axis segment the foot lies on.
    pub seg: [Pt; 2],
    /// The road the foot is on, in the index's order.
    pub road: usize,
    /// How far from the foot, toward the station, the asphalt ends: the
    /// attached road's own half-width on a straight, farther where the
    /// rung crosses another road's ribbon on the way out, as it does in
    /// the notch between two legs.
    pub exit_m: f64,
    /// Landed rather than attached: a footway's end on a kerb, not a
    /// pavement running along it, so no stretch of kerb is claimed from it.
    pub landed: bool,
}

/// The junctions built from their legs, and the surface they make.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Legs {
    pub junctions: Vec<Junction>,
    pub edges: Vec<PieceEdge>,
    /// The paved surface as this step leaves it: the carriageway built
    /// explicitly — every junction and every trimmed ground edge, cut to the
    /// facades — and the pavement laid back outside it.
    pub surface: Surface,
}

/// One junction: the node, its legs in counter-clockwise order, and the
/// polygon between their mouths.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Junction {
    pub at: Pt,
    pub legs: Vec<Leg>,
    pub shape: Shapes,
}

/// One leg of a junction: an edge leaving the node.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Leg {
    /// Which edge, and whether it leaves the node at its start.
    pub edge: usize,
    pub at_start: bool,
    /// Unit direction from the node along the edge's first segment: what the
    /// legs are ordered by.
    pub u: Pt,
    pub half_m: f64,
    pub class: String,
    /// Where the leg's mouth lies, in metres of arc from the node.
    pub mouth_m: f64,
    /// The edge's centreline, oriented away from the node.
    pub line: Vec<Pt>,
    /// The kerb walk past the edge's far node on the left side and on the
    /// right (`legs::walk`): points, each with its stretch's half-width.
    pub ahead: [Vec<(Pt, f64)>; 2],
}

/// A piece cut at the nodes it passes: what a junction's legs trim.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PieceEdge {
    pub piece: usize,
    pub pts: Vec<Pt>,
    pub width_m: f64,
    /// Where the edge is trimmed at each end, in metres of arc: the mouth of
    /// the junction there, or 0 at a free end.
    pub trim: [f64; 2],
    /// Whether each end is a node.
    pub node: [bool; 2],
    /// Whether the edge is on the ground, and so paved here: a deck's edge
    /// shapes its junctions and is paved by the `sheet` step.
    pub ground: bool,
}

/// The room filled: the pavement extended to every wall within reach.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Room {
    /// The paved surface as this step leaves it — and, since this is the
    /// last step that lays any, as it finally stands.
    pub surface: Surface,
    /// The bands and rungs, unioned; kept for the plan view.
    pub room: Shapes,
}

/// The paved surface as sheets that may merge.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Sheets {
    pub sheets: Vec<Sheet>,
}

impl Sheets {
    /// The sheets of `family`, in the order they were built.
    pub fn of(&self, family: Family) -> impl Iterator<Item = &Sheet> {
        self.sheets.iter().filter(move |s| s.family == family)
    }

    /// Every sheet's regions, flattened and not unioned.
    pub fn shapes(&self) -> Shapes {
        shapes(&self.sheets)
    }

    /// Everywhere the paving is over a span rather than on the ground:
    /// what the earthwork stages must not read as earthwork.
    pub fn spanned(&self) -> Shapes {
        poly::union_all(&self.sheets.iter().flat_map(|s| s.spans.iter().cloned()).collect())
    }
}

/// Every sheet's regions, flattened and not unioned: the paved surface as
/// the sheets drew it. Taken by slice so a step can ask it of the sheets it
/// is still building, before they are a [`Sheets`].
pub fn shapes(sheets: &[Sheet]) -> Shapes {
    sheets.iter().flat_map(|s| s.shapes.iter().cloned()).collect()
}

/// One paved sheet: every piece of one group of the piece graph, as one
/// set of regions.
///
/// **Two sheets are never unioned.** That is the whole reason they exist:
/// a viaduct and the street it flies over are two groups, they overlap in
/// plan, and merging them would draw one surface where there are two. Two
/// pieces that share a connector are one group, and within a group the
/// ground and the spans are one polygon with no boundary between them, so
/// there is no seam at an abutment to close.
///
/// Only the families that solve a profile get sheets
/// ([`Family::solves`]): a footbridge has no profile, so a walk span has
/// no field to be lifted by and is paved by the structure step.
/// The pavement is one sheet's worth of regions and is not partitioned.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sheet {
    pub family: Family,
    /// The group of [`crate::partition::groups`] this sheet is.
    pub group: usize,
    pub shapes: Shapes,
    /// The part of `shapes` that is over a span, kept apart from the whole
    /// because the ground steps have to be able to tell.
    ///
    /// A deck is paved but it is not *earthwork*: the fill under a road is
    /// the earthwork's to owe and the standoff under a deck is the
    /// structure's, so counted here `cut`, `fill` and `walled` would read the
    /// deck's own clearance as an embankment nobody built. Nor does a deck's
    /// rim meet the ground — its soffit closes it — so `unmet` must not look
    /// for the ground under it either.
    pub spans: Shapes,
    /// The axes this sheet is lifted by: an index into
    /// [`Profiles::profiles`] and the arc range of one of that way's
    /// pieces. The lift builds one height field per sheet from exactly
    /// these ([`crate::copies::Fields`]), so a vertex is a function of its
    /// own sheet's stations and of nothing else.
    ///
    /// A *profile* index rather than a way index, because the lift holds
    /// the profiles and not the roads, and which ways solve is the
    /// reference step's answer ([`crate::reference::solving_of`]) —
    /// resolved here, where the roads are, rather than carried as a
    /// question for a step that cannot answer it.
    pub axes: Vec<(usize, f64, f64)>,
    /// The same for the stretches that are over a span: the chords of the
    /// decks in `spans`.
    ///
    /// The lift builds a field from each and asks the one belonging to
    /// the surface the vertex is on. **The two overlap by a station at
    /// every boundary between them** (the fields are built a station past
    /// each end of every range), and that is what makes the handover
    /// continuous: the profile is one curve through its own abutment, so two
    /// fields that both sample it there agree there — and both see the
    /// connector, so both blend at the junction. Built flush, the ground's
    /// axes would stop short of the connector, only the chords' field would
    /// have a joint to blend at, and the surface would crease along the
    /// mask's edge.
    pub chords: Vec<(usize, f64, f64)>,
    /// Whether the group holds any piece off the ground. A sheet that
    /// spans is the one with a handover in it.
    pub spanning: bool,
}

/// The rect, partitioned.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Arrangement {
    pub faces: Vec<Face>,
    /// **The second layer: paving over a span that lies over other paving.**
    ///
    /// The partition has one face per point, and it is the *ground's*: where
    /// a deck crosses a street in plan, the face under it is the street's,
    /// which cuts the terrain and meets its own kerbs. The deck's paving
    /// over that face is here instead — the same shape, from the same slice,
    /// so it welds to the rest of its sheet by position exactly as the faces
    /// do. Never in [`Arrangement::edges`] and never in the hole: a deck's
    /// side is closed by its slab, and the ground under it is the street's.
    pub decks: Vec<Face>,
    /// The paving over a span that `Face::spanned` was tagged by
    /// ([`crate::arrangement::over_spans`]), kept so that the lift, the
    /// copies and the earthwork read the one mask rather than rebuilding it:
    /// the hole and the lift must agree on it.
    pub over: Shapes,
    /// The area of the walk a deck carries, the part of `over` the sheets
    /// alone do not hold.
    pub carried_m2: f64,
    /// [`Arrangement::edges`], computed once.
    pub edges: Vec<FaceEdge>,
    /// The tunnels' mouths and the galleries' footprints
    /// ([`crate::portal::Portals`]): the arrangement cuts a face for each
    /// gallery, and the earthwork and the bench leave each mouth open.
    pub portals: crate::portal::Portals,
}

impl Arrangement {
    /// The faces of `material`.
    pub fn of(&self, material: Material) -> impl Iterator<Item = &Face> {
        self.faces.iter().filter(move |f| f.material == material)
    }

    /// Face `i` of both layers: the partition's faces are `0..faces.len()`,
    /// and the decks over them follow — the numbering [`crate::world::Mesh`]
    /// tags its triangles with.
    pub fn face(&self, i: u32) -> &Face {
        let i = i as usize;
        self.faces.get(i).unwrap_or_else(|| &self.decks[i - self.faces.len()])
    }

    /// Whether face `i` is of the partition rather than a deck over it.
    pub fn in_partition(&self, i: u32) -> bool {
        (i as usize) < self.faces.len()
    }

    /// Every face of both layers, in [`Arrangement::face`]'s order.
    pub fn all(&self) -> impl Iterator<Item = &Face> {
        self.faces.iter().chain(&self.decks)
    }

    /// The regions that cut the terrain's hole: every paved face on the
    /// ground, and every gallery's: [`Earthwork::outline`] as faces rather
    /// than as edges.
    pub fn hole(&self) -> Shapes {
        self.faces.iter().filter(|f| f.cuts()).map(|f| f.shape.clone()).collect()
    }
}

/// One face of the subdivision: a region of the rect, and what it is.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Face {
    pub shape: Shape,
    pub material: Material,
    /// The sheet this face belongs to, for the families that have them
    /// ([`crate::width::Family::solves`]). `None` for ground, for the
    /// pavement — which is not partitioned — and for a paved face no sheet
    /// claims.
    pub sheet: Option<usize>,
    /// Over a span rather than on the ground. Paved, and **it does not cut
    /// the ground**: a viaduct flies over terrain that is still there,
    /// and the soffit is what closes under it.
    pub spanned: bool,
    /// Inside a gallery's footprint — a road under the ground whose tube
    /// fits nowhere. Ground, and it *does* cut: the tube stands in the
    /// trench the earthwork digs for it.
    pub gallery: bool,
    /// Within [`crate::standard::WALL_REACH_M`] of the asphalt: the band the
    /// lift raises to the road's height, against the part beyond it that
    /// drapes.
    ///
    /// **A cut, not a boolean.** The two rules can disagree by the whole
    /// drop, so the step between them has to lie on mesh edges. Found by
    /// dilating and intersecting, that line would put vertices on the walk
    /// that no other face has; as a cut it is one more boundary where the
    /// ground does something different on each side.
    pub near: bool,
}

impl Face {
    /// Whether this face cuts the terrain's hole.
    ///
    /// Paving on the ground does; paving over a span does not (a viaduct
    /// flies over ground that is still there, and the soffit closes under
    /// it); a gallery does, though it is ground — the tube stands in
    /// the trench the earthwork digs for it.
    pub fn cuts(&self) -> bool {
        self.gallery || (self.material != Material::Ground && !self.spanned)
    }
}

/// What a face of the arrangement is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Material {
    /// Not paved: the terrain, and the only material the ground mesh draws.
    Ground,
    Carriageway,
    Pavement,
    Ballast,
}

impl Material {
    /// What the paving of a way of `family` is made of.
    pub fn of(family: Family) -> Material {
        match family {
            Family::Carriageway => Material::Carriageway,
            Family::Walk => Material::Pavement,
            Family::Rail => Material::Ballast,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Material::Ground => "ground",
            Material::Carriageway => "carriageway",
            Material::Pavement => "pavement",
            Material::Ballast => "ballast",
        }
    }
}

/// One edge of the subdivision: a segment, and the faces on each side.
///
/// What the arrangement's own checks walk, and what the earthwork asks
/// whether the height field steps across. A segment rather than a maximal
/// shared boundary, because that is what a mesher emits a triangle edge
/// along.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FaceEdge {
    pub a: Pt,
    pub b: Pt,
    /// The faces either side. `right` is `None` on the rect's own boundary,
    /// where there is nothing on the far side.
    pub left: usize,
    pub right: Option<usize>,
}

/// The whole rect as **one** triangulation: every face of the arrangement,
/// both layers, every triangle inside one triangle of the terrain, and one
/// vertex per plan position, at the natural ground.
///
/// The paving and the ground share their boundary vertices **by index**, so
/// no seam between them has to be matched by position afterwards. Which
/// surface a triangle belongs to is its face's to say ([`Mesh::of_face`]),
/// so a vertex's sheet, its side of the room's reach and its material are
/// all read off the face rather than carried beside it.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Mesh {
    pub tri: Tri,
    /// The face each triangle came from, as [`Arrangement::face`]
    /// numbers them: the partition's faces first, then the decks over them.
    pub of_face: Vec<u32>,
}

impl Mesh {
    /// The triangles whose face in `arrangement` `keep` accepts, on vertices
    /// of their own: what a renderer draws for one material before the lift
    /// has moved anything.
    pub fn view(&self, arrangement: &Arrangement, keep: impl Fn(&Face) -> bool) -> Tri {
        let mut out = Tri::default();
        let mut remap: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
        for (t, &f) in self.tri.indices.chunks_exact(3).zip(&self.of_face) {
            if !keep(arrangement.face(f)) {
                continue;
            }
            for &v in t {
                let id = *remap.entry(v).or_insert_with(|| {
                    out.positions.push(self.tri.positions[v as usize]);
                    (out.positions.len() - 1) as u32
                });
                out.indices.push(id);
            }
        }
        out
    }

    /// Whether vertex `v` lies on the rect's own border, to the kernel's
    /// grid: the rect's ring is snapped to it on the way in, so its vertices
    /// stand up to a grid step inside the mesh's extreme.
    pub fn on_border(&self) -> impl Fn(u32) -> bool + '_ {
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for q in &self.tri.positions {
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
        move |v: u32| {
            let q = self.tri.positions[v as usize];
            (0..2).any(|k| (q[k] - lo[k]).abs() <= poly::GRID_M || (q[k] - hi[k]).abs() <= poly::GRID_M)
        }
    }
}

/// The room lifted onto the solved profile: every paved copy of the one mesh
/// at its surface's height, welded where two rules agree. No ground yet.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Lifted {
    /// The height fields the copies were lifted by, and which the earthwork
    /// asks again along every edge it checks.
    pub fields: crate::copies::Fields,
    /// The paved copies at their lifted heights.
    pub copies: crate::copies::Copies,
    /// Per paved copy — the carriageway's, the ballast's, the pavement's —
    /// the foot its rule read.
    pub feet: [Vec<Option<crate::field::Foot>>; 3],
    /// Every edge of the one mesh across which the surface or the rule
    /// changes: the outline, the kerbs and the splits.
    pub bounds: Vec<crate::copies::Boundary>,
}

/// The ground benched to the lifted room.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Earthwork {
    /// The lift's copies with the ground's added at the engineered ground,
    /// and the passive pavement regraded onto it.
    pub copies: crate::copies::Copies,
    /// The room's outline: every one-mesh edge between a face that cuts and
    /// one that does not, the room on its left.
    pub outline: Vec<(u32, u32)>,
    /// The key on each outline edge's cutting side: which paved surface, by
    /// which rule, the room is there.
    pub cutting: Vec<Option<crate::copies::Key>>,
    /// The room's height at the two ends of each outline edge, on its
    /// cutting side.
    pub top: Vec<[f64; 2]>,
    /// The engineered ground, as a function of the point.
    pub ground: crate::ground::Ground,
    /// The midpoint of every edge the height field steps across.
    pub steps: Vec<[f64; 2]>,
}

/// The room and the ground closed into one surface: the four meshes the
/// earthwork's copies make (the carriageway, the pavement, the ballast and
/// the engineered ground) and the faces this step draws between them.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Bench {
    pub carriageway: Tri,
    pub pavement: Tri,
    /// The track bed at the height its railway solved.
    pub ballast: Tri,
    /// The engineered ground: the terrain with the room cut out of it and
    /// a batter run from the room's outline down to the natural ground.
    pub ground: Tri,
    /// The face that closes the step between the room's edge and the
    /// ground beside it wherever a batter could not run: the retaining
    /// wall, and the only thing standing between the two meshes and a
    /// hole you can see the world through.
    pub wall: Tri,
    /// Every face the edge rule draws inside the paving where its two sides
    /// do not weld: the kerb, where the pavement stands its rise over the
    /// carriageway, and the splits between two surfaces or two rules.
    pub kerb: Tri,
}

/// The structures: what the solved profile implies where it left the
/// ground. The deck is the solid under a deck run and the bore the tube over
/// a bore run. A road's or a railway's *deck* is paved by its [`Sheet`]; the
/// paving here is what no sheet lays — a bore's floor, which a sheet's field
/// would read from the road above it, and a walk span, which has no profile.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Structure {
    /// The paving of every bore and every walk span.
    pub roadway: Tri,
    /// The same for a railway's spans: its track bed over a deck and
    /// through a bore.
    pub track: Tri,
    /// The solid under every deck run: a slab of constant thickness under
    /// the roadway, full length — a continuous surface in the air.
    pub deck: Tri,
    pub bore: Tri,
    /// Every span's outline in plan, with the kind the source mapped it,
    /// for the plan view.
    pub plan: Vec<(Kind, Shapes)>,
}

/// The buildings standing on the terrain.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Buildings {
    /// Every footprint's walls, from under the lowest ground along its
    /// outline up to its roof's rim: gable ends and courtyards included.
    pub walls: Tri,
    pub roofs: Tri,
}
