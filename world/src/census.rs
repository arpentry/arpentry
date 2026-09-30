//! The census: what a viewer of the finished world would see wrong, counted
//! by species and located.
//!
//! Every step checks its own layer, and a step's check measures the thing
//! that step built — the arrangement's `unshared`, the mesh's `crack`, the
//! earthwork's `step`. None of them looks at the triangles a viewer draws,
//! and those are where the steps meet: a fix that is right for one step and
//! moves the defect into the next reads as progress on every line. The
//! census reads the drawn world only — the layers [`crate::gltf`] writes,
//! as triangles — so it is the one number a change to *any* step answers to.
//!
//! **A defect here is something a viewer can see**, and each species is one
//! way of seeing it:
//!
//! - `gap`: an edge only one triangle uses, away from the rect's border,
//!   not under the ground, and not a tube's mouth: a hole you see the world
//!   through. Chained, so one hole is one defect.
//! - `crack`: the same, but every point of it lies within [`CRACK_M`] of
//!   another triangle — the far side of a T-junction, or a face whose edge
//!   rests on a surface without sharing its vertices. Nothing to see
//!   through; at worst a sparkle in motion. Counted apart so it cannot hide
//!   a gap.
//! - `fin`: a triangle of a paved surface steeper than the natural ground
//!   under it by more than [`FIN_GRADE`] (45°'s worth of grade), and
//!   steeper than that outright. A street draped on a 100 % flank is the
//!   flank; a paved triangle standing up out of the ground is a surface
//!   that has been pulled apart.
//! - `flip`: a triangle of a plan-facing surface wound clockwise from above:
//!   a fold, a surface showing its underside.
//! - `fight`: two plan-facing triangles over one point within [`FIGHT_M`]
//!   of each other — they z-fight in any viewer.
//! - `buried`: the ground standing over the bench's paving by more than
//!   [`FIGHT_M`]. A bore's floor is under the ground by design, and is not
//!   asked.
//! - `overlap`: two triangles of the arrangement's own partition over one
//!   point at different heights. The partition has one face per point, so
//!   this is a mesh that overlaps itself.
//!
//! Where two surfaces over one point is by design — a deck over the street,
//! a footbridge over the ground — the pair is `stacked`, and counted, not
//! charged.
//!
//! **Every defect says where it came from**: the drawn layers of the
//! triangles it is made of and, for the plan-facing ones, the arrangement
//! faces they were meshed from. The lift and the earthwork move a vertex
//! only up and down, so a drawn triangle has the plan position of the mesh
//! triangle it is, and [`Census::take`] finds it by that alone.
//!
//! Nothing here is a step: it reads the world after the last step and
//! writes nothing into it.

use std::collections::HashMap;

use serde::Serialize;

use crate::frame::Extent;
use crate::poly;
use crate::step::Summary;
use crate::world::{Material, Tri, World};

/// Two drawn vertices closer than this, in metres, are one: the polygon
/// kernel's own grid, below which no step means anything.
const WELD_M: f64 = poly::GRID_M;

/// How far off the rect's border, in metres, an open edge may lie and still
/// be the border: the rect is snapped to the kernel's grid on the way in.
const BORDER_M: f64 = 1e-3;

/// An open edge every sample of which lies this close to another triangle,
/// in metres, is a `crack` rather than a `gap`.
const CRACK_M: f64 = 1e-3;

/// An open edge is sampled every this many metres, at least three times
/// and at most [`MAX_SAMPLES`], for the `crack` test.
const SAMPLE_M: f64 = 0.5;
const MAX_SAMPLES: usize = 256;

/// How far, in metres, an open edge must lie under the earth's surface to be
/// hidden by it: a building's foundation, a wall's foot, a bore's floor.
/// The earth's surface is the ground and the paving that cuts it
/// ([`crate::world::Face::cuts`]); a deck is in the air, and does not hide
/// a hole in the street under it.
const HIDDEN_M: f64 = 0.05;

/// How much steeper than the natural ground under it, as rise over run, a
/// paved triangle may stand — and how steep outright — before it is a fin.
const FIN_GRADE: f64 = 1.0;

/// A defect smaller than this, in square metres — a ten-centimetre square —
/// is a speck: counted, never charged. Most are needles a millimetre wide
/// whose plane a millimetre of height noise tips on end.
const SPECK_M2: f64 = 0.01;

/// Two surfaces over one point closer than this, in metres, z-fight.
const FIGHT_M: f64 = 0.05;

/// Two plan-facing triangles overlapping by less than this, in square
/// metres, are neighbours whose shared edge the clip read as an area.
const OVERLAP_M2: f64 = 1e-3;

/// A triangle with less plan area than this, in square metres, has no
/// height to read over a point: a wall, a kerb, or nothing.
const FLAT_M2: f64 = 1e-9;

/// A triangle with less area than this, in square metres, has no normal.
const DEGENERATE_M2: f64 = 1e-10;

/// The most edges or triangles one defect carries to be drawn.
const MAX_SHAPE: usize = 200;

/// The side of the plan index's cells, in metres: a bucket, nothing about
/// the answer.
const CELL_M: f64 = 2.0;

/// What a drawn layer is, for the species that read it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    /// Faces up and covers the plan once: the ground.
    Ground,
    /// Faces up, is paved, and must not stand up: `fin` reads it. The
    /// bench's paving, which `buried` also reads.
    Paved,
    /// A structure's paving: a bore's floor, a footbridge's deck. Faces up
    /// and must not stand up, but a bore's lies under the ground.
    Floor,
    /// A face, a solid or a roof: in the closure, in nothing else.
    Other,
}

/// The drawn layers, as [`crate::gltf::write_glb`] names its nodes, with
/// their role.
fn layers(world: &World) -> Vec<(&'static str, &Tri, Role)> {
    let mut out = Vec::new();
    if let Some(b) = &world.bench {
        out.push(("ground", &b.ground, Role::Ground));
        out.push(("wall", &b.wall, Role::Other));
        out.push(("kerb", &b.kerb, Role::Other));
        out.push(("carriageway", &b.carriageway, Role::Paved));
        out.push(("pavement", &b.pavement, Role::Paved));
        out.push(("ballast", &b.ballast, Role::Paved));
    }
    if let Some(s) = &world.structure {
        out.push(("roadway", &s.roadway, Role::Floor));
        out.push(("track", &s.track, Role::Floor));
        out.push(("deck", &s.deck, Role::Other));
        out.push(("bore", &s.bore, Role::Other));
    }
    if let Some(b) = &world.building {
        out.push(("building", &b.walls, Role::Other));
        out.push(("roof", &b.roofs, Role::Other));
    }
    out
}

/// One kind of defect. See the module header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Species {
    Gap,
    Fin,
    Flip,
    Buried,
    Fight,
    Overlap,
    Crack,
}

impl Species {
    pub const ALL: [Species; 7] =
        [Species::Gap, Species::Fin, Species::Flip, Species::Buried, Species::Fight, Species::Overlap, Species::Crack];

    pub fn name(self) -> &'static str {
        match self {
            Species::Gap => "gap",
            Species::Fin => "fin",
            Species::Flip => "flip",
            Species::Buried => "buried",
            Species::Fight => "fight",
            Species::Overlap => "overlap",
            Species::Crack => "crack",
        }
    }

    /// Whether [`Defect::size`] is a length rather than an area.
    fn is_length(self) -> bool {
        matches!(self, Species::Gap | Species::Crack)
    }
}

/// One defect: where it is, how big, and what it is made of.
#[derive(Debug, Clone, Serialize)]
pub struct Defect {
    pub species: Species,
    /// A point on it, in local metres: the midpoint of its longest edge,
    /// or the centroid of its largest triangle or overlap.
    pub at: [f64; 3],
    pub lon: f64,
    pub lat: f64,
    /// Its length in metres (`gap`, `crack`) or its area in square metres.
    pub size: f64,
    /// How far it stands: the height span of a gap or a fin, the height
    /// between the two surfaces of an overlap.
    pub rise: f64,
    /// How wide a gap opens, in metres: the most any sample of it stands
    /// from the far side of the hole, capped at a metre. A lower bound.
    pub width: f64,
    /// How many edges, triangles or triangle pairs make it.
    pub parts: usize,
    /// The drawn layers of the triangles it is made of, sorted.
    pub layers: Vec<&'static str>,
    /// The arrangement faces those triangles were meshed from, sorted —
    /// empty for the layers no face makes (the faces, the solids).
    pub faces: Vec<u32>,
    /// What it is, to draw: a gap's or a crack's open edges, and every
    /// other species' triangles, at most [`MAX_SHAPE`] of either.
    pub lines: Vec<[[f64; 3]; 2]>,
    pub triangles: Vec<[[f64; 3]; 3]>,
}

/// What the census counts but does not charge.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Tally {
    pub triangles: usize,
    /// Open edges on the rect's border.
    pub border: usize,
    /// Open edges under the earth's surface: the ground, or paving on it.
    pub hidden: usize,
    /// Open edges of a tube only: its mouths.
    pub mouth: usize,
    /// Edges more than two triangles use.
    pub nonmanifold: usize,
    /// Triangles with no area.
    pub degenerate: usize,
    /// Defects too small to see: a gap whose length times width, or a fin,
    /// flip or overlap whose area, is under [`SPECK_M2`].
    pub specks: usize,
    /// Plan-facing triangles of the bench's four surfaces the mesh step has
    /// no triangle for: drawn where the arrangement put nothing.
    pub unfaced: usize,
    /// Pairs of surfaces over one point by design, and their area.
    pub stacked: usize,
    pub stacked_m2: f64,
}

/// The census of one world.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Census {
    /// The world's bounding box in degrees, `[west, south, east, north]`:
    /// what a reader needs to tell a defect of the clip from one of the
    /// model.
    pub bbox: [f64; 4],
    /// Every defect, by species and then largest first (a gap by its
    /// length times its width).
    pub defects: Vec<Defect>,
    pub tally: Tally,
}

/// The drawn world as one triangle soup: welded vertices, and per triangle
/// its layer and (for the plan-facing ones) its arrangement face.
struct Soup {
    pos: Vec<[f64; 3]>,
    tris: Vec<[u32; 3]>,
    layer: Vec<u8>,
    face: Vec<Option<u32>>,
    names: Vec<&'static str>,
    roles: Vec<Role>,
    /// How many faces the arrangement's partition has: a face number past
    /// it is a deck over the partition ([`crate::world::Arrangement::face`]).
    partition: u32,
    /// Per arrangement face, whether it is the earth's surface: ground, or
    /// paving that cuts it.
    earth: Vec<bool>,
}

impl Soup {
    fn of(world: &World) -> Soup {
        let mut soup =
            Soup { pos: Vec::new(), tris: Vec::new(), layer: Vec::new(), face: Vec::new(), names: vec![], roles: vec![], partition: 0, earth: vec![] };
        if let Some(a) = &world.arrangement {
            soup.partition = a.faces.len() as u32;
            soup.earth = a.all().map(|f| f.material == Material::Ground || f.cuts()).collect();
        }
        let mut weld = Weld::default();
        let faces = FaceIndex::of(world);
        for (k, (name, tri, role)) in layers(world).into_iter().enumerate() {
            soup.names.push(name);
            soup.roles.push(role);
            let ids: Vec<u32> = tri.positions.iter().map(|&p| weld.id(p, &mut soup.pos)).collect();
            for t in tri.indices.chunks_exact(3) {
                let corners = [0, 1, 2].map(|i| tri.positions[t[i] as usize]);
                soup.tris.push([0, 1, 2].map(|i| ids[t[i] as usize]));
                soup.layer.push(k as u8);
                // Only the ground and the bench's paving are the mesh's
                // triangles. A structure's floor is triangulated on its own,
                // and where it covers a lattice cell whole it has the same
                // plan — the same centroid — as the paving over it, so asked
                // by position it would take that paving's face and read as
                // the partition overlapping itself.
                let face = match role {
                    Role::Ground | Role::Paved => faces.as_ref().and_then(|f| f.at(corners)),
                    Role::Floor | Role::Other => None,
                };
                soup.face.push(face);
            }
        }
        soup
    }

    fn corners(&self, t: usize) -> [[f64; 3]; 3] {
        self.tris[t].map(|v| self.pos[v as usize])
    }

    fn role(&self, t: usize) -> Role {
        self.roles[self.layer[t] as usize]
    }

    fn name(&self, t: usize) -> &'static str {
        self.names[self.layer[t] as usize]
    }

    /// Whether triangle `t` is of the earth's surface: the ground, or paving
    /// on it — not a deck, not a structure's floor.
    fn on_earth(&self, t: usize) -> bool {
        matches!(self.role(t), Role::Ground | Role::Paved)
            && self.face[t].is_some_and(|f| self.earth.get(f as usize).copied().unwrap_or(false))
    }

    /// Whether triangle `t` faces up and has a height to read over a point.
    fn plan_facing(&self, t: usize) -> bool {
        self.role(t) != Role::Other && plan_area(self.corners(t)).abs() > FLAT_M2
    }
}

/// Vertices welded at [`WELD_M`]: a hash of the grid cell, searched with its
/// neighbours so two points either side of a cell wall still meet. The first
/// vertex seen is the one kept, so the result is a function of the order the
/// layers are read in, which is fixed.
#[derive(Default)]
struct Weld {
    cells: FxMap<[i64; 3], Vec<u32>>,
}

impl Weld {
    /// The side of a cell: four weld distances, so a point needs its
    /// neighbour on an axis only within a weld distance of that wall —
    /// half the time, rather than always.
    const CELL: f64 = 4.0 * WELD_M;

    fn id(&mut self, p: [f64; 3], pos: &mut Vec<[f64; 3]>) -> u32 {
        let c = p.map(|x| (x / Self::CELL).floor() as i64);
        let reach = |k: usize| {
            let o = p[k] - c[k] as f64 * Self::CELL;
            (if o < WELD_M { -1 } else { 0 })..=(if o > Self::CELL - WELD_M { 1 } else { 0 })
        };
        for dx in reach(0) {
            for dy in reach(1) {
                for dz in reach(2) {
                    if let Some(ids) = self.cells.get(&[c[0] + dx, c[1] + dy, c[2] + dz]) {
                        for &id in ids {
                            let q = pos[id as usize];
                            if (0..3).all(|k| (p[k] - q[k]).abs() <= WELD_M) {
                                return id;
                            }
                        }
                    }
                }
            }
        }
        let id = pos.len() as u32;
        pos.push(p);
        self.cells.entry(c).or_default().push(id);
        id
    }
}

/// The mesh step's triangles by plan centroid, so a drawn plan-facing
/// triangle finds the arrangement face it was meshed from. Where two mesh
/// triangles share a centroid — a deck over the street, the arrangement's
/// second layer — the drawn one is the deck if it is the higher.
struct FaceIndex {
    by: FxMap<[i64; 2], Vec<(u32, bool)>>,
}

impl FaceIndex {
    fn of(world: &World) -> Option<FaceIndex> {
        let (m, a) = (world.mesh.as_ref()?, world.arrangement.as_ref()?);
        let mut by: FxMap<[i64; 2], Vec<(u32, bool)>> = FxMap::default();
        for (t, &f) in m.tri.indices.chunks_exact(3).zip(&m.of_face) {
            let c = [0, 1, 2].map(|i| m.tri.positions[t[i] as usize]);
            by.entry(centroid_key(c)).or_default().push((f, a.in_partition(f)));
        }
        Some(FaceIndex { by })
    }

    fn at(&self, corners: [[f64; 3]; 3]) -> Option<u32> {
        let hits = self.by.get(&centroid_key(corners))?;
        match hits.as_slice() {
            [(f, _)] => Some(*f),
            // The partition's face and the deck's over it: the census
            // cannot tell them apart by plan, and the lift put the deck on
            // top, so the one the drawn triangle is depends on which layer
            // asks. Either names the place.
            many => many.iter().find(|(_, part)| !part).or(many.first()).map(|(f, _)| *f),
        }
    }
}

fn centroid_key(c: [[f64; 3]; 3]) -> [i64; 2] {
    [0, 1].map(|k| ((c[0][k] + c[1][k] + c[2][k]) / 3.0 / WELD_M).round() as i64)
}

/// Twice the signed plan area... halved: the signed area of the triangle's
/// plan, positive when it winds counter-clockwise seen from above.
fn plan_area(c: [[f64; 3]; 3]) -> f64 {
    ((c[1][0] - c[0][0]) * (c[2][1] - c[0][1]) - (c[2][0] - c[0][0]) * (c[1][1] - c[0][1])) / 2.0
}

fn normal(c: [[f64; 3]; 3]) -> [f64; 3] {
    let (u, v) = (sub(c[1], c[0]), sub(c[2], c[0]));
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn area3(c: [[f64; 3]; 3]) -> f64 {
    let n = normal(c);
    (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() / 2.0
}

/// The natural ground's slope, as rise over run, at triangle `c`'s plan
/// centroid: central differences over [`SLOPE_M`] on the terrain lattice.
fn natural_slope(world: &World, c: [[f64; 3]; 3]) -> f64 {
    let Some(t) = &world.terrain else { return 0.0 };
    let (x, y) = ((c[0][0] + c[1][0] + c[2][0]) / 3.0, (c[0][1] + c[1][1] + c[2][1]) / 3.0);
    let h = |x: f64, y: f64| crate::lattice::height_at(t, x, y);
    let gx = (h(x + SLOPE_M, y) - h(x - SLOPE_M, y)) / (2.0 * SLOPE_M);
    let gy = (h(x, y + SLOPE_M) - h(x, y - SLOPE_M)) / (2.0 * SLOPE_M);
    gx.hypot(gy)
}

/// Half the step of [`natural_slope`]'s differences, in metres.
const SLOPE_M: f64 = 0.25;

/// The height of triangle `c`'s plane over the plan point `p`.
fn height_on(c: [[f64; 3]; 3], p: [f64; 2]) -> f64 {
    let n = normal(c);
    c[0][2] - (n[0] * (p[0] - c[0][0]) + n[1] * (p[1] - c[0][1])) / n[2]
}

/// Whether the plan point `p` lies in triangle `c`'s plan, edges included.
fn inside(c: [[f64; 3]; 3], p: [f64; 2]) -> bool {
    let s = plan_area(c).signum();
    (0..3).all(|i| {
        let (a, b) = (c[i], c[(i + 1) % 3]);
        s * ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) >= -1e-12
    })
}

/// The plan-facing triangles, bucketed by the cells their plan bounds touch.
struct PlanIndex {
    x0: f64,
    y0: f64,
    nx: usize,
    ny: usize,
    start: Vec<u32>,
    items: Vec<u32>,
}

impl PlanIndex {
    fn of(soup: &Soup) -> PlanIndex {
        let ts: Vec<usize> = (0..soup.tris.len()).filter(|&t| soup.plan_facing(t)).collect();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for &t in &ts {
            for p in soup.corners(t) {
                for k in 0..2 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
        if ts.is_empty() {
            return PlanIndex { x0: 0.0, y0: 0.0, nx: 0, ny: 0, start: vec![0], items: vec![] };
        }
        let nx = ((hi[0] - lo[0]) / CELL_M).floor() as usize + 1;
        let ny = ((hi[1] - lo[1]) / CELL_M).floor() as usize + 1;
        let mut index = PlanIndex { x0: lo[0], y0: lo[1], nx, ny, start: vec![0; nx * ny + 1], items: vec![] };
        let spans: Vec<(usize, [usize; 4])> = ts.iter().map(|&t| (t, index.span(bounds(soup.corners(t))))).collect();
        for (_, [i0, i1, j0, j1]) in &spans {
            for j in *j0..=*j1 {
                for i in *i0..=*i1 {
                    index.start[j * nx + i + 1] += 1;
                }
            }
        }
        for c in 0..nx * ny {
            index.start[c + 1] += index.start[c];
        }
        let mut fill = index.start.clone();
        index.items = vec![0; index.start[nx * ny] as usize];
        for (t, [i0, i1, j0, j1]) in spans {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let c = j * nx + i;
                    index.items[fill[c] as usize] = t as u32;
                    fill[c] += 1;
                }
            }
        }
        index
    }

    /// The cells `[i0, i1, j0, j1]` a plan box `[x0, y0, x1, y1]` touches.
    fn span(&self, b: [f64; 4]) -> [usize; 4] {
        let cell = |v: f64, o: f64, n: usize| (((v - o) / CELL_M).floor().max(0.0) as usize).min(n - 1);
        [
            cell(b[0], self.x0, self.nx),
            cell(b[2], self.x0, self.nx),
            cell(b[1], self.y0, self.ny),
            cell(b[3], self.y0, self.ny),
        ]
    }

    fn cell(&self, c: usize) -> &[u32] {
        &self.items[self.start[c] as usize..self.start[c + 1] as usize]
    }

    /// The plan-facing triangles over the plan point `p`.
    fn over(&self, soup: &Soup, p: [f64; 2]) -> impl Iterator<Item = usize> + '_ {
        let cells: &[u32] = if self.nx == 0 || p[0] < self.x0 || p[1] < self.y0 {
            &[]
        } else {
            let (i, j) = (((p[0] - self.x0) / CELL_M) as usize, ((p[1] - self.y0) / CELL_M) as usize);
            if i >= self.nx || j >= self.ny { &[] } else { self.cell(j * self.nx + i) }
        };
        let corners: Vec<(usize, [[f64; 3]; 3])> = cells.iter().map(|&t| (t as usize, soup.corners(t as usize))).collect();
        corners.into_iter().filter(move |(_, c)| inside(*c, p)).map(|(t, _)| t)
    }
}

fn bounds(c: [[f64; 3]; 3]) -> [f64; 4] {
    [
        c[0][0].min(c[1][0]).min(c[2][0]),
        c[0][1].min(c[1][1]).min(c[2][1]),
        c[0][0].max(c[1][0]).max(c[2][0]),
        c[0][1].max(c[1][1]).max(c[2][1]),
    ]
}

/// The plan of triangle `a` clipped to triangle `b`'s (Sutherland–Hodgman),
/// both taken counter-clockwise.
fn clip(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> Vec<[f64; 2]> {
    let ccw = |c: [[f64; 3]; 3]| {
        let p = c.map(|q| [q[0], q[1]]);
        if plan_area(c) < 0.0 { [p[0], p[2], p[1]] } else { p }
    };
    let (a, b) = (ccw(a), ccw(b));
    let mut poly: Vec<[f64; 2]> = a.to_vec();
    for i in 0..3 {
        let (e0, e1) = (b[i], b[(i + 1) % 3]);
        let side = |p: [f64; 2]| (e1[0] - e0[0]) * (p[1] - e0[1]) - (e1[1] - e0[1]) * (p[0] - e0[0]);
        let mut out = Vec::with_capacity(poly.len() + 1);
        for k in 0..poly.len() {
            let (p, q) = (poly[k], poly[(k + 1) % poly.len()]);
            let (sp, sq) = (side(p), side(q));
            if sp >= 0.0 {
                out.push(p);
            }
            if (sp >= 0.0) != (sq >= 0.0) {
                let t = sp / (sp - sq);
                out.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])]);
            }
        }
        poly = out;
        if poly.len() < 3 {
            return Vec::new();
        }
    }
    poly
}

/// A polygon's area and centroid.
fn area_centroid(poly: &[[f64; 2]]) -> (f64, [f64; 2]) {
    let (mut a, mut cx, mut cy) = (0.0, 0.0, 0.0);
    let o = poly[0];
    for k in 1..poly.len() - 1 {
        let (p, q) = (poly[k], poly[k + 1]);
        let w = ((p[0] - o[0]) * (q[1] - o[1]) - (q[0] - o[0]) * (p[1] - o[1])) / 2.0;
        a += w;
        cx += w * (o[0] + p[0] + q[0]) / 3.0;
        cy += w * (o[1] + p[1] + q[1]) / 3.0;
    }
    if a.abs() < f64::MIN_POSITIVE { (0.0, o) } else { (a.abs(), [cx / a, cy / a]) }
}

/// A hash map on a multiplicative hash: the census hashes tens of millions
/// of small integer keys, and the standard library's is built to resist an
/// adversary this code does not have. Iteration order is never relied on.
type FxMap<K, V> = HashMap<K, V, std::hash::BuildHasherDefault<Fx>>;

#[derive(Default, Clone, Copy)]
struct Fx(u64);

impl std::hash::Hasher for Fx {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(8) {
            let mut word = [0u8; 8];
            word[..chunk.len()].copy_from_slice(chunk);
            self.0 = (self.0.rotate_left(5) ^ u64::from_le_bytes(word)).wrapping_mul(0x517c_c1b7_2722_0a95);
        }
    }
}

/// Union-find over `n` items, joined in a fixed order.
struct Sets(Vec<usize>);

impl Sets {
    fn new(n: usize) -> Sets {
        Sets((0..n).collect())
    }

    fn find(&mut self, i: usize) -> usize {
        let mut r = i;
        while self.0[r] != r {
            r = self.0[r];
        }
        let mut i = i;
        while self.0[i] != r {
            let next = self.0[i];
            self.0[i] = r;
            i = next;
        }
        r
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b)] = a.min(b);
        }
    }

    /// The groups, each in ascending order, ordered by their first item.
    fn groups(&mut self) -> Vec<Vec<usize>> {
        let mut by: HashMap<usize, Vec<usize>> = HashMap::new();
        for i in 0..self.0.len() {
            let r = self.find(i);
            by.entry(r).or_default().push(i);
        }
        let mut out: Vec<Vec<usize>> = by.into_values().collect();
        out.sort_by_key(|g| g[0]);
        out
    }
}

impl Census {
    /// The census of `world`'s drawn layers. Empty until the bench has run:
    /// before that there is no drawn ground to close anything onto.
    pub fn take(world: &World) -> Census {
        let b = world.extent.bbox;
        let mut census = Census { bbox: [b.west, b.south, b.east, b.north], ..Census::default() };
        if world.bench.is_none() {
            return census;
        }
        let soup = Soup::of(world);
        let index = PlanIndex::of(&soup);
        census.tally.triangles = soup.tris.len();
        census.tally.unfaced = (0..soup.tris.len())
            .filter(|&t| soup.layer[t] < 6 && soup.plan_facing(t) && soup.face[t].is_none())
            .count();
        let extent = world.extent;
        census.open_edges(&soup, &index, &extent);
        census.steep(world, &soup, &extent);
        census.overlaps(&soup, &index, &extent);
        let weight = |d: &Defect| if d.species == Species::Gap { d.size * d.width } else { d.size };
        census.defects.sort_by(|a, b| a.species.cmp(&b.species).then(weight(b).total_cmp(&weight(a))));
        census
    }

    /// The number of defects of `species` and their total size.
    pub fn total(&self, species: Species) -> (usize, f64) {
        self.defects.iter().filter(|d| d.species == species).fold((0, 0.0), |(n, s), d| (n + 1, s + d.size))
    }

    /// One line: every species as `count/size`, then the tallies.
    pub fn summary(&self) -> Summary {
        let mut out = Summary::new();
        for species in Species::ALL {
            let (n, size) = self.total(species);
            let unit = if species.is_length() { "m" } else { "m2" };
            out = out.with(species.name(), format!("{n}/{size:.3}{unit}"));
        }
        let t = &self.tally;
        out.with("triangles", t.triangles)
            .with("border", t.border)
            .with("hidden", t.hidden)
            .with("mouth", t.mouth)
            .with("nonmanifold", t.nonmanifold)
            .with("degenerate", t.degenerate)
            .with("specks", t.specks)
            .with("unfaced", t.unfaced)
            .with("stacked", format!("{}/{:.0}m2", t.stacked, t.stacked_m2))
    }

    /// `gap` and `crack`: the edges one triangle uses, less the border, the
    /// buried and the tubes' mouths, chained.
    fn open_edges(&mut self, soup: &Soup, index: &PlanIndex, extent: &Extent) {
        // Every edge once per triangle using it, undirected, sorted, so a
        // run of equal keys is one edge and its length the number of uses.
        let mut uses: Vec<([u32; 2], u32)> = Vec::with_capacity(soup.tris.len() * 3);
        for (t, tri) in soup.tris.iter().enumerate() {
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] || area3(soup.corners(t)) < DEGENERATE_M2 {
                self.tally.degenerate += 1;
                continue;
            }
            for i in 0..3 {
                let (a, b) = (tri[i], tri[(i + 1) % 3]);
                uses.push(([a.min(b), a.max(b)], t as u32));
            }
        }
        uses.sort_unstable();
        let rect = extent.rect;
        let on_border = |p: [f64; 3]| -> [bool; 4] {
            [
                (p[0] - rect.x0).abs() <= BORDER_M,
                (p[0] - rect.x1).abs() <= BORDER_M,
                (p[1] - rect.y0).abs() <= BORDER_M,
                (p[1] - rect.y1).abs() <= BORDER_M,
            ]
        };
        let mut open: Vec<([u32; 2], u32)> = Vec::new();
        let mut i = 0;
        while i < uses.len() {
            let mut j = i + 1;
            while j < uses.len() && uses[j].0 == uses[i].0 {
                j += 1;
            }
            match j - i {
                1 => open.push(uses[i]),
                2 => {}
                _ => self.tally.nonmanifold += 1,
            }
            i = j;
        }
        let mut charged: Vec<([u32; 2], u32)> = Vec::new();
        for (e, t) in open {
            let (p, q) = (soup.pos[e[0] as usize], soup.pos[e[1] as usize]);
            let (bp, bq) = (on_border(p), on_border(q));
            if (0..4).any(|k| bp[k] && bq[k]) {
                self.tally.border += 1;
                continue;
            }
            if soup.name(t as usize) == "bore" {
                self.tally.mouth += 1;
                continue;
            }
            let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0];
            let under_ground = index
                .over(soup, [mid[0], mid[1]])
                .filter(|&s| soup.on_earth(s))
                .any(|s| height_on(soup.corners(s), [mid[0], mid[1]]) > mid[2] + HIDDEN_M);
            if under_ground {
                self.tally.hidden += 1;
                continue;
            }
            charged.push((e, t));
        }
        // A crack: every sample of the edge within CRACK_M of a triangle
        // other than its own — the far side of a T-junction, or a surface
        // the edge rests on. Only the cells the samples fall in are indexed.
        let samples: Vec<Vec<[f64; 3]>> = charged
            .iter()
            .map(|(e, _)| {
                let (p, q) = (soup.pos[e[0] as usize], soup.pos[e[1] as usize]);
                let len = ((q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) + (q[2] - p[2]).powi(2)).sqrt();
                let n = ((len / SAMPLE_M).ceil() as usize).clamp(3, MAX_SAMPLES);
                (1..=n)
                    .map(|i| {
                        let s = i as f64 / (n + 1) as f64;
                        [p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1]), p[2] + s * (q[2] - p[2])]
                    })
                    .collect()
            })
            .collect();
        let near = Near::of(soup, &charged, samples.iter().flatten());
        let is_crack: Vec<bool> = (0..charged.len())
            .map(|k| samples[k].iter().all(|&x| near.touches(soup, charged[k].1, x, CRACK_M)))
            .collect();
        let opening: Vec<f64> = (0..charged.len())
            .map(|k| {
                if is_crack[k] {
                    return 0.0;
                }
                samples[k].iter().map(|&x| near.opening(soup, &charged, k, x)).fold(0.0, f64::max)
            })
            .collect();
        for (species, pick) in [(Species::Gap, false), (Species::Crack, true)] {
            let edges: Vec<usize> = (0..charged.len()).filter(|&k| is_crack[k] == pick).collect();
            let mut sets = Sets::new(edges.len());
            let mut by_vertex: FxMap<u32, usize> = FxMap::default();
            for (n, &k) in edges.iter().enumerate() {
                for v in charged[k].0 {
                    match by_vertex.get(&v) {
                        Some(&m) => sets.join(n, m),
                        None => {
                            by_vertex.insert(v, n);
                        }
                    }
                }
            }
            for group in sets.groups() {
                let ks: Vec<usize> = group.iter().map(|&n| edges[n]).collect();
                let len = |k: usize| {
                    let e = charged[k].0;
                    let (p, q) = (soup.pos[e[0] as usize], soup.pos[e[1] as usize]);
                    ((q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) + (q[2] - p[2]).powi(2)).sqrt()
                };
                let longest = *ks.iter().max_by(|&&a, &&b| len(a).total_cmp(&len(b))).expect("a chain has an edge");
                let e = charged[longest].0;
                let (p, q) = (soup.pos[e[0] as usize], soup.pos[e[1] as usize]);
                let zs = ks.iter().flat_map(|&k| charged[k].0.map(|v| soup.pos[v as usize][2]));
                let (lo, hi) = zs.fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), z| (l.min(z), h.max(z)));
                let tris: Vec<usize> = ks.iter().map(|&k| charged[k].1 as usize).collect();
                let length: f64 = ks.iter().map(|&k| len(k)).sum();
                let width = ks.iter().map(|&k| opening[k]).fold(0.0, f64::max);
                if species == Species::Gap && ks.iter().map(|&k| len(k) * opening[k]).sum::<f64>() < SPECK_M2 {
                    self.tally.specks += 1;
                    continue;
                }
                let at = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0];
                self.push(soup, extent, species, at, length, hi - lo, &tris);
                let d = self.defects.last_mut().expect("just pushed");
                d.width = width;
                d.lines = ks
                    .iter()
                    .take(MAX_SHAPE)
                    .map(|&k| charged[k].0.map(|v| soup.pos[v as usize]))
                    .collect();
            }
        }
    }

    /// `fin` and `flip`: paved triangles that stand up, plan-facing ones
    /// wound the wrong way. Chained by shared vertex.
    fn steep(&mut self, world: &World, soup: &Soup, extent: &Extent) {
        for species in [Species::Fin, Species::Flip] {
            let ts: Vec<usize> = (0..soup.tris.len())
                .filter(|&t| {
                    let c = soup.corners(t);
                    if area3(c) < DEGENERATE_M2 {
                        return false;
                    }
                    let n = normal(c);
                    match species {
                        Species::Fin => {
                            matches!(soup.role(t), Role::Paved | Role::Floor) && {
                                let s = n[0].hypot(n[1]);
                                s > FIN_GRADE * n[2].abs()
                                    && s > (natural_slope(world, c) + FIN_GRADE) * n[2].abs()
                            }
                        }
                        _ => soup.role(t) != Role::Other && plan_area(c) < -FLAT_M2,
                    }
                })
                .collect();
            let mut sets = Sets::new(ts.len());
            let mut by_vertex: FxMap<u32, usize> = FxMap::default();
            for (n, &t) in ts.iter().enumerate() {
                for v in soup.tris[t] {
                    match by_vertex.get(&v) {
                        Some(&m) => sets.join(n, m),
                        None => {
                            by_vertex.insert(v, n);
                        }
                    }
                }
            }
            for group in sets.groups() {
                let group: Vec<usize> = group.iter().map(|&n| ts[n]).collect();
                let largest = *group
                    .iter()
                    .max_by(|&&a, &&b| area3(soup.corners(a)).total_cmp(&area3(soup.corners(b))))
                    .expect("a group has a triangle");
                let c = soup.corners(largest);
                let zs = group.iter().flat_map(|&t| soup.corners(t).map(|p| p[2]));
                let (lo, hi) = zs.fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), z| (l.min(z), h.max(z)));
                let at = [0, 1, 2].map(|k| (c[0][k] + c[1][k] + c[2][k]) / 3.0);
                let area: f64 = group.iter().map(|&t| area3(soup.corners(t))).sum();
                if area < SPECK_M2 {
                    self.tally.specks += 1;
                    continue;
                }
                self.push(soup, extent, species, at, area, hi - lo, &group);
            }
        }
    }

    /// `fight`, `buried` and `overlap`: two plan-facing triangles over one
    /// point. Each pair is clipped once, in the cell holding the low corner
    /// of the overlap of their plan bounds.
    fn overlaps(&mut self, soup: &Soup, index: &PlanIndex, extent: &Extent) {
        let ground = soup.roles.iter().position(|&r| r == Role::Ground);
        let mut found: Vec<(Species, usize, usize, f64, [f64; 2], f64)> = Vec::new();
        for c in 0..index.nx * index.ny {
            let items = index.cell(c);
            for (i, &a) in items.iter().enumerate() {
                let (a, ca) = (a as usize, soup.corners(a as usize));
                let ba = bounds(ca);
                let va = soup.tris[a];
                for &b in &items[i + 1..] {
                    let b = b as usize;
                    // Two triangles of the one planar mesh sharing a vertex
                    // overlap only if one of them is folded over, and `flip`
                    // counts that. Two copies of one mesh triangle — the
                    // ground's and the paving's, welded where their heights
                    // agree — share a plan and overlap whole, so they are
                    // clipped, and so is anything a structure triangulated
                    // on its own; sharing all three vertices, two triangles
                    // are one drawn twice.
                    let shared = soup.tris[b].iter().filter(|v| va.contains(v)).count();
                    let cb = soup.corners(b);
                    let meshed = soup.face[a].is_some() && soup.face[b].is_some();
                    if (shared == 1 || shared == 2) && meshed && centroid_key(ca) != centroid_key(cb) {
                        continue;
                    }
                    let bb = bounds(cb);
                    let low = [ba[0].max(bb[0]), ba[1].max(bb[1])];
                    if low[0] > ba[2].min(bb[2]) || low[1] > ba[3].min(bb[3]) {
                        continue;
                    }
                    let [i0, _, j0, _] = index.span([low[0], low[1], low[0], low[1]]);
                    if j0 * index.nx + i0 != c {
                        continue;
                    }
                    let (area, at) = if shared == 3 {
                        (plan_area(ca).abs(), [0, 1].map(|k| (ca[0][k] + ca[1][k] + ca[2][k]) / 3.0))
                    } else {
                        let poly = clip(ca, cb);
                        if poly.is_empty() {
                            continue;
                        }
                        area_centroid(&poly)
                    };
                    if area < OVERLAP_M2 {
                        continue;
                    }
                    let (za, zb) = (height_on(ca, at), height_on(cb, at));
                    let dz = (za - zb).abs();
                    let is_ground = |t: usize| Some(soup.layer[t] as usize) == ground;
                    let partition = |t: usize| soup.face[t].is_some_and(|f| f < soup.partition);
                    let species = if dz < FIGHT_M {
                        Some(Species::Fight)
                    } else if (is_ground(a) && soup.role(b) == Role::Paved && za > zb)
                        || (is_ground(b) && soup.role(a) == Role::Paved && zb > za)
                    {
                        Some(Species::Buried)
                    } else if partition(a) && partition(b) && !is_ground(a) && !is_ground(b) {
                        Some(Species::Overlap)
                    } else {
                        None
                    };
                    match species {
                        Some(s) => found.push((s, a, b, area, at, dz)),
                        None => {
                            self.tally.stacked += 1;
                            self.tally.stacked_m2 += area;
                        }
                    }
                }
            }
        }
        // Pairs chained by shared triangle, per species, so one misplaced
        // patch is one defect however finely it was meshed.
        for species in [Species::Fight, Species::Buried, Species::Overlap] {
            let pairs: Vec<&(Species, usize, usize, f64, [f64; 2], f64)> =
                found.iter().filter(|f| f.0 == species).collect();
            let mut sets = Sets::new(pairs.len());
            let mut by_tri: FxMap<usize, usize> = FxMap::default();
            let mut by_vertex: FxMap<u32, usize> = FxMap::default();
            for (n, f) in pairs.iter().enumerate() {
                for t in [f.1, f.2] {
                    match by_tri.get(&t) {
                        Some(&m) => sets.join(n, m),
                        None => {
                            by_tri.insert(t, n);
                        }
                    }
                    for v in soup.tris[t] {
                        match by_vertex.get(&v) {
                            Some(&m) => sets.join(n, m),
                            None => {
                                by_vertex.insert(v, n);
                            }
                        }
                    }
                }
            }
            for group in sets.groups() {
                let largest = *group.iter().max_by(|&&a, &&b| pairs[a].3.total_cmp(&pairs[b].3)).expect("a pair");
                let (_, a, _, _, at, dz) = *pairs[largest];
                let z = height_on(soup.corners(a), at);
                let tris: Vec<usize> = group.iter().flat_map(|&n| [pairs[n].1, pairs[n].2]).collect();
                let area: f64 = group.iter().map(|&n| pairs[n].3).sum();
                if area < SPECK_M2 {
                    self.tally.specks += 1;
                    continue;
                }
                self.push(soup, extent, species, [at[0], at[1], z], area, dz, &tris);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        soup: &Soup,
        extent: &Extent,
        species: Species,
        at: [f64; 3],
        size: f64,
        rise: f64,
        tris: &[usize],
    ) {
        let mut layers: Vec<&'static str> = tris.iter().map(|&t| soup.name(t)).collect();
        layers.sort_unstable();
        layers.dedup();
        let mut faces: Vec<u32> = tris.iter().filter_map(|&t| soup.face[t]).collect();
        faces.sort_unstable();
        faces.dedup();
        let (lon, lat) = extent.frame.to_geo(at[0], at[1]);
        let triangles = if species.is_length() {
            Vec::new()
        } else {
            tris.iter().take(MAX_SHAPE).map(|&t| soup.corners(t)).collect()
        };
        self.defects.push(Defect {
            species,
            at,
            lon,
            lat,
            size,
            rise,
            width: 0.0,
            parts: tris.len(),
            layers,
            faces,
            lines: Vec::new(),
            triangles,
        });
    }
}

/// Every triangle and every charged open edge near some sample: indexed in
/// the cells around the samples' own, so a query reaches [`Near::CELL`]
/// in any direction.
struct Near {
    cells: FxMap<[i64; 2], (Vec<u32>, Vec<u32>)>,
}

impl Near {
    const CELL: f64 = 1.0;

    fn cell(x: f64, y: f64) -> [i64; 2] {
        [(x / Self::CELL).floor() as i64, (y / Self::CELL).floor() as i64]
    }

    fn of<'a>(soup: &Soup, edges: &[([u32; 2], u32)], samples: impl Iterator<Item = &'a [f64; 3]>) -> Near {
        let mut cells: FxMap<[i64; 2], (Vec<u32>, Vec<u32>)> = FxMap::default();
        for x in samples {
            let c = Self::cell(x[0], x[1]);
            for i in -1..=1 {
                for j in -1..=1 {
                    cells.entry([c[0] + i, c[1] + j]).or_default();
                }
            }
        }
        let mut put = |b: [f64; 4], item: u32, tri: bool| {
            let (lo, hi) = (Self::cell(b[0] - CRACK_M, b[1] - CRACK_M), Self::cell(b[2] + CRACK_M, b[3] + CRACK_M));
            for i in lo[0]..=hi[0] {
                for j in lo[1]..=hi[1] {
                    if let Some((ts, es)) = cells.get_mut(&[i, j]) {
                        if tri { ts.push(item) } else { es.push(item) }
                    }
                }
            }
        };
        for t in 0..soup.tris.len() {
            put(bounds(soup.corners(t)), t as u32, true);
        }
        for (k, (e, _)) in edges.iter().enumerate() {
            let (p, q) = (soup.pos[e[0] as usize], soup.pos[e[1] as usize]);
            put([p[0].min(q[0]), p[1].min(q[1]), p[0].max(q[0]), p[1].max(q[1])], k as u32, false);
        }
        Near { cells }
    }

    /// Whether a triangle other than `own` passes within `r` of `x`.
    fn touches(&self, soup: &Soup, own: u32, x: [f64; 3], r: f64) -> bool {
        self.cells.get(&Self::cell(x[0], x[1])).is_some_and(|(ts, _)| {
            ts.iter().any(|&t| t != own && dist2_to_triangle(x, soup.corners(t as usize)) <= r * r)
        })
    }

    /// How far `x`, on charged edge `k`, stands from the far side of its
    /// hole, capped at [`Near::CELL`]: the nearest open edge sharing no
    /// vertex with `k`, or the nearest triangle sharing none with `k`'s own.
    /// A lower bound, since the second ring of `k`'s own surface counts.
    fn opening(&self, soup: &Soup, edges: &[([u32; 2], u32)], k: usize, x: [f64; 3]) -> f64 {
        let (e, own) = edges[k];
        let mine = soup.tris[own as usize];
        let c = Self::cell(x[0], x[1]);
        let mut best = Self::CELL * Self::CELL;
        for i in -1..=1 {
            for j in -1..=1 {
                let Some((ts, es)) = self.cells.get(&[c[0] + i, c[1] + j]) else { continue };
                for &t in ts {
                    if soup.tris[t as usize].iter().any(|v| mine.contains(v)) {
                        continue;
                    }
                    best = best.min(dist2_to_triangle(x, soup.corners(t as usize)));
                }
                for &m in es {
                    let f = edges[m as usize].0;
                    if f.iter().any(|v| e.contains(v)) {
                        continue;
                    }
                    let d = point_segment(x, soup.pos[f[0] as usize], soup.pos[f[1] as usize]);
                    best = best.min(d * d);
                }
            }
        }
        best.sqrt()
    }
}

fn point_segment(x: [f64; 3], p: [f64; 3], q: [f64; 3]) -> f64 {
    let d = sub(q, p);
    let len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let t = if len2 > 0.0 {
        (((x[0] - p[0]) * d[0] + (x[1] - p[1]) * d[1] + (x[2] - p[2]) * d[2]) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let c = [p[0] + t * d[0], p[1] + t * d[1], p[2] + t * d[2]];
    let r = sub(x, c);
    (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt()
}

/// The squared distance from `p` to the triangle `c` (Ericson, *Real-Time
/// Collision Detection*, 5.1.5).
fn dist2_to_triangle(p: [f64; 3], c: [[f64; 3]; 3]) -> f64 {
    let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let at = |o: [f64; 3], u: [f64; 3], s: f64, v: [f64; 3], t: f64| {
        [o[0] + s * u[0] + t * v[0], o[1] + s * u[1] + t * v[1], o[2] + s * u[2] + t * v[2]]
    };
    let (a, b, cc) = (c[0], c[1], c[2]);
    let (ab, ac, ap) = (sub(b, a), sub(cc, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let closest = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let (d3, d4) = (dot(ab, bp), dot(ac, bp));
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                at(a, ab, d1 / (d1 - d3), ac, 0.0)
            } else {
                let cp = sub(p, cc);
                let (d5, d6) = (dot(ab, cp), dot(ac, cp));
                if d6 >= 0.0 && d5 <= d6 {
                    cc
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        at(a, ab, 0.0, ac, d2 / (d2 - d6))
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                            [b[0] + w * (cc[0] - b[0]), b[1] + w * (cc[1] - b[1]), b[2] + w * (cc[2] - b[2])]
                        } else {
                            let denom = 1.0 / (va + vb + vc);
                            at(a, ab, vb * denom, ac, vc * denom)
                        }
                    }
                }
            }
        }
    };
    let d = sub(p, closest);
    dot(d, d)
}

#[cfg(test)]
mod tests {
    //! The census on worlds built by hand, where every answer is known: the
    //! instrument checked before it is believed.

    use super::*;
    use crate::frame::Rect;
    use crate::world::Bench;
    use arpentry_server::project::Bounds;

    fn world(bench: Bench) -> World {
        let mut w = World::new(Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 });
        w.bench = Some(bench);
        w
    }

    /// The square of half-side `h` at the origin, counter-clockwise.
    fn square(h: f64) -> [[f64; 2]; 4] {
        [[-h, -h], [h, -h], [h, h], [-h, h]]
    }

    /// The rect's ground, flat at 0, with the square of half-side `h` cut
    /// out of it: four quads between the rect's edges and the square's.
    fn ground_around(r: Rect, h: f64) -> Tri {
        let outer = [[r.x0, r.y0], [r.x1, r.y0], [r.x1, r.y1], [r.x0, r.y1]];
        let inner = square(h);
        let mut t = Tri::default();
        for i in 0..4 {
            let j = (i + 1) % 4;
            let at = |p: [f64; 2]| [p[0], p[1], 0.0];
            t.quad([at(outer[i]), at(outer[j]), at(inner[j]), at(inner[i])]);
        }
        t
    }

    /// The square of half-side `h` paved flat at `z`, as two triangles.
    fn pad(h: f64, z: f64) -> Tri {
        let s = square(h).map(|p| [p[0], p[1], z]);
        let mut t = Tri::default();
        t.quad(s);
        t
    }

    /// The faces closing the square's rim from the ground up to `z`.
    fn walls(h: f64, z: f64) -> Tri {
        let s = square(h);
        let mut t = Tri::default();
        for i in 0..4 {
            let (p, q) = (s[i], s[(i + 1) % 4]);
            t.face([p[0], p[1], 0.0], [q[0], q[1], 0.0], [0.0, z], [0.0, z]);
        }
        t
    }

    fn closed(h: f64, z: f64) -> Bench {
        let r = World::new(Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 }).extent.rect;
        Bench { ground: ground_around(r, h), carriageway: pad(h, z), wall: walls(h, z), ..Bench::default() }
    }

    fn count(c: &Census, s: Species) -> usize {
        c.total(s).0
    }

    #[test]
    fn a_closed_world_has_nothing_to_charge() {
        let c = Census::take(&world(closed(5.0, 1.0)));
        for s in Species::ALL {
            assert_eq!(count(&c, s), 0, "{s:?}: {}", c.summary());
        }
        assert!(c.tally.border > 0, "the rect's own edges are open, and are the border: {}", c.summary());
    }

    #[test]
    fn a_step_nothing_closes_is_a_gap_as_wide_as_it_is_tall() {
        let mut b = closed(5.0, 0.5);
        b.wall = Tri::default();
        let c = Census::take(&world(b));
        // The ground's rim and the road's: two holes' worth of edges, each
        // a chain round the square, half a metre apart.
        assert_eq!(count(&c, Species::Gap), 2, "{}", c.summary());
        for d in c.defects.iter().filter(|d| d.species == Species::Gap) {
            assert!((d.size - 40.0).abs() < 1e-9, "each rim is 40 m: {d:?}");
            assert!((d.width - 0.5).abs() < 1e-9, "and opens by the step: {d:?}");
        }
    }

    #[test]
    fn a_t_junction_is_a_crack_not_a_gap() {
        let mut b = closed(5.0, 1.0);
        // The road's south edge split at its midpoint, the wall's not.
        let (m, s) = ([0.0, -5.0, 1.0], square(5.0).map(|p| [p[0], p[1], 1.0]));
        b.carriageway = Tri::default();
        b.carriageway.triangle([s[0], m, s[3]]);
        b.carriageway.triangle([m, s[1], s[2]]);
        b.carriageway.triangle([m, s[2], s[3]]);
        let c = Census::take(&world(b));
        assert_eq!(count(&c, Species::Gap), 0, "{}", c.summary());
        assert!(count(&c, Species::Crack) > 0, "{}", c.summary());
    }

    #[test]
    fn a_paved_triangle_standing_up_is_a_fin() {
        let mut b = closed(5.0, 1.0);
        b.carriageway.triangle([[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [1.0, 0.0, 3.0]]);
        let c = Census::take(&world(b));
        assert_eq!(count(&c, Species::Fin), 1, "{}", c.summary());
    }

    #[test]
    fn a_ground_triangle_wound_clockwise_is_a_flip() {
        let mut b = closed(5.0, 1.0);
        b.ground.indices.swap(1, 2);
        let c = Census::take(&world(b));
        assert_eq!(count(&c, Species::Flip), 1, "{}", c.summary());
    }

    /// Paving laid over ground nothing cut: a centimetre over it z-fights, a
    /// metre under it is buried, and a metre over it is a deck — stacked by
    /// design, and not charged.
    #[test]
    fn paving_over_uncut_ground_is_charged_by_how_far_it_stands() {
        let r = World::new(Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 }).extent.rect;
        let mut ground = Tri::default();
        ground.quad([[r.x0, r.y0, 0.0], [r.x1, r.y0, 0.0], [r.x1, r.y1, 0.0], [r.x0, r.y1, 0.0]]);
        let over = |z: f64| Census::take(&world(Bench { ground: ground.clone(), pavement: pad(5.0, z), ..Bench::default() }));
        let fight = over(0.01);
        assert_eq!(count(&fight, Species::Fight), 1, "{}", fight.summary());
        assert!((fight.total(Species::Fight).1 - 100.0).abs() < 1e-6, "{}", fight.summary());
        let buried = over(-1.0);
        assert_eq!(count(&buried, Species::Buried), 1, "{}", buried.summary());
        let deck = over(1.0);
        assert_eq!(count(&deck, Species::Fight) + count(&deck, Species::Buried), 0, "{}", deck.summary());
        assert!(deck.tally.stacked > 0, "{}", deck.summary());
    }

    #[test]
    fn a_needle_is_a_speck() {
        let mut b = closed(5.0, 1.0);
        b.carriageway.triangle([[0.0, 0.0, 1.0], [0.001, 0.0, 1.0], [0.0005, 0.0, 1.5]]);
        let c = Census::take(&world(b));
        assert_eq!(count(&c, Species::Fin), 0, "{}", c.summary());
        assert_eq!(c.tally.specks, 1, "{}", c.summary());
    }
}
