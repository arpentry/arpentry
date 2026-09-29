//! The copies: the one mesh's vertices, once per surface and rule that
//! reaches them.
//!
//! The mesh step triangulates the whole rect once ([`crate::world::Mesh`]),
//! so the paving and the ground share every boundary vertex by index. The
//! lift, the earthwork and the bench each work on copies of those vertices —
//! one per surface that reaches a vertex (the ground, each carriageway and
//! ballast sheet, the pavement's near and far halves) and per [`Rule`] that
//! answers it — and every seam between two surfaces is found by index rather
//! than by where it lies. It used to be two meshes built apart and matched
//! by position at the kernel's grid with an eight-cell search: `seam` and
//! `unmet` were that search's misses, 3.70 % and 1.28 % on the loop box, and
//! both read exactly zero now on every specimen.
//!
//! **A step is declared, not stumbled on.** A paved triangle takes one rule
//! at its centroid — which field, which axis, which stretch of it, and for
//! the far pavement the face or the drape — and all three of its corners are
//! answered by it; a vertex two triangles answer differently is two copies,
//! welded where they agree within a kerb's rise and split otherwise, and the
//! edge rule draws the face across the split (`split_m2`). It used to be
//! decided per vertex, so the switch fell inside whichever triangle
//! straddled it and was drawn as a stretched triangle that nothing closed:
//! 8 400 of them on the loop box.
//!
//! This module is the machinery the three steps share. Which of them builds
//! what is theirs: the lift makes the paved copies, the earthwork the
//! ground's, the bench the faces between them.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

use crate::field::{Field, Foot, PART_M};
use crate::poly::{self, Pt};
use crate::standard::{EARTHWORK_BATTER, KERB_RISE_M, MAX_BENCH_FACE_M, ROOM_REACH_M};
use crate::width::{self, Family};
use crate::world::{Arrangement, Face, Material, Mesh, Profile, Profiles, Sheets, Tri};

/// What the lift did to one family.
#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub vertices: usize,
    /// Walk vertices whose nearest road stands more than one bench face
    /// above the natural ground under the vertex itself: a pavement asked
    /// to follow a road it is not on.
    pub flown: usize,
    /// Vertices that took the road's height whole.
    pub lifted: usize,
    /// Vertices on a batter face between the room's reach and the ground.
    pub battered: usize,
    /// Vertices left where the mesh step put them: the free bands.
    pub draped: usize,
    /// Of those, the ones **no road answered for at all** — nothing within
    /// [`FIELD_LIMIT_M`] — as against the ones whose batter simply
    /// daylighted, which is the face doing its job. A free vertex keeps the
    /// raw DEM, and the ground around it has been benched.
    pub free: usize,
    /// Vertices standing more than one face ([`MAX_BENCH_FACE_M`]) off
    /// the ground: where the ground's answer must be a wall rather than
    /// a batter.
    pub walled: usize,
    /// The deepest a vertex was let into the ground, in metres.
    pub cut: f64,
    /// The highest a vertex was raised above it.
    pub fill: f64,
    pub edges: usize,
    /// Edges the field steps across.
    pub steps: usize,
    /// The largest of those steps, in metres.
    pub worst: f64,
    /// Where they are.
    pub at: Vec<[f64; 2]>,
    /// Edges as steep as a step with both ends on the raw DEM.
    pub dem_steep: usize,
    /// Edges as steep as a step that their rule is continuous along.
    pub steep: usize,
}

/// No copy: a vertex this surface does not reach, or a rule no axis answers.
pub const NONE: u32 = u32::MAX;

/// Which surface a triangle of the one mesh is drawn in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Surface {
    /// The engineered ground: every partition face that does not cut the
    /// terrain, the ground under a deck included.
    Ground,
    /// A carriageway sheet, by its index into [`Sheets::sheets`] —
    /// `u32::MAX` for paving no sheet claims — and the same for the ballast.
    Carriageway(u32),
    Ballast(u32),
    /// The pavement within the room's reach, and past it.
    Near,
    Far,
}

impl Surface {
    /// The paved surface `face` is drawn in, if it is paved.
    pub fn paved(face: &Face) -> Option<Surface> {
        let sheet = face.sheet.map_or(u32::MAX, |s| s as u32);
        match face.material {
            Material::Ground => None,
            Material::Carriageway => Some(Surface::Carriageway(sheet)),
            Material::Ballast => Some(Surface::Ballast(sheet)),
            Material::Pavement if face.near => Some(Surface::Near),
            Material::Pavement => Some(Surface::Far),
        }
    }
}

/// How one paved triangle's height is decided: which of its surface's two
/// fields answered, which axis of that field, and — for the pavement past
/// the room's reach — whether it stands on the face or drapes.
///
/// **A triangle takes one rule, at all three of its corners.** The height of
/// a paved point used to be decided per vertex: the nearest axis, the chord
/// or the ground, the face or the drape, whichever the vertex's own position
/// chose. Where two neighbouring vertices chose differently the triangle
/// between them was stretched across the switch — 8 400 of them on the loop
/// box, 80 % inside one material where no face was drawn to close them,
/// which is plan §3.3's "a discontinuity is an accident of where a positional
/// case-function changes branch". Now the triangle's centroid chooses, every
/// corner is answered by that choice, and a vertex two triangles answer
/// differently is two copies: welded where they agree, and a declared edge
/// with a face on it where they do not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Rule {
    /// The chords' field answered, rather than the ground's.
    pub chord: bool,
    /// The answering axis, or [`NONE`] where no axis reaches.
    pub axis: u32,
    /// Which stretch of that axis, in [`PART_M`] parts of its arc.
    pub part: i32,
    /// The far pavement drapes to the natural ground here rather than
    /// standing on the face.
    pub drape: bool,
}

impl Rule {
    pub const FREE: Rule = Rule { chord: false, axis: NONE, part: 0, drape: false };
}

/// The fields one paved surface is lifted by, and how.
pub struct Lift<'a> {
    pub grounded: &'a Field,
    pub chords: Option<&'a Field>,
    /// Where the paving is over a span, so the chords may answer.
    pub over: Option<&'a poly::Indexed>,
    /// What the surface stands above the road: a kerb for the pavement.
    pub rise: f64,
    /// The pavement: past the room's reach it stands on a face.
    pub walk: bool,
    /// Within the room's reach: the road's height outright.
    pub near: bool,
}

impl Lift<'_> {
    /// The rule at `p`, whose natural ground is `natural`.
    ///
    /// **And the mask is asked, not believed.** It is a boolean kernel's
    /// answer to "which paving is over a deck", and a kernel's answer has
    /// threads in it: at the Montreux overbridge two of them, 0.3 and 0.8 m²,
    /// lay seven metres from any chord in the middle of a junction, and every
    /// vertex they caught was answered by the chords' field — which reaches
    /// [`FIELD_LIMIT_M`] and clamps to the nearest station, so it handed back
    /// the chord's *end* height, a 2.4 m fin. So the chord answers only where
    /// it is **no further away than the ground the sheet also holds**: on a
    /// deck the chord is underfoot and the approach a span away, and in a
    /// sliver it is the other way round.
    pub fn rule(&self, p: Pt, natural: f64) -> Rule {
        let ask = |f: &Field| if f.is_empty() { None } else { f.at(p) };
        let grounded = ask(self.grounded);
        let chord = if self.over.is_some_and(|o| o.contains(p)) { self.chords.and_then(ask) } else { None }
            .filter(|c| grounded.is_none_or(|g| c.d <= g.d));
        let (foot, chord) = match (chord, grounded) {
            (Some(c), _) => (c, true),
            (None, Some(g)) => (g, false),
            (None, None) => return Rule::FREE,
        };
        // Past the reach, a band standing more than one face from the road
        // beside it is not that road's pavement at all and drapes.
        let drape = self.walk && !self.near && (natural - (foot.h + self.rise)).abs() > MAX_BENCH_FACE_M;
        Rule { chord, axis: foot.axis, part: (foot.s / PART_M).floor() as i32, drape }
    }

    /// The height `rule` gives `p`, and the foot it read there.
    ///
    /// The rule's own axis is asked even where another is nearer, and the
    /// face is not switched to a drape by the point's own drop: a corner is
    /// answered as its triangle was, which is what keeps the triangle whole.
    pub fn height(&self, p: Pt, natural: f64, rule: Rule) -> (f64, Option<Foot>) {
        if rule.axis == NONE {
            return (natural, None);
        }
        let field = if rule.chord { self.chords.unwrap_or(self.grounded) } else { self.grounded };
        let Some(foot) = field.on_axis(p, rule.axis, rule.part) else {
            return (natural, None);
        };
        let room_h = foot.h + self.rise;
        let h = if !self.walk || self.near {
            room_h
        } else if rule.drape {
            natural
        } else {
            let slack = (foot.d - foot.half_w - ROOM_REACH_M).max(0.0) / EARTHWORK_BATTER;
            room_h + (natural - room_h).clamp(-slack, slack)
        };
        (h, Some(foot))
    }

    /// What a vertex lifted by `rule` to `h` counts as.
    pub fn account(&self, stats: &mut Stats, rule: Rule, h: f64, natural: f64, foot: Option<Foot>) {
        stats.vertices += 1;
        let Some(foot) = foot else {
            stats.draped += 1;
            stats.free += 1;
            return;
        };
        // **A walk takes a road's height only where that road is on the
        // ground the walk is on**; one bench face over is the threshold.
        stats.flown += (self.walk && foot.h - natural > MAX_BENCH_FACE_M) as usize;
        if !self.walk || self.near {
            stats.lifted += 1;
        } else if rule.drape {
            // It follows the engineered ground, which is not the road's to
            // owe: no cut or fill of its own.
            stats.draped += 1;
            return;
        } else if h == natural {
            stats.draped += 1;
        } else {
            stats.battered += 1;
        }
        // **A deck is paved, not banked.** The standoff under a span is the
        // structure's, and counted here it reads as an embankment nobody
        // built.
        if rule.chord {
            return;
        }
        if (h - natural).abs() > MAX_BENCH_FACE_M {
            stats.walled += 1;
        }
        stats.cut = stats.cut.max(natural - h);
        stats.fill = stats.fill.max(h - natural);
    }
}

/// A paved surface and the rule its triangle took: what a vertex is copied
/// per.
pub type Key = (Surface, Rule);

/// One mesh of copies: the positions, and per copy the one-mesh vertex it is
/// a copy of, its key, and the natural ground under it.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Part {
    pub tri: Tri,
    pub of: Vec<u32>,
    pub key: Vec<Key>,
    pub natural: Vec<f64>,
    /// Per triangle, the key it was answered by.
    pub face_key: Vec<Key>,
}

impl Part {
    /// The copy of `v` under `key`, made the first time it is asked for.
    fn copy(&mut self, slot: &mut u32, pos: &[[f64; 3]], v: u32, key: Key) -> u32 {
        if *slot == NONE {
            let q = pos[v as usize];
            self.tri.positions.push(q);
            self.of.push(v);
            self.key.push(key);
            self.natural.push(q[2]);
            *slot = (self.tri.positions.len() - 1) as u32;
        }
        *slot
    }
}

/// The one mesh's vertices, copied once per surface and rule that reaches
/// them, and the four meshes the copies make.
///
/// **This is the whole of the seam.** A kerb vertex is on the carriageway
/// and on the pavement, and the two answer a kerb's rise apart; an outline
/// vertex is on the paving and on the ground; a vertex between two roads'
/// domains is answered by each. So a vertex cannot be one height — but it is
/// one *position*, one index of [`Mesh::tri`], and every copy of it is found
/// from that index rather than from where it lies.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Copies {
    pub carriageway: Part,
    pub ballast: Part,
    pub pavement: Part,
    pub ground: Part,
    /// Per one-mesh vertex, its copy in the ground.
    pub in_ground: Vec<u32>,
    /// The paved copies, by vertex and key: which part, and which copy.
    pub paved: HashMap<(u32, Key), u32>,
}

impl Copies {
    /// The paved copies: one per vertex, paved surface and rule, at the
    /// natural ground. The ground's are [`Copies::add_ground`]'s.
    pub fn new(mesh: &Mesh, arrangement: &Arrangement, rules: &[Rule]) -> Copies {
        let mut c = Copies::default();
        let pos = &mesh.tri.positions;
        for (i, (t, &f)) in mesh.tri.indices.chunks_exact(3).zip(&mesh.of_face).enumerate() {
            let face = arrangement.face(f);
            let Some(surface) = Surface::paved(face) else { continue };
            let key = (surface, rules[i]);
            for &v in t {
                let slot = c.paved.entry((v, key)).or_insert(NONE);
                let part = match surface {
                    Surface::Carriageway(_) => &mut c.carriageway,
                    Surface::Ballast(_) => &mut c.ballast,
                    _ => &mut c.pavement,
                };
                let id = part.copy(slot, pos, v, key);
                part.tri.indices.push(id);
            }
            match surface {
                Surface::Carriageway(_) => c.carriageway.face_key.push(key),
                Surface::Ballast(_) => c.ballast.face_key.push(key),
                _ => c.pavement.face_key.push(key),
            }
        }
        c
    }

    /// The ground's copies: one per vertex of every partition face that does
    /// not cut the terrain, at the natural ground.
    pub fn add_ground(&mut self, mesh: &Mesh, arrangement: &Arrangement) {
        self.ground = Part::default();
        self.in_ground = vec![NONE; mesh.tri.positions.len()];
        let pos = &mesh.tri.positions;
        for (t, &f) in mesh.tri.indices.chunks_exact(3).zip(&mesh.of_face) {
            if arrangement.in_partition(f) && !arrangement.face(f).cuts() {
                for &v in t {
                    let id = self.ground.copy(&mut self.in_ground[v as usize], pos, v, (Surface::Ground, Rule::FREE));
                    self.ground.tri.indices.push(id);
                }
            }
        }
    }

    pub fn part(&self, surface: Surface) -> &Part {
        match surface {
            Surface::Ground => &self.ground,
            Surface::Carriageway(_) => &self.carriageway,
            Surface::Ballast(_) => &self.ballast,
            Surface::Near | Surface::Far => &self.pavement,
        }
    }

    /// The height of `v`'s copy under `key`, as the parts now hold it.
    pub fn height(&self, v: u32, key: Key) -> Option<f64> {
        let id = if key.0 == Surface::Ground { self.in_ground[v as usize] } else { *self.paved.get(&(v, key))? };
        (id != NONE).then(|| self.part(key.0).tri.positions[id as usize][2])
    }

    /// **Welds the copies of one vertex in one surface that agree.**
    ///
    /// Two triangles either side of the line where two roads' domains meet
    /// answer their shared corner each by its own road. Where the two agree
    /// within a kerb's rise — two legs of one junction, whose blend is
    /// continuous across the line between them — they are one vertex again,
    /// at their mean: a slope that small across a triangle is not a step. Where they do not, they stay two, and the edge between
    /// them is a declared step with a face on it ([`edge_faces`]).
    ///
    /// Returns how many copies went.
    pub fn weld(&mut self) -> usize {
        let mut groups: std::collections::BTreeMap<(u32, Surface), Vec<(Rule, u32)>> = Default::default();
        for (&(v, (surface, rule)), &id) in &self.paved {
            groups.entry((v, surface)).or_default().push((rule, id));
        }
        let mut into: [HashMap<u32, u32>; 3] = Default::default();
        let which = |s: Surface| match s {
            Surface::Carriageway(_) => 0,
            Surface::Ballast(_) => 1,
            _ => 2,
        };
        let mut gone = 0usize;
        for ((_, surface), mut copies) in groups {
            if copies.len() < 2 {
                continue;
            }
            let k = which(surface);
            let part = match k {
                0 => &mut self.carriageway,
                1 => &mut self.ballast,
                _ => &mut self.pavement,
            };
            let z = |id: u32, part: &Part| part.tri.positions[id as usize][2];
            copies.sort_by(|a, b| z(a.1, part).total_cmp(&z(b.1, part)).then(a.0.cmp(&b.0)));
            // Runs whose neighbours agree within a kerb's rise, and whose
            // spread does too, are one.
            let mut start = 0;
            while start < copies.len() {
                let mut end = start + 1;
                while end < copies.len() && z(copies[end].1, part) - z(copies[start].1, part) <= KERB_RISE_M {
                    end += 1;
                }
                if end - start > 1 {
                    let run = &copies[start..end];
                    let mean = run.iter().map(|c| z(c.1, part)).sum::<f64>() / run.len() as f64;
                    let keep = run.iter().map(|c| c.1).min().expect("a run");
                    part.tri.positions[keep as usize][2] = mean;
                    for c in run.iter().filter(|c| c.1 != keep) {
                        into[k].insert(c.1, keep);
                        gone += 1;
                    }
                }
                start = end;
            }
        }
        for (k, part) in [&mut self.carriageway, &mut self.ballast, &mut self.pavement].into_iter().enumerate() {
            for i in part.tri.indices.iter_mut() {
                if let Some(&j) = into[k].get(i) {
                    *i = j;
                }
            }
        }
        for (&(_, (surface, _)), id) in self.paved.iter_mut() {
            if let Some(&j) = into[which(surface)].get(id) {
                *id = j;
            }
        }
        gone
    }
}

/// One edge of the one mesh where the partition's triangles change surface
/// or rule: the vertices, in the winding of the triangle on `a`'s side, the
/// two faces and the two keys.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Boundary {
    pub u: u32,
    pub v: u32,
    /// The faces either side, as [`Arrangement::face`] numbers them.
    pub a: u32,
    pub b: u32,
    pub ka: Option<Key>,
    pub kb: Option<Key>,
    /// Whether the far side is not there at all: a crack, or the rect's own
    /// edge.
    pub open: bool,
}

/// Every edge of the one mesh across which the partition's triangles change
/// what they are, found without building the whole mesh's adjacency.
///
/// Over the loop box the one mesh is 14 M triangles, and a table of all of
/// their edges is most of a gigabyte to find the few hundred thousand that
/// matter. An edge can only be a boundary if both its ends are vertices where
/// two keys meet, so only those are indexed.
pub fn boundaries(mesh: &Mesh, arrangement: &Arrangement, rules: &[Rule]) -> Vec<Boundary> {
    let key = |i: usize| {
        let face = arrangement.face(mesh.of_face[i]);
        (face.cuts(), Surface::paved(face).map(|s| (s, rules[i])))
    };
    let n = mesh.tri.positions.len();
    let mut first: Vec<Option<(bool, Option<Key>)>> = vec![None; n];
    let mut mixed = vec![false; n];
    let partition = |i: usize| arrangement.in_partition(mesh.of_face[i]);
    for (i, t) in mesh.tri.indices.chunks_exact(3).enumerate() {
        if !partition(i) {
            continue;
        }
        let k = key(i);
        for &v in t {
            match first[v as usize] {
                None => first[v as usize] = Some(k),
                Some(seen) if seen != k => mixed[v as usize] = true,
                Some(_) => {}
            }
        }
    }
    let mut edges: Vec<(u32, u32, u32)> = Vec::new();
    for (i, t) in mesh.tri.indices.chunks_exact(3).enumerate() {
        if !partition(i) {
            continue;
        }
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if mixed[a as usize] && mixed[b as usize] {
                edges.push((a.min(b), a.max(b), i as u32));
            }
        }
    }
    edges.sort_unstable();
    let winding = |i: u32, a: u32, b: u32| -> bool {
        let t = &mesh.tri.indices[3 * i as usize..3 * i as usize + 3];
        (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b)
    };
    let mut out = Vec::new();
    for group in edges.chunk_by(|x, y| (x.0, x.1) == (y.0, y.1)) {
        let (lo, hi, t0) = group[0];
        let (u, v) = if winding(t0, lo, hi) { (lo, hi) } else { (hi, lo) };
        let a = mesh.of_face[t0 as usize];
        let ka = key(t0 as usize).1;
        match group {
            [_] => out.push(Boundary { u, v, a, b: a, ka, kb: ka, open: true }),
            [_, (.., t1)] => {
                if key(t0 as usize) != key(*t1 as usize) {
                    let b = mesh.of_face[*t1 as usize];
                    out.push(Boundary { u, v, a, b, ka, kb: key(*t1 as usize).1, open: false });
                }
            }
            // Three triangles on one edge is not a partition; `crack` in the
            // mesh step's line is where that shows.
            _ => {}
        }
    }
    out
}

/// One height field per sheet of `family`, in the sheets' own order — the
/// index a face's `sheet` carries, so a copy's sheet id indexes this
/// directly.
///
/// A sheet names its axes as profile indices with an arc range, and the
/// ranges of one profile are gathered into one entry: a profile is one
/// axis, and [`Field::of_stations`] numbers axes by entry, so splitting a
/// way across two entries would make the blend treat it as two ways
/// meeting itself.
///
/// **Both the ground and the structure stations**, unlike
/// [`Field::grounded`]. A sheet that holds a span holds it because the
/// span shares a connector with the sheet's ground pieces, and the
/// profile is continuous through that connector — so the chord is the
/// honest answer over the deck, and asking the ground runs alone would
/// leave the deck to be answered by an approach eighteen metres away.
fn fields(sheets: &Sheets, family: Family, profiles: &Profiles) -> (Vec<Field>, Vec<Field>) {
    (of_axes(sheets, family, profiles, false), of_axes(sheets, family, profiles, true))
}

/// The height fields the paved copies are lifted by: built once by the lift
/// and read again by the earthwork, which asks the same rules of the same
/// fields along every edge it checks.
///
/// Two for what has no sheet — the roads' and the railways' ground axes. A
/// pavement is a road's cross-section and never a railway's; the railways'
/// whole field is here because a gallery's rim, which no paving reaches,
/// asks which of the two is nearer. And one per sheet of each family that
/// has sheets, of its ground axes and of its chords ([`fields`]).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Fields {
    pub roads: Field,
    pub rails: Field,
    pub car: Vec<Field>,
    pub car_over: Vec<Field>,
    pub beds: Vec<Field>,
    pub beds_over: Vec<Field>,
    /// What a copy no sheet claims is lifted by: nothing.
    empty: Field,
}

impl Fields {
    pub fn new(profiles: &Profiles, sheets: &Sheets) -> Fields {
        let rail = |p: &&Profile| width::family(&p.class) == Family::Rail;
        let (car, car_over) = fields(sheets, Family::Carriageway, profiles);
        let (beds, beds_over) = fields(sheets, Family::Rail, profiles);
        Fields {
            roads: Field::grounded(profiles.profiles.iter().filter(|p| !rail(p))),
            rails: Field::grounded(profiles.profiles.iter().filter(rail)),
            car,
            car_over,
            beds,
            beds_over,
            empty: Field::default(),
        }
    }

    /// How many axes the two unsheeted fields hold.
    pub fn axes(&self) -> usize {
        self.roads.len() + self.rails.len()
    }

    /// How `surface` is lifted. The asphalt is the road: it takes the whole
    /// of its own height wherever it reaches, and only the walk beside it is
    /// asked how far out it lies. `over` is the span mask a chord is believed
    /// within ([`Lift::rule`]); a caller asking the height of a rule already
    /// chosen does not need it.
    pub fn lift<'a>(&'a self, surface: Surface, over: Option<&'a poly::Indexed>) -> Lift<'a> {
        let sheeted = |fs: &'a [Field], k: u32| fs.get(k as usize).unwrap_or(&self.empty);
        match surface {
            Surface::Carriageway(k) => Lift {
                grounded: sheeted(&self.car, k),
                chords: Some(sheeted(&self.car_over, k)),
                over,
                rise: 0.0,
                walk: false,
                near: true,
            },
            Surface::Ballast(k) => Lift {
                grounded: sheeted(&self.beds, k),
                chords: Some(sheeted(&self.beds_over, k)),
                over,
                rise: 0.0,
                walk: false,
                near: true,
            },
            Surface::Near | Surface::Far => Lift {
                grounded: &self.roads,
                chords: None,
                over: None,
                rise: KERB_RISE_M,
                walk: true,
                near: surface == Surface::Near,
            },
            Surface::Ground => unreachable!("the ground is benched, not lifted"),
        }
    }
}

/// One field per sheet of [`Sheets::sheets`], indexed as the mesh's sheet
/// tags are; a sheet of another family has an empty one.
fn of_axes(sheets: &Sheets, family: Family, profiles: &Profiles, chords: bool) -> Vec<Field> {
    sheets
        .sheets
        .iter()
        .map(|sheet| {
            if sheet.family != family {
                return Field::default();
            }
            let mut ranges: BTreeMap<usize, Vec<(usize, usize)>> = BTreeMap::new();
            for &(profile, a0, a1) in if chords { &sheet.chords } else { &sheet.axes } {
                let Some(p) = profiles.profiles.get(profile) else {
                    continue;
                };
                // Inclusive at both ends: the station on a piece boundary is
                // the abutment and belongs to both sides, which is what
                // carries the field across it without a gap.
                let first = p.stations.iter().position(|st| st.s >= a0 - 1e-9);
                let last = p.stations.iter().rposition(|st| st.s <= a1 + 1e-9);
                if let (Some(k0), Some(k1)) = (first, last) {
                    if k0 <= k1 {
                        // **A station past each end.** The two fields have
                        // to overlap where they hand over, or the ground's
                        // axes stop short of the connector, only the
                        // chords' field has a joint to blend at, and the
                        // surface creases along the mask's edge. One
                        // station is `structure::runs_of`'s own abutment
                        // margin, read here for the same reason.
                        let last = p.stations.len() - 1;
                        ranges
                            .entry(profile)
                            .or_default()
                            .push((k0.saturating_sub(1), (k1 + 1).min(last)));
                    }
                }
            }
            Field::of_stations(ranges.into_iter().filter_map(|(i, r)| Some((profiles.profiles.get(i)?, r))))
        })
        .collect()
}
