//! Step 11: the bench — the room holds its height.
//!
//! The mesh step put every paved triangle on the raw ground, coplanar with
//! it. This step gives the room the height the profile solved.
//!
//! **The room takes the profile.** The height of a point of the room — the
//! carriageway, its kerb returns and the pavement together — is the profile
//! height at the perpendicular foot on the nearest carriageway axis, and
//! the pavement stands [`KERB_RISE_M`] above it. The road is therefore
//! level crosswise: a 5.5 m residential along the contour of a 30 % slope
//! is cut 0.825 m at its uphill kerb and filled 0.825 m at its downhill
//! one, exactly, because half its width times the slope is what the ground
//! does across it. A crossfall is a later refinement, and the earth those
//! two numbers name is what the ground has still to answer with.
//!
//! Only the *ground* pieces' axes are in the field. A deck's height is the
//! chord the profile solved and belongs to the structure step; were it in
//! the field, the road under a viaduct would take the viaduct's height
//! wherever the deck's axis happened to be the nearer one.
//!
//! **A cross-section reaches as far as the room does.** The pavement rides
//! the road while it is within [`ROOM_REACH_M`] of the asphalt's edge —
//! the reach the room step itself paves to — and a walk band farther out
//! than that samples the ground, as stratum D says a draped class must.
//! Neither half of that rule will do alone, and the specimen said so.
//! Asked per region, a 20 m footway that merely touches a kerb at one end
//! was dragged 5.88 m up the ramp with the road at its far end. Asked per
//! point with nothing in between, it would stand on a 3.8 m cliff at the
//! line where the answer changed. So past the reach the walk comes down a
//! face at [`EARTHWORK_BATTER`] and stops exactly where it meets the
//! ground — which is the batter of this step's second half, carried by
//! the walk surface because the terrain cannot carry it yet. A fixed band
//! was tried first and rejected: 6 m of it on the box's 30 % flank came
//! out at 103 %, steeper than the wall it was there to avoid, and the
//! `step` check said so before the eye did.
//!
//! **Where two carriageways' domains meet at different heights** — two
//! terraces on a flank, two one-way carriageways across a slope — the
//! field steps on the line between them, and the mesh draws a face there:
//! a retaining wall, which is what the hillside physically has. The
//! alternative, a blend between the two, would ramp a pavement at 60 %
//! between two terraces, which is spectacle (invariant 6). The `step`
//! check counts those edges and the plan view marks them.
//!
//! **What this step does not do yet.** The ground does not answer: the
//! terrain is still the raw lattice, so the room now stands in the air on
//! its fill side and inside the hill on its cut side, and nothing has been
//! benched or walled, and the batter the walk comes down is a surface of
//! the walk's own rather than earth. The hole in the terrain, the real
//! batter and the `contact`, `batter`, `untouched` and `walled` checks
//! are the second
//! half of this step (`data/plans/surface-leaves-the-plane-2026-09-08.md`
//! §3, step 11); `cut` and `fill` here are exactly the earthwork it will
//! have to move.

use std::collections::HashMap;
use std::collections::HashSet;

use crate::poly::{self, Pt};
use crate::step::Summary;
use crate::world::{Bench, Kind, Profile, Tri, World};

/// How far, in metres, the pavement stands above the carriageway beside
/// it: one kerb face (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5).
pub const KERB_RISE_M: f64 = 0.12;

/// How far past the asphalt's edge, in metres, a carriageway's
/// cross-section carries the pavement level with it. It is the room
/// step's own `WALL_REACH_M`: what that step paves from a kerb — the
/// band, the rungs to a facade, a mapped sidewalk beside them — is what
/// this one lifts with the road, and nothing built there is left behind.
/// It is a *plateau*, so it is not free: on a 30 % flank every metre of
/// it is another metre of bench to cut, and 10 m of it left a wall at
/// its edge where 6 m leaves a face.
pub const ROOM_REACH_M: f64 = 6.0;

/// The slope of an earthwork face, as run over rise: 1 in 2.5
/// (`data/plans/surface-leaves-the-plane-2026-09-08.md` §5). Past the
/// room's reach the walk comes down a face of exactly this slope, so it
/// is as wide as the drop it has to close and no wider. A band of fixed
/// width was tried first and rejected: 6 m of it on the box's 30 % flank
/// came out at 103 %, steeper than the wall it was there to avoid, and
/// the `step` check said so before the eye did.
pub const EARTHWORK_BATTER: f64 = 2.5;

/// The tallest earthwork face, in metres, before the bench is walled at
/// its edge instead (`data/plans/surface-leaves-the-plane-2026-09-08.md`
/// §5). It does two things here. A walk band past the room's reach that
/// stands more than one face from the road beside it is not that road's
/// pavement and drapes — without that test a face on the box's flank
/// never daylights at all, because the mountain rises faster than 1 in
/// 2.5, and the first run over the loop box read 225 m of cut where the
/// field had carried a road's height half a kilometre up the hillside.
/// And a vertex of the room itself standing further than this from the
/// ground is counted as `walled`: the ground's answer there is a wall,
/// not a batter.
pub const MAX_BENCH_FACE_M: f64 = 3.0;

/// Slack, in metres, on the kerb's own rise before an edge counts as a
/// step. The rise arrives as a sum of floats and lands a hair either
/// side of itself: without the slack, a flat ground reported two steps,
/// and both were an edge between a lifted pavement and the free band
/// beside it — the kerb, exactly, and no more.
pub const STEP_SLACK_M: f64 = 1e-6;

/// A mesh edge whose ends differ in height by more than [`KERB_RISE_M`]
/// *and* by more than this many metres per metre of its own length is a
/// step: the field is discontinuous across it. Both conditions are needed
/// and neither alone would do. A street on the box's flank climbs 30 %, so
/// a plain gradient test would call every one of its edges a step; a kerb
/// return a centimetre across can drop a metre, so a plain height test
/// would miss nothing but would also flag a long edge on a steep street.
/// One metre per metre is steeper than any surface this world drives on
/// and shallower than any wall it builds.
pub const STEP_GRADE: f64 = 1.0;

/// Cell size of the axis index, in metres. A station is at most
/// `NODE_M` (4 m) from the next, so a cell holds a few segments of each
/// axis crossing it.
const CELL_M: f64 = 16.0;

/// The room's height field: the solved profile of every carriageway axis
/// on the ground, indexed for the nearest-axis query every vertex makes.
///
/// The nearest axis is the road whose cross-section the point rides. Inside
/// a piece's own ribbon that is the piece's own axis, since no other axis
/// comes within its half-width without their ribbons overlapping; in the
/// pavement and the kerb returns it is the nearest road, which is the one
/// the pavement belongs to.
#[derive(Debug, Default)]
pub struct Field {
    seg: Vec<Seg>,
    cells: HashMap<(i32, i32), Vec<u32>>,
    /// The occupied cells' bounds, so a query knows when it has searched
    /// everything there is.
    span: Option<(i32, i32, i32, i32)>,
}

/// One station-to-station piece of an axis: its ends, their solved
/// heights, and the half-width of the road it belongs to.
#[derive(Debug, Clone, Copy)]
struct Seg {
    a: Pt,
    b: Pt,
    ha: f64,
    hb: f64,
    half_w: f64,
}

/// What the nearest axis says about a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Foot {
    /// The road surface's height at the perpendicular foot.
    pub h: f64,
    /// How far the axis runs from the point, in metres.
    pub d: f64,
    /// Half the width of that road: where its asphalt ends.
    pub half_w: f64,
}

impl Foot {
    /// The height of the walk at this foot: `room_h` — the road's height
    /// plus the kerb — while the point is within the room's reach, and
    /// past it a face at [`EARTHWORK_BATTER`] descending to `ground` and
    /// stopping exactly where it meets it.
    ///
    /// The face is a function of the point alone, which is what makes it
    /// cheap and continuous: at `d` metres out it may have closed
    /// `(d − reach) / 2.5` of the difference and no more, so it is the
    /// ground wherever it has daylighted and the room's height wherever
    /// it has not left the reach.
    pub fn batter(&self, room_h: f64, ground: f64) -> f64 {
        let out = self.d - self.half_w - ROOM_REACH_M;
        if out <= 0.0 {
            return room_h;
        }
        // Past the reach the walk joins the road's bench only where a
        // face could reach it. A band standing more than one face from
        // the road is not that road's pavement at all — on the box's
        // flank a footway 20 m out and 21 m below a switchback was
        // otherwise hauled 18 m into the air by it — and it samples the
        // ground, whatever the road above it is doing.
        let drop = ground - room_h;
        if drop.abs() > MAX_BENCH_FACE_M {
            return ground;
        }
        let slack = out / EARTHWORK_BATTER;
        room_h + drop.clamp(-slack, slack)
    }
}

impl Field {
    /// The field of `profiles`: the ground pieces' stations, in order.
    pub fn new(profiles: &[Profile]) -> Field {
        let mut f = Field::default();
        for p in profiles.iter().filter(|p| p.mapped == Kind::Ground) {
            let half_w = p.width_m / 2.0;
            for w in p.stations.windows(2) {
                f.push(w[0].p, w[1].p, w[0].h, w[1].h, half_w);
            }
            if p.stations.len() == 1 {
                let st = p.stations[0];
                f.push(st.p, st.p, st.h, st.h, half_w);
            }
        }
        f
    }

    fn push(&mut self, a: Pt, b: Pt, ha: f64, hb: f64, half_w: f64) {
        let i = self.seg.len() as u32;
        self.seg.push(Seg { a, b, ha, hb, half_w });
        let box_ = [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])];
        for cell in poly::cells_over(box_, CELL_M) {
            self.cells.entry(cell).or_default().push(i);
            self.span = Some(match self.span {
                None => (cell.0, cell.1, cell.0, cell.1),
                Some((c0, r0, c1, r1)) => (c0.min(cell.0), r0.min(cell.1), c1.max(cell.0), r1.max(cell.1)),
            });
        }
    }

    /// How many axis pieces the field holds.
    pub fn len(&self) -> usize {
        self.seg.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seg.is_empty()
    }

    /// What the nearest carriageway axis says about `p`. `None` if the
    /// field is empty.
    ///
    /// Cells are searched in rings about `p`'s own, and the search stops
    /// when the nearest axis found is closer than the ring's own distance:
    /// everything outside a ring of `k` cells is at least `k` cells away,
    /// so nothing nearer can be left unlooked at.
    pub fn at(&self, p: Pt) -> Option<Foot> {
        let (c0, r0) = poly::cell_of(p, CELL_M);
        let (bc0, br0, bc1, br1) = self.span?;
        let rings = (c0 - bc0).abs().max((bc1 - c0).abs()).max((r0 - br0).abs()).max((br1 - r0).abs());
        let mut best: Option<Foot> = None;
        for k in 0..=rings {
            for (c, r) in ring(c0, r0, k) {
                let Some(ids) = self.cells.get(&(c, r)) else {
                    continue;
                };
                for &i in ids {
                    let seg = self.seg[i as usize];
                    let f = poly::nearest_on_segment(seg.a, seg.b, p);
                    let d = (p[0] - f[0]).hypot(p[1] - f[1]);
                    if best.is_some_and(|b| d >= b.d) {
                        continue;
                    }
                    let len2 = (seg.b[0] - seg.a[0]).powi(2) + (seg.b[1] - seg.a[1]).powi(2);
                    let t = if len2 > 0.0 {
                        ((f[0] - seg.a[0]) * (seg.b[0] - seg.a[0]) + (f[1] - seg.a[1]) * (seg.b[1] - seg.a[1])) / len2
                    } else {
                        0.0
                    };
                    best = Some(Foot { h: seg.ha + (seg.hb - seg.ha) * t, d, half_w: seg.half_w });
                }
            }
            if best.is_some_and(|b| b.d <= k as f64 * CELL_M) {
                break;
            }
        }
        best
    }
}

/// The cells exactly `k` away from `(c0, r0)` in the Chebyshev metric, in
/// a fixed order: the query's answer must not depend on a hash order.
fn ring(c0: i32, r0: i32, k: i32) -> Vec<(i32, i32)> {
    if k == 0 {
        return vec![(c0, r0)];
    }
    let mut out = Vec::with_capacity(8 * k as usize);
    for c in c0 - k..=c0 + k {
        for r in r0 - k..=r0 + k {
            if c == c0 - k || c == c0 + k || r == r0 - k || r == r0 + k {
                out.push((c, r));
            }
        }
    }
    out
}

/// What the lift did to one family.
#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub vertices: usize,
    /// Vertices that took the road's height whole.
    pub lifted: usize,
    /// Vertices on a batter face between the room's reach and the ground.
    pub battered: usize,
    /// Vertices left where the mesh step put them: the free bands.
    pub draped: usize,
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
}

impl Stats {
    fn merge(&mut self, o: &Stats) {
        self.vertices += o.vertices;
        self.lifted += o.lifted;
        self.battered += o.battered;
        self.draped += o.draped;
        self.walled += o.walled;
        self.cut = self.cut.max(o.cut);
        self.fill = self.fill.max(o.fill);
        self.edges += o.edges;
        self.steps += o.steps;
        self.worst = self.worst.max(o.worst);
        self.at.extend_from_slice(&o.at);
    }
}

/// Lifts the world's room onto the profile.
pub fn run(world: &mut World) -> Summary {
    let (bench, stats, axes) = {
        let profiles = world.profile.as_ref().expect("the profile step runs first");
        let mesh = world.mesh.as_ref().expect("the mesh step runs first");
        let field = Field::new(&profiles.profiles);
        // The asphalt is the road: it takes the whole of its own height
        // wherever it reaches. Only the walk beside it is asked how far
        // out it lies.
        let (c, cs) = lift(&mesh.carriageway, &field, 0.0, false);
        let (p, ps) = lift(&mesh.pavement, &field, KERB_RISE_M, true);
        let mut stats = Stats::default();
        stats.merge(&cs);
        stats.merge(&ps);
        let steps = std::mem::take(&mut stats.at);
        (Bench { carriageway: c, pavement: p, steps }, stats, field.len())
    };
    let summary = Summary::new()
        .with("axes", axes)
        .with_part("lifted", stats.lifted, stats.vertices)
        .with("battered", stats.battered)
        .with("draped", stats.draped)
        .with_part("walled", stats.walled, stats.vertices)
        .with("cut", format!("{:.3}", stats.cut))
        .with("fill", format!("{:.3}", stats.fill))
        .with_share("step", stats.steps, stats.edges)
        .with("worst", format!("{:.3}", stats.worst));
    world.bench = Some(bench);
    summary
}

/// `tri` at the field's height plus `rise`, and what that did. The
/// asphalt (`walk` false) takes the road's height wherever it reaches,
/// being the road; the walk beside it is asked how far out it lies, and
/// past the room's reach stands on [`Foot::batter`]'s face instead.
///
/// The mesh step left every vertex at the ground, so the ground a vertex
/// was cut from or filled to is the height it arrives with: this reads it
/// there rather than sampling the terrain again, and the two cannot drift
/// apart.
fn lift(tri: &Tri, field: &Field, rise: f64, walk: bool) -> (Tri, Stats) {
    let mut out = tri.clone();
    let mut stats = Stats { vertices: tri.positions.len(), ..Stats::default() };
    for (i, p) in tri.positions.iter().enumerate() {
        let Some(foot) = field.at([p[0], p[1]]) else {
            stats.draped += 1;
            continue;
        };
        let room_h = foot.h + rise;
        let h = if walk { foot.batter(room_h, p[2]) } else { room_h };
        out.positions[i][2] = h;
        // Where the vertex stands, not whether the height moved: on flat
        // ground the room's height *is* the ground, and a road there is
        // still a road.
        if !walk || foot.d <= foot.half_w + ROOM_REACH_M {
            stats.lifted += 1;
        } else if h == p[2] {
            stats.draped += 1;
        } else {
            stats.battered += 1;
        }
        if (h - p[2]).abs() > MAX_BENCH_FACE_M {
            stats.walled += 1;
        }
        stats.cut = stats.cut.max(p[2] - h);
        stats.fill = stats.fill.max(h - p[2]);
    }
    let mut seen: HashSet<(u32, u32)> = HashSet::new();
    for t in out.indices.chunks_exact(3) {
        for e in 0..3 {
            let (a, b) = (t[e], t[(e + 1) % 3]);
            if !seen.insert((a.min(b), a.max(b))) {
                continue;
            }
            stats.edges += 1;
            let (p, q) = (out.positions[a as usize], out.positions[b as usize]);
            let dh = (q[2] - p[2]).abs();
            let len = (q[0] - p[0]).hypot(q[1] - p[1]);
            if dh > KERB_RISE_M + STEP_SLACK_M && dh > STEP_GRADE * len {
                stats.steps += 1;
                stats.worst = stats.worst.max(dh);
                stats.at.push([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0]);
            }
        }
    }
    (out, stats)
}

#[cfg(test)]
pub(crate) mod tests {
    use std::path::Path;

    use crate::terrain::{self, tests::dem};
    use crate::world::Solved;
    use crate::{drape, facade, fillet, kerb, mesh, profile, ribbon, room, surface};

    use super::*;

    /// A world on `ground` with the network of `net` and the buildings of
    /// `houses`, meshed and benched.
    pub(crate) fn world(ground: &str, net: &str, houses: Option<&str>) -> (World, Summary) {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem(ground), 5.0, usize::MAX);
        drape::run(&mut w, Path::new(net)).unwrap();
        profile::run(&mut w);
        facade::run(&mut w, houses.map(Path::new)).unwrap();
        ribbon::run(&mut w);
        surface::run(&mut w);
        kerb::run(&mut w);
        fillet::run(&mut w);
        room::run(&mut w);
        mesh::run(&mut w);
        let s = run(&mut w);
        (w, s)
    }

    fn bench(w: &World) -> &Bench {
        w.bench.as_ref().unwrap()
    }

    #[test]
    fn the_field_is_the_profile_at_the_foot() {
        // One axis along x, rising 1 m per 10 m, stationed every 10 m.
        let stations: Vec<crate::world::Station> = (0..=10)
            .map(|k| {
                let x = k as f64 * 10.0;
                crate::world::Station { s: x, p: [x, 0.0], ground: 400.0, h: 400.0 + x / 10.0, solved: Solved::Grade }
            })
            .collect();
        let p = Profile {
            id: "road".into(),
            class: "residential".into(),
            width_m: 5.5,
            mapped: Kind::Ground,
            stations,
        };
        let f = Field::new(std::slice::from_ref(&p));
        assert_eq!(f.len(), 10);
        // On the axis, beside it, and past its end: the foot's height, and
        // the perpendicular distance to it.
        for (q, want_h, want_d) in
            [([25.0, 0.0], 402.5, 0.0), ([25.0, 7.0], 402.5, 7.0), ([25.0, -3.0], 402.5, 3.0), ([130.0, 0.0], 410.0, 30.0)]
        {
            let foot = f.at(q).unwrap();
            assert!((foot.h - want_h).abs() < 1e-9 && (foot.d - want_d).abs() < 1e-9, "{q:?}: {foot:?}");
            assert_eq!(foot.half_w, 2.75);
        }
        // Past the room's reach the walk comes down a face at 1 in 2.5
        // and stops where it meets the ground: a 4 m drop daylights 10 m
        // out, a 1 m drop 2.5 m out, and inside the reach there is no
        // face at all.
        let reach = 2.75 + ROOM_REACH_M;
        let batter = |d: f64, ground: f64| Foot { h: 0.0, d, half_w: 2.75 }.batter(0.0, ground);
        assert_eq!(batter(0.0, 2.0), 0.0);
        assert_eq!(batter(reach, 2.0), 0.0);
        assert_eq!(batter(reach + 2.5, 2.0), 1.0);
        assert_eq!(batter(reach + 5.0, 2.0), 2.0);
        assert_eq!(batter(reach + 30.0, 2.0), 2.0, "daylighted, and the ground beyond");
        assert_eq!(batter(reach + 2.5, -2.0), -1.0, "the fill side is the mirror");
        assert_eq!(batter(reach + 0.001, 0.0), 0.0, "nothing to close, no face");
        // A difference no face may close is not this road's to close: the
        // band is free and takes the ground.
        assert_eq!(batter(reach + 2.5, MAX_BENCH_FACE_M + 0.5), MAX_BENCH_FACE_M + 0.5);
        // A span is not in the field: nothing but ground pieces sets a
        // height the surface reads.
        let mut deck = p.clone();
        deck.mapped = Kind::Bridge(1);
        assert!(Field::new(std::slice::from_ref(&deck)).at([0.0, 0.0]).is_none());
    }

    #[test]
    fn flat_ground_moves_nothing() {
        let (w, s) = world("flat", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "{s}");
        assert_eq!(s.num("cut"), 0.0);
        assert_eq!(s.num("fill"), 0.0);
        assert_eq!(s.num("step"), 0.0);
        assert_eq!(s.num("lifted"), b.carriageway.positions.len() as f64);
    }

    #[test]
    fn a_road_along_the_contour_is_level_crosswise() {
        // The specimen of the plan: a 5.5 m residential across a 30 %
        // slope. Its axis is level, so the road is level, and the ground
        // it stands in is half the width times the slope — 0.825 m of cut
        // at the uphill kerb and 0.825 m of fill at the downhill one,
        // exactly. That is the earth the ground has still to move.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:straight?len=200", None);
        let b = bench(&w);
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9), "the road is not level");
        assert!((s.num("cut") - 0.825).abs() < 1e-6, "{s}");
        assert!((s.num("fill") - 0.825).abs() < 1e-6, "{s}");
        assert_eq!(s.num("step"), 0.0, "{s}");
        // The road reaches both kerbs: the two numbers above are the
        // ground at the edges of the asphalt, not at some point inside it.
        assert!(b.carriageway.positions.iter().any(|p| (p[1] - 2.75).abs() < 1e-9));
        assert!(b.carriageway.positions.iter().any(|p| (p[1] + 2.75).abs() < 1e-9));
    }

    #[test]
    fn a_road_up_the_slope_is_the_slope() {
        // The same 30 % ramp with the road running straight up it: a
        // street is not grade-limited, so its profile is the ground and
        // the bench moves nothing at all (S9).
        let (w, s) = world("ramp?grade=0.3&bearing=90&radius=100000", "net:straight?len=200", None);
        assert!(s.num("cut") < 1e-9 && s.num("fill") < 1e-9, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().unwrap();
        for p in &b.carriageway.positions {
            assert!((p[2] - crate::terrain::height_at(t, p[0], p[1])).abs() < 1e-9, "{p:?}");
        }
    }

    #[test]
    fn the_pavement_stands_a_kerb_above_the_road() {
        let (w, s) = world("flat", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(!b.pavement.positions.is_empty());
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9), "{s}");
        assert!(b.carriageway.positions.iter().all(|p| (p[2] - 400.0).abs() < 1e-9));
        assert_eq!(s.num("draped"), 0.0, "a sidewalk is pavement, not a free band: {s}");
    }

    #[test]
    fn the_pavement_rides_the_road_it_is_beside() {
        // On the contour, the sidewalk 6 m off the axis is level with the
        // road plus the kerb — not on the hillside it lies on, which at
        // 6 m out stands 1.8 m higher.
        let (w, _) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:sidewalk?d=6", None);
        let b = bench(&w);
        assert!(b.pavement.positions.iter().all(|p| (p[2] - 400.12).abs() < 1e-9));
        let m = w.mesh.as_ref().unwrap();
        assert!(m.pavement.positions.iter().any(|p| p[2] > 401.5), "the ground under it climbs");
    }

    #[test]
    fn a_footway_leaving_the_room_leaves_it_over_one_face() {
        // The stub footway runs 20 m north from the kerb, up a 30 %
        // slope. Its first metres are pavement and ride the road; past
        // the room's reach it comes down a face at 1 in 2.5; and where
        // the hill has climbed further than one face can follow, the band
        // is no longer this road's pavement and takes the ground.
        //
        // That last transition is a wall, and the numbers of it are the
        // measurement. The room's plateau ends 8.75 m out, where the
        // ground stands 2.51 m over the road. The face closes 0.4 m of
        // the difference per metre and the hill opens 0.3 m of it, so
        // they never meet: at 10.4 m out the difference reaches the
        // 3 m a face may be and the band steps to the ground, 2.34 m of
        // wall in the continuum and 3.53 m across the mesh edge that
        // straddles it. It is counted, it is marked in the plan view, and
        // the ground's answer — a batter cut into the hill, in this
        // step's second half — is what removes it.
        let (w, s) = world("ramp?grade=0.3&bearing=0&radius=100000", "net:stub?d=0.5", None);
        let b = bench(&w);
        assert!(s.num("lifted") > 0.0 && s.num("draped") > 0.0, "{s}");
        // Bounded: the plateau is 2.51 m of cut and nothing else is.
        assert!((s.num("cut") - 2.481).abs() < 0.01, "{s}");
        // Where asking per region dragged the far end of this same
        // footway 5.88 m into the air.
        assert!(s.num("cut") < 3.0, "{s}");
        assert!(s.num("step") > 0.0 && s.num("worst") < 4.0, "{s}");
        // The wall stands where the hill outruns the face, not at the
        // kerb: every step is out beyond the plateau.
        assert!(b.steps.iter().all(|p| p[1] > 2.75 + ROOM_REACH_M), "{:?}", b.steps);
    }

    #[test]
    fn a_junction_on_a_hill_has_one_height() {
        // Four legs meeting at the origin: the profile pins the connector,
        // so the four surfaces meet there without a step, and the field is
        // continuous across the whole junction.
        let (w, s) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        assert_eq!(s.num("step"), 0.0, "{s}");
        let b = bench(&w);
        let t = w.terrain.as_ref().unwrap();
        let centre: Vec<f64> =
            b.carriageway.positions.iter().filter(|p| p[0].hypot(p[1]) < 1.0).map(|p| p[2]).collect();
        assert!(!centre.is_empty());
        let top = crate::terrain::height_at(t, 0.0, 0.0);
        assert!(centre.iter().all(|h| (h - top).abs() < 1e-9), "the junction is not at the ground's height");
    }

    #[test]
    fn the_bench_is_a_function_of_the_world() {
        let (a, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (b, _) = world("hill?amp=60&radius=400", "net:cross?len=400", None);
        let (x, y) = (bench(&a), bench(&b));
        assert_eq!(x.carriageway.positions, y.carriageway.positions);
        assert_eq!(x.pavement.positions, y.pavement.positions);
        assert_eq!(x.steps, y.steps);
    }
}
