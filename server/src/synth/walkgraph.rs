//! The pedestrian drape graph: the drawn walk network as one connected object.
//!
//! Today every walk band gets its seats locally — a hosted strip from its own
//! host's profile, a free band from the ground under its own segment, a stub
//! from the nearest hosted band — and connectivity is restored afterwards by
//! pairwise patches (`walkway::weld_joints`, `walkway::taper_along_runs`,
//! `walkway::seat_stubs`), each with its own private notion of what a joint
//! is. The check that scores the result (`network.walk_joint`) builds the
//! joint graph a third way, from connector ids and endpoint coincidence, so
//! the weld can satisfy itself while the check still fails.
//!
//! This module builds the joint graph **once**, in the world model, from the
//! same evidence the check reads: band-end coincidence within
//! [`priors::WALK_JOIN_EPS_M`], mapped connector ids carried by two or more
//! features, and free ends standing on another band's interior. One node per
//! joint, one height per node:
//!
//! - a node where a **hosted** band stands is pinned to the hosted seats'
//!   mean — the host street is the senior datum, read exactly instead of
//!   through the weld's proximity mean;
//! - a node whose joint carries a connector shared with a **profiled
//!   corridor** is pinned to that corridor's solved surface at the connector
//!   arc — a footway ending on a street takes the street's height, the
//!   authority channel the census found missing;
//! - every other node is **free**: seeded from its members' ground-stamped
//!   seats and relaxed toward its neighbours (a screened smoothing in
//!   miniature — §4.4's trick on the junior side, writing nothing back).
//!
//! Every correction is bounded by [`REACH_M`] of the node's own ground seed:
//! past it the disagreement is a structure or a mismap, and moving a drawn
//! band that far off the ground it was fitted to trades a joint step for a
//! wall (`fit_to_ground`'s veto exists for exactly that surface). What the
//! bound declines is counted, not hidden — the census prints it as the
//! authority gap left for the elevated-span and ground stages.
//!
//! Stratum discipline: the graph reads solved profiles and the bands'
//! already-stamped seats; it writes only free walk-band seats. Nothing flows
//! into any senior solve (I7), and the graph is built once from the world
//! model, never per tile (I5).
//!
//! `ARPT_NO_WALK_GRAPH` reverts to the weld; `ARPT_WALK_GRAPH=census` prints
//! the joint census from the same graph.

use geo_types::Coord;

use crate::assemble::grid::GridIndex;
use crate::priors::WALK_JOIN_EPS_M;
use crate::scene::{CorridorId, SceneGraph, DEG_M};
use crate::solve::SolvedModel;
use crate::synth::carriageway::SourceSeg;
use crate::synth::walkway::NO_HOST;

/// How far past a band's own drawn edge a joint point still counts as on it,
/// in metres — the boolean kernel, quantization, and an endpoint one vertex
/// short of the kerb line (the same allowance `network.walk_joint` gives).
const ON_M: f64 = 0.25;

/// Grid cell for the end index, in metres.
const CELL_M: f64 = 16.0;

/// The correction bound, in metres: a node's height may leave its ground
/// seed by at most this. The weld drew the same line (`WELD_MAX_M`) and for
/// the same reason — past it the joint is not a seam to close but a missing
/// structure, and a band lifted that far off the ground it was width-fitted
/// to fails the wall veto's own surface.
const REACH_M: f64 = 3.0;

/// Relaxation sweeps. The clamp bounds the answer, the edge weights are
/// 1/length, and the graph's components are short chains between pins, so a
/// fixed small count converges far past the drawn tolerance; fixed so the
/// result is a function of the graph alone.
const SWEEPS: usize = 32;

/// How far a kerb stub's end may stand from the hosted band it continues and
/// still take its seat — `walkway::seat_stubs`' own reach, inherited with its
/// job: a stub is the strip between the kerb and whatever the crossing joins,
/// and its inner end lands up to a band-width short of the pavement.
const STUB_SEAT_REACH_M: f64 = crate::priors::WALK_WIDTH_M * 1.5;

/// The terrain mass in the free-node update, per metre of incident edge:
/// the screening that keeps a correction local instead of dragging a whole
/// hillside path toward one joint.
const GROUND_MASS: f64 = 0.5;

struct UnionFind {
    parent: Vec<u32>,
}

impl UnionFind {
    fn new(n: usize) -> UnionFind {
        UnionFind { parent: (0..n as u32).collect() }
    }

    fn find(&mut self, mut i: u32) -> u32 {
        while self.parent[i as usize] != i {
            let up = self.parent[self.parent[i as usize] as usize];
            self.parent[i as usize] = up;
            i = up;
        }
        i
    }

    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            self.parent[hi as usize] = lo;
        }
    }
}

/// One member standing at a joint: a band end, or a band interior an end
/// stands on (a T-joint's host).
struct Member {
    band: u32,
    height: f64,
    hosted: bool,
}

pub struct WalkGraph {
    /// Node per band-end slot (`2*band + end`), `u32::MAX` off the graph.
    node_of_slot: Vec<u32>,
    /// Solved height per node.
    height: Vec<f64>,
    /// Positions, seeds and pin flags per node; positions double as the
    /// [`Self::height_near`] index, kept for the census.
    at: Vec<Coord>,
    seed: Vec<f64>,
    pinned: Vec<bool>,
    /// Node positions, spatially indexed for [`Self::height_near`].
    node_grid: GridIndex,
    cos_lat: f64,
    /// Census counters gathered during the build.
    stat_multi: usize,
    stat_multi_corridor: usize,
    stat_connector_unions: usize,
    stat_t_hosts: usize,
    stat_street_pins: usize,
    stat_out_of_reach: usize,
    stat_stub_pins: usize,
    spreads: Vec<(f64, Coord, bool)>,
}

/// One shared mapped connector on a pedestrian line: where it falls in plan,
/// and the street surface there when a profiled corridor carries it.
struct ConnectorSeed {
    p: Coord,
    street: Option<f64>,
}

impl WalkGraph {
    pub fn build(
        scene: &SceneGraph,
        solved: &SolvedModel,
        bands: &[SourceSeg],
        sources: &[u64],
    ) -> WalkGraph {
        Self::build_inner(bands, sources, &connector_seeds(scene, solved))
    }

    fn build_inner(bands: &[SourceSeg], sources: &[u64], seeds: &[ConnectorSeed]) -> WalkGraph {
        let live: Vec<u32> =
            (0..bands.len() as u32).filter(|&i| bands[i as usize].level == 0).collect();
        let mut g = WalkGraph {
            node_of_slot: vec![u32::MAX; bands.len() * 2],
            height: Vec::new(),
            at: Vec::new(),
            seed: Vec::new(),
            pinned: Vec::new(),
            stat_multi: 0,
            stat_multi_corridor: 0,
            stat_connector_unions: 0,
            stat_t_hosts: 0,
            stat_street_pins: 0,
            stat_out_of_reach: 0,
            stat_stub_pins: 0,
            spreads: Vec::new(),
            node_grid: GridIndex::with_cell_m(CELL_M),
            cos_lat: 1.0,
        };
        if live.is_empty() {
            return g;
        }
        let end_pos = |slot: u32| -> Coord {
            let s = &bands[(slot / 2) as usize];
            if slot % 2 == 0 { s.a } else { s.b }
        };
        let end_height = |slot: u32| -> f64 {
            let s = &bands[(slot / 2) as usize];
            if slot % 2 == 0 { s.height_a } else { s.height_b }
        };
        let cos_lat = bands[live[0] as usize].cos_lat;
        let eps = |m: f64| (m / (DEG_M * cos_lat), m / DEG_M);

        let mut grid = GridIndex::with_cell_m(CELL_M);
        let mut slots: Vec<u32> = Vec::with_capacity(live.len() * 2);
        for &i in &live {
            for e in 0..2u32 {
                let slot = i * 2 + e;
                let p = end_pos(slot);
                grid.insert((p.x, p.y, p.x, p.y), slot);
                slots.push(slot);
            }
        }

        let mut uf = UnionFind::new(bands.len() * 2);
        let mut scratch: Vec<u32> = Vec::new();

        // Coincidence: two ends within the shared epsilon are one joint.
        let (ex, ey) = eps(WALK_JOIN_EPS_M);
        for &slot in &slots {
            let p = end_pos(slot);
            grid.query((p.x - ex, p.y - ey, p.x + ex, p.y + ey), &mut scratch);
            for &o in &scratch {
                if o != slot && close_m(end_pos(o), p, cos_lat, WALK_JOIN_EPS_M) {
                    uf.union(slot, o);
                }
            }
        }

        // Connector seeds: a mapped connector carried by two or more features
        // is a joint even where the drawn ends missed each other in plan —
        // and where one carrier is a profiled corridor, the joint's height
        // authority.
        let mut slot_pins: Vec<(u32, f64)> = Vec::new();
        for seed in seeds {
            let p = seed.p;
            grid.query((p.x - ex, p.y - ey, p.x + ex, p.y + ey), &mut scratch);
            let mut first: Option<u32> = None;
            for &o in &scratch {
                if !close_m(end_pos(o), p, cos_lat, WALK_JOIN_EPS_M) {
                    continue;
                }
                match first {
                    None => first = Some(o),
                    Some(f) => {
                        if uf.find(f) != uf.find(o) {
                            g.stat_connector_unions += 1;
                        }
                        uf.union(f, o);
                    }
                }
            }
            if let (Some(f), Some(street)) = (first, seed.street) {
                slot_pins.push((f, street));
            }
        }
        // Resolved to roots only after every union has had its say — a later
        // seed can merge the joint a pin was recorded against.
        let mut root_pin: std::collections::HashMap<u32, (f64, u32)> = Default::default();
        for &(slot, street) in &slot_pins {
            let e = root_pin.entry(uf.find(slot)).or_insert((0.0, 0));
            e.0 += street;
            e.1 += 1;
        }

        // Members per root, deterministically: slots ascending.
        let mut joints: std::collections::BTreeMap<u32, Vec<Member>> = Default::default();
        for &slot in &slots {
            let root = uf.find(slot);
            let s = &bands[(slot / 2) as usize];
            joints.entry(root).or_default().push(Member {
                band: slot / 2,
                height: end_height(slot),
                hosted: s.corridor != NO_HOST,
            });
        }

        // T-joints: a free end standing on another band's interior joins that
        // band as an anchor member at its own interpolated seat.
        let mut band_grid = GridIndex::with_cell_m(CELL_M);
        for &i in &live {
            let s = &bands[i as usize];
            let pad = s.drawn_half() + ON_M;
            let (px, py) = eps(pad);
            let (x0, x1) = (s.a.x.min(s.b.x) - px, s.a.x.max(s.b.x) + px);
            let (y0, y1) = (s.a.y.min(s.b.y) - py, s.a.y.max(s.b.y) + py);
            band_grid.insert((x0, y0, x1, y1), i);
        }
        for &slot in &slots {
            let s = &bands[(slot / 2) as usize];
            if s.corridor != NO_HOST {
                continue;
            }
            let p = end_pos(slot);
            band_grid.query((p.x, p.y, p.x, p.y), &mut scratch);
            let root = uf.find(slot);
            for &o in &scratch {
                if o == slot / 2 {
                    continue;
                }
                let t = &bands[o as usize];
                if close_m(t.a, p, cos_lat, WALK_JOIN_EPS_M)
                    || close_m(t.b, p, cos_lat, WALK_JOIN_EPS_M)
                {
                    continue;
                }
                let (d, tt) = point_to_seg_m(p, t.a, t.b, cos_lat);
                if d - t.drawn_half_at(tt) > ON_M {
                    continue;
                }
                g.stat_t_hosts += 1;
                joints.entry(root).or_default().push(Member {
                    band: o,
                    height: t.height_at(tt),
                    hosted: t.corridor != NO_HOST,
                });
            }
        }

        // Stub pins: a crossing's kerb stub carries no source of its own
        // (`sources[i] == 0`), and its inner end may stand up to a band-width
        // short of the pavement it continues — wider than the joint epsilon.
        // The nearest hosted band within [`STUB_SEAT_REACH_M`] joins its end
        // as a hosted member at the interpolated seat, which is exactly what
        // `walkway::seat_stubs` stamped before the graph owned the joints —
        // now subject to the same reach decline every other pin gets.
        for &slot in &slots {
            let band = (slot / 2) as usize;
            let s = &bands[band];
            if s.corridor != NO_HOST || sources.get(band).copied().unwrap_or(1) != 0 {
                continue;
            }
            let p = end_pos(slot);
            let (rx, ry) = eps(STUB_SEAT_REACH_M);
            band_grid.query((p.x - rx, p.y - ry, p.x + rx, p.y + ry), &mut scratch);
            let root = uf.find(slot);
            let mut best: Option<(f64, u32, f64)> = None;
            for &o in &scratch {
                let t = &bands[o as usize];
                if t.corridor == NO_HOST {
                    continue;
                }
                let (d, tt) = point_to_seg_m(p, t.a, t.b, cos_lat);
                if d <= STUB_SEAT_REACH_M
                    && best.map_or(true, |(bd, bo, _)| d < bd || (d == bd && o < bo))
                {
                    best = Some((d, o, t.height_at(tt)));
                }
            }
            if let Some((_, o, h)) = best {
                g.stat_stub_pins += 1;
                joints.entry(root).or_default().push(Member {
                    band: o,
                    height: h,
                    hosted: true,
                });
            }
        }

        // Nodes. A joint's seed is its **free** members' mean seat — the
        // ground its bands were fitted to. A pin (hosted seats' mean, else
        // the street surface at a shared connector) takes the node only
        // within [`REACH_M`] of that seed; past it the pin is declined whole
        // and counted, exactly the weld's rule — a 10 m disagreement is a
        // structure question, and half-closing it draws a wall where a step
        // stood.
        for (&root, members) in &joints {
            let node = g.height.len() as u32;
            let free: Vec<f64> =
                members.iter().filter(|m| !m.hosted).map(|m| m.height).collect();
            let hosted: Vec<f64> =
                members.iter().filter(|m| m.hosted).map(|m| m.height).collect();
            let seed = if free.is_empty() {
                members.iter().map(|m| m.height).sum::<f64>() / members.len() as f64
            } else {
                free.iter().sum::<f64>() / free.len() as f64
            };
            let (mut h, mut pin) = (seed, false);
            if !hosted.is_empty() {
                let hh = hosted.iter().sum::<f64>() / hosted.len() as f64;
                if free.is_empty() || (hh - seed).abs() <= REACH_M {
                    h = hh;
                    pin = true;
                } else {
                    g.stat_out_of_reach += 1;
                }
            } else if let Some(&(sum, n)) = root_pin.get(&root) {
                if n > 0 {
                    let street = sum / f64::from(n);
                    if (street - seed).abs() <= REACH_M {
                        h = street;
                        pin = true;
                        g.stat_street_pins += 1;
                    } else {
                        g.stat_out_of_reach += 1;
                    }
                }
            }
            g.at.push(end_pos(root));
            g.seed.push(seed);
            g.height.push(h);
            g.pinned.push(pin);
            for m in members {
                for e in 0..2u32 {
                    let slot = m.band * 2 + e;
                    if uf.find(slot) == root {
                        g.node_of_slot[slot as usize] = node;
                    }
                }
            }

            // The census rows, from the same pass.
            let mut seen: Vec<u32> = members.iter().map(|m| m.band).collect();
            seen.sort_unstable();
            seen.dedup();
            if seen.len() >= 2 {
                g.stat_multi += 1;
                let mut cs: Vec<CorridorId> =
                    seen.iter().map(|&b| bands[b as usize].corridor).collect();
                cs.sort_unstable();
                cs.dedup();
                if cs.len() >= 2 {
                    g.stat_multi_corridor += 1;
                }
                let lo = members.iter().map(|m| m.height).fold(f64::INFINITY, f64::min);
                let hi =
                    members.iter().map(|m| m.height).fold(f64::NEG_INFINITY, f64::max);
                let mixed = members.iter().any(|m| m.hosted)
                    && members.iter().any(|m| !m.hosted);
                g.spreads.push((hi - lo, end_pos(root), mixed));
            }
        }

        g.cos_lat = cos_lat;
        for (n, p) in g.at.iter().enumerate() {
            g.node_grid.insert((p.x, p.y, p.x, p.y), n as u32);
        }

        // Edges: one per **free** band, between its two end nodes, weight
        // 1/length. A hosted band is a boundary condition — it enters as its
        // end nodes' pins and never as a smoothing edge, or a pin the reach
        // rule just declined would pull the free node anyway, through the
        // side door.
        let mut edges: Vec<(u32, u32, f64)> = Vec::with_capacity(live.len());
        for &i in &live {
            let s = &bands[i as usize];
            if s.corridor != NO_HOST {
                continue;
            }
            let (na, nb) = (g.node_of_slot[(i * 2) as usize], g.node_of_slot[(i * 2 + 1) as usize]);
            if na == u32::MAX || nb == u32::MAX || na == nb {
                continue;
            }
            let len = seg_len_m(s).max(0.5);
            edges.push((na, nb, 1.0 / len));
        }
        // The relaxation, on the **correction field**, not the heights: a
        // pinned node's correction is `pin − seed`, a free node's decays
        // toward zero under the ground mass. Solving deltas rather than
        // heights is what keeps the graph from re-draping the network: where
        // nothing pins and members agree, the correction is exactly zero and
        // the stamped seat is bit-identical to the fitted one — measured the
        // other way first, smoothing the heights themselves tilted every
        // sloped chain off its ground (`walk_crossfall` 2.67 → 3.29 %,
        // `walk_rim` 0.72 → 1.04 %) while closing nothing.
        let n_nodes = g.height.len();
        let mut c: Vec<f64> = (0..n_nodes)
            .map(|n| if g.pinned[n] { g.height[n] - g.seed[n] } else { 0.0 })
            .collect();
        let mut acc_c = vec![0.0f64; n_nodes];
        let mut acc_w = vec![0.0f64; n_nodes];
        for _ in 0..SWEEPS {
            acc_c.iter_mut().for_each(|v| *v = 0.0);
            acc_w.iter_mut().for_each(|v| *v = 0.0);
            for &(a, b, w) in &edges {
                acc_c[a as usize] += w * c[b as usize];
                acc_w[a as usize] += w;
                acc_c[b as usize] += w * c[a as usize];
                acc_w[b as usize] += w;
            }
            for n in 0..n_nodes {
                if g.pinned[n] || acc_w[n] == 0.0 {
                    continue;
                }
                c[n] = acc_c[n] / (GROUND_MASS + acc_w[n]);
            }
        }
        for n in 0..n_nodes {
            if !g.pinned[n] && c[n] != 0.0 {
                g.height[n] = g.seed[n] + c[n].clamp(-REACH_M, REACH_M);
            }
        }
        g
    }

    /// The solved height of the joint at `p`, when one stands within `eps_m`
    /// — nearest node wins, ties to the lower node id (I5). This is how a
    /// consumer with only a plan position (an elevated span's end, cut from
    /// the raw feature line) reads the network's height authority.
    pub fn height_near(&self, p: Coord, eps_m: f64) -> Option<f64> {
        self.node_near(p, eps_m).map(|n| self.height[n as usize])
    }

    /// [`Self::height_near`], but only where the joint is **pinned** — a
    /// hosted seat or a street connector stands there. A free node's height
    /// is the same ground read its neighbourhood already made, so it is no
    /// authority over another ground read; consumers deciding whether to
    /// trust a joint over the terrain (an elevated span's abutment) must ask
    /// for a pin, or they trade the wall-walk's rescue for nothing — measured
    /// as `clearance.deck_over_ground` 0.570 → 0.596 % when free anchors
    /// were allowed to suppress it.
    pub fn pinned_height_near(&self, p: Coord, eps_m: f64) -> Option<f64> {
        self.node_near(p, eps_m)
            .filter(|&n| self.pinned[n as usize])
            .map(|n| self.height[n as usize])
    }

    fn node_near(&self, p: Coord, eps_m: f64) -> Option<u32> {
        Self::node_near_in(&self.node_grid, &self.at, self.cos_lat, p, eps_m)
    }

    fn node_near_in(
        grid: &GridIndex,
        at: &[Coord],
        cos_lat: f64,
        p: Coord,
        eps_m: f64,
    ) -> Option<u32> {
        let (ex, ey) = (eps_m / (DEG_M * cos_lat), eps_m / DEG_M);
        let mut out = Vec::new();
        grid.query((p.x - ex, p.y - ey, p.x + ex, p.y + ey), &mut out);
        let mut best: Option<(f64, u32)> = None;
        for &n in &out {
            let q = at[n as usize];
            let dx = (q.x - p.x) * DEG_M * cos_lat;
            let dy = (q.y - p.y) * DEG_M;
            let d = dx.hypot(dy);
            if d <= eps_m && best.map_or(true, |(bd, bn)| d < bd || (d == bd && n < bn)) {
                best = Some((d, n));
            }
        }
        best.map(|(_, n)| n)
    }

    /// Write the node heights back onto the free bands' seats. Hosted bands
    /// and seated stubs keep their host-datum seats — they are the pins.
    pub fn stamp(&self, bands: &mut [SourceSeg]) {
        for (i, s) in bands.iter_mut().enumerate() {
            if s.level != 0 || s.corridor != NO_HOST {
                continue;
            }
            let (na, nb) = (self.node_of_slot[i * 2], self.node_of_slot[i * 2 + 1]);
            if na != u32::MAX {
                s.height_a = self.height[na as usize];
            }
            if nb != u32::MAX {
                s.height_b = self.height[nb as usize];
            }
        }
    }

    /// Print the joint census gathered during the build.
    pub fn census(&self, bands_n: usize) {
        let mut spreads = self.spreads.clone();
        spreads
            .sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let over = spreads.iter().filter(|(s, _, _)| *s > 0.30).count();
        let mixed_over = spreads.iter().filter(|(s, _, m)| *s > 0.30 && *m).count();
        let p = |q: f64| -> f64 {
            if spreads.is_empty() {
                return 0.0;
            }
            spreads[((spreads.len() - 1) as f64 * (1.0 - q)) as usize].0
        };
        eprintln!(
            "[walkgraph] {} bands, {} nodes ({} multi-band, {} multi-corridor, \
             {} via connector ids, {} T-hosts, {} street pins, {} pins out of reach, \
             {} stub pins)",
            bands_n,
            self.height.len(),
            self.stat_multi,
            self.stat_multi_corridor,
            self.stat_connector_unions,
            self.stat_t_hosts,
            self.stat_street_pins,
            self.stat_out_of_reach,
            self.stat_stub_pins,
        );
        eprintln!(
            "[walkgraph] seat spread at multi-band joints: p50 {:.2} m, p90 {:.2} m, \
             max {:.2} m; {} joints past 0.30 m ({} of them hosted-vs-free)",
            p(0.5),
            p(0.9),
            spreads.first().map_or(0.0, |s| s.0),
            over,
            mixed_over,
        );
        for (s, at, mixed) in spreads.iter().take(10) {
            eprintln!(
                "[walkgraph]   {:8.2} m at {:.6},{:.6}{}",
                s,
                at.x,
                at.y,
                if *mixed { "  hosted-vs-free" } else { "" }
            );
        }
    }
}

/// Every shared mapped connector on every pedestrian line, with the street
/// surface where a profiled corridor carries it: the joint evidence and the
/// authority channel, read once from the scene.
fn connector_seeds(scene: &SceneGraph, solved: &SolvedModel) -> Vec<ConnectorSeed> {
    let mut carriers: std::collections::HashMap<u64, u32> = Default::default();
    for (line, _) in scene.walks.lines() {
        let mut ids: Vec<u64> = line.connectors.iter().map(|c| c.id).collect();
        ids.sort_unstable();
        ids.dedup();
        for id in ids {
            *carriers.entry(id).or_insert(0) += 1;
        }
    }
    for c in scene.corridors.iter() {
        let mut ids: Vec<u64> = c.connectors.iter().map(|&(id, _)| id).collect();
        ids.sort_unstable();
        ids.dedup();
        for id in ids {
            *carriers.entry(id).or_insert(0) += 1;
        }
    }
    let mut street_h: std::collections::HashMap<u64, (f64, u32)> = Default::default();
    for c in scene.corridors.iter() {
        let Some(p) = solved.profile(c.id) else { continue };
        for &(id, arc) in &c.connectors {
            let e = street_h.entry(id).or_insert((0.0, 0));
            e.0 += p.road_at_arc(arc);
            e.1 += 1;
        }
    }
    let mut seeds = Vec::new();
    for (line, _) in scene.walks.lines() {
        if line.line.len() < 2 {
            continue;
        }
        let arc = crate::scene::cumulative_arc(&line.line);
        let total = *arc.last().unwrap_or(&0.0);
        for c in &line.connectors {
            if carriers.get(&c.id).copied().unwrap_or(0) < 2 {
                continue;
            }
            seeds.push(ConnectorSeed {
                p: point_at(&line.line, &arc, c.at * total),
                street: street_h
                    .get(&c.id)
                    .filter(|&&(_, n)| n > 0)
                    .map(|&(sum, n)| sum / f64::from(n)),
            });
        }
    }
    seeds
}

fn close_m(a: Coord, b: Coord, cos_lat: f64, eps_m: f64) -> bool {
    let dx = (a.x - b.x) * DEG_M * cos_lat;
    let dy = (a.y - b.y) * DEG_M;
    dx.hypot(dy) <= eps_m
}

fn seg_len_m(s: &SourceSeg) -> f64 {
    let dx = (s.b.x - s.a.x) * DEG_M * s.cos_lat;
    let dy = (s.b.y - s.a.y) * DEG_M;
    dx.hypot(dy)
}

/// Lateral distance in metres from `p` to segment `ab`, and the parameter of
/// the closest point.
fn point_to_seg_m(p: Coord, a: Coord, b: Coord, cos_lat: f64) -> (f64, f64) {
    let (ax, ay) = ((p.x - a.x) * cos_lat, p.y - a.y);
    let (bx, by) = ((b.x - a.x) * cos_lat, b.y - a.y);
    let len2 = bx * bx + by * by;
    let t = if len2 > 0.0 { ((ax * bx + ay * by) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (dx, dy) = (ax - bx * t, ay - by * t);
    (dx.hypot(dy) * DEG_M, t)
}

fn point_at(line: &[Coord], arc: &[f64], s: f64) -> Coord {
    let s = s.clamp(0.0, *arc.last().unwrap_or(&0.0));
    for i in 1..line.len() {
        if arc[i] >= s {
            let seg = arc[i] - arc[i - 1];
            let t = if seg > 0.0 { (s - arc[i - 1]) / seg } else { 0.0 };
            return Coord {
                x: line[i - 1].x + (line[i].x - line[i - 1].x) * t,
                y: line[i - 1].y + (line[i].y - line[i - 1].y) * t,
            };
        }
    }
    *line.last().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::priors::Surface;
    use crate::assemble::facades::Section;

    fn band(a: (f64, f64), b: (f64, f64), h: (f64, f64), corridor: CorridorId) -> SourceSeg {
        SourceSeg {
            a: Coord { x: a.0, y: a.1 },
            b: Coord { x: b.0, y: b.1 },
            cos_lat: 1.0,
            half_m: 1.0,
            sect_a: Section::uniform(1.0),
            sect_b: Section::uniform(1.0),
            level: 0,
            layer: 0,
            cut_a: None,
            cut_b: None,
            height_a: h.0,
            height_b: h.1,
            corridor,
            surface: Surface::Walkway,
            rise_m: 0.0,
            arc0: 0.0,
        }
    }

    fn build(bands: &[SourceSeg]) -> WalkGraph {
        WalkGraph::build_inner(bands, &vec![1; bands.len()], &[])
    }

    const D: f64 = 20.0 / DEG_M; // a 20 m step in plan degrees

    #[test]
    fn a_free_joint_takes_one_height_and_both_bands_get_it() {
        // Two free chains meeting at one point with a 2 m disagreement: the
        // joint resolves to one height and both ends are stamped with it.
        let mut bands = vec![
            band((0.0, 0.0), (D, 0.0), (100.0, 100.0), NO_HOST),
            band((D, 0.0), (2.0 * D, 0.0), (102.0, 102.0), NO_HOST),
        ];
        let g = build(&bands);
        g.stamp(&mut bands);
        assert!(
            (bands[0].height_b - bands[1].height_a).abs() < 1e-9,
            "the joint is one height: {} vs {}",
            bands[0].height_b,
            bands[1].height_a
        );
    }

    #[test]
    fn a_hosted_member_pins_the_joint_and_is_never_moved() {
        let mut bands = vec![
            band((0.0, 0.0), (D, 0.0), (120.0, 120.0), 7),
            band((D, 0.0), (2.0 * D, 0.0), (118.5, 118.5), NO_HOST),
        ];
        let g = build(&bands);
        g.stamp(&mut bands);
        assert_eq!(bands[0].height_b, 120.0, "the hosted seat is the pin");
        assert!(
            (bands[1].height_a - 120.0).abs() < 1e-9,
            "the free end takes the hosted seat, got {}",
            bands[1].height_a
        );
    }

    #[test]
    fn a_disagreement_past_the_reach_is_left_standing() {
        // A hosted band 10 m above the free one: past REACH_M the pin is
        // declined whole — the residue is a structure question, and
        // half-closing it would draw a wall where a step stood.
        let mut bands = vec![
            band((0.0, 0.0), (D, 0.0), (120.0, 120.0), 7),
            band((D, 0.0), (2.0 * D, 0.0), (110.0, 110.0), NO_HOST),
        ];
        let g = build(&bands);
        g.stamp(&mut bands);
        assert!(
            (bands[1].height_a - 110.0).abs() < 1e-9,
            "the pin is declined, the free seat stands, got {}",
            bands[1].height_a
        );
    }

    #[test]
    fn a_correction_decays_along_the_chain() {
        // A 3-segment free chain ending on a hosted pin 2 m up: the far end
        // barely moves.
        let mut bands = vec![
            band((0.0, 0.0), (D, 0.0), (102.0, 102.0), 7),
            band((D, 0.0), (2.0 * D, 0.0), (100.0, 100.0), NO_HOST),
            band((2.0 * D, 0.0), (3.0 * D, 0.0), (100.0, 100.0), NO_HOST),
            band((3.0 * D, 0.0), (4.0 * D, 0.0), (100.0, 100.0), NO_HOST),
        ];
        let g = build(&bands);
        g.stamp(&mut bands);
        assert!((bands[1].height_a - 102.0).abs() < 1e-9);
        let far = bands[3].height_b;
        let near = bands[1].height_b;
        assert!(near > far, "the correction decays: near {near}, far {far}");
        assert!(far < 100.7, "the far end barely moves, got {far}");
    }

    #[test]
    fn the_result_is_independent_of_band_order() {
        let mk = || {
            vec![
                band((0.0, 0.0), (D, 0.0), (102.0, 102.0), 7),
                band((D, 0.0), (2.0 * D, 0.0), (100.0, 100.0), NO_HOST),
                band((2.0 * D, 0.0), (3.0 * D, 0.0), (100.4, 100.4), NO_HOST),
            ]
        };
        let mut a = mk();
        let g = build(&a);
        g.stamp(&mut a);
        let mut b = mk();
        b.reverse();
        let g = build(&b);
        g.stamp(&mut b);
        b.reverse();
        for (x, y) in a.iter().zip(b.iter()) {
            assert_eq!(x.height_a.to_bits(), y.height_a.to_bits());
            assert_eq!(x.height_b.to_bits(), y.height_b.to_bits());
        }
    }

    #[test]
    fn a_stub_takes_the_hosted_seat_it_stands_short_of() {
        // A kerb stub (no source of its own) whose inner end lands 2 m from
        // the hosted band it continues: the stub-pin rule seats it there,
        // wider than the joint epsilon reaches.
        let gap = 2.0 / DEG_M;
        let bands = vec![
            band((0.0, 0.0), (D, 0.0), (120.0, 120.0), 7),
            band((D + gap, 0.0), (D + gap + 3.0 / DEG_M, 0.0), (118.9, 118.9), NO_HOST),
        ];
        let mut bands2 = bands.clone();
        let sources = vec![9, 0]; // the second band is a stub
        let g = WalkGraph::build_inner(&bands, &sources, &[]);
        g.stamp(&mut bands2);
        assert!(
            (bands2[1].height_a - 120.0).abs() < 1e-9,
            "the stub's inner end takes the hosted seat, got {}",
            bands2[1].height_a
        );
        // The same band with a real source is a path, not a stub: no pin.
        let mut bands3 = bands.clone();
        let g = WalkGraph::build_inner(&bands, &vec![9, 8], &[]);
        g.stamp(&mut bands3);
        assert!(
            (bands3[1].height_a - 118.9).abs() < 1e-9,
            "a path 2 m short of a band keeps its own seat, got {}",
            bands3[1].height_a
        );
    }

    #[test]
    fn a_pinned_joint_answers_height_near_and_a_free_one_does_not_answer_pinned() {
        let bands = vec![
            band((0.0, 0.0), (D, 0.0), (120.0, 120.0), 7),
            band((D, 0.0), (2.0 * D, 0.0), (119.0, 119.0), NO_HOST),
            band((3.0 * D, 0.0), (4.0 * D, 0.0), (100.0, 100.0), NO_HOST),
        ];
        let g = build(&bands);
        let joint = Coord { x: D, y: 0.0 };
        assert_eq!(g.height_near(joint, 1.0), Some(120.0));
        assert_eq!(g.pinned_height_near(joint, 1.0), Some(120.0), "hosted joint is a pin");
        let free_end = Coord { x: 3.0 * D, y: 0.0 };
        assert_eq!(g.height_near(free_end, 1.0), Some(100.0));
        assert_eq!(g.pinned_height_near(free_end, 1.0), None, "a free joint is no authority");
        assert_eq!(g.height_near(Coord { x: 10.0 * D, y: 0.0 }, 1.0), None);
    }

    #[test]
    fn union_find_unions_toward_the_lower_root_whatever_the_order() {
        let mut a = UnionFind::new(4);
        a.union(3, 1);
        a.union(1, 2);
        let mut b = UnionFind::new(4);
        b.union(1, 2);
        b.union(3, 1);
        for i in 0..4 {
            assert_eq!(a.find(i), b.find(i));
        }
        assert_eq!(a.find(3), 1);
        assert_eq!(a.find(0), 0);
    }

    #[test]
    fn a_point_projects_onto_a_segment_with_its_parameter() {
        let a = Coord { x: 0.0, y: 0.0 };
        let b = Coord { x: 0.001, y: 0.0 };
        let (d, t) = point_to_seg_m(Coord { x: 0.0005, y: 0.00001 }, a, b, 1.0);
        assert!((t - 0.5).abs() < 1e-6);
        assert!((d - 0.00001 * DEG_M).abs() < 1e-6);
    }
}
