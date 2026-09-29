//! The kerb-gap check: a stretch of kerb a sidewalk claims with bare ground
//! just outside it.
//!
//! The kerb, the legs and the room each lay a pavement, and each is checked
//! against it, so the check is a module of its own rather than one step's: it
//! is a function of a surface, and any step's surface can be asked.

use std::collections::HashMap;

use crate::line;
use crate::poly::{self, Indexed, Pt, Shapes};
use crate::standard::{RUNG_HALF_M, SIDEWALK_BRIDGE_M, STATION_M};
use crate::world::{Attached, Surface};

/// The side of the index's cells, in metres: a bucket for the rungs, and
/// nothing about the answer.
const CELL_M: f64 = 16.0;

/// How far outside a kerb station the pavement is probed for `kerb_gap`,
/// in metres: inside the narrowest pavement drawn.
const PROBE_M: f64 = 0.3;

/// A kerb stretch turning by this much, in degrees, is a road's end cap
/// (180°), not a corner between two legs (a right angle inward) — see
/// [`kerb_gaps`].
const CAP_TURN_DEG: f64 = 135.0;

/// A rung meeting the kerb at less than this, in degrees, runs along it
/// rather than across it and claims nothing there: the last rung of a
/// sidewalk that ends where its road does lies along the road's square end
/// face, which is not a kerb the sidewalk owns — see [`kerb_gaps`].
const CLAIM_MIN_DEG: f64 = 30.0;

/// One station of a kerb ring: where it is, the unit tangent there
/// (averaged over the segments either side), and how far the kerb turns
/// at it, in degrees.
#[derive(Debug, Clone, Copy)]
pub struct Station {
    pub at: Pt,
    pub tangent: Pt,
    pub turn_deg: f64,
}

/// The stations of one kerb ring, [`STATION_M`] apart, once round it (the
/// closing point is not repeated).
pub fn stations(ring: &[Pt]) -> Vec<Station> {
    let mut closed = ring.to_vec();
    closed.push(ring[0]);
    let mut pts = line::resample(&closed, STATION_M);
    pts.pop();
    let n = pts.len();
    (0..n)
        .map(|i| {
            let (a, b, c) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
            let (u, v) = (line::unit([b[0] - a[0], b[1] - a[1]]), line::unit([c[0] - b[0], c[1] - b[1]]));
            Station { at: b, tangent: line::unit([u[0] + v[0], u[1] + v[1]]), turn_deg: line::turn_deg(a, b, c) }
        })
        .collect()
}

/// Where a probe finds bare ground: outside the asphalt, the pavement and
/// the walls, the three indexed once for the many probes of a check.
pub struct Bare {
    asphalt: Indexed,
    pavement: Indexed,
    walls: Indexed,
}

impl Bare {
    pub fn new(carriageway: &Shapes, pavement: &Shapes, walls: &Shapes) -> Bare {
        Bare { asphalt: Indexed::new(carriageway), pavement: Indexed::new(pavement), walls: Indexed::new(walls) }
    }

    /// Whether `p` is bare ground: on none of the three. A probe inside a
    /// wall is a facade standing on the kerb, which is not bare.
    pub fn at(&self, p: Pt) -> bool {
        !self.pavement.contains(p) && !self.asphalt.contains(p) && !self.walls.contains(p)
    }
}

/// The share of kerb stations a sidewalk claims that have bare ground just
/// outside them, as `(gaps, stations)`. A kerb station is claimed when some
/// attached station's rung — the segment from its foot on the axis to the
/// station — passes within the rung's half-width less the probe's reach of
/// it, which is the same side of the same road, and not the far kerb
/// across it; and a stretch of kerb shorter than [`SIDEWALK_BRIDGE_M`]
/// between two stations that a *run's* rung claims is claimed with them,
/// because a pavement that stands against the kerb on both sides of a
/// corner stands against the corner
/// (a landing is a footway's end, not a pavement, and claims nothing
/// beyond its own rung). A rung claims the kerb it crosses, not one it
/// runs along (`CLAIM_MIN_DEG`): a road's square end face, with the
/// sidewalk's last rung lying along it, is nobody's. A stretch that long
/// without a rung is a side road's mouth, whose kerb runs away down the
/// leg and back, and is not claimed; nor is a stretch that turns back on
/// itself by `CAP_TURN_DEG` or more, which is the road's end — nothing
/// says the two pavements of a road join round its turning head.
pub fn kerb_gaps(surface: &Surface, attached: &[Attached], footprints: &Shapes) -> (Vec<Pt>, usize) {
    let bare = Bare::new(&surface.senior(), &surface.walk, footprints);
    gaps_on(&surface.carriageway, &bare, attached)
}

/// The same over one carriageway and one reading of what is bare.
fn gaps_on(carriageway: &Shapes, bare: &Bare, attached: &[Attached]) -> (Vec<Pt>, usize) {
    // Every rung's direction, once, and the rungs by cell.
    let dirs: Vec<Pt> =
        attached.iter().map(|a| line::unit([a.station[0] - a.foot[0], a.station[1] - a.foot[1]])).collect();
    let mut cells: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, a) in attached.iter().enumerate() {
        let b = [a.foot[0].min(a.station[0]), a.foot[1].min(a.station[1]), a.foot[0].max(a.station[0]), a.foot[1].max(a.station[1])];
        for cell in poly::cells_over(b, CELL_M) {
            cells.entry(cell).or_default().push(i);
        }
    }
    // A kerb station this close to a rung's segment has its probe inside
    // the rung's quad whatever the angle between them: the probe sits
    // `PROBE_M` past the kerb and the rung reaches `WALK_MIN_M` past it.
    let reach = RUNG_HALF_M - PROBE_M;
    let min_sin = CLAIM_MIN_DEG.to_radians().sin();
    // Whether a rung claims `p`, where the kerb runs along `t`: any rung,
    // and a run's rung.
    let claims = |p: Pt, t: Pt| -> (bool, bool) {
        let (c, r) = poly::cell_of(p, CELL_M);
        let mut any = false;
        for dc in -1..=1 {
            for dr in -1..=1 {
                let Some(v) = cells.get(&(c + dc, r + dr)) else {
                    continue;
                };
                for &i in v {
                    let (a, u) = (&attached[i], dirs[i]);
                    if (u[0] * t[1] - u[1] * t[0]).abs() < min_sin || line::segment_distance(a.foot, a.station, p) >= reach {
                        continue;
                    }
                    if !a.landed {
                        return (true, true);
                    }
                    any = true;
                }
            }
        }
        (any, false)
    };
    let (mut gaps, mut n) = (Vec::new(), 0usize);
    for ring in carriageway.iter().flatten() {
        let stations = stations(ring);
        let claimed: Vec<(bool, bool)> = stations.iter().map(|s| claims(s.at, s.tangent)).collect();
        let turn: Vec<f64> = stations.iter().map(|s| s.turn_deg).collect();
        let by_run =
            claim_between(claimed.iter().map(|c| c.1).collect(), &turn, (SIDEWALK_BRIDGE_M / STATION_M) as usize, CAP_TURN_DEG);
        for (i, s) in stations.iter().enumerate() {
            if !(by_run[i] || claimed[i].0) {
                continue;
            }
            let q = stations[(i + 1) % stations.len()].at;
            let d = line::unit([q[0] - s.at[0], q[1] - s.at[1]]);
            // The region is on the left of every ring, outer or hole, so
            // outward is the right-hand normal.
            let probe = [s.at[0] + d[1] * PROBE_M, s.at[1] - d[0] * PROBE_M];
            n += 1;
            if bare.at(probe) {
                gaps.push(s.at);
            }
        }
    }
    (gaps, n)
}

/// `claimed` with every circular stretch of at most `max` unclaimed
/// stations between two claimed ones claimed as well, unless the stretch
/// turns (the sum of `turn` over it, in degrees) by `cap_deg` or more
/// either way.
pub fn claim_between(mut claimed: Vec<bool>, turn: &[f64], max: usize, cap_deg: f64) -> Vec<bool> {
    let n = claimed.len();
    let Some(first) = claimed.iter().position(|&c| c) else {
        return claimed;
    };
    let mut fill: Vec<usize> = Vec::new();
    let mut run: Vec<usize> = Vec::new();
    let mut flush = |run: &mut Vec<usize>| {
        let total: f64 = run.iter().map(|&i| turn[i]).sum();
        if run.len() <= max && total.abs() < cap_deg {
            fill.extend(run.drain(..));
        }
        run.clear();
    };
    let mut i = (first + 1) % n;
    while i != first {
        if claimed[i] {
            flush(&mut run);
        } else {
            run.push(i);
        }
        i = (i + 1) % n;
    }
    flush(&mut run);
    for i in fill {
        claimed[i] = true;
    }
    claimed
}
