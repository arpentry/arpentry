//! Procedural town layout centred at (0, 0) (port of `gen/town.c`).
//!
//! An 8×8 jittered grid of intersections produces road segments with visible
//! bends; buildings line both sides of each road. The whole town is generated
//! once, lazily, from a fixed-seed xorshift32 PRNG — so the exact sequence of
//! PRNG draws (and therefore the layout) is identical to the C server.

use std::sync::OnceLock;

use crate::project::Bounds;

const DEG_PER_M: f64 = 1.0 / 111_319.5;

// Town bounding box (~1800 m, covering node jitter + building setbacks).
const TOWN_WEST: f64 = -0.0080;
const TOWN_EAST: f64 = 0.0080;
const TOWN_SOUTH: f64 = -0.0080;
const TOWN_NORTH: f64 = 0.0080;

// Grid parameters.
const GRID_N: usize = 8;
const GRID_SPAN_M: f64 = 1492.0;
const GRID_CELL_M: f64 = GRID_SPAN_M / (GRID_N as f64 - 1.0); // ~213 m
const JITTER_FRAC: f64 = 0.25;

// Road parameters.
const PRIMARY_IDX: usize = 3;
const SKIP_FRAC: f64 = 0.20;
const BEND_FRAC: f64 = 0.15;

// Building placement.
const BLDG_STEP_M: f64 = 30.0;
const SETBACK_MIN: f64 = 3.0;
const SETBACK_MAX: f64 = 8.0;
const BLDG_W_MIN: f64 = 10.0;
const BLDG_W_MAX: f64 = 28.0;
const BLDG_D_MIN: f64 = 8.0;
const BLDG_D_MAX: f64 = 22.0;
const TOWN_HALF_M: f64 = 750.0;

const MAX_ROADS: usize = 256;
const MAX_BUILDINGS: usize = 512;

// Town value indices into the tile-scope value dictionary.
pub const TOWN_VAL_PRIMARY: u32 = 7;
pub const TOWN_VAL_RESIDENTIAL: u32 = 8;
pub const TOWN_VAL_BUILDING: u32 = 9;
pub const TOWN_VAL_H5: u32 = 10;
pub const TOWN_VAL_H8: u32 = 11;
pub const TOWN_VAL_H10: u32 = 12;
pub const TOWN_VAL_H12: u32 = 13;
pub const TOWN_VAL_H15: u32 = 14;

/// Town key index into the tile-scope key dictionary.
pub const TOWN_KEY_HEIGHT: u32 = 1;

/// Street names cycled across grid rows and columns (rows take the first
/// half, columns the second), exercising the client's line-following labels.
pub const STREET_NAMES: &[&str] = &[
    "Grand-Rue",
    "Rue de la Gare",
    "Avenue des Alpes",
    "Chemin des Vignes",
    "Rue du Lac",
    "Route des Moulins",
    "Rue des Tilleuls",
    "Avenue de la Poste",
    "Rue du Marché",
    "Chemin des Écoliers",
    "Rue de la Fontaine",
    "Route de Montreux",
    "Rue des Remparts",
    "Avenue du Théâtre",
    "Rue du Château",
    "Chemin du Verger",
];

/// One named road: a polyline through the grid-edge bend (degrees).
pub struct TownRoad {
    pub pts: Vec<(f64, f64)>, // (lon, lat)
    pub cls: u32,
    pub name_idx: u32, // index into STREET_NAMES
}

/// A building footprint (centre in degrees, dimensions in metres).
pub struct TownBuilding {
    pub lon: f64,
    pub lat: f64,
    pub w_m: f64,
    pub h_m: f64,
    pub cls: u32,
    pub height_val: u32,
}

/// The generated town: roads + buildings.
pub struct Town {
    pub roads: Vec<TownRoad>,
    pub buildings: Vec<TownBuilding>,
}

// ── PRNG (xorshift32) ───────────────────────────────────────────────────

struct Rng {
    state: u32,
}

impl Rng {
    fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    fn unit(&mut self) -> f64 {
        (self.next_u32() & 0x7FFF_FFFF) as f64 / 2_147_483_648.0
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.unit() * (hi - lo)
    }
}

// ── Generation ──────────────────────────────────────────────────────────

fn gen_nodes(rng: &mut Rng) -> [[(f64, f64); GRID_N]; GRID_N] {
    let half = GRID_SPAN_M * 0.5;
    let jitter = GRID_CELL_M * JITTER_FRAC;
    let mut nodes = [[(0.0, 0.0); GRID_N]; GRID_N];
    for r in 0..GRID_N {
        for c in 0..GRID_N {
            let x = -half + c as f64 * GRID_CELL_M + rng.range(-jitter, jitter);
            let y = -half + r as f64 * GRID_CELL_M + rng.range(-jitter, jitter);
            nodes[r][c] = (x, y);
        }
    }
    nodes
}

/// Connects two nodes with a bend at the midpoint: one 3-point polyline.
fn add_edge(
    roads: &mut Vec<TownRoad>,
    rng: &mut Rng,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cls: u32,
    name_idx: u32,
) {
    let mut mx = (ax + bx) * 0.5;
    let mut my = (ay + by) * 0.5;
    let dx = bx - ax;
    let dy = by - ay;
    let len = (dx * dx + dy * dy).sqrt();
    if len > 0.01 {
        let off = rng.range(-BEND_FRAC, BEND_FRAC) * len;
        mx += (-dy / len) * off;
        my += (dx / len) * off;
    }
    if roads.len() >= MAX_ROADS {
        return;
    }
    let to_deg = |x: f64, y: f64| (x * DEG_PER_M, y * DEG_PER_M);
    roads.push(TownRoad {
        pts: vec![to_deg(ax, ay), to_deg(mx, my), to_deg(bx, by)],
        cls,
        name_idx,
    });
}

fn gen_roads(rng: &mut Rng, nodes: &[[(f64, f64); GRID_N]; GRID_N]) -> Vec<TownRoad> {
    let mut roads = Vec::new();
    // Horizontal edges: connect (r, c) to (r, c+1).
    for r in 0..GRID_N {
        for c in 0..GRID_N - 1 {
            let pri = r == PRIMARY_IDX;
            if !pri && rng.unit() < SKIP_FRAC {
                continue;
            }
            let cls = if pri { TOWN_VAL_PRIMARY } else { TOWN_VAL_RESIDENTIAL };
            let name_idx = (r % STREET_NAMES.len()) as u32;
            add_edge(&mut roads, rng, nodes[r][c].0, nodes[r][c].1, nodes[r][c + 1].0, nodes[r][c + 1].1, cls, name_idx);
        }
    }
    // Vertical edges: connect (r, c) to (r+1, c).
    for r in 0..GRID_N - 1 {
        for c in 0..GRID_N {
            let pri = c == PRIMARY_IDX;
            if !pri && rng.unit() < SKIP_FRAC {
                continue;
            }
            let cls = if pri { TOWN_VAL_PRIMARY } else { TOWN_VAL_RESIDENTIAL };
            let name_idx = ((GRID_N + c) % STREET_NAMES.len()) as u32;
            add_edge(&mut roads, rng, nodes[r][c].0, nodes[r][c].1, nodes[r + 1][c].0, nodes[r + 1][c].1, cls, name_idx);
        }
    }
    roads
}

/// Building height class by distance from the town centre.
fn choose_height(dist_m: f64) -> u32 {
    if dist_m < 150.0 {
        TOWN_VAL_H15
    } else if dist_m < 300.0 {
        TOWN_VAL_H12
    } else if dist_m < 450.0 {
        TOWN_VAL_H10
    } else if dist_m < 600.0 {
        TOWN_VAL_H8
    } else {
        TOWN_VAL_H5
    }
}

fn overlaps_any(bldgs: &[TownBuilding], cx: f64, cy: f64, w: f64, h: f64) -> bool {
    let hw = w * 0.5;
    let hh = h * 0.5;
    for b in bldgs {
        let ex = b.lon / DEG_PER_M;
        let ey = b.lat / DEG_PER_M;
        let ehw = b.w_m * 0.5;
        let ehh = b.h_m * 0.5;
        if cx - hw < ex + ehw && cx + hw > ex - ehw && cy - hh < ey + ehh && cy + hh > ey - ehh {
            return true;
        }
    }
    false
}

fn place_along_road(bldgs: &mut Vec<TownBuilding>, rng: &mut Rng, road: &TownRoad) {
    // One placement pass per polyline segment, in point order — preserving
    // the PRNG draw sequence of the earlier two-segments-per-edge layout.
    for seg in road.pts.windows(2) {
        place_along_segment(bldgs, rng, seg[0], seg[1]);
    }
}

fn place_along_segment(
    bldgs: &mut Vec<TownBuilding>,
    rng: &mut Rng,
    a: (f64, f64),
    b: (f64, f64),
) {
    let ax = a.0 / DEG_PER_M;
    let ay = a.1 / DEG_PER_M;
    let bx = b.0 / DEG_PER_M;
    let by = b.1 / DEG_PER_M;
    let dx = bx - ax;
    let dy = by - ay;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1.0 {
        return;
    }

    // Perpendicular to road direction.
    let px = -dy / len;
    let py = dx / len;
    let steps = (len / BLDG_STEP_M) as i32;

    for s in 1..steps {
        let t = s as f64 / steps as f64;
        let rx = ax + dx * t;
        let ry = ay + dy * t;

        // Try both sides of the road. The three PRNG draws happen before any
        // rejection test, so the sequence advances regardless of placement.
        for side in [-1.0f64, 1.0] {
            if bldgs.len() >= MAX_BUILDINGS {
                return;
            }
            let w = rng.range(BLDG_W_MIN, BLDG_W_MAX);
            let d = rng.range(BLDG_D_MIN, BLDG_D_MAX);
            let setback = rng.range(SETBACK_MIN, SETBACK_MAX);
            let off = setback + w.max(d) * 0.5;

            let cx = rx + side * px * off;
            let cy = ry + side * py * off;

            // Stay within town bounds.
            let hw = w * 0.5;
            let hd = d * 0.5;
            if cx - hw < -TOWN_HALF_M || cx + hw > TOWN_HALF_M {
                continue;
            }
            if cy - hd < -TOWN_HALF_M || cy + hd > TOWN_HALF_M {
                continue;
            }
            if overlaps_any(bldgs, cx, cy, w, d) {
                continue;
            }

            let dist = (cx * cx + cy * cy).sqrt();
            bldgs.push(TownBuilding {
                lon: cx * DEG_PER_M,
                lat: cy * DEG_PER_M,
                w_m: w,
                h_m: d,
                cls: TOWN_VAL_BUILDING,
                height_val: choose_height(dist),
            });
        }
    }
}

fn generate() -> Town {
    let mut rng = Rng { state: 0xBEEF_1234 };
    let nodes = gen_nodes(&mut rng);
    let roads = gen_roads(&mut rng, &nodes);
    let mut buildings = Vec::new();
    for road in &roads {
        place_along_road(&mut buildings, &mut rng, road);
    }
    Town { roads, buildings }
}

/// Returns the lazily generated town (built once per process).
pub fn town() -> &'static Town {
    static TOWN: OnceLock<Town> = OnceLock::new();
    TOWN.get_or_init(generate)
}

/// Whether the tile bounds overlap the procedural town area.
pub fn town_overlaps(bounds: &Bounds) -> bool {
    bounds.east > TOWN_WEST && bounds.west < TOWN_EAST && bounds.north > TOWN_SOUTH && bounds.south < TOWN_NORTH
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn town_is_stable_and_bounded() {
        let t = town();
        assert!(!t.roads.is_empty());
        assert!(t.roads.len() <= MAX_ROADS);
        assert!(t.buildings.len() <= MAX_BUILDINGS);
        // Building centres stay within the town half-extent.
        for b in &t.buildings {
            assert!((b.lon / DEG_PER_M).abs() <= TOWN_HALF_M + 1.0);
            assert!((b.lat / DEG_PER_M).abs() <= TOWN_HALF_M + 1.0);
        }
    }

    #[test]
    fn overlap_test_matches_box() {
        let inside = Bounds::of_tile(0, 0, 0);
        assert!(town_overlaps(&inside));
    }
}
