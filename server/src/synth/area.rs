//! Intersection areas — the paved region where roads meet (docs/ROADS.md H3,
//! invariant 2).
//!
//! An intersection's paved surface is the union of its legs: one rectangle per
//! road leaving the centre, as wide as that carriageway and long enough to
//! reach the far edge of the widest road through the junction. **Every one of
//! those rectangles contains the centre, so their union is star-shaped about
//! it**, and a star-shaped region is fully described by one function: its
//! boundary radius per direction.
//!
//! That single fact is what keeps this module small. The boundary ring, the
//! point test, and the band trim are three readings of the same radius, with
//! no polygon boolean, no orientation predicate, and no configuration that
//! needs a fallback: legs may be near-parallel, duplicated, five to a
//! junction, or wider than the intersection is long, and the radial maximum is
//! defined and finite for every one of them (docs/DESIGN.md — define errors
//! out of existence). The ring is star-shaped by construction, so fanning it
//! from the centre always yields a valid mesh.
//!
//! Distances are metres in a local ENU frame about the centre; the caller
//! converts to lon/lat with the frame the area carries.

use geo_types::Coord;

use crate::scene::DEG_M;

/// A road leaving the intersection: the unit ENU heading out of the centre and
/// the half-width of the carriageway that runs along it.
#[derive(Debug, Clone, Copy)]
pub struct Leg {
    pub e: f64,
    pub n: f64,
    pub half_w: f64,
}

/// Radial slack, in metres, within which a candidate point counts as *on* the
/// boundary rather than inside it. Absorbs the float error of a rectangle
/// corner evaluated through the radius function that produced it.
const ON_BOUNDARY_EPS_M: f64 = 1e-6;

/// Two ring points closer than this in metres are the same point — duplicate
/// legs and coincident corners collapse instead of emitting slivers.
const DEDUP_M: f64 = 1e-3;

/// The paved area of one intersection: a star-shaped region about `centre`.
pub struct Area {
    centre: Coord,
    m_per_deg_lon: f64,
    legs: Vec<Leg>,
    /// How far every leg rectangle runs from the centre, metres.
    reach: f64,
    /// The boundary, counter-clockwise, as ENU metre offsets from the centre.
    /// Baked once (heights aside, the plan shape is a pure function of the
    /// model) and stored `f32` — a centimetre is far below the quantization
    /// these coordinates end up in, and there is one of these per junction in
    /// the extract.
    ring: Vec<(f32, f32)>,
}

impl Area {
    /// The area of an intersection at `centre` with these legs, or `None` when
    /// it has no extent at all. Legs with a degenerate heading or a
    /// non-positive width are dropped.
    pub fn new(centre: Coord, legs: Vec<Leg>, reach: f64) -> Option<Area> {
        let legs: Vec<Leg> = legs
            .into_iter()
            .filter_map(|l| {
                let len = (l.e * l.e + l.n * l.n).sqrt();
                (len > 1e-9 && l.half_w > 0.0)
                    .then_some(Leg { e: l.e / len, n: l.n / len, half_w: l.half_w })
            })
            .collect();
        if legs.is_empty() || reach <= 0.0 {
            return None;
        }
        let mut area = Area {
            centre,
            m_per_deg_lon: DEG_M * centre.y.to_radians().cos(),
            legs,
            reach,
            ring: Vec::new(),
        };
        area.ring = area.build_ring();
        (area.ring.len() >= 3).then_some(area)
    }

    /// The intersection centre.
    pub fn centre(&self) -> Coord {
        self.centre
    }

    /// The ENU metre offset of a world point from the centre.
    pub fn offset_m(&self, c: Coord) -> (f64, f64) {
        ((c.x - self.centre.x) * self.m_per_deg_lon, (c.y - self.centre.y) * DEG_M)
    }

    /// The world point at an ENU metre offset from the centre.
    pub fn point_at(&self, de: f64, dn: f64) -> Coord {
        Coord { x: self.centre.x + de / self.m_per_deg_lon, y: self.centre.y + dn / DEG_M }
    }

    /// The boundary, counter-clockwise, as ENU metre offsets from the centre.
    pub fn ring(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.ring.iter().map(|&(e, n)| (e as f64, n as f64))
    }

    /// Half the area's bounding box in degrees, `(lon, lat)` — what a spatial
    /// index needs to know how far this intersection can reach from its centre.
    pub fn reach_deg(&self) -> (f64, f64) {
        let (mut e_max, mut n_max) = (0.0f64, 0.0f64);
        for &(e, n) in &self.ring {
            e_max = e_max.max((e as f64).abs());
            n_max = n_max.max((n as f64).abs());
        }
        (e_max / self.m_per_deg_lon, n_max / DEG_M)
    }

    /// How far the boundary lies from the centre along the unit direction
    /// `(e, n)` — the radial maximum over the legs. A leg
    /// rectangle spans `[0, reach]` along its heading and `±half_w` across, so
    /// the ray leaves it at whichever of those four walls it reaches first;
    /// a ray heading backwards out of a leg never enters it at all.
    pub fn radius(&self, e: f64, n: f64) -> f64 {
        let mut r = 0.0f64;
        for leg in &self.legs {
            let along = e * leg.e + n * leg.n;
            let across = e * -leg.n + n * leg.e;
            if along < 0.0 {
                continue; // behind this leg: the rectangle is not on this ray
            }
            let mut t = f64::INFINITY;
            if along > 0.0 {
                t = t.min(self.reach / along);
            }
            if across != 0.0 {
                t = t.min(leg.half_w / across.abs());
            }
            if t.is_finite() {
                r = r.max(t);
            }
        }
        r
    }

    /// Whether a world point is paved.
    pub fn contains(&self, c: Coord) -> bool {
        let (de, dn) = self.offset_m(c);
        let d = (de * de + dn * dn).sqrt();
        d <= 1e-9 || d <= self.radius(de / d, dn / d)
    }

    /// Appends the parameter intervals of the chord `a → b` that fall inside
    /// the area, as fractions of the chord: one per leg, a rectangle clip.
    /// They may overlap or be disjoint, and the caller merges them — a chord
    /// can leave a star-shaped region and re-enter it, which is exactly what
    /// the circular trim this replaced could not express.
    pub fn clip_chord(&self, a: Coord, b: Coord, out: &mut Vec<(f64, f64)>) {
        let f = self.offset_m(a);
        let g = self.offset_m(b);
        let d = (g.0 - f.0, g.1 - f.1);
        if d.0 * d.0 + d.1 * d.1 < 1e-18 {
            return;
        }
        for leg in &self.legs {
            // The chord in the leg's frame: `along` its heading, `across` it.
            let f_along = f.0 * leg.e + f.1 * leg.n;
            let d_along = d.0 * leg.e + d.1 * leg.n;
            let f_across = f.0 * -leg.n + f.1 * leg.e;
            let d_across = d.0 * -leg.n + d.1 * leg.e;
            let mut lo = 0.0f64;
            let mut hi = 1.0f64;
            let slab = |v0: f64, dv: f64, min: f64, max: f64, lo: &mut f64, hi: &mut f64| {
                if dv.abs() < 1e-12 {
                    if v0 < min || v0 > max {
                        *lo = 1.0;
                        *hi = 0.0; // parallel and outside: no overlap
                    }
                    return;
                }
                let (t0, t1) = ((min - v0) / dv, (max - v0) / dv);
                *lo = lo.max(t0.min(t1));
                *hi = hi.min(t0.max(t1));
            };
            slab(f_along, d_along, 0.0, self.reach, &mut lo, &mut hi);
            slab(f_across, d_across, -leg.half_w, leg.half_w, &mut lo, &mut hi);
            if hi > lo {
                out.push((lo, hi));
            }
        }
    }

    /// The boundary ring: every candidate vertex of the union that survives
    /// the radius test, sorted by bearing. A star-shaped region's boundary is
    /// monotone in bearing, so sorting *is* the ordering — no traversal, no
    /// winding rule. The candidates are the rectangle corners and the crossings
    /// between rectangle edges — the points where the leg bounding the union
    /// changes.
    fn build_ring(&self) -> Vec<(f32, f32)> {
        let mut cand: Vec<(f64, f64)> = Vec::new();
        for leg in &self.legs {
            let (pe, pn) = (-leg.n, leg.e);
            for &(along, across) in
                &[(0.0, leg.half_w), (self.reach, leg.half_w), (self.reach, -leg.half_w), (0.0, -leg.half_w)]
            {
                cand.push((leg.e * along + pe * across, leg.n * along + pn * across));
            }
        }
        // Where two legs' rectangles cross, the boundary switches from one to
        // the other; that crossing is a vertex of the union.
        for i in 0..self.legs.len() {
            for j in (i + 1)..self.legs.len() {
                self.edge_crossings(&self.legs[i], &self.legs[j], &mut cand);
            }
        }

        // Keep only what is actually on the boundary: a candidate swallowed by
        // another part of the union is interior and would fold the ring.
        let mut ring: Vec<(f64, f64, f64)> = Vec::with_capacity(cand.len());
        for (e, n) in cand {
            let d = (e * e + n * n).sqrt();
            if d < 1e-9 {
                continue;
            }
            if d + ON_BOUNDARY_EPS_M >= self.radius(e / d, n / d) {
                ring.push((n.atan2(e), e, n));
            }
        }
        ring.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut pts: Vec<(f64, f64)> = Vec::with_capacity(ring.len());
        for (_, e, n) in ring {
            if let Some(&(pe, pn)) = pts.last() {
                if (e - pe).hypot(n - pn) < DEDUP_M {
                    continue;
                }
            }
            pts.push((e, n));
        }
        if let (Some(&first), Some(&last)) = (pts.first(), pts.last()) {
            if pts.len() > 1 && (first.0 - last.0).hypot(first.1 - last.1) < DEDUP_M {
                pts.pop();
            }
        }
        drop_collinear(&mut pts);
        pts.into_iter().map(|(e, n)| (e as f32, n as f32)).collect()
    }

    /// Appends the points where the *sides* of two leg rectangles cross. Only
    /// the long sides matter: the back walls pass through the centre, which is
    /// interior to every other leg, so they never bound the union.
    fn edge_crossings(&self, a: &Leg, b: &Leg, out: &mut Vec<(f64, f64)>) {
        for &sa in &[1.0f64, -1.0] {
            for &sb in &[1.0f64, -1.0] {
                // Side of `a`: from (0, sa·w_a) along a's heading, and likewise b.
                let pa = (-a.n * sa * a.half_w, a.e * sa * a.half_w);
                let pb = (-b.n * sb * b.half_w, b.e * sb * b.half_w);
                let denom = a.e * b.n - a.n * b.e;
                if denom.abs() < 1e-9 {
                    continue; // parallel sides never cross
                }
                let (re, rn) = (pb.0 - pa.0, pb.1 - pa.1);
                let ta = (re * b.n - rn * b.e) / denom;
                let tb = (re * a.n - rn * a.e) / denom;
                if ta < 0.0 || ta > self.reach || tb < 0.0 || tb > self.reach {
                    continue; // the crossing is off one of the two rectangles
                }
                out.push((pa.0 + a.e * ta, pa.1 + a.n * ta));
            }
        }
    }
}

/// Removes ring points that sit on the straight line between their
/// neighbours: a leg's long side is sampled by both of its own corners and by
/// every crossing along it, and only the ends of that side are geometry. A
/// four-way of equal legs comes out of this as four corners, not sixteen
/// collinear ones — the mesh a plate fans is that much smaller.
fn drop_collinear(pts: &mut Vec<(f64, f64)>) {
    if pts.len() < 4 {
        return;
    }
    let mut keep = vec![true; pts.len()];
    let mut prev = pts.len() - 1;
    for i in 0..pts.len() {
        let next = (i + 1) % pts.len();
        let (ax, ay) = pts[prev];
        let (bx, by) = pts[i];
        let (cx, cy) = pts[next];
        let span = (cx - ax).hypot(cy - ay);
        // Distance from b to the chord a→c: twice the triangle area over its
        // base. Below a millimetre, b is on the side and carries no shape.
        if span > 1e-9 && ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax)).abs() / span < DEDUP_M {
            keep[i] = false;
        } else {
            prev = i;
        }
    }
    let mut i = 0;
    pts.retain(|_| {
        i += 1;
        keep[i - 1]
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Four 8 m-wide legs (half-width 4 m) at the compass points.
    fn cross() -> Area {
        let legs = vec![
            Leg { e: 1.0, n: 0.0, half_w: 4.0 },
            Leg { e: -1.0, n: 0.0, half_w: 4.0 },
            Leg { e: 0.0, n: 1.0, half_w: 4.0 },
            Leg { e: 0.0, n: -1.0, half_w: 4.0 },
        ];
        Area::new(Coord { x: 6.0, y: 46.0 }, legs, 4.0).expect("an area")
    }

    #[test]
    fn a_four_way_of_equal_legs_is_the_square_they_share() {
        let a = cross();
        // Along a leg the boundary is its far wall; on the diagonal it is the
        // corner of the square, √2 further out.
        assert!((a.radius(1.0, 0.0) - 4.0).abs() < 1e-9);
        assert!((a.radius(0.0, -1.0) - 4.0).abs() < 1e-9);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!((a.radius(s, s) - 4.0 * 2.0f64.sqrt()).abs() < 1e-6);
        // Four corners, nothing else: the back walls and the candidates
        // swallowed by another leg are dropped, and the collinear samples
        // along each side collapse into the side.
        let ring: Vec<(f64, f64)> = a.ring().collect();
        assert_eq!(ring.len(), 4, "ring {ring:?}");
        for &(e, n) in &ring {
            assert!((e.abs().max(n.abs()) - 4.0).abs() < 1e-6, "{e},{n} off the square");
            assert!((e.abs().min(n.abs()) - 4.0).abs() < 1e-6, "{e},{n} not a corner");
        }
    }

    #[test]
    fn the_ring_is_star_shaped_and_wound_counter_clockwise() {
        // Deliberately awkward: five legs, two near-parallel, one far wider.
        let legs = vec![
            Leg { e: 1.0, n: 0.0, half_w: 5.5 },
            Leg { e: -1.0, n: 0.05, half_w: 5.5 },
            Leg { e: -0.99, n: -0.1, half_w: 1.5 },
            Leg { e: 0.1, n: 1.0, half_w: 2.5 },
            Leg { e: 0.0, n: -1.0, half_w: 3.0 },
        ];
        let a = Area::new(Coord { x: 6.0, y: 46.0 }, legs, 5.5).expect("an area");
        let ring: Vec<(f64, f64)> = a.ring().collect();
        assert!(ring.len() >= 4);
        // Bearings strictly increase: the ring never folds back on itself.
        let mut prev = f64::NEG_INFINITY;
        for &(e, n) in &ring {
            let ang = n.atan2(e);
            assert!(ang > prev, "ring not monotone in bearing at {e},{n}");
            prev = ang;
            // Every ring point is on the boundary its own radius reports.
            let d = (e * e + n * n).sqrt();
            assert!((d - a.radius(e / d, n / d)).abs() < 1e-6, "point {e},{n} off the boundary");
        }
        // Counter-clockwise: the signed area of a star-shaped ring is positive.
        let area: f64 = (0..ring.len())
            .map(|i| {
                let (x0, y0) = ring[i];
                let (x1, y1) = ring[(i + 1) % ring.len()];
                x0 * y1 - x1 * y0
            })
            .sum();
        assert!(area > 0.0, "ring is clockwise");
    }

    #[test]
    fn containment_matches_the_radius_everywhere() {
        let a = cross();
        assert!(a.contains(a.point_at(0.0, 0.0)), "the centre is paved");
        assert!(a.contains(a.point_at(3.9, 3.9)), "inside the square");
        assert!(!a.contains(a.point_at(4.1, 4.1)), "outside the square");
        assert!(a.contains(a.point_at(0.0, -3.9)));
        assert!(!a.contains(a.point_at(0.0, -4.1)));
    }

    #[test]
    fn a_chord_through_the_area_clips_to_its_crossing() {
        let a = cross();
        // Due west to due east through the centre: inside over the middle
        // 8 m of a 40 m chord, i.e. fractions 0.4 to 0.6.
        let mut out = Vec::new();
        a.clip_chord(a.point_at(-20.0, 0.0), a.point_at(20.0, 0.0), &mut out);
        let lo = out.iter().map(|i| i.0).fold(f64::INFINITY, f64::min);
        let hi = out.iter().map(|i| i.1).fold(f64::NEG_INFINITY, f64::max);
        assert!((lo - 0.4).abs() < 1e-6, "enters at {lo}");
        assert!((hi - 0.6).abs() < 1e-6, "leaves at {hi}");
        // A chord that misses entirely clips to nothing.
        out.clear();
        a.clip_chord(a.point_at(-20.0, 9.0), a.point_at(20.0, 9.0), &mut out);
        assert!(out.iter().all(|&(lo, hi)| hi <= lo), "a passing chord clipped {out:?}");
    }

    #[test]
    fn degenerate_input_yields_no_area() {
        let c = Coord { x: 6.0, y: 46.0 };
        assert!(Area::new(c, Vec::new(), 5.0).is_none(), "no legs at all");
        let zero = vec![Leg { e: 0.0, n: 0.0, half_w: 4.0 }, Leg { e: 1.0, n: 0.0, half_w: 0.0 }];
        assert!(Area::new(c, zero, 5.0).is_none(), "every leg degenerate");
        // A single leg is a rectangle — a valid, if pointless, area.
        let one = vec![Leg { e: 1.0, n: 0.0, half_w: 4.0 }];
        assert!(Area::new(c, one, 5.0).is_some());
    }
}
