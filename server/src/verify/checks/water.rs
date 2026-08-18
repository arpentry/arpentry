//! Invariant 4's water clause: **watercourses descend along flow**
//! (docs/GENERATION.md §4.2 H).
//!
//! Flowing water is drawn draped — its vertical profile *is* the drawn
//! terrain along its line — so this is a check on the ground the viewer
//! actually sees under a river, and it is the first instrument for the H
//! stratum's watercourse half (the still-body flatten has `ground.footprint`
//! and the seam checks; descent had nothing).
//!
//! The measure is **ascent above the running minimum along flow**: walking
//! the line downstream, how far the drawn surface stands above the lowest
//! point already passed. That is the depth water would have to pond to get
//! past this point, so it reads directly as "the stream climbs X m here". It
//! is deliberately not step-wise rise — a long false climb of many gentle
//! steps is one defect, not fifty small ones.
//!
//! Flow direction is not trusted from the data. Overture inherits OSM's
//! convention that waterways point downstream, but one reversed line would
//! read as a single climb its whole length, so each part is oriented by its
//! own net drop before walking. A genuinely monotone line scores zero under
//! either orientation.
//!
//! Two absences are meaningful and kept:
//!
//! - A sample where the terrain answers nothing (the hole under at-grade
//!   asphalt — a culvert crossing) is skipped, but the running minimum
//!   carries across the gap: the stream does not forget its level because it
//!   passed under a road.
//! - Parts are walked whole, buffer included, but only samples the tile owns
//!   are recorded — the standard ownership rule, with the buffer giving the
//!   walk upstream context at the border. The minimum does not carry between
//!   tiles, so a climb spanning many tiles is under-read, never over-read.

use crate::verify::dist::Dist;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// How much ascent along flow is measurement rather than defect.
///
/// Read off the measured population before gating (the census discipline):
/// the drawn terrain is a DEM lattice, the line's plan position is a mapped
/// centerline that wanders off the thalweg, and both contribute decimetre
/// oscillation on legitimate ground. Measured over the Montreux extract at
/// z16 (67,639 samples): p50 0.03 m, p90 0.08 m, p95 0.40 m — the noise band
/// ends in decimetres — then p99 2.27 m and a worst of 8.7 m. One metre sits
/// 2.5× clear of the band's edge; the 2.5 % above it is the manufactured
/// tail — a deck or embankment baked into the DTM across the water, a
/// culvert drawn as a dam, or a gorge wall sampled where the line leaves the
/// channel.
const ASCENT_M: f64 = 1.0;

pub struct Water {
    ascent: Dist,
    worst: Worst,
}

impl Water {
    pub fn new(opt: &Options) -> Water {
        // Gorge walls can put a line tens of metres up a flank; ±32 m
        // saturates (the structures lesson), so the range is wider.
        Water { ascent: Dist::new(0.0, 256.0), worst: Worst::new(Sense::HigherIsWorse, opt.worst_k) }
    }
}

impl Check for Water {
    fn visit(&mut self, tile: &TileScene, opt: &Options) {
        let Some(terrain) = &tile.terrain else { return };
        for line in &tile.waters {
            for part in &line.parts {
                // Sample the drawn ground along the line at the sweep spacing,
                // vertices included, so a climb between two far-apart mapped
                // vertices is still seen.
                let mut pts: Vec<(f64, f64)> = Vec::new();
                for w in part.windows(2) {
                    let ((ax, ay), (bx, by)) = (w[0], w[1]);
                    let steps =
                        (tile.scale.dist(ax, ay, bx, by) / opt.spacing_m).ceil().max(1.0) as usize;
                    for s in 0..steps {
                        let t = s as f64 / steps as f64;
                        pts.push((ax + (bx - ax) * t, ay + (by - ay) * t));
                    }
                }
                pts.push(*part.last().unwrap());
                let heights: Vec<Option<f64>> =
                    pts.iter().map(|&(x, y)| terrain.height_at(x, y)).collect();
                for (i, rise) in ascents(&heights) {
                    let (px, py) = pts[i];
                    if !tile.owns(px, py) {
                        continue;
                    }
                    self.ascent.push(rise);
                    if rise > ASCENT_M {
                        let (lon, lat) = tile.lonlat(px, py);
                        self.worst.offer(Offender {
                            lon,
                            lat,
                            zoom: tile.z,
                            value: rise,
                            note: format!(
                                "the drawn ground under this {} stands {rise:.1} m above the \
                                 level the water already reached",
                                line.class
                            ),
                        });
                    }
                }
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![Metric {
            id: "water.descends".into(),
            invariant: Invariant::I4,
            title: "Watercourse climbing along its own flow".into(),
            population: format!(
                "Every {:.0} m-spaced sample of the drawn terrain along every flowing-water \
                 centerline (river, stream, canal) the tile owns, scored by ascent above the \
                 running minimum along flow. Direction is inferred per part from its net drop, \
                 so a reversed line cannot read as a climb; samples over the pavement hole are \
                 skipped with the minimum carried across.",
                1.0
            ),
            detail: "Water level is set by gravity and catchment (§4.2 H): the drawn ground \
                     under a watercourse may pause but never rise along flow. What rises is a \
                     deck or embankment the DTM baked in across the water, a culvert drawn as a \
                     dam, or the line sampling a gorge wall — each a manufactured climb the \
                     monotone conditioning owes a fix, and the exact class S3's freeboard will \
                     read as the water datum if it is left standing."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: ASCENT_M,
            skipped: self
                .ascent
                .is_empty()
                .then(|| "no flowing watercourse over drawn terrain".to_string()),
            dist: self.ascent,
            worst: self.worst.into_vec(),
        }]
    }
}

/// Per-sample ascent above the running minimum, walking `heights` in flow
/// order. Flow is inferred from the net drop between the first and last
/// answered samples: water runs downhill overall, whatever the digitising
/// direction. Returns `(index into heights, ascent)` for every answered
/// sample; a `None` (no terrain — the pavement hole at a culvert) is skipped
/// with the minimum kept.
fn ascents(heights: &[Option<f64>]) -> Vec<(usize, f64)> {
    let known: Vec<(usize, f64)> =
        heights.iter().enumerate().filter_map(|(i, h)| h.map(|h| (i, h))).collect();
    let (Some(&(_, first)), Some(&(_, last))) = (known.first(), known.last()) else {
        return Vec::new();
    };
    let downstream: Box<dyn Iterator<Item = &(usize, f64)>> =
        if first >= last { Box::new(known.iter()) } else { Box::new(known.iter().rev()) };
    let mut min = f64::INFINITY;
    let mut out = Vec::with_capacity(known.len());
    for &(i, h) in downstream {
        min = min.min(h);
        out.push((i, h - min));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn h(v: &[f64]) -> Vec<Option<f64>> {
        v.iter().map(|&x| Some(x)).collect()
    }

    /// A stream that only descends owes nothing.
    #[test]
    fn a_descending_stream_scores_zero() {
        for (_, rise) in ascents(&h(&[10.0, 9.0, 9.0, 7.5, 2.0])) {
            assert_eq!(rise, 0.0);
        }
    }

    /// A bump along flow scores its own height over the level already reached.
    #[test]
    fn a_bump_scores_its_height() {
        let out = ascents(&h(&[10.0, 8.0, 9.5, 8.0, 7.0]));
        assert_eq!(out[2], (2, 1.5));
        assert_eq!(out[3], (3, 0.0));
    }

    /// A line digitised uphill is walked downhill: monotone either way is
    /// zero, and the bump lands on the same sample.
    #[test]
    fn orientation_follows_the_net_drop() {
        for (_, rise) in ascents(&h(&[2.0, 7.5, 9.0, 10.0])) {
            assert_eq!(rise, 0.0);
        }
        let out = ascents(&h(&[7.0, 8.0, 9.5, 8.0, 10.0]));
        assert!(out.contains(&(2, 1.5)), "bump at index 2, got {out:?}");
    }

    /// The running minimum carries across a hole (a culvert under asphalt):
    /// the stream does not forget its level because it passed under a road.
    #[test]
    fn the_minimum_survives_a_gap() {
        let out = ascents(&[Some(10.0), Some(6.0), None, Some(8.0)]);
        assert_eq!(out.len(), 3);
        assert!(out.contains(&(3, 2.0)), "climb after the culvert, got {out:?}");
    }
}
