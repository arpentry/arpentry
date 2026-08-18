//! The B stratum's first instrument: **does the drawn ground meet the
//! building's designed seat** (S11 — a building on a steep slope meets the
//! ground on every side; S13 — a building beside a cut or embankment).
//!
//! The mesher's contract is explicit (`building_mesh`): walls anchor at the
//! highest stack ground under the footprint and the foundation sinks past the
//! lowest by a margin, so the ground surface should cross the foundation band
//! `[foot, foot + relief + margin]` at every perimeter point. The band is
//! recoverable from the archive alone — the mesh's own lowest ring is the
//! foot, and the tiler's `ground_relief` property carries the spread — so the
//! check measures how far the *drawn* terrain escapes it: above the band the
//! walls are swallowed, below it there is daylight under the building.
//!
//! What makes this worth measuring when the contract holds by construction:
//! the stamping samples the ground *stack*, and the viewer sees the ground
//! *mesh* — a per-zoom lattice with holes, aprons and bench chords. Every
//! defect this check can find is a disagreement between those two, which is
//! exactly the class S13 names (the building beside a road cut whose bench
//! reshaped the ground after the footprint sampled it).

use crate::verify::dist::Dist;
use crate::verify::scene::TileScene;
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::{Check, Options};

/// How far outside the designed seat band the drawn ground may sit before it
/// reads as a defect rather than as lattice-versus-stack noise.
///
/// Calibrated on the measured population before gating; see the metric's
/// population text for the read.
const SEAT_M: f64 = 1.0;

/// A vertex within this of the mesh's lowest is part of the foundation's
/// bottom ring. The ring is flat by construction (one `foot_z` per building),
/// so the tolerance only absorbs quantization.
const FOOT_EPS_M: f64 = 0.05;

pub struct Building {
    seat: Dist,
    worst: Worst,
}

impl Building {
    pub fn new(opt: &Options) -> Building {
        Building { seat: Dist::new(0.0, 256.0), worst: Worst::new(Sense::HigherIsWorse, opt.worst_k) }
    }
}

impl Check for Building {
    fn visit(&mut self, tile: &TileScene, _opt: &Options) {
        let Some(terrain) = &tile.terrain else { return };
        for (relief_m, mesh) in &tile.buildings {
            let mut foot = f64::INFINITY;
            for i in 0..mesh.vertex_count() {
                foot = foot.min(mesh.vertex(i).2);
            }
            if !foot.is_finite() {
                continue;
            }
            let base = foot + relief_m + crate::building_mesh::FOUNDATION_MARGIN_MM as f64 / 1000.0;
            for i in 0..mesh.vertex_count() {
                let (x, y, z) = mesh.vertex(i);
                if z > foot + FOOT_EPS_M || !tile.owns(x, y) {
                    continue;
                }
                // The hole under pavement answers nothing; a building footing
                // inside it is its own (future) story, not this one.
                let Some(t) = terrain.height_at(x, y) else { continue };
                let outside = (foot - t).max(t - base).max(0.0);
                self.seat.push(outside);
                if outside > SEAT_M {
                    let (lon, lat) = tile.lonlat(x, y);
                    let side = if t < foot { "daylight under the foundation" } else { "walls buried" };
                    self.worst.offer(Offender {
                        lon,
                        lat,
                        zoom: tile.z,
                        value: outside,
                        note: format!(
                            "{side}: drawn ground {outside:.1} m outside the building's seat \
                             band ({:.0} m relief)",
                            relief_m
                        ),
                    });
                }
            }
        }
    }

    fn finish(self: Box<Self>) -> Vec<Metric> {
        vec![Metric {
            id: "contact.building_seat".into(),
            invariant: Invariant::I4,
            title: "Drawn ground escaping a building's seat band".into(),
            population: "Every bottom-ring vertex of every building mesh the tile owns, over \
                         drawn terrain, scored by how far the ground sits outside the designed \
                         foundation band [foot, foot + relief + margin] — below is daylight \
                         under the building, above is swallowed walls. Measured over the \
                         Montreux extract at z16 (109,456 vertices): p95 0.03 m — the contract \
                         holds to quantization almost everywhere — p99.9 0.40 m, then 98 \
                         samples past 0.5 m to a worst of 3.5 m where a bench or hole reshaped \
                         the ground after the footprint sampled it. The metre gate sits clear \
                         of the band with headroom for steeper extracts."
                .into(),
            detail: "The mesher's own contract, checked against what the viewer sees: walls \
                     anchor at the highest stack ground and the foundation sinks past the \
                     lowest (building_mesh), so the drawn surface should cross the band at \
                     every perimeter point. What escapes it is the stamping's ground stack \
                     disagreeing with the drawn per-zoom mesh — S13's building beside a cut, \
                     and the first measured number the B stratum has."
                .into(),
            sense: Sense::HigherIsWorse,
            threshold: SEAT_M,
            skipped: self
                .seat
                .is_empty()
                .then(|| "no building mesh over drawn terrain".to_string()),
            dist: self.seat,
            worst: self.worst.into_vec(),
        }]
    }
}
