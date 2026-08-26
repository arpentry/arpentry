//! The checks, and the single pass that runs them.
//!
//! Each check is a small accumulator fed one decoded tile at a time and asked
//! for its metrics at the end. Splitting them this way means one archive read
//! serves all of them, and adding a check is a file plus a line in [`run`] —
//! low enough friction that a defect found by eye can become a permanent
//! measurement in the same sitting, which is the whole point.

pub mod abutment;
pub mod building;
pub mod clearance;
pub mod contact;
pub mod fabric;
pub mod handoff;
pub mod lod;
pub mod paint;
pub mod seams;
pub mod slope;
pub mod street;
pub mod water;

use crate::verify::scene::{ArchiveScan, TileScene};
use crate::verify::{Metric, Scorecard};

/// How the pass is scoped.
pub struct Options {
    /// Zooms to measure. Empty means "the archive's detail rung", which is
    /// where the road surface exists and where all the contested geometry is.
    pub zooms: Vec<u8>,
    /// Plan spacing of surface samples, in metres. One metre is fine enough to
    /// catch a chord across a carriageway and coarse enough to sweep a city.
    pub spacing_m: f64,
    /// How many worst offenders each metric keeps.
    pub worst_k: usize,
    /// Restrict to the tile containing this position — the corpus scenarios
    /// and the "what is happening *here*" question both use it.
    pub at: Option<(f64, f64)>,
    /// Cap on tiles visited per zoom, so a continental archive still answers in
    /// bounded time. Reported when it bites; a silent truncation would read as
    /// full coverage.
    pub max_tiles: usize,
}

impl Default for Options {
    fn default() -> Options {
        Options { zooms: Vec::new(), spacing_m: 1.0, worst_k: 8, at: None, max_tiles: 4096 }
    }
}

/// A check: fed tiles, then asked for what it measured.
pub trait Check {
    fn visit(&mut self, tile: &TileScene, opt: &Options);
    fn finish(self: Box<Self>) -> Vec<Metric>;
}

/// Runs every check over the archive and returns the scorecard.
pub fn run(scan: &ArchiveScan<'_>, opt: &Options) -> Scorecard {
    let zooms = if opt.zooms.is_empty() { vec![scan.max_zoom()] } else { opt.zooms.clone() };

    let mut checks: Vec<Box<dyn Check>> = vec![
        Box::new(abutment::Abutment::new(opt)),
        Box::new(building::Building::new(opt)),
        Box::new(handoff::Handoff::new(opt)),
        Box::new(contact::Contact::new(opt)),
        Box::new(clearance::Clearance::new(opt)),
        Box::new(fabric::Fabric::new(opt)),
        Box::new(paint::Paint::new(opt)),
        Box::new(seams::Seams::new(opt)),
        Box::new(slope::Slope::new(opt)),
        Box::new(street::Street::new(opt)),
        Box::new(water::Water::new(opt)),
    ];

    let mut visited = 0usize;
    let mut truncated = false;
    // The extent actually measured, grown tile by tile. Recorded on the
    // scorecard because a metric is only comparable against one taken over the
    // same ground: two archives covering different bboxes differ in every
    // population, and that difference reads exactly like a change in the
    // geometry.
    let mut bbox: Option<(f64, f64, f64, f64)> = None;
    for &z in &zooms {
        let mut tiles = scan.tiles_at(z);
        if let Some((lon, lat)) = opt.at {
            tiles.retain(|&(tz, tx, ty, _)| {
                let b = crate::project::Bounds::of_tile(tz, tx, ty);
                b.contains(lon, lat)
            });
        }
        if tiles.len() > opt.max_tiles {
            tiles.truncate(opt.max_tiles);
            truncated = true;
        }
        for (tz, tx, ty, id) in tiles {
            let Some(tile) = scan.decode(tz, tx, ty, id) else { continue };
            visited += 1;
            let b = crate::project::Bounds::of_tile(tz, tx, ty);
            bbox = Some(match bbox {
                None => (b.west, b.south, b.east, b.north),
                Some((w, s, e, n)) => {
                    (w.min(b.west), s.min(b.south), e.max(b.east), n.max(b.north))
                }
            });
            for c in checks.iter_mut() {
                c.visit(&tile, opt);
            }
        }
    }

    let mut metrics: Vec<Metric> = Vec::new();
    for c in checks {
        metrics.extend(c.finish());
    }
    metrics.extend(lod::measure(scan, &zooms, opt));

    if truncated {
        for m in metrics.iter_mut() {
            m.detail.push_str(&format!(
                " (sampled {visited} tiles; the zoom holds more than the {} cap)",
                opt.max_tiles
            ));
        }
    }

    let scope = crate::verify::Scope {
        tiles: visited,
        bbox,
        spacing_m: opt.spacing_m,
        max_tiles: opt.max_tiles,
        at: opt.at,
        truncated,
        commit: git_commit(),
    };
    Scorecard { archive: String::new(), zooms, scope, metrics }
}

/// The tree this run measured, best-effort. A baseline that cannot name its
/// commit cannot be re-cut, and re-cutting is the only way to tell a stale
/// baseline from a real regression.
fn git_commit() -> Option<String> {
    let out = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}
