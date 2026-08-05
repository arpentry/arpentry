//! Invariant 5 across the LOD ladder: any two zoom levels derive identical
//! heights for shared geometry.
//!
//! Scoped, deliberately, to structures. docs/GROUND.md §4 is explicit that
//! at-grade road height is *supposed* to differ by zoom — below the reference
//! rung the coarse lattice cannot carry a bench, so the road reads that zoom's
//! rendered surface plus a clamped datum lift, which is the coarse-LOD rule and
//! not a workaround. Measuring at-grade equality across zooms would therefore
//! report a large, permanent, entirely correct "violation", and a scorecard
//! with a column nobody can ever drive to zero is a scorecard people learn to
//! skim.
//!
//! What the same section does promise without qualification: "On structures the
//! road rides the deck ramp at every zoom, the same heights the deck and bore
//! solids are swept from." So a deck at z15 and the same deck at z16 must agree
//! to the millimetre, and any drift is D5's forbidden case — a deck changing
//! height between LODs — which on screen is a bridge that jumps as you zoom.
//!
//! ## Only where the match is certain
//!
//! Structures carry no stable identity across zooms — the tile encodes a class
//! and a level ordinal, and nothing else. Where a coarse tile holds several
//! decks of the same class at the same level, "the same structure" is a guess,
//! and the first version of this check made it: it reported 2.06 m of drift on
//! a motorway deck whose parent tile held *nine* candidate meshes, which is not
//! evidence of drift, it is evidence of comparing two different bridges.
//!
//! So a sample counts only when the coarse tile holds exactly one candidate.
//! That gives up coverage — and the count of samples skipped as ambiguous is
//! reported, because a check that quietly measured a tenth of what it claimed
//! would be worse than no check at all.

use std::collections::HashMap;

use crate::verify::dist::Dist;
use crate::verify::scene::{ArchiveScan, TileScene};
use crate::verify::{Invariant, Metric, Offender, Sense, Worst};

use super::Options;

/// Heights are int32 millimetres; two zooms that agree in the model agree here
/// to the millimetre, so anything past half a centimetre is drift.
const DRIFT_M: f64 = 0.005;

/// How many coarse tiles to hold decoded at once. Tiles arrive in Hilbert
/// order, so their parents come in runs and a small cache hits almost always.
const CACHE: usize = 48;

/// Compares structure heights at `z` against the same structures at `z - 1`.
pub fn measure(scan: &ArchiveScan<'_>, zooms: &[u8], opt: &Options) -> Vec<Metric> {
    let mut dist = Dist::new(0.0, 32.0);
    let mut worst = Worst::new(Sense::HigherIsWorse, opt.worst_k);
    let mut compared = 0u64;
    let mut ambiguous = 0u64;
    let mut note: Option<String> = None;

    let fine = zooms.iter().copied().max().unwrap_or(0);
    if fine == 0 || fine <= scan.min_zoom() {
        return vec![metric(dist, worst, Some("no coarser zoom in the archive to compare against"))];
    }
    let coarse = fine - 1;

    let mut tiles = scan.tiles_at(fine);
    if let Some((lon, lat)) = opt.at {
        tiles.retain(|&(z, x, y, _)| crate::project::Bounds::of_tile(z, x, y).contains(lon, lat));
    }
    if tiles.len() > opt.max_tiles {
        tiles.truncate(opt.max_tiles);
    }

    // Index the coarse rung once; looking a parent up per tile would otherwise
    // rescan the whole directory.
    let coarse_index: HashMap<(u32, u32), u64> =
        scan.tiles_at(coarse).into_iter().map(|(_, x, y, id)| ((x, y), id)).collect();

    let mut cache: HashMap<(u32, u32), Option<TileScene>> = HashMap::new();
    for (z, x, y, id) in tiles {
        let Some(tile) = scan.decode(z, x, y, id) else { continue };
        if !tile.roads.iter().any(|r| r.level != 0) {
            continue;
        }
        let key = (x / 2, y / 2);
        if cache.len() > CACHE && !cache.contains_key(&key) {
            cache.clear();
        }
        let parent = cache
            .entry(key)
            .or_insert_with(|| {
                coarse_index.get(&key).and_then(|&pid| scan.decode(coarse, key.0, key.1, pid))
            });
        let Some(parent) = parent.as_ref() else { continue };

        for s in tile.roads.iter().filter(|r| r.level != 0) {
            // The same structure at the coarser rung, identified by class and
            // level — the only identity the format carries. Several candidates
            // means no identification at all.
            let candidates: Vec<_> = parent
                .roads
                .iter()
                .filter(|r| r.level == s.level && r.class == s.class)
                .collect();
            if candidates.len() != 1 {
                ambiguous += candidates.len() as u64;
                continue;
            }
            let coarse_mesh = &candidates[0].mesh;
            s.mesh.sample(&tile.scale, opt.spacing_m.max(4.0), |px, py, _| {
                if !tile.owns(px, py) {
                    return;
                }
                let Some((_, top)) = s.mesh.height_range_at(px, py) else { return };
                let (lon, lat) = tile.lonlat(px, py);
                let cx = (lon - parent.bounds.west) / parent.bounds.width();
                let cy = (lat - parent.bounds.south) / parent.bounds.height();
                let Some((_, coarse_top)) = coarse_mesh.height_range_at(cx, cy) else { return };
                compared += 1;
                let d = (top - coarse_top).abs();
                dist.push(d);
                if d > DRIFT_M {
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: fine,
                        value: d,
                        note: format!(
                            "{} L{}: z{fine} reads {top:.3} m, z{coarse} reads {coarse_top:.3} m",
                            s.class, s.level
                        ),
                    });
                }
            });
        }
    }

    if compared == 0 {
        note = Some(format!(
            "no structure could be matched one-to-one between z{fine} and z{coarse} \
             ({ambiguous} candidate meshes were ambiguous)"
        ));
    }
    let mut m = metric(dist, worst, note.as_deref());
    if ambiguous > 0 && compared > 0 {
        m.detail.push_str(&format!(
            " Coverage: {compared} samples matched one-to-one; {ambiguous} coarse meshes were \
             skipped as ambiguous (several of the same class and level in one tile)."
        ));
    }
    vec![m]
}

fn metric(dist: Dist, worst: Worst, skipped: Option<&str>) -> Metric {
    Metric {
        id: "lod.structure_drift".into(),
        invariant: Invariant::I5,
        title: "Structure height drift between adjacent zooms".into(),
        population: "Structure (level != 0) surface samples at the finest measured zoom, against \
                     the same class and level in the parent tile one rung coarser — and only \
                     where the parent holds exactly one candidate. Structures carry no identity \
                     across zooms, so an ambiguous parent would compare two different bridges; \
                     those are skipped and counted in the detail line."
            .into(),
        detail: "Absolute difference between a deck or bore surface at one zoom and the same \
                 structure one rung coarser. Scoped to structures on purpose: at-grade road \
                 height is zoom-dependent by design (the datum lift, docs/GROUND.md §4), but a \
                 deck must ride the same ramp at every zoom, and drift here is a bridge that \
                 jumps as the camera zooms."
            .into(),
        sense: Sense::HigherIsWorse,
        threshold: DRIFT_M,
        skipped: skipped.map(str::to_string),
        dist,
        worst: worst.into_vec(),
    }
}
