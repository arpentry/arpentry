//! Invariant 5 across the LOD ladder: any two zoom levels derive the same
//! *relation* between shared geometry and the ground drawn under it.
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
//! Structures ride the same rule since the per-zoom datum (`synth::datum`): at
//! a coarse zoom a deck is the solved ramp re-expressed against that zoom's
//! drawn ground, so its *absolute* top legitimately differs between rungs by
//! exactly the two canvases' divergence. What §4 promises instead is the
//! relation: a structure's height over its own zoom's drawn ground is the
//! reference relation at every rung. So the drift compared here is
//! `(top_fine − ground_fine) − (top_coarse − ground_coarse)` — a deck that
//! keeps its clearance while the canvas refines reads zero, and a deck that
//! jumps *against its own ground* as the camera zooms is what the check
//! catches. Samples with no drawn ground on either side (the kerb hole at the
//! detail rung) are absent, as in `clearance.deck_over_ground`.
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

/// The drift gate on the canvas-relative comparison. The residue of a correct
/// datum shift is not zero: the shift is built from the ground *field*
/// evaluated on each zoom's lattice, while this check reads each zoom's
/// *drawn* terrain — which at the detail rung is a breakline-constrained mesh
/// that legitimately departs the lattice interpolation by the size of the
/// relief a lattice cell cannot hold. Measured on Montreux with the shift
/// landed, that residue tails at 1.6 m for the z14/z13 pair, 3.5 m for
/// z15/z14, and 5.9 m for z16/z15 — the last dominated by gorges the
/// constrained mesh draws and the lattice field cannot. Two metres sits
/// above the coarse pairs' tracking residue while still reading the
/// detail-pair refinement honestly; as everywhere on this scorecard, the
/// verdict is the baseline diff, not the absolute rate.
const DRIFT_M: f64 = 2.0;

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
                let Some(g_fine) =
                    tile.terrain.as_ref().and_then(|t| t.height_at(px, py))
                else {
                    return;
                };
                let (lon, lat) = tile.lonlat(px, py);
                let cx = (lon - parent.bounds.west) / parent.bounds.width();
                let cy = (lat - parent.bounds.south) / parent.bounds.height();
                let Some((_, coarse_top)) = coarse_mesh.height_range_at(cx, cy) else { return };
                let Some(g_coarse) =
                    parent.terrain.as_ref().and_then(|t| t.height_at(cx, cy))
                else {
                    return;
                };
                compared += 1;
                let d = ((top - g_fine) - (coarse_top - g_coarse)).abs();
                dist.push(d);
                if d > DRIFT_M {
                    worst.offer(Offender {
                        lon,
                        lat,
                        zoom: fine,
                        value: d,
                        note: format!(
                            "{} L{}: z{fine} clears its ground by {:.2} m, z{coarse} by {:.2} m",
                            s.class,
                            s.level,
                            top - g_fine,
                            coarse_top - g_coarse
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
        detail: "Difference in a structure's clearance over its own zoom's drawn ground, between \
                 one zoom and the same structure one rung coarser. Absolute tops legitimately \
                 differ between rungs by the canvases' divergence (the per-zoom datum, \
                 docs/GROUND.md §4); what must not change is the relation to the ground drawn \
                 under it, and drift here is a bridge that jumps against its own hillside as \
                 the camera zooms."
            .into(),
        sense: Sense::HigherIsWorse,
        threshold: DRIFT_M,
        skipped: skipped.map(str::to_string),
        dist,
        worst: worst.into_vec(),
    }
}
