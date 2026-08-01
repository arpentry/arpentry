//! The canonical situations (docs/GENERATION.md §4), bound to real places.
//!
//! §4 already says what the test set is: "A generator is adequate when it
//! handles all of these; each stresses a different part of the problem. They
//! are the test scenarios for any design." They have been prose since they were
//! written, which means every iteration has been looking at whatever piece of
//! Switzerland was on screen at the time. Impressions gathered that way do not
//! accumulate — fixing the mountain tunnel and breaking the river bridge is
//! indistinguishable from progress, because nobody flies back to the river.
//!
//! A site is *mined*, not invented. [`mine`] finds the strongest example of
//! each detectable situation in an archive — the highest viaduct, the deepest
//! bore, the tile holding both a deck and a portal — and prints them for
//! pasting into the corpus file. Coordinates chosen any other way are a guess
//! about someone else's terrain.

use std::collections::HashMap;
use std::path::Path;

use serde_json::{json, Value as Json};

use super::scene::ArchiveScan;

/// A situation from §4, and where in the world it is exercised.
pub struct Scenario {
    pub id: &'static str,
    pub name: &'static str,
    /// What it stresses, in §4's own words.
    pub stresses: &'static str,
    /// Whether an archive scan can find an instance on its own.
    pub minable: bool,
}

/// A place that exercises a scenario.
#[derive(Clone, Debug)]
pub struct Site {
    pub lon: f64,
    pub lat: f64,
    pub zoom: u8,
    /// How this site was chosen — a mined superlative, or a human note.
    pub source: String,
}

/// The fourteen situations, verbatim from docs/GENERATION.md §4.
pub fn catalogue() -> Vec<Scenario> {
    vec![
        Scenario { id: "S1", name: "Valley viaduct", stresses: "Profile reconstruction, piers, multi-segment structure entities", minable: true },
        Scenario { id: "S2", name: "Saddle bridge", stresses: "S1's degenerate case; deck approximately level", minable: true },
        Scenario { id: "S3", name: "River bridge on flat ground", stresses: "Feature clearance over water; approach ramps rising from flat ground", minable: false },
        Scenario { id: "S4", name: "Overpass / interchange on flat ground", stresses: "Crossing detection, network constraints, embankments", minable: true },
        Scenario { id: "S5", name: "Mountain tunnel", stresses: "Annotation mistrust, portal placement, terrain holes", minable: true },
        Scenario { id: "S6", name: "Urban underpass / cut-and-cover", stresses: "The flat-ground tunnel case terrain alone cannot express", minable: true },
        Scenario { id: "S7", name: "Bridge directly into tunnel", stresses: "Structure-to-structure continuity", minable: true },
        Scenario { id: "S8", name: "Dual carriageway on one structure", stresses: "Entity resolution across parallel segments", minable: false },
        Scenario { id: "S9", name: "At-grade mountain road", stresses: "Knowing when to do nothing; grade limits must not fix a road that genuinely climbs", minable: true },
        Scenario { id: "S10", name: "Annotation noise", stresses: "Robustness; graceful degradation; solved structure ends", minable: false },
        Scenario { id: "S11", name: "Building on a steep slope", stresses: "Building-ground reconciliation, per-LOD terrain agreement", minable: false },
        Scenario { id: "S12", name: "Dense old town with courtyards", stresses: "Roof synthesis, courtyard meshing, LOD aggregation", minable: false },
        Scenario { id: "S13", name: "Building beside a road cut or embankment", stresses: "Cross-class ground agreement", minable: false },
        Scenario { id: "S14", name: "Lakefront", stresses: "Water surfaces, shoreline continuity", minable: false },
    ]
}

/// Reads a corpus file. Sites are a JSON *array* rather than an object keyed by
/// id, so the file reads in scenario order — an object would come back sorted
/// lexically, putting S10 between S1 and S2 in a file people have to edit.
///
/// A missing or unreadable file yields an empty corpus rather than an error:
/// the scorecard is useful without one, and demanding it would make the check
/// harder to adopt than the thing it replaces.
pub fn load(path: &Path) -> HashMap<String, Site> {
    let Ok(text) = std::fs::read_to_string(path) else { return HashMap::new() };
    let Ok(json) = serde_json::from_str::<Json>(&text) else { return HashMap::new() };
    let Some(list) = json.get("sites").and_then(Json::as_array) else { return HashMap::new() };
    list.iter()
        .filter_map(|v| {
            Some((
                v.get("id")?.as_str()?.to_string(),
                Site {
                    lon: v.get("lon")?.as_f64()?,
                    lat: v.get("lat")?.as_f64()?,
                    zoom: v.get("zoom")?.as_u64()? as u8,
                    source: v.get("source").and_then(Json::as_str).unwrap_or("").to_string(),
                },
            ))
        })
        .collect()
}

/// Serializes a corpus back out, so a mined set can be committed as-is.
pub fn to_json(sites: &HashMap<String, Site>) -> Json {
    let mut keys: Vec<&String> = sites.keys().collect();
    keys.sort_by_key(|k| k.trim_start_matches('S').parse::<u32>().unwrap_or(u32::MAX));
    let list: Vec<Json> = keys
        .iter()
        .map(|k| {
            let s = &sites[*k];
            json!({ "id": k, "lon": s.lon, "lat": s.lat, "zoom": s.zoom, "source": s.source })
        })
        .collect();
    json!({
        "comment": "Sites exercising docs/GENERATION.md §4, one per canonical situation. \
                    Mined with `arpentry_verify <archive> --mine`; edit freely, but prefer a \
                    mined superlative over a guessed coordinate.",
        "sites": list,
    })
}

/// Finds the strongest instance of each detectable situation in an archive.
///
/// Each is a superlative rather than a threshold, so it always returns *the*
/// worst case the data holds rather than nothing at all when the data is mild.
pub fn mine(scan: &ArchiveScan<'_>, zoom: u8, max_tiles: usize) -> HashMap<String, Site> {
    let mut best: HashMap<String, (f64, Site)> = HashMap::new();
    let mut keep = |id: &str, score: f64, site: Site| {
        if score.is_finite() && best.get(id).is_none_or(|(s, _)| score > *s) {
            best.insert(id.to_string(), (score, site));
        }
    };

    let mut tiles = scan.tiles_at(zoom);
    tiles.truncate(max_tiles);
    for (z, x, y, id) in tiles {
        let Some(tile) = scan.decode(z, x, y, id) else { continue };
        let centre = tile.lonlat(0.5, 0.5);
        let site = |what: String| Site { lon: centre.0, lat: centre.1, zoom, source: what };

        let decks: Vec<_> = tile.roads.iter().filter(|r| r.is_deck()).collect();
        let bores: Vec<_> = tile.roads.iter().filter(|r| r.is_bore()).collect();
        let paved: Vec<_> = tile.roads.iter().filter(|r| r.is_pavement()).collect();

        // S7: a deck and a bore in the same tile is a portal at an abutment,
        // or close enough to be worth looking at.
        if !decks.is_empty() && !bores.is_empty() {
            keep("S7", decks.len() as f64 + bores.len() as f64, site("deck and bore in one tile".into()));
        }

        // S1/S2: the deck standing furthest above the ground it flies over —
        // and, for S2, the one that clears it by least while still flying.
        if let Some(terrain) = &tile.terrain {
            for d in &decks {
                let mut air = f64::NEG_INFINITY;
                let mut lowest = f64::INFINITY;
                d.mesh.sample(&tile.scale, 8.0, |px, py, _| {
                    if !tile.owns(px, py) {
                        return;
                    }
                    let (Some((soffit, _)), Some(g)) =
                        (d.mesh.height_range_at(px, py), terrain.height_at(px, py))
                    else {
                        return;
                    };
                    air = air.max(soffit - g);
                    lowest = lowest.min(soffit - g);
                });
                if air.is_finite() {
                    keep("S1", air, site(format!("{} deck standing {air:.0} m above the ground", d.class)));
                    // A saddle bridge is a short flat one: reward little air.
                    if air > 1.0 {
                        keep("S2", 1.0 / air, site(format!("{} deck clearing by only {air:.0} m", d.class)));
                    }
                }
            }

            // S5: the bore with the deepest cover over it.
            for b in &bores {
                let mut cover = f64::NEG_INFINITY;
                b.mesh.sample(&tile.scale, 8.0, |px, py, _| {
                    if !tile.owns(px, py) {
                        return;
                    }
                    let (Some((_, roof)), Some(g)) =
                        (b.mesh.height_range_at(px, py), terrain.height_at(px, py))
                    else {
                        return;
                    };
                    cover = cover.max(g - roof);
                });
                if cover.is_finite() {
                    keep("S5", cover, site(format!("{} bore under {cover:.0} m of hill", b.class)));
                }
            }

            // S9: the steepest at-grade carriageway that is still a plausible
            // road — the case where the right answer is to do nothing.
            for p in &paved {
                if let Some((s, (px, py))) = p.mesh.max_slope(&tile.scale) {
                    if s < 0.30 {
                        let (lon, lat) = tile.lonlat(px, py);
                        keep(
                            "S9",
                            s,
                            Site { lon, lat, zoom, source: format!("at-grade road at {:.0} % grade", s * 100.0) },
                        );
                    }
                }
            }
        }

        // S4: a deck over an at-grade carriageway. Score by how many distinct
        // levels stack, so an interchange beats a lone overpass.
        if !decks.is_empty() && !paved.is_empty() {
            let mut levels: Vec<i64> = decks.iter().map(|d| d.level).collect();
            levels.sort_unstable();
            levels.dedup();
            keep("S4", levels.len() as f64 * 100.0 + decks.len() as f64,
                 site(format!("{} deck(s) over at-grade road, levels {levels:?}", decks.len())));
        }

        // S6: a road below grade with something at grade above it.
        if !bores.is_empty() && !paved.is_empty() {
            keep("S6", bores.len() as f64, site(format!("{} bore(s) under an at-grade road", bores.len())));
        }
    }

    best.into_iter().map(|(k, (_, s))| (k, s)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_catalogue_is_the_fourteen_from_the_design_doc() {
        let c = catalogue();
        assert_eq!(c.len(), 14);
        assert_eq!(c[0].id, "S1");
        assert_eq!(c[13].id, "S14");
        // Every id unique, so a corpus file can key on them.
        let mut ids: Vec<&str> = c.iter().map(|s| s.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), 14);
    }

    #[test]
    fn a_corpus_round_trips_through_json() {
        let mut sites = HashMap::new();
        sites.insert(
            "S1".to_string(),
            Site { lon: 6.9290, lat: 46.4200, zoom: 16, source: "mined".into() },
        );
        sites.insert(
            "S5".to_string(),
            Site { lon: 7.1, lat: 46.2, zoom: 16, source: "mined".into() },
        );
        let json = to_json(&sites);
        let dir = std::env::temp_dir().join("arpentry-corpus-test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("scenarios.json");
        std::fs::write(&path, json.to_string()).unwrap();
        let back = load(&path);
        assert_eq!(back.len(), 2);
        assert!((back["S1"].lon - 6.9290).abs() < 1e-9);
        assert_eq!(back["S5"].zoom, 16);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn a_missing_corpus_is_empty_rather_than_fatal() {
        assert!(load(Path::new("/nonexistent/scenarios.json")).is_empty());
    }

    #[test]
    fn sites_serialize_in_scenario_order_not_lexical_order() {
        // S2 must precede S10; sorting the strings would not.
        let mut sites = HashMap::new();
        for id in ["S10", "S2", "S1"] {
            sites.insert(id.to_string(), Site { lon: 0.0, lat: 0.0, zoom: 16, source: String::new() });
        }
        let text = to_json(&sites).to_string();
        let at = |k: &str| text.find(&format!("\"{k}\"")).unwrap();
        assert!(at("S1") < at("S2") && at("S2") < at("S10"), "{text}");
    }
}
