//! Stage-artifact dumps (`--dump <dir>`): each pipeline stage's output as
//! plain GeoJSON, inspectable in QGIS or kepler.gl without running the stages
//! after it (docs/GENERATION.md §6, stage-boundary testability).

use std::fs;
use std::io;
use std::path::Path;

use serde_json::{json, Value as Json};

use crate::ground::GroundModel;
use crate::scene::SceneGraph;
use crate::solve::SolvedModel;

/// Writes `corridors.geojson` + `crossings.geojson` (stage 1),
/// `profiles.geojson` (stage 2), and `earthworks.geojson` (stage 3).
pub fn write(
    dir: &Path,
    scene: &SceneGraph,
    solved: &SolvedModel,
    ground: &GroundModel,
) -> io::Result<()> {
    fs::create_dir_all(dir)?;
    fs::write(dir.join("corridors.geojson"), corridors_geojson(scene).to_string())?;
    fs::write(dir.join("crossings.geojson"), crossings_geojson(scene).to_string())?;
    fs::write(dir.join("underpasses.geojson"), underpasses_geojson(scene).to_string())?;
    fs::write(dir.join("junctions.geojson"), junctions_geojson(scene, solved).to_string())?;
    fs::write(dir.join("profiles.geojson"), profiles_geojson(scene, solved).to_string())?;
    fs::write(dir.join("smooth.geojson"), smooth_geojson(scene, solved).to_string())?;
    fs::write(dir.join("earthworks.geojson"), earthworks_geojson(ground).to_string())?;
    Ok(())
}

/// The detected underpasses: one Point each, with the level ordering.
fn underpasses_geojson(scene: &SceneGraph) -> Json {
    let features: Vec<Json> = scene
        .underpasses
        .iter()
        .map(|u| {
            json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": [u.point.x, u.point.y] },
                "properties": {
                    "corridor": u.corridor,
                    "under_level": u.under_level,
                    "over": u.over,
                    "over_level": u.over_level,
                },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The corridor junctions: one Point each, with every member's corridor, arc,
/// and *welded* road height — the residual disagreement between members is the
/// continuity defect the junction weld left behind.
fn junctions_geojson(scene: &SceneGraph, solved: &SolvedModel) -> Json {
    let features: Vec<Json> = scene
        .junctions
        .iter()
        .map(|j| {
            // The per-member road heights and their spread — the residual
            // disagreement is the continuity defect (docs/CONSISTENCY.md P0).
            let mut heights: Vec<f64> = Vec::new();
            let members: Vec<Json> = j
                .members
                .iter()
                .map(|m| {
                    let road = match solved.profile(m.corridor) {
                        Some(p) => {
                            let h = p.road_at_arc(m.arc);
                            heights.push(h);
                            json!(h)
                        }
                        None => Json::Null,
                    };
                    json!({ "corridor": m.corridor, "arc": m.arc, "road_m": road })
                })
                .collect();
            let step_m = if heights.len() >= 2 {
                let lo = heights.iter().cloned().fold(f64::INFINITY, f64::min);
                let hi = heights.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                json!(hi - lo)
            } else {
                Json::Null
            };
            json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": [j.point.x, j.point.y] },
                "properties": { "connector": j.connector, "step_m": step_m, "members": members },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The detected crossings: one Point each, with the level ordering.
fn crossings_geojson(scene: &SceneGraph) -> Json {
    let features: Vec<Json> = scene
        .crossings
        .iter()
        .map(|c| {
            json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": [c.point.x, c.point.y] },
                "properties": {
                    "upper": c.upper,
                    "upper_level": c.upper_level,
                    "lower": c.lower,
                    "lower_level": c.lower_level,
                    "lower_kind": format!("{:?}", c.lower_kind),
                },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The engineered ground's earthworks: one short LineString per edge with its
/// target height and reach.
fn earthworks_geojson(ground: &GroundModel) -> Json {
    let features: Vec<Json> = ground
        .earthworks()
        .edges()
        .iter()
        .map(|e| {
            json!({
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[e.a.x, e.a.y, e.target_a], [e.b.x, e.b.y, e.target_b]],
                },
                "properties": {
                    "half_width_m": e.half_width_m,
                    "feather_m": e.feather_m,
                },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The scene graph: one LineString per corridor, its spans summarized as a
/// `spans` property (`kind arc0..arc1` per line).
fn corridors_geojson(scene: &SceneGraph) -> Json {
    let features: Vec<Json> = scene
        .corridors
        .iter()
        .map(|c| {
            let coords: Vec<Json> = c.nodes.iter().map(|p| json!([p.x, p.y])).collect();
            let spans: Vec<String> = c
                .spans
                .iter()
                .map(|s| format!("{:?} {:.0}..{:.0}", s.kind, s.arc0, s.arc1))
                .collect();
            json!({
                "type": "Feature",
                "geometry": { "type": "LineString", "coordinates": coords },
                "properties": {
                    "corridor": c.id,
                    "class": format!("{:?}", c.class),
                    "length_m": c.total().round(),
                    "segments": c.segments.len(),
                    "spans": spans.join("; "),
                },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The solved sweep lines: one LineString per profiled corridor holding the
/// *smoothed* centerline at full resolution, for measuring sweep smoothness.
fn smooth_geojson(scene: &SceneGraph, solved: &SolvedModel) -> Json {
    let features: Vec<Json> = scene
        .corridors
        .iter()
        .filter_map(|c| solved.profile(c.id).map(|p| (c, p)))
        .map(|(c, p)| {
            let coords: Vec<Json> = p.smooth().iter().map(|n| json!([n.x, n.y])).collect();
            json!({
                "type": "Feature",
                "geometry": { "type": "LineString", "coordinates": coords },
                "properties": { "corridor": c.id, "class": format!("{:?}", c.class) },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}

/// The solved model: one LineString per profiled corridor with per-vertex
/// solved road and terrain heights (decimated to keep the file browsable).
fn profiles_geojson(scene: &SceneGraph, solved: &SolvedModel) -> Json {
    let features: Vec<Json> = scene
        .corridors
        .iter()
        .filter_map(|c| solved.profile(c.id).map(|p| (c, p)))
        .map(|(c, p)| {
            // Every ~4th node (~32 m) keeps files browsable at full fidelity
            // where it matters (heights vary smoothly between).
            let step = 4.max(1);
            let nodes = p.nodes();
            let road = p.road_m();
            let terrain = p.terrain_m();
            let idx: Vec<usize> =
                (0..nodes.len()).step_by(step).chain(std::iter::once(nodes.len() - 1)).collect();
            let coords: Vec<Json> =
                idx.iter().map(|&i| json!([nodes[i].x, nodes[i].y, road[i]])).collect();
            let road_j: Vec<Json> = idx.iter().map(|&i| json!(road[i])).collect();
            let terrain_j: Vec<Json> = idx.iter().map(|&i| json!(terrain[i])).collect();
            let deck = p.deck_m();
            let deck_j: Vec<Json> = idx.iter().map(|&i| json!(deck[i])).collect();
            let at_grade = p.at_grade();
            let grade_j: Vec<Json> = idx.iter().map(|&i| json!(at_grade[i])).collect();
            json!({
                "type": "Feature",
                "geometry": { "type": "LineString", "coordinates": coords },
                "properties": {
                    "corridor": c.id,
                    "class": format!("{:?}", c.class),
                    "road_m": road_j,
                    "terrain_m": terrain_j,
                    "deck_m": deck_j,
                    "at_grade": grade_j,
                },
            })
        })
        .collect();
    json!({ "type": "FeatureCollection", "features": features })
}
