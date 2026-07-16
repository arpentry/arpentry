//! Stage 1 — assemble the global scene model (docs/GENERATION.md §6).
//!
//! Reads the transportation input once, keeps the segments whose vertical
//! geometry needs solving (structure annotations, or a class that holds an
//! engineered grade), and joins them into [`Corridor`]s with corridor-wide
//! structure [`Span`]s. Everything else tiles as plain draped geometry and
//! never enters the scene graph.
//!
//! The output is the [`SceneGraph`]: a plain, inspectable artifact the solve
//! stage fits profiles over, and the tiling phase resolves features against
//! by source id.

pub mod columns;
pub mod corridors;
pub mod crossings;
pub mod grid;
pub mod road_junctions;
pub mod water;

use std::path::Path;

use geo_types::Geometry;

use crate::geoparquet::{GeoParquet, ReadError};
use crate::priors::{self, RoadClass};
use crate::project::Bounds;
use crate::scene::{source_hash, SceneGraph};
use crate::value::Value;

use corridors::RawSegment;

/// A connector within this fraction of an end is that end's connector.
const END_AT_EPS: f64 = 1e-3;

/// Attribute columns the assemble stage reads. The scalar ones become the
/// styling properties re-emitted with every piece of the segment (matching
/// what the tiling phase reads for transportation); `level_rules`,
/// `road_flags` (the `is_bridge`/`is_tunnel` fallback where no level rule is
/// mapped — see `crate::levels`), and `connectors` are consumed here.
const ATTRS: &[&str] = &[
    "id",
    "type",
    "subtype",
    "class",
    "subclass",
    "names.primary",
    "level_rules",
    "road_flags",
    "connectors",
    "width_rules",
    "road_surface",
    "access_restrictions",
    "cartography.min_zoom",
    "cartography.max_zoom",
    "cartography.sort_key",
];

/// Reads the transportation input (and the water input, when present) and
/// assembles the scene graph.
pub fn run(path: &Path, water: Option<&Path>, bbox: &Bounds) -> Result<SceneGraph, ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups =
        gp.row_groups_intersecting((bbox.west, bbox.south, bbox.east, bbox.north));
    let mut raw: Vec<RawSegment> = Vec::new();
    for feature in gp.features(row_groups, ATTRS)? {
        let f = feature?;
        let class_key = prop_string(&f.properties, "class").unwrap_or_default();
        let class = RoadClass::parse(Some(class_key.as_str()));
        if f.level_runs.is_empty() && class.grade_limit().is_none() {
            continue; // nothing to solve: plain draped road
        }
        // Only a linestring can be linearly referenced and chained.
        let Geometry::LineString(ref line) = f.geometry else {
            continue;
        };
        if line.0.len() < 2 {
            continue;
        }
        let Some(source) = prop_string(&f.properties, "id").map(|s| source_hash(&s)) else {
            continue; // no stable id: the tiling phase could never look it up
        };
        let start_connector =
            f.connectors.iter().find(|c| c.at <= END_AT_EPS).map(|c| c.id);
        let end_connector =
            f.connectors.iter().find(|c| c.at >= 1.0 - END_AT_EPS).map(|c| c.id);
        raw.push(RawSegment {
            source,
            line: line.0.clone(),
            class,
            link: priors::is_link(prop_string(&f.properties, "subclass").as_deref()),
            class_key,
            subtype_key: prop_string(&f.properties, "subtype").unwrap_or_default(),
            level_runs: f.level_runs,
            start_connector,
            end_connector,
            connector_ids: f.connectors.iter().map(|c| c.id).collect(),
            properties: f.properties,
        });
    }
    let (corridors, junctions) = corridors::build(raw);
    let mut scene = SceneGraph::new(corridors);
    scene.junctions = junctions;
    // Second pass: find where the corridors' structure spans cross the rest
    // of the network (the input is streamed again; only geometry near a span
    // is actually tested). Water gets its own pass: a bridge over a river
    // owes freeboard, not road clearance (S3).
    let bb = (bbox.west, bbox.south, bbox.east, bbox.north);
    // At-grade road junctions over every drivable road, minus the connectors
    // the corridor junctions already own, so each junction is plated once.
    // The same pass collects the unclaimed streets' bed lines for the ground
    // stage to bench (D3).
    let corridor_connectors: std::collections::HashSet<u64> =
        scene.junctions.iter().map(|j| j.connector).collect();
    let (road_junctions, beds) =
        road_junctions::build(path, bb, &corridor_connectors, &|h| scene.lookup(h).is_some())?;
    scene.road_junctions = road_junctions;
    scene.beds = beds;
    let (crossings, underpasses) = crossings::detect(path, bb, &scene)?;
    scene.crossings = crossings;
    scene.underpasses = underpasses;
    if let Some(water_path) = water {
        let mut water_crossings = crossings::detect_water(water_path, bb, &scene)?;
        scene.crossings.append(&mut water_crossings);
        // Still water bodies for the ground stage to flatten (invariant 4) —
        // read whatever the network does, since a lake is flattened even where
        // no bridge crosses it.
        scene.water = water::read(water_path, bb)?;
    }
    Ok(scene)
}

fn prop_string(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}
