//! Stage 1 — assemble the global scene model (docs/GENERATION.md §5).
//!
//! Reads the transportation input once, keeps the segments whose vertical
//! geometry needs solving (every drivable road, plus anything carrying
//! structure annotations), and joins them into [`Corridor`]s with
//! corridor-wide structure [`Span`]s. Everything else tiles as plain draped
//! geometry and never enters the scene graph.
//!
//! The output is the [`SceneGraph`]: a plain, inspectable artifact the solve
//! stage fits profiles over, and the tiling phase resolves features against
//! by source id.

pub mod columns;
pub mod corridors;
pub mod grid;
pub mod water;

use std::path::Path;

use geo_types::Geometry;

use crate::geoparquet::{GeoParquet, ReadError};
use crate::priors::{self, Kind};
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
        let subclass = prop_string(&f.properties, "subclass");
        let subtype_key = prop_string(&f.properties, "subtype").unwrap_or_default();
        let kind = Kind::parse(
            Some(subtype_key.as_str()),
            Some(class_key.as_str()),
            subclass.as_deref(),
        );
        // **The stratum decides.** A feature enters the scene graph when it
        // belongs to a stratum that solves — and a draped feature never does,
        // whatever it is annotated with: carrying a structure span is not a
        // promotion (§4.2). That discipline is the point. Draped features are
        // 46.9 % of the road network, and any loophole admitting one into a
        // solve is a loophole through which half the network can perturb the
        // other half. Their structures are *fitted* to the finished ground
        // instead (`synth::draped`).
        //
        // Rail is the one stratum this cannot yet express. It is senior (R)
        // and belongs in the scene unconditionally, but admitting it before it
        // is solved as rail leaves its viaducts chorded across a descent — see
        // [`priors::paves_today`] for the measurement. So rail keeps today's
        // annotation-driven admission until M6 gives it a real alignment.
        if !priors::paves_today(kind)
            && !(matches!(kind, Kind::Rail(_)) && !f.level_runs.is_empty())
        {
            continue; // draped: it samples the finished ground, it never solves
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
            kind,
            link: priors::is_link(subclass.as_deref()),
            class_key,
            subtype_key,
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
    // No crossings. They are a consequence of the solved heights, so they are
    // derived at the solve and handed straight to the graph (`solve::crossings`,
    // §4.5) — the second pass over the input that used to find them here could
    // only ever see the annotated half, and what it found went stale as soon as
    // anything downstream touched a span.
    if let Some(water_path) = water {
        // Still water bodies for the ground stage to flatten (I4) — read
        // whatever the network does, since a lake is flattened even where no
        // bridge crosses it.
        let bb = (bbox.west, bbox.south, bbox.east, bbox.north);
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
