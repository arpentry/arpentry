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
pub mod facades;
pub mod grid;
pub mod walks;
pub mod water;

use std::path::Path;

use geo_types::{Coord, Geometry};

use crate::geoparquet::{GeoParquet, ReadError};
use crate::priors::{self, Kind, Stratum, MIN_STRUCTURE_M};
use crate::project::Bounds;
use crate::scene::{source_hash, Corridor, SceneGraph, SpanKind};

use corridors::RawSegment;
use grid::GridIndex;

/// A connector within this fraction of an end is that end's connector.
pub(crate) const END_AT_EPS: f64 = 1e-3;

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
    let mut witnesses: Vec<Vec<Coord>> = Vec::new();
    let mut pedestrians: Vec<walks::WalkLine> = Vec::new();
    for feature in gp.features(row_groups, ATTRS)? {
        let f = feature?;
        let class_key = crate::value::str_of(&f.properties, "class").unwrap_or_default();
        let subclass = crate::value::str_of(&f.properties, "subclass");
        let subtype_key = crate::value::str_of(&f.properties, "subtype").unwrap_or_default();
        let kind = Kind::parse(Some(subtype_key), Some(class_key), subclass);
        // **The stratum decides.** A feature enters the scene graph when it
        // belongs to a stratum that solves — and a draped feature never does,
        // whatever it is annotated with: carrying a structure span is not a
        // promotion (§4.2). That discipline is the point. Draped features are
        // 46.9 % of the road network, and any loophole admitting one into a
        // solve is a loophole through which half the network can perturb the
        // other half. Their structures are *fitted* to the finished ground
        // instead (`synth::draped`).
        //
        // **The stratum decides, and now it means it.** Rail is senior and
        // belongs in the scene unconditionally — not because it was tagged
        // with a bridge, which is the promotion §4.2 forbids, but because a
        // railway's alignment exists independently of the street network.
        if !matches!(kind.stratum(), Stratum::H | Stratum::R | Stratum::S) {
            // Draped: it samples the finished ground, it never solves. Its
            // *plan* line is still evidence — a short annotated bridge is a
            // bridge because of what passes beneath it, and half the time
            // that is a footpath the scene would otherwise never see.
            if let Geometry::LineString(ref line) = f.geometry {
                if line.0.len() >= 2 {
                    // …and a pedestrian one's plan line is evidence of a
                    // second kind: which street it belongs to. That relation
                    // is resolved once the corridors exist (`walks::attach`);
                    // it is still not a promotion — nothing here solves.
                    if priors::earns_walk_band(kind) {
                        if let Some(source) =
                            crate::value::str_of(&f.properties, "id").map(|s| source_hash(&s))
                        {
                            pedestrians.push(walks::WalkLine {
                                source,
                                line: line.0.clone(),
                                kind,
                                tagged: subclass == Some("sidewalk"),
                                crosswalk: subclass == Some("crosswalk"),
                                connectors: f.connectors.clone(),
                                spans: f
                                    .level_runs
                                    .iter()
                                    .filter(|r| r.level != 0)
                                    .map(|r| (r.start, r.end))
                                    .collect(),
                            });
                        }
                    }
                    witnesses.push(line.0.clone());
                }
            }
            continue;
        }
        // Only a linestring can be linearly referenced and chained.
        let Geometry::LineString(ref line) = f.geometry else {
            continue;
        };
        if line.0.len() < 2 {
            continue;
        }
        let Some(source) = crate::value::str_of(&f.properties, "id").map(|s| source_hash(&s)) else {
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
            link: priors::is_link(subclass),
            class_key: class_key.to_string(),
            subtype_key: subtype_key.to_string(),
            level_runs: f.level_runs,
            start_connector,
            end_connector,
            connectors: f.connectors,
            properties: f.properties,
        });
    }
    let (corridors, junctions) = corridors::build(raw);
    let mut scene = SceneGraph::new(corridors);
    scene.junctions = junctions;
    // Which street each pedestrian way belongs to. Resolved here because it is
    // a plan-space relation between the lines just read and the corridors just
    // chained, and because it must be resolved *once*: a band whose host was
    // decided per tile would move at a tile boundary (invariant 5).
    scene.walks = walks::attach(&scene.corridors, pedestrians);
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
        // Flowing water joins the draped alignments as a crossing witness:
        // a mapped stream under a short annotated bridge is the gully the
        // DEM cannot resolve.
        witnesses.append(&mut water::flowing_lines(water_path, bb)?);
    }
    scene.witnesses = near_short_spans(&scene.corridors, witnesses);
    Ok(scene)
}

/// Keeps only the witness lines that could decide a short structure span's
/// terrain fate. The crossing test is asked strictly inside sub-
/// [`MIN_STRUCTURE_M`] structure spans (`solve::crossings::
/// spans_over_a_mapped_line`), so a line whose every edge misses every such
/// span's stretch of its corridor can never testify — and draped alignments
/// are 46.9 % of the network, dead weight the scene would otherwise carry to
/// the end of the run.
fn near_short_spans(corridors: &[Corridor], witnesses: Vec<Vec<Coord>>) -> Vec<Vec<Coord>> {
    let mut grid = GridIndex::new();
    let mut n = 0u32;
    for c in corridors {
        for s in c.spans.iter().filter(|s| s.kind != SpanKind::Grade) {
            if s.arc1 - s.arc0 >= MIN_STRUCTURE_M {
                continue;
            }
            for i in 0..c.nodes.len().saturating_sub(1) {
                if c.arc[i + 1] < s.arc0 || c.arc[i] > s.arc1 {
                    continue;
                }
                let (a, b) = (c.nodes[i], c.nodes[i + 1]);
                grid.insert(
                    (a.x.min(b.x), a.y.min(b.y), a.x.max(b.x), a.y.max(b.y)),
                    n,
                );
                n += 1;
            }
        }
    }
    if grid.is_empty() {
        return Vec::new();
    }
    let mut hits: Vec<u32> = Vec::new();
    witnesses
        .into_iter()
        .filter(|line| {
            line.windows(2).any(|e| {
                grid.query(
                    (
                        e[0].x.min(e[1].x),
                        e[0].y.min(e[1].y),
                        e[0].x.max(e[1].x),
                        e[0].y.max(e[1].y),
                    ),
                    &mut hits,
                );
                !hits.is_empty()
            })
        })
        .collect()
}

