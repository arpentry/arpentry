//! Reading still water bodies from the water input (docs/GENERATION.md
//! invariant 4).
//!
//! A lake must come out flat, but the data gives no surface elevation. What it
//! gives is the shoreline polygon; the ground stage reads a level from the DEM
//! along that ring and burns the interior flat. This module only assembles the
//! geometry — which polygons are still water, and their rings — leaving the
//! level to the stage that has the DEM.

use std::path::Path;

use geo_types::{Coord, Geometry, Polygon};

use crate::geoparquet::{GeoParquet, ReadError};
use crate::priors::MIN_WATER_BODY_M;
use crate::scene::{WaterBody, DEG_M};
use crate::value::Value;

/// Overture water `subtype`s whose surface is still, so flattening one level is
/// right. Flowing water (`stream`, `river`, `canal`) wants a monotone descent
/// and is left to the DEM for now; marine features (`physical`) and tiny
/// `human_made` basins are skipped.
fn is_still(subtype: Option<&str>) -> bool {
    matches!(subtype, Some("lake" | "pond" | "reservoir" | "water"))
}

/// Reads the still water bodies intersecting `bbox` from the water input.
pub fn read(path: &Path, bbox: (f64, f64, f64, f64)) -> Result<Vec<WaterBody>, ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting(bbox);
    let mut out = Vec::new();
    for feature in gp.features(row_groups, &["subtype", "class"])? {
        let f = feature?;
        if !is_still(prop_str(&f.properties, "subtype").as_deref()) {
            continue;
        }
        collect(&f.geometry, &mut out);
    }
    Ok(out)
}

/// Pushes a [`WaterBody`] per polygon big enough to matter (a multipolygon
/// yields several); ponds finer than the DEM resolves stay draped.
fn collect(g: &Geometry, out: &mut Vec<WaterBody>) {
    let mut push = |p: &Polygon| {
        let b = body(p);
        if significant(b.bbox) {
            out.push(b);
        }
    };
    match g {
        Geometry::Polygon(p) => push(p),
        Geometry::MultiPolygon(mp) => mp.0.iter().for_each(&mut push),
        _ => {} // a stray line/point water feature has no surface to flatten
    }
}

/// Whether a body's bounding box is at least [`MIN_WATER_BODY_M`] across.
fn significant(bbox: (f64, f64, f64, f64)) -> bool {
    let cos_lat = (0.5 * (bbox.1 + bbox.3)).to_radians().cos();
    let w = (bbox.2 - bbox.0) * DEG_M * cos_lat;
    let h = (bbox.3 - bbox.1) * DEG_M;
    w.max(h) >= MIN_WATER_BODY_M
}

fn body(p: &Polygon) -> WaterBody {
    let exterior = p.exterior().0.clone();
    let holes = p.interiors().iter().map(|r| r.0.clone()).collect();
    let bbox = bounds(&exterior);
    WaterBody { exterior, holes, bbox }
}

/// Bounding box `(west, south, east, north)` of a ring.
fn bounds(ring: &[Coord]) -> (f64, f64, f64, f64) {
    ring.iter().fold(
        (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        |b, c| (b.0.min(c.x), b.1.min(c.y), b.2.max(c.x), b.3.max(c.y)),
    )
}

fn prop_str(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}
