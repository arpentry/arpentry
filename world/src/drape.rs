//! The drape: road centrelines laid exactly onto the terrain mesh.
//!
//! "Exactly" means the drawn line lies *on* the drawn surface, not near it: a
//! chord between two points of one plane lies in that plane, so a polyline
//! whose every piece stays inside one triangle of the mesh, and whose
//! vertices take their height from that triangle, is on the mesh to the ulp.
//! Splitting at grid lines alone is not enough — a piece that crosses a
//! cell's diagonal spans two planes and cuts through the ridge or valley the
//! diagonal makes — so the split runs at the diagonals too.
//!
//! What this step does *not* do is engineer anything: the line follows every
//! wrinkle of the raw ground. That is the point of this step. The steps after it
//! will have to earn every metre they move the ground by.

use std::path::Path;

use arpentry_server::geoparquet::ReadError;

use crate::frame::Extent;
use crate::net;
use crate::roads;
use crate::step::Summary;
use crate::lattice::drape;
use crate::world::{Polyline3, Roads, Terrain};

/// Reads the ways of `segments` — a parquet, or a [`net`] spec — and drapes
/// them onto the world's terrain.
pub fn run(extent: &Extent, terrain: &Terrain, segments: &Path) -> Result<(Roads, Summary), String> {
    let read = match segments.to_str().filter(|s| net::is_spec(s)) {
        Some(spec) => synthetic(spec, &extent.rect)?,
        None => roads::read(segments, &extent.bbox, &extent.frame, &extent.rect)
            .map_err(|e: ReadError| e.to_string())?,
    };
    let mut roads = Roads::default();
    let (mut pieces, mut vertices) = (0usize, 0usize);
    // The *whole* way is draped, spans and all: the drawn centreline is the
    // way the source drew, and the cut into ground and structure pieces is
    // the partition step's.
    for way in &read.ways {
        let pts = drape(terrain, &way.pts);
        pieces += pts.len().saturating_sub(1);
        vertices += pts.len();
        roads.lines.push(Polyline3 {
            id: way.id.clone(),
            class: way.class.clone(),
            subclass: way.subclass.clone(),
            width_m: way.width_m,
            pts,
        });
    }
    let summary = Summary::new()
        .with("features", read.features)
        .with("ways", read.kept)
        .with("structures", format!("{} ({} off the ground entirely)", read.structures, read.aloft))
        .with("measured", read.measured)
        .with("oneway", read.oneway)
        .with("rail", format!("{} ({} street rail not kept)", read.rail, read.street_rail))
        .with("layered", read.layered)
        .with("clipped", read.ways.len())
        .with("draped", roads.lines.len())
        .with("pieces", pieces)
        .with("vertices", vertices);
    roads.ways = read.ways;
    Ok((roads, summary))
}

/// The ways of a synthetic network, clipped to the rect like a read one —
/// whole, with their span tables re-based on each clipped run.
fn synthetic(spec: &str, rect: &crate::frame::Rect) -> Result<roads::Read, String> {
    let mut out = roads::Read::default();
    for way in net::parse(spec)? {
        out.features += 1;
        out.kept += 1;
        out.rail += (crate::width::family(&way.class) == crate::width::Family::Rail) as usize;
        if way.has_structure() {
            out.structures += 1;
        }
        out.ways.extend(roads::clip_way(&way, rect));
    }
    Ok(out)
}
