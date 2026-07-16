//! At-grade road junctions from all drivable roads (Plan C, scenario S4).
//!
//! The corridor junctions ([`corridors::junctions`]) cover only the graded and
//! structure network the solver models. The intersections that dominate a town
//! are ordinary streets, which never become corridors — so this pass reads the
//! transportation input once more for *every* drivable road and finds the
//! connectors where three or more of their ends meet. Connectors already
//! handled as corridor junctions are excluded, so a junction is plated once.
//!
//! Overture splits roads at their connectors, so a junction shows up as several
//! segment *ends* sharing a connector; each end contributes a leg (its heading
//! away from the connector and its class half-width). The synth stage drapes
//! the plate on the engineered ground.

use std::collections::HashMap;
use std::collections::HashSet;
use std::path::Path;

use geo_types::{Coord, Geometry};

use crate::geoparquet::{GeoParquet, ReadError};
use crate::priors;
use crate::scene::{BedLine, RoadJunction};
use crate::value::Value;

/// A connector within this fraction of an end is that end's connector (matching
/// the corridor assembler's `END_AT_EPS`).
const END_AT_EPS: f64 = 1e-3;

/// Reads the at-grade road junctions intersecting `bbox` — connectors where
/// three or more drivable road ends meet, skipping any in `exclude` (the
/// corridor-junction connectors) — and, from the same pass, the street
/// [`BedLine`]s: every drivable road the corridors did *not* claim
/// (`claimed`), whose bed the ground stage benches flat across (D3).
pub fn build(
    path: &Path,
    bbox: (f64, f64, f64, f64),
    exclude: &HashSet<u64>,
    claimed: &dyn Fn(u64) -> bool,
) -> Result<(Vec<RoadJunction>, Vec<BedLine>), ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting(bbox);
    // connector id → each end there: (point, heading east, heading north,
    // half-width, class).
    let mut ends: HashMap<u64, Vec<(Coord, f64, f64, f64, String)>> = HashMap::new();
    let mut beds: Vec<BedLine> = Vec::new();
    for feature in
        gp.features(row_groups, &["id", "class", "subclass", "connectors", "width_rules"])?
    {
        let f = feature?;
        let class = prop_str(&f.properties, "class");
        let subclass = prop_str(&f.properties, "subclass");
        // Drivable only (the paint-width set); a path or rail owes no plate.
        // The leg spans the road's surface band edge — the P1-derived painted
        // width (mapped where plausible, class prior otherwise) plus the
        // structure shoulder — so a trimmed band meets the mouth flush.
        let measured = prop_f64(&f.properties, "width_rules");
        let Some(paint_w) =
            priors::carriageway_width_m(class.as_deref(), subclass.as_deref(), measured)
        else {
            continue;
        };
        let half_w = paint_w * 0.5 + priors::STRUCTURE_SHOULDER_M;
        let class_str = class.unwrap_or_default();
        let Geometry::LineString(ref line) = f.geometry else {
            continue;
        };
        let pts = &line.0;
        if pts.len() < 2 {
            continue;
        }
        // An unclaimed street's bed: corridors carry their own solved
        // earthworks, so only the streets the solver never sees bench here.
        let unclaimed = prop_str(&f.properties, "id")
            .is_none_or(|id| !claimed(crate::scene::source_hash(&id)));
        if unclaimed {
            beds.push(BedLine {
                pts: pts.clone(),
                half_width_m: half_w,
                class: priors::RoadClass::parse(Some(class_str.as_str())),
            });
        }
        let mut add = |conn: Option<u64>, at: Coord, toward: Coord| {
            if let Some(c) = conn {
                if !exclude.contains(&c) {
                    let (e, n) = heading(at, toward);
                    ends.entry(c).or_default().push((at, e, n, half_w, class_str.clone()));
                }
            }
        };
        let start = f.connectors.iter().find(|c| c.at <= END_AT_EPS).map(|c| c.id);
        let end = f.connectors.iter().find(|c| c.at >= 1.0 - END_AT_EPS).map(|c| c.id);
        add(start, pts[0], pts[1]);
        add(end, pts[pts.len() - 1], pts[pts.len() - 2]);
    }

    // Deterministic order: drain the map connector-sorted, so the junction order
    // (and thus the owning tile's emit) never depends on hashing.
    let mut conns: Vec<(u64, Vec<(Coord, f64, f64, f64, String)>)> = ends.into_iter().collect();
    conns.sort_by_key(|(id, _)| *id);
    let mut out = Vec::new();
    for (_conn, legs) in conns {
        if legs.len() < 3 {
            continue; // a through-node or a dead end, not an intersection
        }
        let point = legs[0].0;
        // The widest leg's class styles the plate (the dominant road there).
        let class = legs
            .iter()
            .max_by(|a, b| a.3.partial_cmp(&b.3).expect("finite width"))
            .map(|l| l.4.clone())
            .unwrap_or_default();
        let legs = legs.into_iter().map(|(_, e, n, w, _)| (e, n, w)).collect();
        out.push(RoadJunction { point, class, legs });
    }
    Ok((out, beds))
}

/// Unit ENU heading from `at` toward `toward` (the direction into the road).
fn heading(at: Coord, toward: Coord) -> (f64, f64) {
    let cos_lat = at.y.to_radians().cos();
    let (de, dn) = ((toward.x - at.x) * cos_lat, toward.y - at.y);
    let len = (de * de + dn * dn).sqrt();
    if len < 1e-12 {
        (1.0, 0.0)
    } else {
        (de / len, dn / len)
    }
}

fn prop_str(props: &[(String, Value)], key: &str) -> Option<String> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::String(s) => Some(s.clone()),
        _ => None,
    })
}

fn prop_f64(props: &[(String, Value)], key: &str) -> Option<f64> {
    props.iter().find(|(k, _)| k == key).and_then(|(_, v)| match v {
        Value::Double(d) => Some(*d),
        _ => None,
    })
}
