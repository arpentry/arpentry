//! Road centrelines from the transportation source, in the local frame,
//! clipped to the bbox.
//!
//! This is a reader, not a model: no joining into corridors, no levels, no
//! widths. Those are later steps, each of which will be built against what
//! this one returns.

use std::path::Path;

use arpentry_server::geoparquet::{GeoParquet, ReadError};
use arpentry_server::project::Bounds;
use arpentry_server::value::{str_of, Props};
use geo_types::{Geometry, LineString};

use crate::frame::{Frame, Rect};
use crate::world::Polyline2;

/// The columns read. `subtype` and `class` decide admission; `id` is carried
/// so a line in the output can be traced to its source feature.
pub const COLUMNS: &[&str] = &["id", "class", "subtype"];

/// The drivable road classes this iteration keeps (the server's
/// `priors::RoadClass` list, minus paths, tracks and rail).
const CLASSES: &[&str] = &[
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "service",
];

/// What [`read`] found.
#[derive(Debug, Default)]
pub struct Read {
    pub lines: Vec<Polyline2>,
    /// Features decoded from the row groups touching the bbox.
    pub features: usize,
    /// Of those, the drivable roads.
    pub kept: usize,
}

/// Whether a feature's properties name a drivable road.
pub fn keep(props: &Props) -> bool {
    let subtype_ok = matches!(str_of(props, "subtype"), None | Some("road"));
    subtype_ok && str_of(props, "class").is_some_and(|c| CLASSES.contains(&c))
}

/// Reads the drivable roads of `path` touching `bbox`, projected into `frame`
/// and clipped to `rect`.
pub fn read(path: &Path, bbox: &Bounds, frame: &Frame, rect: &Rect) -> Result<Read, ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting((bbox.west, bbox.south, bbox.east, bbox.north));
    let mut out = Read::default();
    for feature in gp.features(row_groups, COLUMNS)? {
        let f = feature?;
        out.features += 1;
        if !keep(&f.properties) {
            continue;
        }
        out.kept += 1;
        let id = str_of(&f.properties, "id").unwrap_or_default().to_string();
        let class = str_of(&f.properties, "class").unwrap_or_default().to_string();
        for line in lines_of(&f.geometry) {
            let pts: Vec<[f64; 2]> = line.0.iter().map(|c| frame.to_local(c.x, c.y)).collect();
            for run in clip(&pts, rect) {
                out.lines.push(Polyline2 { id: id.clone(), class: class.clone(), pts: run });
            }
        }
    }
    Ok(out)
}

/// The line strings of a geometry; anything else is not a centreline.
pub fn lines_of(g: &Geometry) -> Vec<&LineString> {
    match g {
        Geometry::LineString(l) => vec![l],
        Geometry::MultiLineString(m) => m.0.iter().collect(),
        _ => Vec::new(),
    }
}

/// Clips a polyline to `rect`, returning the runs that remain inside. Each
/// segment is clipped with Liang–Barsky; consecutive segments whose clipped
/// parts meet are joined into one run.
pub fn clip(pts: &[[f64; 2]], rect: &Rect) -> Vec<Vec<[f64; 2]>> {
    let mut runs: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut run: Vec<[f64; 2]> = Vec::new();
    let mut flush = |run: &mut Vec<[f64; 2]>| {
        if run.len() >= 2 {
            runs.push(std::mem::take(run));
        } else {
            run.clear();
        }
    };
    for pair in pts.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let Some((t0, t1)) = liang_barsky(p, q, rect) else {
            flush(&mut run);
            continue;
        };
        let a = lerp(p, q, t0);
        let b = lerp(p, q, t1);
        if run.is_empty() {
            run.push(a);
        } else if t0 > 0.0 {
            flush(&mut run);
            run.push(a);
        }
        if a != b {
            run.push(b);
        }
        if t1 < 1.0 {
            flush(&mut run);
        }
    }
    flush(&mut run);
    runs
}

/// The parameter range `[t0, t1] ⊆ [0, 1]` of `p→q` inside `rect`, if any.
fn liang_barsky(p: [f64; 2], q: [f64; 2], rect: &Rect) -> Option<(f64, f64)> {
    let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (num, den) in [
        (p[0] - rect.x0, -dx),
        (rect.x1 - p[0], dx),
        (p[1] - rect.y0, -dy),
        (rect.y1 - p[1], dy),
    ] {
        if den == 0.0 {
            if num < 0.0 {
                return None;
            }
            continue;
        }
        let t = num / den;
        if den < 0.0 {
            t0 = t0.max(t);
        } else {
            t1 = t1.min(t);
        }
    }
    (t0 <= t1).then_some((t0, t1))
}

fn lerp(p: [f64; 2], q: [f64; 2], t: f64) -> [f64; 2] {
    [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]
}

#[cfg(test)]
mod tests {
    use arpentry_server::value::Value;

    use super::*;

    fn props(pairs: &[(&str, &str)]) -> Vec<(String, Value)> {
        pairs.iter().map(|(k, v)| (k.to_string(), Value::String(v.to_string()))).collect()
    }

    #[test]
    fn keeps_drivable_roads_only() {
        assert!(keep(&props(&[("class", "residential"), ("subtype", "road")])));
        assert!(keep(&props(&[("class", "motorway")])));
        assert!(!keep(&props(&[("class", "footway"), ("subtype", "road")])));
        assert!(!keep(&props(&[("class", "rail"), ("subtype", "rail")])));
        assert!(!keep(&props(&[("subtype", "road")])));
        assert!(!keep(&props(&[("class", "primary"), ("subtype", "water")])));
    }

    #[test]
    fn lines_of_flattens_multilines() {
        let l = LineString::from(vec![(0.0, 0.0), (1.0, 1.0)]);
        assert_eq!(lines_of(&Geometry::LineString(l.clone())).len(), 1);
        let m = geo_types::MultiLineString(vec![l.clone(), l]);
        assert_eq!(lines_of(&Geometry::MultiLineString(m)).len(), 2);
        assert!(lines_of(&Geometry::Point((0.0, 0.0).into())).is_empty());
    }

    fn unit() -> Rect {
        Rect { x0: 0.0, y0: 0.0, x1: 10.0, y1: 10.0 }
    }

    #[test]
    fn inside_stays_one_run() {
        let pts = [[1.0, 1.0], [5.0, 2.0], [9.0, 9.0]];
        assert_eq!(clip(&pts, &unit()), vec![pts.to_vec()]);
    }

    #[test]
    fn crossing_is_cut_at_the_edge() {
        let runs = clip(&[[-5.0, 5.0], [5.0, 5.0], [15.0, 5.0]], &unit());
        assert_eq!(runs, vec![vec![[0.0, 5.0], [5.0, 5.0], [10.0, 5.0]]]);
    }

    #[test]
    fn leaving_and_returning_makes_two_runs() {
        let runs = clip(&[[2.0, 2.0], [2.0, 15.0], [8.0, 15.0], [8.0, 2.0]], &unit());
        assert_eq!(runs, vec![vec![[2.0, 2.0], [2.0, 10.0]], vec![[8.0, 10.0], [8.0, 2.0]]]);
    }

    #[test]
    fn outside_yields_nothing() {
        assert!(clip(&[[-5.0, -5.0], [-1.0, 20.0]], &unit()).is_empty());
        assert!(clip(&[[20.0, 0.0], [30.0, 0.0]], &unit()).is_empty());
    }

    #[test]
    fn a_corner_touch_is_not_a_run() {
        // Passes through the corner (10, 10) exactly: one point, no length.
        assert!(clip(&[[5.0, 15.0], [15.0, 5.0]], &unit()).is_empty());
    }
}
