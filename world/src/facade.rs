//! The facades: the building footprints, and what they refuse.
//!
//! A street is a room between buildings. The width a way is given is a
//! prior, and in a town the prior runs into a wall: a 5.5 m residential
//! mapped down a 4 m lane, a footway's band drawn through the house it
//! runs past, the kerb ladder filling the strip between a footway and its
//! street straight through the row of houses between them. Nothing paved
//! stands inside a building, so the footprints are read here, once, and
//! every surface step after this one subtracts them — the surface union,
//! the pavement the kerb step fills, the junctions the legs build. The
//! `kerb_gap` check reads them too: bare ground outside a kerb and a wall
//! outside it are different answers.
//!
//! **The passage.** The building wins wherever it and a way's width merely
//! overlap. Where a way's *axis* runs inside a footprint the data says the
//! way goes through — an arcade, a porch, a garage, a footprint drawn over
//! the lane — and cutting the way there would split the network in two,
//! trading a measured defect for an unmeasured one (the server's
//! `MIN_CARRIAGEWAY_HALF_M` was that lesson). Along that stretch of axis
//! the building yields a corridor of the way's own width, at most
//! [`PASSAGE_M`]. What it refuses is `solid = footprints − passages`.
//!
//! **The pockets.** A facade is not a straight line: it has notches a
//! metre deep, and the next house stands two metres along. A kerb that
//! followed every one of them would zigzag, and no kerb does — the asphalt
//! runs along the *closed* facade, the walls with every notch and gap
//! narrower than `2·POCKET_M` filled ([`POCKET_M`]), and what the closing
//! added is a pocket the asphalt keeps out of but the pavement (the room
//! step) may fill. A way whose axis runs through a closed gap — a footway
//! down an alley between two houses, a lane between facades closer than
//! the closing — must not be cut by a pocket, so every way's corridor is
//! taken out of the pockets, not only the corridors of ways found inside
//! them: a pocket is the closing's addition and never a wall, so opening it
//! along a way that does not need it costs nothing, where sampling the axis
//! for the ways that do missed a lens thinner than a sample step and cut a
//! road at it. So there are two masks: `solid`, the walls less the
//! passages, which nothing paved enters; and `built`, the solid with the
//! pockets less every corridor, which the asphalt does not.
//!
//! Footprints are clipped to the world's rect like every other input,
//! oriented like every other region (outer counter-clockwise, holes
//! clockwise — a source polygon may come either way, and two overlapping
//! rings of opposite winding cancel under the non-zero rule) and unioned:
//! two houses sharing a wall are one solid, and a courtyard is a hole.
//! No class filter — a shed is a wall, and the asphalt has no business in
//! one either.
//!
//! ```text
//! house:beside?d=2[&x=0&l=10&w=10&side=1&notch=0&deep=1&rot=0]   an l×w house with its facade `d` m from the x axis, centred on x=`x`, north (`side=1`) or south; with `notch`, a notch that wide and `deep` m deep in the middle of the facade; turned `rot` degrees about its centre
//! house:across[?x=0&l=10&w=12&rot=0]        a house straddling the x axis, turned `rot` degrees about its centre: the way passes through it
//! house:row?d=2[&l=10&w=10&gap=2]           two houses side by side along the axis, `gap` m apart, facades `d` m off it
//! house:pair?gap=3[&l=10&w=10]              two houses facing each other across the axis, their facades `gap` m apart
//! ```
//!
//! Every spec also takes `h=` (the houses' height; absent, they are stood at
//! the guess a source without one gets), `roof=` (Overture's `roof_shape`)
//! and `rise=` (its `roof_height`): what the building step reads.
//!
//! **One building at a time is kept too.** The masks are a union and forget
//! which footprint was which; the building step needs each one's own height
//! and roof, so the reader hands the buildings on as it read them, and
//! `guessed` counts those whose height the source never gave.

use std::path::Path;

use arpentry_server::geoparquet::{GeoParquet, ReadError};
use arpentry_server::project::Bounds;
use arpentry_server::value::{f64_of, str_of, Value};
use geo_types::{Geometry, Polygon};

use crate::frame::{Extent, Frame, Rect};
use crate::net::Params;
use crate::poly::{self, Indexed, Pt, Shape, Shapes};
use crate::step::Summary;
use crate::world::{Building, Facade, Polyline2, Network, Roof, RoofShape};

/// The widest corridor a building yields to a way running through it, in
/// metres: a lane's worth. A way narrower than this keeps its own width.
pub const PASSAGE_M: f64 = 4.0;

/// How often along its axis a way is asked whether it is inside a
/// footprint, in metres.
const SAMPLE_M: f64 = 1.0;

/// The closing radius for the built mask, in metres: a notch or a gap
/// between houses narrower than twice this is a pocket the asphalt keeps
/// out of. Three metres is under the narrowest lane a car is driven down,
/// and over the widest notch a facade is drawn with.
pub const POCKET_M: f64 = 1.5;

/// Whether `s` is a spec rather than a path.
pub fn is_spec(s: &str) -> bool {
    s.starts_with("house:")
}

/// Reads the buildings and decides what they refuse. `None` is no
/// building input at all: open ground everywhere.
pub fn run(
    extent: &Extent,
    roads: &Network,
    buildings: Option<&Path>,
) -> Result<(Facade, Summary), String> {
    let read = match buildings {
        None => Read::default(),
        Some(p) => match p.to_str().filter(|s| is_spec(s)) {
            Some(spec) => synthetic(spec, &extent.rect)?,
            None => {
                read(p, &extent.bbox, &extent.frame, &extent.rect).map_err(|e| e.to_string())?
            }
        },
    };
    let footprints = poly::union_all(&read.buildings.iter().flat_map(|b| b.footprint.iter().cloned()).collect());
    // A railway asks nothing of a building: it runs under a station roof
    // rather than through a passage the building yields, and its ballast
    // does not stop at a wall. Given a corridor, a train shed's footprint
    // shrank by the tracks' width and the pavement was let into the hall.
    let plan: Vec<crate::world::Polyline2> = roads
        .plan
        .iter()
        .filter(|w| crate::width::family(&w.class) != crate::width::Family::Rail)
        .cloned()
        .collect();
    let (passage_corridors, passages, passage_m) = corridors(&plan, &footprints);
    // The solid is the footprints less the corridors themselves, whose
    // edges cross the walls at an angle. Subtracting the passages (the
    // corridors already cut to the footprints) instead left a hairline of
    // solid along every wall a corridor crossed — the cut's vertices on
    // the wall are lattice-rounded, so its edge and the wall's no longer
    // coincide — and that hairline split a road at every wall it passed.
    let passage_corridors = poly::union_all(&passage_corridors);
    let solid = poly::difference(&footprints, &passage_corridors);
    let passages_shapes = poly::intersect(&passage_corridors, &footprints);
    // The built mask: the closed footprints less every way's corridor,
    // then the solid put back — a corridor opens the pocket a way runs
    // down and never the walls beside it. The union tolerates the
    // closing's lattice-rounded edges lying a hair off the walls', where
    // a difference would have left a sliver. `lanes` counts the ways the
    // pockets would otherwise have cut: an observation, not the mask.
    let closed = if footprints.is_empty() {
        Vec::new()
    } else {
        poly::erode(&poly::dilate(&footprints, POCKET_M), POCKET_M)
    };
    let (_, lanes_all, _) = corridors(&plan, &closed);
    let lanes = lanes_all.saturating_sub(passages);
    let open = poly::difference(&closed, &poly::union_all(&all_corridors(&plan)));
    let built = poly::union_of(&[&open, &solid]);
    let summary = Summary::new()
        .with("footprints", read.buildings.len())
        .with("clipped", read.clipped)
        .with("guessed", read.guessed)
        .with("underground", read.underground)
        .with_m2("footprint_m2", poly::area(&footprints))
        .with("passages", passages)
        .with_m2("passage_m", passage_m)
        .with("lanes", lanes)
        .with_m2("solid_m2", poly::area(&solid))
        .with_m2("pocket_m2", poly::area(&built) - poly::area(&solid));
    Ok((Facade { buildings: read.buildings, footprints, passages: passages_shapes, solid, built }, summary))
}

/// What the reader (or the dial) found.
#[derive(Debug, Default)]
pub struct Read {
    /// The buildings with any part inside the rect, their footprints
    /// clipped to it and oriented, not yet unioned.
    pub buildings: Vec<Building>,
    /// Pieces the rect cut.
    pub clipped: usize,
    /// Buildings the source gave no height, stood at [`DEFAULT_HEIGHT_M`].
    pub guessed: usize,
    /// Buildings the source flags `is_underground`, dropped.
    pub underground: usize,
}

impl Read {
    /// Adds one building, the `shapes` of a feature (several for a
    /// multipolygon) clipped to `rect`, if any part of it is inside;
    /// counts every piece the rect cut.
    fn add(&mut self, shapes: impl IntoIterator<Item = Shape>, rect: &Rect, height_m: Option<f64>, roof: Roof) {
        let mut footprint = Shapes::new();
        for shape in shapes {
            let (clipped, cut) = clip(shape, rect);
            self.clipped += cut as usize;
            footprint.extend(clipped);
        }
        if footprint.is_empty() {
            return;
        }
        self.guessed += height_m.is_none() as usize;
        self.buildings.push(Building { footprint, height_m: height_m.unwrap_or(DEFAULT_HEIGHT_M), roof });
    }
}

/// Reads the footprints of `path` touching `bbox`, projected into `frame`
/// and clipped to `rect`.
pub fn read(path: &Path, bbox: &Bounds, frame: &Frame, rect: &Rect) -> Result<Read, ReadError> {
    let gp = GeoParquet::open(path)?;
    let row_groups = gp.row_groups_intersecting((bbox.west, bbox.south, bbox.east, bbox.north));
    let mut out = Read::default();
    let attrs = ["height", "num_floors", "roof_shape", "roof_height", "is_underground"];
    for feature in gp.features(row_groups, &attrs)? {
        let f = feature?;
        let polygons: Vec<&Polygon> = match &f.geometry {
            Geometry::Polygon(p) => vec![p],
            Geometry::MultiPolygon(m) => m.0.iter().collect(),
            _ => continue,
        };
        let props = &f.properties;
        // A building the source puts under the ground is no facade: nothing
        // paved stops at it and nothing stands up for it. The Veytaux
        // power station's caverns are mapped as footprints on the flank
        // above them, and stood on the highest ground there they were a
        // 5 m box a hundred metres in the air on its low side.
        if props.iter().any(|(k, v)| k == "is_underground" && matches!(v, Value::Bool(true))) {
            out.underground += 1;
            continue;
        }
        let height = mapped_height(f64_of(props, "height"), f64_of(props, "num_floors"));
        let roof = Roof::mapped(str_of(props, "roof_shape"), f64_of(props, "roof_height"));
        out.add(polygons.into_iter().map(|p| local(p, frame)), rect, height, roof);
    }
    Ok(out)
}

/// A source polygon in the local frame, rings open and oriented: the
/// outer counter-clockwise, every hole clockwise. Rings under three points
/// are dropped.
fn local(p: &Polygon, frame: &Frame) -> Shape {
    let ring = |line: &geo_types::LineString, ccw: bool| -> Option<Vec<Pt>> {
        let mut pts: Vec<Pt> = line.0.iter().map(|c| frame.to_local(c.x, c.y)).collect();
        if pts.len() > 1 && pts.first() == pts.last() {
            pts.pop();
        }
        (pts.len() >= 3).then(|| poly::oriented(pts, ccw))
    };
    let mut shape: Shape = Vec::new();
    let Some(outer) = ring(p.exterior(), true) else {
        return shape;
    };
    shape.push(outer);
    shape.extend(p.interiors().iter().filter_map(|h| ring(h, false)));
    shape
}

/// `shape` clipped to `rect`: nothing if its box misses the rect, itself
/// if the box lies inside, the intersection otherwise — and whether it was
/// cut.
fn clip(shape: Shape, rect: &Rect) -> (Shapes, bool) {
    let Some([x0, y0, x1, y1]) = shape.first().and_then(|outer| poly::bounds(outer.iter().copied())) else {
        return (Vec::new(), false);
    };
    if x1 < rect.x0 || x0 > rect.x1 || y1 < rect.y0 || y0 > rect.y1 {
        return (Vec::new(), false);
    }
    if x0 >= rect.x0 && x1 <= rect.x1 && y0 >= rect.y0 && y1 <= rect.y1 {
        return (vec![shape], false);
    }
    let window = vec![poly::rect(rect.x0, rect.y0, rect.x1, rect.y1)];
    (poly::intersect(&vec![shape], &window), true)
}

/// The corridors the buildings yield: for every stretch of a way's axis
/// inside a footprint, the axis from the sample before the stretch to the
/// sample after it, buffered to the way's width capped at [`PASSAGE_M`].
/// Also the number of stretches and their length in metres.
fn corridors(ways: &[Polyline2], footprints: &Shapes) -> (Shapes, usize, f64) {
    let index = Indexed::new(footprints);
    let (mut out, mut n, mut metres) = (Shapes::new(), 0usize, 0.0f64);
    if index.is_empty() {
        return (out, n, metres);
    }
    for w in ways {
        let pts = crate::poly::resample(&w.pts, SAMPLE_M);
        if pts.len() < 2 {
            continue;
        }
        let inside: Vec<bool> = pts.iter().map(|&p| index.contains(p)).collect();
        let mut i = 0;
        while i < pts.len() {
            if !inside[i] {
                i += 1;
                continue;
            }
            let j = (i..pts.len()).take_while(|&k| inside[k]).last().unwrap_or(i);
            let (a, b) = (i.saturating_sub(1), (j + 1).min(pts.len() - 1));
            out.extend(poly::buffer_line(&pts[a..=b], w.width_m.min(PASSAGE_M)));
            n += 1;
            metres += poly::length(&pts[i..=j]);
            i = j + 1;
        }
    }
    (out, n, metres)
}

/// Every way's corridor, whole: its axis buffered to its width capped at
/// [`PASSAGE_M`].
fn all_corridors(ways: &[Polyline2]) -> Shapes {
    ways.iter().flat_map(|w| poly::buffer_line(&w.pts, w.width_m.min(PASSAGE_M))).collect()
}

/// `shape` turned `deg` degrees about `(cx, cy)`.
fn turned(mut shape: Shape, cx: f64, cy: f64, deg: f64) -> Shape {
    let (sin, cos) = deg.to_radians().sin_cos();
    for p in shape.iter_mut().flatten() {
        let (dx, dy) = (p[0] - cx, p[1] - cy);
        *p = [cx + dx * cos - dy * sin, cy + dx * sin + dy * cos];
    }
    shape
}

/// What a `house:` spec draws: its footprints, in local metres, unclipped,
/// and what every house of it stands to.
#[derive(Debug)]
pub struct Houses {
    pub shapes: Shapes,
    /// `None` where the spec gives no `h`: the guess, as for a source.
    pub height_m: Option<f64>,
    pub roof: Roof,
}

/// The houses of a `house:` spec.
pub fn parse(spec: &str) -> Result<Houses, String> {
    let rest = spec.strip_prefix("house:").ok_or_else(|| format!("not a building spec: {spec}"))?;
    let (name, query) = rest.split_once('?').unwrap_or((rest, ""));
    let params = Params::parse(query)?;
    let x = params.num("x", 0.0)?;
    let l = params.num("l", 10.0)?;
    let rect_at = |x: f64, l: f64, y0: f64, y1: f64| poly::rect(x - l / 2.0, y0, x + l / 2.0, y1);
    let rect = |y0: f64, y1: f64| rect_at(x, l, y0, y1);
    let shapes = match name {
        "beside" => {
            let d = params.num("d", 2.0)?;
            let w = params.num("w", 10.0)?;
            let side = params.num("side", 1.0)?.signum();
            let notch = params.num("notch", 0.0)?;
            let deep = params.num("deep", 1.0)?;
            let (y0, y1) = (d * side, (d + w) * side);
            let (lo, hi) = (y0.min(y1), y0.max(y1));
            let rot = params.num("rot", 0.0)?;
            if rot != 0.0 {
                // Turned about its centre: a corner toward the road.
                vec![turned(rect(lo, hi), x, (lo + hi) / 2.0, rot)]
            } else if notch <= 0.0 {
                vec![rect(lo, hi)]
            } else {
                // The facade toward the axis with a notch cut into it,
                // drawn counter-clockwise from the far corners.
                let (front, back) = if side > 0.0 { (lo, hi) } else { (hi, lo) };
                let step = front + deep * side;
                let (xa, xb) = (x - l / 2.0, x + l / 2.0);
                let (na, nb) = (x - notch / 2.0, x + notch / 2.0);
                let ring = vec![[xa, front], [na, front], [na, step], [nb, step], [nb, front], [xb, front], [xb, back], [xa, back]];
                vec![vec![poly::oriented(ring, true)]]
            }
        }
        "across" => {
            // Turned, its walls cross the way obliquely: a building edge
            // and a corridor edge then meet at an angle rather than along
            // a shared line, which is where a lattice hairline would show.
            let w = params.num("w", 12.0)?;
            vec![turned(rect(-w / 2.0, w / 2.0), x, 0.0, params.num("rot", 0.0)?)]
        }
        "row" => {
            let d = params.num("d", 2.0)?;
            let w = params.num("w", 10.0)?;
            let gap = params.num("gap", 2.0)?;
            let off = (l + gap) / 2.0;
            vec![rect_at(x - off, l, d, d + w), rect_at(x + off, l, d, d + w)]
        }
        "pair" => {
            let gap = params.num("gap", 3.0)?;
            let w = params.num("w", 10.0)?;
            vec![rect(gap / 2.0, gap / 2.0 + w), rect(-gap / 2.0 - w, -gap / 2.0)]
        }
        other => return Err(format!("unknown building `{other}` in {spec}")),
    };
    let roof = Roof { shape: params.get("roof").map_or(RoofShape::Flat, RoofShape::parse), rise_m: params.opt("rise")? };
    Ok(Houses { shapes, height_m: mapped_height(params.opt("h")?, None), roof })
}

/// The buildings of a spec, clipped to the rect like read ones: every
/// house of the spec is one building.
fn synthetic(spec: &str, rect: &Rect) -> Result<Read, String> {
    let houses = parse(spec)?;
    let mut out = Read::default();
    for shape in houses.shapes {
        out.add([shape], rect, houses.height_m, houses.roof);
    }
    Ok(out)
}

/// A storey, in metres, where the source gives floors and no height.
pub const FLOOR_M: f64 = 3.0;

/// A building's height, in metres, where the source gives neither height
/// nor floors — which in Overture is most of them. Without it a town reads
/// as a few tall buildings on open ground.
pub const DEFAULT_HEIGHT_M: f64 = 5.0;

/// The height, in metres, a source gives a building: measured, else its
/// floors at [`FLOOR_M`]. `None` when it gives neither, and the reader
/// stands it at [`DEFAULT_HEIGHT_M`].
pub fn mapped_height(height: Option<f64>, floors: Option<f64>) -> Option<f64> {
    height.filter(|h| *h > 0.0).or_else(|| floors.map(|n| n * FLOOR_M).filter(|h| *h > 0.0))
}

#[cfg(test)]
mod tests {
    use crate::world::World;
    use crate::pipeline::tests::{built, plan};
    use crate::step::Step;
    use crate::terrain::tests::extent;

    use super::*;

    /// The same, surfaced, with the surface step's line.
    fn surfaced(net: &str, house: Option<&str>) -> (World, Summary) {
        let (w, ran) = built("flat", net, house, 100.0, &plan(Step::Surface));
        (w, ran.last())
    }

    /// The ways of `net` on flat ground, cut: what this step reads.
    fn roads(net: &str) -> Network {
        built("flat", net, None, 100.0, &plan(Step::Partition)).0.partition.expect("the partition step ran").network
    }

    /// The world of `net` with the house of `house`, paved to the junctions,
    /// with the surface step's line and the kerb step's.
    fn paved(net: &str, house: &str) -> (World, Summary, Summary) {
        let (w, ran) = built("flat", net, Some(house), 100.0, &plan(Step::Legs));
        (w, ran.of(Step::Surface), ran.of(Step::Kerb))
    }

    /// What the world's facade layer holds.
    fn facade(w: &World) -> &Facade {
        w.facade.as_ref().expect("the facade step ran")
    }

    #[test]
    fn every_specimen_parses_with_defaults() {
        for (spec, n, m2) in [
            ("house:beside", 1, 100.0),
            ("house:across", 1, 120.0),
            ("house:beside?d=3&x=-8&l=10&w=10&side=-1", 1, 100.0),
            ("house:beside?notch=2", 1, 98.0),
            ("house:beside?notch=2&deep=0.5&side=-1", 1, 99.0),
            ("house:row", 2, 200.0),
            ("house:pair", 2, 200.0),
        ] {
            let shapes = parse(spec).unwrap().shapes;
            assert_eq!(shapes.len(), n, "{spec}");
            assert!((poly::area(&shapes) - m2).abs() < 1e-9, "{spec}: {}", poly::area(&shapes));
            assert!(shapes.iter().all(|s| poly::ring_area(&s[0]) > 0.0), "{spec}: counter-clockwise");
        }
        let notched = parse("house:beside?d=2&notch=2").unwrap().shapes;
        assert!(!poly::contains(&notched, [0.0, 2.5]), "the notch is open");
        assert!(poly::contains(&notched, [0.0, 3.5]) && poly::contains(&notched, [2.0, 2.5]));
        let row = parse("house:row?d=2&l=10&gap=2").unwrap().shapes;
        assert!(!poly::contains(&row, [0.0, 5.0]) && poly::contains(&row, [2.0, 5.0]));
        let pair = parse("house:pair?gap=3").unwrap().shapes;
        assert!(poly::contains(&pair, [0.0, 2.0]) && poly::contains(&pair, [0.0, -2.0]) && !poly::contains(&pair, [0.0, 0.0]));
        let south = parse("house:beside?d=3&side=-1").unwrap().shapes;
        assert!(poly::contains(&south, [0.0, -5.0]));
        assert!(!poly::contains(&south, [0.0, 5.0]));
        assert!(parse("house:castle").is_err());
        assert!(parse("house:beside?d=abc").is_err());
        assert!(parse("net:straight").is_err());
        assert!(is_spec("house:across") && !is_spec("building.parquet"));
    }

    #[test]
    fn no_building_input_is_open_ground() {
        let (w, _) = surfaced("net:straight?len=200", None);
        let f = facade(&w);
        assert!(f.solid.is_empty() && f.footprints.is_empty());
        let a = poly::area(&w.surface.as_ref().unwrap().carriageway);
        assert!((a - 1100.0).abs() < 1e-3, "{a}");
    }

    #[test]
    fn a_house_beside_the_road_bounds_the_asphalt() {
        // The house's facade at y = 2 stands 0.75 m inside the 2.75 m
        // half-width, over 10 m of road: the asphalt loses 7.5 m².
        let (w, s, _) = paved("net:straight?len=200", "house:beside?d=2&l=10");
        let surf = w.surface.as_ref().unwrap();
        assert_eq!(surf.carriageway.len(), 1, "still one region");
        let a = poly::area(&surf.carriageway);
        assert!((a - (1100.0 - 7.5)).abs() < 1e-3, "{a}");
        assert!(poly::contains(&surf.carriageway, [0.0, 1.9]));
        assert!(!poly::contains(&surf.carriageway, [0.0, 2.1]));
        assert!(poly::contains(&surf.carriageway, [20.0, 2.5]));
        assert!((7.0..=8.0).contains(&s.num("carriageway_in_building_m2")), "{s}");
        // The junctions do not put it back.
        let f = w.legs.as_ref().unwrap();
        assert!(!poly::contains(&f.surface.carriageway, [0.0, 2.1]));
        assert!(poly::intersect(&f.surface.carriageway, &facade(&w).footprints).is_empty());
    }

    #[test]
    fn a_road_through_a_house_keeps_a_passage() {
        // The axis runs through the house for 10 m: the building yields a
        // 4 m corridor, the asphalt loses the 0.75 m either side of it.
        let (w, _) = surfaced("net:straight?len=200", Some("house:across?l=10&w=12"));
        let f = facade(&w);
        assert!((poly::area(&f.footprints) - 120.0).abs() < 1e-9);
        assert!(poly::contains(&f.passages, [0.0, 0.0]));
        assert!(poly::contains(&f.passages, [0.0, 1.9]));
        assert!(!poly::contains(&f.passages, [0.0, 2.1]));
        assert!(poly::contains(&f.solid, [0.0, 2.1]));
        let surf = w.surface.as_ref().unwrap();
        assert_eq!(surf.carriageway.len(), 1, "the passage keeps the road in one piece");
        let a = poly::area(&surf.carriageway);
        assert!((a - (1100.0 - 15.0)).abs() < 1e-3, "{a}");
        assert!(poly::contains(&surf.carriageway, [0.0, 1.9]));
        assert!(!poly::contains(&surf.carriageway, [0.0, 2.1]));
    }

    #[test]
    fn an_oblique_wall_leaves_no_hairline() {
        // The wall crosses the road at 30°: the road outside the house and
        // the corridor through it must be one region, not two pieces that
        // touch along a lattice-rounded edge on the wall.
        let (w, _, _) = paved("net:straight?len=200", "house:across?l=10&w=12&rot=30");
        let surf = w.surface.as_ref().unwrap();
        assert_eq!(surf.carriageway.len(), 1, "{:?}", surf.carriageway.iter().map(|s| poly::ring_area(&s[0])).collect::<Vec<_>>());
        let f = w.legs.as_ref().unwrap();
        assert_eq!(f.surface.carriageway.len(), 1);
        assert!(poly::contains(&f.surface.carriageway, [0.0, 1.9]));
    }

    #[test]
    fn a_lane_between_facades_stays_open_at_the_room() {
        // Facades 3 m apart across the axis are closed over by the
        // pocketing, and the way's corridor opens the pocket again: the
        // asphalt is the 3 m room, not the 4 m corridor, and the houses are
        // whole.
        let (w, s) = surfaced("net:straight?len=200", Some("house:pair?gap=3&l=10"));
        let f = facade(&w);
        assert!(!poly::contains(&f.built, [0.0, 0.0]) && !poly::contains(&f.built, [0.0, 1.4]));
        assert!(poly::contains(&f.built, [0.0, 1.6]) && poly::contains(&f.solid, [0.0, 1.6]));
        assert!((poly::area(&f.solid) - 200.0).abs() < 1e-6);
        let surf = w.surface.as_ref().unwrap();
        assert_eq!(surf.carriageway.len(), 1, "{s}");
        assert!(poly::contains(&surf.carriageway, [0.0, 1.4]));
        assert!(!poly::contains(&surf.carriageway, [0.0, 1.6]));
        assert!(poly::contains(&surf.carriageway, [20.0, 2.5]), "full width past the houses");
    }

    #[test]
    fn a_footway_down_an_alley_keeps_the_alley_and_the_houses() {
        // A 2 m footway down a 1.5 m gap between two houses: the pocket is
        // closed for the asphalt, opened for the footway, and the footway
        // is cut to the gap.
        let (w, _) = surfaced("net:stub?d=-2&len=100", Some("house:row?d=2&l=10&gap=1.5"));
        let f = facade(&w);
        assert!(!poly::contains(&f.built, [0.0, 5.0]), "the alley is opened for the footway");
        assert!(poly::contains(&f.built, [3.0, 3.0]), "the houses stand");
        let surf = w.surface.as_ref().unwrap();
        assert!(poly::contains(&surf.walk, [0.0, 5.0]));
        assert!(!poly::contains(&surf.walk, [0.9, 5.0]), "cut to the 1.5 m gap");
        assert!(poly::intersect(&surf.walk, &f.solid).is_empty());
    }

    #[test]
    fn the_passage_is_reported() {
        let roads = roads("net:straight?len=200");
        let (_, s) = run(&extent(), &roads, Some(Path::new("house:across?l=10&w=12"))).unwrap();
        assert_eq!(s.num("footprints"), 1.0, "{s}");
        assert_eq!(s.num("passages"), 1.0, "{s}");
        // Ten metres of axis inside, give or take the samples on the walls.
        assert!((8.0..=10.0).contains(&s.num("passage_m")), "{s}");
        // The 4 m corridor across the 10 m house: 40 m² yielded.
        assert_eq!(s.num("solid_m2"), 80.0, "{s}");
        let (_, s) = run(&extent(), &roads, Some(Path::new("house:beside?d=2&l=10"))).unwrap();
        assert_eq!(s.num("passages"), 0.0, "{s}");
        assert_eq!(s.num("solid_m2"), 100.0, "{s}");
    }

    #[test]
    fn a_footway_through_a_house_keeps_its_own_width() {
        // A 2 m footway is narrower than the passage cap: the house yields
        // exactly the footway's band and nothing of the walk is lost.
        let (w, s, _) = paved("net:sidewalk?d=6&len=100", "house:beside?d=5&w=4&l=10");
        let surf = w.surface.as_ref().unwrap();
        assert!(poly::contains(&surf.walk, [0.0, 6.0]));
        assert!(poly::contains(&surf.walk, [0.0, 5.1]));
        assert!(poly::contains(&surf.walk, [0.0, 6.9]));
        assert_eq!(s.num("walk_in_building_m2"), 0.0, "{s}");
    }

    #[test]
    fn a_house_between_the_sidewalk_and_the_kerb_is_not_a_gap() {
        // The house fills y ∈ [3, 5] over 10 m between the kerb at 2.75
        // and a sidewalk at [5, 7]. The ladder's fill stops at the wall,
        // and the kerb stations behind the wall are walled, not bare.
        let (w, _, k) = paved("net:sidewalk?d=6&len=100", "house:beside?d=3&w=2&l=10");
        let pav = &w.kerb.as_ref().unwrap().surface.walk;
        assert!(!poly::contains(pav, [0.0, 4.0]), "no pavement in the house");
        assert!(poly::contains(pav, [0.0, 2.85]), "the strip between kerb and wall is paved");
        assert!(poly::contains(pav, [20.0, 4.0]), "past the house the fill reaches the kerb");
        assert_eq!(k.num("kerb_gap"), 0.0, "{k}");
        let f = w.legs.as_ref().unwrap();
        assert!(!poly::contains(&f.surface.walk, [0.0, 4.0]));
        assert!(poly::intersect(&f.surface.walk, &facade(&w).footprints).is_empty());
    }

    #[test]
    fn a_house_at_the_corner_bounds_the_return() {
        // The house's corner stands 0.25 m off both kerbs of the tee's
        // north-west corner, inside the 4 m return's triangle.
        let (w, _, _) = paved("net:tee?len=200", "house:beside?d=3&x=-8&l=10&w=10");
        let f = w.legs.as_ref().unwrap();
        assert!(!poly::contains(&f.surface.carriageway, [-3.5, 3.5]), "no return through the house");
        assert!(poly::contains(&f.surface.carriageway, [-2.85, 2.85]), "the sliver before the wall is still returned");
        assert!(poly::intersect(&f.surface.carriageway, &facade(&w).footprints).is_empty());
        // Without the house the same point is inside the return.
        let (w, _, _) = paved("net:tee?len=200", "house:beside?d=30&x=-8&l=10&w=10");
        assert!(poly::contains(&w.legs.as_ref().unwrap().surface.carriageway, [-3.5, 3.5]));
    }

    #[test]
    fn a_footprint_is_clipped_to_the_world() {
        let roads = roads("net:straight?len=200");
        let x1 = extent().rect.x1;
        let spec = format!("house:beside?d=0&x={x1}&l=10&w=10");
        let (f, s) = run(&extent(), &roads, Some(Path::new(&spec))).unwrap();
        assert!((poly::area(&f.footprints) - 50.0).abs() < 1e-6, "{s}");
        assert_eq!(s.num("clipped"), 1.0, "{s}");
        let spec = format!("house:beside?d=0&x={}&l=10&w=10", x1 + 100.0);
        let (f, s) = run(&extent(), &roads, Some(Path::new(&spec))).unwrap();
        assert_eq!(s.num("footprints"), 0.0, "{s}");
        assert!(f.footprints.is_empty());
    }

    #[test]
    fn a_source_ring_is_oriented_and_opened() {
        let frame = Frame::at(0.0, 0.0);
        let cw = Polygon::new(
            geo_types::LineString::from(vec![(0.0, 0.0), (0.0, 1e-4), (1e-4, 1e-4), (1e-4, 0.0), (0.0, 0.0)]),
            vec![geo_types::LineString::from(vec![(2e-5, 2e-5), (5e-5, 2e-5), (5e-5, 5e-5), (2e-5, 5e-5), (2e-5, 2e-5)])],
        );
        let shape = local(&cw, &frame);
        assert_eq!(shape.len(), 2);
        assert_eq!(shape[0].len(), 4, "the closing point is dropped");
        assert!(poly::ring_area(&shape[0]) > 0.0, "the outer is counter-clockwise");
        assert!(poly::ring_area(&shape[1]) < 0.0, "the hole is clockwise");
    }
}
