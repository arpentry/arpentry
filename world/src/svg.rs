//! The plan view: the world's 2D layers as an SVG, one group per step.
//!
//! The 3D look (`scripts/world-render.py`) answers "does it stand on the
//! ground"; this answers "is the outline right", which is a 2D question — a
//! fillet, a gap between a kerb and its pavement, a cap at a dead end are
//! all invisible from a camera in the air and obvious from straight above at
//! a metre per pixel. The file is text, opens in any browser at any zoom, and
//! is a function of the world alone (fixed precision, no ids invented here),
//! so `diff` on two plans says what moved. Rasterise it for a look at a
//! given scale:
//!
//! ```text
//! rsvg-convert -w 1600 plan.svg -o plan.png
//! ```
//!
//! Coordinates are the world's local metres with `y` negated, so the SVG's
//! y-down axis shows north up; the `viewBox` is the view window in those
//! units, and every stroke width is a width in metres. A way's stroke here is
//! *not* the ribbon step's polygon — SVG rounds its own caps and joins — but
//! it is what that step must reproduce, drawn by a renderer nobody here
//! wrote, which makes it the first thing to compare a ribbon against.

use std::fmt::Write;

use crate::frame::Rect;
use crate::poly::{self, Shapes};
use crate::width::{self, Family};
use crate::world::{Facade, Fillet, Kerb, Polyline3, Ribbon, Room, Surface, World};

/// Decimal places written per coordinate: a centimetre.
const PRECISION: usize = 2;

/// Width in metres of the axis drawn down the middle of each way's band.
const AXIS_M: f64 = 0.3;

/// The axis's colour: a blue no surface wears, so a centreline running
/// inside the asphalt — Overture ends a sidewalk on the road's axis — is
/// not mistaken for a hole in it, as a black one over a kerb was.
const AXIS_COLOR: &str = "#2a5db0";

/// Width in metres of a ribbon's outline. The ribbon layer is translucent
/// and its fills overlap, so a contour needs a line to read; the opaque
/// surfaces after it are drawn with no outline, because two outlines
/// along the kerb where asphalt and pavement meet read as a gap between
/// them.
const EDGE_M: f64 = 0.1;

/// Views narrower than this, in metres, are a debugging zoom and get the
/// construction lines — pockets, the room's raw fill — drawn as well.
const DEBUG_VIEW_M: f64 = 100.0;

/// The SVG of `world`'s 2D layers over `view` (default: the whole rect).
pub fn write_svg(world: &World, view: Option<Rect>) -> String {
    let view = view.unwrap_or(world.rect);
    let mut s = String::new();
    let _ = write!(
        s,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{} {} {} {}\">\n",
        num(view.x0),
        num(-view.y1),
        num(view.width()),
        num(view.height())
    );
    let _ = write!(
        s,
        "<rect id=\"world\" x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"#f4f1ea\"/>\n",
        num(world.rect.x0),
        num(-world.rect.y1),
        num(world.rect.width()),
        num(world.rect.height())
    );
    // The latest surface layer is the one drawn filled; the ones before it
    // are emitted as empty groups so a diff between two plans still finds
    // them. The axis is always on top, whichever surface is under it.
    let ribbons: Vec<&Ribbon> = world
        .ribbons
        .iter()
        .flat_map(|r| r.ribbons.iter())
        .filter(|r| world.surface.is_none() && touches_shape(&r.shape, &view))
        .collect();
    if let Some(roads) = &world.roads {
        let lines: Vec<&Polyline3> = roads.lines.iter().filter(|l| touches(&l.pts, &view)).collect();
        if !lines.is_empty() {
            drape(&mut s, &lines, world.ribbons.is_none());
        }
    }
    if let Some(f) = &world.facade {
        facade(&mut s, f, &view);
    }
    if world.ribbons.is_some() {
        ribbon(&mut s, &ribbons);
    }
    if let Some(surf) = &world.surface {
        surface(&mut s, surf, world.fillet.is_none(), world.kerb.is_none(), &view);
    }
    if let Some(k) = &world.kerb {
        kerb(&mut s, k, world.fillet.is_none(), &view);
    }
    if let Some(f) = &world.fillet {
        fillet(&mut s, f, world.room.is_none(), &view);
    }
    if let Some(r) = &world.room {
        room(&mut s, r, &view);
    }
    if let Some(roads) = &world.roads {
        let lines: Vec<&Polyline3> = roads.lines.iter().filter(|l| touches(&l.pts, &view)).collect();
        if !lines.is_empty() {
            axis(&mut s, &lines);
        }
    }
    s.push_str("</svg>\n");
    s
}

/// The drape layer: each way stroked at its width, round-capped and
/// round-joined, translucent so overlaps read darker. With `band` false the
/// group is emitted empty, so a diff between two plans still finds it.
fn drape(s: &mut String, lines: &[&Polyline3], band: bool) {
    s.push_str("<g id=\"drape\" fill=\"none\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n");
    s.push_str("<g id=\"band\" stroke-opacity=\"0.6\">\n");
    for line in lines.iter().filter(|_| band) {
        let _ = write!(
            s,
            "<path stroke=\"{}\" stroke-width=\"{}\" d=\"{}\"><title>{} {}{}</title></path>\n",
            color(&line.class, &line.subclass),
            num(line.width_m),
            path(&line.pts),
            escape(&line.id),
            line.class,
            if line.subclass.is_empty() { String::new() } else { format!("/{}", line.subclass) }
        );
    }
    s.push_str("</g>\n</g>\n");
}

/// The ribbon layer: one filled polygon per way, translucent so overlaps
/// read darker, outlined so a contour reads even where fills agree.
fn ribbon(s: &mut String, ribbons: &[&Ribbon]) {
    let _ = write!(
        s,
        "<g id=\"ribbon\" fill-opacity=\"0.6\" stroke=\"#000\" stroke-width=\"{}\">\n",
        num(EDGE_M)
    );
    for r in ribbons {
        let _ = write!(
            s,
            "<path fill=\"{}\" d=\"{}\"><title>{} {}{}</title></path>\n",
            color(&r.class, &r.subclass),
            shape_path(&r.shape),
            escape(&r.id),
            r.class,
            if r.subclass.is_empty() { String::new() } else { format!("/{}", r.subclass) }
        );
    }
    s.push_str("</g>\n");
}

/// The surface layer: one filled path per family, opaque, because nothing
/// overlaps any more. Each family is drawn only until a later step has
/// replaced it: the walk by the kerb's pavement, the carriageway by the
/// fillet's.
fn surface(s: &mut String, surf: &Surface, carriageway: bool, walk: bool, view: &Rect) {
    s.push_str("<g id=\"surface\">\n");
    if carriageway {
        filled(s, "carriageway", "#8c8c94", &surf.carriageway, view);
    }
    if walk {
        filled(s, "walk", "#e0a050", &surf.walk, view);
    }
    s.push_str("</g>\n");
}

/// The kerb layer: the pavement, its inner edge the kerb line, until the
/// fillet re-cuts it.
fn kerb(s: &mut String, k: &Kerb, pavement: bool, view: &Rect) {
    s.push_str("<g id=\"kerb\">\n");
    if pavement {
        filled(s, "pavement", "#e0a050", &k.pavement, view);
    }
    s.push_str("</g>\n");
}

/// The fillet layer: the carriageway with its kerb returns, the pavement
/// re-cut by them.
fn fillet(s: &mut String, f: &Fillet, pavement: bool, view: &Rect) {
    s.push_str("<g id=\"fillet\">\n");
    filled(s, "carriageway", "#8c8c94", &f.carriageway, view);
    if pavement {
        filled(s, "pavement", "#e0a050", &f.pavement, view);
    }
    s.push_str("</g>\n");
}

/// The room layer: the pavement extended to the walls.
fn room(s: &mut String, r: &Room, view: &Rect) {
    s.push_str("<g id=\"room\">\n");
    filled(s, "pavement", "#e0a050", &r.pavement, view);
    // The kerb stations still bare after everything: a dot each, at any
    // zoom where a dot can be seen, so a gap is found by looking for the
    // marker rather than for the gap.
    if view.width() < 2000.0 {
        for g in r.gaps.iter().filter(|g| view.contains(**g)) {
            let _ = write!(s, "<circle cx=\"{}\" cy=\"{}\" r=\"0.5\" fill=\"#d0202a\"/>\n", num(g[0]), num(-g[1]));
        }
    }
    // The bands and rungs before the asphalt and the walls were taken
    // back out: outlined, so a fill that failed can be told from one that
    // was taken away.
    if view.width() < DEBUG_VIEW_M {
        outlined(s, "room", "#b06030", 0.2, &r.room, view);
    }
    s.push_str("</g>\n");
}

/// The facade layer: the footprints, under every surface — nothing paved
/// stands in one, and what does is a passage the building yielded, drawn
/// over it by the surface that took it and outlined here. No footprints,
/// no group.
fn facade(s: &mut String, f: &Facade, view: &Rect) {
    if f.footprints.is_empty() {
        return;
    }
    s.push_str("<g id=\"facade\">\n");
    filled(s, "footprint", "#c9bba9", &f.footprints, view);
    // The pockets: what the closing added to the walls, which the
    // asphalt keeps out of. Drawn only at a debugging zoom; a plan of a
    // street or a town is a map.
    if view.width() < DEBUG_VIEW_M {
        filled(s, "pocket", "#e6ddd0", &poly::difference(&f.built, &f.solid), view);
    }
    outlined(s, "passage", "#8c6e50", 0.3, &f.passages, view);
    s.push_str("</g>\n");
}

/// One filled path of the regions of `shapes` that touch the view.
fn filled(s: &mut String, id: &str, fill: &str, shapes: &Shapes, view: &Rect) {
    if let Some(d) = shown(shapes, view) {
        let _ = write!(s, "<path id=\"{id}\" fill=\"{fill}\" d=\"{d}\"/>\n");
    }
}

/// One outlined path, `width_m` wide, of the regions of `shapes` that
/// touch the view.
fn outlined(s: &mut String, id: &str, stroke: &str, width_m: f64, shapes: &Shapes, view: &Rect) {
    if let Some(d) = shown(shapes, view) {
        let _ = write!(s, "<path id=\"{id}\" fill=\"none\" stroke=\"{stroke}\" stroke-width=\"{}\" d=\"{d}\"/>\n", num(width_m));
    }
}

/// The path data of the regions of `shapes` that touch the view; `None`
/// if none does.
fn shown(shapes: &Shapes, view: &Rect) -> Option<String> {
    let shown: Shapes = shapes.iter().filter(|shape| touches_shape(std::slice::from_ref(*shape), view)).cloned().collect();
    (!shown.is_empty()).then(|| shape_path(&shown))
}

/// The mapped axis of every way, over whatever surface was drawn.
fn axis(s: &mut String, lines: &[&Polyline3]) {
    let _ = write!(
        s,
        "<g id=\"axis\" fill=\"none\" stroke=\"{AXIS_COLOR}\" stroke-width=\"{}\" stroke-linecap=\"round\">\n",
        num(AXIS_M)
    );
    for line in lines {
        let _ = write!(s, "<path d=\"{}\"/>\n", path(&line.pts));
    }
    s.push_str("</g>\n");
}

/// The path data of a set of regions: every contour a closed subpath, holes
/// included, which the non-zero rule fills correctly because a hole winds
/// the other way.
fn shape_path(shapes: &Shapes) -> String {
    let mut d = String::new();
    for ring in shapes.iter().flatten() {
        for (i, p) in ring.iter().enumerate() {
            let _ = write!(d, "{}{} {}", if i == 0 { "M" } else { "L" }, num(p[0]), num(-p[1]));
        }
        d.push('Z');
    }
    d
}

/// The fill colour of a way's band: greys down the road ladder, a kerb
/// orange for a sidewalk, yellow for a crossing, green for the rest of the
/// pedestrian network.
pub fn color(class: &str, subclass: &str) -> &'static str {
    match width::family(class) {
        Family::Carriageway => match class {
            "motorway" | "trunk" => "#4a4a5a",
            "primary" | "secondary" => "#5c5c6c",
            "tertiary" => "#707080",
            "service" => "#a0a0a8",
            _ => "#88888f",
        },
        Family::Walk => match (class, subclass) {
            ("footway", "sidewalk") => "#e08a2c",
            ("footway", "crosswalk") => "#e8c928",
            ("steps", _) => "#c03030",
            ("cycleway", _) => "#3060c0",
            ("pedestrian", _) => "#d0a060",
            ("track", _) => "#907050",
            _ => "#5a9a48",
        },
    }
}

/// The path data of a polyline, `y` negated so north is up.
fn path(pts: &[[f64; 3]]) -> String {
    let mut d = String::new();
    for (i, p) in pts.iter().enumerate() {
        let _ = write!(d, "{}{} {}", if i == 0 { "M" } else { "L" }, num(p[0]), num(-p[1]));
    }
    d
}

/// Whether the bounding box of `pts` overlaps `view`. A box, not vertex
/// containment: a straight way whose two vertices both lie outside the
/// window still crosses it.
fn touches(pts: &[[f64; 3]], view: &Rect) -> bool {
    overlaps(pts.iter().map(|p| [p[0], p[1]]), view)
}

fn touches_shape(shapes: &[crate::poly::Shape], view: &Rect) -> bool {
    overlaps(shapes.iter().flatten().flatten().copied(), view)
}

fn overlaps(pts: impl Iterator<Item = [f64; 2]>, view: &Rect) -> bool {
    poly::bounds(pts).is_some_and(|[x0, y0, x1, y1]| x0 <= view.x1 && x1 >= view.x0 && y0 <= view.y1 && y1 >= view.y0)
}

/// A number at [`PRECISION`], with the trailing zeros and a lone point
/// trimmed: `12.50` → `12.5`, `3.00` → `3`, `-0.00` → `0`.
fn num(v: f64) -> String {
    let mut s = format!("{v:.PRECISION$}");
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use crate::drape::drape_line;
    use crate::terrain::{self, tests::dem};
    use crate::world::{Polyline2, Roads};

    use super::*;

    fn flat_world() -> World {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 100.0, usize::MAX);
        let t = w.terrain.as_ref().unwrap();
        let line = |id: &str, class: &str, subclass: &str, pts: Vec<[f64; 2]>| {
            drape_line(
                t,
                &Polyline2 {
                    id: id.into(),
                    class: class.into(),
                    subclass: subclass.into(),
                    width_m: width::of(class, subclass),
                    pts,
                },
            )
        };
        w.roads = Some(Roads {
            plan: Vec::new(),
            lines: vec![
                line("r", "residential", "", vec![[-600.0, -400.0], [0.0, 0.0], [500.0, 300.0]]),
                line("s", "footway", "sidewalk", vec![[-600.0, -395.0], [0.0, 5.0]]),
                line("far", "service", "", vec![[500.0, 400.0], [550.0, 450.0]]),
            ],
        });
        w
    }

    #[test]
    fn one_band_per_way_at_its_width() {
        let svg = write_svg(&flat_world(), None);
        assert_eq!(svg.matches("<path stroke=").count(), 3);
        assert!(svg.contains("stroke-width=\"5.5\""), "{svg}");
        assert!(svg.contains("stroke-width=\"2\""), "{svg}");
        assert!(svg.contains("<title>s footway/sidewalk</title>"));
        assert!(svg.contains("<title>r residential</title>"));
        // North up: the residential's first vertex at y = −400 is written
        // +400. The drape splits the line at grid lines, so only its ends are
        // known here.
        assert!(svg.contains("d=\"M-600 400L"), "{svg}");
        assert!(svg.contains("L500 -300\">"), "{svg}");
    }

    #[test]
    fn the_view_keeps_what_it_touches() {
        let view = Rect { x0: -700.0, y0: -500.0, x1: 100.0, y1: 100.0 };
        let svg = write_svg(&flat_world(), Some(view));
        assert!(svg.starts_with("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"-700 -100 800 600\">"));
        assert_eq!(svg.matches("<path stroke=").count(), 2);
        assert!(!svg.contains("<title>far"));
        // A way crossing the window with both vertices outside it is shown.
        let view = Rect { x0: -300.0, y0: -220.0, x1: -200.0, y1: -180.0 };
        let svg = write_svg(&flat_world(), Some(view));
        assert!(svg.contains("<title>r residential</title>"), "{svg}");
        assert!(!svg.contains("<title>far"));
    }

    #[test]
    fn an_empty_layer_has_no_group() {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 100.0, usize::MAX);
        w.roads = Some(Roads::default());
        let svg = write_svg(&w, None);
        assert!(!svg.contains("id=\"drape\""));
        assert!(svg.contains("id=\"world\""));
    }

    #[test]
    fn text_is_a_function_of_the_world() {
        assert_eq!(write_svg(&flat_world(), None), write_svg(&flat_world(), None));
    }

    #[test]
    fn a_ribbon_replaces_the_band_under_the_axis() {
        let mut w = crate::ribbon::tests::world("net:sidewalk?d=6&len=100");
        crate::ribbon::run(&mut w);
        let svg = write_svg(&w, None);
        assert_eq!(svg.matches("<path fill=").count(), 2, "{svg}");
        assert!(svg.contains("<title>walk-n footway/sidewalk</title>"));
        assert_eq!(svg.matches("<path stroke=").count(), 0, "the band is gone");
        assert!(svg.contains("id=\"band\""), "the group stays");
        let axis = svg.find("id=\"axis\"").unwrap();
        let ribbon = svg.find("id=\"ribbon\"").unwrap();
        assert!(axis > ribbon, "the axis is drawn over the ribbon");
        // Every contour closes.
        assert!(svg.contains("Z\"><title>"));
    }

    #[test]
    fn the_surface_replaces_the_ribbons() {
        let mut w = crate::surface::tests::world("net:crossing?d=6&len=100");
        crate::surface::run(&mut w);
        let svg = write_svg(&w, None);
        assert!(svg.contains("<path id=\"carriageway\""));
        assert!(svg.contains("<path id=\"walk\""));
        assert_eq!(svg.matches("<path fill=").count(), 0, "the ribbons are gone");
        assert!(svg.contains("id=\"ribbon\""), "the group stays");
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"surface\"").unwrap());
    }

    #[test]
    fn the_pavement_replaces_the_walk() {
        let mut w = crate::kerb::tests::world("net:sidewalk?d=6&len=100");
        crate::kerb::run(&mut w);
        let svg = write_svg(&w, None);
        assert!(svg.contains("<path id=\"carriageway\""));
        assert!(svg.contains("<path id=\"pavement\""));
        assert!(!svg.contains("<path id=\"walk\""), "the walk is gone");
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"kerb\"").unwrap());
    }

    #[test]
    fn the_fillet_replaces_both_surfaces() {
        let mut w = crate::fillet::tests::world("net:crossing?d=6&len=100");
        crate::fillet::run(&mut w);
        let svg = write_svg(&w, None);
        assert_eq!(svg.matches("<path id=\"carriageway\"").count(), 1, "{svg}");
        assert_eq!(svg.matches("<path id=\"pavement\"").count(), 1);
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"fillet\"").unwrap());
    }

    #[test]
    fn numbers_are_trimmed() {
        assert_eq!(num(12.5), "12.5");
        assert_eq!(num(3.0), "3");
        assert_eq!(num(-0.001), "0");
        assert_eq!(num(1.234), "1.23");
    }
}
