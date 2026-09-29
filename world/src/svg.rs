//! The plan view: the world's 2D layers as an SVG, one group per step.
//!
//! The GLB answers "does it stand on the ground"; this answers "is the
//! outline right", which is a 2D question — a fillet, a gap between a kerb
//! and its pavement, a cap at a dead end are all invisible from a camera in
//! the air and obvious from straight above at a metre per pixel. The file
//! is text, opens in any browser at any zoom, and
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
use crate::world::{Sheets, 
    Bench, Crossings, Facade, Kerb, Kind, Polyline3, Profile, Profiles, Ribbon, Room, Solved, Structure,
    Surface, Tri, World,
};

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
    let view = view.unwrap_or(world.extent.rect);
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
        num(world.extent.rect.x0),
        num(-world.extent.rect.y1),
        num(world.extent.rect.width()),
        num(world.extent.rect.height())
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
        surface(&mut s, surf, world.legs.is_none(), world.kerb.is_none(), &view);
    }
    if let Some(k) = &world.kerb {
        kerb(&mut s, k, world.legs.is_none(), &view);
    }
    if let Some(l) = &world.legs {
        legs(&mut s, l, world.room.is_none(), &view);
    }
    if let Some(r) = &world.room {
        room(&mut s, r, &view);
    }
    if let Some(sh) = &world.sheets {
        sheets(&mut s, sh, &view);
    }
    if let Some(p) = world.solved() {
        profile(&mut s, p, &view);
    }
    if let Some(st) = &world.structure {
        structure(&mut s, st, &view);
    }
    if let Some(b) = &world.bench {
        mesh(&mut s, &[("carriageway", &b.carriageway), ("pavement", &b.pavement), ("ballast", &b.ballast)], &view);
        bench(&mut s, b, &view);
    } else if let (Some(m), Some(a)) = (&world.mesh, &world.arrangement) {
        use crate::world::Material;
        let of = |x: Material| crate::mesh::view(m, a, |f| f.material == x);
        let (c, p, b) = (of(Material::Carriageway), of(Material::Pavement), of(Material::Ballast));
        mesh(&mut s, &[("carriageway", &c), ("pavement", &p), ("ballast", &b)], &view);
    }
    // The crossings last of the layers: a mark, not a surface, and the
    // one thing here drawn over the asphalt on purpose — a ring under an
    // opaque roadway is a ring nobody finds.
    if let Some(c) = &world.crossing {
        crossing(&mut s, c, &view);
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
/// fillet's. The ballast is laid once and re-cut by nothing, so it is
/// always drawn here.
fn surface(s: &mut String, surf: &Surface, carriageway: bool, walk: bool, view: &Rect) {
    s.push_str("<g id=\"surface\">\n");
    filled(s, "ballast", BALLAST_FILL, &surf.ballast, view);
    if carriageway {
        filled(s, "carriageway", "#8c8c94", &surf.carriageway, view);
    }
    if walk {
        filled(s, "walk", "#e0a050", &surf.walk, view);
    }
    s.push_str("</g>\n");
}

/// The track bed's fill: the server's `rail_surface`.
const BALLAST_FILL: &str = "#9e968a";

/// The kerb layer: the pavement, its inner edge the kerb line, until the
/// junctions re-cut it.
fn kerb(s: &mut String, k: &Kerb, pavement: bool, view: &Rect) {
    s.push_str("<g id=\"kerb\">\n");
    if pavement {
        filled(s, "pavement", "#e0a050", &k.surface.walk, view);
    }
    s.push_str("</g>\n");
}

/// The legs layer: the carriageway built from the junctions' legs, the
/// pavement laid back outside it until the room takes over, each junction
/// outlined, and the kerb stations still bare.
fn legs(s: &mut String, l: &crate::world::Legs, pavement: bool, view: &Rect) {
    s.push_str("<g id=\"legs\">\n");
    filled(s, "carriageway", "#8c8c94", &l.surface.carriageway, view);
    if pavement {
        filled(s, "pavement", "#e0a050", &l.surface.walk, view);
    }
    let junctions: Shapes = l.junctions.iter().flat_map(|j| j.shape.iter().cloned()).collect();
    outlined(s, "junction", "#2040d0", 0.1, &junctions, view);
    for g in l.gaps.iter().filter(|g| view.contains(**g)) {
        let _ = write!(s, "<circle cx=\"{}\" cy=\"{}\" r=\"0.5\" fill=\"#8020c0\"/>\n", num(g[0]), num(-g[1]));
    }
    s.push_str("</g>\n");
}

/// The room layer: the pavement extended to the walls.
fn room(s: &mut String, r: &Room, view: &Rect) {
    s.push_str("<g id=\"room\">\n");
    filled(s, "pavement", "#e0a050", &r.surface.walk, view);
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

/// The sheets layer: the outline of every paved sheet, and the part of it
/// that is over a span filled.
///
/// A sheet is what may merge, so its outline is the answer to "is this one
/// surface or two" in the place that question is decided — the plan. Drawn
/// as an outline rather than a fill because the surfaces below it are
/// filled and the point is to see the *boundary*: a sheet whose junction
/// is one clean ring has merged, and one with a notch at every leg has
/// not.
fn sheets(s: &mut String, sh: &Sheets, view: &Rect) {
    s.push_str("<g id=\"sheet\">\n");
    let spans: Shapes = sh.sheets.iter().flat_map(|x| x.spans.iter().cloned()).collect();
    filled(s, "span", "#c86432", &spans, view);
    let all: Shapes = sh.shapes();
    outlined(s, "sheet_edge", "#20a0c0", 0.4, &all, view);
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

/// The mesh layer: the wireframe of every triangle touching the view, in
/// windows under [`DEBUG_VIEW_M`] only — a sliver is visible from above at
/// that scale and nothing but noise at any other. The group is emitted
/// empty otherwise, so a diff between two plans still finds it.
fn mesh(s: &mut String, layers: &[(&str, &Tri)], view: &Rect) {
    s.push_str("<g id=\"mesh\" fill=\"none\" stroke=\"#000\" stroke-opacity=\"0.5\" stroke-width=\"0.05\">\n");
    for (name, tri) in layers {
        let _ = write!(s, "<g id=\"{name}\">\n");
        if view.width() < DEBUG_VIEW_M {
            let mut d = String::new();
            let mut seen: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
            for t in tri.indices.chunks_exact(3) {
                let pts = [tri.positions[t[0] as usize], tri.positions[t[1] as usize], tri.positions[t[2] as usize]];
                if !overlaps(pts.iter().map(|p| [p[0], p[1]]), view) {
                    continue;
                }
                for e in 0..3 {
                    let (a, b) = (t[e], t[(e + 1) % 3]);
                    if seen.insert((a.min(b), a.max(b))) {
                        let (p, q) = (tri.positions[a as usize], tri.positions[b as usize]);
                        let _ = write!(d, "M{} {}L{} {}", num(p[0]), num(-p[1]), num(q[0]), num(-q[1]));
                    }
                }
            }
            if !d.is_empty() {
                let _ = write!(s, "<path d=\"{d}\"/>\n");
            }
        }
        s.push_str("</g>\n");
    }
    s.push_str("</g>\n");
}

/// The structure layer: every span's own paving, which no surface step
/// lays.
///
/// **Outlined, and off the carriageway's own palette on purpose.** A deck
/// used to fill at `#9a948c` against a carriageway of `#8c8c94` and ballast
/// of `#9e968a` — three greys within ten units of each other, no stroke
/// between any of them — so a viaduct painted no differently from the road
/// it carries. A deck and a bore now get their own hue (slate blue, darker
/// underground than aloft — elevated reads lighter) and the group's own
/// outline, the same device [`ribbon`] uses to keep an opaque fill legible
/// against its neighbours.
fn structure(s: &mut String, st: &Structure, view: &Rect) {
    let _ = write!(s, "<g id=\"structure\" stroke=\"#20202c\" stroke-width=\"{}\">\n", num(1.0));
    for (kind, shapes) in &st.plan {
        let (id, fill) = match kind {
            Kind::Tunnel(_) => ("bore", "#3d5266"),
            _ => ("deck", "#5c7a94"),
        };
        filled(s, id, fill, shapes, view);
    }
    s.push_str("</g>\n");
}

/// The bench layer: where the room's height field steps — the line
/// between two carriageways whose domains meet at different heights, which
/// the mesh draws as a retaining wall. A dot each, at any zoom where a dot
/// can be seen, on the same terms as the room's bare kerb stations: a wall
/// is found by looking for its marker rather than for the wall.
fn bench(s: &mut String, b: &Bench, view: &Rect) {
    s.push_str("<g id=\"bench\">\n");
    if view.width() < 2000.0 {
        for p in b.steps.iter().filter(|p| view.contains(**p)) {
            let _ = write!(s, "<circle cx=\"{}\" cy=\"{}\" r=\"0.5\" fill=\"#7030a0\"/>\n", num(p[0]), num(-p[1]));
        }
    }
    s.push_str("</g>\n");
}

/// A station this far off the ground, in metres, is drawn as cut or fill;
/// nearer than that the axis over the asphalt says enough.
const OFF_MIN_M: f64 = 0.3;

/// The profile layer: along every ground piece, the stretches in cut
/// (blue) and in fill (red) at the way's width, translucent over the
/// surface; a deck dashed and a bore dotted, at their width, so a mapped
/// span that degraded to ground is the one stretch left blank.
fn profile(s: &mut String, p: &Profiles, view: &Rect) {
    s.push_str("<g id=\"profile\" fill=\"none\" stroke-linecap=\"butt\" stroke-linejoin=\"round\">\n");
    let shown: Vec<&Profile> = p.profiles.iter().filter(|p| touches(&p.line(), view)).collect();
    for (id, color, side) in [("cut", "#3a6fd8", -1.0), ("fill", "#d84a3a", 1.0)] {
        let _ = write!(s, "<g id=\"{id}\" stroke=\"{color}\" stroke-opacity=\"0.55\">\n");
        for p in shown.iter().filter(|p| !p.has_chord()) {
            let mut run: Vec<[f64; 3]> = Vec::new();
            let mut flush = |run: &mut Vec<[f64; 3]>| {
                if run.len() >= 2 {
                    let _ = write!(s, "<path stroke-width=\"{}\" d=\"{}\"><title>{}</title></path>\n", num(p.width_m), path(run), escape(&p.id));
                }
                run.clear();
            };
            for st in &p.stations {
                if (st.h - st.ground) * side >= OFF_MIN_M {
                    run.push([st.p[0], st.p[1], st.h]);
                } else {
                    flush(&mut run);
                }
            }
            flush(&mut run);
        }
        s.push_str("</g>\n");
    }
    for (id, color, dash, solved) in
        [("deck", "#20202c", "6 3", Solved::Deck), ("bore", "#7a3fb0", "1.5 3", Solved::Bore)]
    {
        let _ = write!(s, "<g id=\"{id}\" stroke=\"{color}\" stroke-opacity=\"0.9\" stroke-dasharray=\"{dash}\">\n");
        for p in shown.iter().filter(|p| p.has_chord()) {
            let mut run: Vec<[f64; 3]> = Vec::new();
            let mut flush = |run: &mut Vec<[f64; 3]>| {
                if run.len() >= 2 {
                    let _ = write!(s, "<path stroke-width=\"{}\" d=\"{}\"><title>{} {}</title></path>\n", num(p.width_m), path(run), escape(&p.id), p.spans.iter().find(|s| s.kind.is_structure()).map_or("span", |s| s.kind.name()));
                }
                run.clear();
            };
            for st in &p.stations {
                if st.solved == solved {
                    run.push([st.p[0], st.p[1], st.h]);
                } else {
                    flush(&mut run);
                }
            }
            flush(&mut run);
        }
        s.push_str("</g>\n");
    }
    s.push_str("</g>\n");
}

/// Radius in metres of a crossing's mark: wide enough to find at a town's
/// zoom, narrow enough not to hide the two axes under it.
const CROSSING_R_M: f64 = 3.0;

/// The crossing layer: one ring per grade separation — green where the
/// solve met the clearance, red where it did not — and an orange disc
/// where two interiors cross at one level, which is a data error and the
/// one thing here that is not solved. The mark carries what was asked and
/// what was got, so a red one is read without re-deriving anything.
fn crossing(s: &mut String, c: &Crossings, view: &Rect) {
    s.push_str("<g id=\"crossing\" fill=\"none\">\n");
    for x in c.crossings.iter().filter(|x| view.contains(x.at)) {
        let met = x.shortfall() <= 0.0;
        let _ = write!(
            s,
            "<circle cx=\"{}\" cy=\"{}\" r=\"{}\" stroke=\"{}\" stroke-width=\"0.6\"><title>need {:.2} had {:.2} have {:.2}</title></circle>\n",
            num(x.at[0]),
            num(-x.at[1]),
            num(CROSSING_R_M),
            if met { "#2f8f4e" } else { "#d84a3a" },
            x.need,
            x.had,
            x.have
        );
    }
    for p in c.same.iter().filter(|p| view.contains(**p)) {
        let _ = write!(
            s,
            "<circle cx=\"{}\" cy=\"{}\" r=\"{}\" fill=\"#e08a20\"><title>same level</title></circle>\n",
            num(p[0]),
            num(-p[1]),
            num(CROSSING_R_M)
        );
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
        Family::Rail => match class {
            "funicular" => "#7a6450",
            "narrow_gauge" => "#857a6c",
            _ => "#6f675c",
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
    use crate::lattice::drape_line;
    use crate::terrain::{self, tests::{dem, extent}};
    use crate::world::{Polyline2, Roads};

    use crate::pipeline::tests::bbox;

    use super::*;

    fn flat_world() -> World {
        let (ground, _) = terrain::run(&extent(), &mut dem("flat"), 100.0, usize::MAX);
        let mut w = World::new(bbox());
        w.terrain = Some(ground);
        let t = w.terrain.as_ref().expect("just set");
        let line = |id: &str, class: &str, subclass: &str, pts: Vec<[f64; 2]>| {
            drape_line(
                t,
                &Polyline2 {
                    id: id.into(),
                    class: class.into(),
                    subclass: subclass.into(),
                    width_m: width::of(class, subclass),
                    way: usize::MAX,
                    a0: 0.0,
                    a1: 0.0,
                    kind: crate::world::Kind::Ground,
                    pts,
                },
            )
        };
        w.roads = Some(Roads {
            ways: Vec::new(),
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
        let (ground, _) = terrain::run(&extent(), &mut dem("flat"), 100.0, usize::MAX);
        let mut w = World::new(bbox());
        w.terrain = Some(ground);
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
        let (w, _) = crate::ribbon::tests::world("net:sidewalk?d=6&len=100");
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
        let (w, _) = crate::surface::tests::world("net:crossing?d=6&len=100");
        let svg = write_svg(&w, None);
        assert!(svg.contains("<path id=\"carriageway\""));
        assert!(svg.contains("<path id=\"walk\""));
        assert_eq!(svg.matches("<path fill=").count(), 0, "the ribbons are gone");
        assert!(svg.contains("id=\"ribbon\""), "the group stays");
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"surface\"").unwrap());
    }

    #[test]
    fn the_pavement_replaces_the_walk() {
        let (w, _) = crate::kerb::tests::world("net:sidewalk?d=6&len=100");
        let svg = write_svg(&w, None);
        assert!(svg.contains("<path id=\"carriageway\""));
        assert!(svg.contains("<path id=\"pavement\""));
        assert!(!svg.contains("<path id=\"walk\""), "the walk is gone");
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"kerb\"").unwrap());
    }

    #[test]
    fn the_junctions_replace_both_surfaces() {
        let (w, _) = crate::pipeline::tests::built("flat", "net:crossing?d=6&len=100", None, 100.0, &crate::pipeline::tests::plan(crate::step::Step::Legs));
        let svg = write_svg(&w, None);
        assert_eq!(svg.matches("<path id=\"carriageway\"").count(), 1, "{svg}");
        assert_eq!(svg.matches("<path id=\"pavement\"").count(), 1);
        assert!(svg.find("id=\"axis\"").unwrap() > svg.find("id=\"legs\"").unwrap());
    }

    #[test]
    fn numbers_are_trimmed() {
        assert_eq!(num(12.5), "12.5");
        assert_eq!(num(3.0), "3");
        assert_eq!(num(-0.001), "0");
        assert_eq!(num(1.234), "1.23");
    }
}
