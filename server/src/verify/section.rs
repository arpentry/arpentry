//! A vertical slice through the scene, as SVG.
//!
//! The other half of the argument this module makes. The scorecard says *that*
//! something is wrong and where; this says *what* it looks like, in the one
//! projection where a height model is legible.
//!
//! A 3/4 perspective screenshot is close to the worst possible image for
//! judging heights, for a person or for a model: everything is foreshortened,
//! the ground occludes the thing you are trying to see, and a 3 m step at an
//! abutment is a few pixels of shading. In section it is a 3 m step. A deck
//! ploughing into a hillside, a bore roof breaking the surface, asphalt
//! chording over a crest, two at-grade regions metres apart — each is obvious
//! at a glance and each is nearly invisible in perspective.
//!
//! It reads the same decoded tiles the checks do, so a section can be cut at
//! any offender coordinate the scorecard printed, which is the intended
//! workflow: the table names a place, this shows what is there.

use std::fmt::Write;

use crate::verify::scene::{ArchiveScan, TileScene};

/// Where and how to cut.
pub struct Cut {
    pub lon: f64,
    pub lat: f64,
    /// Direction of the section line, degrees clockwise from north.
    pub bearing: f64,
    /// Total length, in metres, centred on `(lon, lat)`.
    pub length_m: f64,
    pub zoom: u8,
    /// Plan spacing of samples along the line, in metres.
    pub step_m: f64,
}

impl Default for Cut {
    fn default() -> Cut {
        Cut { lon: 0.0, lat: 0.0, bearing: 90.0, length_m: 200.0, zoom: 16, step_m: 0.5 }
    }
}

/// One surface's profile along the cut: `(distance, height)` pairs, with gaps
/// where the surface is absent.
struct Trace {
    label: String,
    colour: &'static str,
    dash: bool,
    /// `None` marks a break — the surface stops and starts again, which for a
    /// bridge deck is the span ending and matters as much as its height.
    pts: Vec<Option<(f64, f64)>>,
}

/// Cuts a section and renders it. `None` when the cut falls outside the
/// archive entirely.
pub fn render(scan: &ArchiveScan<'_>, cut: &Cut) -> Option<String> {
    // Tiles at this zoom, indexed so each sample can find its own.
    let tiles: Vec<(u8, u32, u32, u64)> = scan.tiles_at(cut.zoom);
    if tiles.is_empty() {
        return None;
    }

    let n = ((cut.length_m / cut.step_m).ceil() as usize).clamp(2, 20_000);
    let (sin_b, cos_b) = cut.bearing.to_radians().sin_cos();
    // Metres per degree at this latitude; the section is short enough that a
    // local tangent plane is exact for the purpose.
    let m_per_lat = 110_540.0;
    let m_per_lon = 111_320.0 * cut.lat.to_radians().cos().abs().max(1e-6);

    let mut ground = Trace::new("drawn ground", "#8a7b62", false);
    let mut asphalt = Trace::new("asphalt (level 0)", "#2b2b2b", false);
    let mut asphalt_b = Trace::new("asphalt (second region)", "#c2410c", false);
    let mut deck_top = Trace::new("deck / bore top", "#1d4ed8", false);
    let mut deck_low = Trace::new("deck soffit / bore invert", "#1d4ed8", true);

    // Cache the tile of the previous sample: a section walks a short line, so
    // consecutive samples nearly always share one.
    let mut cached: Option<TileScene> = None;
    let mut dists = Vec::with_capacity(n);

    for i in 0..n {
        let d = -cut.length_m * 0.5 + i as f64 * cut.step_m;
        dists.push(d);
        let lon = cut.lon + (d * sin_b) / m_per_lon;
        let lat = cut.lat + (d * cos_b) / m_per_lat;

        let hit = cached.as_ref().is_some_and(|t| t.bounds.contains(lon, lat));
        if !hit {
            cached = tiles
                .iter()
                .find(|&&(z, x, y, _)| crate::project::Bounds::of_tile(z, x, y).contains(lon, lat))
                .and_then(|&(z, x, y, id)| scan.decode(z, x, y, id));
        }
        let Some(tile) = cached.as_ref() else {
            for t in [&mut ground, &mut asphalt, &mut asphalt_b, &mut deck_top, &mut deck_low] {
                t.pts.push(None);
            }
            continue;
        };
        let px = (lon - tile.bounds.west) / tile.bounds.width();
        let py = (lat - tile.bounds.south) / tile.bounds.height();

        ground.pts.push(
            tile.terrain.as_ref().and_then(|t| t.height_at(px, py)).map(|h| (d, h)),
        );

        // At-grade asphalt: the first two regions found, kept apart on purpose
        // so an unordered overlap draws as two lines rather than one.
        let mut paved: Vec<f64> = tile
            .roads
            .iter()
            .filter(|r| r.is_pavement())
            .filter_map(|r| r.mesh.height_range_at(px, py))
            .map(|(_, hi)| hi)
            .collect();
        paved.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        asphalt.pts.push(paved.first().map(|&h| (d, h)));
        asphalt_b.pts.push(paved.get(1).map(|&h| (d, h)));

        let structure = tile
            .roads
            .iter()
            .filter(|r| r.level != 0)
            .filter_map(|r| r.mesh.height_range_at(px, py))
            .reduce(|a, b| (a.0.min(b.0), a.1.max(b.1)));
        deck_top.pts.push(structure.map(|(_, hi)| (d, hi)));
        deck_low.pts.push(structure.map(|(lo, _)| (d, lo)));
    }

    let traces = vec![ground, asphalt, asphalt_b, deck_top, deck_low];
    if traces.iter().all(|t| t.pts.iter().all(Option::is_none)) {
        return None;
    }
    Some(svg(&traces, cut, &dists))
}

impl Trace {
    fn new(label: &str, colour: &'static str, dash: bool) -> Trace {
        Trace { label: label.to_string(), colour, dash, pts: Vec::new() }
    }

    fn present(&self) -> bool {
        self.pts.iter().any(Option::is_some)
    }
}

const W: f64 = 1100.0;
const H: f64 = 460.0;
const PAD_L: f64 = 64.0;
const PAD_R: f64 = 16.0;
const PAD_T: f64 = 40.0;
const PAD_B: f64 = 56.0;

fn svg(traces: &[Trace], cut: &Cut, dists: &[f64]) -> String {
    let (mut z_lo, mut z_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for t in traces {
        for p in t.pts.iter().flatten() {
            z_lo = z_lo.min(p.1);
            z_hi = z_hi.max(p.1);
        }
    }
    // A flat scene would divide by zero and, worse, draw a meaningless
    // full-height wiggle out of centimetres of noise.
    if !z_lo.is_finite() || z_hi - z_lo < 1.0 {
        let mid = if z_lo.is_finite() { (z_lo + z_hi) * 0.5 } else { 0.0 };
        z_lo = mid - 0.5;
        z_hi = mid + 0.5;
    }
    let pad = (z_hi - z_lo) * 0.08;
    let (z_lo, z_hi) = (z_lo - pad, z_hi + pad);
    let (d_lo, d_hi) = (dists[0], dists[dists.len() - 1]);

    let x = |d: f64| PAD_L + (d - d_lo) / (d_hi - d_lo) * (W - PAD_L - PAD_R);
    let y = |z: f64| H - PAD_B - (z - z_lo) / (z_hi - z_lo) * (H - PAD_T - PAD_B);

    let mut s = String::new();
    let _ = write!(
        s,
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="ui-monospace,SFMono-Regular,Menlo,monospace" font-size="11">
<rect width="{W}" height="{H}" fill="#fbfaf8"/>"##
    );

    // Height grid: round steps, so the vertical exaggeration is readable
    // rather than guessed at.
    let span = z_hi - z_lo;
    let step = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0]
        .into_iter()
        .find(|s| span / s <= 8.0)
        .unwrap_or(500.0);
    let mut g = (z_lo / step).ceil() * step;
    while g <= z_hi {
        let _ = write!(
            s,
            r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#e5e1d8"/>
<text x="{:.1}" y="{:.1}" fill="#9a9384" text-anchor="end" dy="3">{g:.0}</text>"##,
            PAD_L,
            y(g),
            W - PAD_R,
            y(g),
            PAD_L - 6.0,
            y(g)
        );
        g += step;
    }

    // Distance axis, including the zero mark at the requested point.
    for d in [d_lo, 0.0, d_hi] {
        let _ = write!(
            s,
            r##"<line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#d8d3c8"/>
<text x="{:.1}" y="{:.1}" fill="#9a9384" text-anchor="middle">{d:+.0} m</text>"##,
            x(d),
            PAD_T - 8.0,
            x(d),
            H - PAD_B,
            x(d),
            H - PAD_B + 16.0
        );
    }

    for t in traces {
        if !t.present() {
            continue;
        }
        let mut path = String::new();
        let mut pen_down = false;
        for p in &t.pts {
            match p {
                Some((d, z)) => {
                    let _ = write!(
                        path,
                        "{}{:.2},{:.2}",
                        if pen_down { "L" } else { "M" },
                        x(*d),
                        y(*z)
                    );
                    pen_down = true;
                }
                None => pen_down = false,
            }
        }
        let _ = write!(
            s,
            r##"<path d="{path}" fill="none" stroke="{}" stroke-width="1.8"{}/>"##,
            t.colour,
            if t.dash { r##" stroke-dasharray="4 3""## } else { "" }
        );
    }

    // Legend, only for the surfaces actually present.
    let mut ly = PAD_T + 4.0;
    for t in traces.iter().filter(|t| t.present()) {
        let _ = write!(
            s,
            r##"<line x1="{:.1}" y1="{ly:.1}" x2="{:.1}" y2="{ly:.1}" stroke="{}" stroke-width="1.8"{}/>
<text x="{:.1}" y="{ly:.1}" fill="#4a453c" dy="3">{}</text>"##,
            W - PAD_R - 210.0,
            W - PAD_R - 186.0,
            t.colour,
            if t.dash { r##" stroke-dasharray="4 3""## } else { "" },
            W - PAD_R - 180.0,
            t.label
        );
        ly += 15.0;
    }

    let _ = write!(
        s,
        r##"<text x="{PAD_L}" y="20" fill="#2b2b2b" font-size="12">section at {:.6},{:.6} — bearing {:.0}°, {:.0} m, z{} (heights in m above the ellipsoid)</text>
</svg>"##,
        cut.lon, cut.lat, cut.bearing, cut.length_m, cut.zoom
    );
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn trace(label: &str, pts: Vec<Option<(f64, f64)>>) -> Trace {
        Trace { label: label.into(), colour: "#000", dash: false, pts }
    }

    #[test]
    fn a_break_in_a_surface_starts_a_new_subpath() {
        // A deck's span ending matters as much as its height; drawing straight
        // through the gap would invent a ramp that is not there.
        let t = trace(
            "deck",
            vec![Some((0.0, 10.0)), Some((1.0, 11.0)), None, Some((3.0, 12.0))],
        );
        let out = svg(&[t], &Cut::default(), &[0.0, 1.0, 2.0, 3.0]);
        let d = out.split(r##"<path d=""##).nth(1).unwrap();
        assert_eq!(d.matches('M').count(), 2, "two subpaths expected in {d}");
    }

    #[test]
    fn a_flat_scene_does_not_become_a_full_height_wiggle() {
        // Centimetres of noise across a flat plain must draw flat, not fill the
        // frame — an autoscaled section of nothing reads as a catastrophe.
        let t = trace("ground", (0..20).map(|i| Some((i as f64, 100.0 + (i % 2) as f64 * 0.01))).collect());
        let dists: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let out = svg(&[t], &Cut::default(), &dists);
        let ys: Vec<f64> = out
            .split(r##"<path d=""##)
            .nth(1)
            .unwrap()
            .split(['M', 'L'])
            .filter_map(|p| p.split(',').nth(1))
            .filter_map(|v| v.split('"').next()?.parse::<f64>().ok())
            .collect();
        let spread = ys.iter().cloned().fold(f64::MIN, f64::max)
            - ys.iter().cloned().fold(f64::MAX, f64::min);
        assert!(spread < 20.0, "1 cm of noise drew {spread:.0} px of relief");
    }

    #[test]
    fn only_present_surfaces_reach_the_legend() {
        let present = trace("asphalt", vec![Some((0.0, 5.0)), Some((1.0, 5.0))]);
        let absent = trace("deck / bore top", vec![None, None]);
        let out = svg(&[present, absent], &Cut::default(), &[0.0, 1.0]);
        assert!(out.contains("asphalt"));
        assert!(!out.contains("deck / bore top"), "an absent surface must not be legended");
    }

    #[test]
    fn the_output_is_well_formed_svg_with_the_place_named() {
        let cut = Cut { lon: 6.9290, lat: 46.4200, ..Cut::default() };
        let t = trace("ground", vec![Some((0.0, 400.0)), Some((1.0, 402.0))]);
        let out = svg(&[t], &cut, &[0.0, 1.0]);
        assert!(out.starts_with("<svg"));
        assert!(out.ends_with("</svg>"));
        assert!(out.contains("6.929000,46.420000"), "the section must name where it was cut");
        assert!(!out.contains("NaN"), "{out}");
    }
}
