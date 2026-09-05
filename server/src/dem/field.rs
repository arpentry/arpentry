//! Synthetic terrain: a ground given by a formula instead of by a DEM archive.
//!
//! The terrain dial of the isolation harness. Everything downstream of
//! [`Dem`](super::Dem) reads elevation through one `elevation(lon, lat)` call,
//! so a ground the run's author chose costs no new plumbing: a `--terrain`
//! value that parses as a spec here builds a [`Field`], and every one of the
//! fifteen `Dem::open` sites gets it without knowing which kind of ground it
//! holds.
//!
//! **Why an analytic ground.** Every integration number this project has
//! produced comes from one real extract, where the DEM, the extract, the
//! tiling and the geometry can each be the culprit and most of the work is
//! elimination. Hold the features fixed and move only the ground — plane, then
//! ramp, then hill, then step, then the real DEM — and a defect present at the
//! plane is in the construction while one that first appears at the hill is in
//! the ground or in how the ground is read. Nothing before this could tell
//! those apart.
//!
//! A field is also *exact*. `docs/VERIFICATION.md` argues its thresholds from
//! the fact that real ground gives every structure-versus-surface check a
//! legitimate contact band. On a plane a kerb is exactly `KERB_RISE_M` and a
//! sidewalk's crossfall is exactly zero, so the scorecard stops being a
//! distribution and becomes assertions.
//!
//! # Grammar
//!
//! ```text
//! flat[?h=400]
//! ramp?grade=0.03[&bearing=90][&radius=400][&h=400][&at=6.909,46.437]
//! hill?amp=60&radius=400[&h=400][&at=…]
//! step?rise=3[&width=0][&bearing=90][&h=400][&at=…]
//! ```
//!
//! `at` is the origin the local metric frame is measured from and `h` the
//! height there; `bearing` is a compass bearing in degrees (0 = north,
//! 90 = east, matching the client's camera and `arpentry_verify --bearing`)
//! naming the direction the ground rises in. The tiler fills `at` in from the
//! centre of `--bbox` when it is omitted, so the value that reaches the
//! pipeline always carries its own origin and a run is reproducible from the
//! spec string alone.

use std::f64::consts::PI;

use crate::scene::DEG_M;

/// Default height at the origin, in metres. Roughly Montreux's town level —
/// high enough that a ramp or a hill has room to fall without going negative,
/// which is the only property of it that matters.
const DEFAULT_BASE_M: f64 = 400.0;

/// An analytic ground surface.
///
/// Cheap to evaluate and to clone, with no cache, no file descriptor and no
/// zoom: a field is the same surface at every zoom, which removes the source
/// of variance that a DEM's per-zoom source tile introduces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Field {
    kind: Kind,
    /// Origin of the local metric frame.
    lon0: f64,
    lat0: f64,
    /// Height at the origin, in metres.
    base: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    /// A horizontal plane at `base`.
    Flat,
    /// A plane of constant `grade` (rise over run) rising toward `bearing`,
    /// levelling off `radius` metres out.
    ///
    /// **The radius is why this rung is usable.** A plane has no extent of its
    /// own, so an unbounded 3 % ramp over a 12 km cut spans ±350 m of height —
    /// and the scene is the whole cut, not the bbox. Unbounded, this rung put
    /// 622–748 m into `slope.terrain_face` and `slope.terrain_tearing` at every
    /// site measured: the rung's own artifact, reported as the pipeline's.
    /// `f64::INFINITY` keeps the unbounded plane for anyone who wants it.
    Ramp { grade: f64, bearing: f64, radius: f64 },
    /// A raised cosine of height `amp` peaking at the origin and meeting the
    /// plane tangentially at `radius`. C¹ everywhere, so nothing downstream
    /// sees a crease that the author did not ask for.
    Hill { amp: f64, radius: f64 },
    /// A rise of `rise` metres across the line through the origin normal to
    /// `bearing`, spread over `width` metres. `width = 0` is a true cliff.
    Step { rise: f64, bearing: f64, width: f64 },
}

impl Field {
    /// Whether `s` names a synthetic ground rather than a DEM path.
    ///
    /// Matched on the kind word alone, so the answer does not depend on the
    /// filesystem: a spec is recognised as one before it is read, and a
    /// mistyped spec is a parse error rather than a missing file that falls
    /// back to sea level.
    pub fn is_spec(s: &str) -> bool {
        let head = s.split('?').next().unwrap_or(s);
        matches!(head, "flat" | "ramp" | "hill" | "step")
    }

    /// Parses a spec (see the module grammar).
    ///
    /// Every failure names the offending text: a field is the harness's
    /// control variable, and a spec that half-parsed would silently move it.
    pub fn parse(s: &str) -> Result<Field, String> {
        let (head, query) = match s.split_once('?') {
            Some((h, q)) => (h, q),
            None => (s, ""),
        };
        if !Field::is_spec(head) {
            return Err(format!("unknown terrain kind `{head}` (want flat, ramp, hill or step)"));
        }

        let mut params: Vec<(&str, &str)> = Vec::new();
        for pair in query.split('&').filter(|p| !p.is_empty()) {
            let (k, v) = pair
                .split_once('=')
                .ok_or_else(|| format!("terrain parameter `{pair}` is not key=value"))?;
            params.push((k, v));
        }

        let num = |key: &str, default: f64| -> Result<f64, String> {
            match params.iter().rev().find(|(k, _)| *k == key) {
                Some((_, v)) => {
                    v.parse().map_err(|_| format!("invalid `{key}` in terrain spec: {v}"))
                }
                None => Ok(default),
            }
        };
        let need = |key: &str| -> Result<f64, String> {
            if params.iter().any(|(k, _)| *k == key) {
                num(key, 0.0)
            } else {
                Err(format!("terrain `{head}` needs `{key}`"))
            }
        };

        // Origin: explicit, or (0, 0) for the tiler to fill in from the bbox.
        let (lon0, lat0) = match params.iter().rev().find(|(k, _)| *k == "at") {
            Some((_, v)) => {
                let (lon, lat) = v
                    .split_once(',')
                    .ok_or_else(|| format!("terrain `at` wants lon,lat, got {v}"))?;
                let lon: f64 =
                    lon.trim().parse().map_err(|_| format!("invalid terrain `at` longitude: {v}"))?;
                let lat: f64 =
                    lat.trim().parse().map_err(|_| format!("invalid terrain `at` latitude: {v}"))?;
                if !(-90.0..=90.0).contains(&lat) {
                    return Err(format!("terrain `at` latitude out of range: {lat}"));
                }
                (lon, lat)
            }
            None => (0.0, 0.0),
        };

        let known: &[&str] = match head {
            "flat" => &["h", "at"],
            "ramp" => &["h", "at", "grade", "bearing", "radius"],
            "hill" => &["h", "at", "amp", "radius"],
            "step" => &["h", "at", "rise", "bearing", "width"],
            _ => &[],
        };
        if let Some((k, _)) = params.iter().find(|(k, _)| !known.contains(k)) {
            let want = known.join(", ");
            return Err(format!("terrain `{head}` has no parameter `{k}` (want {want})"));
        }

        let kind = match head {
            "flat" => Kind::Flat,
            "ramp" => {
                let radius = num("radius", f64::INFINITY)?;
                if radius <= 0.0 {
                    return Err(format!("terrain `ramp` needs a positive radius, got {radius}"));
                }
                Kind::Ramp { grade: need("grade")?, bearing: num("bearing", 90.0)?, radius }
            }
            "hill" => {
                let radius = need("radius")?;
                if radius <= 0.0 {
                    return Err(format!("terrain `hill` needs a positive radius, got {radius}"));
                }
                Kind::Hill { amp: need("amp")?, radius }
            }
            "step" => {
                let width = num("width", 0.0)?;
                if width < 0.0 {
                    return Err(format!("terrain `step` width cannot be negative: {width}"));
                }
                Kind::Step { rise: need("rise")?, bearing: num("bearing", 90.0)?, width }
            }
            _ => unreachable!("kind checked above"),
        };

        Ok(Field { kind, lon0, lat0, base: num("h", DEFAULT_BASE_M)? })
    }

    /// The same field with its origin at `(lon, lat)`, for a spec that named
    /// no `at`. The tiler calls this with the centre of `--bbox`.
    pub fn at(self, lon: f64, lat: f64) -> Field {
        Field { lon0: lon, lat0: lat, ..self }
    }

    /// Whether the spec carried its own origin (rather than defaulting to the
    /// null island the tiler is expected to replace).
    pub fn has_origin(&self) -> bool {
        self.lon0 != 0.0 || self.lat0 != 0.0
    }

    /// Elevation in metres at `(lon, lat)`. Defined everywhere, at every zoom.
    pub fn elevation(&self, lon: f64, lat: f64) -> f64 {
        let east = (lon - self.lon0) * self.lat0.to_radians().cos() * DEG_M;
        let north = (lat - self.lat0) * DEG_M;
        match self.kind {
            Kind::Flat => self.base,
            // Level beyond the radius: the ramp is a *site*, not a continent.
            Kind::Ramp { grade, bearing, radius } => {
                self.base + grade * along(east, north, bearing).clamp(-radius, radius)
            }
            Kind::Hill { amp, radius } => {
                let d = east.hypot(north);
                if d >= radius {
                    self.base
                } else {
                    self.base + amp * 0.5 * (1.0 + (PI * d / radius).cos())
                }
            }
            Kind::Step { rise, bearing, width } => {
                let s = along(east, north, bearing);
                let t = if width <= 0.0 {
                    if s > 0.0 {
                        1.0
                    } else {
                        0.0
                    }
                } else {
                    let u = (s / width + 0.5).clamp(0.0, 1.0);
                    u * u * (3.0 - 2.0 * u)
                };
                self.base + rise * t
            }
        }
    }

    /// The spec string this field round-trips from, origin included. What the
    /// tiler records once it has resolved the origin, so a run is reproducible
    /// from what it was given.
    pub fn spec(&self) -> String {
        let at = format!("&at={:.6},{:.6}", self.lon0, self.lat0);
        let h = format!("h={}", self.base);
        match self.kind {
            Kind::Flat => format!("flat?{h}{at}"),
            Kind::Ramp { grade, bearing, radius } => {
                // An infinite radius is the absence of one, and prints as such
                // so a round-tripped spec reads the way it was written.
                let r = if radius.is_finite() { format!("&radius={radius}") } else { String::new() };
                format!("ramp?grade={grade}&bearing={bearing}{r}&{h}{at}")
            }
            Kind::Hill { amp, radius } => format!("hill?amp={amp}&radius={radius}&{h}{at}"),
            Kind::Step { rise, bearing, width } => {
                format!("step?rise={rise}&bearing={bearing}&width={width}&{h}{at}")
            }
        }
    }
}

/// Distance along `bearing` (compass degrees, 0 = north) from a local offset
/// in metres east and north.
fn along(east: f64, north: f64, bearing: f64) -> f64 {
    let b = bearing.to_radians();
    east * b.sin() + north * b.cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A place with a cosine, so the east/north conversion is exercised at a
    /// latitude where it is not the identity.
    const LON: f64 = 6.9091;
    const LAT: f64 = 46.4374;

    fn field(spec: &str) -> Field {
        Field::parse(spec).unwrap_or_else(|e| panic!("{spec}: {e}"))
    }

    #[test]
    fn a_path_is_not_a_spec() {
        assert!(!Field::is_spec("data/overture-ch/terrain-hires.pmtiles"));
        assert!(!Field::is_spec("flatlands.pmtiles"));
        assert!(Field::is_spec("flat"));
        assert!(Field::is_spec("ramp?grade=0.03"));
    }

    #[test]
    fn flat_is_flat_everywhere() {
        let f = field("flat?h=372").at(LON, LAT);
        assert_eq!(f.elevation(LON, LAT), 372.0);
        assert_eq!(f.elevation(LON + 0.01, LAT - 0.02), 372.0);
    }

    #[test]
    fn a_ramp_rises_at_its_grade() {
        // 100 m east of the origin, at 3 %, is 3 m up.
        let f = field("ramp?grade=0.03&h=400").at(LON, LAT);
        let east = LON + 100.0 / (DEG_M * LAT.to_radians().cos());
        assert!((f.elevation(east, LAT) - 403.0).abs() < 1e-6, "{}", f.elevation(east, LAT));
        // …and northward is across the slope, so level.
        assert!((f.elevation(LON, LAT + 0.001) - 400.0).abs() < 1e-9);
    }

    /// The radius is what keeps the rung a site rather than a continent: an
    /// unbounded 3 % plane over a 12 km cut spans ±350 m, and that height was
    /// read as the pipeline's rather than the rung's at three sites.
    #[test]
    fn a_ramp_levels_off_beyond_its_radius() {
        let f = field("ramp?grade=0.03&radius=200&h=400").at(LON, LAT);
        let east_m = |m: f64| LON + m / (DEG_M * LAT.to_radians().cos());

        // Inside the radius it is the same ramp.
        assert!((f.elevation(east_m(100.0), LAT) - 403.0).abs() < 1e-6);
        // At the radius it reaches its full rise…
        assert!((f.elevation(east_m(200.0), LAT) - 406.0).abs() < 1e-6);
        // …and never exceeds it, however far out.
        assert!((f.elevation(east_m(20_000.0), LAT) - 406.0).abs() < 1e-6);
        assert!((f.elevation(east_m(-20_000.0), LAT) - 394.0).abs() < 1e-6);
    }

    #[test]
    fn an_unbounded_ramp_is_still_available() {
        let f = field("ramp?grade=0.03&h=400").at(LON, LAT);
        let far = LON + 20_000.0 / (DEG_M * LAT.to_radians().cos());
        assert!((f.elevation(far, LAT) - 1000.0).abs() < 1e-3, "{}", f.elevation(far, LAT));
    }

    /// `step` was never the unbounded one: its height range is exactly `rise`,
    /// however far the ground extends, so it needs no radius.
    #[test]
    fn a_step_is_bounded_by_its_own_rise() {
        let f = field("step?rise=3&h=390").at(LON, LAT);
        let east_m = |m: f64| LON + m / (DEG_M * LAT.to_radians().cos());
        assert_eq!(f.elevation(east_m(50_000.0), LAT), 393.0);
        assert_eq!(f.elevation(east_m(-50_000.0), LAT), 390.0);
    }

    #[test]
    fn a_ramp_bearing_turns_the_slope() {
        let f = field("ramp?grade=0.05&bearing=0").at(LON, LAT);
        let north = LAT + 100.0 / DEG_M;
        assert!((f.elevation(LON, north) - 405.0).abs() < 1e-6);
        assert!((f.elevation(LON + 0.001, LAT) - 400.0).abs() < 1e-9);
    }

    #[test]
    fn a_hill_peaks_at_its_origin_and_lands_flat() {
        let f = field("hill?amp=60&radius=400&h=400").at(LON, LAT);
        assert!((f.elevation(LON, LAT) - 460.0).abs() < 1e-9);
        // Half a radius out: the raised cosine's midpoint.
        let half = LAT + 200.0 / DEG_M;
        assert!((f.elevation(LON, half) - 430.0).abs() < 1e-6, "{}", f.elevation(LON, half));
        // Beyond the radius, and just inside it, both read the plane: C¹.
        let edge = LAT + 399.9 / DEG_M;
        assert!((f.elevation(LON, edge) - 400.0).abs() < 1e-3);
        assert_eq!(f.elevation(LON, LAT + 800.0 / DEG_M), 400.0);
    }

    #[test]
    fn a_step_is_a_cliff_unless_given_a_width() {
        let f = field("step?rise=3&h=400").at(LON, LAT);
        let dx = 0.1 / (DEG_M * LAT.to_radians().cos());
        assert_eq!(f.elevation(LON - dx, LAT), 400.0);
        assert_eq!(f.elevation(LON + dx, LAT), 403.0);

        // Spread over 20 m, the same rise is half-way up at the origin.
        let w = field("step?rise=3&width=20&h=400").at(LON, LAT);
        assert!((w.elevation(LON, LAT) - 401.5).abs() < 1e-9);
        let past = LON + 100.0 * dx; // 10 m east, well past the 20 m spread's half
        assert!((w.elevation(past, LAT) - 403.0).abs() < 1e-9);
        assert!((w.elevation(LON - 100.0 * dx, LAT) - 400.0).abs() < 1e-9);
    }

    #[test]
    fn a_spec_round_trips_through_its_string() {
        for spec in [
            "flat",
            "ramp?grade=0.03",
            "ramp?grade=0.03&radius=250",
            "hill?amp=60&radius=400",
            "step?rise=3&width=5",
        ] {
            let f = field(spec).at(LON, LAT);
            let again = field(&f.spec());
            assert_eq!(f.kind, again.kind, "{spec}");
            assert!((f.elevation(LON + 0.003, LAT + 0.002)
                - again.elevation(LON + 0.003, LAT + 0.002))
                .abs()
                < 1e-6);
        }
    }

    #[test]
    fn a_mistyped_spec_is_an_error_not_a_default() {
        assert!(Field::parse("ramp").is_err(), "grade is not optional");
        assert!(Field::parse("ramp?grade=steep").is_err());
        assert!(Field::parse("ramp?gradient=0.03").is_err(), "a near-miss key must not be ignored");
        assert!(Field::parse("hill?amp=60").is_err(), "radius is not optional");
        assert!(Field::parse("hill?amp=60&radius=0").is_err());
        assert!(Field::parse("ramp?grade=0.03&radius=0").is_err());
        assert!(Field::parse("flat?h").is_err());
        assert!(Field::parse("flat?at=6.9").is_err());
    }
}
