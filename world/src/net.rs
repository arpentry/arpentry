//! The synthetic network: a `--segments` spec instead of a parquet.
//!
//! `--terrain` accepts `flat|ramp|hill|step` so a step's output on the
//! ground is an assertion rather than a distribution. The network gets the
//! same dial: a handful of ways drawn in the local frame with the class and
//! subclass the data would carry, so a ribbon's area, a junction's region
//! count and a pavement's edge have a known answer.
//!
//! ```text
//! net:straight[?len=200&class=residential]   one way along x
//! net:tee                                     two halves of it and a leg from the north, meeting at the origin
//! net:cross                                   four legs meeting at the origin
//! net:hairpin?angle=20                        one way bent by `angle` degrees at the origin
//! net:dual?gap=4                              two ways, their kerbs `gap` m apart
//! net:sidewalk?d=6                            the way and a sidewalk `d` m off its axis
//! net:corner?d=5                              the way turning north, a sidewalk wrapping the outside
//! net:crossing?d=6                            sidewalks either side and a crosswalk between them
//! net:roundabout?r=15&d=5                     a ring road in four arcs, four legs, a sidewalk ring `d` m outside its kerb
//! ```
//!
//! Every parameter has a default, so `net:cross` is a complete spec. The
//! way is `residential` unless `class=` says otherwise, and a specimen's
//! ids are stable words (`road`, `walk-n`, …) so a test can pick one out.
//! Ways meet the way Overture's do: cut at every connector, so a junction
//! is always a meeting of way ends and never a crossing of interiors — two
//! ways whose interiors cross are a bridge over a road, not a junction.

use crate::world::Polyline2;

/// Whether `s` is a spec rather than a path.
pub fn is_spec(s: &str) -> bool {
    s.starts_with("net:")
}

/// The ways of `spec`, in local metres.
pub fn parse(spec: &str) -> Result<Vec<Polyline2>, String> {
    let rest = spec.strip_prefix("net:").ok_or_else(|| format!("not a network spec: {spec}"))?;
    let (name, query) = rest.split_once('?').unwrap_or((rest, ""));
    let params = Params::parse(query)?;
    let len = params.num("len", 200.0)?;
    let class = params.get("class").unwrap_or("residential");
    let half = len / 2.0;
    let road = |id: &str, pts: Vec<[f64; 2]>| line(id, class, "", pts);
    let walk = |id: &str, subclass: &str, pts: Vec<[f64; 2]>| line(id, "footway", subclass, pts);
    let x_road = |id: &str| road(id, vec![[-half, 0.0], [half, 0.0]]);
    let ways = match name {
        "straight" => vec![x_road("road")],
        "tee" => vec![
            road("road-w", vec![[-half, 0.0], [0.0, 0.0]]),
            road("road-e", vec![[0.0, 0.0], [half, 0.0]]),
            road("leg", vec![[0.0, 0.0], [0.0, half]]),
        ],
        "cross" => vec![
            road("road-w", vec![[-half, 0.0], [0.0, 0.0]]),
            road("road-e", vec![[0.0, 0.0], [half, 0.0]]),
            road("leg-s", vec![[0.0, -half], [0.0, 0.0]]),
            road("leg-n", vec![[0.0, 0.0], [0.0, half]]),
        ],
        "hairpin" => {
            let a = params.num("angle", 20.0)?.to_radians() / 2.0;
            let (dx, dy) = (half * a.cos(), half * a.sin());
            vec![road("road", vec![[dx, dy], [0.0, 0.0], [dx, -dy]])]
        }
        "dual" => {
            let gap = params.num("gap", 4.0)?;
            let y = (crate::width::of(class, "") + gap) / 2.0;
            vec![
                road("road-n", vec![[-half, y], [half, y]]),
                road("road-s", vec![[half, -y], [-half, -y]]),
            ]
        }
        "sidewalk" => {
            let d = params.num("d", 6.0)?;
            vec![x_road("road"), walk("walk-n", "sidewalk", vec![[-half, d], [half, d]])]
        }
        "corner" => {
            let d = params.num("d", 5.0)?;
            vec![
                road("road", vec![[-half, 0.0], [0.0, 0.0], [0.0, half]]),
                walk("walk", "sidewalk", vec![[-half, -d], [d, -d], [d, half]]),
            ]
        }
        "crossing" => {
            let d = params.num("d", 6.0)?;
            vec![
                x_road("road"),
                walk("walk-n", "sidewalk", vec![[-half, d], [half, d]]),
                walk("walk-s", "sidewalk", vec![[-half, -d], [half, -d]]),
                walk("crossing", "crosswalk", vec![[0.0, -d], [0.0, d]]),
            ]
        }
        "roundabout" => {
            let r = params.num("r", 15.0)?;
            let d = params.num("d", 5.0)?;
            let ring = |radius: f64| -> Vec<[f64; 2]> {
                (0..=36)
                    .map(|k| {
                        let a = (k % 36) as f64 * std::f64::consts::TAU / 36.0;
                        [radius * a.cos(), radius * a.sin()]
                    })
                    .collect()
            };
            // Four arcs, split at the legs: Overture cuts a way at every
            // connector, so a junction is always a meeting of way ends.
            let full = ring(r);
            let mut ways: Vec<Polyline2> = (0..4)
                .map(|q| road(&format!("arc-{q}"), full[q * 9..=(q + 1) * 9].to_vec()))
                .collect();
            for (id, dir) in [("leg-e", [1.0, 0.0]), ("leg-n", [0.0, 1.0]), ("leg-w", [-1.0, 0.0]), ("leg-s", [0.0, -1.0])] {
                ways.push(road(id, vec![[dir[0] * r, dir[1] * r], [dir[0] * half, dir[1] * half]]));
            }
            let kerb = r + crate::width::of(class, "") / 2.0;
            ways.push(walk("walk", "sidewalk", ring(kerb + d)));
            ways
        }
        other => return Err(format!("unknown network `{other}` in {spec}")),
    };
    Ok(ways)
}

fn line(id: &str, class: &str, subclass: &str, pts: Vec<[f64; 2]>) -> Polyline2 {
    Polyline2 {
        id: id.into(),
        class: class.into(),
        subclass: subclass.into(),
        width_m: crate::width::of(class, subclass),
        pts,
    }
}

/// The `k=v&k=v` part of a spec.
struct Params(Vec<(String, String)>);

impl Params {
    fn parse(query: &str) -> Result<Params, String> {
        let mut out = Vec::new();
        for pair in query.split('&').filter(|s| !s.is_empty()) {
            let (k, v) = pair.split_once('=').ok_or_else(|| format!("expected k=v, got `{pair}`"))?;
            out.push((k.to_string(), v.to_string()));
        }
        Ok(Params(out))
    }

    fn get(&self, key: &str) -> Option<&str> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    fn num(&self, key: &str, default: f64) -> Result<f64, String> {
        match self.get(key) {
            None => Ok(default),
            Some(v) => v.parse().map_err(|_| format!("invalid {key}: {v}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_specimen_parses_with_defaults() {
        for (spec, n) in [
            ("net:straight", 1),
            ("net:tee", 3),
            ("net:cross", 4),
            ("net:hairpin", 1),
            ("net:dual", 2),
            ("net:sidewalk", 2),
            ("net:corner", 2),
            ("net:crossing", 4),
            ("net:roundabout", 9),
        ] {
            let ways = parse(spec).unwrap();
            assert_eq!(ways.len(), n, "{spec}");
            assert!(ways.iter().all(|w| w.pts.len() >= 2), "{spec}");
        }
    }

    #[test]
    fn parameters_are_read() {
        let ways = parse("net:straight?len=100&class=primary").unwrap();
        assert_eq!(ways[0].class, "primary");
        assert_eq!(ways[0].pts, vec![[-50.0, 0.0], [50.0, 0.0]]);
        let ways = parse("net:sidewalk?d=3.5").unwrap();
        assert_eq!(ways[1].subclass, "sidewalk");
        assert_eq!(ways[1].pts[0][1], 3.5);
        // A dual's kerbs are `gap` apart: axes at ±(w + gap)/2.
        let ways = parse("net:dual?gap=4").unwrap();
        assert!((ways[0].pts[0][1] - 4.75).abs() < 1e-12);
        assert!((ways[1].pts[0][1] + 4.75).abs() < 1e-12);
    }

    #[test]
    fn a_hairpin_opens_by_its_angle() {
        let ways = parse("net:hairpin?angle=90&len=200").unwrap();
        let [p, q, r] = ways[0].pts[..] else { panic!() };
        assert_eq!(q, [0.0, 0.0]);
        let dot = p[0] * r[0] + p[1] * r[1];
        assert!(dot.abs() < 1e-9, "not a right angle: {p:?} {r:?}");
    }

    #[test]
    fn bad_specs_are_errors() {
        assert!(parse("net:spaghetti").is_err());
        assert!(parse("net:straight?len=abc").is_err());
        assert!(parse("net:straight?len").is_err());
        assert!(parse("flat").is_err());
        assert!(is_spec("net:cross") && !is_spec("segment.parquet"));
    }
}
