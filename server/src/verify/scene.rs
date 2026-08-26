//! What the checks read: the emitted archive, decoded back into surfaces.
//!
//! Deliberately the *shipped* artifact rather than the in-memory model. Stage 2
//! already has an instrument (`solve::consistency`) and it reports the solved
//! model consistent; every defect that has cost real time since lives after it,
//! in the disagreement between two meshes that were each derived correctly.
//! Reading the archive is the only vantage point from which that disagreement
//! is visible, and it costs the quantization and encoding for free.

use crate::archive::Archive;
use crate::fb::tile::arpentry::tiles as fbt;
use crate::layers;
use crate::project::Bounds;

use super::mesh::{Scale, SurfaceMesh};

/// A transportation mesh with the two properties that decide what it is.
///
/// `class` is `road_surface` / `road_rim` for the unioned at-grade asphalt
/// (`rail_surface` / `rail_rim` for a rail formation's ballast band — the
/// same machinery in another material) and the road's own class (`motorway`,
/// `residential`, …) for a baked structure; `level` is the reserved ordinal
/// (FORMAT.md §8) — 0 at grade, positive on a deck, negative in a bore.
pub struct RoadMesh {
    pub class: String,
    pub level: i64,
    /// For a structure, the surface band its running top *is* — `road_surface`
    /// or `rail_surface`, as the tiler named it (`profile::profile`). Empty at
    /// grade, where the band draws itself, and empty for a structure whose
    /// class paves nothing.
    ///
    /// It is the only thing in the archive that says a deck and a band are the
    /// same *surface*: the union dissolves road identity, so pairing them
    /// otherwise means re-deriving the modality from the deck's road class and
    /// hoping the two agree.
    pub band: String,
    /// Whether this mesh carries a non-zero `edge_across` anywhere, i.e. the
    /// client fades its outer pixel into whatever is behind it
    /// (`fs_deck` in `client/shaders/terrain.wgsl`).
    ///
    /// The rim carries the fade only where the ground is *not* cut away under
    /// the asphalt — with the hole, the paved boundary and the terrain
    /// boundary are the same edge and MSAA resolves it for free
    /// (`synth::pave_mesh::build_rim`). So this is the difference between a rim
    /// that draws something of its own and one that is simply the outer third
    /// of a metre of its surface, and a check about what the rim *draws* has to
    /// read it rather than the class.
    pub fades: bool,
    pub mesh: SurfaceMesh,
}

impl RoadMesh {
    /// The opaque at-grade surface band — carriageway or rail formation —
    /// what invariant 4 says must lie on the rendered terrain of every zoom.
    /// One population: the invariant does not care what the band is made of.
    pub fn is_pavement(&self) -> bool {
        self.level == 0 && matches!(self.class.as_str(), "road_surface" | "rail_surface")
    }

    /// The rim: the strip between the surface band's inset interior and
    /// its true silhouette. It is surface, so it answers for the band's height
    /// at the very edge — which the interior mesh, ending an inset short of it,
    /// cannot.
    pub fn is_rim(&self) -> bool {
        self.level == 0 && matches!(self.class.as_str(), "road_rim" | "rail_rim")
    }

    /// The wall drawn between the kerb and the ground beside it
    /// (docs/GROUND.md §3). Never pavement and never a deck: it is vertical by
    /// construction, so a steepness check that counted it would read a
    /// deliberate face as a defect.
    pub fn is_apron(&self) -> bool {
        matches!(self.class.as_str(), "road_apron" | "rail_apron")
    }

    /// A bridge deck: rides above the ground on purpose, and owes the feature
    /// it crosses a class-appropriate gap (invariant 3).
    pub fn is_deck(&self) -> bool {
        self.level > 0
    }

    /// A tunnel bore: runs under the surface on purpose.
    pub fn is_bore(&self) -> bool {
        self.level < 0
    }

    /// A deck *fitted* to the finished ground rather than solved with the
    /// network — a footbridge, a path over a stream (`synth::draped`). Its
    /// abutments come from where the ground is, not from a profile with
    /// anchors and a grade ceiling, so it is the population that can seat an
    /// abutment part way down a wall.
    ///
    /// Named classes only. An unrecognised class takes `RoadClass::Other`,
    /// which is also how a rail class looks when it arrives without its
    /// subtype (the archive carries neither), and counting those would put
    /// solved decks into a fitted deck's population.
    pub fn is_fitted_deck(&self) -> bool {
        use crate::priors::{Kind, RoadClass};
        self.level > 0
            && matches!(
                Kind::parse(Some("road"), Some(&self.class), None),
                Kind::Road(
                    RoadClass::Track
                        | RoadClass::Footway
                        | RoadClass::Pedestrian
                        | RoadClass::Path
                        | RoadClass::Steps
                        | RoadClass::Cycleway
                        | RoadClass::Bridleway
                )
            )
    }
}

/// One drawn road centerline, at the heights the client strokes it at
/// (`tile_build::EncoderFeature::z`). This is the only thing in the archive
/// that carries the road's own *direction*, so it is the only thing a
/// longitudinal grade can be measured along: the carriageway mesh knows where
/// the asphalt is but not which way the traffic goes.
pub struct RoadLine {
    pub class: String,
    pub level: i64,
    /// The stroke's stated physical width in metres (FORMAT.md `width_m`), or 0
    /// when the feature carries none. A painted marking states its own painted
    /// width here — 12 cm for a centre or lane line, 15 cm for an edge line —
    /// which is the only thing in the archive that says *which* line it is.
    pub width_m: f64,
    /// One entry per part, each a run of `(x, y, height_m)` in unit plan space.
    pub parts: Vec<Vec<(f64, f64, f64)>>,
}

impl RoadLine {
    /// Whether this is a railway: its class names a gauge or a system.
    ///
    /// The archive carries a feature's `class` but not its `subtype`, so the
    /// modality has to be read back out of the class alone. `unknown` rail is
    /// not counted — it is indistinguishable here from a road class the parser
    /// does not recognise, and `priors` gives it the junior default for the same
    /// reason.
    pub fn is_rail(&self) -> bool {
        crate::priors::class_is_rail(&self.class)
    }
}

/// One flowing watercourse centerline, plan-only. Water lines carry no
/// heights of their own — flowing water is drawn draped, so its vertical
/// profile *is* the drawn terrain along it — which is exactly why the descent
/// check samples the terrain mesh at these points rather than reading a `z`
/// the format does not have.
pub struct WaterLine {
    /// `river`, `stream` or `canal` (the emitted `class` property).
    pub class: String,
    /// One entry per part, each a run of `(x, y)` in unit plan space.
    pub parts: Vec<Vec<(f64, f64)>>,
}

/// One tile, decoded into the surfaces the checks compare.
pub struct TileScene {
    pub z: u8,
    pub x: u32,
    pub y: u32,
    pub bounds: Bounds,
    pub scale: Scale,
    /// The drawn ground. Absent below the zooms that carry a terrain mesh, in
    /// which case the contact checks have nothing to measure against and say
    /// so rather than guessing.
    pub terrain: Option<SurfaceMesh>,
    pub roads: Vec<RoadMesh>,
    /// Road centerlines carrying per-vertex heights. Empty below the zooms that
    /// stamp elevations, in which case a grade check has nothing to read and
    /// says so rather than reporting a flat scene as perfect.
    pub lines: Vec<RoadLine>,
    /// Flowing watercourse centerlines, for the descent check.
    pub waters: Vec<WaterLine>,
    /// Building meshes with their stamped ground relief, for the seat check.
    /// `relief_m` is the tiler's `ground_relief` property (highest minus
    /// lowest stack ground under the footprint, whole metres; absent = 0), so
    /// a check can reconstruct the designed seat band from the mesh's own
    /// foot: `[foot, foot + relief + foundation margin]`.
    pub buildings: Vec<(f64, SurfaceMesh)>,
}

impl TileScene {
    /// Geodetic position of a unit-space plan point.
    pub fn lonlat(&self, px: f64, py: f64) -> (f64, f64) {
        (
            self.bounds.west + px * self.bounds.width(),
            self.bounds.south + py * self.bounds.height(),
        )
    }

    /// Whether a unit-space point lies in the tile proper rather than the
    /// buffer. Buffer geometry is a neighbour's responsibility; counting it
    /// would double-report every defect near a border and, worse, report
    /// clipped fragments as defects in their own right.
    pub fn owns(&self, px: f64, py: f64) -> bool {
        (0.0..=1.0).contains(&px) && (0.0..=1.0).contains(&py)
    }
}

/// A line feature's parts, in unit plan space with heights in metres. Empty
/// unless the feature carries per-vertex `z`: without heights there is no
/// profile to measure, and a line decoded as flat would read as a perfectly
/// level road rather than as an absent measurement.
fn line_parts(g: &fbt::LineGeometry<'_>) -> Vec<Vec<(f64, f64, f64)>> {
    let (gx, gy) = (g.x(), g.y());
    let Some(gz) = g.z() else { return Vec::new() };
    let n = gx.len().min(gy.len()).min(gz.len());
    if n < 2 {
        return Vec::new();
    }
    // A single linestring omits `line_offsets` (tile_build::line_geometry), so
    // the whole vertex run is one part.
    let bounds: Vec<u32> = match g.line_offsets() {
        Some(o) => (0..o.len()).map(|i| o.get(i)).collect(),
        None => vec![0, n as u32],
    };
    let mut parts = Vec::new();
    for w in bounds.windows(2) {
        let (lo, hi) = (w[0] as usize, (w[1] as usize).min(n));
        if hi <= lo + 1 {
            continue; // a part with one vertex spans nothing
        }
        parts.push(
            (lo..hi)
                .map(|i| {
                    (
                        crate::verify::mesh::dequantize(gx.get(i)),
                        crate::verify::mesh::dequantize(gy.get(i)),
                        gz.get(i) as f64 * 0.001,
                    )
                })
                .collect(),
        );
    }
    parts
}

/// A line feature's parts in unit plan space, heights ignored — for the water
/// layer, whose lines are drawn draped and never carry `z`.
fn plan_parts(g: &fbt::LineGeometry<'_>) -> Vec<Vec<(f64, f64)>> {
    let (gx, gy) = (g.x(), g.y());
    let n = gx.len().min(gy.len());
    if n < 2 {
        return Vec::new();
    }
    let bounds: Vec<u32> = match g.line_offsets() {
        Some(o) => (0..o.len()).map(|i| o.get(i)).collect(),
        None => vec![0, n as u32],
    };
    let mut parts = Vec::new();
    for w in bounds.windows(2) {
        let (lo, hi) = (w[0] as usize, (w[1] as usize).min(n));
        if hi <= lo + 1 {
            continue;
        }
        parts.push(
            (lo..hi)
                .map(|i| {
                    (
                        crate::verify::mesh::dequantize(gx.get(i)),
                        crate::verify::mesh::dequantize(gy.get(i)),
                    )
                })
                .collect(),
        );
    }
    parts
}

/// Decodes tiles from an archive, one zoom at a time.
pub struct ArchiveScan<'a> {
    archive: Archive<'a>,
}

impl<'a> ArchiveScan<'a> {
    pub fn open(data: &'a [u8]) -> Result<ArchiveScan<'a>, String> {
        Archive::open(data).map(|archive| ArchiveScan { archive }).map_err(|e| format!("{e:?}"))
    }

    pub fn min_zoom(&self) -> u8 {
        self.archive.min_zoom()
    }

    pub fn max_zoom(&self) -> u8 {
        self.archive.max_zoom()
    }

    pub fn tile_count(&self) -> u64 {
        self.archive.tile_count()
    }

    /// The `(z, x, y)` of every tile at `z`, in archive order.
    pub fn tiles_at(&self, z: u8) -> Vec<(u8, u32, u32, u64)> {
        self.archive
            .entries()
            .filter(|e| e.z == z)
            .map(|e| (e.z, e.x, e.y, e.hilbert_id))
            .collect()
    }

    /// Decodes one tile. Returns `None` when the blob is missing or malformed —
    /// a tile that will not decode is a finding for the caller's tally, not a
    /// reason to abandon the scan.
    pub fn decode(&self, z: u8, x: u32, y: u32, id: u64) -> Option<TileScene> {
        let raw = self.archive.get_by_id(id)?;
        let mut buf = Vec::new();
        brotli::BrotliDecompress(&mut &raw[..], &mut buf).ok()?;
        let tile = fbt::root_as_tile(&buf).ok()?;
        let layers_v = tile.layers()?;
        let keys = tile.keys()?;
        let values = tile.values()?;

        let bounds = Bounds::of_tile(z, x, y);
        let mut scene = TileScene {
            z,
            x,
            y,
            scale: Scale::of(&bounds),
            bounds,
            terrain: None,
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };

        for li in 0..layers_v.len() {
            let l = layers_v.get(li);
            let name = l.name();
            let Some(feats) = l.features() else { continue };
            if name == layers::NAMES[layers::TERRAIN as usize] {
                // The terrain layer is a single mesh feature (tile_build.rs).
                for fi in 0..feats.len() {
                    if let Some(g) = feats.get(fi).geometry_as_mesh_geometry() {
                        scene.terrain = SurfaceMesh::from_geometry(&g);
                    }
                }
            } else if name == layers::NAMES[layers::TRANSPORTATION as usize] {
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    let (mut class, mut level, mut width_m) = (String::new(), 0i64, 0.0f64);
                    let mut band = String::new();
                    if let Some(props) = f.properties() {
                        for pi in 0..props.len() {
                            let p = props.get(pi);
                            let Some(k) = keys.iter().nth(p.key() as usize) else { continue };
                            let v = values.get(p.value() as usize);
                            match k {
                                "class" => class = v.string_value().unwrap_or("").to_string(),
                                "level" => level = v.int_value(),
                                "width_m" => width_m = v.double_value(),
                                "band_class" => {
                                    band = v.string_value().unwrap_or("").to_string()
                                }
                                _ => {}
                            }
                        }
                    }
                    if let Some(g) = f.geometry_as_mesh_geometry() {
                        if let Some(mesh) = SurfaceMesh::from_geometry(&g) {
                            let fades = g
                                .edge_across()
                                .is_some_and(|a| (0..a.len()).any(|i| a.get(i) != 0));
                            scene.roads.push(RoadMesh { class, level, band, fades, mesh });
                        }
                    } else if let Some(g) = f.geometry_as_line_geometry() {
                        let parts = line_parts(&g);
                        if !parts.is_empty() {
                            scene.lines.push(RoadLine { class, level, width_m, parts });
                        }
                    }
                }
            } else if name == layers::NAMES[layers::WATER as usize] {
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    let mut class = String::new();
                    if let Some(props) = f.properties() {
                        for pi in 0..props.len() {
                            let p = props.get(pi);
                            let Some(k) = keys.iter().nth(p.key() as usize) else { continue };
                            if k == "class" {
                                class = values
                                    .get(p.value() as usize)
                                    .string_value()
                                    .unwrap_or("")
                                    .to_string();
                            }
                        }
                    }
                    // Lines only: a wide river's polygon has no along-flow
                    // order to walk, and the still bodies are the flatten's
                    // business, not the descent's.
                    if !matches!(class.as_str(), "river" | "stream" | "canal") {
                        continue;
                    }
                    if let Some(g) = f.geometry_as_line_geometry() {
                        let parts = plan_parts(&g);
                        if !parts.is_empty() {
                            scene.waters.push(WaterLine { class, parts });
                        }
                    }
                }
            } else if name == layers::NAMES[layers::BUILDING as usize] {
                for fi in 0..feats.len() {
                    let f = feats.get(fi);
                    let mut relief = 0.0f64;
                    if let Some(props) = f.properties() {
                        for pi in 0..props.len() {
                            let p = props.get(pi);
                            let Some(k) = keys.iter().nth(p.key() as usize) else { continue };
                            if k == "ground_relief" {
                                relief = values.get(p.value() as usize).int_value() as f64;
                            }
                        }
                    }
                    if let Some(g) = f.geometry_as_mesh_geometry() {
                        if let Some(mesh) = SurfaceMesh::from_geometry(&g) {
                            scene.buildings.push((relief, mesh));
                        }
                    }
                }
            }
        }
        Some(scene)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mesh_is_classified_by_class_and_level() {
        let mk = |class: &str, level: i64| RoadMesh {
            band: String::new(), fades: false,
            class: class.to_string(),
            level,
            mesh: dummy(),
        };
        assert!(mk("road_surface", 0).is_pavement());
        assert!(!mk("road_rim", 0).is_pavement(), "the rim is not the carriageway");
        assert!(!mk("motorway", 1).is_pavement());
        assert!(mk("motorway", 1).is_deck());
        assert!(mk("motorway", -5).is_bore());
        assert!(!mk("road_surface", 0).is_deck() && !mk("road_surface", 0).is_bore());
    }

    #[test]
    fn tile_ownership_excludes_the_buffer() {
        let s = TileScene {
            z: 16,
            x: 1,
            y: 1,
            bounds: Bounds::of_tile(16, 1, 1),
            scale: Scale { mx: 1.0, my: 1.0 },
            terrain: None,
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };
        assert!(s.owns(0.0, 0.0) && s.owns(1.0, 1.0) && s.owns(0.5, 0.5));
        assert!(!s.owns(-0.01, 0.5), "west buffer belongs to the neighbour");
        assert!(!s.owns(0.5, 1.2), "north buffer belongs to the neighbour");
    }

    #[test]
    fn unit_space_maps_back_onto_the_tile_bounds() {
        let b = Bounds::of_tile(16, 34567, 23456);
        let s = TileScene {
            z: 16,
            x: 34567,
            y: 23456,
            scale: Scale::of(&b),
            bounds: b,
            terrain: None,
            roads: Vec::new(),
            lines: Vec::new(),
            waters: Vec::new(),
            buildings: Vec::new(),
        };
        let (lon, lat) = s.lonlat(0.0, 0.0);
        assert!((lon - b.west).abs() < 1e-12 && (lat - b.south).abs() < 1e-12);
        let (lon, lat) = s.lonlat(1.0, 1.0);
        assert!((lon - b.east).abs() < 1e-12 && (lat - b.north).abs() < 1e-12);
    }

    /// One flat triangle; the classification tests never query it.
    fn dummy() -> SurfaceMesh {
        SurfaceMesh::from_parts(
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
            vec![0.0, 0.0, 0.0],
            vec![0, 1, 2],
        )
        .unwrap()
    }
}
