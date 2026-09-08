//! The world as a binary glTF, for Blender and anything else that opens one.
//!
//! Hand-rolled, because the format's binary container is three chunks and the
//! document is a page of JSON, and a dependency would only hide which bytes
//! are written. The output is a function of the world alone — no timestamps,
//! no hash maps, sorted keys — so two runs over the same inputs are
//! byte-identical and `cmp` is a regression check. The tiler's archive never
//! had that property.
//!
//! One node per layer, named after its step. Axes: glTF is Y-up, so local
//! `(east, north, up)` becomes `(east, up, −north)`, a rotation (determinant
//! +1) rather than a reflection; Blender's importer turns it back into Z-up
//! with +Y north.

use serde_json::{json, Value};

use crate::drape::drape;
use crate::poly::Shapes;
use crate::world::{Profiles, Ribbons, Roads, Surface, Terrain, Tri, World};

const MAGIC: u32 = 0x4654_6C67; // "glTF"
const CHUNK_JSON: u32 = 0x4E4F_534A;
const CHUNK_BIN: u32 = 0x004E_4942;

const ARRAY_BUFFER: u32 = 34962;
const ELEMENT_ARRAY_BUFFER: u32 = 34963;
const FLOAT: u32 = 5126;
const UNSIGNED_INT: u32 = 5125;
const TRIANGLES: u32 = 4;
const LINES: u32 = 1;

/// The terrain's vertex colour, a muted green that reads as ground in a
/// viewer's solid shading.
const TERRAIN_COLOR: [f32; 3] = [0.55, 0.65, 0.45];

/// Serialises the world's layers. A layer that has not been built, or is
/// empty, has no node.
///
/// With `outlines`, the eight construction layers — the draped
/// centrelines, the solved profiles and the six contour sets — are written
/// too, as glTF `LINES`. They are **off by default**, because a viewer is
/// not obliged to draw line topology and Apple's (Preview, Quick Look, and
/// anything else on that pipeline) does not: it reads the line index
/// buffer as triangles instead and invents a long straight shard between
/// every pair of vertices that happen to be neighbours in the buffer, in
/// the layer's own colour, right across the model. The file is valid
/// either way — Blender draws it correctly — but the default has to be the
/// one that opens anywhere, and the 2D question these layers answer
/// belongs to the plan view ([`crate::svg`]) anyway.
pub fn write_glb(world: &World, outlines: bool) -> Vec<u8> {
    let mut doc = Doc::default();
    // The engineered ground once the bench has cut the room out of it;
    // the raw lattice before that. One ground either way.
    if let Some(b) = &world.bench {
        doc.triangles("ground", &b.ground, [0.52, 0.56, 0.44]);
    } else if let Some(t) = &world.terrain {
        doc.terrain(t);
    }
    if let Some(r) = &world.roads {
        if outlines && !r.lines.is_empty() {
            doc.roads(r);
        }
    }
    if let Some(p) = &world.profile {
        if outlines && !p.profiles.is_empty() {
            doc.profile(p);
        }
    }
    // The room at its solved height once the bench has run; on the raw
    // ground before it. One pair of nodes either way, so a viewer opens
    // the same file whichever step the run stopped after.
    if let Some((c, p)) = world
        .bench
        .as_ref()
        .map(|b| (&b.carriageway, &b.pavement))
        .or_else(|| world.mesh.as_ref().map(|m| (&m.carriageway, &m.pavement)))
    {
        doc.triangles("carriageway", c, [0.30, 0.30, 0.33]);
        doc.triangles("pavement", p, [0.80, 0.66, 0.46]);
    }
    if let Some(s) = &world.structure {
        doc.triangles("roadway", &s.roadway, [0.30, 0.30, 0.33]);
        doc.triangles("deck", &s.deck, [0.62, 0.60, 0.56]);
        doc.triangles("bore", &s.bore, [0.35, 0.33, 0.30]);
    }
    if let (Some(r), Some(t)) = (&world.ribbons, &world.terrain) {
        if outlines && !r.ribbons.is_empty() {
            doc.ribbons(r, t);
        }
    }
    if let (Some(s), Some(t)) = (&world.surface, &world.terrain) {
        if outlines && (!s.carriageway.is_empty() || !s.walk.is_empty()) {
            doc.surface(s, t);
        }
    }
    if let (Some(k), Some(t)) = (&world.kerb, &world.terrain) {
        if outlines && !k.pavement.is_empty() {
            doc.loops("kerb", &k.pavement, t, [0.9, 0.6, 0.3]);
        }
    }
    if let (Some(f), Some(t)) = (&world.fillet, &world.terrain) {
        let shapes: Shapes = f.carriageway.iter().chain(f.pavement.iter()).cloned().collect();
        if outlines && !shapes.is_empty() {
            doc.loops("fillet", &shapes, t, [0.3, 0.3, 0.35]);
        }
    }
    if let (Some(r), Some(t)) = (&world.room, &world.terrain) {
        if outlines && !r.pavement.is_empty() {
            doc.loops("room", &r.pavement, t, [0.85, 0.55, 0.25]);
        }
    }
    if let (Some(f), Some(t)) = (&world.facade, &world.terrain) {
        if outlines && !f.footprints.is_empty() {
            doc.loops("facade", &f.footprints, t, [0.55, 0.45, 0.4]);
        }
    }
    doc.pack()
}

/// The glTF document under construction: the JSON arrays and the one buffer
/// they index into.
#[derive(Default)]
struct Doc {
    bin: Vec<u8>,
    views: Vec<Value>,
    accessors: Vec<Value>,
    materials: Vec<Value>,
    meshes: Vec<Value>,
    nodes: Vec<Value>,
}

impl Doc {
    fn terrain(&mut self, t: &Terrain) {
        let n = t.grid.vertex_count();
        let mut positions = Vec::with_capacity(n * 3);
        let mut normals = Vec::with_capacity(n * 3);
        let mut colors = Vec::with_capacity(n * 3);
        for i in 0..n {
            let [x, y, z] = t.position(i);
            positions.extend_from_slice(&to_gltf([x as f32, y as f32, z as f32]));
            let nrm = t.normals[i];
            normals.extend_from_slice(&to_gltf(nrm));
            colors.extend_from_slice(&TERRAIN_COLOR);
        }
        let position = self.vec3(&positions, true);
        let normal = self.vec3(&normals, false);
        let color = self.vec3(&colors, false);
        let indices = self.indices(&t.indices);
        let material = self.material(json!({
            "name": "terrain",
            "doubleSided": true,
            "pbrMetallicRoughness": {
                "baseColorFactor": [1.0, 1.0, 1.0, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0
            }
        }));
        self.layer(
            "terrain",
            json!({
                "attributes": { "POSITION": position, "NORMAL": normal, "COLOR_0": color },
                "indices": indices,
                "mode": TRIANGLES,
                "material": material
            }),
        );
    }

    /// The ribbons' contours as closed line loops on the ground: an outline
    /// a viewer can read until the mesh step fills it.
    fn ribbons(&mut self, r: &Ribbons, t: &Terrain) {
        let shapes: Shapes = r.ribbons.iter().flat_map(|x| x.shape.iter().cloned()).collect();
        self.loops("ribbon", &shapes, t, [0.9, 0.55, 0.15]);
    }

    /// The surface's contours, both families in one node.
    fn surface(&mut self, s: &Surface, t: &Terrain) {
        let shapes: Shapes = s.carriageway.iter().chain(s.walk.iter()).cloned().collect();
        self.loops("surface", &shapes, t, [0.2, 0.2, 0.25]);
    }

    /// A layer of closed line loops, one per contour, lying on the terrain.
    ///
    /// The ring is **draped**, not merely sampled at its own vertices: a
    /// contour edge is as long as the straight road that made it, and the
    /// box has 267 m of it. A single segment between two [`crate::terrain::height_at`]
    /// samples that far apart goes in one side of the flank and out the
    /// other, and the file read as a model with straight lines shot
    /// through it — 970 such edges in the ribbon layer alone. [`drape`]
    /// already answers this for the way centrelines, by cutting at every
    /// lattice crossing, so it answers it here.
    fn loops(&mut self, name: &str, shapes: &Shapes, t: &Terrain, color: [f32; 3]) {
        let mut positions = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut base = 0u32;
        for ring in shapes.iter().flatten() {
            let Some(&first) = ring.first() else {
                continue;
            };
            let mut closed = ring.clone();
            closed.push(first);
            let pts = drape(t, &closed);
            for p in &pts {
                positions.extend_from_slice(&to_gltf([p[0] as f32, p[1] as f32, p[2] as f32]));
            }
            let n = pts.len() as u32;
            for k in 1..n {
                indices.extend_from_slice(&[base + k - 1, base + k]);
            }
            base += n;
        }
        let position = self.vec3(&positions, true);
        let indices = self.indices(&indices);
        let material = self.material(json!({
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": [color[0], color[1], color[2], 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0
            }
        }));
        self.layer(
            name,
            json!({
                "attributes": { "POSITION": position },
                "indices": indices,
                "mode": LINES,
                "material": material
            }),
        );
    }

    /// The draped centrelines, as lines on the terrain.
    fn roads(&mut self, r: &Roads) {
        let lines: Vec<&[[f64; 3]]> = r.lines.iter().map(|l| l.pts.as_slice()).collect();
        self.polylines("roads", &lines, [0.9, 0.1, 0.1]);
    }

    /// The solved profiles, as lines at their heights: on the ground where
    /// a piece is at grade, in the air across a valley, under a hill.
    fn profile(&mut self, p: &Profiles) {
        let owned: Vec<Vec<[f64; 3]>> = p.profiles.iter().map(|p| p.line()).collect();
        let lines: Vec<&[[f64; 3]]> = owned.iter().map(|l| l.as_slice()).collect();
        self.polylines("profile", &lines, [0.15, 0.35, 0.9]);
    }

    /// A layer of triangles with one colour; no normals, so a viewer shades
    /// each face flat, which is what a mesh of planar pieces is.
    fn triangles(&mut self, name: &str, tri: &Tri, color: [f32; 3]) {
        if tri.indices.is_empty() {
            return;
        }
        let mut positions = Vec::with_capacity(tri.positions.len() * 3);
        for p in &tri.positions {
            positions.extend_from_slice(&to_gltf([p[0] as f32, p[1] as f32, p[2] as f32]));
        }
        let position = self.vec3(&positions, true);
        let indices = self.indices(&tri.indices);
        let material = self.material(json!({
            "name": name,
            "doubleSided": true,
            "pbrMetallicRoughness": {
                "baseColorFactor": [color[0], color[1], color[2], 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0
            }
        }));
        self.layer(
            name,
            json!({
                "attributes": { "POSITION": position },
                "indices": indices,
                "mode": TRIANGLES,
                "material": material
            }),
        );
    }

    /// A layer of open polylines.
    fn polylines(&mut self, name: &str, lines: &[&[[f64; 3]]], color: [f32; 3]) {
        let mut positions = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut base = 0u32;
        for line in lines {
            for p in line.iter() {
                positions.extend_from_slice(&to_gltf([p[0] as f32, p[1] as f32, p[2] as f32]));
            }
            for k in 1..line.len() as u32 {
                indices.extend_from_slice(&[base + k - 1, base + k]);
            }
            base += line.len() as u32;
        }
        let position = self.vec3(&positions, true);
        let indices = self.indices(&indices);
        let material = self.material(json!({
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": [color[0], color[1], color[2], 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0
            }
        }));
        self.layer(
            name,
            json!({
                "attributes": { "POSITION": position },
                "indices": indices,
                "mode": LINES,
                "material": material
            }),
        );
    }

    /// One mesh with one primitive, and the node that places it.
    fn layer(&mut self, name: &str, primitive: Value) {
        self.meshes.push(json!({ "name": name, "primitives": [primitive] }));
        self.nodes.push(json!({ "name": name, "mesh": self.meshes.len() - 1 }));
    }

    fn material(&mut self, m: Value) -> usize {
        self.materials.push(m);
        self.materials.len() - 1
    }

    /// A VEC3 float accessor over `data` (3 floats per element), with bounds
    /// when `bounded` (required on POSITION).
    fn vec3(&mut self, data: &[f32], bounded: bool) -> usize {
        let mut bytes = Vec::with_capacity(data.len() * 4);
        for v in data {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        let view = self.view(&bytes, ARRAY_BUFFER);
        let mut accessor = json!({
            "bufferView": view,
            "componentType": FLOAT,
            "count": data.len() / 3,
            "type": "VEC3"
        });
        if bounded {
            let mut min = [f32::INFINITY; 3];
            let mut max = [f32::NEG_INFINITY; 3];
            for p in data.chunks_exact(3) {
                for k in 0..3 {
                    min[k] = min[k].min(p[k]);
                    max[k] = max[k].max(p[k]);
                }
            }
            accessor["min"] = json!(min);
            accessor["max"] = json!(max);
        }
        self.accessors.push(accessor);
        self.accessors.len() - 1
    }

    fn indices(&mut self, data: &[u32]) -> usize {
        let mut bytes = Vec::with_capacity(data.len() * 4);
        for v in data {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        let view = self.view(&bytes, ELEMENT_ARRAY_BUFFER);
        self.accessors.push(json!({
            "bufferView": view,
            "componentType": UNSIGNED_INT,
            "count": data.len(),
            "type": "SCALAR"
        }));
        self.accessors.len() - 1
    }

    /// Appends `bytes` to the buffer, 4-byte aligned, as one bufferView.
    fn view(&mut self, bytes: &[u8], target: u32) -> usize {
        while self.bin.len() % 4 != 0 {
            self.bin.push(0);
        }
        let offset = self.bin.len();
        self.bin.extend_from_slice(bytes);
        self.views.push(json!({
            "buffer": 0,
            "byteOffset": offset,
            "byteLength": bytes.len(),
            "target": target
        }));
        self.views.len() - 1
    }

    /// The three chunks: header, JSON (space-padded), BIN (zero-padded).
    fn pack(mut self) -> Vec<u8> {
        while self.bin.len() % 4 != 0 {
            self.bin.push(0);
        }
        let document = json!({
            "asset": { "version": "2.0", "generator": "arpentry-world" },
            "scene": 0,
            "scenes": [{ "nodes": (0..self.nodes.len()).collect::<Vec<_>>() }],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "accessors": self.accessors,
            "bufferViews": self.views,
            "buffers": [{ "byteLength": self.bin.len() }]
        });
        let mut json_bytes = serde_json::to_vec(&document).expect("a JSON document");
        while json_bytes.len() % 4 != 0 {
            json_bytes.push(b' ');
        }
        let total = 12 + 8 + json_bytes.len() + 8 + self.bin.len();
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(&MAGIC.to_le_bytes());
        out.extend_from_slice(&2u32.to_le_bytes());
        out.extend_from_slice(&(total as u32).to_le_bytes());
        out.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
        out.extend_from_slice(&CHUNK_JSON.to_le_bytes());
        out.extend_from_slice(&json_bytes);
        out.extend_from_slice(&(self.bin.len() as u32).to_le_bytes());
        out.extend_from_slice(&CHUNK_BIN.to_le_bytes());
        out.extend_from_slice(&self.bin);
        out
    }
}

/// Local `(east, north, up)` to glTF `(x, y, z)` = `(east, up, −north)`.
fn to_gltf(v: [f32; 3]) -> [f32; 3] {
    [v[0], v[2], -v[1]]
}

#[cfg(test)]
mod tests {
    use crate::drape::drape_line;
    use crate::terrain::{self, tests::dem};
    use crate::world::{Polyline2, Roads};

    use super::*;

    fn hill_world() -> World {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("hill?amp=60&radius=400"), 25.0, usize::MAX);
        let line = Polyline2 {
            id: "r".into(),
            class: "residential".into(),
            subclass: String::new(),
            width_m: 5.5,
            kind: crate::world::Kind::Ground,
            pts: vec![[-600.0, -400.0], [0.0, 0.0], [500.0, 300.0]],
        };
        let draped = drape_line(w.terrain.as_ref().unwrap(), &line);
        w.roads = Some(Roads { plan: vec![line], spans: Vec::new(), lines: vec![draped] });
        w
    }

    /// The JSON document and the BIN chunk of a GLB, after checking its
    /// container.
    fn unpack(glb: &[u8]) -> (Value, &[u8]) {
        let u32_at = |i: usize| u32::from_le_bytes(glb[i..i + 4].try_into().unwrap());
        assert_eq!(u32_at(0), MAGIC);
        assert_eq!(u32_at(4), 2);
        assert_eq!(u32_at(8) as usize, glb.len());
        let json_len = u32_at(12) as usize;
        assert_eq!(u32_at(16), CHUNK_JSON);
        assert_eq!(json_len % 4, 0);
        let json_bytes = &glb[20..20 + json_len];
        let bin_len = u32_at(20 + json_len) as usize;
        assert_eq!(u32_at(24 + json_len), CHUNK_BIN);
        assert_eq!(bin_len % 4, 0);
        let bin = &glb[28 + json_len..];
        assert_eq!(bin.len(), bin_len);
        (serde_json::from_slice(json_bytes).expect("the JSON chunk parses"), bin)
    }

    #[test]
    fn container_and_document_agree() {
        let w = hill_world();
        let glb = write_glb(&w, true);
        let (doc, bin) = unpack(&glb);
        assert_eq!(doc["buffers"][0]["byteLength"].as_u64().unwrap() as usize, bin.len());
        let views = doc["bufferViews"].as_array().unwrap();
        for v in views {
            let off = v["byteOffset"].as_u64().unwrap() as usize;
            let len = v["byteLength"].as_u64().unwrap() as usize;
            assert!(off + len <= bin.len());
            assert_eq!(off % 4, 0);
        }
        let names: Vec<&str> = doc["nodes"].as_array().unwrap().iter().map(|n| n["name"].as_str().unwrap()).collect();
        assert_eq!(names, ["terrain", "roads"]);

        let t = w.terrain.as_ref().unwrap();
        let acc = doc["accessors"].as_array().unwrap();
        let prim = &doc["meshes"][0]["primitives"][0];
        let pos = prim["attributes"]["POSITION"].as_u64().unwrap() as usize;
        assert_eq!(acc[pos]["count"].as_u64().unwrap() as usize, t.grid.vertex_count());
        let idx = prim["indices"].as_u64().unwrap() as usize;
        assert_eq!(acc[idx]["count"].as_u64().unwrap() as usize, t.indices.len());
        // Y-up: the position bounds' second component is the height range.
        let min_y = acc[pos]["min"][1].as_f64().unwrap();
        let max_y = acc[pos]["max"][1].as_f64().unwrap();
        assert!((min_y - t.zmin).abs() < 1e-3 && (max_y - t.zmax).abs() < 1e-3);

        let roads = w.roads.as_ref().unwrap();
        let prim = &doc["meshes"][1]["primitives"][0];
        assert_eq!(prim["mode"], LINES);
        let idx = prim["indices"].as_u64().unwrap() as usize;
        assert_eq!(acc[idx]["count"].as_u64().unwrap() as usize, 2 * (roads.lines[0].pts.len() - 1));
    }

    #[test]
    fn the_outlines_are_the_only_lines_and_they_are_off_by_default() {
        // Every layer a viewer is not obliged to draw is behind the flag,
        // and what is left is triangles only: the file opens anywhere.
        let w = hill_world();
        let (doc, _) = unpack(&write_glb(&w, false));
        let names: Vec<&str> = doc["nodes"].as_array().unwrap().iter().map(|n| n["name"].as_str().unwrap()).collect();
        assert_eq!(names, ["terrain"]);
        for mesh in doc["meshes"].as_array().unwrap() {
            assert_eq!(mesh["primitives"][0]["mode"], TRIANGLES, "{mesh}");
        }
        let (doc, _) = unpack(&write_glb(&w, true));
        let lines = doc["meshes"].as_array().unwrap().iter().filter(|m| m["primitives"][0]["mode"] == LINES).count();
        assert_eq!(lines, 1, "the draped centrelines come back with the flag");
    }

    #[test]
    fn bytes_are_a_function_of_the_world() {
        let a = write_glb(&hill_world(), true);
        let b = write_glb(&hill_world(), true);
        assert_eq!(a, b);
    }

    #[test]
    fn an_empty_layer_has_no_node() {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 100.0, usize::MAX);
        w.roads = Some(Roads::default());
        let (doc, _) = unpack(&write_glb(&w, true));
        assert_eq!(doc["nodes"].as_array().unwrap().len(), 1);
    }
}
