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

use crate::world::{Roads, Terrain, World};

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
pub fn write_glb(world: &World) -> Vec<u8> {
    let mut doc = Doc::default();
    if let Some(t) = &world.terrain {
        doc.terrain(t);
    }
    if let Some(r) = &world.roads {
        if !r.lines.is_empty() {
            doc.roads(r);
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

    fn roads(&mut self, r: &Roads) {
        let mut positions = Vec::new();
        let mut indices: Vec<u32> = Vec::new();
        let mut base = 0u32;
        for line in &r.lines {
            for p in &line.pts {
                positions.extend_from_slice(&to_gltf([p[0] as f32, p[1] as f32, p[2] as f32]));
            }
            for k in 1..line.pts.len() as u32 {
                indices.extend_from_slice(&[base + k - 1, base + k]);
            }
            base += line.pts.len() as u32;
        }
        let position = self.vec3(&positions, true);
        let indices = self.indices(&indices);
        let material = self.material(json!({
            "name": "roads",
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.9, 0.1, 0.1, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0
            }
        }));
        self.layer(
            "roads",
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
            pts: vec![[-600.0, -400.0], [0.0, 0.0], [500.0, 300.0]],
        };
        let draped = drape_line(w.terrain.as_ref().unwrap(), &line);
        w.roads = Some(Roads { lines: vec![draped] });
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
        let glb = write_glb(&w);
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
    fn bytes_are_a_function_of_the_world() {
        let a = write_glb(&hill_world());
        let b = write_glb(&hill_world());
        assert_eq!(a, b);
    }

    #[test]
    fn an_empty_layer_has_no_node() {
        let mut w = terrain::tests::world();
        terrain::run(&mut w, &mut dem("flat"), 100.0, usize::MAX);
        w.roads = Some(Roads::default());
        let (doc, _) = unpack(&write_glb(&w));
        assert_eq!(doc["nodes"].as_array().unwrap().len(), 1);
    }
}
