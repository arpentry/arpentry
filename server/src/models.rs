//! Builds the `.arpm` model library (port of `resp_model.c`).
//!
//! Three procedural tree models — oak, pine, birch — each a cylinder trunk plus
//! a sphere or cone crown, split into two material parts (trunk, crown). Vertex
//! coordinates are model-local millimetres centred at (`CX`, `CY`) so they stay
//! within uint16 range; the per-vertex `w` channel carries the part index.

use crate::fb::model::arpentry::tiles as fbm;
use crate::fb::tile::arpentry::tiles::{Color, Part};

const BROTLI_QUALITY: i32 = 4;

const SIDES: usize = 8;
const SPHERE_LAT: usize = 4;
const SPHERE_LON: usize = 8;

// Model-space centre in millimetres (exceeds the largest crown radius so all
// coordinates stay positive).
const CX: f64 = 10000.0;
const CY: f64 = 10000.0;

/// Matte natural look: roughness 200/255 ≈ 0.78, metalness 0 (dielectric).
const ROUGHNESS: u8 = 200;
const METALNESS: u8 = 0;

/// Accumulates a model's vertex (x/y/z/w) and index buffers.
#[derive(Default)]
struct Mesh {
    x: Vec<u16>,
    y: Vec<u16>,
    z: Vec<u16>,
    w: Vec<u16>,
    indices: Vec<u32>,
}

impl Mesh {
    fn push_vertex(&mut self, x: u16, y: u16, z: u16, part: u16) {
        self.x.push(x);
        self.y.push(y);
        self.z.push(z);
        self.w.push(part);
    }

    fn vcount(&self) -> u32 {
        self.x.len() as u32
    }

    /// An 8-sided cylinder from `z_bot` to `z_top`.
    fn cylinder(&mut self, radius: f64, z_bot: u16, z_top: u16, part: u16) {
        let base = self.vcount();
        for i in 0..SIDES {
            let a = 2.0 * std::f64::consts::PI * i as f64 / SIDES as f64;
            self.push_vertex((CX + radius * a.cos()) as u16, (CY + radius * a.sin()) as u16, z_bot, part);
        }
        for i in 0..SIDES {
            let a = 2.0 * std::f64::consts::PI * i as f64 / SIDES as f64;
            self.push_vertex((CX + radius * a.cos()) as u16, (CY + radius * a.sin()) as u16, z_top, part);
        }
        let bot0 = base;
        let top0 = base + SIDES as u32;
        // Side quads (2 tris each).
        for i in 0..SIDES as u32 {
            let n = (i + 1) % SIDES as u32;
            self.indices.extend_from_slice(&[bot0 + i, bot0 + n, top0 + i, top0 + i, bot0 + n, top0 + n]);
        }
        // Bottom cap.
        for i in 1..SIDES as u32 - 1 {
            self.indices.extend_from_slice(&[bot0, bot0 + i + 1, bot0 + i]);
        }
        // Top cap.
        for i in 1..SIDES as u32 - 1 {
            self.indices.extend_from_slice(&[top0, top0 + i, top0 + i + 1]);
        }
    }

    /// An 8-sided cone from `z_base` to apex `z_apex`.
    fn cone(&mut self, radius: f64, z_base: u16, z_apex: u16, part: u16) {
        let base = self.vcount();
        for i in 0..SIDES {
            let a = 2.0 * std::f64::consts::PI * i as f64 / SIDES as f64;
            self.push_vertex((CX + radius * a.cos()) as u16, (CY + radius * a.sin()) as u16, z_base, part);
        }
        let apex = self.vcount();
        self.push_vertex(CX as u16, CY as u16, z_apex, part);
        // Side triangles.
        for i in 0..SIDES as u32 {
            let n = (i + 1) % SIDES as u32;
            self.indices.extend_from_slice(&[apex, base + i, base + n]);
        }
        // Base cap.
        for i in 1..SIDES as u32 - 1 {
            self.indices.extend_from_slice(&[base, base + i + 1, base + i]);
        }
    }

    /// A UV-sphere approximation centred at height `cz`.
    fn sphere(&mut self, radius: f64, cz: u16, part: u16) {
        let base = self.vcount();
        // Top pole.
        self.push_vertex(CX as u16, CY as u16, (cz as i32 + radius as i32) as u16, part);
        // Latitude rings.
        for lat in 1..SPHERE_LAT {
            let phi = std::f64::consts::PI * lat as f64 / SPHERE_LAT as f64;
            let sp = phi.sin();
            let cp = phi.cos();
            for lon in 0..SPHERE_LON {
                let theta = 2.0 * std::f64::consts::PI * lon as f64 / SPHERE_LON as f64;
                let x = (CX + radius * sp * theta.cos()) as i32 as u16;
                let y = (CY + radius * sp * theta.sin()) as i32 as u16;
                let z = (cz as i32 + (radius * cp) as i32) as u16;
                self.push_vertex(x, y, z, part);
            }
        }
        // Bottom pole.
        let bot_pole = self.vcount();
        self.push_vertex(CX as u16, CY as u16, (cz as i32 - radius as i32) as u16, part);

        let top_pole = base;
        // Top cap triangles.
        for i in 0..SPHERE_LON as u32 {
            let n = (i + 1) % SPHERE_LON as u32;
            self.indices.extend_from_slice(&[top_pole, base + 1 + i, base + 1 + n]);
        }
        // Middle quads.
        for lat in 0..SPHERE_LAT as u32 - 2 {
            let row0 = base + 1 + lat * SPHERE_LON as u32;
            let row1 = row0 + SPHERE_LON as u32;
            for i in 0..SPHERE_LON as u32 {
                let n = (i + 1) % SPHERE_LON as u32;
                self.indices.extend_from_slice(&[row0 + i, row1 + i, row0 + n, row0 + n, row1 + i, row1 + n]);
            }
        }
        // Bottom cap triangles.
        let last_row = base + 1 + (SPHERE_LAT as u32 - 2) * SPHERE_LON as u32;
        for i in 0..SPHERE_LON as u32 {
            let n = (i + 1) % SPHERE_LON as u32;
            self.indices.extend_from_slice(&[bot_pole, last_row + n, last_row + i]);
        }
    }
}

/// Emits one model (name + buffers + two material parts) into the builder.
#[allow(clippy::too_many_arguments)]
fn emit_model<'a>(
    fbb: &mut flatbuffers::FlatBufferBuilder<'a>,
    name: &str,
    mesh: &Mesh,
    trunk_index_count: u32,
    trunk_color: Color,
    crown_color: Color,
) -> flatbuffers::WIPOffset<fbm::Model<'a>> {
    let name = fbb.create_string(name);
    let x = fbb.create_vector(&mesh.x);
    let y = fbb.create_vector(&mesh.y);
    let z = fbb.create_vector(&mesh.z);
    let w = fbb.create_vector(&mesh.w);
    let indices = fbb.create_vector(&mesh.indices);

    let ni = mesh.indices.len() as u32;
    let parts = [
        Part::new(0, trunk_index_count, &trunk_color, ROUGHNESS, METALNESS),
        Part::new(trunk_index_count, ni - trunk_index_count, &crown_color, ROUGHNESS, METALNESS),
    ];
    let parts = fbb.create_vector(&parts);

    fbm::Model::create(
        fbb,
        &fbm::ModelArgs {
            name: Some(name),
            x: Some(x),
            y: Some(y),
            z: Some(z),
            w: Some(w),
            indices: Some(indices),
            normals: None,
            parts: Some(parts),
        },
    )
}

/// Builds the uncompressed `.arpm` FlatBuffer (identifier `"arpm"`).
fn encode() -> Vec<u8> {
    let mut fbb = flatbuffers::FlatBufferBuilder::new();
    let mut models = Vec::with_capacity(3);

    // Oak: short trunk + wide sphere crown.
    {
        let mut m = Mesh::default();
        m.cylinder(400.0, 0, 2000, 0);
        let trunk_ii = m.indices.len() as u32;
        m.sphere(6000.0, 8000, 1);
        models.push(emit_model(
            &mut fbb,
            "oak",
            &m,
            trunk_ii,
            Color::new(101, 67, 33, 255),
            Color::new(34, 85, 25, 255),
        ));
    }

    // Pine: medium trunk + tall cone crown.
    {
        let mut m = Mesh::default();
        m.cylinder(300.0, 0, 4000, 0);
        let trunk_ii = m.indices.len() as u32;
        m.cone(3000.0, 4000, 18000, 1);
        models.push(emit_model(
            &mut fbb,
            "pine",
            &m,
            trunk_ii,
            Color::new(101, 67, 33, 255),
            Color::new(20, 70, 20, 255),
        ));
    }

    // Birch: slender trunk + small sphere crown.
    {
        let mut m = Mesh::default();
        m.cylinder(200.0, 0, 5000, 0);
        let trunk_ii = m.indices.len() as u32;
        m.sphere(3000.0, 10000, 1);
        models.push(emit_model(
            &mut fbb,
            "birch",
            &m,
            trunk_ii,
            Color::new(200, 200, 195, 255),
            Color::new(80, 160, 50, 255),
        ));
    }

    let models_vec = fbb.create_vector(&models);
    let lib = fbm::ModelLibrary::create(
        &mut fbb,
        &fbm::ModelLibraryArgs { version: 1, models: Some(models_vec) },
    );
    fbm::finish_model_library_buffer(&mut fbb, lib);
    fbb.finished_data().to_vec()
}

/// Builds the Brotli-compressed `.arpm` model library blob.
pub fn build() -> Vec<u8> {
    compress(&encode())
}

fn compress(data: &[u8]) -> Vec<u8> {
    let mut params = brotli::enc::BrotliEncoderParams::default();
    params.quality = BROTLI_QUALITY;
    let mut out = Vec::new();
    let mut input = data;
    brotli::BrotliCompress(&mut input, &mut out, &params).expect("brotli compress in-memory");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decompress(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut input = data;
        brotli::BrotliDecompress(&mut input, &mut out).unwrap();
        out
    }

    #[test]
    fn library_has_three_two_part_models() {
        let raw = encode();
        assert_eq!(&raw[4..8], b"arpm");
        let lib = fbm::root_as_model_library(&raw).unwrap();
        assert_eq!(lib.version(), 1);
        let models = lib.models().unwrap();
        assert_eq!(models.len(), 3);
        let names: Vec<&str> = (0..models.len()).map(|i| models.get(i).name()).collect();
        assert_eq!(names, ["oak", "pine", "birch"]);
        for i in 0..models.len() {
            let m = models.get(i);
            let parts = m.parts().unwrap();
            assert_eq!(parts.len(), 2);
            // Parts cover the whole index buffer contiguously.
            let ni = m.indices().len() as u32;
            assert_eq!(parts.get(0).first_index(), 0);
            assert_eq!(parts.get(1).first_index(), parts.get(0).index_count());
            assert_eq!(parts.get(1).first_index() + parts.get(1).index_count(), ni);
            // Vertex channels are parallel.
            assert_eq!(m.x().len(), m.w().unwrap().len());
        }
        assert_eq!(decompress(&compress(&raw)), raw);
    }
}
