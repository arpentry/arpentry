//! The layers on disk: a step's output written after it runs, and read back
//! in place of running it.
//!
//! `--dump DIR` writes every layer a run builds, one file per step; `--from
//! STEP --load DIR` reads the layers of every step before `STEP` from `DIR`
//! and runs from there. So a step is debugged without the steps before it:
//! the earthwork is 8.5 s of the loop box, and running it used to cost the
//! 25 s before it as well. A layer on disk is also an input a specimen can
//! be given by hand.
//!
//! Each file carries the bbox it was built for, and is refused by a world
//! over any other: a layer is in the local metres of its own frame, and read
//! into another one it would be wrong everywhere without failing anywhere.
//! It also carries the line its step printed, so a run that loads a step
//! still reports what that step said.

use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::step::{Step, Summary};
use crate::world::World;

/// The first bytes of every layer file, and the format's version: a file
/// written by another layout of the layers is refused, not misread.
const MAGIC: &[u8; 8] = b"ARPWLY01";

/// What a layer file says about itself.
#[derive(Serialize, Deserialize)]
struct Header {
    step: String,
    /// The world's bbox, as `[west, south, east, north]`.
    bbox: [f64; 4],
    /// The line the step printed when it built the layer.
    summary: String,
}

/// The file `step`'s layer is kept in under `dir`.
pub fn path_of(dir: &Path, step: Step) -> PathBuf {
    dir.join(format!("{}.layer", step.name()))
}

fn bbox_of(world: &World) -> [f64; 4] {
    let b = &world.extent.bbox;
    [b.west, b.south, b.east, b.north]
}

fn write<T: Serialize>(path: &Path, header: &Header, layer: &T) -> Result<(), String> {
    let fail = |e: &dyn std::fmt::Display| format!("{}: {e}", path.display());
    let mut out = BufWriter::new(File::create(path).map_err(|e| fail(&e))?);
    out.write_all(MAGIC).map_err(|e| fail(&e))?;
    let head = postcard::to_stdvec(header).map_err(|e| fail(&e))?;
    out.write_all(&(head.len() as u64).to_le_bytes()).map_err(|e| fail(&e))?;
    out.write_all(&head).map_err(|e| fail(&e))?;
    postcard::to_io(layer, &mut out).map_err(|e| fail(&e))?;
    out.flush().map_err(|e| fail(&e))
}

fn read<T: DeserializeOwned>(path: &Path, world: &World, step: Step) -> Result<(String, T), String> {
    let fail = |e: &dyn std::fmt::Display| format!("{}: {e}", path.display());
    let mut bytes = Vec::new();
    File::open(path).and_then(|mut f| f.read_to_end(&mut bytes)).map_err(|e| fail(&e))?;
    if bytes.len() < 16 || &bytes[..8] != MAGIC {
        return Err(fail(&"not a layer file of this version"));
    }
    let n = u64::from_le_bytes(bytes[8..16].try_into().expect("eight bytes")) as usize;
    let head = bytes.get(16..16 + n).ok_or_else(|| fail(&"truncated"))?;
    let header: Header = postcard::from_bytes(head).map_err(|e| fail(&e))?;
    if header.step != step.name() {
        return Err(fail(&format!("holds the {} step's layer, not the {} step's", header.step, step.name())));
    }
    if header.bbox != bbox_of(world) {
        return Err(fail(&format!("was built over the bbox {:?}, not this world's {:?}", header.bbox, bbox_of(world))));
    }
    let layer: T = postcard::from_bytes(&bytes[16 + n..]).map_err(|e| fail(&e))?;
    Ok((header.summary, layer))
}

/// Every step and the field of [`World`] its layer lands in, once: what the
/// save and the load both walk, so the two cannot disagree about a step.
macro_rules! layers {
    ($m:ident) => {
        $m! {
            Terrain => terrain,
            Drape => roads,
            Reference => reference,
            Profile => profile,
            Crossing => crossing,
            Partition => partition,
            Facade => facade,
            Ribbon => ribbons,
            Surface => surface,
            Kerb => kerb,
            Legs => legs,
            Room => room,
            Sheet => sheets,
            Arrangement => arrangement,
            Mesh => mesh,
            Lift => lift,
            Earthwork => earthwork,
            Bench => bench,
            Structure => structure,
            Building => buildings
        }
    };
}

/// Writes the layer `step` built into `dir`, with the line it printed.
///
/// Panics if the step has not run: a dump is taken right after it.
pub fn save(world: &World, step: Step, summary: &Summary, dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let path = path_of(dir, step);
    let header = Header { step: step.name().into(), bbox: bbox_of(world), summary: summary.to_string() };
    macro_rules! save {
        ($($s:ident => $f:ident),*) => {
            match step {
                $(Step::$s => write(&path, &header, world.$f.as_ref().expect("a step's layer is saved after it runs")),)*
            }
        };
    }
    layers!(save)
}

/// Reads `step`'s layer from `dir` into `world`, and returns the line the
/// step printed when it built it.
pub fn load(world: &mut World, step: Step, dir: &Path) -> Result<String, String> {
    let path = path_of(dir, step);
    macro_rules! load {
        ($($s:ident => $f:ident),*) => {
            match step {
                $(Step::$s => {
                    let (summary, layer) = read(&path, world, step)?;
                    world.$f = Some(layer);
                    summary
                })*
            }
        };
    }
    Ok(layers!(load))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::tests::{bbox, built, upto};
    use crate::pipeline::{self, Sources};
    use crate::terrain::tests::dem;

    /// **A layer read back is the layer that was written**, for every step:
    /// each step's check reads the same of it, and a step run over loaded
    /// layers reports what it reported over built ones.
    #[test]
    fn a_world_read_back_is_the_world_that_was_built() {
        let (ground, net, houses) = ("ramp?grade=0.15&bearing=45", "net:tee?d=8&hook=5", Some("house:beside?d=12"));
        let (w, ran) = built(ground, net, houses, 5.0, &upto(Step::Building));
        let dir = std::env::temp_dir().join(format!("arpentry-dump-test-{}", std::process::id()));
        for &step in &Step::ALL {
            save(&w, step, &ran.of(step), &dir).expect("a layer writes");
        }
        let mut back = World::new(bbox());
        for &step in &Step::ALL {
            let line = load(&mut back, step, &dir).expect("a layer reads");
            assert_eq!(line, ran.of(step).to_string(), "the {} step's line", step.name());
            assert_eq!(
                pipeline::check(&back, step).to_string(),
                pipeline::check(&w, step).to_string(),
                "the {} step's check",
                step.name()
            );
        }
        // And the last step run again over the layers read back.
        let mut again = World::new(bbox());
        let mut d = dem(ground);
        let mut src = Sources {
            dem: &mut d,
            segments: std::path::Path::new(net),
            buildings: houses.map(std::path::Path::new),
            spacing: 5.0,
            max_vertices: usize::MAX,
        };
        let range = pipeline::Range { from: Step::Structure, until: Step::Structure, dump: None, load: Some(&dir) };
        let mut line = String::new();
        pipeline::run(&mut again, &range, &mut src, &mut |step, r| {
            if let (Step::Structure, pipeline::Ran::Built(s)) = (step, r) {
                line = s.to_string();
            }
        })
        .expect("a run from loaded layers");
        assert_eq!(line, ran.of(Step::Structure).to_string());
        std::fs::remove_dir_all(&dir).ok();
    }

    /// A layer is refused by a world over another bbox, not misread into it.
    #[test]
    fn a_layer_is_refused_by_another_world() {
        let (w, ran) = built("flat", "net:straight", None, 50.0, &upto(Step::Terrain));
        let dir = std::env::temp_dir().join(format!("arpentry-dump-refuse-{}", std::process::id()));
        save(&w, Step::Terrain, &ran.of(Step::Terrain), &dir).expect("a layer writes");
        let mut b = bbox();
        b.north += 0.001;
        let err = load(&mut World::new(b), Step::Terrain, &dir).expect_err("another bbox");
        assert!(err.contains("was built over the bbox"), "{err}");
        let err = load(&mut World::new(bbox()), Step::Drape, &dir).expect_err("no drape layer");
        assert!(err.contains("drape.layer"), "{err}");
        std::fs::remove_dir_all(&dir).ok();
    }
}
