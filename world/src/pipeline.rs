//! The pipeline: the order the steps run in, and the wiring between them.
//!
//! Every step is a function of the layers it reads
//! (`fn run(inputs…) -> (Layer, Summary)`), and this is the one place that
//! knows which layer is which. That split is the point: a step cannot read
//! a layer it did not ask for, cannot read one that has not been built, and
//! cannot quietly start depending on a neighbour — its signature is the
//! whole of its interface, and changing it is a diff here.
//!
//! Three things used to live in the steps and live here now.
//!
//! **"The drape step runs first."** Fifteen steps each opened with an
//! `expect` naming a predecessor: a runtime assertion that the caller had
//! done its job, repeated once per dependency, checked on every run and
//! true by construction every time. [`apply`] unwraps each layer exactly
//! once, at the one call site that can know the order, so the assertion is
//! made where it is decided rather than where it is relied on.
//!
//! **Which surface is the latest.** The paved surface is re-cut four times
//! — the surface step lays it, the kerb fills its strips, the fillet rounds
//! its corners, the room paves its edges — and the steps downstream used to
//! ask the world for "the latest", a resolver on `World` that walked those
//! four layers and returned whichever had been filled. Then it was a literal
//! here, assembled field by field out of two of them. Now there is nothing
//! to resolve or assemble: all four steps hand back a whole
//! [`crate::world::Surface`] — what they changed, and what they passed
//! through — so the latest paving is the last layer that laid any, and
//! `&room(world).surface` is a lookup like every other.
//!
//! **Writing into a layer somebody else built.** Three steps used to take
//! `&mut` and rewrite their predecessors: the reference promoted the
//! terrain's priors onto the ways, the partition rewrote every span table
//! and every profile with it, and between them the way's span table was a
//! field three steps wrote to and no signature admitted. They return those
//! tables now, and the two `match` arms below install them — which is the
//! same rule as everything else here, that a step is a function and the
//! wiring is this file's.
//!
//! The renderers are the exception, and they never used the resolver: they
//! walk the layers themselves and draw the last one filled, because what has
//! been built is genuinely not known until the run stops.

use std::path::Path;

use arpentry_server::dem::Dem;

use crate::step::{Step, Summary};
use crate::world::World;
use crate::{
    arrangement, bench, building, crossing, drape, facade, fillet, kerb, legs, mesh, partition, profile, reference, ribbon,
    room, sheet, structure, surface, terrain,
};

/// The three sources a run reads, and the two knobs the terrain takes.
///
/// Sixteen of the nineteen steps read nothing but the layers before them;
/// only the terrain, the drape and the facade reach outside, so only they
/// take anything from here. The two networks come in as paths because that
/// is what the CLI has, and each of those steps decides for itself whether
/// its path is a file or a synthetic spec — which is why no reader appears
/// in this module.
pub struct Sources<'a> {
    pub dem: &'a mut Dem,
    pub segments: &'a Path,
    /// `None` is no building input at all: open ground everywhere.
    pub buildings: Option<&'a Path>,
    /// Terrain lattice spacing in metres, and the cap that may coarsen it.
    pub spacing: f64,
    pub max_vertices: usize,
}

/// Runs `step` against `world`, reading the layers it needs and storing the
/// one it makes.
///
/// Panics if a layer the step reads has not been built — which is to say if
/// the steps were not run in [`Step::ALL`]'s order. That is the one
/// assertion this module owns on everybody's behalf.
pub fn apply(world: &mut World, step: Step, src: &mut Sources) -> Result<Summary, String> {
    let extent = world.extent;
    Ok(match step {
        Step::Terrain => {
            let (terrain, summary) = terrain::run(&extent, src.dem, src.spacing, src.max_vertices);
            world.terrain = Some(terrain);
            summary
        }
        Step::Drape => {
            let (roads, summary) = drape::run(&extent, terrain(world), src.segments)
                .map_err(|e| format!("{}: {e}", src.segments.display()))?;
            world.roads = Some(roads);
            summary
        }
        Step::Reference => {
            let (reference, spans, summary) = reference::run(terrain(world), roads(world));
            // The terrain's own bridge and tunnel priors, installed. The step
            // derived them and the profile has to chord across them, so they
            // belong on the ways before it runs — and the install is here,
            // because writing into a layer another step built is wiring and
            // wiring lives in this file.
            let ways = &mut world.roads.as_mut().expect("the drape step runs first").ways;
            for (way, spans) in ways.iter_mut().zip(spans) {
                way.spans = spans;
            }
            world.reference = Some(reference);
            summary
        }
        Step::Profile => {
            let (profiles, summary) = profile::run(roads(world), reference(world));
            world.profile = Some(profiles);
            summary
        }
        Step::Crossing => {
            let (crossings, profiles, summary) =
                crossing::run(roads(world), reference(world), profiles(world));
            world.crossing = Some(crossings);
            world.profile = Some(profiles);
            summary
        }
        Step::Partition => {
            // The only step that reads its predecessor's layer *optionally*:
            // a partition with no profile cuts what the source annotated,
            // which is what a flat specimen wants and what the world had
            // before the heights were solved.
            let (cut, summary) = partition::run(roads(world), world.profile.as_ref());
            // The cut, installed. The pieces and the derived span tables are
            // the road network as it now stands; the profiles are rewritten
            // with the same tables, so nothing downstream can read one span
            // truth from the ways and a different one from the heights.
            let roads = world.roads.as_mut().expect("the drape step runs first");
            for (way, spans) in roads.ways.iter_mut().zip(cut.ways) {
                way.spans = spans;
            }
            roads.plan = cut.plan;
            roads.spans = cut.spans;
            if let Some(profiles) = cut.profiles {
                world.profile = Some(profiles);
            }
            summary
        }
        Step::Facade => {
            let (facade, summary) = facade::run(&extent, roads(world), src.buildings)
                .map_err(|e| format!("{}: {e}", src.buildings.unwrap_or(Path::new("")).display()))?;
            world.facade = Some(facade);
            summary
        }
        Step::Ribbon => {
            let (ribbons, summary) = ribbon::run(roads(world));
            world.ribbons = Some(ribbons);
            summary
        }
        Step::Surface => {
            let (surface, summary) = surface::run(roads(world), ribbons(world), facade(world));
            world.surface = Some(surface);
            summary
        }
        Step::Kerb => {
            let (k, summary) = kerb::run(roads(world), surface(world), facade(world));
            world.kerb = Some(k);
            summary
        }
        Step::Fillet => {
            let (f, summary) =
                fillet::run(roads(world), surface(world), kerb(world), facade(world));
            world.fillet = Some(f);
            summary
        }
        Step::Legs => {
            let (l, summary) =
                legs::run(roads(world), surface(world), kerb(world), facade(world), &fillet(world).surface);
            world.legs = Some(l);
            summary
        }
        Step::Room => {
            // The footbridges: no surface step paves a walk span, and the
            // room has to know one is there to keep the stub that climbs it.
            let walk_spans: crate::poly::Shapes = surface::spans_grouped(roads(world))
                .into_iter()
                .filter(|(f, ..)| *f == crate::width::Family::Walk)
                .flat_map(|(.., s)| s)
                .collect();
            let (r, summary) =
                room::run(&fillet(world).surface, &walk_spans, &kerb(world).attached, facade(world));
            world.room = Some(r);
            summary
        }
        Step::Sheet => {
            let (s, summary) = sheet::run(roads(world), profiles(world), &room(world).surface);
            world.sheets = Some(s);
            summary
        }
        Step::Arrangement => {
            let (a, summary) = arrangement::run(
                &extent,
                &roads(world).spans,
                profiles(world),
                &room(world).surface,
                sheets(world),
            );
            world.arrangement = Some(a);
            summary
        }
        Step::Mesh => {
            let (m, summary) = mesh::run(terrain(world), arrangement(world));
            world.mesh = Some(m);
            summary
        }
        Step::Bench => {
            // The span pieces too: where a ground piece ends against a
            // tunnel is a mouth, and the bench must leave it open.
            let (b, summary) = bench::run(
                terrain(world),
                profiles(world),
                mesh(world),
                sheets(world),
                &roads(world).spans,
                arrangement(world),
            );
            world.bench = Some(b);
            summary
        }
        Step::Structure => {
            let (s, summary) = structure::run(
                terrain(world),
                roads(world),
                profiles(world),
                sheets(world),
                bench(world),
            );
            world.structure = Some(s);
            summary
        }
        Step::Building => {
            let (b, summary) = building::run(terrain(world), facade(world));
            world.buildings = Some(b);
            summary
        }
    })
}

/// Runs every step up to and including `until`, calling `report` with each
/// one's name and summary as it finishes.
pub fn upto(
    world: &mut World,
    until: Step,
    src: &mut Sources,
    report: &mut dyn FnMut(Step, &Summary),
) -> Result<(), String> {
    for step in Step::ALL {
        let summary = apply(world, step, src)?;
        report(step, &summary);
        if step == until {
            break;
        }
    }
    Ok(())
}

macro_rules! layer {
    ($name:ident, $field:ident, $ty:ty, $built_by:literal) => {
        fn $name(world: &World) -> &$ty {
            world.$field.as_ref().expect(concat!("the ", $built_by, " step runs first"))
        }
    };
}

layer!(terrain, terrain, crate::world::Terrain, "terrain");
layer!(roads, roads, crate::world::Roads, "drape");
layer!(reference, reference, crate::world::Reference, "reference");
layer!(profiles, profile, crate::world::Profiles, "profile");
layer!(facade, facade, crate::world::Facade, "facade");
layer!(ribbons, ribbons, crate::world::Ribbons, "ribbon");
layer!(surface, surface, crate::world::Surface, "surface");
layer!(kerb, kerb, crate::world::Kerb, "kerb");
layer!(fillet, fillet, crate::world::Fillet, "fillet");
layer!(room, room, crate::world::Room, "room");
layer!(sheets, sheets, crate::world::Sheets, "sheet");
layer!(arrangement, arrangement, crate::arrangement::Arrangement, "arrangement");
layer!(mesh, mesh, crate::world::Mesh, "mesh");
layer!(bench, bench, crate::world::Bench, "bench");

#[cfg(test)]
pub(crate) mod tests {
    use arpentry_server::project::Bounds;

    use super::*;
    use crate::terrain::tests::dem;

    /// The roundabout box the specimens are built in: ~1.5 km × 1.1 km,
    /// centred on (6.92, 46.435), where the synthetic grounds put their
    /// origin.
    pub(crate) fn bbox() -> Bounds {
        Bounds { west: 6.91, south: 46.43, east: 6.93, north: 46.44 }
    }

    /// A world over synthetic sources — the ground of `ground`, the network
    /// of `net`, the buildings of `houses` — run through `steps`, and what
    /// each step reported.
    ///
    /// The steps are named rather than taken as a prefix of [`Step::ALL`]
    /// because several specimens want one left out: a flat world has no
    /// heights to read, and a partition with no profile cuts what the
    /// source annotated, which is what those specimens are about. Saying so
    /// in a list is the point — the ladders used to be written out step by
    /// step in eight test modules and had drifted apart unremarked.
    pub(crate) fn built(
        ground: &str,
        net: &str,
        houses: Option<&str>,
        spacing: f64,
        steps: &[Step],
    ) -> (World, Summaries) {
        let mut world = World::new(bbox());
        let mut dem = dem(ground);
        let mut src = Sources {
            dem: &mut dem,
            segments: Path::new(net),
            buildings: houses.map(Path::new),
            spacing,
            max_vertices: usize::MAX,
        };
        let mut ran = Summaries(Vec::new());
        for &step in steps {
            let summary = apply(&mut world, step, &mut src).expect("a synthetic source reads");
            ran.0.push((step, summary));
        }
        (world, ran)
    }

    /// What each step of a [`built`] world reported, in the order they ran.
    pub(crate) struct Summaries(Vec<(Step, Summary)>);

    impl Summaries {
        /// The last step's line: what a helper named for that step reports.
        pub(crate) fn last(&self) -> Summary {
            self.0.last().expect("a step ran").1.clone()
        }

        /// One step's line, for a specimen that reads a step it did not stop
        /// at.
        pub(crate) fn of(&self, step: Step) -> Summary {
            self.0
                .iter()
                .find(|(s, _)| *s == step)
                .unwrap_or_else(|| panic!("the {} step did not run", step.name()))
                .1
                .clone()
        }
    }

    /// The three steps that solve heights. A specimen on flat ground leaves
    /// them out: there is nothing to solve, and a partition with no profile
    /// cuts what the source annotated.
    pub(crate) const VERTICAL: [Step; 3] = [Step::Reference, Step::Profile, Step::Crossing];

    /// The steps up to and including `until`, in order.
    pub(crate) fn upto(until: Step) -> Vec<Step> {
        let n = Step::ALL.iter().position(|&s| s == until).expect("a step of ALL") + 1;
        Step::ALL[..n].to_vec()
    }

    /// The same, less `drop`: how a specimen says what it leaves out.
    pub(crate) fn without(steps: Vec<Step>, drop: &[Step]) -> Vec<Step> {
        steps.into_iter().filter(|s| !drop.contains(s)).collect()
    }

    /// The flat plan's ladder up to `until`: [`upto`] without [`VERTICAL`].
    ///
    /// **It stops short of [`Step::Sheet`].** From there on the steps read
    /// the profiles — the sheet writes profile indices into every sheet, and
    /// the bench lifts by them — so a ladder with no heights in it has
    /// nothing for them to read. A specimen that needs to reach the sheet,
    /// the arrangement or beyond takes [`upto`], flat ground and all: the
    /// three vertical steps run and report zero, which is what flat ground
    /// means.
    pub(crate) fn plan(until: Step) -> Vec<Step> {
        without(upto(until), &VERTICAL)
    }
}
