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
//! — the surface step lays it, the kerb fills its strips, the legs build
//! its junctions, the room paves its edges — and the steps downstream used to
//! ask the world for "the latest", a resolver on `World` that walked those
//! four layers and returned whichever had been filled. Then it was a literal
//! here, assembled field by field out of two of them. Now there is nothing
//! to resolve or assemble: all four steps hand back a whole
//! [`crate::world::Surface`] — what they changed, and what they passed
//! through — so the latest paving is the last layer that laid any, and
//! `&room(world).surface` is a lookup like every other.
//!
//! **Writing into a layer somebody else built.** Three steps used to rewrite
//! their predecessors — first through `&mut`, then through installs in this
//! file: the reference promoted the terrain's priors onto the drape's ways,
//! the crossing replaced the profile step's profiles, and the partition
//! rewrote both again. After a run no layer but the last held what its own
//! step had made, so a step could not be looked at on its own. Now **every
//! layer is written once, by its own step**: the reference returns the ways
//! with its priors ([`crate::world::Reference::ways`]), the crossing its
//! re-solved profiles ([`crate::world::Crossings::profiles`]) and the
//! partition the network it cut ([`crate::world::Partition`]).
//!
//! **Which step reads what is this file's alone.** A step module imports no
//! other step module (`tests::no_step_imports_another`): what several of
//! them share is a module of its own ([`crate::standard`],
//! [`crate::solve`], [`crate::field`], [`crate::portal`], [`crate::gap`],
//! [`crate::copies`], [`crate::triangulate`], [`crate::lattice`]), and what
//! one step computes and another reads is a layer, passed here.
//!
//! **A step's build and its check are two functions** ([`apply`] runs both):
//! the build makes the layer and says what only it can know, and the check
//! measures the layer it made, so it can be asked again of a world loaded
//! from disk ([`run`], [`crate::dump`]) or put together by hand.
//!
//! The renderers are the exception: they walk the layers themselves and draw
//! the last one filled ([`World::network`], [`World::solved`]), because what
//! has been built is genuinely not known until the run stops.

use std::path::Path;

use arpentry_server::dem::Dem;

use crate::step::{Step, Summary};
use crate::world::World;
use crate::{
    arrangement, bench, building, crossing, drape, earthwork, facade, kerb, legs, lift, mesh, partition, profile,
    reference, ribbon, room, sheet, structure, surface, terrain,
};

/// The three sources a run reads, and the two knobs the terrain takes.
///
/// Fifteen of the eighteen steps read nothing but the layers before them;
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
    let built = build(world, step, src)?;
    Ok(built.and(check(world, step)))
}

/// Runs `step`'s build against `world`: the layer it makes, stored, and the
/// tallies only the build can know — what it dropped, clamped or squared.
fn build(world: &mut World, step: Step, src: &mut Sources) -> Result<Summary, String> {
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
            let (reference, summary) = reference::run(terrain(world), roads(world));
            world.reference = Some(reference);
            summary
        }
        Step::Profile => {
            let (profiles, summary) = profile::run(reference(world));
            world.profile = Some(profiles);
            summary
        }
        Step::Crossing => {
            let (crossings, summary) = crossing::run(reference(world), profiles(world));
            world.crossing = Some(crossings);
            summary
        }
        Step::Partition => {
            // **The one optional input in the chain.** The three vertical
            // steps run together or not at all: a flat specimen leaves them
            // out, and a partition with no heights cuts what the source
            // annotated — the drape's ways, as mapped. With them, it cuts
            // the ways the reference promoted, by the heights the crossing
            // re-solved.
            let (ways, solved) = match (&world.reference, &world.profile, &world.crossing) {
                (Some(r), Some(_), Some(c)) => (&r.ways[..], Some(&c.profiles)),
                (None, None, None) => (&roads(world).ways[..], None),
                _ => panic!("the reference, profile and crossing steps run together or not at all"),
            };
            let (cut, summary) = partition::run(ways, solved);
            world.partition = Some(cut);
            summary
        }
        Step::Facade => {
            let (facade, summary) = facade::run(&extent, network(world), src.buildings)
                .map_err(|e| format!("{}: {e}", src.buildings.unwrap_or(Path::new("")).display()))?;
            world.facade = Some(facade);
            summary
        }
        Step::Ribbon => {
            let (ribbons, summary) = ribbon::run(network(world), &partition(world).groups);
            world.ribbons = Some(ribbons);
            summary
        }
        Step::Surface => {
            let (surface, summary) = surface::run(ribbons(world), facade(world));
            world.surface = Some(surface);
            summary
        }
        Step::Kerb => {
            let (k, summary) = kerb::run(network(world), surface(world), facade(world));
            world.kerb = Some(k);
            summary
        }
        Step::Legs => {
            let (l, summary) =
                legs::run(network(world), surface(world), kerb(world), facade(world), &ribbons(world).masks);
            world.legs = Some(l);
            summary
        }
        Step::Room => {
            let (r, summary) =
                room::run(&legs(world).surface, &ribbons(world).spans, &kerb(world).attached, facade(world));
            world.room = Some(r);
            summary
        }
        Step::Sheet => {
            let (s, summary) = sheet::run(
                network(world),
                solved(world),
                &room(world).surface,
                &partition(world).groups,
                ribbons(world),
            );
            world.sheets = Some(s);
            summary
        }
        Step::Arrangement => {
            let (a, summary) = arrangement::run(
                &extent,
                &network(world).spans,
                solved(world),
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
        Step::Lift => {
            let (l, summary) = lift::run(terrain(world), solved(world), mesh(world), sheets(world), arrangement(world));
            world.lift = Some(l);
            summary
        }
        Step::Earthwork => {
            let (e, summary) = earthwork::run(terrain(world), mesh(world), arrangement(world), lift(world));
            world.earthwork = Some(e);
            summary
        }
        Step::Bench => {
            let (b, summary) =
                bench::run(terrain(world), mesh(world), arrangement(world), lift(world), earthwork(world));
            world.bench = Some(b);
            summary
        }
        Step::Structure => {
            let (s, summary) = structure::run(
                terrain(world),
                network(world),
                solved(world),
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

/// Measures the layer `step` built, from that layer and the layers the step
/// read: its invariants, and how far it moved the world.
///
/// A check is a function of what was built, never of how, so it can be asked
/// of a world the build did not make in this process — one loaded from a
/// dump, or a layer edited by hand — and it cannot steer the build. The steps
/// with nothing to check against their output report their build tallies
/// alone.
pub fn check(world: &World, step: Step) -> Summary {
    match step {
        Step::Reference => reference::check(reference(world)),
        Step::Profile => profile::check(profiles(world)),
        Step::Crossing => crossing::check(world.crossing.as_ref().expect("the crossing step ran")),
        Step::Partition => partition::check(partition(world)),
        Step::Kerb => kerb::check(kerb(world), facade(world)),
        Step::Sheet => sheet::check(sheets(world), &room(world).surface),
        Step::Arrangement => arrangement::check(arrangement(world), &world.extent),
        Step::Mesh => mesh::check(mesh(world), arrangement(world)),
        Step::Earthwork => {
            earthwork::check(terrain(world), mesh(world), arrangement(world), lift(world), earthwork(world))
        }
        _ => Summary::new(),
    }
}

/// Runs every step up to and including `until`, calling `report` with each
/// one's name and summary as it finishes.
pub fn upto(
    world: &mut World,
    until: Step,
    src: &mut Sources,
    report: &mut dyn FnMut(Step, &Summary),
) -> Result<(), String> {
    let range = Range { from: Step::ALL[0], until, dump: None, load: None };
    run(world, &range, src, &mut |step, ran| {
        if let Ran::Built(summary) = ran {
            report(step, summary);
        }
    })
}

/// Which steps a run builds, and what it reads and writes of the layers on
/// disk ([`crate::dump`]).
pub struct Range<'a> {
    /// The first step built. Every step before it is read from `load`.
    pub from: Step,
    /// The last step built.
    pub until: Step,
    /// Where each built step's layer is written, if anywhere.
    pub dump: Option<&'a Path>,
    /// Where the layers of the steps before `from` are read from.
    pub load: Option<&'a Path>,
}

/// What became of one step of a run.
pub enum Ran {
    /// Built here, with its line.
    Built(Summary),
    /// Read from disk, with the line its step printed when it built it.
    Loaded(String),
}

/// Runs the steps of `range`: reads every layer before `range.from` from
/// `range.load`, builds the rest up to `range.until` and writes each to
/// `range.dump`, calling `report` as each is done.
///
/// **This is how one step is debugged on its own.** Build the world once
/// with a dump, then start at the step under suspicion as often as it
/// takes: the steps before it are files, not seconds.
pub fn run(
    world: &mut World,
    range: &Range,
    src: &mut Sources,
    report: &mut dyn FnMut(Step, &Ran),
) -> Result<(), String> {
    let at = |step: Step| Step::ALL.iter().position(|&s| s == step).expect("a step of ALL");
    if at(range.from) > at(range.until) {
        return Err(format!("--from {} comes after --until {}", range.from.name(), range.until.name()));
    }
    for step in Step::ALL {
        if at(step) < at(range.from) {
            let dir = range.load.ok_or("starting past the first step needs the layers before it (--load)")?;
            let line = crate::dump::load(world, step, dir)?;
            report(step, &Ran::Loaded(line));
        } else {
            let summary = apply(world, step, src)?;
            if let Some(dir) = range.dump {
                crate::dump::save(world, step, &summary, dir)?;
            }
            report(step, &Ran::Built(summary));
        }
        if step == range.until {
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
layer!(partition, partition, crate::world::Partition, "partition");
layer!(facade, facade, crate::world::Facade, "facade");
layer!(ribbons, ribbons, crate::world::Ribbons, "ribbon");
layer!(surface, surface, crate::world::Surface, "surface");
layer!(kerb, kerb, crate::world::Kerb, "kerb");
layer!(legs, legs, crate::world::Legs, "legs");
layer!(room, room, crate::world::Room, "room");
layer!(sheets, sheets, crate::world::Sheets, "sheet");
layer!(arrangement, arrangement, crate::world::Arrangement, "arrangement");
layer!(mesh, mesh, crate::world::Mesh, "mesh");
layer!(lift, lift, crate::world::Lifted, "lift");
layer!(earthwork, earthwork, crate::world::Earthwork, "earthwork");
layer!(bench, bench, crate::world::Bench, "bench");

/// The network every step after the partition builds from: its cut.
fn network(world: &World) -> &crate::world::Network {
    &partition(world).network
}

/// The profiles every step after the partition reads: the partition's, with
/// its span table written back. They exist only where the vertical steps ran.
fn solved(world: &World) -> &crate::world::Profiles {
    partition(world).profiles.as_ref().expect("the vertical steps run before a step that reads the heights")
}

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

        /// The lines of `steps` as one: what a specimen written against a
        /// step that has since been split reads, the three halves of the old
        /// bench among them ([`BENCH`]).
        pub(crate) fn merged(&self, steps: &[Step]) -> Summary {
            let mut out = Summary::new();
            for &step in steps {
                out.counts.extend(self.of(step).counts);
            }
            out
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

    /// The three steps the bench used to be: the lift, the earthwork, and
    /// the faces that close them.
    pub(crate) const BENCH: [Step; 3] = [Step::Lift, Step::Earthwork, Step::Bench];

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

    /// The step modules, by the name each step has on the command line.
    fn step_modules() -> Vec<&'static str> {
        Step::ALL.iter().map(|s| s.name()).collect()
    }

    /// The code of `module` a step runs: its source less its tests and its
    /// comments, which may name any module they like.
    fn code_of(module: &str) -> String {
        let path = format!("{}/src/{module}.rs", env!("CARGO_MANIFEST_DIR"));
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        let body = text.split("#[cfg(test)]").next().unwrap_or_default();
        body.lines().filter(|l| !l.trim_start().starts_with("//")).collect::<Vec<_>>().join("\n")
    }

    /// **A step module imports no other step module.** What a step reads from
    /// another is a layer, and which layer goes where is this file's; what
    /// several steps share is a module of its own. A step that reached into
    /// another's code for a constant or a helper depended on it where
    /// nothing here could see it — twenty such reaches were found and moved
    /// out, and this is what keeps them out.
    #[test]
    fn no_step_imports_another() {
        let steps = step_modules();
        let mut found: Vec<String> = Vec::new();
        for &module in &steps {
            let code = code_of(module);
            for &other in steps.iter().filter(|&&o| o != module) {
                let path = [format!("crate::{other}::"), format!("crate::{other};"), format!("super::{other}::")];
                if path.iter().any(|p| code.contains(p.as_str())) {
                    found.push(format!("{module} -> {other}"));
                }
            }
            // A grouped import, `use crate::{a, b};`.
            for group in code.split("use crate::{").skip(1) {
                let names = group.split('}').next().unwrap_or_default();
                for name in names.split(',').map(|n| n.trim().split("::").next().unwrap_or_default()) {
                    if name != module && steps.contains(&name) {
                        found.push(format!("{module} -> {name}"));
                    }
                }
            }
        }
        assert!(found.is_empty(), "step modules reaching into other steps: {found:?}");
    }
}
