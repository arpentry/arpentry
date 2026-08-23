# The Engineered Ground and Its Meshes

`docs/GENERATION.md` states the vertical problem and prescribes that the
ground be an *output* of generation (D3, invariant 1). This document states
how: the universal road profile that stage 2 solves, the imprint rule by
which stage 3 pulls the natural DEM to those profiles, and the meshing rule
by which the per-zoom terrain meshes preserve the result. It exists because
the three are one mechanism — a road is only as smooth as the ground drawn
under it, and the ground is only correct where the profile is plausible.

The failure mode this design retires: the rendered terrain lattice is far
coarser than an earthwork's footprint (a z14 cell spans ~150 m against an
~18 m bench reach), so a grade-limited cut often moved no mesh vertex and
the road had to be clamped up onto every bump the lattice failed to carve.
The road broke visibly against each one. The fix is not a finer lattice —
it is a mesh that *knows where the benches are*.

## 1. The universal profile

Every drivable road gets a solved elevation profile. There is one vertical
model; "engineered corridor" versus "minor street" is a difference of
*parameters*, not of mechanism.

**The conditioned reference.** A profile anchors to the rendered terrain at
the reference zoom, conditioned symmetrically before anything reads it:

- *Closing* (fill): notches narrower than `NOTCH_SPAN_M` are filled up to
  `NOTCH_FILL_MAX_M` — an at-grade road bridges a stream-cut V-notch on a
  short culvert rather than diving through it.
- *Opening* (shave): convex bumps narrower than `BUMP_SPAN_M` are shaved
  down to `BUMP_SHAVE_MAX_M` — DEM noise (DSM artifacts, upsampling ripple)
  never enters a profile. A run whose shave would exceed the cap is
  reverted: a genuine crest keeps the terrain (the S9 mirror — the road
  climbs a real hill, it does not tunnel through survey noise).

Both operators are bounded in span and depth; beyond the bounds the terrain
is trusted. The *raw* terrain remains available for structural reads (rim
anchoring, portal emergence, deck daylighting), which must see the ground
as it is, not as conditioned.

**Per-class parameters.** Each road class carries a grade ceiling
(`grade_limit` for engineered classes, a bed grade for streets), a
deviation budget (how far the profile may leave the conditioned reference:
generous for a motorway, tight for a lane), and a node spacing (dense for
engineered classes, sparse for the long tail of streets — the cost bound on
profiling the whole network). Engineered-only behaviors — rim anchoring,
infeasible-anchor absorption into structures, deck ramps — stay gated on
the class having a true engineering grade.

**Solved structure ends.** An at-grade stretch beside a mapped structure
that the profile cannot honour is absorbed into that structure and the span
grown to match (S10 — the annotation ends where a mapper split the way, not
where the road reaches the ground). Two symptoms mark it: a pitch still far
beyond the class grade ceiling after the limiter, and a road standing more
than `ABSORB_STANDOFF_M` clear of the natural ground, on the side the
structure leaves it, in an unbroken run out from the span edge. The second
is what tells a viaduct mapped as one short span over the road it crosses
from a genuine embankment — and an embankment is what the ground stage
would otherwise build, walling off whatever passes beneath.

**Vertical-curve smoothing.** Engineered profiles round grade breaks only
on nodes lifted off the terrain (draped nodes stay pinned). Street profiles
smooth all nodes, clamped each pass to the class deviation budget of the
conditioned reference — the symmetric low-pass that removes residual
wobble without letting the street float.

**Welds.** Junction continuity holds across the whole network. The
structural weld (raise-only, corridor junctions, clearance-aware) runs
first; the street weld then groups profile endpoints by exact connector
coordinate and pulls them to one height — the engineered corridor's road
height where one passes through, the mean otherwise — with a trust cap
(`BED_WELD_MAX_M`) beyond which the disagreement is treated as a data
contradiction and left alone. Corrections decay into each profile at its
class grade. All deltas are computed against pre-weld heights, so the
result is independent of processing order (invariant 5).

## 2. The imprint

**The ground under a road is the road.** Every at-grade stretch of every
profiled road benches the ground to its solved height — not only the
stretches that depart the natural ground. A bench is not merely how an
embankment or a cutting gets expressed; it is how a carriageway *holds* its
height against the earthworks around it, and how it stays flat across a
cross-slope. A road the DEM already images correctly still needs one: given
no bench of its own, the approach fill of the motorway crossing above it
blended straight over it and buried the road under twelve metres of ground.

The field resolves in two steps:

- **Benches win.** Inside a bench half-width (carriageway + shoulder + a
  narrow verge) the ground *is* that road's profile — the
  nearest bench, deterministically tie-broken, never a mean of several.
  Averaging inside asphalt is what let a neighbour's earthwork dome the
  terrain up through a paved surface. Where two benches at very different
  heights abut, the field steps: that is a retaining wall, which is what an
  underpass beside an embankment physically has. The step falls on a crest
  contact line (§3), so the mesh draws it as a face.

  Nearest, but *a road's own carriageway first*. Proximity is the right
  arbiter between two roads and the wrong one inside a road: where a wide
  road runs closer to a narrower neighbour than its own half-width, the
  neighbour's *verge* is the nearer bench over the wide road's outermost
  asphalt, and the step would then land underneath a drawn surface — a
  retaining wall across the kerb. A bench holds its own carriageway outright
  and proximity decides only in the verge beyond it, so a step always falls
  between two carriageways rather than inside one. This is the same sentence
  as the heading, applied to the tie-break: the ground under a road is the
  road.
- **Batters clamp.** Outside every bench the ground is the natural surface,
  bounded by the straight batter faces that reach it: no lower than the
  highest embankment face, no higher than the lowest cutting face
  (1 in `EARTHWORK_BATTER`). The face is self-limiting — it stops exactly
  where it meets the natural ground — so an earthwork reshapes what it must
  and nothing more, instead of feathering a fixed-width corridor around
  every road. An edge works one side only, chosen by where its road sits
  against the ground there, so a road in a cutting cannot shave its
  neighbour's embankment; where the two contend the embankment survives and
  the cutting takes only what the fill has not claimed.

  Each side's reach is *where that face is expected to daylight*, computed
  from the cut or fill depth at the bench edge and the cross-slope the
  natural ground carries outward — both read from the same bench-edge
  sample. A face that would not close about as fast as it would on flat
  ground is not built at all: the face is a plane and the hillside is not,
  so where the ground runs away with it the earthwork goes on cutting the
  whole way out — a footpath whose estimated reach came to 40 m carved sixty
  metres off a gorge wall. There the face is rebuilt at a **wall** slope
  (1 in `WALL_BATTER`, a 4:1 face) and tried again against the same daylight
  test: a road cut into a steep flank is retained by a wall, not by a batter,
  and the alternative is not a gentler earthwork but *no* earthwork. The wall is
  short by construction — it closes in well under a metre where a batter would
  have run tens — and it is bounded by the same test, so where the ground does
  carry the step it builds nothing. Only where neither face closes does the
  reach collapse to **zero** and the bench stand retained at its own edge —
  zero, not a short bevel, because a bevel on a diverging face is a trench: it
  holds the ground down along a plane the hillside is climbing away from, and
  where it ends the field steps back to the hillside out in open ground, where
  no contact line runs and the mesh draws the step as a row of teeth
  (`slope.terrain_tearing`). A converging face keeps its floor, where it costs
  nothing: past the point such a face daylights the natural ground is already
  inside it, so a longer reach returns the same answer. This is also why the
  bench is kept narrow: a wide flat bench is a wide terrace cut into every
  hillside the road crosses, and a deeper face where it ends.

**Everything outside a bench is a `min`/`max` of planes, and that is the
constraint.** Where two benches stack — a switchback above itself, a street
under a railway — the DEM between them carries a median 84 % of the separation
the two solved profiles carry, and under half of it in 29 % of cases, so the
alignments are the better vertical control and the ground between them ought to
follow *them*. Interpolating between the two benches that bracket a point does
not work and cannot be made to: which pair brackets a point is not continuous in
the point, so two neighbouring lattice vertices pick different pairs and the
field steps between them — `slope.terrain_tearing` caught exactly that, worst
6.46 → 9.24 m, along with the kerb lip and two more tails. A formulation that
can work has to be continuous by construction: let each bench's face reach *to
the other bench's edge* rather than collapse where it cannot daylight, so the
two faces meet each other. That stays a `min`/`max` of planes, and planes
cannot tear.

This is built (`ground::span_bench_gaps`), and it applies exactly where the
collapse rule above leaves a face at the wall slope or at zero: if a *higher*
bench stands within `BENCH_GAP_SPAN_M` of the collapsed side, the face is
rebuilt as the cut plane leaving this bench's verge at its own target and
landing *inside* the partner's bench, so its far end falls where the partner's
crest line already constrains the mesh and no step is left in open ground. The
plane shaves the natural sliver that would otherwise stand proud between them
— the wall of hillside a camera found hugging the outermost rail of the
Territet trench, two metres from the rails and five metres tall. Cut only: the
mirrored fill plane — the upper bench reaching down — was tried and measured,
and where the gap drains a stream that plane is a dam (`water.descends`
2.498 % → 2.511 %); a cut cannot dam anything, and the fill-side deficit
already has its answer in the drawn apron (§3). The pass runs over the
assembled stack, because the pair is usually cross-stratum (a rail trench
beside the road bench above it) and the side that collapsed derives before its
partner exists; it reads only bench geometry, never the batters it rewrites,
so it stays a function of the model alone (invariant 5). Beyond the window,
where no partner stands, and downhill, the collapse rule keeps its answer:
open hillside belongs to the terrain.

**Where a bench is not plausible at all.** Holding a band flat across a
cross-slope costs a face at its edge of half-width × slope. Past
`MAX_BENCH_FACE_M` no bench is emitted and the road is left on the natural
ground, tilted as the hillside is. A terrace on the wall of a gorge is a
fiction — a footpath's eight-metre band held flat there means a twenty-metre
rock cut on one side and as much fill on the other — and it is a fiction the
mesh cannot draw: the crest tears into sawtooth where the wall exceeds what a
cell can hold. For a trail cut into a cliff, draped *is* the truth.

**The ground under a walkway is the walkway.** A pedestrian band is a drawn
surface with its own hole and its own apron (docs/ROADS.md invariant 7), and it
had no bench at all: a sidewalk was seated on its host's cross-section a kerb
above the carriageway while the ground under it was still the street's bench —
which stops a verge past the asphalt — or the batter face beyond it, and a path
*was* the drawn ground, tilted at whatever angle the hillside carried. Measured:
the band met the ground at a step over 19 % of its own rim (`contact.walk_rim`,
p50 0.19 m over the sidewalk half, reaching 15 m), and half of all drawn path
length tilted more than 30 % across its own two metres (`slope.walk_crossfall`).

So every at-grade band benches, in **stratum D** — the layer draped features
imprint, applied last over the finished H → R → S ground, which is what a
walkway is: no authority, no solve, laid on whatever the seniors built. Three
things distinguish it from a corridor bench, and each is the band being narrow:

- **It is derived from the band, not from a class prior.** `synth::walkway`
  builds the bands once, before stage 3, and the ground benches those same
  segments (`ground::walk_earthworks`). One derivation, two readers: a bench
  from a second construction of the same band is a bench that does not fit it.
- **Its target is its seat.** A sidewalk's is the host's road surface plus the
  kerb — the height the band is drawn at. A path's is the ground *beneath this
  stratum*, sampled at **both ends of each segment**, so the bench flattens the
  cross-section and changes nothing along the way: a footpath still climbs
  whatever hill it climbs. Reading one height per segment instead — the
  midpoint's — turns a climbing path into a staircase of terraces, each flat at
  its own middle and stepping to the next, and the lateral face cap cannot see
  it because at the midpoint there is nothing to see. It drew as a 324:1 terrain
  face.
- **It emits no contact lines.** The band's own ring is already a constraint and
  the bench is a verge wider than the band on each side, so the ring falls
  inside the bench exactly where §3 says to draw a crest, and every rim vertex
  samples the flat bench already. A second line half a metre outside the first
  would double the constraint count of the densest network in the extract to say
  the same thing twice. It follows that the walkway must not *contend* for the
  crests of others either: at 0.12 m of kerb rise against a 0.05 m same-bench
  tolerance, every sidewalked street failed the test at its own bench edge and
  dragged 17,635 crest nodes inward, so the crest question is asked of the
  benches that draw crests (`Earthworks::crest_target_at`).

Because it draws no lines, it is **bounded instead of constrained**, and the two
materials do not hold the same bound. A path holds `WALK_MAX_FACE_M` — a metre,
the earthwork a footpath actually builds — so the largest step it can leave in
open ground is two metres, which a lattice cell carries as a slope rather than a
wall. A sidewalk holds the street's own `MAX_BENCH_FACE_M`, because where a
street stands on a terrace with a wall down to the hillside the sidewalk stands
on the same terrace and the wall is the street's; its apron draws it. Held to
the street's allowance on *both*, footpaths start eating tunnel cover, moving
the ground under walls and damming streams — five unrelated metrics move, and
they come back the moment the path is held to its metre.
`ARPT_NO_WALK_BENCH=1` draws the bands and leaves the ground unbenched, which is
the A/B control every number above was measured against.

Carves (portal cuts, under-deck daylighting) remain separate cut-only
notches, bounding the benched ground from above — a carve is a hole, not a
target.

**A portal cut ends at a face.** Every other edge fades out around its ends in
a half-disc of its own reach, which is right for a run that simply stops and
wrong for a trench dug *in front of* a tunnel mouth: the hill behind that mouth
is the cover the tunnel runs under. Left to fade, the trench floor swings
around the mouth and takes half-width-plus-batter of hillside off the first
metres of tube — eight metres of it on a motorway — so the bore surfaces well
before the drawn hill begins and the ground it should have entered stands as a
cliff at the rim of the scoop. A portal carve therefore declares itself
`headwall`: its influence stops at the plane through its inner end, in the
height fold, in the footprint I8 is stated over, and in the bed query alike.
What is left at the plane is a step of the mouth's own height, which is what a
portal is, and the tube stamped through it fills exactly that opening.

The ground model is a pure function of the global solved model: built once,
queried pointwise, identical from every tile that asks (invariant 5).
Terrain meshing, road draping, surface bands, junction plates, and building
founding all read this one field and nothing else (invariant 1).

## 3. The meshing rule

**What constrains.** For every earthwork run the imprint implies one
polyline per side: the *crest* line at the bench edge, where the ground
stops being the road and becomes the batter face — or a wall down to
whatever bench abuts it. These are the contact lines of GENERATION.md
stage 3. At detail zooms (z ≥ z_ref) the terrain mesh is a constrained
triangulation: the regular lattice persists as background points — every
vertex the unconstrained mesh would have, it still has — and the contact
lines enter as constraint edges. Between crest lines the triangulation
cannot place a triangle that straddles the bench, so the drawn ground
holds the bench exactly under every road, and a wall draws as a face
rather than smearing across a cell.

There is no toe line. The batter stops where it meets the natural ground,
so the toe stands wherever the ground happens to rise into the face, which
no offset of the centerline predicts; a constraint at the nominal reach
would pin vertices in the wrong place and double the constraint count to
do it.

**The hole.** At the detail rung the terrain mesh *stops at the kerb*. The
level-0 paved rings — asphalt carriageways and rail formation bands alike, every
region of the unioned surface — enter the triangulation as constraints alongside
the crest lines, and every face whose centroid falls inside them is dropped. The
drawn surface is opaque and watertight, so ground drawn beneath it is redundant
— and being redundant is where every artifact of this family lived. Measured on
Montreux at z16, at-grade asphalt below the drawn ground went from 253,651
samples to **zero**, not by a smaller margin but by construction: there is
nothing left underneath to be below. Rail is in the union for the same reason
it benches unconditionally: a railway used to have no surface at all — no mesh,
no kerb, no hole, no apron — so every vertical disagreement between its solved
profile and the drawn ground was daylight under a floating stroke. As a
`Ballast` region it gets the identical treatment in another material, and the
residual disagreement becomes a kerb lip and an apron wall, measured where a
road's is.

Three rules make it safe rather than merely effective:

- **Only rings that were actually meshed cut.** A level whose asphalt failed to
  triangulate must not leave a hole with nothing over it (invariant 6). And
  *every* at-grade region cuts, not the first one found: a tile can carry
  several level-0 regions on different grade layers, and one left uncut is
  asphalt the burial comes back through. Structures never cut — a deck flies
  and the ground beneath it stays.
- **The rim stays on the ground, and the wall is drawn.** A rim vertex takes the
  *ground's* height there, not the road's. Where a bench holds, the two are the
  same number and nothing more is needed. Where none does they differ by
  whatever the model failed to build, and that difference is emitted as an
  explicit vertical face — one quad per silhouette edge, from the kerb down to
  the ground — as a `road_apron` feature beside the surface and its casing.
  Fifteen metres of it is the retaining wall that is physically there; a few
  centimetres is a kerb and is skipped (`APRON_MIN_M`).

  Pulling the rim *up* to the road instead was tried first and is worse in two
  ways. It hides the wall by smearing it across the first lattice cell, which
  the steepness check duly reported (`slope.terrain_face` 89,898 → 116,589
  violations, worst 419:1); and on a tile border it breaks invariant 2, because
  two neighbours clip the same global region against different rects and one can
  call a shared border vertex a ring vertex while the other does not. That
  measured as a **6.7 m step** down the seam. Deferring to the global ground
  function everywhere removes both: the face count came back to 88,756, below
  where it started.

  A cut edge carries no apron. A cut is a tile border, where the asphalt
  continues into the neighbour and there is no kerb at all — walling it would
  build a fence down every tile edge.

What the hole does *not* do is make the road's height right. It removes the
drawn ground that made a wrong height visible as burial, and the apron draws the
difference instead of hiding it. Both are measured at the kerb
(docs/VERIFICATION.md §4): `contact.kerb_lip` is how tall a wall the model
implies — 12.6 % of the carriageway's edge, reaching 14.8 m — and
`contact.kerb_unwalled` is how much of that wall is missing, which the apron
takes to **0.8 %**. The lip is not a defect to drive to zero; it is the honest
size of the earthwork the profile asked for. The unwalled share is.

**The z rule.** Constraint vertices carry no height of their own; every
mesh vertex, lattice or constraint, evaluates the one ground function at
its position. Breaklines say where to sample, never what value to find.

The crest line is therefore drawn a hand's breadth *inside* the bench edge,
not on it. The edge is where the field steps, tile coordinates quantize to
about a centimetre, and the triangulation rounds its own split vertices: a
line drawn on the step samples the road at one vertex and the hillside at the
next, and the mesh comes out as a row of teeth. Inside the edge every vertex
of the line is unambiguously on the bench, and the step falls between the
crest and the first lattice point beyond it.

**A crest stops where its bench stops holding the ground.** The half-width is
a *geometric* edge, and where two benches overlap it is not where the field
steps. Benches win by proximity, so between two roads closer together than
their half-widths the winner changes at a boundary somewhere between them, and
each road's nominal crest lies past it, inside the other's reign. A crest node
there samples the *neighbour's* height — the z rule cuts both ways: a
breakline says where to sample, so a line in the wrong place finds the wrong
answer — and the triangulation then ramps that height back across the road the
crest was drawn to protect. A service way beside a street three metres higher
went under the hill that way. So each crest node is pulled in to where the
ground under it is its own bench again, and a bench crowded out to its own
axis emits no crest at all: the neighbour holds the ground there and draws the
crest the mesh needs. Uncontended crests keep the nominal offset, so the
pull-in costs nothing where nothing contends; the run stats report how many
nodes were pulled and how many dropped.

A pulled crest is also *sampled* more finely. The line's nodes are the
earthwork's own, a class node-spacing apart — tens of metres on a street —
which is exact while the offset is constant, because the chord between two
samples then lies on the line. Where a neighbour crowds the bench the offset
differs at each end and the chord cuts inside both, under the carriageway's own
asphalt; the mesh holds the bench only inside that chord and ramps the
neighbour's wall over the strip outside it, which beside a railway seven metres
up is a wall drawn across the kerb. Contended segments are therefore
subdivided to about a lattice cell, so the crest tracks the boundary rather
than chording across it.

**The tile-border contract.** Contact lines are global polylines clipped
per tile. A border crossing lands exactly on the quantized tile edge
(16384 / 49152 on the crossed axis); the neighbor clips the same global
polyline against the same border and derives the identical vertex by
determinism. Interior triangulations of adjacent tiles need not match —
only border vertex positions and heights must, and the client's skirts
close the residual cracks as they do today.

**Normals.** At detail zooms vertex normals are central differences of the
ground function at a fixed metric step — a property of the field, not of
the mesh — so they are continuous across tile borders, flat across a
bench, and creased at a batter regardless of how the triangulation fell.

**The LOD ladder.** Coarser rungs (z < z_ref) keep the plain regular
lattice: at those viewing distances a bench narrower than a cell is not
worth vertices, and roads compensate with the datum lift (§4). The
constrained mesh is a detail-zoom instrument.

What they read is *scale-aware*: the ground function is asked at the cell
spacing of the lattice asking, and an earthwork narrower than that spacing is
left out of the answer. Sampling it is what does the damage — a corner that
lands inside a 10 m bench takes the road's height while its neighbour a cell
away takes the hillside, and the mesh spikes; whole slopes of terraced tracks
came out as sawtooth noise one rung out from the reference. The road
compensates with its datum lift (§4), so dropping the bench from the drawn
ground does not float it. At the reference zoom the constrained mesh holds
every bench exactly, so nothing is filtered there.

Their *resolution*, too, grades down a rung at a time rather than
collapsing to the base lattice in one step. What the eye reads is the
metric cell size, and each rung covers four times the area of the one
below: the detail grid straight to the base grid is a 64-fold drop in
vertices, so one zoom out from the reference the ground went from ~3 m
cells to ~50 m and the hillsides turned blocky — including in the
distance of a tilted view, which draws coarser rungs whatever the camera
altitude. Halving per rung roughly doubles the cell size instead, and the
three graded rungs together add ~7 % to the reference rung's vertices,
because a rung has a quarter of the tiles and a quarter of the vertices in
each.

**Degradation.** A tile whose constrained triangulation fails — degenerate
constraint after quantization, offset self-intersection the pre-pass did
not resolve — falls back to the plain lattice for that tile (invariant 6:
plain, not wrong). Hairpin insides whose offset radius exceeds the curve
radius drop the inner contact line rather than emit a folded one.

## 4. What roads read

- **At detail zooms** the road reads its profile directly: the terrain
  holds the bench, so profile height and drawn ground agree by
  construction. No clamp, no lattice-crossing chords.
- **At coarser zooms** the road reads that zoom's rendered surface plus a
  clamped datum lift — its solved height expressed relative to the
  reference-zoom ground, never below the drawn surface. This is the
  coarse-LOD rule, not a workaround: the coarse lattice cannot carry a
  bench, so the road hugs the terrain that *is* drawn (invariant 4).
- **On structures** the road rides the deck ramp, the same heights the deck
  and bore solids are swept from (invariant 2) — and at coarser zooms the
  whole span carries the same rule the road does: every structure vertex
  adds the local **datum shift**, the drawn ground at that zoom over the
  drawn ground at the reference (`synth::datum`), so the span rides the
  coarse canvas by the identical local displacement and every *relative*
  relation — the step at an abutment, the clearance over a crossed road, a
  bore roof's burial — is inherited from the reference world pointwise. An
  absolutely-positioned span at a coarse zoom sat a lattice-error away from
  everything drawn around it: a path bridge 17 m under the hillside a z14
  cell smoothed over its gully, a band arriving metres below the deck it
  hands over to (`seam.band_deck_step` 98 % over at z14). The shift is read
  from one shared piecewise-linear curve per span, stationed on the global
  corridor arc, so the solid and the paint riding it reconstruct the same
  function from any tile; the intersection pin carries it too, with the
  same raise-only clamp its legs have. Zero at the reference zoom by
  definition. A structure's *absolute* top therefore legitimately differs
  between rungs by the canvases' divergence — `lod.structure_drift`
  measures the height over each zoom's own drawn ground instead.
  `ARPT_NO_ZOOM_DATUM=1` restores absolute structures for an A/B re-tile.

Reading the field is not enough on its own: the asphalt has to be *meshed*
finely enough to hold what it read. Sampling the field only at the paved
region's outline leaves triangles spanning the whole carriageway, and on an
unbenched cross-slope the ground — sampled every cell — crosses them. The
paved surface is therefore triangulated over the terrain's own lattice
(docs/ROADS.md §6.1). An intersection pin carries the same raise-only clamp as
every carriageway source: no drawn asphalt sits below the ground drawn under
it, whichever source answered for it.
