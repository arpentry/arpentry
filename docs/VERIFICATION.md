# Verification

How the emitted scene is measured against the invariants, and why the
measurements are shaped the way they are.

Owned by `server/src/verify/`. Run with `arpentry_verify`.

## 1. Why

`GENERATION.md` §7 states eight invariants and calls them "acceptance criteria
for any implementation". §6 states nineteen canonical situations and calls them
"the test scenarios for any design". Both were prose. The only instrument that
existed was `solve::consistency`, which measures the *solved model* and reports
it consistent.

Every defect that has cost real time lives downstream of the solve, in the
ground, synthesis and tiling steps:
asphalt chording over the ground, a crest sampling a neighbour's bench, plates
z-fighting, a deck stepping at its abutment. Each is a **relation between two
emitted surfaces**. Nothing that reads `SolvedModel` can see one, so the only
available instrument was a rendered screenshot and a judgement about it.

Screenshots are good at *discovering* a defect and bad at keeping one dead. They
answer "is something wrong" — rarely in doubt — and not "did what I just do make
it better", which is what every iteration is actually asking. They also do not
accumulate: fixing the mountain tunnel and breaking the river bridge is
indistinguishable from progress, because nobody flies back to the river.

So this measures the shipped archive, on the same ladder of invariants, and
prints a table. A change is judged by the table diff.

**The loop this is for.** Look at a render, find a failure mode nobody knew
about, then — before fixing it — write the check that measures it. The
screenshot finds a class of defect once; the check keeps it dead. That is the
one thing the previous workflow did not do: each defect was found visually,
fixed, and then re-findable only visually.

## 2. Using it

```sh
# The state of the scene.
arpentry_verify data/overture-ch/preview.arpa

# Did this change make it better? Exit 1 on any regression, so it can gate.
arpentry_verify preview.arpa --baseline server/verify/baseline-montreux-z16.json

# What is happening at the place that looks wrong?
arpentry_verify preview.arpa --at 6.9290,46.4200

# One canonical situation, at the strongest instance in this data.
arpentry_verify preview.arpa --scenario S5

# Re-site the corpus after retiling a different extract.
arpentry_verify preview.arpa --mine > server/verify/scenarios.json

# See what is actually there, in the one projection where heights are legible.
arpentry_verify preview.arpa --at 6.928167,46.426206 \
    --bearing 90 --length 120 --section /tmp/cut.svg
```

A full z16 pass over the Montreux extract takes about 7 s and 16 M surface
samples. It reads the archive only, so it can be pointed at one built by an
older revision — which is what makes a baseline diff meaningful.

## 3. The three design rules

**Measure the surface, not the vertices.** The probe this replaced asked every
road *vertex* whether it stood above the terrain and answered "mostly", while
the asphalt was visibly chording across the ground *between* those vertices. The
defect lived strictly between the samples, so the instrument was blind to it by
construction. Checks here sample triangle interiors at a metric spacing and
interrogate the other mesh as a continuous field.

**Report a distribution, not a verdict.** A boolean needs a threshold, and the
thresholds are priors nobody knows in advance. A distribution needs none: it is
comparable against the same measurement taken before the change, which is the
question actually being asked. Every metric carries count, extremes, tail and a
violation tally; the extremes and counts are exact, the quantiles are binned to
about a centimetre.

**Scope to what the design actually promises.** At-grade road height is
*deliberately* zoom-dependent below the reference rung (`GROUND.md` §4, the
datum lift), so the cross-zoom check is scoped to structures. Checking a
property the design never claimed produces noise, and noise is what makes a
scorecard get skimmed.

## 4. What is measured

| metric | inv | what it means |
|---|---|---|
| `contact.kerb_lip` | 4 | Carriageway edge height minus the drawn ground a metre outside it. Not a defect on its own — it is how tall a wall the model implies, and a road on a real embankment has a real drop at its edge. It is what still sees a road standing on an embankment nobody built. |
| `contact.kerb_unwalled` | 4 | The part of that drop with no apron face spanning it. This is the gate: the lip is the wall's height, this is how much of the wall is missing. |
| `order.deck_above_carriageway` | 3 | Deck running surface minus the at-grade asphalt sharing its plan position. Negative past the touchdown band means the level ordinal inverted. |
| `clearance.deck_over_ground` | 4 | Deck soffit minus the terrain beneath it. Past a deck thickness, the deck ploughs into the hillside. |
| `clearance.bore_cover` | 4 | Terrain minus the bore roof. Negative past a portal mouth means the tube is in open air. |
| `seam.terrain_step` | 2 | Spread of the heights two adjacent tiles derive for the same border lattice point. |
| `seam.terrain_split` | 2 | Spread between coincident vertices *inside* one tile: the ground cracked open. |
| `seam.pavement_step` | 2 | Border disagreement, for the at-grade road surface. |
| `order.at_grade_overlap` | 3 | Vertical separation where two level-0 paved regions share a plan position with nothing to order them. |
| `slope.terrain_face` | 6 | Rise over plan run of every terrain triangle spanning ≥10 cm. Finds manufactured retaining walls. |
| `slope.carriageway_face` | 6 | The same for interior asphalt, excluding the kerb rim. |
| `slope.road_grade` | 6 | Rise over run between consecutive vertices of a drawn drivable centerline. Measured *along* the road, which the carriageway mesh cannot answer: a clearance lift dropped on one node is a spike the face metric reads as ordinary cross-fall. |
| `slope.terrain_tearing` | 6 | How far a terrain vertex stands off the plane of its neighbours, counted only where opposite breaks flank it on both sides. Separates a wall from a wall drawn as teeth, which no steepness can. |
| `lod.structure_drift` | 5 | Structure height at one zoom against the same structure one rung coarser. |

### Where the thresholds come from

Every structure-versus-surface check has a **legitimate contact band** that a
naive zero threshold reports as a defect. Each threshold below was read off the
measured distribution, not assumed — and the first version of this module got
one of them badly wrong, which is why they are documented here:

- A deck **touching down at its abutment** has its soffit a deck-thickness below
  the ground. Threshold: −(`DECK_THICKNESS_M` + 0.5).
- A bore's **roof crosses the surface at a portal mouth** by design. Threshold:
  −1.0 m.
- A deck **meeting the road at grade** sits level with it and owes it nothing.
  Threshold: −0.5 m.
- A **steep face spanning no height** is quantization on a sliver, not a cliff.
  Faces must span ≥10 cm to count.
- The **carriageway's silhouette is its kerb**, vertical by design. Excluded;
  unfiltered it was 44 % of the steep faces counted and no change could ever
  have removed it.
- An **apron is vertical by construction**, so no point-in-triangle query can
  find it: its plan projection is a segment with no area. `contact.kerb_unwalled`
  therefore asks what the apron spans *near* the kerb rather than *at* it
  (`APRON_NEAR_M`), and allows `APRON_SLOP_M` at each end for quantization and
  for a probe standing a metre out on sloping ground. Measuring it the obvious
  way reported every apron as absent.
- A **quantized plan run divides into noise.** Plan coordinates are a `uint16`
  lattice, about 2 cm at z16, so a centerline step of a few centimetres carries
  a real height over a rounded run and reports a ratio that is mostly the
  rounding. `slope.road_grade` counts only steps of at least 0.50 m; below it
  sit 4 % of all steps and the entire top of the unfiltered ratio distribution,
  every one spanning under a centimetre of height. Non-drivable classes are
  excluded for the same reason a kerb is: a footway may be a staircase and a
  rack railway climbs at 20 %, so counting them measures the class table rather
  than the defect.
- A **wall put on a lattice always breaks the surface twice**, once low at its
  crest and once high at the first vertex down its face. Counting a vertex with
  one opposite-signed neighbour would therefore report every wall ever drawn, so
  `slope.terrain_tearing` counts only vertices flanked by opposite breaks on
  *both* sides — an oscillation rather than a step. The filter is what makes the
  metric usable: unfiltered it called 3.9 % of the Montreux extract's terrain
  vertices torn, and with it 0.20 %, with the median falling from 3 cm to 3 mm.
  The 0.50 m threshold is read off that distribution — real landform lives in
  the centimetres, and at a ~3 m detail cell half a metre of alternation is the
  mesh disagreeing with itself.

### What is deliberately not measured

**The class clearance gap.** The obvious check — "is the soffit 5 m above the
road it crosses" — cannot be posed from the archive. The at-grade carriageway is
one unioned region, so a deck sample that finds asphalt beneath it cannot tell a
road being *crossed* from its own approach it is about to *join*. Measured
anyway, the population came out plainly bimodal: 23 % of samples sat within half
a metre of the carriageway (abutments) and every one was counted as a 5 m
shortfall. The metric read 36 % violations and was measuring touchdowns.

The clearance inequality belongs at stage 2, where the crossing set is *known*
rather than inferred from plan overlap, and is already measured there as
`consistency.max_clearance_violation_m`. The archive can answer the weaker,
prior-free half of invariant 3 without ambiguity — the level ordering — so that
is what `order.deck_above_carriageway` measures.

**Cross-zoom equality for at-grade roads.** Zoom-dependent by design; see
`GROUND.md` §4.

**Longitudinal grade of every drivable road at the detail zoom.** From z13 the
union paves the carriageway as a mesh and `stamp_synth` drops the fill stroke
that would otherwise draw it twice, so the only centerlines the detail tiles
still carry are the markings, the deck strokes, and the non-drivable classes.
`slope.road_grade` therefore measures a sample of the drivable network, not all
of it — 62 k steps over the Montreux extract — and a road with no markings can
carry a spike it never sees. The measurement is honest about *what* it covers
because the alternative is worse: the carriageway mesh has no direction, so a
grade read off it cannot separate a climb from a cross-fall. Full-population
coverage lives at stage 2, where `examples/crossing_probe.rs` histograms the
solved profiles' node-to-node grade directly.

**Structure drift where the match is ambiguous.** Structures carry no identity
across zooms beyond class and level ordinal. Where a coarse tile holds several
candidates, "the same structure" is a guess — the first version made it and
reported 2.06 m of drift on a deck whose parent held *nine* candidates, which is
evidence of comparing two bridges, not of drift. Samples count only on a
one-to-one match, and the skipped count is reported.

**Buildings and water.** Invariants 4 and 6 cover both (S3, S11–S14); the
verifier decodes only the terrain and transportation layers today. This is the
largest gap.

## 5. Sections: the diagnostic half

The scorecard says *that* something is wrong and where. `--section` says *what
it looks like*, in the one projection where a height model is legible.

A 3/4 perspective screenshot is close to the worst possible image for judging
heights, for a person or for a model: everything is foreshortened, the ground
occludes the thing you are trying to see, and a 3 m step at an abutment is a few
pixels of shading. In section it is a 3 m step.

It draws the drawn ground, the at-grade asphalt (the first *two* regions, kept
apart so an unordered overlap shows as two lines), and structure top and soffit,
with breaks where a surface stops — a deck's span ending matters as much as its
height. Feed it any offender coordinate the table printed; that is the intended
loop.

Three cuts from the first run, each answering in one image what a screenshot
could not:

- At the worst overlap (`6.928167,46.426206`): a level-0 asphalt block floating
  ~9 m on near-vertical sides, with a second level-0 region correctly following
  the ground beneath it. Three metrics — `slope.carriageway_face`,
  `order.at_grade_overlap`, `contact.pavement_over_terrain` — turn out to be one
  defect seen from three angles.
- At the worst burial (`6.933446,46.449455`): the drawn ground spikes 6 m in a
  sharp zigzag exactly where the asphalt ends. The road height is not wrong; a
  manufactured wall sliver is standing over it. The terrain-hole study inferred
  this from statistics; the section shows it.
- At S1, the tallest viaduct: two deck segments flying ~35 m over a valley,
  ending in mid-air. Invariant 4's documented missing-pier deviation, visible
  rather than remembered.

## 6. What the scorecard has already settled

Two changes were tried against the earthwork bench criterion and both were
rejected — the first by a retile and a diff in about ninety seconds, the second
by the unit tests before a retile was needed. Recorded here so nobody spends a
day rediscovering them.

**The finding that prompted it.** The contact check originally reported only
burial. Adding the other tail showed floating is the larger half by far: 3.8 %
of asphalt stands more than a metre clear of the drawn ground and reaches 15 m,
against 1.5 % buried reaching 4.2 m. A one-sided instrument had hidden it.

The cause is `MAX_BENCH_FACE_M`, which caps the face at the bench edge —
`(road − terrain) + (terrain − edge)`, the road's own departure from the ground
plus the hillside's fall across the band. Above the cap no bench is emitted,
which does not put the road back on the ground: it leaves the road where the
profile put it and the ground where the DEM had it, with air between.

| attempt | result |
|---|---|
| Cap the **cross-slope term only**, so an embankment always benches | Six metrics regressed. Terrain faces 264:1 → **542:1**, worst burial −4.2 → **−7.6 m**, deck-into-hillside −6.9 → **−12.5 m**. Every tall fill benched, and a fill whose batter cannot daylight becomes a retaining wall. The worst float did not improve at all. |
| Delete the prior and ask whether the **batter daylights** instead | Far too strict. A 1:2 cross-slope closes nowhere against a 1:2.5 batter, so an ordinary hillside street loses its bench. Rejected by `a_street_bench_is_flat_across_a_side_slope`. |

**The conclusion is structural.** On steep or tall ground the earthwork must
choose between a wall, a float, and no bench. All three are defects, and all
three are visible *only because the ground is drawn under the asphalt at all*.
This is the same wall-versus-accuracy trade the terrain-hole study hit from the
opposite direction when it raised the cap 3 → 6 → 12. Tuning the criterion moves
the defect; it does not remove it. Cutting the terrain back to the kerb
(docs/GROUND.md §3, "the hole") dissolved the choice instead of balancing it,
and these two results were independent evidence for doing so. What the cap still
costs is now measured at the kerb — `contact.kerb_lip` and
`contact.kerb_unwalled` — rather than under the asphalt.

## 7. A worked example of how easy it is to be wrong

The seam check went through three shapes before it measured anything true, and
the sequence is the argument for the "measure its anatomy" rule below.

**First reading: 3.82 m of carriageway seam step, 0.19 % of border points, while
the terrain seam read 0.000 m.** A striking asymmetry — the same instrument, two
meshes, one perfect. It looked like a tiling bug.

**Hypothesis: the paved region is baked per z13 chunk, so adjacent z16 tiles in
different chunks come from different bakes.** Falsified, cleanly. Cross-chunk
borders: 593 shared points, worst 0.000 m. The chunk bake is exactly right.

**Second reading: the instrument was conflating two defects.** It kept a single
min and max per lattice point, so a *single tile* holding two coincident
vertices 3.8 m apart scored as a disagreement with its neighbour. Of 42 stepping
points only 16 were cross-tile, and the worst was not one of them. Separated,
the seam step fell to **0.003 m** — the border contract holds for the asphalt
exactly as it does for the ground. The original headline was entirely artifact.

**Third reading: the remaining defect is not a crack either.** No single
carriageway mesh disagrees with itself anywhere: 784,851 distinct plan
positions, zero carrying two heights. The disagreement is *between* meshes —
53 of 320 tiles carry more than one level-0 `road_surface`. That is by design:
`synth::pavement` keys regions by `(level, layer)`, and its own doc note says
regions on different grade-separation layers "overlap in plan but are metres
apart vertically".

**What is actually wrong:** `add_road_surface` encodes only `level`. The layer
that separated the two regions is dropped, so the client receives several opaque
at-grade surfaces at one ordinal, overlapping in plan by up to 8.83 m, with
nothing to order them. 616 plan points overlap, 74 of them by more than a metre.

Three readings, two of them wrong, and the true finding is in a different module
from where the first number pointed. Nothing about the first reading was
obviously suspect.

## 8. The corpus

`server/verify/scenarios.json` binds each situation from `GENERATION.md` §6 to a
real place. Sites are **mined, not invented**: `--mine` finds the strongest
instance of each detectable situation in an archive — the highest viaduct, the
deepest bore, the tile holding both a deck and a portal — as a superlative
rather than a threshold, so it always returns *the* worst case the data holds.
Coordinates chosen any other way are a guess about someone else's terrain.

Seven of the fourteen are minable from the archive today. The other seven are
listed in the file's `unsited` block with the reason each needs a hand or a
decoder that does not exist yet, rather than being given a plausible coordinate.

Re-mine after retiling a different extract; the sites are extract-specific.

## 9. Baselines

`server/verify/baseline-montreux-z16.json` is the committed scorecard for
`data/overture-ch/preview.arpa` at z16. It is not a statement that the scene is
correct — it records what was true when it was written, including the deviations
that are known and accepted. Its job is to make the *next* change legible.

A metric that regressed exits 1. A metric present in the baseline and absent
from the run also reports, because a check that stopped running looks exactly
like a check that passed.

Regenerate with `--json`, and say in the commit message which numbers moved and
why.

## 10. Adding a check

A check is one file in `server/src/verify/checks/`, implementing `visit` and
`finish`, plus a line in `checks::run`. The friction is deliberately low: a
defect found by eye should become a permanent measurement in the same sitting.

Before believing a new check's first number, measure its *anatomy* — histogram
the population it is scoring and look for a second mode. Three of the metrics
here changed shape after that step, and one was measuring something else
entirely.
