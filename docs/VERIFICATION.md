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

The `inv` column is `GENERATION.md` §7's predicate, and it is a type in the code
(`verify::Invariant`) rather than a hand-typed integer — which is how
`slope.road_grade` came to claim invariant 6 while §8 gives the grade ceiling to
I2. Every metric also states its own population; the table below is the summary.

| metric | inv | what it means |
|---|---|---|
| `contact.kerb_lip` | I4 | Carriageway edge height minus the drawn ground a metre outside it. Not a defect on its own — it is how tall a wall the model implies, and a road on a real embankment has a real drop at its edge. It is what still sees a road standing on an embankment nobody built. |
| `contact.kerb_unwalled` | I4 | The part of that drop with no apron face spanning it. This is the gate: the lip is the wall's height, this is how much of the wall is missing. |
| `contact.deck_seat` | I4 | How far a *fitted* deck's lower abutment stands below the wall beside it, bounded by its own far end. A footbridge whose span edge landed part way down a gorge wall begins in the riverbed it crosses. Both halves of the rule earn their keep: without the wall test a bridge landing at the foot of a slope reads as buried, and without the bound a level footbridge on a hillside scores the hillside. |
| `contact.deck_carried` | I4 | How far a *fitted* deck sinks below the solved deck running alongside it, over the population of fitted decks that one is actually carrying. The sample count is half the measurement: an ordinary footbridge is not a weak instance of this and is not counted, so a rule that puts sidewalks on their bridges empties the population rather than flattening it — 16 carried decks before `synth::carried`, 1 after. |
| `contact.rail_standoff` | I4 | Drawn rail formation minus the drawn ground directly beneath it. Every other I4 contact metric is anchored on asphalt, and a rail class paves none — no kerb, no hole, no apron — so a railway standing in the air was measured nowhere. |
| `order.deck_above_carriageway` | I3 | Deck running surface minus the at-grade asphalt sharing its plan position. Negative past the touchdown band means the level ordinal inverted. |
| `clearance.deck_over_ground` | I4 | Deck soffit minus the terrain beneath it. Past a deck thickness, the deck ploughs into the hillside. |
| `clearance.bore_cover` | I4 | Terrain minus the bore roof. Negative past a portal mouth means the tube is in open air. |
| `seam.terrain_step` | I2 | Spread of the heights two adjacent tiles derive for the same border lattice point. |
| `seam.terrain_split` | I2 | Spread between coincident vertices *inside* one tile: the ground cracked open. |
| `seam.pavement_step` | I2 | Border disagreement, for the at-grade road surface. |
| `seam.abutment_plan` | I2 | Plan distance between the two road-stroke ends that meet at a bridge abutment or tunnel portal. `Corridor::pieces` cuts the approach and the span from one shared coordinate, so the correct answer is zero and there is no prior to argue about — anything here is a generator having moved it. What it catches: the approach and the structure ride different curves (the band is buffered around the raw corridor nodes, a structure is swept along the smoothed sweep line), which puts the deck beside its own road and starts it short of or past the abutment. |
| `seam.abutment_step` | I2 | Height difference between the same two ends: the deck ramp must arrive at the road it launches from. Separated from the plan break because either occurs alone, and because a sweep line that has slid *along* the alignment produces both at once — the deck then carries the height solved for a different station, worth the slide times the grade. |
| `seam.abutment_bare` | I2 | Bare ground between an abutment and the at-grade band that continues it, marched along the approach's own direction. A third quantity with a third cause: the strokes are cut at the exact span arc, but the band was assembled from whole mapped segments, so it ended at a vertex while the deck began at the boundary. The hole that leaves is half a segment wide and a mapper's vertex spacing decides how big. |
| `seam.band_deck_bare` | I2 | Drawn ground between the at-grade *surface band* and the bridge deck that continues it, marched out along the band's own silhouette. The surface half of `seam.abutment_*`, and the only one of the two that can see a road at the zooms where the road surface exists: from `ROAD_SURFACE_MIN_ZOOM` a carriageway's own stroke is deleted, because the union paves it (`pipeline::paves_via_union`), so every abutment sample the stroke check takes at z16 is a railway. The modality also decides whether the defect is *visible*: a railway keeps its stroke and draws that ribbon over both the ballast band and the deck, hiding the joint underneath it, while a road has nothing over the joint and shows every millimetre of it. |
| `seam.band_deck_step` | I2 | Height across that same joint. The two sides are fitted by different machinery and nothing makes them meet: the band's vertices come from the height field over `Profile::road_at_arc`, the deck's from `Profile::deck_at_arc` — a ramp fitted to the middle two thirds of the structure run (`fit_ramp` trims a sixth at each end) and written onto the structure nodes only, so the two series disagree across the boundary edge by whatever the fit's residual is there. |
| `seam.handover_kerb` | I2 | Share of those joints wearing a `road_casing` rim — a kerb line drawn straight across the carriageway a third of a metre before the bridge. The rim edges the paved surface against the ground it stops at, and at a handoff it stops at nothing. Its sample is 1 or 0, so the `over` column is the share. The residual after the fix is not noise: two thirds of it sits on joints `seam.band_deck_bare` also calls broken, where the band's edge genuinely is not at the span boundary and a kerb there is right. |
| `order.at_grade_overlap` | I3 | Vertical separation where two level-0 paved regions share a plan position with nothing to order them. |
| `order.grade_stack` | I3 | Vertical separation between two at-grade surface bands at one plan point, measured whole-mesh (the overlap metric above sees border vertices only). At grade means on the ground, and there is one ground: past 3 m — the same boundary `crossings::SEPARATION_M` draws from the model side — the upper band is in the air over the lower with no structure between them. The class it exists to keep dead: a mapped bore's still-buried tail paved as open cut, sliding beneath the band of the feature crossing just past its portal (the Collonge funicular over the rack railway). |
| `slope.terrain_face` | I6 | Rise over plan run of every terrain triangle spanning ≥10 cm. Finds manufactured retaining walls. |
| `slope.carriageway_face` | I6 | The same for interior asphalt, excluding the kerb rim. |
| `slope.road_grade` | I2 | Rise over run between consecutive vertices of a drawn drivable centerline. Measured *along* the road, which the carriageway mesh cannot answer: a clearance lift dropped on one node is a spike the face metric reads as ordinary cross-fall. |
| `slope.terrain_tearing` | I6 | How far a terrain vertex stands off the plane of its neighbours, counted only where opposite breaks flank it on both sides. Separates a wall from a wall drawn as teeth, which no steepness can. |
| `paint.marking_offside` | I4 | Plan distance from a painted marking to the asphalt it is painted on, zero when it is on it. Contact in *plan*: every other check reads a height, and a marking that has lost its lateral registration is still perfectly draped, so every height check reports it clean. |
| `paint.edge_line_inset` | I4 | How far an edge line's near kerb sits from where the cross-section puts it (0.30 m inset inside a carriageway buffered by a 1.0 m shoulder = 1.30 m). Edge lines are the one painted line the archive can name — 0.15 m is their width and nothing else's — so they are the one place a lost lateral offset is a number rather than a shape. Paint projected onto the road's axis reads half a carriageway here. Centre lines and lane dividers are deliberately outside the population: both are 0.12 m, and the archive cannot tell a divider that belongs a third of the way across from a centre line that belongs in the middle. |
| `lod.structure_drift` | I5 | Structure height at one zoom against the same structure one rung coarser. |

### The model half

Three invariants are not about the emitted scene at all. They are about *how it
was computed*, and no amount of geometry can tell a scene where authority held
from one where it was violated and the numbers happened to come out plausible.
§8 of `GENERATION.md` says so directly: I7 and I8 "are established by
construction and falsifiable by a single perturbation experiment, not sampled by
a metric".

Those checks run in process, against the model, and are written out with
`arpentry_tiler --verify-model <path>`; `arpentry_verify --model <path>` merges
them so one table and one baseline cover both halves.

| metric | inv | what it means |
|---|---|---|
| `solve.determinism` | I5 | The same scene solved twice, compared bit for bit. Non-zero means a height depends on an iteration order or a thread interleaving, and every guarantee below rests on it not doing so. |
| `authority.inversion_R` | I7 | Stratum R re-solved with every junior corridor deleted from the scene, the burial licenses held at the full scene's values (the plan skeleton is input, §7 I7). Any senior height that moved is an authority violation. |
| `authority.inversion_S` | I7 | The same for S. |
| `ground.footprint` | I8 | Every layer of the ground stack against the one beneath it: where a layer moved the ground, its own declared footprint must cover the point. |
| `structure.bore_daylight` | I3 | The crossing premise, measured. A crossing over a mapped bore buys no clearance (§4.5), which stands on the premise that the bore passes beneath the ground the crossing feature rides on. At every plan crossing of a mapped tunnel span by an alignment annotated above it — the same gate that seeds the solver's burial ceilings — the bore's roof plus cover against its own terrain, signed. Positive is a bore daylighting through a roadbed: the waiver stood on nothing, and the two bands draw a storey apart with neither a bore nor a deck between them. Archive-side only `contact.kerb_lip` can see this class, because a dismissed tunnel paves no band for `order.grade_stack` to catch. |

The perturbation checks are the only ones that verify the *design* rather than
the output, and they cannot be passed by luck. They are opt-in because they
re-solve the scene: on the Montreux extract the model half costs about as long
again as the tiling.

They are also the easiest checks in the harness to make vacuous, so each states
what it actually exercised. `ground.footprint` reports how many of its probes
found a layer moving the ground *inside* its footprint — the population the
predicate is about — because a run where no layer moved anything would score a
perfect zero and prove nothing. `authority.inversion` skips, rather than passes,
where the extract holds no junior stratum to delete.

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
- A **mountain above a footbridge is not a defect.** The first version of
  `contact.deck_seat` asked how far the ground beyond an abutment climbs above
  it, and on a Montreux hillside that scores the hillside: over the extract's
  220 abutments the ground's own outward grade is p50 9 %, p75 32 %, p95 83 %,
  so a plain rim search finds the mountain on half the population and reports
  20 m for a level footbridge crossing its own gully. Two filters make the
  measurement mean what it says — only ground steeper than 60 % is followed
  (below that is a slope the path walks), and the climb is bounded by the
  deck's *own far end*, which is the only evidence in the archive of how high
  the ground comes at the span's edge. What is left is a deck tilted because one
  end fell down a wall: 12 of 115 fitted decks, against 60-odd that a rim search
  called broken.
- A **path under a viaduct is not its sidewalk.** `contact.deck_carried` asks
  which fitted decks a solved deck is carrying, and the tempting answer — the
  ones running close alongside it — is wrong on 3 of the 25 the Montreux extract
  offers, the worst being a 12 m footbridge over a stream *underneath* a
  motorway whose deck is 68 m overhead. What separates them is that a sidewalk
  **joins** its bridge: it arrives at the same abutment, so wherever the
  annotation and the DEM agree at one end the fitted chord already lands on the
  deck, while a path passing under one never touches it at either end. Two other
  tests carry their own weight — the decks must be near-parallel, since with a
  10 m lateral reach a short footbridge *crossing* a road bridge still reads as
  79 % alongside it; and the shared run must be most of the span, which rejects
  three long walkways that ride a bridge for part of their length and carry
  themselves for the rest.
- **Take the threshold from the gap, not from the middle.** The join ceiling
  above was first set at a metre, reasoning from the population's median
  (0.49 m). Sorted, the population says otherwise: seventeen candidates meet
  their carrier within 1.91 m, then nothing at all until 4.69 m, then the three
  passing underneath. A metre cuts straight through a dense cluster — 1.16,
  1.26, 1.45, 1.76, 1.77, 1.85, 1.91 — and it showed, leaving six sidewalks
  hanging under their own bridges that a ceiling anywhere in the empty band
  claims. A median says where a population sits; only the sorted list says
  where it can be cut.
- **Two instruments, one population.** The lateral reach is the one constant
  here that needed no judgement. `examples/carried_probe` searched the *scene
  model* out to 25 m and found every carried path between 2.0 m and 9.5 m of a
  solved centerline, with nothing at all in between 9.5 m and 25 m; a 4 m gap
  and a 16 m gap claim the same 25 spans. Reading the *emitted archive* instead,
  against the drawn deck surface rather than the centerline, `contact.deck_carried`
  finds 25 candidates too. A threshold that lands in an empty band, confirmed
  from both ends of the pipeline, is one that will not need revisiting.
- **Under the formation there is no legitimate gap.** `contact.kerb_lip` probes
  a metre *outside* a carriageway, where a road on a real embankment has a real
  drop, so it is a size and not a verdict. `contact.rail_standoff` asks under
  the track, which the bench is supposed to have raised to meet it, so any
  positive answer is air. The threshold is therefore only as wide as the mesh's
  own resolution, and it is read off the classes that do bench: `standard_gauge`
  benches at 98.9 % of its at-grade nodes and runs p95 0.80 m, p98 1.45 m, p99
  1.62 m, then jumps to 5.52 m at p999, with 0.12 % of its samples in the 2–4 m
  bin. The level-0 *road* strokes, which take the same drape path, reach 1.27 m
  at p999. Two metres sits in that empty band. What it costs is one exclusion
  that has to be made or the metric measures the wrong thing: a structure span
  emits its paint stroke *before* the level ordinal is attached to the
  properties (`pipeline.rs`), so a viaduct's stroke arrives at level 0 and
  metres up — 19,993 vertices on the Montreux extract — and is dropped by asking
  whether a drawn structure surface lies within a metre of it.
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
- **At an abutment there is no contact band at all**, which makes the
  `seam.abutment_*` family the one place in this list with nothing to read off a
  distribution. The approach and the span are cut from a *single coordinate* in
  the model, so the two ends are the same point and any distance between them is
  a generator having moved one. The threshold is therefore the format's own
  resolution — 5 cm, just above the ~2.6 cm two independently quantized
  `uint16` vertices can differ by — and it is a floor rather than a tolerance.
  What *did* need care is the **pairing**, which is where a check like this goes
  wrong. Three rules earn their place, each because it was measured without
  them: the two ends must carry the same `class`; they must leave the cut back
  to back within 0.6 rad, or a footway ending beside a bridge pairs with it and
  the angle between two roads is reported as a break in one; and *fitted* decks
  are excluded entirely, because `synth::draped` deliberately walks a
  footbridge's abutment along the bank to seat it (`contact.deck_seat`) — the
  shared-coordinate premise does not hold for them, and including them reported
  the seating rule working as an 11.9 m defect. Carried-ness is read from the
  drawn deck or bore rather than from the stroke's `level`, for the reason the
  rail-standoff note above gives: a solved structure's own paint arrives at
  level 0, so a `level` test sees only the fitted footbridges and misses every
  solved bridge in the archive.
- **The band does not draw its own edge.** The trap that decides whether
  `seam.band_deck_bare` means anything. `road_surface` is an *inset* of the
  paved region — the outer `PAVE_RIM_M` (0.35 m) is a separate `road_casing`
  feature (`synth::pave_mesh`) — so a march that stopped at the interior would
  report a third of a metre of bare ground at every abutment in the extract, a
  floor no change could ever remove. The march anchors on the interior's edge
  but counts interior and casing alike as surface, and reports the distance
  between the last drawn surface and the first drawn deck. Its threshold is the
  instrument's own resolution rather than a tolerance: two 0.1 m march steps,
  which also covers the lattice under two independently encoded surfaces.
- **A road does not hand over to a railway.** The pairing rule for the same
  check, and the one that had to be measured to be believed. The union
  *dissolves* road identity — a `road_surface` region is every carriageway that
  touched it — so the only evidence left that a band and a deck are the same
  feature is what the two are made of. Pairing on geometry alone, a street
  ending near a railway viaduct pairs with it: on the Montreux extract that was
  a third of everything the metric called a gap, asphalt bands handing over to
  `narrow_gauge` and `funicular` decks. Requiring the modalities to match cost
  32 of 886 samples and removed a quarter of the violations with them: the
  discarded pairings were almost all in the tail, which is the signature of a
  false pair rather than of a defect.
- **The skipped samples are half the evidence.** `seam.band_deck_bare` counts
  edges where a deck starts past the band; it *skips* edges where the deck
  already covers the band, as an overlap rather than a handoff, and until those
  were printed (`ARPT_DEBUG_OVERLAP`) the gap looked like the band being short.
  It was not: 40 of the 43 gapping joints had a skipped, overlapping edge within
  12 m of the same cap, which is a joint cut at the wrong *angle* rather than in
  the wrong *place* — and no distribution of the gaps alone could have said so.
  When a metric drops samples on a rule, count them and look at where they land;
  a defect that is signed will hide half of itself in the exclusion.
- **A band metres below is another road, not this one's continuation.** The
  second pairing trap in the same check, and the one that produced its most
  alarming early number. The march looks for where the drawn surface *stops*,
  and at Montreux station it walked off the upper of two at-grade bands standing
  13 m apart (`order.grade_stack`) onto the lower one — which both closed the
  gap it was measuring and gave `seam.band_deck_step` the wrong height to
  compare the deck against, reporting 13.06 m of step at a joint it had just
  measured 0.10 m wide. Surface only counts as this band continuing when it is
  within a slab of the edge's own height, which is also what a real
  continuation can vary by over the reach at the steepest grade a carriageway
  takes. The metric's ceiling is the honest consequence: about 3.5 m, past which
  a joint has come apart rather than stepped, and `order.deck_above_carriageway`
  owns it.

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
  `order.at_grade_overlap`, and the interior-burial probe that has since retired
  — turn out to be one defect seen from three angles.
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

### The mass-aware clearance projection

`GENERATION.md` §4.4 says a correction *"distributes by inverse mass"*, and
`clearance_pass` is the one projection that never asked: raise-only, it always
lifts the upper side. Where the data says a road tunnels *under* another it
therefore lifts the road above instead of sinking the bore, which is why 10 % of
annotated tunnel nodes end at or above the ground.

Splitting the correction by inverse mass — the light (structure) side spending
first, the heavy (ground-pinned) side covering whatever separation remained —
was tried and **rejected by measurement**.

| attempt | result |
|---|---|
| Split the correction in proportion to inverse mass | The inequality stops being guaranteed. Neither side alone satisfies it, and where the light side cannot actually move the demand is simply never met: a 6.5 m fixture settled at **2.0 m**. |
| Spend the light side first, then ask the heavy side for the remainder | Better in the fixture (2.0 → 3.8 m) and far worse on the extract: **clearance shortfall 51.93 → 293.61 m**, a five-fold regression of a hard invariant, with `structure.annotated_lost` also up. |

**Why it fails is geometric, not arithmetic.** A bore is chorded between its
at-grade anchors, so it can only sink as deep as those anchors can be ramped
down to, and the ramp is bounded by the class grade: 6.5 m at 6 % needs ~97 m of
approach. Anchors further out than the ramp reaches hold the chord at grade and
`rigidity_pass` undoes the dip. Reallocating the correction does not change what
the geometry allows; it only takes the guarantee away from the side that could
have met it.

What the rule needs is for the dip and the lift to be solved *together* to
feasibility, rather than allocated between them in one pass — the light side
first is the right instinct and a two-line reallocation is not enough machinery
to keep §4.4's Strong constraint strong. Recorded so nobody spends a day
rediscovering that the principled version of this is the easy part.

**The third attempt landed, but only where the data says a road goes under**,
and what it cost to get there is the part worth keeping. Three things were
needed, and each was found by a measurement rather than by reasoning:

| step | what it fixed | what said so |
|---|---|---|
| A bore yields to a clearance ceiling inside `project_spans` | The dip survives rigidity. A deck is a beam and its chord *is* the constraint; a bore is a hole, and an urban underpass (S6) runs below the chord of its own portals. | The fixture: 6.5 m demanded, 6.5 m delivered, the street above raised 0.26 m. |
| The closing settle stays raise-only | I3 holds at the output whatever the lower side managed. | The rejected version's 293.61 m shortfall, which this reproduces exactly when the settle also splits. |
| The dip fires only where the lower side is in a **bore** | Everything else. | See below. |

Letting *any* peer yield downward — which is what §4.4 says, read plainly —
turns every corridor already off its own datum into a **pump**. A rack railway
solved hundreds of metres below its terrain manufactures a deficit at every
crossing; the dip spreads along it; `grade_pass` drags the crossing's own
reference down with it; the deficit reopens; and 96 sweeps later the railway is
290 m under the ground and the clearance shortfall reads 289.76 m against
58.94 m. Blocking the three variables the demand is read from does not stop it,
because grade drags them; clamping each dipped node against its own terrain does
stop it and cuts a sawtooth instead (`slope.rail_grade` 303 %).

So the general rule stays unimplemented, and it now has a **named
prerequisite**: `datum.float`. A two-sided correction cannot be safe while some
corridors sit hundreds of metres off their own datum, because those are exactly
the ones whose demands are nonsense. Held to the runs the data calls tunnels,
the correction goes where §4.5's prior points and nowhere else: `bore_cover`
violations 14.14 → 13.42 %, `kerb_lip` 13.49 → 13.45 %, 463 m of annotated
tunnel recovered, and every extreme in the scorecard unmoved.

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
