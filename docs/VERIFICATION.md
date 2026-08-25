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
# Judged on how often the defect happens, then how bad its tail is; a lone
# moved extreme prints as `outlier only` and does not gate (§9.1).
arpentry_verify preview.arpa --baseline server/verify/baseline-montreux-z16.json

# The archive half only measures what was drawn. Merge the model-side checks
# (I5, I7, I8) so one baseline covers both, and re-cut it the same way.
arpentry_tiler ... --verify-model data/overture-ch/preview.arpa.model.json
arpentry_verify preview.arpa --model data/overture-ch/preview.arpa.model.json \
    --baseline server/verify/baseline-montreux-z16.json

# And the rungs the detail zoom cannot see: everything per-zoom is correct at
# z_ref by construction, so a broken correction only shows out here (§9).
arpentry_verify preview.arpa --zoom 13,14,15 \
    --baseline server/verify/baseline-montreux-coarse.json

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
| `contact.kerb_lip` | I4 | Carriageway edge height minus the drawn ground a metre outside it. Not a defect on its own — it is how tall a wall the model implies, and a road on a real embankment has a real drop at its edge. It is what still sees a road standing on an embankment nobody built. To tell those two apart, census the profile side with `cargo run --release --example hang_census`: it groups every at-grade run standing past `ABSORB_STANDOFF_M` by what its ends touch — its own structure span, a junction inside somebody else's deck, another hanging corridor, or nothing. Over Montreux the second group was 40 m in two service stubs held in the air on Route de Chernex's bridge, which is a structure the model owed and now draws; the third was 275 m of terrace whose walls `contact.kerb_unwalled` says are all present, which is S13 and correct. |
| `contact.kerb_unwalled` | I4 | The part of that drop with no apron face spanning it. This is the gate: the lip is the wall's height, this is how much of the wall is missing. |
| `contact.deck_seat` | I4 | How far a *fitted* deck's lower abutment stands below the wall beside it, bounded by its own far end. A footbridge whose span edge landed part way down a gorge wall begins in the riverbed it crosses. Both halves of the rule earn their keep: without the wall test a bridge landing at the foot of a slope reads as buried, and without the bound a level footbridge on a hillside scores the hillside. |
| `contact.deck_carried` | I4 | How far a *fitted* deck sinks below the solved deck running alongside it, over the population of fitted decks that one is actually carrying. The sample count is half the measurement: an ordinary footbridge is not a weak instance of this and is not counted, so a rule that puts sidewalks on their bridges empties the population rather than flattening it — 16 carried decks before `synth::carried`, 1 after, and **0 once the coverage test stopped charging a short sidewalk for its bridge's annotation slack**. An empty population here is the success condition, not a skipped check. |
| `contact.rail_standoff` | I4 | Drawn rail formation minus the drawn ground directly beneath it, walked on the rail stroke. Coverage limit: from `ROAD_SURFACE_MIN_ZOOM` the union paves the formation and deletes the stroke, so at the surface zooms this population is empty by construction and the metric reports its skip — it measures pre-surface rungs only. The residue it used to catch there (rail whose ballast band failed to mesh) is unmeasured until the formation-coverage check on the roadmap lands. |
| `order.deck_above_carriageway` | I3 | Deck running surface minus the at-grade asphalt sharing its plan position. Negative past the touchdown band means the level ordinal inverted. Sharing a plan position is not the same as crossing, and the archive cannot tell them apart: a street whose centerline sits at a gorge rim overhangs the slot by half its band, so a footbridge legitimately drawn over the streambed ten metres below reads here as an inversion. Before treating a cluster as a defect, cut a section along the *lower* feature's own span and check whether it is spanning something — at Chauderon the 5 m² patch that holds this metric's worst is a 17 m footbridge over a stream at the bottom of the gorge, and `contact.kerb_unwalled` says the wall above it is drawn. |
| `clearance.deck_over_ground` | I4 | Deck soffit minus the terrain beneath it. Past a deck thickness, the deck ploughs into the hillside. |
| `clearance.bore_cover` | I4 | Terrain minus the bore roof. Negative past a portal mouth means the tube is in open air. Read the population before reading the rate: censused over the Montreux zone (`cargo run --release --example bore_census`), 84 % of every exposed metre of tube lay within 25 m of the end of its buried run and 58 % within ten — the portal transition, which the daylighting cut trenches on purpose, so most of what this metric counts is drawn deliberately and cannot go to zero. The family worth chasing is the *interior*: tube standing proud in the middle of a run, which was 118 m of the whole extract in two stretches. `synth::structure::drawn_runs` withholds it, so the rate moves when that family does; a rate that moves with the mouths instead is a portal change, not a cover change. |
| `seam.terrain_step` | I2 | Spread of the heights two adjacent tiles derive for the same border lattice point. |
| `seam.terrain_split` | I2 | Spread between coincident vertices *inside* one tile: the ground cracked open. |
| `seam.pavement_step` | I2 | Border disagreement, for the at-grade road surface. |
| `seam.abutment_plan` | I2 | Plan distance between the two road-stroke ends that meet at a bridge abutment or tunnel portal. `Corridor::pieces` cuts the approach and the span from one shared coordinate, and both strokes are snapped onto the same smoothed sweep line, so the correct answer is zero and there is no prior to argue about — anything here is a generator having moved one of them off that line. Two pairing rules matter more than the geometry: an end is *carried* only where its height sits at the solid's road face (a deck's top, a bore's floor plus the deck thickness — inside the solid's vertical range is an underpass, not a carry), and the partner is the *nearest* aligned end with the second half of a flush both-carried pair contributing nothing. Before those rules, every top offender was a 0.00 m joint scored as 7–11 m: the leftover half of a flush pair reaching the far end of its own short span, or the parallel track. From `ROAD_SURFACE_MIN_ZOOM` the union paves carriageway and rail formation alike and both strokes are deleted, so at the detail rung this population is empty by design and the check reports its skip; the surface handoff lives in `seam.band_deck_*`. |
| `seam.abutment_step` | I2 | Height difference between the same two ends: the deck ramp must arrive at the road it launches from — which `deck_ramp` now pins at every anchored span boundary, so a step is a boundary the pin does not reach or a sweep line that has slid *along* the alignment (the deck then carries the height solved for a different station, worth the slide times the grade). Separated from the plan break because either occurs alone. |
| `seam.abutment_bare` | I2 | Bare ground between an abutment and the at-grade band that continues it, marched along the approach's own direction — measured only against an *uncarried* partner, so at the surface zooms its population is empty by construction now: a flush approach's end vertex lies under the deck's end cap and reads as carried, making every joint a span-meets-span pair. The band-side gap it used to see lives in `seam.band_deck_bare`, which covers ballast as well as asphalt. |
| `seam.band_deck_bare` | I2 | Drawn ground between the at-grade *surface band* and the bridge deck that continues it, marched out along the band's own silhouette. The surface half of `seam.abutment_*`, and the only abutment instrument at the zooms where the road surface exists: from `ROAD_SURFACE_MIN_ZOOM` the tiler deletes the stroke of every class the union paves — carriageway and rail formation alike (`pipeline::paves_via_union`) — so the stroke check has no detail-rung population at all. Neither modality hides the joint any more: the rail track ribbon that used to be drawn over both the ballast band and the deck is deleted with the carriageway's stroke, so a ballast band must meet its deck as nakedly as asphalt does. |
| `seam.band_deck_step` | I2 | Height across that same joint. The band's vertices come from the height field over `Profile::road_at_arc`, the deck's from `Profile::deck_at_arc` — a ramp fitted to the middle two thirds of the structure run whose ends `deck_ramp` pins back to the road at every anchored boundary, so at the span arc the two are one number by construction. Pairing is by the deck's *top* and by modality: a band at soffit level is passing under a low bridge (that clearance is `order.grade_stack` material), and the surface that continues a ballast band is ballast — without the modality test the march walks the step's reference down the asphalt of the road approaching its own underpass. |
| `seam.handover_kerb` | I2 | Share of those joints wearing a `road_casing` rim — a kerb line drawn straight across the carriageway a third of a metre before the bridge. The rim edges the paved surface against the ground it stops at, and at a handoff it stops at nothing. Its sample is 1 or 0, so the `over` column is the share. The residual after the fix is not noise: two thirds of it sits on joints `seam.band_deck_bare` also calls broken, where the band's edge genuinely is not at the span boundary and a kerb there is right. |
| `order.at_grade_overlap` | I3 | Vertical separation where two level-0 paved regions share a plan position with nothing to order them. |
| `order.grade_stack` | I3 | Vertical separation between two at-grade surface bands at one plan point, measured whole-mesh (the overlap metric above sees border vertices only). At grade means on the ground, and there is one ground: past 3 m — the same boundary `crossings::SEPARATION_M` draws from the model side — the upper band is in the air over the lower with no structure between them. The class it exists to keep dead: a mapped bore's still-buried tail paved as open cut, sliding beneath the band of the feature crossing just past its portal (the Collonge funicular over the rack railway). Read the rate against the population, which is tiny — a few hundred samples over a city extract, because two at-grade bands almost never share a plan position — so a *rate* near 90 % says the population is nothing but the defect, and closing a whole site removes samples rather than moving the rate. Censused with `cargo run --release --example stack_census`, which walks the same burial license the solve reads and reports, per licensed metre, whether the reconciled partition still calls it a tube: that split (62.5 m taken back by the clamp, 21.6 m never annotated) is what named the mouth family and sized its two halves. Its third table is the deck half of the same fact, and the one that named scenario S17: 23 decks over the Montreux zone were shorter than the crossing they carried, by 111 m in total, which `crossings::carried_crossings` now closes. What is left after both halves is a *junction* residue — the license is one crossing's band, while the drawn surface where several roads meet is the union of all of them, so a rail tail can still surface under the plate at Veytaux. |
| `slope.terrain_face` | I6 | Rise over plan run of every terrain triangle spanning ≥10 cm. Finds manufactured retaining walls. |
| `slope.carriageway_face` | I6 | The same for interior asphalt, excluding the kerb rim. |
| `slope.road_grade` | I2 | Rise over run between consecutive vertices of a drawn drivable centerline. Measured *along* the road, which the carriageway mesh cannot answer: a clearance lift dropped on one node is a spike the face metric reads as ordinary cross-fall. |
| `slope.terrain_tearing` | I6 | How far a terrain vertex stands off the plane of its neighbours, counted only where opposite breaks flank it on both sides. Separates a wall from a wall drawn as teeth, which no steepness can. |
| `water.descends` | I4 | Ascent of the drawn terrain above its running minimum, walking every flowing-water centerline (river, stream, canal) along flow — the ponding depth a viewer's river would need to pass the point, so a long gentle false climb reads as one defect, not fifty steps. Flow direction is inferred per part from the net drop (a reversed line cannot read as a climb); samples over the pavement hole are skipped with the minimum carried across, since a stream does not forget its level under a culvert. Flowing water is drawn draped, so this is the H stratum's watercourse half measured where it is visible: the ground the client drapes the river on. Threshold 1.0 m, clear of the measured noise band (p95 = 0.40 m: lattice plus a mapped line off its thalweg); the tail above it is a deck or embankment the DTM baked in across the water, or a gorge wall sampled where the line leaves the channel — the population the monotone conditioning (GENERATION.md §4.2 H) must move. |
| `paint.marking_offside` | I4 | Plan distance from a painted marking to the asphalt it is painted on, zero when it is on it. Contact in *plan*: every other check reads a height, and a marking that has lost its lateral registration is still perfectly draped, so every height check reports it clean. |
| `paint.edge_line_inset` | I4 | How far an edge line's near kerb sits from where the cross-section puts it (0.30 m inset inside a carriageway buffered by a 1.0 m shoulder = 1.30 m). Edge lines are the one painted line the archive can name — 0.15 m is their width and nothing else's — so they are the one place a lost lateral offset is a number rather than a shape. Paint projected onto the road's axis reads half a carriageway here. Centre lines and lane dividers are deliberately outside the population: both are 0.12 m, and the archive cannot tell a divider that belongs a third of the way across from a centre line that belongs in the middle. **Read the population before the rate.** Edge lines exist only on motorway and trunk carriageways, so over the Montreux extract this is a *one-road* metric — every one of its 2,474 violations lies on the A9, in six 0.01° buckets. And most of them are not "a line off its inset": the A9's corridor is the 9.0 m prior with no data width, so its band reaches 5.5 m from the axis, yet 74.7 % of the violations report a nearest kerb further than that — 1,391 of them at *exactly* 6.95 m across four separate sites. A constant that sharp is a structural distance, not scattered misregistration: `near_kerb` marches across whatever asphalt is there, and where the union has paved a carriageway together with its neighbour or its link the march runs past the line's own kerb to the far edge of the merged region. Splitting that term from a genuinely lost lateral offset needs an instrument that can see the *band* a vertex belongs to, which neither this check nor the archive has today. |
| `paint.buried` | I4 | How far under the drawn terrain a stroke vertex at level ≤ 0 sits. The client strokes lines as decals depth-tested against the ground, so buried paint is invisible where it works and drawn across the mountain wherever a coarse rung's chords disagree with the buried run. The mode it keeps dead is a tunnel span's own paint — the stroke, its markings and its rail heads all stop at the portal now (`pipeline::process_feature`), where they used to ride the bore's road surface (4.2 % of z16 stroke vertices, worst 592 m under). Positive levels are excluded (a bridge stroke rides its deck by design), and a vertex over the pavement hole or a portal cut has no ground overhead and contributes nothing. The residue above the metre gate is the portal-mouth approach passing under the cut face that climbs over the bore — ground that is really there — at ~0.03 % of samples. |
| `order.building_overlap` | I3 | How far drawn at-grade asphalt reaches inside a building footprint, over every drawn at-grade road surface sample — zero everywhere outside one, so the rate is the share of the drawn street surface standing in a building and not a rate over the defect itself. A road's width is a class prior, a footprint is surveyed, and nothing reconciles them: the band is laid at the prior's half-width whatever is standing there. Levels order themselves out (a bridge or a bore is not level 0), and rail is excluded because a station roof over its platforms is a level relation the archive cannot state. **Depth separates two families and only one is a width problem.** The shallow mass is a band overrunning a facade, which capping the band against the room removes; the deep mass is a way whose *centerline* runs inside a footprint — a parking structure's internal service ways, the Casino Barrière's 7,533 m² footprint with an unknown-class way through it — which no cap can move. That is why the threshold is low and reasoned rather than cut at the gap the population does show: the gap (only 2.8 % of inside samples in the whole band from 3 m to 5 m) sits between the two families, and gating there would gate on the unfixable half. Read it against a scoped run to see the shallow family alone: at Rue du Marché the whole tile reads 8.3 % inside with a worst of 3.9 m and nothing past 5 m. |
| `contact.sidewalk_grade` | I4 | How far the drawn surface an attached pedestrian way stands on departs from the drawn carriageway at its nearest kerb point. A street's bench reaches its half-width plus a shoulder and a margin and stops, so a way outside that band drapes on whatever the hillside does. Unsigned, and the measurement is why: the population is 44.9 % below the kerb and 55.1 % above it — the fill side and the cutting side of one missing cross-section — so a signed metric would report half the defect and call the other half margin. **Attachment is the whole instrument.** Proximity at a point is not attachment: taken on proximity alone the population is 117 k samples reaching 12.4 m above the kerb and 14.2 m below, which is hillside paths passing near roads. Requiring the *part* to run with a street for 80 % of its length (the cut the plan-space census read off the tagged `subclass='sidewalk'` population) and to be locally parallel within 30° takes it to 30 k and the worst from 14.2 m to 10.2 m. Steps are excluded, for the reason `slope.road_grade` excludes non-drivable classes: a staircase changes height beside what it runs along on purpose. Two limits: the archive carries a way's class but never the sidewalk tag, so the third of tagged sidewalks that fail a geometric test are outside the population; and past a few metres the tail is a street on a terrace with a path along the foot of its wall, which is `contact.kerb_lip`'s question. Read the body, not the extreme. | **Population is a kerb-anchored strip, not a class (2026-08-25).** The band half of this check used to select its samples by drawn material (`walk_surface`/`walk_casing`), which was a proxy for "belongs to a street" and stopped working the moment a path and the sidewalk it continues became one drawn region. It now takes every drawn pedestrian sample within `WALK_WIDTH_M + 1 m` of a kerb: a strip is seated with its inner edge on the carriageway edge (`street.kerb_join` reads 0.00 %) and allotted at most one band-width, so every sample of one qualifies, and a path that merely passes a road does not — the worst site the class test admitted was a footpath 17.7 m up a slope. Read from the geometry, the population survives the material merge; read from the class it did not, going 0.43 % to 8.42 % on the first attempt at it.
| `contact.walk_rim` | I1 | The step where a drawn pedestrian band meets the ground **at the terrain's own hole rim** — the band's region is what cut that hole, so nothing but quantization stands between them and the honest zero is a joint, not a contact band. Read at the rim rather than a metre outside it (which is where `contact.kerb_lip` reads a carriageway's) because a metre out lands on the batter face, whose slope is legitimate: it cannot separate a joint that holds from one that does not. Unsigned — the ground standing above a band is the same missing earthwork as the ground falling away from it. **Two populations, and the split is the finding**: before the walkway bench the sidewalk half read p50 0.19 m, p95 1.85 m, worst 15 m, because a sidewalk is seated on its host's cross-section while the ground under it is still the street's bench or the batter beyond it; the path half read p50 0.01 m, because a path *is* the drawn ground. `ARPT_DEBUG_STREET=1` prints both. A rim with no band over it is a carriageway's and is `contact.kerb_unwalled`'s to score; the two populations do not overlap. |
| `slope.walk_crossfall` | I1 | How far a pedestrian band tilts across its own width: its height at a side edge against its height **the longest baseline the band offers** inward — a metre where there is a metre, then 0.5, then 0.25 — along the edge's own perpendicular. It read a fixed metre until it was caught being blind to the bands most likely to be wrong: `synth::pave_mesh` insets a band's surface by `PAVE_RIM_M` on each side for the casing, so a probe a metre inward needs a **1.70 m** band to land on anything, and narrower bands — which the facade room alone already produces at 0.8 m — left the population rather than entering the offender set. A metric that rewards narrowing a band over flattening it is the wrong instrument to judge a fix that narrows bands, and it flattered the first cut of `fit_to_ground` by 18 % of its own population. The mirror image of the metric above — where the rim step is a sidewalk defect, this is a path one: a sidewalk's height is its host's road surface plus a kerb and so is flat across by construction (p50 1 %), while a path's is the drawn ground and carries the full cross-slope of whatever it crosses (p50 30 % before the bench, half of all drawn path length past 30 %). An edge that still has band under it three metres inward is an *end* — where inward runs along the way and the reading would be its longitudinal grade — or a plaza, and is dropped. Two limits on the tail, both structural: a stairway's purpose is to change height, and a switchback whose arms the union merged into one region reads the *other arm* a metre inward, so beyond about 1:1 the metric has left the cross-section. Read the body. |
| `lod.structure_drift` | I5 | Structure height *over its own zoom's drawn ground* against the same structure one rung coarser. Absolute tops differ between rungs by design since the per-zoom datum (`GROUND.md` §4): the span rides each zoom's canvas, so what must agree across rungs is the relation to the ground drawn under it. |

### The model half

Three invariants are not about the emitted scene at all. They are about *how it
was computed*, and no amount of geometry can tell a scene where authority held
from one where it was violated and the numbers happened to come out plausible.
§8 of `GENERATION.md` says so directly: I7 and I8 "are established by
construction and falsifiable by a single perturbation experiment, not sampled by
a metric".

Those checks run in process, against the model, and are written out with
`arpentry_tiler --verify-model <path>`; `arpentry_verify --model <path>` merges
them so one table and one baseline cover both halves. The model half also
carries one quality metric whose subject left the archive: `slope.rail_grade`
walks the solved profile, because from `ROAD_SURFACE_MIN_ZOOM` the union paves
the rail formation and deletes the stroke the archive-side walk used to ride.

| metric | inv | what it means |
|---|---|---|
| `solve.determinism` | I5 | The same scene solved twice, compared bit for bit. Non-zero means a height depends on an iteration order or a thread interleaving, and every guarantee below rests on it not doing so. |
| `authority.inversion_R` | I7 | Stratum R re-solved with every junior corridor deleted from the scene, the burial licenses held at the full scene's values (the plan skeleton is input, §7 I7). Any senior height that moved is an authority violation. |
| `authority.inversion_S` | I7 | The same for S. |
| `datum.float` | I4 | At grade means on the ground. Every node the reconciled partition leaves at grade, in every stratum, water excluded: how far the solved road stands from its conditioned terrain reference — either direction, a road sunk into its own hillside being the same defect upside down — less the class deviation budget, so 0.0 is within the cut and fill the class is built on. Structure nodes are outside the population by definition. This is the *source* check: a corridor off its own datum is upstream of every clearance, cover and kerb number that will report it as something else. Known legitimate tail: a clearance lift raises an at-grade road by up to `MAX_CLEARANCE_LIFT_M` and keeps it at grade. |
| `ground.footprint` | I8 | Every layer of the ground stack against the one beneath it: where a layer moved the ground, its own declared footprint must cover the point. |
| `ground.single_source` | I1 | The published engineered ground minus the reference surface the solve read, at every profile node **inside a structure span** — I1's "one ground function", and the last §8 row to get an instrument. At-grade nodes are excluded on purpose: there the two differ by the corridor's own bench, which is the imprint working as designed and is scored by `contact.kerb_*`. Inside a structure the corridor benches nothing, so whatever moved the ground was somebody else and no stage re-read it — which is the S21 gallery stated as an invariant, its ceiling capped against a reference 6.5 m above the ground the town's benches finally carved. The reference is passed to the stack as its own base so the DEM-versus-lattice difference cancels. Three quarters of the population is exactly zero and the tail climbs smoothly with no gap, so the threshold is reasoned from what the consumers tolerate (a bore has 0.5 m of cover slack, a deck is called buried at 2.0 m) rather than read off a cliff — one of the few places in this harness where the population does not separate, and the note says so. `ARPT_DEBUG_SS` prints the sorted quantiles. |
| `authority.facade_ground` | I8 | The engineered ground minus the reference surface the solve read, at every 2 m of every building footprint edge — how much of the world's *wall* stands on ground a road decided rather than on the hill. A bench holds the ground flat at road level out to the class half-width plus a shoulder plus a verge, its batter reaches past that, and nothing has ever said that footprint may not contain a building; on the Montreux extract 12.2 % of wall stands on ground an earthwork moved at all. **The archive cannot answer this and neither can `contact.building_seat`.** A building anchors at the *highest drawn ground under its footprint*, so it rides whatever terrace it is given and the contact stays perfect: the seat check reads 0.011 % with this at full strength. What is wrong is the authority, not the contact — a road decided where a building stands — and an authority question is answerable only against the ground that would have been there, which no archive carries. Zero wherever no earthwork reaches, so closing the defect moves the number instead of emptying the population. The reference is passed to the stack as its own base, the same cancellation `ground.single_source` relies on: a raw `Dem::elevation` point sample instead would charge the in-cell interpolation to the earthwork. Threshold reasoned, not read — the population climbs smoothly (p90 0.09, p95 0.46, p98 0.99, p99 1.41, max 10.27) with no gap — from what the drawn world absorbs: `building_mesh` extends a foundation past the lowest ground under the footprint by the relief, which `stamp_elevations` rounds to the metre, so under a metre nothing is visible and past it the wall stands on a shelf the hill does not have. `ARPT_DEBUG_FACADE` prints the sorted quantiles. Costs 3.9 s. |
| `structure.bore_daylight` | I3 | The crossing premise, measured. A crossing over a mapped bore buys no clearance (§4.5), which stands on the premise that the bore passes beneath the ground the crossing feature rides on. At every plan crossing of a mapped tunnel span by an alignment annotated above it — the same gate that seeds the solver's burial ceilings — the bore's roof plus cover against its own terrain, signed. Positive is a bore daylighting through a roadbed: the waiver stood on nothing, and the two bands draw a storey apart with neither a bore nor a deck between them. Archive-side only `contact.kerb_lip` can see this class, because a dismissed tunnel paves no band for `order.grade_stack` to catch. |
| `crossing.orphan` | I3 | §8's structural claim for §4.5, now measured rather than assumed: every derived crossing — the set the clearance demands are seeded from — has a solved profile on both named sides. A lower side of `None` is whole (the crossed feature's height is the ground). Must read zero by construction; any count is a design failure, not a quality regression. |
| `graph.connector_step` | I2 | The spread of solved road heights where two profiled, paving corridors share a source connector — read from the corridors' own connector lists (ends and segment-interior attachments alike), *not* from the junction set, so a connector the assembler failed to turn into a junction still counts against it. Zero is the weld holding; past the 0.5 m sheet separation the union stops merging the two surfaces and the step is drawn — a slab floating over the carriageway it joins, kerb across the asphalt (the Colondalles fork stood 4.7 m over its tertiary this way). The population is most of the network's junctions: on the Swiss extract 749,524 paved connectors are interior to one segment while an end of another, and end-keyed junctions never saw them. |
| `contact.level_crossing` | I3 | Solved height disagreement where two paving corridors cross in plan and both sides' reconciled spans read grade — S15, and the one §8 row that had no instrument. `plan_index` already excludes the pairs that *meet*, so every member is a genuine crossing rather than a junction, and the equality is owed whether or not they share a connector (only a shared one is enforced, by `solve::graph::Contact`). Scoped to pairs within `crossings::SEPARATION_M`, past which the model's own vocabulary stops calling it a level crossing and `order.grade_stack` owns the case; measuring both would report one defect twice. **Empty on the Montreux extract, and that is the finding:** of 808 plan pairs, 799 pave a surface, 22 have both sides at grade, and every one of those 22 lies in the row-group spill outside the bbox — the extract contains no level crossing at all. The threshold is read off that spill population, which separates at 0.06 m (the weld holding) with nothing until 0.24 m. `ARPT_DEBUG_LX` prints the filter counts and every at-grade crossing it saw, which is how an empty population is told apart from a broken filter. |
| `slope.rail_grade` | I2 | The drawn railway's grade in excess of its own class ceiling (signed: negative is margin), walked on consecutive solved-profile nodes of `Profile::deck_m` — the composite the drawn line carries, road at grade and fitted ramp across structures, identical to the deleted stroke's heights at the reference zoom. Steps are clipped to the extract's bbox: assemble admits whole parquet row groups, so corridors spill past the zone into ground no DEM constrained, and a grade there scores the extraction boundary rather than the solve. The 20 pp allowance covers what `measured_grade` legitimately grants over the table (the rack railway's earned bed); the offender note names the solved ceiling so an earned bed is legible at the site. |
| `network.walk_cover` | I4 | Every metre-station of every mapped pedestrian line (crossings included), on-ground stretches only, scored as the plan distance to the nearest drawn *hard surface* of any kind — a walk or path band, a carriageway or formation, or a junction's paved extent, read from the same band sources the union buffers, after every narrowing and drop has had its say. The mapped pedestrian graph is connected; the drawn world re-derives it as bands and loses pieces (a crossing's kerb stub, a dropped or refused band), and each loss draws as a route interrupted by bare ground. Model-side because from `WALK_SURFACE_MIN_ZOOM` the pedestrian strokes are deleted — the mapped line exists only in the model, and only the model knows which stretches it deliberately did not band. Threshold reasoned: 0.5 m is an order above the boolean kernel and under anything a person reads as a hole in pavement. `ARPT_DEBUG_NETWORK` prints the bare length by category (crossing / claimed-but-missing / corner gap / free path). |
| `network.walk_material` | I4 | The same stations, non-crossing lines only, against **walkable bands alone**: a route continuous as asphalt can still read as broken — the corner that drowned in the junction plate, the run a street claimed and built nothing for — and a person walking it walks on the carriageway. An attached stretch scores zero when its *own host* built a walk band within the seat's play (8 m: the band is the mapped line's re-seated stand-in, clamped between kerb and facade at the run's mean offset), so legitimate re-seating is not a violation; an unattached stretch is scored against 1.5 m, since its band follows its own polyline. Known blind spot: the stand-in test cannot tell the host's near side from its far one, so a run that failed on one side of a street with a band on the other is under-counted. |
| `street.strip_continuity` | I4 | Every metre of every (corridor, side) that `assemble::walks` attached a pedestrian way to, over the **merged** extent of its claims — two claims within `WALK_CORNER_MAX_M` are one, since the attachment breaks at every corner and driveway by design — scored as the length of the interruption each station belongs to, zero where any walk or path band lies in the strip from the drawn kerb out to the seat's own play (8 m, so a legitimately re-seated pavement counts). The rate is therefore the share of claimed pavement drawn bare and the worst is the longest single hole. **This is the metric the walk census could not be:** `ARPT_DEBUG_WALK` reports 98.1 % of claimed host arc built while a third of it draws as disjoint slabs, because a census counts arc that produced *a segment* and cannot see that the segments do not join — nothing in a per-feature model owns a whole pavement to hold continuous. A station outside the extract ends the stretch rather than being skipped, so a hole never spans the bbox edge. Threshold 0.5 m sits below the station spacing on purpose: every uncovered station counts, and the threshold's only job is to keep the covered ones out of the tally. `ARPT_DEBUG_STRIP` prints the sides, the extent and the break count. |
| `street.kerb_join` | I4 | Every walk band segment attached to a host corridor, scored as the plan distance from the band's inner edge to the drawn edge of **that same corridor's** asphalt — never the nearest asphalt, since a pavement wrapping a corner stands close to a side street it does not belong to. Overlap clamps to zero (`order.at_grade_overlap` owns that case and the union already subtracts it); hostless bands — paths, corner links, crossing stubs — are out of the population, having no kerb to join. The cross-section model makes this identically zero because the pavement's inner edge and the carriageway's outer edge are one number read twice; today they are two independent allotments of one street (`carriageway::sections_along` spends the facade room on asphalt, `walkway::seat` re-measures the same facades for what it thinks is left) and the band is then seated at the *mapped way's own offset*, which is a fact about where a mapper drew a line and not about where a kerb is. Threshold 0.5 m is the profile-smoothing displacement — the median half-metre a band stands off the raw centerline at a junction mouth — and past it the strip between road and pavement is drawn hillside, which on a grade is the small cliff `contact.kerb_lip` scores from the other side. |
| `street.crossing_extent` | I4 | Every registered crossing chord with an end inside the bbox, marched at 0.25 m and scored as the total length lying further than its own drawn half-width from every corridor **the mapped crosswalk polyline actually intersects**. A crossing crosses one street, so its chord is that street's cross-section at that station; `synth::walkway::crossings` instead extends the mapped stub 8 m at each end and marches for *any* corridor within its prior half-width, merging across 1 m — so a crossing running *alongside* a station forecourt's service roads annexes them into one chord. Measuring with `on_asphalt`'s own predicate would read zero by construction, which is why the check distinguishes crossed from merely-near; that distinction is also the fix's definition, so the two share one derivation. Crossings whose polyline crosses no drivable centerline are out of the population — nothing owes them a width — and their count is reported separately under `ARPT_DEBUG_STRIP`. `paint.buried` finds the same worst site from the archive side when the something being painted on is a hillside. |
| `street.width_step` | I6 | Every shared vertex of two consecutive walk or path band segments that `synth::pavement` will chain into one buffered polyline, scored as the change in **drawn** width across it. Continuity of a ribbon is two properties and `street.strip_continuity` only measures one: whether the surface is *there*. This measures whether it is the *same* surface along its length. `fit_to_ground` resolves the earthwork per **segment**, so applying its verdict as a flat value made the two sides of every shared vertex disagree — at Territet, `path/track` p10 1.20 against p50 2.00 and a quarter of ways varying along themselves, which reads as a different object every few metres. `taper_along_runs` takes the **narrower** of the two allowances at each shared vertex, so the width interpolated across a segment lies between two values that are each at most that segment's own allowance and therefore never exceeds it at any station — a band may give width up and may never take it, said a third time. Zero by construction, so the 0.01 m threshold is an epsilon on the arithmetic rather than a tolerance on the model; measured 12.54 % over with `ARPT_NO_WALK_TAPER=1` and 0.00 % without. |
| `network.walk_reach` | I4 | For every endpoint of a pedestrian line that the data *joins* to something — a connector another feature carries, or another pedestrian line ending within 0.75 m — the bare metres walking in from that endpoint before any drawn hard surface appears (capped at 20 m). Connectivity weighted at the joints, where the eye reads it: a band that stops short of its own endpoint breaks the drawn network at exactly a place the mapped one connects. A true dead end owes nothing and is out of the population. |

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
  here that needed no judgement. Searching the *scene model* out to 25 m
  (`ARPT_DEBUG_CARRY`) puts every carried path between 2.0 m and 9.5 m of a
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
  own resolution, and it was read off the classes that do bench, on the stroke
  population that existed when rail still stroked the detail zooms:
  `standard_gauge` benched at 98.9 % of its at-grade nodes and ran p95 0.80 m,
  p98 1.45 m, p99 1.62 m, then jumped to 5.52 m at p999, with 0.12 % of its
  samples in the 2–4 m bin. The level-0 *road* strokes, which took the same
  drape path, reached 1.27 m at p999. Two metres sits in that empty band. The
  deck-proximity exclusion the first cut needed (a viaduct's paint stroke
  arriving at level 0 and metres up — 19,993 vertices on the Montreux extract)
  now guards an empty class at the surface zooms, where the stroke is deleted
  outright; both it and the metric itself measure pre-surface rungs only.
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
- **A gap in the wrong place is worse than no gap at all.**
  `order.building_overlap` is the one threshold here read *against* a measured
  separation rather than off it. Its population does separate: 17.5 % of
  inside-samples lie past 2 m and 9.6 % past 3 m, then only 2.8 % in the whole
  band from 3 m to 5 m, and 6.8 % beyond it. Cutting there would have been the
  obvious move and it would have been wrong, because the two modes are not
  noise and defect — they are two defects with different fixes. Past the gap is
  a way whose *centerline* is inside a footprint, which no width cap can move;
  below it is a band overrunning a facade, which is the whole point of
  allocating the cross-section out of the room. A threshold at 4 m would
  therefore have gated on exactly the half no change was going to close, and
  reported the half that closed as unmoved. So it sits at `FACADE_CLEAR_M`
  (0.5 m) instead — the clearance the drawn surface is *meant* to keep off a
  footprint, making half a metre inside one a full metre from where the model
  puts it, which is past what a surveyed footprint's own error explains.
  Confirming the split is a scoped run rather than an argument: at Rue du
  Marché the whole tile reads 8.3 % inside, worst 3.9 m, and **nothing past
  5 m**.
- **A perfect contact can hide a violated authority.** `contact.building_seat`
  reads 0.011 % at z16 while `authority.facade_ground` reads 1.933 % over a
  metre with a worst of 10.27 m, and both are correct: a building anchors at
  the highest drawn ground under its footprint, so when a road's bench cuts a
  terrace under a wall the building simply stands on the terrace. The contact
  is exact. What is wrong is *whose* ground it is — and that question has no
  archive-side answer at all, because the archive carries the ground that was
  drawn and never the ground that would have been there. When a defect's whole
  consequence is absorbed by a downstream rule, the instrument has to move
  upstream of that rule, which usually means into the model half. The
  threshold there comes from the same absorbing rule read forwards: the
  foundation the mesher extends covers a metre of movement, so a metre is
  where the shelf becomes visible.
- **A proximity test is not an attachment test.** The trap that decides whether
  `contact.sidewalk_grade` measures sidewalks or hillsides. Scored on "within
  eight metres of a kerb" alone, the population is 117,200 samples running from
  14.2 m below the carriageway to 12.4 m above it — and the top of both tails
  is paths that merely pass near a road on a Montreux flank, not sidewalks in a
  ditch. Two rules fix it, and both were measured rather than assumed: the
  *part* must run with a street for 80 % of its length (53 k samples, worst
  10.97 m), and it must be locally parallel to the kerb within 30° (30 k
  samples, worst 10.19 m). The coverage figure is not invented here either —
  the plan-space census scored the same test against the tagged
  `subclass='sidewalk'` population as ground truth and found 65.7 % of tagged
  sidewalks over 0.8 corridor coverage against 14.4 % of untagged ways. What is
  left still has a tail that is not this metric's defect, and the note says so:
  past a few metres it is a street on a terrace with a path along the foot of
  its wall, which `contact.kerb_lip` and `contact.kerb_unwalled` own.
- **Both signs are the same defect.** `contact.sidewalk_grade` is unsigned, and
  the population is what argues for it: 44.9 % of the departures are below the
  kerb and 55.1 % above it. The plan-space study that prompted the check found
  the sidewalk *below* its street and it was right, but only about the fill
  side of the hill — on the cutting side the bench cuts the road down and
  leaves the pedestrian way stranded on the bank, which is the same missing
  cross-section upside down and is fractionally the larger half. A signed
  metric would have reported a bit under half of it and counted the rest as
  margin. The threshold is then read off the magnitude, which is two-moded:
  27.2 % past 0.25 m, 12.9 % past 0.5 m, 8.2 % past 0.75 m, and then a knee
  into a flat tail — 6.3 % past 1 m, 5.4 % past 1.25 m, 4.7 % past 1.5 m. A
  metre sits at that knee, clear of a kerb rise plus a verge's cross-fall, and
  clear of what the instrument can manufacture on its own: the reference is the
  *nearest* kerb point in plan, up to eight metres away, so at a corner some of
  that distance is along the road and the street's own longitudinal grade
  contributes.
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
`GROUND.md` §4. Since the per-zoom structure datum the same is true of a
structure's absolute height — the equality that remains promised, and that
`lod.structure_drift` measures, is its height over its own zoom's drawn
ground.

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

**Buildings and water, in the parts that are left.** This used to read "the
verifier decodes only the terrain and transportation layers", and it no longer
does: buildings are scored twice — `contact.building_seat` for the foundation
and `order.building_overlap` for the street running through the wall — and
flowing water once, by `water.descends`. What is still unmeasured is narrower
and worth naming so the gap does not get lost inside a sentence that is now
mostly false: **still water** (the flatten's own subject, S12 — the descent
check walks lines and a body has no along-flow order to walk), **building
heights and roof shapes** (the seat check reads the footprint's contact with
the ground and nothing above it), and **land cover and land use**, which the
verifier does not decode at all.

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

So the general rule stayed unimplemented, and it acquired a **named
prerequisite**: `datum.float`. A two-sided correction cannot be safe while some
corridors sit hundreds of metres off their own datum, because those are exactly
the ones whose demands are nonsense. The prerequisite is now measured (§5): on
the Montreux extract 389,250 at-grade nodes read a median 0.01 m and a p99 of
1.45 m off their reference, with 0.15 % past half a metre — so the population is
mostly clean and the question is entirely about its tail, whose worst was a
residential road sitting 20 m *under* its own ground at 6.7042,46.5073. Held to the runs the data calls tunnels,
the correction goes where §4.5's prior points and nowhere else: `bore_cover`
violations 14.14 → 13.42 %, `kerb_lip` 13.49 → 13.45 %, 463 m of annotated
tunnel recovered, and every extreme in the scorecard unmoved.

### What chasing that tail found (2026-08-18)

The `datum.float` tail was not a bad demand — it was the correction machinery
failing in both directions at once, and fixing the two failures is what the
"solved together to feasibility" ambition turns out to mean inside one stratum.

**The dip's footprint was unbounded.** A dip ramp's ceiling rose away from the
crossing at the class grade, absolutely. On a hillside steeper than that grade
the ceiling never overtakes the terrain, so it chased the whole uphill network
— through every junction, permanently, since the walk records its bounds into
the slack box the deviation clamp yields to. At Lutry two 4.5 m rail
underpasses sank a residential grid 8–20 m below its own ground, and
`structure.derived_new` read the sunken streets back as a 324 m phantom
tunnel. The fix bounds the *depth*: no walked node is asked further below its
own reference than the crossing's dip less the budget already spent. On flat
ground this is algebraically identical to the old ceiling; on the hill it ends
the excavation where a real approach ramp would. The grid came back to its
2.5 m class box with the genuine 4.3 m dip kept at the underpass itself
(`a_hillside_underpass_dip_stays_local`).

**The lift missed every structure run that reaches a corridor end.** Such a
run has no at-grade anchor on that side, so the span lookup declared "no span"
and the lift fell to the at-grade ramp: interior nodes rose, `rigidity_pass`
chorded them back through the never-lifted ends, and the closing settle read
the inflated interiors as satisfaction — a hard I3 violation that no counter
counted. Corridor #18064 at Villeneuve, a whole-corridor bridge welded to its
junctions, sat 6.8 m under eight rail-yard demands with *zero demands
dropped*. The fix anchors such runs at their end nodes — the same anchoring
the rigidity chord already uses — and spreads the lift outward from each
anchor through the junction-welded network, so the approaches arrive on a ramp
(`a_bridge_reaching_its_corridor_ends_still_clears_its_crossing`). The
solve-wide clearance shortfall maximum fell 7.66 → 3.75 m.

**The measured trade.** `datum.float` 0.148 → 0.132 %, worst 17.66 m *under*
→ 11.33 m *over*; `order.grade_stack` 13.52 → 13.26 %,
`clearance.deck_over_ground` 1.08 → 0.92 %, `slope.road_grade` 0.55 → 0.46 %.
The sign flip on the worst is the honest cost stated plainly: at La Conversion
a 9 m bridge legitimately crosses the double-track Bern line 15.7 m over the
road's own conditioned reference, and where the old code left that lift on two
anchor nodes with a 13.5 m wall beside them, the ramp now spreads it as the
approach embankment the overpass needs. Ramped approaches also stand off the
ground far enough to *derive* as unannotated bridges
(`structure.derived_new`'s worst is now such an approach) — that is §4.5's
switch territory, not a defect of the lift. `contact.kerb_lip` +0.11 pp is the
same embankment's kerbs. What still holds the rule short of "general": a
junior stratum still cannot ask a senior to share a lift (I7, by design), and
the bore-roof-versus-drawn-ground family (`clearance.bore_cover`, the A9 roof
at 6.9208,46.4336) is a ceiling-seeding question this machinery does not
touch.

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

`server/verify/baseline-montreux-coarse.json` is the same archive over
`--zoom 13,14,15`, and it exists because the detail rung cannot see the rung
problems. Everything per-zoom — the terrain lattice, the structure datum, a
foundation's contact with the drawn ground — is by construction *correct at
z_ref and only there*, so a defect in the correction is invisible to a z16
baseline. `contact.building_seat` was 49.7 % over at z13 while reading 0.012 %
at z16; nothing gated it. Cut both, and diff both.

Regenerate with `--json`, and say in the commit message which numbers moved and
why.

### 9.1 What decides the verdict

A metric is three numbers, and the diff consults them in order of how much of
the distribution has to change for the answer to change:

1. **How often** — `violation_pct`, the share of samples past the threshold.
   A statistic over every sample, so a move in it means the geometry moved.
   Gates.
2. **How bad** — `tail`, the p99.9 a single outlier cannot dominate. Reached
   when the rate holds: the same sites failing, each one worse. Gates.
3. **The extreme** — `worst`. Reported as `outlier only`, and **never gates.**

The order was originally the reverse, and it made the gate unusable. `worst` is
a maximum over as many as thirteen million samples, so it is the least stable
number on the card, and the rate was consulted only if it moved by less than a
centimetre — which it essentially never does. Measured on Montreux, that
reported seven regressions, *none* of which had moved its median to three
decimals: one new sliver triangle took `slope.terrain_face` from 201.8 to 349.9
with the rate at 0.617 → 0.616, and `contact.kerb_lip` was called a regression
on the run where its violation rate fell from 12.8 % to 8.7 %. It failed the
other way too, calling `clearance.bore_cover` and `order.grade_stack` improved
while both were getting steadily worse on rate.

A gate that fires on every run gets turned off, which is strictly worse than no
gate: at the time this was found the committed baseline was three commits stale,
`arpentry_verify --baseline` exited 1 on the committed tree, and the eight
metrics added by those three commits had no baseline entry at all. A genuine
per-site catastrophe is what §8's corpus scenarios and the offender lists are
for; it is not something a max over the whole extract can be asked to find.

**The noise floors.** A rate move must clear all three of: 0.01 percentage
points absolute (so a rare defect is not swamped), 2 % relative (so a common one
has no hair trigger), and **three binomial standard errors** at the pooled rate
over the smaller population. A tail move must clear 5 cm, because the histogram
bins at 1.3 cm and a quantile that slips one bin has not measured anything.

The statistical floor is what makes the small-population metrics usable at all.
`seam.abutment_bare` has twenty samples, so one new offender is a five-point
move; `seam.abutment_plan` has a hundred and forty-three, so one is nearly a
point. Both reported as confident regressions on a run that had changed a
constant by 0.7 %. Three sigma at n = 20 is fifteen points — which is the honest
statement that twenty samples cannot resolve less.

For the same reason the tail only gates above **ten thousand samples**. `tail`
is the p99.9, and it only means "the bulk of the tail" once the top thousandth
holds several samples; at twenty, p99.9 and the maximum are the same number, and
gating on it would smuggle back the single-outlier gate this ladder exists to
remove.

The cost is real and worth stating: the seam metrics (143–922 samples at
Montreux) can only gate on large moves. If a change needs finer resolution than
that, the answer is a larger extract, not a lower floor.

A metric present in the baseline and absent from the run also reports, because a
check that stopped running looks exactly like a check that passed. So does the
reverse — a check with no baseline entry prints `absent` and is named as ungated
on stderr, so newly added checks cannot quietly protect nothing.

### 9.2 What makes two scorecards comparable

Every scorecard carries a `scope` block: the commit, the tiles visited and the
plan extent they cover, the sample spacing, and whether the tile cap bit. A diff
reports any drift in it *before* the verdicts, because a baseline cut over
different ground produces a full column of confident, meaningless verdicts — and
a population that moves on its own moves every metric with it.

The `samples` column carries the same warning per metric, flagged with `!` past
5 %. This is the trap that hid the largest real change on the last five commits:
`contact.rail_standoff` lost 37 % of its population and shifted its median by
12.5 m, and the old table summarised it as a 1.3 m improvement.

The baseline this replaced recorded only an archive *filename*, which pointed at
a throwaway A/B archive in a session scratchpad — no bbox, no zoom, no commit.
Nothing about it could be reproduced or even located.

## 10. Adding a check

A check is one file in `server/src/verify/checks/`, implementing `visit` and
`finish`, plus a line in `checks::run`. The friction is deliberately low: a
defect found by eye should become a permanent measurement in the same sitting.

Before believing a new check's first number, measure its *anatomy* — histogram
the population it is scoring and look for a second mode. Three of the metrics
here changed shape after that step, and one was measuring something else
entirely.
