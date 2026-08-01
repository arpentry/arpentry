# Verification

How the emitted scene is measured against the invariants, and why the
measurements are shaped the way they are.

Owned by `server/src/verify/`. Run with `arpentry_verify`.

## 1. Why

`GENERATION.md` §5 states six invariants and calls them "acceptance criteria for
any implementation". §4 states fourteen canonical situations and calls them "the
test scenarios for any design". Both were prose. The only instrument that
existed was `solve::consistency`, which measures the *solved model* — stage 2 of
five — and reports it consistent.

Every defect that has cost real time lives after stage 2, in stages 3–5:
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
| `contact.pavement_over_terrain` | 4 | Signed clearance of the carriageway surface over the terrain mesh. Negative is buried: the ground is drawn through the road. |
| `contact.pavement_unbacked_pct` | 4 | Per-tile share of asphalt with no terrain triangle beneath it. |
| `order.deck_above_carriageway` | 3 | Deck running surface minus the at-grade asphalt sharing its plan position. Negative past the touchdown band means the level ordinal inverted. |
| `clearance.deck_over_ground` | 4 | Deck soffit minus the terrain beneath it. Past a deck thickness, the deck ploughs into the hillside. |
| `clearance.bore_cover` | 4 | Terrain minus the bore roof. Negative past a portal mouth means the tube is in open air. |
| `seam.terrain_step` | 2 | Spread of the heights two adjacent tiles derive for the same border lattice point. |
| `seam.pavement_step` | 2 | The same, for the at-grade road surface. |
| `slope.terrain_face` | 6 | Rise over plan run of every terrain triangle spanning ≥10 cm. Finds manufactured retaining walls. |
| `slope.carriageway_face` | 6 | The same for interior asphalt, excluding the kerb rim. |
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

**Structure drift where the match is ambiguous.** Structures carry no identity
across zooms beyond class and level ordinal. Where a coarse tile holds several
candidates, "the same structure" is a guess — the first version made it and
reported 2.06 m of drift on a deck whose parent held *nine* candidates, which is
evidence of comparing two bridges, not of drift. Samples count only on a
one-to-one match, and the skipped count is reported.

**Buildings and water.** Invariants 4 and 6 cover both (S3, S11–S14); the
verifier decodes only the terrain and transportation layers today. This is the
largest gap.

## 5. The corpus

`server/verify/scenarios.json` binds each situation from `GENERATION.md` §4 to a
real place. Sites are **mined, not invented**: `--mine` finds the strongest
instance of each detectable situation in an archive — the highest viaduct, the
deepest bore, the tile holding both a deck and a portal — as a superlative
rather than a threshold, so it always returns *the* worst case the data holds.
Coordinates chosen any other way are a guess about someone else's terrain.

Seven of the fourteen are minable from the archive today. The other seven are
listed in the file's `unsited` block with the reason each needs a hand or a
decoder that does not exist yet, rather than being given a plausible coordinate.

Re-mine after retiling a different extract; the sites are extract-specific.

## 6. Baselines

`server/verify/baseline-montreux-z16.json` is the committed scorecard for
`data/overture-ch/preview.arpa` at z16. It is not a statement that the scene is
correct — it records what was true when it was written, including the deviations
that are known and accepted. Its job is to make the *next* change legible.

A metric that regressed exits 1. A metric present in the baseline and absent
from the run also reports, because a check that stopped running looks exactly
like a check that passed.

Regenerate with `--json`, and say in the commit message which numbers moved and
why.

## 7. Adding a check

A check is one file in `server/src/verify/checks/`, implementing `visit` and
`finish`, plus a line in `checks::run`. The friction is deliberately low: a
defect found by eye should become a permanent measurement in the same sitting.

Before believing a new check's first number, measure its *anatomy* — histogram
the population it is scoring and look for a second mode. Three of the metrics
here changed shape after that step, and one was measuring something else
entirely.
