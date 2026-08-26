# The Source Data

Every other document in this repository reasons about a world the generator
builds. This one is about the world it is given: what Overture Maps actually
carries, what the tiler reads of it, and — the reason the document exists —
the gap between the two.

The gap matters because the generator's whole method is to recover a 3D world
from a 2D plan plus priors. A prior is what you invent when the data is
silent. Every attribute the source *does* carry and the tiler *does not* read
is a prior invented over an available fact, and priors are where artifacts
come from. `docs/GENERATION.md` §7 counts manufactured retaining walls as an
invariant violation; Overture ships the real ones. The pedestrian network
synthesises crossings from geometry; Overture says where they are, to a
fractional position along the way.

Measurements below are on release `2026-05` for Switzerland
(`data/overture-ch/`), and on the Montreux preview zone
`6.855469,46.398010,6.981700,46.472168` where a figure is local. Lengths are
spheroid lengths, not projected ones.

## 1. What the tiler reads today

Two lists, and they agree by design — `assemble::ATTRS` and `pipeline`'s
transportation entry request the same columns, so the assemble stage and the
tiling phase cannot disagree about what a segment is:

```
id  type  subtype  class  subclass  names.primary
level_rules  road_flags  connectors
width_rules  road_surface  access_restrictions
cartography.min_zoom  cartography.max_zoom  cartography.sort_key
```

The inputs are set in `scripts/run-overture-ch.sh`: `land_cover`, `land_use`,
`water`, `segment`, `building`, `place`, `division_boundary`. Seven of the
fifteen Overture types.

## 2. How Overture says a thing is true over *part* of a way

This is the single most important fact about the schema, and the one the
tiler currently does not honour.

An OSM way is tagged uniformly; an Overture segment is not. Overture models an
attribute as a **linearly-referenced rule list** —
`list<struct<value, between: [f64; 2]>>` — where `between` is a pair of
fractions along the segment. A rule with no `between` applies to the whole
segment. The scalar column beside it (`subclass`) is a *convenience*: Overture
populates it only when the value is uniform, and leaves it **null** the moment
any rule is partial.

That last clause is the trap. A footway that is a sidewalk over its first 60 %
and a crossing over the rest has `subclass = NULL`. A reader of the scalar
column does not see a mixed way; it sees a way with no subclass at all — an
anonymous footway. The information is not missing, it is in a column nobody
opened.

The tiler honours this shape for `level_rules` (parsed into runs by
`crate::levels`) and reduces it for `width_rules` / `road_surface` (dominant
value, `crate::rules`). It does not read `subclass_rules` at all.

### What that costs, measured

In the Montreux preview zone, footway and cycleway segments:

| subclass | whole-segment | partial rules | share invisible |
|---|---:|---:|---:|
| `sidewalk` | 70.29 km (259 seg) | **24.27 km (217 rules)** | 25.7 % by length |
| `crosswalk` | 2.79 km (222 seg) | **1.91 km (180 rules)** | **45 % by count** |

Switzerland-wide, 29,590 footway segments are a sidewalk over part of their
length and 30,357 are a crosswalk over part; **24,640 are both, over different
stretches.** That last figure is the point. A sidewalk that runs to a corner,
crosses the road, and continues is *one* Overture segment carrying two
subclass runs — which is precisely the sidewalk-continues-across-the-road
relation `assemble::walks` and the `network.*` checks are reconstructing from
geometry and shared connectors.

### What reading it changed

`assemble::walks::split_by_subclass` cuts a pedestrian line where its subclass
changes, so a mixed way becomes the sidewalk and the crossing it actually is.
Two guards earned their place by measurement, both on the Montreux zone against
a control tiled from the same inputs:

**A run shorter than the band is wide is absorbed into its neighbour.** The
sidewalk runs Overture leaves either side of a crossing have a p25 of 1.61 m
and a minimum of 0.59 m — kerb-to-crossing stubs, not stretches of pavement.
Cutting on those shatters the band into pieces that each fit their own bench.
A crossing is exempt: it is paint, never a band, so the width floor says
nothing about it.

**The crossing runs cost nothing once the stub is seated.** Taking them cut
`slope.walk_crossfall` to 2.561 % against a 2.481 % control, and the cause was
not the crossing — it was the kerb stub the crossing brings with it, which
`synth::walkway::seat_stubs` now fixes (see below). Measured:

| variant | crossings | stub | sidewalks attached | `walk_crossfall` | gate |
|---|---|---|---|---|---|
| control | flat only | unseated | 5312 / 354.1 km | 2.481 % | — |
| sidewalk runs only | flat only | unseated | 5384 / 356.2 km | 2.545 % | same |
| + crossing runs | all | unseated | 5387 / 356.3 km | 2.561 % | REGRESSED |
| stubs off (ceiling) | all | none | 5387 / 356.3 km | 2.438 % | — |
| **+ crossing runs** | **all** | **seated** | **5387 / 356.3 km** | **2.469 %** | **none** |

The worst sample is 12.51 in every row — throughout, this was a rate moving on
an unchanged distribution, not a new defect.

### The kerb stub was seated on the ground

Isolating the stub from the paint (`ARPT_NO_CROSSING_STUB`) reproduced the
stubs-off row exactly, so the paint contributes nothing to these metrics and
the whole cost was the stub band. **22 % of stub samples violated cross-fall
against a 2.4 % baseline.**

`fitted_half` takes a `NO_HOST` band's seat from the ground under its own ends
and a hosted band's from `height_a`/`height_b` — the height its street's
cross-section draws it at, kerb included. A stub was built hostless with both
heights zero, so it and the pavement it continues met in plan and nowhere in
section. At 6.856580,46.457663 the band ran 384.2 → 383.67 → 384.3: a 0.6 m
notch a third of a metre wide, and the cross-fall probe read the drop off the
stub's own edge.

`seat_stubs` gives each stub the corridor, drawn height and kerb rise of the
nearest hosted walkway band at its ends — the same claim the stub's own doc
already made in prose, made to the machinery that decides heights. A stub that
finds no band within `STUB_SEAT_REACH_M` stays hostless and drapes as before:
a crossing onto a path, or onto a pavement the fit declined.

This was a defect in its own right, not one the subclass work introduced. Every
row above carries the 222 flat-subclass crossings' stubs, so the unseated stub
was costing the control too.

## 3. The `infrastructure` type, which is not downloaded

`overturemaps download --type=infrastructure` (theme `base`) is a valid type
the pipeline has never fetched. In the Montreux zone alone:

| class | n | extent |
|---|---:|---|
| `transportation/crossing` | 513 | points |
| `bridge/bridge` | 279 | 12.85 km |
| `barrier/kerb` | 133 | 130 points, 3 lines |
| `transit/platform` | 130 | 43 polygons |
| `barrier/hedge` | 121 | 5.90 km |
| `barrier/wall` | 75 | 4.38 km |
| **`barrier/retaining_wall`** | **55** | **3.34 km** |
| `barrier/fence` | 32 | 5.84 km |
| `pier/pier` | 28 | 1.04 km |

Columns: `id, geometry, subtype, class, names, level, height, surface,
wikidata, source_tags`.

Two of these rows are load-bearing for this project.

**Retaining walls.** The obvious hope was that these would license the faces
`slope.terrain_face` counts — its own doc says a retaining wall "is steep and
*correct*" and that the check cannot tell one from a manufactured one.
**Measured, and that hope is wrong.** Distance from a `terrain_face` offender
to the nearest mapped wall, against a uniform-random null over the same bbox:

| population | median | p25 | within 10 m |
|---|---:|---:|---:|
| 500 steepest drawn faces | 407 m | 177 m | 14 / 500 |
| random points (null) | 1120 m | 522 m | 0 / 500 |

14 against 0 is real signal rather than chance — both populations favour built-up
ground — but **97 % of the manufactured faces have no mapped wall anywhere near
them.** Mapped walls do not explain that population and will not license it.

What they *do* mark is the opposite: real vertical faces the generator draws as
graded slopes. Sectioned across the two longest mapped walls in the zone
(6.921734,46.427999 and 6.920255,46.429266, 246 m and 188 m), the drawn ground
steps between road terraces by 3.5 m and by 4–10 m, every one of them a smooth
roughly 1:1 batter where the map says concrete. So the value here is *added
geometry*, not a defect fix — and taking it would raise `terrain_face`, which
counts a vertical face as spectacle. Drawing the wall and licensing it in the
check are one change, not two.

Note the limit before planning on it: `height` is filled on ~2 % of walls. The
data says *where*, not *how tall*.

**Crossing points.** 513 of them against 402 crosswalk subclass runs in the
same zone — a denser and independently-mapped registration of the same fact.

## 4. `source_tags`: the one place OSM survives

`infrastructure` carries a `source_tags` column: a verbatim `map<varchar,
varchar>` of the originating OSM tags, populated on **100 %** of features. It
is the only passthrough of raw OSM tags anywhere in Overture. `segment` has no
equivalent — its `sources` column is a provenance struct (dataset, licence,
record id, confidence), not tags. **OSM tags on ways are genuinely lost; OSM
tags on nodes and barriers are not.**

On the 513 Montreux crossing points:

| tag | present | values |
|---|---:|---|
| `crossing` | 490 | `marked`, `uncontrolled`, `traffic_signals` |
| `crossing:island` | 426 | **79 `yes`** |
| `crossing:markings` | 375 | 335 `zebra`, 37 explicitly `no`, 1 `lines` |
| `tactile_paving` | 225 | |
| `kerb` | 40 | 29 `lowered`, 6 `flush`, 5 `raised` |

Three consequences worth stating plainly. Zebra stripes are currently drawn on
every crossing; 37 of these say there are no markings at all. A traffic island
is a raised refuge in the middle of a carriageway — 79 pieces of 3D geometry
nobody is building. And the synthesised kerb rise is applied uniformly, while
`kerb=flush` and `kerb=lowered` name the exact places where the rise should be
zero — which is where a pavement meets a crossing, the junction the
`street.kerb_join` and `seam.handover_kerb` checks are measuring.

## 5. Flags the level parser discards

`levels::parse_flags` maps `is_bridge` → +1 and `is_tunnel` → −1 and states
that the rest "contribute nothing". The rest are `is_link`, `is_covered`,
`is_indoor`, `is_under_construction`, `is_abandoned`.

So an indoor way was an ordinary level-0 footway to every stage after it: it
earned a walk band, benched stratum D, and was drawn as pavement through the
building's floor. `order.walk_indoors` is the instrument — the pedestrian half
of `order.building_overlap`'s question, which had only ever scored road
surface. It reads **3.318 % over 0.5 m at Zurich Airport, worst 42.9 m**, a
path drawn 43 m inside the terminal, against 0.850 % for the road population in
the same tiles.

The defect **clusters**, which is why Montreux never showed it: 545 indoor ways
and 19 km in one 0.02° cell over the airport, against 0.07 km in the whole
Montreux zone, where the metric reads 0.083 %. Switzerland holds 3,332 covered
and 1,631 indoor footways plus 871 covered and 771 indoor steps. 96.7 % of the
airport's indoor ways carry no level rule, so nothing else ordered them out.

`levels::indoor_runs` now parses the `is_indoor` spans — 18.8 % of them are
*partial*, so a boolean will not do — and they reach `WalkLine::spans`, whose
own doc already named this case ("a passage under a building") and which the
band suppression already read.

**A wholly indoor way is silent, not degraded.** Suppressing the band alone
made it worse in a way worth recording: the way dropped out of `banded_walks`,
fell back to its cartographic stroke under I6, and drew a line through the
building instead of a band — Montreux `paint.stroke_over_band` 10.965 →
19.745 %. I6's fallback exists for a band the surface model declined for lack
of room; it does not license drawing a footway through a terminal, because that
line *is* the spectacle I6 forbids. So a way whose indoor runs cover it
entirely joins `banded_walks` and says nothing at all. A partly indoor way is
untouched: the stretch outside is real, it gets a band, and the band deletes
the stroke by the ordinary rule.

| | Zurich Airport | Montreux |
|---|---|---|
| `order.walk_indoors` | 3.318 % → **0.495 %** | 0.093 % → 0.083 % |
| worst | 42.9 m → 31.0 m | 18.55 m → 18.55 m |
| `paint.stroke_over_band` | 23.041 % → **21.026 %** | 10.965 % → **10.965 %** |

Both gates exit 0. The residual 0.495 % sits against the road population's
0.850 % in the same tiles, which is the ordinary facade-clipping band rather
than a way indoors.

`is_covered` is deliberately left alone. An arcade at grade under its own
building is real pavement people walk on — Bern's Lauben are 9 km of it — and
no property in the archive tells that from a way through a wall.

## 6. `land_use` never reaches the solve

`land_use` is layer 6, a paint layer. It is styled and drawn and it does not
participate in the ground solve, the imprint, or the profile.

Overture's `land_use` carries `subtype = pedestrian`: 5,052 `pedestrian` and
2,365 `plaza` polygons in Switzerland. These are paved, level, walked-on
surfaces — the same material as a carriageway and with the same claim on the
ground under them. They are currently draped colour on whatever the terrain
does, while a footway crossing the same square benches its own band through
it.

## 7. What Overture does not have

Recorded so the search stops here.

- **`sidewalk = both/left/right/no` on the road segment.** OSM's answer to
  "does this street have a pavement" is not modelled anywhere in Overture. The
  only evidence of a pavement is a separately-mapped sidewalk way, so a street
  whose pavement nobody drew is indistinguishable from a street with none.
  `data/plans/` records this being measured and the wrong conclusion nearly
  drawn from it.
- **Steps have no `step_count`, `incline`, or `handrail`.** 9.6 km of steps in
  the Montreux zone, and nothing in the schema distinguishes a flight of
  stairs from a ramp of the same gradient.
- **No sidepath or parent-street relation.** Nothing links a sidewalk to the
  street it serves. Attachment must stay geometric, which is what
  `assemble::walks` does.
- **Sparse where it does exist.** By length in the Montreux zone: footway
  `width_rules` 1.8 %, steps 0.3 %, footway `road_surface` 22.7 %, footway
  `level_rules` 12.5 %. The 2 m nominal band width is not a fallback, it is
  the answer for 98 % of pedestrian ways.

## 8. Schema freshness

Release `2026-08-19.0` has the identical 21-column `segment` schema as the
local extract. Re-downloading buys fresher data, not new fields — worth
knowing before treating a missing attribute as a staleness problem.

`overturemaps download` reaches S3, which is not in the sandbox allowlist; it
fails with `Could not resolve host` and needs the sandbox disabled.

## 9. Ranked

1. **Read `subclass_rules`.** One column in a file already open. Recovers 180
   crossings and 24 km of sidewalk identity in the zone every metric is
   measured on.
2. **Add `infrastructure` as an input.** Mapped retaining walls let the solve
   tell an invented wall from a real one; `source_tags` is the back door to
   real kerb, marking, and island data at crossings.
3. **Stop benching `is_indoor` ways.** Small, bounded, and kills a class of
   defect before it is met.
4. **Give pedestrian and plaza land use a claim on the ground.** The largest
   of the four, and the one that needs a design rather than a read.
