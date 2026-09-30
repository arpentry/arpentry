# The census's defects, by cause (2026-09-30)

What the loop box's census (`world/src/census.rs`, at `0e8d0ae`) is made of,
traced to code. Five read-only investigations, one per owning construction,
each over the sites in `world/sites/montreux.tsv` and the whole loop box;
causes marked **C** were traced in code with the numbers matching, **L** are
likely. Loop-box counts are defects (chains or clusters) unless marked.
User verdicts from the preview (31, on `gap-deck-1`) confirmed the deck gaps,
the deck-landing fights/buried cluster, and separated fins (below).

The probes the investigations wrote are untracked scratch in
`world/examples/probe_{structure,fights,ground,fins,gaps}.rs`.

## 0. The census first — what it charges that nobody can see

Fix these before gating anything on the numbers they move.

| Correction | Evidence | Effect |
|---|---|---|
| **fin charges a height, not a slope.** Charge a fin only if a corner stands > 0.1 m off a neighbouring drawn triangle's plane, or it is not level-edged (two corners at one height) and folds ≥ 2 cm. | 74 % of loop fins (3 154 / 4 280) are millimetre needles creased 1–5 cm by a curved field or the weld's mean shift; the user's 12 fine / 4 real fins separate exactly on "level-edged, outside a blend" vs "in a blend or spiking". 16 samples: confirm with verdicts on the two mixed groups. | ~2 400 fins leave |
| **flip reads winding from unwelded positions** (or ignores triangles whose plan/3D area ratio is < ε). | 104 / 111 bench flips wind correctly in the model; the census's 0.1 mm weld flips needles. The other 7 are mesh needles ≤ 4.6e-5 m². | ~110 flips leave |
| **hidden is tested per sample, and a Ground-layer triangle is always earth.** | Midpoint-only misses edges on sub-mm seams (10 of 62 building gaps are buried at every sample); ground kept under a spanned face does not count as earth (6 structure gaps 40 m under ground). ~50 structure gaps have no open sample. | ~70 gaps leave |
| Same-colour fights (roadway+carriageway, track+ballast) reported apart. | Drawn in one colour (`gltf.rs:114-115`): duplicate geometry, no flicker. | reclassify ~60 |

## 1. "Spanned" is decided in plan, and the heights disagree — ~1 100 defects

The root of the largest family. A face is `spanned` (does not cut the ground,
`world.rs:1168`) by a plan mask; nothing checks that the paving over it is
actually in the air. Where it is not, the ground under it is drawn too
(fight, buried), its rim is never on the outline (no wall, no batter:
`earthwork.rs:225`, `bench.rs:160`), and only a deck solid could close it.

| Way a face becomes spanned while at grade | Loop | Reproducer (`--bbox 6.91,46.43,6.93,46.44 --spacing 5`) | |
|---|---|---|---|
| An apart sheet stores its **round-capped** span ribbon (`sheet.rs:392`), which covers its approach | 206 fight/buried | site `fight-carriageway-ground-1` only | C |
| The 1 cm `OVER_RIM_M` dilation is used as a **cut** (`arrangement.rs:58,134`): 1 cm spanned strips in neighbouring paving | 133 fight/buried + ~12 double-wall gaps | `flat` + `net:overpass?len=201` | C |
| The deck's own chord is at or under the ground: at-grade span, abutment batter, landing on a cross-slope (the user's `gap-deck-1` cluster) | 65 fight/buried | `flat` + `net:straight?span=0.35,0.65`; `gorge?depth=30&width=40` + `net:overpass?len=201` | C |
| **Carried walk**: any walk within 6 m of span paving is spanned (`arrangement.rs:56`) but lifted by the ground roads' field (`copies.rs:540`) | 206 fight/buried + 123 gaps | `flat` + `net:sidewalk?d=6&span=0.35,0.65`; `gorge?depth=30&width=40` + `net:sidewalk?d=6` | C |
| Spanned paving past the Deck run: the solid is capped at the first/last Deck station (`structure.rs:406,427`), the paving runs to the span edge | 100 gaps (+~30 carriageway-deck) | `gorge?depth=30&width=40` + `net:straight` | C |

**Direction (a design decision, not a patch):** decide whether a face cuts
from heights after the lift — the ground is cut, or clamped a slab below,
wherever spanned paving does not clear it by `DECK_THICKNESS_M` — and make
the span mask, the sheets' `spans` (square for apart sheets too) and the
deck runs one extent. It touches invariant I3 (a viaduct does not cut the
ground it flies over), the outline, `wall_m2`, `carried` and `clear`.

## 2. Structures — confirmed, local

| Cause | Loop | Reproducer | | Direction |
|---|---|---|---|---|
| **Bore floors lifted by the world-wide `decks` field** (`structure.rs:249,636`): nearest structure axis within 18 m, so a floor takes a crossing motorway's or twin track's height (30 m spikes) | ~33 defects, ~23 400 m² fin, ~1 350 m gap | `flat` + `net:overpass?len=201&leg=standard_gauge`; `ridge?height=40&width=120` + `net:underpass?class=motorway` | C (×2) | per-group field, or the run's own chord as the deck's `top` does |
| **Bore floor runs to the abutment station**, 0–4 m past the span onto the cutting (`structure.rs:212-221,378-381`) | 44 fights (328 m²) + ~64 gaps + 5 | `flat` + `net:underpass` (18.3 m² ×2); ridge + `net:straight?span=0.3,0.7&kind=tunnel` | C (×3) | sweep the floor a0..a1 (`station_at`), square caps |
| A mapped tunnel that never goes under still gets a floor (`structure.rs:216,378`) | 13 (210 m²) | `flat` + `net:straight?span=0.3,0.7&kind=tunnel` (440 m²) | C | floor only over Bore runs; the partition gives the rest back |
| **Deck slab region = reach mask ∩ all spanned paving** (`structure.rs:426-428`): a dual carriageway's slabs run under each other (the user's two 300 m deck gaps); a walk span gets the road's slab | ~25 (2 360 m) | `gorge?depth=30&width=40` + `net:sidewalk?d=6&span=0.35,0.65` | C | the run's own sheet paving only; walks never take a sheet region |
| **Rail slab 1 m wider than its ballast, no top** (`structure.rs:438-446`, `solid_under`) | 52–64 (0.9–1.7 km) | `gorge?depth=30&width=40` + `net:straight?class=standard_gauge` | C (×2) | top the shoulder at deck height, or widen the ballast paving |
| **Walk-span strips**: drawn over every station incl. at grade; no sides; carried by proximity alone; bow-tie at sharp bends (`structure.rs:356-362,706,842-858`) | 102 fights + 159 gaps + 42 flips | `flat` + `net:straight?span=0.3,0.7&kind=tunnel&class=footway`; `gorge?depth=30&width=80` + `net:sidewalk?d=8&span=0.35,0.65` | C | mesh walk spans as closed buffered regions over their Deck/Bore runs only |
| Kerb/room pave into a bore mouth (bores not in `Surface::senior()`) | 1–2 | ridge + `net:tee?d=8&span=0.3&kind=tunnel` | C | bore footprints in `senior()` |
| A street over a portal lifted to the tunnel (6.5 m pit): ground axes extend one station into a bore run (`copies.rs:580`) | 1 site | none | C | no extension into a bore run |
| Two spans crossing: one loses its face (`arrangement.rs:201-228`) | 2 | none | C/L | lower holder in the partition, higher to `decks` |
| Twin rail bores not unioned | 4 | none | L | union twins |

## 3. The bench's faces

| Cause | Loop | Reproducer | | Direction |
|---|---|---|---|---|
| **Bowtie kerb faces**: `bench.rs:193-195` pairs high with high and low with low, so where the two sides cross in height neither rail is either surface's rim | 129 gaps | `ramp?grade=0.05` + `net:level` | C | pair each rail with its own surface (what-if: gap-kerb 145 → 7, nothing else moved); split at the crossing to keep the face turned |
| Mouth/gallery closure: the wall is suppressed where the ground is inside the tube section (`bench.rs:119`) but the tube end is open; gallery ground faces cut and are never drawn | 74 gaps | none | C/L | headwall/footing across the mouth to the tube section; draw the gallery floor |
| **Building foot from the natural ground** (`building.rs:85`); the earthwork lowers the drawn earth by up to 7 m beside footprints | 52 gaps | `ramp?grade=0.6&bearing=0&radius=100000` + `net:sidewalk?d=6` + `house:beside?d=7&l=10&w=6&h=6` | C | the building reads the earthwork's `Ground` (a dependency in `pipeline.rs`) |

## 4. The lift and the earthwork (the real fins)

| Kind | Fin area share | Reproducer | | Direction |
|---|---|---|---|---|
| **Cross-passive weld** (bug): `Copies::weld` keeps the lowest-id copy across rules (`copies.rs:380`), the earthwork regrades it by its own key (`earthwork.rs:274`): corners moved up to 2.5 m | 3.4 % (5) | none | C | do not weld across `passive()` |
| Passive pavement on a steep blended residual: two outline pins 3 m apart blended over `EARTH_BLEND_M` = 1 m (`ground.rs:65,127`) | 41 % | `ramp?grade=0.6&bearing=45&radius=100000` + `net:tee?d=8&hook=5` (weak) | C | a split with a wall where batters disagree past 1:1 — design |
| Passive pavement on a discontinuous residual: the hard `t>0 && t<1 && left>0` cut (`ground.rs:105`) at concave corners | 18 % | `ramp?grade=0.3&bearing=0&radius=100000` + `net:stub?d=0.5` | C | fade the exclusion by signed distance |
| Junction blend over 1:1: `Field::joined` blends legs 4–5 m apart over `BLEND_M` = 4 m (`field.rs:43,353`) | 10.5 % | `ramp?grade=1.5&bearing=45&radius=100000` + `net:roundabout` + `house:row?gap=2` | C | no blend past 1:1; split and draw the face — design |
| Profile infeasible: `solve::limit` dumps the leftover into one segment (275 % grade) (`solve.rs:488-540`) | 1.7 % | `ramp?grade=0.5&bearing=0&radius=100000` + `net:level?rail=narrow_gauge` | C | spread the violation over the run |

## Order proposed

1. **Census corrections (§0)**, then re-baseline the sites.
2. **Local, confirmed, independent fixes in parallel**, each gated by its
   reproducer turning clean, the sites and corpus gate, and the tests:
   bowtie kerbs · bore floors (own field, clip to span, no floor above
   ground) · deck slabs (own sheet, shoulder top, to the span edge) ·
   building foot · cross-passive weld + profile leftover.
3. **Design decisions for the user**, then build: "spanned" from heights
   (§1, ~1 100 defects); walk spans as regions; wall-or-ramp where batters
   or junction legs disagree past 1:1 (§4).
