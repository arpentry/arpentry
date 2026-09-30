#!/bin/bash
# Runs the world crate's specimen corpus and the Montreux junction box, and
# writes one summary line file, one .glb and one census per specimen into
# OUTDIR. The census line is the last of each .txt, so world-sdiff.py reads
# what a viewer would see move beside what each step reports; the
# .census.json locates every defect (scripts/preview-world.sh draws it).
#
#   scripts/world-corpus.sh [BIN] OUTDIR
#
# The output is byte-deterministic, so two runs of one binary must be
# identical, and two binaries differ exactly where the geometry moved:
#
#   scripts/world-corpus.sh /tmp/claude/before && <change, rebuild> &&
#   scripts/world-corpus.sh /tmp/claude/after &&
#   scripts/world-sdiff.py /tmp/claude/before /tmp/claude/after
#
# The specimens were chosen to reach every step: flat and sloped junctions,
# a sidewalk, an overpass and an underpass, a gorge and a ridge, a cliff, a
# level crossing, a house the way passes through, and a roundabout on a 150 %
# flank — the corpus's cheapest route to the defects the real world reaches
# by combining cross-slope, profile departure and clearance lift.
set -euo pipefail
cd "$(dirname "$0")/.."
if [ $# -eq 1 ]; then BIN=./world/target/release/arpentry_world; OUT=$1; else BIN=$1; OUT=$2; fi
mkdir -p "$OUT"
BB=6.91,46.43,6.93,46.44
run() {
    name=$1; shift
    "$BIN" --bbox $BB --spacing 5 --output "$OUT/$name.glb" --census "$OUT/$name.census.json" "$@" 2>&1 \
        | sed -E 's/  [0-9.]+s$//; s/  [0-9.]+s  .*//' > "$OUT/$name.txt"
}
run cross       --terrain 'ramp?grade=0.15&bearing=45' --segments net:cross
run sidewalk    --terrain 'ramp?grade=0.3&bearing=45' --segments 'net:sidewalk?d=6'
run roundabout  --terrain 'ramp?grade=1.5&bearing=45&radius=100000' --segments net:roundabout --buildings 'house:row?gap=2'
run overpass    --terrain flat --segments 'net:overpass?len=201'
run underpass   --terrain flat --segments net:underpass
run gorge       --terrain 'gorge?depth=30&width=40' --segments net:straight
run ridge       --terrain 'ridge?height=40&width=120' --segments 'net:straight?class=motorway'
run step        --terrain 'step?rise=10' --segments 'net:sidewalk?d=6'
run level       --terrain 'ramp?grade=0.05' --segments net:level
run across      --terrain 'hill?amp=20&radius=300' --segments net:tee --buildings 'house:across?rot=30'
# The last --bbox wins: the Montreux junction box, on the cut zone.
run junction    --zone data/zones/montreux --bbox 6.9110,46.4280,6.9190,46.4330 --spacing 2
