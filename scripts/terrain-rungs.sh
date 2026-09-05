#!/usr/bin/env bash
# Tile one site at every rung of the terrain dial, and score each.
#
# The isolation harness's one moving part (data/plans/isolation-harness-2026-09-04.md
# §"The model"): hold the features fixed — real Overture geometry, one bbox, one
# cut — and change only the ground under them, from a plane to the real DEM. A
# metric that is already off at `flat` is off in the *construction*; one that
# only leaves zero at `hill` or `step` is off in the ground, or in how the ground
# is read. Nothing that reads a single real extract can tell those apart, which
# is why band_deck_bare and the structure_drift "regression" each cost days.
#
# Usage:
#   ./scripts/terrain-rungs.sh <name> <w,s,e,n> [options]
#
# Options:
#   --data <dir>      Layer parquets + terrain.pmtiles (default data/zones/montreux)
#   --out <dir>       Where the archives and scorecards go (default $TMPDIR/rungs)
#   --zoom <z>        Tile this zoom only (default 16, the detail rung)
#   --base <m>        Height of the synthetic grounds at the site (default 400)
#   --rungs <list>    Comma-separated subset of: flat,ramp,hill,step,dem
#   --at <lon,lat>    Score around this point instead of the bbox centre
#   --no-model        Skip the model half (--verify-model). It is ~80 % of the
#                     wall time, and it scores the whole solved scene — every
#                     corridor the row groups admitted — rather than the tiles
#                     this bbox emitted, so its rate is not this site's.
#
# Each rung writes <out>/<name>-<rung>.arpa and <out>/<name>-<rung>.json, and the
# run ends with the table that is the actual output: metric by rung.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
TILER="$ROOT_DIR/server/target/release/arpentry_tiler"
VERIFY="$ROOT_DIR/server/target/release/arpentry_verify"

if [ $# -lt 2 ]; then
    sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
fi

NAME="$1"
BBOX="$2"
shift 2

DATA_DIR="$ROOT_DIR/data/zones/montreux"
OUT_DIR="${TMPDIR:-/tmp}/rungs"
ZOOM=16
BASE=400
RUNGS="flat,ramp,hill,step,dem"
AT=""
MODEL=true

while [ $# -gt 0 ]; do
    case "$1" in
        --data) DATA_DIR="$2"; shift 2 ;;
        --out) OUT_DIR="$2"; shift 2 ;;
        --zoom) ZOOM="$2"; shift 2 ;;
        --base) BASE="$2"; shift 2 ;;
        --rungs) RUNGS="$2"; shift 2 ;;
        --at) AT="$2"; shift 2 ;;
        --no-model) MODEL=false; shift ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "$OUT_DIR"

# The site's centre, for the synthetic grounds' origin and for --at. Every rung
# is anchored at the same point, so `hill` peaks where `ramp` crosses its base
# and the rungs differ only in shape.
CLON=$(awk -F, '{printf "%.6f", ($1+$3)/2}' <<<"$BBOX")
CLAT=$(awk -F, '{printf "%.6f", ($2+$4)/2}' <<<"$BBOX")
# --at is left empty by default: the bbox *is* the site, so scoring the whole
# archive keeps the cross-tile seam checks, which --at blinds.

# The dial. Each is a pure function of position (server/src/dem/field.rs), so a
# rung is reproducible from its string alone.
#   flat  the plane. NOT the same as omitting --terrain: that skips the DEM path
#         entirely and takes the flat-0 fallback, while `flat` walks the real
#         sampling path and gets a constant back. This rung isolates the ground's
#         *value*, not the code path — anything nonzero here is drawn wrong
#   ramp  3 %: a street's own grade, the mildest ground that is not a plane
#   hill  60 m over 400: curvature, so cut and fill both appear on one site
#   step  3 m cliff: the discontinuity every bench and batter rule is written for
#   dem   the real ground, for scale
spec_for() {
    case "$1" in
        flat) echo "flat?h=$BASE&at=$CLON,$CLAT" ;;
        ramp) echo "ramp?grade=0.03&bearing=90&h=$BASE&at=$CLON,$CLAT" ;;
        hill) echo "hill?amp=60&radius=400&h=$BASE&at=$CLON,$CLAT" ;;
        step) echo "step?rise=3&bearing=90&h=$BASE&at=$CLON,$CLAT" ;;
        dem) echo "$DATA_DIR/terrain.pmtiles" ;;
        *) echo "unknown rung: $1" >&2; exit 2 ;;
    esac
}

inputs=()
add_input() { [ -f "$2" ] && inputs+=(--input "$1:$2") || echo "  (no $(basename "$2"))"; }
add_input 1 "$DATA_DIR/land_cover.parquet"
add_input 6 "$DATA_DIR/land_use.parquet"
add_input 3 "$DATA_DIR/water.parquet"
add_input 5 "$DATA_DIR/segment.parquet"
add_input 7 "$DATA_DIR/building.parquet"
add_input 8 "$DATA_DIR/place.parquet"
add_input 9 "$DATA_DIR/division_boundary.parquet"

echo "site $NAME  bbox $BBOX  centre $CLON,$CLAT  z$ZOOM  data $(basename "$DATA_DIR")"

IFS=',' read -ra rungs <<<"$RUNGS"
for rung in "${rungs[@]}"; do
    arpa="$OUT_DIR/$NAME-$rung.arpa"
    spec="$(spec_for "$rung")"
    echo ""
    echo "── $rung ── $spec"
    rm -f "$arpa" "$OUT_DIR/$NAME-$rung.model.json"
    model_arg=()
    verify_model_arg=()
    if [ "$MODEL" = true ]; then
        model_arg=(--verify-model "$OUT_DIR/$NAME-$rung.model.json")
        verify_model_arg=(--model "$OUT_DIR/$NAME-$rung.model.json")
    fi
    "$TILER" \
        --output "$arpa" \
        --bbox "$BBOX" \
        --min-zoom "$ZOOM" \
        --max-zoom "$ZOOM" \
        --mem $((256 * 1024 * 1024)) \
        ${model_arg[@]+"${model_arg[@]}"} \
        --terrain "$spec" \
        "${inputs[@]}" >"$OUT_DIR/$NAME-$rung.log" 2>&1
    grep -E "^(model|ground|consistency|cdt|residuals|done)" "$OUT_DIR/$NAME-$rung.log" || true
    "$VERIFY" "$arpa" ${AT:+--at "$AT"} \
        ${verify_model_arg[@]+"${verify_model_arg[@]}"} \
        --json "$OUT_DIR/$NAME-$rung.json" >/dev/null
done

echo ""
echo "Scorecards: $OUT_DIR/$NAME-<rung>.json"
"$ROOT_DIR/scripts/rung-table.py" "$OUT_DIR/$NAME" "${rungs[@]}"
