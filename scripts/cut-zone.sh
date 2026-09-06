#!/usr/bin/env bash
# Cut the Switzerland inputs down to one zone, once, so the development loop
# stops paying for the country on every iteration.
#
#   ./scripts/cut-zone.sh montreux 6.86,46.40,6.98,46.47
#   ./scripts/run-overture-ch.sh --zone 6.86,46.40,6.98,46.47 --data data/zones/montreux
#
# WHY THIS EXISTS
#
# `--zone` re-tiles a small bbox, but the tiler still reads the full inputs:
# bbox pruning is at parquet *row-group* granularity, and one row group of a
# 954 MB segment.parquet is a lot of Switzerland. Measured on a 25-tile zone
# (1.5 x 1 km at z16): 13 of 645 row groups survived pruning, 165302 features
# were read, and the model stage — assemble + solve + ground, which runs over
# whatever the scene graph admitted — took 51.6 s of a 74 s run. Shrinking the
# bbox does not move that number. Shrinking the *input* does.
#
# The cut writes small row groups (ROW_GROUP_SIZE below), so pruning inside the
# zone is fine-grained too, and the tiler's own bbox filter still applies on
# top.
#
# THE MARGIN, AND WHAT IT WAS MEASURED AT
#
# Cutting the input changes the scene graph. A corridor that ran off the edge
# now *ends* at the edge, and a corridor end is a modelled thing: profiles are
# solved along it, structures terminate on it, the ground is imprinted from it.
# So the cut carries a margin on every side. Measured on the 25-tile bbox
# 6.91,46.43,6.93,46.44, scoring each cut against the same run over the full
# Switzerland inputs (40 metrics with a population):
#
#   margin 0.02 deg   34/40 identical    model 11.7 s   total 29.5 s
#   margin 0.05 deg   35/40 identical    model 25.5 s   total 45.2 s
#   margin 0.15 deg   35/40 identical    model 51.1 s   total 77.0 s
#
# 0.05 is the default because 0.15 buys nothing: the two agree with each other
# on five of the six metrics where either differs from the full-input run.
# What does not converge is the comparison against full inputs, and the reason
# is that the full-input run is the worse-conditioned of the two. Row-group
# granularity admits corridors from most of the canton — 23611 of them for
# this bbox, against 8506 in the 0.05 cut — and the DEM extract does not reach
# them, so they solve against the flat-0 fallback: that run reported a 300.65 m
# clearance shortfall with 3 demands dropped, where both cuts report ~3 m and
# drop none. Cutting features and DEM to the same bounds removes that by
# construction.
#
# The rule that follows:
#
#   Tile the *zone*. Cut the *zone plus margin*. Compare a cut zone's
#   numbers only against another run over the same cut — a run over full
#   inputs is a different population and a differently-conditioned solve.
#
# `zone.env` records the zone, the margin and the source mtimes, so a stale
# cut is a diff and not a mystery.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."
SRC_DIR="$ROOT_DIR/data/overture-ch"

# Rows per parquet row group in the cut. Small enough that the tiler's bbox
# pruning bites within the zone, large enough that per-group overhead stays
# irrelevant.
ROW_GROUP_SIZE=20000

MARGIN=0.05
TERRAIN_MIN_ZOOM=13
TERRAIN_MAX_ZOOM=18

usage() {
    cat >&2 <<'EOF'
Usage: cut-zone.sh <name> <w,s,e,n> [options]

  --margin <deg>        Extra ground cut on every side (default 0.05, ~5.5 km)
  --src <dir>           Source data directory (default data/overture-ch)
  --terrain <file>      DEM to cut (default <src>/terrain-hires.pmtiles)
  --terrain-zooms <a,b> DEM zoom range to cut (default 13,18)
  --force               Re-cut even when the output is present and current
EOF
    exit 2
}

[ $# -ge 2 ] || usage
NAME="$1"
ZONE="$2"
shift 2

TERRAIN_SRC=""
FORCE=false
while [ $# -gt 0 ]; do
    case "$1" in
        --margin)        MARGIN="$2"; shift 2 ;;
        --src)           SRC_DIR="$2"; shift 2 ;;
        --terrain)       TERRAIN_SRC="$2"; shift 2 ;;
        --terrain-zooms) TERRAIN_MIN_ZOOM="${2%%,*}"; TERRAIN_MAX_ZOOM="${2##*,}"; shift 2 ;;
        --force)         FORCE=true; shift ;;
        *) echo "unknown option $1" >&2; usage ;;
    esac
done

[ -n "$TERRAIN_SRC" ] || TERRAIN_SRC="$SRC_DIR/terrain-hires.pmtiles"
OUT_DIR="$ROOT_DIR/data/zones/$NAME"

command -v duckdb >/dev/null || { echo "ERROR: duckdb not found (brew install duckdb)" >&2; exit 1; }

# ── The zone, and the ground actually cut ────────────────────────────────────

IFS=',' read -r W S E N <<< "$ZONE"
for v in "$W" "$S" "$E" "$N"; do
    case "$v" in ''|*[!0-9.+-]*) echo "ERROR: bad bbox '$ZONE'" >&2; exit 1 ;; esac
done
CW=$(python3 -c "print($W - $MARGIN)")
CS=$(python3 -c "print($S - $MARGIN)")
CE=$(python3 -c "print($E + $MARGIN)")
CN=$(python3 -c "print($N + $MARGIN)")

echo "Zone   $W,$S,$E,$N"
echo "Cut    $CW,$CS,$CE,$CN  (margin $MARGIN deg)"
echo "Out    $OUT_DIR"
echo ""

mkdir -p "$OUT_DIR"

# ── Parquet ──────────────────────────────────────────────────────────────────
#
# The filter is on the `bbox` struct Overture ships, not on the geometry, so
# it reads column statistics rather than decoding WKB. Every column is carried
# through unchanged: the tiler reads the `geometry` BLOB and prunes on
# `bbox.*` statistics, and needs no GeoParquet file metadata, so a plain COPY
# round-trips everything it looks at.

cut_parquet() {
    local file="$1"
    local src="$SRC_DIR/$file"
    local dst="$OUT_DIR/$file"
    if [ ! -f "$src" ]; then
        echo "  $file: absent at source, skipping"
        return
    fi
    if [ -f "$dst" ] && [ "$FORCE" = false ] && [ "$dst" -nt "$src" ]; then
        echo "  $file: current ($(du -h "$dst" | cut -f1)), skipping"
        return
    fi
    duckdb -c "
        COPY (
            SELECT * FROM read_parquet('$src')
            WHERE bbox.xmin <= $CE AND bbox.xmax >= $CW
              AND bbox.ymin <= $CN AND bbox.ymax >= $CS
        ) TO '$dst' (FORMAT PARQUET, COMPRESSION ZSTD, ROW_GROUP_SIZE $ROW_GROUP_SIZE);
    " >/dev/null
    local rows
    rows=$(duckdb -noheader -list -c "SELECT count(*) FROM read_parquet('$dst');")
    printf "  %-28s %8s  %s rows\n" "$file" "$(du -h "$dst" | cut -f1)" "$rows"
}

echo "Cutting parquet:"
for f in land_cover.parquet land_use.parquet water.parquet segment.parquet \
         building.parquet place.parquet division_boundary.parquet; do
    cut_parquet "$f"
done
echo ""

# ── Terrain ──────────────────────────────────────────────────────────────────
#
# Cut over the same margin. A DEM that stops at the zone edge gives the tiles
# on the boundary a sea-level fallback, which reads as a cliff — the failure
# mode a stale terrain extract already cost this project once.

TERRAIN_DST="$OUT_DIR/terrain.pmtiles"
if [ ! -f "$TERRAIN_SRC" ]; then
    echo "Terrain: $TERRAIN_SRC absent, skipping (tile with --no-terrain)"
elif [ -f "$TERRAIN_DST" ] && [ "$FORCE" = false ] && [ "$TERRAIN_DST" -nt "$TERRAIN_SRC" ]; then
    echo "Terrain: current ($(du -h "$TERRAIN_DST" | cut -f1)), skipping"
else
    echo "Cutting terrain z$TERRAIN_MIN_ZOOM-$TERRAIN_MAX_ZOOM from $(basename "$TERRAIN_SRC")..."
    pmtiles extract "$TERRAIN_SRC" "$TERRAIN_DST" \
        --bbox="$CW,$CS,$CE,$CN" \
        --minzoom="$TERRAIN_MIN_ZOOM" --maxzoom="$TERRAIN_MAX_ZOOM" --quiet
    echo "  terrain.pmtiles $(du -h "$TERRAIN_DST" | cut -f1)"
fi
echo ""

# ── Provenance ───────────────────────────────────────────────────────────────
#
# What this cut is, and what it was cut from. A cut that has fallen behind its
# source is then a diff rather than a puzzle.

{
    echo "# Written by scripts/cut-zone.sh on $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "ZONE_NAME=$NAME"
    echo "ZONE_BBOX=$W,$S,$E,$N"
    echo "ZONE_CUT_BBOX=$CW,$CS,$CE,$CN"
    echo "ZONE_MARGIN=$MARGIN"
    echo "ZONE_SRC=$SRC_DIR"
    echo "ZONE_TERRAIN_SRC=$TERRAIN_SRC"
    echo "ZONE_TERRAIN_ZOOMS=$TERRAIN_MIN_ZOOM,$TERRAIN_MAX_ZOOM"
    echo "# Source mtimes at cut time:"
    for f in land_cover land_use water segment building place division_boundary; do
        [ -f "$SRC_DIR/$f.parquet" ] || continue
        echo "#   $f.parquet $(date -u -r "$SRC_DIR/$f.parquet" +%Y-%m-%dT%H:%M:%SZ)"
    done
    [ -f "$TERRAIN_SRC" ] && echo "#   $(basename "$TERRAIN_SRC") $(date -u -r "$TERRAIN_SRC" +%Y-%m-%dT%H:%M:%SZ)"
} > "$OUT_DIR/zone.env"

echo "Zone cut: $OUT_DIR ($(du -sh "$OUT_DIR" | cut -f1))"
echo ""
echo "Tile it with:"
echo "  ./scripts/run-overture-ch.sh --zone $ZONE --data $OUT_DIR"
echo ""
echo "Scorecards from this zone compare only against other runs over the same"
echo "cut — the margin bounds the boundary effect, it does not remove it."
