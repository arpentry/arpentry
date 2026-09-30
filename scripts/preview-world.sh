#!/usr/bin/env bash
# Preview a world .glb in the browser, and reload it whenever it is rebuilt.
#
#   ./scripts/preview-world.sh                       # /tmp/claude/world.glb
#   ./scripts/preview-world.sh /tmp/claude/town.glb
#   ./scripts/preview-world.sh out.glb --port 9000
#   ./scripts/preview-world.sh out.glb --census out.census.json
#   ./scripts/preview-world.sh base.glb --compare ramp=a/x.glb --compare wall=b/x.glb
#
# `--compare LABEL=GLB` loads another build of the same place beside the
# watched one (its census is GLB's name with .census.json); `x` cycles the
# builds in place, same camera, and the census follows the build shown.
#
# A run's `--census FILE` (by default the .glb's name with .census.json) is
# drawn over the model: every defect in its species' colour, a list to step
# through (n / p), and a verdict per defect (r real, f fine, c clear) kept
# in the browser, saved with the `verdicts` button and forgotten with `clear`.
#
# Leave the page open. Rebuild the .glb in another shell and it reloads by
# itself, keeping the camera where you left it:
#
#   ./world/target/release/arpentry_world --zone data/zones/montreux \
#       --bbox 6.905,46.425,6.925,46.437 --spacing 2 --output /tmp/claude/town.glb
#
# WHY THIS EXISTS
#
# Quick Look and Preview are what a .glb opens in on a Mac, and they are the
# wrong instrument twice. They draw glTF LINES as triangles, so `--outlines`
# shoots long shards across the model (docs say so, and it is why outlines
# are off by default) — three.js draws them correctly. And they have no
# watch: every look at a change costs an export, an open, and finding the
# camera again. Here the camera survives the reload, which is the whole
# point when the thing you are judging is a metre of kerb.
#
# The page is served rather than opened as a file:// because a module
# import map and fetch() both need an origin. The serve directory holds two
# symlinks — the page, and the chosen .glb as model.glb — so nothing is
# copied and nothing is written next to your output.

set -euo pipefail

GLB=""
CENSUS=""
COMPARE=()
LABEL="current"
PORT=8777
OPEN=1
while [ $# -gt 0 ]; do
  case "$1" in
    --port)    PORT="${2:?--port needs a number}"; shift 2;;
    --census)  CENSUS="${2:?--census needs a path}"; shift 2;;
    --compare) COMPARE+=("${2:?--compare needs LABEL=GLB}"); shift 2;;
    --label)   LABEL="${2:?--label needs a name}"; shift 2;;
    --no-open) OPEN=0; shift;;                      # over ssh, or under a test
    -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0;;
    -*)        echo "error: unknown flag $1" >&2; exit 2;;
    *)         GLB="$1"; shift;;
  esac
done
GLB="${GLB:-/tmp/claude/world.glb}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PAGE="$ROOT/scripts/preview/index.html"
[ -f "$PAGE" ] || { echo "error: missing $PAGE" >&2; exit 1; }

# An absolute path, so the symlink keeps resolving after we cd. The file
# need not exist yet: the page reports it and picks it up when it appears,
# which means you can start this first and build into it.
case "$GLB" in /*) ;; *) GLB="$PWD/$GLB";; esac
[ -e "$GLB" ] || echo "note: $GLB does not exist yet — the page will wait for it"

SERVE="$(mktemp -d "${TMPDIR:-/tmp}/arpentry-preview.XXXXXX")"
trap 'rm -rf "$SERVE"' EXIT
ln -s "$PAGE" "$SERVE/index.html"
ln -s "$GLB"  "$SERVE/model.glb"
# The census the run wrote with `--census`: by default the .glb's name with
# .census.json for .glb. Linked whether or not it exists yet; the page draws
# it when it is there and does without when it is not.
CENSUS="${CENSUS:-${GLB%.glb}.census.json}"
case "$CENSUS" in /*) ;; *) CENSUS="$PWD/$CENSUS";; esac
ln -s "$CENSUS" "$SERVE/census.json"

# models.json lists the watched build first, then the ones to compare.
if [ ${#COMPARE[@]} -gt 0 ]; then
  {
    printf '[{"label":"%s","glb":"model.glb","census":"census.json"}' "$LABEL"
    i=0
    for pair in "${COMPARE[@]}"; do
      label="${pair%%=*}"; glb="${pair#*=}"
      case "$glb" in /*) ;; *) glb="$PWD/$glb";; esac
      [ -e "$glb" ] || { echo "error: $glb does not exist" >&2; exit 1; }
      ln -s "$glb" "$SERVE/compare-$i.glb"
      ln -s "${glb%.glb}.census.json" "$SERVE/compare-$i.census.json"
      printf ',{"label":"%s","glb":"compare-%s.glb","census":"compare-%s.census.json"}' "$label" "$i" "$i"
      i=$((i + 1))
    done
    printf ']\n'
  } > "$SERVE/models.json"
fi

URL="http://localhost:$PORT/"
echo "serving $(basename "$GLB") at $URL  (ctrl-c to stop)"
[ "$OPEN" = 1 ] && command -v open >/dev/null && (sleep 1; open "$URL") &

# ThreadingHTTPServer since 3.7, which matters: the page polls HEAD every
# 1.5 s while a quarter-gigabyte GET is in flight.
cd "$SERVE"
exec python3 -m http.server "$PORT" --bind 127.0.0.1
