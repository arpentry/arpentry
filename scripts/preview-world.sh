#!/usr/bin/env bash
# Preview a world .glb in the browser, and reload it whenever it is rebuilt.
#
#   ./scripts/preview-world.sh                       # /tmp/claude/world.glb
#   ./scripts/preview-world.sh /tmp/claude/town.glb
#   ./scripts/preview-world.sh out.glb --port 9000
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
PORT=8777
OPEN=1
while [ $# -gt 0 ]; do
  case "$1" in
    --port)    PORT="${2:?--port needs a number}"; shift 2;;
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

URL="http://localhost:$PORT/"
echo "serving $(basename "$GLB") at $URL  (ctrl-c to stop)"
[ "$OPEN" = 1 ] && command -v open >/dev/null && (sleep 1; open "$URL") &

# ThreadingHTTPServer since 3.7, which matters: the page polls HEAD every
# 1.5 s while a quarter-gigabyte GET is in flight.
cd "$SERVE"
exec python3 -m http.server "$PORT" --bind 127.0.0.1
