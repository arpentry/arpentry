#!/usr/bin/env python3
"""Print one site's scorecards across the terrain rungs as a single table.

The table is the whole point of the terrain dial. Five scorecards read one
after another say almost nothing; side by side they say *where a metric first
leaves zero*, and that rung is the metric's real threshold — the ground it
needs before it has anything to complain about.

    rung-table.py <out>/<name> flat ramp hill step dem

Reads `<prefix>-<rung>.json` (as written by `arpentry_verify --json`) and prints
the violation rate of every metric that is nonzero at some rung, plus the worst
sample where a rate alone would hide a tail. Metrics flat at zero across every
rung are counted, not listed: on this site, they have nothing to say.

A `*` marks the model half — the checks merged in from `--verify-model`. They
score the solved *scene*, and the scene is everything the row groups admitted,
not the handful of tiles the bbox emitted. Their rung-to-rung diff is honest
(the ground moved under all of it) but their absolute rate belongs to the whole
cut and must not be read as this site's.
"""

import json
import sys


def load(prefix, rung):
    with open(f"{prefix}-{rung}.json") as f:
        card = json.load(f)
    return {m["id"]: m for m in card["metrics"]}


def model_ids(prefix, rungs):
    """The ids that came from `--verify-model`, so the table can mark them."""
    ids = set()
    for rung in rungs:
        try:
            with open(f"{prefix}-{rung}.model.json") as f:
                ids.update(m["id"] for m in json.load(f)["metrics"])
        except (FileNotFoundError, KeyError):
            pass
    return ids


def cell(metric):
    if metric is None:
        return "—"
    if metric.get("skipped"):
        return "·"
    pct = metric.get("violation_pct")
    if pct is None:
        return "·"
    if metric.get("samples", 0) == 0:
        return "·"
    return f"{pct:.2f}"


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    prefix, rungs = sys.argv[1], sys.argv[2:]

    cards = {}
    for rung in rungs:
        try:
            cards[rung] = load(prefix, rung)
        except FileNotFoundError:
            pass
    rungs = [r for r in rungs if r in cards]
    if not rungs:
        print(f"no scorecards at {prefix}-<rung>.json")
        return 1

    ids = []
    for rung in rungs:
        for mid in cards[rung]:
            if mid not in ids:
                ids.append(mid)

    # A metric earns a row by being nonzero somewhere, or by being measurable at
    # one rung and not another — a check that goes silent is as interesting as
    # one that goes red.
    rows, quiet, unmeasured = [], 0, []
    for mid in ids:
        rates = [cards[r].get(mid) for r in rungs]
        measured = [m for m in rates if m and m.get("samples", 0) > 0]
        if not measured:
            unmeasured.append(mid)
            continue
        if all((m.get("violation_pct") or 0.0) == 0.0 for m in measured) and len(
            measured
        ) == len(rungs):
            quiet += 1
            continue
        rows.append((mid, rates, measured))

    scene = model_ids(prefix, rungs)
    w = max([len(mid) + 2 for mid, _, _ in rows] + [8])
    head = f"{'metric':<{w}}  " + "  ".join(f"{r:>7}" for r in rungs) + "   worst"
    print()
    print(head)
    print("-" * len(head))
    for mid, rates, measured in rows:
        cells = "  ".join(f"{cell(m):>7}" for m in rates)
        worst = "  ".join(
            f"{(m.get('worst') or 0.0):.2f}" if m else "—" for m in rates
        )
        name = f"{mid} *" if mid in scene else mid
        print(f"{name:<{w}}  {cells}   {worst}")

    print()
    print(
        f"{quiet} metrics zero at every rung; "
        f"{len(unmeasured)} unmeasured here ({', '.join(unmeasured[:6])}"
        f"{'…' if len(unmeasured) > 6 else ''})"
    )
    print("cells are violation %, then the worst sample per rung in metres")
    print("* = model half (--verify-model): scored over the whole solved scene, not this bbox")
    return 0


if __name__ == "__main__":
    sys.exit(main())
