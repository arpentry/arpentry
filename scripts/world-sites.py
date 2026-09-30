#!/usr/bin/env python3
"""world-sites.py: small windows around the census's defects, and the gate.

A site is a window a few hundred metres across around one defect the census
found in a large run. It runs in half a second where the loop box takes forty,
so a fix is iterated on the site and checked on all of them.

  world-sites.py extract CENSUS.json SITES.tsv [--per 2] [--half 150] [--species gap,fin]
      Per signature (species + layers) the largest few defects, each as a
      window of 2*half metres. A defect inside a window already chosen adds
      nothing.

  world-sites.py run SITES.tsv OUTDIR [--bin BIN] [--zone DIR] [--jobs N] [--prune]
      Runs every site, N at a time, writing NAME.txt (the step lines),
      NAME.glb and NAME.census.json, and says whether the defect the site
      was cut for came back. A window is a different world from the box it
      was cut from — the clip ends ways, and a small bbox gets the 2 m
      lattice the box's vertex cap coarsened — so a site that does not
      reproduce is reported, never assumed; `--prune` drops it from
      SITES.tsv.

  world-sites.py gate BASE NEW
      BASE and NEW are OUTDIRs of `run` or of scripts/world-corpus.sh (any
      directory of *.census.json). Per site, per charged species, the count
      and the size may not rise; exits 1 if one does. Defects within
      EDGE_M of a window's border are the clip's and are left out.

  scripts/preview-world.sh OUTDIR/NAME.glb draws a site with its census.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

DEG_M = 111_320.0      # arpentry_server::scene::DEG_M, which the world's frame uses
EDGE_M = 25.0          # a defect this close to a window's border is the clip's
REPRO_M = 10.0         # the defect a site was cut for is back if one of its species lies this close
CHARGED = ['gap', 'fin', 'flip', 'buried', 'fight', 'overlap']


def weight(d):
    """How much of a defect there is to see: a gap's length times its width."""
    return d['size'] * d['width'] if d['species'] == 'gap' else d['size']


def metres(lat0, lon0, lat, lon):
    """(east, north) of (lat, lon) from (lat0, lon0), in the world's frame."""
    return ((lon - lon0) * math.cos(math.radians(lat0)) * DEG_M, (lat - lat0) * DEG_M)


def window(lat, lon, half):
    dlat = half / DEG_M
    dlon = half / (DEG_M * math.cos(math.radians(lat)))
    return (lon - dlon, lat - dlat, lon + dlon, lat + dlat)


def extract(args):
    census = json.load(open(args.census))
    species = set(args.species.split(',')) if args.species else set(CHARGED)
    by_sig = {}
    for d in census['defects']:
        if d['species'] in species:
            by_sig.setdefault((d['species'], '+'.join(d['layers'])), []).append(d)
    sites = []
    # Signatures with the most defects first: a cause behind many is the one
    # a site should be kept for when two windows would overlap.
    for (sp, layers), ds in sorted(by_sig.items(), key=lambda kv: -len(kv[1])):
        taken = 0
        for d in sorted(ds, key=weight, reverse=True):
            if taken == args.per:
                break
            if any(abs(metres(s['lat'], s['lon'], d['lat'], d['lon'])[0]) < args.half * 0.8
                   and abs(metres(s['lat'], s['lon'], d['lat'], d['lon'])[1]) < args.half * 0.8
                   and s['species'] == sp for s in sites):
                continue
            taken += 1
            name = re.sub(r'[^a-z0-9]+', '-', f"{sp}-{layers}-{taken}")
            sites.append({'name': name, 'species': sp, 'layers': layers, 'lat': d['lat'], 'lon': d['lon'],
                          'size': weight(d), 'count': len(ds)})
    with open(args.sites, 'w') as f:
        f.write('# name\tbbox\tspecies\tlayers\tlat\tlon\tweight\tof_signature\n')
        for s in sites:
            bbox = ','.join(f'{v:.6f}' for v in window(s['lat'], s['lon'], args.half))
            f.write(f"{s['name']}\t{bbox}\t{s['species']}\t{s['layers']}\t{s['lat']:.7f}\t{s['lon']:.7f}"
                    f"\t{s['size']:.3f}\t{s['count']}\n")
    print(f'{len(sites)} sites from {len(by_sig)} signatures -> {args.sites}')


def read_sites(path):
    out = []
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        name, bbox, sp, layers, lat, lon = line.rstrip('\n').split('\t')[:6]
        out.append({'name': name, 'bbox': bbox, 'species': sp, 'layers': layers,
                    'lat': float(lat), 'lon': float(lon)})
    return out


def interior(census):
    """The defects not within EDGE_M of the census's own bbox."""
    w, s, e, n = census['bbox']
    lat0 = (s + n) / 2
    out = []
    for d in census['defects']:
        x0, y0 = metres(lat0, w, d['lat'], d['lon'])
        x1, y1 = metres(lat0, e, d['lat'], d['lon'])
        if min(x0, -x1, (d['lat'] - s) * DEG_M, (n - d['lat']) * DEG_M) > EDGE_M:
            out.append(d)
    return out


def reproduced(census, site):
    """Whether a defect of the site's species lies within REPRO_M of the one
    it was cut for — by its point, its edges or its triangles."""
    w, s, e, n = census['bbox']
    lat0, lon0 = (s + n) / 2, (w + e) / 2
    tx, ty = metres(lat0, lon0, site['lat'], site['lon'])
    for d in census['defects']:
        if d['species'] != site['species']:
            continue
        pts = [d['at']] + [p for l in d['lines'] for p in l] + [p for t in d['triangles'] for p in t]
        if any(math.hypot(p[0] - tx, p[1] - ty) <= REPRO_M for p in pts):
            return True
    return False


def run_one(args, site):
    """Runs one site; its report line, and whether its defect came back."""
    base = os.path.join(args.outdir, site['name'])
    out = subprocess.run(
        [args.bin, '--zone', args.zone, '--bbox', site['bbox'], '--output', base + '.glb',
         '--census', base + '.census.json'],
        capture_output=True, text=True)
    if out.returncode != 0:
        return f"{site['name']:<36} FAILED: {out.stderr.strip()[:200]}", False
    # The step lines without their timings, as world-corpus.sh keeps them.
    lines = [re.sub(r'  [0-9.]+s(  .*)?$', '', l) for l in out.stdout.splitlines()]
    open(base + '.txt', 'w').write('\n'.join(lines) + '\n')
    census = json.load(open(base + '.census.json'))
    score = tally(interior(census))
    back = reproduced(census, site)
    return (f"{site['name']:<36} {'back  ' if back else 'ABSENT'} "
            + ' '.join(f"{k}={v[0]}" for k, v in score.items() if v[0])), back


def run(args):
    from concurrent.futures import ThreadPoolExecutor
    os.makedirs(args.outdir, exist_ok=True)
    sites = read_sites(args.sites)
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda s: run_one(args, s), sites))
    for line, _ in results:
        print(line)
    kept = [s for s, (_, back) in zip(sites, results) if back]
    print(f'{len(kept)}/{len(sites)} sites reproduce')
    if args.prune and len(kept) < len(sites):
        header = [l for l in open(args.sites) if l.startswith('#')]
        body = [l for l in open(args.sites) if not l.startswith('#') and l.strip()]
        names = {s['name'] for s in kept}
        with open(args.sites, 'w') as f:
            f.writelines(header + [l for l in body if l.split('\t', 1)[0] in names])
        print(f'pruned {args.sites} to the {len(kept)} that reproduce')


def tally(defects):
    out = {sp: [0, 0.0] for sp in CHARGED}
    for d in defects:
        if d['species'] in out:
            out[d['species']][0] += 1
            out[d['species']][1] += d['size']
    return out


def gate(args):
    names = sorted(f[:-len('.census.json')] for f in os.listdir(args.base) if f.endswith('.census.json'))
    worse, better = [], []
    for n in names:
        new_path = os.path.join(args.new, n + '.census.json')
        if not os.path.exists(new_path):
            worse.append(f'{n}: missing from {args.new}')
            continue
        a = tally(interior(json.load(open(os.path.join(args.base, n + '.census.json')))))
        b = tally(interior(json.load(open(new_path))))
        for sp in CHARGED:
            (ca, sa), (cb, sb) = a[sp], b[sp]
            if (ca, round(sa, 3)) == (cb, round(sb, 3)):
                continue
            line = f'{n:<34} {sp:<8} {ca}/{sa:.3f} -> {cb}/{sb:.3f}'
            (worse if cb > ca or sb > sa + 1e-3 else better).append(line)
    for l in better:
        print('  better ', l)
    for l in worse:
        print('  WORSE  ', l)
    print(f'{len(names)} sites: {len(better)} better, {len(worse)} worse')
    sys.exit(1 if worse else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    e = sub.add_parser('extract')
    e.add_argument('census')
    e.add_argument('sites')
    e.add_argument('--per', type=int, default=2)
    e.add_argument('--half', type=float, default=150.0)
    e.add_argument('--species')
    r = sub.add_parser('run')
    r.add_argument('sites')
    r.add_argument('outdir')
    r.add_argument('--bin', default='./world/target/release/arpentry_world')
    r.add_argument('--zone', default='data/zones/montreux')
    r.add_argument('--jobs', type=int, default=os.cpu_count())
    r.add_argument('--prune', action='store_true')
    g = sub.add_parser('gate')
    g.add_argument('base')
    g.add_argument('new')
    args = p.parse_args()
    {'extract': extract, 'run': run, 'gate': gate}[args.cmd](args)


if __name__ == '__main__':
    main()
