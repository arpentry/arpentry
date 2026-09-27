#!/usr/bin/env python3
"""world-sdiff.py A B [names...]: per specimen, per step, the summary values that moved.

A and B are directories written by scripts/world-corpus.sh. Every value of
every step's line is compared; a value that did not move is not printed, so
an empty output is two identical corpora.
"""
import os
import re
import sys


def parse(path):
    out = {}
    for line in open(path):
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        step, rest = parts
        # Values may hold spaces, like "34631 (28.5%)": split on " key=".
        out[step] = dict(re.findall(r'(\S+?)=(.*?)(?= \S+?=|$)', rest.strip()))
    return out


def main():
    a, b = sys.argv[1], sys.argv[2]
    names = sys.argv[3:] or sorted(f[:-4] for f in os.listdir(a) if f.endswith('.txt'))
    for n in names:
        A, B = parse(f'{a}/{n}.txt'), parse(f'{b}/{n}.txt')
        moved = [
            f'  {step:<11} {k}: {v} -> {B.get(step, {}).get(k)}'
            for step in A
            if step != 'gltf'
            for k, v in A[step].items()
            if B.get(step, {}).get(k) != v
        ]
        moved += [f'  {step:<11} {k}: (new) {v}' for step in B if step != 'gltf'
                  for k, v in B[step].items() if k not in A.get(step, {})]
        if moved:
            print(f'=== {n}')
            print('\n'.join(moved))


if __name__ == '__main__':
    main()
