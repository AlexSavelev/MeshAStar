#!/usr/bin/env python3
"""Freeze task selection before measuring any heuristic performance."""
import hashlib
import json
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'res/multimap-experiment'
SEED = 20261026

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if (OUT / 'manifest.json').exists():
        raise SystemExit('Manifest already exists; refusing to replace the frozen task set.')
    rng = random.Random(SEED)
    tasks, inputs = [], {}
    for mi, name in enumerate(['Berlin_2_256', 'Milan_1_256', 'AR0700SR']):
        mp = ROOT / 'maps' / (name + '.map')
        sp = Path(str(mp) + '.scen')
        for path in [mp, sp]:
            inputs[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        rows = [(line, text.split()) for line, text in enumerate(sp.read_text().splitlines()[1:], 2)]
        rows.sort(key=lambda x: (float(x[1][-1]), x[0]))
        grid = mp.read_text().splitlines()[4:]
        for band, (lo, hi) in enumerate([(.25, .50), (.65, .90)]):
            selected = rng.sample(rows[int(lo*len(rows)):int(hi*len(rows))], 4)
            for k, (line, row) in enumerate(selected):
                sx, sy, gx, gy = map(int, row[4:8])
                assert grid[sy][sx] in '.GS' and grid[gy][gx] in '.GS'
                st, gt = [0, 5, 8, 14][(k+mi+band) % 4], [0, 3, 8, 13][k]
                tasks.append(dict(id=f'{name}-b{band}-h{gt}', map=str(mp.relative_to(ROOT)),
                                  scenario_line=line, band=band, rank_interval=[lo, hi],
                                  grid_reference_distance=float(row[8]), start=[sy, sx, st], goal=[gy, gx, gt]))
    manifest = dict(seed=SEED, repeats=3, maps=3, task_count=len(tasks), inputs_sha256=inputs,
                    selection='Uniform sample of 4 scenario rows in each distance-rank interval; no timing/solvability filtering.',
                    goal_headings=[0, 3, 8, 13], variants=['max', 'kin64', 'kin256'],
                    algorithms=['LBA', 'Mesh'], tasks=tasks)
    (OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (OUT/'tasks.tsv').write_text(''.join(' '.join(map(str, [t['id'], t['map'], *t['start'], *t['goal']]))+'\n' for t in tasks))
    print(f'Frozen {len(tasks)} tasks in {OUT}/manifest.json')

if __name__ == '__main__':
    main()
