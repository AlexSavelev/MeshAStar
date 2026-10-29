#!/usr/bin/env python3
"""Paired timing of old and nonempty directional potentials on 24 old tasks."""
import hashlib
import json
import random
import statistics
import time
from pathlib import Path
from run_benchmark import ROOT,isolated,append

OUT=ROOT/'res/directional-nonempty-timing-2026-10-27'
FROZEN=ROOT/'res/full-run-2026-10-27/mesha-team-benchmark'
TASKS=ROOT/'res/directional-nonempty/tasks.json'
BINARIES={'baseline':FROZEN/'build-benchmark/benchmark_worker',
          'nonempty':ROOT/'res/directional-nonempty/source/build-benchmark/benchmark_worker'}


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert json.loads((FROZEN/'results/full-status.json').read_text())['active_stage'] is None
    OUT.mkdir(exist_ok=True)
    assert not (OUT/'runs.jsonl').exists(),'Use a new output directory; no overwrite.'
    tasks=json.loads(TASKS.read_text())
    metadata=dict(repeats=5,seed=2026102702,tasks_sha256=sha(TASKS),
        binary_sha256={k:sha(p) for k,p in BINARIES.items()},
        warmup=1,timeout_s=120,started_epoch=time.time(),
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in
            list((ROOT/'res/directional-nonempty/source/src').glob('*.cpp'))+
            list((ROOT/'res/directional-nonempty/source/include').glob('*.hpp'))+[Path(__file__)]})
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    rng=random.Random(metadata['seed']);rows=[]
    for repeat in range(metadata['repeats']):
        order=tasks[:];rng.shuffle(order)
        for task in order:
            engines=['LBA','Mesh'];rng.shuffle(engines)
            for engine in engines:
                variants=list(BINARIES);rng.shuffle(variants);pair={}
                for variant in variants:
                    command=[str(BINARIES[variant]),task['map'],engine,'directional',
                        *map(str,task['start']),*map(str,task['goal']),'1']
                    code,stdout,stderr,wall,timeout=isolated(command,120)
                    if timeout or code!=0:raise RuntimeError((task['id'],engine,variant,code,stderr))
                    r=json.loads(stdout);assert r['path_valid'],(task['id'],r)
                    r.update(task=task['id'],engine=engine,variant=variant,repeat=repeat,
                             process_wall_s=wall,completed_epoch=time.time())
                    append(OUT/'runs.jsonl',r);rows.append(r);pair[variant]=r
                assert pair['baseline']['status']==pair['nonempty']['status']
                assert abs(pair['baseline']['path_cost_double']-pair['nonempty']['path_cost_double'])<=.001
            print('repeat',repeat,task['id'],'PASS',flush=True)
    summary=[]
    for engine in ['LBA','Mesh']:
        for outcome in ['all','solved','exhausted_unreachable']:
            rr=[r for r in rows if r['engine']==engine and (outcome=='all' or r['status']==outcome)]
            totals={v:[sum(r['end_to_end_s'] for r in rr if r['variant']==v and r['repeat']==k)
                       for k in range(5)] for v in BINARIES}
            sums={v:0. for v in BINARIES};step={v:0 for v in BINARIES}
            for task in {r['task'] for r in rr}:
                for variant in BINARIES:
                    runs=[r for r in rr if r['task']==task and r['variant']==variant]
                    assert len(runs)==5
                    assert len({(r['status'],r['cost'],r['steps']) for r in runs})==1
                    sums[variant]+=statistics.median(r['end_to_end_s'] for r in runs)
                    step[variant]+=runs[0]['steps']
            summary.append(dict(engine=engine,outcome=outcome,tasks=len(rr)//10,
                median_sums_s=sums,speedup=sums['baseline']/sums['nonempty'],steps=step,
                repeat_series_s=totals,
                repeat_speedups=[a/b for a,b in zip(totals['baseline'],totals['nonempty'])]))
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
