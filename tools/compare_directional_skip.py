#!/usr/bin/env python3
"""Paired old/new comparison on the old 24 development queries, not the frozen 960."""
import hashlib,json,random,statistics
from pathlib import Path
from run_benchmark import ROOT,isolated,append

out=ROOT/'res/directional-skip'
assert not (out/'runs.jsonl').exists(),'Refusing overwrite'
tasks=json.loads((ROOT/'res/multimap-experiment/manifest.json').read_text())['tasks']
binaries={'before':out/'worker-before','after':ROOT/'build-benchmark/benchmark_worker'}
meta={'seed':20261027,'repeats':3,'tasks':[t['id'] for t in tasks],
      'binaries_sha256':{k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in binaries.items()},
      'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['include/KC_directional.hpp','src/KC_heuristic.cpp']}}
(out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
rng=random.Random(20261027);rows=[]
for rep in range(3):
 order=tasks[:];rng.shuffle(order)
 for t in order:
  for engine in ['LBA','Mesh']:
   variants=list(binaries);rng.shuffle(variants);pair={}
   for variant in variants:
    cmd=[str(binaries[variant]),t['map'],engine,'directional',*map(str,t['start']),*map(str,t['goal']),'1']
    code,stdout,stderr,wall,timed=isolated(cmd,30)
    assert not timed and code==0,(t['id'],variant,code,stderr)
    r=json.loads(stdout);assert r['path_valid'],(t['id'],r)
    r.update(task=t['id'],engine=engine,variant=variant,repeat=rep,process_wall_s=wall)
    append(out/'runs.jsonl',r);rows.append(r);pair[variant]=r
   for field in ['status','cost','path','steps','candidates','pushes','pops','checked_cells']:
    assert pair['before'][field]==pair['after'][field],(t['id'],engine,field)
  print(rep,t['id'],'equivalent',flush=True)
summary={}
for engine in ['LBA','Mesh']:
 data={v:[sum(r['end_to_end_s'] for r in rows if r['engine']==engine and r['variant']==v and r['repeat']==rep) for rep in range(3)] for v in binaries}
 summary[engine]={'series_seconds':data,'median_before_s':statistics.median(data['before']),
                  'median_after_s':statistics.median(data['after']),
                  'paired_speedups':[a/b for a,b in zip(data['before'],data['after'])]}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
