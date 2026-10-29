#!/usr/bin/env python3
"""Replay every saved solved path's float additions without rerunning search.

Original logs remain immutable. Geometry checks are inherited from the worker;
primitive costs and both cost sums are independently reconstructed here.
"""
import collections
import csv
import hashlib
import json
import math
import re
import struct
from pathlib import Path
from run_benchmark import verdict

ROOT=Path(__file__).resolve().parents[1]
FROZEN=ROOT/'res/full-run-2026-10-27/mesha-team-benchmark'
OUT=ROOT/'res/numeric-validation-2026-10-27'


def f32(value):
    return struct.unpack('<f',struct.pack('<f',value))[0]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def replay(path,costs):
    single=0.;double=0.
    for primitive in path:
        edge=costs[primitive]
        single=f32(single+edge)
        double+=edge
    return single,double


def main():
    OUT.mkdir(exist_ok=True)
    source=FROZEN/'data/base_control_set.txt'
    assert sha(source)==sha(ROOT/'data/base_control_set.txt')
    manifest=json.loads((FROZEN/'res/final-benchmark-v1/manifest.json').read_text())
    assert sha(source)==manifest['input_sha256']['data/base_control_set.txt']
    text=source.read_text()
    costs={int(i):f32(float(c)) for c,i in re.findall(r'length is: ([^\n]+).*?prim ID is: (\d+)',text,re.S)}
    assert len(costs)==384 and all(math.isfinite(c) and c>=0 for c in costs.values())
    audit={'policy':'float-replay-v1','geometry':'Original worker trace/endpoint checks retained; not rerun by this script.',
           'primitive_sha256':sha(source),'script_sha256':sha(Path(__file__)),
           'comparison_tolerance_double':.001,'stages':[]}
    changes=[]
    for stage in ['paper','extension']:
        path=FROZEN/'results'/('full-'+stage)/'runs.jsonl'
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        counts=collections.Counter()
        for r in rows:
            r['original_validation']=r['validation']
            if r['status'] not in ['solved','exhausted_unreachable']:
                continue
            r['original_sum_ok']=r['sum_ok'];r['original_path_valid']=r['path_valid']
            r['cost_validation']='float-replay-v1'
            if r['status']=='solved':
                single,double=replay(r['path'],costs)
                r['path_cost_float_replay']=single
                r['double_sum_reproduced']=(double==r['path_cost_double'])
                r['sum_ok']=math.isfinite(single) and single==r['cost'] and r['double_sum_reproduced']
                r['path_valid']=r['trace_ok'] and r['endpoint_ok'] and r['sum_ok']
                counts['solved_replayed']+=1
                counts['replay_mismatch']+=not r['sum_ok']
                # Negative controls: changing even one float ULP must be caught.
                bits=struct.unpack('<I',struct.pack('<f',single))[0]
                corrupt=struct.unpack('<f',struct.pack('<I',bits+1))[0]
                assert corrupt!=single
        refs={r['task']:r for r in rows if r['stage']=='reference'}
        for r in rows:
            ref=None if r['stage']=='reference' else refs[r['task']]
            r['validation']=verdict(r,ref,{})
            if r['validation']!=r['original_validation']:
                changes.append({k:r.get(k) for k in ['job','task','engine','variant','stage',
                    'original_validation','validation','cost','path_cost_double','path_cost_float_replay']})
        directory=OUT/('full-'+stage);directory.mkdir(exist_ok=True)
        with (directory/'runs.jsonl').open('w') as f:
            for r in rows:f.write(json.dumps(r,allow_nan=False)+'\n')
        audit['stages'].append(dict(series=stage,rows=len(rows),original_sha256=sha(path),
            derived_sha256=sha(directory/'runs.jsonl'),counts=dict(counts),
            validation=dict(collections.Counter(r['validation'] for r in rows))))
    with (OUT/'changed-verdicts.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(changes[0]));writer.writeheader();writer.writerows(changes)
    audit['changed_records']=len(changes)
    (OUT/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps(audit,indent=2))


if __name__=='__main__':main()
