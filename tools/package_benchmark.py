#!/usr/bin/env python3
"""Package this exact source/data snapshot; no native binaries or unrelated files."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('output',type=Path)
a=p.parse_args()
if a.output.exists():raise SystemExit('Refusing to overwrite existing archive')
m=json.loads((ROOT/'res/final-benchmark-v1/manifest.json').read_text())
files={ROOT/x for x in m['input_sha256']}
files.update(ROOT/x for x in m['prior_task_sources_sha256'])
for folder,pattern in [('src','*.cpp'),('include','*.hpp'),('tests','*.cpp')]:
    files.update((ROOT/folder).glob(pattern))
files.update((ROOT/'res/final-benchmark-v1').rglob('*'))
for name in ['Makefile','benchmark.mk','LICENSE.txt','README.md','TEAM_BENCHMARK.md',
             'TEAM_VALIDATION.md','tests/benchmark_runner_tests.py',
             'tools/benchmark_worker.cpp','tools/run_benchmark.py','tools/export_benchmark.py',
             'tools/package_benchmark.py','tools/prepare_final_benchmark.py']:
    files.add(ROOT/name)
payload={str(f.relative_to(ROOT)):f.read_bytes() for f in sorted(files) if f.is_file()}
upstream=ROOT/'UPSTREAM_README.md'
payload['UPSTREAM_README.md']=upstream.read_bytes() if upstream.exists() else payload['README.md']
payload['README.md']=payload['TEAM_BENCHMARK.md']
for label,folder,expected in [('edge','final-benchmark-edge-local-v2',448),
                              ('pilot','final-benchmark-pilot-local',456)]:
    base=ROOT/'res'/folder
    if not base.exists():base=ROOT/'validation'/label
    rows=(base/'runs.jsonl').read_text().splitlines()
    assert len(rows)==expected,(label,len(rows))
    for name in ['runs.jsonl','summary.json']:
        payload[f'validation/{label}/{name}']=(base/name).read_bytes()
    # Reference output only; not a resumable active folder. Avoid publishing hostname.
    meta=json.loads((base/'metadata.json').read_text())
    meta['identity']['hostname']='redacted-in-shared-example'
    payload[f'validation/{label}/metadata.json']=(json.dumps(meta,indent=2)+'\n').encode()
inventory={name:hashlib.sha256(data).hexdigest() for name,data in sorted(payload.items())}
payload['PACKAGE_SHA256.json']=(json.dumps(inventory,indent=2)+'\n').encode()
a.output.parent.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED) as z:
    for name,data in sorted(payload.items()):
        z.writestr('mesha-team-benchmark/'+name,data)
with zipfile.ZipFile(a.output) as z:assert z.testzip() is None
print(json.dumps(dict(path=str(a.output.resolve()),files=len(payload),bytes=a.output.stat().st_size,
                     sha256=hashlib.sha256(a.output.read_bytes()).hexdigest())))
