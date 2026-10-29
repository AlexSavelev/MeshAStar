#!/usr/bin/env python3
"""Read-only audit of frozen runs; matched complete cases, no tolerance changes."""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = ['safe-euclidean', 'max', 'directional']
ENGINES = ['LBA', 'Mesh']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def write_csv(path, rows):
    if not rows:
        path.write_text('')
        return
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT/'res/full-analysis-2026-10-27')
    parser.add_argument('--validated-root', type=Path, help='Derived replay-validated JSONL; original metadata still verified')
    args = parser.parse_args()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    frozen = ROOT/'res/full-run-2026-10-27/mesha-team-benchmark'
    rows = []
    sources = {}
    stage_audit = []
    for stage in ['paper', 'extension']:
        directory = frozen/'results'/('full-'+stage)
        path = directory/'runs.jsonl'
        if args.validated_root:
            path = args.validated_root.resolve()/('full-'+stage)/'runs.jsonl'
        rr = [json.loads(line) for line in path.read_text().splitlines()]
        meta = json.loads((directory/'metadata.json').read_text())['identity']
        expected = {f'reference-{t}-LBA-h0-0' for t in meta['task_ids']}
        expected |= {f'measure-{t}-{e}-{v}-{k}' for t in meta['task_ids']
                     for e in ENGINES for v in VARIANTS for k in range(3)}
        assert len(rr) == len(expected) == 9120
        assert {r['job'] for r in rr} == expected
        assert meta['repeats'] == 3 and meta['variants'] == VARIANTS and meta['engines'] == ENGINES
        assert digest(frozen/'build-benchmark/benchmark_worker') == meta['worker_sha256']
        for name, sha in meta['source_sha256'].items():
            assert digest(frozen/name) == sha, name
        repeats = collections.defaultdict(set)
        for r in rr:
            r['series'] = stage
            if r['stage'] == 'measure' and r['status'] in ['solved', 'exhausted_unreachable']:
                repeats[r['task'], r['engine'], r['variant']].add(tuple(r[f] for f in
                    ['status', 'cost', 'steps', 'candidates', 'pushes', 'pops']))
        assert all(len(values) == 1 for values in repeats.values())
        stage_audit.append(dict(series=stage, rows=len(rr),
            status=dict(collections.Counter(r['status'] for r in rr)),
            validation=dict(collections.Counter(r['validation'] for r in rr)),
            last_write_moscow=datetime.datetime.fromtimestamp(path.stat().st_mtime,
                datetime.timezone(datetime.timedelta(hours=3))).isoformat(),
            repeat_disagreements=0))
        sources[str(path.relative_to(ROOT))] = digest(path)
        sources[str((directory/'metadata.json').relative_to(ROOT))] = digest(directory/'metadata.json')
        rows.extend(rr)
    interference_path = ROOT/'res/directional-nonempty/interference.json'
    interference = json.loads(interference_path.read_text())
    excluded = set(interference['exclude_completed_jobs']) | {interference['next_job_to_exclude']}
    assert len(excluded) == 98 and excluded <= {r['job'] for r in rows}
    sources[str(interference_path.relative_to(ROOT))] = digest(interference_path)
    refs = {r['task']: r for r in rows if r['stage'] == 'reference'}
    measures = [r for r in rows if r['stage'] == 'measure']
    assert len(refs) == 960 and len(measures) == 17280
    invalid = [r for r in rows if r['validation'] == 'invalid_path']
    solved = [r for r in rows if r['status'] == 'solved']
    geometric_bad = [r for r in solved if not r['trace_ok'] or not r['endpoint_ok']]
    numeric_rows = []
    for r in invalid:
        numeric_rows.append({k: r.get(k) for k in ['job','series','stage','task','engine','variant','repeat',
            'cost','path_cost_double','trace_ok','endpoint_ok','sum_ok']})
    write_csv(out/'numeric-failures.csv', numeric_rows)
    doubles = [abs(r['path_cost_double']-refs[r['task']]['path_cost_double'])
               for r in measures if r['status'] == 'solved' and refs[r['task']]['status'] == 'solved']
    groups = collections.defaultdict(dict)
    for r in measures:
        groups[r['task'],r['engine']][r['variant'],r['repeat']] = r
    paired, omitted = [], []
    for (task, engine), group in sorted(groups.items()):
        assert len(group) == 9
        # Complete-case selection BEFORE interference removal: a timeout/failure
        # is not made to disappear by selecting favourable surviving repeats.
        reasons = sorted({r['validation'] for r in group.values() if r['validation'] != 'pass'})
        if reasons:
            omitted.append(dict(task=task, engine=engine, reason=','.join(reasons)))
            continue
        assert len({r['status'] for r in group.values()}) == 1
        costs = [r['path_cost_double'] if r.get('cost_validation')=='float-replay-v1' else r['cost'] for r in group.values()]
        if max(costs)-min(costs) > .001:
            omitted.append(dict(task=task, engine=engine, reason='cross_variant_cost_difference'))
            continue
        clean = [k for k in range(3) if all(group[v,k]['job'] not in excluded for v in VARIANTS)]
        if len(clean) < 2:
            omitted.append(dict(task=task, engine=engine, reason='fewer_than_two_clean_repeats'))
            continue
        first = group['max',0]
        row = dict(task=task, engine=engine, map=Path(first['map']).stem,
                   series=first['series'], band=first['band'], status=first['status'],
                   clean_repeats=len(clean), repeat_ids=','.join(map(str,clean)))
        for variant in VARIANTS:
            for field in ['end_to_end_s','search_s','preprocessing_s','grid_s','steps','checked_cells','pops']:
                row[variant+'__'+field] = statistics.median(group[variant,k][field] for k in clean)
        for before, after, name in [('safe-euclidean','max','grid'),('max','directional','directional'),
                                     ('safe-euclidean','directional','combined')]:
            row[name+'_speedup'] = row[before+'__end_to_end_s']/row[after+'__end_to_end_s']
        paired.append(row)
    write_csv(out/'paired-tasks.csv', paired)
    write_csv(out/'excluded-task-engines.csv', omitted)
    summary = []
    scopes = ['ALL','paper','extension'] + sorted({r['map'] for r in paired})
    for engine in ENGINES:
        for scope in scopes:
            for outcome in ['all_completed','solved','exhausted_unreachable']:
                rr = [r for r in paired if r['engine']==engine and
                      (scope=='ALL' or scope in [r['series'],r['map']]) and
                      (outcome=='all_completed' or r['status']==outcome)]
                if not rr:
                    continue
                row = dict(engine=engine,scope=scope,outcome=outcome,tasks=len(rr),
                           tasks_two_repeats=sum(r['clean_repeats']==2 for r in rr))
                for v in VARIANTS:
                    row[v+'_seconds'] = sum(r[v+'__end_to_end_s'] for r in rr)
                    row[v+'_steps'] = sum(r[v+'__steps'] for r in rr)
                for before,after,name in [('safe-euclidean','max','grid'),('max','directional','directional'),
                                          ('safe-euclidean','directional','combined')]:
                    values = [r[name+'_speedup'] for r in rr]
                    row[name+'_aggregate_speedup'] = row[before+'_seconds']/row[after+'_seconds']
                    row[name+'_median_speedup'] = statistics.median(values)
                    row[name+'_geomean_speedup'] = math.exp(statistics.mean(math.log(x) for x in values))
                    row[name+'_wins_over_5pct'] = sum(x>1.05 for x in values)
                    row[name+'_losses_over_5pct'] = sum(x<1/1.05 for x in values)
                    row[name+'_steps_reduction_pct'] = 100*(1-row[after+'_steps']/row[before+'_steps'])
                summary.append(row)
    write_csv(out/'summary.csv', summary)
    status_counts = []
    for series in ['paper','extension']:
        for engine in ENGINES:
            for variant in VARIANTS:
                rr=[r for r in measures if (r['series'],r['engine'],r['variant'])==(series,engine,variant)]
                status_counts.append(dict(series=series,engine=engine,variant=variant,runs=len(rr),
                    solved=sum(r['status']=='solved' for r in rr),
                    exhausted=sum(r['status']=='exhausted_unreachable' for r in rr),
                    timeout=sum(r['status']=='timeout' for r in rr),error=sum(r['status']=='error' for r in rr),
                    invalid_path=sum(r['validation']=='invalid_path' for r in rr),
                    reference_unknown=sum(r['validation']=='reference_unknown' for r in rr)))
    write_csv(out/'all-status-counts.csv',status_counts)
    sensitivity = []
    for engine in ENGINES:
        for outcome in ['all_completed','solved']:
            rr=[r for r in paired if r['engine']==engine and (outcome=='all_completed' or r['status']=='solved')]
            for repeat in [1,2]:
                times=[sum(groups[r['task'],engine][v,repeat]['end_to_end_s'] for r in rr) for v in VARIANTS]
                sensitivity.append(dict(engine=engine,outcome=outcome,repeat=repeat,
                    grid_speedup=times[0]/times[1],directional_speedup=times[1]/times[2]))
    audit = dict(stages=stage_audit,reference_status=dict(collections.Counter(r['status'] for r in refs.values())),
        invalid_path_runs=len(invalid),invalid_path_tasks=len({r['task'] for r in invalid}),
        invalid_references=sum(not r['path_valid'] for r in refs.values()),geometry_bad=len(geometric_bad),
        max_float_accumulation_difference=max(abs(r['cost']-r['path_cost_double']) for r in solved),
        max_double_path_cost_difference_vs_reference=max(doubles),
        recorded_interference_rows=len(excluded),excluded_task_engines=len(omitted),repeat_sensitivity=sensitivity,
        timeout_tasks=[dict(task=t,reference_status=refs[t]['status']) for t in sorted({r['task'] for r in measures if r['status']=='timeout'})],
        main_policy='All 9 runs/task/engine must PASS and costs agree within 0.001. Remove whole three-variant repeat if any is in recorded CPU-interference interval; require >=2 clean repeats. Per-task median full solve time; ratio of sums across tasks. Original logs unchanged.',
        validation_policy='float-replay-v1 (derived logs)' if args.validated_root else 'original fixed-0.001 float/double comparison',
        source_sha256=sources,analysis_sha256=digest(Path(__file__)))
    dump(out/'audit.json',audit)
    dump(out/'summary.json',summary)
    for r in summary:
        if r['scope'] in ['ALL','paper','extension']:
            print(r['engine'],r['scope'],r['outcome'],r['tasks'],
                  'grid',round(r['grid_aggregate_speedup'],3),'dir',round(r['directional_aggregate_speedup'],3),
                  'median',round(r['grid_median_speedup'],3),round(r['directional_median_speedup'],3))
    print(json.dumps({k:v for k,v in audit.items() if k not in ['source_sha256','main_policy']},indent=2))


if __name__=='__main__':
    main()
