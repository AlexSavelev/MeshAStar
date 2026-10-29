#!/usr/bin/env python3
"""Paired analysis of the new heap/bucket series; no historical exclusions."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import statistics
from team_benchmark_plan import SIGNATURE

FIELDS=['end_to_end_s','preprocessing_s','grid_s','search_s','steps','checked_cells']
def csv_write(path,rows):
    with path.open('w',newline='') as f:
        if rows:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def analyze(directory):
    meta=json.loads((directory/'metadata.json').read_text())['identity']
    rows=[json.loads(s) for s in (directory/'runs.jsonl').read_text().splitlines()]
    out=directory/'analysis';out.mkdir(exist_ok=True)
    ids=meta['task_ids'];nrep=meta['repeats'];modes=[tuple(x) for x in meta['combinations']]
    expected={f'reference-{t}-LBA-h0-0' for t in ids}|{
        f'measure-{t}-{e}-{v}-{r}' for t in ids for e,v in modes for r in range(nrep)}
    jobs=[r['job'] for r in rows]
    if len(jobs)!=len(set(jobs)) or not set(jobs)<=expected:raise RuntimeError('Invalid job inventory')
    group=defaultdict(dict);counts=defaultdict(Counter);fail=[];repeat_mismatch=[];bucket_mismatch=[]
    for r in rows:
        counts[r['stage'],r['engine'],r['variant']][r['status']]+=1
        if r['status']=='error' or r['validation'] not in ['pass','reference_run','reference_unknown','not_completed']:fail.append(r['job'])
        if r['stage']=='measure':group[r['task'],r['engine'],r['variant']][r['repeat']]=r
    med={};omitted=[]
    for t in ids:
        for e,v in modes:
            g=group[t,e,v]
            reason=[]
            if set(g)!=set(range(nrep)):reason.append('incomplete_repeats')
            if any(r['validation']!='pass' for r in g.values()):reason+=sorted({r['validation'] for r in g.values() if r['validation']!='pass'})
            if len({r['status'] for r in g.values()})>1:reason.append('mixed_status')
            good=[r for r in g.values() if r['status'] in ['solved','exhausted_unreachable']]
            if good and any(any(r.get(k)!=good[0].get(k) for k in SIGNATURE) for r in good[1:]):
                repeat_mismatch.append([t,e,v]);reason.append('non_deterministic')
            if reason:omitted.append(dict(task=t,engine=e,variant=v,reason=','.join(reason)));continue
            row=dict(task=t,engine=e,variant=v,status=g[0]['status'],map=Path(g[0]['map']).stem,band=g[0]['band'],
                     repeats=nrep,path_cost_double=g[0]['path_cost_double'])
            for k in FIELDS:row[k]=statistics.median(r[k] for r in g.values())
            med[t,e,v]=row
            if v.endswith('-buckets'):
                other=group[t,e,v[:-8]]
                for rep,r in g.items():
                    mate=other.get(rep)
                    if mate and mate['status'] in ['solved','exhausted_unreachable']:
                        bad=[k for k in SIGNATURE if r.get(k)!=mate.get(k)]
                        if bad:bucket_mismatch.append(dict(task=t,engine=e,variant=v,repeat=rep,fields=bad))
    comparisons=[]
    for e in ['LBA','Mesh']:
        for a,b in [('safe-euclidean','max'),('safe-euclidean','directional'),('max','directional'),
                    ('max','max-buckets'),('directional','directional-buckets'),
                    ('safe-euclidean','max-buckets'),('safe-euclidean','directional-buckets')]:
            if (e,a) in modes and (e,b) in modes:comparisons.append((e,a,e,b))
    comparisons += [('LBA','safe-euclidean',e,v) for e,v in modes if (e,v)!=('LBA','safe-euclidean')]
    summary=[]
    for ae,av,be,bv in comparisons:
        for outcome in ['all_completed','solved','exhausted_unreachable']:
            pairs=[]
            for t in ids:
                a=med.get((t,ae,av));b=med.get((t,be,bv))
                if a and b and a['status']==b['status'] and (outcome=='all_completed' or a['status']==outcome):
                    if abs(a['path_cost_double']-b['path_cost_double'])>.001:
                        fail.append('cross_method_cost:'+t+':'+av+':'+bv);continue
                    pairs.append((a,b))
            if not pairs:continue
            for metric in FIELDS:
                ta=math.fsum(a[metric] for a,b in pairs);tb=math.fsum(b[metric] for a,b in pairs)
                ratios=[a[metric]/b[metric] for a,b in pairs if min(a[metric],b[metric])>0]
                summary.append(dict(base_engine=ae,base=av,new_engine=be,new=bv,outcome=outcome,
                    metric=metric,tasks=len(pairs),base_sum=ta,new_sum=tb,ratio_of_sums=ta/tb if tb else None,
                    geomean=math.exp(statistics.mean(math.log(x) for x in ratios)) if ratios else None,
                    geomean_tasks=len(ratios)))
    # One shared solved set for ALL bars of the main presentation.
    common=[t for t in ids if all((t,e,v) in med and med[t,e,v]['status']=='solved' for e,v in modes)]
    chart=[]
    if common:
        totals={(e,v):math.fsum(med[t,e,v]['end_to_end_s'] for t in common) for e,v in modes}
        for e,v in modes:
            chart.append(dict(engine=e,variant=v,tasks=len(common),sum_median_s=totals[e,v],
                speedup_vs_lba_safe=totals['LBA','safe-euclidean']/totals[e,v],
                speedup_vs_own_safe=totals[e,'safe-euclidean']/totals[e,v]))
    audit=dict(records=len(rows),expected=len(expected),complete=set(jobs)==expected,
        validation_failures=fail,repeat_mismatches=repeat_mismatch,bucket_mismatches=bucket_mismatch,
        common_solved_tasks=len(common),method='Complete validated repeats per task/method; per-task medians; pairwise intersection. Main chart uses common solved tasks across all selected modes. No old CPU-interference exclusions applied.')
    csv_write(out/'paired_tasks.csv',list(med.values()));csv_write(out/'excluded.csv',omitted)
    csv_write(out/'comparisons.csv',summary);csv_write(out/'main_chart.csv',chart)
    csv_write(out/'status_counts.csv',[dict(stage=k[0],engine=k[1],variant=k[2],status=s,count=n) for k,c in sorted(counts.items()) for s,n in sorted(c.items())])
    (out/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    (out/'common_solved_tasks.txt').write_text('\n'.join(common)+'\n')
    lines=['# Итоги общего прогона','',f'Записей: {len(rows)} / {len(expected)}. Общих проверенных задач с путём: {len(common)}.',
        '', 'Основная метрика — отношение сумм медианных полных времён. Подготовка включена.',
        'Таймауты исключены из парных времён и отдельно перечислены в status_counts.csv; это не доказательство недостижимости.',
        '', '| Алгоритм | Вариант | Общих задач | К LBA + Евклид | К собственной базе |','|---|---|---:|---:|---:|']
    lines += [f"| {r['engine']} | {r['variant']} | {r['tasks']} | {r['speedup_vs_lba_safe']:.4f}× | {r['speedup_vs_own_safe']:.4f}× |" for r in chart]
    lines += ['',f'Ошибок проверки: {len(fail)}. Несовпадений повторов: {len(repeat_mismatch)}. Несовпадений heap/buckets: {len(bucket_mismatch)}.',
              '', 'comparisons.csv содержит отдельные пары, исходы и фазы. Число задач может различаться между парами; для одного графика используйте main_chart.csv.']
    if not audit['complete']:lines.insert(2,'**НЕПОЛНЫЙ ПРОГОН: промежуточные результаты, не финальная таблица.**')
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(out/'REPORT.md')
    return audit

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args()
    r=analyze(a.directory.resolve())
    raise SystemExit(1 if r['validation_failures'] or r['repeat_mismatches'] or r['bucket_mismatches'] else 0)
