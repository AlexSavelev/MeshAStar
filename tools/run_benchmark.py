#!/usr/bin/env python3
"""Isolated, resumable benchmark. Python >=3.9, macOS/Linux/WSL; stdlib only."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent))
from team_benchmark_plan import MATRICES, SIGNATURE, combinations, model_key

ROOT = Path(__file__).resolve().parents[1]
FROZEN = ROOT/'res/final-benchmark-v1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def atomic_json(path, value):
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)

def load_prior(path):
    prior={}
    if path.exists():
        for number,line in enumerate(path.read_text().splitlines(),1):
            try:r=json.loads(line)
            except ValueError as exc:raise RuntimeError(f'Invalid JSONL line {number}; preserve file before recovery') from exc
            if r['job'] in prior:raise RuntimeError('Duplicate job in results')
            prior[r['job']]=r
    return prior


def isolated(command, timeout, cwd=ROOT):
    """Kill whole worker process group on timeout/interruption; always reap."""
    start=time.monotonic()
    p=subprocess.Popen(command,cwd=cwd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                       text=True,start_new_session=True)
    try:
        out,err=p.communicate(timeout=timeout)
        return p.returncode,out,err,time.monotonic()-start,False
    except subprocess.TimeoutExpired:
        try:os.killpg(p.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        out,err=p.communicate()
        return p.returncode,out,err,time.monotonic()-start,True
    except BaseException:
        try:os.killpg(p.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        p.communicate()
        raise


def verify_inputs():
    m=json.loads((FROZEN/'manifest.json').read_text())
    for group in [m['input_sha256'],m['prior_task_sources_sha256'],
                  json.loads((FROZEN/'frozen-sha256.json').read_text())]:
        for path,digest in group.items():
            if sha(ROOT/path)!=digest:raise RuntimeError('Changed frozen file: '+path)
    return m


def verdict(result, reference, task):
    status=result['status']
    if status not in ['solved','exhausted_unreachable']:return 'not_completed'
    if not result['path_valid']:return 'invalid_path'
    if task.get('expected')=='solved_zero_cost' and (status!='solved' or abs(result['cost'])>.001):return 'expected_zero_failed'
    if task.get('expected')=='unreachable' and status!='exhausted_unreachable':return 'expected_unreachable_failed'
    if reference is None:return 'reference_run'
    if reference['status'] not in ['solved','exhausted_unreachable'] or not reference.get('path_valid',False):return 'reference_unknown'
    if status!=reference['status']:return 'reachability_mismatch'
    cost_key='path_cost_double' if result.get('cost_validation')==reference.get('cost_validation')=='float-replay-v1' else 'cost'
    if status=='solved' and abs(result[cost_key]-reference[cost_key])>.001:return 'cost_mismatch'
    return 'pass'


def append(path,row):
    with path.open('a') as f:
        f.write(json.dumps(row,allow_nan=False)+'\n');f.flush();os.fsync(f.fileno())


def summarize(out):
    rows=[json.loads(s) for s in (out/'runs.jsonl').read_text().splitlines()] if (out/'runs.jsonl').exists() else []
    import statistics
    groups={}
    for r in rows:
        key=(r['stage'],r['engine'],r['variant'])
        groups.setdefault(key,[]).append(r)
    result=[]
    for (stage,engine,variant),rr in sorted(groups.items()):
        good=[r for r in rr if r['status'] in ['solved','exhausted_unreachable']]
        times=[r['end_to_end_s'] for r in good]
        result.append(dict(stage=stage,engine=engine,variant=variant,count=len(rr),
                           statuses=dict(Counter(r['status'] for r in rr)),
                           validation=dict(Counter(r['validation'] for r in rr)),
                           completed_solve_sum_s=sum(times),
                           completed_solve_median_s=statistics.median(times) if times else None,
                           max_peak_rss_bytes=max((r.get('peak_rss_bytes',0) for r in rr),default=0)))
    # Repeat counters should agree on identical deterministic queries.
    repeated={}
    for r in rows:
        if r['stage']=='measure' and r['status'] in ['solved','exhausted_unreachable']:
            key=(r['task'],r['engine'],r['variant'])
            sig=tuple(r[k] for k in ['status','cost','steps','candidates','pushes','pops'])
            repeated.setdefault(key,set()).add(sig)
    summary=dict(groups=result,repeat_disagreements=[list(k) for k,s in repeated.items() if len(s)>1],
                 note='Sums include completed runs only. Timeouts are censored, not unreachable. See rows for per-map paired analysis.')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',choices=['edge','pilot','full'],required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--variants',default='safe-euclidean,max,directional')
    p.add_argument('--engines',default='LBA,Mesh')
    p.add_argument('--matrix',choices=sorted(MATRICES),help='Explicit engine/variant pairs; overrides engines/variants')
    p.add_argument('--eta-model',type=Path,help='Historical per-query process-wall estimates, informational only')
    p.add_argument('--maps',help='Comma-separated map stems; explicit sharding, not resampling')
    p.add_argument('--timeout',type=float,default=30)
    p.add_argument('--reference-timeout',type=float,default=120)
    p.add_argument('--worker',type=Path,default=ROOT/'build-benchmark/benchmark_worker')
    p.add_argument('--build-label',default='benchmark.mk default -std=c++11 -O3 -Wall -Wextra -Werror')
    args=p.parse_args()
    if os.name!='posix':p.error('Use Linux, macOS or WSL for process-group isolation.')
    if args.repeats<1 or min(args.timeout,args.reference_timeout)<=0:p.error('Positive repeats/timeouts required')
    variants=args.variants.split(',');engines=args.engines.split(',')
    if len(set(variants))!=len(variants) or not set(variants)<=set(['safe-euclidean','max','directional','euclidean','max-buckets','directional-buckets']):p.error('Invalid variants')
    if len(set(engines))!=len(engines) or not set(engines)<=set(['LBA','Mesh']):p.error('Invalid engines')
    pairs=combinations(args.matrix,engines,variants)
    if args.matrix:
        engines=list(dict.fromkeys(e for e,v in pairs));variants=list(dict.fromkeys(v for e,v in pairs))
    m=verify_inputs()
    tasks=m['edge_tasks'] if args.suite=='edge' else [t for t in m['tasks'] if args.suite=='full' or t['pilot']]
    if args.maps:
        selected=set(args.maps.split(','));available={Path(t['map']).stem for t in tasks}
        if not selected<=available:p.error('Unknown maps: '+str(selected-available))
        tasks=[t for t in tasks if Path(t['map']).stem in selected]
    worker=args.worker.resolve();out=args.out.resolve()
    if not worker.is_file():p.error('Build worker first: make -f benchmark.mk -j4')
    sources=sorted(list((ROOT/'src').glob('*.cpp'))+list((ROOT/'include').glob('*.hpp'))+
                   [ROOT/'benchmark.mk',ROOT/'tools/benchmark_worker.cpp',ROOT/'tools/team_benchmark_plan.py',Path(__file__).resolve()])
    identity=dict(schema=2,suite=args.suite,task_ids=[t['id'] for t in tasks],repeats=args.repeats,
                  variants=variants,engines=engines,timeout_s=args.timeout,reference_timeout_s=args.reference_timeout,
                  matrix=args.matrix,combinations=[list(x) for x in pairs],expected_records=len(tasks)*(1+len(pairs)*args.repeats),
                  worker_sha256=sha(worker),manifest_sha256=sha(FROZEN/'manifest.json'),
                  source_sha256={str(s.relative_to(ROOT)):sha(s) for s in sources},
                  platform=platform.platform(),machine=platform.machine(),hostname=platform.node(),
                  python=sys.version,build_label=args.build_label,
                  ordering_seed=m['seed']+100,warmup='fresh 32x32 straight query per process, no reused grid')
    out.mkdir(parents=True,exist_ok=True)
    # Prevent simultaneous writers to one output. flock is released on kill/exit.
    import fcntl
    lock=(out/'run.lock').open('a')
    try:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:p.error('Output directory is already running')
        meta=out/'metadata.json'
        if meta.exists():
            old=json.loads(meta.read_text())
            if old['identity']!=identity:raise RuntimeError('Refusing resume with changed inputs/config/build/machine; choose a new --out')
        else:
            meta.write_text(json.dumps(dict(identity=identity,started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())),indent=2)+'\n')
        logs=out/'logs';logs.mkdir(exist_ok=True)
        path=out/'runs.jsonl';prior=load_prior(path)
        if (out/'bucket-mismatch.json').exists() or any(r['status']=='error' or r['validation'] not in
                ['pass','reference_run','reference_unknown','not_completed'] for r in prior.values()):
            raise RuntimeError('Existing results contain failures; investigate before starting a new output directory')
        estimate=json.loads(args.eta_model.read_text()).get('seconds',{}) if args.eta_model else {}
        total=identity['expected_records']
        schedule=[(t,'LBA','h0',0,'reference') for t in tasks]
        rng=random.Random(m['seed']+100)
        for repeat in range(args.repeats):
            order=tasks[:];rng.shuffle(order)
            for task in order:
                choices=pairs[:];rng.shuffle(choices)
                schedule.extend((task,e,v,repeat,'measure') for e,v in choices)
        def job_id(t,e,v,k,stage):return f'{stage}-{t["id"]}-{e}-{v}-{k}'
        expected={job_id(*j) for j in schedule}
        if not set(prior)<=expected:raise RuntimeError('Unexpected job ids in existing results')
        costs={job_id(*j):estimate.get(model_key(j[0]['id'],j[1],j[2]),0) for j in schedule}
        remaining=sum(c for j,c in costs.items() if j not in prior)
        status_path=out/'progress.json'
        def progress(state,active=None):
            atomic_json(status_path,dict(state=state,runner_pid=os.getpid(),completed=len(prior),total=total,
                        remaining=total-len(prior),active_job=active,estimated_remaining_process_s=remaining if estimate else None,
                        updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())))
        def stop(signum,frame):raise KeyboardInterrupt('Stop requested')
        signal.signal(signal.SIGTERM,stop)
        progress('running')
        print(f'PLAN {len(tasks)} tasks x {len(pairs)} modes x {args.repeats} repeats + {len(tasks)} references = {total} records; resume {len(prior)}',flush=True)
        refs={r['task']:r for r in prior.values() if r['stage']=='reference'}
        def run(task,engine,variant,repeat,stage,ref):
            nonlocal remaining
            job=f'{stage}-{task["id"]}-{engine}-{variant}-{repeat}'
            if job in prior:return prior[job]
            progress('running',job)
            started=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
            command=[str(worker),task['map'],engine,variant,*map(str,task['start']),*map(str,task['goal']),'1']
            limit=args.reference_timeout if stage=='reference' else args.timeout
            code,stdout,stderr,wall,timed=isolated(command,limit)
            row=dict(job=job,task=task['id'],map=task['map'],band=task.get('band',task.get('family')),
                     engine=engine,variant=variant,repeat=repeat,stage=stage,process_wall_s=wall,
                     timeout_limit_s=limit,returncode=code)
            row['started_utc']=started
            row['finished_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
            (logs/(job+'.log')).write_text('STDOUT\n'+stdout+'\nSTDERR\n'+stderr)
            if timed:row['status']='timeout'
            elif code!=0:row['status']='error'
            else:
                try:
                    data=json.loads(stdout,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                    if data['schema']!=1 or data['status'] not in ['solved','exhausted_unreachable']:raise ValueError('Worker schema')
                    row.update(data)
                except (ValueError,KeyError,TypeError) as e:
                    row['status']='error';row['protocol_error']=str(e)
            row['validation']=verdict(row,ref,task)
            append(path,row);prior[job]=row
            remaining=max(0,remaining-costs[job]);progress('running')
            eta=f' ETA~{remaining/3600:.2f}h' if estimate else ''
            print(f'[{len(prior)}/{total}] {stage} {task["id"]} {engine}/{variant} {row["status"]} {wall:.2f}s {row["validation"]}{eta}',flush=True)
            # A queue change must not alter the deterministic search. Compare with
            # the matching repetition, irrespective of randomized execution order.
            if stage=='measure' and variant in ['max','directional','max-buckets','directional-buckets']:
                other=variant[:-8] if variant.endswith('-buckets') else variant+'-buckets'
                mate=prior.get(f'measure-{task["id"]}-{engine}-{other}-{repeat}')
                if mate and row['status'] in ['solved','exhausted_unreachable'] and mate['status'] in ['solved','exhausted_unreachable']:
                    mismatch=[key for key in SIGNATURE if row.get(key)!=mate.get(key)]
                    if mismatch:
                        atomic_json(out/'bucket-mismatch.json',dict(job=job,other=mate['job'],fields=mismatch))
                        raise RuntimeError('Heap/buckets changed deterministic result: '+','.join(mismatch))
            if row['status']=='error' or row['validation'] not in ['pass','reference_run','reference_unknown','not_completed']:
                raise RuntimeError('Stopped for invalid result; inspect '+str(logs/(job+'.log')))
            return row
        finished=False
        try:
            for task,engine,variant,repeat,stage in schedule:
                row=run(task,engine,variant,repeat,stage,refs.get(task['id']) if stage=='measure' else None)
                if stage=='reference':refs[task['id']]=row
            finished=True
        finally:
            s=summarize(out)
            progress('completed' if finished else 'interrupted_or_failed')
            print('Summary:',out/'summary.json',flush=True)
        bad=sum(r['validation'] not in ['pass','reference_run','reference_unknown','not_completed'] for r in prior.values())
        errors=sum(r['status']=='error' for r in prior.values())
        print(f'DONE {len(prior)} rows; validation failures={bad}, errors={errors}, repeat disagreements={len(s["repeat_disagreements"])}',flush=True)
        return 1 if bad or errors or s['repeat_disagreements'] else 0

    finally:
        lock.close()


if __name__=='__main__':
    sys.exit(main())
