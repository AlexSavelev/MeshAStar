#!/usr/bin/env python3
"""Prepare/start/status/stop/resume the frozen team experiment. macOS/Linux."""
import argparse
from collections import defaultdict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import sys
import time
from team_benchmark_plan import MATRICES, model_key

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/'res/team-full-2026-10-29'
TOOLS=['team_benchmark.py','team_benchmark_plan.py','run_benchmark.py','benchmark_worker.cpp',
       'analyze_team_series.py','export_benchmark.py','team_timing_model.json']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(v,indent=2)+'\n');tmp.replace(p)
def locked(p):
    p.parent.mkdir(parents=True,exist_ok=True)
    f=p.open('a')
    try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:f.close();return None
    return f
def alive(run):
    f=locked(run/'supervisor.lock')
    if f is None:return True
    f.close();return False
def make_model():
    path=ROOT/'tools/team_timing_model.json'
    if path.exists():return
    group=defaultdict(list)
    for stage in ['paper','extension']:
        source=ROOT/'res/full-run-2026-10-27/mesha-team-benchmark/results'/('full-'+stage)/'runs.jsonl'
        if not source.exists():raise RuntimeError('Historical timing model missing; package must include tools/team_timing_model.json')
        for line in source.read_text().splitlines():
            r=json.loads(line);group[model_key(r['task'],r['engine'],r['variant'])].append(r['process_wall_s'])
    save(path,dict(source='2026-10-27 original process-wall logs; bucket cost conservatively uses matching heap cost',
                   seconds={k:statistics.mean(v) for k,v in group.items()},
                   note='Informational ETA only, no timing rows reused in the new experiment. Excludes supervisor overhead, sleep, pauses.'))
def verify(run):
    info=json.loads((run/'snapshot.json').read_text())
    for name,digest in info['sha256'].items():
        p=run/'snapshot'/name
        if not p.is_file() or sha(p)!=digest:raise RuntimeError('Frozen file changed/missing: '+name)
    return info
def prepare(run,profile):
    run.mkdir(parents=True,exist_ok=True)
    if (run/'snapshot.json').exists():
        info=verify(run)
        if info['profile']!=profile:raise RuntimeError('Different profile; choose another --run-dir')
        return info
    if alive(run):raise RuntimeError('Experiment is running')
    make_model()
    manifest=json.loads((ROOT/'res/final-benchmark-v1/manifest.json').read_text())
    files=set()
    for group in ['input_sha256','prior_task_sources_sha256']:
        for name,digest in manifest[group].items():
            if sha(ROOT/name)!=digest:raise RuntimeError('Frozen input changed: '+name)
            files.add(name)
    for folder,pattern in [('src','*.cpp'),('include','*.hpp'),('tests','*')]:
        files.update(str(p.relative_to(ROOT)) for p in (ROOT/folder).glob(pattern) if p.is_file())
    files.update(str(p.relative_to(ROOT)) for p in (ROOT/'res/final-benchmark-v1').rglob('*') if p.is_file())
    files.update('tools/'+n for n in TOOLS)
    files.update(['benchmark.mk','Makefile','LICENSE.txt','TEAM_FULL_TESTING.md'])
    snap=run/'snapshot';snap.mkdir(exist_ok=True)
    for name in sorted(files):
        dest=snap/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
    with (run/'build.log').open('w') as log:
        subprocess.run(['make','-f','benchmark.mk','-j4'],cwd=snap,stdout=log,stderr=subprocess.STDOUT,check=True)
    files.add('build-benchmark/benchmark_worker')
    compiler=subprocess.check_output(['c++','--version'],text=True)
    info=dict(profile=profile,created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
              compiler=compiler,build='make -f benchmark.mk -j4; default CXXFLAGS',
              sha256={n:sha(snap/n) for n in sorted(files)})
    save(run/'snapshot.json',info)
    model=json.loads((snap/'tools/team_timing_model.json').read_text())['seconds']
    estimate=sum(model[model_key(t['id'],'LBA','h0')]+3*sum(model[model_key(t['id'],e,v)] for e,v in MATRICES[profile]) for t in manifest['tasks'])
    save(run/'estimate.json',dict(historical_process_hours=estimate/3600,profile=profile,
             tasks=960,modes=len(MATRICES[profile]),repeats=3,references=960,
             expected_records=960*(1+3*len(MATRICES[profile])),
             note='Heap timings used for buckets; indicative, excluding orchestration, pauses and environmental changes.'))
    print('Prepared:',run,flush=True)
    return info
def show(run):
    if not (run/'snapshot.json').exists():print('Not prepared:',run);return
    info=json.loads((run/'snapshot.json').read_text())
    estimate=json.loads((run/'estimate.json').read_text())
    state=json.loads((run/'state.json').read_text()) if (run/'state.json').exists() else {'state':'ready_not_started'}
    progress=json.loads((run/'results/progress.json').read_text()) if (run/'results/progress.json').exists() else {}
    running=alive(run)
    print(f"State: {state['state']} | process active: {running} | profile: {info['profile']}")
    print(f"Progress: {progress.get('completed',0)} / {estimate['expected_records']} records (including references)")
    if progress.get('active_job'):print('Current:',progress['active_job'])
    rem=progress.get('estimated_remaining_process_s')
    print('Historical remaining estimate:',f'{rem/3600:.2f} h' if rem is not None else f"{estimate['historical_process_hours']:.2f} h")
    print('Log:',run/'run.log')
    print('Results:',run/'results')
    if (run/'results/analysis/REPORT.md').exists():print('Report:',run/'results/analysis/REPORT.md')
def supervisor(run):
    lock=locked(run/'supervisor.lock')
    if lock is None:raise RuntimeError('Supervisor already running')
    info=verify(run);snap=run/'snapshot'
    state=dict(state='running',pid=os.getpid(),started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    save(run/'state.json',state)
    child=None;awake=None
    def stop(signum,frame):raise KeyboardInterrupt('Stop requested')
    signal.signal(signal.SIGTERM,stop)
    try:
        if sys.platform=='darwin' and shutil.which('caffeinate'):
            awake=subprocess.Popen(['caffeinate','-i','-s','-w',str(os.getpid())])
        cmd=[sys.executable,str(snap/'tools/run_benchmark.py'),'--suite','full','--matrix',info['profile'],
             '--repeats','3','--out',str(run/'results'),'--eta-model',str(snap/'tools/team_timing_model.json')]
        print('RUN', ' '.join(cmd),flush=True)
        child=subprocess.Popen(cmd,cwd=snap,start_new_session=True)
        state['runner_pid']=child.pid;save(run/'state.json',state)
        code=child.wait();state['runner_exitcode']=code
        if code==0:
            analysis=subprocess.run([sys.executable,str(snap/'tools/analyze_team_series.py'),str(run/'results')],cwd=snap)
            state['analysis_exitcode']=analysis.returncode
            state['state']='completed' if analysis.returncode==0 else 'analysis_failed'
        else:state['state']='failed'
    except KeyboardInterrupt:
        state['state']='paused'
    except BaseException as exc:
        state['state']='failed';state['error']=str(exc)
        raise
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                # Worker is in its own group; runner normally kills it in its
                # interrupt handler. Avoid a blind process-group SIGKILL here.
                state['state']='stop_pending';state['error']='Runner has not exited; inspect before resuming'
                child.wait()
        if awake and awake.poll() is None:awake.terminate();awake.wait()
        state['finished_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime());save(run/'state.json',state)
        lock.close()
        print('SUPERVISOR',state['state'],flush=True)
def start(run,profile):
    run.mkdir(parents=True,exist_ok=True)
    guard=locked(run/'launch.lock')
    if guard is None:raise RuntimeError('Another start/prepare command is active')
    try:
        if alive(run):print('Already running.');show(run);return
        info=prepare(run,profile)
        if (run/'state.json').exists():
            s=json.loads((run/'state.json').read_text())
            if s['state']=='completed':print('Already completed.');show(run);return
        with (run/'run.log').open('a',buffering=1) as log:
            process=subprocess.Popen([sys.executable,str(run/'snapshot/tools/team_benchmark.py'),'_run',
                 '--run-dir',str(run),'--profile',info['profile']],cwd=run/'snapshot',
                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True,stdin=subprocess.DEVNULL)
        for _ in range(50):
            if process.poll() is not None:raise RuntimeError('Supervisor failed to start; see '+str(run/'run.log'))
            if (run/'state.json').exists() and json.loads((run/'state.json').read_text()).get('pid')==process.pid:break
            time.sleep(.1)
        print('Started in background, supervisor PID:',process.pid)
        show(run)
    finally:guard.close()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['prepare','start','resume','status','stop','analyze','_run'])
    p.add_argument('--run-dir',type=Path,default=DEFAULT)
    p.add_argument('--profile',choices=sorted(MATRICES),default='team8')
    a=p.parse_args();run=a.run_dir.resolve()
    if a.action=='prepare':
        run.mkdir(parents=True,exist_ok=True);guard=locked(run/'launch.lock')
        if guard is None:raise RuntimeError('Another preparation/start is active')
        try:prepare(run,a.profile);show(run)
        finally:guard.close()
    elif a.action in ['start','resume']:start(run,a.profile)
    elif a.action=='status':show(run)
    elif a.action=='_run':supervisor(run)
    elif a.action=='stop':
        if not alive(run):print('Already stopped.');return
        s=json.loads((run/'state.json').read_text());os.kill(s['pid'],signal.SIGTERM)
        print('Stop requested; completed records are retained. Check status before resuming.')
    elif a.action=='analyze':
        if alive(run):raise RuntimeError('Pause the run before offline analysis')
        verify(run)
        subprocess.run([sys.executable,str(run/'snapshot/tools/analyze_team_series.py'),str(run/'results')],check=True)
if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.CalledProcessError) as exc:raise SystemExit(str(exc))
