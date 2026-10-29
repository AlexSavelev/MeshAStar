#!/usr/bin/env python3
"""Run paper maps then extension maps in a frozen snapshot; durable stage status."""
import json,os,signal,subprocess,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
status_path=ROOT/'results/full-status.json'
status_path.parent.mkdir(exist_ok=True)
state={'supervisor_pid':os.getpid(),'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'stages':[]}
child=None
def terminate(signum,frame):
    raise KeyboardInterrupt('Termination requested')
signal.signal(signal.SIGTERM,terminate)
def save():
    p=status_path.with_suffix('.tmp');p.write_text(json.dumps(state,indent=2)+'\n');p.replace(status_path)
try:
 for label,maps in [('paper','Labyrinth,ht_0_hightown,Moscow_0_512,AR0304SR'),
                    ('extension','Berlin_2_256,Milan_1_256,AR0700SR,WheelofWar')]:
    state['active_stage']=label;state['status']='running';save()
    cmd=[sys.executable,'tools/run_benchmark.py','--suite','full','--maps',maps,'--repeats','3','--out','results/full-'+label]
    with (ROOT/'results'/('full-'+label+'.log')).open('a') as log:
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        state['runner_pid']=child.pid;save();code=child.wait()
    output=ROOT/'results'/('full-'+label)/'runs.jsonl'
    count=len(output.read_text().splitlines()) if output.exists() else 0
    state['stages'].append({'name':label,'exitcode':code,'records':count,'expected_records':9120})
    if count!=9120 or code not in (0,1):
        raise RuntimeError('Stage interrupted or runner failed: '+label)
 state['status']='completed' if all(s['exitcode']==0 for s in state['stages']) else 'completed_with_validation_or_process_failures'
 state['active_stage']=None;save()
except BaseException as e:
 if child is not None and child.poll() is None:
    child.send_signal(signal.SIGINT)
    try:child.wait(timeout=10)
    except subprocess.TimeoutExpired:child.kill();child.wait()
 state['status']='interrupted_or_failed';state['error']=str(e);save();raise
