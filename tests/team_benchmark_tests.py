#!/usr/bin/env python3
"""Integration checks for the new matrix, durable resume and paired analysis."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import run_benchmark as runner
import analyze_team_series as analysis
from team_benchmark_plan import MATRICES

class TeamTests(unittest.TestCase):
    def fixture(self,root):
        frozen=root/'res/final-benchmark-v1';frozen.mkdir(parents=True)
        task=dict(id='test',map='test.map',start=[0,0,0],goal=[0,0,0],expected='solved_zero_cost',family='identity')
        manifest=dict(seed=1,edge_tasks=[task],tasks=[],input_sha256={},prior_task_sources_sha256={})
        (frozen/'manifest.json').write_text(json.dumps(manifest))
        (frozen/'frozen-sha256.json').write_text('{}')
        (root/'tools').mkdir()
        for name in ['benchmark.mk','tools/benchmark_worker.cpp','tools/team_benchmark_plan.py','tools/run_benchmark.py']:
            (root/name).write_text('test')
        worker=root/'worker'
        row=dict(schema=1,status='solved',path_valid=True,cost=0,path_cost_double=0,cost_validation='float-replay-v1',path=[],
                 steps=0,checked_cells=0,candidates=0,pushes=0,pops=0,closed_discards=0,stale_discards=0,dominated=0,peak_open=0,
                 end_to_end_s=.02,preprocessing_s=.01,grid_s=.005,search_s=.01)
        worker.write_text('#!'+sys.executable+'\nprint('+repr(json.dumps(row))+')\n');worker.chmod(0o755)
        return frozen,worker
    def test_matrix(self):
        self.assertEqual(len(MATRICES['team8']),8)
        self.assertEqual(len(MATRICES['extended10']),10)
        self.assertEqual(len(set(MATRICES['team8'])),8)
        self.assertNotIn(('LBA','max-buckets'),MATRICES['team8'])
    def test_run_resume_analysis_and_changed_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp).resolve();frozen,worker=self.fixture(root);out=root/'output'
            args=['runner','--suite','edge','--matrix','extended10','--repeats','2','--out',str(out),'--worker',str(worker)]
            oldsignal=signal.getsignal(signal.SIGTERM)
            try:
                with mock.patch.object(runner,'ROOT',root),mock.patch.object(runner,'__file__',str(root/'tools/run_benchmark.py')),mock.patch.object(runner,'FROZEN',frozen),mock.patch.object(sys,'argv',args),contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(runner.main(),0)
                    original=(out/'runs.jsonl').read_bytes()
                    self.assertEqual(len(original.splitlines()),21)
                    self.assertEqual(runner.main(),0)
                    self.assertEqual((out/'runs.jsonl').read_bytes(),original)
                    report=analysis.analyze(out)
                    self.assertTrue(report['complete']);self.assertEqual(report['common_solved_tasks'],1)
                    self.assertFalse(report['bucket_mismatches'])
                    args[args.index('--repeats')+1]='3'
                    with self.assertRaisesRegex(RuntimeError,'changed'):runner.main()
            finally:signal.signal(signal.SIGTERM,oldsignal)
    def test_truncated_log_rejected_without_modification(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'runs.jsonl';p.write_text('{"job":"ok"}\n{"job":')
            before=p.read_bytes()
            with self.assertRaisesRegex(RuntimeError,'line 2'):runner.load_prior(p)
            self.assertEqual(p.read_bytes(),before)

    def test_background_stop_resume(self):
        # Tiny fake workload exercises the REAL supervisor, never the full suite.
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp).resolve();tools=run/'snapshot/tools';tools.mkdir(parents=True)
            for name in ['team_benchmark.py','team_benchmark_plan.py']:
                shutil.copy2(ROOT/'tools'/name,tools/name)
            (run/'snapshot.json').write_text(json.dumps(dict(profile='team8',sha256={})))
            (run/'estimate.json').write_text(json.dumps(dict(expected_records=30,historical_process_hours=.001)))
            (tools/'run_benchmark.py').write_text('''import sys,time,signal,json
from pathlib import Path
out=Path(sys.argv[sys.argv.index('--out')+1]);out.mkdir(exist_ok=True)
p=out/'fake-count.txt';n=int(p.read_text()) if p.exists() else 0
def stop(s,f):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop)
for i in range(n,30):
 time.sleep(.05);p.write_text(str(i+1))
''')
            (tools/'analyze_team_series.py').write_text('print("fake analysis completed")\n')
            cmd=[sys.executable,str(ROOT/'tools/team_benchmark.py')]
            def invoke(action):return subprocess.check_output(cmd+[action,'--run-dir',str(run)],stderr=subprocess.STDOUT,text=True,timeout=10)
            try:
                invoke('start')
                # Wait for at least one completed fake job before pausing.
                for _ in range(50):
                    if (run/'results/fake-count.txt').exists():break
                    time.sleep(.02)
                invoke('stop')
                for _ in range(100):
                    if json.loads((run/'state.json').read_text())['state']=='paused':break
                    time.sleep(.02)
                self.assertEqual(json.loads((run/'state.json').read_text())['state'],'paused')
                saved=int((run/'results/fake-count.txt').read_text());self.assertGreater(saved,0);self.assertLess(saved,30)
                invoke('resume')
                for _ in range(200):
                    if json.loads((run/'state.json').read_text())['state']=='completed':break
                    time.sleep(.02)
                self.assertEqual(json.loads((run/'state.json').read_text())['state'],'completed')
                self.assertEqual(int((run/'results/fake-count.txt').read_text()),30)
            finally:
                invoke('stop')

if __name__=='__main__':unittest.main()
