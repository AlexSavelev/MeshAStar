#!/usr/bin/env python3
"""Tests of timeout cleanup and distinction between missing and bad references."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('runner',ROOT/'tools/run_benchmark.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)


class RunnerTests(unittest.TestCase):
    def test_success_and_error_are_not_timeouts(self):
        code,out,_,_,timed=r.isolated([sys.executable,'-c','print("ok")'],5)
        self.assertEqual((code,out.strip(),timed),(0,'ok',False))
        code,_,_,_,timed=r.isolated([sys.executable,'-c','raise SystemExit(3)'],5)
        self.assertEqual((code,timed),(3,False))

    def test_timeout_kills_descendant(self):
        with tempfile.TemporaryDirectory() as tmp:
            beat=Path(tmp)/'heartbeat'
            child='from pathlib import Path; import time\np=Path('+repr(str(beat))+')\nwhile True:\n p.write_text(str(time.monotonic()))\n time.sleep(.02)'
            parent='import subprocess,sys,time\nsubprocess.Popen([sys.executable,"-c",'+repr(child)+'])\ntime.sleep(20)'
            code,_,_,wall,timed=r.isolated([sys.executable,'-c',parent],.7)
            self.assertTrue(timed);self.assertNotEqual(code,0);self.assertLess(wall,5)
            self.assertTrue(beat.exists())
            value=beat.read_text();time.sleep(.15)
            self.assertEqual(value,beat.read_text(),'Descendant survived process-group kill')

    def test_validation_distinguishes_cases(self):
        good=dict(status='solved',cost=10,path_valid=True)
        unreachable=dict(status='exhausted_unreachable',cost=-1,path_valid=True)
        self.assertEqual(r.verdict(good,dict(status='timeout'),{}),'reference_unknown')
        self.assertEqual(r.verdict(dict(status='timeout'),good,{}),'not_completed')
        self.assertEqual(r.verdict(good,unreachable,{}),'reachability_mismatch')
        self.assertEqual(r.verdict(dict(good,cost=10.01),good,{}),'cost_mismatch')
        self.assertEqual(r.verdict(dict(good,path_valid=False),good,{}),'invalid_path')
        self.assertEqual(r.verdict(good,good,{}),'pass')
        self.assertEqual(r.verdict(unreachable,unreachable,{}),'pass')
        self.assertEqual(r.verdict(good,None,{'expected':'unreachable'}),'expected_unreachable_failed')

    def test_replayed_paths_compare_double_costs(self):
        ref=dict(status='solved',cost=1000.,path_cost_double=999.99,path_valid=True,cost_validation='float-replay-v1')
        same=dict(ref,cost=1000.01)
        self.assertEqual(r.verdict(same,ref,{}),'pass')
        worse=dict(ref,path_cost_double=1000.01)
        self.assertEqual(r.verdict(worse,ref,{}),'cost_mismatch')
        self.assertEqual(r.verdict(dict(same,path_valid=False),ref,{}),'invalid_path')


if __name__=='__main__':unittest.main()
