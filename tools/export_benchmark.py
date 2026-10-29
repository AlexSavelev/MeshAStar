#!/usr/bin/env python3
"""Export all measured rows, including failures, to spreadsheet-friendly CSV."""
import argparse
import csv
import json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('directory',type=Path)
a=p.parse_args()
rows=[json.loads(s) for s in (a.directory/'runs.jsonl').read_text().splitlines()]
columns=['job','stage','task','map','band','engine','variant','repeat','status','validation',
         'cost','path_cost_double','cost_validation','path_cost_float_replay','legacy_sum_ok','original_validation',
         'end_to_end_s','preprocessing_s','grid_s','directional_s',
         'search_s','setup_s','cleanup_s','reconstruction_s','load_s','warmup_s','validation_s',
         'process_wall_s','timeout_limit_s','steps','checked_cells','candidates','pushes','pops',
         'peak_open','peak_rss_bytes','closed_bytes','returncode']
with (a.directory/'results.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');w.writeheader();w.writerows(rows)
print(a.directory/'results.csv')
