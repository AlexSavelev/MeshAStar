#!/usr/bin/env python3
"""Static figure from matched full-series summary; no search is executed."""
import json
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--directory',type=Path,default=ROOT/'res/full-analysis-2026-10-27')
args=parser.parse_args()
OUT = args.directory
rows = json.loads((OUT/'summary.json').read_text())
maps = ['Labyrinth','ht_0_hightown','Moscow_0_512','AR0304SR',
        'Berlin_2_256','Milan_1_256','AR0700SR','WheelofWar']
counts={(r['engine'],r['scope']):r['tasks'] for r in rows if r['outcome']=='solved'}
labels=[]
for name in maps:
    a,b=counts['LBA',name],counts['Mesh',name]
    labels.append(f'{name} (n={a})' if a==b else f'{name} (n={a}/{b})')
fig, axes = plt.subplots(1,2,figsize=(12,6.4),sharey=True)
for ax,engine in zip(axes,['LBA','Mesh']):
    rr = {r['scope']:r for r in rows if r['engine']==engine and r['outcome']=='solved'}
    for field,color,offset,label in [('grid_aggregate_speedup','#1a728e',-.12,'safe Euclid → max'),
                                    ('directional_aggregate_speedup','#d56a35',.12,'max → directional')]:
        values=[rr[m][field] for m in maps]
        ypos=[i+offset for i in range(len(maps))]
        ax.scatter(values,ypos,s=48,color=color,label=label,zorder=3)
    ax.axvline(1,color='#777777',ls='--',lw=1)
    ax.set_xlim(.85,2.65)
    ax.set_title(('LBA*' if engine=='LBA' else 'MeshA*')+' — matched solved queries',fontsize=12,pad=12)
    ax.set_xlabel('Speedup: sum of baseline times / sum of new times')
    ax.set_yticks(range(len(maps)),labels)
    ax.grid(axis='x',alpha=.15)
    ax.spines[['top','right']].set_visible(False)
axes[0].invert_yaxis()
fig.legend(*axes[1].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.55,.94),ncol=2,fontsize=10)
fig.suptitle('Full solve time includes heuristic preparation',fontsize=15,y=.98)
totals={r['engine']:r['tasks'] for r in rows if r['scope']=='ALL' and r['outcome']=='solved'}
fig.text(.5,.025,f'Per-query medians of 2–3 common clean repeats. Validation policy: see report.\nSolved subset only: {totals["LBA"]} LBA* / {totals["Mesh"]} MeshA* queries. Unreachable queries are reported separately.',ha='center',fontsize=10)
fig.subplots_adjust(left=.22,right=.98,top=.82,bottom=.17,wspace=.10)
fig.savefig(OUT/'solved-speedups.png',dpi=170)
fig.savefig(OUT/'solved-speedups.pdf')
print(OUT/'solved-speedups.png')
