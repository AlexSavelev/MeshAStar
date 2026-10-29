#!/usr/bin/env python3
"""Plot Marat's detour ratio from saved, validated wall results (no new searches)."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=Path(__file__).resolve().parents[1]/'res/wall-detour-ratio'
r=json.loads((p/'data.json').read_text())['rows'][1:]
x=[t['rho'] for t in r]
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
axes[0].plot(x,[100*t['grid_stronger_fraction'] for t in r],'o-',color='#087f78')
axes[0].set_ylabel('Baseline LBA* states improved (%)')
for a in ['LBA','Mesh']:axes[1].plot(x,[t[a+'_speedup'] for t in r],'o-',label=a+'*')
axes[1].axhline(1,color='gray',linestyle='--')
axes[1].set_ylabel('Full-time speedup over safe Euclidean');axes[1].legend()
for ax in axes:
    ax.set_xlabel(r'$\rho=C^*_{obstacles}/C^*_{free}$');ax.grid(alpha=.2);ax.margins(x=.12,y=.15)
for t in r[2:]:axes[1].annotate('offset '+str(t['offset']),(t['rho'],t['Mesh_speedup']),xytext=(0,8),textcoords='offset points',fontsize=8,ha='center')
fig.suptitle('128×128 wall; gap width 17; same start/goal poses')
fig.savefig(p/'detour-ratio.png',dpi=160);fig.savefig(p/'detour-ratio.pdf');plt.close(fig)
