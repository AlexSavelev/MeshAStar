#!/usr/bin/env python3
"""Explain post-hoc best/worst cached P256 cases; no new timing claims."""
import base64
import html
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, Normalize

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'res/case-study'
tasks=json.loads((OUT/'selection.json').read_text())['tasks']
refs={r['task']:r for r in map(json.loads,(ROOT/'res/multimap-experiment/references.jsonl').read_text().splitlines())}
timings=json.loads((ROOT/'res/multimap-experiment/summary.json').read_text())['tasks']
index={(r['task'],r['algorithm'],r['variant'],r['phase']):r for r in timings}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig,axes=plt.subplots(2,3,figsize=(14,10),layout='constrained')
details=[]
angles=[0,np.arctan(.5),np.pi/4,np.arctan(2),np.pi/2,np.pi-np.arctan(2),3*np.pi/4,np.pi-np.arctan(.5),np.pi,np.pi+np.arctan(.5),5*np.pi/4,np.pi+np.arctan(2),3*np.pi/2,2*np.pi-np.arctan(2),7*np.pi/4,2*np.pi-np.arctan(.5)]
for row,t in enumerate(tasks):
    name=t['id'];grid=np.array([[c not in '.GS' for c in line] for line in (ROOT/t['map']).read_text().splitlines()[4:]])
    data=[np.genfromtxt(OUT/(name+'-'+mode+'.csv'),delimiter=',',names=True) for mode in ['base','kin']]
    paths=[np.genfromtxt(OUT/(name+'-'+mode+'-path.csv'),delimiter=',',names=True) for mode in ['base','kin']]
    counts=[]
    for k,d in enumerate(data):
        expected=index[name,'LBA','max' if k==0 else 'kin256','baseline' if k==0 else 'cached']['steps']
        assert len(d)==expected-1
        count=np.zeros(grid.shape);np.add.at(count,(d['i'].astype(int),d['j'].astype(int)),1);counts.append(count)
    base=data[0];delta=np.maximum(0,base['h_kin']-base['h_spatial'])
    summed=np.zeros(grid.shape);np.add.at(summed,(base['i'].astype(int),base['j'].astype(int)),delta)
    avg=np.divide(summed,counts[0],out=np.zeros_like(summed),where=counts[0]>0)
    all_i=np.concatenate([d['i'] for d in data]+[p['i'] for p in paths]);all_j=np.concatenate([d['j'] for d in data]+[p['j'] for p in paths])
    lim=(max(-.5,min(all_j)-12),min(grid.shape[1]-.5,max(all_j)+12),min(grid.shape[0]-.5,max(all_i)+12),max(-.5,min(all_i)-12))
    for col in range(3):
        ax=axes[row,col];ax.imshow(grid,cmap=ListedColormap(['#f4f6f8','#364250']),vmin=0,vmax=1,interpolation='nearest')
        field=counts[col] if col<2 else avg
        masked=np.ma.masked_where(counts[col if col<2 else 0]==0,field)
        im=ax.imshow(masked,cmap='viridis' if col<2 else 'magma',norm=Normalize(1,16) if col<2 else Normalize(0,max(avg.max(),1)),interpolation='nearest')
        p=paths[min(col,1)];ax.plot(p['j'],p['i'],color='#ff6759',lw=1.1,zorder=4)
        for state,label,marker,colour in [(t['start'],'S','o','#1cdc92'),(t['goal'],'G','*','#ffdd5a')]:
            i,j,heading=state;a=angles[heading]
            ax.scatter(j,i,s=65,marker=marker,c=colour,edgecolors='black',linewidths=.6,zorder=6)
            ax.annotate('',xy=(j+12*np.cos(a),i+12*np.sin(a)),xytext=(j,i),arrowprops=dict(arrowstyle='->',color='#ee3535',lw=2),zorder=7)
            ax.annotate(label,(j,i),xytext=(6,-7),textcoords='offset points',weight='bold',color='#a21111',zorder=8)
        ax.set_xlim(lim[:2]);ax.set_ylim(lim[2:]);ax.set_xlabel('Столбец j');ax.set_ylabel('Строка i')
        if col<2:
            ax.set_title(('Прежняя max' if col==0 else 'max + кинематика P256')+f"\n{len(data[col]):,} раскрытых состояний".replace(',',' '))
        else:
            ax.set_title('Где оценка становится сильнее\nСреднее Δh по раскрытым ориентациям')
            fig.colorbar(im,ax=ax,shrink=.65,label='Δh = max(0, hкин − hпростр)')
    keys=lambda d:set(zip(d['i'].astype(int),d['j'].astype(int),d['theta'].astype(int)))
    absent=keys(base)-keys(data[1])
    c=refs[name]['cost'];cross=(base['g']+base['h_spatial']<c-.01)&(base['g']+np.maximum(base['h_spatial'],base['h_kin'])>c+.01)
    candidates=np.flatnonzero(cross)
    # Representative witness near the median positive strengthening, not its maximum.
    example=None
    if len(candidates):
        k=int(candidates[np.argsort(delta[candidates])[len(candidates)//2]])
        example={field:float(base[field][k]) for field in base.dtype.names}
        example['f_old']=example['g']+example['h_spatial'];example['f_new']=example['g']+max(example['h_spatial'],example['h_kin'])
        example['absent_from_new_expansions']=(int(example['i']),int(example['j']),int(example['theta'])) in absent
    details.append(dict(task=name,start=t['start'],goal=t['goal'],optimal_reference=c,
        expanded_old=len(base),expanded_new=len(data[1]),old_states_absent_in_new=len(absent),
        stronger_fraction=float(np.mean(delta>1e-9)),mean_delta=float(np.mean(delta)),
        baseline_f_crosses_reference=int(sum(cross)),example=example,
        old_ms=index[name,'LBA','max','baseline']['end_to_end_s']*1000,
        new_ms=index[name,'LBA','kin256','cached']['end_to_end_s']*1000))
fig.suptitle('LBA*: сильный эффект на Berlin (верх) и слабый на AR0700SR (низ)\nS — старт, G — цель; стрелки — ориентации. Красная линия — клеточный след найденного пути.\nЦвет первых двух столбцов: от 1 (тёмный) до 16 (светлый) раскрытых ориентаций в клетке.',fontsize=12)
fig.savefig(OUT/'search-regions.png',dpi=160);plt.close(fig)
(OUT/'analysis.json').write_text(json.dumps(details,indent=2)+'\n')
text='''# Два случая использования кинематической эвристики

Выбраны ПОСЛЕ benchmark: лучший и худший по ускорению LBA* с готовой P256
среди 17 решаемых задач. Это объяснительный разбор крайних случаев, не новая
репрезентативная выборка. Времена взяты из прежних трёх повторов; диагностические
запуски с выводом состояний не используются для сравнения скорости.

В каждой клетке может быть несколько состояний с разными ориентациями.
Цвет первых двух панелей — количество раскрытых ориентаций (1–16).
Третья панель — средняя добавка к h по ориентациям, раскрытым старым поиском;
она не является самой эвристикой и не используется в A*.
Пути показаны как последовательности клеток collision trace примитивов,
а не как непрерывные гладкие кривые. При одинаковой стоимости пути могут различаться.
Число раскрытых состояний не включает извлечение цели; steps в benchmark на 1 больше.

'''
for d in details:
    text+=f"## {d['task']}\n\nСтарт {d['start']}, цель {d['goal']} (i,j,индекс ориентации).\nСтоимость h=0: {d['optimal_reference']:.6f}.\nРаскрытия: {d['expanded_old']} → {d['expanded_new']}.\nВремя с готовой таблицей: {d['old_ms']:.2f} → {d['new_ms']:.2f} мс.\nУсиление на {100*d['stronger_fraction']:.2f}% прежних раскрытий; средняя добавка {d['mean_delta']:.3f}.\n{d['baseline_f_crosses_reference']} прежних раскрытий переходят с f<C*−0.01 на f>C*+0.01 при том же g и новой h.\n\n"
    text+='Пример состояния:\n\n```json\n'+json.dumps(d['example'],ensure_ascii=False,indent=2)+'\n```\n\n'
text+='''## Что показывает сравнение f

При неизменном g увеличение h поднимает приоритет f=g+h. Состояния,
которые раньше выглядели перспективными, откладываются, и цель может быть
извлечена раньше них. Сравнение с C* сделано постфактум для объяснения;
алгоритм не знает оптимальную стоимость заранее и не применяет этот порог.
Число пересечений порога не равно числу исчезнувших раскрытий: меняются
порядок поиска, обнаруженные пути и g. В коде сохранены float g/f.

Оба диагностических поиска совпали по steps и стоимости с benchmark.
Повторные поиски для экспорта пути также совпали; проверены trace-коллизии,
стыковка ориентаций и конечная поза. Production-код не менялся.

Воспроизведение из корня MeshAStar:
make obj/inspect_cases
./obj/inspect_cases
MPLCONFIGDIR=/tmp/mesha-mpl-cache XDG_CACHE_HOME=/tmp/mesha-font-cache python3 tools/plot_cases.py
'''
(OUT/'REPORT.md').write_text(text)
png=base64.b64encode((OUT/'search-regions.png').read_bytes()).decode()
page='<!doctype html><html lang="ru"><meta charset="utf-8"><title>MeshA*: два примера</title><style>body{font:17px/1.6 system-ui;max-width:1400px;margin:30px auto;padding:20px;color:#172b38}img{width:100%}pre{white-space:pre-wrap;font:16px/1.6 system-ui}</style><h1>Почему кинематическая оценка помогает по-разному</h1>'
page+='<img alt="Области раскрытий и усиление эвристики" src="data:image/png;base64,'+png+'"><pre>'+html.escape(text)+'</pre></html>'
(OUT/'report.html').write_text(page)
print(json.dumps(details,indent=2))
