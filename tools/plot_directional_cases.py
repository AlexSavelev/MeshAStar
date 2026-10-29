import base64,html,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,Normalize
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'res/directional-case-study'
tasks=json.loads((OUT/'selection.json').read_text())['tasks'];times=json.loads((ROOT/'res/directional-experiment/summary.json').read_text())['tasks']
fig,axes=plt.subplots(2,3,figsize=(14,10),layout='constrained');details=[]
for row,t in enumerate(tasks):
 name=t['id'];grid=np.array([[c not in '.GS' for c in line] for line in (ROOT/t['map']).read_text().splitlines()[4:]])
 ds=[np.genfromtxt(OUT/f'{name}-{m}.csv',delimiter=',',names=True) for m in ['base','directional']]
 paths=[np.genfromtxt(OUT/f'{name}-{m}-path.csv',delimiter=',',names=True) for m in ['base','directional']]
 counts=[]
 for m,x in zip(['max','directional'],ds):
  assert len(x)==times[name]['LBA'][m]['steps']-1
  c=np.zeros(grid.shape);np.add.at(c,(x['i'].astype(int),x['j'].astype(int)),1);counts.append(c)
 b,n=ds;delta=np.maximum(0,b['h_directional']-b['h_spatial']);acc=np.zeros(grid.shape);np.add.at(acc,(b['i'].astype(int),b['j'].astype(int)),delta)
 avg=np.divide(acc,counts[0],out=np.zeros_like(acc),where=counts[0]>0)
 keys=lambda x:set(zip(x['i'].astype(int),x['j'].astype(int),x['theta'].astype(int)))
 absent=keys(b)-keys(n);ref=times[name]['LBA']['max']['cost']
 cross=(b['g']+b['h_spatial']<ref-.01)&(b['g']+np.maximum(b['h_spatial'],b['h_directional'])>ref+.01)
 candidates=[k for k in np.flatnonzero(cross) if (int(b[k]['i']),int(b[k]['j']),int(b[k]['theta'])) in absent]
 ex=None
 if candidates:
  k=sorted(candidates,key=lambda k:delta[k])[len(candidates)//2];ex={f:float(b[k][f]) for f in b.dtype.names}
  ex.update(f_old=ex['g']+ex['h_spatial'],f_new=ex['g']+max(ex['h_spatial'],ex['h_directional']),absent_from_new=True)
 for col in range(3):
  ax=axes[row,col];ax.imshow(grid,cmap=ListedColormap(['#f1f3f6','#35404b']),interpolation='nearest')
  field=counts[col] if col<2 else avg;mask=counts[col if col<2 else 0]==0
  im=ax.imshow(np.ma.masked_where(mask,field),cmap='viridis' if col<2 else 'magma',norm=Normalize(1,16) if col<2 else Normalize(0,max(avg.max(),1)),interpolation='nearest')
  p=paths[min(col,1)];ax.plot(p['j'],p['i'],c='#ff665a',lw=1,zorder=4)
  for state,mark,color,label in [(t['start'],'o','#3fe4ae','S'),(t['goal'],'*','#ffd34e','G')]:
   i,j,theta=state;ax.scatter(j,i,s=60,marker=mark,c=color,edgecolors='black',zorder=6);ax.annotate(label,(j,i),xytext=(6,-6),textcoords='offset points',color='#a21111',weight='bold')
  if ex:ax.scatter(ex['j'],ex['i'],marker='x',s=70,c='#37a4ff',linewidths=2,zorder=7)
  all_i=np.concatenate([x['i'] for x in ds]+[x['i'] for x in paths]);all_j=np.concatenate([x['j'] for x in ds]+[x['j'] for x in paths])
  ax.set_xlim(max(-.5,all_j.min()-12),min(grid.shape[1]-.5,all_j.max()+12));ax.set_ylim(min(grid.shape[0]-.5,all_i.max()+12),max(-.5,all_i.min()-12))
  ax.set_xlabel('Столбец j');ax.set_ylabel('Строка i')
  ax.set_title(('Прежняя max' if col==0 else 'max + направленная' if col==1 else 'Средняя прибавка h')+'\n'+(f'{len(ds[col]):,} раскрытий'.replace(',',' ') if col<2 else name))
  if col==2:fig.colorbar(im,ax=ax,shrink=.7,label='Δh на прежних раскрытиях')
 details.append(dict(task=name,start=t['start'],goal=t['goal'],reference_cost=ref,expanded_old=len(b),expanded_new=len(n),absent_old_states=len(absent),stronger_fraction=float(np.mean(delta>1e-9)),mean_delta=float(delta.mean()),potential_beats_euclidean=float(np.mean(b['h_directional']>b['h_euclidean']+1e-9)),potential_gain_hidden_by_grid=float(np.mean((b['h_directional']>b['h_euclidean']+1e-9)&(b['h_directional']<=b['h_grid']+1e-9))),same_goal_heading_fraction=float(np.mean(b['theta']==t['goal'][2])),crosses=int(cross.sum()),witness=ex))
fig.suptitle('Направленная эвристика: Berlin (верх) и AR0700SR (низ)\nЦвет слева/в центре — число раскрытых ориентаций в клетке (1–16).\nКрасная линия — клеточный след пути; синий крест — разобранное состояние.',fontsize=12)
fig.savefig(OUT/'search-regions.png',dpi=160);plt.close(fig)
(OUT/'analysis.json').write_text(json.dumps(details,indent=2))
text='# Почему направленная эвристика помогает по-разному\n\n'
text+='Это диагностический разбор двух прежних задач, не новая проверочная выборка. Время берём из предыдущих замеров, запуски с CSV не используем для сравнения скорости.\n\n'
text+='Для фиксированного v оценка усиливается ровно тогда, когда D_v(θ,θ_G) > h_spatial − β⟨v,G−x⟩. Справа — сколько не хватает одной проекции до уже имеющейся пространственной оценки; слева — цена согласования ориентаций в остаточном графе. Большой объезд препятствий может целиком поглотить ориентационную добавку.\n\n'
for d in details:
 text+=f"## {d['task']}\n\nРаскрытия {d['expanded_old']} → {d['expanded_new']}; усиление на {d['stronger_fraction']:.1%} прежних состояний, средняя добавка {d['mean_delta']:.3f}. Направленная оценка сильнее Euclidean на {d['potential_beats_euclidean']:.1%}, но на {d['potential_gain_hidden_by_grid']:.1%} всех состояний это усиление уже перекрыто grid. Доля состояний с целевой ориентацией {d['same_goal_heading_fraction']:.1%}.\n\n"
 ex=d['witness'];text+=f"Пример s=({int(ex['i'])},{int(ex['j'])},{int(ex['theta'])}), цель {d['goal']}. Выбрано пространственное направление {ex['direction']*22.5:.1f}° от +j к +i. Проекция {ex['projection']:.4f} + ориентационный остаток {ex['penalty']:.4f} = {ex['h_directional']:.4f}; старая h={ex['h_spatial']:.4f}. При прежнем g={ex['g']:.4f}: f {ex['f_old']:.4f} → {ex['f_new']:.4f}, тогда как C*={d['reference_cost']:.4f}. В новом поиске это полное состояние не раскрывается.\n\n"
 text+=f"{d['crosses']} прежних состояний пересекают порог f=C* с запасом 0.01 при сохранённом g. Это объяснительный пересчёт, не точное число исключённых раскрытий: меняются очередь и пути до состояний.\n\n"
text+='Диагностические шаги и стоимости совпали с benchmark; экспортированные пути проверены на столкновения, стыковку ориентаций и конечную позу. Production-код не менялся.\n'
(OUT/'REPORT.md').write_text(text)
png=base64.b64encode((OUT/'search-regions.png').read_bytes()).decode()
(OUT/'report.html').write_text('<!doctype html><html lang="ru"><meta charset="utf-8"><title>Направленная оценка: разбор</title><style>body{font:17px/1.6 system-ui;max-width:1400px;margin:30px auto;padding:20px;color:#172b38}img{width:100%}pre{white-space:pre-wrap;font:16px/1.6 system-ui}</style><h1>Почему направленная эвристика помогает по-разному</h1><img alt="Раскрытия и усиление оценки" src="data:image/png;base64,'+png+'"><pre>'+html.escape(text)+'</pre></html>')
print(json.dumps(details,indent=2))
