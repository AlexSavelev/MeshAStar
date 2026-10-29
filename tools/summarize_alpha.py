#!/usr/bin/env python3
"""Fixed four-way alpha sweep; no holdout/generalization claims."""
import base64, hashlib, html, json, statistics
from collections import defaultdict
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'res/alpha-experiment'
ALPHAS={'a030':.3,'a060':.6,'a075':.75,'a090':.9}
def read(name):return [json.loads(x) for x in (OUT/(name+'.jsonl')).read_text().splitlines()]
def main():
    raw,batches,refs,prep=[read(s) for s in ['runs','batches','references','preparation']]
    assert len(raw)==1296 and len(batches)==54 and len(refs)==24 and len(prep)==96
    manifest=json.loads((OUT/'manifest.json').read_text());tasks={t['id']:t for t in manifest['tasks']}
    assert (OUT/'tasks.tsv').read_bytes()==(ROOT/'res/multimap-experiment/tasks.tsv').read_bytes()
    for p,digest in manifest['inputs_sha256'].items():assert hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==digest
    ref={r['task']:r for r in refs};groups=defaultdict(list)
    for r in raw:
        assert r['task'] in tasks and r['solved']==ref[r['task']]['solved']
        assert abs(r['cost']-ref[r['task']]['cost'])<.001
        groups[r['task'],r['algorithm'],r['variant'],r['phase']].append(r)
    case_stats={}
    for key,rs in groups.items():
        assert len(rs)==3 and {r['repeat'] for r in rs}=={0,1,2} and len({r['steps'] for r in rs})==1
        case_stats[key]=dict(zip(['task','algorithm','variant','phase'],key),steps=rs[0]['steps'],solved=rs[0]['solved'],time_s=statistics.median(r['end_to_end_s'] for r in rs))
    for task in tasks:
        for algo in ['LBA','Mesh']:
            for v in ALPHAS:assert case_stats[task,algo,v,'cold']['steps']==case_stats[task,algo,v,'cached']['steps']
    previous=json.loads((ROOT/'res/additive-experiment/summary.json').read_text())['tasks']
    for r in previous:
        if r['variant'] not in ['max','add64']:continue
        v='max' if r['variant']=='max' else 'a090'
        assert r['steps']==case_stats[r['task'],r['algorithm'],v,r['phase']]['steps']
    for b in batches:
        rs=[r for r in raw if all(r[k]==b[k] for k in ['algorithm','variant','phase','repeat'])]
        assert len(rs)==24 and {r['task'] for r in rs}==set(tasks)
        assert abs(sum(r['end_to_end_s'] for r in rs)+b['build_s']-b['compute_s'])<1e-7
        base=[r for r in raw if r['algorithm']=='LBA' and r['variant']=='max' and r['repeat']==b['repeat']]
        assert [r['task'] for r in sorted(rs,key=lambda x:x['position'])]==[r['task'] for r in sorted(base,key=lambda x:x['position'])]
        b['solved_compute_s']=b['build_s']+sum(r['end_to_end_s'] for r in rs if r['solved'])
    summaries=[]
    combinations=[('max','baseline')]+[(v,p) for v in ALPHAS for p in ['cached','cold']]
    solved=[t for t in tasks if ref[t]['solved']];assert len(solved)==17
    for algo in ['LBA','Mesh']:
        for v,phase in combinations:
            rs=[b for b in batches if (b['algorithm'],b['variant'],b['phase'])==(algo,v,phase)];assert len(rs)==3
            result=dict(algorithm=algo,variant=v,phase=phase)
            for field in ['compute_s','solved_compute_s','build_s']:
                a=[r[field] for r in rs];result[field]=statistics.median(a);result[field+'_min']=min(a);result[field+'_max']=max(a)
            ratios=[]
            for r in rs:
                base=next(b for b in batches if b['algorithm']==algo and b['variant']=='max' and b['repeat']==r['repeat'])
                ratios.append(base['compute_s']/r['compute_s'])
            result['paired_speedups']=ratios
            result['median_query_speedup']=statistics.median(case_stats[t,algo,'max','baseline']['time_s']/case_stats[t,algo,v,phase]['time_s'] for t in solved)
            result['solved_steps']=sum(case_stats[t,algo,v,phase]['steps'] for t in solved)
            result['unchanged_steps_tasks']=sum(case_stats[t,algo,v,phase]['steps']==case_stats[t,algo,'max','baseline']['steps'] for t in solved)
            summaries.append(result)
    error=max(abs(r['cost']-ref[r['task']]['cost']) for r in raw)
    (OUT/'summary.json').write_text(json.dumps(dict(batches=summaries,tasks=list(case_stats.values()),max_cost_error=error),indent=2)+'\n')
    def get(algo,v,phase='cached'):return next(r for r in summaries if (r['algorithm'],r['variant'],r['phase'])==(algo,v,phase))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(11,8),layout='constrained')
    for row,algo in enumerate(['LBA','Mesh']):
        for col,metric in enumerate(['compute_s','solved_compute_s']):
            ax=axes[row,col];baseline=get(algo,'max','baseline')
            ax.axhline(baseline[metric],color='#657383',ls='--',label='Пространственная max')
            ax.axhspan(baseline[metric+'_min'],baseline[metric+'_max'],color='#657383',alpha=.12,label='Диапазон базы')
            for phase,label,colour in [('cold','Таблица на каждый запрос','#e69b36'),('cached','Четыре таблицы на серию','#14859a')]:
                rs=[get(algo,v,phase) for v in ALPHAS];vals=[r[metric] for r in rs]
                ax.errorbar(list(ALPHAS.values()),vals,yerr=[[r[metric]-r[metric+'_min'] for r in rs],[r[metric+'_max']-r[metric] for r in rs]],fmt='o-',color=colour,capsize=3,label=label)
            ax.set_title(algo+'*: '+('все 24 задачи' if col==0 else '17 решаемых задач'))
            ax.set_xlabel('α разделения стоимости');ax.set_ylabel('Суммарное время, с');ax.set_xticks(list(ALPHAS.values()))
            ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('P=64: меняем разделение стоимости, сохраняем пространственную базу α=0.9\nПодготовка включена. Медианы трёх повторов; усы — минимум и максимум. Оси времени увеличены.',fontsize=12)
    fig.savefig(OUT/'results.png',dpi=160);plt.close(fig)
    text=f'''# Проверка четырёх значений alpha

P=64; alpha=0.3,0.6,0.75,0.9 зафиксированы до замеров. Формула:
h=max(beta*d_E, 0.9*d_grid, alpha*d_grid+h_res(alpha)).
Остаточные стоимости r(p)=c(p)-alpha*D_p. beta=0.9933.
Для произвольного control set код ограничивает коэффициент пространственной
базы допустимым calibration.alpha_limit; для нашего набора база равна0.9.
Это максимум согласованных оценок; прежняя пространственная база не ослабляется.
Гарантия математическая, float g/f планировщика сохранены.

Та же зафиксированная выборка24 задач: Berlin, Milan, AR0700SR;17решаемых,
7недостижимых. Подбор на знакомой выборке, не независимая проверка обобщения.
1296 измеряемых поисков,3повтора,24эталонаh=0,10прогревочных запусков.
Все найденные пути проверены. Максимальное отличие стоимости от эталона {error:.8g}.
Проверена неизменность steps между cold/cached и повторами;
max и alpha0.9 также совпали по steps с предыдущим экспериментом.

Время серии=sum(solve end_to_end)+подготовка внешних таблиц и residual costs.
Cached строит четыре таблицы для goal headings0,3,8,13 на24разных запроса;
cold строит таблицу в каждом solve. Grid строится каждый запрос во всехрежимах.
Исключены загрузка входов, первоначальныйHEAP, проверки/вывод и удаление
внешнего кэша. Cold не означает сброс аппаратного кэша или новый процесс.
Правые панели — решаемое подмножество тех же запусков с полной подготовкой.

## Медианы времени серии, секунды

| Алгоритм | Вариант | Cached24 | Cold24 | Cached17 | Cold17 | Шаги на17задачах |
|---|---|---:|---:|---:|---:|---:|
'''
    for algo in ['LBA','Mesh']:
        b=get(algo,'max','baseline');text+=f"| {algo} | max | {b['compute_s']:.3f} | — | {b['solved_compute_s']:.3f} | — | {b['solved_steps']} |\n"
        for v,alpha in ALPHAS.items():
            c=get(algo,v);d=get(algo,v,'cold')
            text+=f"| {algo} | α={alpha} | {c['compute_s']:.3f} | {d['compute_s']:.3f} | {c['solved_compute_s']:.3f} | {d['solved_compute_s']:.3f} | {c['solved_steps']} |\n"
    text+='\n## Изменения отдельных задач\n\n'
    for algo in ['LBA','Mesh']:
        for v,alpha in ALPHAS.items():
            r=get(algo,v);text+=f"- {algo}, α={alpha}: число шагов точно совпало с базой в {r['unchanged_steps_tasks']}/17 решаемых задачах; медиана ускорения одного запроса с готовой таблицей {r['median_query_speedup']:.3f}×.\n"
    text+='\n## Проверки и воспроизведение\n\nmake test obj/alpha_experiment\n\n./obj/alpha_experiment OUTPUT_FOLDER 3\n\nВ OUTPUT_FOLDER положить неизменённый tasks.tsv. Runner отказывается перезаписывать результаты. tools/summarize_alpha.py читает res/alpha-experiment и проверяет полноту, порядок задач, хеши входов, стоимости, число шагов и суммы времени.\n\nДля проверки новых alpha добавлены тесты согласованности на валидных рёбрах, доминирования фиксированной пространственной базы, нуля в цели и независимой прямой Дейкстры. Старые тесты также проходят.\n'
    if (OUT/'table-bounds.jsonl').exists():
        bounds=read('table-bounds');assert len(bounds)==16
        text+='\n## Ограничение усиления при alpha<0.9\n\nПусть M_alpha=max h_res по всей периодической таблице. Если\n(0.9-alpha)*d_grid >= M_alpha,\nто alpha*d_grid+h_res <= 0.9*d_grid: новая сумма заведомо не усиливает базу.\nЭто ограничение по grid-расстоянию с препятствиями, не по евклидову радиусу.\nМаксимумы вычислены полным перебором абстрактных состояний после benchmark.\n\n'
        for alpha in ALPHAS.values():
            rs=[b for b in bounds if b['alpha']==alpha];assert len(rs)==4
            m=max(b['max_residual_h'] for b in rs)
            if alpha<.9:
                cutoff=max(b['grid_distance_cutoff'] for b in rs)
                text+=f'- alpha={alpha}: M не больше {m:.6f}; при d_grid >= {cutoff:.6f} усиление невозможно для всех четырёх проверенных ориентаций цели.\n'
            else:text+=f'- alpha={alpha}: M не больше {m:.6f}; ограничение через деление на0.9-alpha неприменимо.\n'
        text+='\nВнутри указанной области усиление не гарантировано; безопасная евклидова оценка может быть сильнее. Это объясняет ограничение конструкции, но не предсказывает точное время поиска.\n'
    for a,b in [('равна0.9','равна 0.9'),('выборка24','выборка из 24'),(';17решаемых','; 17 решаемых'),('7недостижимых','7 недостижимых'),('3повтора','3 повтора'),('24эталонаh=0','24 эталона h=0'),('10прогревочных','10 прогревочных'),('goal headings0,3,8,13 на24разных','ориентаций цели 0,3,8,13 на 24 разных'),('всехрежимах','всех режимах'),('первоначальныйHEAP','первоначальный HEAP')]:text=text.replace(a,b)
    (OUT/'REPORT.md').write_text(text)
    png=base64.b64encode((OUT/'results.png').read_bytes()).decode()
    (OUT/'report.html').write_text('<!doctype html><html lang="ru"><meta charset="utf-8"><title>Выбор alpha</title><style>body{font:16px/1.6 system-ui;max-width:1200px;margin:30px auto;padding:24px}img{width:100%}pre{white-space:pre-wrap;font:15px/1.6 system-ui}</style><h1>Проверка распределения стоимости</h1><img alt="Время в зависимости от alpha" src="data:image/png;base64,'+png+'"><pre>'+html.escape(text)+'</pre></html>')
    files=list((ROOT/'src').glob('*.cpp'))+list((ROOT/'include').glob('*.hpp'))+[ROOT/'tools/alpha_experiment.cpp',ROOT/'tools/alpha_bounds.cpp',ROOT/'tools/summarize_alpha.py',ROOT/'tests/additive_tests.cpp',ROOT/'obj/alpha_experiment']
    (OUT/'metadata.json').write_text(json.dumps({'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'input_sha256':manifest['inputs_sha256']},indent=2))
    print(text)
if __name__=='__main__':main()
