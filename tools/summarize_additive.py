#!/usr/bin/env python3
"""Analyze fixed-alpha cost-partition experiment; all timings from this run."""
from pathlib import Path
from collections import defaultdict
import json, statistics, hashlib, base64, html
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'res/additive-experiment'
def read(name):return [json.loads(x) for x in (OUT/name).read_text().splitlines()]
def main():
    runs,batches,refs,prep=[read(n+'.jsonl') for n in ['runs','batches','references','preparation']]
    manifest=json.loads((OUT/'manifest.json').read_text());tasks={t['id']:t for t in manifest['tasks']}
    assert len(runs)==720 and len(batches)==30 and len(refs)==24 and len(prep)==48
    assert (OUT/'tasks.tsv').read_bytes()==(ROOT/'res/multimap-experiment/tasks.tsv').read_bytes()
    for file,digest in manifest['inputs_sha256'].items():assert hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==digest
    reference={r['task']:r for r in refs};groups=defaultdict(list)
    for r in runs:
        assert r['solved']==reference[r['task']]['solved']
        assert abs(r['cost']-reference[r['task']]['cost'])<.001
        groups[r['task'],r['algorithm'],r['variant'],r['phase']].append(r)
    summary=[]
    for key,rs in groups.items():
        assert len(rs)==3 and {r['repeat'] for r in rs}=={0,1,2} and len({r['steps'] for r in rs})==1
        summary.append(dict(zip(['task','algorithm','variant','phase'],key),steps=rs[0]['steps'],solved=rs[0]['solved'],
                            time_s=statistics.median(r['end_to_end_s'] for r in rs),search_s=statistics.median(r['search_s'] for r in rs)))
    idx={(r['task'],r['algorithm'],r['variant'],r['phase']):r for r in summary}
    old=json.loads((ROOT/'res/multimap-experiment/summary.json').read_text())['tasks']
    for r in old:
        if r['variant'] in ['max','kin64']:assert idx[r['task'],r['algorithm'],r['variant'],r['phase']]['steps']==r['steps']
    for task in tasks:
        for algo in ['LBA','Mesh']:
            for v in ['kin64','add64']:assert idx[task,algo,v,'cold']['steps']==idx[task,algo,v,'cached']['steps']
    for b in batches:
        rs=[r for r in runs if all(r[k]==b[k] for k in ['algorithm','variant','phase','repeat'])]
        assert len(rs)==24 and {r['task'] for r in rs}==set(tasks)
        assert abs(sum(r['end_to_end_s'] for r in rs)+b['build_s']-b['compute_s'])<1e-7
        b['solved_compute_s']=b['build_s']+sum(r['end_to_end_s'] for r in rs if r['solved'])
        order=sorted(rs,key=lambda r:r['position'])
        base=sorted([r for r in runs if r['repeat']==b['repeat'] and r['algorithm']=='LBA' and r['variant']=='max'],key=lambda r:r['position'])
        assert [r['task'] for r in order]==[r['task'] for r in base]
    variants=[('max','baseline'),('kin64','cold'),('kin64','cached'),('add64','cold'),('add64','cached')]
    totals=[];query=[]
    for algo in ['LBA','Mesh']:
        for v,phase in variants:
            rs=[b for b in batches if (b['algorithm'],b['variant'],b['phase'])==(algo,v,phase)]
            item=dict(algorithm=algo,variant=v,phase=phase)
            for field in ['compute_s','solved_compute_s','build_s']:
                values=[r[field] for r in rs];item[field]=statistics.median(values);item[field+'_min']=min(values);item[field+'_max']=max(values)
            totals.append(item)
            if v=='max':continue
            solved=[t for t in tasks if reference[t]['solved']]
            ratios=[idx[t,algo,'max','baseline']['time_s']/idx[t,algo,v,phase]['time_s'] for t in solved]
            step_ratios=[idx[t,algo,'max','baseline']['steps']/idx[t,algo,v,phase]['steps'] for t in solved]
            query.append(dict(algorithm=algo,variant=v,phase=phase,median_speedup=statistics.median(ratios),
                faster=sum(r>1 for r in ratios),tasks=len(solved),min_speedup=min(ratios),max_speedup=max(ratios),median_step_ratio=statistics.median(step_ratios)))
    error=max(abs(r['cost']-reference[r['task']]['cost']) for r in runs)
    (OUT/'summary.json').write_text(json.dumps(dict(tasks=summary,batches=totals,queries=query,max_cost_error=error),indent=2)+'\n')
    files=list((ROOT/'src').glob('*.cpp'))+list((ROOT/'include').glob('*.hpp'))+[ROOT/'tools/additive_experiment.cpp',ROOT/'tools/diagnose_additive.cpp',ROOT/'tools/summarize_additive.py',ROOT/'tests/additive_tests.cpp',ROOT/'obj/additive_experiment',ROOT/'Makefile']
    (OUT/'metadata.json').write_text(json.dumps({'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'input_sha256':manifest['inputs_sha256']},indent=2))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    labels=['Прежняя\nmax','kin64\nзаново','kin64\nпереисп.','add64\nзаново','add64\nпереисп.']
    for row,algo in enumerate(['LBA','Mesh']):
        for col,field in enumerate(['compute_s','solved_compute_s']):
            rs=[next(r for r in totals if (r['algorithm'],r['variant'],r['phase'])==(algo,v,p)) for v,p in variants]
            ax=axes[row,col];values=[r[field] for r in rs]
            ax.bar(range(5),values,color=['#657383','#e69b36','#e69b36','#14859a','#14859a'])
            ax.errorbar(range(5),values,yerr=[[r[field]-r[field+'_min'] for r in rs],[r[field+'_max']-r[field] for r in rs]],fmt='none',ecolor='#333',capsize=3)
            for k,r in enumerate(rs):ax.text(k,r[field+'_max']+max(values)*.025,f'{r[field]:.2f}',ha='center')
            ax.set_ylim(0,max(r[field+'_max'] for r in rs)*1.16);ax.set_xticks(range(5),labels)
            ax.set_title(algo+'*: '+('все 24 задачи' if col==0 else '17 решаемых задач'));ax.set_ylabel('Суммарное время, с');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Разделение стоимости: P=64, α=0.9, подготовка включена\nМедианы трёх повторов; усы — минимум и максимум. Справа — подмножество тех же серий.',fontsize=12)
    fig.savefig(OUT/'results.png',dpi=160);plt.close(fig)
    text=f'''# Аддитивная эвристика с разделением стоимости

Фиксированные P=64, alpha=0.9. Та же выборка24 задач и порядокизseed20261026;
17 решаемы, 7 недостижимы. Без настройки параметров по результатам этой серии.
720 измеряемых поисков, 24 эталона LBA с h=0, 6 прогревочных; три повтора.
Все варианты замерены заново. Старая база — пространственная max, не исходная
евклидова эвристика репозитория. Стоимость пути отличается от h=0 не более {error:.8g}.
Кэширование и повторы не меняют steps; steps старых max/kin64 совпали с прошлым
benchmark. Путь проверяется по примитивам, коллизиям, ориентациям, цели и стоимости.

## Конструкция

D_p — кратчайший 8-связный путь в множестве клеток следа примитива p.
r(p)=c(p)-alpha*D_p >= 0. Стоимости c — float, используемые планировщиком;
остатки и таблицы вычисляются в double. Отрицательные остатки отклоняются.
Кинематическая таблица на периодической решётке строится со стоимостями r.
add64 = max(beta*d_E, alpha*d_grid+h_res64), beta=0.9933.
kin64 = max(beta*d_E, alpha*d_grid+h_dummy) здесь НЕ используется:
старый kin64 = max(beta*d_E, alpha*d_grid, h_full64).
Новая оценка доминирует пространственную max. Доминирование старой kin64
не утверждается: у неё другой бюджет стоимости. Два pose-поля одновременно
в новом варианте не хранятся.

Для каждого допустимого перехода p:s->t разность grid-оценок <= alpha*D_p,
разность остаточных оценок <= r(p). Их сумма <= c(p), в цели сумма нулевая.
Максимум с согласованной безопасной евклидовой оценкой также согласован.
Это математическая гарантия; float g/f исходного планировщика оставлены.
Точная целевая клетка и ориентация обязательны; допуски R>=1 или A!=0 отклоняются.
Кэш проверяется по heading, периоду и полному набору переходов с эффективными
стоимостями; изменение alpha или следов, меняющее остатки, инвалидирует кэш.

## Полное время

Время серии = сумма end_to_end solve + подготовка внешнего кэша. В cached
строятся4 таблицы для ориентаций0,3,8,13; подготовка residual-стоимостей тоже
включена. Массивы расстояний занимают2МиБ; очередь при построении отдельно.
В cold таблица строится каждый запрос. Grid-поле строится каждый запрос во
всех режимах. Загрузка входных файлов, первоначальный HEAP, проверки/логирование
и удаление внешнего кэша исключены. Cold не означает сброс аппаратного кэша.
Справа решаемое подмножество тех же запусков; вся подготовка4таблиц включена.

| Алгоритм | Вариант | Режим | Все24, с | Решаемые17, с |
|---|---|---|---:|---:|
'''
    for r in totals:text+=f"| {r['algorithm']} | {r['variant']} | {r['phase']} | {r['compute_s']:.3f} | {r['solved_compute_s']:.3f} |\n"
    text+='\n## Отдельные решаемые задачи\n\nCached здесь исключает подготовку таблиц из времени одного запроса; она учтена выше. Ускорение >1 — выигрыш. Малые отличия около1 могут быть шумом.\n\n| Алгоритм | Вариант | Режим | Медиана ускорения | Быстрее | Медиана отношения steps базы/варианта |\n|---|---|---|---:|---:|---:|\n'
    for r in query:text+=f"| {r['algorithm']} | {r['variant']} | {r['phase']} | {r['median_speedup']:.3f} | {r['faster']}/17 | {r['median_step_ratio']:.3f} |\n"
    if (OUT/'diagnostics.jsonl').exists():
        diagnostics=read('diagnostics.jsonl');assert len(diagnostics)==24
        solved=[d for d in diagnostics if d['solved']];assert len(solved)==17
        text+='\n## Сила оценки на раскрытиях прежнего LBA*\n\nДиагностика выполнена после всех измерений времени, на тех же базовых раскрытиях.\n\n'
        for name,key in [('kin64','full'),('add64','add')]:
            fractions=[d['stronger_'+key]/d['expanded'] for d in solved]
            mean=sum(d['delta_'+key+'_sum'] for d in solved)/sum(d['expanded'] for d in solved)
            text+=f'- {name}: медианная доля усиленных состояний {statistics.median(fractions)*100:.2f}%; средняя прибавка по объединению раскрытий {mean:.4f}.\n'
        for task in ['Berlin_2_256-b0-h3','AR0700SR-b1-h0']:
            d=next(d for d in solved if d['task']==task)
            text+=f"- {task}: kin64 усиливает {d['stronger_full']/d['expanded']*100:.2f}% состояний, add64 — {d['stronger_add']/d['expanded']*100:.2f}%; средние прибавки {d['delta_full_sum']/d['expanded']:.4f} и {d['delta_add_sum']/d['expanded']:.4f}.\n"
    text+='\n## Воспроизведение\n\nmake test obj/additive_experiment\n\n./obj/additive_experiment OUTPUT_FOLDER 3\n\nВ OUTPUT_FOLDER должна лежать копия зафиксированного tasks.tsv; runner не перезаписывает существующие результаты. Обработчик tools/summarize_additive.py читает res/additive-experiment.\n\nCLI: --heuristic additive --alpha 0.9 --kin-period 64. В CLI таблица строится отдельно для каждого алгоритма.\n\n24 задачи и3 повтора — ограниченный эксперимент. Дополнительной выборки для проверки обобщения пока нет.\n'
    text=text.replace('kin64 = max(beta*d_E, alpha*d_grid+h_dummy) здесь НЕ используется:\nстарый kin64', 'Старый kin64')
    for a,b in [('выборка24','выборка из 24'),('порядокизseed20261026','порядок с seed 20261026'),('строятся4','строятся 4'),('ориентаций0,3,8,13','ориентаций 0, 3, 8, 13'),('занимают2МиБ','занимают 2 МиБ'),('подготовка4таблиц','подготовка 4 таблиц'),('и3 повтора','и 3 повтора')]:text=text.replace(a,b)
    (OUT/'REPORT.md').write_text(text)
    png=base64.b64encode((OUT/'results.png').read_bytes()).decode()
    (OUT/'report.html').write_text('<!doctype html><html lang="ru"><meta charset="utf-8"><title>Аддитивная эвристика</title><style>body{font:16px/1.6 system-ui;max-width:1200px;margin:30px auto;padding:24px}img{width:100%}pre{white-space:pre-wrap;font:15px/1.6 system-ui}</style><h1>Разделение стоимости: результаты</h1><img alt="Сравнение времени" src="data:image/png;base64,'+png+'"><pre>'+html.escape(text)+'</pre></html>')
    print(json.dumps(dict(totals=totals,queries=query,max_cost_error=error),indent=2))
if __name__=='__main__':main()
