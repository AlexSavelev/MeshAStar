#!/usr/bin/env python3
"""Validate completeness and report frozen multimap benchmarks, including startup."""
import base64
from collections import defaultdict
import hashlib
import html
import json
from pathlib import Path
import platform
import statistics as stats
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'res/multimap-experiment'

def read_jsonl(name):
    return [json.loads(s) for s in (OUT/name).read_text().splitlines()]

def table(headers, rows):
    return '<table><thead><tr>'+''.join('<th>'+html.escape(str(v))+'</th>' for v in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table>'

def main():
    manifest = json.loads((OUT/'manifest.json').read_text())
    tasks = {t['id']:t for t in manifest['tasks']}
    n, repeats = len(tasks), manifest['repeats']
    raw, batches, prep, refs = [read_jsonl(f'{name}.jsonl') for name in ['runs','batches','preparation','references']]
    if '--diagnostic-input-only' in sys.argv:
        records={r['task']:r for r in raw if r['algorithm']=='LBA' and r['variant']=='max' and r['repeat']==0}
        assert len(records)==n
        (OUT/'diagnostic-tasks.tsv').write_text(''.join(' '.join(map(str,[t['id'],t['map'],*t['start'],*t['goal'],records[t['id']]['steps'],int(records[t['id']]['solved'])]))+'\n' for t in tasks.values()))
        print('Prepared diagnostic inputs from baseline repeat 0')
        return
    assert len(raw) == n*10*repeats and len(batches)==10*repeats
    assert len(prep)==2*2*4*repeats and len(refs)==n
    assert len({r['task'] for r in refs})==n
    refs = {r['task']:r for r in refs}
    for path, digest in manifest['inputs_sha256'].items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest
    groups = defaultdict(list)
    for r in raw:
        assert r['task'] in tasks and r['solved']==refs[r['task']]['solved']
        assert abs(r['cost']-refs[r['task']]['cost'])<.001
        groups[r['task'], r['algorithm'], r['variant'], r['phase']].append(r)
    assert len(groups)==n*10
    summary = []
    metrics = ['end_to_end_s','search_s','preprocessing_s','kinematic_s','grid_s']
    for (task,algo,variant,phase), records in sorted(groups.items()):
        assert {r['repeat'] for r in records}==set(range(repeats)) and len(records)==repeats
        assert len({r['steps'] for r in records})==1
        item = dict(task=task, algorithm=algo, variant=variant, phase=phase,
                    solved=records[0]['solved'], steps=records[0]['steps'])
        for metric in metrics:
            values = [r[metric] for r in records]
            item[metric]=stats.median(values)
            item[metric+'_min']=min(values)
            item[metric+'_max']=max(values)
        summary.append(item)
    index = {(r['task'],r['algorithm'],r['variant'],r['phase']):r for r in summary}
    for task in tasks:
        for algo in ['LBA','Mesh']:
            for variant in ['kin64','kin256']:
                assert index[task,algo,variant,'cached']['steps']==index[task,algo,variant,'cold']['steps']
    for batch in batches:
        records = [r for r in raw if all(r[k]==batch[k] for k in ['algorithm','variant','phase','repeat'])]
        assert len(records)==n and {r['task'] for r in records}==set(tasks)
        assert {r['position'] for r in records}==set(range(n))
        assert abs(sum(r['end_to_end_s'] for r in records)-batch['solve_s'])<1e-7
        assert abs(batch['build_s']+batch['solve_s']-batch['compute_s'])<1e-7
        reference_order = sorted([r for r in raw if r['repeat']==batch['repeat'] and r['algorithm']=='LBA' and r['variant']=='max'],key=lambda r:r['position'])
        assert [r['task'] for r in sorted(records,key=lambda r:r['position'])]==[r['task'] for r in reference_order]
        batch['solved_compute_s']=batch['build_s']+sum(r['end_to_end_s'] for r in records if r['solved'])
        batch['unsolved_s']=sum(r['end_to_end_s'] for r in records if not r['solved'])
    batch_groups=defaultdict(list)
    for b in batches: batch_groups[b['algorithm'],b['variant'],b['phase']].append(b)
    totals=[]
    for (algo,variant,phase), records in sorted(batch_groups.items()):
        assert len(records)==repeats
        item=dict(algorithm=algo,variant=variant,phase=phase)
        for metric in ['compute_s','solve_s','build_s','solved_compute_s','unsolved_s']:
            values=[r[metric] for r in records]
            item[metric]=stats.median(values)
            item[metric+'_min']=min(values);item[metric+'_max']=max(values)
        paired=[]
        for record in records:
            baseline=next(b for b in batches if b['algorithm']==algo and b['variant']=='max' and b['repeat']==record['repeat'])
            paired.append(baseline['compute_s']/record['compute_s'])
        item['paired_speedup_median']=stats.median(paired)
        item['paired_speedup_min']=min(paired);item['paired_speedup_max']=max(paired)
        paired_solved=[]
        for record in records:
            baseline=next(b for b in batches if b['algorithm']==algo and b['variant']=='max' and b['repeat']==record['repeat'])
            paired_solved.append(baseline['solved_compute_s']/record['solved_compute_s'])
        item['solved_paired_speedup_median']=stats.median(paired_solved)
        totals.append(item)

    solved=[task for task in tasks if refs[task]['solved']]
    query_results=[]
    for algo in ['LBA','Mesh']:
        for variant in ['kin64','kin256']:
            for phase in ['cached','cold']:
                ratios=[index[t,algo,'max','baseline']['end_to_end_s']/index[t,algo,variant,phase]['end_to_end_s'] for t in solved]
                steps=[index[t,algo,'max','baseline']['steps']/index[t,algo,variant,phase]['steps'] for t in solved]
                query_results.append(dict(algorithm=algo,variant=variant,phase=phase,
                    solved_tasks=len(solved),faster=sum(x>1 for x in ratios),
                    median_speedup=stats.median(ratios),min_speedup=min(ratios),max_speedup=max(ratios),
                    median_step_reduction=stats.median(steps)))
    error=max(abs(r['cost']-refs[r['task']]['cost']) for r in raw)
    source_files = list((ROOT/'src').glob('*.cpp'))+list((ROOT/'include').glob('*.hpp'))
    source_files += [ROOT/'Makefile',ROOT/'tools/prepare_multimap.py',ROOT/'tools/multimap_experiment.cpp',ROOT/'tools/diagnose_multimap.cpp',ROOT/'tools/summarize_multimap.py',ROOT/'data/base_control_set.txt',ROOT/'data/base_mesh_info.txt',ROOT/'obj/multimap_experiment']
    metadata=dict(platform=platform.platform(),machine=platform.machine(),compiler=subprocess.check_output(['g++','--version'],text=True),
                  source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},
                  input_sha256=manifest['inputs_sha256'],max_reference_cost_difference=error,
                  measured_searches=len(raw),reference_searches=len(refs),solved_tasks=len(solved),unsolved_tasks=n-len(solved))
    (OUT/'summary.json').write_text(json.dumps(dict(tasks=summary,batches=totals,solved_query_comparisons=query_results),indent=2)+'\n')
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')

    variants=[('max','baseline'),('kin64','cold'),('kin64','cached'),('kin256','cold'),('kin256','cached')]
    labels=['Прежняя\nmax','P64\nзаново','P64\nпереисп.','P256\nзаново','P256\nпереисп.']
    colours=['#657383','#e69b36','#e69b36','#14859a','#14859a']
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for row,algo in enumerate(['LBA','Mesh']):
        for col,(metric,caption) in enumerate([('compute_s',f'все {n} запросов'),('solved_compute_s',f'{len(solved)} решаемых запросов')]):
            ax=axes[row,col]
            records=[next(r for r in totals if (r['algorithm'],r['variant'],r['phase'])==(algo,v,p)) for v,p in variants]
            vals=[r[metric] for r in records]
            ax.bar(range(5),vals,color=colours)
            ax.errorbar(range(5),vals,yerr=[[r[metric]-r[metric+'_min'] for r in records],[r[metric+'_max']-r[metric] for r in records]],fmt='none',ecolor='#252525',capsize=4)
            for x,r in enumerate(records):
                ax.text(x,r[metric+'_max']+max(vals)*.035,f"{r[metric]:.2f}",ha='center',fontsize=10)
            ax.set_ylim(0,max(r[metric+'_max'] for r in records)*1.18)
            ax.set_xticks(range(5),labels);ax.set_ylabel('Суммарное время, с');ax.set_title(algo+'*: '+caption,fontweight='bold')
            ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Три карты: стоимость подготовки включена\nМедиана трёх повторов; усы — минимум и максимум. Справа — подмножество тех же серий.\nПереисп.: четыре таблицы, по одной на ориентацию цели.',fontsize=12)
    fig.savefig(OUT/'multimap-results.png',dpi=170);plt.close(fig)

    batch_rows=[]
    for algo in ['LBA','Mesh']:
        for variant,phase in variants:
            r=next(r for r in totals if (r['algorithm'],r['variant'],r['phase'])==(algo,variant,phase))
            batch_rows.append([algo,variant,{'baseline':'база','cold':'заново на каждый запрос','cached':'4 таблицы на серию'}[phase],f"{r['compute_s']:.3f}",f"{r['paired_speedup_median']:.2f}",f"{r['solved_compute_s']:.3f}",f"{r['solved_paired_speedup_median']:.2f}"])
    headers=['Алгоритм','Эвристика','Подготовка','Все 24, с','Ускорение всех, ×',f'{len(solved)} решаемых, с','Ускорение решаемых, ×']
    query_rows=[[r['algorithm'],r['variant'],r['phase'],f"{r['faster']}/{r['solved_tasks']}",f"{r['median_speedup']:.2f}",f"{r['min_speedup']:.2f}–{r['max_speedup']:.2f}",f"{r['median_step_reduction']:.2f}"] for r in query_results]
    query_headers=['Алгоритм','Эвристика','Режим','Быстрее, задач','Медиана ускорения, ×','Диапазон, ×','Медиана сокращения steps, ×']
    text = f'''# Проверка кинематической эвристики на трёх картах

До первых измерений зафиксирован набор из {n} задач. Карты Berlin_2_256, Milan_1_256,
AR0700SR; по 8 задач, случайно выбранных из ранговых диапазонов 25–50% и
65–90% по длине пути в исходном grid-сценарии. Seed {manifest['seed']}.
Сценарные координаты (x,y) преобразованы в (строка y, столбец x).
Ориентации добавлены отдельно: цели 0,3,8,13; старты 0,5,8,14 по фиксированной схеме.
Сценарная grid-длина используется только для выбора задач; эталон стоимости — LBA* с h=0.

База max = max(0.9933*d_E, 0.9*d_grid). kin64/kin256 добавляют максимум
с периодической кинематической оценкой. Вес A* равен 1; цель — точная клетка и ориентация.

Проверены {len(raw)} измеряемых поисков (3 повтора), {len(refs)} эталонных и 6 прогревочных.
Решаемы {len(solved)}/{n} задач; недостижимые не заменялись.
Все найденные пути проверены по примитивам, ориентациям, коллизиям, конечному
состоянию и сумме стоимостей. Максимальное отличие стоимости от h=0: {error:.8g}.
Допуск 0.001 связан с оставленной float-арифметикой g/f; это не доказательство
точной оптимальности всей реализации. Переиспользование не изменило steps.

## Что измеряется

Время одного запроса — end_to_end_time внутри solve(): подготовка эвристики,
поиск, восстановление пути, создание/удаление поискового контекста.
Загрузка карт/примитивов/mesh, первоначальное создание HEAP, вывод и проверка
пути исключены. «Холодный» означает построение кинематической таблицы заново,
а не новый процесс или сброс аппаратных кэшей.

Серия — 24 разных запроса; три повтора. В каждом повторе порядок запросов
одинаков для вариантов, но перемешан между повторами; порядок вариантов также
перемешан. Последовательные запуски, без параллельных планировщиков.
cached действительно строит четыре таблицы один раз на серию, затем использует
их на разных картах и координатах цели. Время серии = сумма времён solve()
плюс измеренное построение этих четырёх таблиц. Это медиана трёх полных сумм,
не сумма медиан отдельных запросов. Ускорение серии — медиана трёх отношений
к базовой серии того же повтора. Grid-поле строится заново на каждый запрос.
cached использует 2 МиБ (P64) или 32 МиБ (P256) массивов расстояний для четырёх
ориентаций; память очереди при построении и память самого поиска сюда не входят.
Удаление внешнего кэша в время серии не включено. Постоянного кэша на диске нет.

## Полные серии и подмножество решаемых задач

Столбцы решаемых задач получены суммированием соответствующих запросов из тех
же серий, без повторного запуска и без замены задач. Все четыре построения
таблиц полностью включены и в это подмножество. Набор и порядок замеров
не менялись по результатам проверки решаемости.

'''
    text += '| '+' | '.join(headers)+' |\n|'+'|'.join(['---']*len(headers))+'|\n'
    text += ''.join('| '+' | '.join(row)+' |\n' for row in batch_rows)
    text += '\n## Отдельные решаемые задачи\n\nВ cached подготовка таблиц исключена из времени отдельного запроса; она учтена выше в серии. Ускорение >1 означает выигрыш. Разница около единицы может быть шумом.\n\n'
    text += '| '+' | '.join(query_headers)+' |\n|'+'|'.join(['---']*len(query_headers))+'|\n'
    text += ''.join('| '+' | '.join(row)+' |\n' for row in query_rows)
    diagnosis_text=''
    if (OUT/'diagnostics.jsonl').exists():
        diagnostics=read_jsonl('diagnostics.jsonl');bounds=read_jsonl('table-bounds.jsonl')
        assert len(diagnostics)==n*2 and len(bounds)==8
        for r in diagnostics:
            assert r['steps']==index[r['task'],'LBA','max','baseline']['steps']
        diagnosis_text='\n## Почему выигрыш ограничен\n\nПосле завершения замеров отдельно запущен LBA* с прежней max-эвристикой. На каждом раскрытом состоянии вычислены обе кинематические оценки; поиск использует только прежнюю max, диагностические времена не входят в benchmark. Число steps и решаемость совпали с базой для всех задач.\n\n'
        for period in [64,256]:
            records=[r for r in diagnostics if r['period']==period and r['solved']]
            fractions=[r['improved_fraction'] for r in records]
            weighted=sum(r['improved_states'] for r in records)/sum(r['expanded_states'] for r in records)
            radii=[r['euclidean_radius'] for r in bounds if r['period']==period]
            diagnosis_text+=f'- P={period}: медианная доля раскрытых состояний, где оценка сильнее, {stats.median(fractions)*100:.2f}%; диапазон {min(fractions)*100:.2f}–{max(fractions)*100:.2f}%; доля по объединению раскрытий {weighted*100:.2f}%. Гарантированный радиус возможного усиления: не более {max(radii):.2f} клетки для проверенных ориентаций цели.\n'
        diagnosis_text+='\nРадиус получен полным перебором таблицы: M=max h_kin. При d_E >= M/0.9933 уже h_E >= M >= h_kin, поэтому кинематическая таблица не усиливает max. Внутри радиуса улучшение тоже не гарантировано: grid-оценка может быть сильнее. Это граница полезности данного конечного периодического представления, а не ограничение реального движения робота. Диагностика раскрытий выполнена для LBA*, не для внутренних Mesh-состояний.\n'
        text+=diagnosis_text
    text += '\n## Ограничения и воспроизведение\n\n24 задачи и 3 повтора — разведочный эксперимент, не репрезентативная оценка всех карт. Нельзя переносить выигрыш на новые карты без проверки. Ранги grid-длины не определяют кинематическую сложность.\n\nНедостижимые задачи:\n\n'
    text += ''.join('- '+task+'\n' for task in tasks if not refs[task]['solved'])
    text += '\nФайлы manifest.json и tasks.tsv фиксируют выбор; references.jsonl — эталоны; runs.jsonl — все поиски; batches.jsonl — серии; preparation.jsonl — построения; summary.json и metadata.json — сводка и хеши.\n\nИз корня репозитория:\n\n```sh\nmake multimap-experiment\n# Для повторного запуска: новая папка с копией неизменённого tasks.tsv\n./obj/multimap_experiment NEW_OUTPUT_DIRECTORY 3\nMPLCONFIGDIR=/tmp/mesha-mpl-cache XDG_CACHE_HOME=/tmp/mesha-font-cache python3 tools/summarize_multimap.py\n```\n\nОбработчик по умолчанию читает res/multimap-experiment. Генератор и runner не перезаписывают существующие выборку и результаты.\n'
    (OUT/'REPORT.md').write_text(text)
    details=[]
    for task,t in tasks.items():
        for algo in ['LBA','Mesh']:
            b=index[task,algo,'max','baseline']
            k1=index[task,algo,'kin64','cached'];k2=index[task,algo,'kin256','cached']
            details.append([task,algo,'да' if b['solved'] else 'нет',str(t['start']),str(t['goal']),f"{b['end_to_end_s']*1000:.1f}",f"{k1['end_to_end_s']*1000:.1f}",f"{k2['end_to_end_s']*1000:.1f}",b['steps'],k1['steps'],k2['steps']])
    png=base64.b64encode((OUT/'multimap-results.png').read_bytes()).decode()
    page='''<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MeshA*: проверка на трёх картах</title><style>body{font:16px/1.55 system-ui,sans-serif;max-width:1200px;margin:40px auto;padding:0 24px;color:#172b38;background:#f7fafc}h1,h2{line-height:1.2}table{border-collapse:collapse;width:100%;font-size:13px;background:white}td,th{padding:9px;border-bottom:1px solid #dce3e9;text-align:left}th{background:#e7eef3}img{width:100%;height:auto}pre{white-space:pre-wrap;background:white;padding:24px;border-radius:8px;font:14px/1.65 system-ui}.scroll{overflow:auto}</style><h1>Кинематическая эвристика: три карты</h1>'''
    page+=f'<p>{n} зафиксированных задач, {len(solved)} решаемых, {len(raw)} измерений. Подготовка включена в время серии. Ускорение больше 1 — выигрыш.</p><img alt="Время серии с подготовкой" src="data:image/png;base64,{png}">'
    page+='<h2>Полная серия</h2><div class="scroll">'+table(headers,batch_rows)+'</div><h2>Решаемые задачи по отдельности</h2><div class="scroll">'+table(query_headers,query_rows)+'</div>'
    page+='<h2>Все задачи: медианы запросов с готовыми таблицами</h2><div class="scroll">'+table(['Задача','Алгоритм','Есть путь','Старт (i,j,θ)','Цель (i,j,θ)','max, мс','P64, мс','P256, мс','max steps','P64 steps','P256 steps'],details)+'</div>'
    page+='<h2>Полный протокол</h2><pre>'+html.escape(text)+'</pre></html>'
    (OUT/'report.html').write_text(page)
    print(json.dumps(dict(solved=len(solved),total=n,max_cost_error=error,batches=totals,queries=query_results),indent=2))
    print(diagnosis_text)

if __name__=='__main__':
    main()
