#!/usr/bin/env python3
"""Freeze benchmark inputs without running planners or inspecting performance.

Run from any directory. --verify checks the frozen inputs without regenerating.
"""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'res/final-benchmark-v1'
SEED = 2026102601
MAPS = [
    ('Labyrinth', 'paper'), ('ht_0_hightown', 'paper'),
    ('Moscow_0_512', 'paper'), ('AR0304SR', 'paper'),
    ('Berlin_2_256', 'extension'), ('Milan_1_256', 'extension'),
    ('AR0700SR', 'extension'), ('WheelofWar', 'extension'),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path):
    return str(path.relative_to(ROOT))


def geometry(mp, start, goal):
    return (mp, *sorted((tuple(start[:2]), tuple(goal[:2]))))


def read_map(path):
    lines = path.read_text().splitlines()
    assert lines[0] == 'type octile' and lines[3] == 'map', path
    h, w = int(lines[1].split()[1]), int(lines[2].split()[1])
    cells = lines[4:]
    assert len(cells) == h and all(len(row) == w for row in cells), path
    return w, h, cells


def free(cells, i, j):
    return 0 <= i < len(cells) and 0 <= j < len(cells[0]) and cells[i][j] in '.GS'


def previous_geometries():
    known, sources = set(), {}
    for path in sorted((ROOT / 'res').glob('*/tasks.tsv')):
        if path.parent == OUT:
            continue
        sources[rel(path)] = digest(path)
        for row in path.read_text().splitlines():
            v = row.split()
            if len(v) != 8 or not v[1].startswith('maps/'):
                continue
            try:
                nums = list(map(int, v[2:]))
            except ValueError:
                continue
            known.add(geometry(v[1], nums[:3], nums[3:]))
    # Original manually studied Moscow queries, not necessarily in tasks.tsv.
    for s, g in [([120,50,0], [300,180,4]), ([488,70,3], [26,507,15])]:
        known.add(geometry('maps/Moscow_0_512.map', s, g))
    return known, sources


def write_tasks(path, tasks):
    path.write_text(''.join(' '.join(map(str, [t['id'], t['map'], *t['start'], *t['goal']]))+'\n' for t in tasks))


def edge_cases():
    folder = OUT / 'edge-maps'
    folder.mkdir()
    grids = {'empty': ['.'*64 for _ in range(64)]}
    grids['corridor'] = ['.'*64 if i == 32 else '@'*64 for i in range(64)]
    for name, half in [('gap1', 0), ('gap3', 1), ('disconnected', -1)]:
        grids[name] = ['.'*32 + ('.' if half >= 0 and abs(i-40) <= half else '@') + '.'*31 for i in range(64)]
    for name, rows in grids.items():
        (folder / (name+'.map')).write_text('type octile\nheight 64\nwidth 64\nmap\n'+'\n'.join(rows)+'\n')
    tasks = []
    def add(family, k, mp, s, g, expected=None):
        tasks.append(dict(id=f'edge-{family}-{k:02d}', suite='edge', family=family,
                          map=rel(folder/(mp+'.map')), start=s, goal=g,
                          expected=expected))
    shifts = [(0,12),(12,12),(12,0),(12,-12),(0,-12),(-12,-12),(-12,0),(-12,12)]
    boundaries = [([0,8,0],[0,55,0]),([63,8,0],[63,55,0]),
                  ([8,0,4],[55,0,4]),([8,63,4],[55,63,4]),
                  ([0,55,8],[0,8,8]),([63,55,8],[63,8,8]),
                  ([55,0,12],[8,0,12]),([55,63,12],[8,63,12])]
    for k in range(8):
        theta = 2*k
        add('identical-pose', k, 'empty', [32,32,theta], [32,32,theta], 'solved_zero_cost')
        add('same-cell-other-heading', k, 'empty', [32,32,theta], [32,32,(theta+8)%16])
        di,dj = shifts[k]
        add('empty-short', k, 'empty', [32,32,theta], [32+di,32+dj,theta])
        add('boundary', k, 'empty', *boundaries[k])
        add('one-cell-corridor', k, 'corridor', [32,8,theta], [32,55,0])
        for name in ['gap1','gap3','disconnected']:
            add(name, k, name, [32,8,theta], [32,55,0], 'unreachable' if name=='disconnected' else None)
    return tasks


PROTOCOL = '''# Зафиксированный большой тест эвристик · v1

Состояние: подготовлены входные задачи; новые поиски и измерения ещё не запускались.

## Объём и назначение

960 запросов: 8 карт по 120 уникальных геометрических пар. Основные карты статьи: Labyrinth, ht_0_hightown, Moscow_0_512, AR0304SR. Дополнение: Berlin_2_256, Milan_1_256, AR0700SR, WheelofWar. В статье v2 основных карт четыре, хотя в сообщении ментора упомянуты три.

Отдельно 64 синтетических краевых запроса, по 8 в каждой семье: одинаковая поза; одна клетка с другой ориентацией; короткий путь без препятствий; граница карты; одноклеточный коридор; проход шириной 1; проход шириной 3; полностью разделённая карта. Эти задачи не добавляются к среднему по реальным картам. Для узких проходов не предполагаем заранее кинематическую достижимость.

Четыре карты достаточны для воспроизведения основных условий статьи, но не для вывода обо всех геометриях. Восемь дают более широкую проверку; 960 задач остаются кластеризованными по восьми картам. Не выдаём число запросов за число независимых геометрий. Известный удачный Berlin-пример не добавляем в новую серию как особый случай; его можно показывать отдельно с пометкой «диагностический пример».

## Выбор задач до запуска

Seed: 2026102601. В каждом исходном .scen проверяем размеры карты, границы и свободные концы; исключаем совпадающие клетки, повторные геометрии и геометрические пары прежних экспериментов в обоих направлениях. Ранее разобранные ручные Moscow-запросы также исключены. Полный список исключённых прежних пар и хеши источников сохранены.

Оставшиеся строки сортируем по исходной scenario grid-длине и делим по рангу на три равные части. Из каждой случайно выбираем 40 разных пар. Названия short/middle/long означают длину сценария, а не измеренную трудность планирования. Решаемость и время не используются при выборе; даже длинный запрос может оказаться лёгким, а короткий — трудным.

На карте каждый из 16 конечных heading встречается 7 или 8 раз. В каждой трети 10 запросов с одинаковыми heading, 10 с противоположными (+8 mod 16), 20 с другими. Назначение перемешано и не зависит от результата поиска. При сравнении методов позы и входные файлы строго одинаковы.

Часть карт уже использована в исследовании; это расширенная заранее зафиксированная проверка, а не полностью нетронутый holdout. Восемь карт также не являются независимой ML-выборкой. Новое обучение и подбор параметров на этой серии запрещены без отдельной отметки, что серия стала development.

## Основная матрица измерений

LBA* и MeshA*, w=1, точная конечная поза. Для обоих: safe-euclidean, max, directional. Это 960×2×3×3 = 17 280 измеренных запусков при трёх повторах; краевые проверки идут отдельно. Compact CLOSED одинаков во всех вариантах; best-g выключен. Исходная raw euclidean — отдельная контрольная серия (960×2×3 = 5 760 запусков), её качество проверяется, допустимость не предполагается. Код/компилятор/параметры и hashes исполняемого файла фиксируются перед измерениями, результаты старых серий к новым временам не подмешиваются.

Любое ускорение вычисления directional должно сохранять те же значения эвристики; до большого теста проверить эквивалентность. Если нужна сравнительная серия старой и ускоренной реализации, записать её отдельно до запуска, не менять метод посреди серии. Best-g и ML не входят в основной тест эвристик.

Сначала smoke на всех 64 краевых задачах, затем технический pilot на заранее заданных 3 запросах каждой карты (по одному из каждой трети; первые выбранные в manifest). Pilot проверяет память, формат логов и оценку бюджета. Он не используется для замены задач/карт. Затем полный набор, включая pilot-задачи, запускается заново. Изменение протокола после pilot фиксируется дополнением, исходный manifest не перезаписывается.

Каждый запрос измеряется с новым grid-полем; чтение карты/примитивов вне solve. Время solve включает подготовку h, создание контекста, поиск, восстановление и очистку. Дополнительно сохраняем wall time процесса, загрузку и прогрев, не смешивая их с solve. В каждом повторе порядок запросов и вариантов детерминированно перемешан. Только последовательные поиски, без параллельной нагрузки. Прогревы не включаются в сводку и не переиспользуют grid-поле измеренного запроса.

## Тайм-ауты и корректность

Предварительно фиксируем 30 секунд wall time на отдельный процесс измеренного запроса и 120 секунд на отдельный h=0 эталон. Лимит включает запуск и загрузку; успешный solve измеряется отдельно внутри процесса. Следующий runner должен реализовать изоляцию, очистку дочерних процессов и явные статусы, а не пытаться ограничить старый безлимитный batch-runner. На pilot оцениваем пригодность лимитов; если понадобится изменение, оно применяется ко всей серии с записью причины до основных замеров.

Статусы: solved, exhausted_unreachable, timeout, error. Тайм-аут не равен доказанной недостижимости; ошибка не равна тайм-ауту. Не удаляем и не заменяем задачи по этим статусам. Для h=0 тайм-аут означает reference_unknown, не «всё равно оптимально». Эталон — LBA* с h=0 и best-g (вне измерений основных методов). Проверки найденного пути: все клетки следа, сопряжение heading, конечная поза, сумма c_p. Сравнение со стоимостью эталона: допуск 0.001; при нарушении сохраняем контрпример и расследуем, а не молча расширяем допуск. Исходные float g/f остаются численным ограничением.

Планируем записывать end_to_end, preprocessing, grid, directional, search, setup, cleanup, steps, actual_expansions (если добавлена отдельная инструментализация), candidates/pushes/pops, peak OPEN, стоимость/решаемость и вердикт проверки. steps не называть автоматически числом раскрытий: он включает служебные итерации. Метрики разных графов прежде всего сравнивать внутри каждого алгоритма.

## Кинематические таблицы: отдельный второй этап

P64 и P256 на тех же задачах, если позволяет бюджет. Два явно разных сценария: построение на каждый запрос и повторное использование по целевому heading. В выбранной серии есть все 16 целевых heading: полный кэш P256 занимает 128 МиБ массивов, P64 — 8 МиБ (без очередей). Нельзя переносить старый бюджет четырёх таблиц на этот набор. При кэшировании считать каждое фактическое построение ровно один раз на определённую серию; отдельно показывать серию на каждой карте и общую с межкартовым повторным использованием. Не выдавать время с готовым кэшем за холодный запрос.

## Представление результатов

Сначала четыре карты статьи, затем четыре дополнительные, затем общий итог. На каждой: число solved/exhausted/timeout/error, медиана полного времени, p90, суммарное время завершённых запусков, доля парных выигрышей, парное отношение времён и число шагов. Парные ускорения показываем на пересечении успешно завершившихся вариантов с одинаковой проверенной стоимостью; отдельно приводим размер этого пересечения и результаты остальных задач. Не подставлять лимит как точное время timeout и не объявлять сумму завершённых запусков полной стоимостью серии с тайм-аутами. Три повтора характеризуют шум времени, не заменяют разнообразие карт.

Графики: scatter полного времени база/вариант с диагональю равенства; аналогичный scatter шагов; разложение времени на подготовку/поиск/остальное; сводка по картам и группам длины. Корректность и незавершённые задачи видны рядом с ускорением.

## Метрика обхода — предложение Марата

Для уже существующей синтетики со стеной добавить rho = C*_obstacles(s,G) / C*_free(s,G) рядом со смещением прохода. Одинаковые позы, примитивы, стоимости и границы; в знаменателе убраны только препятствия. Это ось таблицы и объясняющего графика, НЕ правило переключения эвристик. Если C*_free=0 или эталон неизвестен, отношение не определяем. Для недостижимой задачи отдельная категория. Периодическую таблицу не подставлять вместо точного C*_free. Монотонность времени/ускорения не навязываем; монотонную линию тренда не считать доказанным законом. Здесь у всех wall-запросов одинаковые позы, поэтому знаменатель можно переиспользовать из empty-reference.

## Что уже выполнено и что ещё нет

Выполнено: выбор 960 запросов, генерация 64 краевых задач, фиксация входных хешей, проверка размеров/координат/ориентаций/уникальности/квот, создание TSV и pilot-поднабора. Ещё не выполнено: новый runner с лимитами, smoke, h=0 эталоны, pilot и большой benchmark. Никаких новых ускорений этот документ не сообщает.
'''


def generate():
    if OUT.exists():
        raise SystemExit(f'Refusing to overwrite {OUT}; use --verify.')
    known, prior_sources = previous_geometries()
    tasks, inputs, summaries = [], {}, []
    for mi, (name, suite) in enumerate(MAPS):
        rng = random.Random(SEED+mi)
        mp = ROOT/'maps'/(name+'.map')
        sp = Path(str(mp)+'.scen')
        w,h,cells = read_map(mp)
        inputs[rel(mp)], inputs[rel(sp)] = digest(mp), digest(sp)
        excluded, rows, seen = Counter(), [], set()
        for line, text in enumerate(sp.read_text().splitlines()[1:], 2):
            v = text.split()
            assert len(v)==9 and v[1]==mp.name and list(map(int,v[2:4]))==[w,h], (sp,line)
            sx,sy,gx,gy = map(int,v[4:8]); distance = float(v[8])
            key = geometry(rel(mp), [sy,sx], [gy,gx])
            reason = ('invalid_endpoint' if not (free(cells,sy,sx) and free(cells,gy,gx))
                      else 'same_cell' if (sy,sx)==(gy,gx)
                      else 'prior_geometry' if key in known
                      else 'duplicate_geometry' if key in seen else None)
            if reason:
                excluded[reason] += 1
                continue
            assert distance > 0
            seen.add(key); rows.append((distance,line,[sy,sx],[gy,gx]))
        rows.sort()
        goals = list(range(16))*7 + list(range(8))
        goals = [(v+mi)%16 for v in goals]; rng.shuffle(goals)
        band_summary = []
        for bi, band in enumerate(['short','middle','long']):
            low, high = len(rows)*bi//3, len(rows)*(bi+1)//3
            pool = list(enumerate(rows[low:high], low))
            assert len(pool)>=40
            modes = ['same']*10+['opposite']*10+['other']*20; rng.shuffle(modes)
            picked = rng.sample(pool,40)
            for k,(rank,(dist,line,s,g)) in enumerate(picked):
                gt = goals[bi*40+k]
                st = gt if modes[k]=='same' else (gt+8)%16 if modes[k]=='opposite' else (gt+rng.choice([n for n in range(1,16) if n!=8]))%16
                tasks.append(dict(id=f'{name}-{band}-{k:02d}', suite=suite,
                                  map=rel(mp), scenario=rel(sp), scenario_line=line,
                                  band=band, eligible_rank=rank, eligible_count=len(rows),
                                  grid_reference_distance=dist, heading_relation=modes[k],
                                  start=s+[st], goal=g+[gt], pilot=k==0))
            band_summary.append(dict(band=band,eligible=len(pool),selected=40,
                                     pool_distance_min=pool[0][1][0],pool_distance_max=pool[-1][1][0]))
        summaries.append(dict(name=name,suite=suite,width=w,height=h,
                              eligible=len(rows),excluded=dict(excluded),bands=band_summary))
    OUT.mkdir(parents=True)
    edges = edge_cases()
    for p in (OUT/'edge-maps').glob('*.map'):
        inputs[rel(p)] = digest(p)
    for name in ['data/base_control_set.txt','data/base_mesh_info.txt','tools/prepare_final_benchmark.py']:
        inputs[name] = digest(ROOT/name)
    manifest = dict(version=1, seed=SEED, status='inputs_frozen_no_search_run',
                    benchmark_task_count=len(tasks), edge_task_count=len(edges),
                    input_sha256=inputs, prior_task_sources_sha256=prior_sources,
                    excluded_prior_geometries=[list(k) for k in sorted(known)],
                    maps=summaries, tasks=tasks, edge_tasks=edges,
                    protocol=dict(repeats=3,weight=1,exact_pose=True,compact_closed=True,best_g=False,
                                  algorithms=['LBA','Mesh'],primary_variants=['safe-euclidean','max','directional'],
                                  raw_euclidean_separate=True,process_timeout_s=30,reference_timeout_s=120))
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    write_tasks(OUT/'tasks.tsv',tasks)
    write_tasks(OUT/'pilot.tsv',[t for t in tasks if t['pilot']])
    write_tasks(OUT/'edge-tasks.tsv',edges)
    (OUT/'PROTOCOL.md').write_text(PROTOCOL)
    # Self-contained readable report, no renderer/network dependencies.
    body=[]
    for block in PROTOCOL.strip().split('\n\n'):
        tag='h1' if block.startswith('# ') else 'h2' if block.startswith('## ') else 'p'
        content=block[2:] if tag=='h1' else block[3:] if tag=='h2' else block
        body.append(f'<{tag}>{html.escape(content)}</{tag}>')
    summary_rows=''.join(f'<tr><td>{m["name"]}</td><td>{m["suite"]}</td><td>{m["width"]}×{m["height"]}</td><td>40 / 40 / 40</td></tr>' for m in summaries)
    (OUT/'benchmark-protocol.html').write_text('<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Протокол большого теста MeshA*</title><style>body{margin:32px auto;max-width:980px;padding:0 22px;font:17px/1.65 system-ui;color:#183047;background:#f8fafc}h1{font-size:32px;line-height:1.2}h2{font-size:23px;margin-top:36px}table{width:100%;border-collapse:collapse;font-size:15px}td,th{padding:9px;border-bottom:1px solid #cbd8e0;text-align:left}p{background:white;padding:15px;border-left:3px solid #227e8c}small{color:#526373}</style>'+body[0]+'<table><tr><th>Карта</th><th>Группа</th><th>Размер</th><th>Короткие / средние / длинные</th></tr>'+summary_rows+'</table>'+''.join(body[1:])+'</html>')
    outputs = {rel(p):digest(p) for p in OUT.iterdir() if p.is_file()}
    (OUT/'frozen-sha256.json').write_text(json.dumps(outputs,indent=2)+'\n')
    verify()


def verify():
    m=json.loads((OUT/'manifest.json').read_text())
    for paths in [m['input_sha256'],m['prior_task_sources_sha256'],json.loads((OUT/'frozen-sha256.json').read_text())]:
        for path,sha in paths.items():
            assert digest(ROOT/path)==sha, f'Changed frozen file: {path}'
    tasks,edges=m['tasks'],m['edge_tasks']
    assert len(tasks)==960 and len(edges)==64
    assert len({t['id'] for t in tasks+edges})==1024
    assert len({geometry(t['map'],t['start'],t['goal']) for t in tasks})==960
    prior={(x[0],tuple(x[1]),tuple(x[2])) for x in m['excluded_prior_geometries']}
    cache={}
    for t in tasks+edges:
        if t['map'] not in cache:cache[t['map']]=read_map(ROOT/t['map'])[2]
        for pose in [t['start'],t['goal']]:
            assert len(pose)==3 and 0<=pose[2]<16 and free(cache[t['map']],*pose[:2]),t
        if t['suite']!='edge':
            assert geometry(t['map'],t['start'],t['goal']) not in prior
            row=(ROOT/t['scenario']).read_text().splitlines()[t['scenario_line']-1].split()
            assert [int(row[5]),int(row[4])]==t['start'][:2] and [int(row[7]),int(row[6])]==t['goal'][:2]
            assert float(row[8])==t['grid_reference_distance']
    for name,suite in MAPS:
        group=[t for t in tasks if t['map']==f'maps/{name}.map']
        assert len(group)==120
        assert set(Counter(t['goal'][2] for t in group).values())=={7,8}
        for band in ['short','middle','long']:
            sub=[t for t in group if t['band']==band]
            assert len(sub)==40 and Counter(t['heading_relation'] for t in sub)=={'same':10,'opposite':10,'other':20}
        for t in group:
            d=(t['start'][2]-t['goal'][2])%16
            assert (d==0 if t['heading_relation']=='same' else d==8 if t['heading_relation']=='opposite' else d not in [0,8])
    assert Counter(t['family'] for t in edges)=={k:8 for k in ['identical-pose','same-cell-other-heading','empty-short','boundary','one-cell-corridor','gap1','gap3','disconnected']}
    for path,rows in [('tasks.tsv',tasks),('edge-tasks.tsv',edges),('pilot.tsv',[t for t in tasks if t['pilot']])]:
        actual=(OUT/path).read_text().splitlines()
        expected=[' '.join(map(str,[t['id'],t['map'],*t['start'],*t['goal']])) for t in rows]
        assert actual==expected,path
    print(json.dumps(dict(status='PASS',benchmark_tasks=960,edge_tasks=64,pilot_tasks=24,
                         maps=8,known_geometry_overlap=0,hashes_verified=True,searches_run=0)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    verify() if args.verify else generate()
