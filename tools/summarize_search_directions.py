"""Validate both experiments and create a standalone offline report."""
from pathlib import Path
from collections import defaultdict
import json
import statistics as st
import hashlib
import html
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[1]
S = ROOT / 'res/search-structure-experiment'
D = ROOT / 'res/directional-experiment'


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def summarize(folder, algorithms, variants):
    rows = read(folder / 'runs.jsonl')
    assert len(rows) == 432
    groups = defaultdict(list)
    for row in rows:
        groups[row['algorithm'], row['variant'], row['task']].append(row)
    assert len(groups) == 144
    for group in groups.values():
        assert {r['repeat'] for r in group} == {0, 1, 2}
        assert len({(r['steps'], r['solved'], r['cost'], r['pushes']) for r in group}) == 1
    refs = {r['task']: r for r in read(ROOT / 'res/multimap-experiment/references.jsonl')}
    error = 0
    for row in rows:
        ref = refs[row['task']]
        assert row['solved'] == ref['solved']
        if row['solved']:
            error = max(error, abs(row['cost'] - ref['cost']))
            assert abs(row['cost'] - ref['cost']) < .001
    summary = {'max_cost_error': error, 'measured_searches': len(rows), 'aggregates': {}, 'tasks': {}}
    for solved, label in [(False, 'all24'), (True, 'solved17')]:
        summary['aggregates'][label] = {}
        for algorithm in algorithms:
            summary['aggregates'][label][algorithm] = {}
            for variant in variants:
                selected = [r for r in rows if r['algorithm'] == algorithm and r['variant'] == variant
                            and (r['solved'] or not solved)]
                stats = {}
                for metric in ['end_to_end_s', 'search_s', 'setup_s', 'cleanup_s', 'preprocessing_s',
                               'directional_s', 'steps', 'pushes', 'pops', 'dominated', 'closed_discards', 'stale_discards']:
                    if metric not in selected[0]:
                        continue
                    values = [sum(r[metric] for r in selected if r['repeat'] == i) for i in range(3)]
                    stats[metric] = {'median': st.median(values), 'min': min(values), 'max': max(values)}
                for metric in ['peak_open', 'peak_best_g']:
                    stats[metric] = max(r[metric] for r in selected)
                summary['aggregates'][label][algorithm][variant] = stats
    for (algorithm, variant, task), selected in groups.items():
        result = {k: selected[0][k] for k in ['solved', 'cost', 'steps', 'closed_bytes', 'peak_open', 'peak_best_g']}
        for k in ['end_to_end_s', 'search_s']:
            result[k] = st.median(r[k] for r in selected)
        summary['tasks'].setdefault(task, {}).setdefault(algorithm, {})[variant] = result
    (folder / 'summary.json').write_text(json.dumps(summary, indent=2))
    return summary, rows


s, srows = summarize(S, ['LBA', 'Mesh'], ['legacy', 'compact', 'best-g'])
d, drows = summarize(D, ['LBA', 'LBA-best-g', 'Mesh'], ['max', 'directional'])
old = read(ROOT / 'res/alpha-experiment/runs.jsonl')
oldsteps = {(r['task'], r['algorithm']): r['steps'] for r in old if r['variant'] == 'max'}
for r in srows:
    if r['variant'] != 'best-g':
        assert r['steps'] == oldsteps[r['task'], r['algorithm']]
for r in drows:
    if r['variant'] == 'max':
        algorithm = 'LBA' if r['algorithm'] == 'LBA-best-g' else r['algorithm']
        variant = 'best-g' if r['algorithm'] == 'LBA-best-g' else 'compact'
        assert r['steps'] == s['tasks'][r['task']][algorithm][variant]['steps']


def value(summary, subset, algorithm, variant, metric='end_to_end_s'):
    return summary['aggregates'][subset][algorithm][variant][metric]['median']


def table_s(subset):
    table = [['Алгоритм', 'Исходный CLOSED', 'Компактный CLOSED', 'Компактный + best-g']]
    for a in ['LBA', 'Mesh']:
        table.append([a] + [f'{value(s, subset, a, v):.3f}' for v in ['legacy', 'compact', 'best-g']])
    return table


def table_d(subset):
    table = [['Алгоритм', 'Пространственная max', '+ направленная', 'Ускорение']]
    for a in ['LBA', 'LBA-best-g', 'Mesh']:
        baseline = value(d, subset, a, 'max')
        new = value(d, subset, a, 'directional')
        table.append([a, f'{baseline:.3f}', f'{new:.3f}', f'{baseline/new:.2f}×'])
    return table


def example(task):
    table = [['Алгоритм', 'max, мс', 'directional, мс', 'Шаги max → directional']]
    for a in ['LBA', 'LBA-best-g', 'Mesh']:
        t = d['tasks'][task][a]
        b, n = t['max'], t['directional']
        table.append([a, f"{1000*b['end_to_end_s']:.2f}", f"{1000*n['end_to_end_s']:.2f}", f"{b['steps']} → {n['steps']}"])
    return table


diag = read(D / 'diagnostics.jsonl')
solved = [r for r in diag if r['solved']]
prep = read(D / 'preparation.json')[0]
medianprep = st.median(r['directional_s'] for r in drows if r['variant'] == 'directional')
blocks = []


def heading(text):
    blocks.append(('heading', text))


def paragraph(text):
    blocks.append(('paragraph', ' '.join(text.split())))


def table(data):
    blocks.append(('table', data))


heading('Протокол и вывод')
paragraph('''26 октября 2026. Проверены три идеи: уменьшение CLOSED, отсечение по лучшему g и направленная
эвристика. Первая уменьшает буферы с умеренным выигрышем времени; вторая существенно помогает LBA*,
но замедляет MeshA*; третья полезна на части решаемых задач, но не дала общего ускорения MeshA* на всей серии.
Это два отдельных эксперимента: абсолютные времена между ними не сравниваем.''')
paragraph('''Прежние 24 задачи на Berlin_2_256, Milan_1_256 и AR0700SR: 17 решаемых и 7 нерешаемых.
Три повтора, фиксированная случайная перестановка задач и вариантов, seed 20261026.
В каждом эксперименте 432 измеряемых поиска и 6 прогревов. В таблицах — медиана трёх сумм
end_to_end_time по задачам, секунды. Включены подготовка эвристики, создание контекста, поиск,
восстановление пути и очистка. Чтение файлов, первоначальное создание общего пула и проверки результата исключены.''')
paragraph(f'''Все найденные пути проверены по клеткам следа примитивов и сопоставлены с прежними h=0 эталонами.
Максимальное расхождение стоимости {max(s['max_cost_error'], d['max_cost_error']):.8g}, допуск 0.001.
Это численная проверка исходной арифметики float, не доказательство точности всего планировщика.''')
heading('1. CLOSED: изменение реализации')
paragraph('''Исходный код выделял 324 000 001 байт (309 МиБ) под CLOSED для любого поиска. Теперь
по умолчанию используются фактические размеры карты и 16 слоёв LBA*; для MeshA* сохраняются все
1800 слоёв конфигураций. На карте 256×256 это 128 КиБ для LBA* и 14.06 МиБ для MeshA*.
Это размеры буферов, не измерение RSS. Все последовательности примитивов и числа шагов
компактного и исходного вариантов совпали. Числа шагов также совпали с предыдущими экспериментами.''')
paragraph('''Это техническая правка, не алгоритмическая новизна. Все дальнейшие сравнения эвристик
используют её и в базовом варианте. Основной расход времени на этих задачах — поиск и очистка OPEN,
поэтому большого ускорения только от уменьшения CLOSED здесь нет.''')
heading('2. Лучший g среди ожидающих раскрытия путей')
paragraph('''Полный ключ состояния — (i,j,theta) либо (i,j,config). Если уже есть кандидат с не большим g,
новый путь не вставляется в OPEN. Устаревшие записи пропускаются при извлечении. После выбора состояния
запись удаляется из хеш-таблицы; дальше повторы обрабатывает CLOSED. Плотный массив float на все
Mesh-состояния не создаётся. Расчёт эвристики и создание кандидата пока происходят до проверки best-g.''')
paragraph('Все 24 задачи:')
table(table_s('all24'))
paragraph('Только 17 решаемых задач:')
table(table_s('solved17'))
paragraph(f'''В исходном LBA* {value(s, 'all24', 'LBA', 'compact', 'closed_discards') / value(s, 'all24', 'LBA', 'compact', 'pops'):.1%}
извлечений являются повторениями закрытых состояний, в MeshA* —
{value(s, 'all24', 'Mesh', 'compact', 'closed_discards') / value(s, 'all24', 'Mesh', 'compact', 'pops'):.1%}.
Очистка очереди после остановки не входит в эти счётчики. Best-g убирает
{value(s, 'all24', 'LBA', 'best-g', 'dominated'):.0f} вставок LBA* и
{value(s, 'all24', 'Mesh', 'best-g', 'dominated'):.0f} вставок MeshA*.
Для MeshA* обслуживание хеш-таблицы не окупилось. Это результат для данной реализации, не запрет на все способы best-g.''')
paragraph('''Изменение порядка равных приоритетов допускается: шаги best-g могут немного отличаться,
стоимости и допустимость сохранены. Исходный счётчик steps включает итерацию достижения цели,
иногда последнюю итерацию очистки, поэтому не равен строго числу раскрытых вершин.
Для LazyLBA* best-g явно отключается: непроверенный дешёвый путь может оказаться столкновением
и не должен подавлять допустимую альтернативу. Отдельный тест воспроизводит этот случай.
Это стандартное правило доминирования A*, не новый алгоритм. Опция не включена по умолчанию.''')
heading('3. Направленная эвристика: конструкция и доказательство')
paragraph('''v — один из 16 фиксированных единичных векторов на плоскости. Это направления измерения
проекции смещения, а не номера ориентаций робота: ориентации исходного control set распределены неравномерно.
Для примитива p берём остаточную стоимость r_v(p) = c(p) − β⟨v, Δp⟩, где β = 0.9933.
Она неотрицательна, поскольку калибровка всех примитивов гарантирует β‖Δp‖ ≤ c(p).
Строим граф на 16 ориентациях с весами r_v. D_v(a,b) — кратчайшее расстояние в нём.''')
paragraph('''Оценка: h_v(s) = β⟨v, x_goal − x_s⟩ + D_v(θ_s, θ_goal).
Итог: h(s) = max(h_spatial(s), 0, max_v h_v(s)). Таблица хранит все пары ориентаций для всех направлений;
вычисляется Floyd–Warshall, независимо проверяется прямой Дейкстрой.''')
paragraph('''Для ребра p:s→t имеем D_v(θ_s,θ_goal) ≤ r_v(p) + D_v(θ_t,θ_goal).
Поэтому h_v(s) − h_v(t) ≤ β⟨v,Δp⟩ + r_v(p) = c(p). В цели h_v = 0.
Максимум согласованных оценок согласован и допустим в точной арифметике.
Множество направлений фиксировано на весь поиск; мы не меняем его произвольно от состояния к состоянию.''')
paragraph('''Координаты не замыкаются по модулю P: линейная часть растёт со смещением.
Однако ориентационная добавка ограничена. Если θ_s = θ_goal, то D_v(θ_s,θ_goal) = 0,
и направленная часть не усиливает безопасную евклидову. Она не описывает полный манёвр
«разворот–движение–разворот». Препятствия учитываются прежним grid-компонентом через max.
Поддерживается одна целевая клетка и точная ориентация: радиус после EPS меньше 1, A=0.''')
paragraph(f'''Таблица занимает {prep['bytes']} байт ({prep['bytes']/1024:.2f} КиБ), включая векторы направлений.
Медиана её построения в замерах {medianprep*1000:.4f} мс. Она строится заново в каждом запросе;
кэширование не требуется для полученного результата. Это дополнительная память, без grid, CLOSED и OPEN.''')
paragraph(f'''На прежних раскрытиях 17 решаемых задач медианная доля состояний с усилением составила
{st.median(r['stronger']/r['finite'] for r in solved):.1%}, средняя прибавка по объединённым раскрытиям —
{sum(r['delta_sum'] for r in solved)/sum(r['finite'] for r in solved):.3f}.
После диагностики параметры не подбирались: остались 16 направлений.
Счётчик пересечений f>C* в diagnostics использует старые g и не является точным числом исключённых раскрытий.''')
paragraph('Все 24 задачи; время включает всю подготовку новой эвристики:')
table(table_d('all24'))
paragraph('Только 17 решаемых задач:')
table(table_d('solved17'))
paragraph('''Проверены LBA*, LBA* с best-g и MeshA* без best-g. Внутри каждого сравнения эвристик
организация OPEN и CLOSED одинакова. Ускорение от best-g не приписывается эвристике.
На нерешаемых задачах ориентационная оценка не доказывает недостижимость: обход остаётся большим,
а вычисление 16 потенциалов добавляет работу. Поэтому результаты решаемых задач нельзя переносить на все запросы.''')
paragraph('Berlin_2_256-b0-h3, ранее разобранный положительный пример:')
table(example('Berlin_2_256-b0-h3'))
paragraph('AR0700SR-b1-h0, ранее разобранный слабый пример:')
table(example('AR0700SR-b1-h0'))
heading('Проверки, ограничения и следующий шаг')
paragraph('''make test прошёл: прежние grid/kinematic/additive проверки; компактный CLOSED, равные и
устаревшие g, разные слои, владение памятью и недопустимый дешёвый путь в Lazy A*;
251 904 проверки направленной оценки и 69 429 допустимых рёбер карты для совместной эвристики.
Проверены start=goal, отсутствие пути, найденные пути LBA/Lazy/Mesh с best-g и без него.
После замеров проверена командная строка directional + best-g на Berlin для всех трёх алгоритмов.''')
paragraph('''Набор из 24 задач уже участвовал в разработке: это не независимая проверочная выборка.
Следующий исследовательский шаг — новые заранее зафиксированные задачи и анализ, когда
ориентационный штраф действительно сокращает поиск. Историческая новизна такой конструкции здесь
не установлена. Динамические отсечения MeshA* по CLOSED и float g/f наследуются от исходного кода;
доказательство эвристики не является доказательством всей реализации.''')
paragraph('''ML-эвристики в этом заходе не реализовывались. Для отдельного эксперимента нужно учитывать
подготовку/загрузку модели, вычисление оценок, поиск и качество пути, отдельно холодный и повторный запрос.''')
heading('Запуск и артефакты')
blocks.append(('code', '''make -j4
./mesh_astar --mode single --map maps/Berlin_2_256.map \\
  --prim data/base_control_set.txt --mesh data/base_mesh_info.txt \\
  --start "197 95 5" --goal "101 83 3" --weight 1 \\
  --heuristic directional --out-prefix res/directional_'''))
paragraph('''Флаг --best-g включает эксперимент для eager LBA* и MeshA*, LazyLBA* явно сохраняет прежнее
поведение. Для MeshA* эта опция в данной серии невыгодна. --legacy-closed воспроизводит старое
размещение CLOSED. По умолчанию CLOSED компактный, best-g выключен, эвристика остаётся исходной euclidean;
для доказанных оценок явно задавайте max или directional.''')
paragraph('''Сырые данные, диапазоны времени трёх повторов, счётчики и отдельные summary.json сохранены в
res/search-structure-experiment и res/directional-experiment. Этот отчёт строит
tools/summarize_search_directions.py; он также проверяет совпадение базовых шагов с предыдущими экспериментами.
Программы замеров отказываются перезаписывать существующий runs.jsonl. HTML полностью автономный: без сети и JavaScript.''')

markdown = ['# CLOSED, дубликаты OPEN и направленная эвристика']
body = ['<h1>CLOSED, дубликаты OPEN и направленная эвристика</h1>']
for kind, content in blocks:
    if kind == 'heading':
        markdown.append('## ' + content)
        body.append('<h2>' + html.escape(content) + '</h2>')
    elif kind == 'paragraph':
        markdown.append(content)
        body.append('<p>' + html.escape(content) + '</p>')
    elif kind == 'code':
        markdown.append('```sh\n' + content + '\n```')
        body.append('<pre><code>' + html.escape(content) + '</code></pre>')
    else:
        markdown.append('\n'.join(['| ' + ' | '.join(content[0]) + ' |', '| ' + ' | '.join(['---'] * len(content[0])) + ' |']
                                  + ['| ' + ' | '.join(row) + ' |' for row in content[1:]]))
        body.append('<table><thead><tr>' + ''.join('<th>' + html.escape(c) + '</th>' for c in content[0]) + '</tr></thead><tbody>'
                    + ''.join('<tr>' + ''.join('<td>' + html.escape(c) + '</td>' for c in row) + '</tr>' for row in content[1:]) + '</tbody></table>')
report = '\n\n'.join(markdown) + '\n'
for folder in [S, D]:
    (folder / 'REPORT.md').write_text(report)
page = '''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MeshA*: три направления</title><style>body{font:17px/1.65 system-ui,sans-serif;color:#172638;background:#f7f9fc;max-width:1050px;margin:40px auto;padding:0 24px}h1,h2{line-height:1.25;color:#153d64}h2{margin-top:2em}table{border-collapse:collapse;width:100%;background:white;font-variant-numeric:tabular-nums}td,th{padding:10px;border:1px solid #d7e1eb;text-align:left}th{background:#e7eef6}pre{overflow:auto;padding:16px;background:#e8eef5}code{font-size:.88em}</style></head><body>'''
(D / 'report.html').write_text(page + '\n'.join(body) + '</body></html>')
sources = sorted(list((ROOT / 'include').glob('*.hpp')) + list((ROOT / 'src').glob('*.cpp'))
                 + list((ROOT / 'tests').glob('*.cpp'))) + [ROOT / 'Makefile', Path(__file__)]
for folder, binary in [(S, 'search_structure_experiment'), (D, 'directional_experiment')]:
    files = sources + [ROOT / 'tools' / (binary + '.cpp'), folder / 'manifest.json', folder / 'tasks.tsv',
                       folder / 'references.tsv', ROOT / 'data/base_control_set.txt', ROOT / 'data/base_mesh_info.txt']
    files += [ROOT / f for f in sorted({line.split()[1] for line in (folder / 'tasks.tsv').read_text().splitlines()})]
    metadata = {
        'platform': platform.platform(),
        'compiler': subprocess.check_output(['g++', '--version'], text=True).splitlines()[0],
        'flags': '-std=c++11 -Wall -Werror -Wextra -O3; DEBUG assertions enabled by rassert.hpp',
        'binary_sha256': hashlib.sha256((ROOT / 'obj' / binary).read_bytes()).hexdigest(),
        'final_source_and_input_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'note': 'Final source hashes include directional integration added after the structure experiment. Original pre-change files are in source-before; binaries have separate hashes. The earlier run did not use the later final source tree.',
    }
    (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2))
print('PASS: 864 searches, h0 costs, repeat counters, previous baseline steps; offline report generated')
for subset in ['all24', 'solved17']:
    print(subset, 'structure', table_s(subset))
    print(subset, 'directional', table_d(subset))
