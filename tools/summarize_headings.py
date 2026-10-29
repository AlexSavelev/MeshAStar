#!/usr/bin/env python3
"""Summarize the fixed-map heading experiment; no changes to planner heuristics."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HEADINGS = [0, 4, 8, 12]
MODES = ["safe-euclidean", "grid", "max"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, nargs="?", default=ROOT / "res/heading-experiment")
    args = parser.parse_args()
    folder = args.folder.resolve()
    raw = [json.loads(s) for s in (folder / "runs.jsonl").read_text().splitlines()]
    summary = []
    diagnostics = {}
    field_reference = None
    map_hashes = set()
    for st in HEADINGS:
        for gt in HEADINGS:
            stem = folder / f"heading-{st}-{gt}"
            d = json.loads(Path(str(stem) + "-diagnostic.json").read_text())
            diagnostics[st, gt] = d
            field = [row[:4] for row in d["cells"]]
            if field_reference is None:
                field_reference = field
            assert field == field_reference, "2D field changed across heading pairs"
            map_hashes.add(hashlib.sha256(Path(str(stem) + ".map").read_bytes()).hexdigest())
            for algo in ["LBA", "Mesh"]:
                for mode in MODES:
                    group = [r for r in raw if (r["start_heading"], r["goal_heading"], r["algorithm"], r["heuristic"]) == (st, gt, algo, mode)]
                    assert len(group) == 7
                    assert {r["offset"] for r in group} == {24}
                    assert len({r["steps"] for r in group}) == 1
                    assert all(abs(r["cost"] - r["reference_cost"]) < 0.001 for r in group)
                    result = dict(start_heading=st, goal_heading=gt, algorithm=algo, heuristic=mode,
                                  repeats=len(group), steps=group[0]["steps"], cost=group[0]["cost"],
                                  reference_cost=group[0]["reference_cost"])
                    for key in ["end_to_end_s", "search_s", "preprocessing_s", "grid_s", "calibration_s", "total_s"]:
                        values = [r[key] for r in group]
                        result[key] = statistics.median(values)
                        result[key + "_q25"] = float(np.quantile(values, .25))
                        result[key + "_q75"] = float(np.quantile(values, .75))
                    summary.append(result)
    assert len(raw) == 672 and len(map_hashes) == 1
    (folder / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    def get(st, gt, algo="LBA", mode="max"):
        return next(r for r in summary if (r["start_heading"], r["goal_heading"], r["algorithm"], r["heuristic"]) == (st, gt, algo, mode))

    def speedup(st, gt, algo):
        return get(st, gt, algo, "safe-euclidean")["end_to_end_s"] / get(st, gt, algo)["end_to_end_s"]

    costs = np.array([[get(st, gt)["reference_cost"] for gt in HEADINGS] for st in HEADINGS])
    expansions = np.array([[get(st, gt)["steps"] for gt in HEADINGS] for st in HEADINGS])
    speed_lba = np.array([[speedup(st, gt, "LBA") for gt in HEADINGS] for st in HEADINGS])
    speed_mesh = np.array([[speedup(st, gt, "Mesh") for gt in HEADINGS] for st in HEADINGS])
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})
    fig, axes = plt.subplots(2, 2, figsize=(10, 8), layout="constrained")
    speed_limit = max(speed_lba.max(), speed_mesh.max())
    panels = [(costs, "Стоимость пути: LBA* с h=0", "YlGnBu", costs.min(), costs.max(), ".2f"),
              (expansions, "Итерации LBA* с max", "Blues", 0, expansions.max(), ",.0f"),
              (speed_lba, "LBA*: ускорение max\nотносительно safe-euclidean", "YlGnBu", 1, speed_limit, ".2f"),
              (speed_mesh, "MeshA*: ускорение max\nотносительно safe-euclidean", "YlGnBu", 1, speed_limit, ".2f")]
    # Common scales for the two speedup panels. If a slowdown occurs, include it.
    speed_floor = min(1.0, speed_lba.min(), speed_mesh.min())
    for ax, (values, title, cmap, low, high, fmt) in zip(axes.flat, panels):
        if values is speed_lba or values is speed_mesh:
            low = speed_floor
        img = ax.imshow(values, cmap=cmap, vmin=low, vmax=high)
        for i in range(4):
            for j in range(4):
                colour = "white" if (values[i, j]-low) / max(high-low, 1e-12) > .57 else "#172a36"
                suffix = "×" if values is speed_lba or values is speed_mesh else ""
                ax.text(j, i, format(values[i, j], fmt) + suffix, ha="center", va="center", color=colour)
        ax.set_xticks(range(4), labels=HEADINGS)
        ax.set_yticks(range(4), labels=HEADINGS)
        ax.set(xlabel="Направление цели (индекс)", ylabel="Направление старта (индекс)", title=title)
        fig.colorbar(img, ax=ax, shrink=.75)
    h_start = max(diagnostics[0, 0]["start_h_e"], diagnostics[0, 0]["start_h_grid"])
    fig.suptitle(f"Одна карта, одно поле, разные направления\nСмещение прохода 24 клетки; h(start) = {h_start:.3f} во всех 16 задачах", fontsize=15)
    fig.savefig(folder / "heading-results.png", dpi=180)
    plt.close(fig)

    error = max(abs(r["cost"] - r["reference_cost"]) for r in raw)
    selected = [get(st, gt) for st in HEADINGS for gt in HEADINGS]
    least = min(selected, key=lambda r: r["steps"])
    most = max(selected, key=lambda r: r["steps"])
    lines = ["# Направления старта и цели при фиксированной карте\n\n",
             "Карта 128×128 со стеной в столбце 64 и проходом в строках 80…96. "
             "Старт `(64,16,theta_s)`, цель `(64,112,theta_g)`. Направления — индексы 0,4,8,12 "
             "из 16 дискретных направлений, все 16 сочетаний. Координаты и препятствия не меняются.\n\n",
             "## Протокол и проверки\n\n",
             "- LBA* и MeshA*, safe-euclidean / grid / max, w=1, R=A=0.\n",
             "- 7 измерений на сочетание, по одному дополнительному прогреву: 672 измеренных поиска.\n",
             "- Перемешаны порядок задач и вариантов (seed 20261025); все запуски последовательные.\n",
             "- Медианы времени solve() включают подготовку, поиск, восстановление и очистку. Чтение файлов вне замера.\n",
             "- Все пути проверены на столкновения и точное конечное состояние; для каждой задачи выполнен LBA* с h=0.\n",
             f"- Максимальное расхождение стоимости с h=0: {error:.12g}; допуск 0.001 учитывает float g/f.\n",
             "- Диагностика отдельно от времени; её число раскрытий совпадает с обычным запуском.\n",
             "- Проверено совпадение байтов карт и значений обоих 2D-полей во всех клетках для всех 16 пар.\n",
             "- Поле строится заново в каждом solve(); выгода от возможного переиспользования в таблицу не включена.\n\n",
             "## Результаты\n\n",
             f"Одинаковое h_max(start) = {h_start:.9f}. Стоимость решения меняется от {costs.min():.6f} "
             f"до {costs.max():.6f}. Итерации LBA* с max — от {expansions.min():,.0f} "
             f"до {expansions.max():,.0f} ({expansions.max()/expansions.min():.2f}×).\n\n",
             "| theta_s → theta_g | Стоимость h=0 | LBA* итерации: база → max | LBA* мс: база → max | MeshA* мс: база → max |\n",
             "|---|---:|---:|---:|---:|\n"]
    for st in HEADINGS:
        for gt in HEADINGS:
            b, m = get(st, gt, "LBA", "safe-euclidean"), get(st, gt)
            mb, mm = get(st, gt, "Mesh", "safe-euclidean"), get(st, gt, "Mesh")
            lines.append(f"| {st} → {gt} | {m['reference_cost']:.4f} | {b['steps']} → {m['steps']} | "
                         f"{b['end_to_end_s']*1000:.2f} → {m['end_to_end_s']*1000:.2f} | "
                         f"{mb['end_to_end_s']*1000:.2f} → {mm['end_to_end_s']*1000:.2f} |\n")
    lines += ["\n## Выводы\n\n",
              f"1. При одном поле разброс числа итераций max существенный: наименьшее для "
              f"{least['start_heading']}→{least['goal_heading']}, наибольшее для "
              f"{most['start_heading']}→{most['goal_heading']}. Поле не различает эти ограничения.\n",
              f"2. Отношение медианного времени safe-euclidean / max: LBA* "
              f"{speed_lba.min():.2f}…{speed_lba.max():.2f}, MeshA* {speed_mesh.min():.2f}…{speed_mesh.max():.2f}. "
              "Значение больше 1 означает ускорение. Это наблюдение на одной карте, не общий результат.\n",
              "3. Изменение heading меняет реальные допустимые продолжения и условие достижения цели. "
              "Поэтому разброс работы поиска сам по себе не доказывает, что любая эвристика с heading "
              "устранит его. Для проверки такой эвристики потребуется отдельное сравнение на тех же задачах.\n",
              "4. Но потеря информации видна непосредственно: в клетке цели пространственная оценка "
              "равна нулю даже при неправильном направлении, хотя достижение правильного направления "
              "обычно требует дополнительного движения.\n\n",
              "## О взвешивании и smooth max\n\n",
              "Максимум согласованных оценок уже выбирает более сильную в каждом состоянии. "
              "Постоянная выпуклая смесь допустима и согласована, но не превосходит max. "
              "Смесь с коэффициентами, меняющимися от состояния к состоянию, остаётся допустимой "
              "при коэффициентах из [0,1], однако согласованность требует отдельного доказательства.\n\n",
              "Обычный log-sum-exp `tau*log(exp(h1/tau)+exp(h2/tau))` больше max и может завышать "
              "стоимость (в цели даёт tau*log(2)). Нормировка делением суммы экспонент на 2 "
              "при постоянном tau>0 даёт согласованную оценку: функция монотонна по аргументам, "
              "перенос общей константы переносит результат на ту же константу, а F(0,0)=0. "
              "Но эта оценка не сильнее max и требует дополнительных вычислений. "
              "A* не нуждается в гладкости эвристики.\n\n",
              "## Следующее направление\n\n",
              "Простейшая релаксация с ориентацией: граф из 16 headings, ребро для каждого примитива "
              "из его начального heading в конечный со стоимостью c(p). Кратчайшее расстояние "
              "до целевого heading даёт согласованную нижнюю оценку h_turn. Игнорируются координаты "
              "и препятствия. Можно взять max(h_E,h_grid,h_turn). Это теоретический кандидат, "
              "пока не реализован и не измерен. На больших расстояниях может быть слабее пространственных "
              "оценок; его польза не гарантирована. В частности, h_turn=0 и для пары 0→0, "
              "и для 8→8, хотя в эксперименте их стоимость и трудоёмкость сильно различаются.\n\n",
              "Более содержательный кандидат: расстояние на решётке примитивов без препятствий "
              "с сохранением относительного положения и обоих headings. Оно учитывает кинематические "
              "ограничения, которые 2D-поле отбрасывает. Максимум такой оценки с текущими "
              "пространственными оценками останется согласованным, если новая оценка согласована. "
              "При предвычислении нельзя просто запретить выход за границу конечной таблицы: "
              "удаление путей может завысить расстояние и нарушить нижнюю оценку. Конечную "
              "абстракцию или иной способ вычисления ещё предстоит обосновать.\n\n",
              "## Воспроизведение\n\n```sh\nmake wall-experiment\n"
              "./obj/wall_experiment res/heading-experiment 7 headings\n"
              "python3 tools/summarize_headings.py res/heading-experiment\n```\n\n",
              "Числа итераций воспроизводимы; время зависит от среды. Межквартильные диапазоны "
              "сохранены в summary.json. Исходные измерения — runs.jsonl, диагностика — "
              "heading-*-diagnostic.json. Повторный запуск перезаписывает выбранную выходную папку.\n"]
    (folder / "REPORT.md").write_text("".join(lines))
    sources = ["Makefile", "tools/wall_experiment.cpp", "tools/summarize_headings.py"]
    sources += [str(p.relative_to(ROOT)) for name in ["src", "include"] for p in (ROOT/name).glob('*') if p.is_file()]
    metadata = dict(compiler=subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
                    flags="-std=c++11 -Wall -Werror -Wextra -O3", seed=20261025,
                    measured_runs=len(raw), max_cost_difference=error, map_sha256=next(iter(map_hashes)),
                    identical_2d_fields=True, source_sha256={f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources})
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("".join(lines[12:])[:6000])
    print(f"Saved report, figure and summary in {folder}")


if __name__ == "__main__":
    main()
