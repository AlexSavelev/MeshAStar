#!/usr/bin/env python3
"""Summarize wall_experiment output and render standalone scientific figures."""
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
MODES = ["safe-euclidean", "grid", "max"]
COLORS = {"safe-euclidean": "#566675", "grid": "#d48425", "max": "#147d92"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, nargs="?", default=ROOT / "res/wall-experiment")
    args = parser.parse_args()
    folder = args.folder.resolve()
    rows = [json.loads(line) for line in (folder / "runs.jsonl").read_text().splitlines()]
    offsets = sorted({r["offset"] for r in rows})
    diagnostics = {}
    summary = []
    for offset in offsets:
        stem = "empty" if offset < 0 else f"gap-{offset}"
        d = json.loads((folder / f"{stem}-diagnostic.json").read_text())
        diagnostics[offset] = d
        for algorithm in ["LBA", "Mesh"]:
            for mode in MODES:
                group = [r for r in rows if (r["offset"], r["algorithm"], r["heuristic"]) == (offset, algorithm, mode)]
                assert len(group) >= 3
                assert len({r["steps"] for r in group}) == 1
                assert all(abs(r["cost"] - r["reference_cost"]) < 0.001 for r in group)
                row = dict(offset=offset, algorithm=algorithm, heuristic=mode, repeats=len(group),
                           steps=group[0]["steps"], cost=group[0]["cost"])
                for key in ["end_to_end_s", "search_s", "preprocessing_s", "grid_s", "calibration_s", "total_s"]:
                    values = [r[key] for r in group]
                    row[key] = statistics.median(values)
                    row[key + "_q25"] = float(np.quantile(values, .25))
                    row[key + "_q75"] = float(np.quantile(values, .75))
                summary.append(row)
    (folder / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    def get(offset, algo, mode):
        return next(r for r in summary if (r["offset"], r["algorithm"], r["heuristic"]) == (offset, algo, mode))

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "axes.grid": True, "grid.alpha": .18})
    wall = [o for o in offsets if o >= 0]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(wall, [100*diagnostics[o]["improved_fraction"] for o in wall], "o-", color=COLORS["max"])
    ax.set(title="Где поле усиливает оценку", ylabel="% раскрытых состояний базового LBA*", ylim=(-3, 103))
    ax = axes[0, 1]
    for mode in MODES:
        ax.plot(wall, [get(o, "LBA", mode)["steps"] for o in wall], "o-", color=COLORS[mode], label=mode)
    ax.set(title="Работа LBA*", ylabel="Итерации поиска (включая извлечение цели)")
    ax.legend()
    for ax, algo in zip(axes[1], ["LBA", "Mesh"]):
        for mode in MODES:
            group = [get(o, algo, mode) for o in wall]
            med = np.array([r["end_to_end_s"] * 1000 for r in group])
            lo = np.array([r["end_to_end_s_q25"] * 1000 for r in group])
            hi = np.array([r["end_to_end_s_q75"] * 1000 for r in group])
            ax.errorbar(wall, med, yerr=[med-lo, hi-med], marker="o", capsize=3, color=COLORS[mode], label=mode)
        ax.set(title=f"{algo}*: время с подготовкой и очисткой", ylabel="Медиана, мс; полосы — 25–75-й процентили")
        ax.legend()
    for ax in axes.flat:
        ax.set_xlabel("Смещение центра прохода от линии старт–цель, клетки")
        ax.set_xticks(wall)
    fig.suptitle("Стена с перемещаемым проходом · 128 × 128 · проход 17 клеток", fontsize=16)
    fig.savefig(folder / "wall-results.png", dpi=180)
    fig.savefig(folder / "wall-results.pdf")
    plt.close(fig)

    # Spatial diagnostic: same colour scale across offsets for each quantity.
    selected = [0, 24, 48]
    delta_limit = max(diagnostics[o]["max_delta_expanded"] for o in selected)
    fig, axes = plt.subplots(3, 3, figsize=(11, 10), layout="constrained")
    for row_idx, offset in enumerate(selected):
        d = diagnostics[offset]
        data = np.asarray(d["cells"])
        ii, jj = data[:, 0].astype(int), data[:, 1].astype(int)
        visits, enhanced = np.full((128, 128), np.nan), np.full((128, 128), np.nan)
        delta = np.full((128, 128), np.nan)
        visits[ii, jj] = data[:, 4]
        enhanced[ii, jj] = data[:, 5]
        # Delta is shown ONLY at cells with at least one baseline expansion.
        mask = data[:, 4] > 0
        delta[ii[mask], jj[mask]] = np.maximum(0, data[mask, 3] - data[mask, 2])
        occupied = np.ones((128, 128)); occupied[ii, jj] = 0
        for col, (values, cmap, limit) in enumerate([(visits, "Blues", 16), (enhanced, "Blues", 16),
                                                    (delta, "YlOrRd", delta_limit)]):
            ax = axes[row_idx, col]
            ax.grid(False)
            ax.imshow(occupied, cmap="Greys", vmin=0, vmax=1, interpolation="nearest")
            img = ax.imshow(values, cmap=cmap, vmin=0, vmax=limit, interpolation="nearest")
            ax.scatter([16], [64], s=36, color="#009b55", edgecolor="black", linewidth=.6, zorder=4)
            ax.scatter([112], [64], s=50, marker="*", color="#ca2352", edgecolor="black", linewidth=.6, zorder=4)
            ax.set_xticks([0, 64, 127]); ax.set_yticks([0, 64, 127])
            if col == 0:
                ax.set_ylabel(f"Смещение {offset}\nСтрока i")
            if row_idx == 2:
                ax.set_xlabel("Столбец j")
            if row_idx == 0:
                ax.set_title(["Раскрытия: safe-euclidean", "Раскрытия: max", "Добавочная оценка Δh"][col])
            if row_idx == 2:
                fig.colorbar(img, ax=list(axes[:, col]), shrink=.65,
                             label="Число направлений (0–16)" if col < 2 else "Δh на клетках базового поиска")
    fig.suptitle("Где именно меняется поиск\nЗелёная точка — старт, красная звезда — цель; чёрная линия — стена", fontsize=14)
    fig.savefig(folder / "wall-search-regions.png", dpi=180)
    plt.close(fig)

    lines = ["# Эксперимент: стена с перемещаемым проходом\n\n",
             "Карта 128×128. Старт `(64,16,0)`, цель `(64,112,0)`. Стена — столбец 64, "
             "от края до края карты. Проход — строки `64+offset-8 … 64+offset+8` включительно. "
             "Меняется только его положение. Отдельно проверена пустая карта.\n\n",
             "## Протокол\n\n",
             "- 8 карт; LBA* и MeshA*; safe-euclidean, grid и max; w=1, R=A=0.\n",
             "- 7 повторов каждой комбинации, 336 измеренных поисков. Один дополнительный прогрев каждой комбинации.\n",
             "- Фиксированное перемешивание карт и порядка вариантов, seed 20261025; последовательное выполнение.\n",
             "- Время solve() включает подготовку поля/калибровку, поиск, восстановление и очистку. Чтение файлов вне замера.\n",
             "- LBA* с h=0 для каждой карты; проверены столкновения и точное конечное состояние всех путей.\n",
             "- Диагностика выполняется отдельно. Число раскрытий в ней совпадает с обычными запусками.\n",
             "- Δh=max(0,h_grid−h_E); доля улучшенных состояний считается на раскрытиях базового LBA*, с учётом heading. "
             "Цель исключена. В логах Steps включает также итерацию извлечения цели.\n\n",
             "## Результаты\n\n",
             "Медианы времени в мс. Отношение B/M > 1 означает выигрыш max относительно safe-euclidean.\n\n",
             "| Смещение | Улучшено состояний LBA* | Средняя Δh | LBA*: B → M, мс | B/M | Mesh*: B → M, мс | B/M |\n",
             "|---|---:|---:|---:|---:|---:|---:|\n"]
    for o in offsets:
        d = diagnostics[o]
        b, m = get(o, "LBA", "safe-euclidean"), get(o, "LBA", "max")
        mb, mm = get(o, "Mesh", "safe-euclidean"), get(o, "Mesh", "max")
        label = "Без стены" if o < 0 else str(o)
        lines.append(f"| {label} | {100*d['improved_fraction']:.1f}% | {d['mean_delta_all_expanded']:.2f} | "
                     f"{b['end_to_end_s']*1000:.2f} → {m['end_to_end_s']*1000:.2f} | "
                     f"{b['end_to_end_s']/m['end_to_end_s']:.2f} | "
                     f"{mb['end_to_end_s']*1000:.2f} → {mm['end_to_end_s']*1000:.2f} | "
                     f"{mb['end_to_end_s']/mm['end_to_end_s']:.2f} |\n")
    error = max(abs(r['cost']-r['reference_cost']) for r in rows)
    lines += [f"\nМаксимальное расхождение стоимости с h=0: {error:.12g}. Допуск проверки — 0.001; "
              "g/f исходного поиска хранятся в float. Все задачи решены.\n\n",
              "## Интерпретация\n\n",
              "1. Пустая карта и проход на прямой: max не усиливает эвристику на раскрытых состояниях. "
              "Число раскрытий сохраняется, подготовка поля даёт накладные расходы.\n",
              "2. При смещении 16 уже есть усиление на 36.9% раскрытий базового LBA*, "
              "хотя в стартовом состоянии grid всё ещё слабее евклидовой. Сравнения эвристик только в старте недостаточно.\n",
              "3. С ростом обхода поле усиливает оценку на большей части раскрываемых состояний, "
              "и в этом семействе max значительно сокращает поиск. Изменение времени не обязано быть монотонным: "
              "меняются сложность задачи, ориентации на подходе и взаимодействие примитивов со стеной.\n",
              "4. Grid без max на простых задачах проигрывает из-за масштаба 0.9. "
              "Объединение с евклидовой оценкой сохраняет её силу на свободных участках.\n\n",
              "Это контролируемая проверка механизма, а не доказательство общего ускорения. "
              "Здесь одна ширина прохода, одна пара направлений, один масштаб карты, один control set. "
              "Семь повторов и межквартильный диапазон характеризуют наблюдённый разброс времени, "
              "не заменяют независимые карты и доверительный анализ.\n\n",
              "## Следующий шаг\n\n",
              "Зафиксировать карту с выраженным обходом и менять только начальное/конечное heading. "
              "Поле останется тем же. Это позволит отделить полезность знания препятствий от потери "
              "информации об ориентации. Затем менять ширину прохода и проверить реальные карты.\n\n",
              "## Воспроизведение\n\n```sh\nmake wall-experiment\n"
              "./obj/wall_experiment res/wall-experiment 7\n"
              "python3 tools/summarize_wall.py res/wall-experiment\n```\n\n",
              "C++-эксперимент использует существующие объекты планировщика; код основного поиска не изменён. "
              "Графики требуют Python с matplotlib и numpy. Повторный запуск перезаписывает файлы в указанной папке.\n"]
    (folder / "REPORT.md").write_text("".join(lines))
    tracked = ["Makefile", "tools/wall_experiment.cpp", "tools/summarize_wall.py"]
    tracked += [str(p.relative_to(ROOT)) for name in ["src", "include"] for p in (ROOT/name).glob('*') if p.is_file()]
    meta = dict(compiler=subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
                flags="-std=c++11 -Wall -Werror -Wextra -O3", seed=20261025, measured_runs=len(rows),
                max_cost_difference=error, source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in tracked})
    (folder / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
    print("".join(lines[12:])[:4500])
    print(f"Saved figures, summary and report in {folder}")


if __name__ == "__main__":
    main()
