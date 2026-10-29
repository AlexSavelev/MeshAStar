#!/usr/bin/env python3
"""Reports for the periodic kinematic lower bound and orientation explanation."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, nargs="?", default=ROOT / "res/kinematic-experiment")
    folder = parser.parse_args().folder.resolve()
    raw = [json.loads(s) for s in (folder/"runs.jsonl").read_text().splitlines()]
    prep = [json.loads(s) for s in (folder/"preparation.jsonl").read_text().splitlines()]
    assert len(raw) == 736 and len(prep) == 24
    summary = []
    for st in HEADINGS:
        for gt in HEADINGS:
            for algo in ["LBA", "Mesh"]:
                for variant in ["max", "kin64", "kin256"]:
                    for phase in (["steady"] if variant == "max" else ["steady", "cold"]):
                        group = [r for r in raw if (r["start_heading"],r["goal_heading"],r["algorithm"],r["variant"],r["phase"]) == (st,gt,algo,variant,phase)]
                        assert len(group) == (7 if phase == "steady" else 1)
                        assert len({r["steps"] for r in group}) == 1
                        assert all(abs(r["cost"]-r["reference_cost"]) < .001 for r in group)
                        result = dict(start_heading=st,goal_heading=gt,algorithm=algo,variant=variant,phase=phase,
                                      repetitions=len(group),steps=group[0]["steps"],cost=group[0]["cost"],
                                      h_spatial_start=group[0]["h_spatial_start"],h_kin_start=group[0]["h_kin_start"])
                        for key in ["end_to_end_s","search_s","preprocessing_s","kinematic_s","grid_s"]:
                            values = [r[key] for r in group]
                            result[key] = statistics.median(values)
                            result[key+"_q25"] = float(np.quantile(values,.25))
                            result[key+"_q75"] = float(np.quantile(values,.75))
                        summary.append(result)
    (folder/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    def get(st,gt,algo,variant,phase="steady"):
        return next(r for r in summary if (r["start_heading"],r["goal_heading"],r["algorithm"],r["variant"],r["phase"]) == (st,gt,algo,variant,phase))
    for st in HEADINGS:
        for gt in HEADINGS:
            for algo in ["LBA","Mesh"]:
                for variant in ["kin64","kin256"]:
                    assert get(st,gt,algo,variant)["steps"] == get(st,gt,algo,variant,"cold")["steps"]
    previous = ROOT/"res/heading-experiment/summary.json"
    if previous.exists():
        for r in json.loads(previous.read_text()):
            if r["heuristic"] == "max":
                assert r["steps"] == get(r["start_heading"],r["goal_heading"],r["algorithm"],"max")["steps"]
    error = max(abs(r["cost"]-r["reference_cost"]) for r in raw)
    print("Maximum cost difference:",error)
    for algo in ["LBA","Mesh"]:
        for variant in ["kin64","kin256"]:
            for phase in ["steady","cold"]:
                ratios = [get(st,gt,algo,"max")["end_to_end_s"]/get(st,gt,algo,variant,phase)["end_to_end_s"] for st in HEADINGS for gt in HEADINGS]
                print(algo,variant,phase, "speedup range",min(ratios),max(ratios))

    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.top":False,"axes.spines.right":False})
    fig, axes = plt.subplots(1,2,figsize=(10,4.5),layout="constrained")
    for ax,heading,title in zip(axes,[0,8],["Начальная ориентация 0, конечная 0","Начальная ориентация 8, конечная 8"]):
        ax.set_xlim(0,128); ax.set_ylim(128,0); ax.set_aspect("equal")
        ax.plot([64,64],[0,80],color="#5c6570",lw=3)
        ax.plot([64,64],[97,128],color="#5c6570",lw=3)
        direction=1 if heading==0 else -1
        for x,colour,label in [(16,"#148359","Старт"),(112,"#bc3455","Цель")]:
            ax.plot(x,64,"o",color=colour,ms=8)
            ax.annotate("",xy=(x+direction*12,64),xytext=(x,64),
                        arrowprops=dict(arrowstyle="-|>",lw=2.5,color=colour))
            ax.text(x,49,label,ha="center",color=colour,fontweight="bold")
        ax.set_title(title,fontsize=12)
        ax.set_xticks([16,64,112]);ax.set_yticks([64,88]);ax.set_xlabel("Столбец j")
        ax.set_ylabel("Строка i (растёт вниз)")
    fig.suptitle("Стрелки показывают ориентацию робота в двух заданных состояниях\nПуть между ними здесь не нарисован",fontsize=13)
    fig.savefig(folder/"orientations.png",dpi=180);plt.close(fig)

    selected=[(0,0),(0,8),(8,8),(12,0)]
    colours=["#657383","#e69b36","#14859a"]
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout="constrained")
    for row,algo in enumerate(["LBA","Mesh"]):
        for col,phase in enumerate(["steady","cold"]):
            ax=axes[row,col]
            for k,variant in enumerate(["max","kin64","kin256"]):
                actual_phase="steady" if variant=="max" else phase
                records=[get(st,gt,algo,variant,actual_phase) for st,gt in selected]
                values=np.array([r["end_to_end_s"]*1000 for r in records])
                ax.bar(np.arange(4)+(k-1)*.25,values,.23,color=colours[k],label=variant)
                if actual_phase=="steady":
                    low=np.array([r["end_to_end_s_q25"]*1000 for r in records])
                    high=np.array([r["end_to_end_s_q75"]*1000 for r in records])
                    ax.errorbar(np.arange(4)+(k-1)*.25,values,yerr=[values-low,high-values],fmt="none",ecolor="#333333",capsize=2)
            ax.set_xticks(range(4),labels=[f"({st}, {gt})" for st,gt in selected])
            ax.set_xlabel("(Ориентация старта, ориентация цели)")
            ax.set_ylabel("Время solve(), мс")
            ax.set_title(f"{'LBA*' if algo=='LBA' else 'MeshA*'}: "+("таблица уже построена" if phase=="steady" else "с построением таблицы"))
            ax.legend();ax.grid(axis="y",alpha=.15)
    fig.suptitle("Цена кинематической эвристики\nСлева: медианы 7 повторов и межквартильный диапазон; справа: по 1 холодному запуску, база — медиана 7",fontsize=13)
    fig.savefig(folder/"kinematic-results.png",dpi=180);plt.close(fig)

    lines=["# Кинематическая оценка: периодическая решётка примитивов\n\n",
           "## Что означают ориентации\n\n",
           "В состоянии (i,j,theta) theta обозначает ориентацию робота, а не направление линии старт–цель. "
           "В текущем control set на рисунке с i вниз: 0 — вправо, 4 — вниз, 8 — влево, 12 — вверх. "
           "Проверено по прямолинейным примитивам 388…391: смещения (0,1),(1,0),(0,-1),(-1,0). "
           "Запись 0→8 в старых таблицах обозначала пару начальной и требуемой конечной ориентаций, "
           "а не маршрут. Теперь используем обозначение (theta_start,theta_goal). "
           "Всего направлений 16; нечётные индексы не равноотстоят по углу (например theta_1=atan(1/2)).\n\n",
           "## Конструкция и доказательство\n\n",
           "Убираем препятствия и границы реальной карты. Координаты относительно цели берём по модулю P; "
           "ориентацию сохраняем. Абстрактное состояние: ((i-i_G) mod P,(j-j_G) mod P,theta). "
           "Примитив p задаёт переход с тем же смещением и конечной ориентацией, по модулю P, "
           "стоимость равна ControlSet::cost(p), как в поиске. Абстрактная цель — (0,0,theta_G).\n\n",
           "Любое реальное ребро отображается в абстрактное ребро той же стоимости. "
           "Любой реальный путь до цели поэтому отображается в путь абстракции той же стоимости. "
           "Кратчайшее расстояние в абстракции не больше стоимости реального пути — это допустимость. "
           "Для каждого реального ребра s→t непосредственно имеем h_kin(s)≤c(s,t)+h_kin(t), "
           "так как в абстракции можно сначала пройти образ этого ребра. Это согласованность.\n\n",
           "Таблица вычисляется обратной Дейкстрой; направление ребра и смещение обращаются вместе. "
           "Заворачивание координат добавляет короткие пути и ослабляет оценку, но не завышает её. "
           "Это не точное расстояние неограниченной obstacle-free решётки, а её конечная релаксация. "
           "Если P1 делит P2, таблица P2 не слабее P1 благодаря дополнительной проекции по модулю P1. "
           "Не утверждается монотонность для произвольных несвязанных размеров.\n\n",
           "Итог: h=max(beta*d_E,alpha*d_grid,h_kin). Для h_kin не нужна новая alpha: "
           "стоимости примитивов перенесены без изменения. Гарантии относятся к точной арифметике; "
           "в существующем A* g/f остаются float. Поддерживается одна целевая клетка и точный heading; "
           "другие допуски в новом режиме явно отклоняются. Старые режимы не ограничены.\n\n",
           "## Реализация и повторное использование\n\n",
           "`KC_kinematic.hpp/.cpp` содержит таблицу. `--heuristic kinematic --kin-period 64` "
           "включает максимум трёх оценок. CLI строит таблицу заново для каждого алгоритма. "
           "C++ API допускает передачу заранее построенной таблицы в SearchAlgorithm. "
           "Перед повторным использованием проверяются период, heading цели, все переходы и стоимости. "
           "Таблица независима от карты и координат цели; зависит от control set, периода и heading цели. "
           "Изменение карты не требует её перестроения. Grid-поле в эксперименте всё равно строится для каждого solve().\n\n",
           "## Проверки\n\n",
           "Существующие 1 141 362 проверки прошли. Дополнительно 509 828 проверок рёбер, "
           "независимая прямая Дейкстра против обратной, отрицательные координаты/периодичность, "
           "сравнение периодов 8 и 16, проверка несовместимых таблиц, точной цели, "
           "неправильной ориентации в клетке цели, недостижимых задач. Проверены LBA*, LazyLBA*, MeshA*, "
           "построение/переиспользование, восстановленные пути и сравнение стоимости с h=0.\n\n",
           "## Эксперимент\n\n",
           "Та же карта со стеной и проходом, смещённым на 24 клетки, 16 сочетаний ориентаций. "
           "База — текущий max, не евклидова эвристика. Проверены периоды 64 и 256. "
           "672 измерения с готовыми таблицами (7 повторов), 64 отдельных запуска с построением "
           "внутри solve() (по одному), прогрев, 24 замера подготовки таблиц. "
           "Фиксированное перемешивание, seed 20261025. Все пути проверены, для каждой задачи запущен LBA* с h=0.\n\n",
           f"Максимальное расхождение стоимости с h=0: {error:.12g} при допуске 0.001. "
           "Числа итераций базы совпали с предыдущей серией; построение и переиспользование таблиц "
           "дали одинаковые числа итераций.\n\n",
           "### Подготовка одной таблицы\n\n| Период | Число состояний | Массив расстояний | Медиана подготовки¹ |\n|---|---:|---:|---:|\n"]
    for period in [64,256]:
        values=[r["build_s"] for r in prep if r["period"]==period]
        lines.append(f"| {period} | {period*period*16:,} | {period*period*16*8/1048576:g} МиБ | {statistics.median(values)*1000:.2f} мс |\n")
    lines += ["\n¹ По трём замерам для каждой из четырёх целевых ориентаций. Пиковая память больше "
              "указанного массива: при построении есть очередь Дейкстры.\n\n",
              "### Выбранные задачи\n\nВремя solve() в мс: готовая таблица — медиана семи повторов, "
              "с построением — один отдельный запуск. База также включает подготовку grid-поля.\n\n",
              "| Алгоритм | (начальная, конечная) | База max | P64 готова | P64 с построением | P256 готова | P256 с построением |\n",
              "|---|---|---:|---:|---:|---:|---:|\n"]
    for algo in ["LBA","Mesh"]:
        for st,gt in selected:
            values=[get(st,gt,algo,"max")["end_to_end_s"]]+[get(st,gt,algo,v,phase)["end_to_end_s"] for v in ["kin64","kin256"] for phase in ["steady","cold"]]
            lines.append(f"| {algo} | ({st}, {gt}) | "+" | ".join(f"{v*1000:.2f}" for v in values)+" |\n")
    lines += ["\n### Диапазоны ускорения относительно max\n\n",
              "Отношение T_base/T_new > 1 означает ускорение. Для готовых таблиц сравниваются медианы, "
              "для холодных — медиана базы и один запуск нового варианта; холодные значения предварительные.\n\n",
              "| Алгоритм | Период | С готовой таблицей | С построением |\n|---|---|---:|---:|\n"]
    for algo in ["LBA","Mesh"]:
        for variant in ["kin64","kin256"]:
            ranges=[]
            for phase in ["steady","cold"]:
                ratios=[get(st,gt,algo,"max")["end_to_end_s"]/get(st,gt,algo,variant,phase)["end_to_end_s"] for st in HEADINGS for gt in HEADINGS]
                ranges.append(f"{min(ratios):.2f}…{max(ratios):.2f}×")
            lines.append(f"| {algo} | {variant} | {' | '.join(ranges)} |\n")
    lines += ["\nБольший период сохраняет больше информации о взаимном положении, но стоит дороже. "
              "Предвычисление полезно только если его можно повторно использовать или оно окупается "
              "в одном запросе. Время готовой таблицы нельзя выдавать за полную стоимость первого запроса. "
              "Эти 16 задач одной карты не доказывают общего ускорения; следующая проверка — карты "
              "с иной геометрией и честный сценарий повторного использования.\n\n",
              "## Воспроизведение\n\n```sh\nmake -j4 mesh_astar test kinematic-experiment\n"
              "./obj/kinematic_experiment res/kinematic-experiment 7\n"
              "python3 tools/summarize_kinematic.py res/kinematic-experiment\n```\n\n",
              "Один запрос через обычный CLI:\n\n```sh\n./mesh_astar --mode single \\\n"
              "  --map res/wall-experiment/gap-24.map \\\n"
              "  --prim data/base_control_set.txt --mesh data/base_mesh_info.txt \\\n"
              "  --start \"64 16 8\" --goal \"64 112 8\" \\\n"
              "  --heuristic kinematic --kin-period 64 --weight 1 --out-prefix res/kin-demo-\n```\n"]
    (folder/"REPORT.md").write_text("".join(lines))
    sources=["Makefile","tools/kinematic_experiment.cpp","tools/summarize_kinematic.py"]
    sources += [str(p.relative_to(ROOT)) for name in ["src","include","tests"] for p in (ROOT/name).glob('*') if p.is_file()]
    metadata=dict(compiler=subprocess.check_output(["g++","--version"],text=True).splitlines()[0],flags="-std=c++11 -O3 -Wall -Werror -Wextra",
                  seed=20261025,max_cost_difference=error,steady_runs=672,cold_runs=64,
                  source_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources})
    (folder/"metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    print("".join(lines)[-5200:])


if __name__=="__main__":
    main()
