#!/usr/bin/env python3
"""Small reproducible paired benchmark; logs include preprocessing and cleanup.

Run from any directory. No third-party Python packages required.
This is a smoke benchmark, not a representative performance study.
"""
import argparse
import json
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    "user": ("120 50 0", "300 180 4"),
    "euclidean_counterexample": ("488 70 3", "26 507 15"),
}
MODES = ["euclidean", "safe-euclidean", "grid", "max"]
ALGOS = ["LazyLBAstar", "LBAstar", "MeshAstar"]


def read_metrics(path):
    metrics = {}
    for line in path.read_text().splitlines():
        if line.startswith("  ") and ": " in line:
            key, value = line.strip().split(": ", 1)
            try:
                value = float(value)
            except ValueError:
                pass
            metrics[key] = value
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--reference", action="store_true", help="also run h=0 once per case")
    parser.add_argument("--output", type=Path, default=ROOT / "res/grid-smoke")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    records = []
    for case, (start, goal) in CASES.items():
        for repeat in range(args.repeats):
            # Rotate ordering to reduce the effect of warming/thermal drift.
            modes = MODES[repeat % 4:] + MODES[:repeat % 4]
            if repeat == 0 and args.reference:
                modes = ["zero"] + modes
            for mode in modes:
                prefix = out / f"{case}-{mode}-{repeat}-"
                command = [str(ROOT / "mesh_astar"), "--mode", "single",
                           "--map", "maps/Moscow_0_512.map", "--prim", "data/base_control_set.txt",
                           "--mesh", "data/base_mesh_info.txt", "--start", start, "--goal", goal,
                           "--out-prefix", str(prefix), "--weight", "0" if mode == "zero" else "1",
                           "--heuristic", "euclidean" if mode == "zero" else mode]
                print(f"{case}: {mode}, repeat {repeat + 1}", flush=True)
                result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=240)
                (out / f"{case}-{mode}-{repeat}-stdout.txt").write_text(result.stdout + result.stderr)
                result.check_returncode()
                for algo in ALGOS:
                    metrics = read_metrics(Path(str(prefix) + f"result_{algo}.txt"))
                    if metrics["Solved"] != "Yes":
                        raise RuntimeError(f"Unsolved query: {case}, {mode}, {algo}")
                    records.append(dict(case=case, mode=mode, repeat=repeat, algorithm=algo, **metrics))
                # Checkpoint after each completed process.
                (out / "measurements.json").write_text(json.dumps(records, indent=2) + "\n")

    summary = []
    for case in CASES:
        for algo in ALGOS:
            for mode in ["zero"] + MODES:
                group = [r for r in records if (r["case"], r["algorithm"], r["mode"]) == (case, algo, mode)]
                if not group:
                    continue
                summary.append(dict(case=case, algorithm=algo, mode=mode, repetitions=len(group),
                                    **{key: statistics.median(r[key] for r in group)
                                       for key in ["Expansions (Steps)", "Path Cost", "Path Length (m)",
                                                   "Heuristic Preprocessing Time (s)", "Search Time (s)",
                                                   "Total Time (s)", "End-to-end Time (s)"]}))
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Saved {len(records)} algorithm runs in {out}")


if __name__ == "__main__":
    main()
