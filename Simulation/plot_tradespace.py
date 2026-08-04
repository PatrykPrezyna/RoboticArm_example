"""Plot the cost-vs-move-time trade space from a run_sweep.py results CSV.

Reads a sweep result CSV (shoulder_servo_name / elbow_servo_name / move_time_s
/ success / stop_time columns, as written by sim_core.write_result_row) and
scatter-plots total servo cost (sim_core.SERVO_CATALOG's cost_usd, shoulder +
elbow) against move time.

Combinations that did not succeed within stop_time have move_time_s == 0,
which is not a real duration and would look "instant" on a time axis -- they
are plotted separately, as red X markers pinned at stop_time (the actual
sim run length), with a dashed reference line at that same height. Pass
--hide-failed to drop them from the plot entirely instead.

Usage:
  python examples/RoboticArm/Simulation/plot_tradespace.py
  python examples/RoboticArm/Simulation/plot_tradespace.py my_result.csv --output tradespace.png
  python examples/RoboticArm/Simulation/plot_tradespace.py --hide-failed --output tradespace_finished_only.png
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - exercised when matplotlib is absent
    plt = None

import sim_core

HERE = Path(__file__).resolve().parent
DEFAULT_OUTPUT_DIR = HERE.parent / "output"
DEFAULT_OUTPUT = DEFAULT_OUTPUT_DIR / "tradespace.png"


def default_plot_output(output_dir: Path | None = None) -> Path:
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"tradespace_{timestamp}.png"


def latest_result_csv(output_dir: Path | None = None) -> Path:
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR
    csv_files = sorted(output_dir.glob("sim_result_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No result CSV files found in {output_dir}")
    return csv_files[-1]


def servo_cost(name: str) -> float:
    return sim_core.SERVO_CATALOG[name]["cost_usd"]


def load_rows(csv_path: Path) -> list[dict]:
    with csv_path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pareto_frontier(points: list[tuple[float, float, str]]) -> list[tuple[float, float, str]]:
    """Return the non-dominated points for minimization of cost and time."""
    frontier: list[tuple[float, float, str]] = []
    for point in sorted(points, key=lambda item: (item[0], item[1])):
        cost, move_time, _ = point
        if not frontier or move_time < min(prev_move_time for _, prev_move_time, _ in frontier):
            frontier.append(point)
    return frontier


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", type=Path, default=None,
                         help="Sweep result CSV from run_sweep.py")
    parser.add_argument("--output", type=Path, default=None,
                         help="Image file to write the plot to")
    parser.add_argument("--show-failed", action="store_true",
                         help="Include combinations that did not succeed as red X markers")
    args = parser.parse_args()

    csv_path = args.csv_path
    if csv_path is None:
        try:
            csv_path = latest_result_csv()
        except FileNotFoundError as exc:
            sys.exit(str(exc))

    if not csv_path.is_file():
        sys.exit(f"No such file: {csv_path} (run simulate_all.py or run_single.py first)")

    rows = load_rows(csv_path)
    if not rows:
        sys.exit(f"{args.csv_path} has no data rows")

    ok_cost, ok_time, ok_label = [], [], []
    fail_cost, fail_label = [], []
    stop_time = float(rows[0]["stop_time"])
    skipped_unnamed = 0

    for row in rows:
        shoulder_name = row.get("shoulder_servo_name", "")
        elbow_name = row.get("elbow_servo_name", "")
        if not shoulder_name or not elbow_name:
            skipped_unnamed += 1  # no catalog entry to price this run with
            continue
        cost = servo_cost(shoulder_name) + servo_cost(elbow_name)
        label = f"{shoulder_name}/{elbow_name}"
        success = str(row.get("success", "")).strip().lower() in ("true", "1", "yes")
        if success:
            ok_cost.append(cost)
            ok_time.append(float(row["move_time_s"]))
            ok_label.append(label)
        else:
            fail_cost.append(cost)
            fail_label.append(label)

    if skipped_unnamed:
        print(f"Skipped {skipped_unnamed} row(s) with no servo name (not in SERVO_CATALOG)")

    if plt is None:
        raise SystemExit("matplotlib is required to create the trade-space plot")

    fig, ax = plt.subplots(figsize=(9, 6))

    if ok_cost:
        ax.scatter(ok_cost, ok_time, c="tab:blue", marker="o", s=70, label="Successful", zorder=3)
        frontier_points = pareto_frontier(list(zip(ok_cost, ok_time, ok_label)))
        if frontier_points:
            frontier_costs = [point[0] for point in frontier_points]
            frontier_times = [point[1] for point in frontier_points]
            ax.plot(frontier_costs, frontier_times, color="tab:orange", linestyle="--",
                    linewidth=1.8, marker="o", markersize=5, label="Pareto frontier", zorder=4)
            ax.scatter(frontier_costs, frontier_times, c="tab:orange", marker="o", s=80,
                       edgecolor="black", linewidth=0.6, zorder=5)
        for x, y, label in zip(ok_cost, ok_time, ok_label):
            ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=7)

    if fail_cost and args.show_failed:
        fail_y = [stop_time] * len(fail_cost)
        ax.scatter(fail_cost, fail_y, c="tab:red", marker="x", s=90,
                   label=f"Did not succeed (>= {stop_time:g}s)", zorder=3)
        for x, y, label in zip(fail_cost, fail_y, fail_label):
            ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 4),
                        fontsize=7, color="tab:red")
        ax.axhline(stop_time, color="gray", linestyle="--", linewidth=1, alpha=0.6,
                   label=f"stop_time cutoff ({stop_time:g}s)")

    if fail_cost and not args.show_failed:
        print(f"Hiding {len(fail_cost)} failed combination(s)")

    ax.set_xlabel("Total servo cost, shoulder + elbow [USD]")
    ax.set_ylabel("Move time to finish sequence [s]")
    title = "Servo trade space: cost vs. move time"
    if not args.show_failed:
        title += " (successful only)"
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    output_path = args.output or default_plot_output()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
