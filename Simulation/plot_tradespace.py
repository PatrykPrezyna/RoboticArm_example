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
import sys
from pathlib import Path

import matplotlib.pyplot as plt

import sim_core

HERE = Path(__file__).resolve().parent
DEFAULT_CSV = HERE / "sim_sweep_result.csv"
DEFAULT_OUTPUT = HERE  / "tradespace.png"


def servo_cost(name: str) -> float:
    return sim_core.SERVO_CATALOG[name]["cost_usd"]


def load_rows(csv_path: Path) -> list[dict]:
    with csv_path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", type=Path, default=DEFAULT_CSV,
                         help="Sweep result CSV from run_sweep.py")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                         help="Image file to write the plot to")
    parser.add_argument("--hide-failed", action="store_true",
                         help="Drop combinations that didn't finish instead of "
                              "marking them with red X's")
    args = parser.parse_args()

    if not args.csv_path.is_file():
        sys.exit(f"No such file: {args.csv_path} (run run_sweep.py first)")

    rows = load_rows(args.csv_path)
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

    fig, ax = plt.subplots(figsize=(9, 6))

    if ok_cost:
        ax.scatter(ok_cost, ok_time, c="tab:blue", marker="o", s=70, label="Successful", zorder=3)
        for x, y, label in zip(ok_cost, ok_time, ok_label):
            ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=7)

    if fail_cost and not args.hide_failed:
        fail_y = [stop_time] * len(fail_cost)
        ax.scatter(fail_cost, fail_y, c="tab:red", marker="x", s=90,
                   label=f"Did not succeed (>= {stop_time:g}s)", zorder=3)
        for x, y, label in zip(fail_cost, fail_y, fail_label):
            ax.annotate(label, (x, y), textcoords="offset points", xytext=(6, 4),
                        fontsize=7, color="tab:red")
        ax.axhline(stop_time, color="gray", linestyle="--", linewidth=1, alpha=0.6,
                   label=f"stop_time cutoff ({stop_time:g}s)")

    if args.hide_failed and fail_cost:
        print(f"Hiding {len(fail_cost)} failed combination(s)")

    ax.set_xlabel("Total servo cost, shoulder + elbow [USD]")
    ax.set_ylabel("Move time to finish sequence [s]")
    title = "Servo trade space: cost vs. move time"
    if args.hide_failed:
        title += " (successful only)"
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
