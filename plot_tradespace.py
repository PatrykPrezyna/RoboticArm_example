"""Create early-design or simulation-based robotic-arm trade-space plots.

Early trade-space analysis uses only component data from a JSON file; it does
not invoke OpenModelica. Simulation analysis reads an existing sweep CSV.

Examples:
  python plot_tradespace.py early
  python plot_tradespace.py early Simple_tradespace/inout.json
  python plot_tradespace.py simulation
  python plot_tradespace.py simulation output/sim_result_20260101_120000.csv
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import itertools
import json
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - exercised when matplotlib is absent
    plt = None


HERE = Path(__file__).resolve().parent
OUTPUT_DIR = HERE / "output"
DEFAULT_EARLY_INPUT = HERE / "Simple_tradespace" / "inout.json"
DEFAULT_SIMULATION_INPUT = HERE / "Simulation" / "full_input.json"


def timestamped_output(prefix: str, suffix: str, output_dir: Path | None = None) -> Path:
    output_dir = output_dir or OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"{prefix}_{timestamp}{suffix}"


def pareto_frontier(points: list[tuple[float, float, str]]) -> list[tuple[float, float, str]]:
    """Return non-dominated points for minimization of both numeric values."""
    frontier: list[tuple[float, float, str]] = []
    best_second = float("inf")
    for point in sorted(points, key=lambda item: (item[0], item[1], item[2])):
        if point[1] < best_second:
            frontier.append(point)
            best_second = point[1]
    return frontier


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def enumerate_early_designs(payload: dict) -> list[dict[str, object]]:
    """Expand component options and calculate total cost and mass.

    Components without options are descriptive and are not included in the
    Cartesian product. ``quantity`` applies one selected option that many times.
    """
    components = payload.get("components")
    if not isinstance(components, list):
        raise KeyError("Early trade-space input must define a 'components' list")

    selectable: list[tuple[str, int, list[dict]]] = []
    for component in components:
        options = component.get("options")
        if not options:
            continue
        name = str(component["name"])
        quantity = int(component.get("quantity", 1))
        if quantity < 1:
            raise ValueError(f"{name}: quantity must be at least 1")
        selectable.append((name, quantity, options))

    if not selectable:
        raise ValueError("Early trade-space input has no selectable component options")

    designs: list[dict[str, object]] = []
    for design_number, options in enumerate(
        itertools.product(*(component[2] for component in selectable)), start=1
    ):
        choices: dict[str, str] = {}
        total_mass_g = 0.0
        total_cost_usd = 0.0
        for (component_name, quantity, _), option in zip(selectable, options):
            properties = option.get("properties", {})
            try:
                mass_g = float(properties["mass"])
                cost_usd = float(properties["cost"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"{component_name}/{option.get('name', '<unnamed>')} must have "
                    "numeric 'mass' and 'cost' properties"
                ) from exc
            choices[component_name] = str(option["name"])
            total_mass_g += quantity * mass_g
            total_cost_usd += quantity * cost_usd

        label = ", ".join(f"{name}: {option}" for name, option in choices.items())
        designs.append(
            {
                "concept_id": f"C{design_number:02d}",
                "design": label,
                "total_cost_usd": round(total_cost_usd, 4),
                "total_mass_g": round(total_mass_g, 4),
                **choices,
            }
        )
    return designs


def write_early_csv(designs: list[dict[str, object]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(designs[0]))
        writer.writeheader()
        writer.writerows(designs)


def require_matplotlib() -> None:
    if plt is None:
        raise SystemExit("matplotlib is required to create a trade-space plot")


def early_pareto_designs(designs: list[dict[str, object]]) -> list[dict[str, object]]:
    points = [
        (
            float(row["total_cost_usd"]),
            float(row["total_mass_g"]),
            str(row["concept_id"]),
        )
        for row in designs
    ]
    frontier_ids = {point[2] for point in pareto_frontier(points)}
    return [row for row in designs if str(row["concept_id"]) in frontier_ids]


def print_early_pareto(designs: list[dict[str, object]]) -> None:
    frontier = early_pareto_designs(designs)
    print(f"Pareto concepts ({len(frontier)}):")
    for row in frontier:
        print(
            f"  {row['concept_id']}: ${float(row['total_cost_usd']):g}, "
            f"{float(row['total_mass_g']):g} g"
        )
        print(f"    {row['design']}")


def plot_early(designs: list[dict[str, object]], output_path: Path, labels: bool = False) -> None:
    require_matplotlib()
    points = [
        (
            float(row["total_cost_usd"]),
            float(row["total_mass_g"]),
            str(row["concept_id"]),
        )
        for row in designs
    ]
    frontier = pareto_frontier(points)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(
        [point[0] for point in points],
        [point[1] for point in points],
        c="tab:blue",
        alpha=0.7,
        s=55,
        label="Candidate design",
        zorder=3,
    )
    if frontier:
        ax.plot(
            [point[0] for point in frontier],
            [point[1] for point in frontier],
            color="tab:orange",
            linestyle="--",
            marker="o",
            label="Pareto frontier",
            zorder=4,
        )
    designs_by_id = {str(row["concept_id"]): row for row in designs}
    for cost, mass, concept_id in points:
        annotation = concept_id
        if labels:
            annotation += f": {designs_by_id[concept_id]['design']}"
        ax.annotate(
            annotation,
            (cost, mass),
            xytext=(5, 4),
            textcoords="offset points",
            fontsize=7 if not labels else 6,
        )

    ax.set_xlabel("Total component cost [USD]")
    ax.set_ylabel("Total component mass [g]")
    ax.set_title("Early robotic-arm trade space: cost vs. mass")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def latest_result_csv(output_dir: Path | None = None) -> Path:
    output_dir = output_dir or OUTPUT_DIR
    csv_files = sorted(output_dir.glob("sim_result_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No simulation result CSV files found in {output_dir}")
    return csv_files[-1]


def load_servo_costs(input_path: Path) -> dict[str, float]:
    catalog = load_json(input_path).get("servo_catalog")
    if not isinstance(catalog, dict):
        raise KeyError(f"{input_path} does not define a 'servo_catalog' object")
    return {name: float(properties["cost_usd"]) for name, properties in catalog.items()}


def load_csv_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def plot_simulation(
    rows: list[dict[str, str]],
    servo_costs: dict[str, float],
    output_path: Path,
    show_failed: bool,
) -> None:
    require_matplotlib()
    successful: list[tuple[float, float, str]] = []
    failed: list[tuple[float, str]] = []
    stop_time = float(rows[0]["stop_time"])

    for row in rows:
        shoulder = row.get("shoulder_servo_name", "")
        elbow = row.get("elbow_servo_name", "")
        if not shoulder or not elbow:
            continue
        try:
            cost = servo_costs[shoulder] + servo_costs[elbow]
        except KeyError as exc:
            raise KeyError(f"Servo {exc.args[0]!r} is missing from the simulation input catalog") from exc
        label = f"{shoulder}/{elbow}"
        success = row.get("success", "").strip().lower() in {"true", "1", "yes"}
        if success:
            successful.append((cost, float(row["move_time_s"]), label))
        else:
            failed.append((cost, label))

    fig, ax = plt.subplots(figsize=(9, 6))
    if successful:
        ax.scatter(
            [point[0] for point in successful],
            [point[1] for point in successful],
            c="tab:blue",
            s=70,
            label="Successful",
            zorder=3,
        )
        frontier = pareto_frontier(successful)
        ax.plot(
            [point[0] for point in frontier],
            [point[1] for point in frontier],
            color="tab:orange",
            linestyle="--",
            marker="o",
            label="Pareto frontier",
            zorder=4,
        )
        for cost, move_time, label in successful:
            ax.annotate(label, (cost, move_time), xytext=(6, 4), textcoords="offset points", fontsize=7)

    if failed and show_failed:
        ax.scatter(
            [point[0] for point in failed],
            [stop_time] * len(failed),
            c="tab:red",
            marker="x",
            s=90,
            label=f"Did not succeed (>= {stop_time:g}s)",
            zorder=3,
        )
        ax.axhline(stop_time, color="gray", linestyle="--", linewidth=1, alpha=0.6)

    ax.set_xlabel("Total servo cost, shoulder + elbow [USD]")
    ax.set_ylabel("Move time to finish sequence [s]")
    ax.set_title("Simulation servo trade space: cost vs. move time")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)

    early = subparsers.add_parser("early", help="Explore component cost and mass without simulation")
    early.add_argument("input", nargs="?", type=Path, default=DEFAULT_EARLY_INPUT)
    early.add_argument("--output", type=Path, default=None, help="PNG plot output path")
    early.add_argument("--csv-output", type=Path, default=None, help="Calculated design CSV output path")
    early.add_argument("--labels", action="store_true", help="Annotate every candidate point")

    simulation = subparsers.add_parser("simulation", help="Plot an existing Modelica sweep result")
    simulation.add_argument("csv_path", nargs="?", type=Path, default=None)
    simulation.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_SIMULATION_INPUT,
        help="Simulation JSON containing servo_catalog",
    )
    simulation.add_argument("--output", type=Path, default=None, help="PNG plot output path")
    simulation.add_argument("--show-failed", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    try:
        if args.mode == "early":
            designs = enumerate_early_designs(load_json(args.input))
            csv_output = args.csv_output or timestamped_output("early_tradespace", ".csv")
            plot_output = args.output or timestamped_output("early_tradespace", ".png")
            write_early_csv(designs, csv_output)
            plot_early(designs, plot_output, labels=args.labels)
            print(f"Evaluated {len(designs)} design(s)")
            print_early_pareto(designs)
            print(f"Wrote {csv_output}")
            print(f"Wrote {plot_output}")
            return

        csv_path = args.csv_path or latest_result_csv()
        rows = load_csv_rows(csv_path)
        if not rows:
            raise ValueError(f"{csv_path} has no data rows")
        plot_output = args.output or timestamped_output("simulation_tradespace", ".png")
        plot_simulation(rows, load_servo_costs(args.input), plot_output, args.show_failed)
        print(f"Wrote {plot_output}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    main()
