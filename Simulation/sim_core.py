"""Shared plumbing for running RoboticArm.mo in OpenModelica from Python.

Used by simulate_one.py (single run from CLI arguments), run_from_json.py
(single run, arguments sourced from a JSON file), and run_sweep.py (many
runs, argument combinations sourced from a JSON file).
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODEL_FILE = HERE / "RoboticArm.mo"
WORK = HERE.parent / "output" / "om_json_run"
RESULT_PREFIX = "json_run_res"

# One entry per leaf input of RoboticArm.RoboticArmSimulation: the path into
# the nested {shoulder, elbow, load, stop_time} structure, its Modelica
# record default, and whether it's a scalar or a fixed-length list.
LEAF_SPEC: list[tuple[tuple[str, ...], object, str]] = [
    (("shoulder", "position_sequence"), [70, 50, 50, 75, 55, 55, 70], "float_list"),
    (("shoulder", "servo", "tau_stall"), 0.44, "float"),
    (("shoulder", "servo", "max_speed"), 394, "float"),
    (("shoulder", "servo", "servo_mass"), 0.055, "float"),
    (("shoulder", "link_length"), 0.220, "float"),
    (("shoulder", "link_mass"), 0.040, "float"),
    (("elbow", "position_sequence"), [-45, -30, -30, 5, -25, -25, -45], "float_list"),
    (("elbow", "servo", "tau_stall"), 0.44, "float"),
    (("elbow", "servo", "max_speed"), 394, "float"),
    (("elbow", "servo", "servo_mass"), 0.055, "float"),
    (("elbow", "link_length"), 0.120, "float"),
    (("elbow", "link_mass"), 0.045, "float"),
    (("load", "proxy_mass"), 0.01, "float"),
    (("load", "proxy_length"), 0.080, "float"),
    (("load", "proxy_z_offset"), 0.040, "float"),
    (("load", "carry_sequence"), [False, True, True, True, False, False], "bool_list"),
    (("stop_time",), 15, "float"),
]

def load_servo_catalog(catalog_path: Path | None = None) -> dict[str, dict[str, object]]:
    """Load the servo catalog from a JSON input file."""
    if catalog_path is None:
        catalog_path = HERE / "full_input.json"

    with catalog_path.open(encoding="utf-8") as handle:
        payload = json.load(handle)

    catalog = payload.get("servo_catalog")
    if not isinstance(catalog, dict):
        raise KeyError(f"{catalog_path} does not define a 'servo_catalog' object")
    return catalog


SERVO_CATALOG: dict[str, dict[str, object]] = load_servo_catalog()


def cli_flag(path: tuple[str, ...]) -> str:
    return "--" + "-".join(path).replace("_", "-")


def dest_name(path: tuple[str, ...]) -> str:
    return "_".join(path)


def get_nested(d: dict, path: tuple[str, ...]) -> object:
    for key in path:
        d = d[key]
    return d


def set_nested(d: dict, path: tuple[str, ...], value: object) -> None:
    for key in path[:-1]:
        d = d.setdefault(key, {})
    d[path[-1]] = value


def format_cli_value(value: object) -> str:
    if isinstance(value, list):
        return ",".join(str(v) for v in value)
    return str(value)


def parse_float_list(text: str) -> list[float]:
    return [float(x) for x in text.split(",")]


def parse_bool_list(text: str) -> list[bool]:
    return [x.strip().lower() in ("true", "1", "yes") for x in text.split(",")]


def find_omc() -> Path:
    home = os.environ.get("OPENMODELICAHOME")
    if home:
        bin_dir = Path(home) / "bin"
        os.environ["PATH"] = str(bin_dir) + os.pathsep + os.environ.get("PATH", "")
        candidate = bin_dir / ("omc.exe" if os.name == "nt" else "omc")
        if candidate.is_file():
            return candidate
    which = shutil.which("omc")
    if which:
        return Path(which)
    sys.exit(
        "OpenModelica 'omc' not found. Install OpenModelica and set "
        "OPENMODELICAHOME, or put omc on PATH."
    )


def to_modelica_literal(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, list):
        return "{" + ", ".join(to_modelica_literal(v) for v in value) + "}"
    if isinstance(value, (int, float)):
        return repr(value)
    raise TypeError(f"Unsupported value for Modelica literal: {value!r}")


def build_record_modifier(name: str, fields: dict) -> str:
    """'{name}(field=value, nested(sub_field=value), ...)' from a nested dict."""
    parts = []
    for key, value in fields.items():
        if isinstance(value, dict):
            parts.append(build_record_modifier(key, value))
        else:
            parts.append(f"{key}={to_modelica_literal(value)}")
    return f"{name}(" + ", ".join(parts) + ")"


def find_column(fieldnames: list[str], var: str) -> str:
    for name in fieldnames:
        key = name.strip().strip("'\"")
        if key == var or key.endswith(var):
            return name
    sys.exit(f"Column '{var}' not found. Columns: {fieldnames}")


def run_simulation(omc: Path, sim_input: dict, stop_time: float) -> Path:
    """Build+simulate RoboticArmSimulation with sim_input as class modifiers.

    Nested-record parameters (servo.tau_stall etc.) can't be changed at
    runtime with -override (OpenModelica rejects it as a non-constant/
    protected binding), so the modified values are baked in at build time
    instead, via a short class definition.
    """
    if WORK.exists():
        shutil.rmtree(WORK)
    WORK.mkdir(parents=True)
    shutil.copy2(MODEL_FILE, WORK / MODEL_FILE.name)

    model_def = (
        "model JsonRun = RoboticArm.RoboticArmSimulation("
        f"{build_record_modifier('_shoulder_input', sim_input['shoulder'])}, "
        f"{build_record_modifier('_elbow_input', sim_input['elbow'])}, "
        f"{build_record_modifier('_load_input', sim_input['load'])});"
    )

    mos = WORK / "run.mos"
    mos.write_text(
        "\n".join(
            [
                f'cd("{WORK.as_posix()}");',
                f'loadFile("{MODEL_FILE.name}");',
                "getErrorString();",
                f'loadString("{model_def}");',
                "getErrorString();",
                f'simulate(JsonRun, stopTime={stop_time}, tolerance=1e-6, '
                f'outputFormat="csv", fileNamePrefix="{RESULT_PREFIX}");',
                "getErrorString();",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    proc = subprocess.run(
        [str(omc), mos.name], cwd=WORK, capture_output=True, text=True
    )
    print(proc.stdout)
    if proc.returncode != 0 or "LOG_SUCCESS" not in proc.stdout:
        sys.stderr.write(proc.stderr)
        sys.exit("Simulation failed -- see omc output above.")

    result_csv = WORK / f"{RESULT_PREFIX}_res.csv"
    if not result_csv.is_file():
        sys.exit(f"Result CSV not found: {result_csv}")
    return result_csv


def parse_result(result_csv: Path) -> tuple[float, float, float]:
    import csv

    with result_csv.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols = list(rows[0].keys())
    last = rows[-1]
    move_time = float(last[find_column(cols, "move_time")])
    shoulder_final = float(last[find_column(cols, "shoulder_actual_angle")])
    elbow_final = float(last[find_column(cols, "elbow_actual_angle")])
    return move_time, shoulder_final, elbow_final


def flatten_for_csv(obj: object, prefix: str = "", out: dict | None = None) -> dict:
    if out is None:
        out = {}
    if isinstance(obj, dict):
        for key, value in obj.items():
            flatten_for_csv(value, f"{prefix}_{key}" if prefix else key, out)
    elif isinstance(obj, list):
        out[prefix] = ";".join(str(v) for v in obj)
    else:
        out[prefix] = obj
    return out


def build_result_row(
    sim_input: dict,
    stop_time: float,
    move_time: float,
    shoulder_final: float = 0.0,
    elbow_final: float = 0.0,
    extra: dict | None = None,
) -> dict:
    flatten = flatten_for_csv(sim_input)
    row: dict[str, object] = {}
    row["success"] = move_time > 0.0
    row["move_time_s"] = round(move_time, 4)
    row.update(flatten)
    row["stop_time"] = stop_time
    if extra:
        row.update(extra)
    row["shoulder_final_deg"] = round(shoulder_final, 3)
    row["elbow_final_deg"] = round(elbow_final, 3)
    return row


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a single RoboticArm simulation")
    for path, default, kind in LEAF_SPEC:
        flag = cli_flag(path)
        if kind == "float":
            parser.add_argument(flag, type=float, default=default)
        elif kind == "float_list":
            parser.add_argument(flag, type=parse_float_list, default=default)
        elif kind == "bool_list":
            parser.add_argument(flag, type=parse_bool_list, default=default)
        else:
            raise ValueError(f"Unknown kind {kind!r} for {flag}")
    parser.add_argument("--shoulder-servo-name", default="",
                         help="Name of the catalog servo used (CSV labeling only)")
    parser.add_argument("--elbow-servo-name", default="",
                         help="Name of the catalog servo used (CSV labeling only)")
    parser.add_argument("--output", type=Path, default=HERE.parent / "output" / "sim_result.csv",
                         help="CSV file to write the result row to")
    parser.add_argument("--append", action="store_true",
                         help="Append the result row instead of overwriting")
    return parser


def build_sim_input(args: argparse.Namespace) -> tuple[dict, float]:
    sim_input: dict = {}
    stop_time = None
    for path, _default, _kind in LEAF_SPEC:
        value = getattr(args, dest_name(path))
        if path == ("stop_time",):
            stop_time = value
        else:
            set_nested(sim_input, path, value)
    if stop_time is None:
        raise ValueError("stop_time was not provided")
    return sim_input, float(stop_time)


def run_from_cli(argv: list[str] | None = None) -> dict:
    args = build_parser().parse_args(argv)
    sim_input, stop_time = build_sim_input(args)

    extra = {}
    if args.shoulder_servo_name:
        extra["shoulder_servo_name"] = args.shoulder_servo_name
    if args.elbow_servo_name:
        extra["elbow_servo_name"] = args.elbow_servo_name

    return write_result_row(sim_input, stop_time, args.output, args.append, extra=extra)


def write_result_row(
    sim_input: dict,
    stop_time: float,
    output_path: Path,
    append: bool,
    extra: dict | None = None,
) -> dict:
    omc = find_omc()
    result_csv = run_simulation(omc, sim_input, stop_time)
    move_time, shoulder_final, elbow_final = parse_result(result_csv)

    row = build_result_row(
        sim_input,
        stop_time,
        move_time,
        shoulder_final,
        elbow_final,
        extra=extra,
    )

    import csv

    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not (append and output_path.is_file())
    mode = "a" if append and output_path.is_file() else "w"
    with output_path.open(mode, newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)

    print(f"move_time = {move_time:.4f} s ({'success' if row['success'] else 'unsuccessful'})")
    return row
