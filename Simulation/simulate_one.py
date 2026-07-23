"""Run one RoboticArm.mo simulation from command-line arguments.

This is the actual "runner" -- run_from_json.py and run_sweep.py both just
translate their own input format into arguments for this script. Writes a
single-row CSV with the inputs and the result side by side.

Usage:
  python examples/RoboticArm/Simulation/simulate_one.py --help
  python examples/RoboticArm/Simulation/simulate_one.py \
      --shoulder-servo-tau-stall 0.92 --elbow-servo-tau-stall 5.88
"""

from __future__ import annotations

import argparse
from pathlib import Path

import sim_core

DEFAULT_OUTPUT = sim_core.HERE.parent / "output" / "sim_result.csv"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    for path, default, kind in sim_core.LEAF_SPEC:
        flag = sim_core.cli_flag(path)
        if kind == "float":
            parser.add_argument(flag, type=float, default=default)
        elif kind == "float_list":
            parser.add_argument(flag, type=sim_core.parse_float_list, default=default)
        elif kind == "bool_list":
            parser.add_argument(flag, type=sim_core.parse_bool_list, default=default)
        else:
            raise ValueError(f"Unknown kind {kind!r} for {flag}")
    parser.add_argument("--shoulder-servo-name", default="",
                         help="Name of the catalog servo used (CSV labeling only, "
                              "not fed into the model; see run_sweep.py)")
    parser.add_argument("--elbow-servo-name", default="",
                         help="Name of the catalog servo used (CSV labeling only, "
                              "not fed into the model; see run_sweep.py)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                         help="CSV file to write the result row to")
    parser.add_argument("--append", action="store_true",
                         help="Append the result row instead of overwriting "
                              "(used by run_sweep.py to collect many rows)")
    return parser


def main() -> None:
    args = build_parser().parse_args()

    sim_input: dict = {}
    stop_time = None
    for path, _default, _kind in sim_core.LEAF_SPEC:
        value = getattr(args, sim_core.dest_name(path))
        if path == ("stop_time",):
            stop_time = value
        else:
            sim_core.set_nested(sim_input, path, value)

    extra = {}
    if args.shoulder_servo_name:
        extra["shoulder_servo_name"] = args.shoulder_servo_name
    if args.elbow_servo_name:
        extra["elbow_servo_name"] = args.elbow_servo_name

    sim_core.write_result_row(sim_input, stop_time, args.output, args.append, extra=extra)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
