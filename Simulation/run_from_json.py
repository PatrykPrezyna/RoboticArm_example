"""Run one RoboticArm.mo simulation using inputs from a JSON file.

Reads sim_input.json (shoulder/elbow/load parameters), translates it into
command-line arguments, and hands off to simulate_one.py to actually run
the simulation and write the result CSV.

Usage:
  python examples/RoboticArm/Simulation/run_from_json.py
  python examples/RoboticArm/Simulation/run_from_json.py my_input.json
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import sim_core

HERE = Path(__file__).resolve().parent
SIMULATE_ONE = HERE / "simulate_one.py"


def args_from_json(sim_input: dict) -> list[str]:
    args = []
    for path, default, _kind in sim_core.LEAF_SPEC:
        try:
            value = sim_core.get_nested(sim_input, path)
        except KeyError:
            value = default
        # "--flag=value" (not two separate tokens) so negative numbers in
        # position_sequence don't get misparsed as another flag.
        args.append(f"{sim_core.cli_flag(path)}={sim_core.format_cli_value(value)}")
    return args


def main() -> None:
    input_path = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "sim_input.json"
    sim_input = json.loads(input_path.read_text(encoding="utf-8"))

    argv = [sys.executable, str(SIMULATE_ONE), *args_from_json(sim_input)]
    proc = subprocess.run(argv)
    sys.exit(proc.returncode)


if __name__ == "__main__":
    main()
