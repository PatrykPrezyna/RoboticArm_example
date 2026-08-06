"""Compatibility entry point for the repository-level trade-space plotter."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "plot_tradespace.py"
SPEC = importlib.util.spec_from_file_location("robotic_arm_tradespace", SCRIPT)
if SPEC is None or SPEC.loader is None:
    raise ImportError(f"Could not load {SCRIPT}")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

main = MODULE.main
pareto_frontier = MODULE.pareto_frontier
latest_result_csv = MODULE.latest_result_csv


if __name__ == "__main__":
    main(["simulation", *sys.argv[1:]])
