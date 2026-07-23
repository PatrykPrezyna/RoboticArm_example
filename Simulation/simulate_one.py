"""Run a single RoboticArm simulation from command-line arguments.

This wrapper calls the shared logic in sim_core.run_from_cli so the same
single-run path can be used by both JSON-based scripts and the CLI entrypoint.
"""

from __future__ import annotations

import sim_core


if __name__ == "__main__":
    sim_core.run_from_cli()
