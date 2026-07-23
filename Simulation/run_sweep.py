"""Run RoboticArm.mo once per combination of a JSON sweep file, via simulate_one.py.

sweep_input.json has the same shape as sim_input.json, except:

- any scalar leaf other than the servo's tau_stall/max_speed/servo_mass
  (link_length, link_mass, proxy_mass, proxy_length, proxy_z_offset,
  stop_time) may hold a *list* of candidate values instead of a single
  number -- the sweep is the Cartesian product of every such list, all other
  fields held fixed. position_sequence and carry_sequence are always a
  single fixed sequence, not swept.
- a joint's servo is swept by *name*, not by field: put a
  "servo_options": ["SG90", "AD002", ...] list (keys into
  sim_core.SERVO_CATALOG) under "shoulder"/"elbow" instead of a "servo"
  dict. Each name expands to its real tau_stall+max_speed+servo_mass as one
  indivisible unit, so a run never mixes one servo's torque with another
  servo's mass. A plain "servo" dict (with fields that may themselves be
  lists) is still accepted as a fallback and swept per-field as before.

For now the combinations come from this hand-written JSON file; the plan is
to have the system (SysML) model generate/populate it instead.

Usage:
  python examples/RoboticArm/Simulation/run_sweep.py
  python examples/RoboticArm/Simulation/run_sweep.py my_sweep.json
"""

from __future__ import annotations

import itertools
import json
import subprocess
import sys
from pathlib import Path

import sim_core

HERE = Path(__file__).resolve().parent
SIMULATE_ONE = HERE / "simulate_one.py"
OUT_CSV = HERE / "sim_sweep_result.csv"

SERVO_JOINTS = ("shoulder", "elbow")
SERVO_FIELDS = ("tau_stall", "max_speed", "servo_mass")
NON_SERVO_LEAF_SPEC = [
    (path, default, kind) for path, default, kind in sim_core.LEAF_SPEC
    if path[1:2] != ("servo",)
]


def candidates_for(sweep_input: dict, path: tuple[str, ...], default: object, kind: str) -> list:
    try:
        value = sim_core.get_nested(sweep_input, path)
    except KeyError:
        return [default]
    if kind == "float" and isinstance(value, list):
        return value  # this field is being swept
    return [value]  # fixed for every run (includes the list-typed fields)


def servo_candidates(sweep_input: dict, joint: str) -> list[dict]:
    """One dict per candidate servo for `joint`: {name, tau_stall, max_speed, servo_mass}.

    Names in "servo_options" are looked up in sim_core.SERVO_CATALOG and swept
    whole. Falls back to the old per-field sweep of a plain "servo" dict,
    with name="" (not logged) since there's no catalog entry to name it after.
    """
    joint_input = sweep_input.get(joint, {})
    options = joint_input.get("servo_options")
    if options:
        return [{"name": name, **sim_core.SERVO_CATALOG[name]} for name in options]

    servo = joint_input.get("servo", {})
    field_lists = []
    for field in SERVO_FIELDS:
        default = next(
            d for p, d, _k in sim_core.LEAF_SPEC if p == (joint, "servo", field)
        )
        value = servo.get(field, default)
        field_lists.append(value if isinstance(value, list) else [value])
    return [
        {"name": "", "tau_stall": tau_stall, "max_speed": max_speed, "servo_mass": servo_mass}
        for tau_stall, max_speed, servo_mass in itertools.product(*field_lists)
    ]


def main() -> None:
    input_path = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "sweep_input.json"
    sweep_input = json.loads(input_path.read_text(encoding="utf-8"))

    per_field_candidates = [
        candidates_for(sweep_input, path, default, kind)
        for path, default, kind in NON_SERVO_LEAF_SPEC
    ]
    servo_candidates_by_joint = {joint: servo_candidates(sweep_input, joint) for joint in SERVO_JOINTS}

    combos = list(itertools.product(
        *per_field_candidates,
        servo_candidates_by_joint["shoulder"],
        servo_candidates_by_joint["elbow"],
    ))
    print(f"Running {len(combos)} combination(s)...")

    if OUT_CSV.is_file():
        OUT_CSV.unlink()

    for i, combo in enumerate(combos, start=1):
        *field_values, shoulder_servo, elbow_servo = combo
        args = [
            f"{sim_core.cli_flag(path)}={sim_core.format_cli_value(value)}"
            for (path, _default, _kind), value in zip(NON_SERVO_LEAF_SPEC, field_values)
        ]
        servo_by_joint = {"shoulder": shoulder_servo, "elbow": elbow_servo}
        for joint, servo in servo_by_joint.items():
            for field in SERVO_FIELDS:
                args.append(
                    f"{sim_core.cli_flag((joint, 'servo', field))}="
                    f"{sim_core.format_cli_value(servo[field])}"
                )
            if servo["name"]:
                args.append(f"--{joint}-servo-name={servo['name']}")

        label_bits = [f"{j}={s['name']}" for j, s in servo_by_joint.items() if s["name"]]
        label = " ".join(label_bits + args)
        print(f"\n[{i}/{len(combos)}] {label}")
        proc = subprocess.run(
            [sys.executable, str(SIMULATE_ONE), *args, "--output", str(OUT_CSV), "--append"]
        )
        if proc.returncode != 0:
            sys.exit(f"Run {i}/{len(combos)} failed")

    print(f"\nWrote {OUT_CSV} ({len(combos)} row(s))")


if __name__ == "__main__":
    main()
