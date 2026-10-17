#!/usr/bin/env python3
"""Generate a PX4 SITL airframe from a vehicle spec, so PX4's model of the aircraft matches the sim's.

The airframe runs the spec's px4.baseAirframe (a stock SITL airframe, for its tuning) and then
overrides what the spec knows: rotor positions relative to the centre of gravity, thrust axes and
moment coefficients (CA_ROTOR*), control surfaces (CA_SV_CS*), output channels (PWM_MAIN_FUNC*)
and the hover throttle (MPC_THR_HOVER).

    python3 tools/px4_airframe.py quadplane                         # print the airframe
    python3 tools/px4_airframe.py quadplane --px4 ~/PX4-Autopilot   # install it into a SITL build

After installing, start PX4 with PX4_SIM_MODEL=unity_<name>, e.g. unity_quadplane. The spec is a
name in Assets/StreamingAssets/vehicles or a path to a .json file.
"""

import argparse
import csv
import functools
import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
GRAVITY = 9.81
CHANNELS = 16

# CA_SV_CSn_TYPE values, in order from 1. Same list as SurfaceSpec.Types in VehicleSpec.cs.
SURFACE_TYPES = [
    "left_aileron", "right_aileron", "elevator", "rudder", "left_elevon", "right_elevon",
    "left_v_tail", "right_v_tail", "left_flap", "right_flap", "airbrake", "custom",
    "left_a_tail", "right_a_tail", "single_channel_aileron", "steering_wheel", "left_spoiler", "right_spoiler",
]


def resolve(name):
    p = Path(name)
    if p.suffix == ".json" or "/" in name:
        return p.resolve()
    return REPO / "Assets" / "StreamingAssets" / "vehicles" / f"{name}.json"


@functools.lru_cache(maxsize=None)
def read_table(path):
    """Rows of (throttle, thrust, torque or None) from a thrust table CSV, read once per path.
    utf-8-sig drops the byte order mark Excel writes, as File.ReadAllLines does in Unity."""
    rows = []
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if not row or not row[0].strip() or not (row[0].strip()[0].isdigit() or row[0].strip()[0] in ".-"):
                continue
            vals = [float(c) for c in row]
            rows.append((vals[0], vals[1], vals[2] if len(vals) > 2 else None))
    return rows


def load_table(spec_dir, rotor):
    return read_table(spec_dir / rotor["thrustTable"])


def max_thrust(spec_dir, rotor):
    if rotor.get("thrustTable"):
        return max(r[1] for r in load_table(spec_dir, rotor))
    return rotor["maxThrust"]


def moment_ratio(spec_dir, rotor):
    """|CA_ROTORn_KM|: reaction torque per newton of thrust."""
    if rotor.get("thrustTable"):
        top = max(load_table(spec_dir, rotor), key=lambda r: r[1])
        if top[2] is not None and top[1] > 0:
            return top[2] / top[1]
    return rotor.get("momentRatio", 0.016)


def throttle_for(spec_dir, rotor, thrust):
    """Throttle PX4 must command for this thrust, matching RotorSpec.ThrottleFor. None if out of reach."""
    if rotor.get("thrustTable"):
        # The table is already in PX4's throttle, so idle doesn't apply.
        rows = load_table(spec_dir, rotor)
        if thrust > max(r[1] for r in rows):
            return None
        throttle = rows[0][0]
        for (x0, y0, _), (x1, y1, _) in zip(rows, rows[1:]):
            if y0 <= thrust <= y1:
                throttle = x0 + (x1 - x0) * (thrust - y0) / (y1 - y0) if y1 > y0 else x0
                break
        return min(max(throttle, 0.0), 1.0)
    if thrust > rotor["maxThrust"]:
        return None
    idle = rotor.get("idle", 0.0)
    speed = math.sqrt(thrust / rotor["maxThrust"])
    return min(max((speed - idle) / (1 - idle), 0.0), 1.0)


def is3(v):
    return isinstance(v, list) and len(v) == 3 and all(isinstance(c, (int, float)) for c in v)


def problems(spec, spec_dir):
    """What stops the airframe from being generated: the checks VehicleSpec.Problems makes on the
    fields this script reads. Unity checks the rest of the spec when it loads it."""
    p = []
    if not spec.get("name"):
        p.append("name is empty")
    mass = spec.get("massProperties", {})
    if not mass.get("mass", 0) > 0:
        p.append("massProperties.mass must be positive")
    if not is3(mass.get("cg", [0, 0, 0])):
        p.append("massProperties.cg needs 3 values")

    channels = {}

    def channel(ch, who):
        if not isinstance(ch, int) or not 0 <= ch < CHANNELS:
            p.append(f"{who}: channel must be 0 to {CHANNELS - 1}")
        elif ch in channels:
            p.append(f"{who}: channel {ch} is also used by {channels[ch]}")
        else:
            channels[ch] = who

    rotors = spec.get("rotors", [])
    if not rotors:
        p.append("no rotors")
    for r in rotors:
        who = f"rotor {r.get('name')}"
        channel(r.get("channel", 0), who)
        if not is3(r.get("position")):
            p.append(f"{who}: position needs 3 values")
        axis = r.get("axis", [0, 0, -1])
        if not is3(axis) or sum(c * c for c in axis) < 1e-6:
            p.append(f"{who}: axis needs 3 values, not all zero")
        if r.get("direction") not in ("ccw", "cw"):
            p.append(f"{who}: direction must be ccw or cw")
        if r.get("momentRatio", 0.016) < 0:
            p.append(f"{who}: momentRatio can't be negative")
        if not 0 <= r.get("idle", 0) <= 0.5:
            p.append(f"{who}: idle must be 0 to 0.5")
        if r.get("thrustTable"):
            if r.get("idle", 0) != 0:
                p.append(f"{who}: idle must be 0 with a thrustTable; the table's first row is the thrust armed at zero throttle")
            try:
                rows = load_table(spec_dir, r)
                if len(rows) < 2:
                    p.append(f"{who}: thrustTable {r['thrustTable']}: needs at least 2 rows of throttle,thrust")
                elif any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
                    p.append(f"{who}: thrustTable {r['thrustTable']}: throttle must increase row by row")
            except (OSError, ValueError, IndexError) as e:
                p.append(f"{who}: thrustTable {r['thrustTable']}: {e}")
        elif not r.get("maxThrust", 0) > 0:
            p.append(f"{who}: needs maxThrust or a thrustTable")

    for s in spec.get("controlSurfaces", []):
        who = f"control surface {s.get('name')}"
        channel(s.get("channel", 0), who)
        if s.get("type") not in SURFACE_TYPES:
            p.append(f"{who}: unknown type {s.get('type')} (one of {', '.join(SURFACE_TYPES)})")
        if not is3(s.get("torque")):
            p.append(f"{who}: torque needs 3 values (roll, pitch, yaw)")

    px4 = spec.get("px4", {})
    if not px4.get("baseAirframe"):
        p.append("px4.baseAirframe is empty")
    if not px4.get("autostartId", 0) > 0:
        p.append("px4.autostartId must be positive")
    return p


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v]


def fmt(x):
    return f"{x:.4f}".rstrip("0").rstrip(".") if x != 0 else "0"


def generate(spec, spec_path):
    spec_dir = spec_path.parent
    cg = spec["massProperties"].get("cg", [0, 0, 0])
    rotors = spec.get("rotors", [])
    surfaces = spec.get("controlSurfaces", [])
    base = spec["px4"]["baseAirframe"]
    lines = [
        "#!/bin/sh",
        "#",
        f"# @name Unity {spec['name']}",
        "#",
        f"# Generated by tools/px4_airframe.py from {spec_path.name} in drone-SW-Unity-Sim; edit the",
        "# spec and regenerate rather than editing this file. Tuning comes from the base airframe.",
        "#",
        "",
        f". ${{R}}etc/init.d-posix/airframes/{base}",
        "",
        f"param set-default CA_ROTOR_COUNT {len(rotors)}",
    ]
    funcs = {}
    for i, r in enumerate(rotors):
        p = [r["position"][k] - cg[k] for k in range(3)]
        a = unit(r.get("axis", [0, 0, -1]))
        km = moment_ratio(spec_dir, r) * (1 if r["direction"] == "ccw" else -1)
        lines.append(f"# {r['name']}")
        for key, val in zip(("PX", "PY", "PZ"), p):
            lines.append(f"param set-default CA_ROTOR{i}_{key} {fmt(val)}")
        for key, val in zip(("AX", "AY", "AZ"), a):
            lines.append(f"param set-default CA_ROTOR{i}_{key} {fmt(val)}")
        lines.append(f"param set-default CA_ROTOR{i}_KM {fmt(km)}")
        funcs[r["channel"]] = 101 + i

    lines += ["", f"param set-default CA_SV_CS_COUNT {len(surfaces)}"]
    for j, s in enumerate(surfaces):
        t = s["torque"]
        lines.append(f"# {s['name']}")
        lines.append(f"param set-default CA_SV_CS{j}_TYPE {SURFACE_TYPES.index(s['type']) + 1}")
        for key, val in zip(("TRQ_R", "TRQ_P", "TRQ_Y"), t):
            lines.append(f"param set-default CA_SV_CS{j}_{key} {fmt(val)}")
        funcs[s["channel"]] = 201 + j

    lines.append("")
    for ch in range(CHANNELS):
        lines.append(f"param set-default PWM_MAIN_FUNC{ch + 1} {funcs.get(ch, 0)}")

    lift = [r for r in rotors if unit(r.get("axis", [0, 0, -1]))[2] < -0.7]
    hover = None
    if lift:
        per_rotor = spec["massProperties"]["mass"] * GRAVITY / len(lift)
        needed = [throttle_for(spec_dir, r, per_rotor) for r in lift]
        if all(n is not None for n in needed):
            hover = max(needed)
            lines += ["", f"param set-default MPC_THR_HOVER {fmt(min(max(hover, 0.1), 0.8))}"]
    return "\n".join(lines) + "\n", hover


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("vehicle", help="spec name in Assets/StreamingAssets/vehicles, or a path to a .json file")
    ap.add_argument("--px4", type=Path, help="PX4-Autopilot checkout with a px4_sitl_default build to install into")
    args = ap.parse_args()

    path = resolve(args.vehicle)
    spec = json.loads(path.read_text(encoding="utf-8-sig"))
    bad = problems(spec, path.parent)
    if bad:
        sys.exit(f"{path} has problems, no airframe written:\n  " + "\n  ".join(bad))
    text, hover = generate(spec, path)
    model = f"unity_{spec['name']}"
    filename = f"{spec['px4']['autostartId']}_{model}"

    if hover is None:
        print("warning: the lift rotors can't carry the weight; MPC_THR_HOVER left at the base airframe's value", file=sys.stderr)
    else:
        print(f"hover throttle {hover * 100:.0f}%", file=sys.stderr)

    if not args.px4:
        sys.stdout.write(text)
        return
    airframes = args.px4.expanduser() / "build" / "px4_sitl_default" / "etc" / "init.d-posix" / "airframes"
    if not (airframes / spec["px4"]["baseAirframe"]).exists():
        sys.exit(f"{airframes} has no {spec['px4']['baseAirframe']}; build PX4 SITL first (make px4_sitl)")
    (airframes / filename).write_text(text)
    print(f"installed {airframes / filename}\nstart PX4 with PX4_SIM_MODEL={model}", file=sys.stderr)


if __name__ == "__main__":
    main()
