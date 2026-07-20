#!/usr/bin/env python3
"""
configure_servos.py — SO-101 Feetech servo bring-up tool for OmniBot.

The SO-101 arm driver (`arm_driver_node.py`) assumes each of the six
Feetech STS3215 servos already has a unique bus ID (1..6) and a known
home ("zero") tick position (see `config/arm_params.yaml`). Fresh servos
ship with the **same** factory ID (usually 1), so before the arm driver
can talk to all six you have to:

  1. Assign a unique ID to each servo — one servo on the bus at a time.
  2. Record each servo's home tick value with the arm in its zero pose.
  3. Verify the sign/direction of each joint.

This tool automates those three steps. It mirrors the hardware interface
used by `arm_driver_node.py` (LeRobot's ``FeetechMotorsBus``) and falls
back to a **simulation mode** when ``lerobot`` is not installed, so the
workflow — and the emitted YAML — can be exercised without hardware.

Usage
-----
    # 1. Discover which IDs are currently live on the bus
    python configure_servos.py scan --port /dev/ttyACM0

    # 2. Assign IDs — connect ONE servo, repeat for each joint (1..6)
    python configure_servos.py set-id --port /dev/ttyACM0 --from 1 --to 3

    # 3. Put the arm in its zero pose, then capture home ticks
    python configure_servos.py home --port /dev/ttyACM0

    # 4. Nudge one joint a few ticks to confirm positive direction
    python configure_servos.py jog --port /dev/ttyACM0 --id 3 --delta 200

The ``home`` command prints a ready-to-paste ``home_ticks:`` block for
``config/arm_params.yaml``.
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, List, Sequence

# ---------------------------------------------------------------------------
# Joint layout — must match config/arm_params.yaml and the URDF joint names.
# ---------------------------------------------------------------------------
JOINT_NAMES: List[str] = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]
DEFAULT_IDS: List[int] = [1, 2, 3, 4, 5, 6]
SERVO_MODEL = "sts3215"
TICKS_PER_REV = 4096  # STS3215: 4096 ticks == 360 degrees
ID_MIN, ID_MAX = 1, 253  # Feetech protocol addressable range

# ---------------------------------------------------------------------------
# Optional LeRobot import (same guard as arm_driver_node.py)
# ---------------------------------------------------------------------------
try:
    from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

    LEROBOT_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on environment
    LEROBOT_AVAILABLE = False


# ---------------------------------------------------------------------------
# Pure helpers (no hardware, unit-tested)
# ---------------------------------------------------------------------------


def validate_id(servo_id: int) -> int:
    """Return ``servo_id`` if it is a legal Feetech bus ID, else raise."""
    if not ID_MIN <= servo_id <= ID_MAX:
        raise ValueError(
            f"servo id {servo_id} out of range [{ID_MIN}, {ID_MAX}]"
        )
    return servo_id


def joint_for_id(servo_id: int) -> str:
    """Best-effort human label for a target ID (falls back to id N)."""
    if servo_id in DEFAULT_IDS:
        return JOINT_NAMES[DEFAULT_IDS.index(servo_id)]
    return f"id {servo_id}"


def home_ticks_yaml(positions: Sequence[int]) -> str:
    """
    Render captured tick positions as a YAML ``home_ticks:`` block that can
    be pasted straight into ``config/arm_params.yaml``.
    """
    if len(positions) != len(JOINT_NAMES):
        raise ValueError(
            f"expected {len(JOINT_NAMES)} positions, got {len(positions)}"
        )
    ticks = ", ".join(str(int(p)) for p in positions)
    lines = ["    # Captured by configure_servos.py home", f"    home_ticks: [{ticks}]"]
    for name, pos in zip(JOINT_NAMES, positions):
        lines.append(f"    #   {name:18s} = {int(pos)}")
    return "\n".join(lines)


def clamp_delta(delta: int, limit: int = TICKS_PER_REV // 4) -> int:
    """Clamp a manual jog delta so a typo can't slam a joint to its stop."""
    return max(-limit, min(limit, int(delta)))


# ---------------------------------------------------------------------------
# Hardware helpers (guarded; sim fallback when lerobot is absent)
# ---------------------------------------------------------------------------


def _connect(port: str, ids: Sequence[int]):
    """Connect a FeetechMotorsBus for the given ids, or None in sim mode."""
    if not LEROBOT_AVAILABLE:
        print("[sim] lerobot not installed — no hardware will be touched.")
        return None
    motors = {joint_for_id(i): (i, SERVO_MODEL) for i in ids}
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    return bus


def cmd_scan(args: argparse.Namespace) -> int:
    """List the servo IDs that respond on the bus."""
    print(f"Scanning {args.port} for servos {ID_MIN}..{args.scan_max} ...")
    if not LEROBOT_AVAILABLE:
        print("[sim] would ping each ID and report which reply.")
        print(f"[sim] expected final layout: {dict(zip(JOINT_NAMES, DEFAULT_IDS))}")
        return 0
    found: List[int] = []
    for servo_id in range(ID_MIN, args.scan_max + 1):
        bus = None
        try:
            bus = FeetechMotorsBus(
                port=args.port, motors={"probe": (servo_id, SERVO_MODEL)}
            )
            bus.connect()
            bus.read("Present_Position")
            found.append(servo_id)
            print(f"  ✓ id {servo_id} responded")
        except Exception:
            pass
        finally:
            if bus is not None:
                try:
                    bus.disconnect()
                except Exception:
                    pass
    print(f"Found {len(found)} servo(s): {found}")
    return 0


def cmd_set_id(args: argparse.Namespace) -> int:
    """Rewrite a single servo's bus ID. Connect exactly ONE servo first."""
    validate_id(args.from_id)
    validate_id(args.to_id)
    print(
        f"Reassigning the servo currently at id {args.from_id} "
        f"→ id {args.to_id} ({joint_for_id(args.to_id)})."
    )
    print("Make sure ONLY this servo is connected to the bus.")
    if not LEROBOT_AVAILABLE:
        print("[sim] would write register 'ID' = "
              f"{args.to_id} on the servo at id {args.from_id}.")
        return 0
    bus = _connect(args.port, [args.from_id])
    try:
        # Rename the addressed motor to the new id via the ID register.
        bus.write("ID", {joint_for_id(args.from_id): args.to_id})
        print(f"  ✓ servo is now id {args.to_id}. Power-cycle to take effect.")
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass
    return 0


def cmd_home(args: argparse.Namespace) -> int:
    """Read every servo's current tick and emit a home_ticks YAML block."""
    print("Hold the arm in its zero pose, then reading positions ...")
    if not LEROBOT_AVAILABLE:
        centre = TICKS_PER_REV // 2
        positions = [centre] * len(JOINT_NAMES)
        print("[sim] no hardware — emitting centred defaults.")
    else:
        bus = _connect(args.port, DEFAULT_IDS)
        try:
            reading: Dict[str, int] = bus.read("Present_Position")
            positions = [int(reading[joint_for_id(i)]) for i in DEFAULT_IDS]
        finally:
            try:
                bus.disconnect()
            except Exception:
                pass
    print("\nPaste into config/arm_params.yaml (under ros__parameters):\n")
    print(home_ticks_yaml(positions))
    return 0


def cmd_jog(args: argparse.Namespace) -> int:
    """Nudge one joint by a bounded delta to confirm its positive direction."""
    validate_id(args.id)
    delta = clamp_delta(args.delta)
    print(f"Jogging {joint_for_id(args.id)} (id {args.id}) by {delta} ticks.")
    if not LEROBOT_AVAILABLE:
        print("[sim] would read Present_Position, add the delta, and write it back.")
        return 0
    bus = _connect(args.port, [args.id])
    try:
        name = joint_for_id(args.id)
        cur = int(bus.read("Present_Position")[name])
        target = cur + delta
        bus.write("Goal_Position", {name: target})
        print(f"  ✓ {name}: {cur} → {target} (positive delta should move +joint).")
    finally:
        try:
            bus.disconnect()
        except Exception:
            pass
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="SO-101 Feetech servo bring-up tool for OmniBot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("scan", help="List servo IDs currently on the bus.")
    s.add_argument("--port", default="/dev/ttyACM0")
    s.add_argument("--scan-max", type=int, default=ID_MAX, dest="scan_max")
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("set-id", help="Reassign one servo's bus ID.")
    s.add_argument("--port", default="/dev/ttyACM0")
    s.add_argument("--from", type=int, required=True, dest="from_id")
    s.add_argument("--to", type=int, required=True, dest="to_id")
    s.set_defaults(func=cmd_set_id)

    s = sub.add_parser("home", help="Capture home ticks in the zero pose.")
    s.add_argument("--port", default="/dev/ttyACM0")
    s.set_defaults(func=cmd_home)

    s = sub.add_parser("jog", help="Nudge one joint to verify direction.")
    s.add_argument("--port", default="/dev/ttyACM0")
    s.add_argument("--id", type=int, required=True)
    s.add_argument("--delta", type=int, default=200)
    s.set_defaults(func=cmd_jog)

    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
