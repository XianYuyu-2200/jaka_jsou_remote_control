#!/usr/bin/env python3
"""NUC-compatible dry-run process for the JAKA dual-arm teleop interface.

This process intentionally does not import ROS 2 or the JAKA SDK. It exercises
the same control core at a 125 Hz logical rate and writes the records that the
real ROS 2 node will later publish/record.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleop_core import JointLimiter, JointMapper, TeleopController, Watchdog


PERIOD_S = 0.008


def _controller() -> TeleopController:
    return TeleopController(
        mapper=JointMapper([1] * 6, [1] * 6),
        limiter=JointLimiter(
            lower=[-3.14] * 6,
            upper=[3.14] * 6,
            max_velocity=[0.5] * 6,
            max_acceleration=[2.0] * 6,
        ),
        watchdog=Watchdog(24.0, 100.0),
        period_s=PERIOD_S,
    )


def run_dryrun(
    steps: int,
    output_path: str | Path,
    *,
    deadman_release_step: int | None = None,
    realtime: bool = False,
) -> list[dict[str, Any]]:
    if steps < 1:
        raise ValueError("steps must be positive")
    controller = _controller()
    zero = [0.0] * 6
    if not controller.enable(
        zero,
        zero,
        age_ms=0.0,
        valid=True,
        drag=True,
        deadman=True,
    ):
        raise RuntimeError("failed to arm dry-run controller")

    records: list[dict[str, Any]] = []
    start = time.monotonic()
    for step in range(steps):
        leader = [
            0.15 * math.sin(2.0 * math.pi * 0.4 * step * PERIOD_S + joint * 0.2)
            for joint in range(6)
        ]
        deadman = deadman_release_step is None or step < deadman_release_step
        result = controller.process(
            leader,
            age_ms=0.0,
            valid=True,
            drag=True,
            deadman=deadman,
        )
        record = {
            "sequence": step,
            "monotonic_ns": time.monotonic_ns(),
            "period_s": PERIOD_S,
            "leader_position": leader,
            "command": result.command,
            "command_sent": result.command_sent,
            "state": result.state.name,
            "reason": result.reason,
            "deadman": deadman,
        }
        records.append(record)
        if realtime:
            deadline = start + (step + 1) * PERIOD_S
            time.sleep(max(0.0, deadline - time.monotonic()))

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=1250)
    parser.add_argument("--output", default="bags/nuc_teleop_dryrun.jsonl")
    parser.add_argument("--deadman-release-step", type=int, default=None)
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()
    records = run_dryrun(
        args.steps,
        args.output,
        deadman_release_step=args.deadman_release_step,
        realtime=args.realtime,
    )
    print(f"wrote {len(records)} dry-run records to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
