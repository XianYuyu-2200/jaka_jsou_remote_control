#!/usr/bin/env python3
"""Virtual leader/follower demonstration for the hardware-independent core."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from teleop_core import JointLimiter, JointMapper, TeleopController, Watchdog


def simulate(steps: int = 250, deadman_release_step: int | None = None) -> list[dict[str, Any]]:
    if steps < 1:
        raise ValueError("steps must be positive")
    mapper = JointMapper([1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1])
    limiter = JointLimiter(
        [-3.14] * 6,
        [3.14] * 6,
        [0.5] * 6,
        [2.0] * 6,
    )
    controller = TeleopController(mapper, limiter, Watchdog(24.0, 100.0), 0.008)
    leader_zero = [0.0] * 6
    follower_zero = [0.0] * 6
    controller.enable(
        leader_zero,
        follower_zero,
        age_ms=0.0,
        valid=True,
        drag=True,
        deadman=True,
    )

    samples: list[dict[str, Any]] = []
    for step in range(steps):
        t = step * 0.008
        leader = [0.15 * math.sin(2.0 * math.pi * 0.4 * t + joint * 0.2) for joint in range(6)]
        deadman = deadman_release_step is None or step < deadman_release_step
        result = controller.process(
            leader,
            age_ms=0.0,
            valid=True,
            drag=True,
            deadman=deadman,
        )
        samples.append(
            {
                "step": step,
                "time_s": t,
                "leader": leader,
                "command": result.command,
                "command_sent": result.command_sent,
                "state": result.state.name,
                "reason": result.reason,
            }
        )
    return samples


def write_jsonl(samples: list[dict[str, Any]], path: str | Path) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as stream:
        for sample in samples:
            stream.write(json.dumps(sample, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    result = simulate(steps=250)
    write_jsonl(result, "bags/virtual_teleop.jsonl")
    print(f"wrote {len(result)} samples to bags/virtual_teleop.jsonl")
