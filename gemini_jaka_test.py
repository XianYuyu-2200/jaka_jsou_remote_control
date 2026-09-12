#!/usr/bin/env python3
"""Read-only health check for Gemini 215 + JAKA Mini 2 ROS 2 setup.

Run after the camera and, for --full mode, the JAKA driver are already started.
The script never sends motion commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


def run(cmd: list[str], timeout: float = 8.0) -> tuple[int, str]:
    try:
        p = subprocess.run(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        return p.returncode, p.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return 124, str(exc)


def topics() -> set[str]:
    code, out = run(["ros2", "topic", "list"], timeout=10)
    if code != 0:
        return set()
    return {line.strip() for line in out.splitlines() if line.strip().startswith("/")}


def find_imu_topic(current: set[str]) -> str | None:
    for candidate in (
        "/camera/gyro_accel/sample",
        "/camera/imu",
        "/camera/accel/sample",
        "/camera/gyro/sample",
    ):
        if candidate in current:
            return candidate
    return None


def rates_from_timestamp_csv(path: Path, sample_limit: int = 1000) -> dict[str, float]:
    with path.open(newline="") as stream:
        rows = deque(csv.DictReader(stream), maxlen=sample_limit)

    result: dict[str, float] = {}
    for stream_name in ("color", "depth"):
        key = f"{stream_name}_global_ts_delta_us"
        deltas: list[float] = []
        for row in rows:
            try:
                value = float(row[key])
            except (KeyError, TypeError, ValueError):
                continue
            if value > 0:
                deltas.append(value)
        if deltas:
            result[stream_name] = 1_000_000.0 / statistics.median(deltas)
    return result


def latest_timestamp_rates(log_dir: Path, max_age_seconds: float = 15.0) -> dict[str, float]:
    files = list(log_dir.glob("camera_timestamps_*.csv"))
    if not files:
        return {}
    latest = max(files, key=lambda item: item.stat().st_mtime)
    if time.time() - latest.stat().st_mtime > max_age_seconds:
        return {}
    return rates_from_timestamp_csv(latest)


def topic_hz(topic: str, seconds: float) -> tuple[float | None, str]:
    try:
        p = subprocess.Popen(
            ["ros2", "topic", "hz", "--window", "20", topic],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        try:
            out, _ = p.communicate(timeout=seconds)
        except subprocess.TimeoutExpired:
            p.kill()
            out, _ = p.communicate()
    except FileNotFoundError as exc:
        return None, str(exc)

    rates = re.findall(r"average rate:\s*([0-9]+(?:\.[0-9]+)?)", out or "")
    if not rates:
        return None, (out or "").strip()[-300:]
    return float(rates[-1]), f"{rates[-1]} Hz"


def has_message(topic: str) -> tuple[bool, str]:
    code, out = run(["ros2", "topic", "echo", "--once", topic], timeout=6)
    if code != 0:
        return False, (out or "no message").strip()[-250:]
    return bool(out.strip()), "message received"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-only", action="store_true", help="skip JAKA checks")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds per rate check")
    parser.add_argument("--min-camera-hz", type=float, default=25.0)
    parser.add_argument("--min-jaka-hz", type=float, default=1.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict-exit", action="store_true", help="return exit code 1 if any check fails")
    parser.add_argument("--root", default=".", help="workspace root used for disk check")
    args = parser.parse_args()

    checks: list[Check] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append(Check(name, bool(ok), detail.replace("\n", " ")[:300]))

    code, out = run(["ros2", "pkg", "prefix", "orbbec_camera"])
    add("Orbbec ROS package", code == 0, out.strip() or "not found")
    if not args.camera_only:
        code, out = run(["ros2", "pkg", "prefix", "jaka_driver"])
        add("JAKA ROS package", code == 0, out.strip() or "not found")

    code, usb = run(["lsusb", "-t"])
    usb3 = bool(re.search(r"\b(?:5000M|10000M|20000M)\b", usb))
    add("USB 3.x link available", usb3, "USB 3.x bus found" if usb3 else usb.strip()[-200:])

    usage = shutil.disk_usage(Path(args.root).resolve())
    free_gb = usage.free / (1024**3)
    add("Disk free space >= 100 GB", free_gb >= 100, f"{free_gb:.1f} GB free")

    current = topics()
    imu_topic = find_imu_topic(current)
    camera_topics = {
        "RGB topic": "/camera/color/image_raw",
        "Depth topic": "/camera/depth/image_raw",
        "RGB camera_info": "/camera/color/camera_info",
        "Depth camera_info": "/camera/depth/camera_info",
    }
    for name, topic in camera_topics.items():
        add(name, topic in current, topic if topic in current else "missing")
    add("IMU topic", imu_topic is not None, imu_topic or "missing")

    source_rates = latest_timestamp_rates(
        Path(args.root).resolve() / "logs",
        max_age_seconds=max(15.0, args.duration * 4),
    )
    for name, topic in [("RGB", "/camera/color/image_raw"), ("Depth", "/camera/depth/image_raw")]:
        if topic in current:
            stream_name = name.lower()
            rate = source_rates.get(stream_name)
            if rate is not None:
                detail = f"{rate:.3f} Hz from hardware timestamps"
            else:
                rate, detail = topic_hz(topic, args.duration)
            add(f"{name} rate >= {args.min_camera_hz:g} Hz", rate is not None and rate >= args.min_camera_hz, detail)
        else:
            add(f"{name} rate >= {args.min_camera_hz:g} Hz", False, "topic missing")

    message_topics = [("Depth camera_info message", "/camera/depth/camera_info")]
    if imu_topic:
        message_topics.append(("IMU message", imu_topic))
    else:
        add("IMU message", False, "topic missing")
    for name, topic in message_topics:
        if topic in current:
            ok, detail = has_message(topic)
            add(name, ok, detail)
        else:
            add(name, False, "topic missing")

    if not args.camera_only:
        jaka_topics = {
            "JAKA joint topic": "/jaka_driver/joint_position",
            "JAKA TCP topic": "/jaka_driver/tool_position",
            "JAKA state topic": "/jaka_driver/robot_states",
        }
        for name, topic in jaka_topics.items():
            if topic not in current:
                add(name, False, "missing")
                continue
            rate, detail = topic_hz(topic, args.duration)
            add(f"{name} rate >= {args.min_jaka_hz:g} Hz", rate is not None and rate >= args.min_jaka_hz, detail)

    overall = all(item.ok for item in checks)
    if args.json:
        print(json.dumps({"overall": overall, "checks": [item.__dict__ for item in checks]}, ensure_ascii=False, indent=2))
    else:
        print("Gemini 215 + JAKA Mini 2 read-only test")
        print("=" * 52)
        for index, item in enumerate(checks, start=1):
            print(f"{index:02d}. {item.name} = {item.ok!s} ({item.detail})")
        print("=" * 52)
        print(f"ALL_CHECKS_PASS (summary only) = {overall!s}")
    return 1 if args.strict_exit and not overall else 0


if __name__ == "__main__":
    sys.exit(main())
