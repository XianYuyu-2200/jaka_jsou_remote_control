#!/usr/bin/env python3
"""Read-only health check for Gemini 305 + optional JAKA Mini 2 ROS 2 setup.

Run after the Gemini 305 camera driver has started.  This script subscribes to
camera and robot state topics only; it never sends robot motion commands.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from gemini_jaka_test import (
    has_message,
    latest_timestamp_rates,
    run,
    topic_hz,
    topics,
)


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


@dataclass
class DepthSummary:
    frames: int
    valid_percent: float
    median_mm: float | None


def find_gemini305_usb_line(output: str) -> str | None:
    for line in output.splitlines():
        if re.search(r"\bGemini\s*305g?\b", line, flags=re.IGNORECASE):
            return line.strip()
    return None


def gemini305_uses_usb3(usb_line: str | None, tree_output: str) -> bool:
    if not usb_line:
        return False
    identity = re.search(r"Bus\s+(\d+)\s+Device\s+(\d+):", usb_line)
    if not identity:
        return False
    target_bus, target_device = (int(value) for value in identity.groups())
    current_bus: int | None = None
    for line in tree_output.splitlines():
        root = re.match(r"/:\s+Bus\s+(\d+)\.", line)
        if root:
            current_bus = int(root.group(1))
        if current_bus != target_bus:
            continue
        if re.search(rf"\bDev\s+{target_device}\b", line) and re.search(
            r"\b(?:5000M|10000M|20000M)\b", line
        ):
            return True
    return False


def required_camera_topics() -> dict[str, str]:
    return {
        "RGB topic": "/camera/color/image_raw",
        "Depth topic": "/camera/depth/image_raw",
        "RGB camera_info": "/camera/color/camera_info",
        "Depth camera_info": "/camera/depth/camera_info",
    }


def summarize_depth_frames(frames: list[np.ndarray]) -> DepthSummary:
    if not frames:
        return DepthSummary(frames=0, valid_percent=0.0, median_mm=None)

    total = sum(frame.size for frame in frames)
    valid_arrays = [frame[frame > 0] for frame in frames]
    valid_count = sum(values.size for values in valid_arrays)
    nonempty = [values for values in valid_arrays if values.size]
    median_mm = float(statistics.median(np.concatenate(nonempty))) if nonempty else None
    return DepthSummary(
        frames=len(frames),
        valid_percent=100.0 * valid_count / total if total else 0.0,
        median_mm=median_mm,
    )


def evaluate_depth_quality(
    summary: DepthSummary,
    min_valid_percent: float,
    min_distance_mm: float,
    max_distance_mm: float,
) -> tuple[bool, str]:
    distance_ok = (
        summary.median_mm is not None
        and min_distance_mm <= summary.median_mm <= max_distance_mm
    )
    ok = summary.valid_percent >= min_valid_percent and distance_ok
    distance = "none" if summary.median_mm is None else f"{summary.median_mm:.1f} mm"
    detail = (
        f"{summary.frames} frames, center valid {summary.valid_percent:.1f}%, "
        f"median {distance}"
    )
    return ok, detail


def collect_center_depth_frames(
    topic: str,
    frame_count: int,
    timeout_seconds: float,
) -> tuple[list[np.ndarray], str]:
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        return [], str(exc)

    frames: list[np.ndarray] = []
    rclpy.init(args=None)
    node = rclpy.create_node("gemini305_depth_quality_probe")

    def callback(message: Image) -> None:
        if len(frames) >= frame_count or message.encoding not in ("16UC1", "mono16"):
            return
        row_width = message.step // np.dtype(np.uint16).itemsize
        image = np.frombuffer(message.data, dtype=np.uint16).reshape(
            message.height, row_width
        )[:, : message.width]
        y0, y1 = message.height // 4, 3 * message.height // 4
        x0, x1 = message.width // 4, 3 * message.width // 4
        frames.append(image[y0:y1, x0:x1].copy())

    subscription = node.create_subscription(
        Image, topic, callback, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout_seconds
    try:
        while len(frames) < frame_count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    detail = f"received {len(frames)}/{frame_count} depth frames"
    return frames, detail


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-only", action="store_true", help="skip JAKA checks")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds per rate check")
    parser.add_argument("--min-camera-hz", type=float, default=25.0)
    parser.add_argument("--min-jaka-hz", type=float, default=1.0)
    parser.add_argument("--depth-frames", type=int, default=10)
    parser.add_argument("--min-depth-valid-percent", type=float, default=70.0)
    parser.add_argument("--min-distance-mm", type=float, default=40.0)
    parser.add_argument("--max-distance-mm", type=float, default=1000.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict-exit", action="store_true")
    parser.add_argument("--root", default=".")
    args = parser.parse_args()

    checks: list[Check] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append(Check(name, bool(ok), detail.replace("\n", " ")[:300]))

    code, out = run(["ros2", "pkg", "prefix", "orbbec_camera"])
    add("Orbbec ROS package", code == 0, out.strip() or "not found")
    if not args.camera_only:
        code, out = run(["ros2", "pkg", "prefix", "jaka_driver"])
        add("JAKA ROS package", code == 0, out.strip() or "not found")

    _, usb_devices = run(["lsusb"])
    usb_line = find_gemini305_usb_line(usb_devices)
    add("Gemini 305 USB device", usb_line is not None, usb_line or "not found")

    _, usb_tree = run(["lsusb", "-t"])
    usb3 = gemini305_uses_usb3(usb_line, usb_tree)
    add(
        "Gemini 305 USB 3.x link",
        usb3,
        "Gemini 305 is linked at USB 3.x speed" if usb3 else usb_tree.strip()[-200:],
    )

    usage = shutil.disk_usage(Path(args.root).resolve())
    free_gb = usage.free / (1024**3)
    add("Disk free space >= 100 GB", free_gb >= 100.0, f"{free_gb:.1f} GB free")

    current = topics()
    for name, topic in required_camera_topics().items():
        add(name, topic in current, topic if topic in current else "missing")

    source_rates = latest_timestamp_rates(
        Path(args.root).resolve() / "logs",
        max_age_seconds=max(15.0, args.duration * 4),
    )
    for name, topic in (("RGB", "/camera/color/image_raw"), ("Depth", "/camera/depth/image_raw")):
        if topic not in current:
            add(f"{name} rate >= {args.min_camera_hz:g} Hz", False, "topic missing")
            continue
        rate = source_rates.get(name.lower())
        if rate is not None:
            detail = f"{rate:.3f} Hz from hardware timestamps"
        else:
            rate, detail = topic_hz(topic, args.duration)
        add(
            f"{name} rate >= {args.min_camera_hz:g} Hz",
            rate is not None and rate >= args.min_camera_hz,
            detail,
        )

    for name, topic in (
        ("RGB camera_info message", "/camera/color/camera_info"),
        ("Depth camera_info message", "/camera/depth/camera_info"),
    ):
        if topic in current:
            ok, detail = has_message(topic)
            add(name, ok, detail)
        else:
            add(name, False, "topic missing")

    if "/camera/depth/image_raw" in current:
        frames, receive_detail = collect_center_depth_frames(
            "/camera/depth/image_raw",
            max(1, args.depth_frames),
            max(5.0, args.duration * 2),
        )
        summary = summarize_depth_frames(frames)
        depth_detail = (
            f"{receive_detail}, center valid {summary.valid_percent:.1f}%, "
            + ("median none" if summary.median_mm is None else f"median {summary.median_mm:.1f} mm")
        )
        add(
            f"Center depth valid >= {args.min_depth_valid_percent:g}%",
            summary.valid_percent >= args.min_depth_valid_percent,
            depth_detail,
        )
        add(
            f"Median depth in {args.min_distance_mm:g}-{args.max_distance_mm:g} mm",
            summary.median_mm is not None
            and args.min_distance_mm <= summary.median_mm <= args.max_distance_mm,
            depth_detail,
        )
    else:
        add(f"Center depth valid >= {args.min_depth_valid_percent:g}%", False, "topic missing")
        add(
            f"Median depth in {args.min_distance_mm:g}-{args.max_distance_mm:g} mm",
            False,
            "topic missing",
        )

    if not args.camera_only:
        for name, topic in {
            "JAKA joint topic": "/jaka_driver/joint_position",
            "JAKA TCP topic": "/jaka_driver/tool_position",
            "JAKA state topic": "/jaka_driver/robot_states",
        }.items():
            if topic not in current:
                add(name, False, "missing")
                continue
            rate, detail = topic_hz(topic, args.duration)
            add(
                f"{name} rate >= {args.min_jaka_hz:g} Hz",
                rate is not None and rate >= args.min_jaka_hz,
                detail,
            )

    overall = all(check.ok for check in checks)
    if args.json:
        print(
            json.dumps(
                {"overall": overall, "checks": [check.__dict__ for check in checks]},
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        print("Gemini 305 + JAKA Mini 2 read-only test")
        print("=" * 52)
        for index, check in enumerate(checks, start=1):
            print(f"{index:02d}. {check.name} = {check.ok!s} ({check.detail})")
        print("=" * 52)
        print(f"ALL_CHECKS_PASS (summary only) = {overall!s}")
    return 1 if args.strict_exit and not overall else 0


if __name__ == "__main__":
    sys.exit(main())
