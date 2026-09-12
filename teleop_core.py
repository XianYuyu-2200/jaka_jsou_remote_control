#!/usr/bin/env python3
"""Hardware-independent core for JAKA leader/follower teleoperation.

The real robot adapter will call the same mapping, limiting and watchdog logic.
This module intentionally has no ROS or JAKA SDK dependency so it can be tested
before the robots arrive.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable


JOINT_COUNT = 6


def _joints(values: Iterable[float], name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != JOINT_COUNT:
        raise ValueError(f"{name} must contain {JOINT_COUNT} values")
    return result


class JointMapper:
    def __init__(self, sign: Iterable[float], scale: Iterable[float]):
        self.sign = _joints(sign, "sign")
        self.scale = _joints(scale, "scale")
        if any(value not in (-1.0, 1.0) for value in self.sign):
            raise ValueError("each joint sign must be +1 or -1")
        if any(value < 0.0 for value in self.scale):
            raise ValueError("joint scale must be non-negative")
        self._leader_zero: list[float] | None = None
        self._follower_zero: list[float] | None = None

    def arm(self, leader_zero: Iterable[float], follower_zero: Iterable[float]) -> None:
        self._leader_zero = _joints(leader_zero, "leader_zero")
        self._follower_zero = _joints(follower_zero, "follower_zero")

    def map(self, leader: Iterable[float]) -> list[float]:
        if self._leader_zero is None or self._follower_zero is None:
            raise RuntimeError("joint mapper is not armed")
        leader_position = _joints(leader, "leader")
        return [
            follower_zero
            + sign * scale * (position - leader_zero)
            for position, leader_zero, follower_zero, sign, scale in zip(
                leader_position,
                self._leader_zero,
                self._follower_zero,
                self.sign,
                self.scale,
            )
        ]


class JointLimiter:
    def __init__(
        self,
        lower: Iterable[float],
        upper: Iterable[float],
        max_velocity: Iterable[float],
        max_acceleration: Iterable[float],
    ):
        self.lower = _joints(lower, "lower")
        self.upper = _joints(upper, "upper")
        self.max_velocity = _joints(max_velocity, "max_velocity")
        self.max_acceleration = _joints(max_acceleration, "max_acceleration")
        for index in range(JOINT_COUNT):
            if self.lower[index] >= self.upper[index]:
                raise ValueError("lower joint limit must be less than upper limit")
            if self.max_velocity[index] <= 0 or self.max_acceleration[index] <= 0:
                raise ValueError("velocity and acceleration limits must be positive")
        self._command: list[float] | None = None
        self._velocity = [0.0] * JOINT_COUNT

    def reset(self, position: Iterable[float]) -> None:
        self._command = _joints(position, "position")
        self._velocity = [0.0] * JOINT_COUNT

    def step(self, desired: Iterable[float], dt: float) -> list[float]:
        if self._command is None:
            raise RuntimeError("joint limiter is not initialized")
        if dt <= 0:
            raise ValueError("dt must be positive")
        desired_position = _joints(desired, "desired")
        for index in range(JOINT_COUNT):
            bounded_desired = min(
                self.upper[index], max(self.lower[index], desired_position[index])
            )
            requested_velocity = (bounded_desired - self._command[index]) / dt
            requested_velocity = min(
                self.max_velocity[index],
                max(-self.max_velocity[index], requested_velocity),
            )
            max_dv = self.max_acceleration[index] * dt
            dv = requested_velocity - self._velocity[index]
            dv = min(max_dv, max(-max_dv, dv))
            self._velocity[index] += dv
            self._command[index] += self._velocity[index] * dt
            self._command[index] = min(
                self.upper[index], max(self.lower[index], self._command[index])
            )
        return list(self._command)


class WatchdogResult(Enum):
    FRESH = auto()
    HOLD = auto()
    FAULT = auto()


class Watchdog:
    def __init__(self, hold_after_ms: float, fault_after_ms: float):
        if hold_after_ms <= 0 or fault_after_ms <= hold_after_ms:
            raise ValueError("watchdog thresholds are invalid")
        self.hold_after_ms = float(hold_after_ms)
        self.fault_after_ms = float(fault_after_ms)
        self._fault_latched = False

    def evaluate(self, age_ms: float) -> WatchdogResult:
        if self._fault_latched or age_ms >= self.fault_after_ms:
            self._fault_latched = True
            return WatchdogResult.FAULT
        if age_ms >= self.hold_after_ms:
            return WatchdogResult.HOLD
        return WatchdogResult.FRESH

    def reset(self) -> None:
        self._fault_latched = False


class ControllerState(Enum):
    STANDBY = auto()
    RUNNING = auto()
    HOLDING = auto()
    FAULT = auto()


@dataclass(frozen=True)
class ControlResult:
    state: ControllerState
    command: list[float]
    command_sent: bool
    reason: str = ""


class TeleopController:
    def __init__(
        self,
        mapper: JointMapper,
        limiter: JointLimiter,
        watchdog: Watchdog,
        period_s: float,
    ):
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        self.mapper = mapper
        self.limiter = limiter
        self.watchdog = watchdog
        self.period_s = float(period_s)
        self.state = ControllerState.STANDBY
        self._last_command = [0.0] * JOINT_COUNT
        self.reason = ""

    def enable(
        self,
        leader: Iterable[float],
        follower: Iterable[float],
        *,
        age_ms: float,
        valid: bool,
        drag: bool,
        deadman: bool,
    ) -> bool:
        if self.state is ControllerState.FAULT:
            return False
        if not valid or not drag or not deadman:
            return False
        if self.watchdog.evaluate(age_ms) is not WatchdogResult.FRESH:
            return False
        leader_position = _joints(leader, "leader")
        follower_position = _joints(follower, "follower")
        self.mapper.arm(leader_position, follower_position)
        self.limiter.reset(follower_position)
        self._last_command = follower_position
        self.state = ControllerState.RUNNING
        self.reason = ""
        return True

    def process(
        self,
        leader: Iterable[float],
        *,
        age_ms: float,
        valid: bool,
        drag: bool,
        deadman: bool,
    ) -> ControlResult:
        if self.state not in (ControllerState.RUNNING, ControllerState.HOLDING):
            return ControlResult(self.state, list(self._last_command), False, self.reason)
        if not deadman:
            return self._fault("deadman released")
        if not valid or not drag:
            return self._fault("leader sample invalid")

        watchdog_result = self.watchdog.evaluate(age_ms)
        if watchdog_result is WatchdogResult.FAULT:
            return self._fault("leader sample timed out")
        if watchdog_result is WatchdogResult.HOLD:
            self.state = ControllerState.HOLDING
            return ControlResult(self.state, list(self._last_command), True)

        mapped_target = self.mapper.map(leader)
        self._last_command = self.limiter.step(mapped_target, self.period_s)
        self.state = ControllerState.RUNNING
        return ControlResult(self.state, list(self._last_command), True)

    def _fault(self, reason: str) -> ControlResult:
        self.state = ControllerState.FAULT
        self.reason = reason
        return ControlResult(self.state, list(self._last_command), False, reason)
