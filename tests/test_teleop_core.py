import math
import unittest

from teleop_core import (
    ControllerState,
    JointLimiter,
    JointMapper,
    TeleopController,
    Watchdog,
    WatchdogResult,
)
from teleop_sim import simulate


class JointMapperTests(unittest.TestCase):
    def test_zero_delta_preserves_follower_zero(self):
        mapper = JointMapper([1] * 6, [1] * 6)
        mapper.arm([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6])
        self.assertEqual(
            mapper.map([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
            [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
        )

    def test_sign_and_scale_apply_to_leader_delta(self):
        mapper = JointMapper([1, -1, 1, -1, 1, -1], [1, 1, 0.5, 0.5, 2, 2])
        mapper.arm([0] * 6, [1] * 6)
        self.assertEqual(mapper.map([0.1, 0.1, 0.2, 0.2, 0.1, 0.1]), [1.1, 0.9, 1.1, 0.9, 1.2, 0.8])

    def test_rejects_invalid_configuration(self):
        with self.assertRaises(ValueError):
            JointMapper([0, 1, 1, 1, 1, 1], [1] * 6)
        with self.assertRaises(ValueError):
            JointMapper([1] * 6, [-1, 1, 1, 1, 1, 1])


class JointLimiterTests(unittest.TestCase):
    def test_acceleration_and_velocity_limit_first_step(self):
        limiter = JointLimiter([-3] * 6, [3] * 6, [0.5] * 6, [2.0] * 6)
        limiter.reset([0] * 6)
        command = limiter.step([1] * 6, 0.008)
        self.assertTrue(all(math.isclose(value, 0.000128, abs_tol=1e-12) for value in command))

    def test_never_exceeds_position_limits(self):
        limiter = JointLimiter([-1] * 6, [1] * 6, [10] * 6, [100] * 6)
        limiter.reset([0.99] * 6)
        command = [0] * 6
        for _ in range(100):
            command = limiter.step([2] * 6, 0.008)
        self.assertTrue(all(value <= 1.0 for value in command))


class WatchdogTests(unittest.TestCase):
    def test_fresh_hold_fault_thresholds(self):
        watchdog = Watchdog(24.0, 100.0)
        self.assertEqual(watchdog.evaluate(10.0), WatchdogResult.FRESH)
        self.assertEqual(watchdog.evaluate(30.0), WatchdogResult.HOLD)
        self.assertEqual(watchdog.evaluate(101.0), WatchdogResult.FAULT)

    def test_fault_is_latched_until_reset(self):
        watchdog = Watchdog(24.0, 100.0)
        self.assertEqual(watchdog.evaluate(101.0), WatchdogResult.FAULT)
        self.assertEqual(watchdog.evaluate(1.0), WatchdogResult.FAULT)
        watchdog.reset()
        self.assertEqual(watchdog.evaluate(1.0), WatchdogResult.FRESH)


class TeleopControllerTests(unittest.TestCase):
    def make_controller(self):
        return TeleopController(
            mapper=JointMapper([1] * 6, [1] * 6),
            limiter=JointLimiter([-3] * 6, [3] * 6, [1] * 6, [5] * 6),
            watchdog=Watchdog(24.0, 100.0),
            period_s=0.008,
        )

    def test_enable_requires_fresh_valid_drag_and_deadman(self):
        controller = self.make_controller()
        self.assertFalse(controller.enable([0] * 6, [0] * 6, age_ms=10, valid=True, drag=True, deadman=False))
        self.assertEqual(controller.state, ControllerState.STANDBY)
        self.assertTrue(controller.enable([0] * 6, [0] * 6, age_ms=10, valid=True, drag=True, deadman=True))
        self.assertEqual(controller.state, ControllerState.RUNNING)

    def test_process_generates_limited_command(self):
        controller = self.make_controller()
        controller.enable([0] * 6, [0] * 6, age_ms=0, valid=True, drag=True, deadman=True)
        result = controller.process([1] * 6, age_ms=0, valid=True, drag=True, deadman=True)
        self.assertTrue(result.command_sent)
        self.assertAlmostEqual(result.command[0], 0.00032, places=9)
        self.assertEqual(controller.state, ControllerState.RUNNING)

    def test_deadman_release_faults_and_stops_command(self):
        controller = self.make_controller()
        controller.enable([0] * 6, [0] * 6, age_ms=0, valid=True, drag=True, deadman=True)
        result = controller.process([0.2] * 6, age_ms=0, valid=True, drag=True, deadman=False)
        self.assertFalse(result.command_sent)
        self.assertEqual(controller.state, ControllerState.FAULT)

    def test_stale_leader_holds_then_faults(self):
        controller = self.make_controller()
        controller.enable([0] * 6, [0] * 6, age_ms=0, valid=True, drag=True, deadman=True)
        hold = controller.process([0.2] * 6, age_ms=30, valid=True, drag=True, deadman=True)
        self.assertEqual(controller.state, ControllerState.HOLDING)
        self.assertTrue(hold.command_sent)
        fault = controller.process([0.2] * 6, age_ms=101, valid=True, drag=True, deadman=True)
        self.assertEqual(controller.state, ControllerState.FAULT)
        self.assertFalse(fault.command_sent)


class SimulationTests(unittest.TestCase):
    def test_simulation_produces_commanded_samples(self):
        samples = simulate(steps=20)
        self.assertEqual(len(samples), 20)
        self.assertTrue(all(sample["command_sent"] for sample in samples))
        self.assertEqual(samples[0]["state"], "RUNNING")
        self.assertEqual(len(samples[-1]["command"]), 6)

    def test_simulation_faults_when_deadman_is_released(self):
        samples = simulate(steps=20, deadman_release_step=5)
        self.assertEqual(samples[5]["state"], "FAULT")
        self.assertFalse(samples[5]["command_sent"])
        self.assertTrue(all(sample["state"] == "FAULT" for sample in samples[5:]))


if __name__ == "__main__":
    unittest.main()
