import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ego_mpc_controller import (
    EgoMpcController,
    LinearMpc,
    rebase_ned_position,
    validate_solver_time_limit,
)
from constrained_mpc import ConstrainedLinearMpc, MpcSolveError, MpcSolveStats
from px4_control_watchdog import Px4ControlWatchdog


class LinearMpcSafetyTest(unittest.TestCase):
    def make_mpc(self, **kwargs) -> LinearMpc:
        values = dict(
            dt=0.05,
            control_dt=0.02,
            horizon=20,
            position_weight=12.0,
            velocity_weight=3.0,
            acceleration_weight=0.20,
            jerk_weight=0.50,
            terminal_weight=2.0,
            max_velocity=1.0,
            max_acceleration=2.0,
            max_horizontal_acceleration=1.5,
            max_vertical_acceleration=1.0,
            max_jerk=4.0,
            solver_time_limit_ms=100.0,
        )
        values.update(kwargs)
        return LinearMpc(**values)

    def large_reference(self) -> np.ndarray:
        reference = np.zeros((20, 9), dtype=float)
        reference[:, 0] = 5.0
        reference[:, 1] = 5.0
        return reference

    def test_first_command_is_jerk_limited_from_zero(self) -> None:
        mpc = self.make_mpc()

        command = mpc.solve(np.zeros(6), self.large_reference(), None)

        self.assertLessEqual(np.linalg.norm(command), 4.0 * 0.02 + 1e-9)

    def test_horizontal_limit_is_a_vector_norm(self) -> None:
        mpc = self.make_mpc(max_jerk=100.0)

        command = mpc.solve(np.zeros(6), self.large_reference(), None)

        self.assertLessEqual(np.linalg.norm(command[:2]), 1.5 + 1e-9)

    def test_controller_solver_constrains_future_inputs(self) -> None:
        mpc = self.make_mpc(max_jerk=100.0)

        mpc.solve(np.zeros(6), self.large_reference(), None)

        self.assertEqual(mpc.last_sequence.shape, (20, 3))
        self.assertLessEqual(
            np.max(np.linalg.norm(mpc.last_sequence[:, :2], axis=1)),
            1.5 + 1e-9,
        )

    def test_reset_delta_rebases_ned_reference(self) -> None:
        self.assertEqual(
            rebase_ned_position((10.0, -2.0, 3.0), (0.5, -1.0), 0.25),
            (10.5, -3.0, 3.25),
        )

    def test_solver_time_limit_must_leave_control_cycle_margin(self) -> None:
        self.assertEqual(validate_solver_time_limit(50.0, 18.0), 18.0)
        with self.assertRaises(ValueError):
            validate_solver_time_limit(50.0, 20.0)
        with self.assertRaises(ValueError):
            validate_solver_time_limit(50.0, 21.0)


class ConstrainedLinearMpcTest(unittest.TestCase):
    def make_mpc(self, **kwargs) -> ConstrainedLinearMpc:
        values = dict(
            dt=0.05,
            control_dt=0.02,
            horizon=10,
            position_weight=12.0,
            velocity_weight=3.0,
            acceleration_weight=0.20,
            jerk_weight=0.50,
            terminal_weight=2.0,
            max_velocity=1.0,
            max_acceleration=1.2,
            max_horizontal_acceleration=0.8,
            max_vertical_acceleration=0.6,
            max_jerk=4.0,
            solver_time_limit_ms=100.0,
            solver_constraint_tolerance=5e-4,
        )
        values.update(kwargs)
        return ConstrainedLinearMpc(**values)

    @staticmethod
    def aggressive_reference(horizon: int) -> np.ndarray:
        reference = np.zeros((horizon, 9), dtype=float)
        reference[:, :3] = (5.0, -5.0, 2.0)
        reference[:, 3:6] = (0.7, -0.7, 0.2)
        reference[:, 6:9] = (1.0, -1.0, 0.5)
        return reference

    def test_every_prediction_step_satisfies_hard_limits(self) -> None:
        mpc = self.make_mpc()
        previous = np.array([0.02, -0.01, 0.01])

        command = mpc.solve(
            np.zeros(6),
            self.aggressive_reference(mpc.horizon),
            previous,
        )

        controls = mpc.last_sequence
        predicted = mpc.last_predicted_states
        self.assertTrue(np.allclose(command, controls[0]))
        self.assertLessEqual(np.max(np.linalg.norm(predicted[:, 3:6], axis=1)), 1.0 + 2e-5)
        self.assertLessEqual(np.max(np.linalg.norm(controls, axis=1)), 1.2 + 2e-5)
        self.assertLessEqual(np.max(np.linalg.norm(controls[:, :2], axis=1)), 0.8 + 2e-5)
        self.assertLessEqual(np.max(np.abs(controls[:, 2])), 0.6 + 2e-5)

        differences = np.vstack((controls[0] - previous, np.diff(controls, axis=0)))
        jerk_periods = np.array([mpc.control_dt] + [mpc.dt] * (mpc.horizon - 1))
        jerk = np.linalg.norm(differences, axis=1) / jerk_periods
        self.assertLessEqual(np.max(jerk), 4.0 + 2e-5)
        self.assertLessEqual(mpc.last_stats.max_constraint_violation, 5e-4)

    def test_infeasible_velocity_envelope_raises_solver_error(self) -> None:
        mpc = self.make_mpc(max_velocity=0.1, max_jerk=0.1)
        state = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])

        with self.assertRaises(MpcSolveError):
            mpc.solve(state, np.zeros((mpc.horizon, 9)), np.zeros(3))


class _LoggerStub:
    def warn(self, _message: str) -> None:
        pass


class EgoMpcControllerSafetyTest(unittest.TestCase):
    def test_active_long_trajectory_does_not_expire_by_receive_age(self) -> None:
        controller = EgoMpcController.__new__(EgoMpcController)
        controller.latest_reference = SimpleNamespace(start_time_sec=100.0, duration=10.0)
        controller.latest_reference_sec = 100.0
        controller.command_timeout_sec = 2.0
        controller.now_sec = lambda: 105.0

        self.assertTrue(controller.trajectory_is_fresh())

        controller.now_sec = lambda: 112.001
        self.assertFalse(controller.trajectory_is_fresh())

    def test_px4_reset_rebases_all_stored_targets_and_resets_jerk_state(self) -> None:
        controller = EgoMpcController.__new__(EgoMpcController)
        controller.reference_px4 = (1.0, 2.0, 3.0)
        controller.takeoff_origin = (2.0, 3.0, 4.0)
        controller.takeoff_target = (3.0, 4.0, 5.0)
        controller.takeoff_command = [4.0, 5.0, 6.0]
        controller.hold_position = (5.0, 6.0, 7.0)
        controller.reference_heading = 0.1
        controller.hold_yaw = -0.2
        controller.previous_acceleration = np.ones(3)
        controller.reset_recovery_sec = 0.25
        controller.now_sec = lambda: 20.0
        controller.get_logger = lambda: _LoggerStub()

        controller.apply_px4_reset((0.5, -1.0), 0.25, 0.3)

        self.assertEqual(controller.reference_px4, (1.5, 1.0, 3.25))
        self.assertEqual(controller.takeoff_origin, (2.5, 2.0, 4.25))
        self.assertEqual(controller.takeoff_target, (3.5, 3.0, 5.25))
        self.assertEqual(controller.takeoff_command, [4.5, 4.0, 6.25])
        self.assertEqual(controller.hold_position, (5.5, 5.0, 7.25))
        self.assertAlmostEqual(controller.reference_heading, 0.4)
        self.assertAlmostEqual(controller.hold_yaw, 0.1)
        self.assertIsNone(controller.previous_acceleration)
        self.assertEqual(controller.reset_recovery_until_sec, 20.25)

    def test_solver_error_clears_jerk_state_and_enters_position_hold(self) -> None:
        class FailingSolver:
            def solve(self, _state, _references, _previous):
                raise MpcSolveError("primal infeasible")

        controller = EgoMpcController.__new__(EgoMpcController)
        controller.mpc = FailingSolver()
        controller.previous_acceleration = np.ones(3)
        controller.references_in_ned = lambda _now: np.zeros((20, 9))
        controller.current_state = lambda: np.zeros(6)
        controller.now_sec = lambda: 10.0
        events = []
        controller.publish_hold = lambda reason: events.append(("hold", reason))
        controller.manage_px4_state = lambda: events.append(("manage", ""))
        controller.publish_solver_diagnostics = (
            lambda level, message, force=False: events.append(("diagnostic", level, message, force))
        )

        acceleration = controller.solve_mpc_or_hold()

        self.assertIsNone(acceleration)
        self.assertIsNone(controller.previous_acceleration)
        self.assertIn("primal infeasible", events[0][1])
        self.assertEqual(events[0][0], "hold")
        self.assertEqual(events[1][0], "diagnostic")
        self.assertTrue(events[1][3])
        self.assertEqual(events[2][0], "manage")

    def test_solver_diagnostic_values_include_numerical_health(self) -> None:
        stats = MpcSolveStats(
            status="solved",
            solve_time_ms=3.25,
            iterations=75,
            objective=-12.5,
            primal_residual=1e-5,
            dual_residual=2e-5,
            max_constraint_violation=3e-6,
        )

        values = {
            item.key: item.value for item in EgoMpcController.solver_diagnostic_values(stats)
        }

        self.assertEqual(values["status"], "solved")
        self.assertEqual(values["solve_time_ms"], "3.250")
        self.assertEqual(values["iterations"], "75")
        self.assertEqual(values["objective"], "-12.5")
        self.assertEqual(values["primal_residual"], "1e-05")
        self.assertEqual(values["dual_residual"], "2e-05")
        self.assertEqual(values["max_constraint_violation"], "3e-06")


class Px4ControlWatchdogTest(unittest.TestCase):
    @staticmethod
    def mode(*, position: bool = False, acceleration: bool = False):
        return SimpleNamespace(position=position, acceleration=acceleration)

    @staticmethod
    def setpoint(position, velocity, acceleration):
        return SimpleNamespace(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
        )

    def make_watchdog(self, mode, setpoint) -> Px4ControlWatchdog:
        watchdog = Px4ControlWatchdog.__new__(Px4ControlWatchdog)
        watchdog.latest_mode = mode
        watchdog.latest_setpoint = setpoint
        return watchdog

    def test_acceleration_mode_requires_only_acceleration_fields(self) -> None:
        watchdog = self.make_watchdog(
            self.mode(acceleration=True),
            self.setpoint([math.nan] * 3, [math.nan] * 3, [0.1, 0.2, 0.3]),
        )
        self.assertTrue(watchdog.setpoint_matches_mode())

        watchdog.latest_setpoint.position = [0.0, math.nan, math.nan]
        self.assertFalse(watchdog.setpoint_matches_mode())

    def test_position_mode_rejects_active_acceleration_fields(self) -> None:
        watchdog = self.make_watchdog(
            self.mode(position=True),
            self.setpoint([1.0, 2.0, 3.0], [math.nan] * 3, [0.1, 0.2, 0.3]),
        )
        self.assertFalse(watchdog.setpoint_matches_mode())

    def test_multiple_control_modes_are_rejected(self) -> None:
        watchdog = self.make_watchdog(
            self.mode(position=True, acceleration=True),
            self.setpoint([math.nan] * 3, [math.nan] * 3, [0.1, 0.2, 0.3]),
        )
        self.assertFalse(watchdog.setpoint_matches_mode())


if __name__ == "__main__":
    unittest.main()
