"""Exercise startup ordering and reset cleanup without hardware."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import reset_follower_home as runner


class ResetTests(unittest.TestCase):
    def setUp(self):
        self.order = Mock()
        bus = SimpleNamespace(connect=self.order.connect, disable_torque=self.order.disable,
                              sync_read=self.order.read, sync_write=self.order.seed, write=Mock())
        self.order.read.return_value = {"shoulder_lift": 860, "gripper": 1836}
        self.robot = SimpleNamespace(bus=bus, is_calibrated=True, configure=self.order.enable,
                                     get_observation=self.order.observe)
        self.order.observe.return_value = {"gripper.pos": 4.78}
        self.home = {"gripper.pos": 4.777}

    def run_reset(self):
        with redirect_stdout(io.StringIO()):
            runner.reset_and_hold(self.robot, self.home, 3, 50, 2, self.order.hold)

    def test_seed_precedes_enable_then_reset_hold_and_cleanup(self):
        with patch.object(runner.RolloutStrategy, "return_to_initial_position", self.order.reset), \
             patch.object(runner, "finish_partial_robot", self.order.cleanup):
            self.run_reset()
        self.assertEqual([c[0] for c in self.order.mock_calls],
                         ["connect", "disable", "read", "seed", "enable", "reset", "observe", "hold", "cleanup"])
        self.order.seed.assert_called_once_with("Goal_Position", self.order.read.return_value, normalize=False)
        args = self.order.reset.call_args
        self.assertEqual(args.args[0].initial_position, self.home)
        self.assertEqual(args.kwargs, {"duration_s": 3, "fps": 50})

    def test_calibration_mismatch_never_enables_or_sends_targets(self):
        self.robot.is_calibrated = False
        with patch.object(runner, "finish_partial_robot", self.order.cleanup):
            with self.assertRaisesRegex(RuntimeError, "calibration"):
                self.run_reset()
        self.order.seed.assert_not_called()
        self.order.enable.assert_not_called()
        self.order.cleanup.assert_called_once_with(self.robot)

    def test_failed_reset_cleans_up_without_holding(self):
        with patch.object(runner.RolloutStrategy, "return_to_initial_position", return_value=False), \
             patch.object(runner, "finish_partial_robot", self.order.cleanup):
            with self.assertRaisesRegex(RuntimeError, "interpolation"):
                self.run_reset()
        self.order.hold.assert_not_called()
        self.order.cleanup.assert_called_once_with(self.robot)

    def test_interrupt_during_hold_cleans_up(self):
        self.order.hold.side_effect = KeyboardInterrupt
        with patch.object(runner.RolloutStrategy, "return_to_initial_position", return_value=True), \
             patch.object(runner, "finish_partial_robot", self.order.cleanup):
            with self.assertRaises(KeyboardInterrupt):
                self.run_reset()
        self.order.cleanup.assert_called_once_with(self.robot)


if __name__ == "__main__":
    unittest.main()
