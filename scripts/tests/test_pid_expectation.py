"""The reviewed PID is a read-only expectation, never an automatic motor tune."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import json
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_adaptive import Hardware, main
from so101_session import JOINTS, Settings


class LeaderPIDExpectationTests(unittest.TestCase):
    def configuration(self):
        return SimpleNamespace(
            teleop=SimpleNamespace(port="/fake/leader", id="leader"),
            robot=SimpleNamespace(port="/fake/follower", id="follower", cameras={}, max_relative_target=None,
                                  position_p_coefficient=16, position_i_coefficient=0, position_d_coefficient=32),
            dataset=SimpleNamespace(root=Path("/fake/dataset"), num_episodes=7, episode_time_s=120))

    def hardware(self, expected=None, actual_p=16):
        arms = {}
        for role in ("leader", "follower"):
            bus = Mock()
            bus.motors = {j: SimpleNamespace(id=i+1) for i, j in enumerate(JOINTS)}
            bus._comm_success = 0
            bus.is_calibrated = True
            bus._write.return_value = (0, 0)
            bus._read.return_value = (0, 0, 0)
            fields = {"Phase": 0, "Min_Voltage_Limit": 40, "Max_Voltage_Limit": 140,
                      "P_Coefficient": actual_p if role == "leader" else 16,
                      "I_Coefficient": 0, "D_Coefficient": 32, "Operating_Mode": 0}
            bus.read.side_effect = lambda field, _joint, values=fields, **_kw: values[field]
            arms[role] = SimpleNamespace(bus=bus, cameras={})
        with patch("lerobot.robots.make_robot_from_config", return_value=arms["follower"]), \
             patch("lerobot.teleoperators.make_teleoperator_from_config", return_value=arms["leader"]), \
             patch("so101_adaptive.calibration_context", return_value=({}, {})):
            kwargs = {} if expected is None else {"expected_leader_pid": expected}
            hw = Hardware(self.configuration(), threading.Event(), **kwargs)
        hw.health = Mock()
        return hw

    def assert_only_torque_release(self, hw):
        for arm in hw.arms.values():
            arm.bus.write.assert_not_called()
            arm.bus.sync_write.assert_not_called()
            for call in arm.bus._write.call_args_list:
                self.assertEqual(call.args[:2], (40, 1))
                self.assertEqual(call.args[3], 0)

    def test_explicit_replacement_pid_connects_without_tuning_motors(self):
        hw = self.hardware((16, 0, 32))
        with patch("diagnose_teleop.ensure_port_free"):
            hw.connect()
        self.assert_only_torque_release(hw)
        hw.health.assert_called_once()
        for arm in hw.arms.values():
            self.assertEqual(arm.bus._write.call_count, 6)
            for joint in JOINTS:
                for register in ("P_Coefficient", "I_Coefficient", "D_Coefficient", "Operating_Mode"):
                    arm.bus.read.assert_any_call(register, joint, normalize=False, num_retry=0)

    def test_legacy_default_still_rejects_replacement_pid(self):
        hw = self.hardware()
        with patch("diagnose_teleop.ensure_port_free"), self.assertRaisesRegex(RuntimeError, "expected 32; no rewrite"):
            hw.connect()
        self.assert_only_torque_release(hw)
        hw.follower.bus.connect.assert_not_called()
        hw.health.assert_not_called()

    def test_explicit_expectation_does_not_disable_mismatch_checks(self):
        hw = self.hardware((16, 0, 32), actual_p=8)
        with patch("diagnose_teleop.ensure_port_free"), self.assertRaisesRegex(RuntimeError, "expected 16; no rewrite"):
            hw.connect()
        self.assert_only_torque_release(hw)
        hw.follower.bus.connect.assert_not_called()

    def test_cli_review_and_runtime_receive_identical_explicit_pid(self):
        for check in (False, True):
            with self.subTest(check=check):
                raw = {"recording": {}, "session": {}}
                loaded = (self.configuration(), Settings(), {}, {}, Path("/fake/home.json"), None, raw)
                argv = ["so101_adaptive", "--config", "/fake/config.json", "--mode", "setup",
                        "--expected-leader-pid", "16", "0", "32"] + (["--check"] if check else [])
                output = StringIO()
                with patch.object(sys, "argv", argv), patch("so101_adaptive.load_config", return_value=loaded), \
                     patch("so101_adaptive.run") as run, redirect_stdout(output):
                    main()
                reported, _ = json.JSONDecoder().raw_decode(output.getvalue())
                self.assertEqual(reported["review"]["leader"]["expected_pid"], [16, 0, 32])
                self.assertEqual(reported["effective_config"]["runtime"]["expected_leader_pid"], [16, 0, 32])
                if check:
                    run.assert_not_called()
                else:
                    self.assertEqual(run.call_args.args[-1], (16, 0, 32))


if __name__ == "__main__":
    unittest.main()
