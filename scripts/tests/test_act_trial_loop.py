"""No-hardware tests for operator sequencing, timeouts and partial-startup cleanup."""

import csv
import io
import json
import os
from contextlib import redirect_stdout
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from act_trial_session import EnterReader, RoundLog, TrialControls, TrialLoop


class FakeBackend:
    def __init__(self, controls, duration=3):
        self.controls = controls
        self.duration_s = duration
        self.calls = []
        self.reset_gate = None

    def connect(self):
        self.calls.append("connect")

    def reset_home(self):
        self.calls.append("reset")
        if self.reset_gate:
            self.reset_gate.wait(2)

    def prepare_run(self):
        self.calls.append("clear_queue")

    def run_policy(self):
        self.calls.append("run")
        self.controls.segment_stop.wait(self.duration_s)

    def close(self):
        self.calls.append("close")


class TrialSequenceTests(unittest.TestCase):
    def setUp(self):
        self.controls = TrialControls()
        self.backend = FakeBackend(self.controls)
        self.csv = io.StringIO()
        self.messages = []
        self.errors = []
        self.loop = TrialLoop(self.backend, self.controls, RoundLog(self.csv), .15, self.messages.append)

    def launch(self):
        def run():
            try:
                self.loop.run()
            except BaseException as exc:
                self.errors.append(exc)
        self.thread = threading.Thread(target=run)
        self.thread.start()
        self.wait_phase("WAIT_HOME")

    def wait_phase(self, phase):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if self.controls.phase == phase:
                return
            time.sleep(.002)
        self.fail(f"Expected {phase}, observed {self.controls.phase}; errors={self.errors}")

    def rows(self):
        return list(csv.DictReader(io.StringIO(self.csv.getvalue())))

    def tearDown(self):
        self.controls.quit()
        if self.backend.reset_gate:
            self.backend.reset_gate.set()
        if hasattr(self, "thread"):
            self.thread.join(3)
            self.assertFalse(self.thread.is_alive())

    def start_round(self):
        self.controls.enter()
        self.wait_phase("READY")
        self.controls.enter()
        self.wait_phase("RUNNING")

    def test_complete_enter_cycle_and_fresh_queue_each_round(self):
        self.launch()
        self.assertEqual(self.backend.calls, [])
        self.controls.enter()
        self.wait_phase("READY")
        self.assertEqual(self.backend.calls, ["connect", "reset"])
        self.controls.enter()
        self.wait_phase("RUNNING")
        self.controls.enter()
        self.wait_phase("COOLDOWN")
        self.assertFalse(self.controls.enter())
        self.assertFalse(self.controls.enter())
        self.wait_phase("READY")
        self.assertEqual(self.backend.calls.count("run"), 1)
        self.assertEqual(self.backend.calls.count("reset"), 2)
        self.assertEqual(self.rows()[0]["end_reason"], "enter")
        self.controls.enter()
        self.wait_phase("RUNNING")
        self.controls.quit()
        self.wait_phase("STOPPED")
        self.assertEqual(self.backend.calls.count("clear_queue"), 2)
        self.assertEqual(self.backend.calls.count("reset"), 2)  # quit never returns home
        self.assertEqual([r["round"] for r in self.rows()], ["1", "2"])
        self.assertEqual(self.rows()[1]["end_reason"], "quit")
        self.assertNotIn("success", self.csv.getvalue())
        self.assertEqual(self.errors, [])

    def test_enter_during_reset_is_not_saved_for_next_start(self):
        self.backend.reset_gate = threading.Event()
        self.launch()
        self.controls.enter()
        self.wait_phase("RESETTING")
        for _ in range(4):
            self.assertFalse(self.controls.enter())
        self.backend.reset_gate.set()
        self.wait_phase("READY")
        self.assertNotIn("run", self.backend.calls)

    def test_time_limit_returns_home_and_waits_for_fresh_enter(self):
        self.backend.duration_s = .05
        self.launch()
        self.start_round()
        self.wait_phase("COOLDOWN")
        self.wait_phase("READY")
        self.assertEqual(self.rows()[0]["end_reason"], "time_limit")
        self.assertEqual(self.backend.calls.count("reset"), 2)
        self.assertEqual(self.backend.calls.count("run"), 1)

    def test_ten_round_limit_resets_then_closes_without_an_eleventh_trial(self):
        self.loop.max_rounds = 10
        self.loop.scene_reset_s = .01
        self.backend.duration_s = .015
        self.launch()
        self.controls.enter()
        for number in range(1, 11):
            self.wait_phase("READY")
            self.assertEqual(self.backend.calls.count("run"), number - 1)
            self.controls.enter()
            self.wait_phase("STOPPED" if number == 10 else "COOLDOWN")
        self.thread.join(1)
        self.assertEqual(self.backend.calls.count("run"), 10)
        self.assertEqual(self.backend.calls.count("clear_queue"), 10)
        self.assertEqual(self.backend.calls.count("reset"), 11)
        self.assertEqual(self.backend.calls[-2:], ["reset", "close"])
        self.assertEqual([int(r["round"]) for r in self.rows()], list(range(1, 11)))
        self.assertFalse(self.controls.enter())
        self.assertEqual(self.errors, [])

    def test_final_reset_failure_still_closes_and_reports_error(self):
        self.loop.max_rounds = 1
        self.backend.duration_s = .02
        self.backend.reset_home = Mock(side_effect=[None, ConnectionError("final reset failed")])
        self.launch()
        self.start_round()
        self.wait_phase("STOPPED")
        self.assertEqual(self.backend.calls[-1], "close")
        self.assertEqual(len(self.rows()), 1)
        self.assertIsInstance(self.errors[0], ConnectionError)
        self.assertFalse(any("已完成 1 轮并复位" in m for m in self.messages))

    def test_quit_before_first_enter_never_connects(self):
        self.launch()
        self.controls.quit()
        self.wait_phase("STOPPED")
        self.assertEqual(self.backend.calls, ["close"])

    def test_quit_interrupts_ten_second_scene_wait_without_next_motion(self):
        self.loop.scene_reset_s = 10
        self.launch()
        self.start_round()
        self.controls.enter()
        self.wait_phase("COOLDOWN")
        self.controls.quit()
        self.wait_phase("STOPPED")
        self.assertEqual(self.backend.calls.count("run"), 1)

    def test_partial_connect_exception_still_calls_close(self):
        self.backend.connect = Mock(side_effect=ConnectionError("Lock ACK missing"))
        self.launch()
        self.controls.enter()
        self.wait_phase("STOPPED")
        self.assertIsInstance(self.errors[0], ConnectionError)
        self.assertEqual(self.backend.calls, ["close"])

    def test_policy_exception_is_logged_and_does_not_start_a_reset(self):
        self.backend.run_policy = Mock(side_effect=ConnectionError("disconnected"))
        self.launch()
        self.controls.enter()
        self.wait_phase("READY")
        self.controls.enter()
        self.wait_phase("STOPPED")
        self.assertEqual(self.rows()[0]["end_reason"], "error")
        self.assertEqual(self.backend.calls, ["connect", "reset", "clear_queue", "close"])

    def test_reset_failure_never_exposes_ready_or_starts_policy(self):
        self.backend.reset_home = Mock(side_effect=ConnectionError("reset error"))
        self.launch()
        self.controls.enter()
        self.wait_phase("STOPPED")
        self.assertEqual(self.backend.calls, ["connect", "close"])
        self.assertEqual(self.rows(), [])


class TerminalInputTests(unittest.TestCase):
    def test_blank_enter_is_delivered_and_q_ends_reader(self):
        read_fd, write_fd = os.pipe()
        entered, quit_seen = [], threading.Event()
        with os.fdopen(read_fd, "r") as stream:
            reader = EnterReader(stream, lambda: entered.append(1) or True, quit_seen.set)
            reader.start()
            os.write(write_fd, b"\n\nq\n\n")
            self.assertTrue(quit_seen.wait(2))
            reader.stop()
        os.close(write_fd)
        self.assertEqual(len(entered), 2)

    def test_eof_does_not_fabricate_an_enter(self):
        read_fd, write_fd = os.pipe()
        entered, quit_seen = [], threading.Event()
        with os.fdopen(read_fd, "r") as stream:
            reader = EnterReader(stream, lambda: entered.append(1) or True, quit_seen.set)
            reader.start()
            os.close(write_fd)
            self.assertTrue(quit_seen.wait(2))
            reader.stop()
        self.assertEqual(entered, [])


class StartupWriteRetryTests(unittest.TestCase):
    """Exercise official Feetech write/enable_torque code with a fake packet transport."""

    def setUp(self):
        import scservo_sdk as scs
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        self.scs = scs
        self.bus = FeetechMotorsBus(
            "/dev/not-opened-by-test", {"shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES)}
        )
        self.bus.port_handler = SimpleNamespace(is_open=True)
        packet_patch = patch.object(self.bus.packet_handler, "writeTxRx")
        self.tx = packet_patch.start()
        self.addCleanup(packet_patch.stop)

    def test_bad_lock_reply_retries_same_write_then_recovers(self):
        from act_trial_loop import retry_startup_writes
        self.tx.side_effect = [(self.scs.COMM_SUCCESS, 0), (self.scs.COMM_RX_CORRUPT, 0), (self.scs.COMM_SUCCESS, 0)]
        with retry_startup_writes(self.bus, 2):
            self.bus.enable_torque("shoulder_lift")
        calls = self.tx.call_args_list
        self.assertEqual(len(calls), 3)  # Torque_Enable once, Lock twice
        self.assertEqual(calls[1], calls[2])
        self.assertEqual(calls[1].args[1:4], (2, 55, 1))
        self.assertEqual(list(calls[1].args[4]), [1])
        self.assertNotIn("write", vars(self.bus))

    def test_persistent_bad_reply_stops_after_three_attempts_and_restores_method(self):
        from act_trial_loop import retry_startup_writes
        self.tx.return_value = (self.scs.COMM_RX_CORRUPT, 0)
        with self.assertRaises(ConnectionError):
            with retry_startup_writes(self.bus, 2):
                self.bus.write("Lock", "shoulder_lift", 1)
        self.assertEqual(self.tx.call_count, 3)
        self.assertNotIn("write", vars(self.bus))
        self.tx.reset_mock()
        with self.assertRaises(ConnectionError):
            self.bus.write("Lock", "shoulder_lift", 1)
        self.assertEqual(self.tx.call_count, 1)

    def test_motor_fault_is_not_treated_as_a_corrupt_reply(self):
        from act_trial_loop import retry_startup_writes
        self.tx.return_value = (self.scs.COMM_SUCCESS, 0x20)
        with retry_startup_writes(self.bus, 2):
            with self.assertRaises(RuntimeError):
                self.bus.write("Lock", "shoulder_lift", 1)
        self.assertEqual(self.tx.call_count, 1)

    def test_goal_position_writes_do_not_gain_retries(self):
        from act_trial_loop import retry_startup_writes
        self.tx.return_value = (self.scs.COMM_RX_CORRUPT, 0)
        with retry_startup_writes(self.bus, 2):
            with self.assertRaises(ConnectionError):
                self.bus.write("Goal_Position", "shoulder_lift", 2048, normalize=False)
        self.assertEqual(self.tx.call_count, 1)


class OfficialBackendTests(unittest.TestCase):
    def test_cli_check_parses_real_config_without_creating_hardware(self):
        import act_trial_loop as runner
        output = io.StringIO()
        argv = ["act_trial_loop.py", f"--config_path={runner.ROOT}/configs/act_trial_loop_080000.proposed.json", "--check=true"]
        with patch.object(sys, "argv", argv), redirect_stdout(output), \
             patch.object(runner.official_context, "make_robot_from_config") as factory:
            runner.main()
        review = json.loads(output.getvalue())
        factory.assert_not_called()
        self.assertFalse(review["hardware_connected"])
        self.assertEqual(review["trial_duration_s"], 120)
        self.assertEqual(review["scene_reset_s"], 10)
        self.assertEqual(len(review["home_targets"]), 6)
        self.assertIsNone(review["max_relative_target"])

    def test_failed_official_context_retains_robot_for_cleanup_and_restores_factory(self):
        import act_trial_loop as runner
        from lerobot.rollout.configs import BaseStrategyConfig
        backend = runner.OfficialBackend(SimpleNamespace(duration=120, strategy=BaseStrategyConfig(), startup_write_retries=2), {}, TrialControls())
        original_write = Mock()
        robot = SimpleNamespace(bus=SimpleNamespace(write=original_write))

        def fail_build(_cfg, _stop):
            runner.official_context.make_robot_from_config("robot_config")
            raise ConnectionError("startup ACK lost")

        with patch.object(runner.official_context, "make_robot_from_config", return_value=robot) as factory, \
             patch.object(runner.official_context, "build_rollout_context", side_effect=fail_build), \
             patch.object(runner, "finish_partial_robot") as cleanup:
            with self.assertRaises(ConnectionError):
                backend.connect()
            self.assertIs(runner.official_context.make_robot_from_config, factory)
            self.assertIs(robot.bus.write, original_write)
            backend.close()
            cleanup.assert_called_once_with(robot)

    def test_cleanup_releases_bus_even_when_camera_connection_is_incomplete(self):
        import act_trial_loop as runner
        bus = SimpleNamespace(is_connected=True, disconnect=Mock())
        camera = SimpleNamespace(is_connected=True, disconnect=Mock())
        robot = SimpleNamespace(bus=bus, cameras={"top": camera}, is_connected=False)
        with patch.object(runner, "disable_all") as off:
            runner.finish_partial_robot(robot)
        off.assert_called_once_with(bus)
        bus.disconnect.assert_called_once_with(disable_torque=False)
        camera.disconnect.assert_called_once_with()

    def test_cleanup_failure_is_reported_but_other_resources_are_released(self):
        import act_trial_loop as runner
        bus = SimpleNamespace(is_connected=True, disconnect=Mock())
        camera = SimpleNamespace(is_connected=True, disconnect=Mock())
        robot = SimpleNamespace(bus=bus, cameras={"top": camera})
        with patch.object(runner, "disable_all", side_effect=ConnectionError("no response")):
            with self.assertRaisesRegex(RuntimeError, "Torque OFF"):
                runner.finish_partial_robot(robot)
        bus.disconnect.assert_called_once()
        camera.disconnect.assert_called_once()

    def test_queue_reset_and_official_reset_arguments(self):
        import act_trial_loop as runner
        from lerobot.rollout.configs import BaseStrategyConfig
        cfg = SimpleNamespace(duration=120, strategy=BaseStrategyConfig(), reset_duration_s=3, reset_fps=50)
        backend = runner.OfficialBackend(cfg, {"gripper.pos": 4.777}, TrialControls())
        hardware = object()
        backend.ctx = SimpleNamespace(hardware=hardware)
        backend.strategy = Mock()
        backend.strategy.return_to_initial_position.return_value = True
        backend.prepare_run()
        backend.reset_home()
        self.assertEqual(backend.strategy.reset_control_state.call_count, 2)
        backend.strategy.return_to_initial_position.assert_called_once_with(hardware, duration_s=3, fps=50)


if __name__ == "__main__":
    unittest.main()
