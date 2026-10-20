"""Offline fault injection for voltage confirmation, data freshness and stop."""
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_adaptive import Hardware, flush_voltage_events
from so101_session import JOINTS, Session, Settings
from so101_voltage import CONFIRM_SECONDS, LeaderVoltageConfirmation
from test_so101_session import BOUNDS, IDENTITIES, profile

LIMITS = dict.fromkeys(JOINTS, (40, 120))


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


def feedback(voltage=49, error=0, position=2000, torque=False):
    values = {j: dict(Present_Position=position, Present_Voltage=49, Status=0,
                     packet_error=0, Torque_Enable=int(torque)) for j in JOINTS}
    values["gripper"].update(Present_Voltage=voltage, Status=error, packet_error=error)
    return values


class VoltageConfirmationTests(unittest.TestCase):
    def monitor(self, check_stop=lambda: None):
        clock = Clock()
        return LeaderVoltageConfirmation(check_stop, clock=clock, wait=clock.sleep), clock

    def test_normal_feedback_reads_once_without_delay(self):
        guard, clock = self.monitor()
        reader = Mock(return_value=feedback())
        values, sampled = guard.read("leader", reader, LIMITS, False)
        self.assertEqual(sampled, clock.now)
        self.assertEqual(values["gripper"]["Present_Voltage"], 49)
        self.assertEqual(reader.call_count, 1)
        self.assertEqual(guard.events, [])

    def test_single_low_read_and_voltage_error_require_two_clean_samples(self):
        for error in (0, 1):
            with self.subTest(error=error):
                guard, clock = self.monitor()
                reader = Mock(side_effect=[feedback(34, error, 1800), feedback(position=2050), feedback(position=2100)])
                values, sampled = guard.read("leader", reader, LIMITS, False)
                self.assertEqual(reader.call_count, 3)
                self.assertEqual(values["gripper"]["Present_Position"], 2100)
                self.assertEqual(sampled, clock.now)
                self.assertEqual(guard.events[0]["initial"]["gripper"]["Present_Position"], 1800)
                self.assertEqual(guard.events[0]["outcome"], "recovered")

    def test_clean_counter_resets_and_voltage_error_must_clear(self):
        guard, _ = self.monitor()
        reader = Mock(side_effect=[feedback(35, 1), feedback(), feedback(49, 1), feedback(), feedback()])
        guard.read("leader", reader, LIMITS, False)
        self.assertEqual(reader.call_count, 5)
        self.assertEqual(len(guard.events[0]["checks"]), 4)

    def test_sustained_low_voltage_or_latched_error_still_stops(self):
        for subsequent in (feedback(35, 1), feedback(49, 1)):
            with self.subTest(subsequent=subsequent["gripper"]):
                guard, clock = self.monitor()
                read_count = 0
                def read():
                    nonlocal read_count
                    read_count += 1
                    return feedback(35, 1) if read_count == 1 else subsequent
                with self.assertRaisesRegex(RuntimeError, "within 50 ms"):
                    guard.read("leader", read, LIMITS, False)
                self.assertAlmostEqual(clock.now - 100, CONFIRM_SECONDS)
                self.assertEqual(guard.events[0]["outcome"], "failed")

    def test_follower_and_torque_enabled_leader_remain_immediate_stop(self):
        for role, torque in (("follower", False), ("follower", True), ("leader", True)):
            with self.subTest(role=role, torque=torque):
                guard, _ = self.monitor()
                reader = Mock(return_value=feedback(35, 1, torque=torque))
                with self.assertRaises(RuntimeError):
                    guard.read(role, reader, LIMITS, torque)
                self.assertEqual(reader.call_count, 1)
                self.assertFalse(guard.events)

    def test_other_faults_overvoltage_nonfinite_and_torque_change_are_never_debounced(self):
        for field, value in (("Status", 3), ("packet_error", 2), ("Torque_Enable", 1),
                             ("Present_Voltage", 130), ("Present_Voltage", float("nan")),
                             ("Present_Position", float("nan"))):
            with self.subTest(field=field, value=value):
                guard, _ = self.monitor()
                bad = feedback(35, 1)
                bad["shoulder_pan"][field] = value
                reader = Mock(return_value=bad)
                with self.assertRaises(RuntimeError):
                    guard.read("leader", reader, LIMITS, False)
                self.assertEqual(reader.call_count, 1)

    def test_fault_or_communication_failure_during_confirmation_stops(self):
        for result in (ConnectionError("packet lost"), feedback(49, 4), feedback(torque=True)):
            with self.subTest(result=str(result)):
                guard, _ = self.monitor()
                reader = Mock(side_effect=[feedback(35, 1), result])
                with self.assertRaises((ConnectionError, RuntimeError)):
                    guard.read("leader", reader, LIMITS, False)
                self.assertEqual(reader.call_count, 2)
                self.assertEqual(guard.events[0]["outcome"], "failed")

    def test_stop_during_confirmation_prevents_another_read(self):
        stopped = False
        def check():
            if stopped:
                raise InterruptedError("operator stop")
        guard, clock = self.monitor(check)
        def wait(duration):
            nonlocal stopped
            clock.sleep(duration)
            stopped = True
        guard.wait = wait
        reader = Mock(return_value=feedback(35, 1))
        with self.assertRaises(InterruptedError):
            guard.read("leader", reader, LIMITS, False)
        self.assertEqual(reader.call_count, 1)

    def test_late_normal_reply_cannot_pass_confirmation_deadline(self):
        guard, clock = self.monitor()
        count = 0
        def read():
            nonlocal count
            count += 1
            if count == 1:
                return feedback(35, 1)
            clock.sleep(.060)
            return feedback()
        with self.assertRaisesRegex(RuntimeError, "within 50 ms"):
            guard.read("leader", read, LIMITS, False)

    def test_34ms_event_can_clear_with_six_ms_bus_reads(self):
        guard, clock = self.monitor()
        began = clock.now
        def read():
            clock.sleep(.006)
            return feedback(35, 1) if clock.now-began < .034 else feedback()
        values, _ = guard.read("leader", read, LIMITS, False)
        self.assertEqual(values["gripper"]["Present_Voltage"], 49)
        self.assertLess(guard.events[0]["confirmation_ms"], 50)

    def test_confirmation_log_is_written_once_then_drained(self):
        guard, _ = self.monitor()
        guard.read("leader", Mock(side_effect=[feedback(35, 1), feedback(), feedback()]), LIMITS, False)
        hw, stream = SimpleNamespace(voltage_confirmation=guard), StringIO()
        flush_voltage_events(hw, stream, "RECORDING")
        flush_voltage_events(hw, stream, "RECORDING")
        self.assertEqual(len(stream.getvalue().splitlines()), 1)
        event = json.loads(stream.getvalue())
        self.assertEqual(event["state"], "RECORDING")
        self.assertEqual(event["initial"]["gripper"]["Present_Voltage"], 35)
        self.assertEqual(len(event["checks"]), 2)
        self.assertEqual(guard.events, [])

    def test_only_fresh_confirmed_observation_reaches_normalization_and_writer(self):
        clock = Clock()
        hw = Hardware.__new__(Hardware)
        hw.stop, hw.control_deadline = threading.Event(), None
        hw.torque, hw.bounds = {"leader": False, "follower": True}, deepcopy(BOUNDS)
        hw.voltage_limits = {role: LIMITS for role in ("leader", "follower")}
        hw.voltage_confirmation = LeaderVoltageConfirmation(hw.check_stop, clock=clock, wait=clock.sleep)
        hw.arms = {role: SimpleNamespace(bus=Mock(), send_action=Mock(), cameras={}) for role in ("leader", "follower")}
        hw.leader, hw.follower = hw.arms["leader"], hw.arms["follower"]
        hw.follower.send_action.side_effect = lambda action: action
        for arm in hw.arms.values():
            arm.bus.motors = {j: SimpleNamespace(id=i+1) for i,j in enumerate(JOINTS)}
            arm.bus.calibration = {j: SimpleNamespace(range_min=1000, range_max=3000) for j in JOINTS}
            arm.bus._normalize.side_effect = lambda values: {i:(v-1000)/20 for i,v in values.items()}
        writer = Mock()
        data = [feedback(35, 1, 1800), feedback(position=2050), feedback(position=2100), feedback(torque=True)]
        def read(_bus):
            writer.add.assert_not_called()
            hw.follower.send_action.assert_not_called()
            clock.sleep(.001)
            return data.pop(0)
        with patch("so101_adaptive.time.monotonic", clock), patch("diagnose_teleop.feedback", side_effect=read):
            obs = hw.observe()
            session = Session(hw, writer, Settings(), IDENTITIES, deepcopy(BOUNDS), "record", profile(0), 7, 120, None)
            session.state, session.following, session.started = "RECORDING", True, clock.now
            session.step(None, obs, clock.now)
        self.assertEqual(obs["raw"]["leader"]["gripper"], 2100)
        self.assertEqual(obs["leader"]["gripper"], 55)
        self.assertGreater(obs["sample_time"]["leader"], 100)
        self.assertTrue(obs["voltage_warnings"])
        writer.add.assert_called_once()
        self.assertEqual(writer.add.call_args.args[1]["gripper"], 55)
        for arm in hw.arms.values():
            arm.bus.write.assert_not_called()
            arm.bus.sync_write.assert_not_called()
            arm.bus._normalize.assert_called_once()


if __name__ == "__main__":
    unittest.main()
