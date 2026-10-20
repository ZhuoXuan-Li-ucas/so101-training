"""Offline behavioral coverage for recovery, raw feedback, and passive writes."""
from concurrent.futures import Future
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_adaptive import Hardware, ROOT, load_config
from so101_session import JOINTS, Session, Settings, WristMapping
from so101_passive_calibrate import PassiveCalibration, WRITE_FIELDS, run_calibration, validate_write
from so101_performance import summarize
from so101_readonly import buses_from_config, inventory
from test_so101_session import BOUNDS, IDENTITIES, FakeHardware, FakeWriter, pose, profile


class LimitRecoveryTests(unittest.TestCase):
    def session(self, mode="record"):
        p = profile(); h = FakeHardware(p.home); w = FakeWriter()
        s = Session(h, w, Settings(), IDENTITIES, deepcopy(BOUNDS), mode, None if mode == "setup" else p)
        if mode == "setup":
            s.step("c", h.obs, 0); s.step("a", h.obs, .05)
        else:
            s.step("s", h.obs, 0)
        return s, h, w

    def raw(self, obs):
        obs["raw"] = {r: dict.fromkeys(JOINTS, 2000) for r in ("leader", "follower")}
        obs["raw_bounds"] = {r: dict.fromkeys(JOINTS, (1000, 3000)) for r in ("leader", "follower")}

    def test_clipped_leader_gripper_pauses_and_never_sends_bad_request(self):
        s, h, w = self.session(); self.raw(h.obs)
        h.obs["raw"]["leader"]["gripper"] = 3017  # Beyond the approved 16-count allowance.
        h.obs["leader"]["gripper"] = 100  # The legacy normalization hides the violation.
        count = len(h.calls); old = dict(h.obs["follower"])
        s.step("n", h.obs, .1)
        self.assertEqual(s.state, "LIMIT_PAUSED")
        self.assertEqual(s.pause_reason["value"], 3017)
        self.assertEqual(s.pause_hold, old)
        self.assertEqual([r for r, _ in h.calls[count:]], ["hold_follower", "follower"])
        self.assertEqual(w.frames, []); self.assertEqual(w.episodes, []); self.assertEqual(s.completed, 0)

    def test_follower_raw_gripper_violation_is_fatal_before_writes(self):
        s, h, _ = self.session(); self.raw(h.obs)
        h.obs["raw"]["follower"]["gripper"] = 983
        h.obs["follower"]["gripper"] = 0
        count = len(h.calls)
        with self.assertRaises(ValueError): s.step(None, h.obs, .1)
        self.assertEqual(len(h.calls), count)

    def test_different_arm_ranges_pause_at_follower_mapping_bound(self):
        s, h, _ = self.session()
        s.bounds["follower"]["wrist_flex"] = (-100.04, 100.04)
        s.mapping.bounds = s.bounds
        h.obs["leader"]["wrist_flex"] = 100.4
        s.step(None, h.obs, .1)
        self.assertEqual(s.state, "LIMIT_PAUSED")
        self.assertEqual(s.pause_reason["source"], "mapped")
        self.assertEqual(s.last_target["wrist_flex"], 0)

    def test_explicit_aligned_resume_after_async_discard_then_reset_required(self):
        s, h, w = self.session()
        s.completed = 2; w.episodes = ["saved1", "saved2"]
        discarded = Future(); w.discard = Mock(return_value=discarded)
        valid = dict(h.obs["leader"])
        h.obs["leader"]["wrist_roll"] = 170
        s.step(None, h.obs, .1)
        h.obs["leader"] = dict(valid)
        s.step("m", h.obs, .2)
        self.assertEqual(s.state, "LIMIT_PAUSED")
        discarded.set_result(None)
        s.step(None, h.obs, .3)  # No automatic resume.
        self.assertEqual(s.state, "LIMIT_PAUSED")
        h.obs["leader"]["elbow_flex"] = 10
        s.step("m", h.obs, .4)
        self.assertEqual(s.state, "LIMIT_PAUSED")
        h.obs["leader"] = dict(valid)
        s.step("m", h.obs, .5)
        self.assertEqual(s.state, "MANUAL")
        self.assertTrue(s.following)
        self.assertEqual(s.completed, 2); self.assertEqual(w.episodes, ["saved1", "saved2"])
        self.assertEqual(s.last_target, h.obs["follower"])
        s.step("s", h.obs, .6)
        self.assertEqual(s.state, "MANUAL")
        self.assertEqual(w.discard.call_count, 1)

    def test_setup_pause_keeps_mapping_and_does_not_capture_home(self):
        s, h, _ = self.session("setup")
        self.assertIsNone(s.profile)
        valid = dict(h.obs["leader"]); h.obs["leader"]["wrist_roll"] = 170
        s.step(None, h.obs, .1)
        self.assertIsNone(s.profile)
        h.obs["leader"] = valid
        s.step("m", h.obs, .2)
        self.assertEqual(s.state, "SETUP_FOLLOW")
        self.assertIsNone(s.profile)

    def test_reset_from_limit_pause_waits_for_discard_without_requiring_mapping_alignment(self):
        s, h, w = self.session()
        discarded = Future(); w.discard = Mock(return_value=discarded)
        h.obs["leader"]["wrist_roll"] = 170
        s.step(None, h.obs, .1)
        self.assertEqual(s.state, "LIMIT_PAUSED")
        s.step("r", h.obs, .2)
        self.assertEqual(s.state, "SAVING")
        self.assertIsNone(s.reset_plan)
        s.step(None, h.obs, .3)
        self.assertEqual(s.state, "SAVING")
        discarded.set_result(None)
        s.step(None, h.obs, .4)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(s.reset_plan.origin["leader"]["wrist_roll"], 170)
        self.assertEqual(s.completed, 0)
        self.assertEqual(w.discard.call_count, 1)

    def test_pause_still_stops_on_nonfinite_stale_timeout_and_hold_error(self):
        for fault in ("nan", "raw_nan", "stale", "timeout", "hold", "discard"):
            with self.subTest(fault=fault):
                s, h, w = self.session(); self.raw(h.obs)
                h.obs["leader"]["wrist_roll"] = 170
                s.step(None, h.obs, .1); count = len(h.calls)
                now = .2
                if fault == "nan": h.obs["leader"]["elbow_flex"] = float("nan")
                if fault == "raw_nan": h.obs["raw"]["leader"]["elbow_flex"] = float("nan")
                if fault == "stale": h.obs["sample_time"] = dict(leader=-1, follower=-1)
                if fault == "timeout": now = .7
                if fault == "hold": h.obs["follower"]["elbow_flex"] += 6
                if fault == "discard":
                    s.pause_future = Future(); s.pause_future.set_exception(OSError("discard failed"))
                with self.assertRaises((ValueError, TimeoutError, RuntimeError, OSError)):
                    s.step("m", h.obs, now)
                self.assertEqual(len(h.calls), count)

    def test_return_limit_never_becomes_recoverable_pause(self):
        s, h, _ = self.session()
        s.step("n", h.obs, .1)
        h.obs["leader"]["wrist_roll"] = 181
        count = len(h.calls)
        with self.assertRaises(ValueError): s.step(None, h.obs, .2)
        self.assertEqual(len(h.calls), count)
        self.assertNotEqual(s.state, "LIMIT_PAUSED")

    def test_common_ranges_include_offset_and_warning_never_clips(self):
        s, h, _ = self.session()
        self.assertEqual(s.mapping.common_ranges()["wrist_roll"], (-180, 150))
        h.obs["leader"]["wrist_roll"] = 148
        self.assertTrue(s.boundary_warnings(h.obs))
        s.step(None, h.obs, .1)
        self.assertEqual(s.last_target["wrist_roll"], 178)

    def test_follow_mode_has_no_recording_or_automatic_return(self):
        h = FakeHardware({"leader": pose(), "follower": pose()})
        s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "follow", profile(0))
        self.assertNotIn("示教起点", s.message)
        s.step("m", h.obs, -.1)
        self.assertEqual(s.state, "START")
        self.assertEqual(h.calls, [])
        s.step("s", h.obs, 0); s.step("n", h.obs, .1)
        self.assertEqual(s.state, "TELEOP")
        self.assertEqual(s.mapping.offset, 0)

    def test_follow_mode_uses_saved_wrist_mapping(self):
        p = profile(-70); h = FakeHardware(p.home)
        s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "follow", p)
        s.step("s", h.obs, 0)
        self.assertEqual(s.mapping.offset, -70)
        self.assertEqual(s.last_target["wrist_roll"], h.obs["leader"]["wrist_roll"] - 70)
        s.step("r", h.obs, .1)
        self.assertEqual(s.state, "TELEOP")

    def test_nonfinite_mapped_target_cannot_be_paused(self):
        s, h, _ = self.session(); count = len(h.calls)
        s.mapping.requested = Mock(return_value={**pose(), "wrist_roll": float("inf")})
        with self.assertRaises(ValueError): s.step(None, h.obs, .1)
        self.assertNotEqual(s.state, "LIMIT_PAUSED")
        self.assertEqual(len(h.calls), count)


class RawHardwareTests(unittest.TestCase):
    def hardware(self):
        h = Hardware.__new__(Hardware)
        h.stop = threading.Event(); h.control_deadline = None
        h.torque = dict(leader=False, follower=True); h.bounds = BOUNDS
        h.voltage_limits = {r: dict.fromkeys(JOINTS, (40, 140)) for r in ("leader", "follower")}
        h.arms = {r: SimpleNamespace(bus=Mock(), send_action=Mock()) for r in ("leader", "follower")}
        h.leader = h.arms["leader"]; h.follower = h.arms["follower"]; h.follower.cameras = {"top": Mock(), "wrist": Mock()}
        for arm in h.arms.values():
            arm.bus.motors = {j: SimpleNamespace(id=i+1) for i, j in enumerate(JOINTS)}
            arm.bus.calibration = {j: SimpleNamespace(range_min=1000, range_max=3000) for j in JOINTS}
            arm.bus._normalize.side_effect = lambda raw: {k: min(100, max(0, (v-1000)/20)) for k, v in raw.items()}
        return h

    def feedback(self, torque):
        return {j: dict(Present_Position=3001 if j == "gripper" else 2000, Torque_Enable=torque,
                        Status=0, packet_error=0, Present_Voltage=49) for j in JOINTS}

    def test_one_raw_scan_reused_and_both_cameras_continue(self):
        h = self.hardware()
        with patch("diagnose_teleop.feedback", side_effect=[self.feedback(0), self.feedback(1)]) as read:
            obs = h.observe()
        self.assertEqual(read.call_count, 2)
        self.assertEqual(obs["raw"]["leader"]["gripper"], 3001)
        self.assertEqual(obs["leader"]["gripper"], 100)
        for arm in h.arms.values():
            arm.bus.sync_read.assert_not_called(); arm.bus._normalize.assert_called_once()
        for cam in h.follower.cameras.values(): cam.read_latest.assert_called_once()

    def test_paused_hold_never_writes_leader_or_enables_torque(self):
        h = self.hardware(); h.follower.send_action.return_value = {f"{j}.pos": v for j, v in pose().items()}
        h.hold_follower(dict(leader=pose(), follower=pose()))
        self.assertEqual(h.leader.bus.mock_calls, [])
        h.follower.bus.write.assert_not_called()
        h.follower.send_action.assert_called_once()
        h.torque["leader"] = True
        with self.assertRaises(RuntimeError): h.hold_follower(dict(follower=pose()))

    def test_low_voltage_fault_and_communication_error_are_fatal(self):
        for field, value in (("Present_Voltage", 37), ("Status", 1), ("packet_error", 1)):
            h = self.hardware(); rows = self.feedback(0); rows["gripper"][field] = value
            with patch("diagnose_teleop.feedback", return_value=rows):
                with self.assertRaises(RuntimeError): h.observe()
            h.follower.send_action.assert_not_called()
        h = self.hardware()
        with patch("diagnose_teleop.feedback", side_effect=ConnectionError("lost packet")):
            with self.assertRaises(ConnectionError): h.observe()


class PassiveCalibrationTests(unittest.TestCase):
    def test_incomplete_status_and_old_calibration_path_block_session_loading(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw = json.loads((ROOT / "configs/so101_recovery.proposed.json").read_text())
            for role, key in (("leader", "teleop"), ("follower", "robot")):
                arm = raw["recording"][key]
                original = Path(arm["calibration_dir"]) / f'{arm["id"]}.json'
                target = root / "data/calibration" / role
                target.mkdir(parents=True)
                (target / original.name).write_bytes(original.read_bytes())
                arm["calibration_dir"] = str(target)
            raw["recording"]["dataset"]["root"] = str(root / "data/datasets/new")
            config = root / "config.json"; config.write_text(json.dumps(raw))
            status_root = root / "data/calibration/status"; status_root.mkdir()
            arm = raw["recording"]["teleop"]
            status = status_root / f'leader-{arm["id"]}.json'
            with patch("so101_adaptive.ROOT", root):
                status.write_text(json.dumps({"state": "incomplete"}))
                with self.assertRaisesRegex(ValueError, "incomplete"):
                    load_config(config, "follow")
                status.write_text(json.dumps({"state": "complete", "calibration_file": str(root / "different.json")}))
                with self.assertRaisesRegex(ValueError, "new calibration"):
                    load_config(config, "follow")
                status.write_text(json.dumps({"state": "complete", "calibration_file": str(Path(arm["calibration_dir"]) / f'{arm["id"]}.json')}))
                load_config(config, "follow")

    def fixture(self):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE
        bus = FeetechMotorsBus("/dev/forbidden", {j: Motor(i+1, "sts3215", MotorNormMode.DEGREES) for i, j in enumerate(JOINTS)})
        rows = {j: {**dict.fromkeys(STS_SMS_SERIES_CONTROL_TABLE, 0), "ID": i+1, "Model_Number": 777,
                    "Phase": 12, "Present_Voltage": 49, "Min_Voltage_Limit": 40, "Max_Voltage_Limit": 120,
                    "packet_errors": [0, 0]} for i, j in enumerate(JOINTS)}
        state = deepcopy(rows)
        def write(register, joint, value, **kw):
            validate_write(register, value)
            state[joint][register] = value
        bus.write = Mock(side_effect=write)
        bus.read = Mock(side_effect=lambda register, joint, **kw: state[joint][register])
        bus.read_calibration = lambda: {j: MotorCalibration(id=state[j]["ID"], drive_mode=0,
             homing_offset=state[j]["Homing_Offset"], range_min=state[j]["Min_Position_Limit"],
             range_max=state[j]["Max_Position_Limit"]) for j in JOINTS}
        return bus, rows, state

    def test_all_calibration_writes_obey_whitelist_and_zero_only(self):
        bus, rows, state = self.fixture(); c = PassiveCalibration(bus, rows, Mock())
        c.release()
        c.sample = Mock(side_effect=[dict.fromkeys(JOINTS, 2000), dict.fromkeys(JOINTS, 2047)])
        offsets, center = c.center()
        with patch("so101_passive_calibrate.inventory", return_value=state):
            result = c.finish(offsets, dict.fromkeys(JOINTS, 1000), dict.fromkeys(JOINTS, 3000))
        self.assertEqual(set(x.args[0] for x in bus.write.call_args_list), WRITE_FIELDS)
        self.assertTrue(all(x.args[2] == 0 for x in bus.write.call_args_list if x.args[0] in {"Lock", "Operating_Mode", "Torque_Enable"}))
        self.assertEqual(result["wrist_roll"]["range_min"], 1000)
        self.assertEqual(result["wrist_roll"]["range_max"], 3000)
        self.assertEqual(result["elbow_flex"]["homing_offset"], -47)

    def test_unexpected_model_or_phase_writes_nothing(self):
        for field, value in (("Model_Number", 0), ("Phase", 28), ("ID", 99)):
            bus, rows, _ = self.fixture(); rows["gripper"][field] = value
            with self.assertRaises(ValueError): PassiveCalibration(bus, rows, Mock()).release()
            bus.write.assert_not_called()

    def test_forbidden_registers_and_torque_enable_rejected(self):
        for field, value in (("Goal_Position", 1000), ("Torque_Enable", 1), ("Lock", 1),
                             ("Phase", 12), ("P_Coefficient", 32), ("Acceleration", 254),
                             ("Min_Voltage_Limit", 30), ("Homing_Offset", 2048)):
            with self.assertRaises(ValueError): validate_write(field, value)

    def test_failed_calibration_persists_incomplete_and_does_not_restore(self):
        bus, rows, _ = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "new"; output.mkdir(); status = Path(tmp) / "status.json"
            with patch("so101_passive_calibrate.inventory", return_value=rows), \
                 patch.object(PassiveCalibration, "center", side_effect=OSError("wire lost")):
                with self.assertRaises(OSError):
                    run_calibration(bus, {"id": "same_id"}, "leader", output, status, prompt=lambda _: "")
            self.assertEqual(json.loads(status.read_text())["state"], "incomplete")
            self.assertFalse((output / "same_id.json").exists())
            self.assertEqual(len(bus.write.call_args_list), 18)  # release only; no silent restore.

    def test_register_readback_mismatch_stops_without_retry(self):
        bus, rows, _ = self.fixture(); bus.read.return_value = 1; bus.read.side_effect = None
        c = PassiveCalibration(bus, rows, Mock())
        with self.assertRaises(RuntimeError): c.release()
        self.assertEqual(bus.write.call_count, 1)

    def test_direction_or_repeat_failure_leaves_no_success_file(self):
        bus, rows, _ = self.fixture()
        for failure in ("repeat", "direction"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "new"; output.mkdir(); status = Path(tmp) / "status.json"
                samples = [dict.fromkeys(JOINTS, 2500)] if failure == "repeat" else [dict.fromkeys(JOINTS, 2047), dict.fromkeys(JOINTS, 2070)]
                with patch("so101_passive_calibrate.inventory", return_value=rows), \
                     patch.object(PassiveCalibration, "release"), \
                     patch.object(PassiveCalibration, "center", return_value=(dict.fromkeys(JOINTS, 0), dict.fromkeys(JOINTS, 2047))), \
                     patch.object(PassiveCalibration, "capture_ranges", return_value=(dict.fromkeys(JOINTS, 1000), dict.fromkeys(JOINTS, 3000))), \
                     patch.object(PassiveCalibration, "finish", return_value={}), \
                     patch.object(PassiveCalibration, "sample", side_effect=samples):
                    with self.assertRaises(ValueError):
                        run_calibration(bus, {"id": "same_id"}, "leader", output, status, prompt=lambda _: "no", repeat_check=True)
                self.assertEqual(json.loads(status.read_text())["state"], "incomplete")
                self.assertFalse((output / "same_id.json").exists())


class ReadOnlyAndPerformanceTests(unittest.TestCase):
    def test_inventory_open_close_have_zero_register_writes(self):
        from lerobot.motors.feetech import FeetechMotorsBus
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cal = {j: dict(id=i+1, drive_mode=0, homing_offset=0, range_min=1000, range_max=3000) for i, j in enumerate(JOINTS)}
            (root / "arm.json").write_text(json.dumps(cal))
            arm = dict(port="/dev/fake", calibration_dir=str(root), id="arm")
            config = dict(recording=dict(teleop=arm, robot=arm))
            def connect(bus, handshake):
                self.assertFalse(handshake)
                bus.port_handler.is_open = True
                bus.packet_handler.readTxRx = Mock(side_effect=lambda port, motor, start, count: ([0]*count, bus._comm_success, 0))
            def disconnect(bus, disable_torque):
                self.assertFalse(disable_torque); bus.port_handler.is_open = False
            with patch("so101_readonly.ensure_port_free"), \
                 patch.object(FeetechMotorsBus, "connect", connect), \
                 patch.object(FeetechMotorsBus, "disconnect", disconnect), \
                 patch.object(FeetechMotorsBus, "set_baudrate"), \
                 patch.object(FeetechMotorsBus, "set_timeout"), \
                 patch.object(FeetechMotorsBus, "_write", side_effect=AssertionError("forbidden write")) as write:
                with buses_from_config(config) as buses:
                    for bus in buses.values():
                        self.assertEqual(len(inventory(bus)), 6)
                self.assertTrue(all(not bus.is_connected for bus in buses.values()))
                write.assert_not_called()

    def test_performance_needs_60_seconds_and_rejects_gap_or_recovered_ack(self):
        rows = [{"event": "sample", "monotonic": i/30, "following": True, "state": "TELEOP",
                 "timings_ms": {"leader_read": 4, "mapping": .1, "follower_send": 1, "follower_feedback": 4, "images": 1},
                 "target": pose(), "follower": pose()} for i in range(1801)]
        self.assertTrue(summarize(rows)["pass"])
        self.assertFalse(summarize(rows[:100])["pass"])
        self.assertFalse(summarize([*rows, {"event": "communication_warning"}])["pass"])
        gap = deepcopy(rows)
        for row in gap[900:]: row["monotonic"] += .5
        self.assertFalse(summarize(gap)["pass"])


if __name__ == "__main__":
    unittest.main()
