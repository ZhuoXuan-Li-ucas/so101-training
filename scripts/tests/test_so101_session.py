"""Behavioral tests using fake arms, clocks and writers; never open hardware."""

from concurrent.futures import Future
from copy import deepcopy
import json
import math
import multiprocessing as mp
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_session import JOINTS, HomeProfile, DirectResetPlan, Session, Settings, WristMapping
from so101_adaptive import AsyncDatasetWriter, Hardware, StopRequest


def pose(x=0, roll=0):
    return {k: 70.0 if k == "gripper" else float(roll if k == "wrist_roll" else x) for k in JOINTS}


BOUNDS = {role: {k: (0, 100) if k == "gripper" else (-180, 180) for k in JOINTS}
          for role in ("leader", "follower")}
IDENTITIES = {"leader": "L", "follower": "F"}


def profile(offset=30):
    leader = pose(0, -10)
    return HomeProfile({"version": 2, "kind": "paired_home", "identities": IDENTITIES,
                        "wrist_offset_deg": offset,
                        "home": {"leader": leader, "follower": WristMapping(offset, BOUNDS)(leader)}},
                       IDENTITIES, BOUNDS, Settings())


class FakeHardware:
    def __init__(self, pair):
        self.obs = {**deepcopy(pair), "images": {}}
        self.calls = []

    def hold(self, obs):
        self.calls.append(("hold", deepcopy(obs)))

    def hold_follower(self, obs):
        self.calls.append(("hold_follower", dict(obs["follower"])))

    def follow(self, obs):
        self.calls.append(("follow", deepcopy(obs)))

    def send(self, role, target):
        self.calls.append((role, dict(target)))
        self.obs[role] = dict(target)
        return dict(target)


class FakeWriter:
    def __init__(self):
        self.frames, self.episodes, self.finals = [], [], []
        self.failure = None
        self.defer = False

    def add(self, obs, target):
        self.frames.append((deepcopy(obs), dict(target)))

    def save(self, final=False):
        self.future = Future()
        if self.failure:
            self.future.set_exception(self.failure)
        elif not self.defer:
            self.episodes.append(list(self.frames))
            self.frames.clear()
            self.finals.append(final)
            self.future.set_result(None)
        return self.future

    def discard(self):
        self.frames.clear()
        result = Future()
        result.set_result(None)
        return result


def finish_reset(session, hardware, now):
    deadline = now + session.settings.countdown_s + session.reset_plan.duration + session.settings.settle_timeout_s + 2
    while session.state in ("COUNTDOWN", "RETURNING"):
        now += .1
        if now > deadline:
            raise AssertionError(f"Reset stuck in {session.state}")
        session.step(None, hardware.obs, now)
    return now


class MappingTests(unittest.TestCase):
    def test_offset_same_direction_other_joints_unchanged(self):
        for offset in (90, -90, 23):
            mapping = WristMapping(offset, BOUNDS)
            original = pose(7)
            target = mapping(original)
            self.assertEqual(target["wrist_roll"], offset)
            self.assertEqual(mapping(pose(7, 10))["wrist_roll"] - target["wrist_roll"], 10)
            self.assertEqual(original["wrist_roll"], 0)
            for joint in JOINTS:
                if joint != "wrist_roll":
                    self.assertEqual(target[joint], original[joint])

    def test_out_of_range_and_nan_rejected_without_wrap(self):
        for value in (100, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                WristMapping(90, BOUNDS)(pose(roll=value))

    def test_home_identity_old_paths_and_invalid_poses_rejected(self):
        for change in (lambda d: d.update(identities={}), lambda d: d.update(version=1),
                       lambda d: d.update(samples=[]),
                       lambda d: d["home"]["leader"].update(elbow_flex=float("nan")),
                       lambda d: d["home"]["follower"].update(wrist_roll=181),
                       lambda d: d["home"]["follower"].update(shoulder_lift=20)):
            data = deepcopy(profile().data); change(data)
            with self.assertRaises(ValueError):
                HomeProfile(data, IDENTITIES, BOUNDS, Settings())

    def test_home_profile_never_overwrites(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "home.json"; p = profile()
            p.write_new(path)
            self.assertEqual(json.loads(path.read_text()), p.data)
            with self.assertRaises(FileExistsError): p.write_new(path)


class DirectResetTests(unittest.TestCase):
    def test_endpoints_independent_origins_and_no_angle_wrap(self):
        p = profile()
        origin = {"leader": pose(-60, 170), "follower": pose(50, -170)}
        plan = DirectResetPlan(origin, p.home, BOUNDS, Settings())
        self.assertEqual(plan.at(-1), origin)
        self.assertEqual(plan.at(0), origin)
        self.assertEqual(plan.at(plan.duration), p.home)
        self.assertEqual(plan.at(plan.duration + 100), p.home)
        middle = plan.at(plan.duration / 2)
        self.assertAlmostEqual(middle["leader"]["wrist_roll"], 80)
        self.assertAlmostEqual(middle["follower"]["wrist_roll"], -75)
        self.assertNotEqual(middle["follower"], p.mapping(middle["leader"]))

    def test_generated_target_speed_acceleration_and_monotonicity(self):
        origin = {"leader": pose(-70, -160), "follower": pose(80, 150)}
        origin["leader"]["gripper"] = 0
        origin["follower"]["gripper"] = 100
        for settings in (Settings(), Settings(reset_min_duration_s=.01, reset_max_velocity_deg_s=100,
                                              reset_max_acceleration_deg_s2=2,
                                              reset_max_velocity_gripper_s=100,
                                              reset_max_acceleration_gripper_s2=2)):
            p = DirectResetPlan(origin, profile().home, BOUNDS, settings)
            dt = p.duration / 1000
            samples = [p.at(i * dt) for i in range(1001)]
            for role in ("leader", "follower"):
                for joint in JOINTS:
                    values = [x[role][joint] for x in samples]
                    self.assertTrue(all(min(values[0], values[-1]) - 1e-9 <= x <= max(values[0], values[-1]) + 1e-9 for x in values))
                    velocity = [(b-a)/dt for a,b in zip(values, values[1:])]
                    acceleration = [(b-a)/dt for a,b in zip(velocity, velocity[1:])]
                    vmax = settings.reset_max_velocity_gripper_s if joint == "gripper" else settings.reset_max_velocity_deg_s
                    amax = settings.reset_max_acceleration_gripper_s2 if joint == "gripper" else settings.reset_max_acceleration_deg_s2
                    self.assertLessEqual(max(map(abs, velocity)), vmax + 1e-6)
                    self.assertLessEqual(max(map(abs, acceleration)), amax + 1e-5)
                    self.assertLess(abs(velocity[0]), .01)
                    self.assertLess(abs(velocity[-1]), .01)

    def test_invalid_endpoints_and_parameters_rejected(self):
        bad = deepcopy(profile().home); bad["leader"]["elbow_flex"] = 181
        with self.assertRaises(ValueError): DirectResetPlan(bad, profile().home, BOUNDS, Settings())
        bad["leader"]["elbow_flex"] = float("nan")
        with self.assertRaises(ValueError): DirectResetPlan(bad, profile().home, BOUNDS, Settings())
        for kwargs in ({"reset_max_velocity_deg_s": 0}, {"reset_max_acceleration_deg_s2": float("inf")},
                       {"reset_min_duration_s": -1}):
            with self.assertRaises(ValueError): Settings(**kwargs)


class SessionTests(unittest.TestCase):
    def setup_session(self, **kwargs):
        p = profile(); h = FakeHardware(p.home); w = FakeWriter()
        return Session(h, w, Settings(), IDENTITIES, BOUNDS, "record", p, **kwargs), h, w

    def test_reset_accepts_unaligned_current_poses_without_fixed_entry(self):
        p = profile(); h = FakeHardware({"leader": pose(-40, -100), "follower": pose(60, 140)})
        s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        origin = deepcopy(h.obs)
        s.step("r", h.obs, 0)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(s.reset_plan.origin, {r: origin[r] for r in ("leader", "follower")})
        self.assertEqual([name for name, _ in h.calls], ["hold"])
        now = finish_reset(s, h, 0)
        self.assertEqual(s.state, "READY")
        self.assertTrue(s.pair_at(h.obs, p.home))
        self.assertEqual(s.reset_completed_count, 1)
        self.assertTrue(any(role == "leader" for role, _ in h.calls))

    def test_save_once_reset_from_current_pose_and_exclude_all_reset_frames(self):
        s, h, w = self.setup_session()
        s.step("s", h.obs, 0)
        h.obs["leader"] = pose(25, 30); h.obs["follower"] = s.mapping(h.obs["leader"])
        s.step(None, h.obs, .1)
        origin = deepcopy(h.obs)
        s.step("n", h.obs, .2)
        self.assertEqual(s.completed, 1)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(s.reset_plan.origin["leader"], origin["leader"])
        s.step("n", h.obs, .3)
        now = finish_reset(s, h, .3)
        self.assertEqual(s.state, "READY")
        self.assertEqual(len(w.episodes), 1)
        self.assertEqual(len(w.episodes[0]), 2)
        self.assertEqual(w.episodes[0][-1][1]["wrist_roll"], 60)
        self.assertEqual(w.frames, [])
        s.step("s", h.obs, now + .1)
        self.assertEqual(s.state, "RECORDING")

    def test_all_seven_episodes_reset_including_last(self):
        s, h, w = self.setup_session(episodes=7); now = 0.
        for episode in range(7):
            s.step("s", h.obs, now)
            for _ in range(episode):
                now += .1; s.step(None, h.obs, now)
            now += .1; s.step("n", h.obs, now)
            now = finish_reset(s, h, now) + .1
            self.assertEqual(s.state, "DONE" if episode == 6 else "READY")
        self.assertEqual([len(e) for e in w.episodes], list(range(1, 8)))
        self.assertEqual(w.finals, [False]*6 + [True])
        self.assertEqual(s.reset_completed_count, 7)
        s.step("s", h.obs, now)
        self.assertEqual(s.state, "DONE")

    def test_reset_during_recording_discards_instead_of_labelling_success(self):
        s, h, w = self.setup_session(); s.step("s", h.obs, 0)
        s.step("r", h.obs, .1)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(s.completed, 0)
        self.assertEqual(w.frames, []); self.assertEqual(w.episodes, [])
        finish_reset(s, h, .1)
        self.assertEqual(s.state, "READY")

    def test_timeout_requires_success_decision(self):
        s, h, w = self.setup_session(episode_time_s=.3)
        s.step("s", h.obs, 0); s.step(None, h.obs, .2); s.step(None, h.obs, .3)
        self.assertEqual(s.state, "REVIEW")
        self.assertEqual(len(w.frames), 2)
        self.assertEqual(w.episodes, [])
        s.step("r", h.obs, .4)
        self.assertEqual(s.completed, 0)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(w.frames, [])

    def test_save_failure_never_starts_reset(self):
        s, h, w = self.setup_session(); s.step("s", h.obs, 0)
        w.failure = OSError("disk full")
        with self.assertRaisesRegex(RuntimeError, "Dataset save/discard failed") as failure:
            s.step("n", h.obs, .1)
        self.assertIsInstance(failure.exception.__cause__, OSError)
        self.assertEqual(s.completed, 0)
        self.assertIsNone(s.reset_plan)

    def test_deferred_save_keeps_health_ticks_available_and_stalls_still_stop(self):
        s, h, w = self.setup_session(); s.step("s", h.obs, 0); w.defer = True
        s.step("n", h.obs, .1)
        for i in range(2, 8): s.step(None, h.obs, i / 10)
        self.assertEqual(s.state, "SAVING"); self.assertEqual(s.completed, 0)
        with self.assertRaises(TimeoutError): s.step(None, h.obs, 1.5)

    def test_first_review_gate_cannot_be_skipped_by_start_or_reset(self):
        s, h, w = self.setup_session(require_first_review=True)
        s.step("s", h.obs, 0); s.step("n", h.obs, .1)
        now = finish_reset(s, h, .1)
        self.assertEqual(s.state, "FIRST_REVIEW")
        s.step("s", h.obs, now + .1)
        self.assertEqual(s.state, "FIRST_REVIEW")
        s.step("r", h.obs, now + .2)
        now = finish_reset(s, h, now + .2)
        self.assertEqual(s.state, "FIRST_REVIEW")
        s.step("v", h.obs, now + .1)
        self.assertEqual(s.state, "READY")

    def test_stalled_control_and_offset_boundary_never_send_bad_goal(self):
        for fault in ("delay", "range"):
            s, h, _ = self.setup_session(); s.step("s", h.obs, 0); count = len(h.calls)
            if fault == "delay":
                with self.assertRaises(TimeoutError): s.step(None, h.obs, .7)
                self.assertEqual(len(h.calls), count)
            else:
                h.obs["leader"]["wrist_roll"] = 170; s.step(None, h.obs, .1)
                self.assertEqual(s.state, "LIMIT_PAUSED")
                self.assertTrue(all(call[1]["wrist_roll"] != 200 for call in h.calls[count:]))

    def test_reset_does_not_require_current_mapping_to_fit_follower_range(self):
        p = profile(); h = FakeHardware(p.home); s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        s.step("s", h.obs, 0)
        h.obs["leader"]["wrist_roll"] = 170  # Valid leader; mapped request would be 200.
        s.step("r", h.obs, .1)
        self.assertEqual(s.state, "COUNTDOWN")
        self.assertEqual(s.reset_plan.origin["leader"]["wrist_roll"], 170)
        self.assertEqual(s.reset_plan.origin["follower"]["wrist_roll"], 20)

    def test_misaligned_follow_start_is_refused_but_reset_is_available(self):
        s, h, _ = self.setup_session()
        h.obs["follower"]["elbow_flex"] += 10
        s.step("s", h.obs, 0)
        self.assertEqual(s.state, "START"); self.assertEqual(h.calls, [])
        s.step("r", h.obs, .1)
        self.assertEqual(s.state, "COUNTDOWN")

    def test_passive_nonfinite_and_active_follower_range_fault_before_writes(self):
        for raw in (False, True):
            s, h, _ = self.setup_session(); s.step("s", h.obs, 0); count = len(h.calls)
            if raw:
                h.obs["raw"] = {r: dict.fromkeys(JOINTS, 2000) for r in ("leader", "follower")}
                h.obs["raw_bounds"] = {r: dict.fromkeys(JOINTS, (1000, 3000)) for r in ("leader", "follower")}
                h.obs["raw"]["follower"]["gripper"] = 3017
            else:
                h.obs["follower"]["elbow_flex"] = 180.26
            with self.assertRaises(ValueError): s.step(None, h.obs, .1)
            self.assertEqual(len(h.calls), count)
        h = FakeHardware({"leader": pose(), "follower": pose()})
        s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "setup")
        h.obs["leader"]["elbow_flex"] = float("nan")
        with self.assertRaises(ValueError): s.step("c", h.obs, 0)
        self.assertEqual(h.calls, [])

    def test_hold_and_reset_refuse_out_of_range_origin(self):
        s, h, _ = self.setup_session(); h.obs["leader"]["elbow_flex"] = 181
        self.assertFalse(s.request_reset(h.obs, 0))
        with self.assertRaises(ValueError): s.hold(h.obs)
        self.assertEqual(h.calls, [])

    def test_return_tracking_fault_stops_before_further_commands(self):
        p = profile(); h = FakeHardware(p.home); s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        s.step("r", h.obs, 0)
        now = 0
        while s.state != "RETURNING":
            now += .1; s.step(None, h.obs, now)
        original_send = h.send
        def send(role, target):
            if role == "follower": return dict(target)
            return original_send(role, target)
        h.send = send; h.obs["follower"]["elbow_flex"] = 80
        with self.assertRaisesRegex(RuntimeError, "tracking error"):
            for _ in range(10):
                now += .1; s.step(None, h.obs, now)

    def test_return_missing_arrival_times_out(self):
        p = profile(); h = FakeHardware(p.home); s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        s.step("r", h.obs, 0); now = 0
        while s.state != "RETURNING":
            now += .1; s.step(None, h.obs, now)
        original_send = h.send
        def send(role, target):
            result = original_send(role, target)
            if role == "follower": h.obs[role]["shoulder_pan"] = 8
            return result
        h.send = send; h.obs["follower"]["shoulder_pan"] = 8
        with self.assertRaisesRegex(TimeoutError, "settling deadline"):
            for _ in range(100):
                now += .1; s.step(None, h.obs, now)
        self.assertEqual(s.reset_completed_count, 0)

    def test_countdown_motion_is_detected_before_trajectory(self):
        p = profile(); h = FakeHardware(p.home); s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        s.step("r", h.obs, 0)
        h.obs["leader"]["shoulder_pan"] += 8
        count = len(h.calls)
        with self.assertRaisesRegex(RuntimeError, "countdown"): s.step(None, h.obs, .1)
        self.assertEqual(len(h.calls), count)

    def test_setup_captures_offset_and_actual_home_without_motor_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "home.json"
            h = FakeHardware({"leader": pose(roll=60), "follower": pose(roll=-90)})
            s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "setup", profile_path=path)
            s.step("c", h.obs, 0)
            self.assertEqual(s.mapping.offset, -150)
            h.obs["leader"]["wrist_roll"] = 0
            s.step("c", h.obs, .1)
            self.assertEqual(s.mapping.offset, -90)
            h.obs["follower"]["shoulder_pan"] = 2
            s.step("h", h.obs, .2)
            self.assertEqual(h.calls, [])
            p = HomeProfile(json.loads(path.read_text()), IDENTITIES, BOUNDS, Settings())
            self.assertEqual(p.home["follower"]["shoulder_pan"], 2)
            self.assertNotIn("samples", p.data)
            self.assertEqual(s.state, "READY")
            s.step("c", h.obs, .3)
            self.assertEqual(s.mapping.offset, -90)

    def test_reference_and_home_refuse_illegal_or_unaligned_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "home.json"
            h = FakeHardware({"leader": pose(), "follower": pose(roll=90)})
            s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "setup", profile_path=path)
            s.step("c", h.obs, 0)
            h.obs["leader"]["elbow_flex"] = 181
            s.step("a", h.obs, .1)
            self.assertEqual(s.state, "ALIGN"); self.assertEqual(h.calls, [])
            h.obs["leader"]["elbow_flex"] = 0; h.obs["follower"]["shoulder_lift"] = 30
            s.step("h", h.obs, .2)
            self.assertFalse(path.exists())
            s.step("a", h.obs, .3)
            self.assertIn("shoulder_lift", s.message)

    def test_saved_mapping_is_immutable_during_follow(self):
        p = profile(); h = FakeHardware(p.home)
        s = Session(h, None, Settings(), IDENTITIES, BOUNDS, "teleop", p)
        s.step("s", h.obs, 0); h.obs["follower"]["wrist_roll"] += 2
        s.step("c", h.obs, .1)
        self.assertIs(s.mapping, p.mapping); self.assertEqual(s.mapping.offset, 30)

    def test_follow_teleop_record_require_explicit_wrist_and_home_profile(self):
        for mode in ("follow", "teleop", "record"):
            h = FakeHardware({"leader": pose(), "follower": pose()})
            with self.assertRaisesRegex(ValueError, "Capture wrist mapping"):
                Session(h, FakeWriter(), Settings(), IDENTITIES, BOUNDS, mode)
            self.assertEqual(h.calls, [])


class StopRequestTests(unittest.TestCase):
    def test_gui_stop_still_interrupts_hardware_before_writes(self):
        event = mp.get_context("spawn").Event()
        stop = StopRequest(event)
        self.assertFalse(stop.wait(0))
        event.set()
        self.assertTrue(stop.wait(0))
        hw = Hardware.__new__(Hardware)
        hw.stop, hw.control_deadline = stop, None
        hw.arms = {"follower": Mock()}
        with self.assertRaises(InterruptedError):
            hw._enable_at("follower", pose())
        hw.arms["follower"].bus.sync_write.assert_not_called()

    def test_real_signals_while_event_lock_is_held_do_not_deadlock(self):
        # Run in a disposable child so regression to Event.set in the signal
        # handler fails with a timeout instead of hanging the entire test suite.
        script = '''
import multiprocessing as mp
import os
import signal
import sys
from unittest.mock import Mock
sys.path.insert(0, sys.argv[1])
from so101_adaptive import Hardware, StopRequest
for sig in (signal.SIGINT, signal.SIGTERM):
    event = mp.get_context("spawn").Event()
    stop = StopRequest(event)
    previous = signal.signal(sig, stop.handle_signal)
    try:
        # Event.is_set/wait hold this same non-reentrant condition lock.
        with event._cond:
            os.kill(os.getpid(), sig)
        assert stop.is_set() and stop.wait(0)
        assert not event.is_set()
        hw = Hardware.__new__(Hardware)
        hw.stop, hw.control_deadline = stop, None
        hw.arms = {"follower": Mock()}
        try:
            hw._enable_at("follower", {})
        except InterruptedError:
            pass
        else:
            raise AssertionError("A signal must interrupt hardware writes")
        hw.arms["follower"].bus.sync_write.assert_not_called()
    finally:
        signal.signal(sig, previous)
'''
        result = subprocess.run([sys.executable, "-c", script, str(Path(__file__).resolve().parents[1])],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class TorqueEnableTests(unittest.TestCase):
    def hardware(self, on_failure=None):
        hw = Hardware.__new__(Hardware)
        hw.stop = threading.Event()
        hw.control_deadline = None
        hw.torque = {"leader": False, "follower": False}
        leader, follower = Mock(), Mock()
        hw.arms = {"leader": leader, "follower": follower}
        hw.follower = follower
        follower.cameras = {}
        def write(_register, joint, _value, **_kwargs):
            if joint == "wrist_flex":
                if on_failure:
                    on_failure(hw)
                raise ConnectionError("Incorrect status packet!")
        follower.bus.write.side_effect = write
        return hw, follower.bus

    def test_damaged_ack_requires_readback_without_repeating_enable(self):
        hw, bus = self.hardware()
        bus.read.return_value = 1
        hw._enable_at("follower", pose())
        self.assertTrue(hw.torque["follower"])
        self.assertEqual([c.args[1] for c in bus.write.call_args_list], list(JOINTS))
        self.assertTrue(all(c.args[0] == "Torque_Enable" and c.args[2] == 1
                            and c.kwargs["num_retry"] == 0 for c in bus.write.call_args_list))
        bus.read.assert_called_once_with("Torque_Enable", "wrist_flex", normalize=False, num_retry=0)
        bus.sync_write.assert_called_once_with("Goal_Position", pose(), num_retry=0)

    def test_unconfirmed_enable_fails_and_cleanup_releases_both_arms(self):
        for result in (0, ConnectionError("read timed out"), RuntimeError("Input voltage error!")):
            with self.subTest(result=result):
                hw, bus = self.hardware()
                if isinstance(result, Exception):
                    bus.read.side_effect = result
                else:
                    bus.read.return_value = result
                with patch("diagnose_teleop.disable_all") as disable:
                    try:
                        with self.assertRaises((ConnectionError, RuntimeError)):
                            hw._enable_at("follower", pose())
                    finally:
                        hw.close()
                    self.assertEqual(disable.call_count, 2)
                self.assertFalse(hw.torque["follower"])
                self.assertEqual([c.args[1] for c in bus.write.call_args_list], list(JOINTS[:4]))
                for arm in hw.arms.values():
                    arm.bus.sync_write.assert_any_call("Torque_Enable", dict.fromkeys(JOINTS, 0),
                                                       normalize=False, num_retry=0)

    def test_motor_protection_error_is_not_treated_as_lost_ack(self):
        hw, bus = self.hardware()
        bus.write.side_effect = RuntimeError("Input voltage error!")
        with self.assertRaisesRegex(RuntimeError, "Input voltage"):
            hw._enable_at("follower", pose())
        bus.read.assert_not_called()
        self.assertEqual(bus.write.call_count, 1)
        self.assertFalse(hw.torque["follower"])

    def test_stop_or_deadline_before_readback_prevents_new_reads(self):
        for mode in ("stop", "deadline"):
            with self.subTest(mode=mode):
                def interrupt(hw):
                    if mode == "stop":
                        hw.stop.set()
                    else:
                        hw.control_deadline = time.monotonic() - 1
                hw, bus = self.hardware(interrupt)
                with self.assertRaises(InterruptedError if mode == "stop" else TimeoutError):
                    hw._enable_at("follower", pose())
                bus.read.assert_not_called()
                self.assertEqual(bus.write.call_count, 4)
                self.assertFalse(hw.torque["follower"])

    def test_stop_or_deadline_after_readback_prevents_remaining_enables(self):
        for mode in ("stop", "deadline"):
            with self.subTest(mode=mode):
                hw, bus = self.hardware()
                def read(*_args, **_kwargs):
                    if mode == "stop":
                        hw.stop.set()
                    else:
                        hw.control_deadline = time.monotonic() - 1
                    return 1
                bus.read.side_effect = read
                with self.assertRaises(InterruptedError if mode == "stop" else TimeoutError):
                    hw._enable_at("follower", pose())
                self.assertEqual(bus.write.call_count, 4)
                self.assertFalse(hw.torque["follower"])


class WriterTests(unittest.TestCase):
    def dataset(self):
        ds = Mock()
        ds.has_pending_frames.return_value = False
        return ds

    def test_writer_save_error_surfaces_and_finalize_still_runs(self):
        ds = self.dataset()
        ds.save_episode.side_effect = OSError("no space")
        writer = AsyncDatasetWriter(ds, "task", lambda v, t: v)
        with self.assertRaises(OSError):
            writer.save().result(timeout=2)
        with self.assertRaises(RuntimeError):
            writer.close()
        ds.finalize.assert_called_once()

    def test_bounded_queue_never_blocks_control_or_drops_silently(self):
        ds = self.dataset()
        entered, release = threading.Event(), threading.Event()
        def blocked_save():
            entered.set()
            release.wait(2)
        ds.save_episode.side_effect = blocked_save
        writer = AsyncDatasetWriter(ds, "task", lambda v, t: v, max_queue=1)
        try:
            writer.save()
            self.assertTrue(entered.wait(1))
            writer.discard()
            with self.assertRaisesRegex(RuntimeError, "queue full"):
                writer.discard()
        finally:
            release.set()
            writer.close()

    def test_unsuccessful_buffer_discarded_on_close(self):
        ds = self.dataset()
        ds.has_pending_frames.return_value = True
        writer = AsyncDatasetWriter(ds, "task", lambda v, t: v)
        writer.close()
        ds.clear_episode_buffer.assert_called_once()
        ds.save_episode.assert_not_called()
        ds.finalize.assert_called_once()

    def test_partial_hardware_startup_cleanup_attempts_both_arms(self):
        hw = Hardware.__new__(Hardware)
        a, b = Mock(), Mock()
        a.bus.is_connected = b.bus.is_connected = True
        b.cameras = {"top": Mock(is_connected=False)}
        hw.arms = {"leader": a, "follower": b}
        hw.follower = b
        with patch("diagnose_teleop.disable_all", side_effect=[RuntimeError("leader unplugged"), None]) as disable:
            with self.assertRaisesRegex(RuntimeError, "unverified"):
                hw.close()
            self.assertEqual(disable.call_count, 2)
        a.bus.disconnect.assert_called_once()
        b.bus.disconnect.assert_called_once()

    def test_enabling_only_writes_declared_registers_and_stop_prevents_target(self):
        hw = Hardware.__new__(Hardware)
        hw.stop = threading.Event()
        hw.control_deadline = None
        hw.torque = {"leader": False, "follower": False}
        leader, follower = Mock(), Mock()
        hw.arms = {"leader": leader, "follower": follower}
        hw._enable_at("leader", pose())
        leader.bus.enable_torque.assert_not_called()
        self.assertEqual(leader.bus.sync_write.call_args.args[0], "Goal_Position")
        self.assertEqual([call.args[0] for call in leader.bus.write.call_args_list], ["Torque_Enable"] * 6)
        hw.control_deadline = time.monotonic() - 1
        with self.assertRaises(TimeoutError):
            hw._enable_at("follower", pose())
        follower.bus.sync_write.assert_not_called()
        hw.control_deadline = None
        hw.stop.set()
        with self.assertRaises(InterruptedError):
            hw._enable_at("follower", pose())
        follower.bus.sync_write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
