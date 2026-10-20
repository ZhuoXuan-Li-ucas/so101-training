"""Offline fault injection: keep the session alive, stop motion, preserve saves."""

from concurrent.futures import Future
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import json
from pathlib import Path
import queue
import signal
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import so101_adaptive as adaptive
from so101_preview import PreviewWindow
from so101_session import JOINTS, PositionLimit, ResetMotionError, Session, Settings
from so101_voltage import MotorHealthError
import test_recovery
from test_so101_session import BOUNDS, IDENTITIES, FakeHardware, FakeWriter, pose, profile


class RecoveryHardware(FakeHardware):
    def __init__(self, pair):
        super().__init__(pair)
        self.now = 0.0
        self.error = None
        self.release_error = None
        self.torque = dict(leader=False, follower=False)
        self.read_modes = []
        self.control_deadline = None
        self.communication_warnings = []
        self.voltage_confirmation = SimpleNamespace(events=[])

    def observe(self, *, detailed=True):
        self.read_modes.append(detailed)
        if self.error:
            raise self.error
        return {**deepcopy(self.obs), "feedback_source": "full" if detailed else "official_position",
                "raw": {r: dict.fromkeys(JOINTS, 2000) for r in self.torque},
                "raw_bounds": {r: dict.fromkeys(JOINTS, (1000, 3000)) for r in self.torque},
                "sample_time": dict.fromkeys(self.torque, self.now), "timings_ms": {},
                "health": {r: {j: dict(Torque_Enable=int(on), Status=0, packet_error=0,
                                       Present_Voltage=49) for j in JOINTS} for r, on in self.torque.items()}}

    def hold(self, obs):
        super().hold(obs)
        self.torque = dict(leader=True, follower=True)

    def follow(self, obs):
        super().follow(obs)
        self.torque = dict(leader=False, follower=True)

    def release_torque(self):
        self.calls.append(("release", self.control_deadline))
        if self.release_error:
            raise self.release_error
        self.torque = dict(leader=False, follower=False)

    def close(self):
        self.calls.append(("close",))


class RecoveryTests(unittest.TestCase):
    def session(self, mode="teleop", **kwargs):
        saved = profile()
        hardware, writer = RecoveryHardware(saved.home), FakeWriter()
        writer.save = Mock(wraps=writer.save)
        writer.discard = Mock(wraps=writer.discard)
        session = Session(hardware, writer, Settings(), IDENTITIES, BOUNDS, mode, saved, **kwargs)
        return session, hardware, writer

    def tick(self, session, hardware, now, command=None):
        hardware.now = now
        return adaptive.control_tick(session, hardware, command, "official", clock=lambda: now)

    def fault(self, session, hardware, now=.1, error=None):
        hardware.error = error or MotorHealthError("persistent undervoltage")
        obs, event = self.tick(session, hardware, now)
        self.assertIsNone(obs)
        self.assertEqual(event["event"], "fault_paused")
        self.assertEqual(session.state, "FAULT_PAUSED")
        self.assertFalse(session.following or session.holding)
        self.assertIsNone(session.reset_plan)
        self.assertEqual(hardware.calls[-1], ("release", None))
        hardware.error = None
        return event

    def test_device_faults_in_countdown_release_before_pause(self):
        errors = [MotorHealthError("voltage"), ConnectionError("serial"), TimeoutError("deadline"),
                  ResetMotionError("tracking"), PositionLimit("leader", "wrist_roll", 181, -180, 180, "degrees")]
        for error in errors:
            with self.subTest(error=error):
                session, hardware, _ = self.session()
                self.tick(session, hardware, 0, "r")
                hardware.control_deadline = -1
                with patch.object(session, "enter_fault_pause", wraps=session.enter_fault_pause) as pause:
                    original = hardware.release_torque
                    def release():
                        pause.assert_not_called()
                        original()
                    hardware.release_torque = release
                    self.fault(session, hardware, error=error)
                self.assertEqual(len(hardware.calls), 2)  # initial hold, then OFF; no goal

    def test_bad_reads_stay_paused_without_repeated_motor_writes(self):
        session, hardware, _ = self.session()
        self.tick(session, hardware, 0, "r")
        self.fault(session, hardware)
        calls = deepcopy(hardware.calls)
        read_count = len(hardware.read_modes)
        for i, error in enumerate([MotorHealthError("low voltage"), ConnectionError("unplugged"),
                                   TimeoutError("stale")], 1):
            hardware.error = error
            obs, event = self.tick(session, hardware, .1 + .2*i, "r")
            self.assertIsNone(obs)
            self.assertIsNone(event)
            self.assertFalse(session.fault_ready)
        self.assertEqual(hardware.calls, calls)
        self.assertTrue(all(hardware.read_modes[read_count:]))

    def test_two_new_samples_then_new_r_replans_from_current_poses(self):
        session, hardware, _ = self.session()
        self.tick(session, hardware, 0, "r")
        self.fault(session, hardware)
        calls = deepcopy(hardware.calls)
        self.tick(session, hardware, .3, "r")
        self.tick(session, hardware, .3, "r")  # same sample cannot count twice
        self.assertFalse(session.fault_ready)
        self.tick(session, hardware, .5, "r")  # R before readiness is consumed
        self.assertTrue(session.fault_ready)
        for command, now in [(None, .7), ("s", .9), ("n", 1.1), ("m", 1.3), ("v", 1.5)]:
            self.tick(session, hardware, now, command)
        self.assertEqual(session.state, "FAULT_PAUSED")
        self.assertEqual(hardware.calls, calls)
        hardware.obs.update(leader=pose(30, 10), follower=pose(-20, 5))
        current = {r: deepcopy(hardware.obs[r]) for r in hardware.torque}
        _, event = self.tick(session, hardware, 1.7, "r")
        self.assertEqual(event["event"], "reset_retry")
        self.assertEqual(session.state, "COUNTDOWN")
        self.assertEqual(session.reset_plan.origin, current)
        self.assertEqual(session.reset_started, 1.7)
        self.assertEqual(hardware.calls[-1][0], "hold")

    def test_failed_release_and_unhealthy_feedback_block_retry(self):
        for kind in ("torque", "status", "packet", "stale", "nan", "raw_range", "partial", "native"):
            with self.subTest(kind=kind):
                session, hardware, _ = self.session()
                self.tick(session, hardware, 0, "r")
                hardware.release_error = ConnectionError("OFF not confirmed")
                event = self.fault(session, hardware)
                self.assertIn("OFF not confirmed", event["torque_release_error"])
                hardware.torque = dict(leader=False, follower=False)
                self.tick(session, hardware, .3)
                self.tick(session, hardware, .5)
                self.assertTrue(session.fault_ready)
                hardware.now = .7
                obs = hardware.observe()
                if kind == "torque": obs["health"]["leader"]["gripper"]["Torque_Enable"] = 1
                if kind == "status": obs["health"]["follower"]["gripper"]["Status"] = 1
                if kind == "packet": obs["health"]["follower"]["gripper"]["packet_error"] = 1
                if kind == "stale": obs["sample_time"]["leader"] = 0
                if kind == "nan": obs["leader"]["gripper"] = float("nan")
                if kind == "raw_range": obs["raw"]["leader"]["gripper"] = 3017
                if kind == "partial": obs["health"]["follower"].pop("gripper")
                if kind == "native": obs["feedback_source"] = "official_position"
                before = deepcopy(hardware.calls)
                with patch.object(hardware, "observe", return_value=obs):
                    result, event = self.tick(session, hardware, .7, "r")
                self.assertIsNone(result)  # bad data must not reach the UI or JSON samples
                self.assertIsNone(event)
                self.assertEqual(session.state, "FAULT_PAUSED")
                self.assertFalse(session.fault_ready)
                self.assertEqual(hardware.calls, before)

    def test_save_finishes_once_while_voltage_stays_bad_without_reset(self):
        session, hardware, writer = self.session("record", episodes=2, require_first_review=True)
        writer.defer = True
        self.tick(session, hardware, 0, "s")
        self.tick(session, hardware, .1, "n")
        future = session.future
        self.fault(session, hardware, .2)
        self.assertIs(session.future, future)
        writer.discard.assert_not_called()
        self.tick(session, hardware, .4, "r")
        self.tick(session, hardware, .6, "r")
        self.assertFalse(session.fault_ready)
        future.set_result({"report": "offline-first-review"})
        hardware.error = MotorHealthError("still low")
        self.tick(session, hardware, .8)
        self.tick(session, hardware, 1.0)
        self.assertEqual(session.completed, 1)
        self.assertIsNone(session.future)
        self.assertEqual(session.review_report, {"report": "offline-first-review"})
        self.assertEqual(session.state, "FAULT_PAUSED")
        writer.save.assert_called_once_with(final=False)
        writer.discard.assert_not_called()
        hardware.error = None
        self.tick(session, hardware, 1.2)
        self.tick(session, hardware, 1.4)
        self.tick(session, hardware, 1.6, "r")
        now = 1.6
        for _ in range(100):
            if session.state not in ("COUNTDOWN", "RETURNING"):
                break
            now += .1
            self.tick(session, hardware, now)
        self.assertEqual(session.state, "FIRST_REVIEW")
        self.assertEqual(session.completed, 1)
        self.tick(session, hardware, now + .1, "v")
        self.tick(session, hardware, now + .2, "s")
        self.assertEqual(session.state, "RECORDING")
        writer.save.assert_called_once()

    def test_fault_before_save_submission_discards_only_current_frames(self):
        session, hardware, writer = self.session("record")
        session.completed = 1
        writer.episodes.append(["previous saved episode"])
        self.tick(session, hardware, 0, "s")
        hardware.hold = Mock(side_effect=MotorHealthError("enable failed"))
        _, event = self.tick(session, hardware, .1, "n")
        self.assertEqual(event["event"], "fault_paused")
        writer.save.assert_not_called()
        writer.discard.assert_called_once()
        self.tick(session, hardware, .3)
        self.tick(session, hardware, .5)
        self.assertEqual(session.completed, 1)
        self.assertEqual(writer.episodes, [["previous saved episode"]])
        self.assertEqual(writer.frames, [])
        writer.discard.assert_called_once()

    def test_pending_limit_discard_is_reused_once(self):
        session, hardware, writer = self.session("record")
        session.state = "LIMIT_PAUSED"
        session.pause_hold = deepcopy(hardware.obs["follower"])
        pending = Future()
        session.pause_future = pending
        self.fault(session, hardware)
        self.assertIs(session.future, pending)
        self.assertIsNone(session.pause_future)
        pending.set_result(None)
        self.tick(session, hardware, .3)
        writer.discard.assert_not_called()
        self.assertEqual(session.completed, 0)

    def test_countdown_movement_and_hold_drift_pause_before_new_goals(self):
        for state in ("COUNTDOWN", "READY", "LIMIT_PAUSED"):
            with self.subTest(state=state):
                session, hardware, _ = self.session()
                self.tick(session, hardware, 0, "r")
                session.state = state
                if state == "LIMIT_PAUSED":
                    session.holding = False
                    session.pause_hold = deepcopy(hardware.obs["follower"])
                hardware.obs["follower"]["elbow_flex"] += 6
                _, event = self.tick(session, hardware, .1)
                self.assertEqual(event["event"], "fault_paused")
                self.assertEqual([c[0] for c in hardware.calls], ["hold", "release"])

    def test_retry_failure_returns_to_pause_with_no_automatic_second_try(self):
        session, hardware, _ = self.session()
        self.tick(session, hardware, 0, "r")
        self.fault(session, hardware)
        self.tick(session, hardware, .3)
        self.tick(session, hardware, .5)
        hardware.hold = Mock(side_effect=ConnectionError("enable ACK failed"))
        _, event = self.tick(session, hardware, .7, "r")
        self.assertEqual(event["event"], "fault_paused")
        self.tick(session, hardware, .9)
        self.tick(session, hardware, 1.1)
        self.tick(session, hardware, 1.3)
        hardware.hold.assert_called_once()

    def test_automatic_episode_end_hold_failure_also_pauses(self):
        session, hardware, writer = self.session("record", episode_time_s=.05)
        self.tick(session, hardware, 0, "s")
        hardware.hold = Mock(side_effect=MotorHealthError("hold failed"))
        _, event = self.tick(session, hardware, .1)
        self.assertEqual(event["event"], "fault_paused")
        writer.discard.assert_called_once()
        writer.save.assert_not_called()

    def test_episode_expiring_during_observation_uses_hold_recovery(self):
        session, hardware, writer = self.session("record", episode_time_s=.15)
        self.tick(session, hardware, 0, "s")
        hardware.hold = Mock(side_effect=MotorHealthError("hold failed"))
        hardware.now = .1
        with patch.object(hardware, "observe", wraps=hardware.observe) as observe:
            def clock():
                return .2 if observe.called else .1
            _, event = adaptive.control_tick(session, hardware, None, "official", clock=clock)
        self.assertEqual(event["event"], "fault_paused")
        writer.discard.assert_called_once()

    def test_return_send_failure_stops_before_commanding_second_arm(self):
        session, hardware, _ = self.session()
        self.tick(session, hardware, 0, "r")
        session.reset_started = -session.settings.countdown_s
        hardware.send = Mock(side_effect=ConnectionError("lost goal packet"))
        _, event = self.tick(session, hardware, .1)
        self.assertEqual(event["event"], "fault_paused")
        self.assertEqual(event["previous_state"], "RETURNING")
        hardware.send.assert_called_once()
        self.assertEqual(hardware.send.call_args.args[0], "follower")
        self.assertEqual(hardware.calls[-1], ("release", None))

    def test_stop_bugs_and_save_failures_are_not_recoverable(self):
        for error in (InterruptedError("stop"), RuntimeError("bug"), ValueError("invalid"), KeyError("missing")):
            with self.subTest(error=error):
                session, hardware, _ = self.session()
                self.tick(session, hardware, 0, "r")
                hardware.error = error
                with self.assertRaises(type(error)):
                    self.tick(session, hardware, .1)
                self.assertEqual(session.state, "COUNTDOWN")
        for error in (OSError("disk full"), ConnectionError("writer"), TimeoutError("writer")):
            with self.subTest(error=error):
                session, hardware, writer = self.session("record")
                writer.defer = True
                self.tick(session, hardware, 0, "s")
                self.tick(session, hardware, .1, "n")
                self.fault(session, hardware, .2)
                session.future.set_exception(error)
                hardware.error = MotorHealthError("still low")
                with self.assertRaisesRegex(RuntimeError, "Dataset save/discard failed"):
                    self.tick(session, hardware, .4)
                self.assertEqual(session.completed, 0)

    def test_normal_manual_fault_still_exits(self):
        session, hardware, _ = self.session()
        self.tick(session, hardware, 0, "s")
        hardware.error = MotorHealthError("manual failure")
        with self.assertRaises(MotorHealthError):
            self.tick(session, hardware, .1)
        self.assertEqual(session.state, "TELEOP")


class RecoveryTransportTests(unittest.TestCase):
    def test_hold_only_classifies_device_ack_errors_as_motor_faults(self):
        for operation in ("write", "read"):
            with self.subTest(operation=operation):
                hardware = test_recovery.RawHardwareTests().hardware()
                hardware.torque["follower"] = False
                if operation == "read":
                    hardware.follower.bus.write.side_effect = ConnectionError("missing ACK")
                getattr(hardware.follower.bus, operation).side_effect = RuntimeError("Input voltage error")
                with self.assertRaises(MotorHealthError):
                    hardware.hold({"leader": pose(), "follower": pose()})
                hardware.leader.bus.sync_write.assert_not_called()
        hardware = test_recovery.RawHardwareTests().hardware()
        bug = RuntimeError("unexpected normalization error")
        hardware.follower.bus.sync_write.side_effect = bug
        with self.assertRaises(RuntimeError) as error:
            hardware.hold({"leader": pose(), "follower": pose()})
        self.assertIs(error.exception, bug)
        self.assertNotIsInstance(error.exception, MotorHealthError)

    def test_release_attempts_both_buses_and_all_motors_without_disconnect(self):
        hardware = test_recovery.RawHardwareTests().hardware()
        hardware.control_deadline = -1
        for arm in hardware.arms.values():
            arm.bus.is_connected = True
            arm.bus._comm_success = 0
            arm.bus._write.return_value = (0, 0)
            arm.bus._read.return_value = (0, 0, 0)
        hardware.leader.bus.sync_write.side_effect = ConnectionError("broadcast lost")
        hardware.leader.bus._write.side_effect = ConnectionError("leader lost")
        with self.assertRaisesRegex(RuntimeError, "leader torque OFF unverified"):
            hardware.release_torque()
        for arm in hardware.arms.values():
            arm.bus.sync_write.assert_called_once_with("Torque_Enable", dict.fromkeys(JOINTS, 0),
                                                      normalize=False, num_retry=0)
            self.assertEqual(arm.bus._write.call_count, 6)
            self.assertTrue(all(c.args[0] == 40 and c.args[3] == 0 for c in arm.bus._write.call_args_list))
            arm.bus.disconnect.assert_not_called()
        for camera in hardware.follower.cameras.values():
            camera.disconnect.assert_not_called()
        self.assertEqual(hardware.torque, dict(leader=False, follower=False))

    def test_full_feedback_low_voltage_and_communication_are_typed(self):
        hardware = test_recovery.RawHardwareTests().hardware()
        rows = test_recovery.RawHardwareTests().feedback(0)
        rows["gripper"]["Present_Voltage"] = 35
        # Use follower to avoid the leader's confirmation wait.
        hardware.torque["follower"] = False
        with patch("diagnose_teleop.feedback", return_value=rows):
            with self.assertRaises(MotorHealthError):
                hardware.read_healthy_feedback("follower")
        with patch("diagnose_teleop.feedback", side_effect=RuntimeError("bad serial packet")):
            with self.assertRaises(ConnectionError):
                hardware.read_healthy_feedback("leader")
        with patch("diagnose_teleop.feedback", side_effect=InterruptedError("stop")):
            with self.assertRaises(InterruptedError):
                hardware.read_healthy_feedback("leader")
        for arm in hardware.arms.values():
            arm.bus.write.assert_not_called()
            arm.bus.sync_write.assert_not_called()


class RecoveryLoopTests(unittest.TestCase):
    def test_run_keeps_window_and_writer_alive_until_explicit_stop(self):
        # Exercise run itself: a helper-only test would miss an unwired controller.
        saved = profile()
        hardware = RecoveryHardware(saved.home)
        hardware.follower = Mock()
        hardware.connect = Mock()
        writer = FakeWriter()
        writer.check = Mock()
        writer.close = Mock(side_effect=lambda: hardware.calls.append(("writer_close",)))
        frames, commands = queue.Queue(), queue.Queue()
        for channel in (frames, commands):
            channel.cancel_join_thread = Mock()
            channel.close = Mock()
        ready, stop, closed = threading.Event(), threading.Event(), threading.Event()
        ready.set()
        viewer = Mock()
        viewer.is_alive.return_value = True
        context = Mock()
        context.Queue.side_effect = [frames, commands]
        context.Event.side_effect = [ready, stop, closed]
        context.Process.return_value = viewer
        cfg = SimpleNamespace(robot=SimpleNamespace(cameras={}),
                              dataset=SimpleNamespace(num_episodes=2, episode_time_s=120))
        clock = [1.0]
        ticks, packets, intervals = [], [], []
        controller = adaptive.control_tick
        observe = hardware.observe
        sessions = []

        def new_session(*args, **kwargs):
            session = Session(*args, **kwargs)
            sessions.append(session)
            return session

        def read(**kwargs):
            hardware.now = clock[0]
            # The first reset fails on its second tick. Next poll still fails.
            if len(ticks) in (1, 2):
                raise MotorHealthError("persistent voltage fault")
            return observe(**kwargs)

        def wait(_stop, interval):
            intervals.append(interval)
            while not frames.empty():
                packets.append(frames.get_nowait())
            session = sessions[0]
            ticks.append(session.state)
            self.assertFalse(closed.is_set())
            writer.close.assert_not_called()
            self.assertNotIn(("close",), hardware.calls)
            clock[0] += .2
            if len(ticks) == 3:
                commands.put(("r", clock[0], "COUNTDOWN"))  # stale phase, must be ignored
            if len(ticks) == 5:
                self.assertTrue(session.fault_ready)
            if len(ticks) == 6:
                self.assertEqual([c[0] for c in hardware.calls], ["hold", "release"])
                commands.put(("r", clock[0], "FAULT_PAUSED"))
            if len(ticks) == 7:
                stop.set()  # the GUI's Q/Esc/close action sets this same event
            return stop.is_set()

        commands.put(("r", clock[0], "START"))
        with tempfile.TemporaryDirectory(dir=adaptive.ROOT / "outputs") as temp:
            root = Path(temp)
            home = root / "home.json"
            home.write_text(json.dumps(saved.data))
            with patch.object(adaptive, "ROOT", root), \
                 patch.object(adaptive.mp, "get_context", return_value=context), \
                 patch.object(adaptive, "Hardware", return_value=hardware), \
                 patch.object(adaptive, "make_writer", return_value=writer), \
                 patch.object(adaptive, "review_summary", return_value={}), \
                 patch.object(adaptive, "Session", side_effect=new_session), \
                 patch.object(adaptive.time, "monotonic", side_effect=lambda: clock[0]), \
                 patch.object(adaptive.StopRequest, "wait", wait), \
                 patch.object(hardware, "observe", side_effect=read), \
                 patch.object(adaptive, "control_tick", side_effect=lambda *args: controller(*args, clock=lambda: clock[0])), \
                 redirect_stdout(StringIO()):
                handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
                adaptive.run(cfg, Settings(), IDENTITIES, BOUNDS, home, saved, {}, "record", feedback_mode="official")
                self.assertEqual(handlers, {sig: signal.getsignal(sig) for sig in handlers})
            rows = [json.loads(line) for path in root.rglob("events.jsonl") for line in path.read_text().splitlines()]
        self.assertEqual(ticks, ["COUNTDOWN", *(["FAULT_PAUSED"] * 5), "COUNTDOWN"])
        self.assertTrue(all(abs(x - adaptive.FAULT_POLL_SECONDS) < 1e-9 for x in intervals[1:6]))
        faults = [row for row in rows if row.get("event") == "fault_paused"]
        self.assertEqual(len(faults), 1)
        self.assertEqual(sum(row.get("event") == "reset_retry" for row in rows), 1)
        self.assertEqual(sum(row.get("event") == "reset_plan" for row in rows), 2)
        self.assertFalse(any(row.get("event") == "fault" for row in rows))
        self.assertEqual(sum(not packet["observation_valid"] for packet in packets), 2)
        self.assertTrue(closed.is_set())
        self.assertEqual([c[0] for c in hardware.calls], ["hold", "release", "hold", "close", "writer_close"])
        writer.close.assert_called_once()

    def test_real_writer_drains_inflight_save_once_while_paused(self):
        entered, release = threading.Event(), threading.Event()
        dataset = Mock()
        dataset.has_pending_frames.return_value = False
        def save():
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test save not released")
        dataset.save_episode.side_effect = save
        writer = adaptive.AsyncDatasetWriter(dataset, "offline", lambda value, task: value,
                                             review_first=lambda _: {"report": "saved"})
        saved = profile()
        hardware = RecoveryHardware(saved.home)
        session = Session(hardware, writer, Settings(), IDENTITIES, BOUNDS, "record", saved)
        runner = RecoveryTests()
        try:
            runner.tick(session, hardware, 0, "s")
            runner.tick(session, hardware, .1, "n")
            self.assertTrue(entered.wait(1))
            runner.fault(session, hardware, .2)
            future = session.future
            release.set()
            future.result(timeout=2)
            hardware.error = MotorHealthError("still bad")
            runner.tick(session, hardware, .4)
            runner.tick(session, hardware, .6)
            self.assertEqual(session.completed, 1)
            self.assertEqual(session.review_report, {"report": "saved"})
            self.assertEqual(session.state, "FAULT_PAUSED")
        finally:
            release.set()
            writer.close()
        dataset.save_episode.assert_called_once()
        dataset.clear_episode_buffer.assert_not_called()
        self.assertEqual(writer.saved, 1)


class RecoveryPreviewTests(unittest.TestCase):
    def test_invalid_packet_clears_existing_camera_frames_and_measured_values(self):
        saved = profile()
        session = Session(RecoveryHardware(saved.home), None, Settings(), IDENTITIES, BOUNDS, "teleop", saved)
        session.enter_fault_pause(MotorHealthError("voltage"))
        packet = adaptive.preview_packet(session, None, "test", 1.0)
        self.assertFalse(packet["observation_valid"])
        self.assertIsNone(packet["leader"])
        self.assertIsNone(packet["follower"])
        self.assertIsNone(packet["requested"])
        self.assertIsNone(packet["target"])
        window = PreviewWindow.__new__(PreviewWindow)
        window.closed = threading.Event()
        window.frames = queue.Queue()
        window.frames.put(packet)
        window.window = Mock()
        window.current_state = "RETURNING"
        window.camera_frames = {"top": object(), "wrist": object()}
        window.images = {name: (Mock(), i) for i, name in enumerate(window.camera_frames)}
        window.draw_table = Mock()
        window.metadata = Mock()
        window.details = Mock()
        window.state_label = Mock()
        window.last_status = None
        with patch("so101_preview.time.monotonic", return_value=1.0):
            window.refresh()
        self.assertEqual(window.camera_frames, {})
        for canvas, item in window.images.values():
            canvas.itemconfigure.assert_called_once_with(item, image="")
        window.draw_table.assert_called_once()
        self.assertIn("故障暂停", window.state_label.set.call_args.args[0])
        window.window.destroy.assert_not_called()
        window.window.after.assert_called_once()


if __name__ == "__main__":
    unittest.main()
