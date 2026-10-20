"""Trace the real ACT queue and Feetech conversion using a fake serial transport."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from act_execution_trace import ExecutionTrace, VideoJournal
from analyze_execution_trace import analyze, opening_events
from act_trial_loop import ROOT, seed_before_configure
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.rollout.robot_wrapper import ThreadSafeRobot

KEYS = [f"{n}.pos" for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")]


class ToyACT(ACTPolicy):
    def __init__(self, n):
        torch.nn.Module.__init__(self)
        self.config = SimpleNamespace(n_action_steps=n, chunk_size=100, temporal_ensemble_coeff=None,
                                      type="act", pretrained_path="synthetic-test-only")
        self.generated = 0
        self.reset()

    def predict_action_chunk(self, _batch):
        self.generated += 1
        return torch.arange(600, dtype=torch.float32).reshape(1, 100, 6) / 100 + self.generated


class FakeRobot:
    send_action = SOFollower.send_action
    is_connected = True

    def __init__(self):
        self.cameras = {}
        self.config = SimpleNamespace(max_relative_target=None)
        motors = {k[:-4]: Motor(i+1, "sts3215", MotorNormMode.RANGE_0_100 if i == 5 else MotorNormMode.DEGREES)
                  for i, k in enumerate(KEYS)}
        calibration = {name: MotorCalibration(m.id, 0, 0, 1000, 3000) for name, m in motors.items()}
        self.bus = FeetechMotorsBus("/dev/not-opened-by-test", motors, calibration)
        self.bus.port_handler = SimpleNamespace(is_open=True)
        self.bus.sync_writer = Mock()
        self.bus.sync_writer.txPacket.return_value = 0

    def get_observation(self):
        return {k: 10.0 for k in KEYS}


def context(n):
    policy, robot = ToyACT(n), FakeRobot()
    engine = SimpleNamespace(get_action=lambda obs: (policy.select_action(obs) * 10 + 20).squeeze(0))
    cfg = SimpleNamespace(policy=policy.config, inference=SimpleNamespace(type="sync"), interpolation_multiplier=1, fps=30)
    return SimpleNamespace(policy=SimpleNamespace(policy=policy, inference=engine, postprocessor=SimpleNamespace(steps=[])),
                           hardware=SimpleNamespace(robot_wrapper=ThreadSafeRobot(robot)),
                           data=SimpleNamespace(ordered_action_keys=KEYS), runtime=SimpleNamespace(cfg=cfg))


def execute(ctx, count):
    outputs = []
    for _ in range(count):
        obs = ctx.hardware.robot_wrapper.get_observation()
        action = ctx.policy.inference.get_action(obs)
        target = dict(zip(KEYS, action.tolist()))
        target["gripper.pos"] += 1  # Synthetic processor change must be visible separately.
        outputs.append(ctx.hardware.robot_wrapper.send_action(target))
    return outputs


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT / ".cache/tmp")
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name) / "trace"

    def test_real_act_queue_at_100_and_10_preserves_outputs_and_chunk_indices(self):
        for n in (100, 10):
            with self.subTest(n=n):
                expected = execute(context(n), 205)
                ctx = context(n)
                dest = self.directory / str(n)
                with ExecutionTrace(ctx, dest, video=False):
                    actual = execute(ctx, 205)
                self.assertEqual(actual, expected)
                rows = [json.loads(s) for s in (dest / "steps.jsonl").read_text().splitlines()]
                self.assertEqual(len(rows), 205)
                for i, row in enumerate(rows):
                    self.assertEqual((row["chunk_id"], row["chunk_step"]), (i // n + 1, i % n))
                    self.assertTrue(row["selected_matches_chunk"])
                    self.assertAlmostEqual(row["after_robot_processors"]["gripper.pos"] - row["after_policy_postprocess"][5], 1)
                    self.assertLessEqual(row["sent_gripper_target"], 100)  # real bus clamp captured
                self.assertNotIn("predict_action_chunk", vars(ctx.policy.policy))
                self.assertNotIn("send_action", vars(ctx.hardware.robot_wrapper.inner))
                report = analyze(dest)
                self.assertEqual(report["selected_chunk_mismatches"], 0)
                self.assertEqual(report["actions_sent"], 205)

    def test_failed_send_is_not_recorded_as_executed(self):
        ctx = context(100)
        ctx.hardware.robot_wrapper.inner.bus.sync_writer.txPacket.return_value = -1001
        with self.assertRaises(ConnectionError):
            with ExecutionTrace(ctx, self.directory, video=False):
                execute(ctx, 1)
        row = json.loads((self.directory / "steps.jsonl").read_text())
        self.assertFalse(row["sent"])
        self.assertIn("error", row)

    def test_video_runs_without_control_ticks_and_has_timestamps(self):
        import av
        self.directory.mkdir()
        camera = SimpleNamespace(frame_lock=threading.Lock(), latest_timestamp=None, latest_frame=None)
        video = VideoJournal({"wrist": camera}, self.directory, time.perf_counter())
        video.start()
        try:
            for i in range(25):
                with camera.frame_lock:
                    camera.latest_frame = np.full((120, 160, 3), i, dtype=np.uint8)
                    camera.latest_timestamp = time.perf_counter()
                time.sleep(.03)
        finally:
            video.close()
        self.assertNotIn("error", video.report)
        self.assertGreater(video.report["wrist"]["frames"], 10)
        with av.open(str(self.directory / "wrist.mkv")) as container:
            frames = list(container.decode(video=0))
        self.assertEqual(len(frames), video.report["wrist"]["frames"])
        self.assertGreater(float(frames[-1].pts * frames[-1].time_base), .4)

    def test_opening_candidates_include_feedback_only_and_ignore_closing(self):
        rows = [{"index": i, "sent": True, "packet_finished_t_s": i/30,
                 "observe_started_t_s": i/30-.01, "sent_gripper_target": 5,
                 "feedback": {"gripper.pos": 10 + max(0, min(i-20, 10))}}
                for i in range(60)]
        events = opening_events(rows)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["sources"], ["feedback"])
        for row in rows:
            row["feedback"]["gripper.pos"] = 100-row["index"]
        self.assertEqual(opening_events(rows), [])

    def test_seed_is_before_official_configure_and_methods_are_restored(self):
        sequence = Mock()
        sequence.read.return_value = {"gripper": 1836}
        bus = SimpleNamespace(disable_torque=sequence.off, sync_read=sequence.read, sync_write=sequence.seed)
        robot = SimpleNamespace(bus=bus, configure=sequence.configure, calibrate=Mock())
        before = robot.configure
        with seed_before_configure(robot, 2):
            robot.configure()
            with self.assertRaisesRegex(RuntimeError, "calibration"):
                robot.calibrate()
        self.assertIs(robot.configure, before)
        self.assertEqual([c[0] for c in sequence.mock_calls], ["off", "read", "seed", "configure"])


if __name__ == "__main__":
    unittest.main()
