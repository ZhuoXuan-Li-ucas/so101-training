#!/usr/bin/env python3
"""Offline end-to-end collection test: simulated arms/images, real LeRobot writer."""

import json
import multiprocessing as mp
from pathlib import Path
import queue
import time
from unittest.mock import patch

import av
import numpy as np
import pyarrow.parquet as pq

from so101_adaptive import ROOT, load_config, make_writer, preview_window
from so101_session import JOINTS, HomeProfile, Session, WristMapping


class SimulatedArms:
    def __init__(self, home):
        self.observation = {role: dict(home[role]) for role in ("leader", "follower")}
        self.observation["images"] = {
            "top": np.full((480, 640, 3), (40, 100, 180), dtype=np.uint8),
            "wrist": np.full((480, 640, 3), (180, 80, 40), dtype=np.uint8),
        }
        self.goals_sent = 0

    def hold(self, obs):
        pass

    def follow(self, obs):
        pass

    def hold_follower(self, obs):
        pass

    def send(self, role, pose):
        self.observation[role] = dict(pose)
        self.goals_sent += 1
        return dict(pose)


def main():
    import argparse
    from lerobot.datasets import LeRobotDataset
    from lerobot.robots import make_robot_from_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/so101_recovery.proposed.json")
    parser.add_argument("--preview", action="store_true", help="Also open the desktop with synthetic images only")
    args = parser.parse_args()
    cfg, settings, identities, bounds, _, _, _ = load_config(args.config, "teleop")
    root = ROOT / "outputs/adaptive-smoke" / time.strftime("%Y%m%d-%H%M%S")
    cfg.dataset.root = root
    cfg.dataset.repo_id = "local/synthetic_adaptive_smoke"
    cfg.dataset.single_task = "SYNTHETIC TEST; NOT A REAL ROBOT DEMONSTRATION"
    mapping = WristMapping(30, bounds)
    leader = {k: 70.0 if k == "gripper" else -30.0 if k == "wrist_roll" else 0.0 for k in JOINTS}
    profile = HomeProfile({"version": 2, "kind": "paired_home", "identities": identities,
                           "wrist_offset_deg": 30, "home": {"leader": leader, "follower": mapping(leader)}},
                          identities, bounds, settings)
    hardware = SimulatedArms(profile.home)
    # Assert that a regression cannot quietly turn this test into hardware I/O.
    with patch("lerobot.motors.motors_bus.MotorsBus.connect", side_effect=AssertionError("hardware forbidden")), \
         patch("lerobot.cameras.opencv.camera_opencv.OpenCVCamera.connect", side_effect=AssertionError("camera forbidden")):
        robot = make_robot_from_config(cfg.robot)
        writer = make_writer(cfg, robot, review_first=True)
        session = Session(hardware, writer, settings, identities, bounds, "record", profile, episodes=3,
                          require_first_review=True)
        now = 0.0
        lengths = [9, 13, 17]
        try:
            for length in lengths:
                # Exercise cancellation of a real streaming video episode. Its
                # frames must never be appended to the replacement episode.
                session.step("s", hardware.observation, now)
                valid = dict(hardware.observation["leader"])
                hardware.observation["leader"]["wrist_roll"] = 170
                now += 1 / 30
                session.step(None, hardware.observation, now)
                assert session.state == "LIMIT_PAUSED"
                hardware.observation["leader"] = valid
                deadline = time.monotonic() + 30
                while session.state == "LIMIT_PAUSED":
                    if time.monotonic() > deadline:
                        raise TimeoutError("Streaming discard stuck")
                    now += 1 / 30
                    session.step("m", hardware.observation, now)
                    time.sleep(.002)
                assert session.state == "MANUAL"
                hardware.observation["leader"].update(shoulder_pan=-15, wrist_roll=-50)
                hardware.observation["follower"].update(shoulder_pan=12, wrist_roll=20)
                now += 1 / 30
                session.step("r", hardware.observation, now)
                assert session.reset_plan.origin["leader"]["shoulder_pan"] == -15
                assert session.reset_plan.origin["follower"]["shoulder_pan"] == 12
                while session.state != "READY":
                    if time.monotonic() > deadline:
                        raise TimeoutError("Pre-record reset stuck")
                    now += 1 / 30
                    session.step(None, hardware.observation, now)
                    time.sleep(.002)
                now += 1 / 30
                session.step("s", hardware.observation, now)
                for _ in range(length - 1):
                    now += 1 / 30
                    session.step(None, hardware.observation, now)
                hardware.observation["leader"].update(shoulder_pan=20, wrist_roll=-40)
                hardware.observation["follower"].update(shoulder_pan=-10, wrist_roll=12)
                now += 1 / 30
                session.step("n", hardware.observation, now)
                deadline = time.monotonic() + 30
                while session.state not in ("READY", "DONE", "FIRST_REVIEW"):
                    if time.monotonic() > deadline:
                        raise TimeoutError(f"Synthetic episode stuck in {session.state}")
                    writer.check()
                    now += 1 / 30
                    session.step(None, hardware.observation, now)
                    time.sleep(0.002)
                if session.state == "FIRST_REVIEW":
                    assert session.review_report["frames"] == length
                    now += 1 / 30
                    session.step("s", hardware.observation, now)
                    assert session.state == "FIRST_REVIEW"
                    now += 1 / 30
                    session.step("v", hardware.observation, now)
                    assert session.state == "READY"
            assert session.state == "DONE" and session.completed == 3
        finally:
            writer.close()

    info = json.loads((root / "meta/info.json").read_text())
    assert info["codebase_version"] == "v3.0"
    assert info["total_episodes"] == 3 and info["total_frames"] == sum(lengths)
    rows = [r for p in sorted((root / "data").rglob("*.parquet")) for r in pq.read_table(p).to_pylist()]
    for episode, length in enumerate(lengths):
        data = [row for row in rows if row["episode_index"] == episode]
        assert len(data) == length
        assert [r["frame_index"] for r in data] == list(range(length))
        np.testing.assert_allclose([r["timestamp"] for r in data], np.arange(length) / 30, atol=1e-6)
        # Leader wrist = -30 degrees; the saved mapped follower target must be 0.
        np.testing.assert_allclose([r["action"][4] for r in data], 0, atol=1e-6)
    videos = {}
    for name in ("top", "wrist"):
        count = 0
        for path in (root / "videos" / f"observation.images.{name}").rglob("*.mp4"):
            with av.open(str(path)) as container:
                for frame in container.decode(video=0):
                    assert (frame.width, frame.height) == (640, 480)
                    count += 1
        assert count == sum(lengths), (name, count)
        videos[name] = count
    loaded = LeRobotDataset(cfg.dataset.repo_id, root=root, video_backend="pyav")
    assert len(loaded) == sum(lengths)
    first = loaded[0]
    assert tuple(first["observation.images.top"].shape) == (3, 480, 640)
    assert tuple(first["action"].shape) == (6,)

    if args.preview:
        ctx = mp.get_context("spawn")
        frames, commands = ctx.Queue(1), ctx.Queue(4)
        frames.cancel_join_thread()
        ready, stop, closed = ctx.Event(), ctx.Event(), ctx.Event()
        process = ctx.Process(target=preview_window, args=(frames, commands, ready, stop, closed), daemon=True)
        process.start()
        try:
            assert ready.wait(10), "Synthetic desktop preview did not open"
            for _ in range(20):
                if stop.is_set():
                    break
                try:
                    frames.put_nowait({**hardware.observation, "target": profile.home["follower"],
                                       "time": time.monotonic(), "state": "SYNTHETIC",
                                       "message": "离线预览测试：模拟图像，无硬件连接，4 秒后自动关闭"})
                except queue.Full:
                    pass
                time.sleep(0.2)
        finally:
            closed.set()
            process.join(2)
            if process.is_alive():
                process.terminate()
                process.join(2)
            frames.close()
            commands.close()

    result = {"hardware_opened": False, "root": str(root), "episodes": 3, "lengths": lengths,
              "frames": len(rows), "decoded_video_frames": videos, "lerobot_reload": "passed",
              "mapped_action_labels": "passed", "reset_frames_excluded": "passed",
              "paused_episode_video_discard": "passed; three aborted attempts excluded",
              "first_episode_review_gate": "passed",
              "direct_reset_independent_current_poses": "passed; no fixed entry or taught path",
              "last_episode_reset": "passed", "desktop_preview": "passed" if args.preview else "not requested"}
    (root / "smoke-report.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
