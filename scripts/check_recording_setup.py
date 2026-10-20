#!/usr/bin/env python3
"""Offline recording smoke test. Uses synthetic images/states; no device access."""

import json
import logging
import multiprocessing as mp
from pathlib import Path
import queue
import time

import av
import numpy as np
import pyarrow.parquet as pq

from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts
from record_with_preview import JOINTS, ROOT, PreviewFeed, load_config, preview_window


def main():
    cfg = load_config(ROOT / "configs/record_stack_white_on_red_trial.proposed.json")
    # Construction reads local configuration only; neither connect() nor preflight is called.
    robot = make_robot_from_config(cfg.robot)
    processors = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(processors[0], create_initial_features(action=robot.action_features), use_videos=True),
        aggregate_pipeline_dataset_features(processors[2], create_initial_features(observation=robot.observation_features), use_videos=True),
    )
    root = ROOT / "outputs/recording-smoke" / time.strftime("%Y%m%d-%H%M%S")
    dataset = LeRobotDataset.create(
        repo_id="local/synthetic_recording_smoke", root=root, fps=cfg.dataset.fps,
        robot_type=robot.name, features=features, use_videos=True,
        streaming_encoding=cfg.dataset.streaming_encoding,
        rgb_encoder=cfg.dataset.rgb_encoder, encoder_threads=cfg.dataset.encoder_threads,
        encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
        image_writer_processes=0, image_writer_threads=0,
    )
    context = mp.get_context("spawn")
    frames = context.Queue(maxsize=1)
    frames.cancel_join_thread()
    ready, closed = context.Event(), context.Event()
    viewer = context.Process(target=preview_window, args=(frames, ready, closed), daemon=True)
    feed = PreviewFeed(frames, 3)
    processors[1].after_step_hooks.append(feed)
    viewer.start()
    samples = []
    try:
        assert ready.wait(10), "Synthetic preview window failed to open"
        for episode in range(3):
            feed.phase = f"模拟数据验证（无硬件连接） · 第 {episode + 1} / 3 段"
            for i in range(90):
                start = time.perf_counter()
                top = np.zeros((480, 640, 3), dtype=np.uint8)
                wrist = np.zeros_like(top)
                top[:, :, :] = (30, 80, 120)
                wrist[:, :, :] = (120, 60, 30)
                x = i * 5 % 540
                top[160:260, x:x + 100] = 255
                wrist[200:300, x:x + 100] = (255, 0, 0)
                state = {f"{name}.pos": float(i + j + episode) for j, name in enumerate(JOINTS)}
                action = {key: val + 0.5 for key, val in state.items()}
                obs = {**state, "top": top, "wrist": wrist}
                result = processors[1]((action, obs))
                assert result == action, "Preview changed an action"
                assert all(obs[key] == val for key, val in state.items()), "Preview changed an observation"
                dataset.add_frame({
                    **build_dataset_frame(features, obs, prefix="observation"),
                    **build_dataset_frame(features, result, prefix="action"),
                    "task": "SYNTHETIC TEST — NOT A ROBOT DEMONSTRATION",
                })
                samples.append((time.perf_counter() - start) * 1000)
                time.sleep(max(0, 1 / 30 - (time.perf_counter() - start)))
            dataset.save_episode()
        dataset.finalize()
    finally:
        closed.set()
        viewer.join(2)
        if viewer.is_alive():
            viewer.terminate()
            viewer.join(2)
        frames.close()
        dataset.finalize()

    # A blocked preview must drop its update, never wait for the display.
    blocked = queue.Queue(maxsize=1)
    blocked.put("occupied")
    stalled_feed = PreviewFeed(blocked, 3)
    processors[1].after_step_hooks[:] = [stalled_feed]
    assert processors[1]((action, obs)) == action
    assert blocked.qsize() == 1

    info = json.loads((root / "meta/info.json").read_text())
    assert info["total_episodes"] == 3 and info["total_frames"] == 270, info
    tables = [pq.read_table(p) for p in sorted((root / "data").rglob("*.parquet"))]
    rows = [row for table in tables for row in table.to_pylist()]
    assert len(rows) == 270
    for episode in range(3):
        episode_rows = [row for row in rows if row["episode_index"] == episode]
        assert len(episode_rows) == 90
        assert [row["frame_index"] for row in episode_rows] == list(range(90))
        np.testing.assert_allclose([row["timestamp"] for row in episode_rows], np.arange(90) / 30, atol=1e-6)
        for row in episode_rows:
            np.testing.assert_allclose(np.array(row["action"]) - row["observation.state"], 0.5)
    video_counts = {}
    for camera in ("top", "wrist"):
        count = 0
        for path in (root / "videos" / f"observation.images.{camera}").rglob("*.mp4"):
            with av.open(str(path)) as container:
                for frame in container.decode(video=0):
                    assert (frame.width, frame.height) == (640, 480)
                    count += 1
        assert count == 270, (camera, count)
        video_counts[camera] = count
    result = {
        "synthetic_only": True, "hardware_opened": False, "root": str(root),
        "episodes": 3, "frames": 270, "decoded_video_frames": video_counts,
        "capture_work_ms_median": float(np.median(samples)),
        "capture_work_ms_p95": float(np.percentile(samples, 95)),
        "preview_action_identity_and_full_queue": "passed",
    }
    (ROOT / "logs/recording-setup-check.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
