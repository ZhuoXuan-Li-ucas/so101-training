#!/usr/bin/env python3
"""Reviewed SO-101 wrist/home setup, direct reset and recording with camera preview.

--check accesses files only, never serial ports/cameras and never creates a dataset.
Other modes change torque/position goals and require command review before use.
"""

from __future__ import annotations

import argparse
from concurrent.futures import Future
from dataclasses import asdict
import hashlib
import json
import logging
import multiprocessing as mp
from pathlib import Path
import queue
import re
import signal
import threading
import time
from typing import Callable

from so101_session import ENCODER_MAX, POSITION_TOLERANCE_COUNTS, JOINTS, HomeProfile, PositionLimit, ResetMotionError, Session, Settings, calibration_context, check_pose
from so101_voltage import CONFIRM_SECONDS, NORMAL_SAMPLES, RECHECK_SECONDS, LeaderVoltageConfirmation, MotorHealthError, health_issues

ROOT = Path(__file__).resolve().parents[1]
FAULT_POLL_SECONDS = 0.2
RESET_RECOVERY_ERRORS = (MotorHealthError, ConnectionError, TimeoutError, PositionLimit, ResetMotionError)


def checked_leader_pid(values):
    """An explicit readback expectation, never values to configure on motors."""
    values = tuple(values)
    if len(values) != 3 or any(type(v) is not int or not 0 <= v <= 255 for v in values):
        raise ValueError("Expected leader PID must be three integer register values in 0..255")
    return values


def flush_voltage_events(hardware, stream, state):
    confirmation = getattr(hardware, "voltage_confirmation", None)
    if confirmation is None or not confirmation.events:
        return
    for event in confirmation.events:
        stream.write(json.dumps({**event, "state": state, "time": time.time()}, ensure_ascii=False) + "\n")
    stream.flush()
    confirmation.events.clear()


def workspace_path(value: str, subtree: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = ROOT / path
    resolved = path.resolve()
    if not resolved.is_relative_to(ROOT / subtree) or resolved == ROOT / subtree:
        raise ValueError(f"Path must be below {ROOT / subtree}: {path}")
    return resolved


def dataset_path(name: str) -> Path:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name):
        raise ValueError("Dataset name must contain only letters, digits, underscores or hyphens")
    return workspace_path(f"data/datasets/{name}", "data/datasets")


def load_config(path: Path, mode: str, *, dataset_name=None, episodes=None):
    import draccus
    from lerobot.scripts.lerobot_record import RecordConfig

    raw = json.loads(path.read_text())
    if set(raw) != {"recording", "session"}:
        raise ValueError("Expected recording and session configuration sections")
    if dataset_name is not None or episodes is not None:
        if mode != "record":
            raise ValueError("Dataset overrides require record mode")
        if dataset_name is not None:
            raw["recording"]["dataset"].update(repo_id=f"local/{dataset_name}", root=str(dataset_path(dataset_name)))
        if episodes is not None:
            if type(episodes) is not int or episodes < 1:
                raise ValueError("Episode count must be a positive integer")
            raw["recording"]["dataset"]["num_episodes"] = episodes
    cfg = draccus.decode(RecordConfig, raw["recording"])
    options = dict(raw["session"])
    if "profile" in options or "home" not in options:
        raise ValueError("Old taught-return configuration is unsupported; select the new home configuration")
    profile_path = workspace_path(options.pop("home"), "data/motion_profiles")
    settings = Settings(**options)
    # These SO-101 aliases share classes with SO-100; .type returns the first
    # registered alias, so validate the explicit input rather than that property.
    if raw["recording"]["robot"]["type"] != "so101_follower" or raw["recording"]["teleop"]["type"] != "so101_leader":
        raise ValueError("Only the configured SO-101 arm pair is supported")
    if not cfg.robot.use_degrees or not cfg.teleop.use_degrees or cfg.robot.max_relative_target is not None:
        raise ValueError("Both arms must use degrees; normal teleop max_relative_target must be null")
    if cfg.resume or cfg.dataset.push_to_hub or not cfg.dataset.no_stamp:
        raise ValueError("Use a new local dataset: no resume, upload or automatic renaming")
    root = workspace_path(str(cfg.dataset.root), "data/datasets")
    if mode == "record" and root.exists():
        raise FileExistsError(f"Dataset already exists; refusing to overwrite: {root}")
    if cfg.dataset.fps != settings.fps or cfg.dataset.num_episodes < 1 or cfg.dataset.episode_time_s <= 0:
        raise ValueError("Invalid recording duration/count/fps")
    if cfg.dataset.reset_time_s != 0:
        raise ValueError("reset_time_s must be 0; reset and manual start are managed by the session")
    if not cfg.dataset.streaming_encoding or not cfg.dataset.video or cfg.display_data:
        raise ValueError("Use streaming video and the shared desktop preview")
    if set(cfg.robot.cameras) != {"top", "wrist"}:
        raise ValueError("Exactly top and wrist cameras are required")
    for camera in cfg.robot.cameras.values():
        if (camera.width, camera.height, camera.fps) != (640, 480, 30):
            raise ValueError("This preview requires 640x480 at 30 fps")
    for arm in (cfg.teleop, cfg.robot):
        workspace_path(str(Path(arm.calibration_dir) / f"{arm.id}.json"), "data/calibration")
        if not str(arm.port).startswith("/dev/serial/by-id/"):
            raise ValueError("Use stable serial by-id paths")
    identities, bounds = calibration_context(cfg)
    profile = None
    if profile_path.exists():
        if mode == "setup":
            raise FileExistsError(f"Refusing to replace saved wrist/home profile: {profile_path}")
        profile = HomeProfile(json.loads(profile_path.read_text()), identities, bounds, settings)
    for role, arm in (("leader", cfg.teleop), ("follower", cfg.robot)):
        status_path = ROOT / "data/calibration/status" / f"{role}-{arm.id}.json"
        if status_path.exists():
            status = json.loads(status_path.read_text())
            if status["state"] != "complete":
                raise ValueError(f"{role}: calibration incomplete; finish passive calibration before running")
            if Path(status["calibration_file"]).resolve() != (Path(arm.calibration_dir) / f"{arm.id}.json").resolve():
                raise ValueError(f"{role}: select the new calibration file: {status['calibration_file']}")
    return cfg, settings, identities, bounds, profile_path, profile, raw


def load_wrist_reference(path, identities, bounds, settings):
    path = workspace_path(str(path), "data/motion_profiles")
    content = path.read_bytes()
    source = HomeProfile(json.loads(content), identities, bounds, settings)
    return {"path": str(path), "sha256": hashlib.sha256(content).hexdigest(),
            "offset_deg": source.mapping.offset}


def detailed_feedback_required(feedback_mode, session):
    """Autonomous holding/reset keeps full checks; manual collection can use native sync reads."""
    return feedback_mode == "full" or session.holding or session.state in (
        "COUNTDOWN", "RETURNING", "LIMIT_PAUSED", "SAVING", "FIRST_REVIEW", "DONE", "FAULT_PAUSED")


def control_tick(session, hardware, command, feedback_mode, clock=time.monotonic):
    """One read/step; device faults during reset/hold stop motors before pausing.

    Programming errors, writer failures and operator stop are not swallowed.
    No pending R survives an unhealthy read. Paused polling performs no writes.
    """
    paused = session.state == "FAULT_PAUSED"
    recoverable_context = paused or (session.mode != "follow" and session.profile is not None and (
        session.holding or command in ("n", "r") or session.state in (
            "COUNTDOWN", "RETURNING", "SAVING", "FIRST_REVIEW", "DONE", "LIMIT_PAUSED")))
    obs = None
    try:
        try:
            obs = hardware.observe(detailed=detailed_feedback_required(feedback_mode, session))
        except RESET_RECOVERY_ERRORS as exc:
            if not paused:
                raise
            session.fault_paused_step(None, None, clock(), read_error=exc)
            return None, None
        now = clock()
        # The episode may expire during observe(). Its ensuing automatic hold
        # belongs to the same recovery path as a manually requested hold/reset.
        recoverable_context = recoverable_context or (
            session.mode == "record" and session.state == "RECORDING"
            and now - session.started >= session.episode_time_s)
        session.step(command, obs, now)
    except RESET_RECOVERY_ERRORS as exc:
        if not recoverable_context:
            raise
        previous_state = session.state
        # Emergency torque release must not be blocked by an expired motion deadline.
        hardware.control_deadline = None
        release_error = None
        try:
            hardware.release_torque()
        except Exception as release_exc:
            release_error = str(release_exc)
        session.enter_fault_pause(exc, release_error)
        return None, {"event": "fault_paused", "previous_state": previous_state,
                      "error": repr(exc), "torque_release_error": release_error,
                      "raw": obs.get("raw") if obs else None,
                      "health": obs.get("health") if obs else None}
    if paused and session.state != "FAULT_PAUSED":
        return obs, {"event": "reset_retry", "explicit_key": command,
                     "saved_episodes": session.completed}
    if paused and not session.fault_observation_valid:
        return None, None
    return obs, None


def review_summary(cfg, settings, profile_path, profile, mode, expected_leader_pid=(32, 0, 32), wrist_reference=None,
                   feedback_mode="full") -> dict:
    """Everything that can write motors is declared here and in the runbook."""
    return {
        "mode": mode, "home_ready": profile is not None,
        "can_capture_home": mode == "setup",
        "ready_to_start": mode == "setup" or profile is not None,
        "hardware_validated": False,
        "leader": {"port": cfg.teleop.port, "id": cfg.teleop.id,
                   "expected_pid": list(checked_leader_pid(expected_leader_pid))},
        "follower": {"port": cfg.robot.port, "id": cfg.robot.id,
                     "expected_pid": [cfg.robot.position_p_coefficient, cfg.robot.position_i_coefficient,
                                      cfg.robot.position_d_coefficient]},
        "units": "body degrees, gripper 0..100", "max_relative_target": cfg.robot.max_relative_target,
        "position_boundary_tolerance": {
            "encoder_counts": POSITION_TOLERANCE_COUNTS,
            "body_degrees": POSITION_TOLERANCE_COUNTS * 360 / ENCODER_MAX,
            "scope": "Both arms: raw feedback plus body measured/mapped/hold/reset targets; same boundary throughout",
            "encoder_representation": [0, ENCODER_MAX],
            "gripper_normalized_range": [0, 100],
            "measurement_policy": "Keep actual samples; no extra clipping or replacement",
            "calibration_or_motor_register_writes": False,
        },
        "motor_writes": ["Torque_Enable", "Goal_Position"],
        "motor_register_policy": "Read/check existing mode=0 and PID; do NOT configure/calibrate motors",
        "feedback_mode": feedback_mode,
        "manual_feedback": ("Native MotorsBus.sync_read(Present_Position), one request per arm, raw and normalized positions from the same read; no periodic voltage/Status polling"
                            if feedback_mode == "official" else "Full per-motor position/voltage/Status feedback every cycle"),
        "sync_read_retries": {"leader": getattr(cfg.teleop, "num_read_retries", 2),
                              "follower": getattr(cfg.robot, "num_read_retries", 2)},
        "full_feedback_scope": ("Connection; before enabling both-arm hold; holding/reset/limit-pause phases"
                                if feedback_mode == "official" else "All phases"),
        "voltage_policy": {
            "polling_scope": "Only full-feedback phases" if feedback_mode == "official" else "All phases",
            "scope": "Undervoltage only, leader torque OFF; voltage-error bit 0 may accompany the low reading, no other faults",
            "confirmation_budget_s": CONFIRM_SECONDS, "recheck_interval_s": RECHECK_SECONDS,
            "required_consecutive_normal_samples": NORMAL_SAMPLES,
            "behavior": "No new goals/frames while confirming; continue only with fresh full feedback after voltage AND voltage-only error bits clear",
            "still_fatal": "Persistent/unconfirmed low voltage, any overvoltage, other device/communication errors, torque mismatch; follower or torque-enabled leader voltage faults",
            "protection_register_writes": False,
        },
        "connection_behavior": "Disable both arms on connection; seed measured goals before every torque enable",
        "torque_enable_ack_behavior": "One write per joint; on ConnectionError read Torque_Enable once with no retries; continue only if readback=1, no device error, and stop/deadline checks pass; otherwise cleanup",
        "return_behavior": "Disabled in follow mode; pause holds follower only" if mode == "follow" else "Each arm starts at its own fresh measured pose and follows synchronized quintic interpolation to its saved initial pose; no fixed entry and no taught path",
        "return_operator_condition": "Empty grippers and clear path for BOTH arms; release leader during countdown. No collision detection or obstacle avoidance",
        "return_speed_policy": "Software target velocity/acceleration limits apply only to reset; not guaranteed physical motor speed, no servo speed/acceleration register changes",
        "limit_pause": "Finite manual leader/mapped limit only: hold follower at measured pose, leave leader free; discard unsaved episode; explicit m after valid alignment, or r reset from independently legal poses once home exists",
        "feedback": ("Raw and normalized positions share one position sample; health=null means NOT sampled, not healthy; reset uses full feedback"
                     if feedback_mode == "official" else "Raw and normalized positions share one accepted full sample; short leader-voltage confirmation resamples the full leader bus, logs rejected samples and accepts only fresh fault-free feedback"),
        "near_limit_warning": "5 degrees / gripper 5 points; display only",
        "exit_behavior": "Attempt torque OFF on every joint before dataset finalization; support arms before exit",
        "wrist_reference_behavior": ("Reuse and lock the validated source offset only; source home poses are NOT targets; c disabled; a enables aligned follow, h saves a new home" if wrist_reference else
                                     "setup: c samples follower wrist_roll minus leader wrist_roll before following; same-direction 1:1 plus offset only for wrist_roll; other joints unchanged; no motor writes"),
        "wrist_reference_source": wrist_reference,
        "home_capture_behavior": "setup: h saves actual initial poses and offset, bound to calibration hashes; no torque change or home-motion command, existing manual following continues; no overwrite or path teaching",
        "record_buttons": "n confirms success then saves and resets; r discards any unsaved episode then resets; reset never implicitly labels success; reset frames excluded",
        "signal_behavior": "SIGINT/SIGTERM only set a local flag; control flow checks it before writes and performs cleanup",
        "home_profile": str(profile_path), "wrist_offset_deg": profile.mapping.offset if profile else wrist_reference["offset_deg"] if wrist_reference else "NEEDS REFERENCE",
        "reset_button_enabled": mode in ("setup", "teleop", "record"),
        "reset_requires_saved_home": True,
        "reset_fault_recovery": {
            "action": "Stop targets, attempt torque OFF on all joints; keep UI and committed episodes",
            "poll_interval_s": FAULT_POLL_SECONDS,
            "retry": "Fresh explicit R after 2 new full fault-free samples with all torque OFF; use current poses for a new plan",
            "automatic_resume": False,
            "pending_save": "Finish/account once without automatic reset; discard only unsubmitted frames",
            "still_exits": "Operator stop, startup failure, writer failure, UI failure, unexpected programming errors",
        },
        "first_episode_review": "record: check saved video and committed numeric table asynchronously; after reset, v confirms task/visual review before s can start episode 2",
        "cameras_enabled": list(cfg.robot.cameras),
        "session": asdict(settings), "dataset_root": str(cfg.dataset.root),
        "episodes": cfg.dataset.num_episodes, "max_episode_s": cfg.dataset.episode_time_s,
        "upload": False, "overwrite": False, "auto_start_next_episode": False,
    }


class AsyncDatasetWriter:
    """One thread owns all dataset mutations, including add/save/discard/finalize.

    Control never waits for encoding. A full bounded queue faults the session
    instead of silently dropping training frames or delaying robot stop.
    """

    def __init__(self, dataset, task: str, build_frame: Callable, max_queue: int = 30, review_first=None):
        self.dataset, self.task, self.build_frame = dataset, task, build_frame
        self.review_first, self.saved = review_first, 0
        self.jobs = queue.Queue(maxsize=max_queue)
        self.error: BaseException | None = None
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._run, name="dataset-writer", daemon=True)
        self.thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError("Dataset writer failed; stopping hardware") from self.error

    def _submit(self, kind, value=None) -> Future:
        self.check()
        future = Future()
        try:
            self.jobs.put_nowait((kind, value, future))
        except queue.Full as exc:
            raise RuntimeError("Dataset queue full; no frames silently dropped") from exc
        return future

    def add(self, obs: dict, action: dict) -> None:
        self._submit("frame", {"follower": dict(obs["follower"]), "action": dict(action),
                               "images": {k: v.copy() for k, v in obs["images"].items()}})

    def save(self, final=False) -> Future:
        return self._submit("save", final)

    def discard(self) -> Future:
        return self._submit("discard")

    def _run(self):
        try:
            while not self.closing.is_set() or not self.jobs.empty():
                try:
                    kind, value, future = self.jobs.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    if self.error is not None:
                        raise RuntimeError("Earlier writer operation failed") from self.error
                    result = None
                    if kind == "frame":
                        self.dataset.add_frame(self.build_frame(value, self.task))
                    elif kind == "save":
                        self.dataset.save_episode()
                        self.saved += 1
                        if self.saved == 1 and self.review_first:
                            result = self.review_first(self.dataset)
                        if value:
                            self.dataset.finalize()
                    elif kind == "discard":
                        self.dataset.clear_episode_buffer()
                    future.set_result(result)
                except Exception as exc:
                    if self.error is None:
                        self.error = exc
                    future.set_exception(exc)
                finally:
                    self.jobs.task_done()
        finally:
            try:
                if self.error is None and self.dataset.has_pending_frames():
                    self.dataset.clear_episode_buffer()
            except Exception as exc:
                self.error = self.error or exc
            finally:
                try:
                    self.dataset.finalize()
                except Exception as exc:
                    self.error = self.error or exc

    def close(self):
        # This must only be called AFTER devices have been stopped/disconnected.
        self.closing.set()
        while self.thread.is_alive():
            self.thread.join(timeout=0.2)
        self.check()


def review_first_episode(dataset):
    """Check saved video and the committed table in the writer thread.

    The open Parquet shard must be verified again after finalization. Do not
    close/reopen it here: the current LeRobot writer could overwrite that shard.
    """
    import av
    import numpy as np
    from PIL import Image

    data = dataset.writer._latest_episode
    state, action = np.asarray(data["observation.state"]), np.asarray(data["action"])
    count = len(action)
    if not 0 < count <= 3600 or state.shape != (count, 6) or action.shape != (count, 6):
        raise ValueError("First episode: invalid length/state/action dimensions")
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise ValueError("First episode: nonfinite state/action")
    np.testing.assert_allclose(data["timestamp"], np.arange(count) / 30, atol=1e-6)
    if list(data["frame_index"]) != list(range(count)):
        raise ValueError("First episode: discontinuous frame index")
    root = ROOT / "outputs/episode-reviews" / dataset.root.name / time.strftime("%Y%m%d-%H%M%S")
    root.mkdir(parents=True, exist_ok=False)
    counts = {}
    for name in ("top", "wrist"):
        decoded = 0
        for path in sorted((dataset.root / "videos" / f"observation.images.{name}").rglob("*.mp4")):
            with av.open(str(path)) as container:
                for frame in container.decode(video=0):
                    if (frame.width, frame.height) != (640, 480):
                        raise ValueError(f"First episode: {name} resolution mismatch")
                    if decoded in {0, count // 2, count - 1}:
                        Image.fromarray(frame.to_ndarray(format="rgb24")).save(root / f"{name}-{decoded:04d}.png")
                    decoded += 1
        if decoded != count:
            raise ValueError(f"First episode: {name} decoded {decoded}, expected {count}")
        counts[name] = decoded
    np.savez(root / "state-action-timestamps.npz", state=state, action=action, timestamps=data["timestamp"])
    report = {"frames": count, "decoded_video_frames": counts, "dimensions_finite_timestamps": "passed",
              "numeric_source": "committed writer table; recheck finalized Parquet at collection end",
              "task_success": "operator must inspect videos and confirm v; not inferred automatically",
              "dataset": str(dataset.root), "report": str(root / "report.json")}
    (root / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def make_writer(cfg, robot, review_first=False) -> AsyncDatasetWriter:
    from lerobot.datasets import LeRobotDataset, aggregate_pipeline_dataset_features, create_initial_features
    from lerobot.processor import make_default_processors
    from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts

    processors = make_default_processors()
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(processors[0], create_initial_features(action=robot.action_features), use_videos=True),
        aggregate_pipeline_dataset_features(processors[2], create_initial_features(observation=robot.observation_features), use_videos=True),
    )
    ds = cfg.dataset
    dataset = LeRobotDataset.create(
        ds.repo_id, ds.fps, features, root=ds.root, robot_type=robot.name,
        use_videos=True, streaming_encoding=True, rgb_encoder=ds.rgb_encoder,
        depth_encoder=ds.depth_encoder, encoder_threads=ds.encoder_threads,
        encoder_queue_maxsize=ds.encoder_queue_maxsize, batch_encoding_size=1,
        metadata_buffer_size=1, image_writer_processes=0, image_writer_threads=0,
    )

    def frame(value, task):
        obs = {**{f"{k}.pos": v for k, v in value["follower"].items()}, **value["images"]}
        action = {f"{k}.pos": v for k, v in value["action"].items()}
        return {**build_dataset_frame(features, obs, prefix="observation"),
                **build_dataset_frame(features, action, prefix="action"), "task": task}

    return AsyncDatasetWriter(dataset, ds.single_task, frame,
                              review_first=review_first_episode if review_first else None)


class StopRequest:
    """Combine GUI stop events with a signal flag owned by the control process.

    A Python signal can interrupt Event.wait/is_set while its non-reentrant
    condition lock is held. The handler must never call Event.set or acquire
    another lock; normal control flow observes the flag and performs cleanup.
    """

    def __init__(self, event):
        self.event = event
        self.signal_number = None

    def handle_signal(self, signum, _frame):
        self.signal_number = signum

    def is_set(self):
        return self.signal_number is not None or self.event.is_set()

    def wait(self, timeout):
        if not self.is_set():
            self.event.wait(timeout)
        return self.is_set()


class Hardware:
    """No SOLeader/SOFollower.connect(): those configure registers implicitly.

    Connect buses explicitly, verify calibration/PID, and only write torque/goals.
    One thread owns both buses, including emergency cleanup of partial startup.
    """

    def __init__(self, cfg, stop, expected_leader_pid=(32, 0, 32)):
        self.expected_leader_pid = checked_leader_pid(expected_leader_pid)
        from lerobot.robots import make_robot_from_config
        from lerobot.teleoperators import make_teleoperator_from_config

        self.cfg, self.stop = cfg, stop
        self.leader = make_teleoperator_from_config(cfg.teleop)
        self.follower = make_robot_from_config(cfg.robot)
        self.arms = {"leader": self.leader, "follower": self.follower}
        self.torque = {"leader": False, "follower": False}
        self.last_health = 0.0
        self.control_deadline = None
        self.voltage_limits = {}
        self.communication_warnings = []
        self.voltage_confirmation = LeaderVoltageConfirmation(self.check_stop)
        self.voltage_notice_until = 0.0
        self.voltage_notice = ""
        _, self.bounds = calibration_context(cfg)

    def check_stop(self):
        if self.stop.is_set():
            raise InterruptedError("Stop requested")
        if self.control_deadline is not None and time.monotonic() > self.control_deadline:
            raise TimeoutError("Control deadline exceeded; no further motor targets")

    def connect(self):
        from diagnose_teleop import disable_all, ensure_port_free

        for device in [self.cfg.teleop.port, self.cfg.robot.port,
                       *(str(c.index_or_path) for c in self.cfg.robot.cameras.values())]:
            ensure_port_free(device)
        for role, arm in self.arms.items():
            self.check_stop()
            arm.bus.connect(handshake=False)
            arm.bus.set_baudrate(1_000_000)
            arm.bus.set_timeout(100)
            arm.bus._assert_motors_exist()
            for joint in JOINTS:
                if arm.bus.read("Phase", joint, normalize=False, num_retry=0) & 0x10:
                    raise RuntimeError(f"{role}/{joint}: unexpected feedback mode; Phase was not changed")
            # Explicitly authorised torque release; no calibration/configuration.
            disable_all(arm.bus)
            if not arm.bus.is_calibrated:
                raise RuntimeError(f"{role}: saved calibration differs from motor registers")
            self.voltage_limits[role] = {}
            pid = self.expected_leader_pid if role == "leader" else [self.cfg.robot.position_p_coefficient,
                   self.cfg.robot.position_i_coefficient, self.cfg.robot.position_d_coefficient]
            for joint in JOINTS:
                self.voltage_limits[role][joint] = tuple(arm.bus.read(k, joint, normalize=False, num_retry=0)
                                                       for k in ("Min_Voltage_Limit", "Max_Voltage_Limit"))
                for register, expected in zip(("P_Coefficient", "I_Coefficient", "D_Coefficient", "Operating_Mode"), [*pid, 0]):
                    actual = arm.bus.read(register, joint, normalize=False, num_retry=0)
                    if actual != expected:
                        raise RuntimeError(f"{role}/{joint} {register}={actual}, expected {expected}; no rewrite performed")
        self.health()
        for camera in self.follower.cameras.values():
            self.check_stop()
            camera.connect()

    def validate_health(self, role, values):
        hard, voltage = health_issues(values, self.voltage_limits[role], self.torque[role])
        errors = {**hard, **voltage}
        if errors:
            raise MotorHealthError(f"{role}: motor fault/voltage/torque mismatch: {errors}")

    def read_healthy_feedback(self, role):
        from diagnose_teleop import feedback

        if not hasattr(self, "voltage_confirmation"):
            self.voltage_confirmation = LeaderVoltageConfirmation(self.check_stop)
        previous = len(self.voltage_confirmation.events)
        def read_feedback():
            try:
                return feedback(self.arms[role].bus)
            except InterruptedError:
                raise
            except (OSError, RuntimeError) as exc:
                # The full-feedback helper reports failed serial packets as RuntimeError.
                raise ConnectionError(f"{role}: feedback read failed: {exc}") from exc
        values, sampled = self.voltage_confirmation.read(
            role, read_feedback, self.voltage_limits[role], self.torque[role])
        if len(self.voltage_confirmation.events) > previous:
            event = self.voltage_confirmation.events[-1]
            lowest = min(v["Present_Voltage"] for v in event["initial"].values()) / 10
            self.voltage_notice = (f"主臂短时低压 {lowest:.1f} V；{event['confirmation_ms']:.1f} ms 内复核恢复，"
                                   f"连续 {NORMAL_SAMPLES} 次正常，已记录诊断日志")
            self.voltage_notice_until = time.monotonic() + 5
            logging.warning(self.voltage_notice)
        return values, sampled

    def health(self):
        for role, arm in self.arms.items():
            self.check_stop()
            self.read_healthy_feedback(role)
        self.last_health = time.monotonic()

    def observe(self, *, detailed=True):
        self.check_stop()
        started = time.monotonic()
        poses, raw, raw_bounds, sample_time, health, timings = {}, {}, {}, {}, {}, {}
        for role, arm in self.arms.items():
            self.check_stop()
            before = time.monotonic()
            if detailed:
                health[role], sample_time[role] = self.read_healthy_feedback(role)
                raw[role] = {j: v["Present_Position"] for j, v in health[role].items()}
            else:
                # Same transport/register as upstream SOFollower.get_observation /
                # SOLeader.get_action. Keep raw values from this one sample so the
                # existing calibration-bound checks do not depend on clipped gripper values.
                sample_time[role] = before
                raw[role] = arm.bus.sync_read(
                    "Present_Position", normalize=False, num_retry=arm.config.num_read_retries)
            normalized = arm.bus._normalize({arm.bus.motors[j].id: v for j, v in raw[role].items()})
            poses[role] = {j: normalized[m.id] for j, m in arm.bus.motors.items()}
            raw_bounds[role] = {j: (c.range_min, c.range_max) for j, c in arm.bus.calibration.items()}
            timings["leader_read" if role == "leader" else "follower_feedback"] = (time.monotonic() - before) * 1000
        before = time.monotonic()
        images = {name: cam.read_latest(max_age_ms=500) for name, cam in self.follower.cameras.items()}
        timings["images"] = (time.monotonic() - before) * 1000
        self.check_stop()
        if time.monotonic() - started > 0.5:
            raise TimeoutError("Observation stalled; no new motion commands will be sent")
        self.last_observation_detailed = detailed
        voltage_warnings = ([self.voltage_notice] if detailed and time.monotonic() < getattr(self, "voltage_notice_until", 0) else [])
        return {**poses, "images": images, "raw": raw, "raw_bounds": raw_bounds,
                "sample_time": sample_time, "health": health if detailed else None, "timings_ms": timings,
                "feedback_source": "full" if detailed else "official_position",
                "voltage_warnings": voltage_warnings}

    def _enable_at(self, role, pose):
        arm = self.arms[role]
        self.check_stop()
        arm.bus.sync_write("Goal_Position", pose, num_retry=0)
        self.check_stop()
        # Feetech enable_torque() also writes Lock. Keep our motor write contract
        # restricted to Goal_Position and Torque_Enable, with acknowledged writes.
        if not self.torque[role]:
            for joint in JOINTS:
                self.check_stop()
                try:
                    arm.bus.write("Torque_Enable", joint, 1, normalize=False, num_retry=0)
                except ConnectionError as exc:
                    # The write may have reached the motor despite a missing or
                    # damaged ACK. Verify the register without re-sending an
                    # enable command. read() also rejects motor protection bits.
                    self.check_stop()
                    try:
                        enabled = arm.bus.read("Torque_Enable", joint, normalize=False, num_retry=0)
                    except RuntimeError as health_exc:
                        raise MotorHealthError(f"{role}/{joint}: torque readback fault: {health_exc}") from health_exc
                    self.check_stop()
                    if enabled != 1:
                        raise ConnectionError(
                            f"{role}/{joint}: torque enable ACK failed and readback={enabled}; stopping"
                        ) from exc
                    logging.warning("%s/%s: torque enable ACK failed (%s); readback confirms ON",
                                    role, joint, exc)
                    if not hasattr(self, "communication_warnings"):
                        self.communication_warnings = []
                    self.communication_warnings.append({"role": role, "joint": joint, "error": str(exc)})
                except RuntimeError as exc:
                    raise MotorHealthError(f"{role}/{joint}: torque enable fault: {exc}") from exc
        self.check_stop()
        self.torque[role] = True

    def hold(self, obs):
        # N/R can transition out of native manual feedback on this very tick.
        # Check full feedback before the first autonomous hold/torque write.
        if not getattr(self, "last_observation_detailed", True):
            self.health()
        for role in ("follower", "leader"):
            self._enable_at(role, obs[role])

    def hold_follower(self, obs):
        if self.torque["leader"] or not self.torque["follower"]:
            raise RuntimeError("Limit pause requires an already free leader and powered follower")
        self.check_stop()
        self.send("follower", obs["follower"])

    def follow(self, obs):
        from diagnose_teleop import disable_all

        self._enable_at("follower", obs["follower"])
        self.check_stop()
        disable_all(self.leader.bus)
        self.torque["leader"] = False

    def send(self, role, pose):
        self.check_stop()
        check_pose(pose, self.bounds[role])
        values = {f"{k}.pos": v for k, v in pose.items()}
        if role == "follower":
            sent = self.follower.send_action(values)
            return {k.removesuffix(".pos"): v for k, v in sent.items()}
        self.leader.send_feedback(values)
        return dict(pose)

    def release_torque(self):
        """Stop without disconnecting buses/cameras, so the UI can remain available."""
        from diagnose_teleop import disable_all

        failures = []
        for role, arm in self.arms.items():
            if arm.bus.is_connected:
                try:
                    arm.bus.sync_write("Torque_Enable", dict.fromkeys(JOINTS, 0), normalize=False, num_retry=0)
                except Exception:
                    logging.warning("%s fault-stop broadcast failed; verifying each motor", role)
        for role, arm in self.arms.items():
            try:
                if not arm.bus.is_connected:
                    raise ConnectionError("bus disconnected")
                disable_all(arm.bus)
            except Exception as exc:
                failures.append(f"{role} torque OFF unverified: {exc}")
            finally:
                # This is the required state for subsequent validation, not a claim
                # that a failed write succeeded. Recovery requires fresh readback=0.
                self.torque[role] = False
        if failures:
            raise RuntimeError("; ".join(failures))

    def close(self):
        from diagnose_teleop import disable_all

        failures = []
        # Request OFF on both buses first, before potentially slow per-joint
        # verification on a disconnected/faulty arm. Verification still attempts
        # every motor and reports any torque release that cannot be confirmed.
        for role, arm in self.arms.items():
            if arm.bus.is_connected:
                try:
                    arm.bus.sync_write("Torque_Enable", dict.fromkeys(JOINTS, 0), normalize=False, num_retry=0)
                except Exception:
                    logging.warning("%s broadcast torque OFF failed; attempting individual verification", role)
        # Do not use arm.is_connected: camera failures can make it false while
        # the motor bus remains open and energised.
        for role, arm in self.arms.items():
            if arm.bus.is_connected:
                try:
                    disable_all(arm.bus)
                except Exception as exc:
                    failures.append(f"{role} torque OFF unverified: {exc}")
                finally:
                    try:
                        arm.bus.disconnect(disable_torque=False)
                    except Exception as exc:
                        failures.append(f"{role} close: {exc}")
        for name, camera in self.follower.cameras.items():
            if camera.is_connected:
                try:
                    camera.disconnect()
                except Exception as exc:
                    failures.append(f"{name} camera close: {exc}")
        if failures:
            raise RuntimeError("; ".join(failures))


def preview_window(frames, commands, ready, stop, closed, mode="record", camera_names=("top", "wrist"), reuse_wrist=False):
    from so101_preview import preview_window as show_preview

    show_preview(frames, commands, ready, stop, closed, mode, camera_names, reuse_wrist)


def preview_packet(session, obs, progress, now):
    valid = obs is not None
    values = obs if valid else {"images": {}, "leader": None, "follower": None}
    return {**values, "target": session.last_target,
            "requested": session.mapping.requested(obs["leader"]) if valid and session.mapping else None,
            "time": now, "state": session.state, "observation_valid": valid,
            "reference": session.profile.home if session.profile else None,
            "wrist_offset_deg": session.mapping.offset if session.mapping else None,
            "reset_completed_count": session.reset_completed_count,
            "common_ranges": session.mapping.common_ranges() if session.mapping else None,
            "pause_hold": session.pause_hold,
            "boundary_warnings": session.boundary_warnings(obs) if valid else [],
            "message": "\n".join([*values.get("voltage_warnings", []),
                                    f"[{session.state}] {progress} · {session.message}",
                                    *([] if valid else ["运动已暂停；本轮反馈不可用，画面与实测表已清空。"])])}


def run(cfg, settings, identities, bounds, profile_path, profile, raw, mode, expected_leader_pid=(32, 0, 32),
        wrist_reference=None, feedback_mode="full"):
    expected_leader_pid = checked_leader_pid(expected_leader_pid)
    context = mp.get_context("spawn")
    frames, commands = context.Queue(maxsize=1), context.Queue(maxsize=16)
    frames.cancel_join_thread()
    commands.cancel_join_thread()
    ready, stop, closed = context.Event(), context.Event(), context.Event()
    control_stop = StopRequest(stop)
    viewer = context.Process(target=preview_window, args=(frames, commands, ready, stop, closed),
                             kwargs={"mode": mode, "camera_names": tuple(cfg.robot.cameras),
                                     "reuse_wrist": wrist_reference is not None}, daemon=True)
    hardware, writer = None, None
    original_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        original_handlers[sig] = signal.signal(sig, control_stop.handle_signal)
    viewer.start()
    try:
        ready_deadline = time.monotonic() + 10
        while not ready.wait(0.1):
            if control_stop.is_set():
                return
            if time.monotonic() >= ready_deadline:
                raise RuntimeError("Preview failed to open; no hardware connection attempted")
        if control_stop.is_set():
            return
        hardware = Hardware(cfg, control_stop, expected_leader_pid)
        hardware.connect()
        if mode == "record":
            writer = make_writer(cfg, hardware.follower, review_first=True)
        snapshot_root = ROOT / "outputs/adaptive-sessions" / time.strftime("%Y%m%d-%H%M%S")
        snapshot_root.mkdir(parents=True, exist_ok=False)
        snapshot = {"mode": mode, "config": raw, "home_profile": profile.data if profile else None,
                    "identities": identities,
                    "home_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest() if profile else None,
                    "review": review_summary(cfg, settings, profile_path, profile, mode, expected_leader_pid, wrist_reference, feedback_mode)}
        (snapshot_root / "session.json").write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n")
        session = Session(hardware, writer, settings, identities, bounds, mode, profile,
                          cfg.dataset.num_episodes, cfg.dataset.episode_time_s, profile_path,
                          require_first_review=mode == "record",
                          initial_wrist_offset=wrist_reference["offset_deg"] if wrist_reference else None)
        last_preview, last_phase, previous_tick = 0.0, None, None
        logged_home, logged_plan, logged_resets = profile, None, 0
        with (snapshot_root / "events.jsonl").open("x") as events:
            flush_voltage_events(hardware, events, session.state)
            while not control_stop.is_set():
                tick = time.monotonic()
                if not viewer.is_alive():
                    raise RuntimeError("Preview process exited; stopping robot")
                if writer:
                    writer.check()
                hardware.control_deadline = tick + settings.max_tick_gap_s
                command = None
                try:
                    while True:
                        key, created, state = commands.get_nowait()
                        # Ignore stale/held keys and events from an earlier phase.
                        if tick - created <= 0.5 and state == session.state and command is None:
                            command = key
                except queue.Empty:
                    pass
                session.timings_ms = {}
                try:
                    obs, recovery_event = control_tick(session, hardware, command, feedback_mode)
                except Exception as exc:
                    events.write(json.dumps({"event": "fault", "state": session.state, "time": time.time(),
                                             "error": repr(exc)}, ensure_ascii=False) + "\n")
                    events.flush()
                    raise
                finally:
                    flush_voltage_events(hardware, events, session.state)
                if recovery_event:
                    events.write(json.dumps({**recovery_event, "time": time.time()}, ensure_ascii=False) + "\n")
                    events.flush()
                for warning in hardware.communication_warnings:
                    events.write(json.dumps({"event": "communication_warning", "time": time.time(), **warning}) + "\n")
                hardware.communication_warnings.clear()
                if session.profile is not None and session.profile is not logged_home:
                    events.write(json.dumps({"event": "home_saved", "time": time.time(),
                                             "file": str(profile_path),
                                             "sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                                             "home_profile": session.profile.data}, ensure_ascii=False) + "\n")
                    logged_home = session.profile
                if session.reset_plan is not None and session.reset_plan is not logged_plan:
                    events.write(json.dumps({"event": "reset_plan", "time": time.time(),
                                             "countdown_s": settings.countdown_s,
                                             **session.reset_plan.summary()}, ensure_ascii=False) + "\n")
                    logged_plan = session.reset_plan
                if session.reset_completed_count != logged_resets:
                    events.write(json.dumps({"event": "reset_completed", "time": time.time(),
                                             **session.last_reset_report}, ensure_ascii=False) + "\n")
                    logged_resets = session.reset_completed_count
                sample = {"event": "sample", "time": time.time(), "monotonic": tick,
                          "state": session.state, "following": session.following,
                          "observation_valid": obs is not None,
                          "target": session.last_target,
                          "pause_reason": session.pause_reason, "pause_hold": session.pause_hold,
                          "fault_reason": session.fault_reason, "fault_ready": session.fault_ready,
                          "cycle_ms": (tick - previous_tick) * 1000 if previous_tick else None}
                if obs is not None:
                    sample.update({k: obs[k] for k in ("leader", "follower", "raw", "raw_bounds", "health",
                                                      "feedback_source", "sample_time")})
                    sample.update(voltage_warnings=obs.get("voltage_warnings", []),
                                  requested=session.mapping.requested(obs["leader"]) if session.mapping else None,
                                  timings_ms={**obs["timings_ms"], **session.timings_ms})
                events.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + "\n")
                previous_tick = tick
                phase = (session.state, session.message, session.completed)
                if phase != last_phase:
                    event = {"time": time.time(), "state": session.state, "message": session.message,
                             "saved_episodes": session.completed, "frames_in_episode": session.frames}
                    events.write(json.dumps(event, ensure_ascii=False) + "\n")
                    events.flush()
                    if phase[0] != (last_phase[0] if last_phase else None):
                        print(json.dumps(event, ensure_ascii=False), flush=True)
                    last_phase = phase
                if tick - last_preview >= 0.1:
                    progress = f"已保存 {session.completed}/{cfg.dataset.num_episodes}" if mode == "record" else "逐关节跟随检查" if mode == "follow" else "腕部映射/初始姿态设置" if mode == "setup" else "映射跟随/复位测试"
                    packet = preview_packet(session, obs, progress, time.monotonic())
                    try:
                        frames.put_nowait(packet)
                    except queue.Full:
                        pass
                    last_preview = tick
                interval = FAULT_POLL_SECONDS if session.state == "FAULT_PAUSED" else 1 / settings.fps
                control_stop.wait(max(0, interval - (time.monotonic() - tick)))
    except InterruptedError:
        if not control_stop.is_set():
            raise
    finally:
        errors = []
        # Motor cleanup is independent of dataset/camera/UI finalization failures.
        if hardware:
            try:
                hardware.close()
            except Exception as exc:
                logging.exception("Hardware cleanup failed; support arms and switch off external power")
                errors.append(exc)
        closed.set()
        viewer.join(2)
        if viewer.is_alive():
            viewer.terminate()
            viewer.join(2)
        frames.close()
        commands.close()
        if writer:
            try:
                writer.close()
            except Exception as exc:
                logging.exception("Dataset finalization failed; hardware cleanup already attempted")
                errors.append(exc)
        for sig, handler in original_handlers.items():
            signal.signal(sig, handler)
        if errors:
            raise RuntimeError("Cleanup errors: " + "; ".join(map(str, errors)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mode", choices=("setup", "follow", "teleop", "record"), required=True)
    parser.add_argument("--no-cameras", action="store_true", help="follow mode only; validate control without camera acquisition")
    parser.add_argument("--check", action="store_true", help="Read files and print effective config only; no hardware")
    parser.add_argument("--expected-leader-pid", nargs=3, type=int, metavar=("P", "I", "D"),
                        default=(32, 0, 32), help="Read/check leader PID only, never write it; default: 32 0 32")
    parser.add_argument("--reuse-wrist-from", type=Path,
                        help="Setup only: reuse the offset from an identity-validated saved home; capture a NEW home with h")
    parser.add_argument("--feedback-mode", choices=("full", "official"), default="full",
                        help="official: native position sync reads during manual collection; full checks remain for startup and reset")
    parser.add_argument("--dataset-name", help="Record only: new local dataset directory name (no paths)")
    parser.add_argument("--episodes", type=int, help="Record only: number of successful episodes in this new dataset")
    args = parser.parse_args(argv)
    if args.mode == "setup" and args.feedback_mode != "full":
        parser.error("Setup retains full feedback; official feedback is for follow, teleop or record")
    if args.reuse_wrist_from and args.mode != "setup":
        parser.error("--reuse-wrist-from is only available with --mode setup and a new home output")
    try:
        expected_leader_pid = checked_leader_pid(args.expected_leader_pid)
    except ValueError as exc:
        parser.error(str(exc))
    cfg, settings, identities, bounds, profile_path, profile, raw = load_config(
        args.config, args.mode, dataset_name=args.dataset_name, episodes=args.episodes)
    raw.setdefault("runtime", {})["expected_leader_pid"] = list(expected_leader_pid)
    raw["runtime"]["feedback_mode"] = args.feedback_mode
    wrist_reference = load_wrist_reference(args.reuse_wrist_from, identities, bounds, settings) if args.reuse_wrist_from else None
    if wrist_reference:
        raw["runtime"]["wrist_reference"] = wrist_reference
    if args.no_cameras:
        if args.mode != "follow":
            parser.error("--no-cameras is only available in follow mode")
        cfg.robot.cameras = {}
        raw["runtime"]["cameras"] = False
    print(json.dumps({"effective_config": raw, "review": review_summary(cfg, settings, profile_path, profile, args.mode, expected_leader_pid, wrist_reference, args.feedback_mode)},
                     ensure_ascii=False, indent=2), flush=True)
    if args.check:
        print("CHECK ONLY: no hardware opened, no motor writes, no dataset/profile created.")
        return
    if args.mode != "setup" and profile is None:
        parser.error("Capture the wrist mapping and initial poses with --mode setup first. No hardware opened.")
    logging.basicConfig(level=logging.INFO)
    run(cfg, settings, identities, bounds, profile_path, profile, raw, args.mode, expected_leader_pid,
        wrist_reference=wrist_reference, feedback_mode=args.feedback_mode)


if __name__ == "__main__":
    main()
