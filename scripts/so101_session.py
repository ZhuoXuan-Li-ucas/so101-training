"""Hardware-independent wrist mapping, saved home poses and direct reset control."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import time

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ROLES = ("leader", "follower")
POSITION_TOLERANCE_COUNTS = 16
ENCODER_MAX = 4095
Pose = dict[str, float]


class ResetMotionError(RuntimeError):
    """Unexpected motion during hold/reset; requires a fresh manual retry."""


class PositionLimit(ValueError):
    """Only finite position/range errors can enter a recoverable pause."""
    def __init__(self, source, joint, value, low, high, units):
        self.detail = dict(source=source, joint=joint, value=value, low=low, high=high, units=units)
        super().__init__(f"{source}/{joint}: {value:.2f} outside [{low:.2f}, {high:.2f}] {units}")


def check_measured_pose(pose: Pose) -> None:
    if set(pose) != set(JOINTS):
        raise ValueError("Expected exactly six joints")
    for joint, value in pose.items():
        if not math.isfinite(value):
            raise ValueError(f"{joint}: non-finite measurement {value}")


def check_pose(pose: Pose, bounds: dict[str, tuple[float, float]]) -> None:
    check_measured_pose(pose)
    for joint, value in pose.items():
        lo, hi = bounds[joint]
        if not lo <= value <= hi:
            raise ValueError(f"{joint}: {value} outside [{lo:.2f}, {hi:.2f}]; no wrap/clipping")


def allowed_raw_range(low, high):
    """Small calibration-edge allowance; never extends beyond encoder representation."""
    return max(0, low - POSITION_TOLERANCE_COUNTS), min(ENCODER_MAX, high + POSITION_TOLERANCE_COUNTS)


def check_observed(obs, role, bounds):
    check_measured_pose(obs[role])
    if "raw" in obs:
        raw = obs["raw"][role]
        check_measured_pose(raw)
        for joint in JOINTS:
            lo, hi = allowed_raw_range(*obs["raw_bounds"][role][joint])
            if not lo <= raw[joint] <= hi:
                raise PositionLimit(role, joint, raw[joint], lo, hi, "encoder counts")
    for joint, value in obs[role].items():
        lo, hi = bounds[role][joint]
        if not lo <= value <= hi:
            raise PositionLimit(role, joint, value, lo, hi, "points" if joint == "gripper" else "degrees")


def close_pose(a: Pose, b: Pose, deg: float, gripper: float) -> bool:
    return all(abs(a[k] - b[k]) <= (gripper if k == "gripper" else deg) for k in JOINTS)


def interpolate(a: Pose, b: Pose, weight: float) -> Pose:
    weight = min(1.0, max(0.0, weight))
    return {k: a[k] + (b[k] - a[k]) * weight for k in JOINTS}


@dataclass(frozen=True)
class Settings:
    fps: int = 30
    countdown_s: float = 2.0
    alignment_deg: float = 5.0
    alignment_gripper: float = 5.0
    settle_s: float = 0.5
    settle_timeout_s: float = 5.0
    max_tick_gap_s: float = 0.5
    tracking_error_deg: float = 15.0
    tracking_error_gripper: float = 20.0
    tracking_timeout_s: float = 0.5
    reset_min_duration_s: float = 2.0
    reset_max_velocity_deg_s: float = 10.0
    reset_max_acceleration_deg_s2: float = 20.0
    reset_max_velocity_gripper_s: float = 10.0
    reset_max_acceleration_gripper_s2: float = 20.0

    def __post_init__(self):
        if self.fps != 30:
            raise ValueError("This session requires 30 Hz")
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Invalid session parameter {name}: {value}")
        if self.settle_s >= self.settle_timeout_s:
            raise ValueError("Settling must fit the timeout")


def calibration_context(cfg) -> tuple[dict, dict]:
    """Read files only; fingerprint identities and precise normalization ranges."""
    identities, bounds = {}, {}
    for role, arm in (("leader", cfg.teleop), ("follower", cfg.robot)):
        path = Path(arm.calibration_dir) / f"{arm.id}.json"
        content = path.read_bytes()
        calibration = json.loads(content)
        if set(calibration) != set(JOINTS):
            raise ValueError(f"Invalid calibration joints: {path}")
        identities[role] = {"port": arm.port, "id": arm.id,
                            "calibration_sha256": hashlib.sha256(content).hexdigest(),
                            "units": "degrees; gripper 0..100"}
        bounds[role] = {}
        for joint, row in calibration.items():
            low, high = row["range_min"], row["range_max"]
            if not 0 <= low < high <= 4095:
                raise ValueError(f"Invalid calibration range: {role}/{joint}")
            # Keep normalization/calibration unchanged. All body position
            # checks use the same allowance, including mapped goals and resets.
            allowed_low, allowed_high = allowed_raw_range(low, high)
            mid = (low + high) / 2
            bounds[role][joint] = ((0.0, 100.0) if joint == "gripper" else
                                   ((allowed_low - mid) * 360 / ENCODER_MAX,
                                    (allowed_high - mid) * 360 / ENCODER_MAX))
    return identities, bounds


class WristMapping:
    def __init__(self, offset: float, bounds: dict):
        if not math.isfinite(offset):
            raise ValueError("Non-finite wrist offset")
        self.offset, self.bounds = float(offset), bounds

    def requested(self, leader: Pose) -> Pose:
        check_measured_pose(leader)
        target = dict(leader)
        target["wrist_roll"] += self.offset
        check_measured_pose(target)
        return target

    def __call__(self, leader: Pose) -> Pose:
        check_pose(leader, self.bounds["leader"])
        target = self.requested(leader)
        check_pose(target, self.bounds["follower"])
        return target

    def common_ranges(self):
        """Intersection in leader coordinates, for display; never clips commands."""
        return {j: (max(self.bounds["leader"][j][0], self.bounds["follower"][j][0] - (self.offset if j == "wrist_roll" else 0)),
                    min(self.bounds["leader"][j][1], self.bounds["follower"][j][1] - (self.offset if j == "wrist_roll" else 0)))
                for j in JOINTS}


class HomeProfile:
    """One measured pose per arm, plus wrist offset; no path or fixed entry."""
    def __init__(self, data: dict, identities: dict, bounds: dict, settings: Settings):
        if data.get("version") != 2 or data.get("kind") != "paired_home" or "samples" in data:
            raise ValueError("Expected paired_home version 2; old taught paths are not supported")
        if data.get("identities") != identities:
            raise ValueError("Home identity/calibration mismatch; capture a new home")
        self.data = deepcopy(data)
        self.mapping = WristMapping(data["wrist_offset_deg"], bounds)
        self.home = self.data.get("home", {})
        if set(self.home) != set(ROLES):
            raise ValueError("Expected a saved initial pose for each arm")
        for role in ROLES:
            check_pose(self.home[role], bounds[role])
        mapped = self.mapping(self.home["leader"])
        if not close_pose(mapped, self.home["follower"], settings.alignment_deg, settings.alignment_gripper):
            raise ValueError("Saved initial poses do not align under the wrist mapping")

    def write_new(self, path: Path) -> None:
        content = json.dumps(self.data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x") as stream:
            stream.write(content)


class DirectResetPlan:
    """Synchronized quintic joint interpolation from actual poses to saved home.

    Limits bound the generated targets, not physical motor speed. This is not
    collision avoidance; the operator must ensure empty grippers and a clear path.
    """
    PEAK_SLOPE = 15 / 8
    PEAK_CURVATURE = 10 / math.sqrt(3)

    def __init__(self, origin, home, bounds, settings):
        self.origin = {r: dict(origin[r]) for r in ROLES}
        self.home = {r: dict(home[r]) for r in ROLES}
        self.duration = settings.reset_min_duration_s
        self.deltas = {}
        for role in ROLES:
            check_pose(self.origin[role], bounds[role])
            check_pose(self.home[role], bounds[role])
            self.deltas[role] = {}
            for j in JOINTS:
                delta = abs(self.home[role][j] - self.origin[role][j])
                self.deltas[role][j] = delta
                speed = settings.reset_max_velocity_gripper_s if j == "gripper" else settings.reset_max_velocity_deg_s
                accel = settings.reset_max_acceleration_gripper_s2 if j == "gripper" else settings.reset_max_acceleration_deg_s2
                self.duration = max(self.duration, self.PEAK_SLOPE * delta / speed,
                                    math.sqrt(self.PEAK_CURVATURE * delta / accel))
        if not math.isfinite(self.duration):
            raise ValueError("Non-finite reset duration")

    def at(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError("Non-finite reset time")
        if elapsed <= 0:
            return deepcopy(self.origin)
        if elapsed >= self.duration:
            return deepcopy(self.home)
        u = elapsed / self.duration
        weight = u**3 * (10 + u * (-15 + 6 * u))
        return {r: interpolate(self.origin[r], self.home[r], weight) for r in ROLES}

    def summary(self):
        return {"origin": self.origin, "home": self.home, "duration_s": self.duration,
                "joint_displacements": self.deltas,
                "planned_peak_velocity": {r: {j: self.PEAK_SLOPE * d / self.duration for j, d in values.items()}
                                          for r, values in self.deltas.items()},
                "planned_peak_acceleration": {r: {j: self.PEAK_CURVATURE * d / self.duration**2 for j, d in values.items()}
                                              for r, values in self.deltas.items()}}


class Session:
    """Single control thread; all writer mutations are asynchronous Futures."""

    def __init__(self, hardware, writer, settings: Settings, identities: dict, bounds: dict,
                 mode: str, profile: HomeProfile | None = None, episodes: int = 7,
                 episode_time_s: float = 120, profile_path: Path | None = None,
                 require_first_review: bool = False, initial_wrist_offset: float | None = None):
        if mode not in ("setup", "follow", "teleop", "record"):
            raise ValueError("Use setup, follow, teleop or record; taught-path mode was removed")
        if mode != "setup" and profile is None:
            raise ValueError("Capture wrist mapping and home poses before this mode")
        if initial_wrist_offset is not None and (mode != "setup" or profile is not None):
            raise ValueError("Reusing only a wrist offset requires setup with a new home output")
        self.hw, self.writer, self.settings = hardware, writer, settings
        self.identities, self.bounds, self.mode = identities, bounds, mode
        self.profile, self.profile_path = profile, profile_path
        self.mapping = profile.mapping if profile else None
        self.mapping_locked = initial_wrist_offset is not None
        if self.mapping_locked:
            self.mapping = WristMapping(initial_wrist_offset, bounds)
        self.episodes, self.episode_time_s = episodes, episode_time_s
        self.state = "REFERENCE" if mode == "setup" else "START"
        self.message = {
            "setup": "支撑双臂，摆出主臂竖直握把/从臂横向夹爪参照，按 c 采样腕转偏移",
            "follow": "支撑双臂，对照映射请求与实测对齐；握住主臂后按 s 跟随，无固定起点",
            "teleop": "映射对齐后按 s 跟随；空爪且通道空旷时，按 r 从当前位置复位",
            "record": "按 r 回到已保存初始姿态；摆好积木、握住主臂后按 s 开始",
        }[mode]
        if self.mapping_locked:
            self.state = "ALIGN"
            self.message = (f"已载入并锁定腕转偏移 {self.mapping.offset:+.2f}°，无需重采。"
                            "对齐后按 a 跟随摆好新起点，按 h 保存当前位置；r 才是复位。")
        self.following = self.holding = False
        self.hold_pose = None
        self.completed = self.frames = 0
        self.future = None
        self.pending_operation = ""
        self.started = 0.0
        self.last_tick = None
        self.reset_started = 0.0
        self.reset_plan = None
        self.reset_completed_count = 0
        self.last_reset_report = None
        self.settled_since = self.tracking_bad_since = None
        self.last_target: Pose | None = None
        self.pause_hold = self.pause_reason = self.pause_future = None
        self.timings_ms = {}
        self.require_first_review = require_first_review
        self.first_reviewed = False
        self.review_report = None
        self.fault_reason = self.fault_release_error = None
        self.fault_normal_samples = 0
        self.fault_last_sample = None
        self.fault_ready = False
        self.fault_observation_valid = False

    def finish_pending_write(self):
        """Account for a submitted save/discard once, without starting motion."""
        if self.future is None:
            return True
        if not self.future.done():
            return False
        try:
            result = self.future.result()
        except Exception as exc:
            raise RuntimeError("Dataset save/discard failed; cannot resume motion") from exc
        if self.pending_operation == "save_reset":
            self.completed += 1
            if self.completed == 1:
                self.review_report = result
        self.frames = 0
        self.future = None
        self.pending_operation = ""
        return True

    def enter_fault_pause(self, error, release_error=None):
        """Called AFTER the controller attempts torque OFF on both arms."""
        self.fault_reason, self.fault_release_error = str(error), release_error
        self.state = "FAULT_PAUSED"
        self.following = self.holding = False
        self.hold_pose = self.last_target = self.reset_plan = None
        self.last_tick = self.settled_since = self.tracking_bad_since = None
        self.pause_hold = self.pause_reason = None
        self.fault_normal_samples = 0
        self.fault_last_sample = None
        self.fault_ready = False
        self.fault_observation_valid = False
        # Preserve an in-flight save. Never discard or resubmit that episode.
        if self.future is None and self.pause_future is not None:
            self.future, self.pause_future = self.pause_future, None
            self.pending_operation = "fault_discard"
        elif self.future is None and self.mode == "record" and self.frames:
            self.future = self.writer.discard()
            self.pending_operation = "fault_discard"
        self.frames = 0
        release = ("力矩关闭尚未确认，请支撑双臂并检查电源/连接：" + str(release_error)
                   if release_error else "双臂力矩已关闭，请支撑双臂")
        self.message = f"复位/保持故障暂停：{self.fault_reason}。{release}。已保存数据保留；等待正常反馈后重新按 r。"

    def fault_paused_step(self, command, obs, now, read_error=None):
        was_ready = self.fault_ready
        self.fault_observation_valid = False
        pending = not self.finish_pending_write()
        try:
            if obs is None or obs.get("feedback_source") != "full" or not obs.get("health"):
                raise ValueError(str(read_error or "尚未取得完整反馈"))
            for role in ROLES:
                check_observed(obs, role, self.bounds)
                sampled = obs["sample_time"][role]
                if not 0 <= now - sampled <= self.settings.max_tick_gap_s:
                    raise ValueError(f"{role}: feedback not fresh")
                if set(obs["health"][role]) != set(JOINTS):
                    raise ValueError(f"{role}: incomplete feedback")
                for row in obs["health"][role].values():
                    if row["Torque_Enable"] != 0 or row["Status"] or row["packet_error"]:
                        raise ValueError(f"{role}: torque OFF / clear faults not confirmed")
            token = tuple(obs["sample_time"][role] for role in ROLES)
        except (ValueError, KeyError, TypeError) as exc:
            self.fault_normal_samples = 0
            self.fault_last_sample = None
            self.fault_ready = False
            self.message = (f"故障暂停，禁止运动：{exc}。请支撑双臂；已保存 {self.completed} 条。"
                            + ("保存/作废处理中；" if pending else "")
                            + "恢复正常后重新按 r；q 退出。")
            return
        self.fault_observation_valid = True
        if self.fault_last_sample is None or all(a > b for a, b in zip(token, self.fault_last_sample)):
            self.fault_normal_samples = min(2, self.fault_normal_samples + 1)
            self.fault_last_sample = token
        self.fault_ready = self.fault_normal_samples >= 2 and not pending
        self.message = (f"故障暂停；已保存 {self.completed} 条。"
                        + ("反馈正常且双臂力矩全关；空爪、路径畅通后按 r 重新复位。" if self.fault_ready
                           else "等待保存完成及两次新鲜正常反馈，暂不接受复位。")
                        + "不会自动恢复运动；q 退出。")
        if command == "r" and was_ready and self.fault_ready:
            # An early R is consumed, never queued to execute after recovery.
            if self.request_reset(obs, now):
                self.fault_reason = self.fault_release_error = None
                self.fault_ready = False

    def close(self, a, b):
        return close_pose(a, b, self.settings.alignment_deg, self.settings.alignment_gripper)

    def pair_at(self, obs, pair):
        return all(self.close(obs[r], pair[r]) for r in ROLES)

    def manual_target(self, obs):
        check_observed(obs, "leader", self.bounds)
        target = self.mapping.requested(obs["leader"])
        check_measured_pose(target)
        for j, value in target.items():
            lo, hi = self.bounds["follower"][j]
            if not lo <= value <= hi:
                raise PositionLimit("mapped", j, value, lo, hi, "points" if j == "gripper" else "degrees")
        return target

    def boundary_warnings(self, obs):
        if not self.mapping:
            return []
        return [f"{j} 接近共同边界 [{lo:.1f}, {hi:.1f}]：{obs['leader'][j]:.1f}"
                for j, (lo, hi) in self.mapping.common_ranges().items()
                if min(obs["leader"][j] - lo, hi - obs["leader"][j]) <= 5]

    def hold(self, obs):
        for role in ROLES:
            check_observed(obs, role, self.bounds)
        self.hw.hold(obs)
        self.holding, self.following = True, False
        self.hold_pose = {r: dict(obs[r]) for r in ROLES}
        self.last_target = dict(obs["follower"])

    def follow(self, obs):
        check_observed(obs, "follower", self.bounds)
        try:
            target = self.manual_target(obs)
        except PositionLimit as exc:
            self.message = f"尚未开启跟随：{exc}；回到有效范围后重试"
            return False
        if not self.close(target, obs["follower"]):
            differences = [f"{j}：目标 {target[j]:.1f}，实测 {obs['follower'][j]:.1f}" for j in JOINTS
                           if abs(target[j] - obs["follower"][j]) >
                           (self.settings.alignment_gripper if j == "gripper" else self.settings.alignment_deg)]
            self.message = "未对齐（容差 5° / 5 点）：" + "；".join(differences)
            return False
        self.hw.follow(obs)
        self.following, self.holding, self.hold_pose = True, False, None
        return True

    def pause_limits(self, exc, obs):
        self.hw.hold_follower(obs)
        self.pause_hold = dict(obs["follower"])
        self.last_target = dict(self.pause_hold)
        self.pause_reason = exc.detail
        self.following = self.holding = False
        self.hold_pose = None
        if self.mode == "record" and self.frames:
            self.pause_future = self.writer.discard()
        self.frames = 0
        self.state = "LIMIT_PAUSED"

    def paused_step(self, command, obs, now):
        if not self.close(obs["follower"], self.pause_hold):
            raise ResetMotionError("Paused follower moved beyond hold tolerance; stopping")
        target, problem = None, None
        try:
            target = self.manual_target(obs)
        except PositionLimit as exc:
            problem = str(exc)
        pending = self.pause_future is not None and not self.pause_future.done()
        if self.pause_future is not None and not pending:
            try:
                self.pause_future.result()
            except Exception as exc:
                raise RuntimeError("Dataset discard failed; cannot resume motion") from exc
            self.pause_future = None
        aligned = target is not None and self.close(target, obs["follower"])
        self.message = (f"越界暂停：{self.pause_reason}。从臂保持、主臂自由；"
                        f"{problem or ('已对齐' if aligned else '映射请求与从臂须对齐至 ≤5° / 5 点')}。"
                        + ("当前段作废处理中；" if pending else "") + "按 m 显式恢复"
                        + ("；空爪且通道空旷也可按 r 直接复位" if self.profile and self.mode != "follow" else ""))
        if command == "r" and self.mode != "follow" and self.profile:
            self._reset_command(obs, now)
            return
        if command == "m" and aligned and not pending and self.follow(obs):
            self.pause_reason = self.pause_hold = None
            self.state = "SETUP_FOLLOW" if self.mode == "setup" and not self.profile else "MANUAL" if self.mode == "record" else "TELEOP"
            self.message = "已恢复，不录制；按 r 复位后再按 s 重录" if self.mode == "record" else "已恢复映射跟随"
            return
        self.last_target = self.hw.send("follower", self.pause_hold)

    def _setup_command(self, command, obs):
        if command == "c" and self.mapping_locked:
            self.message = "本次保留已载入的腕转映射，不重新采样。摆好新初始姿态后按 h 保存；r 才是复位。"
            return
        if command == "c" and self.state in ("REFERENCE", "ALIGN") and not self.following and self.profile is None:
            try:
                for role in ROLES:
                    check_observed(obs, role, self.bounds)
            except PositionLimit as exc:
                self.message = f"参照未采样：{exc}"
                return
            self.mapping = WristMapping(obs["follower"]["wrist_roll"] - obs["leader"]["wrist_roll"], self.bounds)
            self.state = "ALIGN"
            self.message = f"腕转偏移 {self.mapping.offset:+.2f}°。可按 c 重采；对齐后按 a 跟随选初始姿态，或直接按 h 保存"
        elif command == "a" and self.state == "ALIGN" and self.follow(obs):
            self.state = "SETUP_FOLLOW"
            self.message = "映射跟随中；选好空爪、开阔的初始姿态并停稳，按 h 保存"
        elif command == "h" and self.state in ("ALIGN", "SETUP_FOLLOW") and self.profile is None:
            if self.profile_path is None:
                raise ValueError("Home output path is missing")
            try:
                for role in ROLES:
                    check_observed(obs, role, self.bounds)
                target = self.manual_target(obs)
            except PositionLimit as exc:
                self.message = f"初始姿态未保存：{exc}"
                return
            if not self.close(target, obs["follower"]):
                self.message = "初始姿态未保存：等待双臂映射对齐至 5° / 5 点以内"
                return
            data = {"version": 2, "kind": "paired_home", "identities": self.identities,
                    "wrist_offset_deg": self.mapping.offset,
                    "home": {r: dict(obs[r]) for r in ROLES}}
            if "raw" in obs:
                data["raw_home"] = deepcopy(obs["raw"])
            profile = HomeProfile(data, self.identities, self.bounds, self.settings)
            profile.write_new(self.profile_path)  # Save actual poses; no torque/goal write.
            self.profile = profile
            self.mapping = profile.mapping
            self.state = "TELEOP" if self.following else "READY"
            self.message = ("腕偏移与两臂初始姿态已保存；" + ("继续握住主臂跟随；" if self.following else "")
                            + "空爪且通道空旷时按 r 测试从当前位置复位")

    def request_reset(self, obs, now):
        if self.profile is None:
            self.message = "请先完成腕转映射并保存初始姿态"
            return False
        try:
            for role in ROLES:
                check_observed(obs, role, self.bounds)
        except PositionLimit as exc:
            self.message = f"暂不能复位：{exc}；回到校准有效范围后重试"
            return False
        # Each arm starts from its own measured pose. No mapping or entry-pose
        # alignment is required at reset start; both endpoints are validated.
        plan = DirectResetPlan(obs, self.profile.home, self.bounds, self.settings)
        self.hold(obs)
        self.pause_hold = self.pause_reason = None
        self.reset_plan, self.reset_started = plan, now
        self.settled_since = self.tracking_bad_since = None
        self.state = "COUNTDOWN"
        self.message = f"空爪复位：请松开主臂并让开路径；{self.settings.countdown_s:g} 秒后双臂返回初始姿态，预计运动 {plan.duration:.1f} 秒"
        return True

    def _reset_command(self, obs, now):
        if self.profile is None:
            self.message = ("请先按 h 保存本轮新的初始姿态；r 用于返回保存的姿态" if self.mapping else
                            "请先采样腕转偏移并按 h 保存初始姿态")
            return
        try:
            for role in ROLES:
                check_observed(obs, role, self.bounds)
        except PositionLimit as exc:
            self.message = f"暂不能复位：{exc}；先回到校准有效范围"
            return
        if self.mode == "record" and (self.frames or self.pause_future is not None):
            self.hold(obs)
            self.future = self.pause_future if self.pause_future is not None else self.writer.discard()
            self.pause_future = None
            self.frames = 0
            self.pending_operation, self.state = "discard_reset", "SAVING"
            self.message = "当前未保存段作废中；完成后从当前位置复位，不计为成功"
        else:
            self.request_reset(obs, now)

    def _command(self, command, obs, now):
        if command == "v" and self.state == "FIRST_REVIEW":
            self.first_reviewed = True
            self.state = "DONE" if self.completed >= self.episodes else "READY"
            self.message = "首条检查已确认；摆好积木后按 s 继续"
            return
        if self.state in ("SAVING", "COUNTDOWN", "RETURNING"):
            return
        if self.mode == "setup" and self.profile is None:
            if command == "r":
                self._reset_command(obs, now)
            else:
                self._setup_command(command, obs)
            return
        if command == "r" and self.mode != "follow":
            self._reset_command(obs, now)
        elif command == "s" and self.state in ("START", "READY"):
            if self.mode == "record" and self.completed >= self.episodes:
                return
            if self.mode == "record" and not self.pair_at(obs, self.profile.home):
                self.message = "尚未回到初始姿态；空爪且通道空旷时按 r 复位，再按 s 开始"
                return
            if self.follow(obs):
                self.started, self.frames = now, 0
                self.state = "RECORDING" if self.mode == "record" else "TELEOP"
                self.message = "录制中；成功按 n 保存并复位，失败按 r 作废并复位" if self.mode == "record" else "映射跟随中；空爪且路径空旷时按 r 复位" if self.mode != "follow" else "逐关节跟随检查，不录制、不复位"
        elif command == "m" and self.mode != "follow" and self.state in ("START", "READY", "REVIEW", "MANUAL"):
            if self.follow(obs):
                if self.state != "REVIEW":
                    self.state = "MANUAL" if self.mode == "record" else "TELEOP"
                self.message = "手动整理中，不录制；空爪且通道空旷时按 r 复位"
        elif command == "n" and self.mode == "record" and self.state in ("RECORDING", "REVIEW"):
            if self.frames == 0:
                self.message = "没有录制帧，不能保存空段"
                return
            self.hold(obs)
            self.future = self.writer.save(final=self.completed + 1 == self.episodes)
            self.pending_operation, self.state = "save_reset", "SAVING"
            self.message = "成功段保存中；完成后从当前位置复位，请松开物体并让开路径"

    def _return_step(self, obs, now):
        s = self.settings
        if self.state == "COUNTDOWN" and not self.pair_at(obs, self.reset_plan.origin):
            raise ResetMotionError("Arms moved during reset countdown; return cancelled")
        elapsed = now - self.reset_started - s.countdown_s
        if elapsed < 0:
            return
        self.state = "RETURNING"
        goals = self.reset_plan.at(elapsed)
        tracking_ok = all(close_pose(obs[r], goals[r], s.tracking_error_deg, s.tracking_error_gripper) for r in ROLES)
        if tracking_ok:
            self.tracking_bad_since = None
        elif self.tracking_bad_since is None:
            self.tracking_bad_since = now
        elif now - self.tracking_bad_since >= s.tracking_timeout_s:
            raise ResetMotionError("Return tracking error persisted; stop both arms")
        for role in ROLES:
            check_pose(goals[role], self.bounds[role])
        for role in ("follower", "leader"):
            sent = self.hw.send(role, goals[role])
            if role == "follower":
                self.last_target = sent
        self.message = f"双臂返回初始姿态 {elapsed:.1f} / {self.reset_plan.duration:.1f} 秒；不录制"
        if elapsed >= self.reset_plan.duration:
            if self.pair_at(obs, self.profile.home) and self.close(self.mapping(obs["leader"]), obs["follower"]):
                if self.settled_since is None:
                    self.settled_since = now
                if now - self.settled_since >= s.settle_s:
                    self.reset_completed_count += 1
                    self.last_reset_report = {**self.reset_plan.summary(), "started": self.reset_started,
                                              "completed": now, "completed_count": self.reset_completed_count}
                    self.hold_pose = deepcopy(self.profile.home)
                    self.state = "DONE" if self.mode == "record" and self.completed >= self.episodes else "READY"
                    self.message = "全部成功段已保存并复位；支撑双臂后按 q 退出" if self.state == "DONE" else "已回初始姿态；摆好积木，握住主臂后按 s 开始"
                    if self.require_first_review and self.completed == 1 and not self.first_reviewed:
                        self.state = "FIRST_REVIEW"
                        path = self.review_report.get("report") if isinstance(self.review_report, dict) else self.review_report
                        self.message = f"首条已保存并复位；检查视频、数值及任务成功，按 v 确认。报告：{path}"
            else:
                self.settled_since = None
            if elapsed > self.reset_plan.duration + s.settle_timeout_s:
                raise TimeoutError("Return did not reach home before the settling deadline")

    def step(self, command, obs, now):
        if not math.isfinite(now):
            raise ValueError("Non-finite control time")
        if self.state == "FAULT_PAUSED":
            self.timings_ms = {}
            self.last_tick = now
            self.fault_paused_step(command, obs, now)
            return
        if self.following or self.holding or self.state == "LIMIT_PAUSED":
            if self.last_tick is not None and not 0 <= now - self.last_tick <= self.settings.max_tick_gap_s:
                raise TimeoutError("Control loop stalled; no further trajectory commands")
        self.last_tick = now
        self.timings_ms = {}
        for role in ROLES:
            check_measured_pose(obs[role])
            if "raw" in obs:
                check_measured_pose(obs["raw"][role])
            if "sample_time" in obs and not 0 <= now - obs["sample_time"][role] <= self.settings.max_tick_gap_s:
                raise TimeoutError(f"{role}: stale feedback")
        check_observed(obs, "follower", self.bounds)
        if self.holding:
            check_observed(obs, "leader", self.bounds)
            if self.state not in ("COUNTDOWN", "RETURNING") and not self.pair_at(obs, self.hold_pose):
                raise ResetMotionError("Held arms moved beyond hold tolerance; stopping")
        if self.following:
            try:
                if command == "r" and self.profile is not None and self.mode != "follow":
                    check_observed(obs, "leader", self.bounds)
                else:
                    self.manual_target(obs)
            except PositionLimit as exc:
                self.pause_limits(exc, obs)
                command = None
        if self.state == "LIMIT_PAUSED":
            self.paused_step(command, obs, now)
            return
        if command:
            self._command(command, obs, now)
        if self.state == "SAVING" and self.finish_pending_write():
            self.request_reset(obs, now)
        if self.state == "RECORDING" and now - self.started >= self.episode_time_s:
            self.hold(obs)
            self.state = "REVIEW"
            self.message = "已达时限，停止录帧；成功按 n 保存并复位，失败按 r 作废并复位"
        if self.state in ("COUNTDOWN", "RETURNING"):
            self._return_step(obs, now)
        elif self.following:
            before = time.perf_counter()
            target = self.manual_target(obs)
            self.timings_ms["mapping"] = (time.perf_counter() - before) * 1000
            before = time.perf_counter()
            self.last_target = self.hw.send("follower", target)
            self.timings_ms["follower_send"] = (time.perf_counter() - before) * 1000
            if self.state == "RECORDING":
                self.writer.add(obs, self.last_target)
                self.frames += 1
