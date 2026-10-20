"""Enter-driven trials using LeRobot's official ACT inference and base strategy."""

import _thread
import hashlib
import json
import logging
import math
import signal
import sys
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import lerobot.scripts.lerobot_rollout  # noqa: F401 -- register official robot/camera configs
from lerobot.configs import parser
from lerobot.rollout import context as official_context
from lerobot.rollout.configs import RolloutConfig
from lerobot.rollout.strategies.base import BaseStrategy
from lerobot.utils.utils import init_logging

from act_trial_session import EnterReader, RoundLog, TrialControls, TrialLoop
from diagnose_teleop import disable_all


ROOT = Path(__file__).resolve().parents[1]
STARTUP_REGISTERS = frozenset({
    "Torque_Enable", "Lock", "Operating_Mode", "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "Return_Delay_Time", "Maximum_Acceleration", "Acceleration", "Max_Torque_Limit",
    "Protection_Current", "Overload_Torque", "Phase",
})


@dataclass
class TrialConfig(RolloutConfig):
    home_path: str = str(ROOT / "data/motion_profiles/stack_white_on_red_home_v1.json")
    reset_duration_s: float = 3.0
    reset_fps: int = 50
    scene_reset_s: float = 10.0
    startup_write_retries: int = 2
    execution_trace: bool = False
    seed_current_before_enable: bool = False
    max_rounds: int | None = None
    camera_preview: bool = False
    preview_fps: int = 15
    output_root: str = str(ROOT / "outputs/act-trials")
    check: bool = False

    def __post_init__(self):
        super().__post_init__()
        if self.strategy.type != "base" or self.inference.type != "sync" or self.teleop is not None:
            raise ValueError("This trial frontend uses base + sync with the follower only.")
        if self.display_data or self.return_to_initial_position:
            raise ValueError("Use display_data=false and return_to_initial_position=false; resets are explicit.")
        if not all(math.isfinite(x) and x > 0 for x in (self.reset_duration_s, self.reset_fps, self.duration)):
            raise ValueError("Reset duration/rate and per-trial duration must be positive and finite.")
        if not math.isfinite(self.scene_reset_s) or self.scene_reset_s < 0:
            raise ValueError("Scene reset time must be finite and nonnegative.")
        if self.startup_write_retries < 0:
            raise ValueError("Startup write retries must be nonnegative.")
        if self.max_rounds is not None and self.max_rounds < 1:
            raise ValueError("max_rounds must be positive or None.")
        if self.preview_fps < 1 or self.preview_fps > 30:
            raise ValueError("preview_fps must be between 1 and 30.")
        if self.execution_trace and (self.policy.type != "act" or self.policy.temporal_ensemble_coeff is not None
                                     or self.interpolation_multiplier != 1):
            raise ValueError("Execution tracing requires ACT without temporal ensembling and interpolation_multiplier=1.")


def load_home(cfg):
    path = Path(cfg.home_path)
    profile = json.loads(path.read_text())
    identity = profile["identities"]["follower"]
    calibration = Path(cfg.robot.calibration_dir) / f"{cfg.robot.id}.json"
    if identity["port"] != cfg.robot.port or identity["id"] != cfg.robot.id:
        raise ValueError("Saved home belongs to a different follower.")
    if hashlib.sha256(calibration.read_bytes()).hexdigest() != identity["calibration_sha256"]:
        raise ValueError("Saved home and follower calibration differ; review the home before motion.")
    if not cfg.robot.use_degrees:
        raise ValueError("The saved home uses degrees for the arm and 0..100 for the gripper.")
    pose = profile["home"]["follower"]
    expected = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
    if set(pose) != expected or not all(math.isfinite(v) for v in pose.values()):
        raise ValueError("Incomplete or nonfinite saved follower home.")
    return {f"{key}.pos": value for key, value in pose.items()}


@contextmanager
def retry_startup_writes(bus, retries):
    """Use upstream communication retries for repeated, identical setup writes only."""
    original_write = bus.write
    had_instance_write = "write" in vars(bus)

    def write(data_name, motor, value, *, normalize=True, num_retry=0):
        if data_name in STARTUP_REGISTERS:
            num_retry = max(num_retry, retries)
        return original_write(data_name, motor, value, normalize=normalize, num_retry=num_retry)

    bus.write = write
    try:
        yield
    finally:
        if had_instance_write:
            bus.write = original_write
        else:
            del bus.write


def finish_partial_robot(robot):
    """Also release a partially connected robot when official context construction failed."""
    if robot is None:
        return
    failures = []
    bus = robot.bus
    if bus.is_connected:
        try:
            disable_all(bus)  # only Torque_Enable=0, checked on all six motors
        except Exception as exc:
            failures.append(f"Torque OFF: {exc}")
        try:
            bus.disconnect(disable_torque=False)
        except Exception as exc:
            failures.append(f"Bus disconnect: {exc}")
    for name, camera in robot.cameras.items():
        try:
            if camera.is_connected:
                camera.disconnect()
        except Exception as exc:
            failures.append(f"Camera {name}: {exc}")
    if failures:
        raise RuntimeError("; ".join(failures))


@contextmanager
def seed_before_configure(robot, retries):
    from act_execution_trace import replaced
    original_configure = robot.configure

    def configure():
        robot.bus.disable_torque(num_retry=retries)
        current = robot.bus.sync_read("Present_Position", normalize=False, num_retry=2)
        robot.bus.sync_write("Goal_Position", current, normalize=False)
        logging.info("Seeded current raw positions before torque enable: %s", current)
        return original_configure()

    def no_calibration():
        raise RuntimeError("Hardware calibration differs; this diagnostic session does not write calibration.")

    with replaced(robot, "configure", configure), replaced(robot, "calibrate", no_calibration):
        yield


class OfficialBackend:
    def __init__(self, cfg, home, controls, output_dir=None):
        self.cfg, self.home, self.controls = cfg, home, controls
        self.duration_s = cfg.duration
        self.ctx = None
        self.robot = None
        self.strategy = BaseStrategy(cfg.strategy)
        self.output_dir = output_dir
        self.trace_round = 0
        self.preview = None

    def connect(self):
        # The upstream builder connects before it returns ctx. Retain the actual object
        # so an exception during connect/configure cannot strand enabled motors.
        original_factory = official_context.make_robot_from_config

        with ExitStack() as startup_scope:
            def capture_robot(config):
                self.robot = original_factory(config)
                startup_scope.enter_context(retry_startup_writes(self.robot.bus, self.cfg.startup_write_retries))
                if getattr(self.cfg, "seed_current_before_enable", False):
                    startup_scope.enter_context(seed_before_configure(self.robot, self.cfg.startup_write_retries))
                return self.robot

            official_context.make_robot_from_config = capture_robot
            try:
                self.ctx = official_context.build_rollout_context(self.cfg, self.controls.segment_stop)
            finally:
                official_context.make_robot_from_config = original_factory
        # Use the collection home, never the arbitrary pose at program startup.
        self.ctx.hardware.initial_position = dict(self.home)
        self.strategy.setup(self.ctx)
        if getattr(self.cfg, "camera_preview", False):
            from act_trial_preview import CameraPreview
            self.preview = CameraPreview(self.robot.cameras, self.output_dir, self.cfg.preview_fps)
            self.preview.start()

    def prepare_run(self):
        self.strategy.reset_control_state()

    def run_policy(self):
        try:
            if getattr(self.cfg, "execution_trace", False):
                from act_execution_trace import ExecutionTrace
                self.trace_round += 1
                directory = self.output_dir / f"round-{self.trace_round:03d}"
                with ExecutionTrace(self.ctx, directory):
                    self.strategy.run(self.ctx)
                logging.info("Execution trace saved: %s", directory)
            else:
                self.strategy.run(self.ctx)
        finally:
            self.ctx.policy.inference.pause()

    def reset_home(self):
        self.strategy.reset_control_state()
        completed = self.strategy.return_to_initial_position(
            self.ctx.hardware, duration_s=self.cfg.reset_duration_s, fps=self.cfg.reset_fps
        )
        if not completed:
            raise RuntimeError("Reset communication failed; reset was not completed.")

    def close(self):
        try:
            if self.ctx is not None:
                self.strategy.teardown(self.ctx)
        finally:
            try:
                finish_partial_robot(self.robot)
            finally:
                if self.preview is not None:
                    self.preview.close()


def effective_review(cfg, home):
    return {
        "hardware_connected": False, "policy": str(cfg.policy.pretrained_path),
        "fps": cfg.fps, "trial_duration_s": cfg.duration,
        "n_action_steps": cfg.policy.n_action_steps, "chunk_size": cfg.policy.chunk_size,
        "temporal_ensemble_coeff": cfg.policy.temporal_ensemble_coeff,
        "interpolation_multiplier": cfg.interpolation_multiplier,
        "device": cfg.device, "use_amp": cfg.policy.use_amp,
        "robot_port": cfg.robot.port, "robot_id": cfg.robot.id,
        "calibration_dir": str(cfg.robot.calibration_dir), "leader_connected": False,
        "position_units": {"arm": "degrees", "gripper": "0..100, smaller is closed"},
        "cameras": {name: {"path": str(camera.index_or_path), "width": camera.width,
                           "height": camera.height, "fps": camera.fps, "fourcc": camera.fourcc,
                           "color_mode": str(camera.color_mode), "warmup_s": camera.warmup_s}
                    for name, camera in cfg.robot.cameras.items()},
        "home_path": cfg.home_path, "home_targets": home,
        "reset": {"method": "official linear interpolation, all six joints together",
                  "duration_s": cfg.reset_duration_s, "fps": cfg.reset_fps,
                  "separate_speed_or_acceleration_clamp": None},
        "scene_reset_s": cfg.scene_reset_s, "max_relative_target": cfg.robot.max_relative_target,
        "startup_write_retries": {
            "additional_attempts": cfg.startup_write_retries,
            "registers": sorted(STARTUP_REGISTERS),
            "scope": "Official context construction only; identical register/value on communication errors. Motor-reported faults still raise. Goal_Position and inference writes are unchanged.",
        },
        "motor_setup_registers": {"Operating_Mode": 0, "P_Coefficient": cfg.robot.position_p_coefficient,
                                  "I_Coefficient": cfg.robot.position_i_coefficient,
                                  "D_Coefficient": cfg.robot.position_d_coefficient,
                                  "Return_Delay_Time": 0, "Maximum_Acceleration": 254, "Acceleration": 254},
        "gripper_setup_registers": {"Max_Torque_Limit": 500, "Protection_Current": 250, "Overload_Torque": 25},
        "other_setup": "Official Phase bit 0x10 clearing if set; Torque_Enable/Lock toggled by official connect/disconnect; Goal_Velocity not changed.",
        "torque": "First Enter connects/configures/enables. Idle and scene-reset waits keep torque enabled. Exit disables torque without a return move.",
        "goal_positions": f"First Enter: saved home. Each trial: official policy. End Enter or {cfg.duration:g}-second timeout: saved home.",
        "logging": "New session directory; round timing plus executed-action trace and videos. Success/failure recorded externally by operator." if cfg.execution_trace else "New session directory; round number, timestamps, duration and end reason only. Success/failure recorded externally by operator.",
        "dataset_recording": False, "upload": False,
        "execution_trace": cfg.execution_trace,
        "max_rounds": cfg.max_rounds,
        "round_limit_behavior": "After final trial: reset home then exit/disable torque; no next trial or final cooldown." if cfg.max_rounds is not None else "Unlimited; fresh Enter required each round.",
        "camera_preview": {"enabled": cfg.camera_preview, "fps": cfg.preview_fps,
                           "source": "Existing camera latest-frame caches; no device opens or motor access",
                           "windows": "top and wrist; available after first Enter connects cameras, throughout resets and waits",
                           "window_close_behavior": "Closes only that preview; terminal Enter/q/Ctrl+C controls the trial",
                           "failure_behavior": "Log preview failure and continue unchanged policy loop"},
        "trace_outputs": "Per-round steps/chunks JSONL; top/wrist timestamped videos and frame journals" if cfg.execution_trace else None,
        "seed_current_before_enable": cfg.seed_current_before_enable,
        "startup_seed_behavior": "Disable torque; read current raw positions; write them as Goal_Position before official configure/enabling torque. Calibration mismatch aborts without recalibration." if cfg.seed_current_before_enable else None,
        "additional_tracking_or_voltage_abort_thresholds": None,
    }


@parser.wrap()
def main(cfg: TrialConfig):
    home = load_home(cfg)
    review = effective_review(cfg, home)
    if cfg.check:
        print(json.dumps(review, ensure_ascii=False, indent=2))
        return
    output_root = Path(cfg.output_root).resolve()
    if not output_root.is_relative_to(ROOT):
        raise ValueError("Trial output must stay inside this workspace.")
    out = output_root / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True, exist_ok=False)
    (out / "session.json").write_text(json.dumps(review, ensure_ascii=False, indent=2) + "\n")
    init_logging()
    handler = logging.FileHandler(out / "runtime.log", encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
    logging.getLogger().addHandler(handler)
    controls = TrialControls()

    def emit(message):
        print(message, flush=True)
        logging.getLogger("trial_events").debug(message)

    def quit_from_input():
        if controls.shutdown.is_set():
            return
        controls.quit()
        # Interrupt startup and the official blocking reset too, not just the run loop.
        _thread.interrupt_main()

    def stop_signal(_signum, _frame):
        controls.quit()
        raise KeyboardInterrupt

    previous_handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    reader = EnterReader(sys.stdin, controls.enter, quit_from_input, emit)
    backend = OfficialBackend(cfg, home, controls, out)
    emit(f"轮次计时记录：{out / 'rounds.csv'}")
    emit(f"流程：Enter 复位 → Enter 推理 → Enter 结束并复位 → 等待 {cfg.scene_reset_s:g} 秒 → Enter 下一轮。")
    if cfg.max_rounds is not None:
        emit(f"本次最多 {cfg.max_rounds} 轮；最后一轮结束并复位后自动卸扭退出。")
    if cfg.camera_preview:
        emit("实时相机窗口将在首次连接后出现；Enter / q / Ctrl+C 请在本终端操作，关闭预览窗口不会停止机械臂。")
    try:
        with (out / "rounds.csv").open("x", newline="", encoding="utf-8") as stream:
            try:
                reader.start()
                TrialLoop(backend, controls, RoundLog(stream), cfg.scene_reset_s, emit, max_rounds=cfg.max_rounds).run()
            except KeyboardInterrupt:
                emit("已退出，卸扭清理完成。")
            except Exception:
                logging.getLogger(__name__).exception("实验程序异常退出；完整异常栈已保存到 runtime.log")
                raise
    finally:
        reader.stop()
        for sig, handler_before in previous_handlers.items():
            signal.signal(sig, handler_before)
        logging.getLogger().removeHandler(handler)
        handler.close()


if __name__ == "__main__":
    main()
