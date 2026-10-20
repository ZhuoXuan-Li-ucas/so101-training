"""One follower reset, then hold for camera alignment; no cameras or policy opened."""

import argparse
import json
import logging
import signal
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import draccus
from lerobot.robots import make_robot_from_config
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.rollout.context import HardwareContext
from lerobot.rollout.robot_wrapper import ThreadSafeRobot
from lerobot.rollout.strategies.core import RolloutStrategy

from act_trial_loop import ROOT, STARTUP_REGISTERS, finish_partial_robot, load_home, retry_startup_writes
from diagnose_teleop import ensure_port_free


def load_settings(path):
    cfg = json.loads(path.read_text())
    robot_cfg = dict(cfg["robot"])
    if robot_cfg.pop("type") != "so101_follower":
        raise ValueError("This command resets the SO101 follower only.")
    robot_cfg["cameras"] = {}  # Existing ffplay previews retain their camera handles.
    robot_cfg = draccus.decode(SO101FollowerConfig, robot_cfg)
    home = load_home(SimpleNamespace(robot=robot_cfg, home_path=cfg["home_path"]))
    duration, fps = cfg["reset_duration_s"], cfg["reset_fps"]
    if duration <= 0 or fps <= 0:
        raise ValueError("Reset duration and frequency must be positive.")
    return cfg, robot_cfg, home


def reset_and_hold(robot, home, duration_s, fps, retries, hold):
    """Only the explicit execute path can reach this hardware sequence."""
    try:
        robot.bus.connect()
        if not robot.is_calibrated:
            raise RuntimeError("Motor calibration differs from the saved file; no calibration will be written.")
        with retry_startup_writes(robot.bus, retries):
            robot.bus.disable_torque(num_retry=retries)
            present = robot.bus.sync_read("Present_Position", normalize=False, num_retry=2)
            robot.bus.sync_write("Goal_Position", present, normalize=False)
            logging.info("Seeded current raw positions before enabling torque: %s", present)
            robot.configure()  # Official PID/position-mode/torque setup; enables torque last.
        hardware = HardwareContext(ThreadSafeRobot(robot), teleop=None, initial_position=home)
        if not RolloutStrategy.return_to_initial_position(hardware, duration_s=duration_s, fps=fps):
            raise RuntimeError("Reset interpolation failed.")
        measured = robot.get_observation()
        logging.info("Reset targets: %s; measured after interpolation: %s", home, measured)
        print("复位指令完成；当前实测位置：", json.dumps(measured, ensure_ascii=False), flush=True)
        hold()  # Torque stays on at the final target while aligning the camera.
    finally:
        finish_partial_robot(robot)


def wait_for_exit():
    print("正在保持复位姿态。相机预览继续运行；输入 q 后回车或 Ctrl+C 退出并卸扭。", flush=True)
    while True:
        try:
            if input().strip().lower() in ("q", "quit", "exit"):
                return
        except EOFError:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/act_trial_loop_080000.proposed.json")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="Show actual parameters without opening hardware")
    mode.add_argument("--execute", action="store_true", help="After command review: reset once and hold until quit")
    args = parser.parse_args()
    cfg, robot_cfg, home = load_settings(args.config)
    review = {
        "config": str(args.config.resolve()), "home_path": cfg["home_path"], "targets": home,
        "robot_port": robot_cfg.port, "robot_id": robot_cfg.id,
        "calibration_dir": str(robot_cfg.calibration_dir), "calibration_write": False,
        "leader_connected": False, "cameras_opened": False, "policy_loaded": False,
        "units": "Arm degrees; gripper 0..100, smaller is closed",
        "reset_duration_s": cfg["reset_duration_s"], "reset_fps": cfg["reset_fps"],
        "method": "Official all-joint linear interpolation", "max_relative_target": robot_cfg.max_relative_target,
        "startup_write_retries": cfg["startup_write_retries"], "retry_registers": sorted(STARTUP_REGISTERS),
        "motor_registers": {"Operating_Mode": 0, "P_Coefficient": robot_cfg.position_p_coefficient,
                            "I_Coefficient": robot_cfg.position_i_coefficient, "D_Coefficient": robot_cfg.position_d_coefficient,
                            "Return_Delay_Time": 0, "Maximum_Acceleration": 254, "Acceleration": 254,
                            "gripper/Max_Torque_Limit": 500, "gripper/Protection_Current": 250, "gripper/Overload_Torque": 25},
        "other_motor_setup": "Official Phase bit 0x10 clearing if set. Torque_Enable and Lock toggled during setup. Goal_Velocity unchanged.",
        "sequence": ["Read motor calibration; never recalibrate", "Disable torque, write current raw positions as startup Goal_Position",
                     "Official configure and torque enable", "Interpolate to saved home", "Hold torque until q/EOF/Ctrl+C/SIGTERM",
                     "Disable and verify torque OFF on all six motors, then disconnect"],
        "writes_files": "New timestamped outputs/home-resets directory only; no existing data/model/calibration overwritten",
    }
    print(json.dumps(review, ensure_ascii=False, indent=2), flush=True)
    if args.check:
        return
    ensure_port_free(robot_cfg.port)
    out = ROOT / "outputs/home-resets" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    out.mkdir(parents=True, exist_ok=False)
    (out / "review.json").write_text(json.dumps(review, ensure_ascii=False, indent=2) + "\n")
    logging.basicConfig(level=logging.INFO, handlers=[logging.StreamHandler(), logging.FileHandler(out / "runtime.log")],
                        format="%(asctime)s %(levelname)s %(message)s", force=True)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        robot = make_robot_from_config(robot_cfg)
        reset_and_hold(robot, home, cfg["reset_duration_s"], cfg["reset_fps"], cfg["startup_write_retries"], wait_for_exit)
    except KeyboardInterrupt:
        logging.info("Exited; cleanup completed.")
    except Exception:
        logging.exception("Reset failed; inspect cleanup errors if present.")
        raise
    finally:
        for sig, old in previous.items():
            signal.signal(sig, old)


if __name__ == "__main__":
    main()
