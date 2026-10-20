#!/usr/bin/env python3
"""Run stock LeRobot recording with a nonblocking, shared-frame desktop preview.

--check parses config and reads calibration files only. Normal execution requires
command review: it seeds follower targets and calls LeRobot, which configures
motors and enables torque. Existing processes must release the devices first.
"""

import argparse
import json
import logging
import multiprocessing as mp
from pathlib import Path
import queue
import time

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def preview_window(frames, ready, closed):
    """Only consume already-captured frames; never open cameras or motor ports."""
    import tkinter as tk
    from PIL import Image, ImageTk

    window = tk.Tk()
    window.title("SO101 Recording | top + wrist")
    window.geometry("1300x730+30+50")
    window.protocol("WM_DELETE_WINDOW", window.iconify)
    status = tk.Label(window, text="准备中：请查看录制终端", font=("Sans", 18))
    status.pack(pady=6)
    panels = tk.Frame(window)
    panels.pack()
    images = {}
    for name, title in (("top", "top · Gemini 全局视角"), ("wrist", "wrist · 绿联腕部视角")):
        panel = tk.Frame(panels)
        panel.pack(side="left", padx=4)
        tk.Label(panel, text=title, font=("Sans", 14)).pack()
        blank = ImageTk.PhotoImage(Image.new("RGB", (640, 480), "#202020"))
        images[name] = tk.Label(panel, image=blank, width=640, height=480)
        images[name].image = blank
        images[name].pack()
    joints = tk.Label(window, font=("Monospace", 10), justify="left")
    joints.pack(pady=5)
    tk.Label(window, text="臂关节单位：度；夹爪：0–100。主臂目标是请求值，不是舵机目标寄存器回读。\n"
             "n / →：结束本段或跳过复位等待　 r / ←：重录　 q / Esc：结束采集\n"
             "复位期间仍跟随主臂；× 仅最小化。停止控制也可在终端按 Ctrl+C。", font=("Sans", 12)).pack()
    last_received = None
    phase = "准备中：请查看录制终端"

    def refresh():
        nonlocal last_received, phase
        if closed.is_set():
            window.destroy()
            return
        packet = None
        try:
            while True:
                packet = frames.get_nowait()
        except queue.Empty:
            pass
        if packet is not None:
            last_received = packet["time"]
            phase = packet["phase"]
            for name, label in images.items():
                picture = ImageTk.PhotoImage(Image.fromarray(packet["images"][name]))
                label.configure(image=picture, width=640, height=480, text="")
                label.image = picture
            lines = ["关节                 " + "  ".join(f"{n:>13}" for n in JOINTS)]
            for key, title in (("action", "主臂目标 / action"), ("state", "从臂位置 / state ")):
                lines.append(title + "  " + "  ".join(f"{packet[key][n + '.pos']:13.2f}" for n in JOINTS))
            joints.configure(text="\n".join(lines))
        age = time.monotonic() - last_received if last_received is not None else None
        stale = age is not None and age > 1
        suffix = f" · 画面已 {age:.1f} 秒未更新" if stale else ""
        status.configure(text=phase + suffix, fg="#b14c00" if stale else "#184c2b")
        window.after(80, refresh)

    window.after(0, refresh)
    window.after(100, ready.set)
    window.mainloop()


class PreviewFeed(logging.Filter):
    def __init__(self, frames, count, episode_time_s=30, reset_time_s=30):
        super().__init__()
        self.frames = frames
        self.count = count
        self.episode_time_s = episode_time_s
        self.reset_time_s = reset_time_s
        self.phase = "准备中"
        self.last_sent = 0

    def filter(self, record):
        message = record.getMessage()
        if message.startswith("Recording episode "):
            index = int(message.rsplit(" ", 1)[-1])
            self.phase = f"正在录制第 {index + 1} / {self.count} 段（最长 {self.episode_time_s:g} 秒）"
        elif message == "Reset the environment":
            self.phase = f"复位中（不录制，仍在遥操）：{self.reset_time_s:g} 秒后自动开始；摆好后按 n 提前开始"
        elif message in ("Stop recording", "Exiting"):
            self.phase = "正在结束采集"
        return True

    def __call__(self, _step, transition):
        from lerobot.lerobot_types import TransitionKey

        now = time.monotonic()
        if now - self.last_sent < 0.1:
            return
        self.last_sent = now
        obs = transition[TransitionKey.OBSERVATION]
        packet = {
            "time": now,
            "phase": self.phase,
            "images": {name: obs[name] for name in ("top", "wrist")},
            "state": {f"{name}.pos": obs[f"{name}.pos"] for name in JOINTS},
            "action": dict(transition[TransitionKey.ACTION]),
        }
        try:
            self.frames.put_nowait(packet)
        except queue.Full:
            pass  # Drop preview updates only; stock dataset capture is unaffected.


def load_config(path):
    import draccus
    from lerobot.scripts.lerobot_record import RecordConfig

    cfg = draccus.parse(RecordConfig, config_path=str(path), args=[])
    if cfg.resume or cfg.dataset.push_to_hub:
        raise ValueError("This trial launcher requires resume=false and push_to_hub=false.")
    destination = Path(cfg.dataset.root).resolve()
    if not destination.is_relative_to(ROOT / "data/datasets"):
        raise ValueError("Trial datasets must stay in this workspace's data/datasets directory.")
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite existing dataset: {destination}")
    if cfg.display_data or set(cfg.robot.cameras) != {"top", "wrist"}:
        raise ValueError("This launcher uses its own top/wrist preview; display_data must be false.")
    for arm in (cfg.robot, cfg.teleop):
        calibration = Path(arm.calibration_dir) / f"{arm.id}.json"
        if not calibration.resolve().is_relative_to(ROOT / "data/calibration"):
            raise ValueError(f"Calibration outside the workspace: {calibration}")
        if set(json.loads(calibration.read_text())) != set(JOINTS):
            raise ValueError(f"Expected all six calibrated joints: {calibration}")
    return cfg


def preflight_and_seed(cfg):
    from diagnose_teleop import ensure_port_free, feedback
    from lerobot.robots import make_robot_from_config
    from lerobot.teleoperators import make_teleoperator_from_config

    for device in [cfg.teleop.port, cfg.robot.port, *(str(c.index_or_path) for c in cfg.robot.cameras.values())]:
        ensure_port_free(device)
    leader = make_teleoperator_from_config(cfg.teleop)
    follower = make_robot_from_config(cfg.robot)
    try:
        for role, arm in (("leader", leader), ("follower", follower)):
            arm.bus.connect()
            if not arm.bus.is_calibrated:
                raise RuntimeError(f"{role}: motor calibration does not match the file; aborting")
            faults = {name: v for name, v in feedback(arm.bus).items() if v["Status"] or v["packet_error"]}
            if faults:
                raise RuntimeError(f"{role}: motor fault: {faults}")
            print(f"{role}: all six motors respond; calibration matches; no fault flags")
        current = follower.bus.sync_read("Present_Position", normalize=False)
        follower.bus.sync_write("Goal_Position", current, normalize=False)
        print("Follower startup target set to its current measured position.")
    finally:
        for arm in (leader, follower):
            if arm.bus.is_connected:
                arm.bus.disconnect(disable_torque=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="Validate configuration only; no device access")
    args = parser.parse_args()
    cfg = load_config(args.config)
    import draccus
    print(json.dumps(draccus.encode(cfg), ensure_ascii=False, indent=2))
    if args.check:
        print("Configuration and six-joint calibration files validated; no hardware opened.")
        return

    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_record import record

    context = mp.get_context("spawn")
    frames = context.Queue(maxsize=1)
    frames.cancel_join_thread()
    ready, closed = context.Event(), context.Event()
    viewer = context.Process(target=preview_window, args=(frames, ready, closed), daemon=True)
    feed = PreviewFeed(frames, cfg.dataset.num_episodes, cfg.dataset.episode_time_s, cfg.dataset.reset_time_s)
    viewer.start()
    try:
        if not ready.wait(timeout=10):
            raise RuntimeError("Preview did not open; no hardware access attempted")
        input("请将主从臂摆至对应起始姿态、放好两块积木；准备好后在此终端按 Enter 开始首段录制。\n")
        preflight_and_seed(cfg)
        processors = make_default_processors()
        processors[1].after_step_hooks.append(feed)
        logging.getLogger().addFilter(feed)
        record(cfg, teleop_action_processor=processors[0], robot_action_processor=processors[1],
               robot_observation_processor=processors[2])
    finally:
        logging.getLogger().removeFilter(feed)
        closed.set()
        viewer.join(timeout=2)
        if viewer.is_alive():
            viewer.terminate()
            viewer.join(timeout=2)
        frames.close()


if __name__ == "__main__":
    main()
