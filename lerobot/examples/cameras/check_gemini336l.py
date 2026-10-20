"""Check the Gemini 336L RGB stream through LeRobot without connecting a robot."""

import argparse
import json
import shutil
import subprocess
import time
from contextlib import suppress
from datetime import datetime
from pathlib import Path

import cv2
import draccus
import numpy as np
from PIL import Image

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.utils import make_cameras_from_configs

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = Path(__file__).with_name("gemini336l.json")


def main() -> None:
    """Save RGB snapshots and a short stream diagnostic report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "outputs/gemini336l")
    parser.add_argument("--preview", action="store_true", help="Show live RGB frames with ffplay; press Q to stop")
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds < 1:
        parser.error("--seconds must be a finite number of at least 1")
    if args.preview and shutil.which("ffplay") is None:
        parser.error("ffplay was not found. Activate the lerobot-so101 environment first.")

    raw_config = json.loads(args.config.read_text())
    configs = draccus.decode(dict[str, CameraConfig], raw_config)
    if set(configs) != {"front"} or not isinstance(configs["front"], OpenCVCameraConfig):
        parser.error("The config must contain exactly one OpenCV camera named 'front'.")
    config = configs["front"]
    device = Path(config.index_or_path)
    if not device.exists():
        parser.error(f"Camera not found: {device}. Check USB connection and /dev/v4l/by-id/.")

    output = args.output_dir / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output.mkdir(parents=True)
    camera = make_cameras_from_configs(configs)["front"]
    count = 0
    first_frame = None
    last_frame = None
    first_received = 0.0
    last_received = 0.0
    max_interval = 0.0
    viewer: subprocess.Popen | None = None
    print(f"Connecting to {device} -> {device.resolve()}", flush=True)
    try:
        camera.connect()
        if args.preview:
            # Feed the already decoded LeRobot RGB stream to the viewer. Opening
            # this device directly with ffplay can stall on empty startup packets.
            viewer = subprocess.Popen(
                [
                    "ffplay",
                    "-hide_banner",
                    "-loglevel", "warning",
                    "-nostats",
                    "-f", "rawvideo",
                    "-pixel_format", "rgb24",
                    "-video_size", f"{config.width}x{config.height}",
                    "-framerate", str(config.fps),
                    "-probesize", "32",
                    "-analyzeduration", "0",
                    "-i", "pipe:0",
                    "-autoexit",
                    "-window_title", "Gemini 336L RGB - Q to close",
                ],
                stdin=subprocess.PIPE,
            )
        start = time.perf_counter()
        while time.perf_counter() - start < args.seconds:
            frame = camera.async_read(timeout_ms=2000)
            received = time.perf_counter()
            expected_shape = (config.height, config.width, 3)
            if frame.shape != expected_shape or frame.dtype != np.uint8:
                raise RuntimeError(f"Unexpected image: {frame.shape}, {frame.dtype}; want {expected_shape}, uint8")
            if count == 0:
                first_frame = frame.copy()
                first_received = received
            else:
                max_interval = max(max_interval, received - last_received)
            last_frame = frame
            last_received = received
            count += 1
            if viewer is not None and viewer.stdin is not None:
                if viewer.poll() is not None:
                    if viewer.returncode != 0:
                        raise RuntimeError(f"Preview exited with code {viewer.returncode}.")
                    break
                try:
                    viewer.stdin.write(frame.tobytes())
                except BrokenPipeError:
                    break
    finally:
        try:
            if camera.is_connected:
                camera.disconnect()
        finally:
            if viewer is not None:
                if viewer.stdin is not None:
                    with suppress(BrokenPipeError):
                        viewer.stdin.close()
                if viewer.poll() is None:
                    viewer.terminate()
                try:
                    viewer.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    viewer.kill()
                    viewer.wait()

    if count < 2 or first_frame is None or last_frame is None:
        raise RuntimeError("Not enough frames received to check the stream.")

    Image.fromarray(first_frame).save(output / "rgb_first.png")
    Image.fromarray(last_frame).save(output / "rgb_last.png")
    measured_fps = (count - 1) / (last_received - first_received)
    report = {
        "captured_at": datetime.now().astimezone().isoformat(),
        "config": raw_config,
        "resolved_device": str(device.resolve()),
        "opencv_version": cv2.__version__,
        "frames_received": count,
        "measured_interval_seconds": round(last_received - first_received, 3),
        "measured_read_fps": round(measured_fps, 2),
        "max_read_interval_ms": round(max_interval * 1000, 2),
        "frame_shape": list(last_frame.shape),
        "frame_dtype": str(last_frame.dtype),
        "frame_mean": round(float(last_frame.mean()), 2),
        "frame_std": round(float(last_frame.std()), 2),
        "camera_disconnected": not camera.is_connected,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Received {count} RGB frames, measured read rate {measured_fps:.2f} FPS.")
    print(f"Snapshots and report saved in: {output}")
    print("Camera released. You can now open a preview or use the camera in LeRobot.")


if __name__ == "__main__":
    main()
