"""Check the independent SO-101 software environment without connecting hardware."""

import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--act-smoke", action="store_true")
    parser.add_argument("--act-device", choices=("cpu", "cuda"), default="cpu")
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    expected_env = Path("/home/pangk/miniforge3/envs/lerobot-so101")
    report = {"python": sys.version.split()[0], "prefix": sys.prefix, "checks": {}}
    failures = []
    if Path(sys.prefix) != expected_env:
        failures.append("Python is not running from lerobot-so101")

    for module in [
        "lerobot", "torch", "torchvision", "torchcodec", "cv2", "av",
        "datasets", "accelerate", "serial", "scservo_sdk",
        "lerobot.robots.so_follower.config_so_follower",
        "lerobot.teleoperators.so_leader.config_so_leader",
        "lerobot.policies.act.modeling_act",
    ]:
        try:
            imported = importlib.import_module(module)
            report["checks"][module] = str(getattr(imported, "__file__", "OK"))
        except Exception as exc:
            report["checks"][module] = f"{type(exc).__name__}: {exc}"
            failures.append(f"Import failed: {module}")

    if "lerobot" not in sys.modules or not Path(sys.modules["lerobot"].__file__).is_relative_to(workspace / "lerobot"):
        failures.append("LeRobot is not imported from this workspace")
    for name in ["HF_HOME", "HF_LEROBOT_HOME", "HF_LEROBOT_CALIBRATION", "TORCH_HOME", "UV_CACHE_DIR"]:
        value = os.environ.get(name, "")
        report[name] = value
        if not value or not Path(value).is_relative_to(workspace):
            failures.append(f"Cache/data path is not isolated: {name}")

    ffmpeg = subprocess.run([str(expected_env / "bin/ffmpeg"), "-version"], capture_output=True, text=True)
    report["ffmpeg"] = ffmpeg.stdout.splitlines()[0] if ffmpeg.returncode == 0 else ffmpeg.stderr
    if ffmpeg.returncode:
        failures.append("ffmpeg could not start")

    if "torchcodec" in sys.modules:
        try:
            from torchcodec.decoders import VideoDecoder

            decoder = VideoDecoder(str(workspace / "logs/video-smoke.mp4"), device="cpu")
            frames = decoder.get_frames_at(indices=[0, 1, 2]).data
            report["video_decode_shape"] = list(frames.shape)
            if tuple(frames.shape) != (3, 3, 64, 64):
                failures.append("Unexpected video decoding result")
        except Exception as exc:
            report["video_decode_error"] = f"{type(exc).__name__}: {exc}"
            failures.append("Video decoding failed")

    if "torch" in sys.modules:
        torch = sys.modules["torch"]
        report["torch"] = torch.__version__
        report["cuda_runtime"] = torch.version.cuda
        report["cuda_available"] = torch.cuda.is_available()
        if report["cuda_available"]:
            try:
                value = torch.ones(2, device="cuda").sum().item()
                report["gpu"] = torch.cuda.get_device_name(0)
                report["cuda_tensor_check"] = value == 2
                if value != 2:
                    failures.append("CUDA tensor computation failed")
            except Exception as exc:
                report["cuda_error"] = str(exc)
                failures.append("CUDA tensor computation failed")

        if args.act_smoke and args.act_device == "cuda" and not report["cuda_available"]:
            failures.append("CUDA is unavailable for the requested ACT training check")

        if args.act_smoke and not failures:
            from lerobot.configs.types import FeatureType, PolicyFeature
            from lerobot.policies.act.configuration_act import ACTConfig
            from lerobot.policies.act.modeling_act import ACTPolicy

            torch.set_num_threads(2)
            device = torch.device(args.act_device)
            config = ACTConfig(
                device=args.act_device,
                input_features={
                    "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(6,)),
                    "observation.images.front": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 64, 64)),
                },
                output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(6,))},
                chunk_size=4, n_action_steps=4, dim_model=64, n_heads=4,
                dim_feedforward=128, n_encoder_layers=1, n_decoder_layers=1,
                n_vae_encoder_layers=1, pretrained_backbone_weights=None,
            )
            policy = ACTPolicy(config).to(device)
            batch = {
                "observation.state": torch.randn(2, 6),
                "observation.images.front": torch.rand(2, 3, 64, 64),
                "action": torch.randn(2, 4, 6),
                "action_is_pad": torch.zeros(2, 4, dtype=torch.bool),
            }
            batch = {name: value.to(device) for name, value in batch.items()}
            loss, _ = policy(batch)
            loss.backward()
            optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-5)
            optimizer.step()
            if not torch.isfinite(loss):
                failures.append("ACT produced a non-finite loss")
            report[f"act_{args.act_device}_training_step"] = {
                "device": str(loss.device), "loss": loss.item(),
                "finite": bool(torch.isfinite(loss)),
            }

    report["lerobot_version"] = importlib.metadata.version("lerobot")
    report["failures"] = failures
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return bool(failures)


if __name__ == "__main__":
    sys.exit(main())
