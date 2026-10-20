"""Audit saved ACT checkpoints and exercise official inference without hardware."""

import argparse
import hashlib
import json
import re
import time
from pathlib import Path

import numpy as np
import torch
from safetensors.torch import load_file
from torch.utils.data import DataLoader, Subset

from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.factory import make_train_eval_datasets
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.rollout.inference.sync import SyncInferenceEngine
from lerobot.scripts.lerobot_train import _preprocess_dataset_batch


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "outputs/train/act_stack_20261015_run01"
OUT = ROOT / "outputs/act-deployment-audit-20261015"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_report(name, report):
    with (OUT / name).open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def evaluate(policy, preprocessor, dataset, indices):
    loader = DataLoader(Subset(dataset, indices), batch_size=4, shuffle=False,
                        num_workers=4, multiprocessing_context="spawn", pin_memory=True)
    losses = []
    valid_elements = 0
    weighted_loss = 0.0
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
        for batch in loader:
            batch = _preprocess_dataset_batch(batch, dataset.meta.camera_keys, {}, preprocessor)
            loss, _ = policy(batch)
            value = float(loss)
            assert np.isfinite(value), "Nonfinite evaluation loss"
            count = int((~batch["action_is_pad"]).sum()) * 6
            valid_elements += count
            weighted_loss += value * count
            losses.append(value)
    return {"frames": len(indices), "batches": len(losses),
            "mean_batch_loss_like_training_log": float(np.mean(losses)),
            "valid_action_element_weighted_loss": weighted_loss / valid_elements}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inference-only", action="store_true",
                        help="Reuse the completed immutable checkpoint inventory.")
    args = parser.parse_args()
    torch.set_num_threads(4)
    torch.manual_seed(1000)
    np.random.seed(1000)
    OUT.mkdir(exist_ok=True)
    log = ROOT / "logs/train-act-stack-20261015-run01.log"
    contents = log.read_text()
    losses = [(int(step), float(loss)) for step, loss in
              re.findall(r"step (\d+): eval_loss=([\d.]+)", contents)]
    assert len(losses) == 20 and "End of training" in contents
    gradients = re.findall(r"grdn:([^\s]+)", contents)
    train_losses = re.findall(r"(?<!_)loss:([^\s]+)", contents)
    rows = []
    if args.inference_only:
        inventory_report = json.loads((OUT / "checkpoint-selection.json").read_text())
        assert inventory_report["log_sha256"] == digest(log)
        rows = inventory_report["ranked_checkpoints"]
    for step, loss in ([] if args.inference_only else losses):
        path = RUN / f"checkpoints/{step:06d}/pretrained_model"
        invalid = []
        tensors_checked = 0
        for file in sorted(path.glob("*.safetensors")):
            tensors = load_file(str(file))
            for name, tensor in tensors.items():
                tensors_checked += 1
                if not torch.isfinite(tensor).all():
                    invalid.append(f"{file.name}:{name}")
            del tensors
        assert tensors_checked and not invalid, (step, invalid)
        rows.append({"step": step, "logged_eval_loss": loss, "pretrained_model": str(path),
                     "all_model_and_processor_tensors_finite": True,
                     "tensors_checked": tensors_checked})
    rows.sort(key=lambda row: row["logged_eval_loss"])
    best = rows[0]
    checkpoint = Path(best["pretrained_model"])
    selection = {
        "hardware_connected": False, "training_completed": True, "steps": 100000,
        "selection_basis": "Minimum of 20 logged held-out eval losses; first 500 frames of episode 45 ONLY",
        "validation_scope_warning": "Training sampler takes first 500 frames per task, not all 5 held-out episodes.",
        "normalization_warning": "Saved dataset statistics include all 50 episodes, including held-out episodes.",
        "logged_training_loss_windows": len(train_losses),
        "nonfinite_training_loss_windows": sum(not np.isfinite(float(x)) for x in train_losses),
        "logged_gradient_norm_windows": len(gradients),
        "nonfinite_gradient_norm_windows": sum(not np.isfinite(float(x)) for x in gradients),
        "gradient_note": "Window averages do not reveal exact count of skipped FP16 updates.",
        "log_sha256": digest(log), "ranked_checkpoints": rows, "selected": best,
        "selected_file_sha256": {p.name: digest(p) for p in sorted(checkpoint.iterdir()) if p.is_file()},
    }
    if not args.inference_only:
        write_report("checkpoint-selection.json", selection)
    else:
        assert selection["selected_file_sha256"] == inventory_report["selected_file_sha256"]
    print(json.dumps({"selected": best, "nonfinite_gradient_norm_windows":
                      selection["nonfinite_gradient_norm_windows"]}), flush=True)

    config = TrainPipelineConfig.from_pretrained(str(checkpoint / "train_config.json"), cli_args=[])
    # This is evaluation only: train.validate() rejects an already completed output directory.
    _, val = make_train_eval_datasets(config)
    assert val.episodes == list(range(45, 50))
    policy = ACTPolicy.from_pretrained(checkpoint).to("cuda").eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=checkpoint,
                                        preprocessor_overrides={"device_processor": {"device": "cuda"}})
    started = time.perf_counter()
    recorded = evaluate(policy, pre, val, list(range(500)))
    episode_arr = np.asarray(val.hf_dataset["episode_index"])
    samples = []
    per_episode = {}
    for ep in val.episodes:
        frames = np.flatnonzero(episode_arr == ep)
        chosen = frames[np.linspace(0, len(frames) - 1, 20, dtype=int)].tolist()
        per_episode[str(ep)] = chosen
        samples.extend(chosen)
    balanced = evaluate(policy, pre, val, samples)

    action_keys = val.meta.features["action"]["names"]
    engine = SyncInferenceEngine(policy, pre, post, val.meta.features, action_keys,
                                 "抓起白色积木，叠在红色积木上，再松开夹爪。", "cuda", "so_follower")
    measured = []
    for ep in val.episodes:
        item = val[per_episode[str(ep)][0]]
        obs = {"observation.state": item["observation.state"].numpy()}
        for key in val.meta.camera_keys:
            obs[key] = item[key].numpy().transpose(1, 2, 0).copy()
        engine.reset()
        chunk_times = []
        queued_times = []
        actions = []
        for tick in range(101):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            action = engine.get_action(obs)
            torch.cuda.synchronize()
            elapsed_ms = (time.perf_counter() - t0) * 1000
            assert action.shape == (6,) and torch.isfinite(action).all()
            (chunk_times if tick % 100 == 0 else queued_times).append(elapsed_ms)
            actions.append(action.numpy())
        values = np.stack(actions)
        measured.append({"episode": ep, "actions": len(actions),
                         "fresh_chunk_ms": chunk_times,
                         "queued_action_ms_median": float(np.median(queued_times)),
                         "action_min": values.min(0).tolist(), "action_max": values.max(0).tolist()})
    report = {"checkpoint": str(checkpoint), "hardware_connected": False,
              "evaluation_precision": "FP16 autocast, same as training evaluation",
              "reproduced_logged_subset": recorded,
              "logged_loss": best["logged_eval_loss"],
              "balanced_smoke_100_frames": balanced,
              "balanced_frame_indices": per_episode,
              "balanced_scope_note": "20 evenly spaced frames per held-out episode, selected checkpoint only; not an all-checkpoint full-validation ranking.",
              "official_inference_engine": "SyncInferenceEngine",
              "inference_use_amp": policy.config.use_amp,
              "inference_precision": "FP32", "action_keys": action_keys,
              "inference_results": measured,
              "inference_note": "Repeated fixed observation per episode to test chunk queue; offline, no control-loop timing claim.",
              "elapsed_seconds": time.perf_counter() - started,
              "peak_cuda_memory_mib": torch.cuda.max_memory_allocated() / 2**20,
              "passed": True}
    write_report("offline-inference-check.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
