"""Evaluate every saved ACT checkpoint on the entire held-out split, without hardware."""

import argparse
import csv
import hashlib
import json
import math
import random
import re
import time
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PRIMARY_METRIC = "mean_batch_loss_like_training_log"


def resolve(path):
    path = (ROOT / path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f"Expected a workspace path: {path}")
    return path


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_new(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def emit(event, **fields):
    print(json.dumps({"event": event, **fields}, ensure_ascii=False, allow_nan=False), flush=True)


class LossAccumulator:
    """Keep the official batch mean and separately weight loss by valid action elements."""

    def __init__(self):
        self.frames = 0
        self.batch_losses = []
        self.prefix_losses = []
        self.episodes = {}

    def add(self, loss, episodes, error_sums, valid_counts):
        if not math.isfinite(loss):
            raise ValueError("Nonfinite official evaluation loss")
        if not (len(episodes) == len(error_sums) == len(valid_counts)):
            raise ValueError("Per-frame metrics have different lengths")
        if not episodes:
            raise ValueError("Empty evaluation batch")
        self.batch_losses.append(loss)
        self.frames += len(episodes)
        if self.frames <= 500:
            self.prefix_losses.append(loss)
        for episode, error_sum, valid_count in zip(episodes, error_sums, valid_counts, strict=True):
            if valid_count <= 0 or not math.isfinite(error_sum) or error_sum < 0:
                raise ValueError("Invalid masked action error")
            row = self.episodes.setdefault(int(episode), {
                "frames": 0, "absolute_error_sum": 0.0, "valid_action_elements": 0,
                "sample_loss_sum": 0.0,
            })
            row["frames"] += 1
            row["absolute_error_sum"] += error_sum
            row["valid_action_elements"] += valid_count
            row["sample_loss_sum"] += error_sum / valid_count

    def result(self):
        if not self.frames:
            raise ValueError("No validation frames evaluated")
        per_episode = {}
        for episode, row in sorted(self.episodes.items()):
            per_episode[str(episode)] = {
                "frames": row["frames"],
                "valid_action_elements": row["valid_action_elements"],
                "valid_action_element_weighted_loss": row["absolute_error_sum"] / row["valid_action_elements"],
                "mean_sample_loss": row["sample_loss_sum"] / row["frames"],
            }
        total_error = sum(row["absolute_error_sum"] for row in self.episodes.values())
        total_valid = sum(row["valid_action_elements"] for row in self.episodes.values())
        return {
            "frames": self.frames,
            "batches": len(self.batch_losses),
            PRIMARY_METRIC: math.fsum(self.batch_losses) / len(self.batch_losses),
            "valid_action_elements": total_valid,
            "valid_action_element_weighted_loss": total_error / total_valid,
            "mean_episode_loss": sum(row["valid_action_element_weighted_loss"] for row in per_episode.values()) / len(per_episode),
            "prefix_500_batch_mean": math.fsum(self.prefix_losses) / len(self.prefix_losses),
            "per_episode": per_episode,
        }


def inspect_plan(settings):
    if settings["max_eval_samples"] != 0:
        raise ValueError("This evaluator requires max_eval_samples=0 (all validation frames)")
    if settings["primary_metric"] != PRIMARY_METRIC:
        raise ValueError("Selection must use the official training-log loss aggregation")
    if settings["device"] != "cuda" or settings["mixed_precision"] != "fp16":
        raise ValueError("Use CUDA FP16 autocast, matching the original evaluation")
    steps = settings["checkpoint_steps"]
    if steps != list(range(5000, 100001, 5000)):
        raise ValueError("Expected all 20 checkpoints, 005000 through 100000")
    run = resolve(settings["run_dir"])
    reference = run / f"checkpoints/{settings['reference_checkpoint']:06d}/pretrained_model"
    train_config = json.loads((reference / "train_config.json").read_text())
    if Path(train_config["dataset"]["root"]).resolve() != resolve(settings["dataset_root"]):
        raise ValueError("Dataset root differs from the saved training configuration")
    if train_config["dataset"]["eval_split"] != 0.1:
        raise ValueError("Saved validation split differs from the reviewed 45/5 split")
    if settings["validation_episodes"] != list(range(45, 50)):
        raise ValueError("Expected validation episodes 45 through 49")
    if settings["batch_size"] != train_config["batch_size"]:
        raise ValueError("Batch size must match the original evaluation")
    logged = {int(step): float(loss) for step, loss in re.findall(
        r"step (\d+): eval_loss=([\d.]+)", resolve(settings["training_log"]).read_text()
    )}
    common_files = ["config.json", "policy_preprocessor.json", "policy_postprocessor.json"]
    common_files += sorted(p.name for p in reference.glob("*processor*.safetensors"))
    fingerprints = {name: digest(reference / name) for name in common_files}
    checkpoints = []
    for step in steps:
        path = run / f"checkpoints/{step:06d}/pretrained_model"
        if {name: digest(path / name) for name in common_files} != fingerprints:
            raise ValueError(f"Model architecture or preprocessing differs at step {step}")
        if step not in logged:
            raise ValueError(f"Missing original evaluation result for step {step}")
        checkpoints.append({"step": step, "pretrained_model": str(path),
                            "model_bytes": (path / "model.safetensors").stat().st_size,
                            "logged_subset_eval_loss": logged[step]})
    return {
        "settings": settings, "checkpoints": checkpoints,
        "reference_train_config": str(reference / "train_config.json"),
        "common_model_and_processor_sha256": fingerprints,
        "source_sha256": {
            str(path.relative_to(ROOT)): digest(path) for path in [
                Path(__file__), ROOT / "scripts/evaluate_act_full.sh",
                reference / "train_config.json", resolve(settings["training_log"]),
                ROOT / "lerobot/src/lerobot/policies/act/modeling_act.py",
                ROOT / "lerobot/src/lerobot/scripts/lerobot_train.py",
                ROOT / "lerobot/src/lerobot/datasets/factory.py",
                resolve(settings["dataset_root"]) / "meta/info.json",
                resolve(settings["dataset_root"]) / "meta/stats.json",
            ]
        },
        "hardware_connected": False, "training_or_weight_updates": False,
        "normalization": "Use saved checkpoint processors unchanged; original statistics include all 50 episodes.",
        "selection": "Minimum full-validation batch-mean loss, exactly as the training loop aggregates eval_loss.",
        "additional_metrics": "Valid-action-element-weighted loss, per-episode loss, equal-episode mean, original 500-frame subset.",
    }


def runtime(settings, plan):
    # Keep metadata inspection and the aggregation tests independent of CUDA and model imports.
    import numpy as np
    import torch
    import torch.nn.functional as F
    from torch.utils.data import DataLoader

    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.datasets.factory import make_train_eval_datasets
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.scripts.lerobot_train import _preprocess_dataset_batch

    cfg = TrainPipelineConfig.from_pretrained(plan["reference_train_config"], cli_args=[])
    cfg.max_eval_samples = 0
    torch.set_num_threads(settings["cpu_threads"])
    torch.manual_seed(settings["seed"])
    np.random.seed(settings["seed"])
    random.seed(settings["seed"])
    torch.backends.cudnn.deterministic = cfg.cudnn_deterministic
    torch.backends.cudnn.benchmark = not cfg.cudnn_deterministic
    torch.backends.cuda.matmul.allow_tf32 = True
    _, dataset = make_train_eval_datasets(cfg)
    if dataset.episodes != settings["validation_episodes"] or len(dataset) != settings["expected_validation_frames"]:
        raise ValueError("Loaded validation episodes/frame count differ from the reviewed dataset")
    expected_indices = np.asarray(dataset.hf_dataset.data.column("index").to_numpy())
    episode_indices = np.asarray(dataset.hf_dataset.data.column("episode_index").to_numpy())
    episode_counts = {str(ep): int((episode_indices == ep).sum()) for ep in dataset.episodes}
    if len(np.unique(expected_indices)) != len(dataset):
        raise ValueError("Validation dataset contains duplicate frame indices")
    if not np.all(episode_indices[:500] == 45):
        raise ValueError("Original 500-frame subset no longer corresponds to episode 45")
    # No Subset, sampler truncation, shuffle or drop_last: every held-out frame is visited.
    loader = DataLoader(dataset, batch_size=settings["batch_size"], shuffle=False, drop_last=False,
                        num_workers=settings["num_workers"], pin_memory=True,
                        prefetch_factor=settings["prefetch_factor"],
                        persistent_workers=settings["persistent_workers"],
                        multiprocessing_context=settings["multiprocessing_context"])
    out = resolve(settings["output_dir"])
    out.mkdir(parents=True, exist_ok=False)
    plan["started_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    plan["loaded_validation_episode_counts"] = episode_counts
    plan["loaded_validation_frames"] = len(dataset)
    plan["torch_version"] = torch.__version__
    plan["gpu"] = torch.cuda.get_device_name(0)
    write_new(out / "evaluation-plan.json", plan)
    emit("validation_loaded", frames=len(dataset), episodes=episode_counts, gpu=plan["gpu"])
    completed = []
    started = time.perf_counter()
    for checkpoint in plan["checkpoints"]:
        step = checkpoint["step"]
        path = Path(checkpoint["pretrained_model"])
        checkpoint_started = time.perf_counter()
        emit("checkpoint_started", step=step)
        model_sha = digest(path / "model.safetensors")
        policy = ACTPolicy.from_pretrained(path).to(settings["device"]).eval()
        preprocessor, _ = make_pre_post_processors(
            policy.config, pretrained_path=path,
            preprocessor_overrides={"device_processor": {"device": settings["device"]}},
        )
        captured = []

        def capture_prediction(module, inputs, output):
            captured[:] = [output[0].detach()]

        # The official forward computes the primary loss; this hook only reads predictions
        # to attribute masked absolute error to individual episodes, including mixed batches.
        hook = policy.model.register_forward_hook(capture_prediction)
        metrics = LossAccumulator()
        max_reduction_difference = 0.0
        torch.cuda.reset_peak_memory_stats()
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
            for batch_number, raw in enumerate(loader, 1):
                raw_indices = raw["index"].reshape(-1).tolist()
                raw_episodes = raw["episode_index"].reshape(-1).tolist()
                cursor = metrics.frames
                if raw_indices != expected_indices[cursor:cursor + len(raw_indices)].tolist():
                    raise ValueError("Evaluation skipped, duplicated or reordered validation frames")
                batch = _preprocess_dataset_batch(raw, dataset.meta.camera_keys, cfg.rename_map, preprocessor)
                official_loss, loss_dict = policy(batch)
                if "kld_loss" in loss_dict:
                    raise ValueError("Expected inference-mode ACT L1 loss without the training VAE KL term")
                abs_error = F.l1_loss(batch["action"], captured.pop(), reduction="none")
                valid = ~batch["action_is_pad"]
                counts = valid.sum(dim=1) * abs_error.shape[-1]
                sums = (abs_error * valid.unsqueeze(-1)).sum(dim=(1, 2))
                check_loss = float(sums.sum() / counts.sum())
                value = float(official_loss)
                max_reduction_difference = max(max_reduction_difference, abs(check_loss - value))
                if not math.isfinite(check_loss) or abs(check_loss - value) > 1e-5 * max(1.0, abs(value)):
                    raise ValueError("Per-episode error accounting does not match the official loss")
                metrics.add(value, raw_episodes, sums.double().cpu().tolist(), counts.cpu().tolist())
                if batch_number % 100 == 0:
                    emit("progress", step=step, frames=metrics.frames, total=len(dataset),
                         elapsed_s=round(time.perf_counter() - checkpoint_started, 1))
        hook.remove()
        result = metrics.result()
        if result["frames"] != len(dataset) or {ep: row["frames"] for ep, row in result["per_episode"].items()} != episode_counts:
            raise ValueError("Incomplete full-validation coverage")
        result.update(checkpoint)
        result.update({"model_sha256": model_sha, "max_loss_reduction_difference": max_reduction_difference,
                       "elapsed_seconds": time.perf_counter() - checkpoint_started,
                       "peak_cuda_memory_mib": torch.cuda.max_memory_allocated() / 2**20,
                       "original_subset_loss_difference": result["prefix_500_batch_mean"] - checkpoint["logged_subset_eval_loss"]})
        write_new(out / f"checkpoint-{step:06d}.json", result)
        completed.append(result)
        emit("checkpoint_finished", step=step, full_eval_loss=result[PRIMARY_METRIC],
             element_weighted_loss=result["valid_action_element_weighted_loss"],
             original_subset_loss=result["prefix_500_batch_mean"],
             elapsed_s=round(result["elapsed_seconds"], 1))
        del policy, preprocessor, batch, raw, abs_error, valid, counts, sums, official_loss
        torch.cuda.empty_cache()
    ranked = sorted(completed, key=lambda row: row[PRIMARY_METRIC])
    for rank, row in enumerate(ranked, 1):
        row["rank"] = rank
    report = {"status": "complete", "selection_metric": PRIMARY_METRIC,
              "validation_episodes": dataset.episodes, "validation_frames": len(dataset),
              "checkpoint_count": len(ranked), "selected": ranked[0],
              "ranked_checkpoints": ranked,
              "valid_action_weighted_ranking": [row["step"] for row in sorted(completed, key=lambda row: row["valid_action_element_weighted_loss"])],
              "previous_deployment_step": 80000, "deployment_config_modified": False,
              "elapsed_seconds": time.perf_counter() - started,
              "normalization_note": plan["normalization"], "hardware_connected": False}
    write_new(out / "checkpoint-selection.json", report)
    with (out / "ranking.csv").open("x", newline="", encoding="utf-8") as stream:
        fields = ["rank", "step", PRIMARY_METRIC, "valid_action_element_weighted_loss", "mean_episode_loss", "prefix_500_batch_mean"]
        writer = csv.DictWriter(stream, fieldnames=fields + [f"episode_{ep}_loss" for ep in dataset.episodes])
        writer.writeheader()
        for row in ranked:
            csv_row = {key: row[key] for key in fields}
            csv_row.update({f"episode_{ep}_loss": row["per_episode"][str(ep)]["valid_action_element_weighted_loss"] for ep in dataset.episodes})
            writer.writerow(csv_row)
    emit("evaluation_finished", selected_step=ranked[0]["step"], full_eval_loss=ranked[0][PRIMARY_METRIC],
         checkpoints=len(ranked), frames_per_checkpoint=len(dataset), output_dir=str(out),
         elapsed_s=round(report["elapsed_seconds"], 1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--check-only", action="store_true", help="Inspect configuration/files only; no model or dataset loading, CUDA use, or output creation.")
    args = parser.parse_args()
    settings = json.loads(args.config.read_text())
    plan = inspect_plan(settings)
    plan["evaluation_config"] = str(args.config.resolve())
    plan["evaluation_config_sha256"] = digest(args.config)
    if resolve(settings["output_dir"]).exists():
        raise FileExistsError(f"Output already exists: {settings['output_dir']}; no files overwritten")
    if args.check_only:
        print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
        return
    runtime(settings, plan)


if __name__ == "__main__":
    main()
