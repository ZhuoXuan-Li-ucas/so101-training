"""Validate ACT configuration and samples without constructing/training a policy or opening hardware."""

import json
from pathlib import Path

import draccus
import numpy as np
import torch

from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import FeatureType
from lerobot.datasets.factory import make_train_eval_datasets
from lerobot.policies.act.configuration_act import ACTConfig  # noqa: F401: register config
from lerobot.utils.feature_utils import dataset_to_policy_features


def main():
    workspace = Path(__file__).resolve().parents[1]
    path = workspace / "configs/train_act_stack_20261015.proposed.json"
    cfg = TrainPipelineConfig.from_pretrained(str(path), cli_args=[])
    cfg.validate()
    assert cfg.policy.device == "cuda"
    train, val = make_train_eval_datasets(cfg)
    features = dataset_to_policy_features(train.meta.features)
    cfg.policy.input_features = {k: v for k, v in features.items() if v.type != FeatureType.ACTION}
    cfg.policy.output_features = {k: v for k, v in features.items() if v.type == FeatureType.ACTION}
    cfg.policy.validate_features()
    report = {"config": str(path), "training_started": False, "hardware_connected": False, "splits": {}}
    for label, ds in (("train", train), ("validation", val)):
        ep_indices = np.asarray(ds.hf_dataset["episode_index"])
        sampled = 0
        for ep in ds.episodes:
            available = np.flatnonzero(ep_indices == ep)
            for index in (available[0], available[len(available) // 2], available[-1]):
                item = ds[int(index)]
                assert item["action"].shape == (100, 6)
                assert item["observation.state"].shape == (6,)
                for key in ("observation.images.top", "observation.images.wrist"):
                    assert item[key].shape == (3, 480, 640)
                assert all(torch.isfinite(v).all() for v in item.values() if isinstance(v, torch.Tensor))
                sampled += 1
        report["splits"][label] = {"episodes": ds.episodes, "frames": len(ds), "samples_checked": sampled}
    assert train.episodes == list(range(45)) and val.episodes == list(range(45, 50))
    report["features"] = {k: {"type": v.type.value, "shape": list(v.shape)} for k, v in features.items()}
    report["cuda"] = torch.cuda.get_device_name(0)
    report["passed"] = True
    report["normalization_note"] = "Built-in loader uses merged dataset statistics (all 50 episodes) for both splits."
    out = workspace / "outputs/act-preflight-20261015"
    with (out / "loader-check.json").open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    with draccus.config_type("json"), (out / "resolved-train-config.json").open("x") as stream:
        draccus.dump(cfg, stream, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
