"""Compare recorded gripper closure gaps with ACT predictions, entirely offline."""

import json
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/derived/so101_stack_white_on_red_50_20261015"
MODEL = ROOT / "outputs/train/act_stack_20261015_run01/checkpoints/080000/pretrained_model"
OUT = ROOT / "outputs/gripper-analysis-20261015"


def stats(values):
    values = np.asarray(values)
    return {"count": len(values), "mean": float(values.mean()),
            **dict(zip(["min", "p05", "median", "p95", "max"],
                       np.quantile(values, [0, .05, .5, .95, 1]).tolist()))}


def main():
    torch.set_num_threads(4)
    torch.manual_seed(1000)
    OUT.mkdir(exist_ok=False)
    columns = ["action", "observation.state", "episode_index", "frame_index", "index"]
    table = pa.concat_tables([pq.read_table(p, columns=columns) for p in sorted(DATA.glob("data/**/*.parquet"))])
    actions = np.stack(table["action"].to_numpy())
    states = np.stack(table["observation.state"].to_numpy())
    episodes = table["episode_index"].to_numpy()
    frames = table["frame_index"].to_numpy()
    assert np.array_equal(table["index"].to_numpy(), np.arange(len(actions)))
    gap = states[:, 5] - actions[:, 5]
    selected = []
    stable_by_episode = []
    for ep in range(50):
        indices = np.flatnonzero(episodes == ep)
        a, s = actions[indices, 5], states[indices, 5]
        aw = np.lib.stride_tricks.sliding_window_view(a, 15)
        sw = np.lib.stride_tricks.sliding_window_view(s, 15)
        stable = ((sw - aw).min(1) > 5) & (np.ptp(sw, axis=1) < 1) & (np.ptp(aw, axis=1) < 2)
        candidates = indices[np.flatnonzero(stable) + 7]
        stable_by_episode.append({"episode": ep, "stable_frames": len(candidates)})
        if len(candidates):
            selected.extend(candidates[np.linspace(0, len(candidates)-1, 3, dtype=int)].tolist())
    selected = sorted(set(selected))
    # A dense, fixed episode slice also shows the approach/closure/transport phases.
    ep45 = np.flatnonzero(episodes == 45)
    plot_indices = ep45[np.linspace(0, len(ep45)-1, 60, dtype=int)].tolist()
    query_indices = sorted(set(selected + plot_indices))
    preliminary = {"frames": len(gap), "episodes": 50,
                   "gap_gt_5_frames": int((gap > 5).sum()),
                   "gap_gt_5_fraction": float((gap > 5).mean()),
                   "gripper_state_minus_action": stats(gap),
                   "stable_candidate_criteria": "15-frame window: state-action>5 throughout, state peak-to-peak<1, action peak-to-peak<2; numerical proxy, not force/contact measurement",
                   "stable_by_episode": stable_by_episode,
                   "stable_samples": len(selected), "queried_frames": len(query_indices)}
    print(json.dumps(preliminary), flush=True)
    dataset = LeRobotDataset("local/so101_stack_white_on_red_50_20261015", root=DATA,
                             video_backend="pyav", return_uint8=True)
    policy = ACTPolicy.from_pretrained(MODEL).to("cuda").eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=MODEL,
                                        preprocessor_overrides={"device_processor": {"device": "cuda"}})
    records = []
    predictions = {}
    with torch.inference_mode():
        for offset in range(0, len(query_indices), 4):
            batch_ids = query_indices[offset:offset+4]
            items = [dataset[idx] for idx in batch_ids]
            batch = {key: torch.stack([item[key] for item in items]) for key in policy.config.input_features}
            for key in dataset.meta.camera_keys:
                batch[key] = batch[key].float() / 255.0
            prediction = post(policy.predict_action_chunk(pre(batch))).cpu().numpy()
            assert np.isfinite(prediction).all()
            for idx, pred in zip(batch_ids, prediction):
                predictions[idx] = pred[:, 5]
                records.append({"index": idx, "episode": int(episodes[idx]), "frame": int(frames[idx]),
                                "split": "train" if episodes[idx] < 45 else "validation",
                                "stable_candidate": idx in selected,
                                "recorded_state": float(states[idx, 5]),
                                "recorded_target": float(actions[idx, 5]),
                                "recorded_closure_gap": float(gap[idx]),
                                "predicted_target_first": float(pred[0, 5]),
                                "predicted_closure_gap": float(states[idx, 5] - pred[0, 5]),
                                "target_error_positive_means_less_closed": float(pred[0, 5] - actions[idx, 5]),
                                "predicted_gripper_chunk_min": float(pred[:, 5].min()),
                                "predicted_gripper_chunk_max": float(pred[:, 5].max()),
                                "body_first_step_absolute_errors_degrees": np.abs(pred[0, :5]-actions[idx, :5]).tolist()})
            print(f"Predicted {min(offset+4, len(query_indices))}/{len(query_indices)} frames", flush=True)
    summaries = {}
    for split in ("train", "validation", "all"):
        rows = [r for r in records if r["stable_candidate"] and (split == "all" or r["split"] == split)]
        if not rows:
            continue
        errors = np.array([r["target_error_positive_means_less_closed"] for r in rows])
        predicted_gaps = np.array([r["predicted_closure_gap"] for r in rows])
        summaries[split] = {"samples": len(rows),
                            "recorded_gap": stats([r["recorded_closure_gap"] for r in rows]),
                            "predicted_gap": stats(predicted_gaps),
                            "prediction_minus_demonstration_target": stats(errors),
                            "gripper_target_mae": float(np.abs(errors).mean()),
                            "predicted_gap_nonpositive_count": int((predicted_gaps <= 0).sum()),
                            "predicted_gap_below_5_count": int((predicted_gaps < 5).sum()),
                            "target_more_open_by_over_2_count": int((errors > 2).sum())}
    report = {**preliminary, "checkpoint": str(MODEL), "precision": "FP32",
              "hardware_connected": False, "motor_writes": [],
              "limitation": "Model observes recorded successful demonstrations, not the failed deployment. Closure gap is not a measured force. These predictions are the first action of a fresh chunk, not a replay of runtime's cached 100-step queues.",
              "stable_candidate_summaries": summaries, "samples": records}
    with (OUT / "gripper-analysis.json").open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    np.savez_compressed(OUT / "recorded-gripper.npz", episode=episodes, frame=frames,
                        state=states[:, 5], target=actions[:, 5])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
        t = frames[ep45] / 30
        axes[0].plot(t, states[ep45, 5], label="Measured follower position", lw=1.5)
        axes[0].plot(t, actions[ep45, 5], label="Demonstration target", lw=1.5)
        axes[0].plot(frames[plot_indices]/30, [predictions[i][0] for i in plot_indices],
                     ".-", label="ACT first target (recorded observations)", lw=1)
        axes[0].set(title="Held-out episode 45: gripper position and commanded target",
                    xlabel="Time (s)", ylabel="Gripper 0-100 (lower = closed)")
        axes[0].legend(); axes[0].grid(alpha=.25)
        for split, color in (("train", "tab:blue"), ("validation", "tab:orange")):
            rows = [r for r in records if r["stable_candidate"] and r["split"] == split]
            axes[1].scatter([r["recorded_closure_gap"] for r in rows],
                            [r["predicted_closure_gap"] for r in rows], s=22, alpha=.65,
                            label=split, color=color)
        axes[1].plot([0, 24], [0, 24], "k--", lw=1, label="Same closure gap")
        axes[1].set(xlabel="Demonstration: measured position minus target",
                    ylabel="ACT: measured position minus predicted target",
                    title="Stable positive-gap samples: position error is not force")
        axes[1].legend(); axes[1].grid(alpha=.25)
        fig.savefig(OUT / "gripper-comparison.png", dpi=160)
        plt.close(fig)
    except ImportError:
        print("Matplotlib unavailable; numerical report completed.", flush=True)
    # Save a few sample images for manual contact/holding interpretation.
    from PIL import Image, ImageDraw
    chosen = [r for r in records if r["stable_candidate"] and r["episode"] in (10, 44, 45, 46, 49)]
    sheet = Image.new("RGB", (640*3, 390*((len(chosen)+2)//3)), "white")
    draw = ImageDraw.Draw(sheet)
    for n, row in enumerate(chosen):
        item = dataset[row["index"]]
        frame = item["observation.images.top"].numpy().transpose(1, 2, 0)
        picture = Image.fromarray(frame).resize((640, 360))
        x, y = (n % 3)*640, (n // 3)*390
        sheet.paste(picture, (x, y+30))
        draw.text((x+5, y+5), f"ep{row['episode']} f{row['frame']} state={row['recorded_state']:.1f} target={row['recorded_target']:.1f} ACT={row['predicted_target_first']:.1f}", fill="black")
    sheet.save(OUT / "contact-candidates.jpg")
    print(json.dumps(summaries, indent=2), flush=True)


if __name__ == "__main__":
    main()
