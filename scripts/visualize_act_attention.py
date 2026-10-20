"""Reconstruct ACT cross-attention from a recorded trial; never connect hardware.

The movie pairs live recorded frames with attention on the *chunk input* frames.
Each dispatched action selects its own decoder query from the recorded chunk.
Compressed video permits an approximation, not recovery of the original tensors.
Run through visualize_act_attention.sh to keep all caches in this workspace.
"""

import argparse
from bisect import bisect_left, bisect_right
from contextlib import ExitStack
from datetime import datetime
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import av
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def read_json(path):
    return json.loads(path.read_text())


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def digest(path):
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def write_json(path, data):
    with path.open("x") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def workspace_path(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f"Path must stay inside the workspace: {path}")
    return path


def nearest_frame(times, timestamp):
    """Match a recorded input timestamp, with an explicit signed error."""
    if timestamp is None or not math.isfinite(timestamp) or not len(times):
        raise ValueError("Missing/nonfinite camera input timestamp or empty journal")
    right = bisect_left(times, timestamp)
    candidates = [i for i in (right - 1, right) if 0 <= i < len(times)]
    index = min(candidates, key=lambda i: (abs(times[i] - timestamp), i))
    return index, float(times[index] - timestamp)


def current_index(times, timestamp):
    """Never display a future frame/action as if it already happened."""
    return bisect_right(times, timestamp) - 1


def split_attention(weights, grids, prefix_tokens):
    """Remove latent/state tokens and undo ACT's per-camera (h w) flattening."""
    expected = prefix_tokens + sum(h * w for h, w in grids)
    if weights.ndim != 2 or weights.shape[1] != expected:
        raise ValueError(f"Attention shape {weights.shape} does not match {expected} source tokens")
    if not np.isfinite(weights).all() or (weights < -1e-7).any():
        raise ValueError("Invalid attention probabilities")
    if not np.allclose(weights.sum(-1), 1, atol=1e-5):
        raise ValueError("Attention probabilities must sum to one before removing non-image tokens")
    images = []
    offset = prefix_tokens
    for height, width in grids:
        images.append(weights[:, offset:offset + height * width].reshape(-1, height, width))
        offset += height * width
    return images


class AttentionCapture:
    """Read existing head-averaged weights without changing MHA arguments/output."""

    def __init__(self, model):
        self.model = model
        self.handles = []
        self.clear()

    def clear(self):
        self.grids = []
        self.weights = []

    def __enter__(self):
        def backbone(_module, _args, output):
            self.grids.append(tuple(output["feature_map"].shape[-2:]))

        def attention(_module, _args, output):
            if output[1] is None:
                raise ValueError("This ACT version did not return attention weights")
            self.weights.append(output[1].detach().float().cpu().numpy().copy())

        self.handles = [self.model.backbone.register_forward_hook(backbone),
                        self.model.decoder.layers[-1].multihead_attn.register_forward_hook(attention)]
        return self

    def __exit__(self, *_):
        for handle in self.handles:
            handle.remove()


class FrameReader:
    """Sequential local-file decoder, checked against the recorder's frame journal."""

    def __init__(self, path, journal):
        self.container = av.open(str(path))
        self.frames = self.container.decode(video=0)
        self.journal = journal
        self.index = -1
        self.picture = None
        self.max_pts_error = 0.0

    def at(self, index):
        if index < self.index:
            raise ValueError("FrameReader cannot move backwards")
        while self.index < index:
            frame = next(self.frames)
            self.index += 1
            error = abs(float(frame.time) - self.journal[self.index]["pts_s"])
            self.max_pts_error = max(self.max_pts_error, error)
            # Matroska quantizes this recording to a 1 ms time base.
            if error > 0.0011:
                raise ValueError(f"Video and frame journal disagree by {error} s at frame {self.index}")
            if self.index == index:
                self.picture = frame.to_ndarray(format="rgb24")
        return self.picture

    def close(self):
        self.container.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def load_trace(directory):
    meta = read_json(directory / "trace.json")
    session = read_json(directory.parent / "session.json")
    if meta["temporal_ensemble_coeff"] is not None or session["interpolation_multiplier"] != 1:
        raise ValueError("Only recorded ACT chunks without temporal ensembling/interpolation are supported")
    if session.get("use_amp", False):
        raise ValueError("This exporter expects the recorded full-precision inference configuration")
    steps = read_rows(directory / "steps.jsonl")
    chunks = read_rows(directory / "chunks.jsonl")
    if not steps or not chunks:
        raise ValueError("Trace contains no observations or chunks")
    by_index = {row["index"]: row for row in steps}
    by_chunk = {row["chunk_id"]: row for row in chunks}
    if len(by_index) != len(steps) or len(by_chunk) != len(chunks):
        raise ValueError("Duplicate observation/chunk IDs")
    sent = [row for row in steps if row.get("sent")]
    if not sent:
        raise ValueError("Trace contains no dispatched actions")
    for row in sent:
        actions = by_chunk[row["chunk_id"]]["actions_normalized"]
        q = row["chunk_step"]
        if not 0 <= q < meta["n_action_steps"]:
            raise ValueError("Invalid executed action query index")
        if row["selected_normalized"] != actions[q] or not row["selected_matches_chunk"]:
            raise ValueError("Recorded selected action does not match its chunk/query")
    send_times = [row["packet_finished_t_s"] for row in sent]
    if any(b <= a for a, b in zip(send_times, send_times[1:])):
        raise ValueError("Action dispatch times are not strictly increasing")
    return meta, session, by_index, chunks, sent, send_times


def load_camera(directory, name, chunks, steps):
    journal = read_rows(directory / f"{name}-frames.jsonl")
    times = [row["capture_t_s"] for row in journal]
    if not times or any(not math.isfinite(t) for t in times):
        raise ValueError(f"Invalid {name} frame times")
    if [row["frame"] for row in journal] != list(range(len(journal))):
        raise ValueError(f"Non-contiguous {name} frame journal")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"Non-monotonic {name} frame times")
    matches = []
    for chunk in chunks:
        row = steps[chunk["observation_index"]]
        requested = row["cameras"][name]["capture_t_s"]
        index, delta = nearest_frame(times, requested)
        if abs(delta) > 0.1:
            raise ValueError(f"{name} input is missing by more than 100 ms for chunk {chunk['chunk_id']}")
        matches.append({"frame": index, "requested_capture_t_s": requested,
                        "recorded_capture_t_s": times[index], "offset_ms": delta * 1000,
                        "exact_timestamp_match": abs(delta) < 1e-6})
    pictures = {}
    with FrameReader(directory / f"{name}.mkv", journal) as reader:
        for index in sorted({match["frame"] for match in matches}):
            pictures[index] = reader.at(index).copy()
        reader.at(len(journal) - 1)
        try:
            next(reader.frames)
        except StopIteration:
            pass
        else:
            raise ValueError(f"{name} video has more frames than its journal")
        pts_error = reader.max_pts_error
    return {"journal": journal, "times": times, "matches": matches, "pictures": pictures,
            "decoded_frames": len(journal), "max_pts_error_s": pts_error}


def reconstruct(directory, output, device):
    import torch
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference

    torch.set_num_threads(4)
    cv2.setNumThreads(2)
    meta, session, steps, chunks, sent, send_times = load_trace(directory)
    checkpoint = workspace_path(meta["checkpoint"])
    train = read_json(checkpoint / "train_config.json")
    info_path = workspace_path(train["dataset"]["root"]) / "meta/info.json"
    features = read_json(info_path)["features"]
    state_names = features["observation.state"]["names"]
    if state_names != meta["action_keys"]:
        raise ValueError("Training state order differs from recorded action order; review before proceeding")
    config = ACTConfig.from_pretrained(checkpoint, local_files_only=True)
    config.device = device
    policy = ACTPolicy.from_pretrained(checkpoint, config=config, local_files_only=True, strict=True).to(device).eval()
    if policy.config.use_amp or policy.config.env_state_feature:
        raise ValueError("Only full precision ACT with recorded joint state and camera inputs is supported")
    if policy.config.chunk_size != meta["chunk_size"]:
        raise ValueError("Checkpoint/trace chunk sizes differ")
    preprocessor, _ = make_pre_post_processors(
        policy.config, pretrained_path=checkpoint,
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    image_keys = list(policy.config.image_features)
    cameras = [key.removeprefix("observation.images.") for key in image_keys]
    if cameras != ["top", "wrist"]:
        raise ValueError(f"This two-camera layout expects top/wrist in model order, found {cameras}")
    media = {name: load_camera(directory, name, chunks, steps) for name in cameras}
    prefix_tokens = policy.model.encoder_1d_feature_pos_embed.num_embeddings
    weights, predictions, recorded, chunk_reports = [], [], [], []
    hook_error = None
    grids = None
    with torch.inference_mode(), AttentionCapture(policy.model) as capture:
        for ci, chunk in enumerate(chunks):
            row = steps[chunk["observation_index"]]
            raw = {"observation.state": np.asarray([row["feedback"][k] for k in state_names], dtype=np.float32)}
            for key, name in zip(image_keys, cameras):
                m = media[name]
                raw[key] = m["pictures"][m["matches"][ci]["frame"]]
                expected = tuple(policy.config.image_features[key].shape)
                if expected != (3, 480, 640):
                    raise ValueError("This video layout requires 640 x 480 RGB camera inputs")
                if (raw[key].shape[2], *raw[key].shape[:2]) != expected:
                    raise ValueError(f"Recorded {name} resolution differs from model input")
            batch = preprocessor(prepare_observation_for_inference(raw, torch.device(device)))
            capture.clear()
            prediction = policy.predict_action_chunk(batch).detach().float().cpu().numpy()[0]
            if len(capture.weights) != 1 or len(capture.grids) != len(cameras):
                raise ValueError("Unexpected backbone/cross-attention calls")
            attention = capture.weights[0]
            if attention.shape[:2] != (1, meta["chunk_size"]):
                raise ValueError("Expected a batch of head-averaged action-query attention")
            attention = attention[0]
            if grids is not None and grids != capture.grids:
                raise ValueError("Image feature geometry changed between chunks")
            grids = capture.grids.copy()
            split_attention(attention, grids, prefix_tokens)
            if ci == 0:
                # Removing all read-only hooks must leave the prediction unchanged.
                capture.__exit__()
                reference = policy.predict_action_chunk(batch).detach().float().cpu().numpy()[0]
                hook_error = float(np.max(np.abs(reference - prediction)))
                if not np.allclose(reference, prediction, rtol=0, atol=1e-6):
                    raise ValueError("Attention hooks changed the inference output")
                capture.__enter__()
            original = np.asarray(chunk["actions_normalized"], dtype=np.float32)
            if original.shape != prediction.shape or not np.isfinite(prediction).all():
                raise ValueError("Invalid reconstructed actions")
            error = np.abs(prediction - original)
            weights.append(attention)
            predictions.append(prediction)
            recorded.append(original)
            chunk_reports.append({"chunk_id": chunk["chunk_id"], "observation_index": chunk["observation_index"],
                                  "started_t_s": chunk["started_t_s"], "finished_t_s": chunk["finished_t_s"],
                                  "frames": {name: media[name]["matches"][ci] for name in cameras},
                                  "normalized_action_mae": float(error.mean()),
                                  "normalized_action_max_abs_error": float(error.max())})
            print(f"Reconstructed chunk {ci + 1}/{len(chunks)}; normalized action MAE={error.mean():.6f}", flush=True)
    weights, predictions, recorded = map(np.stack, (weights, predictions, recorded))
    chunk_index = {chunk["chunk_id"]: i for i, chunk in enumerate(chunks)}
    executed = np.stack([weights[chunk_index[r["chunk_id"]], r["chunk_step"]] for r in sent])
    executed_errors = np.stack([predictions[chunk_index[r["chunk_id"]], r["chunk_step"]] -
                               recorded[chunk_index[r["chunk_id"]], r["chunk_step"]] for r in sent])
    vmax = float(np.quantile(executed[:, prefix_tokens:], 0.995))
    if not math.isfinite(vmax) or vmax <= 0:
        raise ValueError("Invalid attention color range")
    state_path = checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors"
    sources = [Path(__file__), ROOT / "scripts/visualize_act_attention.sh",
               ROOT / "lerobot/src/lerobot/policies/act/modeling_act.py",
               checkpoint / "model.safetensors", checkpoint / "config.json",
               checkpoint / "policy_preprocessor.json", state_path, info_path,
               directory.parent / "session.json", directory / "trace.json", directory / "steps.jsonl",
               directory / "chunks.jsonl"]
    sources += [directory / f"{name}{suffix}" for name in cameras for suffix in (".mkv", "-frames.jsonl")]
    report = {
        "created_at": datetime.now().astimezone().isoformat(), "trace_dir": str(directory),
        "checkpoint": str(checkpoint), "device": device, "torch_version": torch.__version__,
        "hardware_connected": False, "weights_updated": False,
        "method": "Last decoder cross-attention to encoder memory; native MHA mean over attention heads.",
        "attention_module": f"model.decoder.layers.{len(policy.model.decoder.layers) - 1}.multihead_attn",
        "head_count": policy.config.n_heads, "camera_order": cameras, "feature_grids_hw": grids,
        "prefix_tokens": prefix_tokens, "prefix_token_order": ["zero_vae_latent", "robot_state"],
        "state_order": state_names, "chunk_size": meta["chunk_size"], "n_action_steps": meta["n_action_steps"],
        "inferences_reconstructed": len(chunks), "actions_dispatched": len(sent),
        "hook_prediction_max_abs_difference": hook_error,
        "attention_probability_sum_max_error": float(np.max(np.abs(weights.sum(-1) - 1))),
        "executed_normalized_action_mae": float(np.abs(executed_errors).mean()),
        "executed_normalized_action_max_abs_error": float(np.abs(executed_errors).max()),
        "executed_normalized_action_mae_per_joint": np.abs(executed_errors).mean(0).tolist(),
        "color_scale": {"vmin": 0.0, "vmax": vmax, "colormap": "TURBO",
                        "rule": "Global 99.5th percentile over both cameras and all dispatched action queries; fixed for entire movie; values above vmax clipped for display only.",
                        "raw_probabilities_renormalized": False},
        "movie_semantics": "Upper row: current recorded camera frames. Lower row: that dispatched action's chunk input frame with its query attention. Inputs remain fixed until a new chunk is used; no heatmap is projected onto a different live frame.",
        "limitations": ["Offline reconstruction from lossy H.264 camera recordings, not the original runtime attention tensors.",
                        "Missing input frames use the nearest recorded timestamp within 100 ms; every offset is reported per chunk.",
                        "Camera timestamps indicate host availability, not exposure. Encoded MKV PTS have millisecond precision.",
                        "Spatial weights address contextualized encoder tokens; they are not pixel-level segmentation or a causal attribution of action correctness.",
                        "Attention is bilinearly enlarged from the backbone grid, without tracking or temporal interpolation."],
        "media": {name: {key: media[name][key] for key in ("decoded_frames", "max_pts_error_s")} for name in cameras},
        "chunks": chunk_reports,
        "source_sha256": {str(path.relative_to(ROOT)): digest(path) for path in sources},
    }
    np.savez_compressed(output / "attention.npz", attention=weights,
                        actions_reconstructed=predictions, actions_recorded=recorded,
                        chunk_ids=np.array([c["chunk_id"] for c in chunks]),
                        feature_grids_hw=np.asarray(grids), prefix_tokens=np.array(prefix_tokens))
    return report, media, weights, sent, send_times, chunk_index


def overlay(picture, weights, vmax, gamma):
    enlarged = cv2.resize(weights, (picture.shape[1], picture.shape[0]), interpolation=cv2.INTER_LINEAR)
    normalized = np.clip(enlarged / vmax, 0, 1) ** gamma
    colors = cv2.cvtColor(cv2.applyColorMap(np.uint8(normalized * 255), cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
    opacity = (0.45 + 0.2 * normalized)[..., None]
    return np.uint8(np.clip(picture * (1 - opacity) + colors * opacity, 0, 255))


def text_label(image, text, at, scale=0.58, color=(235, 240, 245)):
    cv2.putText(image, text, at, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def make_layout(report):
    canvas = Image.new("RGB", (1280, 1192), (16, 22, 31))
    draw = ImageDraw.Draw(canvas)
    font = lambda size: ImageFont.truetype(str(FONT), size)
    round_name = Path(report["trace_dir"]).name
    draw.text((20, 8), f"ACT 推理注意力 · {round_name}", font=font(28), fill=(241, 246, 251))
    draw.text((20, 46), f"最后一层解码器 cross-attention · {report['head_count']} 头均值 · 离线重建", font=font(18), fill=(173, 189, 207))
    for x, name in ((0, "TOP / 俯视"), (640, "WRIST / 腕部")):
        draw.text((x + 16, 77), f"{name} — 当前推理录像", font=font(20), fill=(215, 228, 240))
        draw.text((x + 16, 589), f"{name} — 动作块输入帧 + 注意力", font=font(20), fill=(255, 198, 103))
    draw.text((18, 1162), "下排保留该动作块的输入帧；缓存动作不重新看图。全片统一非线性色阶，非物体分割。", font=font(18), fill=(176, 191, 208))
    arr = np.array(canvas)
    gradient = np.tile(np.linspace(0, 255, 210, dtype=np.uint8), (14, 1))
    arr[34:48, 1036:1246] = cv2.cvtColor(cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
    midpoint = report['color_scale']['vmax'] * .5 ** (1 / report['color_scale']['gamma'])
    text_label(arr, f"0      {midpoint:.4f}    >={report['color_scale']['vmax']:.4f}", (1036, 68), .38)
    text_label(arr, f"WEIGHT (gamma={report['color_scale']['gamma']:g})", (1036, 25), .45)
    return arr


def render(output, directory, report, media, weights, sent, send_times, chunk_index, fps):
    cameras = report["camera_order"]
    grids = report["feature_grids_hw"]
    prefix = report["prefix_tokens"]
    spatial = [split_attention(w, grids, prefix) for w in weights]
    base = make_layout(report)
    duration = max(media[name]["times"][-1] for name in cameras) + 1 / fps
    count = math.ceil(duration * fps)
    preview_indices = {round(count * f): f"preview-{i + 1:02d}.jpg" for i, f in enumerate((.08, .35, .65, .9))}
    video_path = output / "attention.mp4"
    timeline_path = output / "video-timeline.jsonl"
    previous_key, overlays, source_info = None, None, None
    with ExitStack() as stack:
        readers = {name: stack.enter_context(FrameReader(directory / f"{name}.mkv", media[name]["journal"])) for name in cameras}
        video = stack.enter_context(av.open(str(video_path), mode="w", options={"movflags": "+faststart"}))
        stream = video.add_stream("libx264", rate=fps)
        stream.width, stream.height, stream.pix_fmt = base.shape[1], base.shape[0], "yuv420p"
        stream.options = {"crf": "19", "preset": "veryfast"}
        stream.codec_context.thread_count = 2
        stream.time_base = stream.codec_context.time_base = Fraction(1, fps)
        journal_out = stack.enter_context(timeline_path.open("x"))
        for fi in range(count):
            timestamp = fi / fps
            frame = base.copy()
            live_indices = {}
            for ci, name in enumerate(cameras):
                index = current_index(media[name]["times"], timestamp)
                live_indices[name] = index if index >= 0 else None
                if index >= 0:
                    picture = readers[name].at(index)
                    frame[108:588, ci * 640:(ci + 1) * 640] = picture
            si = current_index(send_times, timestamp)
            row = sent[si] if si >= 0 else None
            if row is not None:
                chunk = chunk_index[row["chunk_id"]]
                query = row["chunk_step"]
                key = (chunk, query)
                if key != previous_key:
                    source_info = report["chunks"][chunk]
                    overlays = []
                    for ci, name in enumerate(cameras):
                        match = source_info["frames"][name]
                        picture = media[name]["pictures"][match["frame"]]
                        heat = overlay(picture, spatial[chunk][ci][query], report["color_scale"]["vmax"], report["color_scale"]["gamma"])
                        # Keep input time/alignment visible, including the first missing frames.
                        heat[-27:] = np.uint8(heat[-27:] * .25)
                        suffix = "matched" if match["exact_timestamp_match"] else f"nearest {match['offset_ms']:+.1f}ms"
                        text_label(heat, f"INPUT t={match['requested_capture_t_s']:.3f}s | {suffix}", (12, 472), .5)
                        overlays.append(heat)
                    previous_key = key
                for ci, picture in enumerate(overlays):
                    frame[620:1100, ci * 640:(ci + 1) * 640] = picture
                observation_t = min(x["requested_capture_t_s"] for x in source_info["frames"].values())
                text_label(frame, f"t={timestamp:6.2f}s   CHUNK {row['chunk_id']:02d}/{len(weights):02d}   ACTION {query + 1:03d}/{report['chunk_size']}   INPUT AGE {timestamp - observation_t:.2f}s", (20, 1125), .65)
                mass = [float(spatial[chunk][ci][query].sum()) for ci in range(len(cameras))]
                other = weights[chunk, query, :prefix]
                grid_label = " / ".join(f"{h}x{w}" for h, w in grids)
                text_label(frame, f"WEIGHT MASS:  TOP {mass[0]:.1%}   WRIST {mass[1]:.1%}   LATENT {other[0]:.1%}   STATE {other[1]:.1%}    |    grids {grid_label}", (20, 1150), .55)
            else:
                text_label(frame, "WAITING FOR FIRST ACTION DISPATCH / ATTENTION NOT YET AVAILABLE", (20, 1125), .6)
            frame[108:1100, 638:642] = (16, 22, 31)
            journal_out.write(json.dumps({"video_frame": fi, "trace_t_s": timestamp,
                                         "recorded_camera_frames": live_indices,
                                         "step_index": row["index"] if row else None,
                                         "chunk_id": row["chunk_id"] if row else None,
                                         "query_index": row["chunk_step"] if row else None}) + "\n")
            if fi in preview_indices:
                Image.fromarray(frame).save(output / preview_indices[fi], quality=94)
            encoded = av.VideoFrame.from_ndarray(frame, format="rgb24")
            encoded.pts, encoded.time_base = fi, Fraction(1, fps)
            for packet in stream.encode(encoded):
                video.mux(packet)
            if fi % 300 == 0:
                print(f"Rendered {fi}/{count} frames", flush=True)
        for packet in stream.encode():
            video.mux(packet)
    # A complete decode catches truncated files and timestamp/encoder failures.
    decoded = 0
    with av.open(str(video_path)) as video:
        for fi, frame in enumerate(video.decode(video=0)):
            if abs(float(frame.time) - fi / fps) > .001:
                raise ValueError("Output video timestamp mismatch")
            decoded += 1
    if decoded != count:
        raise ValueError(f"Output video incomplete: {decoded}/{count} frames")
    report["video"] = {"path": str(video_path), "frames": count, "fully_decoded_frames": decoded,
                       "fps": fps, "duration_s": count / fps, "width": base.shape[1], "height": base.shape[0],
                       "codec": "h264", "pixel_format": "yuv420p", "sha256": digest(video_path)}
    previews = [Image.open(output / name) for name in preview_indices.values()]
    sheet = Image.new("RGB", (1280, 1192))
    for i, picture in enumerate(previews):
        sheet.paste(picture.resize((640, 596)), ((i % 2) * 640, (i // 2) * 596))
        picture.close()
    sheet.save(output / "contact-sheet.jpg", quality=92)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory inside the workspace; must not exist")
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--color-gamma", type=float, default=.5,
                        help="Fixed display transform (weight/vmax)**gamma; does not change saved attention")
    args = parser.parse_args()
    if Path(sys.prefix) != Path("/home/pangk/miniforge3/envs/lerobot-so101"):
        raise ValueError("Use the existing lerobot-so101 environment")
    if not 1 <= args.fps <= 60:
        parser.error("--fps must be in 1..60")
    if not 0 < args.color_gamma <= 1:
        parser.error("--color-gamma must be in (0, 1]")
    directory, output = workspace_path(args.trace_dir), workspace_path(args.output_dir)
    if not FONT.is_file():
        raise FileNotFoundError(f"Chinese font is unavailable: {FONT}")
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report, media, weights, sent, send_times, chunk_index = reconstruct(directory, output, args.device)
    report["color_scale"].update(gamma=args.color_gamma,
                                 display_transform="clip(weight/vmax, 0, 1)**gamma; identical across all cameras and frames",
                                 overlay_opacity="0.45 + 0.2 * transformed_weight")
    write_json(output / "reconstruction.json", report)
    render(output, directory, report, media, weights, sent, send_times, chunk_index, args.fps)
    report["elapsed_s"] = time.perf_counter() - started
    write_json(output / "report.json", report)
    print(json.dumps({"video": report["video"], "report": str(output / "report.json"),
                      "elapsed_s": report["elapsed_s"]}, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
