"""Read recorded datasets without hardware; write new audit evidence only."""

import argparse
import hashlib
import json
from pathlib import Path

import av
import numpy as np
import pyarrow.parquet as pq
from PIL import Image, ImageDraw


def audit(root: Path, output: Path) -> dict:
    files = sorted(p for p in root.rglob("*") if p.is_file())
    before = {str(p.relative_to(root)): [p.stat().st_size, p.stat().st_mtime_ns] for p in files}
    info = json.loads((root / "meta/info.json").read_text())
    rows = [r for p in sorted((root / "data").rglob("*.parquet")) for r in pq.read_table(p).to_pylist()]
    episodes = [r for p in sorted((root / "meta/episodes").rglob("*.parquet")) for r in pq.read_table(p).to_pylist()]
    issues = []

    def check(condition, message):
        if not condition:
            issues.append(message)

    check(len(rows) == info["total_frames"], "Total frame count mismatch")
    check(len(episodes) == info["total_episodes"], "Episode count mismatch")
    check([r["index"] for r in rows] == list(range(len(rows))), "Global indices are not contiguous")
    check([e["episode_index"] for e in episodes] == list(range(len(episodes))), "Episode indices are not contiguous")
    report = {"root": str(root), "episodes": len(episodes), "frames": len(rows), "fps": info["fps"],
              "issues": issues, "episode_details": [], "videos": []}
    video_expectations = {}
    thumbnail_requests = {}
    for e in episodes:
        idx, start, end = e["episode_index"], e["dataset_from_index"], e["dataset_to_index"]
        part = rows[start:end]
        n = len(part)
        check(n == e["length"] and n > 0, f"Episode {idx}: length mismatch")
        check(all(r["episode_index"] == idx for r in part), f"Episode {idx}: boundary mismatch")
        check([r["frame_index"] for r in part] == list(range(n)), f"Episode {idx}: frame index mismatch")
        check(all(r["task_index"] == 0 for r in part), f"Episode {idx}: unexpected task")
        check(np.allclose([r["timestamp"] for r in part], np.arange(n) / info["fps"], atol=1e-5), f"Episode {idx}: timestamp mismatch")
        detail = {"episode_index": idx, "frames": n, "duration_s": n / info["fps"]}
        for key in ("action", "observation.state"):
            values = np.asarray([r[key] for r in part])
            check(values.shape == (n, 6) and np.isfinite(values).all(), f"Episode {idx}: invalid {key}")
            detail[key] = {"min": values.min(axis=0).tolist(), "max": values.max(axis=0).tolist(),
                           "max_frame_delta": np.abs(np.diff(values, axis=0)).max(axis=0).tolist()}
        report["episode_details"].append(detail)
        for key, feature in info["features"].items():
            if feature["dtype"] != "video":
                continue
            rel = info["video_path"].format(video_key=key, chunk_index=e[f"videos/{key}/chunk_index"], file_index=e[f"videos/{key}/file_index"])
            first = round(e[f"videos/{key}/from_timestamp"] * info["fps"])
            last = round(e[f"videos/{key}/to_timestamp"] * info["fps"])
            check(last - first == n, f"Episode {idx}: video length mismatch for {key}")
            check(first == video_expectations.get(rel, 0), f"Episode {idx}: video boundary mismatch for {key}")
            video_expectations[rel] = last
            for label, frame_idx in (("start", first), ("middle", first + n // 2), ("end", last - 1)):
                thumbnail_requests.setdefault(rel, {})[frame_idx] = (idx, key, label)
    thumbs = {}
    for rel, expected_frames in video_expectations.items():
        count, bad_pts, previous, sizes = 0, 0, None, set()
        with av.open(str(root / rel)) as container:
            stream = container.streams.video[0]
            stream.codec_context.thread_count = 2
            check(float(stream.average_rate) == info["fps"], f"{rel}: fps mismatch")
            for count, frame in enumerate(container.decode(stream), start=1):
                t = float(frame.pts * frame.time_base)
                if previous is not None and abs(t - previous - 1 / info["fps"]) > 1e-5:
                    bad_pts += 1
                previous = t
                sizes.add((frame.width, frame.height))
                if count - 1 in thumbnail_requests[rel]:
                    thumbs[thumbnail_requests[rel][count - 1]] = frame.to_image().resize((256, 192))
        check(count == expected_frames, f"{rel}: decoded {count}, expected {expected_frames}")
        check(bad_pts == 0 and sizes == {(640, 480)}, f"{rel}: PTS or resolution mismatch")
        report["videos"].append({"path": rel, "decoded_frames": count, "bad_pts": bad_pts})
        print(f"Decoded {root.name}/{rel}: {count} frames", flush=True)
    for offset in range(0, len(episodes), 10):
        group = episodes[offset:offset + 10]
        sheet = Image.new("RGB", (6 * 256, len(group) * 216), "white")
        draw = ImageDraw.Draw(sheet)
        for row_idx, e in enumerate(group):
            idx = e["episode_index"]
            for col, (label, camera) in enumerate((label, camera) for label in ("start", "middle", "end") for camera in ("top", "wrist")):
                xy = (col * 256, row_idx * 216)
                draw.text(xy, f"ep {idx} | {label} | {camera}", fill="black")
                picture = thumbs.get((idx, "observation.images." + camera, label))
                if picture is not None:
                    sheet.paste(picture, (xy[0], xy[1] + 24))
        sheet.save(output / f"{root.name}-episodes-{offset:02d}.jpg", quality=88)
    after = {str(p.relative_to(root)): [p.stat().st_size, p.stat().st_mtime_ns] for p in files}
    check(before == after, "Source files changed during audit")
    report["source_files_unchanged"] = before == after
    report["source_sha256"] = {}
    for p in files:
        with p.open("rb") as stream:
            report["source_sha256"][str(p.relative_to(root))] = hashlib.file_digest(stream, "sha256").hexdigest()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("roots", type=Path, nargs="+")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reports = [audit(p.resolve(), args.output) for p in args.roots]
    infos = [json.loads((p / "meta/info.json").read_text()) for p in args.roots]
    consistent = all((x["features"], x["fps"], x["robot_type"]) == (infos[0]["features"], infos[0]["fps"], infos[0]["robot_type"]) for x in infos)
    report = {"datasets": reports, "compatible_features": consistent,
              "episodes": sum(r["episodes"] for r in reports), "frames": sum(r["frames"] for r in reports),
              "passed": consistent and all(not r["issues"] for r in reports),
              "scope": "Structural integrity, full video decoding and sampled visual evidence; task success requires separate review."}
    with (args.output / "integrity-report.json").open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({k: v for k, v in report.items() if k != "datasets"}, ensure_ascii=False))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
