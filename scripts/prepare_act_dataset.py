"""Create a new ACT dataset from the six audited batches; never touch hardware."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from lerobot.datasets.aggregate import aggregate_datasets

WORKSPACE = Path(__file__).resolve().parents[1]
AUDIT = WORKSPACE / "outputs/act-preflight-20261015/source-audit/integrity-report.json"
DESTINATION = WORKSPACE / "data/derived/so101_stack_white_on_red_50_20261015"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    report = json.loads(AUDIT.read_text())
    if not report["passed"] or report["episodes"] != 50 or report["frames"] != 53475:
        raise RuntimeError("Expected a passing 50-episode source audit")
    if DESTINATION.exists():
        raise FileExistsError(f"Refusing to overwrite {DESTINATION}")
    roots = [Path(d["root"]) for d in report["datasets"]]
    for d in report["datasets"]:
        for rel, expected in d["source_sha256"].items():
            if digest(Path(d["root"]) / rel) != expected:
                raise RuntimeError(f"Source changed since audit: {d['root']}/{rel}")
    aggregate_datasets(
        repo_ids=["local/" + p.name for p in roots],
        roots=roots,
        aggr_repo_id="local/" + DESTINATION.name,
        aggr_root=DESTINATION,
        concatenate_videos=False,
        concatenate_data=False,
    )
    info = json.loads((DESTINATION / "meta/info.json").read_text())
    assert (info["total_episodes"], info["total_frames"]) == (50, 53475)
    merged = [r for p in sorted((DESTINATION / "data").rglob("*.parquet")) for r in pq.read_table(p).to_pylist()]
    mapping, frame_offset, episode_offset = [], 0, 0
    for source in report["datasets"]:
        root = Path(source["root"])
        rows = [r for p in sorted((root / "data").rglob("*.parquet")) for r in pq.read_table(p).to_pylist()]
        for local, combined in zip(rows, merged[frame_offset:frame_offset + len(rows)], strict=True):
            for key in ("action", "observation.state", "timestamp", "frame_index", "task_index"):
                assert np.array_equal(local[key], combined[key]), (root.name, local["index"], key)
            assert combined["episode_index"] == local["episode_index"] + episode_offset
            assert combined["index"] == local["index"] + frame_offset
        mapping.append({"source": str(root), "merged_episode_start": episode_offset,
                        "merged_episode_end_exclusive": episode_offset + source["episodes"]})
        frame_offset += source["frames"]
        episode_offset += source["episodes"]
        for rel, expected in source["source_sha256"].items():
            assert digest(root / rel) == expected, f"Source changed during merge: {root / rel}"
    source_videos = sorted(h for s in report["datasets"] for p, h in s["source_sha256"].items() if p.endswith(".mp4"))
    copied_videos = sorted(digest(p) for p in (DESTINATION / "videos").rglob("*.mp4"))
    assert source_videos == copied_videos, "Copied video content mismatch"
    result = {"dataset": str(DESTINATION), "repo_id": "local/" + DESTINATION.name,
              "episodes": episode_offset, "frames": frame_offset, "source_mapping": mapping,
              "source_files_unchanged": True, "video_copies_identical": True,
              "state_action_and_timestamps_identical": True, "source_audit": str(AUDIT)}
    with (DESTINATION / "preparation-manifest.json").open("x") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
