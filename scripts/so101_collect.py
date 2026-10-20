#!/usr/bin/env python3
"""Terminal entrypoint: native position feedback, existing automatic home reset.

record --check and status read files only. archive moves one stopped dataset
within this workspace; it never deletes it. Only record without --check uses hardware.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path

from so101_adaptive import ROOT, dataset_path, main as adaptive_main


def status(name):
    path = dataset_path(name)
    result = {"dataset": name, "path": str(path), "exists": path.exists(),
              "saved_episodes": 0, "saved_frames": 0, "issues": []}
    if not path.exists():
        result["state"] = "not_created"
        return result
    try:
        info = json.loads((path / "meta/info.json").read_text())
        result.update(saved_episodes=info["total_episodes"], saved_frames=info["total_frames"],
                      format=info.get("codebase_version"))
        if any(type(result[k]) is not int or result[k] < 0 for k in ("saved_episodes", "saved_frames")):
            raise ValueError("Invalid episode/frame counts")
    except (OSError, ValueError, KeyError) as exc:
        result.update(state="needs_inspection", issues=[f"info.json: {exc}"])
        return result
    import pyarrow.parquet as pq

    def parquet_rows(subdir):
        total = 0
        for file in sorted((path / subdir).rglob("*.parquet")):
            try:
                total += pq.ParquetFile(file).metadata.num_rows
            except Exception as exc:
                result["issues"].append(f"{file.relative_to(path)}: {exc}")
        return total

    result["parquet_frames"] = parquet_rows("data")
    result["parquet_episodes"] = parquet_rows("meta/episodes")
    if result["parquet_frames"] != result["saved_frames"]:
        result["issues"].append("Parquet frame count differs from info.json; do not assume a partial save is complete")
    if result["parquet_episodes"] != result["saved_episodes"]:
        result["issues"].append("Episode metadata count differs from info.json")
    result["video_files"] = [str(p.relative_to(path)) for p in sorted((path / "videos").rglob("*.mp4"))]
    if result["saved_frames"]:
        for key, feature in info.get("features", {}).items():
            if feature.get("dtype") == "video" and not any((path / "videos" / key).rglob("*.mp4")):
                result["issues"].append(f"Missing video for {key}")
    elif result["video_files"]:
        result["issues"].append("Video exists without committed frames; preserve for inspection")
    result["state"] = "needs_inspection" if result["issues"] else "saved" if result["saved_episodes"] else "empty"
    result["note"] = "Run after exit. Counts/footer checks do not validate task success or decode every video frame."
    return result


def active_collectors():
    result = []
    for proc in Path("/proc").glob("[0-9]*/cmdline"):
        if proc.parent.name == str(os.getpid()):
            continue
        try:
            args = [s.decode(errors="replace") for s in proc.read_bytes().split(b"\0") if s]
        except OSError:
            continue
        names = {Path(arg).name for arg in args}
        if (names & {"so101_adaptive.py", "record_with_preview.py", "lerobot-record", "lerobot_record.py"}
                or ("so101_collect.py" in names and "record" in args)):
            result.append(proc.parent.name)
    return result


def archive(name):
    path = dataset_path(name)
    if (ROOT / "data/datasets" / name).is_symlink():
        raise ValueError("Refusing to archive a symlink")
    if not path.is_dir():
        raise FileNotFoundError(path)
    users = active_collectors()
    if users:
        raise RuntimeError(f"Stop the collection process before archiving; active PIDs: {users}")
    # Also catch unrelated readers/writers with a file still open under this dataset.
    for fd in Path("/proc").glob("[0-9]*/fd/*"):
        try:
            target = Path(os.readlink(fd)).resolve()
        except OSError:
            continue
        if target.is_relative_to(path):
            raise RuntimeError(f"Dataset is open by PID {fd.parts[2]}; close it before archiving")
    destination_root = (ROOT / "data/dataset_archive").resolve()
    if not destination_root.is_relative_to(ROOT / "data"):
        raise ValueError("Archive directory must remain inside workspace data")
    destination_root.mkdir(parents=True, exist_ok=True)
    destination = destination_root / f"{name}-{datetime.now():%Y%m%d-%H%M%S-%f}"
    if destination.exists():
        raise FileExistsError(destination)
    path.rename(destination)
    return {"archived": str(path), "destination": str(destination), "deleted": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    record = commands.add_parser("record", help="Collect with S/N/R/V and automatic reset")
    record.add_argument("--name", required=True, help="New dataset name, e.g. so101_stack_20261015_01")
    record.add_argument("--episodes", type=int, default=2, help="Successful episodes in this NEW batch (default 2)")
    record.add_argument("--check", action="store_true", help="Print effective configuration only; no hardware")
    for command in ("status", "archive"):
        sub = commands.add_parser(command, help="Inspect dataset" if command == "status" else "Move stopped dataset to archive; no deletion")
        sub.add_argument("name")
    args = parser.parse_args(argv)
    if args.command == "record":
        adaptive_main([
            "--config", str(ROOT / "configs/so101_collect.json"), "--mode", "record",
            "--expected-leader-pid", "16", "0", "32", "--feedback-mode", "official",
            "--dataset-name", args.name, "--episodes", str(args.episodes),
            *(["--check"] if args.check else []),
        ])
    else:
        result = status(args.name) if args.command == "status" else archive(args.name)
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
