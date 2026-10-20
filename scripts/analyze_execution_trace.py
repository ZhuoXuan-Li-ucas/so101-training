"""Find gripper-opening candidates in executed actions and export +/-2 s videos."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np


def opening_events(rows, minimum_rise=3.0, lookback_s=.5):
    """Evidence selection only: never used by the robot control path."""
    signals = (
        ("sent_target", "packet_finished_t_s", lambda r: r["sent_gripper_target"]),
        ("feedback", "observe_started_t_s", lambda r: r["feedback"]["gripper.pos"]),
    )
    candidates = []
    for name, time_key, value in signals:
        window = []
        for row in rows:
            if not row.get("sent") or time_key not in row:
                continue
            t, y = row[time_key], value(row)
            window = [(at, v, i) for at, v, i in window if t-at <= lookback_s]
            if window:
                low_t, low_y, low_index = min(window, key=lambda x: x[1])
                if y-low_y >= minimum_rise:
                    candidates.append({"source": name, "onset_t_s": low_t, "detected_t_s": t,
                                       "rise": y-low_y, "from_index": low_index, "to_index": row["index"]})
            window.append((t, y, row["index"]))
    groups = []
    for candidate in sorted(candidates, key=lambda x: x["detected_t_s"]):
        if not groups or candidate["detected_t_s"]-groups[-1]["last_detected_t_s"] > .75:
            groups.append({"event": len(groups)+1, "onset_t_s": candidate["onset_t_s"],
                           "last_detected_t_s": candidate["detected_t_s"], "sources": [], "max_rise": {}})
        group = groups[-1]
        group["onset_t_s"] = min(group["onset_t_s"], candidate["onset_t_s"])
        group["last_detected_t_s"] = candidate["detected_t_s"]
        name = candidate["source"]
        if name not in group["sources"]:
            group["sources"].append(name)
        group["max_rise"][name] = max(group["max_rise"].get(name, 0), candidate["rise"])
    return groups


def distribution(values):
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        return None
    return {"count": len(values), "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)), "p99": float(np.percentile(values, 99)), "max": float(max(values))}


def analyze(directory, clips=False):
    meta = json.loads((directory / "trace.json").read_text())
    rows = [json.loads(line) for line in (directory / "steps.jsonl").read_text().splitlines() if line]
    sent = [r for r in rows if r.get("sent")]
    if not sent:
        raise ValueError("No successfully sent policy actions in this trace")
    gi = meta["action_keys"].index("gripper.pos")
    analysis = directory / "analysis"
    analysis.mkdir(exist_ok=True)
    fields = ("index", "observe_t_s", "send_t_s", "chunk_id", "chunk_step", "chunk_age_s", "selected_normalized",
              "after_policy_postprocess", "after_robot_processors", "sent_target", "raw_goal", "feedback", "inference_ms")
    with (analysis / "timeline.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fields)
        writer.writeheader()
        for r in sent:
            writer.writerow(dict(zip(fields, (
                r["index"], r["observe_started_t_s"], r["packet_finished_t_s"], r["chunk_id"], r["chunk_step"],
                r["packet_finished_t_s"]-r["chunk_started_t_s"], r["selected_normalized"][gi],
                r["after_policy_postprocess"][gi], r["after_robot_processors"]["gripper.pos"], r["sent_gripper_target"],
                r["raw_goal_packet"][str(meta["gripper_id"])], r["feedback"]["gripper.pos"], r["inference_ms"]))))
    times = np.array([r["packet_finished_t_s"] for r in sent])
    periods = np.diff(times)*1000
    report = {"checkpoint": meta["checkpoint"], "n_action_steps": meta["n_action_steps"], "actions_sent": len(sent),
              "selected_chunk_mismatches": sum(not r.get("selected_matches_chunk", False) for r in sent),
              "send_interval_ms": distribution(periods),
              "send_gaps_over_2_periods": int((periods > 2000/meta["fps"]).sum()),
              "new_chunk_inference_ms": distribution([r["inference_ms"] for r in sent if r["chunk_step"] == 0]),
              "warm_new_chunk_inference_ms": distribution([r["inference_ms"] for r in sent if r["chunk_step"] == 0 and r["chunk_id"] > 1]),
              "cached_action_inference_ms": distribution([r["inference_ms"] for r in sent if r["chunk_step"] != 0]),
              "largest_policy_to_robot_processor_change": max(abs(r["after_policy_postprocess"][gi]-r["after_robot_processors"]["gripper.pos"]) for r in sent),
              "largest_robot_processor_to_raw_sent_change": max(abs(r["after_robot_processors"]["gripper.pos"]-r["sent_gripper_target"]) for r in sent),
              "opening_candidate_rule": "Sent target or position rises >=3 normalized points within 0.5 s. Nearby detections grouped. This does not label success, failure, or premature release.",
              "limitation": "Feedback precedes each row's action. Video timestamps are host frame availability. Opening may be intended placement; inspect video. A 100-vs-10 comparison requires matched scene trials.",
              "events": opening_events(sent)}
    video_status_path = directory / "video-status.json"
    video_status = json.loads(video_status_path.read_text()) if video_status_path.exists() else {}
    report["video_status"] = video_status
    ffmpeg = str(Path("/home/pangk/miniforge3/envs/lerobot-so101/bin/ffmpeg"))
    if not Path(ffmpeg).is_file():
        ffmpeg = shutil.which("ffmpeg")
    for event in report["events"]:
        t = event["onset_t_s"]
        event["window_start_s"], event["window_end_s"] = max(0, t-2), min(float(times[-1]), t+2)
        event["truncated_window"] = bool(t < 2 or t+2 > times[-1])
        nearest = min(sent, key=lambda r: abs(r["packet_finished_t_s"]-t))
        event["nearby_chunk_id"], event["nearby_chunk_step"] = nearest["chunk_id"], nearest["chunk_step"]
        event["clips"] = {}
        if clips:
            for camera in ("top", "wrist"):
                source = directory / f"{camera}.mkv"
                if not source.exists() or not isinstance(video_status.get(camera), dict):
                    continue
                zero = video_status[camera].get("video_zero_t_s", 0)
                begin = max(0, event["window_start_s"]-zero)
                end = max(begin, event["window_end_s"]-zero)
                if end <= begin:
                    continue
                output = analysis / f"opening-{event['event']:03d}-{camera}.mp4"
                subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-y", "-ss", str(begin),
                                "-i", str(source), "-t", str(end-begin), "-an", "-c:v", "libx264", "-preset", "veryfast",
                                "-crf", "18", "-movflags", "+faststart", str(output)], check=True)
                event["clips"][camera] = str(output)
    (analysis / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--clips", action="store_true")
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory, args.clips), ensure_ascii=False, indent=2))
