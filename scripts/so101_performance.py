#!/usr/bin/env python3
"""Offline control-loop/timing/tracking report from a saved session log."""
import argparse
import json
import math
from pathlib import Path


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return {"mean": sum(values) / len(values), "p95": values[math.ceil(.95 * len(values)) - 1], "max": values[-1]}


def summarize(rows):
    segments, active = [], []
    faults = []
    for row in rows:
        if row.get("event") in ("fault", "fault_paused", "communication_warning"):
            faults.append(row)
        if row.get("event") != "sample":
            continue
        if row.get("following") and row["state"] == "TELEOP":
            active.append(row)
        elif active:
            segments.append(active); active = []
    if active:
        segments.append(active)
    reports = []
    for segment in segments:
        elapsed = segment[-1]["monotonic"] - segment[0]["monotonic"]
        intervals = [(b["monotonic"] - a["monotonic"]) * 1000 for a, b in zip(segment, segment[1:])]
        hz = (len(segment) - 1) / elapsed if elapsed else 0
        cycle = stats(intervals)
        fields = sorted({k for r in segment for k in r["timings_ms"]})
        timing = {k: stats([r["timings_ms"][k] for r in segment if k in r["timings_ms"]]) for k in fields}
        # Compare feedback to the PREVIOUS sent goal, not serial write duration.
        tracking = {j: stats([abs(b["follower"][j] - a["target"][j]) for a, b in zip(segment, segment[1:]) if a["target"]])
                    for j in segment[0]["follower"]}
        reports.append({"duration_s": elapsed, "samples": len(segment), "average_hz": hz,
                        "cycle_ms": cycle, "timings_ms": timing, "tracking_error": tracking,
                        "pass": bool(elapsed >= 60 and hz >= 28 and cycle and cycle["p95"] <= 50 and cycle["max"] < 500)})
    return {"segments": reports, "faults": faults, "pass": bool(any(r["pass"] for r in reports) and not faults),
            "units": "body degrees / gripper points; timings milliseconds",
            "interpretation": "Tracking error uses previous sent target; serial write time is not mechanical response latency"}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("session", type=Path)
    args = p.parse_args()
    with (args.session / "events.jsonl").open() as stream:
        report = summarize(json.loads(line) for line in stream if line.strip())
    print(json.dumps(report, ensure_ascii=False, indent=2))
    with (args.session / "performance.json").open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2); stream.write("\n")


if __name__ == "__main__":
    main()
