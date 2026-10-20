#!/usr/bin/env python3
"""Single-arm passive calibration. --check is offline; --run requires command review."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import select
import sys
import time

from diagnose_teleop import ROOT, feedback
from so101_readonly import buses_from_config, inventory
from so101_session import JOINTS

WRITE_FIELDS = {"Torque_Enable", "Lock", "Operating_Mode", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit"}
ZERO_FIELDS = {"Torque_Enable", "Lock", "Operating_Mode"}


def validate_write(register, value):
    if register not in WRITE_FIELDS or not isinstance(value, int):
        raise ValueError(f"Forbidden calibration write: {register}={value}")
    if register in ZERO_FIELDS and value != 0:
        raise ValueError(f"Only zero permitted for {register}")
    if register == "Homing_Offset" and not -2047 <= value <= 2047:
        raise ValueError("Homing offset outside encoder range")
    if register in {"Min_Position_Limit", "Max_Position_Limit"} and not 0 <= value <= 4095:
        raise ValueError("Invalid position limit")


class PassiveCalibration:
    def __init__(self, bus, initial, record):
        self.bus, self.initial, self.record = bus, initial, record

    def preflight(self):
        if set(self.initial) != set(JOINTS):
            raise ValueError("Expected six joints")
        for joint, v in self.initial.items():
            if v["Model_Number"] != 777 or v["ID"] != self.bus.motors[joint].id or v["Phase"] & 0x10:
                raise ValueError(f"{joint}: unexpected ID/model/feedback mode; no register changes")
            if v["Status"] or any(v["packet_errors"]):
                raise RuntimeError(f"{joint}: motor fault before calibration")
            if not v["Min_Voltage_Limit"] <= v["Present_Voltage"] <= v["Max_Voltage_Limit"]:
                raise RuntimeError(f"{joint}: supply outside existing protection thresholds")

    def write(self, register, joint, value):
        validate_write(register, value)
        self.record("write_attempt", register=register, joint=joint, value=value)
        self.bus.write(register, joint, value, normalize=False, num_retry=0)
        actual = self.bus.read(register, joint, normalize=False, num_retry=0)
        if actual != value:
            raise RuntimeError(f"{joint}/{register}: readback {actual}, expected {value}")
        self.record("write_verified", register=register, joint=joint, value=actual)

    def release(self):
        # All motor identity/feedback checks finish before the first write.
        self.preflight()
        for joint in JOINTS:
            for register in ("Torque_Enable", "Lock", "Operating_Mode"):
                self.write(register, joint, 0)

    def sample(self):
        rows = feedback(self.bus)
        for joint, v in rows.items():
            limits = self.initial[joint]
            if v["Torque_Enable"] or v["Status"] or v["packet_error"]:
                raise RuntimeError(f"{joint}: torque/fault during passive calibration")
            if not limits["Min_Voltage_Limit"] <= v["Present_Voltage"] <= limits["Max_Voltage_Limit"]:
                raise RuntimeError(f"{joint}: supply outside protection thresholds")
            if not 0 <= v["Present_Position"] <= 4095:
                raise ValueError(f"{joint}: unexpected raw feedback")
        return {j: v["Present_Position"] for j, v in rows.items()}

    def center(self):
        # Same half-turn algorithm as MotorsBus.set_half_turn_homings, with
        # explicit checked writes instead of calling any device configure path.
        for joint in JOINTS:
            self.write("Homing_Offset", joint, 0)
            self.write("Min_Position_Limit", joint, 0)
            self.write("Max_Position_Limit", joint, 4095)
        offsets = self.bus._get_half_turn_homings(self.sample())
        for joint, offset in offsets.items():
            self.write("Homing_Offset", joint, int(offset))
        centered = self.sample()
        if any(abs(v - 2047) > 57 for v in centered.values()):
            raise ValueError("Middle pose moved by >5 degrees during homing; calibration incomplete")
        self.record("center", offsets=offsets, raw=centered)
        return offsets, centered

    def capture_ranges(self):
        print("缓慢逐一移动全部六个关节（含腕转），覆盖可用行程，不强压端点。按 Enter 结束。", flush=True)
        low = self.sample(); high = dict(low); previous = dict(low)
        last_print = 0.0
        while not select.select([sys.stdin], [], [], 0)[0]:
            tick = time.monotonic()
            current = self.sample()
            if any(abs(current[j] - previous[j]) > 2048 for j in JOINTS):
                raise ValueError("Encoder wrapped during range capture; calibration incomplete")
            low = {j: min(low[j], current[j]) for j in JOINTS}
            high = {j: max(high[j], current[j]) for j in JOINTS}
            self.record("range_sample", raw=current)
            if tick - last_print > 0.5:
                print(" | ".join(f"{j} {low[j]}..{current[j]}..{high[j]}" for j in JOINTS), flush=True)
                last_print = tick
            previous = current
            time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        if not sys.stdin.readline():
            raise EOFError("Calibration input closed")
        if any(not 0 <= low[j] < high[j] <= 4095 for j in JOINTS):
            raise ValueError("Every joint needs a nonempty measured range")
        return low, high

    def finish(self, offsets, low, high):
        from lerobot.motors import MotorCalibration

        calibration = {j: MotorCalibration(id=self.bus.motors[j].id, drive_mode=0,
                                          homing_offset=int(offsets[j]), range_min=int(low[j]), range_max=int(high[j]))
                       for j in JOINTS}
        for j, c in calibration.items():
            self.write("Homing_Offset", j, c.homing_offset)
            self.write("Min_Position_Limit", j, c.range_min)
            self.write("Max_Position_Limit", j, c.range_max)
        actual = self.bus.read_calibration()
        if actual != calibration:
            raise ValueError("Calibration readback differs from captured ranges")
        self.bus.calibration = calibration
        after = inventory(self.bus)
        PassiveCalibration(self.bus, after, self.record).preflight()
        for j in JOINTS:
            if any(after[j][k] != 0 for k in ZERO_FIELDS):
                raise RuntimeError(f"{j}: torque/lock/mode changed during calibration")
        # Check every captured nonvolatile/control register, except the exact
        # whitelist. Feedback registers can legitimately change with posture.
        from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE
        fixed = {k for k, (address, _) in STS_SMS_SERIES_CONTROL_TABLE.items()
                 if (address < 56 or address >= 80) and k not in WRITE_FIELDS}
        for j in JOINTS:
            for k in fixed:
                if after[j][k] != self.initial[j][k]:
                    raise ValueError(f"Unexpected register change: {j}/{k}")
        self.record("registers_verified", registers=after)
        return {j: asdict(v) for j, v in calibration.items()}


def check_direction(cal, low, high, prompt, record, trial=1, baseline=None):
    """Direction check from any legal pose; no return to center needed."""
    if baseline is None:
        prompt("停在已采集范围内任意舒适姿态，按 Enter 读取方向检查起点（不用回中位）。")
        baseline = cal.sample()
    if any(not low[j] <= baseline[j] <= high[j] for j in JOINTS):
        raise ValueError("Direction-check starting pose outside captured range")
    prompt("缓慢向各关节自选正方向稍移，并记住所选方向，按 Enter 查看读数。")
    moved = cal.sample()
    if any(not low[j] <= moved[j] <= high[j] for j in JOINTS):
        raise ValueError("Direction check outside captured range")
    delta = {j: moved[j] - baseline[j] for j in JOINTS}
    print(f"原始位置变化：{delta}", flush=True)
    if any(v == 0 for v in delta.values()):
        raise ValueError("Every joint must move for the direction check")
    if prompt("确认六关节读数方向均符合所选方向，输入 yes：").strip().lower() != "yes":
        raise ValueError("Direction check not confirmed; calibration incomplete")
    record("direction_confirmed", trial=trial, raw_start=baseline, raw_end=moved, raw_delta=delta)


def human_checks(cal, center, low, high, prompt, record, repeat_check=False, direction_check=False):
    if not repeat_check:
        record("repeat_center_optional", performed=False,
               reason="Manual placement repeatability is not required for coordinate calibration")
        if direction_check:
            check_direction(cal, low, high, prompt, record)
        else:
            record("direction_check_deferred", reason="Verify arm-to-arm response during the separate follow acceptance test")
        return
    for trial in range(2):
        prompt(f"第 {trial+1}/2 次回位：手动回到刚才中位，按 Enter 读数。")
        position = cal.sample()
        if any(not low[j] <= position[j] <= high[j] for j in JOINTS):
            raise ValueError("Repeat middle pose outside captured range")
        errors = {j: (position[j] - center[j]) * (100 / (high[j] - low[j]) if j == "gripper" else 360 / 4095)
                  for j in JOINTS}
        record("repeat_center", trial=trial+1, raw=position, error=errors)
        if any(abs(v) > 5 for v in errors.values()):
            raise ValueError(f"Repeat middle pose exceeds 5 degrees/points: {errors}")
        check_direction(cal, low, high, prompt, record, trial=trial+1, baseline=position)


def complete_files(result, status, output, status_path, record, repeat_check, direction_check=False):
    path = Path(status["calibration_file"])
    with path.open("x") as stream:
        json.dump(result, stream, indent=2); stream.write("\n")
    if json.loads(path.read_text()) != result:
        raise ValueError("Calibration file verification failed")
    status.pop("error", None)
    status.update(state="complete", sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  validation={"register_readback": "passed",
                              "direction": "operator_confirmed" if direction_check or repeat_check else "deferred_to_follow_test",
                              "manual_repeat": "passed" if repeat_check else "not_required"})
    record("complete", **status)
    status_path.write_text(json.dumps(status, indent=2) + "\n")
    (output / "status.json").write_text(json.dumps(status, indent=2) + "\n")


def run_calibration(bus, arm, role, output, status_path, prompt=input, repeat_check=False, direction_check=False):
    initial = inventory(bus)
    (output / "before.json").write_text(json.dumps(initial, indent=2) + "\n")
    status = {"state": "incomplete", "role": role, "id": arm["id"], "output": str(output),
              "calibration_file": str(output / f'{arm["id"]}.json')}
    status_path.parent.mkdir(parents=True, exist_ok=True)
    # Persist BEFORE the first write. Any interrupted/partial calibration blocks
    # normal session startup, including attempts to reuse an old file.
    status_path.write_text(json.dumps(status, indent=2) + "\n")
    with (output / "events.jsonl").open("x") as log:
        def record(event, **data):
            log.write(json.dumps({"event": event, "time": time.time(), **data}, ensure_ascii=False) + "\n")
            log.flush()
        cal = PassiveCalibration(bus, initial, record)
        try:
            cal.release()
            prompt(f"仅手动摆放 {role} 到各关节中位；支撑机械臂，按 Enter 设置中位。")
            offsets, center = cal.center()
            low, high = cal.capture_ranges()
            if any(not low[j] <= center[j] <= high[j] for j in JOINTS):
                raise ValueError("Captured usable range must include the reference middle pose")
            result = cal.finish(offsets, low, high)
            human_checks(cal, center, low, high, prompt, record, repeat_check, direction_check)
            complete_files(result, status, output, status_path, record, repeat_check, direction_check)
        except BaseException as exc:
            status["state"] = "incomplete"
            status["error"] = repr(exc)
            record("incomplete", **status)
            raise
        finally:
            status_path.write_text(json.dumps(status, indent=2) + "\n")
            (output / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    return status


def resume_source(output, arm, role):
    """Files only: accept only a verified session rejected by manual repeat."""
    status = json.loads((output / "status.json").read_text())
    expected_path = output / f'{arm["id"]}.json'
    if (status["state"] != "incomplete" or status["role"] != role or status["id"] != arm["id"]
            or Path(status["calibration_file"]).resolve() != expected_path.resolve()):
        raise ValueError("Resume identity/status mismatch")
    if expected_path.exists():
        raise FileExistsError(f"Calibration file already exists: {expected_path}")
    events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    if not events or events[-1]["event"] != "incomplete" or not events[-1].get("error", "").startswith(
            "ValueError(\"Repeat middle pose exceeds 5 degrees/points:"):
        raise ValueError("Only a manual repeat-tolerance failure after verified writes is resumable")
    stages = [e["event"] for e in events if e["event"] in ("center", "registers_verified", "repeat_center", "incomplete")]
    if stages != ["center", "registers_verified", "repeat_center", "incomplete"]:
        raise ValueError("Missing or ambiguous verified calibration stages")
    index = next(i for i, e in enumerate(events) if e["event"] == "registers_verified")
    if any(e["event"].startswith("write_") for e in events[index+1:]):
        raise ValueError("Motor writes occurred after register verification")
    registers = events[index]["registers"]
    center = next(e for e in events if e["event"] == "center")["raw"]
    before = json.loads((output / "before.json").read_text())
    if set(registers) != set(JOINTS) or set(center) != set(JOINTS) or set(before) != set(JOINTS):
        raise ValueError("Expected six verified joints")
    result = {}
    for j in JOINTS:
        v = registers[j]
        low, high = v["Min_Position_Limit"], v["Max_Position_Limit"]
        if not 0 <= low <= center[j] <= high <= 4095 or low == high:
            raise ValueError(f"{j}: invalid verified range")
        if v["ID"] != before[j]["ID"] or v["Model_Number"] != 777 or v["Status"] or any(v["packet_errors"]):
            raise ValueError(f"{j}: invalid verified identity/health")
        validate_write("Homing_Offset", v["Homing_Offset"])
        result[j] = dict(id=v["ID"], drive_mode=0, homing_offset=v["Homing_Offset"], range_min=low, range_max=high)
    return status, result, registers, before


def verify_resume_registers(bus, before, expected):
    """No writes, including torque release. Mismatches require separate handling."""
    from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE

    current = inventory(bus)
    PassiveCalibration(bus, current, lambda *a, **k: None).preflight()
    fixed = {k for k, (address, _) in STS_SMS_SERIES_CONTROL_TABLE.items()
             if (address < 56 or address >= 80) and k not in WRITE_FIELDS}
    for j in JOINTS:
        for k in WRITE_FIELDS:
            wanted = 0 if k in ZERO_FIELDS else expected[j][k]
            if current[j][k] != wanted:
                raise ValueError(f"{j}/{k}: current register differs from verified session; no rewrite performed")
        for k in fixed:
            if current[j][k] != before[j][k]:
                raise ValueError(f"{j}/{k}: non-calibration register changed; no rewrite performed")
    return current


def resume_calibration(bus, arm, role, output, status_path, prompt=input, direction_check=False):
    status, result, expected, before = resume_source(output, arm, role)
    if json.loads(status_path.read_text()) != status:
        raise ValueError("Another calibration changed the current status; refusing stale resume")
    stamp = str(time.time_ns())
    # Preserve the original failure, events, and all historical calibrations.
    (output / f"status-before-resume-{stamp}.json").write_text(json.dumps(status, indent=2) + "\n")
    with (output / f"resume-{stamp}.jsonl").open("x") as log:
        def record(event, **data):
            log.write(json.dumps({"event": event, "time": time.time(), **data}, ensure_ascii=False) + "\n")
            log.flush()
        try:
            current = verify_resume_registers(bus, before, expected)
            record("resume_readback", registers=current, motor_writes=[])
            cal = PassiveCalibration(bus, before, record)
            low = {j: v["range_min"] for j, v in result.items()}
            high = {j: v["range_max"] for j, v in result.items()}
            human_checks(cal, None, low, high, prompt, record, direction_check=direction_check)
            current = verify_resume_registers(bus, before, expected)
            record("resume_final_readback", registers=current, motor_writes=[])
            status["resumed_without_motor_writes"] = True
            complete_files(result, status, output, status_path, record, repeat_check=False, direction_check=direction_check)
        except BaseException as exc:
            if status["state"] == "complete":
                status.update(state="incomplete", error=repr(exc))
                status_path.write_text(json.dumps(status, indent=2) + "\n")
                (output / "status.json").write_text(json.dumps(status, indent=2) + "\n")
            record("resume_incomplete", error=repr(exc))
            raise
    return status


def check_empty_output(output):
    """A failed connection may leave an empty directory, but never replace files."""
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError(f"Output is not empty; refusing to replace calibration evidence: {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--arm", choices=("leader", "follower"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeat-check", action="store_true", help="Optional strict manual middle-pose repeatability check, twice")
    parser.add_argument("--direction-check", action="store_true", help="Optional manual direction confirmation; normally deferred to follow testing")
    parser.add_argument("--resume", action="store_true", help="Read-only finish of a verified session rejected only by the manual repeat check")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--run", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    arm = config["recording"]["teleop" if args.arm == "leader" else "robot"]
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / "data/calibration") or output == ROOT / "data/calibration":
        parser.error("Output must be below data/calibration")
    if args.resume:
        if args.repeat_check:
            parser.error("--resume does not repeat manual middle-pose checks")
        resume_source(output, arm, args.arm)
    else:
        try:
            check_empty_output(output)
        except FileExistsError as exc:
            parser.error(str(exc))
    summary = {"arm": args.arm, "port": arm["port"], "id": arm["id"], "config": str(args.config.resolve()),
               "output": str(output), "resume": args.resume,
               "write_whitelist": [] if args.resume else sorted(WRITE_FIELDS), "zero_only": [] if args.resume else sorted(ZERO_FIELDS),
               "homing": "reuse verified offsets; no writes" if args.resume else "raw position after offset reset minus 2047",
               "limits": "reuse verified ranges; no writes" if args.resume else "measured min/max for ALL six joints, including wrist_roll",
               "sampling_hz": None if args.resume else 20, "sampling": "on operator prompts" if args.resume else "20 Hz range capture",
               "units": "encoder counts; verification in degrees / gripper percentage points",
               "repeat_tolerance": "5 degrees / 5 points, twice" if args.repeat_check else "not required",
               "direction_check": "manual confirmation" if args.direction_check or args.repeat_check else "deferred to follow acceptance; no extra operator step",
               "torque_enable": False, "goal_position_write": False,
               "cameras": False, "other_arm_connected": False, "max_relative_target": None,
               "PID_speed_acceleration_protection_Phase": "read/check only; no writes", "overwrite": False,
               "output_directory_policy": "Existing verified session" if args.resume else "New or empty directory only"}
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if args.check:
        return
    if not sys.stdin.isatty():
        parser.error("Passive calibration requires an interactive operator terminal")
    if not Path(arm["port"]).exists():
        parser.error(f"串口不存在：{arm['port']}。请检查该控制板的 USB 连接并运行 ls -l /dev/serial/by-id/；尚未写入校准。")
    status_path = ROOT / "data/calibration/status" / f'{args.arm}-{arm["id"]}.json'
    with buses_from_config(config, (args.arm,)) as buses:
        if args.resume:
            status = resume_calibration(buses[args.arm], arm, args.arm, output, status_path, direction_check=args.direction_check)
        else:
            # Only after the serial connection succeeds. Recheck before creating
            # any files; retries may reuse an empty directory, never old evidence.
            check_empty_output(output)
            output.mkdir(parents=True, exist_ok=True)
            status = run_calibration(buses[args.arm], arm, args.arm, output, status_path,
                                     repeat_check=args.repeat_check, direction_check=args.direction_check)
        print(json.dumps(status, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
