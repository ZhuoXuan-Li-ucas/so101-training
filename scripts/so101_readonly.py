#!/usr/bin/env python3
"""Read-only inventory / power survey. Never writes a motor register, even on exit."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import time

from diagnose_teleop import ROOT, ensure_port_free, feedback


@contextmanager
def buses_from_config(config, roles=("leader", "follower")):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    arms = {"leader": config["recording"]["teleop"], "follower": config["recording"]["robot"]}
    buses = {}
    for role in roles:
        ensure_port_free(arms[role]["port"])
    try:
        for role in roles:
            arm = arms[role]
            calibration = json.loads((Path(arm["calibration_dir"]) / f'{arm["id"]}.json').read_text())
            bus = FeetechMotorsBus(
                arm["port"],
                {j: Motor(v["id"], "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.DEGREES)
                 for j, v in calibration.items()},
                {j: MotorCalibration(**v) for j, v in calibration.items()},
            )
            buses[role] = bus
            bus.connect(handshake=False)
            bus.set_baudrate(1_000_000)  # Host serial setting only; no Baud_Rate write.
            bus.set_timeout(100)
        yield buses
    finally:
        for bus in buses.values():
            if bus.is_connected:
                bus.disconnect(disable_torque=False)


def inventory(bus):
    from lerobot.motors.feetech.tables import STS_SMS_SERIES_CONTROL_TABLE

    rows = {}
    for joint, motor in bus.motors.items():
        values = {"expected_id": motor.id, "expected_model": 777, "packet_errors": []}
        for start, count in ((0, 71), (80, 7)):
            data, comm, error = bus.packet_handler.readTxRx(bus.port_handler, motor.id, start, count)
            if comm != bus._comm_success or len(data) != count:
                raise ConnectionError(f"{joint}: {bus.packet_handler.getTxRxResult(comm)}; bytes={len(data)}")
            values["packet_errors"].append(error)
            for field, (address, size) in STS_SMS_SERIES_CONTROL_TABLE.items():
                if start <= address and address + size <= start + count:
                    raw = int.from_bytes(bytes(data[address-start:address-start+size]), "little")
                    values[field] = bus._decode_sign(field, {motor.id: raw})[motor.id]
        cal = bus.calibration[joint]
        values["calibration_matches"] = all(values[k] == v for k, v in {
            "Homing_Offset": cal.homing_offset, "Min_Position_Limit": cal.range_min,
            "Max_Position_Limit": cal.range_max}.items())
        rows[joint] = values
    return rows


def summarize_inventory(arms):
    return {role: {j: {"id": v["ID"], "model": v["Model_Number"],
                       "raw_position": v["Present_Position"], "torque": v["Torque_Enable"],
                       "voltage": v["Present_Voltage"] / 10, "min_voltage": v["Min_Voltage_Limit"] / 10,
                       "status": v["Status"], "packet_errors": v["packet_errors"],
                       "pid": [v[k] for k in ("P_Coefficient", "I_Coefficient", "D_Coefficient")],
                       "mode": v["Operating_Mode"], "phase": v["Phase"],
                       "calibration_matches": v["calibration_matches"]} for j, v in rows.items()}
            for role, rows in arms.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/so101_adaptive.proposed.json")
    parser.add_argument("--duration", type=float, default=0, help="0: inventory only; otherwise 20 Hz read-only survey")
    parser.add_argument("--cameras", action="store_true", help="Acquire configured RGB cameras during survey")
    args = parser.parse_args()
    if not 0 <= args.duration <= 300 or (args.cameras and not args.duration):
        parser.error("duration must be 0..300; cameras require a survey")
    config = json.loads(args.config.read_text())
    output = ROOT / "outputs/device-checks" / time.strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    cameras = {}
    report = {"mode": "READ ONLY", "motor_writes": [], "config": str(args.config),
              "cameras": args.cameras, "requested_s": args.duration, "target_hz": 20,
              "samples": 0, "communication_errors": 0, "fault_samples": 0, "joints": {}, "pass": None}
    started = None
    try:
        with buses_from_config(config) as buses:
            initial = {role: inventory(bus) for role, bus in buses.items()}
            (output / "inventory.json").write_text(json.dumps(initial, indent=2) + "\n")
            print(json.dumps(summarize_inventory(initial), indent=2), flush=True)
            report["inventory_completed"] = True
            report["torque_on"] = [f"{r}/{j}" for r, rows in initial.items() for j, v in rows.items() if v["Torque_Enable"]]
            report["inventory_issues"] = [f"{r}/{j}" for r, rows in initial.items() for j, v in rows.items()
                                          if v["Status"] or any(v["packet_errors"]) or v["Model_Number"] != 777
                                          or v["ID"] != v["expected_id"]
                                          or not v["Min_Voltage_Limit"] <= v["Present_Voltage"] <= v["Max_Voltage_Limit"]]
            if not args.duration:
                return 0
            report["pass"] = False
            if report["inventory_issues"]:
                raise RuntimeError(f"Initial inventory failed: {report['inventory_issues']}")
            if any(v["Torque_Enable"] for rows in initial.values() for v in rows.values()):
                raise RuntimeError("Torque is ON; survey cancelled without changing torque")
            if args.cameras:
                from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
                for name, raw in config["recording"]["robot"]["cameras"].items():
                    ensure_port_free(raw["index_or_path"])
                    camera = OpenCVCamera(OpenCVCameraConfig(**{k: v for k, v in raw.items() if k != "type"}))
                    cameras[name] = camera
                    camera.connect()
            started = time.monotonic()
            with (output / "samples.jsonl").open("x") as log:
                while time.monotonic() - started < args.duration:
                    tick = time.monotonic()
                    try:
                        try:
                            arms = {role: feedback(bus) for role, bus in buses.items()}
                        except (ConnectionError, RuntimeError):
                            report["communication_errors"] += 1
                            raise
                        for name, camera in cameras.items():
                            frame = camera.read_latest(max_age_ms=500)
                            if report["samples"] == 0:
                                from PIL import Image
                                Image.fromarray(frame).save(output / f"{name}.png")
                        faults = []
                        for role, rows in arms.items():
                            for joint, v in rows.items():
                                key = f"{role}/{joint}"; voltage = v["Present_Voltage"] / 10
                                item = report["joints"].setdefault(key, {"voltage_min": voltage, "voltage_max": voltage})
                                item["voltage_min"] = min(item["voltage_min"], voltage)
                                item["voltage_max"] = max(item["voltage_max"], voltage)
                                limits = initial[role][joint]
                                if v["Status"] or v["packet_error"] or v["Torque_Enable"] or not limits["Min_Voltage_Limit"] <= v["Present_Voltage"] <= limits["Max_Voltage_Limit"]:
                                    faults.append(key)
                        report["fault_samples"] += bool(faults)
                        report["samples"] += 1
                        log.write(json.dumps({"elapsed": tick - started, "arms": arms, "faults": faults}) + "\n")
                        if faults:
                            log.flush()
                            raise RuntimeError(f"Survey failed: {faults}; no motor writes performed")
                    except ConnectionError:
                        raise
                    time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
            report["observed_s"] = time.monotonic() - started
            report["actual_hz"] = report["samples"] / report["observed_s"]
            report["pass"] = not report["fault_samples"] and not report["communication_errors"]
    except Exception as exc:
        report["error"] = str(exc)
        print(str(exc), flush=True)
        return 1
    finally:
        if started is not None:
            report["observed_s"] = time.monotonic() - started
            report["actual_hz"] = report["samples"] / report["observed_s"]
        for camera in cameras.values():
            if camera.is_connected:
                camera.disconnect()
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"Report: {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
