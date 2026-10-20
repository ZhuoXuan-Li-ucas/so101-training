#!/usr/bin/env python3
"""Read-only leader voltage survey; no cameras, torque writes or motion goals."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import time

from diagnose_teleop import FEEDBACK_FIELDS, ROOT
from so101_readonly import buses_from_config, inventory, summarize_inventory


def read_block(bus, joint):
    started = time.monotonic()
    data, comm, error = bus.packet_handler.readTxRx(bus.port_handler, bus.motors[joint].id, 40, 31)
    result = {"started": started, "finished": time.monotonic(), "comm": comm,
              "packet_error": error, "raw_hex": bytes(data).hex()}
    if comm != bus._comm_success or len(data) != 31:
        result["error"] = f"{bus.packet_handler.getTxRxResult(comm)}; bytes={len(data)}"
        return result
    result["values"] = {
        name: bus._decode_sign(name, {bus.motors[joint].id: int.from_bytes(bytes(data[address-40:address-40+size]), "little")})[bus.motors[joint].id]
        for name, (address, size) in FEEDBACK_FIELDS.items()
    }
    return result


def read_voltage(bus, joint):
    started = time.monotonic()
    data, comm, error = bus.packet_handler.readTxRx(bus.port_handler, bus.motors[joint].id, 62, 1)
    return {"started": started, "finished": time.monotonic(), "comm": comm, "packet_error": error,
            "raw": data[0] if comm == bus._comm_success and len(data) == 1 else None,
            "raw_hex": bytes(data).hex()}


def forbidden_write(*_args, **_kwargs):
    raise AssertionError("Motor writes are forbidden in the voltage survey")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--hz", type=float, default=30)
    args = parser.parse_args()
    if not 0 < args.duration <= 120 or not 1 <= args.hz <= 30:
        parser.error("duration must be 0..120 seconds; hz must be 1..30")
    config = json.loads(args.config.read_text())
    output = ROOT / "outputs/voltage-surveys" / time.strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    report = {"mode": "READ ONLY", "role": "leader", "port": config["recording"]["teleop"]["port"],
              "motor_writes": [], "cameras": False, "requested_s": args.duration, "target_hz": args.hz,
              "samples": 0, "communication_errors": 0, "device_error_samples": 0,
              "voltage_excursions": [], "joints": {}, "completed": False, "output": str(output)}
    timings = []
    counts = Counter()
    started = None
    print(json.dumps({"output": str(output), "requested_s": args.duration, "hz": args.hz,
                      "port": report["port"], "motor_writes": []}), flush=True)
    try:
        with buses_from_config(config, ("leader",)) as buses:
            bus = buses["leader"]
            for name in ("write", "sync_write", "_write", "_sync_write", "enable_torque", "disable_torque", "write_calibration"):
                setattr(bus, name, forbidden_write)
            initial = inventory(bus)
            (output / "before.json").write_text(json.dumps(initial, indent=2) + "\n")
            print(json.dumps(summarize_inventory({"leader": initial}), ensure_ascii=False), flush=True)
            if any(v["Torque_Enable"] for v in initial.values()):
                raise RuntimeError("Torque is ON; stopped without changing torque")
            if any(v["Model_Number"] != 777 or v["ID"] != v["expected_id"] for v in initial.values()):
                raise RuntimeError("Unexpected motor identity")
            limits = {j: (v["Min_Voltage_Limit"], v["Max_Voltage_Limit"]) for j, v in initial.items()}
            report["voltage_limits_v"] = {j: [lo/10, hi/10] for j, (lo, hi) in limits.items()}
            started = time.monotonic()
            next_tick, next_check, next_progress = started, started, started + 20
            with (output / "samples.jsonl").open("x") as log:
                while time.monotonic() - started < args.duration:
                    tick = time.monotonic()
                    blocks = {j: read_block(bus, j) for j in bus.motors}
                    out_of_range, faults = [], []
                    for joint, block in blocks.items():
                        if "values" not in block:
                            report["communication_errors"] += 1
                            continue
                        values = block["values"]
                        if values["Torque_Enable"]:
                            raise RuntimeError(f"{joint}: torque became ON; no torque writes performed")
                        raw = values["Present_Voltage"]
                        item = report["joints"].setdefault(joint, {"min_v": raw/10, "max_v": raw/10,
                            "outside_samples": 0, "max_consecutive_outside_samples": 0,
                            "position_min": values["Present_Position"], "position_max": values["Present_Position"]})
                        item["min_v"], item["max_v"] = min(item["min_v"], raw/10), max(item["max_v"], raw/10)
                        item["position_min"] = min(item["position_min"], values["Present_Position"])
                        item["position_max"] = max(item["position_max"], values["Present_Position"])
                        if not limits[joint][0] <= raw <= limits[joint][1]:
                            out_of_range.append(joint)
                            item["outside_samples"] += 1
                            counts[joint] += 1
                            item["max_consecutive_outside_samples"] = max(item["max_consecutive_outside_samples"], counts[joint])
                        else:
                            counts[joint] = 0
                        if values["Status"] or block["packet_error"]:
                            faults.append(joint)
                    report["device_error_samples"] += bool(faults)
                    # Independent single-register reads on every excursion and
                    # once per second in normal operation. This is diagnostic
                    # confirmation, never a substituted control observation.
                    check_joints = list(bus.motors) if tick >= next_check else out_of_range
                    crosscheck = {j: read_voltage(bus, j) for j in check_joints}
                    for check in crosscheck.values():
                        if check["raw"] is None:
                            report["communication_errors"] += 1
                    if tick >= next_check:
                        next_check = tick + 1
                    for joint in out_of_range:
                        event = {"elapsed_s": tick-started, "joint": joint,
                                 "block_v": blocks[joint]["values"]["Present_Voltage"]/10,
                                 "status": blocks[joint]["values"]["Status"],
                                 "packet_error": blocks[joint]["packet_error"],
                                 "single_register": crosscheck[joint]}
                        report["voltage_excursions"].append(event)
                        print(json.dumps({"excursion": event}), flush=True)
                    cycle = {"elapsed_s": tick-started, "blocks": blocks, "crosscheck": crosscheck,
                             "outside": out_of_range, "faults": faults}
                    log.write(json.dumps(cycle) + "\n")
                    report["samples"] += 1
                    timings.append(time.monotonic()-tick)
                    if tick >= next_progress:
                        log.flush()
                        print(json.dumps({"elapsed_s": round(tick-started, 1), "samples": report["samples"],
                            "excursions": len(report["voltage_excursions"]),
                            "device_error_samples": report["device_error_samples"],
                            "communication_errors": report["communication_errors"]}), flush=True)
                        next_progress = tick + 20
                    next_tick += 1/args.hz
                    if next_tick < time.monotonic():
                        next_tick = time.monotonic()
                    time.sleep(max(0, min(next_tick, started+args.duration)-time.monotonic()))
            report["observed_s"] = time.monotonic()-started
            report["actual_hz"] = report["samples"]/report["observed_s"]
            report["read_cycle_p95_ms"] = statistics.quantiles(timings, n=100)[94]*1000 if len(timings)>1 else None
            final = inventory(bus)
            (output / "after.json").write_text(json.dumps(final, indent=2) + "\n")
            report["final_all_torque_off"] = all(v["Torque_Enable"] == 0 for v in final.values())
            report["completed"] = True
    except BaseException as exc:
        report["error"] = repr(exc)
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps({k:v for k,v in report.items() if k != "voltage_excursions"}, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
