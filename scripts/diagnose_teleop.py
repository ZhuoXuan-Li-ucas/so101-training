#!/usr/bin/env python3
"""Record SO-101 motor feedback; --run enables a short, bounded teleop test."""

import argparse
import json
import os
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ARMS = {
    "leader": ("5B79050214", "teleoperators/so_leader/my_awesome_leader_arm.json"),
    "follower": ("5AB9065574", "robots/so_follower/my_awesome_follower_arm.json"),
}
FEEDBACK_FIELDS = {
    "Torque_Enable": (40, 1),
    "Goal_Position": (42, 2),
    "Torque_Limit": (48, 2),
    "Present_Position": (56, 2),
    "Present_Velocity": (58, 2),
    "Present_Load": (60, 2),
    "Present_Voltage": (62, 1),
    "Present_Temperature": (63, 1),
    "Status": (65, 1),
    "Moving": (66, 1),
    "Present_Current": (69, 2),
}


def ensure_port_free(port):
    target = str(Path(port).resolve(strict=True))
    for fd in Path("/proc").glob("[0-9]*/fd/*"):
        try:
            if os.readlink(fd) == target:
                raise RuntimeError(f"Serial port {target} is in use by PID {fd.parts[2]}; stop teleop first.")
        except OSError:
            pass


def feedback(bus):
    result = {}
    for name, motor in bus.motors.items():
        data, comm, error = bus.packet_handler.readTxRx(bus.port_handler, motor.id, 40, 31)
        if comm != bus._comm_success or len(data) != 31:
            raise RuntimeError(f"{name}: {bus.packet_handler.getTxRxResult(comm)}; received {len(data)} bytes")
        values = {"id": motor.id, "packet_error": error}
        for field, (address, size) in FEEDBACK_FIELDS.items():
            raw = int.from_bytes(bytes(data[address - 40 : address - 40 + size]), "little")
            values[field] = bus._decode_sign(field, {motor.id: raw})[motor.id]
        result[name] = values
    return result


def disable_all(bus):
    """Attempt every motor even when one reports a pre-existing protection error."""
    failures = []
    for name, motor in bus.motors.items():
        try:
            comm, _ = bus._write(40, 1, motor.id, 0, num_retry=2, raise_on_error=False)
            torque, read_comm, _ = bus._read(40, 1, motor.id, num_retry=2, raise_on_error=False)
            if comm != bus._comm_success or read_comm != bus._comm_success or torque != 0:
                failures.append(name)
        except Exception as exc:
            failures.append(f"{name}: {exc}")
    if failures:
        raise RuntimeError(f"Could not verify torque OFF for {failures}; disconnect external power and USB.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Enable a short teleop test that moves the follower")
    parser.add_argument("--duration", type=float, default=15, help="Seconds to record (default: 15; maximum: 30)")
    args = parser.parse_args()
    if not 0 < args.duration <= 30:
        parser.error("--duration must be greater than 0 and at most 30")

    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    buses = {}
    for role, (serial, relative_path) in ARMS.items():
        port = f"/dev/serial/by-id/usb-1a86_USB_Single_Serial_{serial}-if00"
        ensure_port_free(port)
        calibration = json.loads((ROOT / "data/calibration" / relative_path).read_text())
        buses[role] = FeetechMotorsBus(
            port,
            {
                name: Motor(value["id"], "sts3215", MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.DEGREES)
                for name, value in calibration.items()
            },
            {name: MotorCalibration(**value) for name, value in calibration.items()},
        )

    output = ROOT / "logs" / f"teleop-diagnostic-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Log: {output}", flush=True)
    print("Mode: TELEOP (follower will move)" if args.run else "Mode: READ ONLY", flush=True)
    started = time.monotonic()
    exit_code = 0
    with output.open("x") as log:
        def record(event, **data):
            log.write(json.dumps({"event": event, "elapsed": time.monotonic() - started, **data}) + "\n")
            log.flush()

        try:
            for bus in buses.values():
                bus.connect(handshake=False)
                bus.set_baudrate(1_000_000)
                bus.set_timeout(100)
            record("before", arms={role: feedback(bus) for role, bus in buses.items()})
            if args.run:
                # Keep the existing calibration and PID settings. Start at the
                # measured position so enabling torque cannot apply a stale goal.
                for bus in buses.values():
                    disable_all(bus)
                follower = buses["follower"]
                present = follower.sync_read("Present_Position", normalize=False)
                follower.sync_write("Goal_Position", present, normalize=False)
                for role, bus in buses.items():
                    if not bus.is_calibrated:
                        raise RuntimeError(f"{role}: motor calibration does not match the saved file")
                follower.enable_torque()
                record("enabled", arms={role: feedback(bus) for role, bus in buses.items()})

            deadline = time.monotonic() + args.duration
            next_print = 0.0
            while time.monotonic() < deadline:
                tick = time.monotonic()
                arms = {role: feedback(bus) for role, bus in buses.items()}
                record("sample", arms=arms)
                faults = [
                    f"{role}/{name}: status={v['Status']}, packet_error={v['packet_error']}"
                    for role, joints in arms.items()
                    for name, v in joints.items()
                    if v["Status"] or v["packet_error"]
                ]
                if args.run and faults:
                    raise RuntimeError("Motor protection reported: " + "; ".join(faults))
                if args.run:
                    positions = {}
                    for role, bus in buses.items():
                        positions[role] = bus._normalize({bus.motors[name].id: v["Present_Position"] for name, v in arms[role].items()})
                    goals = {}
                    for name, motor in buses["follower"].motors.items():
                        current = positions["follower"][motor.id]
                        requested = positions["leader"][buses["leader"].motors[name].id]
                        # Per update: <= 2 degrees for arm joints, <= 2% for gripper.
                        goals[name] = max(current - 2, min(current + 2, requested))
                    buses["follower"].sync_write("Goal_Position", goals)
                    record("command", goals=goals)
                if tick >= next_print:
                    l = arms["leader"]["gripper"]
                    f = arms["follower"]["gripper"]
                    print(f"gripper: leader={l['Present_Position']} goal={f['Goal_Position']} actual={f['Present_Position']} voltage={f['Present_Voltage']/10:.1f}V torque={f['Torque_Enable']} status={f['Status']} load={f['Present_Load']}", flush=True)
                    next_print = tick + 0.5
                time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        except KeyboardInterrupt:
            record("interrupted")
        except Exception as exc:
            record("error", message=str(exc))
            print(f"Stopped: {exc}", flush=True)
            exit_code = 1
        finally:
            for role, bus in buses.items():
                if not bus.is_connected:
                    continue
                if args.run:
                    try:
                        disable_all(bus)
                        record("torque_disabled", role=role)
                    except Exception as exc:
                        record("cleanup_error", role=role, message=str(exc))
                        print(f"{role}: {exc}", flush=True)
                        exit_code = 1
                bus.disconnect(disable_torque=False)
    print(f"Finished. Log: {output}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
