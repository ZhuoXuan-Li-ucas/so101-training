"""Bounded confirmation of brief undervoltage on a torque-free leader only.

No motor writes. An accepted observation is always the final fresh full sample;
failed samples and confirmation times remain available as diagnostic events.
"""
from copy import deepcopy
import math
import time

CONFIRM_SECONDS = 0.050
RECHECK_SECONDS = 0.002
NORMAL_SAMPLES = 2
VOLTAGE_ERROR_BIT = 1


class MotorHealthError(RuntimeError):
    """Device-health failure; lets the controller stop and pause instead of exit."""


def health_issues(values, limits, expected_torque):
    hard, voltage = {}, {}
    for joint, row in values.items():
        low, high = limits[joint]
        measured = row["Present_Voltage"]
        if (not math.isfinite(measured)
                or ("Present_Position" in row and not math.isfinite(row["Present_Position"]))
                or row["Status"] & ~VOLTAGE_ERROR_BIT
                or row["packet_error"] & ~VOLTAGE_ERROR_BIT
                or bool(row["Torque_Enable"]) != expected_torque
                or measured > high):
            hard[joint] = row
        elif measured < low or row["Status"] or row["packet_error"]:
            voltage[joint] = row
    return hard, voltage


class LeaderVoltageConfirmation:
    def __init__(self, check_stop, *, clock=time.monotonic, wait=time.sleep):
        self.check_stop, self.clock, self.wait = check_stop, clock, wait
        self.events = []

    def read(self, role, reader, limits, expected_torque):
        self.check_stop()
        started = self.clock()
        values = reader()
        self.check_stop()
        hard, voltage = health_issues(values, limits, expected_torque)
        if not hard and not voltage:
            return values, started
        eligible = (not hard and role == "leader" and not expected_torque
                    and any(row["Present_Voltage"] < limits[j][0] for j, row in voltage.items()))
        if not eligible:
            raise MotorHealthError(f"{role}: motor fault/voltage/torque mismatch: {hard or voltage}")

        detected = self.clock()
        deadline = detected + CONFIRM_SECONDS
        event = {"event": "leader_voltage_confirmation", "role": role, "detected": detected,
                 "budget_s": CONFIRM_SECONDS, "required_normal_samples": NORMAL_SAMPLES,
                 "initial": deepcopy(values), "initial_sample_time": started,
                 "limits_raw": deepcopy(limits), "checks": [], "outcome": "failed"}
        normal = 0
        try:
            while self.clock() < deadline:
                self.check_stop()
                self.wait(min(RECHECK_SECONDS, max(0, deadline-self.clock())))
                self.check_stop()
                sampled = self.clock()
                if sampled >= deadline:
                    break
                current = reader()
                self.check_stop()
                finished = self.clock()
                event["checks"].append({"sample_time": sampled, "finished": finished,
                                        "values": deepcopy(current)})
                hard, voltage = health_issues(current, limits, expected_torque)
                if hard:
                    raise MotorHealthError(f"{role}: fault during voltage confirmation: {hard}")
                if finished >= deadline:
                    break
                normal = 0 if voltage else normal + 1
                if normal >= NORMAL_SAMPLES:
                    event.update(outcome="recovered", recovered=finished,
                                 confirmation_ms=(finished-detected)*1000)
                    return current, sampled
            raise MotorHealthError(f"{role}: undervoltage did not clear with {NORMAL_SAMPLES} normal samples "
                               f"within {CONFIRM_SECONDS*1000:.0f} ms; stopping")
        except BaseException as exc:
            event["error"] = repr(exc)
            raise
        finally:
            event["finished"] = self.clock()
            self.events.append(event)
