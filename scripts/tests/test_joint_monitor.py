"""Offline checks for diagnostic control and cleanup, without serial access."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import joint_monitor as jm


class Bus:
    def __init__(self, *, fail_enable=False, bad_ack=False):
        self.is_connected = True
        self.is_calibrated = True
        self.fail_enable = fail_enable
        self.bad_ack = bad_ack
        self.enabled = False
        self.writes = []
        self.goal = dict.fromkeys(jm.JOINTS, 0.0)

    def sync_read(self, register, **kwargs):
        if register == 'Goal_Position':
            return {k: v + (1 if self.bad_ack else 0) for k, v in self.goal.items()}
        return dict.fromkeys(jm.JOINTS, 0.0)

    def sync_write(self, register, values, **kwargs):
        self.writes.append((register, values))
        self.goal = dict(values)

    def enable_torque(self):
        self.enabled = True
        if self.fail_enable:
            raise RuntimeError('partial enable failure')

    def disconnect(self, **kwargs):
        self.is_connected = False


def health(bus):
    return {n: {'Status': 0, 'packet_error': 0, 'Torque_Enable': int(bus.enabled)} for n in jm.JOINTS}


class MonitorTests(unittest.TestCase):
    def run_monitor(self, follower, start=False):
        buses = {'leader': Bus(), 'follower': follower}
        def disable(bus):
            bus.enabled = False
        with tempfile.TemporaryDirectory() as directory:
            monitor = jm.Monitor(duration=0.08)
            monitor.log_path = Path(directory) / 'log.jsonl'
            if start:
                monitor.commands.put(('start', 2))
            with patch.object(jm, 'open_buses', return_value=buses), \
                 patch.object(jm, 'read_bounds', return_value=dict.fromkeys(jm.JOINTS, (-100, 100))), \
                 patch.object(jm, 'feedback', side_effect=health), \
                 patch.object(jm, 'disable_all', side_effect=disable):
                monitor.run()
            return monitor, buses

    def test_read_only_never_writes(self):
        monitor, buses = self.run_monitor(Bus())
        self.assertFalse(monitor.failed)
        self.assertTrue(all(not b.writes and not b.enabled for b in buses.values()))

    def test_partial_enable_failure_disables_follower(self):
        monitor, buses = self.run_monitor(Bus(fail_enable=True), start=True)
        self.assertTrue(monitor.failed)
        self.assertFalse(buses['follower'].enabled)
        self.assertTrue(all(not b.is_connected for b in buses.values()))

    def test_mismatched_goal_readback_stops_motion(self):
        monitor, buses = self.run_monitor(Bus(bad_ack=True), start=True)
        self.assertTrue(monitor.failed)
        self.assertFalse(buses['follower'].enabled)

    def test_bounds_and_step_both_hold(self):
        bounds = dict.fromkeys(jm.JOINTS, (-90, 90))
        result = jm.targets(dict.fromkeys(jm.JOINTS, 200), dict.fromkeys(jm.JOINTS, 89), bounds, 2)
        self.assertEqual(set(result.values()), {90})
        result = jm.targets(dict.fromkeys(jm.JOINTS, -200), dict.fromkeys(jm.JOINTS, 89), bounds, 2)
        self.assertEqual(set(result.values()), {87})
        with self.assertRaises(ValueError):
            jm.targets(dict.fromkeys(jm.JOINTS, 0), dict.fromkeys(jm.JOINTS, 95), bounds, 2)


if __name__ == '__main__':
    unittest.main()
