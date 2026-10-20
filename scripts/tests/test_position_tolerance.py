"""Offline calibration-edge coverage using fake arms/writer and read-only calibration files."""
from concurrent.futures import Future
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import so101_session as session_code


class BoundaryToleranceTests(unittest.TestCase):
    def setUp(self):
        config = json.loads((ROOT / "configs/so101_collect.json").read_text())["recording"]
        self.cfg = SimpleNamespace(robot=SimpleNamespace(**config["robot"]),
                                   teleop=SimpleNamespace(**config["teleop"]))
        self.identities, self.bounds = session_code.calibration_context(self.cfg)
        self.calibration = {}
        for role, arm in (("leader", self.cfg.teleop), ("follower", self.cfg.robot)):
            self.calibration[role] = json.loads((Path(arm.calibration_dir) / f"{arm.id}.json").read_text())
        self.obs = {"images": {}, "raw": {}, "raw_bounds": {}}
        for role, cal in self.calibration.items():
            self.obs[role] = {j: 50.0 if j == "gripper" else 0.0 for j in session_code.JOINTS}
            self.obs["raw"][role] = {j: (c["range_min"] + c["range_max"]) / 2 for j, c in cal.items()}
            self.obs["raw_bounds"][role] = {j: (c["range_min"], c["range_max"]) for j, c in cal.items()}

    def set_raw(self, role, joint, value):
        cal = self.calibration[role][joint]
        low, high = cal["range_min"], cal["range_max"]
        self.obs["raw"][role][joint] = value
        self.obs[role][joint] = (min(100, max(0, (value-low)*100/(high-low))) if joint == "gripper" else
                                 (value - (low+high)/2) * 360/4095)

    def test_reported_elbow_and_shoulder_cases_pass_without_changing_samples(self):
        for joint, values in (("elbow_flex", (3076, 3078, 3079, 3080)), ("shoulder_lift", (855,))):
            for value in values:
                with self.subTest(joint=joint, value=value):
                    self.set_raw("follower", joint, value)
                    before = deepcopy(self.obs)
                    session_code.check_observed(self.obs, "follower", self.bounds)
                    session_code.check_pose(self.obs["follower"], self.bounds["follower"])
                    self.assertEqual(self.obs, before)

    def test_every_joint_accepts_16_counts_but_rejects_17(self):
        for role, cal in self.calibration.items():
            for joint, row in cal.items():
                for edge, sign in ((row["range_min"], -1), (row["range_max"], 1)):
                    with self.subTest(role=role, joint=joint, sign=sign):
                        low, high = session_code.allowed_raw_range(row["range_min"], row["range_max"])
                        self.assertEqual((low, high), (max(0, row["range_min"]-16), min(4095, row["range_max"]+16)))
                        if joint != "gripper":
                            mid = (row["range_min"] + row["range_max"]) / 2
                            self.assertEqual(self.bounds[role][joint], ((low-mid)*360/4095, (high-mid)*360/4095))
                        self.set_raw(role, joint, low if sign == -1 else high)
                        session_code.check_observed(self.obs, role, self.bounds)
                        session_code.check_pose(self.obs[role], self.bounds[role])
                        self.set_raw(role, joint, (low-1) if sign == -1 else (high+1))
                        with self.assertRaises(session_code.PositionLimit):
                            session_code.check_observed(self.obs, role, self.bounds)
                        self.set_raw(role, joint, (row["range_min"] + row["range_max"]) / 2)

    def test_representation_and_nonfinite_checks_remain(self):
        self.assertEqual(session_code.allowed_raw_range(0, 4095), (0, 4095))
        self.assertEqual(session_code.allowed_raw_range(5, 4090), (0, 4095))
        for value in (float("nan"), float("inf"), -1, 4096):
            with self.subTest(value=value):
                self.set_raw("follower", "elbow_flex", value)
                with self.assertRaises(ValueError):
                    session_code.check_observed(self.obs, "follower", self.bounds)

    def test_body_target_checks_use_same_boundary_and_gripper_stays_0_100(self):
        for role in session_code.ROLES:
            for joint, (low, high) in self.bounds[role].items():
                for value in (low, high):
                    current = dict(self.obs[role], **{joint: value})
                    session_code.check_pose(current, self.bounds[role])
                for value in (low-0.01, high+0.01):
                    with self.assertRaises(ValueError):
                        session_code.check_pose(dict(self.obs[role], **{joint: value}), self.bounds[role])
            self.assertEqual(self.bounds[role]["gripper"], (0, 100))

    def test_record_and_save_reset_accept_real_edge_reading(self):
        home = {r: dict(self.obs[r]) for r in session_code.ROLES}
        profile = session_code.HomeProfile({"version": 2, "kind": "paired_home", "identities": self.identities,
                                        "wrist_offset_deg": 0, "home": home},
                                       self.identities, self.bounds, session_code.Settings())
        hardware = Mock()
        hardware.send.side_effect = lambda role, target: dict(target)
        writer = Mock()
        saved = Future()
        saved.set_result(None)
        writer.save.return_value = saved
        session = session_code.Session(hardware, writer, session_code.Settings(), self.identities,
                                    self.bounds, "record", profile)
        session.step("s", self.obs, 0)
        self.set_raw("follower", "elbow_flex", 3080)
        measured = self.obs["follower"]["elbow_flex"]
        session.step(None, self.obs, .1)
        self.assertEqual(session.state, "RECORDING")
        self.assertEqual(writer.add.call_args.args[0]["follower"]["elbow_flex"], measured)
        session.step("n", self.obs, .2)
        self.assertEqual(session.state, "COUNTDOWN")
        self.assertEqual(session.completed, 1)
        self.assertEqual(session.reset_plan.origin["follower"]["elbow_flex"], measured)
        for elapsed in (0, session.reset_plan.duration/2, session.reset_plan.duration):
            for role, target in session.reset_plan.at(elapsed).items():
                session_code.check_pose(target, self.bounds[role])
        writer.discard.assert_not_called()

    def test_saved_home_and_calibration_identity_remain_compatible(self):
        home = json.loads((ROOT / "data/motion_profiles/stack_white_on_red_home_v1.json").read_text())
        profile = session_code.HomeProfile(home, self.identities, self.bounds, session_code.Settings())
        self.assertEqual(profile.data["identities"], home["identities"])
        self.assertEqual(profile.mapping.offset, home["wrist_offset_deg"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
