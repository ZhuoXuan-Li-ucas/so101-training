"""Reuse a validated wrist mapping without importing or overwriting old home."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_adaptive import load_wrist_reference
from so101_session import Session, Settings
from test_so101_session import BOUNDS, IDENTITIES, FakeHardware, pose, profile


class ReuseWristTests(unittest.TestCase):
    def session(self, path, offset=30):
        hardware = FakeHardware({"leader": pose(20, 40), "follower": pose(20, 40+offset)})
        session = Session(hardware, None, Settings(), IDENTITIES, deepcopy(BOUNDS), "setup",
                          profile_path=path, initial_wrist_offset=offset)
        return session, hardware

    def test_h_captures_current_pose_without_motion_and_r_uses_new_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/"new.json"
            session, hw = self.session(output)
            self.assertEqual(session.state, "ALIGN")
            session.step("c", hw.obs, 0)
            self.assertEqual(session.mapping.offset, 30)
            session.step("r", hw.obs, .1)
            self.assertIsNone(session.reset_plan)
            self.assertEqual(hw.calls, [])
            selected = {r:dict(hw.obs[r]) for r in ("leader", "follower")}
            session.step("h", hw.obs, .2)
            self.assertEqual(hw.calls, [])
            saved = json.loads(output.read_text())
            self.assertEqual(saved["home"], selected)
            self.assertEqual(saved["wrist_offset_deg"], 30)
            hw.obs["leader"] = pose(-30, -50)
            hw.obs["follower"] = pose(10, 100)
            session.step("r", hw.obs, .3)
            self.assertEqual(session.reset_plan.home, selected)
            self.assertEqual(session.state, "COUNTDOWN")

    def test_zero_offset_is_preserved_and_existing_file_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"old.json"
            path.write_text("original")
            session, hw = self.session(path, 0)
            self.assertEqual(session.state, "ALIGN")
            self.assertTrue(session.mapping_locked)
            with self.assertRaises(FileExistsError):
                session.step("h", hw.obs, 0)
            self.assertEqual(path.read_text(), "original")
            self.assertEqual(hw.calls, [])

    def test_source_fingerprint_checked_and_only_offset_returned(self):
        with tempfile.TemporaryDirectory() as tmp, patch("so101_adaptive.ROOT", Path(tmp)):
            source = Path(tmp)/"data/motion_profiles/source.json"
            source.parent.mkdir(parents=True)
            content = json.dumps(profile().data)
            source.write_text(content)
            result = load_wrist_reference(source, IDENTITIES, deepcopy(BOUNDS), Settings())
            self.assertEqual(result["offset_deg"], 30)
            self.assertNotIn("home", result)
            with self.assertRaisesRegex(ValueError, "identity/calibration mismatch"):
                load_wrist_reference(source, {**IDENTITIES, "leader":"different"}, deepcopy(BOUNDS), Settings())
            self.assertEqual(source.read_text(), content)
