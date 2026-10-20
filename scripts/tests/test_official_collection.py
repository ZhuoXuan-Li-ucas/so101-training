"""No-device tests for native collection feedback and terminal dataset operations."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
import json
import os
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import so101_adaptive as adaptive
import so101_collect as collect
from so101_session import JOINTS, PositionLimit, Session, Settings, check_observed
import test_recovery
from test_so101_session import BOUNDS, IDENTITIES, FakeHardware, FakeWriter, finish_reset, pose, profile


class OfficialFeedbackTests(unittest.TestCase):
    def hardware(self):
        hardware = test_recovery.RawHardwareTests().hardware()
        for arm in hardware.arms.values():
            arm.config = SimpleNamespace(num_read_retries=2)
            arm.bus.sync_read.return_value = dict.fromkeys(JOINTS, 2000)
        return hardware

    def test_manual_read_uses_native_sync_read_without_status_polling_or_writes(self):
        hardware = self.hardware()
        hardware.read_healthy_feedback = Mock(side_effect=AssertionError("extra polling"))
        obs = hardware.observe(detailed=False)
        self.assertIsNone(obs["health"])
        self.assertEqual(obs["feedback_source"], "official_position")
        self.assertEqual(obs["voltage_warnings"], [])
        for role, arm in hardware.arms.items():
            arm.bus.sync_read.assert_called_once_with("Present_Position", normalize=False, num_retry=2)
            arm.bus._normalize.assert_called_once()
            self.assertEqual(obs["raw"][role], dict.fromkeys(JOINTS, 2000))
            arm.bus.write.assert_not_called()
            arm.bus.sync_write.assert_not_called()
        for camera in hardware.follower.cameras.values():
            camera.read_latest.assert_called_once_with(max_age_ms=500)

    def test_native_read_retains_raw_gripper_bounds_despite_normalization_clipping(self):
        hardware = self.hardware()
        hardware.follower.bus.sync_read.return_value["gripper"] = 3017
        obs = hardware.observe(detailed=False)
        self.assertEqual(obs["follower"]["gripper"], 100)
        with self.assertRaises(PositionLimit):
            check_observed(obs, "follower", BOUNDS)

    def test_native_communication_failure_and_stop_propagate_without_targets(self):
        for stop in (False, True):
            with self.subTest(stop=stop):
                hardware = self.hardware()
                hardware.leader.bus.sync_read.side_effect = ConnectionError("disconnected")
                if stop:
                    hardware.stop.set()
                with self.assertRaises(InterruptedError if stop else ConnectionError):
                    hardware.observe(detailed=False)
                hardware.follower.bus.sync_write.assert_not_called()
                hardware.follower.send_action.assert_not_called()

    def test_first_automatic_hold_checks_health_before_any_motor_write(self):
        hardware = self.hardware()
        hardware.last_observation_detailed = False
        hardware.health = Mock(side_effect=RuntimeError("active device fault"))
        hardware._enable_at = Mock()
        with self.assertRaisesRegex(RuntimeError, "active device fault"):
            hardware.hold({"leader": pose(), "follower": pose()})
        hardware._enable_at.assert_not_called()

    def test_automatic_hold_uses_current_pose_after_health_check(self):
        hardware = self.hardware()
        hardware.last_observation_detailed = False
        calls = Mock()
        hardware.health = calls.health
        hardware._enable_at = calls.enable
        observation = {"leader": pose(1), "follower": pose(2)}
        hardware.hold(observation)
        self.assertEqual([call[0] for call in calls.mock_calls], ["health", "enable", "enable"])
        self.assertEqual(calls.enable.call_args_list[0].args, ("follower", observation["follower"]))

    def test_n_save_reset_and_first_review_remain_with_native_manual_feedback(self):
        saved = profile()
        hardware = FakeHardware(saved.home)
        writer = FakeWriter()
        session = Session(hardware, writer, Settings(), IDENTITIES, BOUNDS, "record", saved,
                          episodes=2, require_first_review=True)
        self.assertFalse(adaptive.detailed_feedback_required("official", session))
        session.step("s", hardware.obs, 0)
        self.assertFalse(adaptive.detailed_feedback_required("official", session))
        hardware.obs["leader"] = pose(15, 10)
        hardware.obs["follower"] = session.mapping(hardware.obs["leader"])
        session.step(None, hardware.obs, .1)
        session.step("n", hardware.obs, .2)
        self.assertTrue(adaptive.detailed_feedback_required("official", session))
        now = finish_reset(session, hardware, .2)
        self.assertEqual(session.state, "FIRST_REVIEW")
        self.assertEqual(session.reset_completed_count, 1)
        self.assertEqual(len(writer.episodes[0]), 2)
        session.step("v", hardware.obs, now + .1)
        session.step("s", hardware.obs, now + .2)
        self.assertFalse(adaptive.detailed_feedback_required("official", session))
        self.assertEqual(session.state, "RECORDING")

    def test_full_mode_and_autonomous_states_retain_full_feedback(self):
        for state in ("COUNTDOWN", "RETURNING", "SAVING", "FIRST_REVIEW", "LIMIT_PAUSED", "DONE"):
            self.assertTrue(adaptive.detailed_feedback_required("official", SimpleNamespace(state=state, holding=False)))
        self.assertTrue(adaptive.detailed_feedback_required("official", SimpleNamespace(state="READY", holding=True)))
        self.assertTrue(adaptive.detailed_feedback_required("full", SimpleNamespace(state="RECORDING", holding=False)))

    def test_check_override_uses_new_name_and_never_constructs_hardware(self):
        name = f"offline_check_{os.getpid()}"
        output = StringIO()
        with patch.object(adaptive, "Hardware", side_effect=AssertionError("hardware forbidden")), \
             patch.object(adaptive, "run", side_effect=AssertionError("run forbidden")), redirect_stdout(output):
            collect.main(["record", "--name", name, "--episodes", "2", "--check"])
        result, _ = json.JSONDecoder().raw_decode(output.getvalue())
        self.assertEqual(result["review"]["feedback_mode"], "official")
        self.assertEqual(result["review"]["episodes"], 2)
        self.assertEqual(result["review"]["dataset_root"], str(adaptive.ROOT / "data/datasets" / name))
        self.assertFalse((adaptive.ROOT / "data/datasets" / name).exists())


class DatasetToolsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=adaptive.ROOT / "outputs")
        self.root = Path(self.tmp.name)
        self.patches = [patch.object(adaptive, "ROOT", self.root), patch.object(collect, "ROOT", self.root)]
        for item in self.patches:
            item.start()

    def tearDown(self):
        for item in reversed(self.patches):
            item.stop()
        self.tmp.cleanup()

    def dataset(self, name="demo", episodes=0, frames=0):
        path = self.root / "data/datasets" / name
        (path / "meta").mkdir(parents=True)
        (path / "meta/info.json").write_text(json.dumps({"total_episodes": episodes, "total_frames": frames,
                                                        "codebase_version": "v3.0", "features": {}}))
        return path

    def test_empty_and_missing_dataset_are_distinct(self):
        self.assertEqual(collect.status("demo")["state"], "not_created")
        self.dataset()
        self.assertEqual(collect.status("demo")["state"], "empty")

    def test_partial_save_is_flagged_without_modifying_files(self):
        path = self.dataset(episodes=1, frames=30)
        before = (path / "meta/info.json").read_bytes()
        result = collect.status("demo")
        self.assertEqual(result["state"], "needs_inspection")
        self.assertEqual(result["saved_episodes"], 1)
        self.assertEqual((path / "meta/info.json").read_bytes(), before)

    def test_archive_preserves_bytes_and_frees_old_name(self):
        path = self.dataset()
        before = (path / "meta/info.json").read_bytes()
        with patch.object(collect, "active_collectors", return_value=[]):
            result = collect.archive("demo")
        self.assertFalse(path.exists())
        self.assertFalse(result["deleted"])
        self.assertEqual((Path(result["destination"]) / "meta/info.json").read_bytes(), before)

    def test_archive_rejects_running_collector_and_paths_outside_dataset_root(self):
        self.dataset()
        with patch.object(collect, "active_collectors", return_value=["123"]):
            with self.assertRaisesRegex(RuntimeError, "123"):
                collect.archive("demo")
        for name in ("..", "../calibration", "/tmp/demo", "", "a/b"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                collect.archive(name)
        (self.root / "data/datasets/link").symlink_to(self.root / "data/datasets/demo", target_is_directory=True)
        with self.assertRaisesRegex(ValueError, "symlink"):
            collect.archive("link")


if __name__ == "__main__":
    unittest.main()
