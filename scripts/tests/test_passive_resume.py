"""Manual repeatability is optional; recovery must never write a motor register."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_session import JOINTS
from so101_passive_calibrate import PassiveCalibration, check_empty_output, main, resume_calibration, resume_source, run_calibration
import test_recovery as existing


class PassiveResumeTests(unittest.TestCase):
    def test_empty_retry_directory_is_allowed_but_any_existing_evidence_is_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "output"
            check_empty_output(output)
            output.mkdir()
            check_empty_output(output)
            evidence = output / "before.json"; evidence.write_text("preserve")
            with self.assertRaises(FileExistsError): check_empty_output(output)
            self.assertEqual(evidence.read_text(), "preserve")

    def test_missing_port_never_opens_bus_or_creates_output_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); output = root / "data/calibration/follower"
            config = root / "config.json"
            config.write_text(json.dumps({"recording": {"robot": {"port": str(root / "missing-serial"), "id": "follower"}}}))
            args = ["calibrate", "--config", str(config), "--arm", "follower", "--output", str(output), "--run"]
            with patch("so101_passive_calibrate.ROOT", root), patch.object(sys, "argv", args), \
                 patch.object(sys.stdin, "isatty", return_value=True), \
                 patch("so101_passive_calibrate.buses_from_config") as connect, \
                 redirect_stdout(StringIO()), redirect_stderr(StringIO()) as stderr:
                with self.assertRaises(SystemExit) as exc: main()
            self.assertEqual(exc.exception.code, 2)
            self.assertIn("串口不存在", stderr.getvalue())
            connect.assert_not_called()
            self.assertFalse(output.exists())

    def source(self, root):
        bus, before, _ = existing.PassiveCalibrationTests().fixture()
        expected = deepcopy(before)
        for j in JOINTS:
            expected[j].update(Homing_Offset=10, Min_Position_Limit=1000, Max_Position_Limit=3000)
        output = root / "source"; output.mkdir()
        error = 'ValueError("Repeat middle pose exceeds 5 degrees/points: elbow_flex=17")'
        status = dict(state="incomplete", role="leader", id="same_id", output=str(output),
                      calibration_file=str(output / "same_id.json"), error=error)
        events = [dict(event="center", raw=dict.fromkeys(JOINTS, 2047)),
                  dict(event="registers_verified", registers=expected),
                  dict(event="repeat_center", trial=1), dict(event="incomplete", error=error)]
        (output / "before.json").write_text(json.dumps(before))
        (output / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
        (output / "status.json").write_text(json.dumps(status))
        status_path = root / "status.json"; status_path.write_text(json.dumps(status))
        bus.write = Mock(side_effect=AssertionError("No register writes during resume"))
        return bus, output, status_path, expected

    def test_resume_far_from_middle_saves_only_after_direction_and_fresh_readback(self):
        with tempfile.TemporaryDirectory() as tmp:
            bus, output, status_path, expected = self.source(Path(tmp))
            original = (output / "events.jsonl").read_bytes()
            samples = [dict.fromkeys(JOINTS, 2500), dict.fromkeys(JOINTS, 2520)]
            prompts = Mock(side_effect=["", "", "yes"])
            with patch("so101_passive_calibrate.inventory", return_value=expected) as read, \
                 patch.object(PassiveCalibration, "sample", side_effect=samples), \
                 patch.object(PassiveCalibration, "release", side_effect=AssertionError("no release")), \
                 patch.object(PassiveCalibration, "center", side_effect=AssertionError("no centering")), \
                 patch.object(PassiveCalibration, "finish", side_effect=AssertionError("no calibration writes")):
                result = resume_calibration(bus, {"id": "same_id"}, "leader", output, status_path, prompt=prompts, direction_check=True)
            self.assertEqual(read.call_count, 2)
            bus.write.assert_not_called()
            self.assertEqual(result["state"], "complete")
            self.assertEqual(result["validation"]["manual_repeat"], "not_required")
            self.assertTrue(result["resumed_without_motor_writes"])
            self.assertEqual(json.loads((output / "same_id.json").read_text())["elbow_flex"]["homing_offset"], 10)
            self.assertEqual((output / "events.jsonl").read_bytes(), original)
            self.assertEqual(len(list(output.glob("status-before-resume-*.json"))), 1)
            with self.assertRaises((ValueError, FileExistsError)):
                resume_source(output, {"id": "same_id"}, "leader")

    def test_changed_register_torque_or_fault_cannot_be_accepted(self):
        for register, value in (("Homing_Offset", 11), ("Torque_Enable", 1), ("P_Coefficient", 99),
                                ("Phase", 28), ("Status", 1), ("Present_Voltage", 37)):
            with self.subTest(register=register), tempfile.TemporaryDirectory() as tmp:
                bus, output, status_path, expected = self.source(Path(tmp))
                current = deepcopy(expected); current["gripper"][register] = value
                prompt = Mock()
                with patch("so101_passive_calibrate.inventory", return_value=current):
                    with self.assertRaises((ValueError, RuntimeError)):
                        resume_calibration(bus, {"id": "same_id"}, "leader", output, status_path, prompt=prompt, direction_check=True)
                prompt.assert_not_called(); bus.write.assert_not_called()
                self.assertFalse((output / "same_id.json").exists())
                self.assertEqual(json.loads(status_path.read_text())["state"], "incomplete")

    def test_final_readback_mismatch_and_unconfirmed_direction_do_not_publish(self):
        for failure in ("readback", "direction"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                bus, output, status_path, expected = self.source(Path(tmp))
                changed = deepcopy(expected); changed["elbow_flex"]["Min_Position_Limit"] = 1001
                prompt = Mock(side_effect=["", "", "yes" if failure == "readback" else "no"])
                with patch("so101_passive_calibrate.inventory", side_effect=[expected, changed]), \
                     patch.object(PassiveCalibration, "sample", side_effect=[dict.fromkeys(JOINTS, 2500), dict.fromkeys(JOINTS, 2520)]):
                    with self.assertRaises(ValueError):
                        resume_calibration(bus, {"id": "same_id"}, "leader", output, status_path, prompt=prompt, direction_check=True)
                bus.write.assert_not_called()
                self.assertFalse((output / "same_id.json").exists())
                self.assertEqual(json.loads(status_path.read_text())["state"], "incomplete")

    def test_arbitrary_failure_or_postverification_write_cannot_resume(self):
        for failure in ("different_error", "missing_verification", "later_write", "wrong_role"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                _, output, _, _ = self.source(Path(tmp))
                p = output / "events.jsonl"
                events = [json.loads(line) for line in p.read_text().splitlines()]
                if failure == "different_error": events[-1]["error"] = "OSError('wire lost')"
                if failure == "missing_verification": events.pop(1)
                if failure == "later_write": events.insert(-1, {"event": "write_attempt"})
                p.write_text("".join(json.dumps(e) + "\n" for e in events))
                with self.assertRaises(ValueError):
                    resume_source(output, {"id": "same_id"}, "follower" if failure == "wrong_role" else "leader")

    def test_newer_session_marker_blocks_stale_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            bus, output, status_path, _ = self.source(Path(tmp))
            status_path.write_text(json.dumps({"state": "incomplete", "output": "another_session"}))
            with self.assertRaisesRegex(ValueError, "stale resume"):
                resume_calibration(bus, {"id": "same_id"}, "leader", output, status_path)
            bus.write.assert_not_called()

    def test_new_calibration_default_only_asks_middle_and_range(self):
        with tempfile.TemporaryDirectory() as tmp:
            bus, output, status_path, expected = self.source(Path(tmp))
            # Use a separate fresh output for the normal calibration path.
            output = Path(tmp) / "new"; output.mkdir()
            result = {j: dict(id=v["ID"], drive_mode=0, homing_offset=10, range_min=1000, range_max=3000)
                      for j, v in expected.items()}
            prompt = Mock(side_effect=[""])
            with patch("so101_passive_calibrate.inventory", return_value=expected), \
                 patch.object(PassiveCalibration, "release"), \
                 patch.object(PassiveCalibration, "center", return_value=(dict.fromkeys(JOINTS, 10), dict.fromkeys(JOINTS, 2047))), \
                 patch.object(PassiveCalibration, "capture_ranges", return_value=(dict.fromkeys(JOINTS, 1000), dict.fromkeys(JOINTS, 3000))), \
                 patch.object(PassiveCalibration, "finish", return_value=result), \
                 patch.object(PassiveCalibration, "sample", side_effect=AssertionError("No extra direction/repeat samples")):
                status = run_calibration(bus, {"id": "same_id"}, "leader", output, status_path, prompt=prompt)
            self.assertEqual(status["state"], "complete")
            self.assertEqual(status["validation"]["manual_repeat"], "not_required")
            self.assertEqual(status["validation"]["direction"], "deferred_to_follow_test")
            self.assertEqual(prompt.call_count, 1)  # capture_ranges has its own Enter prompt, mocked above.

    def test_default_resume_needs_no_extra_manual_step(self):
        with tempfile.TemporaryDirectory() as tmp:
            bus, output, status_path, expected = self.source(Path(tmp))
            prompt = Mock(side_effect=AssertionError("No manual steps expected"))
            with patch("so101_passive_calibrate.inventory", return_value=expected), \
                 patch.object(PassiveCalibration, "sample", side_effect=AssertionError("No extra samples")):
                result = resume_calibration(bus, {"id": "same_id"}, "leader", output, status_path, prompt=prompt)
            prompt.assert_not_called(); bus.write.assert_not_called()
            self.assertEqual(result["state"], "complete")
            self.assertEqual(result["validation"]["direction"], "deferred_to_follow_test")

    def test_final_hardware_fault_still_rejects_simplified_calibration(self):
        for register, value in (("Status", 1), ("Present_Voltage", 37), ("Torque_Enable", 1)):
            with self.subTest(register=register):
                bus, before, state = existing.PassiveCalibrationTests().fixture()
                state["gripper"][register] = value
                cal = PassiveCalibration(bus, before, Mock())
                with patch("so101_passive_calibrate.inventory", return_value=state):
                    with self.assertRaises(RuntimeError):
                        cal.finish(dict.fromkeys(JOINTS, 10), dict.fromkeys(JOINTS, 1000), dict.fromkeys(JOINTS, 3000))


if __name__ == "__main__":
    unittest.main()
