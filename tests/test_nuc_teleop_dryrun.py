import json
import tempfile
import unittest
from pathlib import Path

from nuc_setup.teleop_dryrun import run_dryrun


class NucDryRunTests(unittest.TestCase):
    def test_writes_one_json_record_per_control_step(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "teleop.jsonl"
            records = run_dryrun(steps=10, output_path=output)
            self.assertEqual(len(records), 10)
            lines = output.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 10)
            first = json.loads(lines[0])
            self.assertEqual(first["state"], "RUNNING")
            self.assertEqual(len(first["command"]), 6)
            self.assertTrue(first["command_sent"])

    def test_deadman_release_is_recorded_as_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "teleop.jsonl"
            records = run_dryrun(steps=8, output_path=output, deadman_release_step=3)
            self.assertEqual(records[3]["state"], "FAULT")
            self.assertFalse(records[3]["command_sent"])
            self.assertEqual(records[-1]["state"], "FAULT")


if __name__ == "__main__":
    unittest.main()
