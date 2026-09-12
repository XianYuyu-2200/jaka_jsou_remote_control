import csv
import tempfile
import unittest
from pathlib import Path

import gemini_jaka_test as health


class GeminiJakaHealthTests(unittest.TestCase):
    def test_selects_combined_orbbec_imu_topic(self):
        current = {
            "/camera/accel/imu_info",
            "/camera/gyro/imu_info",
            "/camera/gyro_accel/sample",
        }
        self.assertEqual(health.find_imu_topic(current), "/camera/gyro_accel/sample")

    def test_calculates_camera_rates_from_driver_timestamp_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera_timestamps.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=["color_global_ts_delta_us", "depth_global_ts_delta_us"],
                )
                writer.writeheader()
                for _ in range(10):
                    writer.writerow(
                        {
                            "color_global_ts_delta_us": "33402",
                            "depth_global_ts_delta_us": "33399",
                        }
                    )
            rates = health.rates_from_timestamp_csv(path)
            self.assertAlmostEqual(rates["color"], 29.938, places=3)
            self.assertAlmostEqual(rates["depth"], 29.941, places=3)


if __name__ == "__main__":
    unittest.main()
