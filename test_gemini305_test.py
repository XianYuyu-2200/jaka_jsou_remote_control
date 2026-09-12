import unittest

import numpy as np

import gemini305_test as health


class Gemini305HealthTests(unittest.TestCase):
    def test_detects_gemini_305_without_accepting_gemini_215(self):
        usb = """\
Bus 002 Device 002: ID 2bc5:0808 Orbbec Gemini 215
Bus 002 Device 003: ID 2bc5:080a Orbbec Gemini 305
"""
        self.assertEqual(health.find_gemini305_usb_line(usb), "Bus 002 Device 003: ID 2bc5:080a Orbbec Gemini 305")
        self.assertIsNone(health.find_gemini305_usb_line("Orbbec Gemini 215"))

    def test_checks_that_the_gemini_device_itself_uses_usb3(self):
        usb_line = "Bus 002 Device 003: ID 2bc5:0840 Orbbec Gemini 305"
        usb_tree = """\
/:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 10000M
    |__ Port 2: Dev 3, If 0, Class=Video, Driver=usbfs, 5000M
"""
        self.assertTrue(health.gemini305_uses_usb3(usb_line, usb_tree))
        self.assertFalse(health.gemini305_uses_usb3(usb_line, usb_tree.replace("5000M", "480M")))

    def test_requires_rgb_depth_and_camera_info_but_not_imu(self):
        self.assertEqual(
            health.required_camera_topics(),
            {
                "RGB topic": "/camera/color/image_raw",
                "Depth topic": "/camera/depth/image_raw",
                "RGB camera_info": "/camera/color/camera_info",
                "Depth camera_info": "/camera/depth/camera_info",
            },
        )

    def test_summarizes_depth_validity_and_median_distance(self):
        frames = [
            np.array([[0, 100], [105, 110]], dtype=np.uint16),
            np.array([[95, 100], [0, 105]], dtype=np.uint16),
        ]
        summary = health.summarize_depth_frames(frames)
        self.assertAlmostEqual(summary.valid_percent, 75.0)
        self.assertAlmostEqual(summary.median_mm, 102.5)
        self.assertEqual(summary.frames, 2)

    def test_depth_quality_fails_when_target_is_closer_than_four_cm(self):
        frames = [np.full((4, 4), 20, dtype=np.uint16)]
        summary = health.summarize_depth_frames(frames)
        ok, _ = health.evaluate_depth_quality(
            summary,
            min_valid_percent=70.0,
            min_distance_mm=40.0,
            max_distance_mm=1000.0,
        )
        self.assertFalse(ok)

    def test_depth_quality_passes_in_ideal_range(self):
        frames = [np.full((4, 4), 200, dtype=np.uint16)]
        summary = health.summarize_depth_frames(frames)
        ok, detail = health.evaluate_depth_quality(
            summary,
            min_valid_percent=70.0,
            min_distance_mm=40.0,
            max_distance_mm=1000.0,
        )
        self.assertTrue(ok)
        self.assertIn("200.0 mm", detail)


if __name__ == "__main__":
    unittest.main()
