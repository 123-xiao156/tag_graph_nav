import json
import math
import os
import tempfile
import unittest

from tag_pose_calibrator import (
    atomic_write_route_map,
    robust_pose_estimate,
    update_route_map,
)


class TagPoseCalibrationTest(unittest.TestCase):
    def test_robust_pose_estimate_rejects_outlier_and_wraps_yaw(self):
        samples = [
            (1.00, 2.00, math.pi - 0.02),
            (1.01, 1.99, -math.pi + 0.01),
            (0.99, 2.01, math.pi - 0.01),
            (8.00, -4.00, 0.0),
        ]
        estimate = robust_pose_estimate(samples)
        self.assertEqual(estimate["sample_count"], 4)
        self.assertEqual(estimate["inlier_count"], 3)
        self.assertAlmostEqual(estimate["pose"]["x"], 1.0, places=6)
        self.assertAlmostEqual(estimate["pose"]["y"], 2.0, places=6)
        self.assertLess(
            abs(abs(estimate["pose"]["yaw"]) - math.pi), 0.02)

    def test_update_route_map_preserves_edges_and_other_fields(self):
        template = {
            "frame_id": "map",
            "nodes": [
                {"id": 0, "tag_pose": {"x": 0, "y": 0, "yaw": 0},
                 "custom": "keep"},
                {"id": 1, "tag_pose": {"x": 0, "y": 1, "yaw": 0}},
            ],
            "edges": [{"source": 0, "destination": 1, "cost": 1.0}],
        }
        estimates = {
            0: {
                "pose": {"x": 2.0, "y": 3.0, "yaw": 0.2},
                "sample_count": 30, "inlier_count": 29,
                "xy_rms": 0.01, "yaw_rms": 0.02,
            }
        }
        result = update_route_map(template, estimates, "2026-01-01T00:00:00Z")
        self.assertEqual(result["nodes"][0]["tag_pose"],
                         {"x": 2.0, "y": 3.0, "yaw": 0.2})
        self.assertEqual(result["nodes"][0]["custom"], "keep")
        self.assertEqual(result["nodes"][1], template["nodes"][1])
        self.assertEqual(result["edges"], template["edges"])
        self.assertNotIn("calibration", template)

    def test_update_route_map_adds_localization_only_tag(self):
        template = {
            "frame_id": "map",
            "nodes": [{"id": 4, "tag_pose": {"x": 0, "y": 0, "yaw": 0}}],
            "edges": [],
        }
        estimates = {
            5: {
                "pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
                "sample_count": 30, "inlier_count": 30,
                "xy_rms": 0.01, "yaw_rms": 0.02,
            }
        }
        result = update_route_map(template, estimates, "now", {5})
        self.assertEqual(result["nodes"][-1], {
            "id": 5,
            "tag_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            "targetable": False,
        })

    def test_atomic_write_creates_backup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "tag_route_map.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump({"old": True}, stream)
            backup = atomic_write_route_map({"new": True}, path)
            self.assertTrue(os.path.exists(backup))
            with open(backup, "r", encoding="utf-8") as stream:
                self.assertEqual(json.load(stream), {"old": True})
            with open(path, "r", encoding="utf-8") as stream:
                self.assertEqual(json.load(stream), {"new": True})


if __name__ == "__main__":
    unittest.main()
