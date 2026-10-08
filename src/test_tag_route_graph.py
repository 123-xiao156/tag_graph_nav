import json
import math
import os
import tempfile
import unittest

from tag_route_graph import TagRouteGraph


class TagRouteGraphTest(unittest.TestCase):
    def setUp(self):
        data = {
            "frame_id": "map",
            "nodes": [
                {"id": i, "tag_pose": {"x": 0.0, "y": i * 0.5,
                                         "yaw": 0.0},
                 "targetable": i < 5}
                for i in range(7)
            ],
            "edges": [
                {"source": i, "destination": i + 1, "cost": 1.0,
                 "speed_limit": 0.25, "motion": "forward"}
                for i in range(4)
            ] + [
                {"source": i + 1, "destination": i, "cost": 1.0,
                 "speed_limit": 0.20, "motion": "reverse"}
                for i in range(4)
            ],
        }
        stream = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False)
        json.dump(data, stream)
        stream.close()
        self.addCleanup(os.unlink, stream.name)
        self.graph = TagRouteGraph(stream.name)

    def test_forward_and_reverse_route(self):
        self.assertEqual(self.graph.shortest_path(0, 3)[1], [(0, 1), (1, 2), (2, 3)])
        self.assertEqual(self.graph.shortest_path(3, 0)[1], [(3, 2), (2, 1), (1, 0)])
        self.assertIsNone(self.graph.shortest_path(0, 6)[1])
        with self.assertRaisesRegex(ValueError, "localization-only"):
            self.graph.plan_from_pose((0.0, 2.5, 1.57), 6, 0.10, 0.20)
        self.assertEqual(self.graph.tag_target_pose(6),
                         (0.0, 3.0, 1.5707963267948966))

    def test_mid_edge_start_uses_graph_direction(self):
        self.assertEqual(self.graph.plan_from_pose((0.0, 0.75, 1.57), 4, 0.10, 0.20),
                         [(1, 2), (2, 3), (3, 4)])
        self.assertEqual(self.graph.plan_from_pose((0.0, 0.75, 1.57), 0, 0.10, 0.20),
                         [(2, 1), (1, 0)])

    def test_route_line_uses_all_nodes_and_keeps_direction(self):
        forward = [(0, 1), (1, 2), (2, 3), (3, 4)]
        reverse = [(4, 3), (3, 2), (2, 1), (1, 0)]
        self.assertAlmostEqual(self.graph.route_line_yaw(forward), math.pi / 2)
        self.assertAlmostEqual(abs(self.graph.route_line_yaw(reverse)), math.pi / 2)

    def test_start_behind_tag_zero_goes_to_tag_zero_first(self):
        pose = (0.0, -0.80, 1.57)
        self.assertEqual(self.graph.plan_from_pose(pose, 3, 0.10, 0.20),
                         [(0, 1), (1, 2), (2, 3)])
        waypoints = self.graph.remaining_waypoints(pose, (0, 1))
        self.assertEqual([point[:2] for point in waypoints],
                         [(0.0, 0.0), (0.0, 0.5)])
        for point in waypoints:
            self.assertAlmostEqual(point[2], 1.5707963268)
        self.assertEqual(self.graph.plan_from_pose(pose, 0, 0.10, 0.20), [])

    def test_start_extension_still_obeys_lateral_tolerance(self):
        self.assertIsNone(self.graph.plan_from_pose(
            (0.25, -0.80, 1.57), 3, 0.10, 0.20))

    def test_off_route_and_one_way_unreachable(self):
        self.assertIsNone(self.graph.plan_from_pose((3.0, 3.0, 0.0), 4, 0.10, 0.20))
        data = {
            "frame_id": "map",
            "nodes": [
                {"id": i,
                 "tag_pose": {"x": float(i), "y": 0.0, "yaw": 0.0}}
                for i in range(3)
            ],
            "edges": [
                {"source": 0, "destination": 1, "cost": 1.0, "speed_limit": 0.2},
                {"source": 1, "destination": 2, "cost": 1.0, "speed_limit": 0.2},
            ],
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as stream:
            json.dump(data, stream)
            stream.flush()
            one_way = TagRouteGraph(stream.name)
        self.assertIsNone(one_way.shortest_path(2, 0)[1])
        self.assertIsNone(one_way.plan_from_pose((2.0, 0.0, 0.0), 0, 0.10, 0.20))

    def test_curved_edge_uses_remaining_waypoints(self):
        data = {
            "frame_id": "map",
            "nodes": [
                {"id": 1, "tag_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
                {"id": 2, "tag_pose": {"x": 1.0, "y": 1.0, "yaw": 0.0}},
            ],
            "edges": [{"source": 1, "destination": 2, "cost": 2.0, "speed_limit": 0.2,
                       "waypoints": [{"x": 0.0, "y": 1.0, "yaw": 0.0}]}],
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json") as stream:
            json.dump(data, stream)
            stream.flush()
            curved = TagRouteGraph(stream.name)
        self.assertEqual(curved.plan_from_pose((0.0, 0.7, 0.0), 2, 0.1, 0.2), [(1, 2)])
        self.assertEqual(curved.remaining_waypoints((0.0, 0.7, 0.0), (1, 2)),
                         [(0.0, 1.0, 0.0), (1.0, 1.0, 1.5707963267948966)])
        self.assertEqual(curved.remaining_waypoints((0.5, 1.0, 0.0), (1, 2)),
                         [(1.0, 1.0, 1.5707963267948966)])


if __name__ == "__main__":
    unittest.main()
