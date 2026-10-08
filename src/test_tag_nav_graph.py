import math
import unittest
from unittest import mock

import rospy
import tf

from tag_nav_graph import TagNavigator


class TagControlTest(unittest.TestCase):
    def setUp(self):
        self.navigator = TagNavigator.__new__(TagNavigator)
        self.navigator.control_rate = 30
        self.navigator.navigation_timeout = 60.0
        self.navigator.max_lost_count = 0
        self.navigator.dis_err = 0.1
        self.navigator.ang_err = 0.1
        self.navigator.heading_kp = 2.0
        self.navigator.lateral_kp = 1.0
        self.navigator.heading_hold = 0.35
        self.navigator.linear_kp = 0.7
        self.navigator.min_linear = 0.05
        self.navigator.max_linear = 0.35
        self.navigator.max_angular = 0.3
        self.navigator.navigating = True
        self.navigator.cmd_vel = mock.Mock()
        self.started_at = rospy.Time(1.0)
        self.patches = [
            mock.patch("tag_nav_graph.rospy.is_shutdown", return_value=False),
            mock.patch("tag_nav_graph.rospy.Rate"),
            mock.patch("tag_nav_graph.rospy.Time.now", return_value=rospy.Time(2.0)),
            mock.patch("tag_nav_graph.rospy.loginfo_throttle"),
            mock.patch("tag_nav_graph.rospy.logerr"),
        ]
        for patcher in self.patches:
            patcher.start()
            self.addCleanup(patcher.stop)

    def test_forward_and_reverse_commands(self):
        for start_y, goal_y, motion, expected_sign in (
                (0.0, 1.0, "forward", 1), (1.0, 0.0, "reverse", -1)):
            with self.subTest(motion=motion):
                self.navigator.cmd_vel.reset_mock()
                self.navigator.get_robot_pose = mock.Mock(side_effect=[
                    (0.0, start_y, math.pi / 2),
                    (0.0, goal_y, math.pi / 2),
                ])
                edge = {"motion": motion, "speed_limit": 0.20}
                path_yaw = math.pi / 2 if motion == "forward" else -math.pi / 2
                self.assertTrue(self.navigator.follow_line(
                    (0.0, goal_y, math.pi / 2), path_yaw, edge,
                    self.started_at))
                first_cmd = self.navigator.cmd_vel.publish.call_args_list[0].args[0]
                self.assertGreater(first_cmd.linear.x * expected_sign, 0.0)
                self.assertLessEqual(abs(first_cmd.linear.x), 0.20)
                last_cmd = self.navigator.cmd_vel.publish.call_args_list[-1].args[0]
                self.assertEqual(last_cmd.linear.x, 0.0)
                self.assertEqual(last_cmd.angular.z, 0.0)

    def test_lost_tag_stops_immediately(self):
        self.navigator.get_robot_pose = mock.Mock(return_value=None)
        edge = {"motion": "forward", "speed_limit": 0.20}
        self.assertFalse(self.navigator.follow_line(
            (1.0, 0.0, 0.0), 0.0, edge, self.started_at))
        first_cmd = self.navigator.cmd_vel.publish.call_args_list[0].args[0]
        self.assertEqual(first_cmd.linear.x, 0.0)
        self.assertEqual(first_cmd.angular.z, 0.0)

    def test_lateral_error_turns_toward_the_line(self):
        for motion, start_x, goal_x, path_yaw, expected_sign in (
                ("forward", 0.0, 1.0, 0.0, -1),
                ("reverse", 1.0, 0.0, math.pi, 1)):
            with self.subTest(motion=motion):
                self.navigator.cmd_vel.reset_mock()
                self.navigator.get_robot_pose = mock.Mock(side_effect=[
                    (start_x, 0.1, 0.0), (goal_x, 0.0, 0.0)])
                edge = {"motion": motion, "speed_limit": 0.20}
                self.assertTrue(self.navigator.follow_line(
                    (goal_x, 0.0, 0.0), path_yaw, edge,
                    self.started_at))
                first_cmd = self.navigator.cmd_vel.publish.call_args_list[0].args[0]
                self.assertGreater(first_cmd.angular.z * expected_sign, 0.0)

    def test_whole_route_is_followed_once(self):
        self.navigator.navigating = False
        self.navigator.node_snap_distance = 0.2
        self.navigator.edge_snap_distance = 0.2
        self.navigator.get_robot_pose = mock.Mock(return_value=(0.0, 0.0, 0.0))
        self.navigator.publish_path = mock.Mock()
        self.navigator.follow_line = mock.Mock(return_value=True)
        self.navigator.graph = mock.Mock()
        self.navigator.graph.plan_from_pose.return_value = [(1, 2), (2, 3)]
        first_edge = {"motion": "forward", "speed_limit": 0.20}
        second_edge = {"motion": "forward", "speed_limit": 0.15}
        self.navigator.graph.edges = {(1, 2): first_edge, (2, 3): second_edge}
        self.navigator.graph.route_line_yaw.return_value = math.pi / 2
        self.navigator.graph.tag_target_pose.side_effect = lambda tag_id: {
            1: (0.0, 0.0, math.pi / 2),
            3: (0.0, 2.0, math.pi / 2),
        }[tag_id]

        self.navigator.start_navigation(3)

        self.navigator.follow_line.assert_called_once()
        args = self.navigator.follow_line.call_args.args
        self.assertAlmostEqual(args[1], math.pi / 2)
        self.assertEqual(args[2], {"motion": "forward", "speed_limit": 0.15})

    def test_visible_tag_localizes_robot_in_graph_frame(self):
        self.navigator.graph = mock.Mock()
        self.navigator.graph.nodes = {
            2: {"tag_pose": (0.0, 1.0, 0.0), "targetable": True}}
        self.navigator.base_frame = "base_link"
        self.navigator.pose_pub = mock.Mock()
        self.navigator.tag_tf_is_fresh = mock.Mock(side_effect=lambda frame: frame == "tag_2")
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi / 2)
        self.navigator.tf_listener = mock.Mock()
        self.navigator.tf_listener.lookupTransform.side_effect = [
            ((0.2, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((0.0, -0.2, 0.0), rotation),
        ]
        pose = self.navigator.get_robot_pose()
        self.assertAlmostEqual(pose[0], 0.0)
        self.assertAlmostEqual(pose[1], 0.8)
        self.assertAlmostEqual(pose[2], math.pi / 2)
        self.navigator.pose_pub.publish.assert_called_once()

    def test_tag_at_base_origin_is_still_usable(self):
        self.navigator.graph = mock.Mock()
        self.navigator.graph.nodes = {
            6: {"tag_pose": (0.0, 3.0, 0.0), "targetable": False}}
        self.navigator.base_frame = "base_link"
        self.navigator.pose_pub = mock.Mock()
        self.navigator.tag_tf_is_fresh = mock.Mock(side_effect=lambda frame: frame == "tag_6")
        self.navigator.tf_listener = mock.Mock()
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi / 2)
        self.navigator.tf_listener.lookupTransform.side_effect = [
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ((0.0, 0.0, 0.0), rotation),
        ]
        pose = self.navigator.get_robot_pose()
        self.assertAlmostEqual(pose[0], 0.0)
        self.assertAlmostEqual(pose[1], 3.0)
        self.assertAlmostEqual(pose[2], math.pi / 2)


if __name__ == "__main__":
    unittest.main()
