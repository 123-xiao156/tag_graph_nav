"""通过 AprilTag 实时定位，并控制机器人跟踪有向图规划的路径。"""
import math
import os
import threading

import geometry_msgs.msg
import nav_msgs.msg
import rospy
import tf
from std_msgs.msg import Int32

from tag_route_graph import TagRouteGraph, angle_error

class TagNavigator:
    def __init__(self):
        rospy.init_node("tag_navigator")
        route_file = rospy.get_param("~route_map_file", "/home/iimt/ltz_ws/tag_route_map.json")
        if not route_file:
            raise ValueError("Set ~route_map_file to a surveyed Tag route JSON file")
        self.graph = TagRouteGraph(os.path.expanduser(route_file))
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.dis_err = rospy.get_param("~arrival_tolerance", 0.10)
        self.ang_err = rospy.get_param("~angle_tolerance", 0.10)
        self.max_linear = rospy.get_param("~max_linear", 0.35)
        self.max_angular = rospy.get_param("~max_angular", 0.30)
        self.min_linear = rospy.get_param("~min_linear", 0.05)
        self.control_rate = rospy.get_param("~control_rate", 30)
        self.heading_kp = rospy.get_param("~heading_kp", 2.0)
        self.lateral_kp = rospy.get_param("~lateral_kp", 1.0)
        self.linear_kp = rospy.get_param("~linear_kp", 0.7)
        self.heading_hold = rospy.get_param("~heading_hold", 0.35)
        self.node_snap_distance = rospy.get_param("~node_snap_distance", 0.10) # ???
        self.edge_snap_distance = rospy.get_param("~edge_snap_distance", 0.40) # ???
        self.max_tag_age = rospy.get_param("~max_tag_age", 0.5) # 如果时间大于这个值，认为TF不够新
        self.max_lost_count = rospy.get_param("~max_lost_count", 15) # 最大丢帧次数
        self.navigation_timeout = rospy.get_param("~navigation_timeout", 60.0) #最长导航时间

        self.tf_listener = tf.TransformListener()
        self.cmd_vel = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=5)
        self.path_pub = rospy.Publisher("~planned_path", nav_msgs.msg.Path, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("~estimated_pose", geometry_msgs.msg.PoseStamped, queue_size=1) # 机器人在map下的位姿
        rospy.Subscriber("/target_tag_id", Int32, self.target_callback)
        rospy.on_shutdown(self.shutdown)

        self.target_tag = None
        self.target_received = False
        self.navigating = False
        self.state_lock = threading.Lock()
        rospy.loginfo("Loaded %d Tags and %d directed edges in %s", len(self.graph.nodes), len(self.graph.edges), self.graph.frame_id)

    def target_callback(self, msg):
        if msg.data not in self.graph.nodes:
            rospy.logwarn("Target tag_%d is not in the route map", msg.data)
            return
        if not self.graph.nodes[msg.data]["targetable"]:
            rospy.logwarn("Target tag_%d is localization-only", msg.data)
            return
        with self.state_lock:
            self.target_tag = msg.data
            self.target_received = True
        rospy.loginfo("Target tag_%d received", msg.data)

    # 判断机器人与指定 Tag 之间的 TF 是否足够新
    def tag_tf_is_fresh(self, tag_frame):
        try:
            stamp = self.tf_listener.getLatestCommonTime(self.base_frame, tag_frame)
            age = (rospy.Time.now() - stamp).to_sec()
            return stamp.to_sec() > 0.0 and 0.0 <= age <= self.max_tag_age
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    # 由最近的有效 Tag TF 计算机器人在路网坐标系中的位姿
    def get_robot_pose(self):
        observations = []
        for tag_id, node in self.graph.nodes.items():
            tag_frame = "tag_%d" % tag_id
            if not self.tag_tf_is_fresh(tag_frame):
                continue
            try:
                tag_in_base, _ = self.tf_listener.lookupTransform(self.base_frame, tag_frame, rospy.Time(0))
                base_in_tag, rotation = self.tf_listener.lookupTransform(tag_frame, self.base_frame, rospy.Time(0))
                _, _, base_yaw_in_tag = tf.transformations.euler_from_quaternion(rotation) #base_yaw_in_tag = 机器人相对 Tag 的偏航角
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            tag_x, tag_y, tag_yaw = node["tag_pose"]
            rel_x, rel_y = base_in_tag[:2]
            # T_graph_base = T_graph_tag * T_tag_base；仅使用平面位置和偏航角。
            robot_pose = (
                tag_x + math.cos(tag_yaw) * rel_x - math.sin(tag_yaw) * rel_y,
                tag_y + math.sin(tag_yaw) * rel_x + math.cos(tag_yaw) * rel_y,
                math.atan2(math.sin(tag_yaw + base_yaw_in_tag), math.cos(tag_yaw + base_yaw_in_tag)),
            )
            observations.append((math.hypot(tag_in_base[0], tag_in_base[1]), tag_id, robot_pose))
        if not observations:
            return None
        _, ref_id, pose = min(observations)
        self.pose_pub.publish(self.to_pose_stamped(pose))
        rospy.loginfo_throttle(2.0, "Robot localized from tag_%d: x=%.2f y=%.2f yaw=%.2f", ref_id, pose[0], pose[1], pose[2])
        return pose

    # 把二维位姿转换成PoseStamped
    def to_pose_stamped(self, pose):
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = self.graph.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y = pose[:2]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, pose[2])
        msg.pose.orientation.x, msg.pose.orientation.y = q[:2]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2:]
        return msg

    # 发布机器人当前位置在参考直线上的投影点和最终目标点
    def publish_path(self, robot_pose, target_pose, path_yaw):
        path = nav_msgs.msg.Path()
        path.header.frame_id = self.graph.frame_id
        path.header.stamp = rospy.Time.now()
        tx, ty = math.cos(path_yaw), math.sin(path_yaw)
        along = ((robot_pose[0] - target_pose[0]) * tx +
                 (robot_pose[1] - target_pose[1]) * ty)
        projection = (target_pose[0] + along * tx,
                      target_pose[1] + along * ty, path_yaw)
        path.poses.append(self.to_pose_stamped(projection))
        path.poses.append(self.to_pose_stamped(
            (target_pose[0], target_pose[1], path_yaw)))
        self.path_pub.publish(path)

    def stop_robot(self):
        self.cmd_vel.publish(geometry_msgs.msg.Twist())

    # 利用实时 Tag 定位，对整条有向直线做横向误差和航向误差 P 控制
    def follow_line(self, target_pose, path_yaw, edge, started_at):
        rate = rospy.Rate(self.control_rate)
        lost_count = 0
        direction = 1.0 if edge["motion"] == "forward" else -1.0
        speed_limit = min(self.max_linear, edge["speed_limit"])
        body_yaw = path_yaw if direction > 0 else path_yaw + math.pi
        while not rospy.is_shutdown() and self.navigating:
            if (rospy.Time.now() - started_at).to_sec() > self.navigation_timeout:
                rospy.logerr("Tag route navigation timed out")
                self.stop_robot()
                return False
            pose = self.get_robot_pose()
            if pose is None:
                lost_count += 1
                self.stop_robot()
                if lost_count > self.max_lost_count:
                    rospy.logerr("Lost all fresh Tag observations")
                    return False
                rate.sleep()
                continue
            lost_count = 0
            dx, dy = target_pose[0] - pose[0], target_pose[1] - pose[1]
            distance = math.hypot(dx, dy)
            if distance <= self.dis_err:
                yaw_error = angle_error(body_yaw, pose[2])
                if abs(yaw_error) <= self.ang_err:
                    self.stop_robot()
                    return True
                linear_cmd = 0.0
                angular_cmd = self.heading_kp * yaw_error
            else:
                heading_error = angle_error(body_yaw, pose[2])
                # 横向误差以运动方向左侧为正；前进和倒车使用同一修正公式。
                lateral_error = (-math.sin(path_yaw) * (pose[0] - target_pose[0]) + math.cos(path_yaw) * (pose[1] - target_pose[1]))
                angular_cmd = (self.heading_kp * heading_error - self.lateral_kp * lateral_error)
                if abs(heading_error) > self.heading_hold:
                    linear_cmd = 0.0
                else:
                    linear_cmd = direction * min(speed_limit, max(self.min_linear, self.linear_kp * distance))
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear_cmd
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, angular_cmd))
            self.cmd_vel.publish(cmd)
            rospy.loginfo_throttle(1.0, "Line tracking: distance=%.3f v=%.3f w=%.3f", distance, cmd.linear.x, cmd.angular.z)
            rate.sleep()
        self.stop_robot()
        return False

    # 定位当前位姿、规划到目标 Tag，并沿整条路线拟合出的有向直线跟踪
    def start_navigation(self, target_id):
        if self.navigating:
            rospy.logwarn("Already navigating")
            return
        self.navigating = True
        started_at = rospy.Time.now()
        success = False
        try:
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                rospy.logerr("No fresh visible Tag for initial localization")
                return
            route = self.graph.plan_from_pose(robot_pose, target_id, self.node_snap_distance, self.edge_snap_distance)
            if route is None:
                rospy.logerr("No directed path to tag_%d from current position", target_id)
                return
            rospy.loginfo("Directed route: %s", " -> ".join("%d:%d" % key for key in route) or "at target node")
            target_pose = self.graph.tag_target_pose(target_id)
            if route:
                edges = [self.graph.edges[key] for key in route]
                motions = {edge["motion"] for edge in edges}
                if len(motions) != 1:
                    raise ValueError("one fitted line cannot mix forward and reverse edges")
                path_yaw = self.graph.route_line_yaw(route)
                edge = {"motion": edges[0]["motion"], "speed_limit": min(e["speed_limit"] for e in edges)}
            else:
                path_yaw = target_pose[2]
                edge = {"motion": "forward", "speed_limit": self.max_linear}
            self.publish_path(robot_pose, target_pose, path_yaw)
            if not self.follow_line(target_pose, path_yaw, edge, started_at):
                return
            success = True
        except (ValueError, KeyError, tf.Exception) as exc:
            rospy.logerr("Tag route navigation failed: %s", exc)
        finally:
            self.navigating = False
            self.stop_robot()
            rospy.loginfo("Navigation %s: tag_%d", "finished" if success else "failed", target_id)

    def shutdown(self):
        self.navigating = False
        self.stop_robot()

    # 持续等待目标 Tag 消息，并串行执行收到的导航任务
    def run(self):
        rospy.loginfo("Waiting for a target on /target_tag_id")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.state_lock:
                target = self.target_tag if self.target_received else None
                self.target_received = False
            if target is not None:
                self.start_navigation(target)
            rate.sleep()
