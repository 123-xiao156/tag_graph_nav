#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import geometry_msgs.msg
import tf
from std_msgs.msg import Int32
from math import fabs, sqrt, atan2
from std_srvs.srv import SetBool, SetBoolResponse
from lib.angle import diffAng
import numpy as np


class TagNavigator:
    def __init__(self):
        rospy.init_node('tag_navigator')
        rospy.on_shutdown(self.shutdown)

        # ========== 导航控制参数 ==========
        self.dis_err = 0.05          # 到位距离阈值 (m)
        self.ang_err = 0.05          # 到位角度阈值 (rad)
        self.max_linear = 0.4        # 最大线速度 (m/s)
        self.max_angular = 0.5       # 最大角速度 (rad/s)
        self.min_linear = 0.05       # 最小线速度 (m/s)
        self.min_angular = 0.03      # 最小角速度 (rad/s)，根据底盘实测调整
        self.base_frame = 'base_link'

        # ========== TF与通信接口 ==========
        self.tf_listener = tf.TransformListener()
        self.cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=5)
        rospy.Subscriber('/target_tag_id', Int32, self.target_callback)

        # ========== 状态与配置参数 ==========
        self.target_tag = 0          # 目标标签ID
        self.target_received = False # 是否收到目标话题
        self.auto_start = rospy.get_param('~auto_start', True)
        # self.serve = rospy.Service('nav_to_tag', SetBool, self.handle_nav)
        self.navigating = False

        rospy.loginfo("Tag Navigator Ready")
        rospy.loginfo("Auto start: %s", self.auto_start)

    def target_callback(self, msg):
        """接收话题传来的目标标签ID"""
        rospy.loginfo("Received target tag: %d", msg.data)
        self.target_tag = msg.data
        self.target_received = True

    def get_tag_in_base(self, tag_id):
        """获取tag在base_link下的相对位姿 [x, y, yaw]"""
        tag_frame = 'tag_' + str(tag_id)
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.base_frame, tag_frame, rospy.Time(0))
            x, y, z = trans
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            return [x, y, yaw]
        except:
            return None

    def get_current_visible_tag(self):
        """检测当前视野中所有可见的码，返回距离机器人最近的那个ID"""
        visible_tags = {}
        for tag_id in range(1, 100):
            tag_frame = 'tag_' + str(tag_id)
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    self.base_frame, tag_frame, rospy.Time(0))
                x, y, z = trans
                distance = sqrt(x * x + y * y)
                visible_tags[tag_id] = distance
            except:
                continue

        if len(visible_tags) == 0:
            return None
        return min(visible_tags, key=visible_tags.get)

    def search_tag(self, target_id=None, max_angle=6.28, search_speed=0.25):
        """
        【仅允许在导航起步前调用】原地旋转搜索标签
        :param target_id: 指定搜索的tag_id，None表示搜索任意可见tag
        :param max_angle: 最大搜索角度 (rad)，默认6.28≈360度
        :param search_speed: 搜索角速度 (rad/s)
        :return: 找到返回对应tag_id，失败返回None
        """
        rospy.loginfo("Searching for tag before navigation...")
        rate = rospy.Rate(30)
        rotated_angle = 0.0
        last_time = rospy.Time.now()

        while not rospy.is_shutdown() and self.navigating:
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now
            rotated_angle += search_speed * dt

            # 搜完指定角度还没找到，停止并返回失败
            if rotated_angle >= max_angle:
                self.cmd_vel.publish(geometry_msgs.msg.Twist())
                rospy.logwarn("Search completed, tag not found")
                return None

            # 检测标签
            found_id = None
            if target_id is not None:
                pose = self.get_tag_in_base(target_id)
                if pose is not None:
                    found_id = target_id
            else:
                found_id = self.get_current_visible_tag()

            if found_id is not None:
                self.cmd_vel.publish(geometry_msgs.msg.Twist())
                rospy.loginfo("Tag found during search: tag_%d", found_id)
                return found_id

            # 发布旋转速度
            cmd = geometry_msgs.msg.Twist()
            cmd.angular.z = search_speed
            self.cmd_vel.publish(cmd)
            rate.sleep()

        self.cmd_vel.publish(geometry_msgs.msg.Twist())
        return None

    def follow_tag(self, tag_id):
        """
        视觉伺服跟随单标签（行进过程中不做原地旋转）
        基于tag在base_link下的坐标做闭环控制，不依赖地图定位
        """
        rate = rospy.Rate(30)
        rospy.loginfo("Following tag_%d", tag_id)

        lost_count = 0
        max_lost = 15  # 连续丢帧数阈值

        while not rospy.is_shutdown() and self.navigating:
            tag_pose = self.get_tag_in_base(tag_id)

            # ========== 丢码处理：行进中不原地搜索，超时直接停车 ==========
            if tag_pose is None:
                lost_count += 1
                if lost_count > max_lost:
                    rospy.logerr("Lost tag_%d for too long! Stopping." % tag_id)
                    self.cmd_vel.publish(geometry_msgs.msg.Twist())
                    return False
                else:
                    rospy.logwarn_throttle(0.5, "Lost tag_%d, holding..." % tag_id)
                rate.sleep()
                continue
            else:
                lost_count = 0

            x, y, yaw = tag_pose
            distance = sqrt(x * x + y * y)

            # ========== 到位判定 ==========
            if distance < self.dis_err and fabs(y) < self.dis_err and fabs(yaw) < self.ang_err:
                rospy.loginfo("Reached tag_%d", tag_id)
                self.cmd_vel.publish(geometry_msgs.msg.Twist())
                return True

            # move_cmd = geometry_msgs.msg.Twist()
            # y_threshold = 0.05       # 横向对齐阈值
            # yaw_threshold = 0.1      # 航向对齐阈值
            # angle_to_tag = atan2(y, x)
            # heading_error = yaw + angle_to_tag

            # # ========== 线速度控制：对齐后再前进 ==========
            # if fabs(y) < y_threshold and fabs(heading_error) < yaw_threshold:
            #     move_cmd.linear.x = min(self.max_linear * fabs(distance), self.max_linear)
            #     if move_cmd.linear.x < self.min_linear:
            #         move_cmd.linear.x = self.min_linear
            # else:
            #     move_cmd.linear.x = 0.0
            #     rospy.loginfo_throttle(1.0, "Aligning: y=%.3f, heading_err=%.3f", y, heading_error)

            # # ========== 角速度控制：死区逻辑，解决震荡 ==========
            # kp_y = 1.5
            # kp_heading = 1.0
            # angular_cmd = kp_y * y + kp_heading * heading_error
            # move_cmd.angular.z = self.max_angular * angular_cmd

            # # 死区逻辑：小于最小角速度直接输出0，避免小偏差过冲震荡
            # if abs(move_cmd.angular.z) < self.min_angular:
            #     move_cmd.angular.z = self.min_angular
            # else:
            #     move_cmd.angular.z = max(-self.max_angular, min(move_cmd.angular.z, self.max_angular))
            move_cmd = geometry_msgs.msg.Twist()
            y_threshold = 0.05       # 横向对齐阈值
            yaw_threshold = 0.1      # 航向对齐阈值
            angle_to_tag = atan2(y, x)
            heading_error = yaw + angle_to_tag

            # ========== 分阶段控制 ==========
            
            # 阶段1：横向对齐（只调整y，不前进，不旋转朝向）
            if fabs(y) > y_threshold:
                # 横向有偏差，原地旋转调整横向位置
                # 当tag在左侧(y>0)时，需要逆时针旋转使tag对准中心
                kp_y = 2.0
                angular_cmd = kp_y * y
                
                move_cmd.angular.z = max(-self.max_angular, min(angular_cmd, self.max_angular))
                if abs(move_cmd.angular.z) < self.min_angular:
                    move_cmd.angular.z = 0.0
                    
                rospy.loginfo_throttle(0.5, "Stage1 - Aligning y: y=%.3f, angular.z=%.3f", 
                                        y, move_cmd.angular.z)
            
            # 阶段2：朝向对齐（y已对齐，只旋转调整heading）
            elif fabs(heading_error) > yaw_threshold:
                kp_heading = 2.0
                angular_cmd = kp_heading * heading_error
                
                move_cmd.angular.z = max(-self.max_angular, min(angular_cmd, self.max_angular))
                if abs(move_cmd.angular.z) < self.min_angular:
                    move_cmd.angular.z = 0.0
                    
                rospy.loginfo_throttle(0.5, "Stage2 - Aligning heading: err=%.3f, angular.z=%.3f", 
                                        heading_error, move_cmd.angular.z)
            
            # 阶段3：直线前进（y和heading都已对齐）
            else:
                move_cmd.linear.x = min(self.max_linear * fabs(distance), self.max_linear)
                if move_cmd.linear.x < self.min_linear:
                    move_cmd.linear.x = self.min_linear
                    
                rospy.loginfo_throttle(0.5, "Stage3 - Moving forward: distance=%.3f, linear.x=%.3f", 
                                        distance, move_cmd.linear.x)
            self.cmd_vel.publish(move_cmd)
        rate.sleep()

        return False

    def run_sequence(self, start_id, end_id):
        """按ID顺序逐个跟随标签；反向时【起步前】先转身搜索，找到再起步"""
        step = 1 if end_id > start_id else -1

        # ========== 反向导航：仅在起步前原地转身搜索 ==========
        if step == -1:
            rospy.loginfo("Reverse navigation: turning to search tags before moving")
            # 原地转180度搜索任意可见标签（还没开始行进，允许旋转）
            found_id = self.search_tag(target_id=None, max_angle=3.2, search_speed=0.3)
            if found_id is None:
                rospy.logerr("Reverse navigation failed: no tags found after turning")
                return False
            
            # 确保找到的标签在返程路径上，自动跳过太近看不到的标签（比如5）
            if end_id < found_id < start_id:
                rospy.loginfo("Start reverse from tag_%d (skip closer tags)", found_id)
                actual_start = found_id
            else:
                rospy.logwarn("Found tag_%d not on path, use original start", found_id)
                actual_start = start_id
        else:
            actual_start = start_id

        # ========== 按序列逐个导航（行进中不再原地旋转） ==========
        for tag_id in range(actual_start, end_id + step, step):
            if not self.navigating:
                rospy.loginfo("Navigation cancelled")
                return False

            rospy.loginfo("Navigating to tag_%d...", tag_id)
            success = self.follow_tag(tag_id)
            if not success:
                return False

            rospy.loginfo("Reached tag_%d", tag_id)
            rospy.sleep(0.5)

        rospy.loginfo("Sequence complete: tag_%d -> tag_%d", start_id, end_id)
        return True

    def start_navigation(self):
        """启动导航：检测当前码，然后按顺序跟随到目标"""
        if self.navigating:
            rospy.logwarn("Already navigating!")
            return

        self.navigating = True
        rospy.loginfo("Waiting for tag detection...")

        # 等待检测到当前可见标签（正向起步前，不旋转）
        current_id = None
        wait_rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.navigating:
            current_id = self.get_current_visible_tag()
            if current_id is not None:
                break
            wait_rate.sleep()

        if current_id is None:
            rospy.loginfo("Navigation cancelled while waiting for tag")
            self.navigating = False
            return

        rospy.loginfo("Detected tag_%d, starting navigation to tag_%d",
                      current_id, self.target_tag)

        # 执行导航序列
        success = self.run_sequence(current_id, self.target_tag)
        if not success:
            self.navigating = False
            rospy.logerr("Navigation failed")
            return

        self.navigating = False
        rospy.loginfo("All navigation tasks done")

    # def handle_nav(self, req):
    #     """服务回调：手动启动/停止导航"""
    #     if req.data:
    #         self.start_navigation()
    #         return SetBoolResponse(True, "Navigation started")
    #     else:
    #         self.navigating = False
    #         return SetBoolResponse(True, "Navigation stopped")

    def shutdown(self):
        """节点关闭时停止机器人"""
        self.navigating = False
        self.cmd_vel.publish(geometry_msgs.msg.Twist())
        rospy.sleep(0.1)

    def run(self):
        """主运行逻辑"""
        if self.auto_start:
            rospy.sleep(1.0)  # 等待TF初始化
            rospy.loginfo("Waiting for target tag from topic /target_tag_id...")
            wait_rate = rospy.Rate(10)
            while not rospy.is_shutdown() and not self.target_received:
                wait_rate.sleep()

            if self.target_received:
                rospy.loginfo("Target received: tag_%d, starting navigation", self.target_tag)
                self.start_navigation()

        rospy.spin()


if __name__ == '__main__':
    nav = TagNavigator()
    nav.run()
