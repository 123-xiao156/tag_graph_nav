#!/usr/bin/env python3
# -- coding: UTF-8 --
"""利用 AprilTag TF 自动标定 Tag 在 map 下的位姿。"""

import copy
import datetime
import json
import math
import os
import shutil
import statistics
import tempfile

import rospy
import tf


# 标定任务固定配置：直接修改这里，不需要在启动命令中传入。
ROUTE_MAP_FILE = "/home/iimt/dtx_ws/src/tag_graph_nav/config/tag_route_map.json"
OUTPUT_FILE = "/home/iimt/dtx_ws/src/tag_graph_nav/config/tag_route_map.json"
SAMPLES_PER_TAG = 30
TAG_IDS = list(range(13))
LOCALIZATION_ONLY_TAG_IDS = {9, 10, 11, 12}


def normalize_angle(angle):
    """把角度归一化到 ``[-pi, pi]``。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_mean(angles):
    """计算一组弧度角的圆周平均值。

    Args:
        angles: 非空弧度角序列。

    Returns:
        位于 ``[-pi, pi]`` 的平均角。

    Raises:
        ValueError: 输入为空，或角度分布没有稳定的平均方向。
    """
    if not angles:
        raise ValueError("angles must not be empty")
    sin_sum = sum(math.sin(angle) for angle in angles)
    cos_sum = sum(math.cos(angle) for angle in angles)
    concentration = math.hypot(sin_sum, cos_sum) / len(angles)
    if concentration < 1e-6:
        raise ValueError("yaw samples do not have a stable mean direction")
    return math.atan2(sin_sum, cos_sum)


def robust_pose_estimate(samples, position_floor=0.03, yaw_floor=0.05,
                         mad_scale=3.5):
    """过滤离群值并估计静止 Tag 的二维位姿。

    先以位置中位数和圆周平均角为中心，使用 MAD（中位绝对偏差）建立
    位置与角度阈值，再对保留样本求均值。

    Args:
        samples: ``(x, y, yaw)`` 样本序列。
        position_floor: 位置离群阈值的最小值，单位 m。
        yaw_floor: 角度离群阈值的最小值，单位 rad。
        mad_scale: MAD 阈值倍数。

    Returns:
        字典，包含 ``pose``、样本数、内点数和离散程度。

    Raises:
        ValueError: 样本不足、参数无效或过滤后内点不足。
    """
    if len(samples) < 3:
        raise ValueError("at least three samples are required")
    if position_floor <= 0.0 or yaw_floor <= 0.0 or mad_scale <= 0.0:
        raise ValueError("outlier thresholds must be positive")
    if any(len(sample) != 3 or not all(math.isfinite(v) for v in sample)
           for sample in samples):
        raise ValueError("samples must contain finite (x, y, yaw) values")

    center_x = statistics.median(sample[0] for sample in samples)
    center_y = statistics.median(sample[1] for sample in samples)
    center_yaw = circular_mean([sample[2] for sample in samples])
    position_errors = [
        math.hypot(sample[0] - center_x, sample[1] - center_y)
        for sample in samples
    ]
    yaw_errors = [
        abs(normalize_angle(sample[2] - center_yaw)) for sample in samples
    ]

    position_median = statistics.median(position_errors)
    yaw_median = statistics.median(yaw_errors)
    position_mad = statistics.median(
        abs(error - position_median) for error in position_errors)
    yaw_mad = statistics.median(
        abs(error - yaw_median) for error in yaw_errors)
    # 1.4826 把正态分布的 MAD 换算为标准差量级。
    position_limit = max(
        position_floor,
        position_median + mad_scale * 1.4826 * position_mad)
    yaw_limit = max(
        yaw_floor,
        yaw_median + mad_scale * 1.4826 * yaw_mad)

    inliers = [
        sample for sample, position_error, yaw_error in
        zip(samples, position_errors, yaw_errors)
        if position_error <= position_limit and yaw_error <= yaw_limit
    ]
    if len(inliers) < 3:
        raise ValueError("fewer than three inliers remain after filtering")

    mean_x = sum(sample[0] for sample in inliers) / len(inliers)
    mean_y = sum(sample[1] for sample in inliers) / len(inliers)
    mean_yaw = circular_mean([sample[2] for sample in inliers])
    xy_rms = math.sqrt(sum(
        (sample[0] - mean_x) ** 2 + (sample[1] - mean_y) ** 2
        for sample in inliers) / len(inliers))
    yaw_rms = math.sqrt(sum(
        normalize_angle(sample[2] - mean_yaw) ** 2
        for sample in inliers) / len(inliers))

    return {
        "pose": {"x": mean_x, "y": mean_y, "yaw": mean_yaw},
        "sample_count": len(samples),
        "inlier_count": len(inliers),
        "xy_rms": xy_rms,
        "yaw_rms": yaw_rms,
        "position_limit": position_limit,
        "yaw_limit": yaw_limit,
    }


def load_route_map(path):
    """读取路网模板并返回 JSON 对象。"""
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data.get("frame_id"), str) or not data["frame_id"]:
        raise ValueError("route map frame_id must be a nonempty string")
    nodes = data.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        raise ValueError("route map nodes must be a nonempty list")
    tag_ids = []
    for node in nodes:
        tag_id = node.get("id") if isinstance(node, dict) else None
        if (isinstance(tag_id, bool) or not isinstance(tag_id, int) or
                tag_id < 0 or tag_id in tag_ids):
            raise ValueError("route map Tag IDs must be unique nonnegative integers")
        tag_ids.append(tag_id)
    return data


def update_route_map(template, estimates, generated_at,
                     localization_only_ids=()):
    """把标定位姿写入路网副本，并保留原有边和其他节点字段。"""
    result = copy.deepcopy(template)
    node_ids = {node["id"] for node in result["nodes"]}
    for tag_id in sorted(set(estimates) - node_ids):
        result["nodes"].append({"id": tag_id, "tag_pose": {}})
    for node in result["nodes"]:
        estimate = estimates.get(node["id"])
        if estimate is not None:
            node["tag_pose"] = {
                key: round(float(estimate["pose"][key]), 9)
                for key in ("x", "y", "yaw")
            }
        if node["id"] in localization_only_ids:
            node["targetable"] = False
    result["calibration"] = {
        "generated_at": generated_at,
        "frame_id": result["frame_id"],
        "tags": {
            str(tag_id): {
                key: (int(estimate[key]) if key.endswith("count")
                      else round(float(estimate[key]), 9))
                for key in ("sample_count", "inlier_count", "xy_rms", "yaw_rms")
            }
            for tag_id, estimate in sorted(estimates.items())
        },
    }
    return result


def atomic_write_route_map(data, output_path, make_backup=True):
    """原子写入 JSON；覆盖既有文件前创建带时间戳的备份。

    Returns:
        备份文件路径；没有既有输出文件时返回 ``None``。
    """
    output_path = os.path.abspath(os.path.expanduser(output_path))
    output_dir = os.path.dirname(output_path)
    if not os.path.isdir(output_dir):
        raise ValueError("output directory does not exist: %s" % output_dir)

    backup_path = None
    if make_backup and os.path.exists(output_path):
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        backup_path = "%s.%s.bak" % (output_path, timestamp)
        shutil.copy2(output_path, backup_path)

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=output_dir,
                prefix=".tag_route_map.", suffix=".tmp", delete=False) as stream:
            temporary_path = stream.name
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None and os.path.exists(temporary_path):
            os.unlink(temporary_path)
    return backup_path


class TagPoseCalibrator:
    """从 TF 自动采集多个 Tag 的 map 位姿并更新路网文件。"""

    def __init__(self):
        """读取 ROS 参数和路网模板，并建立 TF 监听器。"""
        rospy.init_node("tag_pose_calibrator")
        self.route_map_file = os.path.abspath(os.path.expanduser(ROUTE_MAP_FILE))
        self.output_file = os.path.abspath(os.path.expanduser(OUTPUT_FILE))
        self.template = load_route_map(self.route_map_file)
        self.map_frame = rospy.get_param("~map_frame", self.template["frame_id"])
        if (self.map_frame.lstrip("/") !=
                self.template["frame_id"].lstrip("/")):
            raise ValueError(
                "~map_frame must match route map frame_id because sampled "
                "coordinates are written into that frame")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tag_frame_format = rospy.get_param("~tag_frame_format", "tag_%d")

        self.tag_ids = rospy.get_param("~tag_ids", TAG_IDS)
        if (not isinstance(self.tag_ids, list) or
                any(isinstance(tag_id, bool) or not isinstance(tag_id, int)
                    for tag_id in self.tag_ids)):
            raise ValueError("~tag_ids must be a list of integers")
        if len(set(self.tag_ids)) != len(self.tag_ids):
            raise ValueError("~tag_ids must not contain duplicates")
        for tag_id in self.tag_ids:
            self._tag_frame(tag_id)

        self.samples_per_tag = SAMPLES_PER_TAG
        self.sample_rate = float(rospy.get_param("~sample_rate", 10.0))
        self.max_tag_age = float(rospy.get_param("~max_tag_age", 0.5))
        self.position_floor = float(rospy.get_param("~position_outlier_floor", 0.03))
        self.yaw_floor = float(rospy.get_param("~yaw_outlier_floor", 0.05))
        self.mad_scale = float(rospy.get_param("~mad_scale", 3.5))
        self.max_xy_rms = float(rospy.get_param("~max_xy_rms", 0.05))
        self.max_yaw_rms = float(rospy.get_param("~max_yaw_rms", 0.10))
        if (self.samples_per_tag < 3 or self.sample_rate <= 0.0 or
                self.max_tag_age <= 0.0 or
                self.max_xy_rms <= 0.0 or self.max_yaw_rms <= 0.0):
            raise ValueError("invalid calibration parameter")

        self.tf_listener = tf.TransformListener()
        self.samples = {tag_id: [] for tag_id in self.tag_ids}
        self.last_stamp = {}

    def _tag_frame(self, tag_id):
        """根据配置格式生成 Tag TF 名称。"""
        try:
            frame = self.tag_frame_format % tag_id
        except (TypeError, ValueError) as exc:
            raise ValueError("~tag_frame_format must accept an integer: %s" % exc)
        if not isinstance(frame, str) or not frame:
            raise ValueError("Tag frame name must be nonempty")
        return frame

    def _read_tag_sample(self, tag_id):
        """读取一个当前可见 Tag 的新 ``map -> tag`` 样本。"""
        tag_frame = self._tag_frame(tag_id)
        try:
            stamp = self.tf_listener.getLatestCommonTime(
                self.base_frame, tag_frame)
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp.to_sec() <= 0.0 or age < -0.1 or age > self.max_tag_age:
                return None
            stamp_key = (stamp.secs, stamp.nsecs)
            if self.last_stamp.get(tag_id) == stamp_key:
                return None
            translation, rotation = self.tf_listener.lookupTransform(
                self.map_frame, tag_frame, stamp)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None

        _, _, yaw = tf.transformations.euler_from_quaternion(rotation)
        sample = (float(translation[0]), float(translation[1]),
                  normalize_angle(float(yaw)))
        if not all(math.isfinite(value) for value in sample):
            return None
        self.last_stamp[tag_id] = stamp_key
        return sample

    def _status_text(self):
        """生成所有目标 Tag 的采样进度字符串。"""
        return " ".join(
            "tag_%d:%d/%d" %
            (tag_id, len(self.samples[tag_id]), self.samples_per_tag)
            for tag_id in self.tag_ids)

    def _build_estimates(self):
        """对所有已完成采集的 Tag 计算稳健位姿。"""
        estimates = {
            tag_id: robust_pose_estimate(
                self.samples[tag_id], self.position_floor,
                self.yaw_floor, self.mad_scale)
            for tag_id in self.tag_ids
        }
        poor_quality = [
            "tag_%d(xy_rms=%.4f,yaw_rms=%.4f)" %
            (tag_id, estimate["xy_rms"], estimate["yaw_rms"])
            for tag_id, estimate in sorted(estimates.items())
            if (estimate["xy_rms"] > self.max_xy_rms or
                estimate["yaw_rms"] > self.max_yaw_rms)
        ]
        if poor_quality:
            raise ValueError(
                "calibration result is too noisy; no file was written: %s" %
                ", ".join(poor_quality))
        return estimates

    def save(self):
        """计算标定结果并安全写入目标路网 JSON。"""
        estimates = self._build_estimates()
        generated_at = datetime.datetime.now(
            datetime.timezone.utc).isoformat()
        output = update_route_map(
            self.template, estimates, generated_at,
            LOCALIZATION_ONLY_TAG_IDS)
        backup_path = atomic_write_route_map(output, self.output_file)
        for tag_id, estimate in sorted(estimates.items()):
            pose = estimate["pose"]
            rospy.loginfo(
                "tag_%d calibrated: x=%.4f y=%.4f yaw=%.4f, "
                "inliers=%d/%d xy_rms=%.4f yaw_rms=%.4f",
                tag_id, pose["x"], pose["y"], pose["yaw"],
                estimate["inlier_count"], estimate["sample_count"],
                estimate["xy_rms"], estimate["yaw_rms"])
        if backup_path:
            rospy.loginfo("Previous route map backed up to %s", backup_path)
        rospy.loginfo("Calibrated route map written to %s", self.output_file)

    def run(self):
        """循环采集各 Tag；全部完成后自动保存并退出。"""
        rospy.loginfo(
            "Collecting %d fresh samples for Tag IDs %s in frame %s",
            self.samples_per_tag, self.tag_ids, self.map_frame)
        rospy.loginfo("Keep the target Tags visible while samples are collected")
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            for tag_id in self.tag_ids:
                if len(self.samples[tag_id]) >= self.samples_per_tag:
                    continue
                sample = self._read_tag_sample(tag_id)
                if sample is not None:
                    self.samples[tag_id].append(sample)
                    rospy.loginfo(
                        "Accepted tag_%d sample %d/%d: x=%.3f y=%.3f yaw=%.3f",
                        tag_id, len(self.samples[tag_id]),
                        self.samples_per_tag, *sample)
            rospy.loginfo_throttle(5.0, "Calibration progress: %s",
                                   self._status_text())
            if all(len(samples) >= self.samples_per_tag
                   for samples in self.samples.values()):
                self.save()
                return
            rate.sleep()


if __name__ == "__main__":
    try:
        TagPoseCalibrator().run()
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        rospy.logfatal("Tag pose calibration failed: %s", exc)
