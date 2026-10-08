#include "tag_graph_nav/tag_navigator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace tag_graph_nav
{
TagNavigator::TagNavigator()
    : nh_(),
      private_nh_("~"),
      tf_listener_(ros::Duration(10.0))
{
  std::string route_file;
  private_nh_.param<std::string>("route_map_file", route_file, "");
  if (route_file.empty())
  {
    ROS_ERROR("Set ~route_map_file to a surveyed Tag route JSON file");
    return;
  }
  graph_.reset(new TagRouteGraph(route_file));
  if (!graph_->isValid())
  {
    ROS_ERROR_STREAM("Failed to load route map: " << route_file);
    return;
  }

  private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
  private_nh_.param("arrival_tolerance", arrival_tolerance_, 0.10);
  private_nh_.param("angle_tolerance", angle_tolerance_, 0.10);
  private_nh_.param("max_linear", max_linear_, 0.35);
  private_nh_.param("max_angular", max_angular_, 0.30);
  private_nh_.param("min_linear", min_linear_, 0.05);
  private_nh_.param("control_rate", control_rate_, 30.0);
  private_nh_.param("heading_kp", heading_kp_, 2.0);
  private_nh_.param("lateral_kp", lateral_kp_, 1.0);
  private_nh_.param("linear_kp", linear_kp_, 0.7);
  private_nh_.param("heading_hold", heading_hold_, 0.35);
  private_nh_.param("node_snap_distance", node_snap_distance_, 0.10);
  private_nh_.param("edge_snap_distance", edge_snap_distance_, 0.40);
  private_nh_.param("max_tag_age", max_tag_age_, 0.5);
  private_nh_.param("max_lost_count", max_lost_count_, 15);
  private_nh_.param("navigation_timeout", navigation_timeout_, 60.0);

  if (!validateParameters())
  {
    return;
  }

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  path_pub_ = private_nh_.advertise<nav_msgs::Path>("planned_path", 1, true);
  pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("estimated_pose", 1);
  target_sub_ = nh_.subscribe("/target_tag_id", 1, &TagNavigator::targetCallback, this);
  initialized_ = true;
  ROS_INFO("Loaded %zu Tags and %zu directed edges in %s", graph_->nodes().size(), graph_->edges().size(), graph_->frameId().c_str());
}
TagNavigator::~TagNavigator()
{
  shutdown();
}
void TagNavigator::run()
{
  if (!initialized_)
  {
    ROS_ERROR("TagNavigator is not initialized");
    return;
  }
  ROS_INFO("Waiting for a target on /target_tag_id");
  ros::Rate rate(10.0);
  while (ros::ok())
  {
    int target = -1;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (target_received_)
      {
        target = target_tag_;
        target_received_ = false;
      }
    }
    if (target >= 0)
    {
      startNavigation(target);
    }
    rate.sleep();
  }
}
bool TagNavigator::validateParameters() const
{
  if (arrival_tolerance_ < 0.0 || angle_tolerance_ < 0.0 ||
      max_linear_ <= 0.0 || max_angular_ <= 0.0 || min_linear_ < 0.0 ||
      min_linear_ > max_linear_ || control_rate_ <= 0.0 ||
      node_snap_distance_ < 0.0 || edge_snap_distance_ < 0.0 ||
      max_tag_age_ <= 0.0 || max_lost_count_ < 0 || navigation_timeout_ <= 0.0)
  {
    ROS_ERROR("Invalid navigation parameter value");
    return false;
  }
  return true;
}
void TagNavigator::targetCallback(const std_msgs::Int32::ConstPtr &message)
{
  const auto node = graph_->nodes().find(message->data);
  if (node == graph_->nodes().end())
  {
    ROS_WARN("Target tag_%d is not in the route map", message->data);
    return;
  }
  if (!node->second.targetable)
  {
    ROS_WARN("Target tag_%d is localization-only", message->data);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_tag_ = message->data;
    target_received_ = true;
  }
  ROS_INFO("Target tag_%d received", message->data);
}
bool TagNavigator::tagTransformIsFresh(const std::string &tag_frame) const
{
  try
  {
    ros::Time stamp;
    std::string error;
    tf_listener_.getLatestCommonTime(base_frame_, tag_frame, stamp, &error);
    const double age = (ros::Time::now() - stamp).toSec();
    return stamp.toSec() > 0.0 && age >= 0.0 && age <= max_tag_age_;
  }
  catch (const tf::TransformException &error)
  {
    ROS_DEBUG_THROTTLE(2.0, "Cannot query fresh TF %s <-> %s: %s", base_frame_.c_str(), tag_frame.c_str(), error.what());
    return false;
  }
}
bool TagNavigator::getRobotPose(Pose2D *robot_pose)
{
  struct Observation
  {
    double distance;
    int tag_id;
    Pose2D pose;
  };
  std::vector<Observation> observations;

  for (const auto &item : graph_->nodes())
  {
    const int tag_id = item.first;
    const std::string tag_frame = "tag_" + std::to_string(tag_id);
    if (!tagTransformIsFresh(tag_frame))
    {
      continue;
    }
    try
    {
      tf::StampedTransform tag_in_base;
      tf::StampedTransform base_in_tag;
      tf_listener_.lookupTransform(base_frame_, tag_frame, ros::Time(0), tag_in_base);
      tf_listener_.lookupTransform(tag_frame, base_frame_, ros::Time(0), base_in_tag);

      const Pose2D &tag_pose = item.second.tag_pose;
      const double relative_x = base_in_tag.getOrigin().x();
      const double relative_y = base_in_tag.getOrigin().y();
      const double relative_yaw = tf::getYaw(base_in_tag.getRotation());
      const Pose2D pose{
          tag_pose.x + std::cos(tag_pose.yaw) * relative_x - std::sin(tag_pose.yaw) * relative_y,
          tag_pose.y + std::sin(tag_pose.yaw) * relative_x + std::cos(tag_pose.yaw) * relative_y,
          TagRouteGraph::angleError(tag_pose.yaw + relative_yaw, 0.0)};
      observations.push_back(Observation{
          std::hypot(tag_in_base.getOrigin().x(), tag_in_base.getOrigin().y()),
          tag_id,
          pose});
    }
    catch (const tf::TransformException &error)
    {
      ROS_DEBUG_THROTTLE(2.0, "Cannot transform with %s: %s", tag_frame.c_str(), error.what());
      continue;
    }
  }

  if (observations.empty())
  {
    return false;
  }
  const Observation &best = *std::min_element(
      observations.begin(), observations.end(),
      [](const Observation &a, const Observation &b)
      {
        return std::tie(a.distance, a.tag_id) < std::tie(b.distance, b.tag_id);
      });
  *robot_pose = best.pose;
  pose_pub_.publish(toPoseStamped(best.pose));
  ROS_INFO_THROTTLE(2.0, "Robot localized from tag_%d: x=%.2f y=%.2f yaw=%.2f", best.tag_id, best.pose.x, best.pose.y, best.pose.yaw);
  return true;
}
geometry_msgs::PoseStamped TagNavigator::toPoseStamped(const Pose2D &pose) const
{
  geometry_msgs::PoseStamped message;
  message.header.frame_id = graph_->frameId();
  message.header.stamp = ros::Time::now();
  message.pose.position.x = pose.x;
  message.pose.position.y = pose.y;
  message.pose.orientation = tf::createQuaternionMsgFromYaw(pose.yaw);
  return message;
}
void TagNavigator::publishPath(const Pose2D &robot_pose, const Pose2D &target_pose, double path_yaw)
{
  nav_msgs::Path path;
  path.header.frame_id = graph_->frameId();
  path.header.stamp = ros::Time::now();
  const double tangent_x = std::cos(path_yaw);
  const double tangent_y = std::sin(path_yaw);
  const double along = (robot_pose.x - target_pose.x) * tangent_x + (robot_pose.y - target_pose.y) * tangent_y;
  path.poses.push_back(toPoseStamped(Pose2D{
      target_pose.x + along * tangent_x,
      target_pose.y + along * tangent_y,
      path_yaw}));
  path.poses.push_back(toPoseStamped(Pose2D{target_pose.x, target_pose.y, path_yaw}));
  path_pub_.publish(path);
}
void TagNavigator::stopRobot() const
{
  cmd_vel_pub_.publish(geometry_msgs::Twist());
}
bool TagNavigator::followLine(const Pose2D &target_pose, double path_yaw, const RouteEdge &route_settings, const ros::Time &started_at)
{
  ros::Rate rate(control_rate_);
  int lost_count = 0;
  const double direction = route_settings.motion == "forward" ? 1.0 : -1.0;
  const double speed_limit = std::min(max_linear_, route_settings.speed_limit);
  const double body_yaw = direction > 0.0 ? path_yaw : path_yaw + M_PI;

  while (ros::ok() && navigating_)
  {
    if ((ros::Time::now() - started_at).toSec() > navigation_timeout_)
    {
      ROS_ERROR("Tag route navigation timed out");
      stopRobot();
      return false;
    }

    Pose2D pose;
    if (!getRobotPose(&pose))
    {
      ++lost_count;
      stopRobot();
      if (lost_count > max_lost_count_)
      {
        ROS_ERROR("Lost all fresh Tag observations");
        return false;
      }
      rate.sleep();
      continue;
    }
    lost_count = 0;

    const double dx = target_pose.x - pose.x;
    const double dy = target_pose.y - pose.y;
    const double distance = std::hypot(dx, dy);
    double linear_command = 0.0;
    double angular_command = 0.0;
    if (distance <= arrival_tolerance_)
    {
      const double yaw_error = TagRouteGraph::angleError(body_yaw, pose.yaw);
      if (std::abs(yaw_error) <= angle_tolerance_)
      {
        stopRobot();
        return true;
      }
      angular_command = heading_kp_ * yaw_error;
    }
    else
    {
      const double heading_error = TagRouteGraph::angleError(body_yaw, pose.yaw);
      const double lateral_error = -std::sin(path_yaw) * (pose.x - target_pose.x) + std::cos(path_yaw) * (pose.y - target_pose.y);
      angular_command = heading_kp_ * heading_error - lateral_kp_ * lateral_error;
      if (std::abs(heading_error) <= heading_hold_)
      {
        linear_command = direction * std::min(speed_limit, std::max(min_linear_, linear_kp_ * distance));
      }
    }

    geometry_msgs::Twist command;
    command.linear.x = linear_command;
    command.angular.z = std::max(-max_angular_, std::min(max_angular_, angular_command));
    cmd_vel_pub_.publish(command);
    ROS_INFO_THROTTLE(1.0, "Line tracking: distance=%.3f v=%.3f w=%.3f", distance, command.linear.x, command.angular.z);
    rate.sleep();
  }
  stopRobot();
  return false;
}
void TagNavigator::startNavigation(int target_id)
{
  if (navigating_.exchange(true))
  {
    ROS_WARN("Already navigating");
    return;
  }
  const ros::Time started_at = ros::Time::now();
  bool success = false;
  Pose2D robot_pose;
  if (!getRobotPose(&robot_pose))
  {
    ROS_ERROR("No fresh visible Tag for initial localization");
  }
  else
  {
    std::vector<EdgeKey> route;
    if (!graph_->planFromPose(robot_pose, target_id, node_snap_distance_, edge_snap_distance_, &route))
    {
      ROS_ERROR("No directed path to tag_%d from current position", target_id);
    }
    else
    {
      std::ostringstream route_text;
      for (std::size_t index = 0; index < route.size(); ++index)
      {
        if (index != 0U)
        {
          route_text << " -> ";
        }
        route_text << route[index].first << ":" << route[index].second;
      }
      ROS_INFO("Directed route: %s", route.empty() ? "at target node" : route_text.str().c_str());

      const Pose2D target_pose = graph_->tagTargetPose(target_id);
      double path_yaw = target_pose.yaw;
      RouteEdge route_settings;
      route_settings.motion = "forward";
      route_settings.speed_limit = max_linear_;
      bool route_settings_valid = true;
      if (!route.empty())
      {
        std::set<std::string> motions;
        double minimum_speed = std::numeric_limits<double>::infinity();
        for (const EdgeKey &key : route)
        {
          const RouteEdge &edge = graph_->edges().at(key);
          motions.insert(edge.motion);
          minimum_speed = std::min(minimum_speed, edge.speed_limit);
        }
        if (motions.size() != 1U)
        {
          ROS_ERROR("One fitted line cannot mix forward and reverse edges");
          route_settings_valid = false;
        }
        else
        {
          path_yaw = graph_->routeLineYaw(route);
          route_settings.motion = graph_->edges().at(route.front()).motion;
          route_settings.speed_limit = minimum_speed;
        }
      }
      if (route_settings_valid)
      {
        publishPath(robot_pose, target_pose, path_yaw);
        success = followLine(target_pose, path_yaw, route_settings, started_at);
      }
    }
  }

  navigating_ = false;
  stopRobot();
  ROS_INFO("Navigation %s: tag_%d", success ? "finished" : "failed", target_id);
}
void TagNavigator::shutdown()
{
  navigating_ = false;
  stopRobot();
}

} // namespace tag_graph_nav
