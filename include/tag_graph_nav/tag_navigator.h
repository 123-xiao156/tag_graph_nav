#ifndef TAG_GRAPH_NAV_TAG_NAVIGATOR_H
#define TAG_GRAPH_NAV_TAG_NAVIGATOR_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "tag_graph_nav/tag_route_graph.h"

namespace tag_graph_nav
{
// 通过AprilTag实时定位，并执行有向图路径规划和直线跟踪控制
class TagNavigator
{
public:
  /**********************************************************************
   * Description: 初始化导航参数、路网、TF监听器和ROS通信接口
   * Input:     无
   * Output:    无
   * Return:    无
   **********************************************************************/
  TagNavigator();
  /**********************************************************************
   * Description: 判断导航参数、路网和ROS通信接口是否初始化成功
   * Input:     无
   * Output:    无
   * Return:    初始化成功时返回true，否则返回false
   **********************************************************************/
  bool IsInitialized() const { return initialized_; }
  /**********************************************************************
   * Description: 持续等待目标Tag消息，并串行执行导航任务
   * Input:     无
   * Output:    无
   * Return:    无
   **********************************************************************/
  void Run();

private:
  /**********************************************************************
   * Description: 校验控制器参数是否在有效范围内
   * Input:     无
   * Output:    无
   * Return:    全部参数有效时返回true，否则返回false
   **********************************************************************/
  bool ValidateParameters() const;
  /**********************************************************************
   * Description: 接收并校验目标Tag ID
   * Input:     message：目标Tag ID消息
   * Output:    无
   * Return:    无
   **********************************************************************/
  void TargetCallback(const std_msgs::Int32::ConstPtr &message);
  /**********************************************************************
   * Description: 判断机器人与指定Tag之间的TF是否足够新
   * Input:     tag_frame：Tag坐标系名称
   * Output:    无
   * Return:    TF时间戳有效且未超过最大时间时返回true
   **********************************************************************/
  bool TagTransformIsFresh(const std::string &tag_frame) const;
  /**********************************************************************
   * Description: 使用距离最近的新鲜Tag观测计算机器人二维位姿
   * Input:     无
   * Output:    robot_pose：机器人在路网坐标系中的二维位姿
   * Return:    成功定位时返回true，否则返回false
   **********************************************************************/
  bool GetRobotPose(Pose2D &robot_pose) const;
  /**********************************************************************
   * Description: 将二维位姿转换为ROS PoseStamped消息
   * Input:     pose：路网坐标系中的二维位姿
   * Output:    无
   * Return:    带路网坐标系和当前时间戳的PoseStamped消息
   **********************************************************************/
  geometry_msgs::PoseStamped ToPoseStamped(const Pose2D &pose) const;
  /**********************************************************************
   * Description: 发布机器人投影点和最终目标点组成的规划路径
   * Input:     robot_pose：机器人当前二维位姿
   *            target_pose：最终目标二维位姿
   *            path_yaw：拟合路径直线的偏航角
   * Output:    无
   * Return:    无
   **********************************************************************/
  void PublishPath(const Pose2D &robot_pose, const Pose2D &target_pose, double path_yaw) const;
  /**********************************************************************
   * Description: 发布零速度使机器人停止
   * Input:     无
   * Output:    无
   * Return:    无
   **********************************************************************/
  void StopRobot() const;
  /**********************************************************************
   * Description: 使用横向误差和航向误差P控制跟踪拟合直线
   * Input:     target_pose：最终目标二维位姿
   *            path_yaw：拟合路径直线的偏航角
   *            route_settings：整条路线的运动方向和速度限制
   *            started_at：本次导航任务开始时间
   * Output:    无
   * Return:    到达目标位置和角度时返回true，否则返回false
   **********************************************************************/
  bool FollowLine(const Pose2D &target_pose, double path_yaw, const RouteEdge &route_settings, const ros::Time &started_at) const;
  /**********************************************************************
   * Description: 定位机器人、规划有向路径并启动直线跟踪
   * Input:     target_id：目标Tag ID
   * Output:    无
   * Return:    无
   **********************************************************************/
  void StartNavigation(int target_id);
private:
  bool initialized_{false};                // 导航器是否初始化成功
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  mutable tf::TransformListener tf_listener_; // 监听base_link与Tag之间的TF
  std::unique_ptr<TagRouteGraph> graph_;       // 有向Tag路网
  ros::Publisher cmd_vel_pub_;                // 发布底盘速度
  ros::Publisher path_pub_;                   // 发布拟合后的规划路径
  ros::Publisher pose_pub_;                   // 发布Tag定位得到的机器人位姿
  ros::Subscriber target_sub_;                // 接收目标Tag ID

  std::string base_frame_;              // 机器人底盘坐标系
  double arrival_tolerance_{0.10};       // 到达目标的位置误差阈值
  double angle_tolerance_{0.10};         // 到达目标的角度误差阈值
  double max_linear_{0.35};              // 最大线速度
  double max_angular_{0.30};             // 最大角速度
  double min_linear_{0.05};              // 最小非零线速度
  double control_rate_{30.0};            // 控制频率
  double heading_kp_{2.0};               // 航向误差比例系数
  double lateral_kp_{1.0};               // 横向误差比例系数
  double linear_kp_{0.7};                // 剩余距离比例系数
  double heading_hold_{0.35};             // 允许输出线速度的最大航向误差
  double node_snap_distance_{0.10};       // 节点匹配距离
  double edge_snap_distance_{0.40};       // 有向边匹配横向距离
  double max_tag_age_{0.5};               // Tag TF最大允许时间
  int max_lost_count_{15};                // 最大连续定位丢失次数
  double navigation_timeout_{60.0};       // 单次导航超时时间

  std::atomic<bool> navigating_{false}; // 是否正在执行导航
  std::mutex state_mutex_;              // 保护目标Tag接收状态
  int target_tag_{-1};                  // 最近接收的目标Tag ID
  bool target_received_{false};         // 是否收到尚未执行的新目标
};

} // namespace tag_graph_nav

#endif
