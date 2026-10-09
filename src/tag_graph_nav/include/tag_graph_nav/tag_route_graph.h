#ifndef TAG_GRAPH_NAV_TAG_ROUTE_GRAPH_H
#define TAG_GRAPH_NAV_TAG_ROUTE_GRAPH_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <json/json.h>
#include <ros/ros.h>

namespace tag_graph_nav
{
// 路网坐标系中的二维位姿
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 有向路网中的Tag节点
struct TagNode
{
  Pose2D tag_pose;       // Tag在路网坐标系中的位姿
  bool targetable{true}; // 是否允许作为导航目标
};

// 有向边标识，first为起点Tag ID，second为终点Tag ID
using EdgeKey = std::pair<int, int>;

// 有向路网中的边
struct RouteEdge
{
  int source{0};                 // 起点Tag ID
  int destination{0};            // 终点Tag ID
  double cost{0.0};              // Dijkstra路径规划代价
  double speed_limit{0.0};       // 当前边最大线速度
  std::string motion;            // 行驶方式：forward或reverse
  std::vector<Pose2D> waypoints; // 起点与终点之间的可选航点
};

// 二维点在有向边折线上的投影结果
struct EdgeProjection
{
  double distance{0.0};         // 点到折线的最短距离
  double fraction{0.0};         // 沿整条折线已行驶的比例
  std::size_t segment_index{0}; // 最近投影点所在的线段下标
};

// 保存有向Tag路网，并提供路网校验、投影和Dijkstra路径规划
class TagRouteGraph
{
public:
  /**********************************************************************
   * Description: 从JSON文件加载并校验有向Tag路网
   * Input:     route_file：路网JSON文件路径
   * Output:    无
   * Return:    无
   **********************************************************************/
  explicit TagRouteGraph(const std::string &route_file);
  /**********************************************************************
   * Description: 判断路网JSON文件是否已成功加载并通过校验
   * Input:     无
   * Output:    无
   * Return:    路网有效时返回true，否则返回false
   **********************************************************************/
  bool IsValid() const { return valid_; }
  /**********************************************************************
   * Description: 计算两个角度之间的最短有符号角度差
   * Input:     a：目标角度
   *            b：当前角度
   * Output:    无
   * Return:    从角度b转到角度a的最短有符号角度差
   **********************************************************************/
  static double AngleError(double a, double b);
  /**********************************************************************
   * Description: 使用Dijkstra算法搜索最小累计代价路径
   * Input:     start：起点Tag ID
   *            goal：目标Tag ID
   * Output:    total_cost：最小累计代价
   *            route：有向边序列，start等于goal时为空序列
   * Return:    目标可达时返回true，否则返回false
   **********************************************************************/
  bool ShortestPath(int start, int goal, double &total_cost, std::vector<EdgeKey> &route) const;
  /**********************************************************************
   * Description: 计算指定Tag对应的机器人目标位姿
   * Input:     tag_id：Tag ID
   * Output:    无
   * Return:    Tag原点处、朝向Tag +Y方向的机器人目标位姿
   **********************************************************************/
  Pose2D TagTargetPose(int tag_id) const;
  /**********************************************************************
   * Description: 使用路线中的全部Tag节点拟合一条有向二维直线
   * Input:     route：规划得到的有向边序列
   * Output:    无
   * Return:    从路线起点指向终点的拟合直线偏航角
   **********************************************************************/
  double RouteLineYaw(const std::vector<EdgeKey> &route) const;
  /**********************************************************************
   * Description: 按照行驶顺序获取指定边上的全部位姿
   * Input:     key：有向边标识
   * Output:    无
   * Return:    边起点、可选中间航点和边终点组成的位姿列表
   **********************************************************************/
  std::vector<Pose2D> EdgePoints(const EdgeKey &key) const;
  /**********************************************************************
   * Description: 将二维点投影到指定有向边的折线路径上
   * Input:     point：待投影的二维点
   *            key：有向边标识
   * Output:    无
   * Return:    横向距离、已行驶比例和最近线段下标
   **********************************************************************/
  EdgeProjection ProjectToEdge(const Pose2D &point, const EdgeKey &key) const;
  /**********************************************************************
   * Description: 获取机器人当前位置之后仍需经过的航点
   * Input:     point：机器人当前二维位置
   *            key：当前有向边标识
   * Output:    无
   * Return:    投影位置之后的航点列表
   **********************************************************************/
  std::vector<Pose2D> RemainingWaypoints(const Pose2D &point, const EdgeKey &key) const;
  /**********************************************************************
   * Description: 将机器人当前位置接入路网，并规划到目标Tag
   * Input:     pose：机器人在路网坐标系中的二维位姿
   *            goal：目标Tag ID
   *            node_snap_distance：允许匹配节点的最大距离
   *            edge_snap_distance：允许匹配边或延长线的最大横向距离
   * Output:    route：规划得到的有向边序列，已位于目标节点时为空序列
   * Return:    成功接入路网且目标可达时返回true，否则返回false
   **********************************************************************/
  bool PlanFromPose(const Pose2D &pose, int goal, double node_snap_distance, double edge_snap_distance, std::vector<EdgeKey> &route) const;
  /**********************************************************************
   * Description: 获取路网坐标系名称
   * Input:     无
   * Output:    无
   * Return:    路网坐标系名称
   **********************************************************************/
  const std::string &FrameId() const { return frame_id_; }
  /**********************************************************************
   * Description: 获取全部Tag节点
   * Input:     无
   * Output:    无
   * Return:    Tag ID到节点数据的映射
   **********************************************************************/
  const std::map<int, TagNode> &Nodes() const { return nodes_; }
  /**********************************************************************
   * Description: 获取全部有向边
   * Input:     无
   * Output:    无
   * Return:    有向边标识到边数据的映射
   **********************************************************************/
  const std::map<EdgeKey, RouteEdge> &Edges() const { return edges_; }

private:
  // 点位于有向边起点后方延长线时的投影结果
  struct StartExtension
  {
    bool valid{false};            // 是否位于起点后方延长线
    double lateral_distance{0.0}; // 点到延长线的横向距离
    double source_distance{0.0};  // 点到边起点的直线距离
  };

  /**********************************************************************
   * Description: 计算二维点到有限线段的最短距离和投影比例
   * Input:     point：待投影的二维点
   *            start：有限线段起点
   *            end：有限线段终点
   * Output:    无
   * Return:    first为最短距离，second为限制在[0, 1]内的投影比例
   **********************************************************************/
  static std::pair<double, double> SegmentProjection(const Pose2D &point, const Pose2D &start, const Pose2D &end);
  /**********************************************************************
   * Description: 判断二维点是否位于有向边起点后方的延长线上
   * Input:     point：待判断的二维点
   *            key：有向边标识
   * Output:    无
   * Return:    起点后方延长线的投影结果
   **********************************************************************/
  StartExtension ProjectToStartExtension(const Pose2D &point, const EdgeKey &key) const;

private:
  bool valid_{false};                             // 路网是否加载并校验成功
  std::string frame_id_;                         // 路网坐标系名称
  std::map<int, TagNode> nodes_;                 // Tag ID到节点的映射
  std::map<EdgeKey, RouteEdge> edges_;           // 有向边标识到边数据的映射
  std::map<int, std::vector<EdgeKey>> outgoing_; // 每个Tag的全部出边
  std::vector<EdgeKey> edge_order_;              // JSON中有向边的原始顺序
};

} // namespace tag_graph_nav

#endif
