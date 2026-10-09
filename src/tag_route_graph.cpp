#include "tag_graph_nav/tag_route_graph.h"

namespace tag_graph_nav
{
namespace
{
bool finiteNumber(const Json::Value &value, const std::string &name, double *number)
{
  if (number == nullptr)
  {
    ROS_ERROR("finiteNumber output pointer is null");
    return false;
  }
  if (value.isBool() || !value.isNumeric())
  {
    ROS_ERROR_STREAM(name << " must be a finite number");
    return false;
  }
  const double parsed_number = value.asDouble();
  if (!std::isfinite(parsed_number))
  {
    ROS_ERROR_STREAM(name << " must be a finite number");
    return false;
  }
  *number = parsed_number;
  return true;
}

const Json::Value *requiredMember(const Json::Value &object, const std::string &key, const std::string &owner)
{
  if (!object.isObject() || !object.isMember(key))
  {
    ROS_ERROR_STREAM(owner << "." << key << " is required");
    return nullptr;
  }
  return &object[key];
}

bool parsePose(const Json::Value &value, const std::string &name, Pose2D *pose)
{
  if (pose == nullptr)
  {
    ROS_ERROR("parsePose output pointer is null");
    return false;
  }
  if (!value.isObject())
  {
    ROS_ERROR_STREAM(name << " must be an object");
    return false;
  }
  const Json::Value *x_value = requiredMember(value, "x", name);
  const Json::Value *y_value = requiredMember(value, "y", name);
  const Json::Value *yaw_value = requiredMember(value, "yaw", name);
  if (x_value == nullptr || y_value == nullptr || yaw_value == nullptr)
  {
    return false;
  }
  return finiteNumber(*x_value, name + ".x", &pose->x) &&
         finiteNumber(*y_value, name + ".y", &pose->y) &&
         finiteNumber(*yaw_value, name + ".yaw", &pose->yaw);
}

std::string edgeName(const EdgeKey &key)
{
  std::ostringstream stream;
  stream << "(" << key.first << ", " << key.second << ")";
  return stream.str();
}

double distance2D(const Pose2D &a, const Pose2D &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
} // namespace

TagRouteGraph::TagRouteGraph(const std::string &route_file)
{
  // 读取并解析路网JSON；任一必需字段无效时保持valid_为false。
  std::ifstream stream(route_file);
  if (!stream)
  {
    ROS_ERROR_STREAM("Cannot open route map: " << route_file);
    return;
  }

  Json::CharReaderBuilder builder;
  Json::Value data;
  std::string errors;
  if (!Json::parseFromStream(builder, stream, &data, &errors))
  {
    ROS_ERROR_STREAM("Cannot parse route map " << route_file << ": " << errors);
    return;
  }

  const Json::Value *frame = requiredMember(data, "frame_id", "route graph");
  if (frame == nullptr || !frame->isString() || frame->asString().empty())
  {
    ROS_ERROR("frame_id must be a nonempty string");
    return;
  }
  frame_id_ = frame->asString();

  // 先加载节点，后续解析边时即可校验端点是否存在。
  const Json::Value *node_array = requiredMember(data, "nodes", "route graph");
  if (node_array == nullptr || !node_array->isArray())
  {
    ROS_ERROR("nodes must be an array");
    return;
  }
  for (const Json::Value &item : *node_array)
  {
    const Json::Value *id_value = requiredMember(item, "id", "node");
    const Json::Value *pose_value = requiredMember(item, "tag_pose", "node");
    if (id_value == nullptr || pose_value == nullptr)
    {
      return;
    }
    if (!id_value->isInt() || id_value->asInt() < 0 || nodes_.count(id_value->asInt()) != 0U)
    {
      ROS_ERROR("Tag IDs must be unique nonnegative integers");
      return;
    }
    const int tag_id = id_value->asInt();
    bool targetable = true;
    if (item.isMember("targetable"))
    {
      if (!item["targetable"].isBool())
      {
        ROS_ERROR("node.targetable must be a boolean");
        return;
      }
      targetable = item["targetable"].asBool();
    }
    Pose2D tag_pose;
    if (!parsePose(*pose_value, "tag_pose", &tag_pose))
    {
      return;
    }
    nodes_[tag_id] = TagNode{tag_pose, targetable};
    outgoing_[tag_id] = {};
  }
  if (nodes_.empty())
  {
    ROS_ERROR("Route graph has no nodes");
    return;
  }

  // 边按照JSON中的顺序保存，保证候选路径选择具有确定性。
  const Json::Value *edge_array = requiredMember(data, "edges", "route graph");
  if (edge_array == nullptr || !edge_array->isArray())
  {
    ROS_ERROR("edges must be an array");
    return;
  }
  for (const Json::Value &item : *edge_array)
  {
    const Json::Value *source_value = requiredMember(item, "source", "edge");
    const Json::Value *destination_value = requiredMember(item, "destination", "edge");
    const Json::Value *cost_value = requiredMember(item, "cost", "edge");
    const Json::Value *speed_value = requiredMember(item, "speed_limit", "edge");
    if (source_value == nullptr || destination_value == nullptr || cost_value == nullptr || speed_value == nullptr)
    {
      return;
    }
    if (!source_value->isInt() || !destination_value->isInt())
    {
      ROS_ERROR("edge source and destination must be integers");
      return;
    }
    const EdgeKey key{source_value->asInt(), destination_value->asInt()};
    if (nodes_.count(key.first) == 0U || nodes_.count(key.second) == 0U || key.first == key.second || edges_.count(key) != 0U)
    {
      ROS_ERROR_STREAM("Invalid or duplicate directed edge " << edgeName(key));
      return;
    }

    double cost = 0.0;
    double speed_limit = 0.0;
    if (!finiteNumber(*cost_value, "edge.cost", &cost) || !finiteNumber(*speed_value, "edge.speed_limit", &speed_limit))
    {
      return;
    }
    std::string motion = "forward";
    if (item.isMember("motion"))
    {
      if (!item["motion"].isString())
      {
        ROS_ERROR("edge.motion must be a string");
        return;
      }
      motion = item["motion"].asString();
    }
    if (cost <= 0.0 || speed_limit <= 0.0 || (motion != "forward" && motion != "reverse"))
    {
      ROS_ERROR_STREAM("Invalid cost, speed_limit or motion for edge " << edgeName(key));
      return;
    }

    std::vector<Pose2D> waypoints;
    if (item.isMember("waypoints"))
    {
      if (!item["waypoints"].isArray())
      {
        ROS_ERROR("edge.waypoints must be an array");
        return;
      }
      for (const Json::Value &waypoint_value : item["waypoints"])
      {
        Pose2D waypoint;
        if (!parsePose(waypoint_value, "waypoint", &waypoint))
        {
          return;
        }
        waypoints.push_back(waypoint);
      }
    }
    edges_[key] = RouteEdge{key.first, key.second, cost, speed_limit, motion, std::move(waypoints)};
    outgoing_[key.first].push_back(key);
    edge_order_.push_back(key);
  }

  valid_ = true;
}

  double TagRouteGraph::angleError(double a, double b)
  {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  bool TagRouteGraph::shortestPath(int start, int goal, double *total_cost, std::vector<EdgeKey> *route) const
  {
    if (total_cost == nullptr || route == nullptr)
    {
      ROS_ERROR("shortestPath output pointer is null");
      return false;
    }
    if (!valid_)
    {
      ROS_ERROR("Cannot search an invalid route graph");
      return false;
    }
    if (nodes_.count(start) == 0U || nodes_.count(goal) == 0U)
    {
      ROS_ERROR("Start or goal Tag is absent from graph");
      *total_cost = std::numeric_limits<double>::infinity();
      route->clear();
      return false;
    }

    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    std::map<int, double> distance;
    std::map<int, EdgeKey> previous;
    distance[start] = 0.0;
    queue.push({0.0, start});

    // 使用Dijkstra算法搜索总代价最小的有向路径。
    while (!queue.empty())
    {
      const double current_cost = queue.top().first;
      int current = queue.top().second;
      queue.pop();
      if (current_cost != distance.at(current))
      {
        continue;
      }
      if (current == goal)
      {
        route->clear();
        while (current != start)
        {
          const EdgeKey edge_key = previous.at(current);
          route->push_back(edge_key);
          current = edge_key.first;
        }
        std::reverse(route->begin(), route->end());
        *total_cost = current_cost;
        return true;
      }
      for (const EdgeKey &key : outgoing_.at(current))
      {
        const RouteEdge &edge = edges_.at(key);
        const double next_cost = current_cost + edge.cost;
        const auto found = distance.find(edge.destination);
        if (found == distance.end() || next_cost < found->second)
        {
          distance[edge.destination] = next_cost;
          previous[edge.destination] = key;
          queue.push({next_cost, edge.destination});
        }
      }
    }

    *total_cost = std::numeric_limits<double>::infinity();
    route->clear();
    return false;
  }

  Pose2D TagRouteGraph::tagTargetPose(int tag_id) const
  {
    const auto found = nodes_.find(tag_id);
    if (found == nodes_.end())
    {
      ROS_ERROR("Target tag_%d is absent from graph", tag_id);
      return Pose2D{};
    }
    Pose2D target = found->second.tag_pose;
    target.yaw = angleError(target.yaw + M_PI_2, 0.0);
    return target;
  }

  double TagRouteGraph::routeLineYaw(const std::vector<EdgeKey> &route) const
  {
    if (route.empty())
    {
      ROS_ERROR("Cannot fit a line from an empty route");
      return 0.0;
    }
    std::vector<Pose2D> points;
    const auto start_node = nodes_.find(route.front().first);
    if (start_node == nodes_.end())
    {
      ROS_ERROR("Route start tag_%d is absent from graph", route.front().first);
      return 0.0;
    }
    points.push_back(start_node->second.tag_pose);
    for (const EdgeKey &key : route)
    {
      const auto node = nodes_.find(key.second);
      if (node == nodes_.end())
      {
        ROS_ERROR("Route tag_%d is absent from graph", key.second);
        return 0.0;
      }
      points.push_back(node->second.tag_pose);
    }

    // 计算所有路线节点的中心，用二维主方向拟合一条全局直线。
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const Pose2D &point : points)
    {
      mean_x += point.x;
      mean_y += point.y;
    }
    mean_x /= static_cast<double>(points.size());
    mean_y /= static_cast<double>(points.size());

    double xx = 0.0;
    double yy = 0.0;
    double xy = 0.0;
    for (const Pose2D &point : points)
    {
      xx += (point.x - mean_x) * (point.x - mean_x);
      yy += (point.y - mean_y) * (point.y - mean_y);
      xy += (point.x - mean_x) * (point.y - mean_y);
    }
    if (xx + yy <= 1e-12)
    {
      ROS_ERROR("Route nodes cannot define a line");
      return 0.0;
    }

    double yaw = 0.5 * std::atan2(2.0 * xy, xx - yy);
    const Pose2D &start = points.front();
    const Pose2D &goal = points.back();
    // 主方向本身没有正反，按路线起点到终点的方向消除二义性。
    if (std::cos(yaw) * (goal.x - start.x) + std::sin(yaw) * (goal.y - start.y) < 0.0)
    {
      yaw += M_PI;
    }
    return angleError(yaw, 0.0);
  }

  std::vector<Pose2D> TagRouteGraph::edgePoints(const EdgeKey &key) const
  {
    const auto found = edges_.find(key);
    if (found == edges_.end())
    {
      ROS_ERROR_STREAM("Directed edge " << edgeName(key) << " is absent from graph");
      return {};
    }
    const RouteEdge &edge = found->second;
    std::vector<Pose2D> points;
    points.reserve(edge.waypoints.size() + 2U);
    points.push_back(tagTargetPose(key.first));
    points.insert(points.end(), edge.waypoints.begin(), edge.waypoints.end());
    points.push_back(tagTargetPose(key.second));
    return points;
  }

  std::pair<double, double> TagRouteGraph::segmentProjection(const Pose2D &point, const Pose2D &start, const Pose2D &end)
  {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared == 0.0)
    {
      ROS_ERROR("Directed edge segment has zero geometric length");
      return {std::numeric_limits<double>::infinity(), 0.0};
    }
    const double raw_fraction = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared;
    // 将投影比例限制在线段范围内，而不是使用无限延长线。
    const double fraction = std::max(0.0, std::min(1.0, raw_fraction));
    const double projection_x = start.x + fraction * dx;
    const double projection_y = start.y + fraction * dy;
    return {std::hypot(point.x - projection_x, point.y - projection_y), fraction};
  }

  EdgeProjection TagRouteGraph::projectToEdge(const Pose2D &point, const EdgeKey &key) const
  {
    const std::vector<Pose2D> points = edgePoints(key);
    if (points.size() < 2U)
    {
      ROS_ERROR_STREAM("Directed edge " << edgeName(key) << " has insufficient points");
      return EdgeProjection{std::numeric_limits<double>::infinity(), 0.0, 0U};
    }
    std::vector<double> lengths;
    lengths.reserve(points.size() - 1U);
    double total_length = 0.0;
    for (std::size_t index = 0; index + 1U < points.size(); ++index)
    {
      const double length = distance2D(points[index], points[index + 1U]);
      lengths.push_back(length);
      total_length += length;
    }
    if (total_length == 0.0)
    {
      ROS_ERROR_STREAM("Directed edge " << edgeName(key) << " has zero geometric length");
      return EdgeProjection{std::numeric_limits<double>::infinity(), 0.0, 0U};
    }

    EdgeProjection best{std::numeric_limits<double>::infinity(), 0.0, 0U};
    double traveled = 0.0;
    // 遍历折线的每一段，保留横向距离最小的投影及其全边进度。
    for (std::size_t index = 0; index < lengths.size(); ++index)
    {
      if (lengths[index] > 0.0)
      {
        const auto projection = segmentProjection(point, points[index], points[index + 1U]);
        const EdgeProjection candidate{projection.first, (traveled + projection.second * lengths[index]) / total_length, index};
        if (candidate.distance < best.distance)
        {
          best = candidate;
        }
      }
      traveled += lengths[index];
    }
    return best;
  }

  TagRouteGraph::StartExtension TagRouteGraph::projectToStartExtension(const Pose2D &point, const EdgeKey &key) const
  {
    const std::vector<Pose2D> points = edgePoints(key);
    if (points.size() < 2U)
    {
      ROS_ERROR_STREAM("Directed edge " << edgeName(key) << " has insufficient points");
      return StartExtension{};
    }
    const Pose2D &start = points.front();
    // 只检查机器人是否位于边起点之前，位于边内部的情况由projectToEdge处理。
    for (std::size_t index = 1; index < points.size(); ++index)
    {
      const double dx = points[index].x - start.x;
      const double dy = points[index].y - start.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared == 0.0)
      {
        continue;
      }
      const double fraction = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared;
      if (fraction >= 0.0)
      {
        return StartExtension{};
      }
      const double projection_x = start.x + fraction * dx;
      const double projection_y = start.y + fraction * dy;
      return StartExtension{
          true,
          std::hypot(point.x - projection_x, point.y - projection_y),
          std::hypot(point.x - start.x, point.y - start.y)};
    }
    ROS_ERROR_STREAM("Directed edge " << edgeName(key) << " has zero geometric length");
    return StartExtension{};
  }

  std::vector<Pose2D> TagRouteGraph::remainingWaypoints(const Pose2D &point, const EdgeKey &key) const
  {
    const std::vector<Pose2D> points = edgePoints(key);
    if (points.size() < 2U)
    {
      return {};
    }
    if (projectToStartExtension(point, key).valid)
    {
      return points;
    }
    const EdgeProjection projection = projectToEdge(point, key);
    if (!std::isfinite(projection.distance) || projection.segment_index + 1U > points.size())
    {
      return {};
    }
    return std::vector<Pose2D>(points.begin() + static_cast<std::ptrdiff_t>(projection.segment_index + 1U), points.end());
  }

  bool TagRouteGraph::planFromPose(const Pose2D &pose, int goal, double node_snap_distance, double edge_snap_distance,
                                   std::vector<EdgeKey> *route) const
  {
    if (route == nullptr)
    {
      ROS_ERROR("planFromPose output pointer is null");
      return false;
    }
    route->clear();
    if (!valid_)
    {
      ROS_ERROR("Cannot plan with an invalid route graph");
      return false;
    }
    const auto goal_node = nodes_.find(goal);
    if (goal_node == nodes_.end())
    {
      ROS_ERROR("Target tag_%d is absent from graph", goal);
      return false;
    }
    if (!goal_node->second.targetable)
    {
      ROS_ERROR("Target tag_%d is localization-only", goal);
      return false;
    }

    int nearest_node = nodes_.begin()->first;
    double nearest_node_distance = std::numeric_limits<double>::infinity();
    for (const auto &item : nodes_)
    {
      const double distance = distance2D(pose, item.second.tag_pose);
      if (distance < nearest_node_distance)
      {
        nearest_node = item.first;
        nearest_node_distance = distance;
      }
    }
    if (nearest_node_distance <= node_snap_distance)
    {
      // 足够接近节点时直接以该节点作为最短路起点。
      double cost = 0.0;
      return shortestPath(nearest_node, goal, &cost, route);
    }

    struct Candidate
    {
      double lateral_distance;
      double remaining_cost;
      EdgeKey first_key;
      std::vector<EdgeKey> rest;
    };
    struct ExtensionCandidate
    {
      double lateral_distance;
      double source_distance;
      double route_cost;
      std::vector<EdgeKey> route;
    };
    std::vector<Candidate> candidates;
    std::vector<Candidate> interior_candidates;
    std::vector<ExtensionCandidate> extension_candidates;

    // 同时收集边内部投影和边起点延长线上的可行入口。
    for (const EdgeKey &key : edge_order_)
    {
      const RouteEdge &edge = edges_.at(key);
      const EdgeProjection projection = projectToEdge(pose, key);
      if (projection.distance <= edge_snap_distance)
      {
        double rest_cost = 0.0;
        std::vector<EdgeKey> rest;
        if (shortestPath(key.second, goal, &rest_cost, &rest))
        {
          Candidate candidate{projection.distance, (1.0 - projection.fraction) * edge.cost, key, rest};
          candidates.push_back(candidate);
          if (projection.fraction > 0.0 && projection.fraction < 1.0)
          {
            interior_candidates.push_back(candidate);
          }
        }
      }

      const StartExtension extension = projectToStartExtension(pose, key);
      if (extension.valid && extension.lateral_distance <= edge_snap_distance)
      {
        double route_cost = 0.0;
        std::vector<EdgeKey> extension_route;
        if (shortestPath(key.first, goal, &route_cost, &extension_route) && (extension_route.empty() || extension_route.front() == key))
        {
          extension_candidates.push_back(ExtensionCandidate{
              extension.lateral_distance, extension.source_distance, route_cost, std::move(extension_route)});
        }
      }
    }

    if (!interior_candidates.empty())
    {
      // 内部投影比端点投影更能确定机器人当前所在的边。
      candidates = interior_candidates;
    }
    else if (!extension_candidates.empty())
    {
      // 起点延长线候选先比较横向距离，再比较到源点距离和路线代价。
      const double nearest_lateral = std::min_element(
                                         extension_candidates.begin(), extension_candidates.end(),
                                         [](const ExtensionCandidate &a, const ExtensionCandidate &b)
                                         {
                                           return a.lateral_distance < b.lateral_distance;
                                         })
                                         ->lateral_distance;
      const auto best = std::min_element(
          extension_candidates.begin(), extension_candidates.end(),
          [nearest_lateral](const ExtensionCandidate &a, const ExtensionCandidate &b)
          {
            const bool a_allowed = a.lateral_distance <= nearest_lateral + 0.02;
            const bool b_allowed = b.lateral_distance <= nearest_lateral + 0.02;
            if (a_allowed != b_allowed)
            {
              return a_allowed;
            }
            return std::tie(a.source_distance, a.route_cost) < std::tie(b.source_distance, b.route_cost);
          });
      *route = best->route;
      return true;
    }

    if (candidates.empty())
    {
      route->clear();
      return false;
    }
    const double nearest_lateral = std::min_element(
                                       candidates.begin(), candidates.end(),
                                       [](const Candidate &a, const Candidate &b)
                                       {
                                         return a.lateral_distance < b.lateral_distance;
                                       })
                                       ->lateral_distance;

    const Candidate *best = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();
    // 在横向距离近似相同的候选中选择剩余总代价最小的路线。
    for (const Candidate &candidate : candidates)
    {
      if (candidate.lateral_distance > nearest_lateral + 0.02)
      {
        continue;
      }
      double cost = candidate.remaining_cost;
      for (const EdgeKey &key : candidate.rest)
      {
        cost += edges_.at(key).cost;
      }
      if (best == nullptr || cost < best_cost)
      {
        best = &candidate;
        best_cost = cost;
      }
    }
    route->clear();
    route->push_back(best->first_key);
    route->insert(route->end(), best->rest.begin(), best->rest.end());
    return true;
  }

} // namespace tag_graph_nav
