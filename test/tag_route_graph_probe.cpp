#include "tag_graph_nav/tag_route_graph.hpp"

#include <ros/ros.h>

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
bool parseInt(const char *value, int *number)
{
  if (value == nullptr || number == nullptr)
  {
    ROS_ERROR("parseInt received a null pointer");
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const long parsed_number = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || parsed_number < INT_MIN || parsed_number > INT_MAX)
  {
    ROS_ERROR("Invalid integer: %s", value);
    return false;
  }
  *number = static_cast<int>(parsed_number);
  return true;
}

bool parseDouble(const char *value, double *number)
{
  if (value == nullptr || number == nullptr)
  {
    ROS_ERROR("parseDouble received a null pointer");
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const double parsed_number = std::strtod(value, &end);
  if (errno != 0 || end == value || *end != '\0' || !std::isfinite(parsed_number))
  {
    ROS_ERROR("Invalid floating-point number: %s", value);
    return false;
  }
  *number = parsed_number;
  return true;
}

bool parseRoute(int argc, char **argv, int first_argument, std::vector<tag_graph_nav::EdgeKey> *route)
{
  if (route == nullptr)
  {
    ROS_ERROR("parseRoute output pointer is null");
    return false;
  }
  route->clear();
  for (int index = first_argument; index < argc; ++index)
  {
    const std::string value(argv[index]);
    const std::size_t separator = value.find(':');
    if (separator == std::string::npos)
    {
      ROS_ERROR("Edge must use source:destination format: %s", value.c_str());
      return false;
    }
    int source = 0;
    int destination = 0;
    if (!parseInt(value.substr(0, separator).c_str(), &source) || !parseInt(value.substr(separator + 1U).c_str(), &destination))
    {
      return false;
    }
    route->emplace_back(source, destination);
  }
  return true;
}

void printRoute(const std::vector<tag_graph_nav::EdgeKey> &route)
{
  for (std::size_t index = 0; index < route.size(); ++index)
  {
    if (index != 0U)
    {
      std::cout << ',';
    }
    std::cout << route[index].first << ':' << route[index].second;
  }
  std::cout << '\n';
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_route_graph_probe", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  if (argc < 3)
  {
    ROS_ERROR("Usage: tag_route_graph_probe ROUTE_FILE OPERATION [ARGS...]");
    return 1;
  }

  const tag_graph_nav::TagRouteGraph graph(argv[1]);
  if (!graph.isValid())
  {
    ROS_ERROR("Route graph probe cannot load the route map");
    return 2;
  }
  const std::string operation(argv[2]);
  std::cout << std::setprecision(17);

  if (operation == "summary" && argc == 3)
  {
    std::cout << graph.frameId() << ' ' << graph.nodes().size() << ' ' << graph.edges().size() << '\n';
    return 0;
  }
  if (operation == "shortest" && argc == 5)
  {
    int start = 0;
    int goal = 0;
    if (!parseInt(argv[3], &start) || !parseInt(argv[4], &goal))
    {
      return 2;
    }
    double cost = 0.0;
    std::vector<tag_graph_nav::EdgeKey> route;
    if (!graph.shortestPath(start, goal, &cost, &route))
    {
      std::cout << "UNREACHABLE\n";
      return 0;
    }
    std::cout << cost << ' ';
    printRoute(route);
    return 0;
  }
  if (operation == "plan" && argc == 9)
  {
    tag_graph_nav::Pose2D pose;
    int goal = 0;
    double node_snap_distance = 0.0;
    double edge_snap_distance = 0.0;
    if (!parseDouble(argv[3], &pose.x) || !parseDouble(argv[4], &pose.y) || !parseDouble(argv[5], &pose.yaw) ||
        !parseInt(argv[6], &goal) || !parseDouble(argv[7], &node_snap_distance) || !parseDouble(argv[8], &edge_snap_distance))
    {
      return 2;
    }
    std::vector<tag_graph_nav::EdgeKey> route;
    if (!graph.planFromPose(pose, goal, node_snap_distance, edge_snap_distance, &route))
    {
      std::cout << "UNREACHABLE\n";
      return 0;
    }
    printRoute(route);
    return 0;
  }
  if (operation == "target" && argc == 4)
  {
    int tag_id = 0;
    if (!parseInt(argv[3], &tag_id))
    {
      return 2;
    }
    const tag_graph_nav::Pose2D pose = graph.tagTargetPose(tag_id);
    std::cout << pose.x << ' ' << pose.y << ' ' << pose.yaw << '\n';
    return 0;
  }
  if (operation == "line_yaw" && argc >= 4)
  {
    std::vector<tag_graph_nav::EdgeKey> route;
    if (!parseRoute(argc, argv, 3, &route))
    {
      return 2;
    }
    std::cout << graph.routeLineYaw(route) << '\n';
    return 0;
  }
  if (operation == "remaining" && argc == 8)
  {
    tag_graph_nav::Pose2D pose;
    int source = 0;
    int destination = 0;
    if (!parseDouble(argv[3], &pose.x) || !parseDouble(argv[4], &pose.y) || !parseDouble(argv[5], &pose.yaw) ||
        !parseInt(argv[6], &source) || !parseInt(argv[7], &destination))
    {
      return 2;
    }
    const std::vector<tag_graph_nav::Pose2D> points = graph.remainingWaypoints(pose, {source, destination});
    for (std::size_t index = 0; index < points.size(); ++index)
    {
      if (index != 0U)
      {
        std::cout << ';';
      }
      std::cout << points[index].x << ',' << points[index].y << ',' << points[index].yaw;
    }
    std::cout << '\n';
    return 0;
  }

  ROS_ERROR("Invalid operation or argument count: %s", operation.c_str());
  return 1;
}
