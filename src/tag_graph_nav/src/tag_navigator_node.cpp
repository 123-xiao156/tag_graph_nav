#include "tag_graph_nav/tag_navigator.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_navigator");
  tag_graph_nav::TagNavigator navigator;
  if (!navigator.isInitialized())
  {
    ROS_FATAL("Failed to initialize tag navigator");
    return 1;
  }
  ros::AsyncSpinner spinner(1);
  spinner.start();
  navigator.run();
  return 0;
}
