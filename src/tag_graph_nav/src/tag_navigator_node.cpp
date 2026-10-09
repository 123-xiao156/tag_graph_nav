#include "tag_graph_nav/tag_navigator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_navigator");
  tag_graph_nav::TagNavigator navigator;
  if (!navigator.IsInitialized())
  {
    ROS_ERROR("Failed to initialize tag navigator");
    return 1;
  }
  // 订阅回调由独立线程处理，主线程负责串行执行导航控制循环。
  ros::AsyncSpinner spinner(1);
  spinner.start();
  navigator.Run();
  return 0;
}
