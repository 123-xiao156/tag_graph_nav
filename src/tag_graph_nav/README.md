# tag_graph_nav

ROS Noetic C++ implementation of the original Python directed AprilTag graph
navigator. The original Python files remain unchanged.

## Build

```bash
cd /home/iimt/dtx_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg tag_graph_nav
source devel/setup.bash
```

Run the pytest suite against the compiled C++ graph library:

```bash
catkin_make run_tests_tag_graph_nav
catkin_test_results build/test_results/tag_graph_nav
```

## Run

Start the robot drivers and AprilTag detector first. Their TF tree must provide
fresh `base_link <-> tag_<id>` transforms.

```bash
roslaunch tag_graph_nav tag_navigation.launch
```

Send a target Tag ID:

```bash
rostopic pub -1 /target_tag_id std_msgs/Int32 "data: 4"
```

The node publishes velocity to `/cmd_vel`, the fitted path to
`/tag_navigator/planned_path`, and the Tag-derived robot pose to
`/tag_navigator/estimated_pose`.

Override the map or topics from the launch command line when needed:

```bash
roslaunch tag_graph_nav tag_navigation.launch \
  route_map_file:=/absolute/path/to/tag_route_map.json \
  cmd_vel_topic:=/robot/cmd_vel
```
