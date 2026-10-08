# AprilTag 有向图导航

入口为 `tag_nav.py`。启动前设置 ROS 私有参数 `~route_map_file`，指向现场测绘的 `tag_route_map.json`；程序收到 `/target_tag_id` (`std_msgs/Int32`) 后规划并执行。`~planned_path` 发布整条路线拟合出的有向直线，`~estimated_pose` 发布视觉定位结果。

只维护这一份现场地图。Tag 0～4 可作为目标；Tag 5、6 标定后写入同一文件，但只用于视觉定位，不创建通往它们的边。

路网字段：

- `frame_id`：所有坐标共同使用的坐标系，例如 `map`。
- `nodes[].id`：唯一 Tag ID，允许不连续。
- `nodes[].targetable`：能否作为导航目标，省略时为 `true`；仅定位 Tag 设置为 `false`。
- `nodes[].tag_pose`：已测量的 Tag 坐标系原点在 `frame_id` 中的 `x/y/yaw`；必须与相机发布的 `tag_N` 坐标轴方向一致。机器人以 Tag 原点为目标位置，以 Tag 的 +Y 轴为目标朝向。
- `edges[].source/destination`：允许通行的方向；需要双向通行时填写两条边。
- `edges[].cost`：正数，Dijkstra 搜索累计代价。
- `edges[].speed_limit`：该边线速度上限，单位 m/s。
- `edges[].motion`：`forward` 或 `reverse`，表示沿该有向边前进或倒车；省略时为 `forward`。
- `edges[].waypoints`：可选的中间点列表，各点具有 `x/y/yaw`。曲线路段应增加中间点，避免直接切过弯道。

机器人每次使用最新的 Tag TF 估计在图中的位姿。Tag 在 `base_link` 原点附近时也允许用于定位。若没有新鲜的视觉定位，控制器立即发布零速度；超过 `~max_lost_count` 后任务失败。默认 TF 最大年龄为 0.5 秒。到达判定默认位置误差 0.1 m、朝向误差 0.1 rad，可用 `~arrival_tolerance` 和 `~angle_tolerance` 调整。若相机在贴近 Tag 时无法继续识别，机器人会停车并可能报告失败；现场应检查相机最近识别距离，必要时调整停车容差或加入里程计融合。

## 自动标定 Tag 位姿

`tag_pose_calibrator.py` 直接读取当前可见 Tag 的 `map -> tag_N` TF，
对 Tag 0～6 累积新鲜样本，使用 MAD 过滤位置和偏航角离群值。所有 Tag
达到目标样本数后，脚本更新 `nodes[].tag_pose`；若 5、6 尚不存在则直接
新增到原文件并设置 `targetable: false`。原有 `edges` 会被保留，并原子写入
`tag_route_map.json`。覆盖前会在
同一目录生成带时间戳的 `.bak` 备份。脚本不会发布速度指令，需要人工或
其他安全方式让机器人缓慢经过所有 Tag。

使用方法：

```bash
source /opt/ros/noetic/setup.bash
python3 /home/iimt/20260804/tag_pose_calibrator.py
```

输入文件、输出文件、每个 Tag 的采样数量和默认标定编号固定配置在
`tag_pose_calibrator.py` 顶部的 `ROUTE_MAP_FILE`、`OUTPUT_FILE`、
`SAMPLES_PER_TAG` 和 `TAG_IDS`，需要调整时直接修改这些常量。当前默认
标定 Tag 0～6，其中 5、6 只用于定位。

脚本不再检查 `/amcl_pose`、定位协方差或 Tag 距离；只要目标 Tag 的 TF
存在并持续更新，就会采样。常用私有参数：

- `~tag_ids`：临时覆盖默认标定 ID，例如 `[0, 1, 2]`；默认使用源码顶部
  `TAG_IDS` 中配置的 `[0, 1, 2, 3, 4, 5, 6]`。
- `~tag_frame_format`：Tag TF 名称格式，默认 `tag_%d`。
- `~sample_rate`：检查 TF 的频率，默认 10 Hz。
- `~max_tag_age`：Tag TF 最大年龄，默认 0.5 s。
- `~max_xy_rms`：过滤后位置均方根误差上限，默认 0.05 m。
- `~max_yaw_rms`：过滤后偏航角均方根误差上限，默认 0.10 rad。

例子：

```bash
rosparam set /tag_navigator/route_map_file /absolute/path/to/site_tag_route_map.json
python3 /home/iimt/20260804/tag_nav.py
rostopic pub -1 /target_tag_id std_msgs/Int32 'data: 4'
```
