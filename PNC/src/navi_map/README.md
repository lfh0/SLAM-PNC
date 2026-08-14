# navi_map

## 功能作用

`navi_map` 是导航系统的地图准备与维护包，负责把静态栅格地图或 PCD 点云整理成规划可使用的地图，并提供任务点标注工具。主要工作包括：

- 加载二维静态地图，生成膨胀后的全局地图及车辆附近的局部地图；
- 读取、滤波 PCD 点云，并通过 `octomap_server` 转换为 OctoMap/二维投影地图；
- 使用带法向约束的 RANSAC 将 PCD 分为地面点和非地面点；
- 在 RViz 中依次标注任务点，并保存任务点之间的连接关系；
- 基于 `costmap_2d` 组合静态地图、实时点云和膨胀层；
- 将 PGM 地图转换成“仅保留障碍物”的 `OccupancyGrid`。

## 节点与输入输出

### `map_maintain_node`

源码：`src/MapMaintain.cpp`

对静态地图中的占用栅格进行膨胀，并依据当前里程计位置裁剪局部地图。节点以 20 Hz 发布地图，每 100 个循环重新计算一次全局膨胀结果。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/merged_map` | `nav_msgs/OccupancyGrid` | 原始静态地图，话题名在源码中固定 |
| 订阅 | `navi_map/mapmaintain/odom_topic` 参数指定的话题 | `nav_msgs/Odometry` | 车辆在地图坐标系中的当前位置，默认 `/lio/odom` |
| 发布 | `/global_map` | `nav_msgs/OccupancyGrid` | 障碍物膨胀后的全局二值地图 |
| 发布 | `/local_map` | `nav_msgs/OccupancyGrid` | 以车辆位置附近区域裁剪出的局部膨胀地图 |

主要参数：

| 参数 | `missionmarker.launch` 配置值 | 源码缺省值 | 说明 |
| --- | --- | --- | --- |
| `navi_map/mapmaintain/inflation_radius` | `0.3` | `0.7` | 障碍物膨胀距离，单位 m |
| `navi_map/mapmaintain/local_map_size` | `8.0` | `5.0` | 局部地图边长，单位 m |
| `navi_map/frame/mapFrame` | `map` | `map` | 输出地图的坐标系 |
| `navi_map/mapmaintain/odom_topic` | `/lio/odom` | `/lio/odom` | 里程计输入话题 |

### `read_pcd_node`

源码：`src/readPCD.cpp`

从磁盘读取 PCD 点云，进行统计离群点滤波，并以 10 Hz 发布 ROS 点云。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 文件输入 | `navi_map/map_dir/pcd_file_path` | PCD 文件 | 原始 `pcl::PointXYZ` 点云 |
| 发布 | `navi_map/output/readpcd_topic` 参数指定的话题 | `sensor_msgs/PointCloud2` | 滤波后的点云，示例配置为 `/readpcd` |

主要参数：

| 参数 | 说明 |
| --- | --- |
| `navi_map/frame/pointcloudFrame` | 输出点云的 `frame_id` |
| `navi_map/OutlierRemoval/meanK` | 统计离群点滤波的邻域点数 |
| `navi_map/OutlierRemoval/Thresh` | 标准差倍数阈值 |

### `ransac_ground_node`

源码：`src/groud_ransac.cpp`

读取 PCD 后先按传感器高度截取候选地面区域并移除统计离群点，再使用法向接近 Z 轴的 RANSAC 平面模型，将完整输入点云分为地面和非地面两部分。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 文件输入 | `navi_map/map_dir/pcd_file_path` | PCD 文件 | 待分割的完整点云 |
| 文件输出 | `navi_map/map_dir/pcd_ground_out_path` | PCD 文件 | 分割得到的地面点 |
| 文件输出 | `navi_map/map_dir/pcd_non_ground_out_path` | PCD 文件 | 分割得到的非地面点 |
| 发布 | `navi_map/OutlierRemoval/topic` 参数指定的话题 | `sensor_msgs/PointCloud2` | 高度窗截取并去除离群点后的 RANSAC 候选点云，示例为 `/remove_outliers/pointcloud` |

> 注意：ROS 话题输出不是最终分割出的 `ground` 点云；最终地面和非地面结果保存在两个 PCD 文件中。

其余参数包括 `navi_map/sensor/height`、`z_window`、`dist_thresh`、`max_tilt_deg`、`max_iters`、`meanK` 和 `Thresh`，分别控制候选地面高度、RANSAC 距离/角度/迭代阈值及离群点滤波。

### `navi_map_missionmaker`

源码：`src/missionmaker.cpp`

接收 RViz 的“2D Nav Goal”，按点击顺序生成任务点。相邻点会建立双向连接，节点正常退出时将结果覆盖写入 `config/mission.yaml`。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 中点击的任务点位姿 |
| 发布 | `/mission_points` | `geometry_msgs/PoseArray` | 已标注任务点的可视化集合，坐标系为 `map` |
| 文件输出 | `config/mission.yaml` | YAML | 任务点 ID、`[x, y, yaw]`、停车标志和相邻点 ID |

当前实现将每个任务点的 `isStop` 固定为 `true`，连接关系仅按点击顺序生成。

### `obstacle_only_map_publisher.py`

读取 P2/P5 格式的 PGM 地图，将超过占用阈值的像素设为 `100`，其余像素（包括原地图中的未知区域）均设为已知空闲 `0`。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 文件输入 | `~map_yaml`，或 `~image` 及地图参数 | YAML/PGM | 地图图片、分辨率、原点和占用阈值 |
| 发布 | `~topic` | `nav_msgs/OccupancyGrid` | 仅包含障碍物和空闲区域的地图，默认 `/map` |

私有参数：`~frame_id`（默认 `map`）、`~publish_rate`（默认 `1.0` Hz）、`~latch`（默认 `true`）、`~resolution`、`~origin`、`~negate` 和 `~occupied_thresh`。当 `~publish_rate <= 0` 时只发布一次，并依靠锁存保留消息。

## Launch 文件

| 文件 | 作用 | 主要数据流 |
| --- | --- | --- |
| `launch/missionmarker.launch` | 加载 `lab10flour` 地图，启动地图维护和任务点标注 | `/merged_map` + `/lio/odom` -> `/global_map`、`/local_map`；RViz 目标点 -> `config/mission.yaml` |
| `launch/octomap_convert.launch` | 读取 PCD 并交给 `octomap_server` | PCD -> `/readpcd` -> OctoMap 及 `/projected_map` |
| `launch/real_map.launch` | 将实时 LIO 点云转换为 OctoMap | `/lio/cloud_imu` -> OctoMap 及 `/projected_map` |
| `launch/global_inflation_local_obstacle.launch` | 启动全局/局部 `costmap_2d` | `/map` 用于全局静态层，`/lio/cloud_imu` 用于局部障碍层 |
| `launch/ground.launch` | 启动离线地面分割及 RViz | PCD -> 地面/非地面 PCD |
| `launch/save_map.launch` | 保存 OctoMap 的二维投影 | `/projected_map` -> `maps/sim.pgm` 和 `maps/sim.yaml` |

其中 `global_inflation_local_obstacle.launch` 启动的外部 `costmap_2d_node` 主要发布：

- `/global_costmap_node/costmap/costmap`：融合静态层与膨胀层的全局代价地图，供前端搜索和轨迹优化使用；
- `/local_costmap_node/costmap/costmap`：融合实时点云障碍层与膨胀层的滚动局部代价地图。

### TF 前提

- `octomap_convert.launch` 中 `/readpcd` 按当前 YAML 标记为 `world`，而 `octomap_server` 的目标帧为 `map`，运行环境必须提供可用的 `world -> map` TF；也可以将 `navi_map/frame/pointcloudFrame` 改成与点云数据实际一致且可转换到 `map` 的坐标系。
- `real_map.launch` 需要 `/lio/cloud_imu` 消息坐标系到 `map` 的 TF，当前配置预期传感器基准帧为 `imu`。
- 全局和局部 costmap 的 `global_frame` 为 `map`、`robot_base_frame` 为 `robot`，因此需要连续可用的 `map -> robot` TF；局部点云障碍层还需要 `imu` 到 `map` 的变换。
- `map_maintain_node` 不使用 TF，仍要求里程计位置本身已位于地图坐标系中。

启动示例：

```bash
catkin_make --pkg navi_map
source devel/setup.bash
roslaunch navi_map missionmarker.launch
```

仅运行 PGM 障碍物地图发布器：

```bash
rosrun navi_map obstacle_only_map_publisher.py \
  _map_yaml:="$(rospack find navi_map)/maps/sim.yaml" \
  _topic:=/map
```

## 配置与数据文件

- `config/missionmarker_map.yaml`：地图维护、PCD 读入及 RANSAC 参数。文件中的 PCD 路径是本机绝对路径，换环境后必须修改。
- `config/global_inflation_local_obstacle.yaml`：全局和局部 `costmap_2d` 图层配置。
- `config/mission.yaml`、`config/sim.yaml`：任务点图结构；每个点包含 `id`、`point: [x, y, yaw]`、`isStop` 和 `conPts`。
- `maps/`：二维 PGM/PNG 地图及其 YAML 元数据。
- `PCD/`：示例或处理后的点云文件。

## 当前实现注意事项

- `ground.launch` 当前加载的是任务点文件 `config/sim.yaml`，其中没有 `ransac_ground_node` 所需的 `navi_map/...` 参数。直接启动前应改为加载 `config/missionmarker_map.yaml`，或自行向参数服务器提供完整参数。
- `global_inflation_local_obstacle.launch` 将 `/use_sim_time` 设为 `true`；脱离仿真运行时需要有效的 `/clock`，或在启动前调整该设置。
- `msg/StateMsg.msg` 目前没有在 `CMakeLists.txt` 中注册消息生成，也没有被本包节点使用，因此当前不能作为已构建的 ROS 消息接口使用。
- `map_maintain_node` 直接使用里程计中的位置裁剪地图，不执行 TF 坐标变换；输入里程计位置必须已经与静态地图处于同一坐标系，并应避免局部窗口超出全局地图边界。
- `octomap_server`、`map_server`、`costmap_2d`、RViz 和 Python 脚本依赖属于运行环境要求，当前 `package.xml` 未完整声明这些启动期依赖。
