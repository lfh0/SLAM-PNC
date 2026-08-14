# front2end_search

## 功能概述

`front2end_search` 是 ROS 1 全局前端路径搜索包。包内的 `planning_server` 节点接收二维占据栅格、车辆里程计和 RViz 目标位姿，以里程计位姿作为起点，运行 A*、障碍物混合 A*、JPS 或 RRT*，并发布 `nav_msgs/Path` 供轨迹优化、控制和 RViz 使用。

本包完成的主要工作包括：

- 统一管理地图、里程计和目标点输入，并在地图更新后同步四个搜索器。
- 提供四种二维全局路径搜索实现及统一的 ROS 发布接口。
- 将栅格代价值同时用于硬碰撞判断和软代价计算，使路径倾向远离高代价膨胀区。
- 为输出路径补全朝向，并发布起点、终点方向箭头用于 RViz 检查。
- 输出每次目标请求、地图同步、搜索、路径构建和发布的耗时日志。

## 节点与可执行程序

| 节点名 | 可执行程序 | 作用 |
| --- | --- | --- |
| `planning_server` | `planning_server` | 加载四种搜索器，等待地图，接收里程计和目标点，执行所选搜索并发布路径 |

节点启动后会在构造阶段等待第一帧有效地图。在收到地图前，节点不会进入正常规划服务状态。

## 处理流程

1. 读取 ROS 参数，创建 A*、障碍物混合 A*、JPS、RRT 搜索器，并预分配搜索节点。
2. 订阅地图并等待首帧 `nav_msgs/OccupancyGrid`。
3. 持续保存最新里程计；地图更新时仅标记为待同步。
4. 收到 `/move_base_simple/goal` 后，读取最新里程计作为规划起点。尚未收到里程计时，本次目标会被忽略。
5. 如地图已更新，将新地图同步给全部搜索器，再按照 `planner/search_type` 串行执行搜索。
6. 搜索成功后构造 `nav_msgs/Path`：起点朝向按前视点计算，中间点朝向按邻近路径方向计算，最后一点保留目标消息中的四元数。
7. 发布路径和起终点箭头，并输出带 `seq` 的分阶段耗时日志。

节点不会对输入做 TF 变换。地图、里程计位置和目标位置必须已经处于同一坐标系；输出路径固定声明为 `map` 坐标系。

## 规划算法

| `planner/search_type` | 算法 | 路径话题 | 当前实现 |
| --- | --- | --- | --- |
| `0` | 全部算法 | 四个路径话题 | 按 A*、JPS、RRT*、障碍物混合 A* 的顺序串行运行 |
| `1` | A* | `/astar_path` | 八邻域栅格 A*，欧氏距离启发，支持限制搜索窗口和逐级扩大窗口重试 |
| `2` | 障碍物混合 A* | `/obs_hybridastar_path` | 自行车模型运动原语，支持前进、倒车及转向代价，接近目标时用 Reeds-Shepp 曲线解析连接 |
| `3` | JPS | `/jps_path` | 八方向跳点搜索，通过自然邻居和强制邻居剪枝 |
| `4` | RRT* | `/rrt_path` | 目标偏置随机采样、邻域最优父节点选择和重布线；类名及话题沿用 `RRT` |

非法的 `search_type` 会打印警告并回退为运行全部算法。

### 栅格与代价规则

- 地图外视为不可通行。
- 当 `unknown_as_occupied=true` 时，值为 `-1` 的未知栅格视为障碍。
- 栅格值大于等于 `occupied_threshold` 时视为硬障碍。
- `[0, occupied_threshold)` 内的非负代价值按归一化值的平方乘以 `obstacle_cost_weight`，作为软障碍代价加入路径长度。
- 随包配置的阈值为 `80`，因此 `80` 及以上为硬障碍，`1` 至 `79` 为可通行软代价区。

### 算法实现说明

- **A***：每次目标最多按 `search_window_retry_margins` 配置的边界余量重试。搜索窗口是起终点包围盒向四周扩展指定米数后与地图边界的交集。
- **JPS**：跳跃段上的每个栅格都参与软代价累计。代码创建了开放集、关闭集、路径和跳点可视化发布器，但正常搜索中的调用当前被注释。
- **RRT***：碰撞检测按地图分辨率沿边采样，边代价也沿边累计软栅格代价；`search_radius` 同时用于选择更优父节点和重布线。
- **障碍物混合 A***：运动原语使用车轮距和转角传播 `x/y/yaw`，弧长包含前进与倒车。距终点小于 `3 m` 时尝试以 `1/max_cur` 为转弯半径的 Reeds-Shepp 连接，并按 `checkl` 采样检查。当前扩展节点的哈希键只使用二维位置栅格，未使用 yaw 索引；活动搜索中的碰撞检查使用路径中心点，已实现的车辆矩形轮廓碰撞函数未被主搜索调用。

## ROS 输入

下列绝对话题名按启动文件未设置节点命名空间时的默认解析结果书写；若在外部命名空间中启动节点，源码中的相对发布话题会随命名空间变化。

### 订阅话题

| 话题 | 消息类型 | 队列 | 作用 |
| --- | --- | ---: | --- |
| `search/map_topic`，随包配置为 `/global_costmap_node/costmap/costmap` | `nav_msgs/OccupancyGrid` | 10 | 全局二维代价地图；首帧用于初始化，后续帧在下次目标到来时同步 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 1 | 规划终点位置及终点朝向，通常由 RViz 的“2D Nav Goal”发送 |
| `planner/odom_topic`，随包配置为 `/lio/robo/odom` | `nav_msgs/Odometry` | 10 | 提供规划起点位置和起始 yaw |

输入只使用位置和朝向字段。代码不会检查或转换消息中的 `header.frame_id`。

## ROS 输出

### 正常发布话题

| 话题 | 消息类型 | 队列/锁存 | 发布条件与内容 |
| --- | --- | --- | --- |
| `/astar_path` | `nav_msgs/Path` | 10，非锁存 | A* 被选中且返回成功时发布 |
| `/obs_hybridastar_path` | `nav_msgs/Path` | 10，非锁存 | 障碍物混合 A* 被选中后发布搜索结果 |
| `/jps_path` | `nav_msgs/Path` | 10，非锁存 | JPS 被选中且到达终点时发布 |
| `/rrt_path` | `nav_msgs/Path` | 10，非锁存 | RRT* 被选中且到达终点时发布 |
| `/front_path_endpoints` | `visualization_msgs/MarkerArray` | 1，锁存 | 非空路径发布前，先清空旧标记，再发布青色起点箭头和品红色终点箭头 |

四种路径的 `header.frame_id` 均固定为 `map`，二维位置的 `z` 为 `0`。路径首点 yaw 使用 `planner/path_yaw_lookahead_distance` 换算出的采样偏移计算；末点朝向严格使用目标消息的四元数。

当 `search_type=0` 时，四个算法依次发布各自路径；`/front_path_endpoints` 每次都会先发送 `DELETEALL`，因此最终只保留最后一个非空路径的起终点标记。

### 已创建但当前正常流程不发布的话题

| 话题 | 消息类型 | 当前状态 |
| --- | --- | --- |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | 发布辅助函数存在，但没有调用点 |
| `/jps/open_set` | `visualization_msgs/Marker` | 发布器已创建，搜索循环中的可视化调用已注释 |
| `/jps/close_set` | `visualization_msgs/Marker` | 发布器已创建，搜索循环中的可视化调用已注释 |
| `/jps/path` | `visualization_msgs/Marker` | 发布器已创建，搜索循环中的可视化调用已注释 |
| `/jps/jump_points` | `visualization_msgs/Marker` | 发布器已创建，搜索循环中的可视化调用已注释 |

### 服务与动作

本包当前不提供 ROS Service、Action，也不定义自定义消息。规划由目标话题触发。

## 参数

`launch/navigation.launch` 会分别加载 `planning_server.yaml`、`astar.yaml`、`jps.yaml`、`obs_hybridastar.yaml` 和 `rrt.yaml`。下表的“配置值”是随包启动文件实际使用的值；“源码缺省值”仅在参数未加载时生效。

### 节点与通用搜索参数

| 参数 | 配置值 | 源码缺省值 | 作用 |
| --- | ---: | ---: | --- |
| `planner/search_type` | `1` | `0` | 选择算法，取值见上文算法表 |
| `planner/odom_topic` | `/lio/robo/odom` | `/lio/robo/odom` | 起点里程计话题 |
| `planner/path_yaw_lookahead_distance` | `0.3` | `0.3` | 计算路径首点朝向的前视距离，单位 m |
| `search/map_topic` | `/global_costmap_node/costmap/costmap` | `/projected_map` | 地图订阅话题 |
| `{astar,jps,obs_hybridastar}/lambda_heu` | `1.0001` | A* `1.0001`、JPS `1.0`、障碍物混合 A* `1.5` | 各搜索器独立的启发项权重 |
| `{astar,jps,obs_hybridastar,rrt}/occupied_threshold` | `80` | A* `99`、其他 `50` | 各搜索器独立的硬障碍阈值 |
| `{astar,jps,obs_hybridastar,rrt}/obstacle_cost_weight` | `5.0` | `5.0` | 软栅格代价权重 |
| `{astar,jps,obs_hybridastar,rrt}/unknown_as_occupied` | `true` | `true` | 是否将未知栅格视为障碍 |
| `{astar,jps,obs_hybridastar}/allocate_num` | `3000000` | `500000` | 各搜索器独立的预分配节点数 |
| `{astar,jps,obs_hybridastar}/max_search_time` | `50000.0` | A*/JPS `5000.1`、障碍物混合 A* `3000.1` | 搜索时间上限，单位 ms |

### A* 搜索窗口

| 参数 | 配置值 | 源码缺省值 | 作用 |
| --- | ---: | ---: | --- |
| `astar/use_search_window` | `true` | `true` | 是否限制 A* 只在起终点附近矩形窗口内扩展 |
| `astar/search_window_margin` | `20.0` | `20.0` | A* 初始窗口余量，单位 m |
| `astar/search_window_retry_margins` | `[20.0, 50.0, 100.0]` | 同左 | A* 失败后的窗口余量重试序列，单位 m |

### 障碍物混合 A* 参数

| 参数 | 配置值 | 源码缺省值 | 当前用途 |
| --- | ---: | ---: | --- |
| `obs_hybridastar/step_arc` | `1.0` | `1.0` | 运动原语弧长基值，生成正负 `0.5` 倍和 `1.0` 倍弧长 |
| `obs_hybridastar/check_num` | `5` | `5` | 每条运动原语的碰撞与软代价采样段数 |
| `obs_hybridastar/checkl` | `0.2` | `0.2` | Reeds-Shepp 连接的采样间距，单位 m |
| `obs_hybridastar/traj_forward_penalty` | `1.0` | `1.0` | 前进弧长代价系数 |
| `obs_hybridastar/traj_back_penalty` | `5.0` | `5.0` | 倒车弧长代价系数 |
| `obs_hybridastar/traj_gear_switch_penalty` | `0.0` | `0.0` | 前进/倒车切换惩罚 |
| `obs_hybridastar/traj_steer_penalty` | `0.2` | `0.2` | 转角乘弧长惩罚 |
| `obs_hybridastar/traj_steer_change_penalty` | `0.0` | `0.0` | 相邻运动原语转角变化惩罚 |
| `obs_hybridastar/max_cur` | `0.3` | `0.3` | Reeds-Shepp 状态空间最大曲率，转弯半径为其倒数 |
| `obs_hybridastar/max_vel` | `0.5` | `0.5` | 仅用于计算解析连接的估计时长 |
| `obs_hybridastar/horizon` | `50.0` | `50.0` | 已读取，当前主搜索未使用 |
| `obs_hybridastar/yaw_resolution` | `0.3` | `0.3` | yaw 离散分辨率 |
| `obs_hybridastar/max_acc` | `0.3` | `0.3` | 最大加速度 |
| `obs_hybridastar/time_resolution` | `0.1` | `0.1` | 时间分辨率 |
| `obs_hybridastar/distance_resolution` | `0.5` | `0.5` | 距离分辨率 |
| `obs_hybridastar/velocity_resolution` | `0.5` | `0.5` | 速度分辨率 |

### RRT* 参数

| 参数 | 配置值 | 源码缺省值 | 作用 |
| --- | ---: | ---: | --- |
| `rrt/step_size` | `1.0` | `1.0` | 单次扩展最大长度，单位 m |
| `rrt/goal_bias` | `0.7` | `0.7` | 直接采样目标点的概率 |
| `rrt/search_radius` | `3.0` | `3.0` | 最优父节点搜索和重布线半径，单位 m |
| `rrt/max_search_time` | `50000.0` | `50000.0` | 搜索时间上限，单位 ms |
| `rrt/max_iterations` | `500000` | `50000` | 最大采样迭代数 |

### 车辆参数

| 参数 | 配置值 | 源码缺省值 | 当前用途 |
| --- | ---: | ---: | --- |
| `obs_hybridastar/vehicle/car_wheelbase` | `0.8` | `0.8` | 自行车模型轴距，单位 m |
| `obs_hybridastar/vehicle/car_max_steering_angle` | `45.0` | `45.0` | 最大转角，输入单位为度 |
| `obs_hybridastar/vehicle/car_length` | `1.0` | `1.0` | 车辆长度 |
| `obs_hybridastar/vehicle/car_width` | `0.6` | `0.6` | 车辆宽度 |
| `obs_hybridastar/vehicle/car_d_cr` | `0.0` | `0.0` | 车辆几何中心相对参考点偏移 |

## 启动方式

### 编译

在当前 catkin 工作空间根目录执行：

```bash
catkin_make --pkg front2end_search
source devel/setup.bash
```

当前工作空间的 `build` 目录由 `catkin_make` 创建，不要在同一 `build`/`devel` 目录中直接混用 `catkin build`。

### 启动完整导航链路

```bash
roslaunch front2end_search navigation.launch
```

`navigation.launch` 的默认行为如下：

- 分别加载五个前端参数 YAML 文件。
- 启动 `navi_map/launch/global_inflation_local_obstacle.launch` 提供代价地图。
- 启动 `trajopt/launch/trajopt.launch`。
- 启动 `controller/launch/purepursuit.launch`。
- 启动 RViz，并加载 `navi_map/rviz/navi_2d.rviz`。RViz 节点设置了 `required=true`，退出 RViz 会结束该 launch。
- 启动本包的 `planning_server`。

可通过 launch 参数关闭外围模块，例如只启动本包节点和 RViz，并由外部系统提供地图与里程计：

```bash
roslaunch front2end_search navigation.launch \
  start_costmap:=false trajopt:=false controller:=false
```

### 单独启动规划节点

先确保地图话题已经存在，再执行：

```bash
for file in planning_server astar jps obs_hybridastar rrt; do
  rosparam load "$(rospack find front2end_search)/config/${file}.yaml"
done
rosrun front2end_search planning_server
```

随后由里程计发布节点提供起点，在 RViz 中发送 `/move_base_simple/goal` 即可触发规划。

## 依赖关系

编译依赖：

- ROS/catkin：`roscpp`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`tf`、`visualization_msgs`
- 系统库：Eigen3、OMPL
- 包内第三方代码：`thirdparty/odeint-v2`，当前 `planning_server` 构建目标未直接引用

完整 `navigation.launch` 还依赖工作空间中的 `navi_map`、`trajopt`、`controller` 和 ROS `rviz` 包。

## 目录说明

```text
front2end_search/
├── config/                         # 规划服务及四种搜索器的独立 YAML
├── include/path_searching/           # 四种搜索器头文件及栅格射线工具
├── launch/navigation.launch          # 完整导航链路启动文件
├── src/app/PlanningServer.cpp        # ROS 节点、接口编排和路径发布
├── src/search/                        # A*、JPS、障碍物混合 A*、RRT* 实现
└── thirdparty/odeint-v2/              # 随包保存的第三方 ODEInt 源码
```

## 当前实现注意事项

- A* 达到 `max_search_time` 时当前实现返回 `REACH_END`，但未必已经构造路径，因此可能发布空的 `/astar_path`。
- 障碍物混合 A* 的状态枚举值均为非零，而上层仅用 `if (!status)` 判断失败；当前搜索失败或超时时仍可能发布空的 `/obs_hybridastar_path`。
- 障碍物混合 A* 的车辆轮廓碰撞检查未接入活动搜索，当前仅检查路径中心采样点。使用未充分膨胀的地图时不能保证车体无碰撞。
- `astar/allocate_num`、`jps/allocate_num` 和 `obs_hybridastar/allocate_num` 会让各搜索器在启动时预分配大量节点，需要预留足够内存。
- `package.xml` 当前只声明 ROS 消息依赖；Eigen3、OMPL 需由环境预装并由 `CMakeLists.txt` 查找，完整 launch 使用的工作空间包也未写入包清单。
