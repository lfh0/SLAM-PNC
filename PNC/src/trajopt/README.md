# trajopt 功能包

## 1. 功能概述

`trajopt` 是 ROS 1 下的二维局部轨迹优化功能包。它接收前端搜索得到的离散路径、全局代价地图和车辆里程计，在代价地图中为路径构造一组相互连通的二维凸安全走廊，然后使用 MINCO 五次多项式与 L-BFGS 优化器生成连续、平滑、带时间参数的前向轨迹。

当前处理流程如下：

1. 接收 `nav_msgs/Path`，路径消息到达时同步触发一次完整优化。
2. 根据路径弦线碰撞、弧长、折线偏差和转角条件，自适应拆分或合并路径段。
3. 从 `nav_msgs/OccupancyGrid` 提取局部障碍物点，为每段路径膨胀出凸多面体，并检查相邻走廊的交集面积。
4. 使用路径首尾位置、可选的首尾 yaw 方向约束和分段几何长度构造 MINCO 初值。
5. 优化轨迹的内部连接点和总时长；各段时长比例在初始化后保持不变。
6. 以 `0.01 s` 周期采样优化结果，发布带位姿、速度、加速度、曲率、纵向 jerk、累计弧长和相对时间的轨迹，同时发布调试路径及安全走廊可视化。

优化目标包含最小化 jerk、总时间、车辆外形越出安全走廊的惩罚，以及速度、加速度和曲率超限惩罚。当前节点没有接入其他车辆的轨迹，因此源码中保留的动态障碍物/多车轨迹代价在本运行链路中不生效。

## 2. 节点

| 项目 | 当前值 |
| --- | --- |
| 可执行文件 | `traj_opt` |
| `roslaunch` 节点名 | `/traj_opt` |
| 源码中的默认节点名 | `/trajopt_server`，仅直接使用 `rosrun` 且未重命名时生效 |
| 参数句柄 | 全局 `ros::NodeHandle`，读取全局参数，例如 `/topics/path` |
| 回调方式 | 主循环 `10 Hz` 调用 `ros::spinOnce()`；轨迹优化在路径回调内同步执行 |
| 输出坐标系 | 固定写为 `map` |

路径回调执行优化期间不会处理新的回调。输入订阅队列长度为 10，路径高频发布时可能产生排队和处理延迟。

## 3. 输入

以下话题名是 `trajopt.launch` 加载 `config/trajopt.yaml` 后的实际值。

| 话题 | 消息类型 | 队列 | 作用 |
| --- | --- | ---: | --- |
| `/global_costmap_node/costmap/costmap` | `nav_msgs/OccupancyGrid` | 10 | 提供安全走廊构造、弦线碰撞检测和地图边界。代价值达到阈值的栅格视为障碍物，未知栅格是否占用由参数控制。 |
| `/lio/robo/odom` | `nav_msgs/Odometry` | 10 | 更新车辆状态；启用 `boundary/consider_start_yaw` 时使用最新里程计 yaw 约束轨迹起始方向。 |
| `/astar_path` | `nav_msgs/Path` | 10 | 前端搜索路径，也是一次优化的触发输入。位置用于构造路径与走廊，最后一个位姿的 yaw 用作终点期望朝向。 |

输入约束和坐标约定：

- 地图、路径和里程计必须已经位于同一二维世界坐标系中，通常为 `map`。节点不查询 TF，也不会根据消息中的 `header.frame_id` 做坐标变换。
- 路径至少需要 2 个位姿，且不能包含相邻重复点。
- 安全走廊和 MINCO 当前至少需要生成 2 个多项式段；如果整条路径最终只形成一个凸段，节点会以“多项式段数不足”拒绝该路径。
- 当前固定生成正向轨迹，内部方向标志为 `1`，不会根据里程计速度生成倒车轨迹。
- 启用起点 yaw 约束时，必须在路径到达前收到至少一帧里程计；否则本次优化会被拒绝。
- 开启首点或末点 yaw 约束时，对应边界速度设为沿期望 yaw 的 `0.01 m/s`，加速度保持为 0。该微小速度用于在 MINCO 的 P/V/A 边界中编码方向，不代表期望行驶速度。
- 只要启用了任一 yaw 约束，`optimizing/max_vel` 就必须大于 `0.01 m/s`，否则节点会拒绝本次优化，避免优化器限速时消除或反转朝向向量。
- 关闭 yaw 约束时，对应边界速度和加速度均为 0，因此不编码朝向，但仍然是零导数硬边界，并非将端点速度作为自由变量参与优化。
- 应先提供有效代价地图，再发布路径。地图缺失、尺寸/分辨率无效或路径越出地图时，走廊构造会失败。

## 4. 输出

| 话题 | 消息类型 | 队列/锁存 | 作用 |
| --- | --- | --- | --- |
| `/trajopt/minco_traj` | `robot_trajectory_msgs/RobotTrajectory` | 2，非锁存 | 最终带时间参数的轨迹，供支持该消息的下游模块消费。话题名当前为源码硬编码。 |
| `/trajopt/debug_path` | `nav_msgs/Path` | 1，锁存 | 将同一条优化轨迹转换为仅含位姿的路径，话题名可配置；当前工程的 `controller` 实际订阅此话题。 |
| `/trajopt/path_endpoints` | `visualization_msgs/MarkerArray` | 1，锁存 | 优化轨迹首尾箭头；起点为绿色、终点为红色，话题名可配置。 |
| `/trajopt/polyhedrons` | `decomp_ros_msgs/PolyhedronArray` | 1，锁存 | 每个二维凸安全走廊的半空间可视化，话题名当前为源码硬编码。 |
| `/trajopt/corridor_points` | `visualization_msgs/MarkerArray` | 1，锁存 | 走廊关键点球形标记，话题名当前为源码硬编码。 |

当前工程内没有节点订阅 `/trajopt/minco_traj`。默认导航链路仍是 `trajopt` 发布 `/trajopt/debug_path`，再由 `controller` 将其作为路径输入，因此时间、速度、加速度、曲率等扩展字段尚未进入现有控制器。

### 4.1 最终轨迹字段

`robot_trajectory_msgs/RobotTrajectory` 包含一个 `std_msgs/Header header` 和 `RobotTrajectoryPoint[] points`。节点以 `0.01 s` 为周期采样，每个点包含：

| 字段 | 含义 |
| --- | --- |
| `time_from_start` | 从整条轨迹起点开始累计的时间 |
| `pose` | 二维轨迹位姿；`z` 固定为 `0.2 m`。起点开启约束时使用里程计 yaw，关闭时使用优化轨迹起点切线；终点始终发布输入 Path 最后一点 yaw，其中开启约束时轨迹末端切线也受该 yaw 约束 |
| `velocity` | 世界坐标系下的线速度；`angular.z = speed * curvature` |
| `acceleration` | 世界坐标系下的线加速度 |
| `curvature` | 有符号几何曲率，单位 `1/m` |
| `longitudinal_jerk` | 纵向加速度的数值微分，单位 `m/s^3` |
| `arc_length` | 从轨迹起点开始累计的采样折线弧长，单位 `m` |

所有最终轨迹和可视化消息的 `header.frame_id` 均固定为 `map`。优化失败时不会发布新的 `/trajopt/minco_traj`；锁存的可视化话题可能仍保留上一次消息。

## 5. 服务

当前功能包**不提供也不调用任何 ROS 服务**。

工作区中的旧 `srv/SendPath.srv` 已删除，`CMakeLists.txt` 也已移除 `add_service_files()`，当前源码不再包含 `trajopt/SendPath.h`，不会生成 `trajopt/SendPath` 服务类型。旧版本曾向 `/trajopt/server/global_traj_path` 发送 `nav_msgs/Path`；面向支持带时间轨迹的外部下游，当前替代接口是 `/trajopt/minco_traj`，消息类型为 `robot_trajectory_msgs/RobotTrajectory`。仍调用旧服务或包含旧服务头文件的外部节点需要同步迁移；工程内现有 `controller` 的兼容链路则使用 `/trajopt/debug_path`。

`config/navigation.yaml` 中保留的 `/trajopt/server/global_traj_path` 只是未被本节点加载的历史配置，不能代表当前接口。

## 6. 参数

`launch/trajopt.launch` 将 `config/trajopt.yaml` 加载到全局命名空间。下表“配置值”表示通过该启动文件运行时的实际值，“代码默认值”表示参数不存在时的回退值。

### 6.1 话题参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | --- | --- | --- |
| `topics/global_map` | `/global_costmap_node/costmap/costmap` | `/global_costmap_node/costmap/costmap` | 全局代价地图输入 |
| `topics/odom` | `/lio/robo/odom` | `/lio/odom` | 里程计输入 |
| `topics/path` | `/astar_path` | `/front2end_search_path` | 前端路径输入 |
| `topics/debug_path` | `/trajopt/debug_path` | `/trajopt/debug_path` | 调试路径输出 |
| `topics/path_endpoints` | `/trajopt/path_endpoints` | `/trajopt/path_endpoints` | 首尾位姿标记输出 |

### 6.2 安全走廊参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `corridor/collision_cost_threshold` | 90 | 80 | 栅格代价值大于等于该值时视为占用 |
| `corridor/unknown_as_occupied` | `true` | `true` | 是否将代价值为 `-1` 的未知栅格视为占用 |
| `corridor/max_longitudinal` | 0.3 | 0.2 | 每条分解种子前后方向的局部膨胀范围，单位 m |
| `corridor/max_lateral` | 0.6 | 0.6 | 每条分解种子左右方向的局部膨胀范围，单位 m |
| `corridor/min_seed_length` | 0.25 | 0.25 | 尝试合并过短相邻路径段的长度阈值，单位 m |
| `corridor/max_seed_length` | 2.0 | 0.8 | 单个路径种子的最大弧长，超出时继续拆分，单位 m |
| `corridor/max_seed_deviation` | 0.1 | 0.1 | 原路径相对种子弦线的最大允许偏差，单位 m |
| `corridor/max_seed_yaw_error` | 0.2 | 0.2 | 合并短段时允许的最大转角，单位 rad |
| `corridor/min_overlap_area` | 0.01 | 0.01 | 相邻凸走廊交集的最小面积，单位 `m^2` |

### 6.3 MINCO 优化参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `optimizing/traj_resolution` | 8 | 8 | 中间多项式段的固定数量约束采样分辨率 |
| `optimizing/des_traj_resolution` | 20 | 20 | 第一段和最后一段的加密约束采样分辨率 |
| `optimizing/wei_sta_obs` | 100000.0 | 7000.0 | 车辆外形越出静态安全走廊的惩罚权重 |
| `optimizing/wei_dyn_obs` | 7000.0 | 7000.0 | 动态/多车避障权重；当前节点未接入其他车辆轨迹，因此不生效 |
| `optimizing/wei_feas` | 1000.0 | 1000.0 | 速度、加速度、曲率可行性惩罚权重 |
| `optimizing/wei_time` | 500.0 | 500.0 | 总轨迹时间代价权重 |
| `optimizing/dyn_obs_clearance` | 1.0 | 1.0 | 动态障碍物安全距离；当前运行链路不生效，单位 m |
| `optimizing/max_vel` | 3.0 | 3.0 | 优化器最大速度，并用于内部节点速度和初始时间分配，单位 m/s |
| `optimizing/max_acc` | 1.5 | 1.5 | 最大加速度，同时用于初始时间分配，单位 `m/s^2` |
| `optimizing/max_cur` | 0.523598 | 0.523598 | 最大绝对曲率，单位 `1/m` |
| `optimizing/half_margin` | 0.25 | 0.25 | 加到车辆长、宽两侧的安全余量，即总长和总宽各增加 `2 * half_margin`，单位 m |

速度、加速度、曲率和走廊约束在优化器中以采样惩罚项实现，并非独立的硬约束。当前代码在 L-BFGS 达到最大迭代次数或最大线搜索次数时也可能接受结果；优化后不再执行额外的发布前硬验收。

### 6.4 初始时间参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `initial_time/min_speed` | 0.2 | 0.2 | 计算分段初始时长时使用的最小平均速度，单位 m/s |
| `initial_time/min_piece_time` | 0.2 | 0.2 | 每个多项式段的最小初始时长，单位 s |
| `initial_time/min_turn_speed_ratio` | 0.25 | 0.25 | 转角处节点速度相对 `max_vel` 的最小比例 |

每段初始时长取“段长/平均节点速度”“节点速度变化量/最大加速度”和 `min_piece_time` 三者的最大值。

### 6.5 首尾朝向边界参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `boundary/consider_start_yaw` | `true` | `true` | 是否使用最新里程计 yaw 构造 `0.01 m/s` 起点速度向量；关闭时起点输出 yaw 取优化轨迹起点切线 |
| `boundary/consider_end_yaw` | `true` | `true` | 是否使用输入 Path 末点 yaw 构造 `0.01 m/s` 终点速度向量；关闭时不约束末端切线，但发布的终点 yaw 仍强制使用 Path 末点 yaw |

两个开关相互独立，可以组合成四种首尾边界模式。改变 YAML 后需要重启节点才能生效。

### 6.6 车辆参数

| 参数 | 配置值 | 代码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `vehicle/cars_num` | 1 | 1 | 多车容器大小；当前节点只规划单车 |
| `vehicle/car_id` | 0 | 0 | 自车编号；当前没有多车轨迹输入 |
| `vehicle/car_length` | 0.6 | 0.6 | 车辆矩形长度，之后会叠加安全余量，单位 m |
| `vehicle/car_width` | 0.6 | 0.6 | 车辆矩形宽度，之后会叠加安全余量，单位 m |
| `vehicle/car_d_cr` | 0.0 | 0.0 | 车辆矩形中心相对轨迹参考点的纵向偏移，单位 m |

### 6.7 当前未生效的配置

- `validation/sample_period` 和 `validation/corridor_tolerance` 仍存在于 `trajopt.yaml`，但当前源码没有读取或使用它们。
- `config/navigation.yaml` 与 `config/mission.yaml` 不会被 `trajopt.launch` 加载，`traj_opt` 源码也不读取它们。
- `proto/minco_config.proto` 会由 CMake 生成并编译对应 C++ 文件，但节点没有读取 protobuf 配置；运行参数以 ROS 参数服务器上的 `trajopt.yaml` 为准。

## 7. 自定义消息

`CMakeLists.txt` 当前仍生成以下四种消息，但 `traj_opt` 节点不订阅或发布它们。实际最终输出使用独立功能包中的 `robot_trajectory_msgs/RobotTrajectory`。

| 消息 | 字段和用途 |
| --- | --- |
| `trajopt/StateMsg` | `std_msgs/Header header`、`uint8 state`；文件注释保留了状态枚举设想，但没有定义消息常量 |
| `trajopt/SingleMinco` | `start_time`、首尾 x/y 边界向量、二维位置点数组 `pos_pts`、分段时间数组 `t_pts` 和 `reverse` 标志 |
| `trajopt/MincoTraj` | `SingleMinco[] trajs` |
| `trajopt/Trajectory` | `traj_type`、`nav_msgs/Path nav_path`、`MincoTraj minco_path`；注释约定 `0` 为测试路径、`1` 为 MINCO 路径 |

这些接口当前属于保留接口，不应据此推断运行时会发布对应话题。

## 8. 构建依赖

该包使用 C++14 和 catkin。`CMakeLists.txt` 当前查找的主要依赖包括：

- ROS/catkin：`roscpp`、`rospy`、`std_msgs`、`sensor_msgs`、`nav_msgs`、`visualization_msgs`、`tf`、`roslib`、`message_generation`、`cv_bridge`、`image_transport`、`controller`、`decomp_ros_utils`、`robot_trajectory_msgs`。
- 系统库：Protobuf、OpenCV、PCL、OMPL、Eigen3、yaml-cpp。
- 包内库：`geo_utils2d`、MINCO 多项式工具、L-BFGS、DecompROS 和随包保存的 Boost odeint 源码。

当前工作区使用 `catkin_make`。在工作区根目录构建该包并加载环境：

```bash
catkin_make --pkg trajopt
source devel/setup.zsh
```

不要在现有 `build` 目录中直接混用 `catkin build`；两种工具的构建空间元数据不兼容。

当前 `package.xml` 只显式列出了部分依赖，与 `CMakeLists.txt` 并不完全一致。因此在全新环境中，仅执行 `rosdep install` 可能不足以准备全部依赖，应同时按上述 CMake 依赖检查系统和工作区功能包。

## 9. 启动方式

推荐使用启动文件，它会先把有效配置加载到全局参数命名空间：

```bash
source devel/setup.zsh
roslaunch trajopt trajopt.launch
```

直接运行时，需要手动加载参数：

```bash
source devel/setup.zsh
rosparam load "$(rospack find trajopt)/config/trajopt.yaml"
rosrun trajopt traj_opt
```

最小数据顺序为：先发布有效的全局代价地图；当 `boundary/consider_start_yaw=true` 时等待至少一帧里程计；然后发布前端路径。可用以下命令确认当前接口：

```bash
rostopic info /astar_path
rostopic info /trajopt/minco_traj
rostopic echo -n 1 /trajopt/minco_traj
```

`launch/trajopt.launch` 当前不会启动 RViz。包内 `rviz/navi_2d.rviz` 保存的是一组历史话题显示配置，其中多数话题与当前输出不一致；调试时应在 RViz 中手动添加当前第 4 节列出的 Path、MarkerArray 和 PolyhedronArray 话题。

## 10. 常见失败条件

- 未收到地图、地图为空或分辨率无效。
- 启用起点 yaw 约束，但尚未收到有效里程计。
- 启用了首点或末点 yaw 约束，但 `optimizing/max_vel` 不大于 `0.01 m/s`。
- 路径点不足、含相邻重复点，或相邻点之间已经穿过占用栅格。
- 路径或膨胀区域越出地图有效范围。
- 某段无法生成有效凸走廊，或相邻走廊交集面积小于阈值。
- 自适应简化后只剩一个多项式段。
- 约束采样数量与走廊数量不一致，或 L-BFGS 返回未被接受的错误状态。

节点会在控制台输出 `[planner_timing][trajopt]` 日志，包含路径接收、走廊生成、输入构造、优化、采样和最终发布耗时，可用于定位延迟发生在哪个阶段。
