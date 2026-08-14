# controller

`controller` 是 ROS 1 路径跟踪功能包。当前只实现了 Pure Pursuit 控制器：节点接收规划路径和机器人里程计，根据路径跟踪状态机计算线速度、角速度，并向真实底盘和仿真底盘发布相同的速度指令。到达终点后，节点还会发布到达标志、采集实际停止位姿并记录终点误差。

## 功能作用

- 使用路径首点朝向完成起步前原地对正。
- 在路径跟踪阶段选择前视点，结合 Pure Pursuit 曲率、航向误差和横向误差生成控制量。
- 根据曲率、航向误差和横向误差降低参考速度；目标点位于车后或航向偏差过大时停止前进并优先转向。
- 接近终点后切换为位置 PID，位置满足要求后再进行终点航向 PID 调整。
- 对线速度和角速度进行同比例限幅，避免改变限幅前的速度/角速度比例。
- 记录每次新路径执行期间的实际里程计轨迹。
- 完成后发布一次到达消息，并对随后若干帧里程计取平均，追加写入终点位置和航向误差日志。

## 节点

| 节点名 | 可执行文件 | 作用 |
| --- | --- | --- |
| `controller_server` | `controller_server` | 管理 ROS 输入输出、Pure Pursuit 状态机、速度限幅、到达通知、实际轨迹和终点误差日志 |

节点按 `controller_server/rate` 指定的频率运行。只有同时收到非空路径和里程计后才会开始控制；条件不满足或控制完成时仍会持续发布零速度。

> 当前源码始终构造并调用 `PurePursuit`。`controller_server/controller_type` 虽然存在，但没有实现运行时控制器切换，因此应保持为 `pure_pursuit`。

## 输入

### 订阅话题

| 默认配置话题 | 消息类型 | 队列长度 | 用途 |
| --- | --- | --- | --- |
| `/trajopt/debug_path` | `nav_msgs/Path` | 1 | 待跟踪路径。首个 `PoseStamped` 的朝向用于起始航向对正，最后一个的位置和朝向用于终点控制；空路径会被拒绝 |
| `/lio/robo/odom` | `nav_msgs/Odometry` | 1 | 当前机器人位姿和速度。控制使用 `pose.pose.position` 与四元数转换得到的 yaw；`twist.twist.linear.x` 和 `twist.twist.angular.z` 会被保存到内部状态 |

话题名分别由 `controller_server/path_topic` 和 `controller_server/odom_topic` 指定。

### 输入约束

- 本包不执行 TF 坐标变换，路径位姿和里程计位姿必须处于可直接比较的同一坐标系。
- 路径首点、路径终点及跟踪点的四元数朝向会参与控制，应提供有效朝向。
- 路径与里程计没有时间同步机制，控制循环使用最近一次收到的数据。
- 收到新路径时会重置控制状态、到达上报状态、终点误差采样和实际轨迹；新路径必须至少包含一个位姿。

## 输出

### 发布话题

| 默认配置话题 | 消息类型 | 队列长度 | 输出内容 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 1 | 底盘速度指令，仅设置 `linear.x` 和 `angular.z` |
| `/car1/cmd_vel` | `geometry_msgs/Twist` | 1 | 仿真速度指令，内容与 `/cmd_vel` 相同；若配置成与主速度话题同名，则不进行第二次发布 |
| `/arrive/finish` | `std_msgs/Bool` | 1 | 状态首次进入 `Finished` 时发布一次 `data: true`。新路径到来时只重置内部标志，不额外发布 `false` |
| `/controller/odom_path` | `nav_msgs/Path` | 1（锁存） | 当前路径执行期间的实际里程计轨迹。收到新路径时先发布空轨迹，之后机器人每移动至少 `0.01 m` 添加一个位姿 |

话题名依次由 `cmd_vel_topic`、`sim_cmd_vel_topic`、`arrive_topic` 和 `odom_path_topic` 参数指定。

### 文件输出

控制完成后，节点从后续里程计消息中收集 `finish_average_frames` 帧数据，对 `x`、`y` 求算术平均，对 yaw 通过正弦/余弦求圆周平均，然后以追加方式写入 `finish_error_log_path`：

```text
goal_x ... goal_y ... goal_yaw ... avg_odom_x ... avg_odom_y ... avg_odom_yaw ... error_x ... error_y ... error_xy ... error_yaw ...
```

其中 `error_x`、`error_y` 和 `error_yaw` 均为“平均里程计值减目标值”，`error_xy` 为平面欧氏距离，`error_yaw` 被归一化到 `(-pi, pi]`。进入完成状态后若没有足够的新里程计消息，日志不会写入。

### 服务与 Action

本功能包不提供或调用 ROS 服务，也不提供 Action 接口。

## 控制流程

1. `None`：新路径装载后的过渡状态，输出零速度并进入 `Init`。
2. `Init`：原地调整到路径首点 yaw；误差不大于 `goal_yaw_tolerance` 后进入 `Tracking`。
3. `Tracking`：从上次最近点附近向前搜索最近路径点，再选择满足前视距离的目标点。根据 Pure Pursuit 曲率、航向误差和横向误差计算速度。
4. `CloseEnd`：距离终点不大于 `goal_slowdown_distance` 后，使用终点位置 PID 控制。位置误差达标后进入 `GoalYawAdjust`；若在终点附近的小范围内持续蠕动超时，也会进入下一状态。
5. `GoalYawAdjust`：停止平移，原地调整到路径终点 yaw。
6. `Finished`：持续输出零速度；首次进入时发布到达消息并启动终点误差采样。

`ControlState::TakingOff` 在枚举和分支中保留，但当前状态转换不会进入该状态。

## 参数

启动文件将两个 YAML 文件加载到全局参数服务器。源码使用普通 `ros::NodeHandle` 读取相对参数名，并非私有参数 `~param`；在默认启动方式下，完整名称分别为 `/controller_server/...` 和 `/purepursuit_node/...`。

### 节点与接口参数

配置文件：`config/controller_server.yaml`

| 参数 | 当前配置值 | 未配置时源码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `controller_server/controller_type` | `pure_pursuit` | `pure_pursuit` | 控制器类型标识；当前仅支持 Pure Pursuit |
| `controller_server/rate` | `20.0` | `10.0` | 控制循环频率，单位 Hz；小于等于 0 时回退到 10 Hz |
| `controller_server/path_topic` | `/trajopt/debug_path` | `/astar_path_o` | 路径输入话题 |
| `controller_server/odom_topic` | `/lio/robo/odom` | `/lio/robo/odom` | 里程计输入话题 |
| `controller_server/cmd_vel_topic` | `/cmd_vel` | `/cmd_vel` | 主速度输出话题 |
| `controller_server/sim_cmd_vel_topic` | `/car1/cmd_vel` | `/car1/cmd_vel` | 仿真速度输出话题 |
| `controller_server/arrive_topic` | `/arrive/finish` | `/arrive/finish` | 到达标志输出话题 |
| `controller_server/odom_path_topic` | `/controller/odom_path` | `/controller/odom_path` | 实际里程计轨迹输出话题 |
| `controller_server/finish_error_log_path` | `/home/lfh/SLAM+PNC/PNC/src/controller/finish_error_log.txt` | 同当前配置 | 终点误差日志路径；父目录必须存在且进程必须有写权限 |
| `controller_server/finish_average_frames` | `20` | `20` | 完成后参与平均的里程计帧数；小于等于 0 时回退到 20 |
| `controller_server/max_forward_linear_velocity` | `0.7` | `0.5` | 最大前进线速度，单位 m/s |
| `controller_server/max_backward_linear_velocity` | `-0.3` | `-0.3` | 最大后退幅度对应的负速度下限，单位 m/s |
| `controller_server/max_angular_velocity` | `0.3` | `0.3` | 角速度绝对值上限，单位 rad/s |

### Pure Pursuit 参数

配置文件：`config/purepursuit.yaml`

| 参数 | 当前配置值 | 未配置时源码默认值 | 说明 |
| --- | ---: | ---: | --- |
| `purepursuit_node/lookahead_distance` | `0.5` | `0.1` | 前视距离，单位 m；运行时最小限制为 `0.001` |
| `purepursuit_node/vehicle_avg_speed` | `0.5` | `0.28` | 跟踪阶段参考线速度，单位 m/s |
| `purepursuit_node/kp_yaw_` | `0.05` | `0.5` | 跟踪阶段航向误差比例系数 |
| `purepursuit_node/kd_yaw_` | `0.0` | `0.1` | 跟踪阶段航向误差微分系数 |
| `purepursuit_node/kp_pos_` | `0.0` | `0.5` | 跟踪阶段横向误差反馈系数 |
| `purepursuit_node/end_yaw_kp_` | `0.1` | `0.5` | 起点/终点航向 PID 比例系数，同时用于近终点位置控制的角度比例反馈 |
| `purepursuit_node/end_yaw_ki_` | `0.0` | `0.0` | 起点/终点航向 PID 积分系数 |
| `purepursuit_node/end_yaw_kd_` | `0.01` | `0.1` | 起点/终点航向 PID 微分系数 |
| `purepursuit_node/end_pos_kp_` | `0.5` | `0.68` | 近终点距离 PID 比例系数 |
| `purepursuit_node/end_pos_ki_` | `0.001` | `0.0` | 近终点距离 PID 积分系数 |
| `purepursuit_node/end_pos_kd_` | `0.01` | `0.0` | 近终点距离 PID 微分系数 |
| `purepursuit_node/curvature_velocity_gain` | `0.2` | `1.2` | 曲率导致的线速度衰减权重 |
| `purepursuit_node/lateral_velocity_gain` | `0.5` | `0.5` | 横向误差导致的线速度衰减权重 |
| `purepursuit_node/rotate_in_place_threshold` | `1.5` | `1.0` | 航向误差绝对值超过该阈值时将跟踪线速度置零，单位 rad |
| `purepursuit_node/derivative_filter_lambda` | `0.8` | `0.8` | 航向误差微分低通滤波中历史值的权重 |
| `purepursuit_node/goal_slowdown_distance` | `0.6` | `0.4` | 从路径跟踪切换到近终点位置控制的距离，单位 m |
| `purepursuit_node/goal_position_tolerance` | `0.03` | `0.05` | 终点位置完成容差，单位 m |
| `purepursuit_node/goal_yaw_tolerance` | `0.05` | `0.05` | 起点和终点航向容差，单位 rad |
| `purepursuit_node/close_end_wriggle_distance` | `0.06` | `0.1` | 近终点持续蠕动检测范围，单位 m；运行时不会小于位置容差 |
| `purepursuit_node/close_end_timeout` | `8.0` | `5.0` | 在蠕动检测范围内连续停留的超时时间，单位 s；设为 0 可关闭超时切换 |

## 构建与启动

依赖 ROS 1、catkin、`roscpp`、`nav_msgs`、`geometry_msgs`、`std_msgs` 和 `tf2`。在工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
roslaunch controller purepursuit.launch
```

`launch/purepursuit.launch` 会依次加载 `config/purepursuit.yaml`、`config/controller_server.yaml`，然后启动 `controller_server`。需要更换话题、控制频率、限幅或控制增益时，在节点启动前修改对应 YAML 参数。

## 目录说明

```text
controller/
├── config/                         # 节点接口、限幅和 Pure Pursuit 参数
├── include/controller/             # 控制服务器、控制器及状态数据结构声明
├── launch/purepursuit.launch       # 参数加载与节点启动入口
├── src/app/Controller_server.cpp   # ROS 节点、接口、限幅和误差记录
├── src/controller/purepursuit.cpp  # Pure Pursuit 与终点控制状态机
└── finish_error_log.txt            # 默认终点误差追加日志
```
