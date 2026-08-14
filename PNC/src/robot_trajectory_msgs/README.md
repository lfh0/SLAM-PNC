# robot_trajectory_msgs

## 功能作用

`robot_trajectory_msgs` 是本工程的轨迹消息接口包，用统一格式描述一条带时间、运动学状态和几何信息的机器人轨迹。它只定义消息，不包含 ROS 节点、固定话题、服务或运行参数。

当前工程中，`trajopt` 使用该包在 `/trajopt/minco_traj` 上发布优化后的轨迹。工程内暂时没有节点订阅该消息；现有 `controller` 仍通过 `/trajopt/debug_path` 接收 `nav_msgs/Path`。

## 输入输出

从数据流角度看，本包接收的“输入”是上游规划器填入的轨迹点数据，输出是 Catkin 生成的 ROS 消息类型，供发布者、订阅者和记录工具使用。

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 数据输入 | 位姿、速度、加速度、曲率、jerk、弧长和相对时间 | 由轨迹生成或优化模块填充 |
| 类型输出 | `robot_trajectory_msgs/RobotTrajectoryPoint` | 单个带时间参数的轨迹点 |
| 类型输出 | `robot_trajectory_msgs/RobotTrajectory` | 带 `Header` 的完整轨迹点序列 |
| 构建输出 | C++、Python 等 ROS 消息绑定 | 由 `message_generation`/`message_runtime` 生成 |

## 消息定义

### `RobotTrajectoryPoint.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `time_from_start` | `duration` | 从整条轨迹起点到当前点的相对时间 |
| `pose` | `geometry_msgs/Pose` | 轨迹点的位置与姿态 |
| `velocity` | `geometry_msgs/Twist` | 线速度与角速度 |
| `acceleration` | `geometry_msgs/Accel` | 线加速度与角加速度 |
| `curvature` | `float64` | 轨迹曲率，单位 `1/m` |
| `longitudinal_jerk` | `float64` | 沿轨迹方向的 jerk，单位 `m/s^3` |
| `arc_length` | `float64` | 从轨迹起点累计的路径弧长，单位 m |

### `RobotTrajectory.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 轨迹时间戳与坐标系 |
| `points` | `RobotTrajectoryPoint[]` | 按轨迹时间顺序排列的轨迹点 |

## 使用约定

- 同一条轨迹内的所有位姿和运动学量应与 `header.frame_id` 所指坐标系一致。
- `points` 应按 `time_from_start` 单调不减排列；`arc_length` 也应随轨迹前进而非递减。
- `duration` 是相对轨迹起点的时间，不是 ROS 绝对时间；绝对起始时刻放在 `header.stamp`。
- 消息本身不限制二维或三维运动。当前 PNC 链路主要使用平面位置、偏航角和纵向运动量。

## 构建与检查

```bash
catkin_make --pkg robot_trajectory_msgs
source devel/setup.bash
rosmsg show robot_trajectory_msgs/RobotTrajectory
rosmsg show robot_trajectory_msgs/RobotTrajectoryPoint
```

在其他包中使用时，需要在 `package.xml` 和 `CMakeLists.txt` 中声明对 `robot_trajectory_msgs` 的依赖。
