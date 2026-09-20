# 端到端局部规划器需求说明

## 1. 文档目的

本文档定义在现有 PNC 工程基础上实现端到端局部规划器的功能边界、系统接口、关键约束、实施阶段与验收标准。

本项目不是从原始传感器直接输出底盘控制量的完整端到端自动驾驶系统，而是使用神经网络替代传统局部轨迹生成模块，同时保留定位、全局规划、安全检查和 MPC 控制。

目标数据流为：

```text
静态地图 + 导航目标
        |
        v
   A* 全局规划器
        |
        v
   全局路径（map）--------------------+
                                       |
LiDAR 点云 -> 过滤 -> 历史帧运动补偿   |
                    -> Temporal BEV ----+--> Neural Local Planner
                                       |             |
Ego State -----------------------------+             v
                                                Ego 局部轨迹
                                                     |
                                                     v
                                              轨迹适配器
                                                     |
                                                     v
                                              Safety Checker
                                                     |
                                                     v
                                          Map Frame RobotTrajectory
                                                     |
                                                     v
                                                    MPC
                                                     |
                                                     v
                                                   底盘
```

## 2. 当前工程基础

当前工程已经具备以下可复用模块：

| 模块 | 功能包 | 当前接口 | 处理方式 |
| --- | --- | --- | --- |
| 全局规划 | `front2end_search` | 发布 `/astar_path`，类型为 `nav_msgs/Path` | 保留 |
| 传统轨迹优化 | `trajopt` | 发布 `/trajopt/minco_traj` | 保留为传统方案、回退方案和对照基线 |
| 轨迹消息 | `robot_trajectory_msgs` | `RobotTrajectory`、`RobotTrajectoryPoint` | 复用 |
| MPC 控制 | `controller` | 订阅 `/trajopt/minco_traj` | 改造滚动轨迹更新接口后复用 |
| 定位与点云 | Super-LIO | `/lio/robo/odom`、`/lio/cloud_imu` | 复用，但必须统一坐标系和时间戳 |
| 局部代价地图 | `navi_map` | 当前点云障碍层 | 可用于可视化和安全检查参考，不能替代 Temporal BEV |

当前实际链路为：

```text
Global Costmap -> A* -> MINCO -> RobotTrajectory -> MPC -> Chassis
```

新增功能应以独立 ROS 1 功能包 `neural_local_planner` 实现。第一版不把神经网络代码直接写入 `trajopt` 或 `controller`。

## 3. 系统边界

### 3.1 保留的传统模块

- LiDAR-IMU/LIO 定位；
- A* 全局规划；
- 确定性轨迹适配；
- Safety Checker；
- MPC 控制；
- 底盘驱动与执行器约束。

### 3.2 学习模块职责

神经局部规划器负责根据以下输入生成未来局部运动意图：

- 多帧局部环境 BEV；
- 全局路径的局部导航编码；
- 当前车辆运动状态。

学习模块不负责：

- 构造 ROS Header 和时间戳；
- 预测四元数；
- 直接预测弧长和采样间隔；
- 无条件保证轨迹安全；
- 直接输出底盘控制命令；
- 判断整个导航任务是否完成。

## 4. 坐标系与时间约定

开发前必须确认并固定完整 TF 关系。推荐约定：

```text
map -> robot -> imu
```

现有系统还使用 `world`。必须明确 `map` 与 `world` 的关系，并提供有效 TF；禁止默认认为二者相同后直接混用数值。

统一定义：

- 全局路径坐标系：`map`；
- 网络输入和网络原始输出：当前观测基准时刻的 `robot`/Ego Frame；
- 点云原始坐标系：以消息 `header.frame_id` 为准，当前预期为 `imu`；
- 发布给当前 MPC 的轨迹坐标系：`map`；
- 所有变换以消息时间戳查询，不得使用不对应采样时刻的最新位姿；
- 超出允许时间差的数据不得强行组成训练样本或在线推理输入。

历史帧点云补偿到当前车辆坐标系时，应使用：

```text
T(robot_t <- imu_t-k)
= inverse(T(map <- robot_t))
  * T(map <- robot_t-k)
  * T(robot_t-k <- imu_t-k)
```

## 5. ROS 接口需求

### 5.1 输入接口

第一版默认输入：

| 数据 | 默认话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 点云 | `/lio/cloud_imu` | `sensor_msgs/PointCloud2` | 使用实际 `frame_id` 和时间戳 |
| 自车状态 | `/lio/robo/odom` | `nav_msgs/Odometry` | 位置、yaw、纵向速度和角速度 |
| 全局路径 | `/astar_path` | `nav_msgs/Path` | `map` 坐标系 |
| 全局任务目标 | 待配置 | `geometry_msgs/PoseStamped` 或从任务管理器获取 | 用于任务完成判断，不等同于局部轨迹末点 |
| 底盘反馈 | 待配置 | 现有 YHS CAN 消息 | 可选输入，优先补充真实转角和车辆速度 |

所有话题名均应通过 ROS 参数配置，不在源码中写死。

### 5.2 输出接口

神经规划链路最终输出：

| 数据 | 建议话题 | 消息类型 |
| --- | --- | --- |
| 安全局部轨迹 | `/neural_local_planner/trajectory` | `robot_trajectory_msgs/RobotTrajectory` |
| 网络原始轨迹可视化 | `/neural_local_planner/raw_path` | `nav_msgs/Path` |
| 安全轨迹可视化 | `/neural_local_planner/safe_path` | `nav_msgs/Path` |
| BEV 调试图 | `/neural_local_planner/bev` | `sensor_msgs/Image` |
| 规划状态 | `/neural_local_planner/status` | 待定义状态消息 |

集成初期可将安全轨迹重映射到 `/trajopt/minco_traj` 供现有 MPC 使用。运行神经规划器时不得同时让 `trajopt` 向相同话题发布轨迹。

## 6. 输入同步与点云处理

### 6.1 时间同步

系统应维护带时间戳缓存，而不是简单使用每个话题的最新值。

每个规划周期应确定统一基准时间 `t`，并完成：

- 选取与 `t` 对应的当前点云；
- 对 odom 位姿进行插值，获得每个历史点云时刻和当前时刻的车辆位姿；
- 将全局路径转换到时刻 `t` 的 Ego Frame；
- 计算时刻 `t` 的 Ego State；
- 检查各输入的数据年龄和最大允许时间差。

定位跳变、时间倒退或 rosbag 循环播放时必须清空历史帧缓存。

### 6.2 点云预处理

推荐第一版流程：

```text
Raw PointCloud
 -> ROI Crop
 -> VoxelGrid
 -> Outlier Removal
 -> Ground Removal
 -> Collision-Relevant Height Filter
 -> Ego Motion Compensation
 -> BEV Projection
```

高度过滤参数必须基于：

- LiDAR 到车辆坐标系外参；
- 车辆真实碰撞高度；
- 路面坡度；
- 安全余量。

不得仅依靠固定传感器坐标 `z` 区间判断是否可通行。

## 7. Temporal BEV 需求

第一版建议参数：

```text
历史帧数：4
参考时间：t-0.3、t-0.2、t-0.1、t
分辨率：0.1 m/cell
X 范围：-5 m ～ 20 m
Y 范围：-10 m ～ 10 m
栅格大小：250 × 200
```

每个历史帧至少包含：

- Occupancy；
- Height；
- Density。

系统还应表示有效观测范围。没有回波的区域不能默认全部当作空闲区域。可采用额外的 Observed/Unknown Mask，或者在现有通道定义中明确编码未知状态。

必须固定并通过单元测试验证：

- Ego 原点对应的像素；
- X/Y 与图像 row/column 的映射；
- 左右方向；
- 边界 Cell 规则；
- Height 和 Density 的归一化方式；
- 历史不足四帧时的填充与 mask；
- 训练和在线推理使用完全相同的 BEV 生成实现。

若增加 Observed Mask，实际网络输入通道数应相应调整，不强制维持 13 通道。

## 8. Global Path 编码

全局路径应先确定与车辆对应的最近进度，并沿路径单调向前截取局部段，再转换到 Ego Frame。

禁止仅按空间距离选择所有附近路径点，否则在回环或平行道路附近可能选中错误分支。

第一版至少应提供：

- Route Mask；
- 路径行驶方向或沿程 Progress 信息。

可以采用以下一种编码：

```text
方案 A：route_mask + route_progress 两个通道
方案 B：单通道内用沿路径递增数值表示方向
```

车辆明显偏离全局路径，或全局路径与更新后的静态地图冲突时，应触发 A* 重规划，而不是无限使用旧路径。

## 9. Ego State

第一版至少输入：

```text
[v, omega]
```

Ackermann 车辆推荐扩展为：

```text
[v, omega, steering_angle, acceleration]
```

优先使用 odom 或底盘真实反馈，不以 `/cmd_vel` 作为唯一真实状态来源。所有输入量必须归一化，并在训练与部署中保持相同定义。

## 10. 网络输出定义

### 10.1 推荐输出

第一版网络输出：

```text
15 × [delta_x, delta_y, v]
+ trajectory_confidence
+ stop_probability
```

其中：

- 坐标表达在时刻 `t` 的 Ego Frame；
- `delta_x`、`delta_y` 经累积得到未来位置；
- `v` 为带符号或非负纵向速度，其约定必须固定；
- `trajectory_confidence` 和 `stop_probability` 只能辅助安全决策，不能替代确定性检查。

也允许第一版使用 `15 × [x, y, v]`，但训练时应加入位置、速度和时间的一致性损失。

### 10.2 不由网络直接预测的字段

以下字段由轨迹适配器确定性生成：

- ROS Header；
- `time_from_start`；
- `sampling_interval`；
- yaw 和四元数；
- 二维速度向量；
- 二维加速度向量；
- 角速度；
- 曲率；
- jerk；
- 累计弧长。

原因是这些字段存在严格的几何和运动学关系。让网络独立输出会产生位置、速度、yaw、曲率和角速度互相矛盾的问题。

## 11. 轨迹适配器需求

轨迹适配器位于网络输出和 Safety Checker 之间，必须完成：

1. 检查点数、有限数值、坐标范围和速度范围；
2. 累积增量位置；
3. 对离散位置做有限平滑；
4. 将 0.2 s 网络轨迹重采样到 MPC 所需时间分辨率；
5. 根据轨迹切向计算 yaw；
6. 根据相邻点计算弧长；
7. 根据 yaw/弧长计算曲率；
8. 根据 `omega = v * curvature` 计算角速度；
9. 根据速度有限差分计算纵向加速度和 jerk；
10. 将纵向速度和加速度投影成 Map Frame 二维向量；
11. 使用观测基准时刻位姿完成 Ego Frame 到 Map Frame 的转换；
12. 填充 `RobotTrajectory` 所有字段。

适配器应处理低速或相邻点重合时的数值退化，避免除零和曲率爆炸。

适配器只负责格式和数学一致性。明显不可执行的轨迹不得通过简单裁剪伪装修复，应交由 Safety Checker 拒绝并进入安全制动。

## 12. MPC 滚动轨迹接口改造

### 12.1 当前阻断问题

当前控制器收到每一条 `RobotTrajectory` 都调用 `MpcController::reset()`，并执行：

- 重置最近点索引；
- 重置起点航向调整；
- 将局部轨迹首点作为任务起点；
- 将局部轨迹末点作为任务终点；
- 重置控制状态和到达状态；
- 清空里程计轨迹和终点统计。

同时，控制状态变化会触发约 0.5 s 的阻塞式停车。若局部规划器以 10 Hz 发布轨迹，MPC 会不断被重置并反复停车，无法连续跟踪。

### 12.2 必须实现的接口语义

MPC 应区分：

```text
新全局任务：允许重置任务状态
新局部轨迹：只更新参考轨迹，不重置状态机
```

建议接口：

```cpp
bool startMission(const MissionGoal& goal);
bool updateLocalTrajectory(
    const robot_trajectory_msgs::RobotTrajectory& trajectory);
void cancelMission();
ControlCommand computeCommand(const RobotState& state);
```

`startMission()` 负责：

- 保存真正的全局任务终点；
- 重置到达状态；
- 重置上一任务控制历史；
- 如确有需要，只执行一次起始航向调整。

`updateLocalTrajectory()` 负责：

- 验证并原子替换最新局部参考轨迹；
- 在新轨迹内重新关联当前车辆最近点；
- 保留控制状态、任务终点、上一控制量和到达状态；
- 不触发起始航向调整；
- 不触发状态切换停车；
- 不清空完整里程计轨迹记录。

局部轨迹数组索引与上一条轨迹不具有直接对应关系。新轨迹到来后应在整条短轨迹上重新进行位置和航向联合匹配，不应盲目继承旧索引，也不应把控制状态重置为初始状态。

### 12.3 任务终点与局部时域终点

控制器应分别保存：

```text
mission_goal：真正的全局导航终点
local_horizon_end：当前局部轨迹末点
```

- `mission_goal` 用于判断整个导航任务是否完成；
- `local_horizon_end` 仅用于判断当前参考轨迹是否即将耗尽；
- 不得将每条 3 s 局部轨迹的末点当作全局任务终点。

### 12.4 非阻塞状态转换

当前阻塞式停车循环应改为非阻塞状态计时。正常的局部参考更新不得触发停车。

只在确有必要的模式变化中进入过渡停车，例如：

- 前进切换为倒车；
- 跟踪切换为终点原地转向；
- 系统故障切换为紧急停止。

即使处于过渡停车状态，主循环也必须继续处理 odom、轨迹和安全状态回调。

## 13. Safety Checker 需求

网络轨迹不得直接交给 MPC。Safety Checker 至少检查：

- NaN、Inf、点数异常和时间不单调；
- 轨迹首点与当前车辆状态的连续性；
- 车辆完整矩形 footprint 碰撞；
- 相邻轨迹点之间的 swept volume；
- 静态障碍距离；
- 动态障碍的时序占用；
- 速度、加速度、减速度；
- 曲率、前轮转角和转角变化率；
- jerk；
- 末端剩余制动距离；
- 轨迹、点云和定位数据的新鲜度；
- 模型置信度和推理状态。

安全检查必须使用车辆真实尺寸、参考点位置、轴距和安全余量。

### 13.1 失败处理

不安全轨迹不得仅转换为所有点速度为零的原地轨迹。车辆运动中应生成满足最大减速度和转角连续性的受控制动轨迹。

系统必须具有独立 watchdog，建议初始策略：

```text
轨迹年龄 < 0.2 s：正常跟踪
轨迹年龄 0.2～0.5 s：限速并准备减速
轨迹年龄 > 0.5 s：执行受控制动
定位或点云严重超时：保持停车并上报故障
```

具体阈值应通过实测调整。

## 14. 网络结构与训练

第一版采用 CNN + MLP：

```text
Temporal BEV + Route -> CNN -> Scene Feature
Ego State             -> MLP -> Ego Feature
Scene Feature + Ego Feature -> Decoder -> Local Trajectory
```

第一版使用行为克隆，但训练目标不应只有位置和速度 MSE。建议至少包含：

- 位置损失；
- 速度损失；
- 航向/切向损失；
- 平滑性损失；
- 曲率约束损失；
- 位置变化与速度的运动学一致性损失；
- 障碍碰撞代价；
- 全局路径进度损失；
- 停车场景加权。

多种避障方式同时合理时，单纯 MSE 可能把左绕和右绕平均成碰撞轨迹。第一版数据应尽量保持相似场景中的专家决策一致；后续再考虑多模态输出、候选轨迹或生成式模型。

## 15. 数据采集与样本构造

专家数据至少包含：

- 无障碍路径跟随；
- 左右转弯；
- 静态障碍绕行；
- 绕障后回归路径；
- 主动偏离后的恢复；
- 不同横向误差和航向误差；
- 不同初始速度；
- 动态障碍减速和停车；
- Safety Checker 触发及恢复场景。

建议记录：

```text
/lio/cloud_imu
/lio/robo/odom
/astar_path
/tf
/tf_static
/cmd_vel
底盘真实速度
真实前轮转角
全局任务目标
```

标签是从当前时刻开始的未来实际轨迹，不是当前控制指令。未来位姿必须转换到当前时刻 Ego Frame。

建议明确第一点语义：

```text
输入时刻为 t
第 i 个标签时刻为 t + (i + 1) * 0.2 s
i = 0 ... 14
```

不得让第一个预测点同时表示当前时刻和未来 0.2 s。

训练集、验证集和测试集应按完整路线或完整采集批次划分，避免相邻帧跨集合造成数据泄漏。

## 16. 实施阶段

### 阶段 0：冻结基础约定

- 确认 `map/world/robot/imu` TF；
- 确认车辆轴距、尺寸、参考点和转角限制；
- 确认话题、时间戳和轨迹消息语义；
- 建立参数文件和诊断状态定义。

### 阶段 1：打通无网络滚动闭环

- 改造 MPC 的任务接口和局部轨迹更新接口；
- 实现非阻塞状态转换；
- 实现轨迹 watchdog；
- 实现轨迹适配器；
- 实现 Safety Checker 基础版本；
- 使用“截取全局路径”作为假网络输出，以 10 Hz 发布局部轨迹；
- 验证 MPC 50 Hz 连续跟踪且不会因参考更新而重置。

### 阶段 2：数据与感知输入

- 完成 rosbag 记录规范；
- 完成时间同步和位姿插值；
- 完成未来轨迹标签；
- 完成点云过滤和运动补偿；
- 完成 Temporal BEV；
- 完成 Route 编码；
- 建立离线可视化和一致性测试。

### 阶段 3：基础网络

- 实现 CNN + MLP；
- 完成无障碍行为克隆训练；
- 对比网络输出、专家轨迹和全局路径；
- 通过离线几何、运动学和碰撞指标。

### 阶段 4：静态障碍闭环

- 加入静态障碍绕行数据；
- 加入偏离与恢复数据；
- 在仿真中完成绕行和回归；
- 收集闭环失败样本并迭代。

### 阶段 5：动态障碍

- 第一目标为可靠减速和停车；
- 验证动态障碍接近时的制动距离；
- 动态绕行作为后续增强目标；
- 必要时引入 DAgger 或闭环数据聚合。

### 阶段 6：低速实车

- 限制最高速度和测试区域；
- 配置物理急停和人工接管；
- 先验证无障碍，再验证静态障碍，最后验证动态目标；
- 记录所有安全触发和失败 Case。

## 17. 验收标准

### 17.1 工程接口

- 神经规划器以 10 Hz 更新局部轨迹时，MPC 不重置任务状态；
- MPC 以 50 Hz 连续运行，不因轨迹更新阻塞 0.5 s；
- 局部轨迹更新不清空完整里程计轨迹；
- 全局任务终点与局部时域终点完全分离；
- 所有输出轨迹时间单调、弧长单调且字段有限；
- 神经规划器与 `trajopt` 不会同时向同一控制话题发布。

### 17.2 轨迹质量

- 轨迹首点与当前状态连续；
- yaw 与位置切向一致；
- `omega = v * curvature` 在允许误差内成立；
- 速度、加速度、曲率、转角和 jerk 不超过车辆限制；
- Ego 到 Map 转换可通过已知测试轨迹验证；
- Safety Checker 能拒绝 NaN、跳变、碰撞和不可执行轨迹。

### 17.3 闭环行为

- 无障碍时稳定沿全局路径行驶；
- 静态障碍出现时生成安全绕行轨迹；
- 绕障结束后重新回归全局路径；
- 偏离路径后具备一定恢复能力；
- 动态障碍出现时第一版能够可靠减速或停车；
- 轨迹断流、推理超时或网络异常时能够受控制动；
- MPC 能稳定跟踪通过 Safety Checker 的滚动局部轨迹；
- 只有真正到达全局任务终点时才发布任务完成状态。

## 18. 风险与后续增强

主要风险：

- 行为克隆的闭环分布偏移；
- 左绕/右绕多模态决策被 MSE 平均；
- 0.3 s 历史对动态目标预测能力有限；
- 定位、点云和全局路径坐标系不一致；
- 未观测区域被错误视为空闲；
- 网络输出与车辆动力学不一致；
- 实车推理超时或输入数据陈旧。

完成基础闭环后再考虑：

```text
CNN -> Transformer
单轨迹回归 -> 多候选轨迹
MSE -> 多模态损失 / Diffusion / Flow Matching
Human Demonstration -> Human + Traditional Planner Expert
BC -> DAgger / Closed-loop Data Aggregation
LiDAR BEV -> Camera / Multi-modal Representation
Unsafe -> Stop -> MINCO/传统局部规划器 Fallback
```

## 19. 最终技术决策

当前阶段采用以下方案：

1. 新增独立 `neural_local_planner` ROS 1 功能包；
2. 保留 A* 和 MPC；
3. 保留 MINCO 作为对照和后续 fallback；
4. 网络只输出最小局部轨迹变量，不直接预测完整 ROS 消息；
5. 使用确定性轨迹适配器生成 `RobotTrajectory`；
6. 在接入神经网络前先完成 MPC 滚动轨迹更新改造；
7. 所有网络输出必须经过 Safety Checker；
8. 第一版动态障碍目标以可靠减速和停车为主。
