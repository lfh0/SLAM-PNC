# 技术决策记录

> 文件名 `DESICIONS.md` 按当前项目约定保留。

## DEC-LA-004：LA-03 双前端与动态地图处理边界

- 状态：已实现基础接口，待运行态验收
- 日期：2026-09-16

LA-03保留`hybrid_astar`为默认前端：仅前进Ackermann原语，局部实例使用Dubins终点连接；全局前端仍保留Reeds-Shepp接口和原有默认行为。新增`front_end: astar`可选二维A*前端，复用现有`front2end_search` A*核心并使用节点私有`local_astar`参数，避免覆盖全局A*配置。A*每轮搜索前后必须`reset()`，超时必须返回失败，禁止八邻域从占据格夹角对角穿越。

局部A*的通行语义固定为：最新局部costmap中`cost < occupied_threshold`可通行，`cost >= occupied_threshold`及未知格不可通行；不增加A* footprint膨胀。阈值以下保留原始cost梯度，并按全局A*同一软代价公式惩罚高cost区域。由于二维A*不具备Ackermann与完整footprint保证，它仅作为几何候选，后续必须经局部MINCO、F5和轨迹仲裁处理。

每轮搜索使用固定地图快照；`reset()`仅清空搜索器内部状态，不修改地图。当前尚未实现地图版本失效检查、异步取消或超时/无路细分状态，故LA-03不得接入在线控制链路。

## DEC-F1-001：F1坐标系与车辆参考点

- 状态：已接受
- 日期：2026-09-15
- 适用范围：模块一F1，以及后续轨迹适配、Safety Checker和MPC输入

### 决策

正式规划控制链使用：

```text
map -> world -> robot -> imu
```

物理语义冻结为：

```text
robot = IMU中心 = 车辆旋转中心 = 规划与控制参考点
```

因此：

- `/lio/robo/odom` 是F1的正式位姿、速度和角速度来源；
- `robot -> imu` 使用单位变换；
- 不执行IMU到控制参考点的杆臂补偿；
- PNC内部统一使用 `map` 作为全局坐标系、`robot` 作为车辆坐标系；
- 轨迹适配器和Safety Checker均以 `robot` 为车辆参考点。

### Gazebo真值隔离

Gazebo车辆真值链与LIO估计链保持独立：

```text
LIO估计链：    map -> world -> robot -> imu
Gazebo车模链： car1_base_footprint -> base_link -> imu_link -> mid360_link
```

禁止发布 `robot -> car1_base_footprint`、`robot -> base_link` 或其他用于强制绑定估计位姿与仿真真值位姿的TF。Gazebo真值只用于评估定位与控制误差，不作为规划控制的正式状态输入。

### 理由

LIO位姿是估计值，Gazebo车辆位姿是真值。将二者通过静态TF强制绑定会混淆两个状态来源，并可能造成同一child frame存在多个父节点。保持独立可保留真实定位误差并支持客观评估。

## DEC-F1-002：复用Super-LIO车辆状态输出

- 状态：已接受
- 日期：2026-09-15

### 决策

F1不新增重复的车辆状态发布、缓存或TF节点，直接复用Super-LIO已经提供的：

```text
/lio/robo/odom：world坐标下robot参考点的位姿、速度和角速度
/lio/cloud_imu：imu坐标下的点云
TF：map -> world -> robot -> imu
```

后续模块各自按业务需要缓存带原始时间戳的odom，并通过现有TF在消息时刻进行坐标变换。不得新增内容相同但时间或坐标语义不同的“统一车辆状态”话题。

运行态确认结果：

- `/lio/robo/odom` 约200 Hz，平均传输延迟约2 ms；
- `/lio/cloud_imu` 约10 Hz，平均传输延迟约7 ms；
- `/lio/robo/odom.header.frame_id = world`；
- `/lio/robo/odom.child_frame_id = robot`；
- `/lio/cloud_imu.header.frame_id = imu`；
- `map -> robot` 和 `map -> imu` 可通过TF查询。

## DEC-F1-003：车辆参数来源

- 状态：已接受
- 日期：2026-09-15

### 决策

- 当前仿真车辆轴距以Gazebo控制模型的 `0.4 m` 为准；
- MPC轴距从 `0.5 m` 修正为 `0.4 m`；
- 当前未启用Hybrid A*，其 `2.0 m` 参数不属于本次F1修改范围；
- 后续若启用Hybrid A*，必须在启用前与车辆实际参数同步；
- MPC配置当前记录轴距 `0.4 m`，其他尚未实测的车身参数不得臆测为正式值。

## DEC-F1-004：状态来源边界

- 状态：已接受
- 日期：2026-09-15

### 决策

- F1只确认定位链和当前控制所需状态来源，不重复实现Super-LIO已有输出；
- 位姿、线速度和角速度直接来自 `/lio/robo/odom`；
- `/cmd_vel` 是控制命令，不作为车辆真实反馈；
- 当前MPC接口未要求F1额外发布统一转角或加速度消息，因此不在F1新增；
- 后续训练Ego State若需要真实转角和加速度，在训练输入模块中接入底盘/仿真反馈并独立定义有效性。

## DEC-F1-005：尚未冻结的标定量

以下内容不阻塞当前F1完成，但在相应后续模块开始前必须确认：

1. 当前硬编码 `map -> world` 的Z轴 `2.0 m` 是否为真实地图标定；
2. 车辆轮距、footprint、前后悬及真实转角限制；
3. 正式的定位健康质量来源。目前odom协方差全零，后续Safety Checker/watchdog不能将全零理解为零不确定性；
4. 数据过期、位置跳变和yaw跳变阈值在F4/F5/F6实现时通过仿真与实车统计调整。

## DEC-F2-001：任务生命周期与滚动局部轨迹分离

- 状态：已接受并实现
- 日期：2026-09-15

MPC使用 `startMission()`、`updateLocalTrajectory()` 和 `cancelMission()` 三个独立接口。`/move_base_simple/goal`启动新任务，只有新任务允许清空控制状态；`/trajopt/minco_traj`只更新当前任务的局部参考。局部轨迹末点不再覆盖全局任务终点，新任务到达后旧局部轨迹立即失效。

局部轨迹在接收时完整复制，轨迹、任务状态与持久化求解器由同一互斥锁保护；因此即使以后改为多线程Spinner，控制计算也不会观察到半更新轨迹。

## DEC-F2-002：新局部轨迹重新匹配

- 状态：已接受并实现
- 日期：2026-09-15

新轨迹不继承旧数组索引，也不固定从索引0开始。每条新轨迹在整条短轨迹上使用位置与航向联合代价重新匹配当前车辆。匹配距离、航向误差和航向权重均通过参数配置；匹配失败时拒绝新轨迹并保留已有有效参考。

## DEC-F2-003：起始航向与任务完成

- 状态：已接受并实现
- 日期：2026-09-15

- 起始航向只在每个任务的第一条有效局部轨迹到达后锁存；
- 后续滚动轨迹更新不重置起始航向状态；
- 任务完成只根据 `/move_base_simple/goal`判断；
- 到达局部时域末点不能触发 `Finished`；
- 新任务进入 `WaitingForTrajectory`，没有有效轨迹时输出零命令。

## DEC-F2-004：非阻塞控制循环

- 状态：已接受并实现
- 日期：2026-09-15

删除“任意控制状态变化都阻塞发布0.5秒零命令”的通用逻辑。正常状态变化不再阻塞ROS回调与50 Hz控制循环。受控制动、前进/倒车切换和故障Stop Hold属于F6安全降级职责。

## DEC-F2-005：OSQP跨周期热启动

- 状态：已接受并实现
- 日期：2026-09-15

OSQP求解器作为 `MpcController`持久成员。QP维度不变时更新矩阵、梯度和上下界，并使用上一周期解设置primal warm start；更新失败或维度变化时自动重新初始化。只有新全局任务或取消任务清空求解器状态，局部轨迹更新不清空热启动。

## DEC-F2-006：第一版消息兼容策略

- 状态：已接受并实现
- 日期：2026-09-15

第一版不修改 `RobotTrajectory`消息，使用独立目标话题、内部 `mission_id`、内部轨迹序列号和任务开始时间管理任务边界。显式消息级 `mission_id`留到F3/F4统一轨迹协议时再评估，避免阻塞现有MINCO链路。

## DEC-F3-001：统一原始局部轨迹协议

- 状态：已接受并实施
- 日期：2026-09-15

基线与未来神经网络统一输出 `robot_trajectory_msgs/RawLocalTrajectory`。消息携带观测时间、任务ID、序列号、来源、固定采样周期、置信度、停车概率和任务终点标志；点字段为观测时刻 `robot` 坐标系下的 `[x,y,yaw,velocity]`。`robot` 已确认同时是IMU中心与车辆旋转中心，故不再使用文档中的 `base_link(t_obs)`表述。

## DEC-F3-002：独立确定性基线节点

- 状态：已接受并实施
- 日期：2026-09-15

新增独立 `raw_local_planner` 包，不把F3放入控制器或MINCO。节点接收 `/astar_path`、`/lio/robo/odom`和`/move_base_simple/goal`，以10 Hz生成原始局部轨迹。F3不直接发布给MPC；历史观测轨迹到Map坐标、延迟裁剪、执行状态连接及完整 `RobotTrajectory`字段生成属于F4。

## DEC-F3-003：A*确定性轨迹生成算法

- 状态：已接受并实施
- 日期：2026-09-15

选用“单调进度匹配、受限局部平滑、曲率/终点联合限速和固定时间采样”。新任务或新路径执行全局匹配，后续只在上次进度邻域内匹配并限制回退；平滑点相对原路径的位移受限；速度同时满足车辆速度、横向加速度、纵向加减速度和终点制动约束。普通局部窗口末端不强制停车，只有真实任务终点进入并被采样到时才设置 `contains_mission_goal=true`。

MINCO继续作为传统回退、效果对照和未来专家标签来源，不作为F3主链路；直接等距固定速度采样只适合接口调试，不进入正式方案。

## DEC-F4-001：原始轨迹适配方案

- 状态：已接受并实施
- 日期：2026-09-15

F4采用独立 `trajectory_adapter` 节点，将 `RawLocalTrajectory` 转换为当前MPC兼容的 `robot_trajectory_msgs/RobotTrajectory`。处理顺序固定为：按 `observation_stamp` 查找历史里程计、将观测时刻 `robot` 坐标转换到Map、根据当前时间裁剪已过期点、按固定时间重采样并补齐速度/加速度/角速度/弧长字段。

## DEC-F4-002：历史状态和延迟处理

- 状态：已接受并实施
- 日期：2026-09-15

适配器缓存有限长度的 `/lio/robo/odom`历史，优先按时间插值获取 `observation_stamp` 位姿；查找误差超过阈值则拒绝轨迹。执行时刻使用最新里程计，原始轨迹时间减去观测年龄后裁掉过期段；不对整条历史轨迹做平移对齐，不建立仿真真值与LIO之间的TF。

## DEC-F4-003：连续接管和输出协议

- 状态：已接受并实施
- 日期：2026-09-15

不再人为插入“当前车位到参考轨迹”的连接段：F3的正常局部轨迹已从MINCO参考上的最近匹配点开始，F4必须保留该几何起点，否则MPC会错误地认为横向误差接近零。输出固定Map坐标，候选话题为`/safety_checker/candidate_trajectory`；线速度和加速度均转换为Map分量。F4不执行碰撞检查，安全准入属于F5。

## DEC-F4-004：方案取舍

- 状态：已接受

不采用“整条轨迹平移到当前车位”（会破坏障碍物相对位置），不采用“只做TF转换保留过期前段”（会把历史状态直接喂给MPC），也不把F4塞入MINCO或控制器。独立适配器在实时性、可测试性和未来神经网络兼容性之间最优。

## DEC-F2-007：预测时域与新任务起始转向

- 状态：已接受并实施
- 日期：2026-09-15

适配器输出固定时间间隔的局部预测窗口，MPC预测步数由`mpc.yaml`配置，避免短预测造成路径偏离。新任务由F3发布零线速度、目标yaw的原地对齐参考，MPC进入原地航向调整；起始调整最小角速度提高到0.12 rad/s，避免小角速度抖动。正常滚动轨迹不重置该状态。

## DEC-F2-008：调试可视化与一键启动

- 状态：已接受并实施
- 日期：2026-09-15

新增`/local_planner/raw_path_map`和`/trajectory_adapter/debug_path`两个Map坐标调试路径；新增`raw_local_planner/launch/full_navigation.launch`，统一启动A*、F3、F4和MPC。F4独占`/trajopt/minco_traj`，不与MINCO同时启动。

## DEC-F3-004：局部轨迹首点与车辆状态一致

- 状态：已接受并实施
- 日期：2026-09-15

仅在任务启动的原地航向对齐阶段，F3输出当前车辆位置上的零线速度参考。正常跟踪时，F3从MINCO全局轨迹的最近匹配点开始生成局部轨迹，保留车辆到参考线的真实横向误差；不再把首点强行改写为当前车辆位置，避免F4/MPC错误地把误差视为零。

## DEC-F3-005：F3改用MINCO全局优化轨迹

- 状态：替代DEC-F3-003中的“A*直接作为F3输入”
- 日期：2026-09-15

正式链路改为“前端路径搜索 → MINCO全局优化 → F3局部规划”。A*仅作为MINCO内部上游输入，F3订阅`/trajopt/global_trajectory`，不再订阅`/astar_path`。由于MINCO已经完成几何连续优化，F3关闭二次拉普拉斯平滑，只执行去重、等弧长重采样、单调进度截取、局部时间采样和车辆约束速度规划。

MINCO全局轨迹发布到`/trajopt/global_trajectory`；F4输出的MPC局部轨迹发布到`/controller/local_trajectory`。两个话题必须分离，避免全局优化轨迹绕过F3/F4直接进入控制器，也避免多个发布者竞争同一话题。
## DEC-F3-006：局部规划启动阶段先完成航向对齐

- 状态：已接受并实施
- 日期：2026-09-15

F3采用“位置匹配 → 原地航向对齐 → 正常局部跟踪”的启动状态机。车辆尚未对齐MINCO轨迹切线方向时，轨迹匹配阶段仅使用位置误差，避免因初始yaw差异导致无法生成局部轨迹；输出点全部位于当前车辆位置、线速度为零、yaw指向匹配点切线，交由MPC执行原地旋转。航向误差小于`yaw_alignment_tolerance`（默认0.12 rad）后，才恢复位置+yaw联合匹配和正常速度规划。正常跟踪中若航向再次严重偏离，也会重新进入对齐状态。
## DEC-F5-001：静态栅格轨迹准入安全检查

- 状态：已接受并实施
- 日期：2026-09-15

F5新增独立`trajectory_safety_checker`包，位于F4与MPC之间。它订阅F4候选轨迹`/safety_checker/candidate_trajectory`、`/lio/robo/odom`和`/local_costmap_node/costmap/costmap`，仅在轨迹同时通过新鲜度、数值/时间单调性、起点连续性、速度/加速度/jerk/曲率约束，以及完整车辆矩形footprint和相邻点swept-volume静态栅格碰撞检查时，原样发布到`/trajectory_manager/safe_trajectory`。拒绝时不伪造零速度轨迹，仅发布结构化文本状态和拒绝可视化；F6和MPC的watchdog负责停止。动态障碍时序占用和受控制动轨迹属于后续增强，不在F5基础版本伪实现。

速度、加速度、jerk和曲率的检查使用`dynamics_tolerance=0.02`作为浮点重采样/有限差分比较余量；车辆物理约束本身不变。例如`max_acceleration=0.5`时，F5在超过`0.52 m/s²`才拒绝。
## DEC-F6-001：唯一安全轨迹发布与断流停止

- 状态：已接受并实施
- 日期：2026-09-15

新增`trajectory_manager`作为MPC的唯一轨迹发布源。F5的合格轨迹发布到`/trajectory_manager/safe_trajectory`，F6只转发该来源到`/controller/reference_trajectory`；MPC改订阅最终话题。F6在0.4秒未收到合格安全轨迹时停止发布并发布`STOPPED reason=SAFE_TRAJECTORY_TIMEOUT`。随后MPC在自身0.5秒watchdog到期后切换`WaitingForTrajectory`并输出零命令。当前底盘接口没有独立的、经碰撞证明安全的制动轨迹执行通道，故拒绝时不生成可能继续驶入障碍物的伪制动轨迹。

## DEC-F7-001：模块一基线闭环验收范围

- 状态：已接受并实施
- 日期：2026-09-15

模块一闭环冻结为`MINCO全局轨迹 → F3局部截取/速度剖面 → F4坐标与时间适配 → F5静态栅格安全准入 → F6唯一发布与断流停止 → F2 MPC`。F7验收以各模块单元测试、10Hz候选和安全轨迹频率、50Hz控制频率、静态碰撞拒绝、候选断流停车和转弯跟踪为最低通过标准。动态障碍时序占用、神经网络输入/训练及独立底盘受控制动列入后续模块，不宣称由模块一完成。

## DEC-T7-001：模块二首版网络采用早期融合CNN

- 状态：已接受
- 日期：2026-09-16

模块二首版不因结构复杂度而引入Transformer、ConvGRU、Cross-Attention或生成式轨迹模型。网络输入为4帧Temporal BEV的各通道、Route Mask和输入有效性Mask的早期通道拼接；该栅格张量由单一CNN编码器提取空间和短时变化特征。Ego State通过小型MLP编码后与CNN特征融合，非自回归Decoder一次输出15个未来`[x,y,v]`点，坐标固定为`robot(t_obs)`。

选择依据是当前历史窗口仅0.3秒、首版任务以静态/低速局部导航为主且训练数据规模尚未建立。此方案的数据效率、可解释性、ONNX/TensorRT部署稳定性和实时性优于未经实证的复杂时序结构。训练/测试数据、预处理、损失、随机种子和硬件预算均固定后，只有独立测试集证明时序遮挡、轨迹跳变或安全通过率存在明确瓶颈，并且候选结构在相同数据划分上显著改善这些指标时，才允许以消融实验的结果替换本决策。

## DEC-LA-001：模块四采用局部重搜、局部优化与全局重新汇入

- 状态：已接受
- 日期：2026-09-16

模块四不在局部障碍出现时重做整条全局路径的MINCO优化。正常状态继续使用全局MINCO轨迹经F3截取的局部参考；仅当最新局部代价地图证明当前参考的未来窗口被阻断时，触发局部规划。局部前端使用受车辆运动学约束、默认仅前进的Hybrid A*；它从实际或延迟预测后的车辆状态出发，搜索到全局参考上障碍物之后的可重新汇入状态。局部MINCO只优化该有限窗口。

局部MINCO的起点边界固定为预测执行状态`[p,v,a,yaw,curvature]`，终点边界固定为正常F3/全局MINCO参考在重新汇入点的`[p,v,a,yaw,curvature]`。不允许把起点强行投影到全局参考，也不允许首尾默认零速度；轨迹必须满足位置、速度、加速度连续，并通过F5后才替换正常局部轨迹。F3保持`/local_planner/raw_trajectory`的唯一发布权，局部避障候选使用内部话题或内部数据结构，不新增多个发布者竞争该话题。

只有局部窗口无法找到可行绕行、连续局部规划失败、障碍阻断超出局部窗口，或无法重新汇入全局参考时，才请求全局前端重搜。规划期间旧轨迹只能在最新局部地图复检仍安全且剩余时域充足时短暂复用；否则进入受控制动。时间预算、窗口长度和可复用时域在取得当前局部地图的分辨率、范围与实际耗时统计后再冻结。

## DEC-LA-002：全局与局部MINCO共用优化核心，但使用独立节点和配置

- 状态：已接受
- 日期：2026-09-16

复用现有`trajopt`中的MINCO、走廊构建和车辆约束实现，抽取为不依赖ROS话题的`minco_core`库；全局节点继续承担“前端路径→全局MINCO”，新增局部节点仅承担“局部Hybrid A*几何路径＋明确起终点状态→局部MINCO候选”。二者不得共用同一个节点实例或同一套参数命名空间，以免局部重规划覆盖全局参数、订阅或发布器。

局部优化请求显式携带起终点`[p,v,a,yaw,curvature]`与局部地图/安全走廊，输出带完整`time_from_start`、速度、加速度和航向的候选`RobotTrajectory`。LA-00只完成可复用核心边界和全局回归，不改变现有全局MINCO行为；任何局部轨迹接入F3之前，必须先通过LA-02、LA-04和LA-05的验收。

## DEC-TRJ-001：全局MINCO以最新前端路径抢占旧优化

- 状态：已实现，待运行态验收
- 日期：2026-09-16

`trajopt`仅在收到有效前端`nav_msgs/Path`时创建新的路径版本；新路径覆盖尚未开始的待优化请求，并使正在运行的L-BFGS在下一次迭代进度回调中安全取消。goal不是MINCO的中断条件，因为它本身不含可优化的几何路径。

MINCO在独立工作线程中运行，ROS异步回调线程持续接收地图、里程计和前端路径。每次优化使用路径到达时的地图与里程计快照；优化完成、提取轨迹和正式发布前均检查路径版本。`LBFGS_CANCELED`视为“无结果”，不得发布`/trajopt/global_trajectory`。始终只运行一个优化任务，避免多个MINCO实例争用CPU。

## DEC-LA-003：模块四按七个可独立验收模块实施

- 状态：已接受
- 日期：2026-09-16

全局MINCO不是周期规划器。每个有效全局前端路径只优化一次，得到固定全局参考；只有新的前端搜索结果，或局部绕障明确失败后发起的全局重搜结果，才允许重新全局优化。当前A*和独立Hybrid A*均只在goal回调内搜索并发布一次路径，故LA-00不实现路径去重；只有未来前端改为持续重发同一路径时，才增加几何去重防御。

模块顺序冻结为：`LA-00 全局MINCO请求管理与可复用核心 → LA-01 全局参考窗口与重汇入目标 → LA-02 参考阻断检测 → LA-03 局部Hybrid A* → LA-04 非零边界局部MINCO → LA-05 轨迹仲裁、安全复检与降级 → LA-06 回归测试与训练教师标签`。

LA-00输入为前端路径、全局地图、里程计和MINCO配置，输出为一次全局MINCO结果及`request_id/success/cancelled`状态；验收为路径B在路径A优化中到达时A被取消且仅B允许发布，且单个goal只发布一次全局MINCO。LA-01输出单调局部窗口与重汇入状态，LA-02输出`NORMAL/REPLAN_REQUIRED/BRAKE_REQUIRED`及阻断区间，LA-03输出局部无碰撞几何路径，LA-04输出连续局部候选`RobotTrajectory`，LA-05保持`/local_planner/raw_trajectory`唯一发布权并负责失败降级，LA-06输出固定场景回归报告和训练标签。每项独立验收通过前不得接入下一项在线输出。

| 模块 | 输入 | 输出 | 功能方案 | 独立验收 |
| --- | --- | --- | --- | --- |
| LA-00 全局MINCO请求管理与可复用核心 | 前端`Path`、全局地图、里程计、MINCO配置 | 一次全局MINCO、`request_id/success/cancelled` | 当前前端每次goal只发布一次路径；新路径通过L-BFGS进度回调取消旧优化；已落地`minco_core`和`MincoConfig/MincoRequest/MincoResult`边界，未来持续发布时才增加去重 | 离线已验证边界拒绝、非零速度边界、取消回调；运行态待验收B在A优化期间到达时A不发布、只发布B，且单个goal只发布一次 |
| LA-01 全局参考窗口与重汇入目标 | 全局MINCO、odom、局部地图范围、上次匹配进度 | 单调局部窗口、重汇入状态`[p,v,a,yaw,curvature]` | 沿全局参考稳定匹配车辆进度，以单调索引和回退滞回避免平行支路跳变；选择障碍之后且位于局部地图内的重汇入点 | 直线、回环、平行相邻路径中匹配不跳支；重汇入点在地图内且在阻断区之后 |
| LA-02 参考阻断检测 | 最新local_costmap、车辆footprint、LA-01窗口、当前速度 | `NORMAL/REPLAN_REQUIRED/BRAKE_REQUIRED`、阻断区间 | 对未来参考做footprint和扫掠体碰撞检查；结合剩余距离、速度和制动距离判断绕障或制动 | 空地图/参考外障碍不误触发；覆盖轨迹扫掠体时触发；近距离障碍进入`BRAKE_REQUIRED` |
| LA-03 局部Hybrid A* | 预测执行起点、LA-01重汇入状态、local_costmap、车辆模型 | 局部无碰撞几何路径或失败原因 | 默认仅前进，使用车辆运动学、最小转弯半径和碰撞代价，从实际状态搜索并接回全局参考 | 直通、左绕、右绕、窄通道、无解五类场景；验证起终点、无碰撞和耗时预算 |
| LA-04 非零边界局部MINCO | LA-03路径、局部安全走廊、起终点P/V/A状态、车辆约束 | 连续局部候选`RobotTrajectory` | 将局部几何路径优化为时间参数化轨迹；边界yaw由速度方向编码，曲率由动力学约束验收；首尾不得默认零速度 | 起终点位置/速度/加速度连续；速度、加速度、jerk、曲率与走廊约束均通过 |
| LA-05 轨迹仲裁、安全复检与降级 | F3正常参考、LA-04候选、LA-02状态、F5结果、最新地图 | 唯一`/local_planner/raw_trajectory`、仲裁状态 | 正常走F3；阻断后接管局部候选；失败/超时先复检旧轨迹，不能安全则受控制动；禁止多发布者竞争 | 正常、绕障成功、无解、超时、F5拒绝五类场景；输出话题始终只有一个发布者 |
| LA-06 回归测试与训练教师标签 | LA-00至LA-05状态、rosbag、场景注释 | 回归报告、正常/绕障/停车专家标签 | 固定场景回放验证端到端行为；传统局部绕障结果作为训练数据教师 | 直行、左右绕障、无解停车、重新汇入自动回放通过；标签与仲裁状态一致 |

### LA-01实现冻结（2026-09-16）

采用独立的`local_reference_manager`包，不修改当前F3/F4/F5/F6/MPC，也不把该节点加入现有全导航launch。输入固定为全局MINCO`/trajopt/global_trajectory`、里程计`/lio/robo/odom`和局部代价地图`/local_costmap_node/costmap/costmap`；输出固定为`/local_reference/window`、`/local_reference/rejoin_state`和`/local_reference/debug_window`。`ReferenceWindow`消息携带窗口点、单调`progress_index`、`rejoin_index`、匹配距离和完整重汇入P/V/A状态。

匹配方案采用“有界最近点 + 单调索引”：首次全局轨迹全局搜索，后续只在上次进度附近搜索；最终索引不得小于已有进度。窗口沿累计弧长截取，且窗口与候选重汇入点必须位于local costmap边界内并留出`map_margin_m`。若定位偏差超过`max_match_distance_m`，拒绝输出而不跳转到另一支路。LA-01不推断障碍后的位置；LA-02只报告阻断区间，LA-03从该区间之后的窗口点选择实际重汇入点。代码和离线测试完成，待仿真运行态验收。

### LA-02实现冻结（2026-09-16）

采用独立`local_obstacle_monitor`包，输入`/local_reference/window`、`/local_costmap_node/costmap/costmap`和`/lio/robo/odom`，输出`/local_reference/blockage`及调试碰撞样本；不接入当前控制链路，不发布轨迹和控制量。消息状态固定为`NORMAL`、`REPLAN_REQUIRED`、`BRAKE_REQUIRED`，并携带阻断索引、阻断距离、制动距离和首个碰撞栅格代价。

碰撞模型固定为以`robot`中心（也是IMU与车辆旋转中心）的矩形footprint，长宽、余量和采样步长均参数化。每个窗口点检测footprint，并在相邻点之间以不大于半个栅格分辨率插值检测扫掠体；地图边界外和未知栅格默认按障碍处理。制动判定固定为`v*t_reaction + v²/(2*a_brake) + margin`。配置参数必须在仿真验收前依据实际车辆尺寸校准。代码与四类离线测试完成，待仿真运行态验收。

### LA-03实现冻结（2026-09-16）

采用独立`local_hybrid_astar`包，不复用全局Hybrid A*节点，原因是局部规划必须消费LA-01/LA-02的窗口、阻断索引和实时车辆状态，且必须保证单独的时间预算与不接管现有控制链路。输入为参考窗口、阻断状态、局部costmap和odom；输出仅为`/local_replanner/geometric_path`与状态字符串。当前版本默认只允许前进，避免局部搜索擅自输出倒车；倒车只能在未来经专门的仲裁与车辆能力验收后开放。

只有`REPLAN_REQUIRED`触发搜索；`BRAKE_REQUIRED`不启动搜索，输出明确状态并等待后续LA-05制动策略。起点不是当前odom点，而是全局参考窗口中可执行的短时预测状态：默认请求前视0.5 m，实际值为`min(0.5 m, blockage_distance-braking_distance-start_before_brake_margin)`，即制动余量不足时自动缩短。可用距离非正时直接拒绝搜索。这保证局部绕障轨迹从控制器可衔接的预测状态出发，并且预测起点在最晚制动起点之前。目标为阻断索引之后至少3 m的窗口点，防止从障碍前的候选重汇入点开始搜索。2026-09-16起，LA-03直接复用`front2end_search`的`path_searching::HybridAstar`搜索核心，不再使用自写简化运动原语。局部代价地图经阈值二值化适配后传入该核心；与全局前端共用同一份`hybridastar.yaml`，从而统一Ackermann模型、车辆尺寸、转向离散、圆弧采样、footprint碰撞检查、解析连接、搜索代价与时间上限。通过`init(nh, false)`禁用其原有全局话题接口，避免抢占目标、地图或可视化话题。LA-03对该核心实例设置`allow_reverse=false`和`enable_analytic_expansion=false`：搜索树仅扩展前进原语，且禁用可能含倒车段的Reeds-Shepp终点连接；全局前端实例保持默认行为。代码已编译，待运行态验收。

局部A*时间预算固定使用显式毫秒参数`local_astar/max_search_time_ms`，当前为`50.0 ms`，并用墙钟而非ROS仿真时间计时。搜索结果必须区分`TIMEOUT`与`NO_PATH`，未构造出路径时绝不返回成功。开放集采用不可变`f_score`快照；节点降代价时压入新快照，弹出时跳过已关闭或过期条目，保证堆序正确。相同costmap内容与相同起终点栅格只执行一次搜索；时间戳变化不算新请求，地图内容或起终点栅格变化才触发重搜。该去重不替代搜索中的地图版本取消，后者继续作为独立安全任务实施。

### LA-04A实现冻结：A*候选门控与MINCO请求状态机（2026-09-16）

局部A*输出不得直接触发MINCO。LA-03在`NORMAL/BRAKE_REQUIRED → REPLAN_REQUIRED`时创建单调`episode_id`，冻结该episode的世界坐标重汇入状态，并发布带地图内容版本和首尾P/V/A边界的`LocalPathCandidate`。候选管理使用独立`local_optimization_manager`节点，输出`LocalOptimizationRequest`，不与全局MINCO节点共用ROS实例。

第一条在最新地图上有效的候选立即进入`OPTIMIZING`；等价更新不重复请求，实质变化在优化期间只保留最新一条。只有当前路径被最新地图判为碰撞、新episode到达或阻断状态离开`REPLAN_REQUIRED`才立即发`CANCEL`。MINCO失败后才以最新待处理候选重试；MINCO成功后进入`COMMITTED`，安全的A*更新不触发重新优化。阈值固定为终点变化0.5 m、归一化路径横向变化0.3 m、相对长度变化10%和绕行方向死区0.2 m，均可通过私有参数调整。

候选与请求的地图碰撞检查沿路径段按不大于半个栅格分辨率插值，遵循`cost >= occupied_threshold`、未知格和地图外为障碍的语义；这是进入MINCO前的点路径门控，不替代LA-05/F5的完整车辆footprint复检。请求携带0.15 s初始预计接管延迟，但真正的延迟补偿和从已提交轨迹采样P/V/A由LA-04B局部MINCO消费者完成。

### DEC-LA-004：LA-04B采用局部走廊MINCO与可取消工作线程

- 状态：已实现，待运行态验收
- 日期：2026-09-16

局部MINCO消费者固定为`local_optimization_manager/local_minco_optimizer_node`。输入是LA-04A的`/local_optimizer/request`及其对应时刻的局部costmap快照，输出仅是内部候选`/local_optimizer/candidate_trajectory`和带`episode_id/request_id`的`/local_optimizer/status`；在LA-05完成前，禁止直接向F3、F5、F6或MPC发布，避免多源轨迹争用控制链路。

优化核心复用`trajopt`导出的`minco_core`与`MincoConfig/MincoRequest/MincoResult`，不复制MINCO求解器。局部安全走廊暂采用与局部几何路径方向对齐的保守矩形：每段横向宽度从最大可用宽度向车辆半宽、footprint余量收缩，以半栅格采样检查局部costmap；未知、地图外及`cost >= occupied_threshold`一律不可通过。该实现与全局MINCO共享求解核心，但尚未抽取全局节点内部的凸分解走廊生成器；后续若需要更复杂障碍几何，应将两者的走廊接口统一，不能改变当前输出接口。

起终边界严格取请求的P/V/A；不得用几何路径首尾替代实际起点，更不得默认零速度。yaw由速度方向生成，曲率和角速度由速度/加速度导出。求解在单工作线程运行，所有新OPTIMIZE或CANCEL均提升版本号；取消由MINCO迭代回调协作响应，完成后再次校验版本，因此取消或被抢占的旧任务绝无资格发布轨迹。

### DEC-LA-005：局部MINCO与全局MINCO统一关键点、凸走廊与可视化语义

- 状态：已实现，待运行态验收
- 日期：2026-09-16

局部和全局均采用同一策略族：RDP横向偏差递归 + 弦线碰撞检查 + 最大段长切分 + 最小段长保护；短段仅在转角、偏差、长度和无碰撞全部成立时合并。局部走廊不再使用固定宽度矩形，而是使用全局同一`decomp_util::LineSegment2D`凸分解方法，从占据栅格中心提取障碍点，附加地图边界半空间，并验证相邻凸走廊的真实交集面积。

两端继续共用`minco_core`优化器和相同的“一走廊一多项式段、按采样分辨率展开走廊”输入语义；局部与全局仅允许因地图范围、车辆约束和时间预算不同而使用独立参数值。局部发布橙色`/local_optimizer/debug_trajectory`及`/local_optimizer/polyhedrons`；后者与全局`/trajopt/polyhedrons`同为`decomp_ros_msgs/PolyhedronArray`、同一半空间语义和Decomp RViz显示方式，均为latched调试输出，不改变RViz文件和控制链路。

### DEC-LA-006：全局轨迹版本必须贯穿局部重规划链路

- 状态：已实现
- 日期：2026-09-16

`REPLAN_REQUIRED`不是全局参考的身份标识；新全局MINCO轨迹到达时，阻断状态可连续不变。故`ReferenceWindow.global_trajectory_id`由LA-01在每次有效全局MINCO更新时递增，LA-02必须无修改写入`ReferenceBlockage.global_trajectory_id`。LA-03仅在两条消息ID相等时规划；ID改变时必须清除冻结重汇入目标和A*缓存，并将阻断状态机复位，使连续的`REPLAN_REQUIRED`也创建新episode。

局部优化请求管理器同样消费`ReferenceWindow`。新ID一到达，立即取消正在优化或已提交的局部MINCO请求、清除缓存候选并重置episode；随后只能接受由新ID窗口和同ID阻断结果触发的新候选。不得以ROS消息时间戳、路径点索引或阻断状态本身替代该版本字段。

### DEC-LA-007：局部MINCO约束、权重和初始时间与全局MINCO统一

- 状态：已实现
- 日期：2026-09-16

局部`local_minco_optimizer.yaml`使用与全局`trajopt.yaml`相同的参数分组和数值：`corridor`的占据阈值、未知栅格、RDP选点、走廊范围与交集面积；`vehicle`的长宽和参考点偏移；`optimizing`的采样分辨率、静态/动态障碍、可行性、时间、锚定权重、动态间距、速度/加速度/曲率及安全余量；`initial_time`的最低速度、最低分段时间和转角降速比例。局部初始时间也改为全局同款“内部节点按转角降速、速度/加速度共同约束分段时间”。

唯一且必要的边界差异是：全局普通导航可用微小速度仅编码路径yaw，而局部绕障必须使用LA-03/LA-01给出的真实起终点P/V/A，以便未来LA-05能连续接入当前执行参考。局部当前没有动态障碍轨迹输入，故`wei_dyn_obs`与`dyn_obs_clearance`已按全局加载但不产生额外动态障碍项；不得误认为这已经实现动态障碍预测。

### DEC-LA-008：局部重汇入终点以连续阻断区尾部之后 3 m 确定

- 状态：已实现，待运行态验收
- 日期：2026-09-16

`blockage_index`只表示首个碰撞位置，不能直接用于设定重汇入终点。`ReferenceBlockage`新增`blockage_end_index`：LA-02从首次footprint/扫掠体碰撞起持续扫描，直到出现一整段无碰撞扫掠；其前一段即连续阻断区尾部，后一参考点才是可安全离开该阻断区的`safe_exit`。首个碰撞距离继续只用于计算制动边界，不能混用为绕障终点。

LA-03固定从`safe_exit = blockage_end_index + 1`开始，沿全局参考弧长再前推`rejoin_distance_after_blockage_m=3.0 m`，将得到的点冻结为本次episode的重汇入目标。若连续阻断区未在参考窗口内结束，输出`NO_SAFE_EXIT_AFTER_BLOCKAGE_IN_REFERENCE_WINDOW`；若安全出口之后窗口长度不足3 m，输出`REJOIN_MARGIN_OUTSIDE_REFERENCE_WINDOW`，不猜测窗口外目标。新的全局轨迹版本仍会清除冻结目标并重新选择。

### DEC-LA-009：局部MINCO可选关联障碍点距离模式

- 状态：已实现，待运行态验收
- 日期：2026-09-16

`corridor/use_safe_corridor_constraints`可选开启；当前局部配置默认关闭，以减少局部优化耗时。开启时保持现有RDP关键点、凸安全走廊构建和半空间越界代价。关闭时，局部MINCO仍构建相同RDP关键点和时间分段，但**不构建凸安全走廊、不展开走廊采样，也不计算任何走廊半空间代价或梯度**。若同时开启`associated_obstacle/enabled`，则在每个MINCO约束采样点附近关联有限数量的局部costmap占据栅格中心，对车辆footprint顶点施加`clearance - distance`的平滑正值惩罚。

关联范围由`associated_obstacle/search_radius_m`、每采样点上限由`max_points_per_sample`、额外距离余量由`clearance_m`配置；车辆尺寸和`optimizing/half_margin`仍参与footprint建模。该模式是软距离约束，不能替代未来LA-05/F5的高频完整footprint复检；关闭走廊时发布空`/local_optimizer/polyhedrons`以清除旧可视化。

### DEC-LA-010：局部候选端点复核与路径失效抢占

- 状态：已实现，待运行态验收
- 日期：2026-09-17

LA-03对每次准备搜索的预测起点和重汇入终点执行完整矩形车辆footprint栅格采样。端点仅在`endpoint_collision_threshold=50`以下、未知栅格语义满足`unknown_is_obstacle`且未越界时可用。起点失效时仅沿全局参考向上游回退到首个可行点；终点失效时沿参考向前寻找首个可行点。冻结终点后继续复核，若它被新地图占据则清除冻结目标及A*输入缓存，并在同一重规划episode选择并冻结新终点。预测起点不静态冻结：车辆在运动，必须保持“接管时刻可达”的时间前视语义；若静态冻结，MINCO完成时该起点已过期。

LA-04A对已选局部几何路径使用独立的`path_replan_collision_threshold=80`逐段插值检查；该阈值高于LA-03局部A*的`occupied_threshold=50`，因此它是抑制膨胀代价短时跳变的**迟滞确认阈值**，不是更早的障碍检测阈值（同一张代价图中阈值越高，越靠近障碍物才触发）。新地图使已选路径触发该阈值时，请求管理器立即发送`CANCEL`（原因`SELECTED_PATH_INVALIDATED_BY_MAP`）以协作中断局部MINCO，清除已选候选并回到`SEARCHING`；LA-03随后按新地图生成候选。更早的重规划检测应由LA-02以不高于A*阈值的碰撞语义承担。此检查是对已膨胀costmap路径中心线的快速失效判断，不替代LA-05/F5的完整车辆安全复检。

### DEC-LA-011：局部MINCO必须受通道约束，禁止无约束切弯

- 状态：已实现，待运行态验收
- 日期：2026-09-17

局部MINCO若同时关闭凸安全走廊和关联障碍点约束，目标函数只剩平滑、时间、动力学和内部关键点锚定；多项式会自然选择比局部A*折线更短的大圆弧，可能跨越绕障通道甚至穿入障碍物。因此当前`corridor/use_safe_corridor_constraints`默认恢复为`true`，以局部A*关键点构建凸安全走廊并在优化中约束每段采样点。

另增加发布前硬保护：每个优化轨迹采样点均计算到局部A*折线的最小距离，超过`validation/max_reference_deviation_m=0.35 m`即以`REFERENCE_DEVIATION_LIMIT`拒绝；同时按车辆长宽和`optimizing/half_margin`采样完整footprint，代价达到局部MINCO碰撞阈值、未知格或地图外即以`OPTIMIZED_TRAJECTORY_COLLISION`拒绝。两项拒绝不会发布候选，随后由已有状态机等待新的局部A*候选；它们是优化输出保护，不替代LA-05/F5最终仲裁。

### DEC-G-001：全局MINCO安全走廊最小段长与地图分辨率一致

- 状态：已实现，待运行态验收
- 日期：2026-09-17

全局costmap实际分辨率为`0.05 m`，而原`corridor/min_seed_length=0.10 m`会使栅格A*在紧凑转角中出现“需要按偏差/碰撞拆分，但任何切点均产生小于0.10 m子段”的必然失败。将其调为`0.05 m`，即允许单栅格短段进入关键点序列；`initial_time/min_piece_time=0.2 s`仍为每段提供正时间下界，避免MINCO时间变量退化。该改动只影响全局MINCO安全走廊关键点分段，不改变costmap膨胀或前端A*。

### DEC-LA-012：模块四保持现有局部A*/Hybrid A*与MINCO路线，不引入TEB替换

- 状态：已接受
- 日期：2026-09-17

模块四继续采用“LA-01参考窗口 → LA-02阻断检测 → LA-03局部A*/Hybrid A* → LA-04局部MINCO → LA-05仲裁/F5/MPC”的既定路线；不以TEB替换局部搜索、局部MINCO或现有MPC。

当前局部MINCO不稳定的直接原因是局部A*、局部MINCO校验及F5间的车辆包络/阈值语义不一致，以及地图失效保护过早取消；替换规划器不能消除这些系统性问题。后续只允许在现有链路稳定并完成LA-05/LA-06后，将TEB作为独立、可切换的对照基线评估；其输出不得绕过F5或直接替换MPC输入。

### DEC-LA-013：局部MINCO恢复安全走廊模式并统一LA-03车体包络

- 状态：已实现，待运行态验收
- 日期：2026-09-19

本次运行态失败的实际求解器错误为L-BFGS `-1011`（线搜索步长小于最小步长），发生在局部MINCO启用“关联障碍点软距离约束”、关闭凸安全走廊、并使用过强关键点锚定权重的配置下。故局部MINCO固定恢复为`corridor/use_safe_corridor_constraints=true`，关闭`associated_obstacle/enabled`，并将`optimizing/wei_anchor`从`40000`恢复为`5000`。

局部MINCO车体统一为LA-03的物理车体`0.6 x 0.5 m`，`optimizing/half_margin=0.10 m`；MINCO约束车体和发布前footprint复检均采用该外扩量，得到一致的`0.8 x 0.7 m`包络。新增并加载`validation/footprint_margin_m`，避免复检仍错误使用代码默认`0.25 m`余量。LA-02、LA-04A和F5仍是不同安全层，阈值差异必须明确表达为各自的检测/迟滞/最终准入职责，不能再依赖未加载的默认余量。

### DEC-LA-014：局部膨胀地图与局部MINCO安全走廊分层

- 状态：已实现，待运行态验收
- 日期：2026-09-20

局部costmap保留`robot_radius=0.3 m`，但将局部`inflation_radius`设为`1.0 m`、`cost_scaling_factor`设为`5.0`；全局costmap参数保持原值。该配置在车辆外接半径之外提供可见的低代价缓冲带，供局部A*以软代价选择绕障拓扑，而不是将原`0.8 m / 2.0`配置下几乎整圈代价均高于50的膨胀区二值化为障碍。

局部MINCO安全走廊和发布前footprint校验的`collision_cost_threshold`统一为`253`，仅以致命障碍核心构造凸走廊；车辆物理余量仅由MINCO的`0.6 x 0.5 m + 0.1 m`包络施加一次。局部costmap不跟踪未知空间，因此LA-02和局部MINCO均设`unknown_is_obstacle=false`，与LA-03一致。LA-02仍可用其独立阈值提前发现全局参考阻断，LA-04A的80阈值继续只作冻结路径的迟滞失效判定。

该分层禁止“膨胀层作为MINCO硬障碍”与“MINCO完整车辆footprint”同时扩大同一障碍；否则走廊内的中心线仍可能因双重外扩而不可行。运行态验收必须确认局部MINCO候选通过自身完整footprint复检，且安全走廊与绿色候选轨迹均正常发布。

### DEC-LA-015：局部MINCO终端可观测性与A*候选失败强制报错

- 状态：已实现，待运行态验收
- 日期：2026-09-20

局部MINCO启动时必须输出当前走廊模式、碰撞阈值、车辆包络、全部优化权重、动力学约束和L-BFGS上限。每个A*候选请求必须输出开始摘要；成功、取消和失败均输出实际迭代次数、求解耗时、端到端耗时和最终代价。`logging/info_every_n`默认50，开启后每50次代价函数评估输出平滑、时间、约束、锚定和总代价。

任何已进入局部MINCO的A*候选未产生轨迹，且原因不是协作取消时，必须使用`ROS_ERROR`输出`episode_id`、`request_id`、精确`reason`、L-BFGS返回码、迭代数、求解/总耗时和最终代价。`/local_optimizer/status`改为latched，保证失败发生后仍可查询最后一条精确状态。

### DEC-LA-016：L-BFGS -1009中间解不具备局部轨迹发布资格

- 状态：已实现，待运行态验收
- 日期：2026-09-20

运行态实测`solver_result=-1009`（线搜索达到最大评估次数）在381次迭代、约5.19秒后返回。该返回码现固定映射为`LBFGS_MAXIMUM_LINESEARCH`失败，绝不再进入轨迹采样和发布前校验。局部MINCO新增`optimizing/max_solver_time_ms`，默认`400 ms`；预算覆盖走廊构建和求解，触发后返回`TIME_BUDGET_EXCEEDED`并使用`ROS_ERROR`报告。时间预算触发时不发布中间轨迹，后续由LA-05执行既定降级。
