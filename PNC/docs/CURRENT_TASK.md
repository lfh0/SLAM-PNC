# 当前任务：模块四局部避障重规划（LA-00 MINCO可复用核心）

## 1. 当前任务状态

- 当前阶段：模块四方案已冻结，开始LA-00实现与验收
- 总体状态：模块一基线代码完成；模块二T1/T2工具完成但首个验收bag待录制；模块四未接入在线链路
- 当前入口任务：LA-00，将现有MINCO抽为可供全局和局部节点共同调用的`minco_core`，且全局行为不回归
- 后续阶段：LA-00验收通过后进入LA-01参考窗口与重新汇入目标
- 关联文档：
  - `docs/REQUIREMENTS.md`：系统需求和约束
  - `docs/TASKS.md`：完整任务分解、依赖和验收标准
  - `docs/ACHIEVEMENTS.md`：已确认的设计成果与方案决策

本文档同时保留模块一实施记录，并从本文末尾开始跟踪模块二。设计完成不等于代码完成；只有实现、自动化测试和规定验收均通过后，任务状态才能标记为完成。

## 2. 模块一目标

在没有训练模型的条件下，利用确定性局部轨迹生成器完成安全、稳定、可独立验证的滚动局部规划闭环：

```text
Super-LIO + TF + 车辆反馈
             |
             v
F1 坐标、时间与车辆状态基础层
             |
A*全局路径 --+
             v
F3 统一轨迹协议与确定性基线
             |
             v
F4 延迟补偿与RobotTrajectory适配
             |
环境障碍 ----+
             v
F5 确定性Safety Checker
             |
健康状态 ----+
             v
F6 轨迹管理、唯一来源与安全降级
             |
mission_goal-+
             v
F2 MPC任务与滚动参考控制
             |
             v
F7 模块一集成与回归验证
```

模块一完成后，训练模块和推理模块只能替换F3的轨迹生成来源，不再改变坐标、时间、轨迹适配、安全检查、降级和MPC接口。

## 3. 当前工程基础与已知问题

### 3.1 已有基础

- `front2end_search` 已发布A*全局路径 `/astar_path`；
- `trajopt` 已生成MINCO `RobotTrajectory`；
- `controller` 已具备MPC轨迹跟踪能力；
- `robot_trajectory_msgs` 已定义控制器使用的轨迹消息；
- Super-LIO可提供 `/lio/robo/odom` 和 `/lio/cloud_imu`；
- 传统链路在代码层面为 `A* -> MINCO -> MPC`，MINCO继续保留为传统模式和回退来源。

### 3.2 已确认的阻断问题

- 当前MPC收到每条轨迹都会调用完整 `reset()`；
- 完整重置会重置最近点、控制状态、起始航向调整和局部终点；
- 状态转换包含约0.5秒阻塞式停车，不适配10 Hz滚动轨迹；
- 当前把局部轨迹末点当作任务终点；
- `map` 与Super-LIO的 `world` 关系尚未最终确认；
- Super-LIO的 `robot` 原点是否等同MPC车辆参考点尚未确认；
- 当前没有统一原始局部轨迹协议、延迟补偿适配器、独立Safety Checker和watchdog；
- 当前启动方式没有完整保证MINCO、baseline和未来neural轨迹源互斥。

## 4. 冻结的总体方案

| 小模块 | 选定方案 |
| --- | --- |
| F1 | 复用Super-LIO状态与TF，冻结车辆参考点语义 |
| F2 | 任务与局部参考分离、互斥保护的完整轨迹快照、新轨迹重新匹配 |
| F3 | MINCO全局优化轨迹截取、单调进度匹配和曲率限速 |
| F4 | 历史位姿转换、时间裁剪、执行状态预测、多项式/样条连接 |
| F5 | 二维距离场、完整footprint、swept volume和结构化拒绝 |
| F6 | 独立轨迹管理器、唯一最终发布源、分级安全降级 |
| F7 | 单元测试、ROS测试、回放、仿真、台架和低速实车分级验证 |

统一开发原则：

- 算法逻辑实现为可独立测试的C++核心库；
- ROS节点只负责订阅、发布、参数、TF和诊断；
- 所有模块同时提供正常输出和结构化诊断输出；
- 训练与在线推理必须复用相同预处理及数据定义；
- 网络或基线输出不得直接进入MPC；
- 轨迹源必须唯一，所有异常必须有确定的安全出口。

## 5. 小模块任务、方案和完成情况

状态定义：

```text
未开始：尚未实施
进行中：正在编码或验证
待确认：存在必须由实测或配置确认的外部条件
已实现：代码完成，但尚未通过全部验收
已完成：代码、独立测试和规定验收全部通过
```

### F1 坐标、时间与车辆状态基础层

状态：已完成。

目标：确认并冻结Super-LIO提供的车辆状态、TF、时间和车辆参考点语义，不重复实现已有输入链路。

输入：

```text
/lio/robo/odom
/lio/cloud_imu
/tf、/tf_static
```

输出：

```text
后续模块直接使用带原始时间戳的Super-LIO输出
map -> robot和map -> imu变换
冻结的robot参考点与MPC轴距
```

选定方案：直接复用Super-LIO的 `/lio/robo/odom`、`/lio/cloud_imu` 和现有TF；不新增内容重复的车辆状态节点。需要历史缓存或时间插值的后续模块在各自输入同步边界实现。

推荐TF树：

```text
map -> world -> robot -> imu
```

已确认：

- [x] `robot = IMU中心 = 车辆旋转中心 = 规划与控制参考点`；
- [x] `robot -> imu`为单位变换；
- [x] LIO估计链和Gazebo真值链保持独立；
- [x] `/lio/robo/odom`为 `world -> robot`，约200 Hz、平均延迟约2 ms；
- [x] `/lio/cloud_imu`位于 `imu`，约10 Hz、平均延迟约7 ms；
- [x] `map -> robot`和`map -> imu`可查询；
- [x] 当前仿真车辆轴距为0.4 m，MPC配置已同步为0.4 m。

独立验证：已在当前Super-LIO和Gazebo运行态检查ROS Master、话题、frame、频率、传输延迟和TF可查询性。

完成条件：当前F1范围已经满足。`map -> world`的2 m高度偏移保留为后续BEV高度处理前的标定核验项，不阻塞F1定位链使用。

### F2 MPC任务与滚动参考控制层

状态：已完成。

目标：MPC以高频连续控制，同时允许约10 Hz更新局部参考轨迹而不重置任务。

输入：

```text
startMission(mission_goal)
updateLocalTrajectory(RobotTrajectory in map)
cancelMission()
`/lio/robo/odom`
```

输出：

```text
ControlCommand
ControllerState
mission_id、trajectory_sequence_id、matched_index
tracking_error、solver_status和周期诊断
```

选定方案：任务目标和滚动参考分离；接收回调先完整复制新轨迹，并用同一互斥锁保护轨迹、任务状态和求解器，使控制周期只能看到更新前或更新后的完整快照；新轨迹根据位置、航向和进度重新匹配；状态切换采用非阻塞状态机。

禁止行为：

- 局部更新调用完整 `reset()`；
- 直接继承旧轨迹数组索引；
- 用局部轨迹末点覆盖全局任务终点；
- 在控制线程内部循环等待停车。

独立验证：人工发布完整Map直线、圆弧和不同长度轨迹，以10 Hz更新，检查MPC频率、任务状态、匹配点和控制连续性。

完成条件：10 Hz轨迹更新下MPC约50 Hz稳定运行，不发生任务重置、0.5秒阻塞停车或明显控制跳变。

实际完成：

- [x] 新增 `startMission()`、`updateLocalTrajectory()`、`cancelMission()`；
- [x] ControllerServer独立订阅 `/move_base_simple/goal`；
- [x] 局部轨迹末点不再覆盖任务终点；
- [x] 新局部轨迹按位置和航向重新匹配；
- [x] 局部更新不清空odom记录、完成状态或起始航向状态；
- [x] 删除通用0.5秒阻塞式状态转换停车；
- [x] OSQP求解器持久化并跨周期使用上一周期解热启动；
- [x] 新任务使旧轨迹和旧求解器状态失效；
- [x] 新增7项滚动更新单元测试，其中包含左右圆弧、滚动圆弧和航向跨±π；
- [x] 仿真运行态连续接收50条滚动轨迹，`/cmd_vel`和预测路径稳定约50 Hz。

修改文件：

```text
controller/include/controller/state.h
controller/include/controller/mpc.h
controller/include/controller/controller_server.h
controller/src/controller/mpc.cpp
controller/src/controller/purepursuit.cpp
controller/src/app/Controller_server.cpp
controller/config/mpc.yaml
controller/launch/mpc.launch
controller/CMakeLists.txt
controller/test/test_mpc_rolling_update.cpp
controller/scripts/publish_rolling_mpc_test.py
```

依赖：F1。

### F3 统一轨迹协议与确定性基线层

状态：方案已冻结，代码未开始。

目标：在没有模型时，从MINCO优化后的全局轨迹产生与未来网络完全一致的原始局部轨迹。

输入：

```text
/trajopt/global_trajectory，map坐标系
`/lio/robo/odom`
mission_goal
局部时域、采样间隔和速度配置
```

输出：

```text
RawLocalTrajectory
├── observation_stamp
├── frame_id = base_link(t_obs)
├── dt = 0.2 s
├── 15 × [x, y, v]
├── confidence
├── stop_probability
├── contains_mission_goal
├── source = baseline
└── sequence_id
```

选定方案：MINCO全局优化轨迹按单调进度截取、等弧长重采样并进行曲率限速，不再二次修改其几何形状。第一版使用绝对局部坐标 `[x,y,v]`，MINCO同时作为后续专家标签来源。

独立验证：人工直线、圆弧、S弯、回环和平行路径，验证进度匹配、坐标转换、时间语义和终点标志。

完成条件：固定输入产生确定性输出，协议测试全部通过，并能以10 Hz稳定发布。

依赖：F1。

### F4 延迟补偿与RobotTrajectory适配层

状态：方案已冻结，代码未开始。

目标：把历史观测时刻Ego坐标的原始轨迹转换成与预计执行状态连续、字段完整的Map坐标候选轨迹。

输入：

```text
RawLocalTrajectory
按 `observation_stamp` 查询/缓存的 `/lio/robo/odom`
current_time和预计接管时刻t_exec
RobotTrajectory采样配置
```

输出：

```text
CandidateRobotTrajectory，map坐标系
observation_age
cropped_duration
connection_duration
transform_used
numerical_quality
```

选定方案：

```text
使用T_map_base(t_obs)转换到map
 -> 计算t_exec-t_obs
 -> 裁掉已经过期的轨迹段
 -> 预测t_exec车辆状态
 -> 使用三次/五次多项式或曲率连续样条生成短连接段
 -> 按时间平滑和重采样
 -> 计算完整RobotTrajectory字段
```

禁止将整条历史轨迹平移到当前车辆位置。连接误差超限时必须拒绝，不得强行拉伸连接。

独立验证：注入0、50、100、200和500 ms延迟，验证过期段裁剪、当前状态连接、已知直线/圆弧数值关系和低速退化处理。

完成条件：轨迹不回跳，时间和弧长单调，字段有限，几何/运动学关系满足误差阈值。

依赖：F1、F3定义的统一协议；实现测试可使用人工RawLocalTrajectory，不必等待F3代码完成。

### F5 确定性Safety Checker

状态：方案已冻结，代码未开始。

目标：在候选轨迹进入轨迹管理器和MPC之前作确定性安全准入，不承担复杂轨迹修复。

输入：

```text
CandidateRobotTrajectory
`/lio/robo/odom`
静态障碍栅格/点云
可选动态障碍时序占用
VehicleModel和安全阈值
上一条已接受轨迹
```

输出：

```text
SafetyResult
├── accepted
├── reason_code
├── first_unsafe_index
├── minimum_clearance
└── measured_limit_values

SafeRobotTrajectory，仅accepted时有效
```

选定方案：静态检查以二维距离场为主，点云和动态占用为补充；车辆使用完整footprint，并检查相邻轨迹点之间的swept volume。

检查顺序：

```text
格式和数值
 -> 数据新鲜度
 -> 起点及新旧轨迹连续性
 -> 运动学约束
 -> 静态footprint
 -> swept volume
 -> 动态时空碰撞
 -> 末端制动距离
```

独立验证：人工注入NaN、Inf、旧时间戳、轨迹跳变、动力学超限和车体不同位置的障碍物。

完成条件：预定义危险轨迹全部被拒绝，合法基线轨迹不被误拒绝，每次拒绝均有结构化原因，耗时满足10 Hz预算。

依赖：F1、F4；实现测试可使用人工候选轨迹和人工障碍场。

### F6 轨迹管理、唯一发布源与安全降级层

状态：方案已冻结，代码未开始。

目标：保证MPC只有一个最终轨迹输入，并在轨迹、规划、定位或传感器故障时执行确定的降级策略。

输入：

```text
minco/baseline/neural候选来源状态
SafetyResult和SafeRobotTrajectory
`/lio/robo/odom`
定位、传感器、规划器和控制器健康状态
trajectory_source配置
```

输出：

```text
/controller/reference_trajectory
PlannerSafetyState
SourceDiagnostics
```

选定方案：独立轨迹管理器订阅各候选来源，只发布一个最终控制参考；使用分级状态机：

```text
NORMAL
 -> DEGRADED
 -> DECELERATING
 -> STOPPED
 -> EMERGENCY
```

制动优先沿最后一条仍安全轨迹减速；剩余轨迹不足时按当前车辆状态生成受约束停车轨迹；严重定位异常或明确即时碰撞才进入紧急制动。

独立验证：模拟多发布源、轨迹断流、连续拒绝、规划进程退出、定位失效和故障恢复。

完成条件：最终轨迹发布源唯一，旧轨迹不会超期使用，断流能在规定时间内进入受控制动，状态转换非阻塞且具有迟滞。

依赖：F1、F2接口、F5；实现测试可使用模拟时钟、人工健康状态和人工安全轨迹。

### F7 模块一集成与回归验证层

状态：方案已冻结，尚未开始。

目标：把F1至F6组成可复现的无网络闭环，并建立持续回归测试。

选定方案：分层准入验证：

```text
算法单元测试
 -> ROS节点级rostest
 -> 固定rosbag回放
 -> 运动学仿真闭环
 -> 台架/架空轮
 -> 封闭场地低速实车
```

统一记录：

- 输入输出频率和数据年龄；
- TF查询成功率；
- 轨迹接受率及拒绝原因；
- MPC周期和求解耗时；
- 控制量变化率；
- 最小障碍距离；
- 降级响应时间；
- 回调队列和内存变化。

完成条件：无模型链路完成正常跟踪、最终到达和故障停车；10 Hz局部轨迹不会导致MPC重置或阻塞；所有测试均可通过固定数据和launch复现。

依赖：F1至F6全部通过独立验证。

## 6. 实施顺序与准入门槛

```text
阶段A：F1
冻结TF、时间、车辆参考点和车辆参数
        |
        +------------------+
        v                  v
阶段B：F2             阶段C：F3/F4
滚动MPC               原始轨迹与适配
        |                  |
        +---------+--------+
                  v
阶段D：F5
Safety Checker
                  |
                  v
阶段E：F6
轨迹管理与安全降级
                  |
                  v
阶段F：F7
完整无网络闭环
```

准入规则：

1. F1的TF和时间定义已冻结；正式训练数据采集仍需在BEV高度处理前确认 `map -> world` 的Z轴标定含义；
2. F2未通过10 Hz滚动轨迹测试前，不接入任何神经轨迹；
3. F4未通过延迟注入测试前，不把实时生成轨迹交给控制器；
4. F5和F6未完成前，不进行无人值守闭环测试；
5. F1至F6独立验证全部通过后，才能进行F7完整集成；
6. F7无网络闭环通过后，才进入模型训练后的在线接管。

## 7. 当前进度表

| 编号 | 小模块 | 方案 | 代码 | 独立测试 | 集成验收 | 当前状态 |
| --- | --- | --- | --- | --- | --- | --- |
| F1 | 坐标、时间与车辆状态 | 已冻结 | 已完成 | 运行态已验证 | 待F7复验 | 已完成 |
| F2 | MPC滚动参考控制 | 已冻结 | 已完成 | 7项通过 | 预测时域、起始转向和可视化修正完成 | 已完成 |
| F3 | 统一协议与确定性基线 | 已冻结 | 已完成 | 7项通过 | 核心节点已可联调 | 已完成 |
| F4 | 延迟补偿与轨迹适配 | 已冻结 | 已完成 | 4项通过 | 节点可接入MPC话题 | 已完成 |
| F5 | Safety Checker | 已冻结 | 已完成 | 4项通过 | 静态栅格安全准入已接入 | 已完成 |
| F6 | 轨迹管理与安全降级 | 已冻结 | 已完成 | 启动编排静态检查通过 | 唯一最终话题与断流停止已接入 | 已完成 |
| F7 | 集成与回归验证 | 已冻结 | 已完成 | F2 7项、F3 9项、F4 5项、F5 4项通过 | 完整launch静态检查通过；运行态验收待重启后执行 | 代码完成 |

模块一总体进度：F1至F7的基线代码与接口均已完成。动态障碍时序占用、神经网络训练/推理和独立底盘受控制动不属于本模块冻结范围；最终运行态验收须在重启全链路后执行。

## 8. 当前下一步

下一步：按DEC-F7-001重启`full_navigation.launch`，执行10Hz轨迹、50Hz控制、断流停车、静态障碍拒绝和转弯跟踪的运行态验收。

## 9. 进度更新规则

每完成一个小模块，必须在本文同步更新：

```text
状态
实际修改文件
输入输出接口
测试命令
测试结果
仍存在的问题
对后续模块的影响
```

只有代码实现、独立测试和该模块规定的验收全部通过，才能把状态改为“已完成”。

## 10. 模块二：T1 模型任务与张量规范

状态：已完成并冻结。

实际修改文件：`docs/MODEL_IO_V1.yaml`、`docs/CURRENT_TASK.md`。

输入：冻结的车辆参考点`robot(t_obs)`、模块一F3原始局部轨迹协议、Super-LIO点云与状态接口、MINCO全局路线。

输出：模型I/O版本`bev_route_cnn_v1`，其中BEV张量为`[B,21,250,200]`、Ego State为`[B,4]`、专家/网络轨迹为`[B,15,3]`及对应有效Mask。历史时刻固定为`[-0.3,-0.2,-0.1,0.0] s`，未来标签时刻固定为`[0.2,...,3.0] s`。

冻结方案：4帧BEV的Occupancy、Height、Density、Observed通道，加Route Mask和4个广播后的帧有效性Mask，按既定通道顺序做早期拼接；单一CNN编码器与Ego State MLP融合，非自回归输出15个`[x,y,v]`点。默认不引入Transformer、ConvGRU、Cross-Attention或生成式模型。

验收：训练端和部署端必须对同一原始观测产生相同形状、通道顺序、像素映射、归一化结果和时间语义；任何字段、范围、归一化或坐标变化均须递增`io_version`。

当前下一步：使用T2采集工具录制并检查首个可回放rosbag；通过后进入T3。

## 11. 模块二：T2 原始数据采集

状态：已实现，待运行态验收。

实际修改文件：`src/training_data_tools/`、`docs/CURRENT_TASK.md`。

输入：已启动的A*或Hybrid A*完整导航链路、ROS Master和对应前端路径话题。

输出：每次录制生成独立目录，包含LZ4压缩且按2 GiB切分的`raw_*.bag`、`metadata.yaml`、完整`rosparams.yaml`、`MODEL_IO_V1.yaml`快照及录制话题清单。默认输出目录为`/home/lfh/SLAM+PNC/datasets/raw_t2`。

录制范围：Super-LIO点云与odom、`/tf`、`/tf_static`、任务目标、全局/局部代价地图、MINCO轨迹、F3/F4/F5/F6轨迹和状态、`/cmd_vel`及仿真底盘命令。前端原始路径按`--front-end`可选录制，仅用于数据溯源和错误分析；它不是网络输入，也不是T2完整性检查的必要话题。训练路线输入固定为MINCO的`/trajopt/global_trajectory`，专家标签固定为F3经F4适配且被F5接受的局部安全轨迹。

使用方法：

```bash
rosrun training_data_tools record_t2_dataset.sh --scene sim1_turn --front-end astar
# 完成直线、左右转弯、停车等场景录制后按Ctrl-C停止。
rosrun training_data_tools check_t2_bag.sh /home/lfh/SLAM+PNC/datasets/raw_t2/<录制目录>
```

验收：检查器必须报告`T2 bag完整性检查通过`；录制目录必须同时含有bag、元数据、参数快照和模型I/O快照。随后用`rosbag play --clock raw_*.bag`验证可回放性，并人工确认点云、odom、TF、选定前端路径与MINCO/F3/F4/F5/F6链路均有消息。

仍存在的问题：尚未由当前运行态生成首个验收bag，因此不能标为“已完成”。该验收由你录制后执行，不影响采集工具代码已经可用。

## 12. 模块四：传统局部避障重规划与训练教师

状态：进行中。LA-00～LA-04B的代码与离线测试已完成，但局部MINCO尚未通过运行态验收；LA-05、LA-06尚未实施。因此局部绕障目前不得接入F3/F5/F6/MPC正式控制链路。

目标：局部障碍阻断当前参考时，以当前预测车辆状态为起点、以全局参考的可重新汇入状态为终点，执行局部Hybrid A*与局部MINCO；输出在位置、速度、加速度上连续的绕障轨迹，并继续复用F4/F5/F6/MPC安全链路。

实施顺序：`LA-00 全局MINCO请求管理与可复用核心 → LA-01 全局参考窗口与重汇入目标 → LA-02 参考阻断检测 → LA-03 局部Hybrid A* → LA-04 非零边界局部MINCO → LA-05 轨迹仲裁、安全复检与降级 → LA-06 回归测试与训练教师标签`。每项都必须通过独立验收才可接入下一项。

### 当前完成度冻结（2026-09-17）

| 子模块 | 代码/离线测试 | 运行态验收 | 当前结论 |
| --- | --- | --- | --- |
| LA-00 全局MINCO可复用核心与路径抢占 | 已完成 | 待验收 | 不阻塞当前局部MINCO问题。 |
| LA-01 参考窗口与重汇入状态 | 已完成 | 待验收 | 已具备全局轨迹版本隔离和单调进度匹配。 |
| LA-02 阻断检测 | 已完成 | 待验收 | 已输出NORMAL、REPLAN_REQUIRED、BRAKE_REQUIRED。 |
| LA-03 局部A*/Hybrid A* | 已完成 | 部分验收 | 已支持冻结起终点和已选路径；仅在端点footprint或已选路径失效时重搜，并在新搜索结果出现时触发MINCO。 |
| LA-04A 候选与请求管理 | 已完成 | 部分验收 | 已能对新候选发起优化，并在选中路径被地图判失效时协作取消当前优化。 |
| LA-04B 局部MINCO | 已完成 | 未通过 | 求解器可启动，但当前会出现`OPTIMIZED_TRAJECTORY_COLLISION`拒绝，或在求解期间因`SELECTED_PATH_INVALIDATED_BY_MAP`被取消，尚不能稳定发布可接管轨迹。 |
| LA-05 仲裁、安全复检与降级 | 未开始 | 未开始 | 未实现局部候选接入F3/F5/F6的唯一发布权和安全降级。 |
| LA-06 回归与训练教师标签 | 未开始 | 未开始 | 依赖LA-05完成后实施。 |

当前运行态阻塞项：

1. **局部碰撞模型不一致。** LA-03局部搜索使用的车体与阈值，和LA-04B优化后完整footprint校验的车体余量、阈值并不一致；当前LA-04B的`vehicle_length_m/vehicle_width_m`再叠加`footprint_margin_m`形成的校验包络，可能大于局部A*实际通过的通道，故优化结果被`OPTIMIZED_TRAJECTORY_COLLISION`拒绝。
2. **地图更新过早取消优化。** LA-04A在已选局部A*路径按`path_replan_collision_threshold`检测到失效后，会正确发送`SELECTED_PATH_INVALIDATED_BY_MAP`取消MINCO；当前局部costmap膨胀代价跳变或阈值不协调时，这一保护会频繁发生，使优化尚未完成即被抢占。
3. **尚未授权接入控制。** 在以上两项通过固定场景运行态验收前，`/local_optimizer/candidate_trajectory`仅是调试候选，禁止接入MPC；当前正式控制仍使用既有全局/F3链路。

2026-09-19已修复局部MINCO的首要数值配置问题：恢复凸安全走廊、关闭关联障碍软约束、将锚定权重由`40000`降为`5000`，并统一优化约束与发布前复检包络为LA-03的`0.6 x 0.5 m + 0.1 m`。已通过`local_optimization_manager`编译及12项单元测试；必须重启`/local_minco_optimizer`后进行运行态验收。若仍无输出，再以`/local_optimizer/status.reason`区分走廊构建失败、L-BFGS失败、取消或发布前碰撞拒绝。

2026-09-20已完成局部膨胀与安全走廊分层：仅局部costmap改为`inflation_radius=1.0 m`、`cost_scaling_factor=5.0`，以给局部A*保留低代价缓冲层；局部MINCO走廊与自身发布前复检仅使用`cost>=253`的致命障碍核心，并由完整车辆`0.6 x 0.5 m + 0.1 m`包络保证物理余量。局部MINCO和LA-02的未知栅格语义同步为可通行。重启局部costmap、LA-02、LA-03、LA-04A和LA-04B后，需验证局部A*仍可绕障、MINCO走廊正常生成、绿色候选轨迹通过校验。

2026-09-20运行态记录：局部MINCO收到`episode=6/request=7`的A*候选后，L-BFGS运行381次迭代、求解耗时`5189.702 ms`，以`solver_result=-1009`（线搜索达到最大评估次数）结束。旧核心将该返回码作为可继续校验的中间解；发布前检查发现优化轨迹到局部A*折线的最大距离超过`validation/max_reference_deviation_m=0.35 m`，故以`REFERENCE_DEVIATION_LIMIT`拒绝，未发布候选轨迹。该结果证明失败点在“非健康求解结果的后验贴合检查”，不在A*触发或消息链路。已修复：`-1009`现严格返回`LBFGS_MAXIMUM_LINESEARCH`，且局部MINCO默认使用`optimizing/max_solver_time_ms=400 ms`预算；超时返回`TIME_BUDGET_EXCEEDED`，不再占用重规划链路数秒。

下一步固定为：先统一LA-03、LA-04B及F5的车辆参考点、车体包络和各层碰撞阈值语义；再用静态障碍场景验证“一次局部搜索→一次MINCO→稳定候选输出”，最后才开始LA-05仲裁接入。TEB不纳入当前模块四方案。

| 模块 | 输入 | 输出 | 功能 | 独立验收 |
| --- | --- | --- | --- | --- |
| LA-00 | 前端`Path`、全局地图、odom、MINCO配置 | 一次全局MINCO、请求状态 | 当前前端每goal单次发布；最新路径抢占、可复用MINCO边界；持续发布时才增加去重 | 单个goal仅优化一次；新路径取消旧优化且只发布最新结果 |
| LA-01 | 全局MINCO、odom、局部地图、匹配进度 | 局部窗口、重汇入状态 | 单调匹配和稳定重新汇入 | 回环/平行路径不跳支，重汇入点位于局部地图和阻断之后 |
| LA-02 | local_costmap、footprint、参考窗口、速度 | 三态阻断结果、阻断区间 | 判断正常、重规划或制动 | 无障碍不触发；扫掠体碰撞触发；近障进入制动 |
| LA-03 | 预测起点、重汇入点、local_costmap、车辆模型 | 局部几何路径/失败原因 | 运动学约束Hybrid A*绕障 | 直通、左右绕障、窄通道、无解均有确定结果 |
| LA-04 | 局部路径、安全走廊、P/V/A边界、车辆约束 | 局部候选轨迹 | 非零边界MINCO平滑 | P/V/A连续且动力学、曲率、走廊约束通过 |
| LA-05 | F3参考、局部候选、阻断状态、F5、最新地图 | 唯一原始局部轨迹、仲裁状态 | 切换、复检和受控制动降级 | 五种仲裁场景通过且无多发布者 |
| LA-06 | 全链路状态、rosbag、场景注释 | 回归报告、专家标签 | 自动回放与训练标签生成 | 绕障、停车、重新汇入通过且标签正确 |

### LA-01：全局参考窗口与重汇入目标（代码完成，待运行态验收）

实现包：`local_reference_manager`。输入为`/trajopt/global_trajectory`、`/lio/robo/odom`和`/local_costmap_node/costmap/costmap`；输出为`/local_reference/window`（`robot_trajectory_msgs/ReferenceWindow`）、`/local_reference/rejoin_state`和`/local_reference/debug_window`。该包尚未加入任何全导航launch，故不会改变现有F3/F4/F5/F6/MPC行为。

核心保护：全局MINCO更新时才重置进度；每次匹配仅在上次进度附近的受限窗口中搜索，且进度索引禁止倒退，避免回环、平行路径和轻微LIO回跳选中历史支路；窗口和候选重汇入点均受局部地图边界与`map_margin_m`限制；车辆距参考超过`max_match_distance_m`时拒绝输出，而不是输出错误支路。当前LA-01不判断障碍物是否阻断参考；LA-02只输出阻断区间，LA-03将从该区间之后的窗口点选择实际重汇入状态。

离线验证已通过：局部地图边界内生成窗口和重汇入点、轻微定位回跳时进度不倒退、远离参考时拒绝输出。已通过`catkin_make --pkg local_reference_manager -j2`、`test_reference_window`和`roslaunch --nodes local_reference_manager local_reference_manager.launch`。待运行态验收：在全局MINCO持续有效时确认`/local_reference/window`以10Hz发布，`progress_index`单调非减，`rejoin_state`始终处于local costmap范围内。

### LA-02：参考阻断检测（代码完成，待运行态验收）

实现包：`local_obstacle_monitor`。输入为LA-01的`/local_reference/window`、`/local_costmap_node/costmap/costmap`和`/lio/robo/odom`；输出为`/local_reference/blockage`（`ReferenceBlockage`）及`/local_reference/debug_collision_samples`。状态为`NORMAL`、`REPLAN_REQUIRED`、`BRAKE_REQUIRED`，同时输出首个阻断索引、距离、制动距离和栅格代价。该包未加入任何全导航launch，绝不直接发布控制命令或替换现有轨迹。

检测使用以`robot`为中心的车辆矩形footprint，按`vehicle_length_m/vehicle_width_m/footprint_padding_m`离散采样；相邻参考点之间按半个栅格分辨率插值扫掠，避免稀疏轨迹跳过障碍。制动距离为`v*reaction_time + v²/(2*max_deceleration) + braking_margin`：阻断距离小于等于它时为`BRAKE_REQUIRED`，否则为`REPLAN_REQUIRED`。未知栅格默认视为障碍。`ReferenceBlockage`同时提供首个碰撞的`blockage_index`与同一连续碰撞区的尾部`blockage_end_index`；前者仅服务制动距离，后者服务重汇入目标选择。运行验收前必须核实配置中的车辆长宽与仿真车辆实际footprint一致。

离线测试已通过：空地图正常、远距离障碍要求重规划、近距离障碍要求制动、障碍位于稀疏轨迹两点之间仍能被扫掠检测。已通过`catkin_make --pkg local_obstacle_monitor -j2`、`test_blockage_checker`和`roslaunch --nodes local_obstacle_monitor local_obstacle_monitor.launch`。已修复配置作用域：YAML加载在节点私有命名空间，`occupied_threshold: 100`、footprint和制动参数均真正生效，节点启动时会打印实际生效值。待运行态验收：无障碍时持续`NORMAL`；将障碍放入窗口远处得到`REPLAN_REQUIRED`；放在制动距离内得到`BRAKE_REQUIRED`，并在调试话题上看到对应碰撞样本。

参数作用域审计（2026-09-16）：已扫描当前导航相关launch及所有私有NodeHandle读取点。F3/F4/F5/F6的YAML均以对应节点名作为根键，全局加载后正好映射到节点私有参数，运行态已抽查`trajectory_adapter`、`raw_local_planner`、`trajectory_safety_checker`和`trajectory_manager`参数均生效。仅LA-01/LA-02的YAML原为无根键格式，已改为在各自`<node>`内加载。重启LA-01/LA-02后必须以`rosparam get /local_reference_manager/window_length_m`和`rosparam get /local_obstacle_monitor/occupied_threshold`确认参数生效。

### LA-03：局部 Hybrid A*（代码完成，待运行态验收）

实现包：`local_hybrid_astar`。输入为`/local_reference/window`、`/local_reference/blockage`、`/local_costmap_node/costmap/costmap`和`/lio/robo/odom`；输出为`/local_replanner/geometric_path`（`nav_msgs/Path`）与`/local_replanner/status`。该包未加入全导航launch，既不替换F3输出，也不发布控制指令。

2026-09-16：已停止在节点中使用自写的简化三原语搜索器，LA-03 直接链接并调用`front2end_search`的`path_searching::HybridAstar`核心。该核心的多档转向、精确Ackermann圆弧积分、原语逐采样碰撞检查、完整车辆footprint、解析连接及搜索代价均与全局前端一致。为可复用而新增`init(nh, false)`核心模式：不订阅全局目标、不订阅地图、不发布`/hybridastar_path`或搜索树，LA-03仅显式传入局部地图、预测起点和重汇入终点。局部代价地图先按`occupied_threshold/unknown_is_obstacle`二值化为前端核心所需的`0=空闲、非0=占据`语义。启动文件加载同一个`front2end_search/config/hybridastar.yaml`，因此车辆模型和运动原语参数只维护一份。前端核心的状态原点是后轴中心；依据既有决策，`robot`也是车辆旋转中心，二者在当前车辆模型中对应。

仅当LA-02为`REPLAN_REQUIRED`时执行搜索；`NORMAL`不搜索，`BRAKE_REQUIRED`明确输出`BRAKE_REQUIRED_NO_SEARCH`，优先留给后续安全制动仲裁。搜索起点使用LA-01窗口上的短时预测状态，不使用过时的实际odom起点：默认前视`predicted_start_distance=0.5 m`，但按`start_distance=min(predicted_start_distance, blockage_distance-braking_distance-start_before_brake_margin)`自动缩短；若该安全余量非正则输出`INSUFFICIENT_LEAD_BEFORE_BRAKE`并拒绝搜索，绝不越过最晚制动起点。目标不是首个阻断点之后的固定距离：先取连续阻断区尾部后的首个完整footprint无碰撞参考点`safe_exit=blockage_end_index+1`，再沿参考前推`rejoin_distance_after_blockage_m=3.0 m`。若安全出口或其后3 m超出当前窗口，分别输出`NO_SAFE_EXIT_AFTER_BLOCKAGE_IN_REFERENCE_WINDOW`或`REJOIN_MARGIN_OUTSIDE_REFERENCE_WINDOW`。搜索只生成前进运动原语，状态为`x/y/yaw`离散栅格；每一原语采用自行车模型积分并以完整矩形footprint碰撞检查，地图外、未知格及代价大于等于阈值均拒绝。搜索有扩展节点和墙钟时间上限，失败会输出明确原因。

离线测试通过：直通路径、绕过横向障碍、目标占据拒绝。已通过`catkin_make --pkg local_hybrid_astar -j2`、`test_local_hybrid_astar`和`roslaunch --nodes local_hybrid_astar local_hybrid_astar.launch`。运行态验收：在`REPLAN_REQUIRED`时查看`/local_replanner/geometric_path`从车辆当前位姿起点绕开障碍并向阻断后窗口点汇入；在`NORMAL/BRAKE_REQUIRED`时不得发布新的几何路径。

补充决策（2026-09-16）：LA-03 对共享核心设置`allow_reverse=false`与`enable_analytic_expansion=false`。搜索只扩展前进Ackermann原语，且禁用可能含倒车段的Reeds-Shepp终点连接；仅当一串前进原语进入终点位置和航向容差时成功。全局前端实例仍保持既有默认行为，不受此局部策略影响。

补充实现（2026-09-16）：共享Hybrid A*新增解析连接模式枚举`DISABLED/REEDS_SHEPP/DUBINS`。全局前端默认仍为`REEDS_SHEPP`；LA-03设置为`DUBINS`并保持`allow_reverse=false`。Dubins终点曲线全程前进、按车辆最小转弯半径生成，并与Reeds-Shepp一样逐采样检查完整车辆footprint；连接被碰撞拒绝时，回退到前进Hybrid A*离散扩展。

补充实现（2026-09-16）：LA-03 的搜索起点改为以最晚刹车点为基准沿参考轨迹上游预留`predicted_start_distance_m`（默认0.5 m），可用距离不足时自动缩短；而非简单从当前点固定前视。`/local_replanner/debug_start_goal`发布绿色起点、黄色最晚刹车点和红色重汇入终点三个箭头Marker。

补充实现（2026-09-16）：LA-03 保留`hybrid_astar`默认前端，并新增可选`astar`前端。设置`front_end: astar`后，复用`front2end_search`的二维A*核心，以同一局部地图快照、预测起点和重汇入终点搜索，输出仍为`/local_replanner/geometric_path`；成功、超时和确定无路分别输出`SUCCESS_ASTAR`、`FAILED:ASTAR_TIMEOUT`和`FAILED:ASTAR_NO_PATH`。局部A*使用节点私有`local_astar`短预算参数，不影响全局前端A*配置。A*仅保证点栅格可行，不保证Ackermann和完整车辆footprint；因此它仅适合比较或作为局部MINCO/F5前的几何候选，默认仍推荐Hybrid A*。

### LA-03 当前进度冻结（2026-09-16）

已完成：`local_hybrid_astar`独立包；输入`/local_reference/window`、`/local_reference/blockage`、`/local_costmap_node/costmap/costmap`和odom，输出`/local_replanner/geometric_path`、`/local_replanner/status`及`/local_replanner/debug_start_goal`。Marker颜色固定为绿色预测搜索起点、黄色最晚刹车点、红色全局参考重汇入终点。起点以最晚刹车点为基准沿参考上游预留`predicted_start_distance_m`，余量不足时缩短；`BRAKE_REQUIRED`不启动搜索。

已完成两种可选前端：默认`hybrid_astar`复用全局Hybrid A*核心，关闭倒车原语并改用前进Dubins终点连接；`astar`复用全局A*核心，使用私有短预算参数、完整局部costmap搜索、每轮前后清空搜索状态、禁止对角切角。局部A*遵循`cost < occupied_threshold`可通行、达到阈值或未知不可通行，不做footprint膨胀；阈值以下cost保留为软代价，`obstacle_cost_weight: 10.0`与全局A*一致。

已验证：`catkin_make --pkg front2end_search local_hybrid_astar -j2`、全局`planning_server`回归编译、LA-03 launch结构检查均通过。待运行态验收且不得接入控制链路：安全重汇入终点扫描、地图版本失效检测/取消、A*超时与真正无路状态区分、局部A*真实耗时与成功率、LA-04局部MINCO、LA-05仲裁及F5复检。

#### 局部A*耗时/误报修复（2026-09-16）

已确认原配置`max_search_time: 0.05`被共享A*核心按毫秒解释，导致局部搜索只有`0.05 ms`预算。现冻结为显式`local_astar/max_search_time_ms: 50.0`并改用墙钟计时，不受`/clock`暂停或仿真倍率影响；超时输出`FAILED:ASTAR_TIMEOUT`，开放集耗尽输出`FAILED:ASTAR_NO_PATH`。开放集改为优先级快照，节点获得更低代价时重新入队并惰性跳过旧快照，修复直接修改队列节点而堆不重排的问题。

LA-03现在按“costmap实际内容＋地图几何＋A*起终点栅格”去重。仅地图时间戳更新时不再以10 Hz重复完整搜索；地图内容、起点栅格或终点栅格变化仍会触发新搜索，退出`REPLAN_REQUIRED`后清除去重状态。每次真正搜索均记录总墙钟耗时。已通过`front2end_search`和`local_hybrid_astar`编译、现有局部规划3项单元测试及launch运行检查；启动日志已确认实际加载`max_search_time_ms=50.000`。仍待仿真验证当前地图下的成功率；旧地图结果进入MINCO前的失效隔离已由LA-04A实现，A*内部协作式提前取消仅保留为性能增强。

### LA-04A：局部A*候选与MINCO请求管理（代码完成，待运行态验收）

新增独立包`local_optimization_manager`，当前只负责选择局部A*候选和产生局部MINCO请求，不包含MINCO求解，也不接管F3/F5/MPC。LA-03保留`/local_replanner/geometric_path`，新增`/local_replanner/candidate`；候选携带`episode_id`、地图内容版本、几何路径以及与路径首尾对应的完整P/V/A参考状态。一次`REPLAN_REQUIRED`事件内，LA-03冻结世界坐标下的重汇入终点；车辆靠近只允许起点和中间路径更新，不再推动终点逐帧前移。

请求管理状态冻结为`IDLE → SEARCHING → OPTIMIZING → COMMITTED/FAILED`。第一条在最新地图上逐段无碰撞的候选立即发布`/local_optimizer/request`的`OPTIMIZE`请求；相同候选或小于阈值的变化被忽略。优化期间的实质变化只覆盖单槽最新候选，不中断正在运行的MINCO；当前请求失败时才用最新待处理候选重试。最新地图使已选路径碰撞、阻断状态退出`REPLAN_REQUIRED`或新episode替代旧episode时，发布带原请求编号的`CANCEL`。ROS跨话题回调顺序不确定时，节点会缓存候选，并在阻断状态或地图就绪后重新校验，避免丢失唯一一次A*结果。

冻结接口：`LocalOptimizationRequest`含`OPTIMIZE/CANCEL`、`episode_id/request_id/map_version`、几何路径、首尾`RobotTrajectoryPoint`和`predicted_execution_delay`；局部MINCO以后必须回传`LocalOptimizationStatus`的`STARTED/SUCCEEDED/FAILED/CANCELLED/REJECTED`。默认实质变化阈值为终点0.5 m、路径横向0.3 m、长度10%、绕行方向死区0.2 m，预计接管延迟初值0.15 s。

已通过消息生成、`local_hybrid_astar`与`local_optimization_manager`编译、4项请求管理单元测试以及组合launch结构检查。组合验收入口为`roslaunch local_optimization_manager request_pipeline.launch`；当前它启动LA-01、LA-02、LA-03、请求管理器和LA-04B局部MINCO，但仍不改变控制输出。

### LA-04B：局部 MINCO 消费者（代码完成，待运行态验收）

新增`local_minco_optimizer_node`，已加入`local_optimization_manager/request_pipeline.launch`。它订阅`/local_optimizer/request`和`/local_costmap_node/costmap/costmap`，对`OPTIMIZE`请求在独立工作线程执行局部MINCO，并发布`/local_optimizer/status`与`/local_optimizer/candidate_trajectory`。轨迹输出为`robot_trajectory_msgs/RobotTrajectory`，包含严格递增的`time_from_start`、位置、二维速度/加速度、由速度方向计算的yaw、曲率、角速度及纵向jerk。

局部安全走廊以LA-03几何路径的每段切向矩形生成：沿法向从最大宽度向车辆半宽加余量收缩，按局部costmap半栅格采样；未知、地图外或`cost >= occupied_threshold`均视为不可通行。走廊无法容纳车辆时明确返回`NO_VEHICLE_SAFE_CORRIDOR`，不输出候选轨迹。MINCO起终点使用请求携带的真实P/V/A，未将速度强制置零；边界yaw由对应速度向量表达。全局和局部共享`trajopt`导出的`minco_core`，但局部走廊构建和参数命名空间保持独立，避免影响全局优化。

新`OPTIMIZE`或`CANCEL`到达时，节点递增任务版本；正在优化的任务通过`cancel_checker`协作退出，旧版本即使随后完成也禁止发布轨迹。`CANCEL`会立即回传`CANCELLED`状态。当前仅生成内部候选，**尚未连接**`/local_planner/raw_trajectory`、F5或MPC；该接管、复检和制动降级属于LA-05。

离线验证已通过：非零起终速度边界保持（允许MINCO数值残差`1e-3`）、取消请求不进入求解、被障碍占据的走廊被拒绝；同时LA-04A请求状态机4项测试回归通过。命令：`catkin_make run_tests_local_optimization_manager -j2`。待仿真验收：触发一次`REPLAN_REQUIRED`后，观察`STARTED → SUCCEEDED`与`/local_optimizer/candidate_trajectory`；在优化中改变地图或退出重规划状态时，观察相同请求编号的`CANCELLED`且不再发布旧候选。

2026-09-16更新：LA-04B选点与走廊已对齐全局MINCO策略。选点采用RDP偏差递归，并同时施加弦线碰撞、最大段长和最小段长约束；之后仅在yaw变化、偏差、长度、碰撞均满足时合并短段。走廊采用全局同款`LineSegment2D`凸分解：从局部costmap的占据栅格中心提取障碍点，按每个关键路径段生成凸多边形，并要求相邻走廊交集面积不小于`corridor_min_overlap_area_m2`。局部优化继续共用`minco_core`，而参数语义已与全局`corridor/*`保持一一对应。

2026-09-16修复：新全局MINCO轨迹到达时，局部规划可能因`REPLAN_REQUIRED`状态连续而继续复用旧episode的冻结重汇入点。现`ReferenceWindow`和`ReferenceBlockage`均携带单调`global_trajectory_id`：LA-01每次接收有效全局轨迹递增ID，LA-02原样透传；LA-03仅接受两者ID相同的输入，并在ID变化时清除冻结终点、A*去重缓存和状态，强制开启新episode。请求管理器同时订阅参考窗口；新ID到达立即取消运行中或已提交的旧局部MINCO请求并丢弃旧候选。离线回归覆盖了“局部MINCO成功后新全局轨迹到达必须发送CANCEL”的情况。

可视化：橙色优化轨迹为`/local_optimizer/debug_trajectory`（`Marker`）；安全走廊为`/local_optimizer/polyhedrons`（`decomp_ros_msgs/PolyhedronArray`，latched），与全局`/trajopt/polyhedrons`使用相同的消息类型、半空间语义和Decomp RViz插件显示方式。二者仅在局部MINCO成功后更新；固定坐标系使用`map`，不需要修改任何现有RViz配置。

局部MINCO实际输入关键点另发布为`/local_optimizer/corridor_points`（`MarkerArray`，latched）：青色球体与全局`/trajopt/corridor_points`语义一致，表示RDP、长度、碰撞检查和短段合并后进入MINCO的关键点，不是原始局部A*全部栅格点。关键点数量必须等于凸走廊数量加一。

2026-09-16参数核查与修复：局部原先的走廊阈值、RDP阈值、车辆余量、动态障碍权重和初始时间分配与全局不一致。现局部配置已改用相同的`corridor`、`vehicle`、`optimizing`和`initial_time`分组及全局数值；局部初始时间按全局同样的转角降速规则生成。局部唯一保留真实P/V/A边界，作为将来接入正在执行轨迹的连续性要求；动态障碍项目前无局部动态轨迹输入，权重仅保持配置一致而不会虚构动态避障能力。

### LA-00：MINCO可复用核心（代码完成，待运行态验收）

输入：前端有效路径、路径到达时的全局地图和里程计快照、现有MINCO求解、走廊构建、车辆约束和全局配置。

输出：一次全局MINCO结果、`request_id/success/cancelled`状态，以及后续可抽取为`minco_core`的明确请求/结果边界；全局MINCO节点维持输入`前端路径`与输出`/trajopt/global_trajectory`不变。

边界：全局MINCO每个有效前端路径只规划一次，不因控制循环、odom或局部地图刷新而重算。当前A*和独立Hybrid A*均只在goal回调中发布一次路径，故不增加路径去重；若未来前端改为持续发布，再将其作为防御性功能加入。LA-00不创建局部规划器、不替换F3输出、不修改F4/F5/F6/MPC，也不改变全局MINCO参数。全局与局部将共用核心库但使用独立节点与命名空间。

验收：路径B在路径A优化中到达时A取消且只发布B；单个goal只触发一次全局MINCO；在同一前端路径、地图和车辆参数下，全局节点仍输出时间严格单调、无NaN/Inf的MINCO轨迹，且全局启动链路不出现话题、参数或节点冲突。通过后再读取真实`/local_costmap_node/costmap/costmap`的frame、分辨率、尺寸、更新频率和障碍代价语义，进入LA-01。

当前核查（2026-09-16）：已完成“最新前端路径抢占”代码：独立MINCO工作线程、路径版本、L-BFGS进度回调取消，以及取消/发布前的版本检查。已将求解器编译为`minco_core`库，新增显式`MincoConfig`、`MincoRequest`、`MincoResult`接口；全局节点已实际经由该接口调用核心。离线测试`test_minco_core`已通过：非法P/V/A边界形状拒绝、非零起终速度保持、取消回调中止。`catkin_make --pkg trajopt -j2`和`roslaunch --nodes trajopt trajopt.launch ...`已通过。当前A*和独立Hybrid A*均只在goal回调中发布一次路径，因此不实现路径去重。尚待用户在仿真中验收单goal一次输出、A/B路径抢占以及旧结果不发布；在运行态验收通过前，LA-00不标记为验收完成，LA-01不接入在线输出。

### LA-03/LA-04A 地图连续失效检查（代码完成，待运行态验收）

LA-03现在在每个局部规划周期检查预测起点、候选重汇入终点和已冻结重汇入终点的完整车辆footprint；端点阈值为`endpoint_collision_threshold=50`。起点失效时向上游回退，终点失效时向前寻找可行点；冻结终点被新障碍占据时清除并重新选择。预测起点保持时间前视而不静态冻结，避免局部MINCO完成时起点已经落后车辆。

LA-04A对已选局部几何路径按半栅格插值持续复核，`path_replan_collision_threshold=80`高于LA-03 A*的可通行阈值`50`。它用于抵抗膨胀代价跳变的迟滞确认，不能作为提前触发条件：同一张代价图阈值越高，越靠近障碍物才会达到该值。新地图使路径达到该更高阈值时，立即发布`CANCEL`（`SELECTED_PATH_INVALIDATED_BY_MAP`）中断局部MINCO，清除候选并回到`SEARCHING`等待新局部A*候选；更早阻断由LA-02的碰撞阈值负责。已通过`local_hybrid_astar` 3项、`local_optimization_manager` 10项离线测试和两包编译；仍需仿真确认障碍靠近时的取消、重搜和新候选闭环。

2026-09-17修复：局部MINCO配置曾同时关闭安全走廊和关联障碍点约束，使优化只受平滑/时间/动力学/锚点影响，出现大圆弧跨越局部A*通道。现默认恢复凸安全走廊，并增加发布前轨迹复检：中心线偏离局部A*超过`0.35 m`时拒绝`REFERENCE_DEVIATION_LIMIT`；任何采样footprint与局部MINCO占据阈值、未知格或地图外碰撞时拒绝`OPTIMIZED_TRAJECTORY_COLLISION`。局部优化管理器10项离线测试通过，待仿真检查该场景下绿色优化轨迹是否被限制在局部A*通道内。

2026-09-17运行态诊断与修复：LA-03并非无路，运行日志持续为`SUCCESS_ASTAR`（约3--5 ms）；其输出为`0.05 m`分辨率栅格折线，而局部MINCO仍使用`corridor/min_seed_length=0.10 m`，会与全局MINCO相同地在紧凑转角拒绝拆分。现局部最小段长改为`0.05 m`。另发现请求管理器在局部MINCO失败后进入`FAILED`，同一等价候选仅输出`IGNORED_EQUIVALENT_CANDIDATE`而永久不再请求。现失败结论绑定当前地图版本：同一地图上的相同候选不以10 Hz空转；地图内容或路径实质变化后自动发送`RETRY_AFTER_MAP_OR_PATH_CHANGE`。离线验证为局部MINCO 5项、请求管理器6项测试通过；重启`local_minco_optimizer`和`local_optimization_manager`后做运行态验收。

### 全局MINCO最新路径抢占（已实现，待运行态验收）

输入：连续发布的前端路径话题、路径到达时的全局代价地图和里程计快照。

输出：仅最新路径版本允许发布`/trajopt/global_trajectory`；被后续路径抢占的优化返回取消且不发布旧轨迹。

验收：在第一条较长前端路径的MINCO尚未完成时，发布第二条不同前端路径。日志应依次出现`front_path_received seq=1`、`front_path_received seq=2`、`MINCO被更新的前端路径取消`或`superseded_by_newer_frontend_path`；随后只能看到`seq=2`对应的`final_traj_publish`，不能看到`seq=1`的正式轨迹发布。
