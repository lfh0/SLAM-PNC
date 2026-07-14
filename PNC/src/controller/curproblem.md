# PurePursuit 当前代码问题清单

本文档针对当前 `PurePursuit` 完整代码进行检查，列出影响差速轮轨迹跟踪稳定性、终点收敛性和实车安全性的主要问题，并给出建议修改方向。

## 1. 总体评价

当前控制器已经实现了以下核心结构：

\[
\omega
=
v\kappa_{\mathrm{pp}}
+
K_{\theta}e_{\theta}
+
K_{y}e_{y}
+
K_{d}\dot e_{\theta}
\]

整体流程包括：

```text
起始航向调整
    ↓
路径最近点搜索
    ↓
按路径弧长寻找前视点
    ↓
Pure Pursuit 曲率前馈
    ↓
航向误差和横向误差反馈
    ↓
终点位置判断
    ↓
终点航向调整
```

整体框架合理，但仍有以下问题。

---

## 2. 高优先级问题

### 2.1 横向误差反馈符号可能错误

当前横向误差定义：

```cpp
double PurePursuit::computeLateralError(
    const RobotState& robot,
    const PathPoint& reference) const
{
    const double dx = robot.x - reference.x;
    const double dy = robot.y - reference.y;

    return
        -std::sin(reference.yaw) * dx
        +std::cos(reference.yaw) * dy;
}
```

对应：

\[
e_y
=
-\sin\theta_r(x-x_r)
+
\cos\theta_r(y-y_r)
\]

假设路径沿世界坐标系 `+x` 方向，机器人位于路径左侧，则：

\[
e_y>0
\]

机器人需要向右转回路径，因此在标准 ROS 坐标系中应产生负角速度。

当前代码：

```cpp
const double target_angular_velocity =
    omega_feedforward
    + kp_yaw_ * heading_error
    + kp_pos_ * lateral_error
    + kd_yaw_ * filtered_heading_error_rate_;
```

横向误差项使用 `+ kp_pos_ * lateral_error`，可能使机器人继续向左转。

建议改为：

```cpp
const double target_angular_velocity =
    omega_feedforward
    + kp_yaw_ * heading_error
    - kp_pos_ * lateral_error
    + kd_yaw_ * filtered_heading_error_rate_;
```

即：

\[
\boxed{
\omega
=
v\kappa
+
K_\theta e_\theta
-
K_y e_y
+
K_d\dot e_\theta
}
\]

静态验证方法：

```text
参考路径沿 +x
机器人位于路径左侧
机器人 yaw 与路径 yaw 一致
```

此时应有：

```text
heading_error = 0
lateral_error > 0
angular_velocity < 0
```

---

### 2.2 未处理前视点位于机器人后方

代码已经计算：

```cpp
const double target_x_body =
    cos_yaw * target_dx + sin_yaw * target_dy;
```

但没有判断：

```cpp
target_x_body <= 0.0
```

如果机器人越过终点、最近点跳变或路径方向错误，前视点可能位于机器人后方。此时 Pure Pursuit 曲率可能接近 0，控制器仍可能输出正线速度，使机器人继续远离目标。

建议：

1. 继续沿路径向前搜索，直到找到 `target_x_body > 0` 的目标点；
2. 如果找不到，则停止前进并进入终点接近或原地转向逻辑。

示例：

```cpp
if (target_x_body <= 0.0)
{
    const double target_bearing =
        std::atan2(target_dy, target_dx);

    const double bearing_error =
        getTheta(target_bearing, robot_state.yaw);

    return makeCommand(
        0.0,
        clampValue(
            kp_yaw_ * bearing_error,
            -max_angular_velocity_,
            max_angular_velocity_));
}
```

---

### 2.3 缺少终点接近减速状态

当前状态机：

```text
None
  ↓
Init
  ↓
Tracking
  ↓
GoalYawAdjust
  ↓
Finished
```

当前 `Tracking` 状态中，只要终点距离大于 `goal_position_tolerance_`，机器人就继续按正常参考速度跟踪。

默认参数：

```text
goal_position_tolerance = 0.05 m
reference_velocity = 0.28 m/s
```

机器人可能在距离终点很近时仍保持较高速度，导致：

```text
终点过冲
突然刹车
前视点落到车后
终点位置误差不稳定
```

代码中已有：

```cpp
computeGoalPositionPID(...)
```

但没有在状态机中调用。

建议增加：

```cpp
GoalApproach
```

状态。

```cpp
enum class ControlState
{
    None,
    Init,
    Tracking,
    GoalApproach,
    GoalYawAdjust,
    Finished
};
```

当距离终点小于减速距离时切换：

```cpp
if (distance_to_goal <= goal_slowdown_distance_)
{
    state_ = ControlState::GoalApproach;
    reset_PID();
    return makeCommand(0.0, 0.0);
}
```

建议初值：

```text
goal_slowdown_distance = 0.3 ~ 0.5 m
```

---

### 2.4 PID 首次微分冲击

`reset_PID()` 中：

```cpp
last_heading_error_ = 0.0;
goal_yaw_last_error_ = 0.0;
```

状态切换后第一次计算微分时，可能产生很大的误差变化率。

例如：

\[
e_\theta=1.0\ \mathrm{rad},\quad dt=0.02\ \mathrm{s}
\]

则：

\[
\dot e_\theta=50\ \mathrm{rad/s}
\]

如果 `Kd = 0.1`，微分项就会输出 `5 rad/s`。

建议增加初始化标志：

```cpp
bool heading_pid_initialized_{false};
bool goal_yaw_pid_initialized_{false};
```

首次进入控制时令微分项为 0：

```cpp
double raw_heading_error_rate = 0.0;

if (heading_pid_initialized_)
{
    raw_heading_error_rate =
        getTheta(
            heading_error,
            last_heading_error_) / safe_dt;
}
else
{
    heading_pid_initialized_ = true;
}

last_heading_error_ = heading_error;
```

`reset_PID()` 中应将标志重新设为 `false`。

---

### 2.5 缺少速度、角速度和变化率限制

当前最终输出没有限制：

\[
|v|\le v_{\max}
\]

\[
|\omega|\le \omega_{\max}
\]

也没有限制：

\[
|\dot v|\le a_{\max}
\]

\[
|\dot\omega|\le\alpha_{\max}
\]

可能导致：

```text
角速度瞬间过大
速度突然下降到 0
控制指令跳变
底盘执行器饱和
实车振荡
```

建议增加：

```cpp
double max_linear_velocity_;
double max_angular_velocity_;
double max_linear_acceleration_;
double max_linear_deceleration_;
double max_angular_acceleration_;

double last_linear_command_{0.0};
double last_angular_command_{0.0};
```

至少做基本限幅：

```cpp
linear_velocity =
    clampValue(
        linear_velocity,
        0.0,
        max_linear_velocity_);

angular_velocity =
    clampValue(
        angular_velocity,
        -max_angular_velocity_,
        max_angular_velocity_);
```

进一步增加变化率限制：

```cpp
const double limited_linear =
    clampValue(
        target_linear,
        last_linear_command_
            - max_linear_deceleration_ * dt,
        last_linear_command_
            + max_linear_acceleration_ * dt);

const double limited_angular =
    clampValue(
        target_angular,
        last_angular_command_
            - max_angular_acceleration_ * dt,
        last_angular_command_
            + max_angular_acceleration_ * dt);
```

---

## 3. 中优先级问题

### 3.1 最近点每周期全路径搜索可能跳点

当前每个控制周期都从路径第一个点遍历到最后一个点。

问题：

```text
计算复杂度为 O(N)
交叉路径可能跳到另一条路径段
折返路径可能跳到错误索引
定位跳变时最近点索引可能突变
```

建议保存：

```cpp
int last_nearest_index_{0};
```

并只在局部窗口内搜索：

```cpp
const int search_begin =
    std::max(
        0,
        last_nearest_index_ - 3);

const int search_end =
    std::min(
        static_cast<int>(path_.poses.size()) - 1,
        last_nearest_index_ + 50);
```

只允许正向跟踪时可防止索引倒退：

```cpp
nearest_index =
    std::max(
        nearest_index,
        last_nearest_index_);

last_nearest_index_ = nearest_index;
```

---

### 3.2 前视点未在线段内插值

当前代码只选择达到前视距离后的离散路径点。

例如：

```text
lookahead = 0.5 m
当前累计长度 = 0.4 m
下一段长度 = 0.4 m
```

最终目标点位于约 `0.8 m` 处，而不是 `0.5 m` 处。

可能造成：

```text
前视距离跳动
曲率不连续
角速度抖动
弯道切角
```

建议在线段内插值：

```cpp
if (arc_length + segment_length >= lookahead_distance_)
{
    const double ratio =
        clampValue(
            (lookahead_distance_ - arc_length)
                / segment_length,
            0.0,
            1.0);

    target_x =
        pose0.position.x
        + ratio * segment_dx;

    target_y =
        pose0.position.y
        + ratio * segment_dy;

    break;
}
```

yaw 也需要按最短角度差插值。

---

### 3.3 路径 orientation 可能无效

当前多处使用：

```cpp
tf2::getYaw(path_.poses[i].pose.orientation)
```

这要求所有路径点的四元数有效且归一化。

部分规划器只填写位置，orientation 可能为：

```text
x = 0
y = 0
z = 0
w = 0
```

建议检查四元数模长，若无效则根据相邻路径点计算 yaw：

```cpp
yaw =
    std::atan2(
        next_y - current_y,
        next_x - current_x);
```

---

### 3.4 横向误差基于离散最近点，可能跳变

当前横向误差使用最近离散路径点计算。

路径点较稀疏时，最近点索引变化会造成横向误差跳变。

更精确的方法：

```text
搜索最近路径线段
将机器人位置投影到该线段
使用投影点和线段切线计算横向误差
```

---

## 4. 参数与实现一致性问题

### 4.1 部分 PID 参数读取后未使用

读取了：

```cpp
ki_yaw_
ki_pos_
kd_pos_
```

但路径跟踪中实际只使用：

```cpp
kp_yaw_
kp_pos_
kd_yaw_
```

当前结构实际上是：

```text
Pure Pursuit 前馈
+ yaw PD
+ lateral P
```

不是完整的 yaw PID 和 position PID。

建议将参数重命名为：

```cpp
heading_kp_
heading_kd_
lateral_kp_
```

并删除未使用参数，避免配置误导。

---

### 4.2 固定前视距离不够稳健

当前前视距离固定，默认：

```text
lookahead_distance = 0.1 m
reference_velocity = 0.28 m/s
```

0.1 m 通常偏小，容易引起：

```text
左右振荡
对定位噪声敏感
角速度变化过快
```

建议使用速度自适应前视距离：

\[
L_d
=
\operatorname{clamp}
(L_{\min}+k_v|v|,
L_{\min},
L_{\max})
\]

建议初值：

```text
L_min = 0.25 ~ 0.35 m
L_max = 0.8 ~ 1.2 m
```

---

## 5. reset 与边界处理问题

### 5.1 空路径时旧路径未清除

当前空路径输入后直接返回，但旧路径和旧状态仍可能保留。

建议：

```cpp
if (path.poses.empty())
{
    path_.poses.clear();
    state_ = ControlState::None;
    target_index_ = 0;
    last_nearest_index_ = 0;
    reset_PID();

    ROS_ERROR("path is empty");
    return;
}
```

---

### 5.2 单点路径时 `target_index_ = 1` 越界

建议：

```cpp
target_index_ =
    path_.poses.size() > 1 ? 1 : 0;
```

---

### 5.3 Finished 状态立即清除

当前 `Finished` 只保持一个控制周期，随后清空路径并回到 `None`。

如果外部模块需要持续读取完成状态，建议保持：

```cpp
case ControlState::Finished:
    return makeCommand(0.0, 0.0);
```

等待外部显式加载新路径或清除状态。

---

## 6. 坐标系与时间问题

### 6.1 路径和机器人状态必须处于同一坐标系

必须保证：

```text
path_.poses
robot_state.x
robot_state.y
robot_state.yaw
```

位于同一 frame。

例如路径在 `map`、机器人状态在 `odom`，会导致：

```text
最近点错误
横向误差错误
前视点错误
终点距离错误
```

---

### 6.2 dt 只有下限，没有上限

当前只处理：

```cpp
dt <= 1.0e-3
```

如果节点卡顿，`dt` 可能很大，影响积分、微分和变化率限制。

建议：

```cpp
dt = clampValue(dt, 1.0e-3, 0.1);
```

或在 `dt` 超过阈值时重置微分状态。

---

## 7. 推荐修改优先级

### 第一优先级：实车前必须处理

1. 修正横向误差反馈符号；
2. 处理目标点位于机器人后方；
3. 增加终点接近减速状态；
4. 消除 PID 首次微分冲击；
5. 增加速度、角速度及变化率限制；
6. 保证路径和机器人状态坐标系一致。

### 第二优先级：提高跟踪稳定性

1. 最近点局部窗口搜索；
2. 防止最近点索引倒退；
3. 前视点在线段内插值；
4. 使用速度自适应前视距离；
5. 基于线段投影计算横向误差。

### 第三优先级：代码清理

1. 删除或重命名未使用 PID 参数；
2. 修复单点路径索引；
3. 空路径时清理旧状态；
4. Finished 状态不要立即清除；
5. 检查路径四元数有效性；
6. 对 `dt` 增加上限。

---

## 8. 最终推荐控制律

路径跟踪阶段建议使用：

\[
\boxed{
\omega_{\mathrm{cmd}}
=
v_{\mathrm{cmd}}\kappa_{\mathrm{pp}}
+
K_{\theta p}e_\theta
+
K_{\theta d}\dot e_\theta
-
K_y e_y
}
\]

线速度建议为：

\[
\boxed{
v_{\mathrm{cmd}}
=
v_{\mathrm{ref}}
\cdot
\frac{1}{1+k_\kappa|\kappa|}
\cdot
\max(0,\cos e_\theta)
\cdot
\frac{1}{1+k_e|e_y|}
}
\]

之后依次执行：

```text
线速度限幅
角速度限幅
线加速度限制
角加速度限制
终点减速状态切换
安全检查
发布控制命令
```

---

## 9. 结论

当前代码总体思路正确，已经具备：

```text
Pure Pursuit 曲率前馈
航向误差反馈
横向误差反馈
起终点航向状态机
```

但当前版本仍存在几个可能直接影响实车运行的问题，最关键的是：

```text
横向误差反馈方向可能错误
前视点可能位于机器人后方
缺少终点减速阶段
状态切换时存在微分冲击
缺少速度和角速度约束
```

建议先完成高优先级问题修复，再进行实车参数调试。
