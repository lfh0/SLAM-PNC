# 差速轮机器人前视轨迹跟踪实现方案

## 1. 目标

实现一个适用于差速轮机器人的路径跟踪控制器，采用以下结构：

\[
\omega_{cmd}=\omega_{ff}+K_\theta e_\theta+K_y e_y+K_d\dot e_\theta
\]

其中：

- \(\omega_{ff}\)：Pure Pursuit 几何前馈角速度；
- \(e_\theta\)：机器人当前航向与参考路径切线方向之间的航向误差；
- \(e_y\)：机器人相对参考路径的横向误差；
- \(v_{cmd}\)：根据参考速度、曲率、航向误差和横向误差动态调整的线速度。

推荐控制律：

\[
\kappa_{pp}=\frac{2\sin\alpha}{L_d}
\]

\[
\omega_{ff}=v_{cmd}\kappa_{pp}
\]

\[
\omega_{cmd}=v_{cmd}\kappa_{pp}+K_\theta e_\theta+K_y e_y+K_d\dot e_\theta
\]

线速度：

\[
v_{cmd}=v_{ref}\,f_\kappa f_\theta f_y
\]

接近终点后切换到终点位姿控制模式。

---

## 2. 整体流程

```text
规划器输出路径
    ↓
路径预处理
  - 删除重复点
  - 计算累计弧长
  - 计算路径 yaw
  - 可选：计算曲率和参考速度
    ↓
获取机器人当前状态 x、y、yaw、v、ω
    ↓
搜索最近路径点
    ↓
计算自适应前视距离
    ↓
沿路径累计弧长搜索前视点
    ↓
计算 Pure Pursuit 前馈曲率
    ↓
计算横向误差和航向误差
    ↓
计算 v_cmd、ω_cmd
    ↓
速度、角速度及加速度限幅
    ↓
接近终点时切换终点位姿控制
    ↓
发布 geometry_msgs/Twist
```

---

## 3. 数据结构

```cpp
struct RobotState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};

struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
    double reference_velocity = 0.0;
    double arc_length = 0.0;
};

struct ControlCommand
{
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};
```

ROS 中一般发布：

```cpp
geometry_msgs::Twist cmd;
cmd.linear.x = control.linear_velocity;
cmd.angular.z = control.angular_velocity;
```

---

## 4. 路径预处理

### 4.1 删除重复点

如果相邻点距离满足：

\[
\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}<d_{min}
\]

则删除后一个点。建议：

```text
d_min = 0.01 ~ 0.03 m
```

### 4.2 计算路径 yaw

中间点可用中心差分：

\[
\theta_i=\operatorname{atan2}(y_{i+1}-y_{i-1},x_{i+1}-x_{i-1})
\]

首尾点使用相邻点差分。

### 4.3 计算累计弧长

\[
s_0=0
\]

\[
s_i=s_{i-1}+\sqrt{(x_i-x_{i-1})^2+(y_i-y_{i-1})^2}
\]

前视点必须优先沿累计弧长搜索，避免在路径交叉或回头段错误选择欧氏距离最近点。

### 4.4 可选：计算离散曲率

\[
\kappa_i\approx\frac{\operatorname{wrap}(\theta_{i+1}-\theta_{i-1})}{s_{i+1}-s_{i-1}}
\]

如果轨迹规划器已经输出曲率，优先使用规划器结果。

---

## 5. 最近点搜索

保存上一周期最近点索引：

```cpp
std::size_t nearest_index_ = 0;
```

下一周期只在局部窗口搜索：

```text
[nearest_index - back_window,
 nearest_index + forward_window]
```

建议：

```text
back_window = 3 ~ 10 个点
forward_window = 30 ~ 100 个点
```

目标函数：

\[
i_{nearest}=\arg\min_i[(x_i-x)^2+(y_i-y)^2]
\]

只支持前进时，可防止索引倒退：

```cpp
nearest_index_ = std::max(nearest_index_, new_nearest_index);
```

如果系统支持倒车，需要单独设计路径方向和误差符号逻辑。

---

## 6. 自适应前视距离

推荐：

\[
L_d=L_{min}+k_v|v|
\]

并限制：

\[
L_{min}\le L_d\le L_{max}
\]

示例：

```yaml
lookahead:
  min_distance: 0.30
  max_distance: 1.00
  velocity_gain: 0.70
```

曲率较大时还可以缩短前视距离：

\[
L_d=\frac{L_{min}+k_v|v|}{1+k_L|\kappa_{ref}|}
\]

注意：

- 前视距离太小：响应快，但容易左右振荡；
- 前视距离太大：更平滑，但容易切弯；
- 低速使用较小前视距离；
- 高速使用较大前视距离。

---

## 7. 前视点搜索

从最近点开始计算目标弧长：

\[
s_{target}=s_{nearest}+L_d
\]

找到：

\[
s_j\le s_{target}<s_{j+1}
\]

插值比例：

\[
r=\frac{s_{target}-s_j}{s_{j+1}-s_j}
\]

前视点：

\[
x_l=x_j+r(x_{j+1}-x_j)
\]

\[
y_l=y_j+r(y_{j+1}-y_j)
\]

航向角插值：

\[
\theta_l=\theta_j+r\operatorname{wrap}(\theta_{j+1}-\theta_j)
\]

---

## 8. 坐标变换与 Pure Pursuit 曲率

世界坐标系误差：

\[
\Delta x=x_l-x,\qquad \Delta y=y_l-y
\]

转换到机器人坐标系：

\[
x_l^b=\cos\theta\Delta x+\sin\theta\Delta y
\]

\[
y_l^b=-\sin\theta\Delta x+\cos\theta\Delta y
\]

前视夹角：

\[
\alpha=\operatorname{atan2}(y_l^b,x_l^b)
\]

实际前视距离：

\[
L_d^{actual}=\sqrt{(x_l^b)^2+(y_l^b)^2}
\]

Pure Pursuit 曲率：

\[
\kappa_{pp}=\frac{2\sin\alpha}{L_d^{actual}}
\]

如果 `x_l_body <= 0`，说明前视点位于机器人后方，应检查：

- 最近点索引是否错误；
- 路径点顺序是否反了；
- 前视距离是否过小；
- 路径是否包含回头段。

---

## 9. 航向误差和横向误差

### 9.1 航向误差

建议使用参考路径切线方向，而不是仅使用机器人指向前视点的方向：

\[
e_\theta=\operatorname{wrap}(\theta_{ref}-\theta)
\]

推荐使用前视点插值得到的 `target.yaw`。

```cpp
double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}
```

### 9.2 横向误差

以最近参考点 \((x_r,y_r,\theta_r)\) 为例：

\[
e_y=-\sin\theta_r(x-x_r)+\cos\theta_r(y-y_r)
\]

```cpp
double computeLateralError(
    const RobotState& robot,
    const PathPoint& reference)
{
    const double dx = robot.x - reference.x;
    const double dy = robot.y - reference.y;

    return -std::sin(reference.yaw) * dx
           + std::cos(reference.yaw) * dy;
}
```

如果实车测试发现反馈方向相反，应检查坐标系方向和误差符号，而不是直接大幅修改增益。

---

## 10. 线速度控制

曲率降速：

\[
f_\kappa=\frac{1}{1+k_\kappa|\kappa_{pp}|}
\]

航向误差降速：

\[
f_\theta=\max(0,\cos e_\theta)
\]

横向误差降速：

\[
f_y=\frac{1}{1+k_y^v|e_y|}
\]

最终线速度：

\[
v_{target}=v_{ref}f_\kappa f_\theta f_y
\]

如果航向误差过大：

\[
|e_\theta|>\theta_{rotate}
\]

则先停止前进并原地转向：

```cpp
if (std::abs(heading_error) > rotate_in_place_threshold)
{
    target_linear_velocity = 0.0;
}
```

---

## 11. 角速度控制

推荐：

\[
\omega_{cmd}=v_{cmd}\kappa_{pp}+K_\theta e_\theta+K_y e_y+K_d\dot e_\theta
\]

其中：

\[
\dot e_\theta=\frac{\operatorname{wrap}(e_\theta-e_{\theta,last})}{\Delta t}
\]

微分项建议低通滤波：

\[
\dot e_{\theta,f}=\lambda\dot e_{\theta,f,last}+(1-\lambda)\dot e_\theta
\]

初始阶段建议：

- 先不使用积分项；
- 先调 Pure Pursuit；
- 再加入航向反馈；
- 最后加入横向误差和少量微分项。

不推荐直接使用世界坐标系 PID-x、PID-y 作为差速轮主控制器。

---

## 12. 限幅与加速度约束

角速度限幅：

\[
-\omega_{max}\le\omega_{cmd}\le\omega_{max}
\]

线速度加速度约束：

\[
v_{cmd}=\operatorname{clamp}(v_{target},v_{last}-a_{dec}\Delta t,v_{last}+a_{acc}\Delta t)
\]

角加速度约束：

\[
\omega_{cmd}=\operatorname{clamp}(\omega_{target},\omega_{last}-\alpha_{max}\Delta t,\omega_{last}+\alpha_{max}\Delta t)
\]

建议分别配置最大加速度和最大减速度，重载底盘尤其需要限制启动和停车冲击。

---

## 13. 终点状态机

```text
TRACK_PATH
    ↓ 距离终点小于 switch_distance
APPROACH_GOAL
    ↓ 位置误差小于 position_tolerance
ALIGN_GOAL_YAW
    ↓ yaw 误差小于 yaw_tolerance
GOAL_REACHED
```

### 13.1 路径跟踪阶段

使用 Pure Pursuit 前馈和误差反馈。

### 13.2 接近终点阶段

\[
\rho=\sqrt{(x_g-x)^2+(y_g-y)^2}
\]

\[
\alpha=\operatorname{wrap}[\operatorname{atan2}(y_g-y,x_g-x)-\theta]
\]

控制：

\[
v=k_\rho\rho\cos\alpha
\]

\[
\omega=k_\alpha\alpha
\]

### 13.3 最终航向对齐

位置达到阈值后：

\[
v=0
\]

\[
\omega=K_{goal\_yaw}\operatorname{wrap}(\theta_g-\theta)
\]

满足位置和 yaw 阈值后发布零速度。

---

## 14. 推荐 C++ 类结构

```cpp
class DifferentialPathTracker
{
public:
    struct Parameters
    {
        double control_frequency = 50.0;

        double min_lookahead = 0.30;
        double max_lookahead = 1.00;
        double lookahead_velocity_gain = 0.70;

        double heading_gain = 1.20;
        double lateral_gain = 0.80;
        double heading_derivative_gain = 0.05;

        double curvature_velocity_gain = 1.20;
        double lateral_velocity_gain = 0.50;

        double max_linear_velocity = 0.60;
        double min_linear_velocity = 0.00;
        double max_angular_velocity = 1.00;

        double max_linear_acceleration = 0.50;
        double max_linear_deceleration = 0.70;
        double max_angular_acceleration = 1.50;

        double rotate_in_place_threshold = 1.00;

        double goal_switch_distance = 0.40;
        double goal_position_tolerance = 0.05;
        double goal_yaw_tolerance = 0.05;

        double goal_position_gain = 0.80;
        double goal_heading_gain = 1.20;
        double goal_yaw_gain = 1.00;
    };

    explicit DifferentialPathTracker(const Parameters& params);

    void setPath(const std::vector<PathPoint>& path);

    ControlCommand computeCommand(
        const RobotState& state,
        double dt);

    bool isGoalReached() const;

private:
    enum class TrackingState
    {
        TRACK_PATH,
        APPROACH_GOAL,
        ALIGN_GOAL_YAW,
        GOAL_REACHED
    };

    std::size_t findNearestPoint(const RobotState& state);

    PathPoint findLookaheadPoint(
        std::size_t nearest_index,
        double lookahead_distance) const;

    double computeLateralError(
        const RobotState& state,
        const PathPoint& reference) const;

    ControlCommand computePathTrackingCommand(
        const RobotState& state,
        double dt);

    ControlCommand computeGoalApproachCommand(
        const RobotState& state,
        double dt);

    ControlCommand applyLimits(
        const ControlCommand& target,
        double dt);

    static double normalizeAngle(double angle);

private:
    Parameters params_;
    std::vector<PathPoint> path_;
    std::size_t nearest_index_ = 0;

    TrackingState tracking_state_ = TrackingState::TRACK_PATH;

    double last_heading_error_ = 0.0;
    double filtered_heading_error_rate_ = 0.0;
    double last_linear_command_ = 0.0;
    double last_angular_command_ = 0.0;
};
```

---

## 15. 核心控制伪代码

```cpp
ControlCommand DifferentialPathTracker::computePathTrackingCommand(
    const RobotState& robot,
    double dt)
{
    const std::size_t nearest_index = findNearestPoint(robot);
    const PathPoint& nearest = path_.at(nearest_index);

    const double lookahead = std::clamp(
        params_.min_lookahead
            + params_.lookahead_velocity_gain
                * std::abs(robot.linear_velocity),
        params_.min_lookahead,
        params_.max_lookahead);

    const PathPoint target =
        findLookaheadPoint(nearest_index, lookahead);

    const double dx = target.x - robot.x;
    const double dy = target.y - robot.y;

    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);

    const double target_x_body = c * dx + s * dy;
    const double target_y_body = -s * dx + c * dy;

    const double actual_lookahead = std::max(
        1.0e-3,
        std::hypot(target_x_body, target_y_body));

    const double alpha =
        std::atan2(target_y_body, target_x_body);

    const double curvature =
        2.0 * std::sin(alpha) / actual_lookahead;

    const double heading_error =
        normalizeAngle(target.yaw - robot.yaw);

    const double lateral_error =
        computeLateralError(robot, nearest);

    const double curvature_scale =
        1.0 /
        (1.0
         + params_.curvature_velocity_gain
             * std::abs(curvature));

    const double heading_scale =
        std::max(0.0, std::cos(heading_error));

    const double lateral_scale =
        1.0 /
        (1.0
         + params_.lateral_velocity_gain
             * std::abs(lateral_error));

    double target_linear_velocity =
        target.reference_velocity
        * curvature_scale
        * heading_scale
        * lateral_scale;

    if (std::abs(heading_error)
        > params_.rotate_in_place_threshold)
    {
        target_linear_velocity = 0.0;
    }

    const double raw_heading_error_rate =
        normalizeAngle(
            heading_error - last_heading_error_)
        / std::max(dt, 1.0e-3);

    constexpr double derivative_filter = 0.8;

    filtered_heading_error_rate_ =
        derivative_filter * filtered_heading_error_rate_
        + (1.0 - derivative_filter)
            * raw_heading_error_rate;

    const double omega_feedforward =
        target_linear_velocity * curvature;

    const double target_angular_velocity =
        omega_feedforward
        + params_.heading_gain * heading_error
        + params_.lateral_gain * lateral_error
        + params_.heading_derivative_gain
            * filtered_heading_error_rate_;

    last_heading_error_ = heading_error;

    ControlCommand target_command;
    target_command.linear_velocity = target_linear_velocity;
    target_command.angular_velocity = target_angular_velocity;

    return applyLimits(target_command, dt);
}
```

---

## 16. 参数调试顺序

### 第一阶段：只使用 Pure Pursuit

```yaml
heading_gain: 0.0
lateral_gain: 0.0
heading_derivative_gain: 0.0
```

只调前视距离，使机器人能基本跟踪路径且不过度振荡。

### 第二阶段：加入航向反馈

逐步增加：

```yaml
heading_gain
```

如果振荡：

- 减小 `heading_gain`；
- 增大前视距离；
- 降低线速度；
- 少量加入 `heading_derivative_gain`。

### 第三阶段：加入横向误差反馈

从小值开始增加：

```yaml
lateral_gain
```

目标是解决机器人与路径平行但存在横向偏差的问题。

如果蛇形振荡：

- 减小 `lateral_gain`；
- 增大前视距离；
- 加强弯道降速；
- 检查定位噪声和控制延迟。

### 第四阶段：调速度策略

调节：

```yaml
curvature_velocity_gain
lateral_velocity_gain
max_linear_velocity
max_linear_acceleration
max_linear_deceleration
```

目标：直线速度高、弯道主动降速、启停平滑。

---

## 17. 推荐初始参数

```yaml
controller:
  frequency: 50.0

  lookahead:
    min_distance: 0.30
    max_distance: 1.00
    velocity_gain: 0.70

  feedback:
    heading_gain: 1.20
    lateral_gain: 0.80
    heading_derivative_gain: 0.05

  velocity:
    max_linear: 0.60
    min_linear: 0.00
    max_angular: 1.00
    curvature_gain: 1.20
    lateral_error_gain: 0.50

  acceleration:
    max_linear_acceleration: 0.50
    max_linear_deceleration: 0.70
    max_angular_acceleration: 1.50

  behavior:
    rotate_in_place_threshold: 1.00

  goal:
    switch_distance: 0.40
    position_tolerance: 0.05
    yaw_tolerance: 0.05
    position_gain: 0.80
    heading_gain: 1.20
    yaw_gain: 1.00
```

这些值只是低速差速底盘的起始参数，必须结合轴距、轮距、最大速度、控制延迟和定位噪声实机调试。

---

## 18. 调试时记录的数据

建议记录：

```text
当前 x、y、yaw
最近点索引
前视点 x、y
前视距离
横向误差 e_y
航向误差 e_theta
Pure Pursuit 曲率
前馈角速度 omega_ff
反馈角速度 omega_fb
最终 v_cmd、omega_cmd
状态机状态
距离终点误差
终点 yaw 误差
```

RViz 建议显示：

```text
原始路径
最近点
前视点
机器人实际轨迹
横向误差连线
当前控制曲率圆
```

---

## 19. 常见问题

### 左右振荡

可能原因：前视距离太小、反馈增益过大、线速度太高、定位 yaw 抖动或控制延迟。

优先处理：

```text
增大前视距离
降低 lateral_gain
降低 heading_gain
弯道降速
对微分项进行低通滤波
```

### 急弯切角

可能原因：前视距离太大、速度过高、角速度饱和或路径本身不平滑。

优先处理：

```text
急弯减小前视距离
增加曲率降速
检查最大角速度
平滑规划路径
```

### 与路径平行但无法回到路径

通常是只使用路径 yaw 误差，没有使用横向误差。加入：

\[
K_y e_y
\]

### 接近终点后绕圈

使用终点状态机，先收敛位置，再原地对齐最终 yaw，并允许线速度降到 0。

### 角速度方向错误

检查：

- ROS 坐标系是否为右手系；
- `angular.z > 0` 是否代表逆时针；
- 横向误差的正负定义；
- 路径点顺序；
- `atan2(y, x)` 的输入顺序。

---

## 20. 最终推荐结构

\[
\boxed{L_d=\operatorname{clamp}(L_{min}+k_v|v|,L_{min},L_{max})}
\]

\[
\boxed{\kappa_{pp}=\frac{2\sin\alpha}{L_d}}
\]

\[
\boxed{v_{cmd}=v_{ref}f_\kappa f_\theta f_y}
\]

\[
\boxed{\omega_{cmd}=v_{cmd}\kappa_{pp}+K_\theta e_\theta+K_y e_y+K_d\dot e_\theta}
\]

执行顺序：

```text
最近点搜索
→ 自适应前视点
→ Pure Pursuit 前馈
→ 航向与横向反馈
→ 速度/角速度限幅
→ 加速度限制
→ 终点状态机
→ 安全检查
→ 发布控制命令
```

差速轮底盘应围绕自身可控量 \(v,\omega\) 设计控制器，并在路径局部坐标系中处理航向误差和横向误差，不建议直接使用世界坐标系 PID-x、PID-y 作为主跟踪控制器。

