# 从整条轨迹起点到该轨迹点的时间
duration time_from_start

# 采样间隔，单位 s
float64 sampling_interval

# 轨迹点位姿
geometry_msgs/Pose pose

# 轨迹点速度
geometry_msgs/Twist velocity

# 轨迹点加速度
geometry_msgs/Accel acceleration

# 轨迹几何曲率，单位 1/m
float64 curvature

# 沿轨迹方向的 jerk，单位 m/s^3
float64 longitudinal_jerk

# 累计路径弧长，单位 m
float64 arc_length
