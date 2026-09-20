std_msgs/Header header

# 每次接收到有效全局 MINCO 轨迹递增；用于跨模块拒绝旧窗口/旧阻断结果。
uint64 global_trajectory_id

# 单调匹配到全局参考的起始索引。
uint32 progress_index

# 重汇入点在全局参考中的索引。
uint32 rejoin_index

# 当前匹配点与车辆位置的距离，单位 m。
float64 match_distance

# 局部窗口内按时间顺序排列的全局参考点。
RobotTrajectoryPoint[] window_points

# 局部规划器应重新汇入的完整状态。
RobotTrajectoryPoint rejoin_state
