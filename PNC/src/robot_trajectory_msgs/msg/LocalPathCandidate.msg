std_msgs/Header header

# 同一次REPLAN_REQUIRED事件内保持不变。
uint64 episode_id

# 生成该候选时使用的局部地图内容版本。
uint64 map_version

# 局部A*输出的几何路径。
nav_msgs/Path geometric_path

# 与几何路径首尾对应的完整边界状态。
RobotTrajectoryPoint initial_state
RobotTrajectoryPoint final_state
