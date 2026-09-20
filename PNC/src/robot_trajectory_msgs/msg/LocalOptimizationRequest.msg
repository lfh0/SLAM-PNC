std_msgs/Header header

uint8 OPTIMIZE=0
uint8 CANCEL=1
uint8 action

uint64 episode_id
uint64 request_id
uint64 map_version

nav_msgs/Path geometric_path
RobotTrajectoryPoint initial_state
RobotTrajectoryPoint final_state

# 预计从请求产生到轨迹接管的时间，供局部MINCO进行起点延迟补偿。
duration predicted_execution_delay
string reason
