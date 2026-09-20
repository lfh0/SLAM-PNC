std_msgs/Header header

# 与产生该阻断判断的 ReferenceWindow 对应的全局轨迹版本。
uint64 global_trajectory_id

uint8 NORMAL=0
uint8 REPLAN_REQUIRED=1
uint8 BRAKE_REQUIRED=2

# 参考窗口的阻断状态。
uint8 state

# 首个碰撞点在 ReferenceWindow.window_points 中的索引；无阻断时为 0。
uint32 blockage_index

# 同一连续阻断区中最后一个发生 footprint/swept-volume 碰撞的索引；无阻断时为 0。
uint32 blockage_end_index

# 从当前单调进度点到首个碰撞点的弧长距离，单位 m。
float64 blockage_distance

# 按当前速度计算的最小安全制动距离，单位 m。
float64 braking_distance

# 用于诊断的碰撞栅格代价值；无阻断时为 -1。
int16 blockage_cost
