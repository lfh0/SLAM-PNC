cmake_minimum_required(VERSION 3.0.2)
project(yhs_can_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  tf
)


 add_message_files(
   FILES
   bms_fb.msg
   bms_flag_fb.msg
   ctrl_cmd.msg
   ctrl_fb.msg
   front_angle_fb.msg
   front_angle_free_ctrl_cmd.msg
   front_velocity_free_ctrl_cmd.msg
   io_cmd.msg
   io_fb.msg
   lf_wheel_fb.msg
   lr_wheel_fb.msg
   rear_angle_fb.msg
   rear_angle_free_ctrl_cmd.msg
   rear_velocity_free_ctrl_cmd.msg
   rf_wheel_fb.msg
   rr_wheel_fb.msg
   steering_ctrl_cmd.msg
   steering_ctrl_fb.msg
 )


# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robuster_mr_msgs
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs
#  DEPENDS system_lib
)

