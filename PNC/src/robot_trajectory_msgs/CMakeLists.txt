cmake_minimum_required(VERSION 2.8.3)
project(robot_trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

add_message_files(
  FILES
  RawLocalTrajectoryPoint.msg
  RawLocalTrajectory.msg
  ReferenceBlockage.msg
  ReferenceWindow.msg
  LocalPathCandidate.msg
  LocalOptimizationRequest.msg
  LocalOptimizationStatus.msg
  RobotTrajectoryPoint.msg
  RobotTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    std_msgs
)
