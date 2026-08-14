cmake_minimum_required(VERSION 2.8.3)
project(trajopt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
## Find catkin macros and libraries

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)
set(PLAN_UTILS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/plan_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
  nav_msgs
  robot_trajectory_msgs
  visualization_msgs
  roslib
  tf
  controller
  decomp_ros_utils
)

find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)



add_message_files(
  FILES
  StateMsg.msg
  SingleMinco.msg
  MincoTraj.msg
  Trajectory.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDR_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${THIRD_PARTY_INCLUDE_DIRS}
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/minco_config.proto
)

catkin_package(
    INCLUDE_DIRS include ${THIRD_PARTY_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PLAN_UTILS_INCLUDE_DIRS}
    CATKIN_DEPENDS controller decomp_ros_utils robot_trajectory_msgs #mpc 
    # traj_server_ros
)

link_directories(
	include
  ${PCL_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARIES}
  # yaml-cpp
)


add_executable(traj_opt 
    src/app/Trajopt_Server.cpp
    src/traj_opt/Minco.cpp
    ${PROTO_SRCS}
)
target_link_libraries(traj_opt ${catkin_LIBRARIES} yaml-cpp ${Protobuf_LIBRARIES})
add_dependencies(traj_opt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
