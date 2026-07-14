cmake_minimum_required(VERSION 2.8.3)
project(front2end_search)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
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
  roslib
  tf
  decomp_ros_utils
  tf2
  tf2_geometry_msgs
)

add_service_files(
  FILES
  SendPath.srv
)

find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED io)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)



add_message_files(
  FILES
  StateMsg.msg
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
    CATKIN_DEPENDS decomp_ros_utils #mpc 
    # traj_server_ros
)

link_directories(
	include
  ${PCL_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARIES}
  # yaml-cpp
)

add_executable(waypoints_record_by_car_pose src/test/WaypointsRecordByCarPose.cpp)
target_link_libraries(waypoints_record_by_car_pose ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(waypoints_record_by_car_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(waypoints_record_by_2DNavGoal src/test/WaypointsRecordBy2DNavGoal.cpp)
target_link_libraries(waypoints_record_by_2DNavGoal ${catkin_LIBRARIES} yaml-cpp)


add_executable(pathpubbyyaml src/test/PathPubByYaml.cpp)
target_link_libraries(pathpubbyyaml ${catkin_LIBRARIES} yaml-cpp)


add_executable(planning_server
    src/app/PlanningServer.cpp
    src/test/traj_optimizer.cpp
    src/search/kino_astar.cpp
    src/search/astar.cpp
    src/search/jps.cpp
    src/search/rrt.cpp
    ${PROTO_SRCS}
)

target_link_libraries(planning_server ${catkin_LIBRARIES} yaml-cpp ${Protobuf_LIBRARIES} ompl)
add_dependencies(planning_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(astar_PlanningServer
    src/app/astar_PlanningServer.cpp
    src/test/traj_optimizer.cpp
    src/search/astar.cpp
    ${PROTO_SRCS}
)
target_link_libraries(astar_PlanningServer ${catkin_LIBRARIES} yaml-cpp ${Protobuf_LIBRARIES} ompl)
add_dependencies(astar_PlanningServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
