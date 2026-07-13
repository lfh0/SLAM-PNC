cmake_minimum_required(VERSION 2.8.3)
project(navi_map)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
  roslib
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED) ## fj
find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES navi_map
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib PCL
)

include_directories(
  include
  3rdparty
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDR_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
	include
  ${PCL_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARIES}
  # yaml-cpp
)
add_executable(ransac_ground_node src/groud_ransac.cpp)
target_link_libraries(ransac_ground_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_maintain_node src/MapMaintain.cpp)
target_link_libraries(map_maintain_node ${catkin_LIBRARIES} yaml-cpp)

add_executable(read_pcd_node src/readPCD.cpp)
target_link_libraries(read_pcd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(navi_map_missionmaker src/missionmaker.cpp)
target_link_libraries(navi_map_missionmaker ${catkin_LIBRARIES} yaml-cpp)
