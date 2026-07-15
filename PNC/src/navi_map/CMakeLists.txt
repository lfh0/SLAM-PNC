cmake_minimum_required(VERSION 2.8.3)
project(navi_map)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  roslib
  tf
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED) ## fj
find_package(yaml-cpp REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs roslib tf
 DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(ransac_ground_node src/groud_ransac.cpp)
target_link_libraries(ransac_ground_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_maintain_node src/MapMaintain.cpp)
target_link_libraries(map_maintain_node ${catkin_LIBRARIES} yaml-cpp)

add_executable(read_pcd_node src/readPCD.cpp)
target_link_libraries(read_pcd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(navi_map_missionmaker src/missionmaker.cpp)
target_link_libraries(navi_map_missionmaker ${catkin_LIBRARIES} yaml-cpp)
