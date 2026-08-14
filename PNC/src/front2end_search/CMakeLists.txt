cmake_minimum_required(VERSION 2.8.3)
project(front2end_search)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -fPIC")
## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf visualization_msgs
)

add_executable(planning_server
    src/app/PlanningServer.cpp
    src/search/obs_hybridastar.cpp
    src/search/astar.cpp
    src/search/jps.cpp
    src/search/rrt.cpp
)

target_link_libraries(planning_server ${catkin_LIBRARIES} ompl)

add_executable(hybridastar_node
    src/app/HybridAstarNode.cpp
    src/search/hybridastar.cpp
)

target_link_libraries(hybridastar_node ${catkin_LIBRARIES} ompl)
