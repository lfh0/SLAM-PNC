cmake_minimum_required(VERSION 3.16)
project(basic LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES basic
)


file(GLOB_RECURSE BASIC_SOURCES 
  "src/*.cpp"
)

file(GLOB_RECURSE BASIC_HEADERS 
  "include/*.h"
  "include/*.hpp"
)


add_library(basic
  ${BASIC_SOURCES}
  ${BASIC_HEADERS}
)


target_include_directories(basic PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(basic PUBLIC
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  gflags
  glog
)