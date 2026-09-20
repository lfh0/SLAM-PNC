cmake_minimum_required(VERSION 3.0.2)
project(local_optimization_manager)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  decomp_ros_utils
  decomp_ros_msgs
  nav_msgs
  robot_trajectory_msgs
  roscpp
  std_msgs
  tf2
  trajopt
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_optimization_manager_core local_minco_optimizer_core
  CATKIN_DEPENDS decomp_ros_msgs decomp_ros_utils nav_msgs robot_trajectory_msgs roscpp std_msgs tf2 trajopt visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(local_optimization_manager_core src/request_manager.cpp)
add_dependencies(local_optimization_manager_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_optimization_manager_core ${catkin_LIBRARIES})

add_library(local_minco_optimizer_core src/local_minco_optimizer.cpp)
add_dependencies(local_minco_optimizer_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_minco_optimizer_core minco_core ${catkin_LIBRARIES})

add_executable(local_optimization_manager_node src/request_manager_node.cpp)
add_dependencies(local_optimization_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_optimization_manager_node
  local_optimization_manager_core ${catkin_LIBRARIES})

add_executable(local_minco_optimizer_node src/local_minco_optimizer_node.cpp)
add_dependencies(local_minco_optimizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_minco_optimizer_node
  local_minco_optimizer_core minco_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_request_manager test/test_request_manager.cpp)
  if(TARGET test_request_manager)
    add_dependencies(test_request_manager ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_request_manager
      local_optimization_manager_core ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_local_minco_optimizer test/test_local_minco_optimizer.cpp)
  if(TARGET test_local_minco_optimizer)
    add_dependencies(test_local_minco_optimizer ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_local_minco_optimizer
      local_minco_optimizer_core minco_core ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS local_optimization_manager_core local_minco_optimizer_core
  local_optimization_manager_node local_minco_optimizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
