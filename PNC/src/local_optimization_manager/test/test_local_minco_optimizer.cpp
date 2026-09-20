#include <gtest/gtest.h>

#include "local_optimization_manager/local_minco_optimizer.h"

namespace {

nav_msgs::OccupancyGrid makeMap() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.width = 80;
  map.info.height = 60;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = -3.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

robot_trajectory_msgs::LocalOptimizationRequest makeRequest() {
  robot_trajectory_msgs::LocalOptimizationRequest request;
  request.header.frame_id = "map";
  request.episode_id = 1;
  request.request_id = 1;
  for (int index = 0; index <= 20; ++index) {
    geometry_msgs::PoseStamped pose;
    pose.header = request.header;
    pose.pose.position.x = 0.1 * index;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    request.geometric_path.poses.push_back(pose);
  }
  request.initial_state.pose = request.geometric_path.poses.front().pose;
  request.final_state.pose = request.geometric_path.poses.back().pose;
  request.initial_state.velocity.linear.x = 0.2;
  request.final_state.velocity.linear.x = 0.2;
  return request;
}

local_optimization_manager::LocalMincoConfig config() {
  local_optimization_manager::LocalMincoConfig value;
  value.vehicle_width_m = 0.2;
  value.footprint_margin_m = 0.0;
  value.corridor_max_lateral_m = 0.5;
  value.nominal_speed_mps = 0.3;
  value.minco.wei_obs = 0.0;
  value.minco.wei_surround = 0.0;
  value.minco.wei_feas = 10.0;
  value.minco.wei_time = 1.0;
  value.minco.wei_anchor = 0.0;
  value.minco.max_vel = 1.0;
  value.minco.max_acc = 1.0;
  value.minco.half_margin = 0.0;
  value.minco.car_width = 0.2;
  value.minco.car_length = 0.2;
  return value;
}

TEST(LocalMincoOptimizer, PreservesNonzeroBoundaryVelocity) {
  local_optimization_manager::LocalMincoOptimizer optimizer(config());
  const auto result = optimizer.optimize(makeRequest(), makeMap(), [] { return false; });
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GT(result.trajectory.points.size(), 2U);
  EXPECT_NEAR(0.2, result.trajectory.points.front().velocity.linear.x, 1e-6);
  // MINCO 数值求解允许很小的终端约束残差。
  EXPECT_NEAR(0.2, result.trajectory.points.back().velocity.linear.x, 1e-3);
  EXPECT_GT(result.trajectory.points.back().time_from_start.toSec(), 0.0);
}

TEST(LocalMincoOptimizer, CancellationStopsBeforeOptimization) {
  local_optimization_manager::LocalMincoOptimizer optimizer(config());
  const auto result = optimizer.optimize(makeRequest(), makeMap(), [] { return true; });
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.cancelled);
  EXPECT_EQ("CANCELLED", result.reason);
}

TEST(LocalMincoOptimizer, RejectsBlockedCorridor) {
  auto map = makeMap();
  const int grid_x = static_cast<int>((1.0 - map.info.origin.position.x) / map.info.resolution);
  const int grid_y = static_cast<int>((0.0 - map.info.origin.position.y) / map.info.resolution);
  map.data[grid_y * map.info.width + grid_x] = 100;
  local_optimization_manager::LocalMincoOptimizer optimizer(config());
  const auto result = optimizer.optimize(makeRequest(), map, [] { return false; });
  EXPECT_FALSE(result.success);
  EXPECT_NE("SUCCESS", result.reason);
}

TEST(LocalMincoOptimizer, AssociatedObstacleModeSkipsCorridorConstruction) {
  auto local_config = config();
  local_config.use_safe_corridor_constraints = false;
  local_config.minco.wei_obs = 100.0;
  local_optimization_manager::LocalMincoOptimizer optimizer(local_config);
  const auto result = optimizer.optimize(makeRequest(), makeMap(), [] { return false; });
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_FALSE(result.key_points.empty());
  EXPECT_TRUE(result.corridors.empty());
}

TEST(LocalMincoOptimizer, CanDisableAssociatedObstacleConstraints) {
  auto local_config = config();
  local_config.use_safe_corridor_constraints = false;
  local_config.use_associated_obstacle_constraints = false;
  local_optimization_manager::LocalMincoOptimizer optimizer(local_config);
  const auto result = optimizer.optimize(makeRequest(), makeMap(), [] { return false; });
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_TRUE(result.corridors.empty());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_local_minco_optimizer", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
