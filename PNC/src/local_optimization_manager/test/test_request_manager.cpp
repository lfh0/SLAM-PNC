#include <gtest/gtest.h>

#include <cmath>

#include "local_optimization_manager/request_manager.h"

namespace {

nav_msgs::OccupancyGrid makeMap() {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.1;
  map.info.width = 120;
  map.info.height = 80;
  map.info.origin.position.x = -1.0;
  map.info.origin.position.y = -4.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

robot_trajectory_msgs::LocalPathCandidate makeCandidate(
    const uint64_t episode, const double lateral_offset) {
  robot_trajectory_msgs::LocalPathCandidate candidate;
  candidate.header.frame_id = "map";
  candidate.header.stamp = ros::Time(1.0);
  candidate.episode_id = episode;
  candidate.geometric_path.header = candidate.header;
  for (int index = 0; index <= 20; ++index) {
    geometry_msgs::PoseStamped pose;
    pose.header = candidate.header;
    pose.pose.position.x = 0.2 * index;
    pose.pose.position.y = lateral_offset * std::sin(M_PI * index / 20.0);
    pose.pose.orientation.w = 1.0;
    candidate.geometric_path.poses.push_back(pose);
  }
  candidate.initial_state.pose = candidate.geometric_path.poses.front().pose;
  candidate.final_state.pose = candidate.geometric_path.poses.back().pose;
  return candidate;
}

void beginReplan(local_optimization_manager::RequestManager* manager,
                 const nav_msgs::OccupancyGrid& map) {
  manager->updateMap(map);
  manager->updateBlockage(
      robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED,
      ros::Time(1.0), "map");
}

TEST(RequestManager, RepeatedEquivalentCandidatesProduceOneRequest) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  const auto map = makeMap();
  beginReplan(&manager, map);
  auto candidate = makeCandidate(1, 0.5);
  candidate.map_version = local_optimization_manager::RequestManager::mapVersion(map);

  const auto first = manager.consider(candidate);
  ASSERT_EQ(1U, first.requests.size());
  EXPECT_EQ(robot_trajectory_msgs::LocalOptimizationRequest::OPTIMIZE,
            first.requests.front().action);
  for (int repeat = 0; repeat < 10; ++repeat) {
    candidate.header.stamp += ros::Duration(0.1);
    EXPECT_TRUE(manager.consider(candidate).requests.empty());
  }
  EXPECT_EQ(1U, manager.requestId());
}

TEST(RequestManager, MaterialUpdateIsCoalescedUntilFailure) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  const auto map = makeMap();
  beginReplan(&manager, map);
  ASSERT_EQ(1U, manager.consider(makeCandidate(2, 0.5)).requests.size());
  EXPECT_TRUE(manager.consider(makeCandidate(2, 1.0)).requests.empty());

  robot_trajectory_msgs::LocalOptimizationStatus status;
  status.episode_id = 2;
  status.request_id = 1;
  status.state = robot_trajectory_msgs::LocalOptimizationStatus::FAILED;
  const auto retry = manager.updateOptimizerStatus(status);
  ASSERT_EQ(1U, retry.requests.size());
  EXPECT_EQ(2U, retry.requests.front().request_id);
  EXPECT_EQ("RETRY_LATEST_PENDING_CANDIDATE", retry.requests.front().reason);
}

TEST(RequestManager, NewMapInvalidatesSelectedPathImmediately) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  auto map = makeMap();
  beginReplan(&manager, map);
  ASSERT_EQ(1U, manager.consider(makeCandidate(3, 0.0)).requests.size());

  const int grid_x = static_cast<int>((2.0 - map.info.origin.position.x) /
                                      map.info.resolution);
  const int grid_y = static_cast<int>((0.0 - map.info.origin.position.y) /
                                      map.info.resolution);
  map.data[grid_y * map.info.width + grid_x] = 100;
  const auto invalidated = manager.updateMap(map);
  ASSERT_EQ(1U, invalidated.requests.size());
  EXPECT_EQ(robot_trajectory_msgs::LocalOptimizationRequest::CANCEL,
            invalidated.requests.front().action);
  EXPECT_EQ(local_optimization_manager::ManagerState::SEARCHING,
            manager.state());
}

TEST(RequestManager, FailedCandidateRetriesAfterMapChanges) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  auto map = makeMap();
  beginReplan(&manager, map);
  auto candidate = makeCandidate(6, 0.5);
  ASSERT_EQ(1U, manager.consider(candidate).requests.size());

  robot_trajectory_msgs::LocalOptimizationStatus status;
  status.episode_id = 6;
  status.request_id = 1;
  status.state = robot_trajectory_msgs::LocalOptimizationStatus::FAILED;
  EXPECT_TRUE(manager.updateOptimizerStatus(status).requests.empty());
  EXPECT_EQ(local_optimization_manager::ManagerState::FAILED, manager.state());
  EXPECT_TRUE(manager.consider(candidate).requests.empty());

  // 路径外的地图内容变化也会使上次失败的环境假设失效，应允许重新请求。
  map.data[0] = 1;
  manager.updateMap(map);
  candidate.header.stamp += ros::Duration(0.1);
  const auto retry = manager.consider(candidate);
  ASSERT_EQ(1U, retry.requests.size());
  EXPECT_EQ("RETRY_AFTER_MAP_OR_PATH_CHANGE", retry.requests.front().reason);
  EXPECT_EQ(2U, retry.requests.front().request_id);
}

TEST(RequestManager, NewAstarSearchAlwaysTriggersOptimization) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  const auto map = makeMap();
  beginReplan(&manager, map);
  auto first = makeCandidate(7, 0.5);
  first.header.seq = 10;
  ASSERT_EQ(1U, manager.consider(first).requests.size());

  auto second = first;
  second.header.seq = 11;
  second.geometric_path.header.seq = 11;
  const auto update = manager.consider(second);
  ASSERT_EQ(1U, update.requests.size());
  EXPECT_EQ("NEW_ASTAR_SEARCH_RESULT", update.requests.front().reason);
  EXPECT_EQ(2U, update.requests.front().request_id);
}

TEST(RequestManager, SuccessCommitsAndIgnoresSafeUpdates) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  const auto map = makeMap();
  beginReplan(&manager, map);
  ASSERT_EQ(1U, manager.consider(makeCandidate(4, 0.5)).requests.size());
  robot_trajectory_msgs::LocalOptimizationStatus status;
  status.episode_id = 4;
  status.request_id = 1;
  status.state = robot_trajectory_msgs::LocalOptimizationStatus::SUCCEEDED;
  manager.updateOptimizerStatus(status);
  EXPECT_EQ(local_optimization_manager::ManagerState::COMMITTED,
            manager.state());
  EXPECT_TRUE(manager.consider(makeCandidate(4, 1.0)).requests.empty());
}

TEST(RequestManager, NewGlobalTrajectoryCancelsCommittedLocalOptimization) {
  local_optimization_manager::RequestManager manager{
      local_optimization_manager::Config()};
  const auto map = makeMap();
  manager.updateGlobalTrajectory(1, ros::Time(1.0), "map");
  beginReplan(&manager, map);
  ASSERT_EQ(1U, manager.consider(makeCandidate(5, 0.5)).requests.size());
  robot_trajectory_msgs::LocalOptimizationStatus status;
  status.episode_id = 5;
  status.request_id = 1;
  status.state = robot_trajectory_msgs::LocalOptimizationStatus::SUCCEEDED;
  manager.updateOptimizerStatus(status);

  const auto update = manager.updateGlobalTrajectory(2, ros::Time(2.0), "map");
  ASSERT_EQ(1U, update.requests.size());
  EXPECT_EQ(robot_trajectory_msgs::LocalOptimizationRequest::CANCEL,
            update.requests.front().action);
  EXPECT_EQ("SUPERSEDED_BY_NEW_GLOBAL_TRAJECTORY", update.requests.front().reason);
  EXPECT_EQ(local_optimization_manager::ManagerState::IDLE, manager.state());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
