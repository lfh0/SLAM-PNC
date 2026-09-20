#ifndef LOCAL_OPTIMIZATION_MANAGER_REQUEST_MANAGER_H_
#define LOCAL_OPTIMIZATION_MANAGER_REQUEST_MANAGER_H_

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <robot_trajectory_msgs/LocalOptimizationRequest.h>
#include <robot_trajectory_msgs/LocalOptimizationStatus.h>
#include <robot_trajectory_msgs/LocalPathCandidate.h>
#include <robot_trajectory_msgs/ReferenceBlockage.h>
#include <robot_trajectory_msgs/ReferenceWindow.h>

namespace local_optimization_manager {

enum class ManagerState {
  IDLE,
  SEARCHING,
  OPTIMIZING,
  COMMITTED,
  FAILED
};

struct Config {
  // 已选局部路径的提前重搜阈值；必须高于局部A*的通行阈值。
  int path_replan_collision_threshold = 90;
  bool unknown_is_obstacle = true;
  double endpoint_change_distance_m = 0.5;
  double path_change_distance_m = 0.3;
  double relative_length_change = 0.10;
  double topology_deadband_m = 0.20;
  double predicted_execution_delay_s = 0.15;
};

struct Update {
  std::vector<robot_trajectory_msgs::LocalOptimizationRequest> requests;
  std::string reason;
};

class RequestManager {
 public:
  explicit RequestManager(const Config& config);

  Update updateBlockage(uint8_t blockage_state, const ros::Time& stamp,
                        const std::string& frame_id);
  Update updateGlobalTrajectory(uint64_t global_trajectory_id,
                                const ros::Time& stamp,
                                const std::string& frame_id);
  Update updateMap(const nav_msgs::OccupancyGrid& map);
  Update consider(const robot_trajectory_msgs::LocalPathCandidate& candidate);
  Update updateOptimizerStatus(
      const robot_trajectory_msgs::LocalOptimizationStatus& status);

  ManagerState state() const { return state_; }
  uint64_t episodeId() const { return episode_id_; }
  uint64_t requestId() const { return request_id_; }
  static uint64_t mapVersion(const nav_msgs::OccupancyGrid& map);
  static const char* stateName(ManagerState state);

 private:
  bool candidateValid(const robot_trajectory_msgs::LocalPathCandidate& candidate,
                      std::string* reason) const;
  bool pathCollision(const nav_msgs::Path& path) const;
  bool materiallyDifferent(
      const robot_trajectory_msgs::LocalPathCandidate& lhs,
      const robot_trajectory_msgs::LocalPathCandidate& rhs) const;
  double pathLength(const nav_msgs::Path& path) const;
  int topologySide(const nav_msgs::Path& path) const;
  robot_trajectory_msgs::LocalOptimizationRequest makeOptimize(
      const robot_trajectory_msgs::LocalPathCandidate& candidate,
      const std::string& reason);
  robot_trajectory_msgs::LocalOptimizationRequest makeCancel(
      const ros::Time& stamp, const std::string& frame_id,
      const std::string& reason) const;
  void resetEpisode();

  Config config_;
  ManagerState state_ = ManagerState::IDLE;
  uint8_t blockage_state_ = robot_trajectory_msgs::ReferenceBlockage::NORMAL;
  uint64_t episode_id_ = 0;
  uint64_t request_id_ = 0;
  uint64_t global_trajectory_id_ = 0;
  nav_msgs::OccupancyGrid map_;
  bool has_map_ = false;
  robot_trajectory_msgs::LocalPathCandidate selected_;
  robot_trajectory_msgs::LocalPathCandidate pending_;
  bool has_selected_ = false;
  bool has_pending_ = false;
  // 失败候选只在地图实际变化或几何路径实质变化后才允许重试，避免10Hz空转。
  uint64_t failed_map_version_ = 0;
  bool has_failed_map_version_ = false;
};

}  // namespace local_optimization_manager

#endif
