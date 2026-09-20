#include "local_optimization_manager/request_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_optimization_manager {
namespace {
double pointDistance(const geometry_msgs::Point& lhs,
                     const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool finitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

void hashValue(std::uint64_t* hash, const std::uint64_t value) {
  for (int shift = 0; shift < 64; shift += 8) {
    *hash ^= (value >> shift) & 0xffU;
    *hash *= 1099511628211ULL;
  }
}
}  // namespace

RequestManager::RequestManager(const Config& config) : config_(config) {}

const char* RequestManager::stateName(const ManagerState state) {
  switch (state) {
    case ManagerState::IDLE: return "IDLE";
    case ManagerState::SEARCHING: return "SEARCHING";
    case ManagerState::OPTIMIZING: return "OPTIMIZING";
    case ManagerState::COMMITTED: return "COMMITTED";
    case ManagerState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

uint64_t RequestManager::mapVersion(const nav_msgs::OccupancyGrid& map) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashValue(&hash, map.info.width);
  hashValue(&hash, map.info.height);
  hashValue(&hash, static_cast<std::uint64_t>(
      std::llround(map.info.resolution * 1e9)));
  hashValue(&hash, static_cast<std::uint64_t>(
      std::llround(map.info.origin.position.x * 1e6)));
  hashValue(&hash, static_cast<std::uint64_t>(
      std::llround(map.info.origin.position.y * 1e6)));
  for (const auto cost : map.data) {
    hash ^= static_cast<std::uint8_t>(cost);
    hash *= 1099511628211ULL;
  }
  return hash;
}

void RequestManager::resetEpisode() {
  state_ = ManagerState::IDLE;
  episode_id_ = 0;
  has_selected_ = false;
  has_pending_ = false;
  has_failed_map_version_ = false;
}

Update RequestManager::updateBlockage(const uint8_t blockage_state,
                                      const ros::Time& stamp,
                                      const std::string& frame_id) {
  Update update;
  if (blockage_state != robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED) {
    if ((state_ == ManagerState::OPTIMIZING || state_ == ManagerState::COMMITTED)
        && request_id_ != 0) {
      update.requests.push_back(makeCancel(stamp, frame_id,
                                           "BLOCKAGE_STATE_CLEARED"));
    }
    resetEpisode();
    update.reason = blockage_state == robot_trajectory_msgs::ReferenceBlockage::BRAKE_REQUIRED
        ? "BRAKE_REQUIRED" : "NORMAL";
  } else if (blockage_state_ !=
             robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED) {
    state_ = ManagerState::SEARCHING;
    update.reason = "REPLAN_EPISODE_STARTED";
  }
  blockage_state_ = blockage_state;
  return update;
}

Update RequestManager::updateGlobalTrajectory(const uint64_t global_trajectory_id,
                                              const ros::Time& stamp,
                                              const std::string& frame_id) {
  Update update;
  if (global_trajectory_id == 0 || global_trajectory_id == global_trajectory_id_) {
    return update;
  }
  if (global_trajectory_id_ != 0 &&
      (state_ == ManagerState::OPTIMIZING || state_ == ManagerState::COMMITTED) &&
      request_id_ != 0) {
    update.requests.push_back(makeCancel(stamp, frame_id,
                                         "SUPERSEDED_BY_NEW_GLOBAL_TRAJECTORY"));
  }
  global_trajectory_id_ = global_trajectory_id;
  resetEpisode();
  blockage_state_ = robot_trajectory_msgs::ReferenceBlockage::NORMAL;
  update.reason = "GLOBAL_TRAJECTORY_CHANGED";
  return update;
}

Update RequestManager::updateMap(const nav_msgs::OccupancyGrid& map) {
  map_ = map;
  has_map_ = map.info.resolution > 0.0 && map.info.width > 0 &&
      map.info.height > 0 && map.data.size() ==
      static_cast<std::size_t>(map.info.width) * map.info.height;
  Update update;
  if (has_selected_ && pathCollision(selected_.geometric_path)) {
    if (request_id_ != 0) {
      update.requests.push_back(makeCancel(map.header.stamp,
                                           map.header.frame_id,
                                           "SELECTED_PATH_INVALIDATED_BY_MAP"));
    }
    has_selected_ = false;
    state_ = ManagerState::SEARCHING;
    update.reason = "SELECTED_PATH_INVALIDATED_BY_MAP";
  }
  return update;
}

bool RequestManager::pathCollision(const nav_msgs::Path& path) const {
  if (!has_map_ || path.poses.empty()) return true;
  const auto occupied = [this](const double x, const double y) {
    const int grid_x = static_cast<int>(std::floor(
        (x - map_.info.origin.position.x) / map_.info.resolution));
    const int grid_y = static_cast<int>(std::floor(
        (y - map_.info.origin.position.y) / map_.info.resolution));
    if (grid_x < 0 || grid_y < 0 ||
        grid_x >= static_cast<int>(map_.info.width) ||
        grid_y >= static_cast<int>(map_.info.height)) return true;
    const int cost = map_.data[grid_y * map_.info.width + grid_x];
    return cost < 0 ? config_.unknown_is_obstacle
                    : cost >= config_.path_replan_collision_threshold;
  };

  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto& point = path.poses[index].pose.position;
    if (!finitePoint(point) || occupied(point.x, point.y)) return true;
    if (index == 0) continue;
    const auto& previous = path.poses[index - 1].pose.position;
    const double distance = pointDistance(previous, point);
    const int samples = std::max(1, static_cast<int>(std::ceil(
        distance / std::max(1e-3, 0.5 * map_.info.resolution))));
    for (int sample = 1; sample < samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      if (occupied(previous.x + ratio * (point.x - previous.x),
                   previous.y + ratio * (point.y - previous.y))) return true;
    }
  }
  return false;
}

bool RequestManager::candidateValid(
    const robot_trajectory_msgs::LocalPathCandidate& candidate,
    std::string* reason) const {
  if (blockage_state_ != robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED) {
    *reason = "NOT_IN_REPLAN_STATE";
    return false;
  }
  if (!has_map_) {
    *reason = "NO_VALID_MAP";
    return false;
  }
  if (candidate.episode_id == 0 || candidate.geometric_path.poses.size() < 2) {
    *reason = "INVALID_CANDIDATE_STRUCTURE";
    return false;
  }
  if (pathCollision(candidate.geometric_path)) {
    *reason = "CANDIDATE_COLLISION_ON_LATEST_MAP";
    return false;
  }
  *reason = "VALID";
  return true;
}

double RequestManager::pathLength(const nav_msgs::Path& path) const {
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    length += pointDistance(path.poses[index - 1].pose.position,
                            path.poses[index].pose.position);
  }
  return length;
}

int RequestManager::topologySide(const nav_msgs::Path& path) const {
  if (path.poses.size() < 3) return 0;
  const auto& start = path.poses.front().pose.position;
  const auto& goal = path.poses.back().pose.position;
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double norm = std::hypot(dx, dy);
  if (norm < 1e-6) return 0;
  double strongest = 0.0;
  for (const auto& pose : path.poses) {
    const auto& point = pose.pose.position;
    const double lateral = (dx * (point.y - start.y) -
                            dy * (point.x - start.x)) / norm;
    if (std::abs(lateral) > std::abs(strongest)) strongest = lateral;
  }
  if (std::abs(strongest) < config_.topology_deadband_m) return 0;
  return strongest > 0.0 ? 1 : -1;
}

bool RequestManager::materiallyDifferent(
    const robot_trajectory_msgs::LocalPathCandidate& lhs,
    const robot_trajectory_msgs::LocalPathCandidate& rhs) const {
  if (pointDistance(lhs.final_state.pose.position,
                    rhs.final_state.pose.position) >
      config_.endpoint_change_distance_m) return true;
  if (topologySide(lhs.geometric_path) != topologySide(rhs.geometric_path)) return true;

  const double lhs_length = pathLength(lhs.geometric_path);
  const double rhs_length = pathLength(rhs.geometric_path);
  if (std::abs(lhs_length - rhs_length) /
      std::max(1e-3, lhs_length) > config_.relative_length_change) return true;

  const std::size_t samples = std::max<std::size_t>(
      2, std::min(lhs.geometric_path.poses.size(), rhs.geometric_path.poses.size()));
  for (std::size_t index = 0; index < samples; ++index) {
    const double ratio = static_cast<double>(index) / (samples - 1);
    const std::size_t lhs_index = static_cast<std::size_t>(std::llround(
        ratio * (lhs.geometric_path.poses.size() - 1)));
    const std::size_t rhs_index = static_cast<std::size_t>(std::llround(
        ratio * (rhs.geometric_path.poses.size() - 1)));
    if (pointDistance(lhs.geometric_path.poses[lhs_index].pose.position,
                      rhs.geometric_path.poses[rhs_index].pose.position) >
        config_.path_change_distance_m) return true;
  }
  return false;
}

robot_trajectory_msgs::LocalOptimizationRequest RequestManager::makeOptimize(
    const robot_trajectory_msgs::LocalPathCandidate& candidate,
    const std::string& reason) {
  robot_trajectory_msgs::LocalOptimizationRequest request;
  request.header = candidate.header;
  request.action = robot_trajectory_msgs::LocalOptimizationRequest::OPTIMIZE;
  request.episode_id = candidate.episode_id;
  request.request_id = ++request_id_;
  request.map_version = mapVersion(map_);
  request.geometric_path = candidate.geometric_path;
  request.initial_state = candidate.initial_state;
  request.final_state = candidate.final_state;
  request.predicted_execution_delay = ros::Duration(
      config_.predicted_execution_delay_s);
  request.reason = reason;
  return request;
}

robot_trajectory_msgs::LocalOptimizationRequest RequestManager::makeCancel(
    const ros::Time& stamp, const std::string& frame_id,
    const std::string& reason) const {
  robot_trajectory_msgs::LocalOptimizationRequest request;
  request.header.stamp = stamp;
  request.header.frame_id = frame_id;
  request.action = robot_trajectory_msgs::LocalOptimizationRequest::CANCEL;
  request.episode_id = episode_id_;
  request.request_id = request_id_;
  request.map_version = has_map_ ? mapVersion(map_) : 0;
  request.reason = reason;
  return request;
}

Update RequestManager::consider(
    const robot_trajectory_msgs::LocalPathCandidate& candidate) {
  Update update;
  if (episode_id_ != 0 && candidate.episode_id < episode_id_) {
    update.reason = "STALE_EPISODE";
    return update;
  }
  if (candidate.episode_id != episode_id_) {
    if ((state_ == ManagerState::OPTIMIZING || state_ == ManagerState::COMMITTED) &&
        request_id_ != 0) {
      update.requests.push_back(makeCancel(candidate.header.stamp,
          candidate.header.frame_id, "SUPERSEDED_BY_NEW_EPISODE"));
    }
    episode_id_ = candidate.episode_id;
    state_ = ManagerState::SEARCHING;
    has_selected_ = false;
    has_pending_ = false;
  }

  std::string validation_reason;
  if (!candidateValid(candidate, &validation_reason)) {
    update.reason = validation_reason;
    return update;
  }

  if (!has_selected_) {
    selected_ = candidate;
    has_selected_ = true;
    state_ = ManagerState::OPTIMIZING;
    update.requests.push_back(makeOptimize(candidate, "FIRST_VALID_CANDIDATE"));
    update.reason = "OPTIMIZE_FIRST_VALID_CANDIDATE";
    return update;
  }

  // header.seq由LA-03只在真实A*搜索成功时递增；每个新搜索结果都必须触发MINCO。
  if (candidate.header.seq != selected_.header.seq) {
    selected_ = candidate;
    has_pending_ = false;
    has_failed_map_version_ = false;
    state_ = ManagerState::OPTIMIZING;
    update.requests.push_back(makeOptimize(candidate, "NEW_ASTAR_SEARCH_RESULT"));
    update.reason = "OPTIMIZE_NEW_ASTAR_SEARCH_RESULT";
    return update;
  }

  if (pointDistance(selected_.final_state.pose.position,
                    candidate.final_state.pose.position) >
      config_.endpoint_change_distance_m) {
    update.reason = "REJECT_CHANGED_FROZEN_REJOIN_GOAL";
    return update;
  }

  // 求解失败后不能把同一条候选以10Hz反复送入MINCO；但地图内容或路径拓扑
  // 真正变化后，旧失败结论已不再成立，必须允许一次新的优化请求。
  if (state_ == ManagerState::FAILED) {
    const bool map_changed_since_failure = !has_failed_map_version_ ||
        mapVersion(map_) != failed_map_version_;
    if (map_changed_since_failure || materiallyDifferent(selected_, candidate)) {
      selected_ = candidate;
      state_ = ManagerState::OPTIMIZING;
      has_failed_map_version_ = false;
      update.requests.push_back(makeOptimize(candidate,
                                             "RETRY_AFTER_MAP_OR_PATH_CHANGE"));
      update.reason = "OPTIMIZE_RETRY_AFTER_FAILURE";
    } else {
      update.reason = "IGNORED_FAILED_EQUIVALENT_CANDIDATE";
    }
    return update;
  }
  if (!materiallyDifferent(selected_, candidate)) {
    update.reason = "IGNORED_EQUIVALENT_CANDIDATE";
    return update;
  }

  if (state_ == ManagerState::OPTIMIZING) {
    pending_ = candidate;
    has_pending_ = true;
    update.reason = "COALESCED_MATERIAL_CANDIDATE";
    return update;
  }
  if (state_ == ManagerState::COMMITTED) {
    update.reason = "IGNORED_SAFE_UPDATE_AFTER_COMMIT";
    return update;
  }

  selected_ = candidate;
  state_ = ManagerState::OPTIMIZING;
  update.requests.push_back(makeOptimize(candidate, "RETRY_WITH_NEW_CANDIDATE"));
  update.reason = "OPTIMIZE_RETRY_CANDIDATE";
  return update;
}

Update RequestManager::updateOptimizerStatus(
    const robot_trajectory_msgs::LocalOptimizationStatus& status) {
  Update update;
  if (status.episode_id != episode_id_ || status.request_id != request_id_) {
    update.reason = "IGNORED_STALE_OPTIMIZER_STATUS";
    return update;
  }
  if (status.state == robot_trajectory_msgs::LocalOptimizationStatus::CANCELLED &&
      state_ == ManagerState::SEARCHING && !has_selected_) {
    update.reason = "CANCEL_ACKNOWLEDGED";
    return update;
  }
  if (status.state == robot_trajectory_msgs::LocalOptimizationStatus::STARTED) {
    update.reason = "OPTIMIZER_STARTED";
    return update;
  }
  if (status.state == robot_trajectory_msgs::LocalOptimizationStatus::SUCCEEDED) {
    state_ = ManagerState::COMMITTED;
    has_pending_ = false;
    has_failed_map_version_ = false;
    update.reason = "OPTIMIZATION_COMMITTED";
    return update;
  }
  if (has_pending_) {
    std::string reason;
    if (candidateValid(pending_, &reason)) {
      selected_ = pending_;
      has_pending_ = false;
      state_ = ManagerState::OPTIMIZING;
      update.requests.push_back(makeOptimize(selected_,
                                             "RETRY_LATEST_PENDING_CANDIDATE"));
      update.reason = "OPTIMIZE_PENDING_AFTER_FAILURE";
      return update;
    }
    has_pending_ = false;
  }
  state_ = ManagerState::FAILED;
  failed_map_version_ = has_map_ ? mapVersion(map_) : 0;
  has_failed_map_version_ = has_map_;
  update.reason = "OPTIMIZATION_FAILED_NO_PENDING_CANDIDATE";
  return update;
}

}  // namespace local_optimization_manager
