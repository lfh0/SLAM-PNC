#ifndef LOCAL_OPTIMIZATION_MANAGER_LOCAL_MINCO_OPTIMIZER_H_
#define LOCAL_OPTIMIZATION_MANAGER_LOCAL_MINCO_OPTIMIZER_H_

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <robot_trajectory_msgs/LocalOptimizationRequest.h>
#include <robot_trajectory_msgs/RobotTrajectory.h>

#include <plan_manage/traj_optimizer.h>

namespace local_optimization_manager {

struct LocalMincoConfig {
  int occupied_threshold = 95;
  bool unknown_is_obstacle = true;
  double min_seed_length_m = 0.10;
  double max_seed_length_m = 0.50;
  double max_seed_deviation_m = 0.05;
  double max_seed_yaw_error_rad = 0.10;
  double corridor_max_longitudinal_m = 0.30;
  double corridor_max_lateral_m = 0.60;
  double corridor_min_overlap_area_m2 = 0.01;
  double vehicle_width_m = 0.60;
  double vehicle_length_m = 0.60;
  double footprint_margin_m = 0.25;
  // true：凸安全走廊；false：关联障碍点距离约束，不构建也不计算走廊代价。
  bool use_safe_corridor_constraints = true;
  bool use_associated_obstacle_constraints = true;
  double associated_obstacle_search_radius_m = 1.5;
  int max_associated_obstacles_per_sample = 12;
  double associated_obstacle_clearance_m = 0.08;
  // 优化结果相对局部A*折线允许的最大中心线偏离；用于拒绝无约束切弯。
  double max_reference_deviation_m = 0.35;
  double nominal_speed_mps = 0.30;
  double min_speed_mps = 0.20;
  double min_piece_time_s = 0.20;
  double min_turn_speed_ratio = 0.25;
  double max_acceleration_mps2 = 0.50;
  double sample_period_s = 0.02;
  plan_manage::MincoConfig minco;
};

struct LocalMincoResult {
  bool success = false;
  bool cancelled = false;
  std::string reason;
  int iterations = 0;
  double optimize_time_ms = 0.0;
  double final_cost = 0.0;
  int solver_result = 0;
  robot_trajectory_msgs::RobotTrajectory trajectory;
  std::vector<Eigen::Vector2d> key_points;
  std::vector<Eigen::MatrixXd> corridors;
};

class LocalMincoOptimizer {
 public:
  explicit LocalMincoOptimizer(const LocalMincoConfig& config);

  LocalMincoResult optimize(
      const robot_trajectory_msgs::LocalOptimizationRequest& request,
      const nav_msgs::OccupancyGrid& map,
      const std::function<bool()>& cancel_checker) const;

 private:
  bool occupied(const nav_msgs::OccupancyGrid& map, double x, double y) const;
  bool buildCorridors(const nav_msgs::Path& path,
                      const nav_msgs::OccupancyGrid& map,
                      std::vector<Eigen::Vector2d>* key_points,
                      std::vector<Eigen::MatrixXd>* corridors,
                      std::string* reason,
                      const std::function<bool()>& cancel_checker) const;
  bool buildAssociatedObstaclePoints(
      const std::vector<Eigen::Vector2d>& key_points,
      const nav_msgs::OccupancyGrid& map,
      std::vector<std::vector<Eigen::Vector2d>>* associated_points,
      std::string* reason,
      const std::function<bool()>& cancel_checker) const;
  robot_trajectory_msgs::RobotTrajectory sampleTrajectory(
      const plan_utils::Trajectory& trajectory,
      const robot_trajectory_msgs::LocalOptimizationRequest& request) const;
  bool validateTrajectory(
      const robot_trajectory_msgs::RobotTrajectory& trajectory,
      const nav_msgs::Path& reference_path,
      const nav_msgs::OccupancyGrid& map,
      std::string* reason) const;

  LocalMincoConfig config_;
};

}  // namespace local_optimization_manager

#endif
