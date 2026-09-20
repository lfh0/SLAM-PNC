#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/OccupancyGrid.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <robot_trajectory_msgs/LocalOptimizationRequest.h>
#include <robot_trajectory_msgs/LocalOptimizationStatus.h>
#include <robot_trajectory_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "local_optimization_manager/local_minco_optimizer.h"

namespace local_optimization_manager {

class LocalMincoOptimizerNode {
 public:
  LocalMincoOptimizerNode() : nh_(), pnh_("~") {
    LocalMincoConfig config;
    pnh_.param("corridor/use_safe_corridor_constraints",
               config.use_safe_corridor_constraints, true);
    pnh_.param("corridor/collision_cost_threshold", config.occupied_threshold, 95);
    pnh_.param("corridor/unknown_as_occupied", config.unknown_is_obstacle, true);
    pnh_.param("corridor/min_seed_length", config.min_seed_length_m, 0.1);
    pnh_.param("corridor/max_seed_length", config.max_seed_length_m, 0.5);
    pnh_.param("corridor/max_seed_deviation", config.max_seed_deviation_m, 0.05);
    pnh_.param("corridor/max_seed_yaw_error", config.max_seed_yaw_error_rad, 0.1);
    pnh_.param("corridor/max_longitudinal", config.corridor_max_longitudinal_m, 0.3);
    pnh_.param("corridor/max_lateral", config.corridor_max_lateral_m, 0.6);
    pnh_.param("corridor/min_overlap_area", config.corridor_min_overlap_area_m2, 0.01);
    pnh_.param("associated_obstacle/search_radius_m",
               config.associated_obstacle_search_radius_m, 1.5);
    pnh_.param("associated_obstacle/enabled",
               config.use_associated_obstacle_constraints, true);
    pnh_.param("associated_obstacle/max_points_per_sample",
               config.max_associated_obstacles_per_sample, 12);
    pnh_.param("associated_obstacle/clearance_m",
               config.associated_obstacle_clearance_m, 0.08);
    pnh_.param("validation/max_reference_deviation_m",
               config.max_reference_deviation_m, 0.35);
    pnh_.param("validation/footprint_margin_m", config.footprint_margin_m, 0.10);
    pnh_.param("vehicle/car_width", config.vehicle_width_m, 0.6);
    pnh_.param("vehicle/car_length", config.vehicle_length_m, 0.6);
    pnh_.param("vehicle/car_d_cr", config.minco.car_d_cr, 0.0);
    pnh_.param("sample_period_s", config.sample_period_s, 0.02);
    pnh_.param("optimizing/traj_resolution", config.minco.traj_resolution, 8);
    pnh_.param("optimizing/des_traj_resolution", config.minco.destraj_resolution, 12);
    pnh_.param("optimizing/wei_sta_obs", config.minco.wei_obs, 5000.0);
    pnh_.param("optimizing/wei_dyn_obs", config.minco.wei_surround, 7000.0);
    pnh_.param("optimizing/wei_feas", config.minco.wei_feas, 5000.0);
    pnh_.param("optimizing/wei_time", config.minco.wei_time, 500.0);
    pnh_.param("optimizing/wei_anchor", config.minco.wei_anchor, 5000.0);
    pnh_.param("optimizing/dyn_obs_clearance", config.minco.surround_clearance, 1.0);
    pnh_.param("optimizing/max_vel", config.minco.max_vel, config.nominal_speed_mps);
    pnh_.param("optimizing/max_acc", config.minco.max_acc, config.max_acceleration_mps2);
    pnh_.param("optimizing/max_cur", config.minco.max_cur, 0.4);
    pnh_.param("optimizing/half_margin", config.minco.half_margin, 0.25);
    pnh_.param("optimizing/max_solver_time_ms", max_solver_time_ms_, 400.0);
    pnh_.param("logging/info_every_n", config.minco.logging_every_n, 50);
    bool enable_iteration_log = true;
    pnh_.param("logging/enable_iteration_log", enable_iteration_log, true);
    if (!enable_iteration_log) config.minco.logging_every_n = 0;
    pnh_.param("initial_time/min_speed", config.min_speed_mps, 0.2);
    pnh_.param("initial_time/min_piece_time", config.min_piece_time_s, 0.2);
    pnh_.param("initial_time/min_turn_speed_ratio", config.min_turn_speed_ratio, 0.25);
    config.minco.car_width = config.vehicle_width_m;
    config.minco.car_length = config.vehicle_length_m;
    optimizer_.reset(new LocalMincoOptimizer(config));

    pnh_.param("request_topic", request_topic_, std::string("/local_optimizer/request"));
    pnh_.param("costmap_topic", costmap_topic_, std::string("/local_costmap_node/costmap/costmap"));
    pnh_.param("status_topic", status_topic_, std::string("/local_optimizer/status"));
    pnh_.param("trajectory_topic", trajectory_topic_, std::string("/local_optimizer/candidate_trajectory"));
    pnh_.param("visualization_topic", visualization_topic_,
               std::string("/local_optimizer/debug_trajectory"));
    pnh_.param("corridor_visualization_topic", corridor_visualization_topic_,
               std::string("/local_optimizer/polyhedrons"));
    pnh_.param("corridor_points_visualization_topic", corridor_points_visualization_topic_,
               std::string("/local_optimizer/corridor_points"));
    request_sub_ = nh_.subscribe(request_topic_, 4, &LocalMincoOptimizerNode::requestCallback, this);
    map_sub_ = nh_.subscribe(costmap_topic_, 1, &LocalMincoOptimizerNode::mapCallback, this);
    // 失败状态必须保留，便于在规划结束后仍能通过rostopic定位精确原因。
    status_pub_ = nh_.advertise<robot_trajectory_msgs::LocalOptimizationStatus>(status_topic_, 1, true);
    trajectory_pub_ = nh_.advertise<robot_trajectory_msgs::RobotTrajectory>(trajectory_topic_, 1);
    visualization_pub_ = nh_.advertise<visualization_msgs::Marker>(visualization_topic_, 1, true);
    corridor_visualization_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>(
        corridor_visualization_topic_, 1, true);
    corridor_points_visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        corridor_points_visualization_topic_, 1, true);
    if (!config.use_safe_corridor_constraints) {
      // PolyhedronArray为latched话题；启动时显式清空，避免RViz保留上次走廊模式的旧结果。
      decomp_ros_msgs::PolyhedronArray clear_corridors;
      clear_corridors.header.frame_id = "map";
      clear_corridors.header.stamp = ros::Time::now();
      corridor_visualization_pub_.publish(clear_corridors);
    }
    worker_ = std::thread(&LocalMincoOptimizerNode::workerLoop, this);
    ROS_INFO("Local MINCO optimizer ready: request=%s trajectory=%s visualization=%s corridors=%s points=%s mode=%s",
             request_topic_.c_str(), trajectory_topic_.c_str(), visualization_topic_.c_str(),
             corridor_visualization_topic_.c_str(), corridor_points_visualization_topic_.c_str(),
             config.use_safe_corridor_constraints ? "SAFE_CORRIDOR" :
             (config.use_associated_obstacle_constraints ? "ASSOCIATED_OBSTACLE_POINTS" : "NO_STATIC_OBSTACLE_CONSTRAINT"));
    ROS_INFO("Local MINCO settings: corridor_cost>=%d unknown=%s vehicle=%.2fx%.2f margin=%.2f "
             "weights(obs=%.0f,dyn=%.0f,feas=%.0f,time=%.0f,anchor=%.0f) "
             "limits(v=%.2f,a=%.2f,cur=%.2f) lbfgs(max_iter=1000,mem=64,g_eps=1e-4,min_step=1e-12) "
             "iteration_log=%s/%d solver_budget_ms=%.1f",
             config.occupied_threshold, config.unknown_is_obstacle ? "true" : "false",
             config.vehicle_length_m, config.vehicle_width_m, config.footprint_margin_m,
             config.minco.wei_obs, config.minco.wei_surround, config.minco.wei_feas,
             config.minco.wei_time, config.minco.wei_anchor, config.minco.max_vel,
             config.minco.max_acc, config.minco.max_cur,
             enable_iteration_log ? "on" : "off", config.minco.logging_every_n,
             max_solver_time_ms_);
  }

  ~LocalMincoOptimizerNode() {
    {
      std::lock_guard<std::mutex> lock(lock_);
      shutting_down_ = true;
      ++generation_;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& message) {
    std::lock_guard<std::mutex> lock(lock_);
    map_ = *message;
    has_map_ = map_.info.resolution > 0.0 && map_.info.width > 0 &&
        map_.info.height > 0 && map_.data.size() ==
        static_cast<std::size_t>(map_.info.width) * map_.info.height;
  }

  void publishStatus(const uint8_t state, const uint64_t episode_id,
                     const uint64_t request_id, const std::string& reason) {
    robot_trajectory_msgs::LocalOptimizationStatus status;
    status.header.stamp = ros::Time::now();
    status.state = state;
    status.episode_id = episode_id;
    status.request_id = request_id;
    status.reason = reason;
    status_pub_.publish(status);
  }

  void publishTrajectoryMarker(const robot_trajectory_msgs::RobotTrajectory& trajectory) {
    visualization_msgs::Marker marker;
    marker.header = trajectory.header;
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_minco";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    // 局部MINCO优化输出固定为绿色，便于与其他规划层轨迹区分。
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) marker.points.push_back(point.pose.position);
    visualization_pub_.publish(marker);
  }

  void publishCorridorMarkers(const std::vector<Eigen::MatrixXd>& corridors,
                              const std_msgs::Header& header) {
    vec_E<Polyhedron2D> polyhedra;
    polyhedra.reserve(corridors.size());
    for (const auto& corridor : corridors) {
      Polyhedron2D polyhedron;
      for (Eigen::Index index = 0; index < corridor.cols(); ++index) {
        polyhedron.add(Hyperplane2D(corridor.col(index).tail<2>(),
                                    corridor.col(index).head<2>()));
      }
      polyhedra.push_back(polyhedron);
    }
    decomp_ros_msgs::PolyhedronArray message = DecompROS::polyhedron_array_to_ros(polyhedra);
    message.header = header;
    message.header.stamp = ros::Time::now();
    corridor_visualization_pub_.publish(message);
  }

  void publishCorridorPoints(const std::vector<Eigen::Vector2d>& key_points,
                             const std_msgs::Header& header) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const ros::Time stamp = ros::Time::now();
    for (std::size_t index = 0; index < key_points.size(); ++index) {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.header.stamp = stamp;
      marker.ns = "corridor_points";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = key_points[index].x();
      marker.pose.position.y = key_points[index].y();
      marker.pose.position.z = 0.05;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.10;
      marker.scale.y = 0.10;
      marker.scale.z = 0.10;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      markers.markers.push_back(marker);
    }
    corridor_points_visualization_pub_.publish(markers);
  }

  void requestCallback(const robot_trajectory_msgs::LocalOptimizationRequestConstPtr& message) {
    if (message->action == robot_trajectory_msgs::LocalOptimizationRequest::CANCEL) {
      {
        std::lock_guard<std::mutex> lock(lock_);
        ++generation_;
        has_pending_ = false;
      }
      condition_.notify_all();
      publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::CANCELLED,
                    message->episode_id, message->request_id, message->reason);
      return;
    }
    if (message->action != robot_trajectory_msgs::LocalOptimizationRequest::OPTIMIZE) return;
    {
      std::lock_guard<std::mutex> lock(lock_);
      pending_request_ = *message;
      has_pending_ = true;
      ++generation_;
    }
    condition_.notify_one();
  }

  void workerLoop() {
    while (ros::ok()) {
      robot_trajectory_msgs::LocalOptimizationRequest request;
      nav_msgs::OccupancyGrid map;
      std::uint64_t generation = 0;
      {
        std::unique_lock<std::mutex> lock(lock_);
        condition_.wait(lock, [this] { return shutting_down_ || has_pending_; });
        if (shutting_down_) return;
        request = pending_request_;
        has_pending_ = false;
        generation = generation_;
        if (!has_map_) {
          lock.unlock();
          ROS_ERROR("Local MINCO rejected A* candidate: episode=%llu request=%llu reason=NO_VALID_COSTMAP",
                    static_cast<unsigned long long>(request.episode_id),
                    static_cast<unsigned long long>(request.request_id));
          publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::REJECTED,
                        request.episode_id, request.request_id, "NO_VALID_COSTMAP");
          continue;
        }
        map = map_;
      }
      publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::STARTED,
                    request.episode_id, request.request_id, "STARTED");
      ROS_INFO("Local MINCO start: episode=%llu request=%llu astar_points=%zu map=%ux%u resolution=%.3f",
               static_cast<unsigned long long>(request.episode_id),
               static_cast<unsigned long long>(request.request_id),
               request.geometric_path.poses.size(), map.info.width, map.info.height,
               map.info.resolution);
      const auto total_start = std::chrono::steady_clock::now();
      const auto deadline = total_start + std::chrono::milliseconds(
          static_cast<long long>(std::max(0.0, max_solver_time_ms_)));
      bool time_budget_exceeded = false;
      const auto superseded = [this, generation] {
        return shutting_down_ || generation_.load() != generation;
      };
      const auto cancelled = [&superseded, &deadline, &time_budget_exceeded, this] {
        if (superseded()) return true;
        if (max_solver_time_ms_ > 0.0 && std::chrono::steady_clock::now() >= deadline) {
          time_budget_exceeded = true;
          return true;
        }
        return false;
      };
      LocalMincoResult result = optimizer_->optimize(request, map, cancelled);
      const double total_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - total_start).count();
      if (time_budget_exceeded && !superseded()) {
        result.cancelled = false;
        result.reason = "TIME_BUDGET_EXCEEDED";
        ROS_ERROR("Local MINCO exceeded time budget for A* candidate: episode=%llu request=%llu "
                  "budget_ms=%.1f iter=%d solver_ms=%.3f total_ms=%.3f",
                  static_cast<unsigned long long>(request.episode_id),
                  static_cast<unsigned long long>(request.request_id), max_solver_time_ms_,
                  result.iterations, result.optimize_time_ms, total_time_ms);
        publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::FAILED,
                      request.episode_id, request.request_id, result.reason);
      } else if (superseded() || result.cancelled) {
        ROS_WARN("Local MINCO cancelled: episode=%llu request=%llu reason=%s iter=%d solver_ms=%.3f total_ms=%.3f",
                 static_cast<unsigned long long>(request.episode_id),
                 static_cast<unsigned long long>(request.request_id), result.reason.c_str(),
                 result.iterations, result.optimize_time_ms, total_time_ms);
        publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::CANCELLED,
                      request.episode_id, request.request_id, result.reason);
      } else if (!result.success) {
        ROS_ERROR("Local MINCO failed for A* candidate: episode=%llu request=%llu reason=%s iter=%d "
                  "solver_result=%d solver_ms=%.3f total_ms=%.3f final_cost=%.6f",
                  static_cast<unsigned long long>(request.episode_id),
                  static_cast<unsigned long long>(request.request_id), result.reason.c_str(),
                  result.iterations, result.solver_result, result.optimize_time_ms,
                  total_time_ms, result.final_cost);
        publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::FAILED,
                      request.episode_id, request.request_id, result.reason);
      } else {
        ROS_INFO("Local MINCO success: episode=%llu request=%llu key_points=%zu corridors=%zu "
                 "trajectory_points=%zu iter=%d solver_ms=%.3f total_ms=%.3f final_cost=%.6f",
                 static_cast<unsigned long long>(request.episode_id),
                 static_cast<unsigned long long>(request.request_id), result.key_points.size(),
                 result.corridors.size(), result.trajectory.points.size(), result.iterations,
                 result.optimize_time_ms, total_time_ms, result.final_cost);
        trajectory_pub_.publish(result.trajectory);
        publishTrajectoryMarker(result.trajectory);
        publishCorridorMarkers(result.corridors, result.trajectory.header);
        publishCorridorPoints(result.key_points, result.trajectory.header);
        publishStatus(robot_trajectory_msgs::LocalOptimizationStatus::SUCCEEDED,
                      request.episode_id, request.request_id, "SUCCESS");
      }
    }
  }

  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<LocalMincoOptimizer> optimizer_;
  ros::Subscriber request_sub_, map_sub_;
  ros::Publisher status_pub_, trajectory_pub_, visualization_pub_, corridor_visualization_pub_,
      corridor_points_visualization_pub_;
  std::string request_topic_, costmap_topic_, status_topic_, trajectory_topic_, visualization_topic_,
      corridor_visualization_topic_, corridor_points_visualization_topic_;
  std::mutex lock_;
  std::condition_variable condition_;
  std::thread worker_;
  nav_msgs::OccupancyGrid map_;
  robot_trajectory_msgs::LocalOptimizationRequest pending_request_;
  bool has_map_ = false, has_pending_ = false;
  std::atomic<bool> shutting_down_{false};
  std::atomic<std::uint64_t> generation_{0};
  double max_solver_time_ms_ = 400.0;
};

}  // namespace local_optimization_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_minco_optimizer");
  local_optimization_manager::LocalMincoOptimizerNode node;
  ros::spin();
  return 0;
}
