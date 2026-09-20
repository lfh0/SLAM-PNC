#include "local_optimization_manager/local_minco_optimizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <tf2/utils.h>

#include <decomp_util/ellipsoid_decomp.h>

namespace local_optimization_manager {
namespace {
double distance(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
  return (lhs - rhs).norm();
}

double pointToSegmentDistance(const Eigen::Vector2d& point,
                              const Eigen::Vector2d& begin,
                              const Eigen::Vector2d& end) {
  const Eigen::Vector2d segment = end - begin;
  const double squared_length = segment.squaredNorm();
  if (squared_length < 1e-12) return distance(point, begin);
  const double ratio = std::max(0.0, std::min(1.0,
      (point - begin).dot(segment) / squared_length));
  return distance(point, begin + ratio * segment);
}

}  // namespace

LocalMincoOptimizer::LocalMincoOptimizer(const LocalMincoConfig& config)
    : config_(config) {}

bool LocalMincoOptimizer::occupied(const nav_msgs::OccupancyGrid& map,
                                   const double x, const double y) const {
  if (map.info.resolution <= 0.0 || map.data.empty()) return true;
  const double origin_yaw = tf2::getYaw(map.info.origin.orientation);
  const double dx = x - map.info.origin.position.x;
  const double dy = y - map.info.origin.position.y;
  const int grid_x = static_cast<int>(std::floor(
      (std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy) /
      map.info.resolution));
  const int grid_y = static_cast<int>(std::floor(
      (-std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy) /
      map.info.resolution));
  if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(map.info.width) ||
      grid_y >= static_cast<int>(map.info.height)) return true;
  const int cost = map.data[grid_y * map.info.width + grid_x];
  return cost < 0 ? config_.unknown_is_obstacle
                  : cost >= config_.occupied_threshold;
}

bool LocalMincoOptimizer::buildCorridors(
    const nav_msgs::Path& path, const nav_msgs::OccupancyGrid& map,
    std::vector<Eigen::Vector2d>* key_points,
    std::vector<Eigen::MatrixXd>* corridors, std::string* reason,
    const std::function<bool()>& cancel_checker) const {
  if (path.poses.size() < 2) {
    *reason = "PATH_TOO_SHORT";
    return false;
  }
  std::vector<Eigen::Vector2d> points;
  points.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      *reason = "PATH_NOT_FINITE";
      return false;
    }
    if (points.empty() || distance(points.back(), point) > 1e-4) points.push_back(point);
  }
  if (points.size() < 2) {
    *reason = "PATH_HAS_ONLY_DUPLICATE_POINTS";
    return false;
  }

  std::vector<double> cumulative_length(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    cumulative_length[index] = cumulative_length[index - 1] +
        distance(points[index - 1], points[index]);
  }
  const auto arc_length = [&](const std::size_t begin, const std::size_t end) {
    return cumulative_length[end] - cumulative_length[begin];
  };
  const auto line_collision = [&](const Eigen::Vector2d& begin,
                                  const Eigen::Vector2d& end) {
    const double length = distance(begin, end);
    const int samples = std::max(1, static_cast<int>(std::ceil(
        length / std::max(0.025, 0.5 * static_cast<double>(map.info.resolution)))));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const Eigen::Vector2d point = begin + ratio * (end - begin);
      if (occupied(map, point.x(), point.y())) return true;
    }
    return false;
  };
  const auto deviation_info = [&](const std::size_t begin, const std::size_t end) {
    const Eigen::Vector2d start = points[begin];
    const Eigen::Vector2d finish = points[end];
    const Eigen::Vector2d segment = finish - start;
    const double squared_length = segment.squaredNorm();
    double maximum = 0.0;
    std::size_t maximum_index = begin;
    for (std::size_t index = begin + 1; index < end; ++index) {
      const double ratio = squared_length < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0,
          (points[index] - start).dot(segment) / squared_length));
      const double deviation = distance(points[index], start + ratio * segment);
      if (deviation > maximum) {
        maximum = deviation;
        maximum_index = index;
      }
    }
    return std::make_pair(maximum, maximum_index);
  };
  const auto arc_midpoint = [&](const std::size_t begin, const std::size_t end) {
    const double target = 0.5 * (cumulative_length[begin] + cumulative_length[end]);
    const auto iterator = std::lower_bound(cumulative_length.begin() + begin + 1,
        cumulative_length.begin() + end, target);
    return iterator == cumulative_length.begin() + end ? end - 1 :
        static_cast<std::size_t>(std::distance(cumulative_length.begin(), iterator));
  };

  // 与全局MINCO一致：RDP偏差、最大段长和弦线碰撞共同决定关键点。
  std::vector<std::size_t> key_indices{0};
  std::function<bool(std::size_t, std::size_t)> refine;
  refine = [&](const std::size_t begin, const std::size_t end) {
    if (cancel_checker && cancel_checker()) {
      *reason = "CANCELLED";
      return false;
    }
    const auto deviation = deviation_info(begin, end);
    if (!line_collision(points[begin], points[end]) &&
        arc_length(begin, end) <= config_.max_seed_length_m &&
        deviation.first <= config_.max_seed_deviation_m) {
      key_indices.push_back(end);
      return true;
    }
    if (end <= begin + 1) {
      *reason = "PATH_SEGMENT_COLLIDES_OR_EXCEEDS_LIMIT";
      return false;
    }
    std::size_t split = deviation.second;
    if (split <= begin || split >= end ||
        arc_length(begin, split) < config_.min_seed_length_m ||
        arc_length(split, end) < config_.min_seed_length_m) {
      split = arc_midpoint(begin, end);
    }
    if (split <= begin || split >= end ||
        arc_length(begin, split) < config_.min_seed_length_m ||
        arc_length(split, end) < config_.min_seed_length_m) {
      *reason = "CANNOT_SPLIT_WITH_MIN_SEED_LENGTH";
      return false;
    }
    return refine(begin, split) && refine(split, end);
  };
  if (!refine(0, points.size() - 1)) return false;

  // 与全局MINCO一致：安全合并短段，急弯、偏差或碰撞时不合并。
  bool merged = true;
  while (merged && key_indices.size() > 2) {
    merged = false;
    for (std::size_t index = 1; index + 1 < key_indices.size(); ++index) {
      const std::size_t previous = key_indices[index - 1];
      const std::size_t current = key_indices[index];
      const std::size_t next = key_indices[index + 1];
      if (std::min(arc_length(previous, current), arc_length(current, next)) >=
          config_.min_seed_length_m) continue;
      const Eigen::Vector2d incoming = (points[current] - points[previous]).normalized();
      const Eigen::Vector2d outgoing = (points[next] - points[current]).normalized();
      const double turn = std::acos(std::max(-1.0, std::min(1.0, incoming.dot(outgoing))));
      if (turn <= config_.max_seed_yaw_error_rad &&
          arc_length(previous, next) <= config_.max_seed_length_m &&
          deviation_info(previous, next).first <= config_.max_seed_deviation_m &&
          !line_collision(points[previous], points[next])) {
        key_indices.erase(key_indices.begin() + index);
        merged = true;
        break;
      }
    }
  }

  if (key_indices.size() < 3) {
    *reason = "MINCO_NEEDS_AT_LEAST_TWO_PIECES";
    return false;
  }
  key_points->clear();
  for (const std::size_t index : key_indices) key_points->push_back(points[index]);
  corridors->clear();
  if (!config_.use_safe_corridor_constraints) {
    // 关联点模式只复用RDP关键点；不构建凸分解，也不产生走廊约束。
    return true;
  }

  const double map_yaw = tf2::getYaw(map.info.origin.orientation);
  const double map_cos = std::cos(map_yaw);
  const double map_sin = std::sin(map_yaw);
  const auto world_to_map = [&](const Eigen::Vector2d& point) {
    const double dx = point.x() - map.info.origin.position.x;
    const double dy = point.y() - map.info.origin.position.y;
    return Eigen::Vector2d((map_cos * dx + map_sin * dy) / map.info.resolution,
                           (-map_sin * dx + map_cos * dy) / map.info.resolution);
  };
  const auto map_to_world = [&](const double x, const double y) {
    return Eigen::Vector2d(map.info.origin.position.x +
        map_cos * x * map.info.resolution - map_sin * y * map.info.resolution,
        map.info.origin.position.y +
        map_sin * x * map.info.resolution + map_cos * y * map.info.resolution);
  };
  std::vector<Polyhedron2D> polyhedra;
  for (std::size_t index = 0; index + 1 < key_points->size(); ++index) {
    const Eigen::Vector2d start = key_points->at(index);
    const Eigen::Vector2d finish = key_points->at(index + 1);
    const Eigen::Vector2d direction = (finish - start).normalized();
    const Eigen::Vector2d normal(-direction.y(), direction.x());
    const std::array<Eigen::Vector2d, 4> corners{{
        start - config_.corridor_max_longitudinal_m * direction + config_.corridor_max_lateral_m * normal,
        start - config_.corridor_max_longitudinal_m * direction - config_.corridor_max_lateral_m * normal,
        finish + config_.corridor_max_longitudinal_m * direction + config_.corridor_max_lateral_m * normal,
        finish + config_.corridor_max_longitudinal_m * direction - config_.corridor_max_lateral_m * normal}};
    Eigen::Vector2d min_cell = world_to_map(corners.front());
    Eigen::Vector2d max_cell = min_cell;
    for (const auto& corner : corners) {
      min_cell = min_cell.cwiseMin(world_to_map(corner));
      max_cell = max_cell.cwiseMax(world_to_map(corner));
    }
    vec_Vec2f obstacles;
    for (int y = std::max(0, static_cast<int>(std::floor(min_cell.y())) - 1);
         y <= std::min(static_cast<int>(map.info.height) - 1, static_cast<int>(std::ceil(max_cell.y())) + 1); ++y) {
      for (int x = std::max(0, static_cast<int>(std::floor(min_cell.x())) - 1);
           x <= std::min(static_cast<int>(map.info.width) - 1, static_cast<int>(std::ceil(max_cell.x())) + 1); ++x) {
        const int cost = map.data[y * map.info.width + x];
        if ((cost < 0 && config_.unknown_is_obstacle) || cost >= config_.occupied_threshold)
          obstacles.push_back(map_to_world(x + 0.5, y + 0.5));
      }
    }
    LineSegment2D decomposition(start, finish);
    decomposition.set_local_bbox(Vec2f(config_.corridor_max_longitudinal_m,
                                       config_.corridor_max_lateral_m));
    decomposition.set_obs(obstacles);
    decomposition.dilate(0.0);
    Polyhedron2D polyhedron = decomposition.get_polyhedron();
    const Eigen::Vector2d axis_x(map_cos, map_sin), axis_y(-map_sin, map_cos);
    const Eigen::Vector2d origin(map.info.origin.position.x, map.info.origin.position.y);
    polyhedron.add(Hyperplane2D(origin + axis_x * map.info.width * map.info.resolution, axis_x));
    polyhedron.add(Hyperplane2D(origin, -axis_x));
    polyhedron.add(Hyperplane2D(origin + axis_y * map.info.height * map.info.resolution, axis_y));
    polyhedron.add(Hyperplane2D(origin, -axis_y));
    const auto planes = polyhedron.hyperplanes();
    if (planes.empty()) {
      *reason = "NO_CONVEX_SAFE_CORRIDOR";
      return false;
    }
    Eigen::MatrixXd corridor(4, planes.size());
    for (std::size_t plane = 0; plane < planes.size(); ++plane) {
      corridor.col(plane).head<2>() = planes[plane].n_;
      corridor.col(plane).tail<2>() = planes[plane].p_;
    }
    corridors->push_back(corridor);
    polyhedra.push_back(polyhedron);
  }
  for (std::size_t index = 0; index + 1 < polyhedra.size(); ++index) {
    Polyhedron2D intersection;
    for (const auto& plane : polyhedra[index].hyperplanes()) intersection.add(plane);
    for (const auto& plane : polyhedra[index + 1].hyperplanes()) intersection.add(plane);
    const vec_Vec2f vertices = cal_vertices(intersection);
    double area = 0.0;
    for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
      const Vec2f& first = vertices[vertex];
      const Vec2f& second = vertices[(vertex + 1) % vertices.size()];
      area += first.x() * second.y() - first.y() * second.x();
    }
    if (vertices.size() < 3 || 0.5 * std::abs(area) < config_.corridor_min_overlap_area_m2) {
      *reason = "CORRIDOR_OVERLAP_TOO_SMALL";
      corridors->clear();
      return false;
    }
  }
  return true;
}

bool LocalMincoOptimizer::buildAssociatedObstaclePoints(
    const std::vector<Eigen::Vector2d>& key_points,
    const nav_msgs::OccupancyGrid& map,
    std::vector<std::vector<Eigen::Vector2d>>* associated_points,
    std::string* reason,
    const std::function<bool()>& cancel_checker) const {
  if (key_points.size() < 3 || map.info.resolution <= 0.0 || map.data.empty()) {
    *reason = "INVALID_ASSOCIATED_OBSTACLE_INPUT";
    return false;
  }
  const double map_yaw = tf2::getYaw(map.info.origin.orientation);
  const double map_cos = std::cos(map_yaw);
  const double map_sin = std::sin(map_yaw);
  const auto map_to_world = [&](const double x, const double y) {
    return Eigen::Vector2d(map.info.origin.position.x +
        map_cos * x * map.info.resolution - map_sin * y * map.info.resolution,
        map.info.origin.position.y +
        map_sin * x * map.info.resolution + map_cos * y * map.info.resolution);
  };
  const auto world_to_map = [&](const Eigen::Vector2d& point) {
    const double dx = point.x() - map.info.origin.position.x;
    const double dy = point.y() - map.info.origin.position.y;
    return Eigen::Vector2d((map_cos * dx + map_sin * dy) / map.info.resolution,
                           (-map_sin * dx + map_cos * dy) / map.info.resolution);
  };
  const double radius = std::max(static_cast<double>(map.info.resolution),
                                 config_.associated_obstacle_search_radius_m);
  associated_points->clear();
  for (std::size_t segment = 0; segment + 1 < key_points.size(); ++segment) {
    const int samples = segment == 0 || segment + 2 == key_points.size()
        ? config_.minco.destraj_resolution : config_.minco.traj_resolution;
    for (int sample = 0; sample <= samples; ++sample) {
      if (cancel_checker && cancel_checker()) {
        *reason = "CANCELLED";
        return false;
      }
      const double ratio = static_cast<double>(sample) / samples;
      const Eigen::Vector2d center = key_points[segment] + ratio *
          (key_points[segment + 1] - key_points[segment]);
      const Eigen::Vector2d center_cell = world_to_map(center);
      const int cell_radius = static_cast<int>(std::ceil(radius / map.info.resolution));
      std::vector<std::pair<double, Eigen::Vector2d>> nearest;
      for (int y = std::max(0, static_cast<int>(std::floor(center_cell.y())) - cell_radius);
           y <= std::min(static_cast<int>(map.info.height) - 1,
                         static_cast<int>(std::floor(center_cell.y())) + cell_radius); ++y) {
        for (int x = std::max(0, static_cast<int>(std::floor(center_cell.x())) - cell_radius);
             x <= std::min(static_cast<int>(map.info.width) - 1,
                           static_cast<int>(std::floor(center_cell.x())) + cell_radius); ++x) {
          const int cost = map.data[y * map.info.width + x];
          if (!((cost < 0 && config_.unknown_is_obstacle) ||
                cost >= config_.occupied_threshold)) continue;
          const Eigen::Vector2d obstacle = map_to_world(x + 0.5, y + 0.5);
          const double squared_distance = (obstacle - center).squaredNorm();
          if (squared_distance <= radius * radius) nearest.emplace_back(squared_distance, obstacle);
        }
      }
      std::sort(nearest.begin(), nearest.end(),
                [](const std::pair<double, Eigen::Vector2d>& lhs,
                   const std::pair<double, Eigen::Vector2d>& rhs) {
                  return lhs.first < rhs.first;
                });
      std::vector<Eigen::Vector2d> points;
      const std::size_t limit = std::min(nearest.size(), static_cast<std::size_t>(
          std::max(1, config_.max_associated_obstacles_per_sample)));
      points.reserve(limit);
      for (std::size_t index = 0; index < limit; ++index) points.push_back(nearest[index].second);
      associated_points->push_back(std::move(points));
    }
  }
  return true;
}

robot_trajectory_msgs::RobotTrajectory LocalMincoOptimizer::sampleTrajectory(
    const plan_utils::Trajectory& trajectory,
    const robot_trajectory_msgs::LocalOptimizationRequest& request) const {
  robot_trajectory_msgs::RobotTrajectory output;
  // 单元测试和离线调用可能尚未启动 ROS 时钟，此时保留零时间戳。
  output.header.stamp = ros::Time::isValid() ? ros::Time::now() : ros::Time(0);
  output.header.frame_id = request.header.frame_id;
  const double duration = trajectory.getTotalDuration();
  double previous_yaw = tf2::getYaw(request.initial_state.pose.orientation);
  double arc_length = 0.0;
  Eigen::Vector2d previous_position;
  Eigen::Vector2d previous_acceleration;
  for (double time = 0.0; time < duration + 1e-9;
       time += config_.sample_period_s) {
    const double sample_time = std::min(time, duration);
    const Eigen::Vector2d position = trajectory.getPos(sample_time);
    const Eigen::Vector2d velocity = trajectory.getdSigma(sample_time);
    const Eigen::Vector2d acceleration = trajectory.getddSigma(sample_time);
    robot_trajectory_msgs::RobotTrajectoryPoint point;
    point.time_from_start = ros::Duration(sample_time);
    point.sampling_interval = config_.sample_period_s;
    point.pose.position.x = position.x();
    point.pose.position.y = position.y();
    const double speed = velocity.norm();
    if (speed > 1e-3) previous_yaw = std::atan2(velocity.y(), velocity.x());
    point.pose.orientation.z = std::sin(0.5 * previous_yaw);
    point.pose.orientation.w = std::cos(0.5 * previous_yaw);
    point.velocity.linear.x = velocity.x();
    point.velocity.linear.y = velocity.y();
    point.acceleration.linear.x = acceleration.x();
    point.acceleration.linear.y = acceleration.y();
    point.curvature = speed > 1e-3
        ? (velocity.x() * acceleration.y() - velocity.y() * acceleration.x()) /
              std::pow(speed, 3)
        : 0.0;
    point.velocity.angular.z = point.curvature * speed;
    if (!output.points.empty()) {
      arc_length += distance(previous_position, position);
      const double dt = std::max(1e-3, sample_time -
          output.points.back().time_from_start.toSec());
      const Eigen::Vector2d tangent = speed > 1e-3
          ? velocity / speed : Eigen::Vector2d(std::cos(previous_yaw), std::sin(previous_yaw));
      point.longitudinal_jerk = (acceleration - previous_acceleration).dot(tangent) / dt;
    }
    point.arc_length = arc_length;
    output.points.push_back(point);
    previous_position = position;
    previous_acceleration = acceleration;
    if (sample_time >= duration) break;
  }
  return output;
}

bool LocalMincoOptimizer::validateTrajectory(
    const robot_trajectory_msgs::RobotTrajectory& trajectory,
    const nav_msgs::Path& reference_path, const nav_msgs::OccupancyGrid& map,
    std::string* reason) const {
  if (trajectory.points.empty() || reference_path.poses.size() < 2) {
    *reason = "INVALID_TRAJECTORY_OR_REFERENCE";
    return false;
  }
  const double half_length = 0.5 * config_.vehicle_length_m + config_.footprint_margin_m;
  const double half_width = 0.5 * config_.vehicle_width_m + config_.footprint_margin_m;
  const double footprint_step = std::max(0.025, 0.5 * map.info.resolution);
  for (const auto& point : trajectory.points) {
    const Eigen::Vector2d position(point.pose.position.x, point.pose.position.y);
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < reference_path.poses.size(); ++index) {
      const Eigen::Vector2d begin(reference_path.poses[index - 1].pose.position.x,
                                  reference_path.poses[index - 1].pose.position.y);
      const Eigen::Vector2d end(reference_path.poses[index].pose.position.x,
                                reference_path.poses[index].pose.position.y);
      closest_distance = std::min(closest_distance,
          pointToSegmentDistance(position, begin, end));
    }
    if (closest_distance > config_.max_reference_deviation_m) {
      *reason = "REFERENCE_DEVIATION_LIMIT";
      return false;
    }
    const double yaw = tf2::getYaw(point.pose.orientation);
    for (double local_x = -half_length; local_x <= half_length + 1e-6;
         local_x += footprint_step) {
      for (double local_y = -half_width; local_y <= half_width + 1e-6;
           local_y += footprint_step) {
        const double x = point.pose.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
        const double y = point.pose.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y;
        if (occupied(map, x, y)) {
          *reason = "OPTIMIZED_TRAJECTORY_COLLISION";
          return false;
        }
      }
    }
  }
  return true;
}

LocalMincoResult LocalMincoOptimizer::optimize(
    const robot_trajectory_msgs::LocalOptimizationRequest& request,
    const nav_msgs::OccupancyGrid& map,
    const std::function<bool()>& cancel_checker) const {
  LocalMincoResult result;
  if (cancel_checker && cancel_checker()) {
    result.cancelled = true;
    result.reason = "CANCELLED";
    return result;
  }
  std::vector<Eigen::Vector2d> key_points;
  std::vector<Eigen::MatrixXd> corridors;
  if (!buildCorridors(request.geometric_path, map, &key_points, &corridors,
                      &result.reason, cancel_checker)) {
    result.cancelled = result.reason == "CANCELLED";
    return result;
  }
  std::vector<std::vector<Eigen::Vector2d>> associated_obstacle_points;
  if (!config_.use_safe_corridor_constraints && config_.use_associated_obstacle_constraints &&
      !buildAssociatedObstaclePoints(key_points, map, &associated_obstacle_points,
                                     &result.reason, cancel_checker)) {
    result.cancelled = result.reason == "CANCELLED";
    return result;
  }

  Eigen::MatrixXd initial_state = Eigen::MatrixXd::Zero(2, 3);
  Eigen::MatrixXd final_state = Eigen::MatrixXd::Zero(2, 3);
  initial_state.col(0) = key_points.front();
  final_state.col(0) = key_points.back();
  initial_state(0, 1) = request.initial_state.velocity.linear.x;
  initial_state(1, 1) = request.initial_state.velocity.linear.y;
  initial_state(0, 2) = request.initial_state.acceleration.linear.x;
  initial_state(1, 2) = request.initial_state.acceleration.linear.y;
  final_state(0, 1) = request.final_state.velocity.linear.x;
  final_state(1, 1) = request.final_state.velocity.linear.y;
  final_state(0, 2) = request.final_state.acceleration.linear.x;
  final_state(1, 2) = request.final_state.acceleration.linear.y;

  Eigen::MatrixXd inner_points(2, static_cast<int>(key_points.size()) - 2);
  for (std::size_t index = 1; index + 1 < key_points.size(); ++index) {
    inner_points.col(index - 1) = key_points[index];
  }
  // 与全局MINCO一致：内部节点按转角降速，首尾仍保留局部请求给出的真实速度。
  Eigen::VectorXd node_speeds = Eigen::VectorXd::Constant(
      static_cast<Eigen::Index>(key_points.size()), config_.minco.max_vel);
  node_speeds[0] = initial_state.col(1).norm();
  node_speeds[node_speeds.size() - 1] = final_state.col(1).norm();
  for (std::size_t index = 1; index + 1 < key_points.size(); ++index) {
    const Eigen::Vector2d incoming = key_points[index] - key_points[index - 1];
    const Eigen::Vector2d outgoing = key_points[index + 1] - key_points[index];
    const double cosine = std::max(-1.0, std::min(1.0,
        incoming.normalized().dot(outgoing.normalized())));
    const double turn_angle = std::acos(cosine);
    const double speed_ratio = std::max(
        config_.min_turn_speed_ratio, 1.0 - turn_angle / M_PI);
    node_speeds[static_cast<Eigen::Index>(index)] = config_.minco.max_vel * speed_ratio;
  }

  const std::size_t piece_count = key_points.size() - 1;
  Eigen::VectorXd piece_times(piece_count);
  for (std::size_t index = 0; index < piece_count; ++index) {
    const double length = distance(key_points[index], key_points[index + 1]);
    const double begin_speed = node_speeds[static_cast<Eigen::Index>(index)];
    const double end_speed = node_speeds[static_cast<Eigen::Index>(index + 1)];
    const double velocity_time = length / std::max(config_.min_speed_mps,
        0.5 * (begin_speed + end_speed));
    const double acceleration_time = std::abs(end_speed - begin_speed) /
        std::max(1e-3, config_.minco.max_acc);
    piece_times[index] = std::max(config_.min_piece_time_s,
                                   std::max(velocity_time, acceleration_time));
  }

  plan_manage::MincoRequest minco_request;
  minco_request.initial_state = initial_state;
  minco_request.final_state = final_state;
  minco_request.initial_inner_points = inner_points;
  minco_request.total_time = piece_times.sum();
  minco_request.use_safe_corridor_constraints = config_.use_safe_corridor_constraints;
  minco_request.use_associated_obstacle_constraints =
      !config_.use_safe_corridor_constraints && config_.use_associated_obstacle_constraints;
  minco_request.piece_time_ratios = piece_times / minco_request.total_time;
  minco_request.singular_direction = 1;
  minco_request.start_time = ros::Time::isValid() ? ros::Time::now().toSec() : 0.0;
  minco_request.cancel_checker = cancel_checker;
  minco_request.associated_obstacle_points = std::move(associated_obstacle_points);
  minco_request.static_obstacle_clearance = std::max(
      config_.associated_obstacle_clearance_m,
      static_cast<double>(map.info.resolution) * 0.71);
  for (std::size_t index = 0; index < corridors.size(); ++index) {
    const int resolution = index == 0 || index + 1 == corridors.size()
        ? config_.minco.destraj_resolution : config_.minco.traj_resolution;
    for (int sample = 0; sample <= resolution; ++sample) {
      minco_request.sampled_corridors.push_back(corridors[index]);
    }
  }

  plan_manage::PolyTrajOptimizer optimizer;
  optimizer.configure(config_.minco);
  // 局部节点按配置输出迭代代价分解；关闭时仍保留最终成功/失败摘要。
  optimizer.setVerbose(config_.minco.logging_every_n > 0);
  const plan_manage::MincoResult optimized = optimizer.optimize(minco_request);
  result.iterations = optimized.iterations;
  result.optimize_time_ms = optimized.optimize_time_ms;
  result.final_cost = optimized.final_cost;
  result.solver_result = optimized.solver_result;
  if (!optimized.success) {
    result.cancelled = optimized.cancelled;
    result.reason = optimized.cancelled ? "CANCELLED" : optimized.failure_reason;
    return result;
  }
  if (cancel_checker && cancel_checker()) {
    result.cancelled = true;
    result.reason = "CANCELLED";
    return result;
  }
  result.trajectory = sampleTrajectory(optimized.trajectory, request);
  if (result.trajectory.points.size() < 2) {
    result.reason = "OPTIMIZED_TRAJECTORY_TOO_SHORT";
    return result;
  }
  // MINCO的数值收敛不等价于能直接发布：必须仍贴合局部A*，且整车不碰撞。
  if (!validateTrajectory(result.trajectory, request.geometric_path, map, &result.reason)) {
    return result;
  }
  result.success = true;
  result.key_points = key_points;
  result.corridors = corridors;
  result.reason = "SUCCESS";
  return result;
}

}  // namespace local_optimization_manager
