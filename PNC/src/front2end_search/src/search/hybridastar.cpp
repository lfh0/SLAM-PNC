#include "path_searching/hybridastar.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <ompl/base/ScopedState.h>

namespace path_searching {
namespace {

template <typename T>
void loadRequiredParam(ros::NodeHandle& nh,
                       const std::string& name,
                       T& value) {
    if (!nh.getParam(name, value)) {
        throw std::runtime_error("缺少 Hybrid A* 参数：" + name);
    }
}

double quaternionToYaw(const geometry_msgs::Quaternion& quaternion) {
    const double sin_yaw = 2.0
        * (quaternion.w * quaternion.z
           + quaternion.x * quaternion.y);
    const double cos_yaw = 1.0
        - 2.0 * (quaternion.y * quaternion.y
                 + quaternion.z * quaternion.z);
    return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::Quaternion yawToQuaternion(const double yaw) {
    geometry_msgs::Quaternion quaternion;
    quaternion.z = std::sin(0.5 * yaw);
    quaternion.w = std::cos(0.5 * yaw);
    return quaternion;
}

struct SearchTiming {
    double initialization_ms = 0.0;
    double obstacle_heuristic_ms = 0.0;
    double queue_ms = 0.0;
    double analytic_ms = 0.0;
    double primitive_generation_ms = 0.0;
    double collision_check_ms = 0.0;
    double state_index_ms = 0.0;
    double transition_cost_ms = 0.0;
    double heuristic_ms = 0.0;
    double node_update_ms = 0.0;
    double path_retrieval_ms = 0.0;

    std::size_t queue_pop_count = 0;
    std::size_t stale_entry_count = 0;
    std::size_t analytic_attempt_count = 0;
    std::size_t primitive_count = 0;
    std::size_t collision_check_count = 0;
    std::size_t collision_reject_count = 0;
    std::size_t heuristic_count = 0;
    std::size_t created_node_count = 0;
};

double elapsedMilliseconds(const ros::WallTime& start) {
    return (ros::WallTime::now() - start).toSec() * 1000.0;
}

}  // namespace

HybridAstar::HybridAstar()
    : odom_initialized_(false),
      search_tree_max_edges_(0),
      map_origin_(Eigen::Vector2d::Zero()),
      map_size_(Eigen::Vector2i::Zero()),
      map_resolution_(0.0),
      inverse_map_resolution_(0.0),
      map_initialized_(false),
      yaw_grid_size_(0),
      yaw_resolution_(0.0),
      inverse_yaw_resolution_(0.0),
      vehicle_width_(0.0),
      vehicle_length_(0.0),
      wheel_base_(0.0),
      rear_overhang_(0.0),
      max_steering_angle_(0.0),
      minimum_turning_radius_(0.0),
      steering_discrete_num_(0),
      primitive_length_(0.0),
      primitive_sample_step_(0.0),
      forward_penalty_(0.0),
      reverse_penalty_(0.0),
      gear_switch_penalty_(0.0),
      steering_penalty_(0.0),
      steering_change_penalty_(0.0),
      heuristic_weight_(0.0),
      reeds_shepp_switch_distance_(0.0),
      analytic_expansion_distance_(0.0),
      goal_position_tolerance_(0.0),
      goal_yaw_tolerance_(0.0),
      max_iterations_(0),
      max_search_time_(0.0),
      obstacle_heuristic_goal_index_(Eigen::Vector2i::Zero()),
      obstacle_heuristic_ready_(false) {
}

HybridAstar::~HybridAstar() = default;

void HybridAstar::init(ros::NodeHandle& nh) {
    loadRequiredParam(nh, "hybridastar/yaw_grid_size", yaw_grid_size_);
    loadRequiredParam(nh, "hybridastar/primitive_length", primitive_length_);
    loadRequiredParam(nh, "hybridastar/primitive_sample_step", primitive_sample_step_);
    loadRequiredParam(nh, "hybridastar/steering_discrete_num", steering_discrete_num_);
    loadRequiredParam(nh, "hybridastar/forward_penalty", forward_penalty_);
    loadRequiredParam(nh, "hybridastar/reverse_penalty", reverse_penalty_);
    loadRequiredParam(nh, "hybridastar/gear_switch_penalty", gear_switch_penalty_);
    loadRequiredParam(nh, "hybridastar/steering_penalty", steering_penalty_);
    loadRequiredParam(nh, "hybridastar/steering_change_penalty", steering_change_penalty_);
    loadRequiredParam(nh, "hybridastar/heuristic_weight", heuristic_weight_);
    loadRequiredParam(nh, "hybridastar/reeds_shepp_switch_distance",
                      reeds_shepp_switch_distance_);
    loadRequiredParam(nh, "hybridastar/analytic_expansion_distance",
                      analytic_expansion_distance_);
    loadRequiredParam(nh, "hybridastar/goal_position_tolerance", goal_position_tolerance_);
    loadRequiredParam(nh, "hybridastar/goal_yaw_tolerance", goal_yaw_tolerance_);
    loadRequiredParam(nh, "hybridastar/max_iterations", max_iterations_);
    loadRequiredParam(nh, "hybridastar/max_search_time", max_search_time_);
    loadRequiredParam(nh, "hybridastar/search_tree_max_edges",
                      search_tree_max_edges_);
    loadRequiredParam(nh, "hybridastar/vehicle/width", vehicle_width_);
    loadRequiredParam(nh, "hybridastar/vehicle/length", vehicle_length_);
    loadRequiredParam(nh, "hybridastar/vehicle/wheel_base", wheel_base_);
    loadRequiredParam(nh, "hybridastar/vehicle/rear_overhang",rear_overhang_);

    double max_steering_angle_deg = 0.0;
    loadRequiredParam(nh, "hybridastar/vehicle/max_steering_angle_deg",
                      max_steering_angle_deg);

    if (yaw_grid_size_ <= 0 || primitive_length_ <= 0.0
        || primitive_sample_step_ <= 0.0
        || primitive_sample_step_ > primitive_length_
        || steering_discrete_num_ <= 0 || vehicle_width_ <= 0.0
        || vehicle_length_ <= 0.0 || wheel_base_ <= 0.0
        || rear_overhang_ < 0.0 || max_steering_angle_deg <= 0.0
        || max_steering_angle_deg >= 90.0 || forward_penalty_ < 1.0
        || reverse_penalty_ < 1.0 || gear_switch_penalty_ < 0.0
        || steering_penalty_ < 0.0 || steering_change_penalty_ < 0.0
        || heuristic_weight_ <= 0.0
        || reeds_shepp_switch_distance_ <= 0.0
        || analytic_expansion_distance_ <= 0.0
        || goal_position_tolerance_ <= 0.0
        || goal_yaw_tolerance_ <= 0.0 || max_iterations_ <= 0
        || max_search_time_ <= 0.0 || search_tree_max_edges_ <= 0) {
        throw std::runtime_error("Hybrid A* 参数存在非法值");
    }

    constexpr double kPi = 3.14159265358979323846;
    yaw_resolution_ = 2.0 * kPi / static_cast<double>(yaw_grid_size_);
    inverse_yaw_resolution_ = 1.0 / yaw_resolution_;
    max_steering_angle_ = max_steering_angle_deg * kPi / 180.0;

    minimum_turning_radius_ =
        wheel_base_ / std::tan(max_steering_angle_);
    reeds_shepp_state_space_ =
        std::make_shared<ompl::base::ReedsSheppStateSpace>(
            minimum_turning_radius_);

    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("search/map_topic", map_topic_, "/map");
    private_nh.param<std::string>("planner/odom_topic", odom_topic_,
                                  "/lio/robo/odom");
    private_nh.param<std::string>("planner/goal_topic", goal_topic_,
                                  "/move_base_simple/goal");
    map_subscriber_ = nh.subscribe(map_topic_, 1,
                                   &HybridAstar::mapCallback, this);
    odom_subscriber_ = nh.subscribe(odom_topic_, 10,
                                    &HybridAstar::odomCallback, this);
    goal_subscriber_ = nh.subscribe(goal_topic_, 1,
                                    &HybridAstar::goalCallback, this);
    path_publisher_ = nh.advertise<nav_msgs::Path>("/hybridastar_path", 1,
                                                   true);
    search_tree_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/hybridastar_search_tree", 1, true);

    reset();

    ROS_INFO("HybridAstar parameters loaded: yaw_bins=%d, primitive=%.3fm, "
             "sample=%.3fm, steering_levels=%d, turning_radius=%.3fm",
             yaw_grid_size_, primitive_length_, primitive_sample_step_,
             steering_discrete_num_, minimum_turning_radius_);
    ROS_INFO("HybridAstar topics: map=%s, odom=%s, goal=%s, "
             "path=/hybridastar_path, tree=/hybridastar_search_tree",
             map_topic_.c_str(), odom_topic_.c_str(), goal_topic_.c_str());
}

void HybridAstar::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& message) {
    setMap(*message);
}

void HybridAstar::odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_ = *message;
    odom_initialized_ = true;
}

void HybridAstar::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!map_initialized_) {
        ROS_WARN("HybridAstar has not received the raw map from %s",
                 map_topic_.c_str());
        return;
    }
    if (!odom_initialized_) {
        ROS_WARN("HybridAstar has not received odometry from %s",
                 odom_topic_.c_str());
        return;
    }

    if (!message->header.frame_id.empty()
        && !map_.header.frame_id.empty()
        && message->header.frame_id != map_.header.frame_id) {
        ROS_WARN("HybridAstar goal frame '%s' differs from map frame '%s'; "
                 "no TF conversion is currently applied",
                 message->header.frame_id.c_str(),
                 map_.header.frame_id.c_str());
    }

    Eigen::Vector3d start_state(
        latest_odom_.pose.pose.position.x,
        latest_odom_.pose.pose.position.y,
        quaternionToYaw(latest_odom_.pose.pose.orientation));
    Eigen::Vector3d goal_state(
        message->pose.position.x,
        message->pose.position.y,
        quaternionToYaw(message->pose.orientation));

    const int status = search(start_state, goal_state);
    if (status == REACH_END) {
        publishPath(final_path_);
        return;
    }

    // 搜索失败时发布空路径，清除锁存的旧结果。
    publishPath(HybridStateVector());
    ROS_WARN("HybridAstar failed to plan to goal (%.3f, %.3f, %.3f)",
             goal_state.x(), goal_state.y(), goal_state.z());
}

void HybridAstar::publishPath(const HybridStateVector& path) {
    nav_msgs::Path path_message;
    path_message.header.stamp = ros::Time::now();
    path_message.header.frame_id = map_.header.frame_id.empty()
        ? "map"
        : map_.header.frame_id;
    path_message.poses.reserve(path.size());

    for (const Eigen::Vector3d& state : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_message.header;
        pose.pose.position.x = state.x();
        pose.pose.position.y = state.y();
        pose.pose.orientation = yawToQuaternion(state.z());
        path_message.poses.push_back(pose);
    }
    path_publisher_.publish(path_message);
}

void HybridAstar::publishSearchTree() const {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    const ros::Time stamp = ros::Time::now();
    const std::string frame_id = map_.header.frame_id.empty()
        ? "map"
        : map_.header.frame_id;
    const auto make_tree_marker = [&](const int id,
                                      const std::string& name_space,
                                      const float red,
                                      const float green,
                                      const float blue) {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id;
        marker.ns = name_space;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.015;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 0.45;
        return marker;
    };

    visualization_msgs::Marker forward_marker = make_tree_marker(
        0, "hybridastar_forward_tree", 0.1F, 0.9F, 0.2F);
    visualization_msgs::Marker backward_marker = make_tree_marker(
        1, "hybridastar_backward_tree", 1.0F, 0.45F, 0.05F);

    const std::size_t maximum_edges =
        static_cast<std::size_t>(search_tree_max_edges_);
    const std::size_t stride = std::max<std::size_t>(
        1, (node_pool_.size() + maximum_edges - 1) / maximum_edges);
    std::size_t published_edges = 0;
    for (std::size_t i = 0;
         i < node_pool_.size() && published_edges < maximum_edges;
         i += stride) {
        const HybridNodePtr node = node_pool_[i].get();
        if (node == nullptr || node->parent == nullptr) {
            continue;
        }

        geometry_msgs::Point parent_point;
        parent_point.x = node->parent->state.x();
        parent_point.y = node->parent->state.y();
        parent_point.z = 0.03;
        geometry_msgs::Point node_point;
        node_point.x = node->state.x();
        node_point.y = node->state.y();
        node_point.z = 0.03;

        visualization_msgs::Marker& marker =
            node->direction == Direction::BACKWARD
                ? backward_marker
                : forward_marker;
        marker.points.push_back(parent_point);
        marker.points.push_back(node_point);
        ++published_edges;
    }

    marker_array.markers.push_back(std::move(forward_marker));
    marker_array.markers.push_back(std::move(backward_marker));
    search_tree_publisher_.publish(marker_array);
    ROS_INFO("HybridAstar search tree published: edges=%zu, total_nodes=%zu",
             published_edges, node_pool_.size());
}

void HybridAstar::setMap(const nav_msgs::OccupancyGrid& map) {
    reset();
    map_initialized_ = false;

    const std::size_t expected_size =
        static_cast<std::size_t>(map.info.width)
        * static_cast<std::size_t>(map.info.height);
    if (map.info.resolution <= 0.0 || map.info.width == 0
        || map.info.height == 0 || map.data.size() != expected_size) {
        ROS_ERROR("HybridAstar received an invalid raw map");
        return;
    }

    map_ = map;
    map_origin_ = Eigen::Vector2d(map.info.origin.position.x,
                                  map.info.origin.position.y);
    map_size_ = Eigen::Vector2i(static_cast<int>(map.info.width),
                                static_cast<int>(map.info.height));
    map_resolution_ = map.info.resolution;
    inverse_map_resolution_ = 1.0 / map_resolution_;
    map_initialized_ = true;

    ROS_INFO("HybridAstar raw map set: size=(%d,%d), resolution=%.3f",
             map_size_.x(), map_size_.y(), map_resolution_);
}

void HybridAstar::reset() {
    expanded_nodes_.clear();
    open_set_ = decltype(open_set_)();
    node_pool_.clear();
    final_path_.clear();
    visited_states_.clear();
    obstacle_heuristic_cost_.clear();
    obstacle_heuristic_goal_index_.setZero();
    obstacle_heuristic_ready_ = false;
}

double HybridAstar::normalizeAngle(const double angle) const {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;

    double normalized_angle = std::fmod(angle + kPi, kTwoPi);
    if (normalized_angle < 0.0) {
        normalized_angle += kTwoPi;
    }
    return normalized_angle - kPi;
}

int HybridAstar::yawToIndex(const double yaw) const {
    constexpr double kPi = 3.14159265358979323846;
    const double normalized_yaw = normalizeAngle(yaw);
    int yaw_index = static_cast<int>(
        std::floor((normalized_yaw + kPi) * inverse_yaw_resolution_));

    // 防止浮点舍入使 pi 附近的角度落到区间之外。
    if (yaw_index < 0) {
        yaw_index = 0;
    } else if (yaw_index >= yaw_grid_size_) {
        yaw_index = yaw_grid_size_ - 1;
    }
    return yaw_index;
}

bool HybridAstar::isInsideMap(const Eigen::Vector2i& grid_index) const {
    return map_initialized_ && grid_index.x() >= 0 && grid_index.y() >= 0
        && grid_index.x() < map_size_.x()
        && grid_index.y() < map_size_.y();
}

bool HybridAstar::positionToIndex(const Eigen::Vector2d& position,
                                  Eigen::Vector2i& grid_index) const {
    if (!map_initialized_ || !position.allFinite()) {
        return false;
    }

    const Eigen::Vector2d map_position =
        (position - map_origin_) * inverse_map_resolution_;
    grid_index.x() = static_cast<int>(std::floor(map_position.x()));
    grid_index.y() = static_cast<int>(std::floor(map_position.y()));
    return isInsideMap(grid_index);
}

bool HybridAstar::stateToIndex(const Eigen::Vector3d& state,
                               Eigen::Vector3i& grid_index) const {
    if (!state.allFinite()) {
        return false;
    }

    Eigen::Vector2i position_index;
    if (!positionToIndex(state.head<2>(), position_index)) {
        return false;
    }

    grid_index.x() = position_index.x();
    grid_index.y() = position_index.y();
    grid_index.z() = yawToIndex(state.z());
    return true;
}

bool HybridAstar::isOccupied(const Eigen::Vector2i& grid_index) const {
    if (!isInsideMap(grid_index)) {
        return true;
    }

    const std::size_t map_index =
        static_cast<std::size_t>(grid_index.y())
            * static_cast<std::size_t>(map_size_.x())
        + static_cast<std::size_t>(grid_index.x());

    // 原始地图中只有数值零表示确定空闲。
    return map_.data[map_index] != 0;
}

VehicleCorners HybridAstar::getVehicleCorners(
    const Eigen::Vector3d& state) const {
    const double front = vehicle_length_ - rear_overhang_;
    const double rear = -rear_overhang_;
    const double half_width = 0.5 * vehicle_width_;

    const double cos_yaw = std::cos(state.z());
    const double sin_yaw = std::sin(state.z());
    const Eigen::Matrix2d rotation =
        (Eigen::Matrix2d() << cos_yaw, -sin_yaw,
                              sin_yaw, cos_yaw).finished();
    const Eigen::Vector2d position = state.head<2>();

    VehicleCorners corners;
    corners[0] = position + rotation * Eigen::Vector2d(front, half_width);
    corners[1] = position + rotation * Eigen::Vector2d(front, -half_width);
    corners[2] = position + rotation * Eigen::Vector2d(rear, -half_width);
    corners[3] = position + rotation * Eigen::Vector2d(rear, half_width);
    return corners;
}

bool HybridAstar::isGridLineCollisionFree(
    const Eigen::Vector2i& start,
    const Eigen::Vector2i& end) const {
    int x = start.x();
    int y = start.y();
    const int delta_x = end.x() - start.x();
    const int delta_y = end.y() - start.y();
    const int step_x = (delta_x > 0) - (delta_x < 0);
    const int step_y = (delta_y > 0) - (delta_y < 0);
    const int count_x = std::abs(delta_x);
    const int count_y = std::abs(delta_y);
    int traversed_x = 0;
    int traversed_y = 0;

    if (isOccupied(Eigen::Vector2i(x, y))) {
        return false;
    }

    while (traversed_x < count_x || traversed_y < count_y) {
        const long decision =
            static_cast<long>(1 + 2 * traversed_x) * count_y
            - static_cast<long>(1 + 2 * traversed_y) * count_x;

        if (decision == 0) {
            // 线段穿过栅格角点时，同时检查角点两侧的栅格。
            if (isOccupied(Eigen::Vector2i(x + step_x, y))
                || isOccupied(Eigen::Vector2i(x, y + step_y))) {
                return false;
            }
            x += step_x;
            y += step_y;
            ++traversed_x;
            ++traversed_y;
        } else if (decision < 0) {
            x += step_x;
            ++traversed_x;
        } else {
            y += step_y;
            ++traversed_y;
        }

        if (isOccupied(Eigen::Vector2i(x, y))) {
            return false;
        }
    }
    return true;
}

bool HybridAstar::isVehicleBoundaryCollisionFree(
    const Eigen::Vector3d& state) const {
    if (!map_initialized_ || !state.allFinite()) {
        return false;
    }

    const VehicleCorners corners = getVehicleCorners(state);
    std::array<Eigen::Vector2i, 4> corner_indices;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        if (!positionToIndex(corners[i], corner_indices[i])) {
            return false;
        }
    }

    for (std::size_t i = 0; i < corner_indices.size(); ++i) {
        const std::size_t next = (i + 1) % corner_indices.size();
        if (!isGridLineCollisionFree(corner_indices[i],
                                     corner_indices[next])) {
            return false;
        }
    }
    return true;
}

bool HybridAstar::isVehicleFootprintCollisionFree(
    const Eigen::Vector3d& state) const {
    if (!isVehicleBoundaryCollisionFree(state)) {
        return false;
    }

    const VehicleCorners corners = getVehicleCorners(state);
    std::array<Eigen::Vector2i, 4> corner_indices;
    for (std::size_t i = 0; i < corners.size(); ++i) {
        if (!positionToIndex(corners[i], corner_indices[i])) {
            return false;
        }
    }

    int min_x = corner_indices[0].x();
    int max_x = corner_indices[0].x();
    int min_y = corner_indices[0].y();
    int max_y = corner_indices[0].y();
    for (std::size_t i = 1; i < corner_indices.size(); ++i) {
        min_x = std::min(min_x, corner_indices[i].x());
        max_x = std::max(max_x, corner_indices[i].x());
        min_y = std::min(min_y, corner_indices[i].y());
        max_y = std::max(max_y, corner_indices[i].y());
    }

    const double front = vehicle_length_ - rear_overhang_;
    const double rear = -rear_overhang_;
    const double half_width = 0.5 * vehicle_width_;
    const double cos_yaw = std::cos(state.z());
    const double sin_yaw = std::sin(state.z());

    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            const Eigen::Vector2d cell_center(
                map_origin_.x() + (static_cast<double>(x) + 0.5)
                    * map_resolution_,
                map_origin_.y() + (static_cast<double>(y) + 0.5)
                    * map_resolution_);
            const Eigen::Vector2d delta = cell_center - state.head<2>();
            const double local_x = cos_yaw * delta.x()
                + sin_yaw * delta.y();
            const double local_y = -sin_yaw * delta.x()
                + cos_yaw * delta.y();

            const bool center_inside_vehicle =
                local_x >= rear && local_x <= front
                && std::abs(local_y) <= half_width;
            if (center_inside_vehicle
                && isOccupied(Eigen::Vector2i(x, y))) {
                return false;
            }
        }
    }
    return true;
}

double HybridAstar::steeringIndexToAngle(const int steering_index) const {
    return static_cast<double>(steering_index)
        * max_steering_angle_
        / static_cast<double>(steering_discrete_num_);
}

int HybridAstar::calculatePrimitiveSampleCount(const double steering) const {
    const double front = vehicle_length_ - rear_overhang_;
    const double longitudinal_radius = std::max(front, rear_overhang_);
    const double corner_radius = std::hypot(longitudinal_radius,
                                            0.5 * vehicle_width_);
    const double curvature = std::tan(steering) / wheel_base_;

    // 转弯时外侧车角的运动距离大于后轴中心弧长。
    const double maximum_corner_travel = primitive_length_
        * (1.0 + corner_radius * std::abs(curvature));
    return std::max(
        1,
        static_cast<int>(
            std::ceil(maximum_corner_travel / primitive_sample_step_)));
}

bool HybridAstar::generatePrimitive(
    const HybridNode& current_node,
    const Direction direction,
    const int steering_index,
    HybridStateVector& intermediate_states) const {
    intermediate_states.clear();

    if ((direction != Direction::FORWARD
         && direction != Direction::BACKWARD)
        || steering_index < -steering_discrete_num_
        || steering_index > steering_discrete_num_
        || !current_node.state.allFinite()) {
        return false;
    }

    const double steering = steeringIndexToAngle(steering_index);
    const double curvature = std::tan(steering) / wheel_base_;
    const int sample_count = calculatePrimitiveSampleCount(steering);
    const double direction_sign =
        direction == Direction::FORWARD ? 1.0 : -1.0;
    const double signed_step = direction_sign
        * primitive_length_ / static_cast<double>(sample_count);

    intermediate_states.reserve(static_cast<std::size_t>(sample_count));
    Eigen::Vector3d state = current_node.state;
    for (int sample = 0; sample < sample_count; ++sample) {
        const double yaw = state.z();
        if (std::abs(curvature) < 1.0e-9) {
            state.x() += signed_step * std::cos(yaw);
            state.y() += signed_step * std::sin(yaw);
        } else {
            const double next_yaw = yaw + signed_step * curvature;
            state.x() += (std::sin(next_yaw) - std::sin(yaw)) / curvature;
            state.y() += (-std::cos(next_yaw) + std::cos(yaw)) / curvature;
            state.z() = next_yaw;
        }
        state.z() = normalizeAngle(state.z());
        intermediate_states.push_back(state);
    }
    return true;
}

bool HybridAstar::isPrimitiveCollisionFree(
    const HybridStateVector& intermediate_states) const {
    if (intermediate_states.empty()) {
        return false;
    }

    for (std::size_t i = 0; i + 1 < intermediate_states.size(); ++i) {
        if (!isVehicleBoundaryCollisionFree(intermediate_states[i])) {
            return false;
        }
    }

    return isVehicleFootprintCollisionFree(intermediate_states.back());
}

double HybridAstar::calculateReedsSheppHeuristic(
    const Eigen::Vector3d& state,
    const Eigen::Vector3d& goal_state) const {
    if (!reeds_shepp_state_space_ || !state.allFinite()
        || !goal_state.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }

    ompl::base::ScopedState<> from(reeds_shepp_state_space_);
    ompl::base::ScopedState<> to(reeds_shepp_state_space_);
    from[0] = state.x();
    from[1] = state.y();
    from[2] = normalizeAngle(state.z());
    to[0] = goal_state.x();
    to[1] = goal_state.y();
    to[2] = normalizeAngle(goal_state.z());
    return reeds_shepp_state_space_->distance(from(), to());
}

double HybridAstar::calculateHeuristic(
    const Eigen::Vector3d& state,
    const Eigen::Vector3d& goal_state) const {
    if (!state.allFinite() || !goal_state.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }

    const double euclidean_distance =
        (goal_state.head<2>() - state.head<2>()).norm();
    const double kinematic_heuristic =
        euclidean_distance > reeds_shepp_switch_distance_
            ? euclidean_distance
            : calculateReedsSheppHeuristic(state, goal_state);
    const double obstacle_heuristic = calculateObstacleHeuristic(state);
    const double combined_heuristic = std::isfinite(obstacle_heuristic)
        ? std::max(kinematic_heuristic, obstacle_heuristic)
        : kinematic_heuristic;
    return heuristic_weight_ * combined_heuristic;
}

bool HybridAstar::buildObstacleHeuristic(
    const Eigen::Vector2i& goal_index) {
    obstacle_heuristic_cost_.clear();
    obstacle_heuristic_goal_index_.setZero();
    obstacle_heuristic_ready_ = false;
    if (!isInsideMap(goal_index) || isOccupied(goal_index)) {
        return false;
    }

    const std::size_t cell_count =
        static_cast<std::size_t>(map_size_.x())
        * static_cast<std::size_t>(map_size_.y());
    obstacle_heuristic_cost_.assign(
        cell_count, std::numeric_limits<double>::infinity());

    typedef std::pair<double, std::size_t> QueueEntry;
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
        queue;
    const auto to_linear_index = [this](const int x, const int y) {
        return static_cast<std::size_t>(y)
            * static_cast<std::size_t>(map_size_.x())
            + static_cast<std::size_t>(x);
    };

    const std::size_t goal_linear_index =
        to_linear_index(goal_index.x(), goal_index.y());
    obstacle_heuristic_cost_[goal_linear_index] = 0.0;
    queue.emplace(0.0, goal_linear_index);

    const int neighbor_x[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int neighbor_y[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    constexpr double kSqrtTwo = 1.4142135623730950488;

    while (!queue.empty()) {
        const QueueEntry current = queue.top();
        queue.pop();
        const double current_cost = current.first;
        const std::size_t current_linear_index = current.second;
        if (current_cost
            != obstacle_heuristic_cost_[current_linear_index]) {
            continue;
        }

        const int current_x = static_cast<int>(
            current_linear_index
            % static_cast<std::size_t>(map_size_.x()));
        const int current_y = static_cast<int>(
            current_linear_index
            / static_cast<std::size_t>(map_size_.x()));
        for (int neighbor = 0; neighbor < 8; ++neighbor) {
            const int next_x = current_x + neighbor_x[neighbor];
            const int next_y = current_y + neighbor_y[neighbor];
            const Eigen::Vector2i next_index(next_x, next_y);
            if (isOccupied(next_index)) {
                continue;
            }

            const bool diagonal = neighbor_x[neighbor] != 0
                && neighbor_y[neighbor] != 0;
            if (diagonal
                && (isOccupied(Eigen::Vector2i(next_x, current_y))
                    || isOccupied(Eigen::Vector2i(current_x, next_y)))) {
                continue;
            }

            const double step_cost = map_resolution_
                * (diagonal ? kSqrtTwo : 1.0);
            const double next_cost = current_cost + step_cost;
            const std::size_t next_linear_index =
                to_linear_index(next_x, next_y);
            if (next_cost
                >= obstacle_heuristic_cost_[next_linear_index]) {
                continue;
            }
            obstacle_heuristic_cost_[next_linear_index] = next_cost;
            queue.emplace(next_cost, next_linear_index);
        }
    }

    obstacle_heuristic_goal_index_ = goal_index;
    obstacle_heuristic_ready_ = true;
    return true;
}

double HybridAstar::calculateObstacleHeuristic(
    const Eigen::Vector3d& state) const {
    if (!obstacle_heuristic_ready_) {
        return std::numeric_limits<double>::infinity();
    }

    Eigen::Vector2i grid_index;
    if (!positionToIndex(state.head<2>(), grid_index)) {
        return std::numeric_limits<double>::infinity();
    }
    const std::size_t linear_index =
        static_cast<std::size_t>(grid_index.y())
            * static_cast<std::size_t>(map_size_.x())
        + static_cast<std::size_t>(grid_index.x());
    return obstacle_heuristic_cost_[linear_index];
}

bool HybridAstar::tryReedsSheppConnection(
    const Eigen::Vector3d& start_state,
    const Eigen::Vector3d& goal_state,
    HybridStateVector& connection_states) const {
    connection_states.clear();
    if (!reeds_shepp_state_space_ || !start_state.allFinite()
        || !goal_state.allFinite() || minimum_turning_radius_ <= 0.0) {
        return false;
    }

    ompl::base::ScopedState<> from(reeds_shepp_state_space_);
    ompl::base::ScopedState<> to(reeds_shepp_state_space_);
    ompl::base::ScopedState<> sample(reeds_shepp_state_space_);
    from[0] = start_state.x();
    from[1] = start_state.y();
    from[2] = normalizeAngle(start_state.z());
    to[0] = goal_state.x();
    to[1] = goal_state.y();
    to[2] = normalizeAngle(goal_state.z());

    const double reeds_shepp_length =
        reeds_shepp_state_space_->distance(from(), to());
    if (!std::isfinite(reeds_shepp_length)) {
        return false;
    }

    const double front = vehicle_length_ - rear_overhang_;
    const double longitudinal_radius = std::max(front, rear_overhang_);
    const double corner_radius = std::hypot(longitudinal_radius,
                                            0.5 * vehicle_width_);
    const double maximum_corner_travel = reeds_shepp_length
        * (1.0 + corner_radius / minimum_turning_radius_);
    const int sample_count = std::max(
        1,
        static_cast<int>(
            std::ceil(maximum_corner_travel / primitive_sample_step_)));

    connection_states.reserve(static_cast<std::size_t>(sample_count));
    for (int i = 1; i <= sample_count; ++i) {
        const double ratio =
            static_cast<double>(i) / static_cast<double>(sample_count);
        reeds_shepp_state_space_->interpolate(from(), to(), ratio, sample());
        const std::vector<double> values = sample.reals();
        connection_states.emplace_back(
            values[0], values[1], normalizeAngle(values[2]));
    }

    // 使用调用方给出的精确目标，避免插值末端的浮点误差。
    connection_states.back() = goal_state;
    connection_states.back().z() = normalizeAngle(connection_states.back().z());
    if (!isPrimitiveCollisionFree(connection_states)) {
        connection_states.clear();
        return false;
    }
    return true;
}

void HybridAstar::appendAnalyticPath(
    const HybridStateVector& connection_states) {
    for (const Eigen::Vector3d& state : connection_states) {
        if (!final_path_.empty()) {
            const Eigen::Vector3d& previous = final_path_.back();
            const bool same_position =
                (previous.head<2>() - state.head<2>()).squaredNorm()
                <= 1.0e-18;
            const bool same_yaw =
                std::abs(normalizeAngle(previous.z() - state.z()))
                <= 1.0e-9;
            if (same_position && same_yaw) {
                continue;
            }
        }
        final_path_.push_back(state);
    }
}

int HybridAstar::search(const Eigen::Vector3d& start_state,
                        const Eigen::Vector3d& goal_state) {
    const ros::WallTime search_start_time = ros::WallTime::now();
    SearchTiming timing;
    int iterations = 0;
    const auto report_timing = [&](const char* result) {
        const double total_ms = elapsedMilliseconds(search_start_time);
        const double measured_ms = timing.initialization_ms
            + timing.obstacle_heuristic_ms
            + timing.queue_ms + timing.analytic_ms
            + timing.primitive_generation_ms + timing.collision_check_ms
            + timing.state_index_ms + timing.transition_cost_ms
            + timing.heuristic_ms + timing.node_update_ms
            + timing.path_retrieval_ms;
        ROS_INFO("[hybridastar_timing] result=%s total_ms=%.3f "
                 "iterations=%d nodes=%zu queue_pop=%zu stale=%zu "
                 "primitives=%zu collision_checks=%zu collision_rejects=%zu "
                 "analytic_attempts=%zu heuristic_calls=%zu",
                 result, total_ms, iterations, timing.created_node_count,
                 timing.queue_pop_count, timing.stale_entry_count,
                 timing.primitive_count, timing.collision_check_count,
                 timing.collision_reject_count,
                 timing.analytic_attempt_count, timing.heuristic_count);
        ROS_INFO("[hybridastar_timing] init_ms=%.3f dijkstra_ms=%.3f "
                 "queue_ms=%.3f "
                 "primitive_ms=%.3f collision_ms=%.3f index_ms=%.3f "
                 "transition_ms=%.3f heuristic_ms=%.3f analytic_ms=%.3f "
                 "node_update_ms=%.3f path_ms=%.3f other_ms=%.3f",
                 timing.initialization_ms, timing.obstacle_heuristic_ms,
                 timing.queue_ms,
                 timing.primitive_generation_ms, timing.collision_check_ms,
                 timing.state_index_ms, timing.transition_cost_ms,
                 timing.heuristic_ms, timing.analytic_ms,
                 timing.node_update_ms, timing.path_retrieval_ms,
                 std::max(0.0, total_ms - measured_ms));
        publishSearchTree();
    };

    const ros::WallTime initialization_start = ros::WallTime::now();
    reset();
    if (!map_initialized_ || !start_state.allFinite()
        || !goal_state.allFinite() || !reeds_shepp_state_space_) {
        timing.initialization_ms += elapsedMilliseconds(initialization_start);
        ROS_ERROR("HybridAstar search prerequisites are not ready");
        report_timing("prerequisite_failed");
        return NO_PATH;
    }

    Eigen::Vector3d normalized_start = start_state;
    Eigen::Vector3d normalized_goal = goal_state;
    normalized_start.z() = normalizeAngle(normalized_start.z());
    normalized_goal.z() = normalizeAngle(normalized_goal.z());

    Eigen::Vector3i start_index;
    Eigen::Vector3i goal_index;
    if (!stateToIndex(normalized_start, start_index)
        || !stateToIndex(normalized_goal, goal_index)) {
        timing.initialization_ms += elapsedMilliseconds(initialization_start);
        ROS_WARN("HybridAstar start or goal is outside the raw map");
        report_timing("outside_map");
        return NO_PATH;
    }
    if (!isVehicleFootprintCollisionFree(normalized_start)) {
        timing.initialization_ms += elapsedMilliseconds(initialization_start);
        ROS_WARN("HybridAstar start footprint is in collision");
        report_timing("start_collision");
        return NO_PATH;
    }
    if (!isVehicleFootprintCollisionFree(normalized_goal)) {
        timing.initialization_ms += elapsedMilliseconds(initialization_start);
        ROS_WARN("HybridAstar goal footprint is in collision");
        report_timing("goal_collision");
        return NO_PATH;
    }

    timing.initialization_ms += elapsedMilliseconds(initialization_start);
    const ros::WallTime obstacle_heuristic_start = ros::WallTime::now();
    if (!buildObstacleHeuristic(goal_index.head<2>())) {
        ROS_WARN("HybridAstar failed to build the reverse Dijkstra "
                 "heuristic; falling back to the kinematic heuristic");
    }
    timing.obstacle_heuristic_ms +=
        elapsedMilliseconds(obstacle_heuristic_start);

    const ros::WallTime start_node_initialization = ros::WallTime::now();
    HybridNodePtr start_node = createNode();
    start_node->state = normalized_start;
    start_node->grid_index = start_index;
    start_node->direction = Direction::NONE;
    start_node->steering_index = 0;
    start_node->steering = 0.0;
    start_node->g_score = 0.0;
    start_node->f_score = calculateHeuristic(normalized_start,
                                             normalized_goal);
    start_node->status = NodeStatus::IN_OPEN_SET;

    const HybridNodeKey start_key(start_index, Direction::NONE);
    expanded_nodes_[start_key] = start_node;
    open_set_.emplace(start_node->f_score,
                      start_node->g_score,
                      start_node);
    timing.created_node_count = 1;
    timing.heuristic_count = 1;
    timing.initialization_ms +=
        elapsedMilliseconds(start_node_initialization);

    double best_goal_distance = std::numeric_limits<double>::infinity();

    while (!open_set_.empty()) {
        if (iterations >= max_iterations_
            || (ros::WallTime::now() - search_start_time).toSec()
                >= max_search_time_) {
            ROS_WARN("HybridAstar reached search limit: iterations=%d, "
                     "time=%.3fs",
                     iterations,
                     (ros::WallTime::now() - search_start_time).toSec());
            report_timing("search_limit");
            return NO_PATH;
        }

        const ros::WallTime queue_start = ros::WallTime::now();
        const OpenSetEntry current_entry = open_set_.top();
        open_set_.pop();
        ++timing.queue_pop_count;
        HybridNodePtr current_node = current_entry.node;
        if (current_node == nullptr) {
            ++timing.stale_entry_count;
            timing.queue_ms += elapsedMilliseconds(queue_start);
            continue;
        }

        const HybridNodeKey current_key(current_node->grid_index,
                                        current_node->direction);
        const auto current_iter = expanded_nodes_.find(current_key);
        if (current_iter == expanded_nodes_.end()
            || current_iter->second != current_node
            || current_entry.g_score != current_node->g_score
            || current_entry.f_score != current_node->f_score
            || current_node->status == NodeStatus::IN_CLOSE_SET) {
            ++timing.stale_entry_count;
            timing.queue_ms += elapsedMilliseconds(queue_start);
            continue;
        }

        current_node->status = NodeStatus::IN_CLOSE_SET;
        visited_states_.push_back(current_node->state);
        ++iterations;
        timing.queue_ms += elapsedMilliseconds(queue_start);

        const double goal_distance =
            (current_node->state.head<2>()
             - normalized_goal.head<2>()).norm();
        const bool is_closer_to_goal =
            goal_distance + 1.0e-9 < best_goal_distance;
        const bool may_try_analytic =
            goal_distance <= analytic_expansion_distance_
            || isGoalReached(current_node->state, normalized_goal);
        if (may_try_analytic && is_closer_to_goal) {
            best_goal_distance = goal_distance;
            HybridStateVector connection_states;
            const ros::WallTime analytic_start = ros::WallTime::now();
            ++timing.analytic_attempt_count;
            if (tryReedsSheppConnection(current_node->state,
                                        normalized_goal,
                                        connection_states)) {
                timing.analytic_ms += elapsedMilliseconds(analytic_start);
                const ros::WallTime path_start = ros::WallTime::now();
                retrievePath(current_node);
                appendAnalyticPath(connection_states);
                timing.path_retrieval_ms += elapsedMilliseconds(path_start);
                ROS_INFO("HybridAstar reached goal: iterations=%d, "
                         "time=%.3fs, path_states=%zu",
                         iterations,
                         (ros::WallTime::now() - search_start_time).toSec(),
                         final_path_.size());
                report_timing("success");
                return REACH_END;
            }
            timing.analytic_ms += elapsedMilliseconds(analytic_start);
        }

        const Direction directions[2] = {
            Direction::FORWARD,
            Direction::BACKWARD
        };
        for (const Direction direction : directions) {
            for (int steering_index = -steering_discrete_num_;
                 steering_index <= steering_discrete_num_;
                 ++steering_index) {
                HybridStateVector intermediate_states;
                ++timing.primitive_count;
                const ros::WallTime primitive_start = ros::WallTime::now();
                const bool primitive_generated = generatePrimitive(
                    *current_node, direction, steering_index,
                    intermediate_states);
                timing.primitive_generation_ms +=
                    elapsedMilliseconds(primitive_start);
                if (!primitive_generated) {
                    continue;
                }

                ++timing.collision_check_count;
                const ros::WallTime collision_start = ros::WallTime::now();
                const bool collision_free =
                    isPrimitiveCollisionFree(intermediate_states);
                timing.collision_check_ms +=
                    elapsedMilliseconds(collision_start);
                if (!collision_free) {
                    ++timing.collision_reject_count;
                    continue;
                }

                const Eigen::Vector3d& next_state =
                    intermediate_states.back();
                Eigen::Vector3i next_index;
                const ros::WallTime index_start = ros::WallTime::now();
                if (!stateToIndex(next_state, next_index)) {
                    timing.state_index_ms += elapsedMilliseconds(index_start);
                    continue;
                }
                timing.state_index_ms += elapsedMilliseconds(index_start);

                const double steering =
                    steeringIndexToAngle(steering_index);
                const ros::WallTime transition_start = ros::WallTime::now();
                const double transition_cost = calculateTransitionCost(
                    *current_node, direction, steering);
                const double next_g_score =
                    current_node->g_score + transition_cost;
                timing.transition_cost_ms +=
                    elapsedMilliseconds(transition_start);
                if (!std::isfinite(next_g_score)) {
                    continue;
                }

                const ros::WallTime node_lookup_start = ros::WallTime::now();
                const HybridNodeKey next_key(next_index, direction);
                const auto existing_iter = expanded_nodes_.find(next_key);
                if (existing_iter != expanded_nodes_.end()
                    && existing_iter->second->g_score
                        <= next_g_score + 1.0e-9) {
                    timing.node_update_ms +=
                        elapsedMilliseconds(node_lookup_start);
                    continue;
                }
                timing.node_update_ms +=
                    elapsedMilliseconds(node_lookup_start);

                const ros::WallTime heuristic_start = ros::WallTime::now();
                ++timing.heuristic_count;
                const double heuristic = calculateHeuristic(next_state,
                                                             normalized_goal);
                timing.heuristic_ms += elapsedMilliseconds(heuristic_start);
                if (!std::isfinite(heuristic)) {
                    continue;
                }

                const ros::WallTime node_update_start = ros::WallTime::now();
                HybridNodePtr next_node = createNode();
                next_node->state = next_state;
                next_node->grid_index = next_index;
                next_node->direction = direction;
                next_node->steering_index = steering_index;
                next_node->steering = steering;
                next_node->g_score = next_g_score;
                next_node->f_score = next_g_score + heuristic;
                next_node->status = NodeStatus::IN_OPEN_SET;
                next_node->parent = current_node;
                next_node->intermediate_states =
                    std::move(intermediate_states);

                expanded_nodes_[next_key] = next_node;
                open_set_.emplace(next_node->f_score,
                                  next_node->g_score,
                                  next_node);
                ++timing.created_node_count;
                timing.node_update_ms +=
                    elapsedMilliseconds(node_update_start);
            }
        }
    }

    ROS_WARN("HybridAstar open set is empty: iterations=%d, time=%.3fs",
             iterations,
             (ros::WallTime::now() - search_start_time).toSec());
    report_timing("open_set_empty");
    return NO_PATH;
}

const HybridStateVector& HybridAstar::getPath() const {
    return final_path_;
}

const HybridStateVector& HybridAstar::getVisitedStates() const {
    return visited_states_;
}

double HybridAstar::calculateTransitionCost(
    const HybridNode& current_node,
    const Direction next_direction,
    const double next_steering) const {
    if ((next_direction != Direction::FORWARD
         && next_direction != Direction::BACKWARD)
        || !std::isfinite(next_steering)
        || std::abs(next_steering) > max_steering_angle_ + 1.0e-9) {
        return std::numeric_limits<double>::infinity();
    }

    const double direction_penalty =
        next_direction == Direction::FORWARD
            ? forward_penalty_
            : reverse_penalty_;
    double transition_cost = primitive_length_ * direction_penalty;

    if (current_node.direction != Direction::NONE
        && current_node.direction != next_direction) {
        transition_cost += gear_switch_penalty_;
    }

    transition_cost += steering_penalty_
        * std::abs(next_steering) / max_steering_angle_;
    if (current_node.direction != Direction::NONE) {
        transition_cost += steering_change_penalty_
            * std::abs(next_steering - current_node.steering)
            / max_steering_angle_;
    }
    return transition_cost;
}

bool HybridAstar::isGoalReached(
    const Eigen::Vector3d& state,
    const Eigen::Vector3d& goal_state) const {
    if (!state.allFinite() || !goal_state.allFinite()) {
        return false;
    }

    const double position_error =
        (state.head<2>() - goal_state.head<2>()).norm();
    const double yaw_error =
        std::abs(normalizeAngle(state.z() - goal_state.z()));
    return position_error <= goal_position_tolerance_
        && yaw_error <= goal_yaw_tolerance_;
}

HybridNodePtr HybridAstar::createNode() {
    node_pool_.emplace_back(new HybridNode);
    return node_pool_.back().get();
}

void HybridAstar::retrievePath(HybridNodePtr goal_node) {
    final_path_.clear();
    if (goal_node == nullptr) {
        return;
    }

    std::vector<HybridNodePtr> node_chain;
    for (HybridNodePtr node = goal_node;
         node != nullptr;
         node = node->parent) {
        node_chain.push_back(node);
    }
    std::reverse(node_chain.begin(), node_chain.end());

    const auto append_state = [this](const Eigen::Vector3d& state) {
        if (!final_path_.empty()) {
            const Eigen::Vector3d& previous = final_path_.back();
            const bool same_position =
                (previous.head<2>() - state.head<2>()).squaredNorm()
                <= 1.0e-18;
            const bool same_yaw =
                std::abs(normalizeAngle(previous.z() - state.z()))
                <= 1.0e-9;
            if (same_position && same_yaw) {
                return;
            }
        }
        final_path_.push_back(state);
    };

    append_state(node_chain.front()->state);
    for (std::size_t i = 1; i < node_chain.size(); ++i) {
        const HybridNodePtr node = node_chain[i];
        if (node->intermediate_states.empty()) {
            append_state(node->state);
            continue;
        }
        for (const Eigen::Vector3d& state : node->intermediate_states) {
            append_state(state);
        }
    }
}

}  // namespace path_searching
