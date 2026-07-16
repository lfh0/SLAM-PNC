#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajopt/SendPath.h>
#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense> 
#include <Eigen/Core>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <trajopt/Trajectory.h>
#include <algorithm>
#include <array>
#include <functional>

#include "plan_manage/traj_optimizer.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"
#include "controller/state.h"

#include "Utility.h"
#include "smoother.hpp"


class TrajoptServer
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber globalMapSub_;
    ros::Subscriber odomSub_;
    ros::Subscriber pathSub_;

    ros::Publisher minco_traj_pub_;
    ros::Publisher debug_traj_pub_;
    ros::Publisher path_endpoints_pub_;
    ros::Publisher Rectangle_poly_pub_;


    ros::Publisher midpointshowPub_;

    ros::ServiceClient path_send_client_;

    nav_msgs::OccupancyGrid globalMap_;
    RobotState robot_state_;

    // 前端搜索直接输出的路径
    nav_msgs::Path path_nodes;

    std::vector<Eigen::MatrixXd> hPolys_;
    plan_manage::PolyTrajOptimizer::Ptr ploy_traj_opt_;
    plan_utils::TrajContainer traj_container_;

    int corridor_collision_threshold_;
    bool corridor_unknown_as_occupied_;
    double corridor_max_longitudinal_;
    double corridor_max_lateral_;
    double corridor_min_seed_length_;
    double corridor_max_seed_length_;
    double corridor_max_seed_deviation_;
    double corridor_max_seed_yaw_error_;
    double corridor_min_overlap_area_;
    double initial_max_vel_;
    double initial_max_acc_;
    double initial_min_speed_;
    double initial_min_piece_time_;
    double initial_min_turn_speed_ratio_;
    double validation_max_curvature_;
    double validation_sample_period_;
    double validation_corridor_tolerance_;
    double validation_car_length_;
    double validation_car_width_;
    double validation_car_d_cr_;

private:
    void globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
    void pathCallBack(const nav_msgs::Path::ConstPtr &msg);

    bool generateSafeCorridor(const nav_msgs::Path& path,
                              std::vector<Eigen::Vector3d>& key_points);
    bool checkCollisionUsingLine(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt) const;
    bool validateTrajectory(const plan_utils::Trajectory& trajectory) const;


public:
    explicit TrajoptServer(ros::NodeHandle nh);
    bool RunMINCOParking();
    void displayMincoTraj(plan_utils::SingulTrajData display_traj,
                          bool publish_to_controller = true);
    void displayPathEndpoints(const nav_msgs::Path& path);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayPoint(const std::vector<Eigen::Vector3d>& points);
    // void globalSearch(std::vector<int> routes);
    ~TrajoptServer();
};

TrajoptServer::TrajoptServer(ros::NodeHandle nh)
{
    nh_.param("corridor/collision_cost_threshold", corridor_collision_threshold_, 80);
    nh_.param("corridor/unknown_as_occupied", corridor_unknown_as_occupied_, true);
    nh_.param("corridor/max_longitudinal", corridor_max_longitudinal_, 0.2);
    nh_.param("corridor/max_lateral", corridor_max_lateral_, 0.6);
    nh_.param("corridor/min_seed_length", corridor_min_seed_length_, 0.25);
    nh_.param("corridor/max_seed_length", corridor_max_seed_length_, 0.8);
    nh_.param("corridor/max_seed_deviation", corridor_max_seed_deviation_, 0.1);
    nh_.param("corridor/max_seed_yaw_error", corridor_max_seed_yaw_error_, 0.2);
    nh_.param("corridor/min_overlap_area", corridor_min_overlap_area_, 0.01);
    nh_.param("optimizing/max_vel", initial_max_vel_, 3.0);
    nh_.param("optimizing/max_acc", initial_max_acc_, 1.5);
    nh_.param("initial_time/min_speed", initial_min_speed_, 0.2);
    nh_.param("initial_time/min_piece_time", initial_min_piece_time_, 0.2);
    nh_.param("initial_time/min_turn_speed_ratio", initial_min_turn_speed_ratio_, 0.25);
    nh_.param("optimizing/max_cur", validation_max_curvature_, 0.523598);
    nh_.param("validation/sample_period", validation_sample_period_, 0.02);
    nh_.param("validation/corridor_tolerance", validation_corridor_tolerance_, 1.0e-4);
    nh_.param("vehicle/car_length", validation_car_length_, 0.6);
    nh_.param("vehicle/car_width", validation_car_width_, 0.6);
    nh_.param("vehicle/car_d_cr", validation_car_d_cr_, 0.0);
    double validation_half_margin = 0.25;
    nh_.param("optimizing/half_margin", validation_half_margin, 0.25);
    validation_car_length_ += 2.0 * validation_half_margin;
    validation_car_width_ += 2.0 * validation_half_margin;
    
    
    std::string global_map_topic;
    std::string odom_topic;
    std::string path_topic;
    std::string debug_path_topic;
    std::string path_endpoints_topic;
    nh_.param<std::string>("topics/global_map", global_map_topic, "/global_costmap_node/costmap/costmap");
    nh_.param<std::string>("topics/odom", odom_topic, "/lio/odom");
    nh_.param<std::string>("topics/path", path_topic, "/front2end_search_path");
    nh_.param<std::string>("topics/debug_path", debug_path_topic, "/trajopt/debug_path");
    nh_.param<std::string>("topics/path_endpoints", path_endpoints_topic,
                           "/trajopt/path_endpoints");
    
    globalMapSub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(global_map_topic, 10, &TrajoptServer::globalMapCallBack, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 10, &TrajoptServer::odomCallBack, this);
    pathSub_ = nh_.subscribe<nav_msgs::Path>(path_topic, 10, &TrajoptServer::pathCallBack, this);
    
    path_send_client_ = nh_.serviceClient<trajopt::SendPath>("/trajopt/server/global_traj_path");
    minco_traj_pub_ = nh_.advertise<nav_msgs::Path>("/trajopt/minco_traj", 2);
    debug_traj_pub_ = nh_.advertise<nav_msgs::Path>(debug_path_topic, 1, true);
    path_endpoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        path_endpoints_topic, 1, true);
    Rectangle_poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("/trajopt/polyhedrons", 1, true);
    midpointshowPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajopt/corridor_points", 1, true);

    ploy_traj_opt_.reset(new plan_manage::PolyTrajOptimizer);
    ploy_traj_opt_->init(nh);
}

TrajoptServer::~TrajoptServer(){}

void TrajoptServer::globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    globalMap_ = *msg;
}

void TrajoptServer::odomCallBack(const nav_msgs::OdometryConstPtr &msg){
    robot_state_.x = msg->pose.pose.position.x;
    robot_state_.y = msg->pose.pose.position.y;
    robot_state_.yaw = tf::getYaw(msg->pose.pose.orientation);
    robot_state_.linear_velocity = msg->twist.twist.linear.x;
    robot_state_.angular_velocity = msg->twist.twist.angular.z;
}

void TrajoptServer::pathCallBack(const nav_msgs::Path::ConstPtr &msg){
    if (msg->poses.size() < 2) {
        ROS_WARN("收到的全局路径点数不足，跳过轨迹优化");
        return;
    }
    path_nodes = *msg;
    if (!RunMINCOParking()) {
        return;
    }
}

void TrajoptServer::displayPoint(const std::vector<Eigen::Vector3d>& points)
{
    visualization_msgs::MarkerArray waypoints_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    waypoints_array.markers.push_back(clear_marker);

    const ros::Time stamp = ros::Time::now();
    for (size_t i = 0; i < points.size(); ++i) {
        visualization_msgs::Marker waypoint;
        waypoint.header.stamp = stamp;
        waypoint.header.frame_id = "map";
        waypoint.ns = "corridor_points";
        waypoint.id = static_cast<int>(i);
        waypoint.type = visualization_msgs::Marker::SPHERE;
        waypoint.action = visualization_msgs::Marker::ADD;
        waypoint.color.r = 0.0;
        waypoint.color.g = 1.0;
        waypoint.color.b = 1.0;
        waypoint.color.a = 1.0;
        waypoint.scale.x = 0.1;
        waypoint.scale.y = 0.1;
        waypoint.scale.z = 0.1;
        waypoint.pose.position.x = points[i][0];
        waypoint.pose.position.y = points[i][1];
        waypoint.pose.position.z = 0.05;
        waypoint.pose.orientation.w = 1.0;
        waypoints_array.markers.push_back(waypoint);
    }
    midpointshowPub_.publish(waypoints_array);
}

void TrajoptServer::displayPolyH(const std::vector<Eigen::MatrixXd> hPolys)
{
    vec_E<Polyhedron2D> polyhedra;
    polyhedra.reserve(hPolys.size());
    for (const auto &ele : hPolys)
    {
      Polyhedron2D hPoly;
      for (int i = 0; i < ele.cols(); i++)
      {
        hPoly.add(Hyperplane2D(ele.col(i).tail<2>(), ele.col(i).head<2>()));
      }
      polyhedra.push_back(hPoly);
    }

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();
    Rectangle_poly_pub_.publish(poly_msg);
}

void TrajoptServer::displayMincoTraj(plan_utils::SingulTrajData display_traj,
                                     bool publish_to_controller)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    constexpr double sample_period = 0.01;
    constexpr double velocity_epsilon = 1.0e-6;

    for (unsigned int i = 0; i < display_traj.size(); ++i)
    {
        const auto& trajectory = display_traj.at(i).traj;
        const double total_duration = display_traj.at(i).duration;
        const int direction = trajectory.getDirection();
        const size_t sample_count = static_cast<size_t>(
            std::ceil(total_duration / sample_period));

        for (size_t sample_index = 0; sample_index <= sample_count; ++sample_index)
        {
            const double t = std::min(
                total_duration, sample_index * sample_period);
            const Eigen::Vector2d pt = trajectory.getPos(t);
            Eigen::Vector2d tangent = direction * trajectory.getdSigma(t);

            // 起终点速度为零时，用邻近轨迹位置计算真实切线，避免使用坐标原点计算 yaw。
            if (tangent.squaredNorm() <= velocity_epsilon * velocity_epsilon) {
                const double previous_t = std::max(0.0, t - sample_period);
                const double next_t = std::min(total_duration, t + sample_period);
                tangent = direction * (
                    trajectory.getPos(next_t) - trajectory.getPos(previous_t));
            }

            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
            pose.pose.position.z = 0.2;
            const double yaw = std::atan2(tangent.y(), tangent.x());
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            path_msg.poses.push_back(pose);
        }
    }
    // 候选轨迹生成后先发布首尾位姿，便于观察验收失败轨迹的边界状态。
    displayPathEndpoints(path_msg);
    if (!publish_to_controller) {
        debug_traj_pub_.publish(path_msg);
        return;
    }
    minco_traj_pub_.publish(path_msg);

    trajopt::SendPath sendpath;
    sendpath.request.path = path_msg;
    sendpath.request.path.header.frame_id = "map";
    if(!path_send_client_.call(sendpath)){
        path_send_client_.call(sendpath);
    }
}

void TrajoptServer::displayPathEndpoints(const nav_msgs::Path& path)
{
    if (path.poses.empty()) {
        return;
    }

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    const std::array<const geometry_msgs::PoseStamped*, 2> endpoint_poses{
        &path.poses.front(), &path.poses.back()};
    for (size_t i = 0; i < endpoint_poses.size(); ++i) {
        visualization_msgs::Marker arrow;
        arrow.header = path.header;
        arrow.ns = "minco_path_endpoints";
        arrow.id = static_cast<int>(i);
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose = endpoint_poses[i]->pose;
        arrow.pose.position.z += 0.1;
        arrow.scale.x = 0.6;
        arrow.scale.y = 0.12;
        arrow.scale.z = 0.12;
        arrow.color.a = 1.0;
        if (i == 0) {
            arrow.color.g = 1.0;
        } else {
            arrow.color.r = 1.0;
        }
        markers.markers.push_back(arrow);
    }
    path_endpoints_pub_.publish(markers);
}

bool TrajoptServer::checkCollisionUsingLine(const Eigen::Vector2d& start_pt,
                                            const Eigen::Vector2d& end_pt) const
{
    const auto& info = globalMap_.info;
    if (info.resolution <= 0.0 || info.width == 0 || info.height == 0 || globalMap_.data.empty()) {
        return true;
    }

    const double origin_yaw = tf::getYaw(info.origin.orientation);
    const double cos_yaw = std::cos(origin_yaw);
    const double sin_yaw = std::sin(origin_yaw);
    const double sample_step = info.resolution * 0.5;
    const double length = (end_pt - start_pt).norm();
    const int sample_count = std::max(1, static_cast<int>(std::ceil(length / sample_step)));

    for (int i = 0; i <= sample_count; ++i) {
        const double ratio = static_cast<double>(i) / sample_count;
        const Eigen::Vector2d point = start_pt + ratio * (end_pt - start_pt);
        const double dx = point.x() - info.origin.position.x;
        const double dy = point.y() - info.origin.position.y;
        const double map_x = cos_yaw * dx + sin_yaw * dy;
        const double map_y = -sin_yaw * dx + cos_yaw * dy;
        const int mx = static_cast<int>(std::floor(map_x / info.resolution));
        const int my = static_cast<int>(std::floor(map_y / info.resolution));

        if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width)
            || my >= static_cast<int>(info.height)) {
            return true;
        }

        const int8_t cost = globalMap_.data[my * info.width + mx];
        if ((cost < 0 && corridor_unknown_as_occupied_)
            || cost >= corridor_collision_threshold_) {
            return true;
        }
    }
    return false;
}

bool TrajoptServer::validateTrajectory(const plan_utils::Trajectory& trajectory) const
{
    if (trajectory.getPieceNum() != static_cast<int>(hPolys_.size())
        || validation_sample_period_ <= 0.0) {
        ROS_ERROR("轨迹段数与安全走廊数量不一致，无法验收");
        return false;
    }

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_curvature = 0.0;
    for (int piece_index = 0; piece_index < trajectory.getPieceNum(); ++piece_index) {
        const auto& piece = trajectory[piece_index];
        const double duration = piece.getDuration();
        const size_t sample_count = static_cast<size_t>(
            std::ceil(duration / validation_sample_period_));

        for (size_t sample_index = 0; sample_index <= sample_count; ++sample_index) {
            const double t = std::min(
                duration, sample_index * validation_sample_period_);
            const Eigen::Vector2d position = piece.getPos(t);
            const Eigen::Vector2d velocity = piece.getdSigma(t);
            const Eigen::Vector2d acceleration = piece.getddSigma(t);
            if (!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite()) {
                ROS_ERROR("轨迹第 %d 段出现非有限数值", piece_index);
                return false;
            }

            const double speed = velocity.norm();
            const double tangential_acceleration = speed > 1.0e-6
                ? std::abs(acceleration.dot(velocity) / speed) : 0.0;
            const double curvature = speed > 1.0e-4
                ? std::abs(velocity.x() * acceleration.y()
                           - velocity.y() * acceleration.x())
                    / (speed * speed * speed)
                : 0.0;
            max_velocity = std::max(max_velocity, speed);
            max_acceleration = std::max(max_acceleration, tangential_acceleration);
            max_curvature = std::max(max_curvature, curvature);
            if (speed > initial_max_vel_ + 1.0e-3
                || tangential_acceleration > initial_max_acc_ + 1.0e-3
                || curvature > validation_max_curvature_ + 1.0e-3) {
                ROS_ERROR("轨迹动力学验收失败：段=%d, t=%.3f, v=%.3f, a=%.3f, k=%.3f",
                          piece_index, t, speed, tangential_acceleration, curvature);
                return false;
            }

            Eigen::Vector2d tangent = velocity;
            if (tangent.squaredNorm() <= 1.0e-12) {
                const double previous_t = std::max(0.0, t - validation_sample_period_);
                const double next_t = std::min(duration, t + validation_sample_period_);
                tangent = piece.getPos(next_t) - piece.getPos(previous_t);
            }
            if (tangent.squaredNorm() <= 1.0e-12) {
                ROS_ERROR("轨迹第 %d 段在 t=%.3f 无法确定车身方向", piece_index, t);
                return false;
            }
            tangent.normalize();
            tangent *= trajectory.getDirection();
            const Eigen::Vector2d normal(-tangent.y(), tangent.x());
            const std::array<Eigen::Vector2d, 4> corners{
                position + (validation_car_d_cr_ + 0.5 * validation_car_length_) * tangent
                         + 0.5 * validation_car_width_ * normal,
                position + (validation_car_d_cr_ + 0.5 * validation_car_length_) * tangent
                         - 0.5 * validation_car_width_ * normal,
                position + (validation_car_d_cr_ - 0.5 * validation_car_length_) * tangent
                         - 0.5 * validation_car_width_ * normal,
                position + (validation_car_d_cr_ - 0.5 * validation_car_length_) * tangent
                         + 0.5 * validation_car_width_ * normal};

            const Eigen::MatrixXd& corridor = hPolys_[piece_index];
            for (const auto& corner : corners) {
                for (int plane_index = 0; plane_index < corridor.cols(); ++plane_index) {
                    const Eigen::Vector2d normal_vector = corridor.col(plane_index).head<2>();
                    const Eigen::Vector2d plane_point = corridor.col(plane_index).tail<2>();
                    const double signed_distance = normal_vector.dot(corner - plane_point)
                        / normal_vector.norm();
                    if (signed_distance > validation_corridor_tolerance_) {
                        ROS_ERROR("轨迹走廊验收失败：段=%d, t=%.3f, 越界=%.4fm",
                                  piece_index, t, signed_distance);
                        return false;
                    }
                }
            }
        }
    }

    ROS_INFO("轨迹验收通过：max_v=%.3f, max_a=%.3f, max_k=%.3f",
             max_velocity, max_acceleration, max_curvature);
    return true;
}

bool TrajoptServer::generateSafeCorridor(const nav_msgs::Path& path,
                                         std::vector<Eigen::Vector3d>& key_points)
{
    hPolys_.clear();
    key_points.clear();
    if (path.poses.size() < 2 || globalMap_.info.resolution <= 0.0
        || globalMap_.data.empty()) {
        ROS_WARN("尚未收到安全走廊使用的膨胀地图");
        return false;
    }

    const auto path_point = [&](size_t index) {
        return Eigen::Vector2d(path.poses[index].pose.position.x,
                               path.poses[index].pose.position.y);
    };

    // 累计弧长只计算一次，后续区间长度和弧长中点均可直接查询。
    std::vector<double> cumulative_length(path.poses.size(), 0.0);
    for (size_t i = 1; i < path.poses.size(); ++i) {
        const double segment_length = (path_point(i) - path_point(i - 1)).norm();
        if (segment_length <= 1.0e-9) {
            ROS_ERROR("前端路径包含连续重复点：%zu 和 %zu", i - 1, i);
            return false;
        }
        cumulative_length[i] = cumulative_length[i - 1] + segment_length;
    }

    const auto arc_length = [&](size_t begin, size_t end) {
        return cumulative_length[end] - cumulative_length[begin];
    };
    const auto arc_midpoint = [&](size_t begin, size_t end) {
        const double target = 0.5 * (cumulative_length[begin] + cumulative_length[end]);
        const auto first = cumulative_length.begin() + begin + 1;
        const auto last = cumulative_length.begin() + end;
        const auto midpoint = std::lower_bound(first, last, target);
        return midpoint == last
            ? end - 1
            : static_cast<size_t>(std::distance(cumulative_length.begin(), midpoint));
    };

    const auto deviation_info = [&](size_t begin, size_t end) {
        const Eigen::Vector2d start = path_point(begin);
        const Eigen::Vector2d finish = path_point(end);
        const Eigen::Vector2d segment = finish - start;
        const double squared_length = segment.squaredNorm();
        double deviation = 0.0;
        size_t deviation_index = begin;
        if (squared_length <= 1.0e-12) {
            return std::make_pair(deviation, deviation_index);
        }
        for (size_t i = begin + 1; i < end; ++i) {
            const double ratio = std::max(0.0, std::min(
                1.0, (path_point(i) - start).dot(segment) / squared_length));
            const double current_deviation =
                (path_point(i) - (start + ratio * segment)).norm();
            if (current_deviation > deviation) {
                deviation = current_deviation;
                deviation_index = i;
            }
        }
        return std::make_pair(deviation, deviation_index);
    };
    // 一次递归同时完成路径简化、最大段长限制和弦线碰撞检查。
    std::vector<size_t> refined_indices{0};
    std::function<bool(size_t, size_t)> refine_segment;
    refine_segment = [&](size_t begin, size_t end) {
        const bool collision = checkCollisionUsingLine(path_point(begin), path_point(end));
        const auto deviation = deviation_info(begin, end);
        const bool needs_split = collision
            || arc_length(begin, end) > corridor_max_seed_length_
            || deviation.first > corridor_max_seed_deviation_;

        if (!needs_split) {
            refined_indices.push_back(end);
            return true;
        }
        if (end <= begin + 1) {
            ROS_ERROR("相邻路径点之间发生碰撞，无法构造凸走廊");
            return false;
        }

        size_t split = deviation.second;
        if (split <= begin || split >= end) {
            split = arc_midpoint(begin, end);
        }
        return refine_segment(begin, split) && refine_segment(split, end);
    };
    if (!refine_segment(0, path.poses.size() - 1)) {
        return false;
    }

    // 安全合并过短段；急弯、碰撞或偏差超限时保留原关键点。
    bool merged = true;
    while (merged && refined_indices.size() > 2) {
        merged = false;
        for (size_t i = 1; i + 1 < refined_indices.size(); ++i) {
            const size_t previous = refined_indices[i - 1];
            const size_t current = refined_indices[i];
            const size_t next = refined_indices[i + 1];
            const double previous_length = (path_point(current) - path_point(previous)).norm();
            const double next_length = (path_point(next) - path_point(current)).norm();
            if (std::min(previous_length, next_length) >= corridor_min_seed_length_) {
                continue;
            }

            const Eigen::Vector2d incoming = (path_point(current) - path_point(previous)).normalized();
            const Eigen::Vector2d outgoing = (path_point(next) - path_point(current)).normalized();
            const double turn_angle = std::acos(std::max(-1.0, std::min(1.0,
                incoming.dot(outgoing))));
            const bool can_merge = turn_angle <= corridor_max_seed_yaw_error_
                && arc_length(previous, next) <= corridor_max_seed_length_
                && deviation_info(previous, next).first <= corridor_max_seed_deviation_
                && !checkCollisionUsingLine(path_point(previous), path_point(next));
            if (can_merge) {
                refined_indices.erase(refined_indices.begin() + i);
                merged = true;
                break;
            }
        }
    }

    const std::vector<size_t>& key_indices = refined_indices;

    // 将局部栅格中心转换为障碍物点，再以每条路径段作为凸分解种子。
    const auto& info = globalMap_.info;
    const double origin_yaw = tf::getYaw(info.origin.orientation);
    const double origin_cos = std::cos(origin_yaw);
    const double origin_sin = std::sin(origin_yaw);
    const auto world_to_map_continuous = [&](const Eigen::Vector2d& point) {
        const double dx = point.x() - info.origin.position.x;
        const double dy = point.y() - info.origin.position.y;
        return Eigen::Vector2d(
            (origin_cos * dx + origin_sin * dy) / info.resolution,
            (-origin_sin * dx + origin_cos * dy) / info.resolution);
    };
    const auto map_to_world = [&](double mx, double my) {
        const double local_x = mx * info.resolution;
        const double local_y = my * info.resolution;
        return Eigen::Vector2d(
            info.origin.position.x + origin_cos * local_x - origin_sin * local_y,
            info.origin.position.y + origin_sin * local_x + origin_cos * local_y);
    };

    vec_E<Polyhedron2D> polyhedra;
    polyhedra.reserve(key_indices.size() - 1);
    for (size_t segment_index = 0; segment_index + 1 < key_indices.size(); ++segment_index) {
        const Eigen::Vector2d start = path_point(key_indices[segment_index]);
        const Eigen::Vector2d finish = path_point(key_indices[segment_index + 1]);
        const Eigen::Vector2d direction = (finish - start).normalized();
        const Eigen::Vector2d normal(-direction.y(), direction.x());

        std::array<Eigen::Vector2d, 4> corners{
            start - corridor_max_longitudinal_ * direction + corridor_max_lateral_ * normal,
            start - corridor_max_longitudinal_ * direction - corridor_max_lateral_ * normal,
            finish + corridor_max_longitudinal_ * direction + corridor_max_lateral_ * normal,
            finish + corridor_max_longitudinal_ * direction - corridor_max_lateral_ * normal};
        Eigen::Vector2d map_min = world_to_map_continuous(corners[0]);
        Eigen::Vector2d map_max = map_min;
        for (const auto& corner : corners) {
            const Eigen::Vector2d map_corner = world_to_map_continuous(corner);
            map_min = map_min.cwiseMin(map_corner);
            map_max = map_max.cwiseMax(map_corner);
        }

        const int min_mx = std::max(0, static_cast<int>(std::floor(map_min.x())) - 1);
        const int max_mx = std::min(static_cast<int>(info.width) - 1,
                                    static_cast<int>(std::ceil(map_max.x())) + 1);
        const int min_my = std::max(0, static_cast<int>(std::floor(map_min.y())) - 1);
        const int max_my = std::min(static_cast<int>(info.height) - 1,
                                    static_cast<int>(std::ceil(map_max.y())) + 1);

        vec_Vec2f obstacles;
        for (int my = min_my; my <= max_my; ++my) {
            for (int mx = min_mx; mx <= max_mx; ++mx) {
                const int8_t cost = globalMap_.data[my * info.width + mx];
                if ((cost < 0 && corridor_unknown_as_occupied_)
                    || cost >= corridor_collision_threshold_) {
                    obstacles.push_back(map_to_world(mx + 0.5, my + 0.5));
                }
            }
        }

        LineSegment2D decomposition(start, finish);
        decomposition.set_local_bbox(
            Vec2f(corridor_max_longitudinal_, corridor_max_lateral_));
        decomposition.set_obs(obstacles);
        decomposition.dilate(0.0);
        Polyhedron2D polyhedron = decomposition.get_polyhedron();
        const Eigen::Vector2d map_axis_x(origin_cos, origin_sin);
        const Eigen::Vector2d map_axis_y(-origin_sin, origin_cos);
        const Eigen::Vector2d map_origin(info.origin.position.x, info.origin.position.y);
        polyhedron.add(Hyperplane2D(
            map_origin + map_axis_x * info.width * info.resolution, map_axis_x));
        polyhedron.add(Hyperplane2D(map_origin, -map_axis_x));
        polyhedron.add(Hyperplane2D(
            map_origin + map_axis_y * info.height * info.resolution, map_axis_y));
        polyhedron.add(Hyperplane2D(map_origin, -map_axis_y));
        const auto planes = polyhedron.hyperplanes();
        if (planes.empty()) {
            ROS_ERROR("路径段 %zu 未能生成有效凸走廊", segment_index);
            return false;
        }

        Eigen::MatrixXd hPoly(4, planes.size());
        for (size_t i = 0; i < planes.size(); ++i) {
            hPoly.col(i).head<2>() = planes[i].n_;
            hPoly.col(i).tail<2>() = planes[i].p_;
        }
        hPolys_.push_back(hPoly);
        polyhedra.push_back(polyhedron);
    }

    // 合并相邻凸区域的半空间，直接计算真实交集面积。
    for (size_t i = 0; i + 1 < polyhedra.size(); ++i) {
        Polyhedron2D intersection;
        for (const auto& plane : polyhedra[i].hyperplanes()) {
            intersection.add(plane);
        }
        for (const auto& plane : polyhedra[i + 1].hyperplanes()) {
            intersection.add(plane);
        }

        const vec_Vec2f vertices = cal_vertices(intersection);
        double overlap_area = 0.0;
        if (vertices.size() >= 3) {
            for (size_t j = 0; j < vertices.size(); ++j) {
                const Vec2f& current = vertices[j];
                const Vec2f& next = vertices[(j + 1) % vertices.size()];
                overlap_area += current.x() * next.y() - current.y() * next.x();
            }
            overlap_area = 0.5 * std::abs(overlap_area);
        }

        if (overlap_area < corridor_min_overlap_area_) {
            ROS_ERROR("相邻凸走廊 %zu 和 %zu 的交集面积不足：%.6f < %.6f m^2",
                      i, i + 1, overlap_area, corridor_min_overlap_area_);
            hPolys_.clear();
            return false;
        }
    }

    key_points.reserve(key_indices.size());
    for (size_t i = 0; i < key_indices.size(); ++i) {
        const size_t index = key_indices[i];
        const Eigen::Vector2d point = path_point(index);
        key_points.emplace_back(
            point.x(), point.y(), tf::getYaw(path.poses[index].pose.orientation));
    }
    return true;
}

bool TrajoptServer::RunMINCOParking()
{
    std::vector<Eigen::Vector3d> key_points;
    if (!generateSafeCorridor(path_nodes, key_points)) {
        ROS_ERROR("安全走廊生成失败，停止本次轨迹优化");
        return false;
    }
    displayPoint(key_points);
    displayPolyH(hPolys_);
    ROS_INFO("安全走廊生成成功：%zu 个关键点，%zu 个凸区域",
             key_points.size(), hPolys_.size());

    // 选择多段多项式的端点作为关键点：
    // 1. 起点和终点严格使用前端路径的首尾位姿，作为整条轨迹的固定边界。
    // 2. 每个凸走廊对应一段多项式，因此多项式段数应等于 hPolys_.size()，
    //    端点数量应等于 hPolys_.size() + 1。
    // 3. 第 i 个内部端点连接第 i-1 段和第 i 段，其初值使用 key_points[i]；
    //    该点必须同时位于 hPolys_[i-1] 和 hPolys_[i] 中，即位于相邻走廊交集内。
    // 4. 内部端点不需要固定在前端原路径上。优化时允许它在相邻走廊交集内移动，
    //    这样 MINCO 才能主动减小折线转角并生成平滑轨迹。
    // 5. 不再按固定点数或固定距离额外选点；关键点数量由前面的长度、偏差、
    //    碰撞及短段合并条件自适应确定，避免直线段点过多、转弯处约束不足。

    const size_t piece_num = hPolys_.size();
    if (key_points.size() != piece_num + 1 || piece_num < 2) {
        ROS_ERROR("MINCO 输入数量不匹配或多项式段数不足");
        return false;
    }

    // 起点使用车辆当前速度，终点默认停车；加速度初值均设为零。
    Eigen::MatrixXd initial_state = Eigen::MatrixXd::Zero(2, 3);
    Eigen::MatrixXd final_state = Eigen::MatrixXd::Zero(2, 3);
    initial_state.col(0) = key_points.front().head<2>();
    final_state.col(0) = key_points.back().head<2>();
    const double start_speed = std::min(
        initial_max_vel_, std::max(0.0, robot_state_.linear_velocity));
    initial_state.col(1) << start_speed * std::cos(key_points.front().z()),
                            start_speed * std::sin(key_points.front().z());

    Eigen::MatrixXd inner_points(2, piece_num - 1);
    for (size_t i = 1; i + 1 < key_points.size(); ++i) {
        inner_points.col(i - 1) = key_points[i].head<2>();
    }

    // 转角越大，连接点速度越低；首点使用当前速度，末点速度为零。
    Eigen::VectorXd node_speeds = Eigen::VectorXd::Constant(
        key_points.size(), initial_max_vel_);
    node_speeds[0] = start_speed;
    node_speeds[node_speeds.size() - 1] = 0.0;
    for (size_t i = 1; i + 1 < key_points.size(); ++i) {
        const Eigen::Vector2d incoming =
            key_points[i].head<2>() - key_points[i - 1].head<2>();
        const Eigen::Vector2d outgoing =
            key_points[i + 1].head<2>() - key_points[i].head<2>();
        const double cosine = std::max(-1.0, std::min(
            1.0, incoming.normalized().dot(outgoing.normalized())));
        const double turn_angle = std::acos(cosine);
        const double speed_ratio = std::max(
            initial_min_turn_speed_ratio_, 1.0 - turn_angle / M_PI);
        node_speeds[i] = initial_max_vel_ * speed_ratio;
    }

    Eigen::VectorXd piece_times(piece_num);
    for (size_t i = 0; i < piece_num; ++i) {
        const double length = (key_points[i + 1].head<2>()
                               - key_points[i].head<2>()).norm();
        const double average_speed = std::max(
            initial_min_speed_, 0.5 * (node_speeds[i] + node_speeds[i + 1]));
        const double velocity_time = length / average_speed;
        const double acceleration_time = std::abs(
            node_speeds[i + 1] - node_speeds[i]) / initial_max_acc_;
        piece_times[i] = std::max(
            initial_min_piece_time_, std::max(velocity_time, acceleration_time));
    }
    const double total_time = piece_times.sum();
    const Eigen::VectorXd piece_time_ratios = piece_times / total_time;

    std::vector<Eigen::MatrixXd> initial_states{initial_state};
    std::vector<Eigen::MatrixXd> final_states{final_state};
    std::vector<Eigen::MatrixXd> initial_inner_points{inner_points};
    Eigen::VectorXd initial_times(1);
    initial_times[0] = total_time;
    std::vector<Eigen::VectorXd> time_ratios{piece_time_ratios};
    std::vector<int> singuls{1};

    // 优化器按每个多项式采样点读取走廊，因此将“一段一个走廊”展开到各采样点。
    std::vector<Eigen::MatrixXd> sampled_corridors;
    for (size_t i = 0; i < piece_num; ++i) {
        const int resolution = (i == 0 || i + 1 == piece_num)
            ? ploy_traj_opt_->get_destraj_resolution_()
            : ploy_traj_opt_->get_traj_resolution_();
        for (int j = 0; j <= resolution; ++j) {
            sampled_corridors.push_back(hPolys_[i]);
        }
    }
    std::vector<std::vector<Eigen::MatrixXd>> corridor_container{
        sampled_corridors};

    if (!ploy_traj_opt_->OptimizeTrajectory(
            initial_states, final_states, initial_inner_points, initial_times,
            corridor_container, time_ratios, singuls)) {
        ROS_ERROR("MINCO 轨迹优化失败");
        return false;
    }

    const auto* min_jerk_optimizers = ploy_traj_opt_->getMinJerkOptPtr();
    if (min_jerk_optimizers->empty()) {
        ROS_ERROR("MINCO 未返回有效轨迹");
        return false;
    }
    const plan_utils::Trajectory optimized_trajectory =
        min_jerk_optimizers->front().getTraj(1);
    traj_container_.clearSingul();
    traj_container_.addSingulTraj(
        optimized_trajectory, ros::Time::now().toSec(), 0);
    // 先发布候选轨迹及其首尾位姿，再执行发布前验收。
    displayMincoTraj(traj_container_.singul_traj, false);

    if (!validateTrajectory(optimized_trajectory)) {
        ROS_ERROR("MINCO 轨迹未通过发布前验收，本次轨迹不会发送给控制器");
        return false;
    }
    nav_msgs::Path empty_debug_path;
    empty_debug_path.header.frame_id = "map";
    empty_debug_path.header.stamp = ros::Time::now();
    debug_traj_pub_.publish(empty_debug_path);
    displayMincoTraj(traj_container_.singul_traj);
    return true;
}



int main(int argc, char** argv){
    std::cout << " Trajopt Server is running ... " << std::endl << std::endl;
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "trajopt_server");
    ros::NodeHandle nh;
    TrajoptServer trajoptServer(nh);
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    ros::Rate rate(10);
}
