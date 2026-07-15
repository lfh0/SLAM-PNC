#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>             //转换函数头文件
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
    ros::Subscriber localMapSub_;
    ros::Subscriber odomSub_;
    ros::Subscriber pathSub_;

    ros::Publisher minco_traj_pub_;
    ros::Publisher Rectangle_poly_pub_;


    ros::Publisher missionPub_;
    ros::Publisher globalTrajPub_;
    ros::Publisher waypointshowPub_;
    ros::Publisher midpointshowPub_;

    ros::ServiceClient path_send_client_;

    nav_msgs::OccupancyGrid globalMap_;
    nav_msgs::OccupancyGrid localMap_;
    RobotState robot_state_;

    std::vector<MissionPoint> missionPoints_;
    std::map<int, MissionPoint> missionMaps_;
    std::string yamlFileName;
    // 任务点间0.05m插值后的路径
    nav_msgs::Path path_nodes;

    std::vector<Eigen::MatrixXd> hPolys_;
    plan_manage::PolyTrajOptimizer::Ptr ploy_traj_opt_;
    plan_utils::TrajContainer traj_container_;

    int targetId_;
    double MAX_VEL;
    int corridor_collision_threshold_;
    bool corridor_unknown_as_occupied_;
    double corridor_max_longitudinal_;
    double corridor_max_lateral_;
    double corridor_rdp_epsilon_;
    double corridor_min_seed_length_;
    double corridor_max_seed_length_;
    double corridor_max_seed_deviation_;
    double corridor_max_seed_yaw_error_;
    double corridor_min_overlap_clearance_;

private:
    void globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void localMapCallBack(const nav_msgs::OccupancyGrid::Ptr &msg);
    void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
    void pathCallBack(const nav_msgs::Path::ConstPtr &msg);
    
    bool RuninPath(const nav_msgs::Path& global_path);
    bool generateSafeCorridor(const nav_msgs::Path& path,
                              std::vector<Eigen::Vector3d>& key_points);
    bool checkCollisionUsingLine(const Eigen::Vector2d& start_pt, const Eigen::Vector2d& end_pt) const;


public:
    TrajoptServer(ros::NodeHandle nh, ros::NodeHandle nhPrivate);
    bool RunMINCOParking(double duration);
    void displayMincoTraj(plan_utils::SingulTrajData display_traj);
    void displayPolyH(const std::vector<Eigen::MatrixXd> hPolys);
    void displayPoint(const std::vector<Eigen::Vector3d>& points);
    // void globalSearch(std::vector<int> routes);
    ~TrajoptServer();
};

TrajoptServer::TrajoptServer(ros::NodeHandle nh, ros::NodeHandle nhPrivate)
{
    nh_.param("trajopt/max_vel", MAX_VEL, 3.0);
    nh_.param("corridor/collision_cost_threshold", corridor_collision_threshold_, 80);
    nh_.param("corridor/unknown_as_occupied", corridor_unknown_as_occupied_, true);
    nh_.param("corridor/max_longitudinal", corridor_max_longitudinal_, 0.2);
    nh_.param("corridor/max_lateral", corridor_max_lateral_, 0.6);
    nh_.param("corridor/rdp_epsilon", corridor_rdp_epsilon_, 0.08);
    nh_.param("corridor/min_seed_length", corridor_min_seed_length_, 0.25);
    nh_.param("corridor/max_seed_length", corridor_max_seed_length_, 0.8);
    nh_.param("corridor/max_seed_deviation", corridor_max_seed_deviation_, 0.1);
    nh_.param("corridor/max_seed_yaw_error", corridor_max_seed_yaw_error_, 0.2);
    nh_.param("corridor/min_overlap_clearance", corridor_min_overlap_clearance_, 0.05);
    
    
    std::string global_map_topic;
    std::string local_map_topic;
    std::string odom_topic;
    std::string path_topic;
    nh_.param<std::string>("topics/global_map", global_map_topic, "/global_costmap_node/costmap/costmap");
    nh_.param<std::string>("topics/local_map", local_map_topic, "/local_map");
    nh_.param<std::string>("topics/odom", odom_topic, "/lio/odom");
    nh_.param<std::string>("topics/path", path_topic, "/front2end_search_path");
    
    globalMapSub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(global_map_topic, 10, &TrajoptServer::globalMapCallBack, this);
    localMapSub_ = nh_.subscribe(local_map_topic, 10, &TrajoptServer::localMapCallBack, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 10, &TrajoptServer::odomCallBack, this);
    pathSub_ = nh_.subscribe<nav_msgs::Path>(path_topic, 10, &TrajoptServer::pathCallBack, this);
    
    path_send_client_ = nh_.serviceClient<trajopt::SendPath>("/trajopt/server/global_traj_path");
    minco_traj_pub_ = nh_.advertise<nav_msgs::Path>("/trajopt/minco_traj", 2);
    Rectangle_poly_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("/trajopt/polyhedrons", 1, true);
    midpointshowPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajopt/corridor_points", 1, true);

    ploy_traj_opt_.reset(new plan_manage::PolyTrajOptimizer);
    ploy_traj_opt_->init(nh);
}

TrajoptServer::~TrajoptServer(){}

void TrajoptServer::globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    globalMap_ = *msg;
}

void TrajoptServer::localMapCallBack(const nav_msgs::OccupancyGrid::Ptr &msg){
    localMap_ = *msg;
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
    nav_msgs::Path globalpath = *msg;
    double total_distance = 0.0;
    for (size_t i = 1; i < globalpath.poses.size(); ++i) {
        total_distance += GetDistanceFromPathpoint(globalpath.poses[i - 1],
                                                   globalpath.poses[i]);
    }
    if (total_distance <= 1.0e-6 || MAX_VEL <= 1.0e-6) {
        ROS_WARN("全局路径长度或最大速度无效，跳过轨迹优化");
        return;
    }
    // 将外部全局路径加密后送入现有的 MINCO 优化流程。
    if (!RuninPath(globalpath)) {
        ROS_WARN("全局路径重采样失败，跳过轨迹优化");
        return;
    }
    
    const double initial_duration = total_distance / MAX_VEL;
    if (!RunMINCOParking(initial_duration)) {
        return;
    }

    // displayPolyH(display_hPolys_);
    // displayMincoTraj(traj_container_.singul_traj);
}

bool TrajoptServer::RuninPath(const nav_msgs::Path& global_path){
    constexpr double sample_distance = 0.05;
    constexpr double epsilon = 1.0e-9;
    if (global_path.poses.size() < 2) {
        return false;
    }
    path_nodes.header = global_path.header;
    path_nodes.poses.clear();

    // 计算原始折线路径的累计弧长，重复点对应零长度路径段。
    std::vector<double> arc_lengths(global_path.poses.size(), 0.0);
    for (size_t i = 1; i < global_path.poses.size(); ++i) {
        const auto& previous = global_path.poses[i - 1].pose.position;
        const auto& current = global_path.poses[i].pose.position;
        arc_lengths[i] = arc_lengths[i - 1]
            + std::hypot(current.x - previous.x, current.y - previous.y);
    }

    const double total_length = arc_lengths.back();
    if (total_length <= epsilon) {
        return false;
    }
    // 从起点开始，每隔严格的 0.05m 沿整条折线采样一次。
    size_t segment_index = 1;
    for (double sample_s = 0.0; sample_s < total_length; sample_s += sample_distance) {
        while (segment_index < arc_lengths.size()
               && arc_lengths[segment_index] + epsilon < sample_s) {
            ++segment_index;
        }
        while (segment_index < arc_lengths.size()
               && arc_lengths[segment_index] - arc_lengths[segment_index - 1] <= epsilon) {
            ++segment_index;
        }
        if (segment_index >= arc_lengths.size()) {
            break;
        }

        const size_t previous_index = segment_index - 1;
        const double segment_length = arc_lengths[segment_index] - arc_lengths[previous_index];
        const double ratio = std::max(0.0, std::min(
            1.0, (sample_s - arc_lengths[previous_index]) / segment_length));
        const auto& start = global_path.poses[previous_index];
        const auto& end = global_path.poses[segment_index];

        geometry_msgs::PoseStamped sample = start;
        sample.header = global_path.header;
        sample.pose.position.x = start.pose.position.x
            + ratio * (end.pose.position.x - start.pose.position.x);
        sample.pose.position.y = start.pose.position.y
            + ratio * (end.pose.position.y - start.pose.position.y);
        sample.pose.position.z = start.pose.position.z
            + ratio * (end.pose.position.z - start.pose.position.z);
        path_nodes.poses.push_back(sample);
    }

    // 保留原始终点；最后一小段允许短于 0.05m。
    const auto& original_end = global_path.poses.back();
    const auto& sampled_end = path_nodes.poses.back().pose.position;
    if (std::hypot(original_end.pose.position.x - sampled_end.x,
                   original_end.pose.position.y - sampled_end.y) > epsilon) {
        path_nodes.poses.push_back(original_end);
    } else {
        path_nodes.poses.back() = original_end;
    }

    // 中间点 yaw 使用局部轨迹切线，起点和终点保留前端给定的 yaw。
    for (size_t i = 1; i + 1 < path_nodes.poses.size(); ++i) {
        const size_t previous_index = i - 1;
        const size_t next_index = std::min(i + 1, path_nodes.poses.size() - 1);
        const auto& previous = path_nodes.poses[previous_index].pose.position;
        const auto& next = path_nodes.poses[next_index].pose.position;
        const double yaw = std::atan2(next.y - previous.y, next.x - previous.x);
        path_nodes.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    // 首尾位姿是优化边界条件，严格使用输入路径给定的值。
    path_nodes.poses.front().pose.position = global_path.poses.front().pose.position;
    path_nodes.poses.front().pose.orientation = global_path.poses.front().pose.orientation;
    path_nodes.poses.back().pose.position = global_path.poses.back().pose.position;
    path_nodes.poses.back().pose.orientation = global_path.poses.back().pose.orientation;
    return true;
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

void TrajoptServer::displayMincoTraj(plan_utils::SingulTrajData display_traj)
{
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose;
    Eigen::Vector2d pt(0, 0);
    Eigen::Vector2d pt_before(0, 0);
    for (unsigned int i = 0; i < display_traj.size(); ++i)
    {
        double total_duration = display_traj.at(i).duration;
        for (double t = 0; t <= total_duration; t += 0.01)
        {
            pt_before = pt;
            pt = display_traj.at(i).traj.getPos(t);
            pose.pose.position.x = pt(0);
            pose.pose.position.y = pt(1);
            pose.pose.position.z = 0.2;

            double yaw = atan2(pt(1)- pt_before(1), pt(0) - pt_before(0));
            tf::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            path_msg.poses.push_back(pose);
        }
    }
    path_msg.header.frame_id = "map";
    minco_traj_pub_.publish(path_msg);

    trajopt::SendPath sendpath;
    sendpath.request.path = path_msg;
    sendpath.request.path.header.frame_id = "map";
    if(!path_send_client_.call(sendpath)){
        path_send_client_.call(sendpath);
    }
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
    const auto arc_length = [&](size_t begin, size_t end) {
        double length = 0.0;
        for (size_t i = begin + 1; i <= end; ++i) {
            length += (path_point(i) - path_point(i - 1)).norm();
        }
        return length;
    };
    const auto arc_midpoint = [&](size_t begin, size_t end) {
        const double target = 0.5 * arc_length(begin, end);
        double length = 0.0;
        for (size_t i = begin + 1; i < end; ++i) {
            length += (path_point(i) - path_point(i - 1)).norm();
            if (length >= target) {
                return i;
            }
        }
        return begin + (end - begin) / 2;
    };

    // RDP只根据几何偏差保留显著拐点，先去除A*路径的栅格锯齿。
    std::vector<size_t> rdp_indices{0};
    std::function<void(size_t, size_t)> rdp_simplify;
    rdp_simplify = [&](size_t begin, size_t end) {
        const auto deviation = deviation_info(begin, end);
        if (deviation.first > corridor_rdp_epsilon_
            && deviation.second > begin && deviation.second < end) {
            rdp_simplify(begin, deviation.second);
            rdp_simplify(deviation.second, end);
        } else {
            rdp_indices.push_back(end);
        }
    };
    rdp_simplify(0, path.poses.size() - 1);

    // 对简化后的每条线段继续施加长度、碰撞和原路径偏差约束。
    std::vector<size_t> refined_indices{rdp_indices.front()};
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
    for (size_t i = 0; i + 1 < rdp_indices.size(); ++i) {
        if (!refine_segment(rdp_indices[i], rdp_indices[i + 1])) {
            return false;
        }
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

    // 公共关键点必须位于相邻两个凸区域内部，并保留给定的数值余量。
    for (size_t i = 0; i + 1 < polyhedra.size(); ++i) {
        const Eigen::Vector2d common_point = path_point(key_indices[i + 1]);
        for (const auto* polyhedron : {&polyhedra[i], &polyhedra[i + 1]}) {
            for (const auto& plane : polyhedron->hyperplanes()) {
                const double normal_length = plane.n_.norm();
                if (plane.n_.dot(common_point - plane.p_) / normal_length
                    > -corridor_min_overlap_clearance_) {
                    ROS_ERROR("相邻凸走廊 %zu 和 %zu 的重叠余量不足", i, i + 1);
                    hPolys_.clear();
                    return false;
                }
            }
        }
    }

    key_points.reserve(key_indices.size());
    for (size_t i = 0; i < key_indices.size(); ++i) {
        const size_t previous = i == 0 ? key_indices[i] : key_indices[i - 1];
        const size_t next = i + 1 < key_indices.size() ? key_indices[i + 1] : key_indices[i];
        const Eigen::Vector2d point = path_point(key_indices[i]);
        const Eigen::Vector2d tangent = path_point(next) - path_point(previous);
        key_points.emplace_back(point.x(), point.y(),
                                std::atan2(tangent.y(), tangent.x()));
    }
    return true;
}

bool TrajoptServer::RunMINCOParking(double duration)
{
    (void)duration;
    std::vector<Eigen::Vector3d> key_points;
    if (!generateSafeCorridor(path_nodes, key_points)) {
        ROS_ERROR("安全走廊生成失败，停止本次轨迹优化");
        return false;
    }

    displayPoint(key_points);
    displayPolyH(hPolys_);
    ROS_INFO("安全走廊生成成功：%zu 个关键点，%zu 个凸区域",
             key_points.size(), hPolys_.size());
    return true;
}



int main(int argc, char** argv){
    std::cout << " Trajopt Server is running ... " << std::endl << std::endl;
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "trajopt_server");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    TrajoptServer trajoptServer(nh, nhPrivate);
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    ros::Rate rate(10);
}
