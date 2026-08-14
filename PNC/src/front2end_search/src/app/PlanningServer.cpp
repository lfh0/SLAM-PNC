#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "path_searching/obs_hybridastar.h"
#include "path_searching/astar.h"
#include "path_searching/jps.h"
#include "path_searching/rrt.h"

class Color : public std_msgs::ColorRGBA {
public:
    Color() : std_msgs::ColorRGBA() {}

    Color(int hex_color) {
        int _r = (hex_color >> 16) & 0xFF;
        int _g = (hex_color >> 8) & 0xFF;
        int _b = hex_color & 0xFF;
        r = static_cast<double>(_r) / 255.0;
        g = static_cast<double>(_g) / 255.0;
        b = static_cast<double>(_b) / 255.0;
    }

    Color(Color c, double alpha) {
        r = c.r;
        g = c.g;
        b = c.b;
        a = alpha;
    }

    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
    }

    Color(double red, double green, double blue, double alpha) : Color() {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }

    static const Color Black() { return Color(0.0, 0.0, 0.0); }

    static const Color Gray() { return Color(0.5, 0.5, 0.5); }

    static const Color Red() { return Color(1.0, 0.0, 0.0); }

    static const Color Green() { return Color(0.0, 0.96, 0.0); }

    static const Color Blue() { return Color(0.0, 0.0, 1.0); }

    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }

    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }

    static Color Orange() { return Color(1.0, 0.5, 0.0); }

    static const Color Purple() { return Color(0.5, 0.0, 1.0); }

    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }

    static const Color Teal() { return Color(0.0, 1.0, 1.0); }

    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};


class PlanningServer
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber globalMapSub_;
    ros::Subscriber goalSub_;
    ros::Subscriber odomSub_;

    ros::Publisher obsHybridAstarPathPub_;
    ros::Publisher AstarPathPub_;
    ros::Publisher JPSPathPub_;
    ros::Publisher RRTPathPub_;
    ros::Publisher pathEndpointsPub_;

    ros::Publisher mkr_pub;


    nav_msgs::OccupancyGrid globalMap_;
    nav_msgs::Odometry carOdom_;
    std::mutex odom_mutex_;
    bool has_odom_ = false;
    bool search_maps_dirty_ = false;
    // geometry_msgs::Pose goalPose_;

    geometry_msgs::Quaternion goal_orientation_;

    std::vector<Eigen::Vector2d> search_path_;
    
    std::unique_ptr<path_searching::ObsHybridAstar> obs_hybridastar_path_finder_;
    std::unique_ptr<path_searching::Astar> astar_path_finder_;
    std::unique_ptr<path_searching::JPS> jps_path_finder_;
    std::unique_ptr<path_searching::RRT> rrt_path_finder_;

    double resolution_;
    double start_time_;
    std::vector<double> astar_search_window_retry_margins_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string goal_topic_;
    double path_yaw_lookahead_distance_ = 0.3;
    size_t start_yaw_sample_offset_ = 4;
    int search_type_;
    ros::WallTime active_goal_wall_start_;
    uint64_t planning_seq_ = 0;

private:
    void map_init();
    void syncSearchMaps();
    void globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
    
    void publishPointWithText(const Eigen::Vector2d& p, const std::string& text, const Color c);
    void runSelectedSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void updatePathOrientations(nav_msgs::Path& path_msg);
    void displayPathEndpoints(const nav_msgs::Path& path_msg, const std::string& ns);
    static double elapsedMs(const ros::WallTime& start_time);
    double elapsedFromActiveGoalMs() const;

public:
    explicit PlanningServer(ros::NodeHandle nh);
    bool PlanbyObsHybridAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    bool PlanbyAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    bool PlanbyJPSSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    bool PlanbyRRTSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    void displayObsHybridAstarPath(const std::vector<Eigen::Vector2d>& final_path);
    void displayAstarPath(const std::vector<Eigen::Vector2d>& final_path);
    void displayJPSPath(const std::vector<Eigen::Vector2d>& final_path);
    void displayRRTPath(const std::vector<Eigen::Vector2d>& final_path);

    ~PlanningServer();
};

PlanningServer::PlanningServer(ros::NodeHandle nh):
nh_(nh)
{
    // launch 中配置在 <node> 下的话题参数属于节点私有命名空间。
    ros::NodeHandle private_nh("~");
    goal_orientation_.w = 1.0;
    std::vector<double> default_astar_retry_margins = {20.0, 50.0, 100.0};
    nh_.param("astar/search_window_retry_margins",
              astar_search_window_retry_margins_,
              default_astar_retry_margins);
    private_nh.param<std::string>("search/map_topic", map_topic_, "/projected_map");
    private_nh.param<std::string>("planner/odom_topic", odom_topic_, "/lio/robo/odom");
    private_nh.param<std::string>("planner/goal_topic", goal_topic_, "/move_base_simple/goal");
    nh_.param("planner/path_yaw_lookahead_distance", path_yaw_lookahead_distance_, 0.3);
    nh_.param("planner/search_type", search_type_, 0);
    ROS_INFO("PlanningServer params: map_topic=%s, odom_topic=%s, search_type=%d, path_yaw_lookahead=%.3f",
             map_topic_.c_str(), odom_topic_.c_str(), search_type_,
             path_yaw_lookahead_distance_);


    globalMapSub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic_, 10, &PlanningServer::globalMapCallBack, this);
    goalSub_ = nh_.subscribe(goal_topic_, 1, &PlanningServer::goalCallBack, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &PlanningServer::odomCallBack, this);

    obsHybridAstarPathPub_ = nh_.advertise<nav_msgs::Path>("/obs_hybridastar_path", 10);
    AstarPathPub_ = nh_.advertise<nav_msgs::Path>("/astar_path", 10);
    JPSPathPub_ = nh_.advertise<nav_msgs::Path>("/jps_path", 10);
    RRTPathPub_ = nh_.advertise<nav_msgs::Path>("/rrt_path", 10);
    pathEndpointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/front_path_endpoints", 1, true);

    mkr_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);

    // TODO
    astar_path_finder_.reset(new path_searching::Astar);
    astar_path_finder_->init(nh_);
    
    obs_hybridastar_path_finder_.reset(new path_searching::ObsHybridAstar);
    obs_hybridastar_path_finder_->init(nh_);

    jps_path_finder_.reset(new path_searching::JPS);
    jps_path_finder_->init(nh_);

    rrt_path_finder_.reset(new path_searching::RRT);
    rrt_path_finder_->init(nh_);
    map_init();
}

PlanningServer::~PlanningServer(){}

void PlanningServer::globalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    globalMap_ = *msg;
    search_maps_dirty_ = true;
}

void PlanningServer::odomCallBack(const nav_msgs::OdometryConstPtr &msg){
    std::lock_guard<std::mutex> lock(odom_mutex_);
    carOdom_ = *msg;
    has_odom_ = true;
}

double PlanningServer::elapsedMs(const ros::WallTime& start_time)
{
    return (ros::WallTime::now() - start_time).toSec() * 1000.0;
}

double PlanningServer::elapsedFromActiveGoalMs() const
{
    if (active_goal_wall_start_.isZero()) {
        return 0.0;
    }
    return (ros::WallTime::now() - active_goal_wall_start_).toSec() * 1000.0;
}

void PlanningServer::goalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg){
    active_goal_wall_start_ = ros::WallTime::now();
    const uint64_t seq = ++planning_seq_;
    ROS_INFO("[planner_timing][front][seq=%lu] goal_received stamp=%.6f",
             static_cast<unsigned long>(seq), msg->header.stamp.toSec());

    const ros::WallTime odom_wait_start = ros::WallTime::now();
    nav_msgs::Odometry odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!has_odom_) {
            ROS_WARN("No odom received from %s yet, ignore goal", odom_topic_.c_str());
            return;
        }
        odom = carOdom_;
    }
    ROS_INFO("[planner_timing][front][seq=%lu] odom_snapshot_ms=%.3f",
             static_cast<unsigned long>(seq), elapsedMs(odom_wait_start));

    const double start_yaw = tf::getYaw(odom.pose.pose.orientation);
    const double goal_yaw = tf::getYaw(msg->pose.orientation);

    Eigen::Vector3d plan_start(odom.pose.pose.position.x, odom.pose.pose.position.y, start_yaw);
    Eigen::Vector3d plan_goal(msg->pose.position.x, msg->pose.position.y, goal_yaw);
    goal_orientation_ = msg->pose.orientation;

    ROS_INFO_STREAM("Planning from odom start: " << plan_start.transpose());
    ROS_INFO_STREAM("Planning to goal: " << plan_goal.transpose());

    const ros::WallTime sync_start = ros::WallTime::now();
    syncSearchMaps();
    ROS_INFO("[planner_timing][front][seq=%lu] sync_maps_ms=%.3f",
             static_cast<unsigned long>(seq), elapsedMs(sync_start));

    const ros::WallTime search_start = ros::WallTime::now();
    runSelectedSearch(plan_start, plan_goal);
    ROS_INFO("[planner_timing][front][seq=%lu] run_selected_total_ms=%.3f goal_to_return_ms=%.3f",
             static_cast<unsigned long>(seq), elapsedMs(search_start), elapsedFromActiveGoalMs());
}

void PlanningServer::runSelectedSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    switch (search_type_) {
        case 0:
            PlanbyAstarSearch(start_pt, end_pt);
            PlanbyJPSSearch(start_pt, end_pt);
            PlanbyRRTSearch(start_pt, end_pt);
            PlanbyObsHybridAstarSearch(start_pt, end_pt);
            break;
        case 1:
            PlanbyAstarSearch(start_pt, end_pt);
            break;
        case 2:
            PlanbyObsHybridAstarSearch(start_pt, end_pt);
            break;
        case 3:
            PlanbyJPSSearch(start_pt, end_pt);
            break;
        case 4:
            PlanbyRRTSearch(start_pt, end_pt);
            break;
        default:
            ROS_WARN("Invalid planner/search_type=%d, run all searchers instead", search_type_);
            PlanbyAstarSearch(start_pt, end_pt);
            PlanbyObsHybridAstarSearch(start_pt, end_pt);
            PlanbyJPSSearch(start_pt, end_pt);
            PlanbyRRTSearch(start_pt, end_pt);
            break;
    }
}

void PlanningServer::publishPointWithText(const Eigen::Vector2d& p, const std::string& text, const Color c) {
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = text + "_pos";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = p(0);
    point_marker.pose.position.y = p(1);
    point_marker.pose.position.z = p(2);
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.2;
    point_marker.scale.y = 0.2;
    point_marker.scale.z = 0.2;
    point_marker.color = c;
    point_marker.color.a = 1.0;


    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = text;
    text_marker.id = 1;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = p(0);
    text_marker.pose.position.y = p(1);
    text_marker.pose.position.z = p(2) + 0.3;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.5;
    text_marker.color = c;
    text_marker.color.a = 1.0;
    text_marker.text = text;

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(point_marker);
    marker_array.markers.push_back(text_marker);
    mkr_pub.publish(marker_array);
}

void PlanningServer::map_init()
{
    ros::spinOnce();
    ros::Rate loop_rate(10);
    while(globalMap_.data.empty())
    {
        ROS_INFO_THROTTLE(2.0, "Waiting for inflated costmap from %s", map_topic_.c_str());
        ros::spinOnce();
        loop_rate.sleep();
    }
    while (!globalMap_.data.empty() && globalMap_.info.width * globalMap_.info.height != globalMap_.data.size()) {
        ROS_WARN("Map data incomplete, waiting...");
        ros::Duration(0.1).sleep();
    }
    resolution_ = globalMap_.info.resolution;
    if (resolution_ > 1.0e-6) {
        start_yaw_sample_offset_ = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(path_yaw_lookahead_distance_ / resolution_)));
    } else {
        start_yaw_sample_offset_ = 4;
    }
    ROS_INFO("Path start yaw uses lookahead %.3fm, map resolution %.3fm, sample offset %zu",
             path_yaw_lookahead_distance_, resolution_, start_yaw_sample_offset_);

    // TODO
    astar_path_finder_->setMap(globalMap_);
    // ObsHybridAstar 仍使用膨胀后的分层代价地图。
    obs_hybridastar_path_finder_->setMap(globalMap_);
    jps_path_finder_->setMap(globalMap_);
    rrt_path_finder_->setMap(globalMap_);
    search_maps_dirty_ = false;
}

void PlanningServer::syncSearchMaps()
{
    if (!search_maps_dirty_) {
        return;
    }
    astar_path_finder_->setMap(globalMap_);
    obs_hybridastar_path_finder_->setMap(globalMap_);
    jps_path_finder_->setMap(globalMap_);
    rrt_path_finder_->setMap(globalMap_);
    resolution_ = globalMap_.info.resolution;
    if (resolution_ > 1.0e-6) {
        start_yaw_sample_offset_ = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(path_yaw_lookahead_distance_ / resolution_)));
    }
    search_maps_dirty_ = false;
}

bool PlanningServer::PlanbyObsHybridAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    const ros::WallTime planner_start = ros::WallTime::now();
    Eigen::Vector4d start_state, end_state;
    Eigen::Vector2d init_ctrl(0, 0);
    start_time_ = ros::Time::now().toSec();
    start_state << start_pt, 0.01;
    end_state << end_pt, 0.01;

    int status = obs_hybridastar_path_finder_->search(start_state, init_ctrl, end_state);
    const double search_ms = elapsedMs(planner_start);
    if(!status) {
        ROS_WARN("[planner_timing][front][seq=%lu][obs_hybridastar] search_failed status=%d search_ms=%.3f goal_to_now_ms=%.3f",
                 static_cast<unsigned long>(planning_seq_), status, search_ms, elapsedFromActiveGoalMs());
        return false;
    }
    const ros::WallTime output_start = ros::WallTime::now();
    search_path_ = obs_hybridastar_path_finder_->getPath();
    obs_hybridastar_path_finder_->reset();
    ROS_INFO("[planner_timing][front][seq=%lu][obs_hybridastar] search_ms=%.3f output_reset_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), search_ms,
             elapsedMs(output_start), search_path_.size());

    displayObsHybridAstarPath(search_path_);

    return true;
}

bool PlanningServer::PlanbyAstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    const ros::WallTime planner_start = ros::WallTime::now();
    Eigen::Vector2d start_state(start_pt[0], start_pt[1]);
    Eigen::Vector2d end_state(end_pt[0], end_pt[1]);

    int status = path_searching::Astar::NO_PATH;
    for (const double margin : astar_search_window_retry_margins_) {
        const ros::WallTime attempt_start = ros::WallTime::now();
        astar_path_finder_->setSearchWindowMargin(margin);
        ROS_INFO("Try Astar search with window margin %.2fm", margin);
        status = astar_path_finder_->search(start_state, end_state, start_pt[2]);
        const double attempt_ms = elapsedMs(attempt_start);
        ROS_INFO("[planner_timing][front][seq=%lu][astar] attempt_margin=%.2f status=%d attempt_ms=%.3f",
                 static_cast<unsigned long>(planning_seq_), margin, status, attempt_ms);
        if(status == path_searching::Astar::REACH_END) {
            const ros::WallTime output_start = ros::WallTime::now();
            search_path_ = astar_path_finder_->getPath();
            astar_path_finder_->reset();
            ROS_INFO("[planner_timing][front][seq=%lu][astar] search_total_ms=%.3f output_reset_ms=%.3f path_points=%zu",
                     static_cast<unsigned long>(planning_seq_), elapsedMs(planner_start),
                     elapsedMs(output_start), search_path_.size());
            displayAstarPath(search_path_);
            return true;
        }
        astar_path_finder_->reset();
    }

    ROS_WARN("Astar failed after %zu search-window attempts", astar_search_window_retry_margins_.size());
    ROS_WARN("[planner_timing][front][seq=%lu][astar] failed search_total_ms=%.3f goal_to_now_ms=%.3f",
             static_cast<unsigned long>(planning_seq_), elapsedMs(planner_start),
             elapsedFromActiveGoalMs());
    return false;
}

bool PlanningServer::PlanbyJPSSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    const ros::WallTime planner_start = ros::WallTime::now();
    Eigen::Vector2d start_state(start_pt[0], start_pt[1]);
    Eigen::Vector2d end_state(end_pt[0], end_pt[1]);

    ROS_INFO("Start JPS search: start=(%.3f, %.3f), goal=(%.3f, %.3f)",
             start_state.x(), start_state.y(), end_state.x(), end_state.y());
    jps_path_finder_->reset();
    int status = jps_path_finder_->search(start_state, end_state);
    if(status != path_searching::JPS::REACH_END) {
        ROS_WARN("JPS failed with status %d", status);
        jps_path_finder_->reset();
        ROS_WARN("[planner_timing][front][seq=%lu][jps] failed status=%d search_ms=%.3f goal_to_now_ms=%.3f",
                 static_cast<unsigned long>(planning_seq_), status,
                 elapsedMs(planner_start), elapsedFromActiveGoalMs());
        return false;
    }
    const double search_ms = elapsedMs(planner_start);
    const ros::WallTime output_start = ros::WallTime::now();
    search_path_ = jps_path_finder_->getPath();
    jps_path_finder_->reset();
    ROS_INFO("[planner_timing][front][seq=%lu][jps] search_ms=%.3f output_reset_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), search_ms,
             elapsedMs(output_start), search_path_.size());

    displayJPSPath(search_path_);

    return true;
}

bool PlanningServer::PlanbyRRTSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    const ros::WallTime planner_start = ros::WallTime::now();
    Eigen::Vector2d start_state(start_pt[0], start_pt[1]);
    Eigen::Vector2d end_state(end_pt[0], end_pt[1]);

    ROS_INFO("Start RRT search: start=(%.3f, %.3f), goal=(%.3f, %.3f)",
             start_state.x(), start_state.y(), end_state.x(), end_state.y());
    int status = rrt_path_finder_->search(start_state, end_state);
    if(status != path_searching::RRT::REACH_END) {
        ROS_WARN("RRT failed with status %d", status);
        rrt_path_finder_->reset();
        ROS_WARN("[planner_timing][front][seq=%lu][rrt] failed status=%d search_ms=%.3f goal_to_now_ms=%.3f",
                 static_cast<unsigned long>(planning_seq_), status,
                 elapsedMs(planner_start), elapsedFromActiveGoalMs());
        return false;
    }
    const double search_ms = elapsedMs(planner_start);
    const ros::WallTime output_start = ros::WallTime::now();
    search_path_ = rrt_path_finder_->getPath();
    rrt_path_finder_->reset();
    ROS_INFO("[planner_timing][front][seq=%lu][rrt] search_ms=%.3f output_reset_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), search_ms,
             elapsedMs(output_start), search_path_.size());

    displayRRTPath(search_path_);

    return true;
}


void PlanningServer::displayObsHybridAstarPath(const std::vector<Eigen::Vector2d>& final_path)
{
    const ros::WallTime build_start = ros::WallTime::now();
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "map";
    for (const auto& pt : final_path) {
        tmpPose.pose.position.x = pt[0];
        tmpPose.pose.position.y = pt[1];
        path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    updatePathOrientations(path_msg);
    const double build_ms = elapsedMs(build_start);

    displayPathEndpoints(path_msg, "obs_hybridastar_path_endpoints");
    const ros::WallTime publish_start = ros::WallTime::now();
    obsHybridAstarPathPub_.publish(path_msg);
    ROS_INFO("[planner_timing][front][seq=%lu][obs_hybridastar] path_build_ms=%.3f publish_call_ms=%.3f goal_to_path_publish_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), build_ms,
             elapsedMs(publish_start), elapsedFromActiveGoalMs(), path_msg.poses.size());
}
void PlanningServer::displayAstarPath(const std::vector<Eigen::Vector2d>& final_path)
{
    const ros::WallTime build_start = ros::WallTime::now();
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "map";
    for (const auto& pt : final_path) {
        tmpPose.pose.position.x = pt[0];
        tmpPose.pose.position.y = pt[1];
        path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    updatePathOrientations(path_msg);
    const double build_ms = elapsedMs(build_start);
    displayPathEndpoints(path_msg, "astar_path_endpoints");
    const ros::WallTime publish_start = ros::WallTime::now();
    AstarPathPub_.publish(path_msg);
    ROS_INFO("[planner_timing][front][seq=%lu][astar] path_build_ms=%.3f publish_call_ms=%.3f goal_to_path_publish_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), build_ms,
             elapsedMs(publish_start), elapsedFromActiveGoalMs(), path_msg.poses.size());
}
void PlanningServer::displayJPSPath(const std::vector<Eigen::Vector2d>& final_path)
{
    const ros::WallTime build_start = ros::WallTime::now();
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "map";
    for (const auto& pt : final_path) {
        tmpPose.pose.position.x = pt[0];
        tmpPose.pose.position.y = pt[1];
        path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    updatePathOrientations(path_msg);
    const double build_ms = elapsedMs(build_start);
    displayPathEndpoints(path_msg, "jps_path_endpoints");
    const ros::WallTime publish_start = ros::WallTime::now();
    JPSPathPub_.publish(path_msg);
    ROS_INFO("[planner_timing][front][seq=%lu][jps] path_build_ms=%.3f publish_call_ms=%.3f goal_to_path_publish_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), build_ms,
             elapsedMs(publish_start), elapsedFromActiveGoalMs(), path_msg.poses.size());
}
void PlanningServer::displayRRTPath(const std::vector<Eigen::Vector2d>& final_path)
{
    const ros::WallTime build_start = ros::WallTime::now();
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "map";
    for (const auto& pt : final_path) {
        tmpPose.pose.position.x = pt[0];
        tmpPose.pose.position.y = pt[1];
        path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    updatePathOrientations(path_msg);
    const double build_ms = elapsedMs(build_start);
    displayPathEndpoints(path_msg, "rrt_path_endpoints");
    const ros::WallTime publish_start = ros::WallTime::now();
    RRTPathPub_.publish(path_msg);
    ROS_INFO("[planner_timing][front][seq=%lu][rrt] path_build_ms=%.3f publish_call_ms=%.3f goal_to_path_publish_ms=%.3f path_points=%zu",
             static_cast<unsigned long>(planning_seq_), build_ms,
             elapsedMs(publish_start), elapsedFromActiveGoalMs(), path_msg.poses.size());
}

void PlanningServer::updatePathOrientations(nav_msgs::Path& path_msg)
{
    if (path_msg.poses.empty()) {
        return;
    }

    auto compute_path_yaw = [this, &path_msg](size_t index) {
        const size_t last_index = path_msg.poses.size() - 1;
        if (last_index == 0) {
            return 0.0;
        }

        if (index == 0) {
            const size_t direction_index = std::min<size_t>(start_yaw_sample_offset_, last_index);
            const auto& first_pose = path_msg.poses.front().pose.position;
            const auto& direction_pose = path_msg.poses[direction_index].pose.position;
            const double dx = direction_pose.x - first_pose.x;
            const double dy = direction_pose.y - first_pose.y;
            if (dx * dx + dy * dy > 1.0e-12) {
                return std::atan2(dy, dx);
            }
        }

        for (size_t offset = 1; offset <= 25; ++offset) {
            const size_t prev_index = index > offset ? index - offset : 0;
            const size_t next_index = std::min(last_index, index + offset);
            if (prev_index == next_index) {
                continue;
            }

            const auto& prev_pose = path_msg.poses[prev_index].pose.position;
            const auto& next_pose = path_msg.poses[next_index].pose.position;
            const double dx = next_pose.x - prev_pose.x;
            const double dy = next_pose.y - prev_pose.y;
            if (dx * dx + dy * dy > 1.0e-12) {
                return std::atan2(dy, dx);
            }
        }

        return 0.0;
    };

    for (size_t i = 0; i + 1 < path_msg.poses.size(); ++i) {
        path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(compute_path_yaw(i));
    }

    path_msg.poses.back().pose.orientation = goal_orientation_;
}

void PlanningServer::displayPathEndpoints(const nav_msgs::Path& path_msg, const std::string& ns)
{
    if (path_msg.poses.empty()) {
        return;
    }

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    const std::array<const geometry_msgs::PoseStamped*, 2> endpoint_poses{
        &path_msg.poses.front(), &path_msg.poses.back()};
    for (size_t i = 0; i < endpoint_poses.size(); ++i) {
        visualization_msgs::Marker arrow;
        arrow.header = path_msg.header;
        arrow.ns = "front_" + ns;
        arrow.id = static_cast<int>(i);
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose = endpoint_poses[i]->pose;
        arrow.pose.position.z += 0.35;
        arrow.scale.x = 0.45;
        arrow.scale.y = 0.09;
        arrow.scale.z = 0.09;
        arrow.color.a = 1.0;
        if (i == 0) {
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 1.0;
        } else {
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 1.0;
        }
        markers.markers.push_back(arrow);
    }
    pathEndpointsPub_.publish(markers);
}

int main(int argc, char** argv){
    std::cout << " Initialize the road network, please wait  ..." << std::endl;
    ros::init(argc, argv, "planning_server");
    ros::NodeHandle nh;
    PlanningServer planServer(nh);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();
    
    ros::waitForShutdown();
    return 0;
}
