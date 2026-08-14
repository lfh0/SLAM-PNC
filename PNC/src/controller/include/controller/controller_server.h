#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <robot_trajectory_msgs/RobotTrajectory.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>

#include "controller/mpc.h"
#include "controller/purepursuit.h"

class ControllerServer
{
public:
    explicit ControllerServer(ros::NodeHandle nh);

    bool ok() const;
    void spin();

private:
    void loadParams();
    void setupRosIo();
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void trajectoryCallback(const robot_trajectory_msgs::RobotTrajectory::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void publishCommand(const ControlCommand& command);
    void publishStopBeforeStateTransition(ControlState next_state);
    ControlCommand vwlimit(const ControlCommand& command) const;
    void beginFinishOdomAveraging();
    void collectFinishOdomSample(const RobotState& odom_state);
    void writeFinishErrorLog();
    double normalizeAngle(double angle) const;

    ros::NodeHandle nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_cmd_;
    ros::Publisher pub_sim_cmd_;
    ros::Publisher pub_ctrl_cmd_;
    ros::Publisher pub_steering_ctrl_cmd_;
    ros::Publisher pub_arrive_;
    ros::Publisher pub_odom_path_;
    std::string path_topic_;
    std::string trajectory_topic_;
    std::string odom_topic_;
    std::string finish_error_log_path_;
    RobotState robot_state_;
    PathPoint goal_point_;
    nav_msgs::Path odom_path_;
    geometry_msgs::Point last_odom_path_point_;
    std::vector<RobotState> finish_odom_samples_;

    int controller_type_ = 0;
    PurePursuit pure_pursuit_;
    MpcController mpc_controller_;

    double rate_hz_ = 10.0;
    double max_forward_linear_velocity_ = 0.5;
    double max_backward_linear_velocity_ = -0.3;
    double max_angular_velocity_ = 0.3;
    double state_transition_stop_duration_ = 0.5;
    int finish_average_frames_ = 20;
    bool path_reset_ = false;
    bool odom_received_ = false;
    bool arrive_reported_ = false;
    ControlState last_control_state_ = ControlState::None;
    bool has_goal_point_ = false;
    bool collecting_finish_odom_ = false;
    bool finish_log_written_ = false;
    bool has_last_odom_path_point_ = false;
    bool odom_path_active_ = false;
};
