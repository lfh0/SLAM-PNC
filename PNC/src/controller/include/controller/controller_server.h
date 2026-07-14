#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <string>

#include "controller/purepursuit.h"

class ControllerServer
{
public:
    ControllerServer(ros::NodeHandle nh, ros::NodeHandle nh_private);

    bool ok() const;
    void spin();

private:
    void loadParams();
    void setupRosIo();
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void startCallback(const std_msgs::Bool::ConstPtr& msg);
    void publishCommand(const ControlCommand& command);
    ControlCommand vwlimit(const ControlCommand& command) const;

    ros::NodeHandle nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_start_;
    ros::Publisher pub_cmd_;
    ros::Publisher pub_sim_cmd_;
    ros::Publisher pub_arrive_;
    std::string path_topic_;
    std::string odom_topic_;
    std::string start_topic_;
    std::string cmd_vel_topic_;
    std::string sim_cmd_vel_topic_;
    std::string arrive_topic_;
    RobotState robot_state_;

    std::string controller_type_;
    PurePursuit pure_pursuit_;

    double rate_hz_ = 10.0;
    double max_forward_linear_velocity_ = 0.5;
    double max_backward_linear_velocity_ = -0.3;
    double max_angular_velocity_ = 0.3;
    bool path_reset_ = false;
    bool odom_received_ = false;
    bool start_ = true;
    bool arrive_reported_ = false;
};
