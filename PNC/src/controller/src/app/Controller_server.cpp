#include "controller/controller_server.h"

#include <tf2/utils.h>

#include <algorithm>

ControllerServer::ControllerServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), pure_pursuit_(nh)
{
    (void)nh_private;
    loadParams();
    setupRosIo();
}

bool ControllerServer::ok() const
{
    return controller_type_ == "pure_pursuit";
}

void ControllerServer::spin()
{
    ros::Rate loop_rate(rate_hz_);
    ROS_INFO("controller_server started with controller_type=%s, rate=%.2fHz",
             controller_type_.c_str(), rate_hz_);

    while (ros::ok()) {
        ros::spinOnce();
        ControlCommand command;
        command.linear_velocity = 0.0;
        command.angular_velocity = 0.0;

        if (odom_received_ && start_ && path_reset_) {
            command = pure_pursuit_.PurePursuitbyYaw(robot_state_);
        }

        if (command.state == ControlState::Finished) {
            if (!arrive_reported_) {
                std_msgs::Bool arrive_msg;
                arrive_msg.data = true;
                pub_arrive_.publish(arrive_msg);
                arrive_reported_ = true;
            }
            ROS_INFO("Arrive at the goal");
            start_ = false;
        }

        publishCommand(command);
        loop_rate.sleep();
    }
}

void ControllerServer::loadParams()
{
    nh_.param<std::string>("controller_server/controller_type", controller_type_, "pure_pursuit");
    nh_.param("controller_server/rate", rate_hz_, 10.0);
    if (rate_hz_ <= 0.0) {
        ROS_WARN("controller_server/rate <= 0, use 10Hz");
        rate_hz_ = 10.0;
    }

    nh_.param("controller_server/path_topic", path_topic_, std::string("/astar_path_o"));
    nh_.param("controller_server/odom_topic", odom_topic_, std::string("/lio/robo/odom"));
    nh_.param("controller_server/start_topic", start_topic_, std::string("/start"));
    nh_.param("controller_server/cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param("controller_server/sim_cmd_vel_topic", sim_cmd_vel_topic_, std::string("/car1/cmd_vel"));
    nh_.param("controller_server/arrive_topic", arrive_topic_, std::string("/arrive/finish"));
    nh_.param("controller_server/max_forward_linear_velocity", max_forward_linear_velocity_, 0.5);
    nh_.param("controller_server/max_backward_linear_velocity", max_backward_linear_velocity_, -0.3);
    nh_.param("controller_server/max_angular_velocity", max_angular_velocity_, 0.3);
}

void ControllerServer::setupRosIo()
{
    sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic_, 1, &ControllerServer::pathCallback, this);
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &ControllerServer::odomCallback, this);
    sub_start_ = nh_.subscribe<std_msgs::Bool>(start_topic_, 1, &ControllerServer::startCallback, this);
    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    pub_sim_cmd_ = nh_.advertise<geometry_msgs::Twist>(sim_cmd_vel_topic_, 1);
    pub_arrive_ = nh_.advertise<std_msgs::Bool>(arrive_topic_, 1);
}

void ControllerServer::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_reset_ = pure_pursuit_.reset(*msg);
    if (path_reset_) {
        arrive_reported_ = false;
    }
}

void ControllerServer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_state_.x = msg->pose.pose.position.x;
    robot_state_.y = msg->pose.pose.position.y;
    robot_state_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    robot_state_.linear_velocity = msg->twist.twist.linear.x;
    robot_state_.angular_velocity = msg->twist.twist.angular.z;
    odom_received_ = true;
}

void ControllerServer::startCallback(const std_msgs::Bool::ConstPtr& msg)
{
    start_ = msg->data;
}

void ControllerServer::publishCommand(const ControlCommand& command)
{
    const ControlCommand limited_command = vwlimit(command);
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = limited_command.linear_velocity;
    cmd_msg.angular.z = limited_command.angular_velocity;
    pub_cmd_.publish(cmd_msg);
    if (sim_cmd_vel_topic_ != cmd_vel_topic_) {
        pub_sim_cmd_.publish(cmd_msg);
    }
}

ControlCommand ControllerServer::vwlimit(const ControlCommand& command) const
{
    ControlCommand limited_command = command;
    double scale = 1.0;

    if (limited_command.linear_velocity > max_forward_linear_velocity_) {
        scale = std::min(scale, max_forward_linear_velocity_ / limited_command.linear_velocity);
    } else if (limited_command.linear_velocity < max_backward_linear_velocity_) {
        scale = std::min(scale, max_backward_linear_velocity_ / limited_command.linear_velocity);
    }

    if (limited_command.angular_velocity > max_angular_velocity_) {
        scale = std::min(scale, max_angular_velocity_ / limited_command.angular_velocity);
    } else if (limited_command.angular_velocity < -max_angular_velocity_) {
        scale = std::min(scale, -max_angular_velocity_ / limited_command.angular_velocity);
    }

    if (scale < 1.0) {
        limited_command.linear_velocity *= scale;
        limited_command.angular_velocity *= scale;
    }
    return limited_command;
}
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "controller_server");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ControllerServer server(nh, nh_private);

    server.spin();
    return 0;
}
