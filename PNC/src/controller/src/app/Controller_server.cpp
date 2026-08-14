#include "controller/controller_server.h"

#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <yhs_can_msgs/ctrl_cmd.h>
#include <yhs_can_msgs/steering_ctrl_cmd.h>

#include <algorithm>
#include <clocale>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>

ControllerServer::ControllerServer(ros::NodeHandle nh)
    : nh_(nh), pure_pursuit_(nh), mpc_controller_(nh)
{
    loadParams();
    setupRosIo();
}

bool ControllerServer::ok() const
{
    return controller_type_ == 0 || controller_type_ == 1;
}

void ControllerServer::spin()
{
    ros::Rate loop_rate(rate_hz_);
    ROS_INFO("controller_server started with controller_type=%d, rate=%.2fHz",
             controller_type_, rate_hz_);

    while (ros::ok()) {
        ros::spinOnce();
        ControlCommand command;
        command.linear_velocity = 0.0;
        command.angular_velocity = 0.0;

        if (odom_received_ && path_reset_) {
            if (controller_type_ == 0) {
                command = pure_pursuit_.PurePursuitbyYaw(robot_state_);
            } else if (controller_type_ == 1) {
                command = mpc_controller_.computeCommand(robot_state_);
            }
        }

        const bool finished_edge =
            command.state == ControlState::Finished
            && last_control_state_ != ControlState::Finished;
        if (finished_edge) {
            if (!arrive_reported_) {
                std_msgs::Bool arrive_msg;
                arrive_msg.data = true;
                pub_arrive_.publish(arrive_msg);
                arrive_reported_ = true;
                beginFinishOdomAveraging();
                ROS_INFO("Arrive at the goal");
            }
        }

        const bool state_changed =
            command.state != ControlState::None
            && command.state != last_control_state_;
        if (state_changed) {
            publishStopBeforeStateTransition(command.state);
        }

        publishCommand(command);
        last_control_state_ = command.state;
        loop_rate.sleep();
    }
}

void ControllerServer::loadParams()
{
    nh_.param("controller_server/controller_type", controller_type_, 0);
    nh_.param("controller_server/rate", rate_hz_, 10.0);
    if (rate_hz_ <= 0.0) {
        ROS_WARN("controller_server/rate <= 0, use 10Hz");
        rate_hz_ = 10.0;
    }

    ros::NodeHandle private_nh("~");
    private_nh.param("path_topic", path_topic_, std::string("/astar_path_o"));
    private_nh.param("trajectory_topic", trajectory_topic_, std::string("/trajopt/minco_traj"));
    private_nh.param("odom_topic", odom_topic_, std::string("/lio/robo/odom"));
    nh_.param("controller_server/finish_error_log_path",
              finish_error_log_path_,
              std::string("/home/lfh/SLAM+PNC/PNC/src/controller/finish_error_log.txt"));
    nh_.param("controller_server/finish_average_frames", finish_average_frames_, 20);
    nh_.param("controller_server/max_forward_linear_velocity", max_forward_linear_velocity_, 0.5);
    nh_.param("controller_server/max_backward_linear_velocity", max_backward_linear_velocity_, -0.3);
    nh_.param("controller_server/max_angular_velocity", max_angular_velocity_, 0.3);
    nh_.param("controller_server/state_transition_stop_duration", state_transition_stop_duration_, 0.5);
    if (finish_average_frames_ <= 0) {
        ROS_WARN("controller_server/finish_average_frames <= 0, use 20");
        finish_average_frames_ = 20;
    }
    if (state_transition_stop_duration_ < 0.0) {
        ROS_WARN("controller_server/state_transition_stop_duration < 0, use 0.5s");
        state_transition_stop_duration_ = 0.5;
    }
}

void ControllerServer::setupRosIo()
{
    sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic_, 1, &ControllerServer::pathCallback, this);
    sub_trajectory_ = nh_.subscribe<robot_trajectory_msgs::RobotTrajectory>(
        trajectory_topic_, 1, &ControllerServer::trajectoryCallback, this);
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &ControllerServer::odomCallback, this);
    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_sim_cmd_ = nh_.advertise<geometry_msgs::Twist>("/car1/cmd_vel", 1);
    pub_ctrl_cmd_ = nh_.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);
    pub_steering_ctrl_cmd_ = nh_.advertise<yhs_can_msgs::steering_ctrl_cmd>("/steering_ctrl_cmd", 1);
    pub_arrive_ = nh_.advertise<std_msgs::Bool>("/arrive/finish", 1);
    pub_odom_path_ = nh_.advertise<nav_msgs::Path>("/controller/odom_path", 1, true);
}

void ControllerServer::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (controller_type_ != 0) {
        return;
    }

    // 新控制路径对应一次新的跟踪过程，清空并立即发布空的历史里程计轨迹。
    odom_path_.poses.clear();
    odom_path_.header.frame_id = msg->header.frame_id.empty()
        ? std::string("map") : msg->header.frame_id;
    odom_path_.header.stamp = ros::Time::now();
    has_last_odom_path_point_ = false;
    odom_path_active_ = true;
    pub_odom_path_.publish(odom_path_);

    path_reset_ = pure_pursuit_.reset(*msg);
    if (path_reset_) {
        arrive_reported_ = false;
        last_control_state_ = ControlState::None;
        collecting_finish_odom_ = false;
        finish_log_written_ = false;
        finish_odom_samples_.clear();

        const auto& goal_pose = msg->poses.back().pose;
        goal_point_.x = goal_pose.position.x;
        goal_point_.y = goal_pose.position.y;
        goal_point_.yaw = tf2::getYaw(goal_pose.orientation);
        has_goal_point_ = true;
    }
}

void ControllerServer::trajectoryCallback(
    const robot_trajectory_msgs::RobotTrajectory::ConstPtr& msg)
{
    if (controller_type_ != 1) {
        return;
    }

    odom_path_.poses.clear();
    odom_path_.header.frame_id = msg->header.frame_id.empty()
        ? std::string("map") : msg->header.frame_id;
    odom_path_.header.stamp = ros::Time::now();
    has_last_odom_path_point_ = false;
    odom_path_active_ = true;
    pub_odom_path_.publish(odom_path_);

    path_reset_ = mpc_controller_.reset(*msg);
    if (path_reset_) {
        arrive_reported_ = false;
        last_control_state_ = ControlState::None;
        collecting_finish_odom_ = false;
        finish_log_written_ = false;
        finish_odom_samples_.clear();

        const auto& goal_pose = msg->points.back().pose;
        goal_point_.x = goal_pose.position.x;
        goal_point_.y = goal_pose.position.y;
        goal_point_.yaw = tf2::getYaw(goal_pose.orientation);
        has_goal_point_ = true;
    }
}

void ControllerServer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_state_.x = msg->pose.pose.position.x;
    robot_state_.y = msg->pose.pose.position.y;
    robot_state_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    robot_state_.linear_velocity =
        msg->twist.twist.linear.x * std::cos(robot_state_.yaw)
        + msg->twist.twist.linear.y * std::sin(robot_state_.yaw);
    robot_state_.angular_velocity = msg->twist.twist.angular.z;
    odom_received_ = true;

    const auto& current_point = msg->pose.pose.position;
    const double distance = has_last_odom_path_point_
        ? std::hypot(current_point.x - last_odom_path_point_.x,
                     current_point.y - last_odom_path_point_.y)
        : std::numeric_limits<double>::infinity();
    if (odom_path_active_ && distance >= 0.01) {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        odom_path_.header.frame_id = msg->header.frame_id.empty()
            ? std::string("map") : msg->header.frame_id;
        odom_path_.header.stamp = msg->header.stamp;
        odom_path_.poses.push_back(pose);
        last_odom_path_point_ = current_point;
        has_last_odom_path_point_ = true;
        pub_odom_path_.publish(odom_path_);
    }

    collectFinishOdomSample(robot_state_);
}

void ControllerServer::publishCommand(const ControlCommand& command)
{
    const ControlCommand limited_command = vwlimit(command);
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = limited_command.linear_velocity;
    cmd_msg.angular.z = limited_command.angular_velocity;
    pub_cmd_.publish(cmd_msg);
    pub_sim_cmd_.publish(cmd_msg);

    if (limited_command.state == ControlState::Init
        || limited_command.state == ControlState::GoalYawAdjust) {
        yhs_can_msgs::ctrl_cmd ctrl_cmd_msg;
        ctrl_cmd_msg.ctrl_cmd_gear = 6;
        ctrl_cmd_msg.ctrl_cmd_linear = 0.0;
        ctrl_cmd_msg.ctrl_cmd_angular = limited_command.angular_velocity * 180.0 * M_1_PI;
        ctrl_cmd_msg.ctrl_cmd_slipangle = 0.0;
        pub_ctrl_cmd_.publish(ctrl_cmd_msg);
        return;
    }

    yhs_can_msgs::steering_ctrl_cmd steering_cmd_msg;
    steering_cmd_msg.ctrl_cmd_gear = 5;
    steering_cmd_msg.steering_ctrl_cmd_velocity = limited_command.linear_velocity;
    steering_cmd_msg.steering_ctrl_cmd_steering = limited_command.angular_velocity * 180.0 * M_1_PI;
    steering_cmd_msg.steering_ctrl_cmd_slipangle = 0.0;
    pub_steering_ctrl_cmd_.publish(steering_cmd_msg);
}

void ControllerServer::publishStopBeforeStateTransition(ControlState next_state)
{
    ControlCommand stop_command;
    stop_command.state = next_state;
    stop_command.linear_velocity = 0.0;
    stop_command.angular_velocity = 0.0;

    ros::Rate stop_rate(rate_hz_);
    const ros::WallTime stop_end =
        ros::WallTime::now() + ros::WallDuration(state_transition_stop_duration_);
    while (ros::ok() && ros::WallTime::now() < stop_end) {
        // 状态切换前持续发布零速度，避免上一状态的控制量残留。
        publishCommand(stop_command);
        stop_rate.sleep();
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

void ControllerServer::beginFinishOdomAveraging()
{
    if (!has_goal_point_) {
        ROS_WARN("No goal point recorded, skip finish error log");
        return;
    }

    finish_odom_samples_.clear();
    collecting_finish_odom_ = true;
    finish_log_written_ = false;
    ROS_INFO("Start collecting %d odom frames after Finished", finish_average_frames_);
}

void ControllerServer::collectFinishOdomSample(const RobotState& odom_state)
{
    if (!collecting_finish_odom_ || finish_log_written_) {
        return;
    }

    finish_odom_samples_.push_back(odom_state);
    if (static_cast<int>(finish_odom_samples_.size()) >= finish_average_frames_) {
        collecting_finish_odom_ = false;
        writeFinishErrorLog();
    }
}

void ControllerServer::writeFinishErrorLog()
{
    if (finish_odom_samples_.empty() || !has_goal_point_) {
        return;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin_yaw = 0.0;
    double sum_cos_yaw = 0.0;
    for (const auto& sample : finish_odom_samples_) {
        sum_x += sample.x;
        sum_y += sample.y;
        sum_sin_yaw += std::sin(sample.yaw);
        sum_cos_yaw += std::cos(sample.yaw);
    }

    const double inv_count = 1.0 / static_cast<double>(finish_odom_samples_.size());
    const double avg_x = sum_x * inv_count;
    const double avg_y = sum_y * inv_count;
    const double avg_yaw = std::atan2(sum_sin_yaw, sum_cos_yaw);

    const double error_x = avg_x - goal_point_.x;
    const double error_y = avg_y - goal_point_.y;
    const double error_xy = std::hypot(error_x, error_y);
    const double error_yaw = normalizeAngle(avg_yaw - goal_point_.yaw);

    std::ofstream log_file(finish_error_log_path_, std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
        ROS_ERROR("Failed to open finish error log: %s", finish_error_log_path_.c_str());
        return;
    }

    log_file << std::fixed << std::setprecision(6)
             << " goal_x " << goal_point_.x
             << " goal_y " << goal_point_.y
             << " goal_yaw " << goal_point_.yaw
             << " avg_odom_x " << avg_x
             << " avg_odom_y " << avg_y
             << " avg_odom_yaw " << avg_yaw
             << " error_x " << error_x
             << " error_y " << error_y
             << " error_xy " << error_xy
             << " error_yaw " << error_yaw
             << '\n';

    finish_log_written_ = true;
    ROS_INFO("xy_error=%.4f m, yaw_error=%.4f rad",
             error_xy, error_yaw);
}

double ControllerServer::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "controller_server");

    ros::NodeHandle nh;
    ControllerServer server(nh);

    server.spin();
    return 0;
}
