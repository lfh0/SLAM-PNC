#include "controller/purepursuit.h"
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
double clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}
}  // namespace

PurePursuit::PurePursuit(const ros::NodeHandle& nh)
{
    param_init(nh);
}

void PurePursuit::param_init(const ros::NodeHandle& nh)
{
    // 基础控制参数
    nh.param("purepursuit_node/lookahead_distance", lookahead_distance_, 0.1);
    nh.param("purepursuit_node/vehicle_avg_speed", reference_velocity_, 0.28);
    nh.param("purepursuit_node/kp_yaw_", kp_yaw_, 0.5);
    nh.param("purepursuit_node/kd_yaw_", kd_yaw_, 0.1);
    nh.param("purepursuit_node/kp_pos_", kp_pos_, 0.5);
    nh.param("purepursuit_node/end_yaw_kp_", end_yaw_kp_, 0.5);
    nh.param("purepursuit_node/end_yaw_ki_", end_yaw_ki_, 0.0);
    nh.param("purepursuit_node/end_yaw_kd_", end_yaw_kd_, 0.1);
    nh.param("purepursuit_node/end_pos_kp_", end_pos_kp_, 0.68);
    nh.param("purepursuit_node/end_pos_ki_", end_pos_ki_, 0.0);
    nh.param("purepursuit_node/end_pos_kd_", end_pos_kd_, 0.0);

    // 速度策略和误差权重参数
    nh.param("purepursuit_node/curvature_velocity_gain", curvature_velocity_gain_, 1.2);
    nh.param("purepursuit_node/lateral_velocity_gain", lateral_velocity_gain_, 0.5);
    nh.param("purepursuit_node/rotate_in_place_threshold", rotate_in_place_threshold_, 1.0);
    nh.param("purepursuit_node/derivative_filter_lambda", derivative_filter_lambda_, 0.8);
    nh.param("purepursuit_node/goal_position_tolerance", goal_position_tolerance_, 0.05);
    nh.param("purepursuit_node/goal_yaw_tolerance", goal_yaw_tolerance_, 0.05);
    nh.param("purepursuit_node/goal_slowdown_distance", goal_slowdown_distance_, 0.4);
    lookahead_distance_ = std::max(lookahead_distance_, 1.0e-3);
    reset_PID();
    state_ = ControlState::None;
}


bool PurePursuit::reset(const nav_msgs::Path& path)
{
    if (path.poses.empty()) {
        ROS_ERROR("path is empty");
        return false;
    }
    path_ = path;
    reset_PID();
    last_nearest_index_ = 0;
    state_ = ControlState::None;
    start_point_.x = path_.poses.front().pose.position.x;
    start_point_.y = path_.poses.front().pose.position.y;
    start_point_.yaw = tf2::getYaw(path_.poses.front().pose.orientation);
    end_point_.x = path_.poses.back().pose.position.x;
    end_point_.y = path_.poses.back().pose.position.y;
    end_point_.yaw = tf2::getYaw(path_.poses.back().pose.orientation);
    ROS_INFO("path size: %lu", static_cast<unsigned long>(path_.poses.size()));
    return true;
}

void PurePursuit::reset_PID()
{
    last_heading_error_ = 0.0;
    filtered_heading_error_rate_ = 0.0;
    goal_pos_integral_ = 0.0;
    goal_pos_last_error_ = 0.0;
    goal_yaw_integral_ = 0.0;
    goal_yaw_last_error_ = 0.0;
    last_control_time_ = ros::Time::now();
    heading_pid_initialized_ = false;
    goal_yaw_pid_initialized_ = false;
}

double PurePursuit::getTheta(double ref_theta, double vehicleYaw)
{
    double theta = ref_theta - vehicleYaw;
    while (theta > M_PI) {
        theta -= 2.0 * M_PI;
    }
    while (theta <= -M_PI) {
        theta += 2.0 * M_PI;
    }
    return theta;
}


double PurePursuit::computeLateralError(const RobotState& robot, const PathPoint& reference) const
{
    const double dx = robot.x - reference.x;
    const double dy = robot.y - reference.y;
    return -std::sin(reference.yaw) * dx + std::cos(reference.yaw) * dy;
}

double PurePursuit::computeLinearVelocity(double reference_velocity,
                                          double curvature,
                                          double heading_error,
                                          double lateral_error) const
{
    const double curvature_scale = 1.0 / (1.0 + curvature_velocity_gain_ * std::fabs(curvature));
    const double heading_scale = std::max(0.0, std::cos(heading_error));
    const double lateral_scale = 1.0 / (1.0 + lateral_velocity_gain_ * std::fabs(lateral_error));

    double target_linear_velocity = reference_velocity * curvature_scale * heading_scale * lateral_scale;
    if (std::fabs(heading_error) > rotate_in_place_threshold_) {
        target_linear_velocity = 0.0;
    }

    return target_linear_velocity;
}

double PurePursuit::computeAngularVelocity(double linear_velocity,
                                           double curvature,
                                           double heading_error,
                                           double lateral_error,
                                           double dt)
{
    const double safe_dt = std::max(dt, 1.0e-3);
    double raw_heading_error_rate = 0.0;
    if (heading_pid_initialized_) {
        raw_heading_error_rate = getTheta(heading_error, last_heading_error_) / safe_dt;
    } else {
        heading_pid_initialized_ = true;
    }

    filtered_heading_error_rate_ =
        derivative_filter_lambda_ * filtered_heading_error_rate_
        + (1.0 - derivative_filter_lambda_) * raw_heading_error_rate;

    const double omega_feedforward = linear_velocity * curvature;
    const double target_angular_velocity =
        omega_feedforward
        + kp_yaw_ * heading_error
        - kp_pos_ * lateral_error
        + kd_yaw_ * filtered_heading_error_rate_;

    last_heading_error_ = heading_error;
    return target_angular_velocity;
}



ControlCommand PurePursuit::computeGoalPositionPID(const RobotState& robot, const PathPoint& goal, double dt)
{
    const double safe_dt = std::max(dt, 1.0e-3);
    const double dx = goal.x - robot.x;
    const double dy = goal.y - robot.y;

    const double rho = std::hypot(dx, dy);
    const double alpha = getTheta(std::atan2(dy, dx), robot.yaw);

    goal_pos_integral_ += rho * safe_dt;
    goal_pos_integral_ = clampValue(goal_pos_integral_, -0.5, 0.5);
    const double rho_dot = (rho - goal_pos_last_error_) / safe_dt;
    goal_pos_last_error_ = rho;

    const double linear_velocity =
        (end_pos_kp_ * rho + end_pos_ki_ * goal_pos_integral_ + end_pos_kd_ * rho_dot) * std::cos(alpha);
    const double angular_velocity = end_yaw_kp_ * alpha;

    return makeCommand(linear_velocity, angular_velocity);
}

double PurePursuit::StartORGoalYawPID(const double yaw_error, double dt)
{
    const double safe_dt = std::max(dt, 1.0e-3);
    goal_yaw_integral_ += yaw_error * safe_dt;
    goal_yaw_integral_ = clampValue(goal_yaw_integral_, -0.5, 0.5);
    double yaw_error_dot = 0.0;
    if (goal_yaw_pid_initialized_) {
        yaw_error_dot = getTheta(yaw_error, goal_yaw_last_error_) / safe_dt;
    } else {
        goal_yaw_pid_initialized_ = true;
    }
    goal_yaw_last_error_ = yaw_error;

    const double angular_velocity =
        end_yaw_kp_ * yaw_error + end_yaw_ki_ * goal_yaw_integral_ + end_yaw_kd_ * yaw_error_dot;

    return angular_velocity;
}


ControlCommand PurePursuit::PurePursuitbyYaw(const RobotState& robot_state)
{
    const ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_control_time_).toSec();
    if (dt <= 1.0e-3) {
        dt = 1.0e-3;
    }
    dt = std::min(dt, 0.1);
    last_control_time_ = current_time;

    if (path_.poses.empty()) {
        return makeCommand(0.0, 0.0);
    }

    switch (state_) {
    case ControlState::None:
        if(!path_.poses.empty()){
            state_ = ControlState::Init;
            ROS_INFO("Init.............");
        }
        return makeCommand(0.0, 0.0);
    case ControlState::Init:
    {
        const double yaw_error = getTheta(start_point_.yaw, robot_state.yaw);
        if(std::fabs(yaw_error) <= goal_yaw_tolerance_){
            state_ = ControlState::Tracking;
            ROS_INFO("Tracking.............");
            reset_PID();
            return makeCommand(0.0, 0.0);
        }
        return makeCommand(0.0, StartORGoalYawPID(yaw_error, dt));
    }

    case ControlState::TakingOff:
        state_ = ControlState::Tracking;
        reset_PID();
        return makeCommand(0.0, 0.0);

    case ControlState::Tracking: {
        const auto& goal_pose = path_.poses.back().pose;
        const double dx = goal_pose.position.x - robot_state.x;
        const double dy = goal_pose.position.y - robot_state.y;
        if (std::hypot(dx, dy) <= goal_slowdown_distance_) {
            state_ = ControlState::CloseEnd;
            ROS_INFO("CloseEnd.............");
            reset_PID();
            return computeGoalPositionPID(robot_state, end_point_, dt);
        }

        const int path_last_index = static_cast<int>(path_.poses.size()) - 1;
        const int search_begin = std::max(0, last_nearest_index_ - 3);
        const int search_end = std::min(path_last_index, last_nearest_index_ + 50);
        int nearest_index = last_nearest_index_;
        double nearest_distance_sq = std::numeric_limits<double>::max();
        for (int i = search_begin; i <= search_end; ++i) {
            const double path_dx = path_.poses[i].pose.position.x - robot_state.x;
            const double path_dy = path_.poses[i].pose.position.y - robot_state.y;
            const double distance_sq = path_dx * path_dx + path_dy * path_dy;
            if (distance_sq < nearest_distance_sq) {
                nearest_distance_sq = distance_sq;
                nearest_index = i;
            }
        }
        nearest_index = std::max(nearest_index, last_nearest_index_);
        last_nearest_index_ = nearest_index;

        int target_index = path_last_index;
        for (int i = nearest_index; i <= path_last_index; ++i) {
            const double path_dx = path_.poses[i].pose.position.x - robot_state.x;
            const double path_dy = path_.poses[i].pose.position.y - robot_state.y;
            if (std::hypot(path_dx, path_dy) >= lookahead_distance_) {
                target_index = i;
                break;
            }
        }
        const auto& nearest_pose = path_.poses[nearest_index].pose;
        const auto& target_pose = path_.poses[target_index].pose;
        const double target_x = target_pose.position.x;
        const double target_y = target_pose.position.y;
        const double target_yaw = tf2::getYaw(target_pose.orientation);
        const double target_dx = target_x - robot_state.x;
        const double target_dy = target_y - robot_state.y;
        const double cos_yaw = std::cos(robot_state.yaw);
        const double sin_yaw = std::sin(robot_state.yaw);
        const double target_x_body = cos_yaw * target_dx + sin_yaw * target_dy;
        const double target_y_body = -sin_yaw * target_dx + cos_yaw * target_dy;
        if (target_x_body <= 0.0) {
            const double bearing_error = getTheta(std::atan2(target_dy, target_dx), robot_state.yaw);
            return makeCommand(0.0, kp_yaw_ * bearing_error);
        }

        const double actual_lookahead = std::max(1.0e-3, std::hypot(target_x_body, target_y_body));
        const double alpha = std::atan2(target_y_body, target_x_body);
        const double curvature = 2.0 * std::sin(alpha) / actual_lookahead;

        PathPoint nearest;
        nearest.x = nearest_pose.position.x;
        nearest.y = nearest_pose.position.y;
        nearest.yaw = tf2::getYaw(nearest_pose.orientation);

        const double heading_error = getTheta(target_yaw, robot_state.yaw);
        const double lateral_error = computeLateralError(robot_state, nearest);
        const double linear_velocity =
            computeLinearVelocity(reference_velocity_, curvature, heading_error, lateral_error);
        const double angular_velocity =
            computeAngularVelocity(linear_velocity, curvature, heading_error, lateral_error, dt);

        return makeCommand(linear_velocity, angular_velocity);
    }

    case ControlState::CloseEnd: {
        const double dx = end_point_.x - robot_state.x;
        const double dy = end_point_.y - robot_state.y;
        if (std::hypot(dx, dy) <= goal_position_tolerance_) {
            state_ = ControlState::GoalYawAdjust;
            ROS_INFO("GoalYawAdjust.............");
            reset_PID();
            return makeCommand(0.0, 0.0);
        }
        return computeGoalPositionPID(robot_state, end_point_, dt);
    }

    case ControlState::GoalYawAdjust: {
        const double yaw_error = getTheta(end_point_.yaw, robot_state.yaw);
        if (std::fabs(yaw_error) <= goal_yaw_tolerance_) {
            state_ = ControlState::Finished;
            ROS_INFO("\033[32mFinished\033[0m");
            reset_PID();
            return makeCommand(0.0, 0.0);
        }

        return makeCommand(0.0, StartORGoalYawPID(yaw_error, dt));
    }
    case ControlState::Finished:
        return makeCommand(0.0, 0.0);
    }
    return makeCommand(0.0, 0.0);
}

ControlCommand PurePursuit::makeCommand(double linear_velocity, double angular_velocity) const
{
    ControlCommand command;
    command.state = state_;
    command.linear_velocity = linear_velocity;
    command.angular_velocity = angular_velocity;
    return command;
}
