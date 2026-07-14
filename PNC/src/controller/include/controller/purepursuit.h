#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "controller/state.h"



class PurePursuit
{
private:
    nav_msgs::Path path_;
    int last_nearest_index_ = 0;
    PathPoint start_point_ , end_point_;

    double kp_yaw_ = 0.0;
    double kd_yaw_ = 0.0;
    double kp_pos_ = 0.0;
    double end_yaw_kp_ = 0.0;
    double end_yaw_ki_ = 0.0;
    double end_yaw_kd_ = 0.0;
    double end_pos_kp_ = 0.0;
    double end_pos_ki_ = 0.0;
    double end_pos_kd_ = 0.0;

    double lookahead_distance_ = 0.0;
    double reference_velocity_ = 0.0;

    double curvature_velocity_gain_ = 1.2;
    double lateral_velocity_gain_ = 0.5;
    double rotate_in_place_threshold_ = 1.0;
    double derivative_filter_lambda_ = 0.8;
    double goal_position_tolerance_ = 0.05;
    double goal_yaw_tolerance_ = 0.05;
    double goal_slowdown_distance_ = 0.4;

    double last_heading_error_ = 0.0;
    double filtered_heading_error_rate_ = 0.0;
    double goal_pos_integral_ = 0.0;
    double goal_pos_last_error_ = 0.0;
    double goal_yaw_integral_ = 0.0;
    double goal_yaw_last_error_ = 0.0;

    ros::Time last_control_time_;
    bool heading_pid_initialized_ = false;
    bool goal_yaw_pid_initialized_ = false;

    ControlState state_ = ControlState::Init;

    void param_init(const ros::NodeHandle& nh);
    void reset_PID();
    double getTheta(double ref_theta, double vehicleYaw);

    double computeLateralError(const RobotState& robot, const PathPoint& reference) const;
    double computeLinearVelocity(double reference_velocity,
                                 double curvature,
                                 double heading_error,
                                 double lateral_error) const;
    double computeAngularVelocity(double linear_velocity,
                                  double curvature,
                                  double heading_error,
                                  double lateral_error,
                                  double dt);
    ControlCommand makeCommand(double linear_velocity, double angular_velocity) const;
    ControlCommand computeGoalPositionPID(const RobotState& robot, const PathPoint& goal, double dt);
    double StartORGoalYawPID(double yaw_error, double dt);

public:
    explicit PurePursuit(const ros::NodeHandle& nh);
    ~PurePursuit() = default;
    bool reset(const nav_msgs::Path& path);
    ControlCommand PurePursuitbyYaw(const RobotState& robot_state);
};
