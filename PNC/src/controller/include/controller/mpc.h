#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <robot_trajectory_msgs/RobotTrajectory.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <string>
#include <vector>

#include "controller/state.h"

struct MpcReferencePoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double a = 0.0;
    double w = 0.0;
    double steering_angle = 0.0;
    double time_from_start = 0.0;
};

struct MpcQpData
{
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> constraint_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
};

class MpcController
{
public:
    explicit MpcController(const ros::NodeHandle& nh);
    ~MpcController() = default;

    bool reset(const robot_trajectory_msgs::RobotTrajectory& trajectory);
    ControlCommand computeCommand(const RobotState& robot_state);

private:
    void loadParams(const ros::NodeHandle& nh);
    int findNearestIndex(const RobotState& robot_state);
    bool buildReferenceWindow(int nearest_index,
                              std::vector<MpcReferencePoint>& reference_window) const;
    Eigen::Matrix<double, 4, 1> buildCurrentState(const RobotState& robot_state) const;
    Eigen::Matrix<double, 4, 1> buildStateError(
        const Eigen::Matrix<double, 4, 1>& current_state,
        const MpcReferencePoint& reference) const;
    void buildLinearizedModel(
        const std::vector<MpcReferencePoint>& reference_window,
        std::vector<Eigen::Matrix4d>& a_matrices,
        std::vector<Eigen::Matrix<double, 4, 2>>& b_matrices) const;
    bool buildQpProblem(
        const Eigen::Matrix<double, 4, 1>& initial_error,
        const std::vector<MpcReferencePoint>& reference_window,
        const std::vector<Eigen::Matrix4d>& a_matrices,
        const std::vector<Eigen::Matrix<double, 4, 2>>& b_matrices,
        MpcQpData& qp_data) const;
    bool solveQp(MpcQpData& qp_data,
                 Eigen::Matrix<double, 2, 1>& control_delta,
                 Eigen::VectorXd& solution) const;
    void publishPredictedPath(
        const std::vector<MpcReferencePoint>& reference_window,
        const Eigen::VectorXd& solution) const;
    void printReferenceWindowDebug(
        int nearest_index,
        const Eigen::Matrix<double, 4, 1>& initial_error,
        const std::vector<MpcReferencePoint>& reference_window) const;
    int stateIndex(int step, int state_offset) const;
    int controlIndex(int step, int control_offset) const;
    Eigen::Matrix4d buildAMatrix(const MpcReferencePoint& reference) const;
    Eigen::Matrix<double, 4, 2> buildBMatrix(const MpcReferencePoint& reference) const;
    MpcReferencePoint convertTrajectoryPoint(
        const robot_trajectory_msgs::RobotTrajectoryPoint& point) const;
    double getTrajectorySampleInterval() const;
    double projectLongitudinal(double x, double y, double yaw) const;
    double normalizeAngle(double angle) const;

    robot_trajectory_msgs::RobotTrajectory trajectory_;
    PathPoint start_point_;
    PathPoint goal_point_;
    int last_nearest_index_ = 0;

    double prediction_dt_ = 0.1;
    int horizon_ = 10;
    double q_x_ = 1.0;
    double q_y_ = 1.0;
    double q_yaw_ = 0.5;
    double q_v_ = 0.1;
    double r_a_ = 0.1;
    double r_delta_ = 0.2;
    double rd_a_ = 0.5;
    double rd_delta_ = 1.0;
    double min_velocity_ = 0.0;
    double max_velocity_ = 0.7;
    double min_acceleration_ = -0.5;
    double max_acceleration_ = 0.5;
    double min_angular_velocity_ = -0.3;
    double max_angular_velocity_ = 0.3;
    double min_acceleration_delta_ = -0.1;
    double max_acceleration_delta_ = 0.1;
    double wheel_base_ = 1.0;
    double min_front_steering_angle_ = -0.436332;
    double max_front_steering_angle_ = 0.436332;
    double min_front_steering_angle_delta_ = -0.1;
    double max_front_steering_angle_delta_ = 0.1;
    double min_steering_conversion_speed_ = 0.02;
    double goal_position_tolerance_ = 0.05;
    double goal_yaw_tolerance_ = 0.08;
    double goal_yaw_adjust_kp_ = 0.8;
    double goal_yaw_adjust_max_angular_velocity_ = 0.3;
    double goal_yaw_adjust_min_angular_velocity_ = 0.03;
    int osqp_max_iteration_ = 4000;
    double osqp_absolute_tolerance_ = 1.0e-4;
    double osqp_relative_tolerance_ = 1.0e-4;
    bool constrain_velocity_ = false;
    bool constrain_control_rate_ = false;
    bool limit_reference_dynamics_ = true;
    bool resample_reference_by_arc_length_ = true;
    bool publish_predicted_path_ = true;
    std::string predicted_path_topic_ = "/controller/mpc_predicted_path";
    std::string predicted_path_frame_id_ = "map";
    bool debug_reference_ = false;
    bool debug_model_ = false;
    bool has_trajectory_ = false;
    bool start_yaw_adjusted_ = false;
    ros::Publisher predicted_path_pub_;
    ControlState state_ = ControlState::None;
};
