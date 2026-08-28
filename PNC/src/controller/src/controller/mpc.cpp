#include "controller/mpc.h"

#include <OsqpEigen/OsqpEigen.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr int kStateDim = 4;
constexpr int kControlDim = 2;
constexpr double kInfinity = 1.0e10;

double clampValue(const double value, const double lower, const double upper)
{
    return std::max(lower, std::min(value, upper));
}
}

MpcController::MpcController(const ros::NodeHandle& nh)
{
    loadParams(nh);
    if (publish_predicted_path_) {
        ros::NodeHandle advertise_nh(nh);
        predicted_path_pub_ =
            advertise_nh.advertise<nav_msgs::Path>(predicted_path_topic_, 1, false);
    }
}

void MpcController::loadParams(const ros::NodeHandle& nh)
{
    nh.param("mpc_node/prediction_dt", prediction_dt_, 0.1);
    nh.param("mpc_node/horizon", horizon_, 10);
    nh.param("mpc_node/q_x", q_x_, 1.0);
    nh.param("mpc_node/q_y", q_y_, 1.0);
    nh.param("mpc_node/q_yaw", q_yaw_, 0.5);
    nh.param("mpc_node/q_v", q_v_, 0.1);
    nh.param("mpc_node/r_a", r_a_, 0.1);
    nh.param("mpc_node/r_delta", r_delta_, 0.2);
    nh.param("mpc_node/rd_a", rd_a_, 0.5);
    nh.param("mpc_node/rd_delta", rd_delta_, 1.0);
    nh.param("mpc_node/min_velocity", min_velocity_, 0.0);
    nh.param("mpc_node/max_velocity", max_velocity_, 0.7);
    nh.param("mpc_node/min_acceleration", min_acceleration_, -0.5);
    nh.param("mpc_node/max_acceleration", max_acceleration_, 0.5);
    nh.param("mpc_node/min_angular_velocity", min_angular_velocity_, -0.3);
    nh.param("mpc_node/max_angular_velocity", max_angular_velocity_, 0.3);
    nh.param("mpc_node/min_acceleration_delta", min_acceleration_delta_, -0.1);
    nh.param("mpc_node/max_acceleration_delta", max_acceleration_delta_, 0.1);
    nh.param("mpc_node/wheel_base", wheel_base_, 1.0);
    nh.param("mpc_node/min_front_steering_angle", min_front_steering_angle_, -0.436332);
    nh.param("mpc_node/max_front_steering_angle", max_front_steering_angle_, 0.436332);
    nh.param("mpc_node/min_front_steering_angle_delta",
             min_front_steering_angle_delta_, -0.1);
    nh.param("mpc_node/max_front_steering_angle_delta",
             max_front_steering_angle_delta_, 0.1);
    nh.param("mpc_node/min_steering_conversion_speed",
             min_steering_conversion_speed_, 0.02);
    nh.param("mpc_node/goal_position_tolerance", goal_position_tolerance_, 0.05);
    nh.param("mpc_node/goal_yaw_tolerance", goal_yaw_tolerance_, 0.08);
    nh.param("mpc_node/goal_yaw_adjust_kp", goal_yaw_adjust_kp_, 0.8);
    nh.param("mpc_node/goal_yaw_adjust_max_angular_velocity",
             goal_yaw_adjust_max_angular_velocity_, 0.3);
    nh.param("mpc_node/goal_yaw_adjust_min_angular_velocity",
             goal_yaw_adjust_min_angular_velocity_, 0.03);
    nh.param("mpc_node/osqp_max_iteration", osqp_max_iteration_, 4000);
    nh.param("mpc_node/osqp_absolute_tolerance", osqp_absolute_tolerance_, 1.0e-4);
    nh.param("mpc_node/osqp_relative_tolerance", osqp_relative_tolerance_, 1.0e-4);
    nh.param("mpc_node/constrain_velocity", constrain_velocity_, false);
    nh.param("mpc_node/constrain_control_rate", constrain_control_rate_, false);
    nh.param("mpc_node/limit_reference_dynamics", limit_reference_dynamics_, true);
    nh.param("mpc_node/resample_reference_by_arc_length", resample_reference_by_arc_length_, true);
    nh.param("mpc_node/publish_predicted_path", publish_predicted_path_, true);
    nh.param("mpc_node/predicted_path_topic",
             predicted_path_topic_, std::string("/controller/mpc_predicted_path"));
    nh.param("mpc_node/predicted_path_frame_id",
             predicted_path_frame_id_, std::string("map"));
    nh.param("mpc_node/debug_reference", debug_reference_, false);
    nh.param("mpc_node/debug_model", debug_model_, false);

    if (prediction_dt_ <= 0.0) {
        ROS_WARN("mpc_node/prediction_dt <= 0, use 0.1s");
        prediction_dt_ = 0.1;
    }
    if (horizon_ <= 0) {
        ROS_WARN("mpc_node/horizon <= 0, use 10");
        horizon_ = 10;
    }
    if (osqp_max_iteration_ <= 0) {
        ROS_WARN("mpc_node/osqp_max_iteration <= 0, use 4000");
        osqp_max_iteration_ = 4000;
    }
    if (wheel_base_ <= 0.0) {
        ROS_WARN("mpc_node/wheel_base <= 0, use 1.0m");
        wheel_base_ = 1.0;
    }
    if (min_front_steering_angle_ > max_front_steering_angle_) {
        ROS_WARN("mpc_node front steering angle bounds invalid, swap bounds");
        std::swap(min_front_steering_angle_, max_front_steering_angle_);
    }
    if (min_front_steering_angle_delta_ > max_front_steering_angle_delta_) {
        ROS_WARN("mpc_node front steering angle delta bounds invalid, swap bounds");
        std::swap(min_front_steering_angle_delta_, max_front_steering_angle_delta_);
    }
    if (min_acceleration_delta_ > max_acceleration_delta_) {
        ROS_WARN("mpc_node acceleration delta bounds invalid, swap bounds");
        std::swap(min_acceleration_delta_, max_acceleration_delta_);
    }
    if (min_steering_conversion_speed_ < 0.0) {
        min_steering_conversion_speed_ = -min_steering_conversion_speed_;
    }
    if (goal_yaw_adjust_kp_ < 0.0) {
        ROS_WARN("mpc_node/goal_yaw_adjust_kp < 0, use 0.8");
        goal_yaw_adjust_kp_ = 0.8;
    }
    if (goal_yaw_adjust_max_angular_velocity_ < 0.0) {
        goal_yaw_adjust_max_angular_velocity_ = -goal_yaw_adjust_max_angular_velocity_;
    }
    goal_yaw_adjust_max_angular_velocity_ = std::min(
        goal_yaw_adjust_max_angular_velocity_, max_angular_velocity_);
    if (goal_yaw_adjust_min_angular_velocity_ < 0.0) {
        goal_yaw_adjust_min_angular_velocity_ = -goal_yaw_adjust_min_angular_velocity_;
    }
    goal_yaw_adjust_min_angular_velocity_ = std::min(
        goal_yaw_adjust_min_angular_velocity_, goal_yaw_adjust_max_angular_velocity_);
}

bool MpcController::reset(const robot_trajectory_msgs::RobotTrajectory& trajectory)
{
    if (trajectory.points.empty()) {
        ROS_ERROR("MPC trajectory is empty");
        has_trajectory_ = false;
        state_ = ControlState::None;
        return false;
    }

    trajectory_ = trajectory;
    const auto& start_pose = trajectory_.points.front().pose;
    const auto& goal_pose = trajectory_.points.back().pose;
    start_point_.x = start_pose.position.x;
    start_point_.y = start_pose.position.y;
    start_point_.yaw = tf2::getYaw(start_pose.orientation);
    goal_point_.x = goal_pose.position.x;
    goal_point_.y = goal_pose.position.y;
    goal_point_.yaw = tf2::getYaw(goal_pose.orientation);

    last_nearest_index_ = 0;
    has_trajectory_ = true;
    start_yaw_adjusted_ = false;
    state_ = ControlState::GoalYawAdjust;
    ROS_INFO("MPC trajectory received, points=%lu, sample_interval=%.4f, prediction_dt=%.4f, horizon=%d",
             static_cast<unsigned long>(trajectory_.points.size()),
             getTrajectorySampleInterval(), prediction_dt_, horizon_);
    return true;
}

ControlCommand MpcController::computeCommand(const RobotState& robot_state)
{
    ControlCommand command;
    command.linear_velocity = 0.0;
    command.angular_velocity = 0.0;

    if (!has_trajectory_) {
        command.state = ControlState::None;
        return command;
    }

    const double dx = goal_point_.x - robot_state.x;
    const double dy = goal_point_.y - robot_state.y;
    const double goal_distance = std::hypot(dx, dy);
    const double yaw_error = normalizeAngle(goal_point_.yaw - robot_state.yaw);
    const double start_yaw_error = normalizeAngle(start_point_.yaw - robot_state.yaw);

    if (!start_yaw_adjusted_) {
        if (std::fabs(start_yaw_error) <= goal_yaw_tolerance_) {
            start_yaw_adjusted_ = true;
            state_ = ControlState::Tracking;
        } else {
            state_ = ControlState::GoalYawAdjust;
            double angular_velocity_cmd = clampValue(
                goal_yaw_adjust_kp_ * start_yaw_error,
                -goal_yaw_adjust_max_angular_velocity_,
                goal_yaw_adjust_max_angular_velocity_);
            if (std::fabs(angular_velocity_cmd) < goal_yaw_adjust_min_angular_velocity_) {
                angular_velocity_cmd = std::copysign(
                    goal_yaw_adjust_min_angular_velocity_, start_yaw_error);
            }
            command.linear_velocity = 0.0;
            command.angular_velocity = angular_velocity_cmd;
            command.state = state_;
            ROS_INFO_THROTTLE(
                1.0,
                "MPC start yaw adjust: yaw_error=%.3f cmd_w=%.3f",
                start_yaw_error, command.angular_velocity);
            return command;
        }
    }

    if (goal_distance <= goal_position_tolerance_) {
        state_ = std::fabs(yaw_error) <= goal_yaw_tolerance_
            ? ControlState::Finished : ControlState::GoalYawAdjust;
    } else if (state_ == ControlState::None || state_ == ControlState::Finished
               || state_ == ControlState::GoalYawAdjust) {
        state_ = ControlState::Tracking;
    }

    if (state_ == ControlState::GoalYawAdjust) {
        double angular_velocity_cmd = clampValue(
            goal_yaw_adjust_kp_ * yaw_error,
            -goal_yaw_adjust_max_angular_velocity_,
            goal_yaw_adjust_max_angular_velocity_);
        if (std::fabs(angular_velocity_cmd) < goal_yaw_adjust_min_angular_velocity_
            && std::fabs(yaw_error) > goal_yaw_tolerance_) {
            angular_velocity_cmd = std::copysign(
                goal_yaw_adjust_min_angular_velocity_, yaw_error);
        }
        command.linear_velocity = 0.0;
        command.angular_velocity = angular_velocity_cmd;
        ROS_INFO_THROTTLE(
            1.0,
            "MPC goal yaw adjust: distance=%.3f yaw_error=%.3f cmd_w=%.3f",
            goal_distance, yaw_error, command.angular_velocity);
    } else if (state_ == ControlState::Tracking) {
        const int nearest_index = findNearestIndex(robot_state);
        std::vector<MpcReferencePoint> reference_window;
        if (!buildReferenceWindow(nearest_index, reference_window)) {
            ROS_WARN_THROTTLE(1.0, "Failed to build MPC reference window");
            command.state = ControlState::None;
            return command;
        }

        const Eigen::Matrix<double, 4, 1> current_state =
            buildCurrentState(robot_state);
        const Eigen::Matrix<double, 4, 1> initial_error =
            buildStateError(current_state, reference_window.front());
        std::vector<Eigen::Matrix4d> a_matrices;
        std::vector<Eigen::Matrix<double, 4, 2>> b_matrices;
        buildLinearizedModel(reference_window, a_matrices, b_matrices);
        MpcQpData qp_data;
        if (!buildQpProblem(initial_error, reference_window, a_matrices, b_matrices, qp_data)) {
            ROS_WARN_THROTTLE(1.0, "Failed to build MPC QP problem");
            command.state = ControlState::None;
            return command;
        }

        Eigen::Matrix<double, 2, 1> control_delta =
            Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::VectorXd solution;
        if (!solveQp(qp_data, control_delta, solution)) {
            printReferenceWindowDebug(nearest_index, initial_error, reference_window);
            ROS_WARN_THROTTLE(1.0, "Failed to solve MPC QP problem");
            command.state = state_;
            return command;
        }
        publishPredictedPath(reference_window, solution);

        const auto& ref0 = reference_window.front();
        const auto& ref1 = reference_window.size() > 1 ? reference_window[1] : ref0;
        const double raw_acceleration_cmd = ref0.a + control_delta(0);
        const double raw_steering_angle_cmd = ref0.steering_angle + control_delta(1);
        const double raw_linear_velocity_cmd = ref1.v + solution(stateIndex(1, 3));
        const double acceleration_cmd = clampValue(
            raw_acceleration_cmd, min_acceleration_, max_acceleration_);
        const double linear_velocity_cmd = clampValue(
            raw_linear_velocity_cmd, min_velocity_, max_velocity_);
        const double steering_angle_cmd = clampValue(
            raw_steering_angle_cmd,
            min_front_steering_angle_,
            max_front_steering_angle_);
        const double angular_velocity_cmd = clampValue(
            linear_velocity_cmd / wheel_base_ * std::tan(steering_angle_cmd),
            min_angular_velocity_,
            max_angular_velocity_);

        command.linear_velocity = linear_velocity_cmd;
        command.angular_velocity = angular_velocity_cmd;

        if (std::fabs(raw_acceleration_cmd - acceleration_cmd) > 1.0e-4
            || std::fabs(raw_linear_velocity_cmd - linear_velocity_cmd) > 1.0e-4
            || std::fabs(raw_steering_angle_cmd - steering_angle_cmd) > 1.0e-4) {
            ROS_WARN_THROTTLE(
                1.0,
                "MPC command saturated: raw_a=%.3f limited_a=%.3f raw_v=%.3f limited_v=%.3f raw_steer=%.3f limited_steer=%.3f",
                raw_acceleration_cmd, acceleration_cmd,
                raw_linear_velocity_cmd, linear_velocity_cmd,
                raw_steering_angle_cmd, steering_angle_cmd);
        }

        if (debug_reference_ && !reference_window.empty()) {
            printReferenceWindowDebug(nearest_index, initial_error, reference_window);
        }
        if (debug_model_ && !a_matrices.empty() && !b_matrices.empty()) {
            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "MPC e0=[" << initial_error.transpose() << "]\n"
                << "A0=\n" << a_matrices.front() << "\n"
                << "B0=\n" << b_matrices.front() << "\n"
                << "QP vars=" << qp_data.gradient.size()
                << " constraints=" << qp_data.lower_bound.size()
                << " H_nnz=" << qp_data.hessian.nonZeros()
                << " C_nnz=" << qp_data.constraint_matrix.nonZeros()
                << "\n"
                << "du0=[" << control_delta.transpose() << "]"
                << " cmd_a=" << acceleration_cmd
                << " cmd_steer=" << steering_angle_cmd
                << " cmd_v=" << command.linear_velocity
                << " cmd_w=" << command.angular_velocity);
        }
    }

    command.state = state_;
    return command;
}

int MpcController::findNearestIndex(const RobotState& robot_state)
{
    if (trajectory_.points.empty()) {
        return 0;
    }

    const int last_index = static_cast<int>(trajectory_.points.size()) - 1;
    const int search_begin = std::max(0, last_nearest_index_ - 5);
    const int search_end = std::min(last_index, last_nearest_index_ + 500);

    int nearest_index = last_nearest_index_;
    double nearest_distance_sq = std::numeric_limits<double>::max();
    for (int i = search_begin; i <= search_end; ++i) {
        const auto& position = trajectory_.points[i].pose.position;
        const double dx = position.x - robot_state.x;
        const double dy = position.y - robot_state.y;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq) {
            nearest_distance_sq = distance_sq;
            nearest_index = i;
        }
    }

    nearest_index = std::max(nearest_index, last_nearest_index_);
    last_nearest_index_ = nearest_index;
    return nearest_index;
}

bool MpcController::buildReferenceWindow(
    const int nearest_index,
    std::vector<MpcReferencePoint>& reference_window) const
{
    if (trajectory_.points.empty() || horizon_ <= 0) {
        return false;
    }

    const int last_index = static_cast<int>(trajectory_.points.size()) - 1;

    reference_window.clear();
    reference_window.reserve(static_cast<size_t>(horizon_ + 1));

    if (resample_reference_by_arc_length_
        && trajectory_.points.back().arc_length > trajectory_.points.front().arc_length) {
        const double step_distance =
            std::max(0.01, std::max(std::fabs(min_velocity_), std::fabs(max_velocity_))
                           * prediction_dt_);
        const double start_arc_length = trajectory_.points[nearest_index].arc_length;
        int search_index = nearest_index;
        for (int k = 0; k <= horizon_; ++k) {
            const double target_arc_length = start_arc_length + k * step_distance;
            while (search_index < last_index
                   && trajectory_.points[search_index].arc_length < target_arc_length) {
                ++search_index;
            }
            reference_window.push_back(convertTrajectoryPoint(trajectory_.points[search_index]));
        }
        return reference_window.size() == static_cast<size_t>(horizon_ + 1);
    }

    const double sample_interval = getTrajectorySampleInterval();
    const int index_step = std::max(
        1, static_cast<int>(std::round(prediction_dt_ / sample_interval)));
    for (int k = 0; k <= horizon_; ++k) {
        const int index = std::min(last_index, nearest_index + k * index_step);
        reference_window.push_back(convertTrajectoryPoint(trajectory_.points[index]));
    }
    return reference_window.size() == static_cast<size_t>(horizon_ + 1);
}

Eigen::Matrix<double, 4, 1> MpcController::buildCurrentState(
    const RobotState& robot_state) const
{
    Eigen::Matrix<double, 4, 1> state;
    state << robot_state.x,
             robot_state.y,
             robot_state.yaw,
             robot_state.linear_velocity;
    return state;
}

Eigen::Matrix<double, 4, 1> MpcController::buildStateError(
    const Eigen::Matrix<double, 4, 1>& current_state,
    const MpcReferencePoint& reference) const
{
    Eigen::Matrix<double, 4, 1> error;
    error << current_state(0) - reference.x,
             current_state(1) - reference.y,
             normalizeAngle(current_state(2) - reference.yaw),
             current_state(3) - reference.v;
    return error;
}

void MpcController::buildLinearizedModel(
    const std::vector<MpcReferencePoint>& reference_window,
    std::vector<Eigen::Matrix4d>& a_matrices,
    std::vector<Eigen::Matrix<double, 4, 2>>& b_matrices) const
{
    a_matrices.clear();
    b_matrices.clear();
    if (reference_window.size() < 2) {
        return;
    }

    const size_t model_count = reference_window.size() - 1;
    a_matrices.reserve(model_count);
    b_matrices.reserve(model_count);
    for (size_t i = 0; i < model_count; ++i) {
        a_matrices.push_back(buildAMatrix(reference_window[i]));
        b_matrices.push_back(buildBMatrix(reference_window[i]));
    }
}

bool MpcController::buildQpProblem(
    const Eigen::Matrix<double, 4, 1>& initial_error,
    const std::vector<MpcReferencePoint>& reference_window,
    const std::vector<Eigen::Matrix4d>& a_matrices,
    const std::vector<Eigen::Matrix<double, 4, 2>>& b_matrices,
    MpcQpData& qp_data) const
{
    const int state_count = horizon_ + 1;
    const int control_count = horizon_;
    if (static_cast<int>(reference_window.size()) != state_count
        || static_cast<int>(a_matrices.size()) != control_count
        || static_cast<int>(b_matrices.size()) != control_count) {
        return false;
    }

    const int variable_count = kStateDim * state_count + kControlDim * control_count;
    const int equality_count = kStateDim * state_count;
    const int velocity_bound_count = control_count;
    const int control_bound_count = kControlDim * control_count;
    const int rate_bound_count = constrain_control_rate_
        ? kControlDim * std::max(0, control_count - 1) : 0;
    const int constraint_count =
        equality_count + velocity_bound_count + control_bound_count + rate_bound_count;

    qp_data.gradient = Eigen::VectorXd::Zero(variable_count);
    qp_data.lower_bound = Eigen::VectorXd::Constant(constraint_count, -kInfinity);
    qp_data.upper_bound = Eigen::VectorXd::Constant(constraint_count, kInfinity);

    std::vector<Eigen::Triplet<double>> hessian_triplets;
    hessian_triplets.reserve(static_cast<size_t>(
        kStateDim * state_count + kControlDim * control_count + 4 * rate_bound_count));

    const double state_weights[kStateDim] = {q_x_, q_y_, q_yaw_, q_v_};
    for (int k = 0; k < state_count; ++k) {
        for (int i = 0; i < kStateDim; ++i) {
            hessian_triplets.emplace_back(stateIndex(k, i), stateIndex(k, i),
                                          2.0 * state_weights[i]);
        }
    }

    const double control_weights[kControlDim] = {r_a_, r_delta_};
    for (int k = 0; k < control_count; ++k) {
        for (int i = 0; i < kControlDim; ++i) {
            hessian_triplets.emplace_back(controlIndex(k, i), controlIndex(k, i),
                                          2.0 * control_weights[i]);
        }
    }

    const double rate_weights[kControlDim] = {rd_a_, rd_delta_};
    for (int k = 0; k + 1 < control_count; ++k) {
        for (int i = 0; i < kControlDim; ++i) {
            const int current = controlIndex(k, i);
            const int next = controlIndex(k + 1, i);
            const double weight = 2.0 * rate_weights[i];
            hessian_triplets.emplace_back(current, current, weight);
            hessian_triplets.emplace_back(next, next, weight);
            hessian_triplets.emplace_back(current, next, -weight);
            hessian_triplets.emplace_back(next, current, -weight);
        }
    }

    qp_data.hessian.resize(variable_count, variable_count);
    qp_data.hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());
    qp_data.hessian.makeCompressed();

    std::vector<Eigen::Triplet<double>> constraint_triplets;
    constraint_triplets.reserve(static_cast<size_t>(
        kStateDim + control_count * (kStateDim * (1 + kStateDim + kControlDim))
        + velocity_bound_count
        + control_bound_count + rate_bound_count * 2));

    int row = 0;
    for (int i = 0; i < kStateDim; ++i) {
        constraint_triplets.emplace_back(row, stateIndex(0, i), 1.0);
        qp_data.lower_bound(row) = initial_error(i);
        qp_data.upper_bound(row) = initial_error(i);
        ++row;
    }

    for (int k = 0; k < control_count; ++k) {
        for (int i = 0; i < kStateDim; ++i) {
            constraint_triplets.emplace_back(row, stateIndex(k + 1, i), 1.0);
            for (int j = 0; j < kStateDim; ++j) {
                const double value = -a_matrices[k](i, j);
                if (std::fabs(value) > 1.0e-12) {
                    constraint_triplets.emplace_back(row, stateIndex(k, j), value);
                }
            }
            for (int j = 0; j < kControlDim; ++j) {
                const double value = -b_matrices[k](i, j);
                if (std::fabs(value) > 1.0e-12) {
                    constraint_triplets.emplace_back(row, controlIndex(k, j), value);
                }
            }
            qp_data.lower_bound(row) = 0.0;
            qp_data.upper_bound(row) = 0.0;
            ++row;
        }
    }

    for (int k = 1; k < state_count; ++k) {
        constraint_triplets.emplace_back(row, stateIndex(k, 3), 1.0);
        qp_data.lower_bound(row) = min_velocity_ - reference_window[k].v;
        qp_data.upper_bound(row) = max_velocity_ - reference_window[k].v;
        ++row;
    }

    for (int k = 0; k < control_count; ++k) {
        constraint_triplets.emplace_back(row, controlIndex(k, 0), 1.0);
        qp_data.lower_bound(row) = min_acceleration_ - reference_window[k].a;
        qp_data.upper_bound(row) = max_acceleration_ - reference_window[k].a;
        ++row;

        constraint_triplets.emplace_back(row, controlIndex(k, 1), 1.0);
        qp_data.lower_bound(row) =
            min_front_steering_angle_ - reference_window[k].steering_angle;
        qp_data.upper_bound(row) =
            max_front_steering_angle_ - reference_window[k].steering_angle;
        ++row;
    }

    if (constrain_control_rate_) {
        for (int k = 0; k + 1 < control_count; ++k) {
            for (int i = 0; i < kControlDim; ++i) {
                constraint_triplets.emplace_back(row, controlIndex(k + 1, i), 1.0);
                constraint_triplets.emplace_back(row, controlIndex(k, i), -1.0);

                const double ref_delta = (i == 0)
                    ? reference_window[k + 1].a - reference_window[k].a
                    : reference_window[k + 1].steering_angle
                          - reference_window[k].steering_angle;
                const double lower_delta = (i == 0)
                    ? min_acceleration_delta_ : min_front_steering_angle_delta_;
                const double upper_delta = (i == 0)
                    ? max_acceleration_delta_ : max_front_steering_angle_delta_;
                qp_data.lower_bound(row) = lower_delta - ref_delta;
                qp_data.upper_bound(row) = upper_delta - ref_delta;
                ++row;
            }
        }
    }

    qp_data.constraint_matrix.resize(constraint_count, variable_count);
    qp_data.constraint_matrix.setFromTriplets(
        constraint_triplets.begin(), constraint_triplets.end());
    qp_data.constraint_matrix.makeCompressed();

    return row == constraint_count;
}

bool MpcController::solveQp(
    MpcQpData& qp_data,
    Eigen::Matrix<double, 2, 1>& control_delta,
    Eigen::VectorXd& solution) const
{
    const int variable_count = static_cast<int>(qp_data.gradient.size());
    const int constraint_count = static_cast<int>(qp_data.lower_bound.size());
    if (variable_count <= 0 || constraint_count <= 0
        || qp_data.upper_bound.size() != constraint_count
        || qp_data.constraint_matrix.rows() != constraint_count
        || qp_data.constraint_matrix.cols() != variable_count) {
        return false;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setMaxIteration(osqp_max_iteration_);
    solver.settings()->setAbsoluteTolerance(osqp_absolute_tolerance_);
    solver.settings()->setRelativeTolerance(osqp_relative_tolerance_);

    solver.data()->setNumberOfVariables(variable_count);
    solver.data()->setNumberOfConstraints(constraint_count);
    if (!solver.data()->setHessianMatrix(qp_data.hessian)
        || !solver.data()->setGradient(qp_data.gradient)
        || !solver.data()->setLinearConstraintsMatrix(qp_data.constraint_matrix)
        || !solver.data()->setLowerBound(qp_data.lower_bound)
        || !solver.data()->setUpperBound(qp_data.upper_bound)) {
        return false;
    }

    if (!solver.initSolver()) {
        return false;
    }

    const OsqpEigen::ErrorExitFlag error_flag = solver.solveProblem();
    const OsqpEigen::Status status = solver.getStatus();
    if (error_flag != OsqpEigen::ErrorExitFlag::NoError
        || (status != OsqpEigen::Status::Solved
            && status != OsqpEigen::Status::SolvedInaccurate)) {
        ROS_WARN_THROTTLE(1.0, "OSQP failed, error=%d, status=%d",
                          static_cast<int>(error_flag), static_cast<int>(status));
        return false;
    }

    solution = solver.getSolution();
    if (solution.size() != variable_count) {
        return false;
    }

    control_delta(0) = solution(controlIndex(0, 0));
    control_delta(1) = solution(controlIndex(0, 1));
    return std::isfinite(control_delta(0))
        && std::isfinite(control_delta(1));
}

void MpcController::publishPredictedPath(
    const std::vector<MpcReferencePoint>& reference_window,
    const Eigen::VectorXd& solution) const
{
    if (!publish_predicted_path_ || !predicted_path_pub_ || reference_window.empty()) {
        return;
    }

    const int state_count = horizon_ + 1;
    if (solution.size() < kStateDim * state_count) {
        return;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = predicted_path_frame_id_;
    path_msg.poses.reserve(static_cast<size_t>(state_count));

    for (int k = 0; k < state_count; ++k) {
        const MpcReferencePoint& reference = reference_window[k];
        const double x = reference.x + solution(stateIndex(k, 0));
        const double y = reference.y + solution(stateIndex(k, 1));
        const double yaw = normalizeAngle(reference.yaw + solution(stateIndex(k, 2)));

        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.05;
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
        path_msg.poses.push_back(pose);
    }

    predicted_path_pub_.publish(path_msg);
}

void MpcController::printReferenceWindowDebug(
    const int nearest_index,
    const Eigen::Matrix<double, 4, 1>& initial_error,
    const std::vector<MpcReferencePoint>& reference_window) const
{
    if (reference_window.empty()) {
        return;
    }

    double min_v = std::numeric_limits<double>::max();
    double max_v = -std::numeric_limits<double>::max();
    double min_a = std::numeric_limits<double>::max();
    double max_a = -std::numeric_limits<double>::max();
    double min_w = std::numeric_limits<double>::max();
    double max_w = -std::numeric_limits<double>::max();
    double min_steer = std::numeric_limits<double>::max();
    double max_steer = -std::numeric_limits<double>::max();
    for (const auto& reference : reference_window) {
        min_v = std::min(min_v, reference.v);
        max_v = std::max(max_v, reference.v);
        min_a = std::min(min_a, reference.a);
        max_a = std::max(max_a, reference.a);
        min_w = std::min(min_w, reference.w);
        max_w = std::max(max_w, reference.w);
        min_steer = std::min(min_steer, reference.steering_angle);
        max_steer = std::max(max_steer, reference.steering_angle);
    }

    const auto& ref0 = reference_window.front();
    ROS_WARN_THROTTLE(
        1.0,
        "MPC debug nearest=%d e0=[%.3f %.3f %.3f %.3f] "
        "ref0=[x %.3f y %.3f yaw %.3f v %.3f a %.3f w %.3f steer %.3f] "
        "ref_range v[%.3f, %.3f] a[%.3f, %.3f] w[%.3f, %.3f] steer[%.3f, %.3f] "
        "bounds v[%.3f, %.3f] a[%.3f, %.3f] w[%.3f, %.3f] steer[%.3f, %.3f] "
        "hard_velocity=%d hard_rate=%d",
        nearest_index,
        initial_error(0), initial_error(1), initial_error(2), initial_error(3),
        ref0.x, ref0.y, ref0.yaw, ref0.v, ref0.a, ref0.w, ref0.steering_angle,
        min_v, max_v, min_a, max_a, min_w, max_w, min_steer, max_steer,
        min_velocity_, max_velocity_, min_acceleration_, max_acceleration_,
        min_angular_velocity_, max_angular_velocity_,
        min_front_steering_angle_, max_front_steering_angle_,
        static_cast<int>(constrain_velocity_),
        static_cast<int>(constrain_control_rate_));
}

int MpcController::stateIndex(const int step, const int state_offset) const
{
    return kStateDim * step + state_offset;
}

int MpcController::controlIndex(const int step, const int control_offset) const
{
    return kStateDim * (horizon_ + 1) + kControlDim * step + control_offset;
}

Eigen::Matrix4d MpcController::buildAMatrix(const MpcReferencePoint& reference) const
{
    Eigen::Matrix4d a_matrix = Eigen::Matrix4d::Identity();
    a_matrix(0, 2) = -reference.v * std::sin(reference.yaw) * prediction_dt_;
    a_matrix(0, 3) = std::cos(reference.yaw) * prediction_dt_;
    a_matrix(1, 2) = reference.v * std::cos(reference.yaw) * prediction_dt_;
    a_matrix(1, 3) = std::sin(reference.yaw) * prediction_dt_;
    a_matrix(2, 3) = std::tan(reference.steering_angle) / wheel_base_
                     * prediction_dt_;
    return a_matrix;
}

Eigen::Matrix<double, 4, 2> MpcController::buildBMatrix(
    const MpcReferencePoint& reference) const
{
    Eigen::Matrix<double, 4, 2> b_matrix =
        Eigen::Matrix<double, 4, 2>::Zero();
    const double cos_steer = std::cos(reference.steering_angle);
    const double cos_steer_sq = std::max(1.0e-6, cos_steer * cos_steer);
    b_matrix(2, 1) = reference.v / (wheel_base_ * cos_steer_sq)
                     * prediction_dt_;
    b_matrix(3, 0) = prediction_dt_;
    return b_matrix;
}

MpcReferencePoint MpcController::convertTrajectoryPoint(
    const robot_trajectory_msgs::RobotTrajectoryPoint& point) const
{
    MpcReferencePoint ref;
    ref.x = point.pose.position.x;
    ref.y = point.pose.position.y;
    ref.yaw = tf2::getYaw(point.pose.orientation);
    const double raw_v = projectLongitudinal(point.velocity.linear.x,
                                             point.velocity.linear.y,
                                             ref.yaw);
    const double raw_a = projectLongitudinal(point.acceleration.linear.x,
                                             point.acceleration.linear.y,
                                             ref.yaw);
    const double raw_w = point.velocity.angular.z;
    double raw_steering_angle = 0.0;
    if (std::fabs(raw_v) > min_steering_conversion_speed_) {
        raw_steering_angle = std::atan(wheel_base_ * raw_w / raw_v);
    }
    ref.v = raw_v;
    ref.a = raw_a;
    ref.w = raw_w;
    ref.steering_angle = raw_steering_angle;
    if (limit_reference_dynamics_) {
        ref.v = clampValue(raw_v, min_velocity_, max_velocity_);
        ref.a = clampValue(raw_a, min_acceleration_, max_acceleration_);
        ref.steering_angle = clampValue(raw_steering_angle,
                                        min_front_steering_angle_,
                                        max_front_steering_angle_);
    }
    ref.w = ref.v / wheel_base_ * std::tan(ref.steering_angle);
    ref.w = clampValue(ref.w, min_angular_velocity_, max_angular_velocity_);
    ref.time_from_start = point.time_from_start.toSec();
    return ref;
}

double MpcController::getTrajectorySampleInterval() const
{
    for (const auto& point : trajectory_.points) {
        if (point.sampling_interval > 1.0e-6) {
            return point.sampling_interval;
        }
    }

    for (size_t i = 1; i < trajectory_.points.size(); ++i) {
        const double dt = (trajectory_.points[i].time_from_start
                           - trajectory_.points[i - 1].time_from_start).toSec();
        if (dt > 1.0e-6) {
            return dt;
        }
    }

    return prediction_dt_;
}

double MpcController::projectLongitudinal(double x, double y, double yaw) const
{
    return x * std::cos(yaw) + y * std::sin(yaw);
}

double MpcController::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
