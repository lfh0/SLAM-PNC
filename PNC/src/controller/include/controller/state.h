#pragma once

enum class ControlState
{
    None,
    Init,
    TakingOff,
    Tracking,
    CloseEnd,
    GoalYawAdjust,
    Finished
};

struct RobotState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};

struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
    double reference_velocity = 0.0;
    double arc_length = 0.0;
};

struct ControlCommand
{
    ControlState state = ControlState::None;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};
