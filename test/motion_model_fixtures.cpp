#include "motion_model_fixtures.hpp"

void LinearMotionModel::SetUp()
{
    double time_delta = 0.1; // second
    double map_resolution = 0.02; // meter / pixel
    unsigned int angle_resolution = 64; // subdivisions of 2*pi
                                        //
    double linear_velocity_maximum = 0.5; // meter / second
    double angular_velocity_maximum = 1.0; // radian / second

    unsigned int linear_velocity_steps = 10;
    unsigned int angular_velocity_steps = 5;

    double linear_acceleration_maximum = 1.1;
    double angular_acceleration_maximum = 3.5;

    mm = std::make_unique<trajectory_planner::MotionModel>(
        time_delta,
        map_resolution,
        angle_resolution,
        trajectory_planner::MotionModel::VelocitySpacing::LINEAR,
        trajectory_planner::MotionModel::VelocitySpacing::LINEAR,
        linear_velocity_maximum,
        angular_velocity_maximum,
        linear_velocity_steps,
        angular_velocity_steps,
        linear_acceleration_maximum,
        angular_acceleration_maximum);
}
