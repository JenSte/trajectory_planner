#ifndef TRAJECTORY_PLANNER_MOTION_MODEL_HPP
#define TRAJECTORY_PLANNER_MOTION_MODEL_HPP

#include "trajectory_planner/types.hpp"

namespace trajectory_planner
{

class MotionModel
{
public:

    // Describes how to subdivide the maximum linear/angular velocities.
    enum class VelocitySpacing
    {
        // Divide the maximum velocity evenly by an estimated minimum velocity.
        LINEAR,

        // Divide the range to the maximum velocity using exponentially growing parts.
        LOGARITHMIC
    };

    // The type of the key used in the internal lookup table.
    using KeyType = std::tuple<Pose5D::LinearVelocity, Pose5D::AngularVelocity>;

    // The type of the value used in the internal lookup table.
    using ValueType = std::vector<KeyType>;

    MotionModel(
        double maximum_wheel_velocity,
        double maximum_wheel_acceleration,
        double wheel_distance,
//        double map_resolution,
//        unsigned int angle_granularity,
        unsigned int linear_velocity_steps,
        unsigned int angular_velocity_steps);

    MotionModel(MotionModel&&) = default;

    static std::vector<double> linear_steps(
        unsigned int steps,
        double maximum_value);

    // Return numbers from '0.0' to 'maximum' in 'smallest' steps.
    static std::vector<double> linearly_distributed(
        double smallest,
        double maximum);

    // Return numbers from '0.0' to 'maximum' with logarithmic steps in between.
    static std::vector<double> logarithmically_distributed(
        double smallest,
        double maximum);

    // Return a good estimate for a small linear velocity (in m/s) for the
    // given simulation parameters.
    static double smallest_linear_velocity(
        double time_delta,
        double map_resolution);

    // Return a good estimate for a small angular velocity (in rad/s) for the
    // given simulation parameters.
    static double smallest_angular_velocity(
        double time_delta,
        unsigned int angle_granularity);

    // Create the possible velocity combinations.
    static std::set<KeyType> create_velocity_combinations(
        const std::vector<double>& linear_steps,
        const std::vector<double>& angular_steps);

    // Create a lookup table that contains possible neighbours
    // for each valid velocity combination.
    static std::map<KeyType, ValueType> create_lookup_table(
        const std::set<KeyType>& keys);

    // Reverse the linear velocity indices in a lookup table.
    static std::map<KeyType, ValueType> reverse_lookup_table(
        const std::map<KeyType, ValueType>& lut);

    // Return the tuple containing the real velocities (in m/s and rad/s) for
    // the given velocity indices.
    std::tuple<double, double> velocities(
        Pose5D::LinearVelocity linear_velocity,
        Pose5D::AngularVelocity angular_velocity) const;

    // Return the possible velocities that can be reached from a given combination.
    const ValueType& lookup(
        bool forward,
        Pose5D::LinearVelocity linear_velocity,
        Pose5D::AngularVelocity angular_velocity) const;

    // Calculate the distances the vehicle moves in 'time_delta' seconds (the value
    // provided to the constructor. The arguments to the function are the current
    // orientation of the vehicle (in radian) and the real (in m/s and rad/s)
    // velocities, the result is a tuple consisting of the distances travelled in
    // x and y direction, and the orientation change.
    std::tuple<double, double, double> calculate_displacement(
        double theta,
        double real_linear_velocity,
        double real_angular_velocity) const;

    // Return the time delta simulation parameter that was passed to the constructor.
    double time_delta() const;

    // Return a reference to the internal vector holding the linear velocity values.
    const std::vector<double>& linear_steps() const;

    // Return a reference to the internal vector holding the linear angular values.
    const std::vector<double>& angular_steps() const;

    // Return a set containing all possible linear/angular velocity combinations.
    std::set<KeyType> velocity_combinations() const;

private:

    MotionModel(const MotionModel&) = delete;

    // The simulation time step.
    double time_delta_;

    // Contains a number of linear velocity values (in m/s).
    std::vector<double> linear_steps_;

    // Contains a number of angular velocity values (in rad/s).
    std::vector<double> angular_steps_;

    // Lookup table that maps a velocity combination (linear/angular) to possible
    // neighbours. Note that this is "one sided" for only forward linear speeds.
    // TODO: make this a vector for faster lookup?
    std::map<KeyType, ValueType> lookup_table_forward_;

    std::map<KeyType, ValueType> lookup_table_backward_;

    // Distribute the values between 'smallest' and 'maximum' according to 'spacing'.
    std::vector<double> distribute_values(
        MotionModel::VelocitySpacing spacing,
        double smallest,
        double maximum);
};

}

#endif
