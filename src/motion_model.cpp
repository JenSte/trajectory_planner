#include "trajectory_planner/motion_model.hpp"

#include <cmath>

namespace trajectory_planner
{

MotionModel::MotionModel(
    double time_delta,
    double /*map_resolution*/,
    unsigned int /*angle_granularity*/,
    MotionModel::VelocitySpacing /*linear_velocity_spacing*/,
    MotionModel::VelocitySpacing /*angular_velocity_spacing*/,
    double linear_velocity_maximum,
    double angular_velocity_maximum,
    unsigned int linear_velocity_steps,
    unsigned int angular_velocity_steps,
    double /*linear_acceleration_maximum*/,
    double /*angular_acceleration_maximum*/)
    : time_delta_(time_delta)
    , linear_steps_(linear_steps(linear_velocity_steps, linear_velocity_maximum))
    , angular_steps_(linear_steps(angular_velocity_steps, angular_velocity_maximum))
//    , linear_steps_(
//        distribute_values(
//            linear_velocity_spacing,
//            smallest_linear_velocity(time_delta, map_resolution),
//            linear_velocity_maximum))
//    , angular_steps_(
//        distribute_values(
//            angular_velocity_spacing,
//            smallest_angular_velocity(time_delta, angle_granularity),
//            angular_velocity_maximum))
    , lookup_table_forward_(
        create_lookup_table(
            create_velocity_combinations(linear_steps_, angular_steps_)))
    , lookup_table_backward_(reverse_lookup_table(lookup_table_forward_))
{
}

std::vector<double> MotionModel::linear_steps(
    unsigned int steps,
    double maximum_value)
{
    std::vector<double> result;

    for (unsigned int i = 0; i <= std::max(1u, steps); i++) {
        result.push_back(i * maximum_value / std::max(1u, steps));
    }

    return result;
}

std::vector<double> MotionModel::linearly_distributed(
    double smallest,
    double maximum)
{
    std::vector<double> result;

    for (int i = 0; i < (maximum / smallest + 1); i++) {
        result.push_back(i * smallest);
    }

    return result;
}

std::vector<double> MotionModel::logarithmically_distributed(
    double smallest,
    double maximum)
{
    std::vector<double> result{0.0};

    unsigned int i = 1;
    while (i * smallest < maximum) {
        result.push_back(i * smallest);
        i *= 2;
    }

    return result;
}

double MotionModel::smallest_linear_velocity(
    double time_delta,
    double map_resolution)
{
    return sqrt(2.0) * map_resolution / time_delta;
}

double MotionModel::smallest_angular_velocity(
    double time_delta,
    unsigned int angle_granularity)
{
    return (2.0 * M_PI / angle_granularity) / time_delta;
}

std::vector<double> MotionModel::distribute_values(
    MotionModel::VelocitySpacing spacing,
    double smallest,
    double maximum)
{
    if (spacing == VelocitySpacing::LINEAR) {
        return linearly_distributed(smallest, maximum);
    }

    return logarithmically_distributed(smallest, maximum);
}

std::set<MotionModel::KeyType> MotionModel::create_velocity_combinations(
    const std::vector<double>& linear_steps,
    const std::vector<double>& angular_steps)
{
    std::set<KeyType> result;

    const size_t ls = linear_steps.size();
    const size_t as = angular_steps.size();

    // The parameters of a line equation "k * l + d" that is used to
    // rule out some of the linear/angular velocity combinattions.
    const double k = -(as - 0.5) / (ls - 0.5);
    const double d = as - 0.5;

    // The linear velocity index starts with '1', i.e. turning on the spot is not
    // supported by the 5D planner.
    for (size_t l = 1; l < ls; l++) {
        for (size_t a = 0; a < as; a++) {
            // If the current combination is above the line, it is not included
            // in the result. This is to rule out combinations where both index
            // values correspond to high (absolute) velocity values.
            if (a > (k * l + d)) {
                continue;
            }

            Pose5D::LinearVelocity linear_index = Pose5D::LinearVelocity(l);

            // Each angular velocity is added twice, once for turning CW and once
            // for turning CCW.
            result.insert(std::make_tuple(linear_index, Pose5D::AngularVelocity(a)));
            result.insert(std::make_tuple(linear_index, Pose5D::AngularVelocity(-a)));
        }
    }

    return result;
}

std::map<MotionModel::KeyType, MotionModel::ValueType> MotionModel::create_lookup_table(
    const std::set<KeyType>& keys)
{
    std::map<KeyType, ValueType> result;

    for (const auto& key: keys) {
        // The vector of neighbours for the combination we are currently processing.
        ValueType v;

        // Add a combination to 'v' if it is valid (= the neighbour exists in 'keys').
        auto add = [&keys, &v](int l, int a) {
            Pose5D::LinearVelocity linear_index = static_cast<Pose5D::LinearVelocity>(l);
            Pose5D::AngularVelocity angular_index = static_cast<Pose5D::AngularVelocity>(a);

            KeyType k = std::make_tuple(linear_index, angular_index);

            const auto it = keys.find(k);
            if (it != keys.end()) {
                // Only add valid velocity combinations.
                v.push_back(k);
            }
        };

        int l = static_cast<int>(std::get<0>(key));
        int a = static_cast<int>(std::get<1>(key));

        add(l, a);
        add(l + 1, a);
        add(l + 1, a + 1);
        add(l, a + 1);
        add(l - 1, a + 1);
        add(l - 1, a);
        add(l - 1, a - 1);
        add(l, a - 1);
        add(l + 1, a - 1);

        result[key] = v;
    }

    return result;
}

std::map<MotionModel::KeyType, MotionModel::ValueType> MotionModel::reverse_lookup_table(
    const std::map<MotionModel::KeyType, MotionModel::ValueType>& lut)
{
    // Invert the linear velocity of a given velocity combination, leaving the
    // angular velocity untouched.
    auto reverse = [](const KeyType& k) {
        int index = static_cast<int>(std::get<0>(k));
        return std::make_tuple(Pose5D::LinearVelocity(-index), std::get<1>(k));
    };

    std::map<KeyType, ValueType> result;

    for (const auto& kv: lut) {
        ValueType vs;

        for (const KeyType& k: std::get<1>(kv)) {
            vs.emplace_back(reverse(k));
        }

        result[reverse(std::get<0>(kv))] = vs;
    }

    return result;
}

std::tuple<double, double> MotionModel::velocities(
    Pose5D::LinearVelocity linear_velocity,
    Pose5D::AngularVelocity angular_velocity) const
{
    int linear_index = static_cast<int>(linear_velocity);
    int angular_index = static_cast<int>(angular_velocity);

    // The 'linear_/angular_steps_' vectors only contain the values for
    // the positive indices, for negative velocities we have to add the
    // correct sign later.
    const double linear_sign = linear_index < 0 ? -1.0 : 1.0;
    const double angular_sign = angular_index < 0 ? -1.0 : 1.0;

    const double linear_value = copysign(linear_steps_.at(abs(linear_index)), linear_sign);
    const double angular_value = copysign(angular_steps_.at(abs(angular_index)), angular_sign);

    return std::make_tuple(linear_value, angular_value);
}

const MotionModel::ValueType& MotionModel::lookup(
    bool forward,
    Pose5D::LinearVelocity linear_velocity,
    Pose5D::AngularVelocity angular_velocity) const
{
    const std::map<KeyType, ValueType>& lut = forward ? lookup_table_forward_ : lookup_table_backward_;

    const auto it = lut.find(std::make_tuple(linear_velocity, angular_velocity));
    if (it == lut.end()) {
        throw std::runtime_error("Unknown velocity combination.");
    }

    return it->second;
}

std::tuple<double, double, double> MotionModel::calculate_displacement(
    double theta,
    double real_linear_velocity,
    double real_angular_velocity) const
{
    const double dx = real_linear_velocity * cos(theta) * time_delta_;
    const double dy = real_linear_velocity * sin(theta) * time_delta_;
    const double dtheta = real_angular_velocity * time_delta_;

    return std::make_tuple(dx, dy, dtheta);
}

double MotionModel::time_delta() const
{
    return time_delta_;
}

const std::vector<double>& MotionModel::linear_steps() const
{
    return linear_steps_;
}

const std::vector<double>& MotionModel::angular_steps() const
{
    return angular_steps_;
}

std::set<MotionModel::KeyType> MotionModel::velocity_combinations() const
{
    std::set<KeyType> result;

    for (const auto& pair: lookup_table_forward_) {
        result.insert(pair.first);
    }

    for (const auto& pair: lookup_table_backward_) {
        result.insert(pair.first);
    }

    return result;
}

}
