#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "motion_model_fixtures.hpp"

using namespace trajectory_planner;
using LinearVelocity = Pose5D::LinearVelocity;
using AngularVelocity = Pose5D::AngularVelocity;

TEST(MotionModel, SmallestVelocities)
{
    EXPECT_NEAR(MotionModel::smallest_linear_velocity(0.1, 0.02), 0.283, 0.001);
    EXPECT_NEAR(MotionModel::smallest_linear_velocity(0.1, 0.01), 0.141, 0.001);

    EXPECT_NEAR(MotionModel::smallest_angular_velocity(0.1, 64), 0.982, 0.001);
    EXPECT_NEAR(MotionModel::smallest_angular_velocity(0.1, 128), 0.491, 0.001);
}

TEST(MotionModel, LinearSteps)
{
    std::vector<double> l = MotionModel::linear_steps(4, 0.4);

    ASSERT_EQ(l.size(), 5u);
    EXPECT_DOUBLE_EQ(l.at(0), 0.0);
    EXPECT_DOUBLE_EQ(l.at(1), 0.1);
    EXPECT_DOUBLE_EQ(l.at(2), 0.2);
    EXPECT_DOUBLE_EQ(l.at(3), 0.3);
    EXPECT_DOUBLE_EQ(l.at(4), 0.4);
}

TEST(MotionModel, LinearDistribution)
{
    std::vector<double> l = MotionModel::linearly_distributed(0.1, 0.9);

    ASSERT_EQ(l.size(), 10u);
    EXPECT_DOUBLE_EQ(l.at(0), 0.0);
    EXPECT_DOUBLE_EQ(l.at(1), 0.1);
    EXPECT_DOUBLE_EQ(l.at(8), 0.8);
    EXPECT_DOUBLE_EQ(l.at(9), 0.9);
}

TEST(MotionModel, LogarithmicDistribution)
{
    std::vector<double> l = MotionModel::logarithmically_distributed(0.1, 0.9);

    ASSERT_EQ(l.size(), 5u);
    EXPECT_DOUBLE_EQ(l.at(0), 0.0);
    EXPECT_DOUBLE_EQ(l.at(1), 0.1);
    EXPECT_DOUBLE_EQ(l.at(2), 0.2);
    EXPECT_DOUBLE_EQ(l.at(3), 0.4);
    EXPECT_DOUBLE_EQ(l.at(4), 0.8);
}

TEST(MotionModel, VelocityCombinations)
{
    std::vector<double> ls{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> as{0.0, 0.1, 0.2, 0.3, 0.4};

    std::set<MotionModel::KeyType> cs =
        MotionModel::create_velocity_combinations(ls, as);

    auto key = [](int l, int a) {
        return std::make_tuple(
            static_cast<LinearVelocity>(l),
            static_cast<AngularVelocity>(a));
    };

    EXPECT_FALSE(cs.contains(key(0, -5)));
    EXPECT_FALSE(cs.contains(key(0, 0)));
    EXPECT_FALSE(cs.contains(key(0, 5)));

    EXPECT_TRUE(cs.contains(key(1, 0)));
    EXPECT_TRUE(cs.contains(key(1, 1)));
    EXPECT_TRUE(cs.contains(key(1, 2)));
    EXPECT_TRUE(cs.contains(key(1, 3)));
    EXPECT_FALSE(cs.contains(key(1, 4)));

    EXPECT_TRUE(cs.contains(key(2, 0)));
    EXPECT_TRUE(cs.contains(key(2, 1)));
    EXPECT_TRUE(cs.contains(key(2, 2)));
    EXPECT_FALSE(cs.contains(key(2, 3)));

    EXPECT_TRUE(cs.contains(key(3, 0)));
    EXPECT_TRUE(cs.contains(key(3, 1)));
    EXPECT_TRUE(cs.contains(key(3, 2)));
    EXPECT_FALSE(cs.contains(key(3, 3)));

    EXPECT_TRUE(cs.contains(key(4, 0)));
    EXPECT_TRUE(cs.contains(key(4, 1)));
    EXPECT_FALSE(cs.contains(key(4, 2)));

    EXPECT_TRUE(cs.contains(key(5, 0)));
    EXPECT_FALSE(cs.contains(key(5, 1)));

    EXPECT_FALSE(cs.contains(key(6, 0)));
}

TEST(MotionModel, LookupTable)
{
    auto key = [](int l, int a) {
        return std::make_tuple(
            static_cast<LinearVelocity>(l),
            static_cast<AngularVelocity>(a));
    };

    std::set<MotionModel::KeyType> keys{
        key(0,  1), key(1,  1), key(2,  1), key(3,  1),
                    key(1,  0), key(2,  0), key(3,  0),
        key(0, -1), key(1, -1), key(2, -1), key(3, -1),
    };

    std::map<MotionModel::KeyType, MotionModel::ValueType> lut =
        MotionModel::create_lookup_table(keys);

    EXPECT_FALSE(lut.contains(key(0, 0)));

    ASSERT_TRUE(lut.contains(key(3, 1)));
    EXPECT_THAT(
       lut.find(key(3, 1))->second,
       testing::UnorderedElementsAre(key(3, 1), key(2, 1), key(3, 0)));

    ASSERT_TRUE(lut.contains(key(1, 0)));
    EXPECT_THAT(
       lut.find(key(1, 0))->second,
       testing::UnorderedElementsAre(key(1, 0), key(1, 1), key(2, 0), key(1, -1)));

    ASSERT_TRUE(lut.contains(key(2, 0)));
    EXPECT_THAT(
       lut.find(key(2, 0))->second,
       testing::UnorderedElementsAre(key(2, 0), key(1, 0), key(2, 1), key(3, 0), key(2, -1)));

    // Turn the lookup table around and check it.
    std::map<MotionModel::KeyType, MotionModel::ValueType> reversed =
        MotionModel::reverse_lookup_table(lut);

    EXPECT_FALSE(reversed.contains(key(3, 1)));
    EXPECT_FALSE(reversed.contains(key(1, 1)));
    EXPECT_FALSE(reversed.contains(key(2, 0)));

    ASSERT_TRUE(reversed.contains(key(-3, 1)));
    EXPECT_THAT(
       reversed.find(key(-3, 1))->second,
       testing::UnorderedElementsAre(key(-3, 1), key(-2, 1), key(-3, 0)));

    ASSERT_TRUE(reversed.contains(key(-1, 0)));
    EXPECT_THAT(
       reversed.find(key(-1, 0))->second,
       testing::UnorderedElementsAre(key(-1, 0), key(-1, 1), key(-2, 0), key(-1, -1)));

    ASSERT_TRUE(reversed.contains(key(-2, 0)));
    EXPECT_THAT(
       reversed.find(key(-2, 0))->second,
       testing::UnorderedElementsAre(key(-2, 0), key(-1, 0), key(-2, 1), key(-3, 0), key(-2, -1)));
}

TEST_F(LinearMotionModel, CheckMotionModel)
{
    auto key = [](int l, int a) {
        return std::make_tuple(
            static_cast<LinearVelocity>(l),
            static_cast<AngularVelocity>(a));
    };

    ASSERT_EQ(mm->linear_steps().size(), 11u);
    EXPECT_NEAR(mm->linear_steps().at(0), 0.000, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(1), 0.050, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(2), 0.100, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(3), 0.150, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(4), 0.200, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(5), 0.250, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(6), 0.300, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(7), 0.350, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(8), 0.400, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(9), 0.450, 0.001);
    EXPECT_NEAR(mm->linear_steps().at(10), 0.500, 0.001);

    ASSERT_EQ(mm->angular_steps().size(), 6u);
    EXPECT_NEAR(mm->angular_steps().at(0), 0.000, 0.001);
    EXPECT_NEAR(mm->angular_steps().at(1), 0.200, 0.001);
    EXPECT_NEAR(mm->angular_steps().at(2), 0.400, 0.001);
    EXPECT_NEAR(mm->angular_steps().at(3), 0.600, 0.001);
    EXPECT_NEAR(mm->angular_steps().at(4), 0.800, 0.001);
    EXPECT_NEAR(mm->angular_steps().at(5), 1.000, 0.001);

    std::tuple<double, double> ss;
    ss = mm->velocities(static_cast<LinearVelocity>(1), static_cast<AngularVelocity>(-1));
    EXPECT_NEAR(std::get<0>(ss), 0.050, 0.001);
    EXPECT_NEAR(std::get<1>(ss), -0.200, 0.001);

    ss = mm->velocities(static_cast<LinearVelocity>(-1), static_cast<AngularVelocity>(1));
    EXPECT_NEAR(std::get<0>(ss), -0.050, 0.001);
    EXPECT_NEAR(std::get<1>(ss), 0.200, 0.001);

    // Drive forward.
    EXPECT_THAT(
        mm->lookup(true, LinearVelocity(1), AngularVelocity(0)),
        testing::UnorderedElementsAre(key(1, 0), key(2, 0), key(1, 1), key(1, -1)));

    // Drive backward.
    EXPECT_THAT(
        mm->lookup(false, LinearVelocity(-1), AngularVelocity(0)),
        testing::UnorderedElementsAre(key(-1, 0), key(-2, 0), key(-1, 1), key(-1, -1)));
}
