#include "gtest/gtest.h"

#include "trajectory_planner/costs.hpp"

using namespace trajectory_planner;

TEST(Costs, Intersection)
{
    Costs c(16, 1);

    EXPECT_DOUBLE_EQ(c.get_cost(0, 0, 0), Costs::invalid_cost);
    EXPECT_DOUBLE_EQ(c.get_cost(0, 0, 1), Costs::invalid_cost);
    EXPECT_DOUBLE_EQ(c.get_cost(1, 0, 0), Costs::invalid_cost);
    EXPECT_DOUBLE_EQ(c.get_cost(2, 0, 0), Costs::invalid_cost);

    c.set_cost(0, 0, 0, 0.1);
    c.set_cost(0, 0, 1, 0.2);
    c.set_cost(1, 0, 0, 0.3);
    c.set_cost(2, 0, 0, 0.4);

    EXPECT_NEAR(c.get_cost(0, 0, 0), 0.1, 0.001);
    EXPECT_NEAR(c.get_cost(0, 0, 1), 0.2, 0.001);
    EXPECT_NEAR(c.get_cost(1, 0, 0), 0.3, 0.001);
    EXPECT_NEAR(c.get_cost(2, 0, 0), 0.4, 0.001);

    std::vector<Pose2D> subset = {Pose2D{0, 0}, Pose2D{2, 0}};
    Costs d = c.intersect(subset);

    EXPECT_NEAR(d.get_cost(0, 0, 0), 0.1, 0.001);
    EXPECT_NEAR(d.get_cost(0, 0, 1), 0.2, 0.001);
    EXPECT_DOUBLE_EQ(d.get_cost(1, 0, 0), Costs::invalid_cost);
    EXPECT_NEAR(d.get_cost(2, 0, 0), 0.4, 0.001);
}
