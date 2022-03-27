#include "gtest/gtest.h"

#include "trajectory_planner/a_star.hpp"

#include "trajectory_planner/planning_3d.hpp"

using namespace trajectory_planner;

TEST(AStar, PriorityQueue)
{
    auto pose = [](unsigned int x, unsigned int y, three::MovementIndex m) {
        return three::Pose3D{x, y, m};
    };

    PriorityQueue<three::Pose3D> pq;

    EXPECT_TRUE(pq.empty());

    pq.push(pose(1, 3, 0), 1.3);
    pq.push(pose(3, 1, 0), 3.1);
    pq.push(pose(2, 4, 0), 2.4);

    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(1, 3, 0));
    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(2, 4, 0));
    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(3, 1, 0));

    EXPECT_TRUE(pq.empty());

    pq.push(pose(1, 3, 0), 1.3);
    pq.push(pose(3, 1, 0), 3.1);
    pq.push(pose(2, 4, 0), 2.4);

    // Update priorities.
    pq.push(pose(1, 3, 0), 5.0);
    pq.push(pose(3, 1, 0), 4.0);

    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(2, 4, 0));
    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(3, 1, 0));
    ASSERT_FALSE(pq.empty());
    EXPECT_EQ(pq.pop(), pose(1, 3, 0));

    EXPECT_TRUE(pq.empty());
}
