#include "gtest/gtest.h"

#include "trajectory_planner/planning_5d.hpp"

using namespace trajectory_planner::five;

TEST(Planning5D, SplitSegments)
{
    auto pose = [](unsigned int x, unsigned int y, unsigned int angle_index) {
        return trajectory_planner::Pose{x, y, angle_index};
    };

    trajectory_planner::Path path;

    // Starting point.
    path.push_back({0, 0, 0});

    std::vector<Segment> result = split_path(16, path);
    EXPECT_TRUE(result.empty());

    // Drive forward.
    path.push_back({1, 0, 0});

    result = split_path(16, path);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.at(0).direction, Direction::FORWARD);
    ASSERT_EQ(result.at(0).path.size(), 2u);
    EXPECT_EQ(result.at(0).path.at(0), pose(0, 0, 0));
    EXPECT_EQ(result.at(0).path.at(1), pose(1, 0, 0));

    // Turn left 90 degrees.
    path.push_back({1, 0, 1});
    path.push_back({1, 0, 2});
    path.push_back({1, 0, 3});
    path.push_back({1, 0, 4});

    result = split_path(16, path);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_EQ(result.at(0).direction, Direction::FORWARD);
    ASSERT_EQ(result.at(0).path.size(), 2u);
    EXPECT_EQ(result.at(0).path.at(0), pose(0, 0, 0));
    EXPECT_EQ(result.at(0).path.at(1), pose(1, 0, 0));
    EXPECT_EQ(result.at(1).direction, Direction::TURN);
    ASSERT_EQ(result.at(1).path.size(), 5u);
    EXPECT_EQ(result.at(1).path.at(0), pose(1, 0, 0));
    EXPECT_EQ(result.at(1).path.at(1), pose(1, 0, 1));
    EXPECT_EQ(result.at(1).path.at(2), pose(1, 0, 2));
    EXPECT_EQ(result.at(1).path.at(3), pose(1, 0, 3));
    EXPECT_EQ(result.at(1).path.at(4), pose(1, 0, 4));

    // Drive forward (upward, in the current orientation).
    path.push_back({1, 1, 4});
    path.push_back({1, 2, 4});
    path.push_back({1, 3, 4});
    path.push_back({1, 4, 4});

    // Turn left again, but not all 90 degrees.
    path.push_back({1, 4, 5});
    path.push_back({1, 4, 6});
    path.push_back({1, 4, 7});

    // Drive backward, slightly diagonal.
    path.push_back({3, 3, 7});
    path.push_back({5, 2, 7});
    path.push_back({7, 1, 7});

    result = split_path(16, path);
    ASSERT_EQ(result.size(), 5u);
    EXPECT_EQ(result.at(0).direction, Direction::FORWARD);
    ASSERT_EQ(result.at(0).path.size(), 2u);
    EXPECT_EQ(result.at(0).path.at(0), pose(0, 0, 0));
    EXPECT_EQ(result.at(0).path.at(1), pose(1, 0, 0));
    EXPECT_EQ(result.at(1).direction, Direction::TURN);
    ASSERT_EQ(result.at(1).path.size(), 5u);
    EXPECT_EQ(result.at(1).path.at(0), pose(1, 0, 0));
    EXPECT_EQ(result.at(1).path.at(4), pose(1, 0, 4));
    EXPECT_EQ(result.at(2).direction, Direction::FORWARD);
    ASSERT_EQ(result.at(2).path.size(), 5u);
    EXPECT_EQ(result.at(2).path.at(0), pose(1, 0, 4));
    EXPECT_EQ(result.at(2).path.at(4), pose(1, 4, 4));
    EXPECT_EQ(result.at(3).direction, Direction::TURN);
    ASSERT_EQ(result.at(3).path.size(), 4u);
    EXPECT_EQ(result.at(3).path.at(0), pose(1, 4, 4));
    EXPECT_EQ(result.at(3).path.at(3), pose(1, 4, 7));

    ASSERT_EQ(result.at(4).direction, Direction::BACKWARD);
    ASSERT_EQ(result.at(4).path.size(), 4u);
    EXPECT_EQ(result.at(4).path.at(0), pose(1, 4, 7));
    EXPECT_EQ(result.at(4).path.at(3), pose(7, 1, 7));
}

TEST(Planning5D, CircleCoordinates)
{
    const CircleCoordinates c = circle_coordinates(1);
    std::set<std::tuple<int, int>> s(c.cbegin(), c.cend());
    EXPECT_EQ(s.size(), 5u);
    EXPECT_NE(s.find(std::make_tuple(0, -1)), s.end());
    EXPECT_NE(s.find(std::make_tuple(-1, 0)), s.end());
    EXPECT_NE(s.find(std::make_tuple(0, 0)), s.end());
    EXPECT_NE(s.find(std::make_tuple(0, 1)), s.end());
    EXPECT_NE(s.find(std::make_tuple(1, 0)), s.end());
}
