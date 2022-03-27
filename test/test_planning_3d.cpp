#include "gtest/gtest.h"

#include "trajectory_planner/planning_3d.hpp"

using namespace trajectory_planner::three;

TEST(Planning3D, LookupTable)
{
    auto lut = create_lookup_table(16);
    EXPECT_EQ(lut.at(0), 0u);
    EXPECT_EQ(lut.at(1), 1u);
    EXPECT_EQ(lut.at(2), 2u);
    EXPECT_EQ(lut.at(3), 3u);
    EXPECT_EQ(lut.at(4), 4u);
    EXPECT_EQ(lut.at(5), 5u);
    EXPECT_EQ(lut.at(6), 6u);
    EXPECT_EQ(lut.at(7), 7u);
    EXPECT_EQ(lut.at(8), 8u);
    EXPECT_EQ(lut.at(9), 9u);
    EXPECT_EQ(lut.at(10), 10u);
    EXPECT_EQ(lut.at(11), 11u);
    EXPECT_EQ(lut.at(12), 12u);
    EXPECT_EQ(lut.at(13), 13u);
    EXPECT_EQ(lut.at(14), 14u);
    EXPECT_EQ(lut.at(15), 15u);

    lut = create_lookup_table(256);
    EXPECT_EQ(lut.at(0), 0u);
    EXPECT_EQ(lut.at(1), 19u);
    EXPECT_EQ(lut.at(2), 32u);
    EXPECT_EQ(lut.at(3), 45u);
    EXPECT_EQ(lut.at(4), 64u);
    EXPECT_EQ(lut.at(8), 128u);
    EXPECT_EQ(lut.at(12), 192u);
    EXPECT_EQ(lut.at(15), 237u);
}

TEST(Planning3D, LinearNeighbours)
{
    auto pose = [](unsigned int x, unsigned int y, MovementIndex m) {
        return Pose3D{x, y, m};
    };

    auto n = linear_neighbours(Pose3D{10, 10, 0});
    EXPECT_EQ(std::get<0>(n), pose(11, 10, 0));
    EXPECT_EQ(std::get<1>(n), pose(9, 10, 0));

    n = linear_neighbours(Pose3D{10, 10, 1});
    EXPECT_EQ(std::get<0>(n), pose(12, 11, 1));
    EXPECT_EQ(std::get<1>(n), pose(8, 9, 1));

    n = linear_neighbours(Pose3D{10, 10, 2});
    EXPECT_EQ(std::get<0>(n), pose(11, 11, 2));
    EXPECT_EQ(std::get<1>(n), pose(9, 9, 2));

    n = linear_neighbours(Pose3D{10, 10, 3});
    EXPECT_EQ(std::get<0>(n), pose(11, 12, 3));
    EXPECT_EQ(std::get<1>(n), pose(9, 8, 3));

    n = linear_neighbours(Pose3D{10, 10, 4});
    EXPECT_EQ(std::get<0>(n), pose(10, 11, 4));
    EXPECT_EQ(std::get<1>(n), pose(10, 9, 4));

    n = linear_neighbours(Pose3D{10, 10, 5});
    EXPECT_EQ(std::get<0>(n), pose(9, 12, 5));
    EXPECT_EQ(std::get<1>(n), pose(11, 8, 5));

    n = linear_neighbours(Pose3D{10, 10, 6});
    EXPECT_EQ(std::get<0>(n), pose(9, 11, 6));
    EXPECT_EQ(std::get<1>(n), pose(11, 9, 6));

    n = linear_neighbours(Pose3D{10, 10, 7});
    EXPECT_EQ(std::get<0>(n), pose(8, 11, 7));
    EXPECT_EQ(std::get<1>(n), pose(12, 9, 7));

    n = linear_neighbours(Pose3D{10, 10, 8});
    EXPECT_EQ(std::get<0>(n), pose(11, 10, 8));
    EXPECT_EQ(std::get<1>(n), pose(9, 10, 8));

    n = linear_neighbours(Pose3D{10, 10, 9});
    EXPECT_EQ(std::get<0>(n), pose(12, 11, 9));
    EXPECT_EQ(std::get<1>(n), pose(8, 9, 9));

    n = linear_neighbours(Pose3D{10, 10, 10});
    EXPECT_EQ(std::get<0>(n), pose(11, 11, 10));
    EXPECT_EQ(std::get<1>(n), pose(9, 9, 10));

    n = linear_neighbours(Pose3D{10, 10, 11});
    EXPECT_EQ(std::get<0>(n), pose(11, 12, 11));
    EXPECT_EQ(std::get<1>(n), pose(9, 8, 11));

    n = linear_neighbours(Pose3D{10, 10, 12});
    EXPECT_EQ(std::get<0>(n), pose(10, 11, 12));
    EXPECT_EQ(std::get<1>(n), pose(10, 9, 12));

    n = linear_neighbours(Pose3D{10, 10, 13});
    EXPECT_EQ(std::get<0>(n), pose(9, 12, 13));
    EXPECT_EQ(std::get<1>(n), pose(11, 8, 13));

    n = linear_neighbours(Pose3D{10, 10, 14});
    EXPECT_EQ(std::get<0>(n), pose(9, 11, 14));
    EXPECT_EQ(std::get<1>(n), pose(11, 9, 14));

    n = linear_neighbours(Pose3D{10, 10, 15});
    EXPECT_EQ(std::get<0>(n), pose(8, 11, 15));
    EXPECT_EQ(std::get<1>(n), pose(12, 9, 15));
}

TEST(Planning3D, TurnNeighbours)
{
    auto pose = [](unsigned int x, unsigned int y, MovementIndex m) {
        return Pose3D{x, y, m};
    };

    auto n = turn_neighbours(Pose3D{10, 10, 0});
    EXPECT_EQ(std::get<0>(n), pose(10, 10, 1));
    EXPECT_EQ(std::get<1>(n), pose(10, 10, 15));

    n = turn_neighbours(Pose3D{10, 10, 4});
    EXPECT_EQ(std::get<0>(n), pose(10, 10, 5));
    EXPECT_EQ(std::get<1>(n), pose(10, 10, 3));
}

TEST(Planning3D, RefinePose)
{
    trajectory_planner::Costs costs(256);
    AngleIndexLUT lut = create_lookup_table(costs.angle_granularity());

    // 29 * (360 deg / 256) is approx. 41 deg, this should be rounded
    // up to 45 deg.
    trajectory_planner::Pose pose{10, 11, 29};

    // As the costs object is empty, the calculation will fail and throw.
    EXPECT_THROW(refine_pose(costs, lut, pose), std::runtime_error);

    // Add the cost value for 45 deg (32 * (360 deg / 256) = 45 deg).
    costs.set_cost(10, 11, 32, 1.0);

    // The calculation shall not throw any more, and return a converted pose.
    Pose3D pose_3d = refine_pose(costs, lut, pose);
    EXPECT_EQ(pose_3d.x, 10u);
    EXPECT_EQ(pose_3d.y, 11u);
    EXPECT_EQ(pose_3d.movement, 2u);

    // The same again with an orientation that should be rounded down
    // (35 * (360 deg / 256) is approx. 49 deg).
    pose = trajectory_planner::Pose{10, 11, 35};
    pose_3d = refine_pose(costs, lut, pose);
    EXPECT_EQ(pose_3d.x, 10u);
    EXPECT_EQ(pose_3d.y, 11u);
    EXPECT_EQ(pose_3d.movement, 2u);

    // Test around the zero angle.
    pose = trajectory_planner::Pose{10, 11, 1};
    EXPECT_THROW(refine_pose(costs, lut, pose), std::runtime_error);
    pose = trajectory_planner::Pose{10, 11, 255};
    EXPECT_THROW(refine_pose(costs, lut, pose), std::runtime_error);

    // Add a cost value at 0 deg, so that the function can do the conversion.
    costs.set_cost(10, 11, 0, 1.0);
    pose = trajectory_planner::Pose{10, 11, 1};
    EXPECT_NO_THROW(refine_pose(costs, lut, pose));
    pose = trajectory_planner::Pose{10, 11, 255};
    EXPECT_NO_THROW(refine_pose(costs, lut, pose));
}
