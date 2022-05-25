#include "gtest/gtest.h"

#include "trajectory_planner/motion_model.hpp"

class LinearMotionModel: public ::testing::Test
{
protected:

    std::unique_ptr<trajectory_planner::MotionModel> mm;

    virtual void SetUp();
};
