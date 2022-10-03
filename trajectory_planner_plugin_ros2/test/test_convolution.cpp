#include "gtest/gtest.h"

#include "trajectory_planner/convolution.hpp"

using namespace trajectory_planner;

TEST(Convolution, FastFFTSize)
{
    EXPECT_EQ(fast_fft_size(0), 1u);
    EXPECT_EQ(fast_fft_size(1), 1u);

    EXPECT_EQ(fast_fft_size(511), 512u);
    EXPECT_EQ(fast_fft_size(512), 512u);
    EXPECT_EQ(fast_fft_size(513), 525u);

    EXPECT_EQ(fast_fft_size(7679), 7680u);
    EXPECT_EQ(fast_fft_size(7680), 7680u);
    EXPECT_EQ(fast_fft_size(7681), 7776u);

    EXPECT_EQ(fast_fft_size(1024 * 1024 - 1), 1024u * 1024u);
    EXPECT_EQ(fast_fft_size(1024 * 1024), 1024u * 1024u);
    EXPECT_EQ(fast_fft_size(1024 * 1024 + 1), 1024u * 1024u + 1u);
}
