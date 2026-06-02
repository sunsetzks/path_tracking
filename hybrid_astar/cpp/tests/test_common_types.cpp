#include <gtest/gtest.h>
#include "../include/common_types.hpp"

using namespace hybrid_astar;

TEST(CommonTypesTest, StateCreation) {
    State state(1.0, 2.0, 3.14, DirectionMode::FORWARD, 0.5);
    EXPECT_DOUBLE_EQ(state.x, 1.0);
    EXPECT_DOUBLE_EQ(state.y, 2.0);
    EXPECT_DOUBLE_EQ(state.yaw, 3.14);
    EXPECT_EQ(state.direction, DirectionMode::FORWARD);
    EXPECT_DOUBLE_EQ(state.steer, 0.5);
}

TEST(CommonTypesTest, PlanningConfigDefaults) {
    PlanningConfig config;
    EXPECT_DOUBLE_EQ(config.wheelbase, 2.5);
    EXPECT_DOUBLE_EQ(config.max_steer, 0.6);
    EXPECT_DOUBLE_EQ(config.grid_resolution, 1.0);
    EXPECT_FALSE(config.debug_enabled);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
