#include <gtest/gtest.h>
#include "simulator.h"

namespace {
    int GetMeaningOfLife() {
        return 42;
    }
}

TEST(TestSuiteName, TestName) {
    EXPECT_EQ(GetMeaningOfLife(), 42);
}

TEST(TestSuiteName, TestName2) {
    ASSERT_EQ(GetMeaningOfLife(), 42) << "No, It can't be!";
    EXPECT_FLOAT_EQ(23.23F, 23.23F);
}

