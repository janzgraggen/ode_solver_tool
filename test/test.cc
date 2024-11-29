#include <cmath>
#include <gtest/gtest.h>
#include "../src/AdamsBashforth.hh"

TEST(sqrt, integer) { EXPECT_EQ(2, std::sqrt(4)); }
