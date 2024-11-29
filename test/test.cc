#include <cmath>
#include <gtest/gtest.h>
#include "AdamsBashforth.hh"
#include <Eigen/Dense>

TEST(sqrt, integer) { EXPECT_EQ(2, std::sqrt(4)); }
