// Offline unit test for the Odometry -> floating-base 13-scalar mapping (M6.3).
// This locks the convention that the Gazebo round-trip then validates against
// real physics: slot layout, quaternion order (xyzw) + normalization, and the
// pose-in-world / twist-in-body framing that matches Pinocchio's free-flyer.
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "kontrolem_state_bridge/odom_to_base.hpp"

using kontrolem_state_bridge::base_from_odom;

TEST(OdomToBase, LayoutIsPoseThenQuatThenLinThenAng)
{
  std::array<double, 13> b{};
  // Distinct values in every field so a transposed slot is caught.
  base_from_odom(1, 2, 3, 0, 0, 0, 1, 4, 5, 6, 7, 8, 9, b);
  EXPECT_DOUBLE_EQ(b[0], 1);  EXPECT_DOUBLE_EQ(b[1], 2);  EXPECT_DOUBLE_EQ(b[2], 3);   // pos
  EXPECT_DOUBLE_EQ(b[3], 0);  EXPECT_DOUBLE_EQ(b[4], 0);
  EXPECT_DOUBLE_EQ(b[5], 0);  EXPECT_DOUBLE_EQ(b[6], 1);                                // quat xyzw
  EXPECT_DOUBLE_EQ(b[7], 4);  EXPECT_DOUBLE_EQ(b[8], 5);  EXPECT_DOUBLE_EQ(b[9], 6);    // lin
  EXPECT_DOUBLE_EQ(b[10], 7); EXPECT_DOUBLE_EQ(b[11], 8); EXPECT_DOUBLE_EQ(b[12], 9);   // ang
}

TEST(OdomToBase, QuaternionIsNormalizedToUnitManifold)
{
  std::array<double, 13> b{};
  // A non-unit quaternion (norm 2) must come out unit — otherwise every
  // downstream SE(3) op is silently scaled/skewed.
  base_from_odom(0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, b);
  EXPECT_DOUBLE_EQ(b[6], 1.0);  // 2 / 2
  const double n = std::sqrt(b[3]*b[3] + b[4]*b[4] + b[5]*b[5] + b[6]*b[6]);
  EXPECT_NEAR(n, 1.0, 1e-12);
}

TEST(OdomToBase, NonAxisAlignedQuaternionStaysUnitAndKeepsDirection)
{
  std::array<double, 13> b{};
  const double s = 3.0;  // scale an already-normalized quat by 3
  base_from_odom(0, 0, 0, 0.5 * s, 0.5 * s, 0.5 * s, 0.5 * s, 0, 0, 0, 0, 0, 0, b);
  const double n = std::sqrt(b[3]*b[3] + b[4]*b[4] + b[5]*b[5] + b[6]*b[6]);
  EXPECT_NEAR(n, 1.0, 1e-12);
  EXPECT_NEAR(b[3], 0.5, 1e-12);  // direction preserved after re-normalization
  EXPECT_NEAR(b[6], 0.5, 1e-12);
}

TEST(OdomToBase, DegenerateQuaternionFallsBackToIdentity)
{
  std::array<double, 13> b{};
  base_from_odom(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, b);  // all-zero quat
  EXPECT_DOUBLE_EQ(b[3], 0.0);
  EXPECT_DOUBLE_EQ(b[4], 0.0);
  EXPECT_DOUBLE_EQ(b[5], 0.0);
  EXPECT_DOUBLE_EQ(b[6], 1.0);  // identity, not NaN from a 0/0
}

TEST(OdomToBase, TwistFramePassthroughNoRotation)
{
  // Documents the crux: a REP-145 odometry twist (body/child frame) maps straight
  // onto the free-flyer velocity v(0..5) with NO frame rotation. If a future
  // producer reported world-frame twist, this passthrough would be wrong — the
  // Gazebo round-trip is what proves the producer is body-frame.
  std::array<double, 13> b{};
  base_from_odom(0, 0, 0, 0, 0, 0, 1, /*lin*/ 1.5, -0.5, 0.25, /*ang*/ 0.1, 0.2, -0.3, b);
  EXPECT_DOUBLE_EQ(b[7], 1.5);   EXPECT_DOUBLE_EQ(b[8], -0.5);  EXPECT_DOUBLE_EQ(b[9], 0.25);
  EXPECT_DOUBLE_EQ(b[10], 0.1);  EXPECT_DOUBLE_EQ(b[11], 0.2);  EXPECT_DOUBLE_EQ(b[12], -0.3);
}
