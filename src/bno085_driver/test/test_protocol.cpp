#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "bno085_driver/protocol.hpp"

using bno085_driver::parse_line;
using bno085_driver::quaternion_to_rpy;

namespace
{

std::string frame(const std::string & body)
{
  unsigned int cs = 0;
  for (const char c : body) {
    cs ^= static_cast<unsigned char>(c);
  }
  char hex[3];
  std::snprintf(hex, sizeof(hex), "%02X", cs);
  return "$" + body + "*" + hex;
}

const char * const kBase = "IMU,12,0.1,0.2,0.3,0.9,1,-2,3,0.5,-0.5,9.81";

}  // namespace

TEST(ParseLine, ParsesSampleWithoutMag)
{
  const auto s = parse_line(frame(kBase));
  ASSERT_TRUE(s);
  EXPECT_DOUBLE_EQ(s->orientation[0], 0.1);
  EXPECT_DOUBLE_EQ(s->orientation[3], 0.9);
  EXPECT_DOUBLE_EQ(s->angular_velocity[1], -2.0);
  EXPECT_DOUBLE_EQ(s->linear_acceleration[2], 9.81);
  EXPECT_FALSE(s->magnetic_field);
}

TEST(ParseLine, ParsesSampleWithMag)
{
  const auto s = parse_line(frame(std::string(kBase) + ",20.5,-3,41"));
  ASSERT_TRUE(s);
  ASSERT_TRUE(s->magnetic_field);
  EXPECT_DOUBLE_EQ((*s->magnetic_field)[0], 20.5);
  EXPECT_DOUBLE_EQ((*s->magnetic_field)[2], 41.0);
}

TEST(ParseLine, RejectsBadChecksum)
{
  auto line = frame(kBase);
  line[7] = '9';
  EXPECT_FALSE(parse_line(line));
}

TEST(ParseLine, RejectsBadFraming)
{
  const auto good = frame(kBase);
  EXPECT_FALSE(parse_line(""));
  EXPECT_FALSE(parse_line("$"));
  EXPECT_FALSE(parse_line("$*"));
  EXPECT_FALSE(parse_line(good.substr(1)));
  EXPECT_FALSE(parse_line(good.substr(0, good.find('*'))));
  EXPECT_FALSE(parse_line(good.substr(0, good.find('*') + 1)));
  EXPECT_FALSE(parse_line(good + "zz"));
}

TEST(ParseLine, RejectsBadContent)
{
  EXPECT_FALSE(parse_line(frame("GPS,12,0,0,0,1,0,0,0,0,0,0")));
  EXPECT_FALSE(parse_line(frame("IMU,12,0,0,0,1,0,0,0,0,0")));
  EXPECT_FALSE(parse_line(frame(std::string(kBase) + ",1")));
  EXPECT_FALSE(parse_line(frame("IMU,12,0,0,0,1,0,0,0,0,0,abc")));
  EXPECT_FALSE(parse_line(frame("IMU,12,0,0,0,1,0,0,0,0,0,1.5x")));
  EXPECT_FALSE(parse_line(frame("IMU,12,0,0,0,1,0,0,0,0,0,")));
}

TEST(QuaternionToRpy, Identity)
{
  const auto r = quaternion_to_rpy(0, 0, 0, 1);
  EXPECT_DOUBLE_EQ(r.roll, 0.0);
  EXPECT_DOUBLE_EQ(r.pitch, 0.0);
  EXPECT_DOUBLE_EQ(r.yaw, 0.0);
}

TEST(QuaternionToRpy, Yaw90)
{
  const double h = std::sqrt(0.5);
  const auto r = quaternion_to_rpy(0, 0, h, h);
  EXPECT_NEAR(r.yaw, M_PI / 2, 1e-9);
  EXPECT_NEAR(r.roll, 0.0, 1e-9);
}

TEST(QuaternionToRpy, ClampsPitch)
{
  // Slightly over-unit quaternion must not produce NaN at the singularity.
  const auto r = quaternion_to_rpy(0, 0.7072, 0, 0.7072);
  EXPECT_NEAR(r.pitch, M_PI / 2, 1e-9);
}
