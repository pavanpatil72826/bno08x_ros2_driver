#ifndef BNO085_DRIVER__PROTOCOL_HPP_
#define BNO085_DRIVER__PROTOCOL_HPP_

#include <array>
#include <optional>
#include <string_view>

namespace bno085_driver
{

struct ImuSample
{
  std::array<double, 4> orientation;          // x y z w
  std::array<double, 3> angular_velocity;     // rad/s
  std::array<double, 3> linear_acceleration;  // m/s^2, includes gravity
  std::optional<std::array<double, 3>> magnetic_field;  // uT
};

/// Parse "$IMU,<ms>,<10 or 13 values>*<XOR checksum hex>" (no line ending).
/// Returns std::nullopt if the framing, checksum or any value is invalid.
std::optional<ImuSample> parse_line(std::string_view line);

struct Rpy
{
  double roll;
  double pitch;
  double yaw;
};

/// ZYX roll/pitch/yaw in radians from a quaternion (REP-103).
Rpy quaternion_to_rpy(double x, double y, double z, double w);

}  // namespace bno085_driver

#endif  // BNO085_DRIVER__PROTOCOL_HPP_
