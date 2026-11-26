#include "bno085_driver/protocol.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <system_error>
#include <vector>

namespace bno085_driver
{
namespace
{

template<typename T, typename ... Args>
bool parse_all(std::string_view text, T & out, Args... args)
{
  const char * end = text.data() + text.size();
  const auto result = std::from_chars(text.data(), end, out, args ...);
  return result.ec == std::errc() && result.ptr == end;
}

std::vector<std::string_view> split(std::string_view text, char sep)
{
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  for (std::size_t pos = text.find(sep); pos != std::string_view::npos;
    pos = text.find(sep, start))
  {
    fields.push_back(text.substr(start, pos - start));
    start = pos + 1;
  }
  fields.push_back(text.substr(start));
  return fields;
}

}  // namespace

std::optional<ImuSample> parse_line(std::string_view line)
{
  if (line.empty() || line.front() != '$') {
    return std::nullopt;
  }
  const std::size_t star = line.rfind('*');
  if (star == std::string_view::npos) {
    return std::nullopt;
  }
  const std::string_view body = line.substr(1, star - 1);

  unsigned int expected = 0;
  if (!parse_all(line.substr(star + 1), expected, 16)) {
    return std::nullopt;
  }
  unsigned int actual = 0;
  for (const char c : body) {
    actual ^= static_cast<unsigned char>(c);
  }
  if (actual != expected) {
    return std::nullopt;
  }

  // "IMU", timestamp, then quaternion + gyro + accel and optionally mag.
  const auto fields = split(body, ',');
  if ((fields.size() != 12 && fields.size() != 15) || fields[0] != "IMU") {
    return std::nullopt;
  }
  std::array<double, 13> v{};
  for (std::size_t i = 2; i < fields.size(); ++i) {
    if (!parse_all(fields[i], v[i - 2])) {
      return std::nullopt;
    }
  }

  ImuSample sample;
  sample.orientation = {v[0], v[1], v[2], v[3]};
  sample.angular_velocity = {v[4], v[5], v[6]};
  sample.linear_acceleration = {v[7], v[8], v[9]};
  if (fields.size() == 15) {
    sample.magnetic_field = {v[10], v[11], v[12]};
  }
  return sample;
}

Rpy quaternion_to_rpy(double x, double y, double z, double w)
{
  return {
    std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)),
    std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  };
}

}  // namespace bno085_driver
