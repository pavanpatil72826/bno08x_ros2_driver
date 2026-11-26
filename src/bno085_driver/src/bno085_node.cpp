#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "bno085_driver/protocol.hpp"
#include "bno085_driver/serial_port.hpp"

namespace bno085_driver
{
namespace
{

using namespace std::chrono_literals;

std::array<double, 9> diagonal_covariance(double stddev)
{
  std::array<double, 9> cov{};
  cov[0] = cov[4] = cov[8] = stddev * stddev;
  return cov;
}

/// Strips surrounding whitespace and drops non-ASCII bytes (serial noise).
std::string clean(std::string line)
{
  line.erase(
    std::remove_if(
      line.begin(), line.end(),
      [](char c) {return static_cast<unsigned char>(c) > 127;}),
    line.end());
  const char * whitespace = " \t\r\n\v\f";
  const std::size_t first = line.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }
  return line.substr(first, line.find_last_not_of(whitespace) - first + 1);
}

}  // namespace

class Bno085Node : public rclcpp::Node
{
public:
  Bno085Node()
  : Node("bno085_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = static_cast<int>(declare_parameter<int>("baud", 921600));
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    const auto topic = declare_parameter<std::string>("topic", "imu/data");
    const auto o_sd = declare_parameter<double>("orientation_stddev", 0.01);
    const auto g_sd = declare_parameter<double>("angular_velocity_stddev", 0.005);
    const auto a_sd = declare_parameter<double>("linear_acceleration_stddev", 0.05);
    const auto rpy_topic = declare_parameter<std::string>("rpy_topic", "imu/rpy");
    rpy_degrees_ = declare_parameter<bool>("rpy_degrees", false);
    const auto mag_topic = declare_parameter<std::string>("mag_topic", "imu/mag");
    // Firmware reports uT; sensor_msgs/MagneticField wants Tesla.
    mag_scale_ = declare_parameter<double>("mag_scale", 1e-6);

    if (!SerialPort::is_supported_baud(baud_)) {
      throw std::invalid_argument("unsupported baud rate " + std::to_string(baud_));
    }

    orientation_cov_ = diagonal_covariance(o_sd);
    angular_velocity_cov_ = diagonal_covariance(g_sd);
    linear_acceleration_cov_ = diagonal_covariance(a_sd);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(rpy_topic, 10);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 10);

    thread_ = std::thread(&Bno085Node::read_loop, this);
  }

  ~Bno085Node() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  bool active() const
  {
    return running_ && rclcpp::ok();
  }

  void read_loop()
  {
    std::size_t bad = 0;
    while (active()) {
      try {
        SerialPort serial(port_, baud_);
        RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud_);
        auto last_data = std::chrono::steady_clock::now();
        while (active()) {
          // Short timeout so shutdown is prompt.
          const auto raw = serial.read_line(100ms);
          if (!raw) {
            if (std::chrono::steady_clock::now() - last_data >= 1s) {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "No data from ESP32 (1 s)");
            }
            continue;
          }
          last_data = std::chrono::steady_clock::now();
          const std::string line = clean(*raw);
          if (!line.empty() && line.front() == '#') {
            RCLCPP_INFO(get_logger(), "ESP32: %s", line.c_str() + 1);
            continue;
          }
          const auto sample = parse_line(line);
          if (!sample) {
            if (++bad % 50 == 1) {
              RCLCPP_WARN(get_logger(), "Dropped malformed line(s): %zu total", bad);
            }
            continue;
          }
          publish(*sample);
        }
      } catch (const std::exception & e) {
        if (!active()) {
          break;
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 10000, "Serial error: %s; retrying in 2 s", e.what());
        for (int i = 0; i < 20 && active(); ++i) {
          std::this_thread::sleep_for(100ms);
        }
      }
    }
  }

  void publish(const ImuSample & s)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    imu.orientation.x = s.orientation[0];
    imu.orientation.y = s.orientation[1];
    imu.orientation.z = s.orientation[2];
    imu.orientation.w = s.orientation[3];
    imu.angular_velocity.x = s.angular_velocity[0];
    imu.angular_velocity.y = s.angular_velocity[1];
    imu.angular_velocity.z = s.angular_velocity[2];
    imu.linear_acceleration.x = s.linear_acceleration[0];
    imu.linear_acceleration.y = s.linear_acceleration[1];
    imu.linear_acceleration.z = s.linear_acceleration[2];
    imu.orientation_covariance = orientation_cov_;
    imu.angular_velocity_covariance = angular_velocity_cov_;
    imu.linear_acceleration_covariance = linear_acceleration_cov_;
    imu_pub_->publish(imu);

    // x = roll, y = pitch, z = yaw.
    const auto rpy = quaternion_to_rpy(
      s.orientation[0], s.orientation[1], s.orientation[2], s.orientation[3]);
    const double k = rpy_degrees_ ? 180.0 / M_PI : 1.0;
    geometry_msgs::msg::Vector3Stamped rpy_msg;
    rpy_msg.header = imu.header;
    rpy_msg.vector.x = rpy.roll * k;
    rpy_msg.vector.y = rpy.pitch * k;
    rpy_msg.vector.z = rpy.yaw * k;
    rpy_pub_->publish(rpy_msg);

    if (s.magnetic_field) {
      sensor_msgs::msg::MagneticField mag;
      mag.header = imu.header;
      mag.magnetic_field.x = (*s.magnetic_field)[0] * mag_scale_;
      mag.magnetic_field.y = (*s.magnetic_field)[1] * mag_scale_;
      mag.magnetic_field.z = (*s.magnetic_field)[2] * mag_scale_;
      mag_pub_->publish(mag);
    }
  }

  std::string port_;
  int baud_;
  std::string frame_id_;
  bool rpy_degrees_;
  double mag_scale_;
  std::array<double, 9> orientation_cov_;
  std::array<double, 9> angular_velocity_cov_;
  std::array<double, 9> linear_acceleration_cov_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace bno085_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<bno085_driver::Bno085Node>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("bno085_node"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
