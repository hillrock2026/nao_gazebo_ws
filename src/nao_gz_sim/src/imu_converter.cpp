// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Task 3D minimal IMU converter.
///
/// Subscribes to the internal raw ``sensor_msgs/msg/Imu`` produced by
/// ros_gz_bridge from the native Gazebo IMU sensor and republishes the frozen
/// public ``/imu/data``:
///
/// * ``header.frame_id`` is set to the canonical ``ImuTorsoAccelerometer_frame``
///   (the stock bridge would expose the Gazebo scoped name instead);
/// * the Gazebo simulation timestamp is preserved (never a wall/ROS timer);
/// * orientation is checked for finite values and non-zero norm, then normalized;
/// * angular velocity and linear acceleration are checked for finite values;
/// * covariance is written explicitly (all zeros = "no noise model in simulation").
///
/// This node never publishes /tf, never uses a controller and never writes to
/// the model.

#include <array>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuConverter : public rclcpp::Node
{
public:
  ImuConverter()
  : rclcpp::Node("imu_converter")
  {
    raw_topic_ = this->declare_parameter<std::string>("raw_topic", "/nao/imu_raw");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/imu/data");
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id", "ImuTorsoAccelerometer_frame");
    const int qos_depth = this->declare_parameter<int>("qos_depth", 10);

    rclcpp::QoS in_qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth))));
    in_qos.reliable().durability_volatile();
    rclcpp::QoS out_qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth))));
    out_qos.reliable().durability_volatile();

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, out_qos);
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      raw_topic_, in_qos,
      std::bind(&ImuConverter::on_raw, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "imu_converter: raw='%s' -> '%s' (frame_id='%s', reliable, volatile, "
                "keep_last %d)", raw_topic_.c_str(), output_topic_.c_str(),
                frame_id_.c_str(), qos_depth);
  }

private:
  void fail(const std::string & reason)
  {
    if (!failed_) {
      failed_ = true;
      RCLCPP_ERROR(this->get_logger(), "CONTRACT VIOLATION: %s; publishing stopped",
                   reason.c_str());
    }
  }

  static bool finite3(const std::array<double, 3> & v)
  {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  void on_raw(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (failed_) {
      return;
    }

    const auto & stamp = msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0u) {
      return;  // no valid simulation time yet
    }

    std::array<double, 4> q = {msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w};
    for (double v : q) {
      if (!std::isfinite(v)) {
        fail("non-finite orientation");
        return;
      }
    }
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1e-9) {
      fail("zero-norm / non-finite orientation quaternion");
      return;
    }

    const std::array<double, 3> av = {msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z};
    const std::array<double, 3> la = {msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};
    if (!finite3(av)) {
      fail("non-finite angular velocity");
      return;
    }
    if (!finite3(la)) {
      fail("non-finite linear acceleration");
      return;
    }

    // Timestamp policy: strictly increasing Gazebo simulation time.
    if (have_last_stamp_) {
      const bool backwards =
        (stamp.sec < last_sec_) ||
        (stamp.sec == last_sec_ && stamp.nanosec < last_nsec_);
      const bool duplicate =
        (stamp.sec == last_sec_ && stamp.nanosec == last_nsec_);
      if (backwards) {
        fail("simulation timestamp went backwards: " +
             std::to_string(last_sec_) + "." + std::to_string(last_nsec_) + " -> " +
             std::to_string(stamp.sec) + "." + std::to_string(stamp.nanosec));
        return;
      }
      if (duplicate) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "duplicate IMU timestamp; skipping");
        return;
      }
    }

    sensor_msgs::msg::Imu out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;
    const double inv = 1.0 / norm;
    out.orientation.x = q[0] * inv;
    out.orientation.y = q[1] * inv;
    out.orientation.z = q[2] * inv;
    out.orientation.w = q[3] * inv;
    out.orientation_covariance.fill(0.0);        // no noise model in simulation
    out.angular_velocity.x = av[0];
    out.angular_velocity.y = av[1];
    out.angular_velocity.z = av[2];
    out.angular_velocity_covariance.fill(0.0);   // no noise model in simulation
    out.linear_acceleration.x = la[0];
    out.linear_acceleration.y = la[1];
    out.linear_acceleration.z = la[2];
    out.linear_acceleration_covariance.fill(0.0);  // no noise model in simulation

    pub_->publish(out);
    last_sec_ = stamp.sec;
    last_nsec_ = stamp.nanosec;
    have_last_stamp_ = true;
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::string raw_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int64_t last_sec_{0};
  uint32_t last_nsec_{0};
  bool have_last_stamp_{false};
  bool failed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuConverter>());
  rclcpp::shutdown();
  return 0;
}
